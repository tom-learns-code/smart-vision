#include <string.h>
#include "bomb_solver_adapter.h"
#include "bomb_solver_core.h"

#define RAW_POS(r, c) ((uint8)(((r) << 4) | (c)))
#define RAW_ROW(pos)   ((uint8)((pos) >> 4))
#define RAW_COL(pos)   ((uint8)((pos) & 0x0FU))

static const int8 bomb_dx[4] = {0, 0, -1, 1};
static const int8 bomb_dy[4] = {-1, 1, 0, 0};
static const char *const bomb_dir_name[4] = {"UP", "DOWN", "LEFT", "RIGHT"};
static const float bomb_dir_theta[4] = {-90.0f, 90.0f, 180.0f, 0.0f};

static bomb_phase_result_t phase1_result;
static uint8 phase1_valid;
static uint8 phase1_goal_pos[MAX_BOXES];

typedef struct {
    solver_output_t *out;
    uint8 active;
    uint8 count;
    uint8 x[MAX_WP];
    uint8 y[MAX_WP];
} walk_builder_t;

static void output_init(solver_output_t *out, solver_plan_phase_t phase)
{
    memset(out, 0, sizeof(*out));
    out->status = SOLVER_STATUS_BAD_ARGUMENT;
    out->plan_phase = phase;
    out->plan_check = SOLVER_PLAN_CHECK_NOT_RUN;
    out->plan_check_action = 255U;
    out->plan_check_step = 255U;
    out->plan_check_x = 255U;
    out->plan_check_y = 255U;
    memset(out->assignment, 255, sizeof(out->assignment));
}

static uint8 append_action(solver_output_t *out, action_t **action)
{
    if(out->action_count >= MAX_ACTIONS) return 0U;
    *action = &out->actions[out->action_count++];
    memset(*action, 0, sizeof(**action));
    return 1U;
}

static uint8 flush_walk(walk_builder_t *walk)
{
    action_t *action;
    int i;

    if(!walk->active || walk->count < 2U) {
        walk->active = 0U;
        walk->count = 0U;
        return 1U;
    }
    if(!append_action(walk->out, &action)) return 0U;
    action->type = ACTION_FREE_MOVE;
    action->target_x = walk->x[walk->count - 1U];
    action->target_y = walk->y[walk->count - 1U];
    action->wp_count = walk->count;
    for(i = 0U; i < walk->count; i++) {
        action->waypoints[i].x_mm = (int16)walk->x[i] * SOLVER_GRID_SIZE_MM;
        action->waypoints[i].y_mm = (int16)walk->y[i] * SOLVER_GRID_SIZE_MM;
    }
    walk->out->total_free_moves++;
    walk->out->total_waypoints += walk->count;
    walk->active = 0U;
    walk->count = 0U;
    return 1U;
}

static uint8 append_walk_step(walk_builder_t *walk,
                              uint8 from_x, uint8 from_y,
                              uint8 to_x, uint8 to_y)
{
    if(!walk->active) {
        walk->active = 1U;
        walk->count = 1U;
        walk->x[0] = from_x;
        walk->y[0] = from_y;
    }
    if(walk->count >= MAX_WP) {
        if(!flush_walk(walk)) return 0U;
        walk->active = 1U;
        walk->count = 1U;
        walk->x[0] = from_x;
        walk->y[0] = from_y;
    }
    walk->x[walk->count] = to_x;
    walk->y[walk->count] = to_y;
    walk->count++;
    return 1U;
}

static void set_push_direction(push_meta_t *push, uint8 dir)
{
    push->push_dir = dir;
    strncpy(push->push_dir_str, bomb_dir_name[dir],
            sizeof(push->push_dir_str) - 1U);
    push->push_dir_str[sizeof(push->push_dir_str) - 1U] = '\0';
}

static uint8 push_actions_are_contiguous(const action_t *first,
                                         const action_t *second)
{
    const push_meta_t *left;
    const push_meta_t *right;
    int dx;
    int dy;
    int left_car_end_x;
    int left_car_end_y;

    if(first == 0 || second == 0 ||
       (first->type != ACTION_PUSH_BOX &&
        first->type != ACTION_PUSH_BOMB) ||
       first->type != second->type)
        return 0U;
    left = &first->push_meta;
    right = &second->push_meta;
    if(left->push_dir > DIR_RIGHT || right->push_dir != left->push_dir ||
       left->box_id != right->box_id || left->n_steps == 0U ||
       right->n_steps == 0U ||
       (uint16)left->n_steps + right->n_steps > (MAP_MAX_W - 3U))
        return 0U;

    dx = (int)bomb_dx[left->push_dir];
    dy = (int)bomb_dy[left->push_dir];
    left_car_end_x = (int)left->box_start_x +
                     dx * ((int)left->n_steps - 1);
    left_car_end_y = (int)left->box_start_y +
                     dy * ((int)left->n_steps - 1);
    return (uint8)(
        left->box_target_x == right->box_start_x &&
        left->box_target_y == right->box_start_y &&
        left_car_end_x == (int)right->car_target_x &&
        left_car_end_y == (int)right->car_target_y);
}

static void compact_contiguous_push_actions(solver_output_t *out)
{
    uint8 index = 0U;

    if(out == 0) return;
    while(index + 1U < out->action_count)
    {
        action_t *first = &out->actions[index];
        action_t *second = &out->actions[index + 1U];
        if(push_actions_are_contiguous(first, second))
        {
            uint8 move;
            first->push_meta.n_steps = (uint8)(
                first->push_meta.n_steps + second->push_meta.n_steps);
            first->push_meta.box_target_x = second->push_meta.box_target_x;
            first->push_meta.box_target_y = second->push_meta.box_target_y;
            first->push_meta.wall_target_x = second->push_meta.wall_target_x;
            first->push_meta.wall_target_y = second->push_meta.wall_target_y;
            first->target_x = second->target_x;
            first->target_y = second->target_y;
            for(move = (uint8)(index + 1U);
                move + 1U < out->action_count; move++)
                out->actions[move] = out->actions[move + 1U];
            out->action_count--;
            if(out->total_pushes > 0U) out->total_pushes--;
            continue;
        }
        index++;
    }
}

static int object_index_at(const uint8 objects[MAX_BOXES], uint8 count,
                           uint8 pos)
{
    uint8 i;
    for(i = 0U; i < count; i++) {
        if(objects[i] == pos) return (int)i;
    }
    return -1;
}

static uint8 map_has_goal(const map_input_t *map, uint8 x, uint8 y)
{
    uint8 i;
    for(i = 0U; i < map->goal_count; i++) {
        if(map->goals[i][0] == x && map->goals[i][1] == y) return 1U;
    }
    return 0U;
}

static void collect_goal_positions_row_major(const map_input_t *map,
                                             uint8 positions[MAX_BOXES])
{
    uint8 x;
    uint8 y;
    uint8 count = 0U;
    memset(positions, 255, MAX_BOXES);
    for(y = 0U; y < map->height && count < map->goal_count; y++) {
        for(x = 0U; x < map->width && count < map->goal_count; x++) {
            if(map_has_goal(map, x, y)) positions[count++] = RAW_POS(y, x);
        }
    }
}

static uint8 build_raw_map(const map_input_t *map,
                           char raw[BOMB_TRACK_ROWS][BOMB_TRACK_COLS])
{
    uint8 x;
    uint8 y;
    uint8 i;

    if(map == 0 || map->width != BOMB_TRACK_COLS ||
       map->height != BOMB_TRACK_ROWS || map->bomb_count > MAX_BOXES ||
       map->box_count == 0U ||
       map->box_count > MAX_BOXES || map->box_count != map->goal_count) {
        return 0U;
    }
    for(y = 0U; y < BOMB_TRACK_ROWS; y++) {
        for(x = 0U; x < BOMB_TRACK_COLS; x++) {
            raw[y][x] = IS_WALL(map, x, y) ? '#' : '-';
        }
    }
    for(i = 0U; i < map->goal_count; i++)
        raw[map->goals[i][1]][map->goals[i][0]] = '.';
    for(i = 0U; i < map->box_count; i++)
        raw[map->boxes[i][1]][map->boxes[i][0]] = '$';
    for(i = 0U; i < map->bomb_count; i++)
        raw[map->bombs[i][1]][map->bombs[i][0]] = '*';
    raw[map->car_y][map->car_x] =
        map_has_goal(map, map->car_x, map->car_y) ? '+' : '@';
    return 1U;
}

static uint8 convert_phase1(const map_input_t *map,
                            const bomb_phase_result_t *raw,
                            solver_output_t *out)
{
    walk_builder_t walk;
    int i;
    uint8 goal_pos[MAX_BOXES] = {255U, 255U, 255U};

    memset(&walk, 0, sizeof(walk));
    walk.out = out;
    collect_goal_positions_row_major(map, goal_pos);

    for(i = 0; i < raw->path_length; i++) {
        int8 code = raw->path[i];
        const bomb_raw_state_t *cur = &raw->states[i];
        const bomb_raw_state_t *next = &raw->states[i + 1U];

        if(code >= 0 && code <= 3) {
            uint8 dir = (uint8)code;
            uint8 nr = (uint8)((int)cur->r + bomb_dy[dir]);
            uint8 nc = (uint8)((int)cur->c + bomb_dx[dir]);
            uint8 adjacent = RAW_POS(nr, nc);
            int box = object_index_at(cur->boxes, raw->box_count, adjacent);
            int bomb = object_index_at(cur->bombs, raw->bomb_count, adjacent);
            uint8 pushed_box = (uint8)(box >= 0 &&
                next->boxes[box] != cur->boxes[box]);
            uint8 pushed_bomb = (uint8)(bomb >= 0 &&
                next->bombs[bomb] != cur->bombs[bomb]);

            if(!pushed_box && !pushed_bomb) {
                if(!append_walk_step(&walk, cur->c, cur->r, nc, nr)) return 0U;
                continue;
            }
            if(!flush_walk(&walk)) return 0U;
            {
                action_t *action;
                push_meta_t *push;
                uint8 object_pos;
                uint8 target_pos;
                if(!append_action(out, &action)) return 0U;
                action->type = pushed_bomb ? ACTION_PUSH_BOMB : ACTION_PUSH_BOX;
                action->target_x = nc;
                action->target_y = nr;
                push = &action->push_meta;
                object_pos = pushed_bomb ? cur->bombs[bomb] : cur->boxes[box];
                target_pos = pushed_bomb ? next->bombs[bomb] : next->boxes[box];
                push->box_id = (uint8)(pushed_bomb ? bomb : box);
                push->box_start_x = RAW_COL(object_pos);
                push->box_start_y = RAW_ROW(object_pos);
                if(target_pos == 255U) {
                    push->box_target_x = (uint8)((int)nc + bomb_dx[dir]);
                    push->box_target_y = (uint8)((int)nr + bomb_dy[dir]);
                } else {
                    push->box_target_x = RAW_COL(target_pos);
                    push->box_target_y = RAW_ROW(target_pos);
                }
                push->wall_target_x = push->box_target_x;
                push->wall_target_y = push->box_target_y;
                push->n_steps = 1U;
                push->car_target_x = cur->c;
                push->car_target_y = cur->r;
                set_push_direction(push, dir);
                out->total_pushes++;
                out->total_push_steps++;
            }
        } else if(code == 4) {
            action_t *action;
            if(!flush_walk(&walk) || !append_action(out, &action)) return 0U;
            action->type = ACTION_WAIT;
            action->target_x = cur->c;
            action->target_y = cur->r;
            action->wait_duration = 0.5f;
        } else if(code >= 5 && code <= 8) {
            action_t *action;
            uint8 dir = (uint8)(code - 5);
            uint8 target = RAW_POS((uint8)((int)cur->r + bomb_dy[dir]),
                                   (uint8)((int)cur->c + bomb_dx[dir]));
            int box = object_index_at(cur->boxes, raw->box_count, target);
            int goal = object_index_at(goal_pos, raw->goal_count, target);
            if(!flush_walk(&walk) || !append_action(out, &action)) return 0U;
            action->type = ACTION_OBSERVE;
            action->target_x = cur->c;
            action->target_y = cur->r;
            action->theta = bomb_dir_theta[dir];
            action->observe_meta.direction = dir;
            action->observe_meta.object_type = (box >= 0 && goal >= 0) ?
                OBSERVE_OBJECT_BOTH : (box >= 0 ? OBSERVE_OBJECT_BOX :
                (goal >= 0 ? OBSERVE_OBJECT_GOAL : OBSERVE_OBJECT_NONE));
            action->observe_meta.box_index =
                (uint8)(box >= 0 ? box : 255);
            action->observe_meta.goal_index =
                (uint8)(goal >= 0 ? goal : 255);
            action->observe_meta.dwell_ms = 500U;
        } else {
            return 0U;
        }
    }
    if(!flush_walk(&walk)) return 0U;
    {
        action_t *action;
        if(!append_action(out, &action)) return 0U;
        action->type = ACTION_PHASE_END;
        action->target_x = raw->states[raw->path_length].c;
        action->target_y = raw->states[raw->path_length].r;
    }
    return 1U;
}

static uint8 convert_phase2(const bomb_phase_result_t *raw,
                            solver_output_t *out)
{
    walk_builder_t walk;
    bomb_raw_state_t state;
    uint8 goal_for_box[MAX_BOXES] = {255U, 255U, 255U};
    int i;

    if(!phase1_valid) return 0U;
    memset(&walk, 0, sizeof(walk));
    walk.out = out;
    state = phase1_result.states[phase1_result.path_length];
    for(i = 0; i < phase1_result.box_count; i++) {
        uint8 gid = raw->assignment[i];
        if(gid >= phase1_result.goal_count) return 0U;
        goal_for_box[i] = phase1_goal_pos[gid];
        out->assignment[i] = gid;
    }

    for(i = 0; i < raw->path_length; i++) {
        int8 code = raw->path[i];
        uint8 dir;
        uint8 nr;
        uint8 nc;
        uint8 adjacent;
        int box;
        if(code < 0 || code > 3) return 0U;
        dir = (uint8)code;
        nr = (uint8)((int)state.r + bomb_dy[dir]);
        nc = (uint8)((int)state.c + bomb_dx[dir]);
        adjacent = RAW_POS(nr, nc);
        box = object_index_at(state.boxes, phase1_result.box_count, adjacent);
        if(box < 0) {
            if(!append_walk_step(&walk, state.c, state.r, nc, nr)) return 0U;
        } else {
            action_t *action;
            push_meta_t *push;
            uint8 target = RAW_POS((uint8)((int)nr + bomb_dy[dir]),
                                   (uint8)((int)nc + bomb_dx[dir]));
            if(!flush_walk(&walk) || !append_action(out, &action)) return 0U;
            action->type = ACTION_PUSH_BOX;
            action->target_x = nc;
            action->target_y = nr;
            push = &action->push_meta;
            push->box_id = (uint8)box;
            push->box_start_x = nc;
            push->box_start_y = nr;
            push->box_target_x = RAW_COL(target);
            push->box_target_y = RAW_ROW(target);
            push->n_steps = 1U;
            push->car_target_x = state.c;
            push->car_target_y = state.r;
            set_push_direction(push, dir);
            state.boxes[box] = (target == goal_for_box[box]) ? 255U : target;
            out->total_pushes++;
            out->total_push_steps++;
        }
        state.r = nr;
        state.c = nc;
    }
    if(!flush_walk(&walk)) return 0U;
    return 1U;
}

solver_status_t bomb_solver_plan_phase1(const map_input_t *map,
                                        solver_output_t *out)
{
    char raw_map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS];
    uint8 i;

    if(out == 0) return SOLVER_STATUS_BAD_ARGUMENT;
    output_init(out, SOLVER_PLAN_BOMB_P1);
    phase1_valid = 0U;
    if(!build_raw_map(map, raw_map)) {
        out->status = SOLVER_STATUS_BAD_MAP;
        return out->status;
    }
    if(!bomb_solver_run_phase1(raw_map, &phase1_result, 0)) {
        out->status = SOLVER_STATUS_NO_SOLUTION;
        return out->status;
    }
    if(!convert_phase1(map, &phase1_result, out)) {
        out->action_count = 0U;
        out->status = SOLVER_STATUS_SCHEDULE_FAILED;
        return out->status;
    }
    compact_contiguous_push_actions(out);
    collect_goal_positions_row_major(map, phase1_goal_pos);
    phase1_valid = 1U;
    out->requires_observation_ids = 1U;
    out->observed_box_mask = phase1_result.observed_box_mask;
    out->observed_goal_mask = phase1_result.observed_goal_mask;
    out->required_box_observations = (uint8)(map->box_count - 1U);
    out->required_goal_observations = (uint8)(map->goal_count - 1U);
    out->blast_count = phase1_result.bomb_count;
    for(i = 0U; i < out->blast_count; i++) {
        out->blast_x[i] = phase1_result.blast_x[i];
        out->blast_y[i] = phase1_result.blast_y[i];
    }
    out->plan_check_ok = 1U;
    out->plan_check = SOLVER_PLAN_CHECK_OK;
    out->success = 1U;
    out->status = SOLVER_STATUS_OK;
    return out->status;
}

solver_status_t bomb_solver_plan_phase2(const int8 box_ids[MAX_BOXES],
                                        const int8 goal_ids[MAX_BOXES],
                                        solver_output_t *out)
{
    bomb_phase_result_t phase2;
    if(out == 0 || box_ids == 0 || goal_ids == 0)
        return SOLVER_STATUS_BAD_ARGUMENT;
    output_init(out, SOLVER_PLAN_BOMB_P2);
    if(!phase1_valid) {
        out->status = SOLVER_STATUS_BAD_MAP;
        return out->status;
    }
    if(!bomb_solver_run_phase2((const int8_t *)box_ids,
                               (const int8_t *)goal_ids, &phase2)) {
        out->status = SOLVER_STATUS_NO_ASSIGNMENT;
        return out->status;
    }
    if(!convert_phase2(&phase2, out)) {
        out->action_count = 0U;
        out->status = SOLVER_STATUS_SCHEDULE_FAILED;
        return out->status;
    }
    compact_contiguous_push_actions(out);
    out->plan_check_ok = 1U;
    out->plan_check = SOLVER_PLAN_CHECK_OK;
    out->success = 1U;
    out->status = SOLVER_STATUS_OK;
    return out->status;
}

static uint8 bomb_solver_popcount3(uint8 value)
{
    uint8 count = 0U;
    value &= 0x07U;
    while(value != 0U)
    {
        count = (uint8)(count + (value & 1U));
        value >>= 1;
    }
    return count;
}

static uint8 bomb_solver_fill_missing_id(uint8 count, int8 ids[MAX_BOXES],
                                         uint8 mask)
{
    uint8 used = 0U;
    uint8 missing_slot = 0xFFU;
    uint8 index;

    for(index = 0U; index < count; index++)
    {
        if(mask & (uint8)(1U << index))
        {
            uint8 id;
            if(ids[index] < 0 || ids[index] >= (int8)count) return 0U;
            id = (uint8)ids[index];
            if(used & (uint8)(1U << id)) return 0U;
            used |= (uint8)(1U << id);
        }
        else
        {
            if(missing_slot != 0xFFU) return 0U;
            missing_slot = index;
        }
    }
    if(missing_slot == 0xFFU) return 0U;
    for(index = 0U; index < count; index++)
    {
        if(!(used & (uint8)(1U << index)))
        {
            ids[missing_slot] = (int8)index;
            return 1U;
        }
    }
    return 0U;
}

uint8 bomb_solver_resolve_n2(uint8 count,
                             int8 box_ids[MAX_BOXES], uint8 box_mask,
                             int8 goal_ids[MAX_BOXES], uint8 goal_mask)
{
    uint8 valid_mask;

    if(count < 1U || count > MAX_BOXES || box_ids == 0 || goal_ids == 0)
        return 0U;
    valid_mask = (uint8)((1U << count) - 1U);
    if((box_mask & (uint8)~valid_mask) != 0U ||
       (goal_mask & (uint8)~valid_mask) != 0U ||
       bomb_solver_popcount3(box_mask) != (uint8)(count - 1U) ||
       bomb_solver_popcount3(goal_mask) != (uint8)(count - 1U))
        return 0U;
    if(!bomb_solver_fill_missing_id(count, box_ids, box_mask) ||
       !bomb_solver_fill_missing_id(count, goal_ids, goal_mask))
        return 0U;
    return 1U;
}
