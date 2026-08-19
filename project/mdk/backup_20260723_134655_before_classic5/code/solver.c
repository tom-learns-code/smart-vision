#include "solver.h"
#include "bomb_solver_adapter.h"
#include <stdlib.h>
#include <string.h>

#define BFS_QUEUE_SIZE       (MAP_CELLS * 2)
#define SOLVER_DEBUG_OUTPUT  0

extern push_result_t push_box_bfs(
    uint8 bx, uint8 by, uint8 gx, uint8 gy,
    map_input_t *map, uint8 *obstacles, uint8 obs_count,
    uint8 *blocked, uint8 cx, uint8 cy, map_preprocess_t *pp, int16 max_steps);

extern car_path_t car_bfs_path(uint8 sx, uint8 sy, uint8 tx, uint8 ty,
    map_input_t *map, uint8 *extra_obs, uint8 obs_count);

extern int32 hungarian(int32 cost[][4], int n, int8 assignment[]);

extern int micro_schedule_and_generate(
    push_result_t *pushes, uint8 box_count,
    uint8 car_x, uint8 car_y,
    map_input_t *map, uint8 boxes[][2],
    map_preprocess_t *pp,
    action_t *actions, uint8 *act_count,
    int8 *assignment);

const int8 DIR_DX[4] = { 0,  0, -1,  1};
const int8 DIR_DY[4] = {-1,  1,  0,  0};

static uint16 bfs_queue[BFS_QUEUE_SIZE];
static uint16 bfs_head;
static uint16 bfs_tail;
static map_preprocess_t solver_pp;
static action_t solver_tmp_actions[MAX_ACTIONS];
static action_t solver_best_actions[MAX_ACTIONS];
static int32 solver_cost[4][4];
static push_result_t solver_pushes[4][4];
static int8 solver_assign[4];
static push_result_t solver_assigned_pushes[MAX_BOXES];
static solver_mode_t solver_mode = SOLVER_MODE_CLASSIC;

void solver_set_mode(solver_mode_t mode)
{
    solver_mode = mode <= SOLVER_MODE_BOMB_IMAGE ?
                  mode : SOLVER_MODE_CLASSIC;
}

solver_mode_t solver_get_mode(void)
{
    return solver_mode;
}

const char *solver_mode_name(solver_mode_t mode)
{
    if(mode == SOLVER_MODE_IMAGE_ONLY) return "IMAGE_ONLY";
    if(mode == SOLVER_MODE_BOMB_IMAGE) return "BOMB_IMAGE";
    return "CLASSIC";
}

static inline void queue_push(uint8 x, uint8 y)
{
    bfs_queue[bfs_tail++] = ((uint16)y << 8) | x;
    if(bfs_tail >= BFS_QUEUE_SIZE) bfs_tail = 0;
}

static inline void queue_pop(uint8 *x, uint8 *y)
{
    uint16 v = bfs_queue[bfs_head++];
    if(bfs_head >= BFS_QUEUE_SIZE) bfs_head = 0;
    *x = (uint8)(v & 0xFF);
    *y = (uint8)(v >> 8);
}

static inline int queue_empty(void)
{
    return bfs_head == bfs_tail;
}

static inline void queue_clear(void)
{
    bfs_head = 0;
    bfs_tail = 0;
}

void preprocess_init(map_preprocess_t *pp, map_input_t *map)
{
    static uint8 visited[MAP_CELLS];

    pp->width = map->width;
    pp->height = map->height;
    pp->num_components = 0;
    memset(pp->component, 0, MAP_CELLS);
    memset(pp->distances, 0xFF, sizeof(pp->distances));
    memset(visited, 0, MAP_CELLS);

    for(uint8 y = 0; y < map->height; y++) {
        for(uint8 x = 0; x < map->width; x++) {
            uint16 idx = (uint16)y * MAP_MAX_W + x;
            uint8 cid;
            if(IS_WALL(map, x, y) || visited[idx]) continue;

            cid = pp->num_components++;
            queue_clear();
            visited[idx] = 1;
            pp->component[idx] = cid;
            queue_push(x, y);

            while(!queue_empty()) {
                uint8 cx;
                uint8 cy;
                queue_pop(&cx, &cy);
                for(uint8 d = 0; d < 4; d++) {
                    uint8 nx = (uint8)(cx + DIR_DX[d]);
                    uint8 ny = (uint8)(cy + DIR_DY[d]);
                    uint16 nidx;
                    if(nx >= map->width || ny >= map->height) continue;
                    if(IS_WALL(map, nx, ny)) continue;
                    nidx = (uint16)ny * MAP_MAX_W + nx;
                    if(visited[nidx]) continue;
                    visited[nidx] = 1;
                    pp->component[nidx] = cid;
                    queue_push(nx, ny);
                }
            }
        }
    }

    for(uint8 sy = 0; sy < map->height; sy++) {
        for(uint8 sx = 0; sx < map->width; sx++) {
            uint16 sidx;
            if(IS_WALL(map, sx, sy)) continue;

            sidx = (uint16)sy * MAP_MAX_W + sx;
            pp->distances[sidx][sidx] = 0;
            memset(visited, 0, MAP_CELLS);
            queue_clear();
            visited[sidx] = 1;
            queue_push(sx, sy);

            while(!queue_empty()) {
                uint8 cx;
                uint8 cy;
                uint16 cd;
                queue_pop(&cx, &cy);
                cd = pp->distances[sidx][(uint16)cy * MAP_MAX_W + cx];

                for(uint8 d = 0; d < 4; d++) {
                    uint8 nx = (uint8)(cx + DIR_DX[d]);
                    uint8 ny = (uint8)(cy + DIR_DY[d]);
                    uint16 nidx;
                    if(nx >= map->width || ny >= map->height) continue;
                    if(IS_WALL(map, nx, ny)) continue;
                    nidx = (uint16)ny * MAP_MAX_W + nx;
                    if(visited[nidx]) continue;
                    visited[nidx] = 1;
                    pp->distances[sidx][nidx] = (uint8)(cd + 1);
                    queue_push(nx, ny);
                }
            }
        }
    }
}

int16 preprocess_distance(map_preprocess_t *pp, uint8 x1, uint8 y1,
                          uint8 x2, uint8 y2)
{
    uint8 d;
    if(x1 == x2 && y1 == y2) return 0;
    if(x1 >= pp->width || y1 >= pp->height ||
       x2 >= pp->width || y2 >= pp->height) return -1;

    d = pp->distances[(uint16)y1 * MAP_MAX_W + x1][(uint16)y2 * MAP_MAX_W + x2];
    return (d == 255) ? -1 : (int16)d;
}

typedef struct {
    solver_plan_check_t code;
    uint8 action;
    uint8 step;
    uint8 x;
    uint8 y;
} plan_check_result_t;

static void plan_check_fail(plan_check_result_t *result,
                            solver_plan_check_t code,
                            uint8 action, uint8 step,
                            uint8 x, uint8 y)
{
    result->code = code;
    result->action = action;
    result->step = step;
    result->x = x;
    result->y = y;
}

static int active_box_at(uint8 box_count, const uint8 active[],
                         uint8 boxes[][2], uint8 x, uint8 y,
                         int ignore_box)
{
    for(uint8 i = 0U; i < box_count; i++) {
        if(!active[i] || (int)i == ignore_box) continue;
        if(boxes[i][0] == x && boxes[i][1] == y) return (int)i;
    }
    return -1;
}

static int active_goal_at(map_input_t *map, const uint8 active_goal[],
                          uint8 x, uint8 y)
{
    for(uint8 i = 0U; i < map->goal_count; i++) {
        if(!active_goal[i]) continue;
        if(map->goals[i][0] == x && map->goals[i][1] == y) return (int)i;
    }
    return -1;
}

static int validate_action_plan(map_input_t *map,
                                const action_t actions[], uint8 action_count,
                                plan_check_result_t *result)
{
    uint8 car_x = map->car_x;
    uint8 car_y = map->car_y;
    uint8 boxes[MAX_BOXES][2];
    uint8 active_box[MAX_BOXES] = {1U, 1U, 1U};
    uint8 active_goal[MAX_BOXES] = {1U, 1U, 1U};

    result->code = SOLVER_PLAN_CHECK_OK;
    result->action = 255U;
    result->step = 255U;
    result->x = 255U;
    result->y = 255U;
    memcpy(boxes, map->boxes, map->box_count * 2U * sizeof(uint8));

    for(uint8 bid = 0U; bid < map->box_count; bid++) {
        int gid = active_goal_at(map, active_goal, boxes[bid][0], boxes[bid][1]);
        if(gid >= 0) {
            active_box[bid] = 0U;
            active_goal[gid] = 0U;
        }
    }

    for(uint8 ai = 0U; ai < action_count; ai++) {
        const action_t *action = &actions[ai];

        if(action->type == ACTION_FREE_MOVE) {
            if(action->wp_count == 0U || action->wp_count > MAX_WP) {
                plan_check_fail(result, SOLVER_PLAN_CHECK_BAD_WAYPOINT,
                                ai, 0U, car_x, car_y);
                return 0;
            }

            for(uint8 wi = 0U; wi < action->wp_count; wi++) {
                int16 xmm = action->waypoints[wi].x_mm;
                int16 ymm = action->waypoints[wi].y_mm;
                int gx;
                int gy;

                if((xmm % SOLVER_GRID_SIZE_MM) != 0 ||
                   (ymm % SOLVER_GRID_SIZE_MM) != 0) {
                    plan_check_fail(result, SOLVER_PLAN_CHECK_BAD_WAYPOINT,
                                    ai, wi, 255U, 255U);
                    return 0;
                }
                gx = xmm / SOLVER_GRID_SIZE_MM;
                gy = ymm / SOLVER_GRID_SIZE_MM;
                if(gx < 0 || gy < 0 || gx >= map->width || gy >= map->height) {
                    plan_check_fail(result, SOLVER_PLAN_CHECK_BAD_WAYPOINT,
                                    ai, wi, (uint8)gx, (uint8)gy);
                    return 0;
                }
                if(wi == 0U) {
                    if((uint8)gx != car_x || (uint8)gy != car_y) {
                        plan_check_fail(result, SOLVER_PLAN_CHECK_NON_ADJACENT,
                                        ai, wi, (uint8)gx, (uint8)gy);
                        return 0;
                    }
                } else {
                    int delta = abs(gx - (int)car_x) + abs(gy - (int)car_y);
                    if(delta != 1) {
                        plan_check_fail(result, SOLVER_PLAN_CHECK_NON_ADJACENT,
                                        ai, wi, (uint8)gx, (uint8)gy);
                        return 0;
                    }
                }
                if(IS_WALL(map, (uint8)gx, (uint8)gy) ||
                   active_box_at(map->box_count, active_box, boxes,
                                 (uint8)gx, (uint8)gy, -1) >= 0) {
                    plan_check_fail(result, SOLVER_PLAN_CHECK_CAR_COLLISION,
                                    ai, wi, (uint8)gx, (uint8)gy);
                    return 0;
                }
                car_x = (uint8)gx;
                car_y = (uint8)gy;
            }
            if(car_x != action->target_x || car_y != action->target_y) {
                plan_check_fail(result, SOLVER_PLAN_CHECK_BAD_WAYPOINT,
                                ai, action->wp_count, car_x, car_y);
                return 0;
            }
        } else if(action->type == ACTION_PUSH_BOX) {
            const push_meta_t *push = &action->push_meta;
            uint8 bid = push->box_id;
            uint8 bx;
            uint8 by;

            if(bid >= map->box_count || !active_box[bid] ||
               push->push_dir > DIR_RIGHT || push->n_steps == 0U ||
               boxes[bid][0] != push->box_start_x ||
               boxes[bid][1] != push->box_start_y ||
               push->car_target_x != (uint8)(push->box_start_x - DIR_DX[push->push_dir]) ||
               push->car_target_y != (uint8)(push->box_start_y - DIR_DY[push->push_dir]) ||
               action->target_x != push->car_target_x ||
               action->target_y != push->car_target_y ||
               car_x != push->car_target_x || car_y != push->car_target_y) {
                plan_check_fail(result, SOLVER_PLAN_CHECK_BAD_PUSH_META,
                                ai, 0U, push->box_start_x, push->box_start_y);
                return 0;
            }

            bx = push->box_start_x;
            by = push->box_start_y;
            for(uint8 pi = 0U; pi < push->n_steps; pi++) {
                uint8 nx = (uint8)(bx + DIR_DX[push->push_dir]);
                uint8 ny = (uint8)(by + DIR_DY[push->push_dir]);
                int gid;

                if(nx >= map->width || ny >= map->height || IS_WALL(map, nx, ny) ||
                   active_box_at(map->box_count, active_box, boxes, nx, ny, bid) >= 0) {
                    plan_check_fail(result, SOLVER_PLAN_CHECK_BOX_COLLISION,
                                    ai, pi, nx, ny);
                    return 0;
                }
                car_x = bx;
                car_y = by;
                bx = nx;
                by = ny;
                boxes[bid][0] = bx;
                boxes[bid][1] = by;

                gid = active_goal_at(map, active_goal, bx, by);
                if(gid >= 0) {
                    if((uint8)(pi + 1U) != push->n_steps) {
                        plan_check_fail(result, SOLVER_PLAN_CHECK_EARLY_GOAL,
                                        ai, pi, bx, by);
                        return 0;
                    }
                    active_box[bid] = 0U;
                    active_goal[gid] = 0U;
                }
            }
            if(bx != push->box_target_x || by != push->box_target_y) {
                plan_check_fail(result, SOLVER_PLAN_CHECK_BAD_PUSH_META,
                                ai, push->n_steps, bx, by);
                return 0;
            }
        } else {
            plan_check_fail(result, SOLVER_PLAN_CHECK_BAD_ACTION,
                            ai, 0U, car_x, car_y);
            return 0;
        }
    }

    for(uint8 bid = 0U; bid < map->box_count; bid++) {
        if(active_box[bid]) {
            plan_check_fail(result, SOLVER_PLAN_CHECK_BOXES_REMAIN,
                            action_count, bid, boxes[bid][0], boxes[bid][1]);
            return 0;
        }
    }
    for(uint8 gid = 0U; gid < map->goal_count; gid++) {
        if(active_goal[gid]) {
            plan_check_fail(result, SOLVER_PLAN_CHECK_GOAL_MISSING,
                            action_count, gid,
                            map->goals[gid][0], map->goals[gid][1]);
            return 0;
        }
    }
    return 1;
}

static void copy_plan_check(solver_output_t *out,
                            const plan_check_result_t *result)
{
    out->plan_check_ok = (result->code == SOLVER_PLAN_CHECK_OK) ? 1U : 0U;
    out->plan_check = result->code;
    out->plan_check_action = result->action;
    out->plan_check_step = result->step;
    out->plan_check_x = result->x;
    out->plan_check_y = result->y;
}

static int try_permutation(uint8 *order, uint8 *goal_for_box,
    uint8 box_count, uint8 car_x, uint8 car_y, map_input_t *map,
    map_preprocess_t *pp, action_t *best_actions,
    uint8 *best_count, int32 *best_dist, uint8 *best_assignment)
{
    action_t *tmp_actions = solver_tmp_actions;
    uint8 tmp_count = 0;
    uint8 cur_cx = car_x;
    uint8 cur_cy = car_y;
    uint8 cur_boxes[MAX_BOXES][2];
    uint8 active_box[MAX_BOXES] = {1U, 1U, 1U};
    int32 total = 0;
    plan_check_result_t check;

    memcpy(cur_boxes, map->boxes, box_count * 2U * sizeof(uint8));

    for(uint8 oi = 0; oi < box_count; oi++) {
        uint8 bid = order[oi];
        uint8 gid = goal_for_box[bid];
        uint8 gx = map->goals[gid][0];
        uint8 gy = map->goals[gid][1];
        uint8 obs[MAX_BOXES * 2];
        uint8 obs_cnt = 0;
        push_result_t pr;

        for(uint8 j = 0; j < box_count; j++) {
            if(j != bid && active_box[j]) {
                obs[obs_cnt * 2] = cur_boxes[j][0];
                obs[obs_cnt * 2 + 1] = cur_boxes[j][1];
                obs_cnt++;
            }
        }

        pr = push_box_bfs(cur_boxes[bid][0], cur_boxes[bid][1], gx, gy,
                          map, obs, obs_cnt, 0, cur_cx, cur_cy, pp, 200);
        if(pr.steps < 0) return -1;

        for(uint16 i = 0; i < pr.path_len; ) {
            uint8 seg_dir = pr.path[i].dir;
            uint8 seg_start_x = cur_boxes[bid][0];
            uint8 seg_start_y = cur_boxes[bid][1];
            uint8 seg_steps = 0;
            push_step_t *last_seg;
            uint8 ct_x;
            uint8 ct_y;
            uint8 ce_x;
            uint8 ce_y;

            while(i < pr.path_len && pr.path[i].dir == seg_dir) {
                seg_steps++;
                i++;
            }

            last_seg = &pr.path[i - 1];
            ct_x = (uint8)(seg_start_x - DIR_DX[seg_dir]);
            ct_y = (uint8)(seg_start_y - DIR_DY[seg_dir]);
            ce_x = (uint8)(last_seg->box_x - DIR_DX[seg_dir]);
            ce_y = (uint8)(last_seg->box_y - DIR_DY[seg_dir]);

            if(cur_cx != ct_x || cur_cy != ct_y) {
                uint8 car_obs[MAX_BOXES * 2];
                uint8 car_obs_count = 0U;
                car_path_t cp;
                action_t *a;

                if(tmp_count >= MAX_ACTIONS) return -1;
                a = &tmp_actions[tmp_count++];

                for(uint8 j = 0U; j < box_count; j++)
                {
                    if(!active_box[j]) continue;
                    car_obs[car_obs_count * 2U] = cur_boxes[j][0];
                    car_obs[car_obs_count * 2U + 1U] = cur_boxes[j][1];
                    car_obs_count++;
                }
                cp = car_bfs_path(cur_cx, cur_cy, ct_x, ct_y,
                                  map, car_obs, car_obs_count);
                if(cp.dist < 0) return -1;
                if(cp.path_len > MAX_WP) return -1;

                memset(a, 0, sizeof(action_t));
                a->type = ACTION_FREE_MOVE;
                a->target_x = ct_x;
                a->target_y = ct_y;
                a->theta = NAN;
                a->wp_count = (uint8)cp.path_len;
                for(uint8 w = 0U; w < a->wp_count; w++)
                {
                    a->waypoints[w].x_mm = (int16)cp.px[w] * SOLVER_GRID_SIZE_MM;
                    a->waypoints[w].y_mm = (int16)cp.py[w] * SOLVER_GRID_SIZE_MM;
                }
                total += cp.dist;
            }

            {
                action_t *pa;
                if(tmp_count >= MAX_ACTIONS) return -1;
                pa = &tmp_actions[tmp_count++];
                memset(pa, 0, sizeof(action_t));
                pa->type = ACTION_PUSH_BOX;
                pa->target_x = ct_x;
                pa->target_y = ct_y;
                pa->theta = NAN;
                pa->push_meta.box_id = bid;
                pa->push_meta.box_start_x = seg_start_x;
                pa->push_meta.box_start_y = seg_start_y;
                pa->push_meta.box_target_x = last_seg->box_x;
                pa->push_meta.box_target_y = last_seg->box_y;
                pa->push_meta.push_dir = seg_dir;
                pa->push_meta.n_steps = seg_steps;
                pa->push_meta.car_target_x = ct_x;
                pa->push_meta.car_target_y = ct_y;
                strcpy(pa->push_meta.push_dir_str, PUSH_DIR_UP);
                if(seg_dir == DIR_DOWN) strcpy(pa->push_meta.push_dir_str, PUSH_DIR_DOWN);
                else if(seg_dir == DIR_LEFT) strcpy(pa->push_meta.push_dir_str, PUSH_DIR_LEFT);
                else if(seg_dir == DIR_RIGHT) strcpy(pa->push_meta.push_dir_str, PUSH_DIR_RIGHT);
                total += seg_steps;
            }

            cur_cx = ce_x;
            cur_cy = ce_y;
            cur_boxes[bid][0] = last_seg->box_x;
            cur_boxes[bid][1] = last_seg->box_y;
        }
        if(cur_boxes[bid][0] != gx || cur_boxes[bid][1] != gy) return -1;
        active_box[bid] = 0U;
    }

    if(!validate_action_plan(map, tmp_actions, tmp_count, &check)) return -1;
    if(*best_dist < 0 || total < *best_dist) {
        *best_dist = total;
        memcpy(best_actions, tmp_actions, tmp_count * sizeof(action_t));
        memcpy(best_assignment, goal_for_box, box_count * sizeof(uint8));
        *best_count = tmp_count;
    }

    return 0;
}

typedef struct {
    uint8 box_count;
    uint8 car_x;
    uint8 car_y;
    map_input_t *map;
    map_preprocess_t *pp;
    uint8 order[MAX_BOXES];
    uint8 order_used[MAX_BOXES];
    uint8 goal_for_box[MAX_BOXES];
    uint8 goal_used[MAX_BOXES];
    uint8 best_assignment[MAX_BOXES];
    uint8 best_count;
    int32 best_dist;
} fallback_context_t;

static fallback_context_t fallback_context;

static void fallback_order_dfs(fallback_context_t *ctx, uint8 depth)
{
    if(depth == ctx->box_count) {
        (void)try_permutation(ctx->order, ctx->goal_for_box,
                              ctx->box_count, ctx->car_x, ctx->car_y,
                              ctx->map, ctx->pp, solver_best_actions,
                              &ctx->best_count, &ctx->best_dist,
                              ctx->best_assignment);
        return;
    }

    for(uint8 bid = 0U; bid < ctx->box_count; bid++) {
        if(ctx->order_used[bid]) continue;
        ctx->order_used[bid] = 1U;
        ctx->order[depth] = bid;
        fallback_order_dfs(ctx, (uint8)(depth + 1U));
        ctx->order_used[bid] = 0U;
    }
}

static void fallback_goal_dfs(fallback_context_t *ctx, uint8 bid)
{
    if(bid == ctx->box_count) {
        memset(ctx->order_used, 0, sizeof(ctx->order_used));
        fallback_order_dfs(ctx, 0U);
        return;
    }

    for(uint8 gid = 0U; gid < ctx->box_count; gid++) {
        if(ctx->goal_used[gid]) continue;
        ctx->goal_used[gid] = 1U;
        ctx->goal_for_box[bid] = gid;
        fallback_goal_dfs(ctx, (uint8)(bid + 1U));
        ctx->goal_used[gid] = 0U;
    }
}

static int solve_fallback(uint8 box_count, uint8 car_x, uint8 car_y,
                          map_input_t *map, map_preprocess_t *pp,
                          solver_output_t *out)
{
    fallback_context_t *ctx = &fallback_context;
    plan_check_result_t check;

    memset(ctx, 0, sizeof(*ctx));
    ctx->box_count = box_count;
    ctx->car_x = car_x;
    ctx->car_y = car_y;
    ctx->map = map;
    ctx->pp = pp;
    ctx->best_dist = -1;
    fallback_goal_dfs(ctx, 0U);

    if(ctx->best_dist < 0) return -1;
    memcpy(out->actions, solver_best_actions,
           ctx->best_count * sizeof(action_t));
    memcpy(out->assignment, ctx->best_assignment,
           box_count * sizeof(uint8));
    out->action_count = ctx->best_count;
    out->assignment_cost = 0;
    for(uint8 bid = 0U; bid < box_count; bid++) {
        out->assignment_cost += solver_cost[bid][ctx->best_assignment[bid]];
    }
    if(!validate_action_plan(map, out->actions, out->action_count, &check)) {
        copy_plan_check(out, &check);
        return -1;
    }
    copy_plan_check(out, &check);
    out->success = 1;
    return 0;
}

solver_status_t solver_solve(map_input_t *map, solver_output_t *out)
{
    uint8 n;
    int32 total_cost;
    int ret;
    plan_check_result_t check;

    if(out == 0)
    {
        return SOLVER_STATUS_BAD_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    out->status = SOLVER_STATUS_BAD_ARGUMENT;
    out->plan_check = SOLVER_PLAN_CHECK_NOT_RUN;
    out->plan_check_action = 255U;
    out->plan_check_step = 255U;
    out->plan_check_x = 255U;
    out->plan_check_y = 255U;
    if(map == 0)
    {
        return out->status;
    }
    if(solver_mode == SOLVER_MODE_IMAGE_ONLY)
    {
        if(map->bomb_count != 0U)
        {
            out->status = SOLVER_STATUS_BAD_MAP;
            return out->status;
        }
        return bomb_solver_plan_phase1(map, out);
    }
    if(solver_mode == SOLVER_MODE_BOMB_IMAGE)
    {
        if(map->bomb_count == 0U)
        {
            out->status = SOLVER_STATUS_BAD_MAP;
            return out->status;
        }
        return bomb_solver_plan_phase1(map, out);
    }
    if(map->bomb_count > 0U)
    {
        out->status = SOLVER_STATUS_BAD_MAP;
        return out->status;
    }
    if(map->width == 0U || map->width > MAP_MAX_W ||
       map->height == 0U || map->height > MAP_MAX_H)
    {
        out->status = SOLVER_STATUS_BAD_MAP;
        return out->status;
    }

    n = map->box_count;
    if(n == 0) {
        out->success = 1;
        out->status = SOLVER_STATUS_OK;
        check.code = SOLVER_PLAN_CHECK_OK;
        check.action = 255U;
        check.step = 255U;
        check.x = 255U;
        check.y = 255U;
        copy_plan_check(out, &check);
        return out->status;
    }
    if(n > MAX_BOXES || n != map->goal_count)
    {
        out->status = SOLVER_STATUS_BAD_MAP;
        return out->status;
    }

    preprocess_init(&solver_pp, map);
    memset(solver_pushes, 0, sizeof(solver_pushes));
    memset(solver_assign, -1, sizeof(solver_assign));

    for(uint8 i = 0; i < n; i++) {
        for(uint8 j = 0; j < n; j++) {
            push_result_t pr = push_box_bfs(
                map->boxes[i][0], map->boxes[i][1],
                map->goals[j][0], map->goals[j][1],
                map, 0, 0, 0,
                map->car_x, map->car_y, &solver_pp, 200);
            solver_pushes[i][j] = pr;
            solver_cost[i][j] = (pr.steps < 0) ? 32767 : (pr.steps + pr.turns * 2);
        }
    }

    total_cost = hungarian(solver_cost, n, solver_assign);
    out->assignment_cost = total_cost;
    if(total_cost >= 16384) {
        out->status = SOLVER_STATUS_NO_ASSIGNMENT;
        return out->status;
    }

#if SOLVER_DEBUG_OUTPUT
    for(uint8 i = 0; i < n; i++) fprintf(stderr, "Box%d->Goal%d ", i, solver_assign[i]);
    fprintf(stderr, "total=%d\n", total_cost);
#endif

    for(uint8 i = 0; i < n; i++) {
        solver_assigned_pushes[i] = solver_pushes[i][solver_assign[i]];
    }
    memcpy(out->assignment, solver_assign, n);

    ret = micro_schedule_and_generate(
        solver_assigned_pushes, n,
        map->car_x, map->car_y,
        map, map->boxes, &solver_pp,
        out->actions, &out->action_count, solver_assign);

    if(ret == 0 && validate_action_plan(map, out->actions,
                                        out->action_count, &check)) {
        copy_plan_check(out, &check);
        out->success = 1;
    } else {
        if(ret == 0) {
            copy_plan_check(out, &check);
            ret = -4;
        }
        out->action_count = 0U;
    }

    if(!out->success && solve_fallback(n, map->car_x, map->car_y,
                                       map, &solver_pp, out) == 0) {
        out->success = 1;
        out->used_fallback = 1U;
    }

    if(out->success) {
        for(uint8 i = 0; i < out->action_count; i++) {
            if(out->actions[i].type == ACTION_PUSH_BOX)
            {
                out->total_pushes++;
                out->total_push_steps += out->actions[i].push_meta.n_steps;
            }
            else if(out->actions[i].type == ACTION_FREE_MOVE)
            {
                out->total_free_moves++;
                out->total_waypoints += out->actions[i].wp_count;
            }
        }
        out->status = SOLVER_STATUS_OK;
    }
    else
    {
        out->status = (ret == 0) ? SOLVER_STATUS_NO_SOLUTION :
                                   SOLVER_STATUS_SCHEDULE_FAILED;
    }

    return out->status;
}

const char *solver_status_name(solver_status_t status)
{
    switch(status)
    {
        case SOLVER_STATUS_OK: return "OK";
        case SOLVER_STATUS_BAD_ARGUMENT: return "BAD_ARGUMENT";
        case SOLVER_STATUS_BAD_MAP: return "BAD_MAP";
        case SOLVER_STATUS_NO_ASSIGNMENT: return "NO_ASSIGNMENT";
        case SOLVER_STATUS_SCHEDULE_FAILED: return "SCHEDULE_FAILED";
        case SOLVER_STATUS_NO_SOLUTION: return "NO_SOLUTION";
        default: return "UNKNOWN";
    }
}

const char *solver_plan_check_name(solver_plan_check_t check)
{
    switch(check)
    {
        case SOLVER_PLAN_CHECK_NOT_RUN: return "NOT_RUN";
        case SOLVER_PLAN_CHECK_OK: return "OK";
        case SOLVER_PLAN_CHECK_BAD_ACTION: return "BAD_ACTION";
        case SOLVER_PLAN_CHECK_BAD_WAYPOINT: return "BAD_WAYPOINT";
        case SOLVER_PLAN_CHECK_NON_ADJACENT: return "NON_ADJACENT";
        case SOLVER_PLAN_CHECK_CAR_COLLISION: return "CAR_COLLISION";
        case SOLVER_PLAN_CHECK_BAD_PUSH_META: return "BAD_PUSH_META";
        case SOLVER_PLAN_CHECK_BOX_COLLISION: return "BOX_COLLISION";
        case SOLVER_PLAN_CHECK_EARLY_GOAL: return "EARLY_GOAL";
        case SOLVER_PLAN_CHECK_GOAL_MISSING: return "GOAL_MISSING";
        case SOLVER_PLAN_CHECK_BOXES_REMAIN: return "BOXES_REMAIN";
        default: return "UNKNOWN";
    }
}
