#include <stdlib.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "blue.h"
#include "motion_control.h"
#include "vision_link.h"
#include "point_test.h"
#include "roi_camera_link.h"
#include "bomb_solver_adapter.h"
#include "planner_service.h"
#include "status_buzzer.h"
#include "mission_manager.h"
#include "app_config.h"

#define MISSION_ARM_POSE_TOLERANCE_X10 (6)
#define MISSION_PUSH_VERIFY_TIMEOUT_TICKS       (1600U) /* 8 s */
#define MISSION_PUSH_VERIFY_RETRY_TICKS         (100U)  /* 500 ms */
#define MISSION_PUSH_VERIFY_MIN_MISMATCH_TICKS  (600U)  /* 3 s */
#define MISSION_PUSH_VERIFY_MAX_MAPS            (6U)
#define MISSION_FAST_MAX_SEGMENT_CELLS   (APP_FOLLOWER_NORMAL_MAX_CELLS)
#define MISSION_MULTI_PUSH_MIN_STEPS      (APP_MISSION_MULTI_PUSH_MIN_STEPS)
#define MISSION_MULTI_PUSH_MAX_CRUISE     (APP_MISSION_MULTI_PUSH_MAX_CRUISE)
#define MISSION_TERMINAL_START_TOLERANCE_MM (280U)
#define MISSION_TERMINAL_ALIGN_MAX_MM       (300U)
#define MISSION_NORMAL_DRAIN_MS           (1000U)
#define MISSION_NORMAL_STABLE_FRAMES      (6U)
#define MISSION_NORMAL_STABLE_MS          (200U)
#define MISSION_PUSH_DRAIN_MS             (APP_MISSION_PUSH_DRAIN_MS)
#define MISSION_PUSH_STABLE_FRAMES        (APP_MISSION_PUSH_STABLE_FRAMES)
#define MISSION_PUSH_STABLE_MS            (100U)
#define MISSION_OBSERVE_ROTATE_SPEED        (APP_MISSION_OBSERVE_SPEED)
#define MISSION_OBSERVE_SETTLE_MIN_TICKS   (60U)  /* 300 ms. */
#define MISSION_OBSERVE_POSE_STALE_TICKS   (60U)  /* 300 ms. */
#define MISSION_OBSERVE_POSE_WAIT_TICKS    (APP_MISSION_OBSERVE_TIMEOUT_MS / 5U)
#define MISSION_OBSERVE_ALIGN_MAX_MM        (180U)
#define MISSION_POST_PUSH_MIN_TICKS          (60U)  /* 300 ms after follower hold. */
#define MISSION_POST_PUSH_TIMEOUT_TICKS     (400U)  /* 2 s. */
#define MISSION_POST_PUSH_STABLE_TICKS       (40U)  /* 200 ms. */
#define MISSION_POST_PUSH_STABLE_FRAMES       (4U)
#define MISSION_POST_PUSH_CLUSTER_X10          (2)
#define MISSION_POST_PUSH_TARGET_X10           (1)
#define MISSION_POST_PUSH_RAMP_MS            (300U)
#define MISSION_POST_PUSH_RAMP_DELTA         (150U)
#define MISSION_POST_PUSH_NONE                 (0U)
#define MISSION_POST_PUSH_STABLE               (1U)
#define MISSION_POST_PUSH_FALLBACK             (2U)
#define MISSION_POST_PUSH_MISMATCH             (3U)
#define MISSION_START_WAIT_TICKS              (1000U) /* 5 s. */
#define MISSION_START_STABLE_TICKS              (40U) /* 200 ms. */
#define MISSION_START_STABLE_FRAMES              (6U)
#define MISSION_START_CLUSTER_X10                (2)  /* 40 mm. */
#define MISSION_START_TARGET_X10                 (1)  /* 20 mm per axis. */
#define MISSION_START_ALIGN_MAX_MM             (180U)
#define MISSION_START_NONE                       (0U)
#define MISSION_START_CENTERED                   (1U)
#define MISSION_START_ALIGNED                    (2U)

typedef struct {
    action_follower_step_context_t follower;
    uint8 box_near;
    uint8 bomb_near;
    uint8 object_near;
    uint8 goal_near;
    uint8 transition;
    uint8 merge_eligible;
    uint8 wall_relation;
} mission_exec_context_t;

typedef struct {
    uint8 car_from_x;
    uint8 car_from_y;
    uint8 car_to_x;
    uint8 car_to_y;
    uint8 object_from_x;
    uint8 object_from_y;
    uint8 object_to_x;
    uint8 object_to_y;
    uint8 is_bomb;
    uint8 detonate;
    uint8 box_on_goal;
} mission_push_projection_t;

static mission_status_t mission_status;
static const solver_output_t *mission_plan;
static vision_link_map_t mission_expected_map;
static vision_link_map_t mission_push_expected_map;
static uint8 mission_expected_map_valid;
static uint32 mission_push_verify_start_tick;
static uint32 mission_push_verify_request_tick;
static uint8 mission_ignored_car_box_valid;
static uint8 mission_ignored_car_box_x;
static uint8 mission_ignored_car_box_y;
static uint32 mission_ignored_car_box_map_version;
static int8 mission_box_ids[MAX_BOXES];
static int8 mission_goal_ids[MAX_BOXES];
static uint8 mission_observe_pending_type;
static uint8 mission_observe_pending_slot;
static uint8 mission_observe_second_type;
static uint8 mission_observe_second_slot;
static uint16 mission_observe_rotation_deg;
static uint8 mission_observe_rotation_clockwise;
static uint32 mission_observe_state_tick;
static uint32 mission_post_push_start_tick;
static uint32 mission_post_push_cluster_tick;
static uint32 mission_post_push_last_packets;
static uint8 mission_post_push_last_frame;
static int16 mission_post_push_anchor_x10;
static int16 mission_post_push_anchor_y10;
static uint32 mission_start_reanchor_start_tick;
static uint32 mission_start_reanchor_cluster_tick;
static uint32 mission_start_reanchor_last_packets;
static uint8 mission_start_reanchor_last_frame;
static int16 mission_start_reanchor_anchor_x10;
static int16 mission_start_reanchor_anchor_y10;
static uint8 mission_force_logical_origin_once;
static uint8 mission_terminal_pending;
static uint8 mission_terminal_from_x;
static uint8 mission_terminal_from_y;
static uint8 mission_terminal_to_x;
static uint8 mission_terminal_to_y;
static mission_exec_context_t mission_terminal_context;

static mission_result_t mission_start_current_step(uint8 allow_push);
static void mission_set_state(mission_state_t state,
                              mission_result_t result);

static void mission_clear_segment_runtime(void)
{
    mission_terminal_pending = 0U;
    mission_terminal_from_x = 0U;
    mission_terminal_from_y = 0U;
    mission_terminal_to_x = 0U;
    mission_terminal_to_y = 0U;
    memset(&mission_terminal_context, 0,
           sizeof(mission_terminal_context));
    mission_status.segment_phase = MISSION_SEGMENT_SINGLE;
    mission_status.segment_total_cells = 1U;
    mission_status.segment_cruise_cells = 0U;
    mission_status.segment_node_x = 0U;
    mission_status.segment_node_y = 0U;
}

static uint8 mission_round_grid_x10(int16 value)
{
    if(value <= 0) return 0U;
    return (uint8)(((uint16)value + 5U) / 10U);
}

static uint8 mission_observe_pose_fresh(
    const vision_link_snapshot_t *pose)
{
    return (uint8)(pose != 0 && vision_link_is_online() &&
        pose->pose_valid && pose->last_packet_tick != 0U &&
        (uint32)(pit_count - pose->last_packet_tick) <=
            MISSION_OBSERVE_POSE_STALE_TICKS);
}

static void mission_pause_for_pose_replan(void)
{
    motion_heading_lock_release();
    pentagram_enable = 0U;
    device_init_flag = 1;
    motion_fast_brake();
    motion_emergency_stop();
    mission_status.auto_run = 0U;
    mission_status.armed = 0U;
    mission_status.observe_wait_ms = 0U;
    mission_set_state(MISSION_PAUSED, MISSION_RESULT_REPLAN_REQUIRED);
    status_buzzer_request(STATUS_BUZZER_EVENT_LOCKED);
}

static void mission_wait_for_observe_pose(void)
{
    motion_heading_lock_stop();
    mission_observe_state_tick = pit_count;
    mission_status.observe_wait_ms = MISSION_OBSERVE_POSE_WAIT_TICKS * 5U;
    mission_set_state(MISSION_OBSERVE_POSE_WAIT, MISSION_RESULT_OK);
}

static uint8 mission_action_step_count(const action_t *action)
{
    if(action == 0) return 0U;
    if(action->type == ACTION_FREE_MOVE)
        return action->wp_count > 1U ? (uint8)(action->wp_count - 1U) : 0U;
    if(action->type == ACTION_PUSH_BOX || action->type == ACTION_PUSH_BOMB)
        return action->push_meta.n_steps;
    return 0U;
}

uint16 mission_manager_count_plan_steps(const solver_output_t *plan)
{
    uint16 count = 0U;
    uint8 i;

    if(plan == 0) return 0U;
    for(i = 0U; i < plan->action_count; i++)
        count += mission_action_step_count(&plan->actions[i]);
    return count;
}

static uint8 mission_cell_in_map(const vision_link_map_t *map, int x, int y)
{
    return (uint8)(map != 0 && x >= 0 && y >= 0 &&
                   x < (int)map->width && y < (int)map->height);
}

static uint8 mission_wall_at(const vision_link_map_t *map, uint8 x, uint8 y)
{
    uint16 index = (uint16)y * VISION_LINK_GRID_W + x;
    return (uint8)((map->wall_bits[index >> 3] >> (index & 7U)) & 1U);
}

static uint8 mission_cell_list_has(const vision_link_cell_t *cells,
                                   uint8 count, uint8 x, uint8 y)
{
    uint8 i;

    for(i = 0U; i < count; i++)
    {
        if(cells[i].gx == (int8)x && cells[i].gy == (int8)y) return 1U;
    }
    return 0U;
}

static uint8 mission_cell_near_list(const vision_link_cell_t *cells,
                                    uint8 count, uint8 x, uint8 y)
{
    uint8 i;

    for(i = 0U; i < count; i++)
    {
        if(abs((int)cells[i].gx - (int)x) <= 1 &&
           abs((int)cells[i].gy - (int)y) <= 1)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8 mission_wall_side_at(const vision_link_map_t *map,
                                  uint8 x, uint8 y, int step_dx, int step_dy)
{
    uint8 side = 0U;

    if(step_dy != 0)
    {
        if(x > 0U && mission_wall_at(map, (uint8)(x - 1U), y)) side |= 1U;
        if(x + 1U < map->width &&
           mission_wall_at(map, (uint8)(x + 1U), y)) side |= 2U;
    }
    else if(step_dx != 0)
    {
        if(y > 0U && mission_wall_at(map, x, (uint8)(y - 1U))) side |= 1U;
        if(y + 1U < map->height &&
           mission_wall_at(map, x, (uint8)(y + 1U))) side |= 2U;
    }
    return side;
}

static uint8 mission_build_exec_context(const vision_link_map_t *map,
                                        uint8 from_x, uint8 from_y,
                                        uint8 to_x, uint8 to_y,
                                        uint8 interaction,
                                        mission_exec_context_t *out)
{
    int step_dx = (int)to_x - (int)from_x;
    int step_dy = (int)to_y - (int)from_y;
    uint8 from_side;
    uint8 to_side;
    uint8 side;

    if(map == 0 || out == 0 || !map->valid ||
       ((step_dx == 0) == (step_dy == 0))) return 0U;
    memset(out, 0, sizeof(*out));
    step_dx = step_dx > 0 ? 1 : (step_dx < 0 ? -1 : 0);
    step_dy = step_dy > 0 ? 1 : (step_dy < 0 ? -1 : 0);
    from_side = mission_wall_side_at(map, from_x, from_y, step_dx, step_dy);
    to_side = mission_wall_side_at(map, to_x, to_y, step_dx, step_dy);
    side = (uint8)(from_side | to_side);

    out->box_near = (uint8)(
        mission_cell_near_list(map->boxes, map->box_count, from_x, from_y) ||
        mission_cell_near_list(map->boxes, map->box_count, to_x, to_y));
    out->bomb_near = (uint8)(
        mission_cell_near_list(map->bombs, map->bomb_count, from_x, from_y) ||
        mission_cell_near_list(map->bombs, map->bomb_count, to_x, to_y));
    out->object_near = (uint8)(out->box_near || out->bomb_near);
    out->goal_near = (uint8)(
        mission_cell_near_list(map->goals, map->goal_count, from_x, from_y) ||
        mission_cell_near_list(map->goals, map->goal_count, to_x, to_y));
    out->transition = (from_side != to_side) ? 1U : 0U;
    out->wall_relation = side;
    out->follower.segment_cells = 1U;
    out->follower.interaction_locked = interaction ? 1U : 0U;
    out->follower.strict_position = (uint8)(
        (interaction || out->object_near || out->goal_near) &&
        (side == 0U || out->transition));
    out->follower.strong_reanchor = (uint8)(out->transition ||
        out->follower.strict_position);
    out->follower.vision_drain_ms = MISSION_NORMAL_DRAIN_MS;
    out->follower.vision_stable_frames = MISSION_NORMAL_STABLE_FRAMES;
    out->follower.vision_stable_ms = MISSION_NORMAL_STABLE_MS;
    if(out->transition)
        out->follower.align_max_distance_mm =
            APP_FOLLOWER_EXIT_ALIGN_MAX_MM;
    else if(out->follower.strict_position)
        out->follower.align_max_distance_mm =
            APP_FOLLOWER_RECOVERY_MAX_MM;

    if(step_dy != 0)
    {
        out->follower.wall_axis_mask = side ? 0x01U : 0U;
        out->follower.corridor_axis_mask =
            (from_side == 3U || to_side == 3U) ? 0x01U : 0U;
        if(side == 1U || side == 2U)
        {
            int wall_x = side == 1U ? -1 : 1;
            int cross_x = step_dy < 0 ? 1 : -1;
            out->follower.single_wall_correction_sign =
                (int8)(wall_x * cross_x);
        }
    }
    else
    {
        out->follower.wall_axis_mask = side ? 0x02U : 0U;
        out->follower.corridor_axis_mask =
            (from_side == 3U || to_side == 3U) ? 0x02U : 0U;
        if(side == 1U || side == 2U)
        {
            int wall_y = side == 1U ? -1 : 1;
            int cross_y = step_dx > 0 ? 1 : -1;
            out->follower.single_wall_correction_sign =
                (int8)(wall_y * cross_y);
        }
    }

    if(interaction)
        out->follower.gear = ACTION_GEAR_PUSH_PRECISE;
    else if(out->transition)
        out->follower.gear = ACTION_GEAR_TRANSITION_REANCHOR;
    else if(out->follower.corridor_axis_mask != 0U)
        out->follower.gear = ACTION_GEAR_CORRIDOR_FAST;
    else if(side != 0U)
        out->follower.gear = ACTION_GEAR_SINGLE_WALL_FAST;
    else if(out->object_near || out->goal_near)
        out->follower.gear = ACTION_GEAR_BOX_NEAR_PRECISE;
    else
        out->follower.gear = ACTION_GEAR_OPEN_FAST;

    out->merge_eligible = (uint8)(!interaction && !out->transition &&
        ((side != 0U) || (!out->object_near && !out->goal_near)) &&
        (out->follower.gear == ACTION_GEAR_OPEN_FAST ||
         out->follower.gear == ACTION_GEAR_SINGLE_WALL_FAST ||
         out->follower.gear == ACTION_GEAR_CORRIDOR_FAST));
    return 1U;
}

static uint8 mission_remove_box_at(vision_link_map_t *map, uint8 x, uint8 y)
{
    uint8 i;

    if(map == 0) return 0U;
    for(i = 0U; i < map->box_count; i++)
    {
        if(map->boxes[i].gx == (int8)x && map->boxes[i].gy == (int8)y)
        {
            uint8 j;
            for(j = i; j + 1U < map->box_count; j++)
                map->boxes[j] = map->boxes[j + 1U];
            map->box_count--;
            return 1U;
        }
    }
    return 0U;
}

static uint8 mission_remove_bomb_at(vision_link_map_t *map, uint8 x, uint8 y)
{
    uint8 i;

    if(map == 0) return 0U;
    for(i = 0U; i < map->bomb_count; i++)
    {
        if(map->bombs[i].gx == (int8)x && map->bombs[i].gy == (int8)y)
        {
            uint8 j;
            for(j = i; j + 1U < map->bomb_count; j++)
                map->bombs[j] = map->bombs[j + 1U];
            map->bomb_count--;
            return 1U;
        }
    }
    return 0U;
}

static uint8 mission_remove_goal_at(vision_link_map_t *map, uint8 x, uint8 y)
{
    uint8 i;

    if(map == 0) return 0U;
    for(i = 0U; i < map->goal_count; i++)
    {
        if(map->goals[i].gx == (int8)x && map->goals[i].gy == (int8)y)
        {
            uint8 j;
            for(j = i; j + 1U < map->goal_count; j++)
                map->goals[j] = map->goals[j + 1U];
            map->goal_count--;
            return 1U;
        }
    }
    return 0U;
}

static uint8 mission_restore_expected_goal_at(vision_link_map_t *map,
                                              const vision_link_map_t *expected,
                                              uint8 x, uint8 y)
{
    if(map == 0 || expected == 0 ||
       !mission_cell_list_has(expected->goals, expected->goal_count, x, y) ||
       mission_cell_list_has(map->goals, map->goal_count, x, y))
    {
        return 0U;
    }
    if(map->goal_count >= VISION_LINK_MAX_GOALS) return 0U;
    map->goals[map->goal_count].gx = (int8)x;
    map->goals[map->goal_count].gy = (int8)y;
    map->goal_count++;
    return 1U;
}

static void mission_apply_cached_car_box_filter(vision_link_map_t *map)
{
    if(map != 0 && mission_ignored_car_box_valid &&
       map->map_version == mission_ignored_car_box_map_version)
    {
        mission_remove_box_at(map, mission_ignored_car_box_x,
                              mission_ignored_car_box_y);
        mission_remove_bomb_at(map, mission_ignored_car_box_x,
                               mission_ignored_car_box_y);
        mission_restore_expected_goal_at(map, &mission_expected_map,
                                         mission_ignored_car_box_x,
                                         mission_ignored_car_box_y);
    }
}

static uint8 mission_same_cell_set(const vision_link_cell_t *left,
                                   uint8 left_count,
                                   const vision_link_cell_t *right,
                                   uint8 right_count)
{
    uint8 matched = 0U;
    uint8 i;

    if(left_count != right_count || left_count > 8U) return 0U;
    for(i = 0U; i < left_count; i++)
    {
        uint8 j;
        uint8 found = 0U;
        for(j = 0U; j < right_count; j++)
        {
            if((matched & (uint8)(1U << j)) == 0U &&
               left[i].gx == right[j].gx && left[i].gy == right[j].gy)
            {
                matched |= (uint8)(1U << j);
                found = 1U;
                break;
            }
        }
        if(!found) return 0U;
    }
    return 1U;
}

static uint8 mission_goal_view_matches(const vision_link_map_t *expected,
                                       const vision_link_map_t *latest)
{
    uint8 i;

    if(expected->goal_count < latest->goal_count) return 0U;
    for(i = 0U; i < latest->goal_count; i++)
    {
        if(!mission_cell_list_has(expected->goals, expected->goal_count,
                                  (uint8)latest->goals[i].gx,
                                  (uint8)latest->goals[i].gy)) return 0U;
    }
    for(i = 0U; i < expected->goal_count; i++)
    {
        uint8 x = (uint8)expected->goals[i].gx;
        uint8 y = (uint8)expected->goals[i].gy;
        if(!mission_cell_list_has(latest->goals, latest->goal_count, x, y) &&
           !mission_cell_list_has(latest->boxes, latest->box_count, x, y))
            return 0U;
    }
    return 1U;
}

static uint8 mission_same_static_map(const vision_link_map_t *left,
                                     const vision_link_map_t *right)
{
    if(left == 0 || right == 0 || !left->valid || !right->valid ||
       left->width != right->width || left->height != right->height ||
       memcmp(left->wall_bits, right->wall_bits,
              VISION_LINK_WALL_BYTES) != 0)
    {
        return 0U;
    }
    /* A box may visually hide its goal. Keep the original goal set canonical
       and accept a missing goal only when the latest map has a box there. */
    if(!mission_goal_view_matches(left, right)) return 0U;
    if(!mission_same_cell_set(left->bombs, left->bomb_count,
                              right->bombs, right->bomb_count)) return 0U;
    return 1U;
}

static uint8 mission_same_map(const vision_link_map_t *left,
                              const vision_link_map_t *right)
{
    if(!mission_same_static_map(left, right)) return 0U;
    return mission_same_cell_set(left->boxes, left->box_count,
                                 right->boxes, right->box_count);
}

static uint8 mission_map_diff_mask(const vision_link_map_t *expected,
                                   const vision_link_map_t *latest)
{
    uint8 mask = 0U;

    if(expected == 0 || latest == 0 || !expected->valid || !latest->valid ||
       expected->width != latest->width ||
       expected->height != latest->height ||
       memcmp(expected->wall_bits, latest->wall_bits,
              VISION_LINK_WALL_BYTES) != 0)
        mask |= 0x01U;
    if(expected == 0 || latest == 0 ||
       !mission_goal_view_matches(expected, latest))
        mask |= 0x02U;
    if(expected == 0 || latest == 0 ||
       !mission_same_cell_set(expected->bombs, expected->bomb_count,
                              latest->bombs, latest->bomb_count))
        mask |= 0x04U;
    if(expected == 0 || latest == 0 ||
       !mission_same_cell_set(expected->boxes, expected->box_count,
                              latest->boxes, latest->box_count))
        mask |= 0x08U;
    return mask;
}

static uint8 mission_plan_environment_current(void)
{
    vision_link_map_t latest;

    if(!mission_expected_map_valid) return 0U;
    if(mission_status.auto_run) return 1U;
    if(!vision_link_is_online()) return 0U;
    if(!vision_link_get_map(&latest)) return 0U;
    mission_apply_cached_car_box_filter(&latest);
    return mission_same_map(&mission_expected_map, &latest);
}

static void mission_clear_wall_at(vision_link_map_t *map, uint8 x, uint8 y)
{
    uint16 index;
    if(map == 0 || x >= map->width || y >= map->height) return;
    index = (uint16)y * VISION_LINK_GRID_W + x;
    map->wall_bits[index >> 3] &= (uint8)~(1U << (index & 7U));
}

static uint8 mission_project_push_step(const action_t *action,
                                       uint8 substep,
                                       vision_link_map_t *map,
                                       mission_push_projection_t *projection)
{
    const push_meta_t *push;
    int dx;
    int dy;
    int car_from_x;
    int car_from_y;
    int box_from_x;
    int box_from_y;
    int box_to_x;
    int box_to_y;
    uint8 i;
    uint8 moved = 0U;
    uint8 is_bomb;
    uint8 detonate = 0U;

    if(action == 0 || map == 0 || projection == 0 ||
       (action->type != ACTION_PUSH_BOX &&
        action->type != ACTION_PUSH_BOMB) || !map->valid)
        return 0U;
    is_bomb = action->type == ACTION_PUSH_BOMB ? 1U : 0U;
    push = &action->push_meta;
    if(push->push_dir > DIR_RIGHT || substep >= push->n_steps) return 0U;
    dx = (int)DIR_DX[push->push_dir];
    dy = (int)DIR_DY[push->push_dir];
    car_from_x = (int)push->car_target_x + dx * substep;
    car_from_y = (int)push->car_target_y + dy * substep;
    box_from_x = (int)push->box_start_x + dx * substep;
    box_from_y = (int)push->box_start_y + dy * substep;
    box_to_x = box_from_x + dx;
    box_to_y = box_from_y + dy;

    if(!mission_cell_in_map(map, car_from_x, car_from_y) ||
       !mission_cell_in_map(map, box_from_x, box_from_y) ||
       !mission_cell_in_map(map, box_to_x, box_to_y) ||
       car_from_x + dx != box_from_x ||
       car_from_y + dy != box_from_y)
    {
        return 0U;
    }
    detonate = (uint8)(is_bomb &&
        mission_wall_at(map, (uint8)box_to_x, (uint8)box_to_y));
    if(mission_wall_at(map, (uint8)car_from_x, (uint8)car_from_y) ||
       mission_cell_list_has(map->bombs, map->bomb_count,
                             (uint8)car_from_x, (uint8)car_from_y) ||
       mission_cell_list_has(map->boxes, map->box_count,
                             (uint8)car_from_x, (uint8)car_from_y) ||
       (is_bomb ?
          !mission_cell_list_has(map->bombs, map->bomb_count,
                                 (uint8)box_from_x, (uint8)box_from_y) :
          !mission_cell_list_has(map->boxes, map->box_count,
                                 (uint8)box_from_x, (uint8)box_from_y)) ||
       (!is_bomb &&
         mission_wall_at(map, (uint8)box_to_x, (uint8)box_to_y)) ||
       (!detonate &&
         mission_cell_list_has(map->bombs, map->bomb_count,
                               (uint8)box_to_x, (uint8)box_to_y)) ||
       mission_cell_list_has(map->boxes, map->box_count,
                             (uint8)box_to_x, (uint8)box_to_y))
    {
        return 0U;
    }

    if(is_bomb)
    {
        for(i = 0U; i < map->bomb_count; i++)
        {
            if(map->bombs[i].gx == (int8)box_from_x &&
               map->bombs[i].gy == (int8)box_from_y)
            {
                uint8 j;
                if(moved) return 0U;
                moved = 1U;
                if(detonate)
                {
                    for(j = i; j + 1U < map->bomb_count; j++)
                        map->bombs[j] = map->bombs[j + 1U];
                    map->bomb_count--;
                }
                else
                {
                    map->bombs[i].gx = (int8)box_to_x;
                    map->bombs[i].gy = (int8)box_to_y;
                }
                break;
            }
        }
        if(detonate)
        {
            int clear_x;
            int clear_y;
            for(clear_y = box_to_y - 1; clear_y <= box_to_y + 1; clear_y++)
                for(clear_x = box_to_x - 1; clear_x <= box_to_x + 1;
                    clear_x++)
                    if(clear_x >= 0 && clear_y >= 0 &&
                       clear_x < (int)map->width &&
                       clear_y < (int)map->height)
                        mission_clear_wall_at(map,
                                              (uint8)clear_x,
                                              (uint8)clear_y);
        }
    }
    else
    {
        for(i = 0U; i < map->box_count; i++)
        {
            if(map->boxes[i].gx == (int8)box_from_x &&
               map->boxes[i].gy == (int8)box_from_y)
            {
                if(moved) return 0U;
                map->boxes[i].gx = (int8)box_to_x;
                map->boxes[i].gy = (int8)box_to_y;
                moved = 1U;
            }
        }
    }
    if(!moved) return 0U;

    memset(projection, 0, sizeof(*projection));
    projection->car_from_x = (uint8)car_from_x;
    projection->car_from_y = (uint8)car_from_y;
    projection->car_to_x = (uint8)box_from_x;
    projection->car_to_y = (uint8)box_from_y;
    projection->object_from_x = (uint8)box_from_x;
    projection->object_from_y = (uint8)box_from_y;
    projection->object_to_x = (uint8)box_to_x;
    projection->object_to_y = (uint8)box_to_y;
    projection->is_bomb = is_bomb;
    projection->detonate = detonate;
    projection->box_on_goal = (uint8)(!is_bomb &&
        box_to_x == (int)push->box_target_x &&
        box_to_y == (int)push->box_target_y &&
        mission_cell_list_has(map->goals, map->goal_count,
                              (uint8)box_to_x, (uint8)box_to_y));
    return 1U;
}

static uint8 mission_prepare_push_segment(const action_t *action,
                                          uint8 segment_cells,
                                          uint8 *from_x, uint8 *from_y,
                                          uint8 *to_x, uint8 *to_y)
{
    vision_link_map_t latest;
    vision_link_map_t projected;
    mission_push_projection_t first;
    mission_push_projection_t current;
    uint8 p;
    uint8 i;

    if(action == 0 || segment_cells == 0U ||
       !mission_expected_map_valid) return 0U;
    if(mission_status.auto_run)
    {
        latest = mission_expected_map;
    }
    else
    {
        if(!vision_link_get_map(&latest)) return 0U;
        mission_apply_cached_car_box_filter(&latest);
        if(!mission_same_map(&mission_expected_map, &latest)) return 0U;
    }

    p = mission_status.substep_cursor;
    if((uint16)p + segment_cells > action->push_meta.n_steps) return 0U;
    projected = latest;
    memset(&first, 0, sizeof(first));
    memset(&current, 0, sizeof(current));
    for(i = 0U; i < segment_cells; i++)
    {
        if(!mission_project_push_step(action, (uint8)(p + i),
                                      &projected, &current))
            return 0U;
        if(i == 0U) first = current;
        if((i + 1U < segment_cells ||
            (uint16)p + i + 1U < action->push_meta.n_steps) &&
           (current.detonate || current.box_on_goal))
            return 0U;
    }

    mission_push_expected_map = projected;
    *from_x = first.car_from_x;
    *from_y = first.car_from_y;
    *to_x = current.car_to_x;
    *to_y = current.car_to_y;
    mission_status.push_step = 1U;
    mission_status.push_is_bomb = current.is_bomb;
    mission_status.box_from_x = first.object_from_x;
    mission_status.box_from_y = first.object_from_y;
    mission_status.box_to_x = current.object_to_x;
    mission_status.box_to_y = current.object_to_y;
    mission_status.push_box_on_goal = current.box_on_goal;
    mission_status.push_box_consumed = 0U;
    return 1U;
}

static uint8 mission_push_candidate_matches(const vision_link_map_t *latest,
                                            uint8 *consumed)
{
    vision_link_map_t without_completed_box;
    vision_link_map_t without_completed_goal;
    uint8 i;
    uint8 found = 0U;

    *consumed = 0U;
    if(mission_same_map(&mission_push_expected_map, latest)) return 1U;
    if(!mission_status.push_box_on_goal || latest->box_count + 1U !=
       mission_push_expected_map.box_count) return 0U;

    without_completed_box = mission_push_expected_map;
    for(i = 0U; i < without_completed_box.box_count; i++)
    {
        if(without_completed_box.boxes[i].gx ==
               (int8)mission_status.box_to_x &&
           without_completed_box.boxes[i].gy ==
               (int8)mission_status.box_to_y)
        {
            uint8 j;
            if(found) return 0U;
            found = 1U;
            for(j = i; j + 1U < without_completed_box.box_count; j++)
                without_completed_box.boxes[j] =
                    without_completed_box.boxes[j + 1U];
            without_completed_box.box_count--;
            break;
        }
    }
    if(!found) return 0U;
    if(mission_same_map(&without_completed_box, latest))
    {
        *consumed = 1U;
        return 1U;
    }

    /* The game may retire both a matched box and its goal. Keep this as a
       separate strict variant so unrelated missing goals are still rejected. */
    without_completed_goal = without_completed_box;
    if(!mission_remove_goal_at(&without_completed_goal,
                               mission_status.box_to_x,
                               mission_status.box_to_y) ||
       !mission_same_map(&without_completed_goal, latest))
    {
        return 0U;
    }
    *consumed = 1U;
    return 1U;
}

static uint8 mission_push_result_matches(const vision_link_map_t *latest,
                                         uint8 *consumed,
                                         vision_link_map_t *accepted,
                                         uint8 *ignored_car_box)
{
    uint8 normalized;

    *accepted = *latest;
    *ignored_car_box = 0U;
    if(mission_status.push_is_bomb)
    {
        *consumed = mission_cell_list_has(
            mission_push_expected_map.bombs,
            mission_push_expected_map.bomb_count,
            mission_status.box_to_x, mission_status.box_to_y) ? 0U : 1U;
        if(mission_same_map(&mission_push_expected_map, accepted)) return 1U;
        normalized = mission_remove_box_at(accepted, mission_status.to_x,
                                           mission_status.to_y);
        normalized |= mission_remove_bomb_at(accepted, mission_status.to_x,
                                             mission_status.to_y);
        normalized |= mission_restore_expected_goal_at(
            accepted, &mission_push_expected_map,
            mission_status.to_x, mission_status.to_y);
        if(normalized &&
           mission_same_map(&mission_push_expected_map, accepted))
        {
            *ignored_car_box = 1U;
            return 1U;
        }
        return 0U;
    }
    if(mission_push_candidate_matches(accepted, consumed)) return 1U;

    /* The car can hide the old box cell and its underlying goal. Normalize
       only that impossible overlap, then keep the normal strict comparison
       for every other wall/goal/bomb/box cell. */
    *accepted = *latest;
    normalized = mission_remove_box_at(accepted, mission_status.to_x,
                                       mission_status.to_y);
    normalized |= mission_restore_expected_goal_at(
        accepted, &mission_push_expected_map,
        mission_status.to_x, mission_status.to_y);
    if(!normalized) return 0U;
    if(!mission_push_candidate_matches(accepted, consumed)) return 0U;
    *ignored_car_box = 1U;
    return 1U;
}

static void mission_set_state(mission_state_t state, mission_result_t result)
{
    mission_status.state = state;
    mission_status.last_result = result;
    mission_status.event_counter++;
}

static void mission_hard_stop(mission_state_t state, mission_result_t result)
{
    action_follower_abort();
    motion_heading_lock_release();
    pentagram_enable = 0U;
    device_init_flag = 1;
    motion_fast_brake();
    motion_emergency_stop();
    mission_status.auto_run = 0U;
    mission_status.armed = 0U;
    mission_status.push_step = 0U;
    if(result != MISSION_RESULT_PUSH_MAP_TIMEOUT &&
       result != MISSION_RESULT_PUSH_MAP_MISMATCH)
        mission_status.push_is_bomb = 0U;
    mission_status.push_verify_pending = 0U;
    mission_status.post_push_ramp_pending = 0U;
    mission_status.logical_origin_override = 0U;
    mission_force_logical_origin_once = 0U;
    mission_clear_segment_runtime();
    if(state == MISSION_FAULT)
        status_buzzer_request(STATUS_BUZZER_EVENT_LOCKED);
    mission_set_state(state, result);
}

static void mission_latch_follower_fault(void)
{
    motion_heading_lock_release();
    pentagram_enable = 0U;
    device_init_flag = 1;
    motion_fast_brake();
    motion_emergency_stop();
    mission_status.auto_run = 0U;
    mission_status.armed = 0U;
    mission_status.push_step = 0U;
    mission_status.push_verify_pending = 0U;
    mission_status.post_push_ramp_pending = 0U;
    mission_status.logical_origin_override = 0U;
    mission_force_logical_origin_once = 0U;
    mission_clear_segment_runtime();
    status_buzzer_request(STATUS_BUZZER_EVENT_LOCKED);
    mission_set_state(MISSION_FAULT, MISSION_RESULT_FOLLOWER_FAULT);
}

static void mission_select_current_action(void)
{
    while(mission_plan != 0 &&
          mission_status.action_cursor < mission_status.action_count)
    {
        const action_t *action =
            &mission_plan->actions[mission_status.action_cursor];
        mission_status.current_action_type = action->type;
        mission_status.action_step_count = mission_action_step_count(action);
        if(mission_status.action_step_count > 0U ||
           action->type == ACTION_OBSERVE ||
           action->type == ACTION_PHASE_END ||
           action->type == ACTION_WAIT)
            return;
        mission_status.action_cursor++;
        mission_status.substep_cursor = 0U;
    }

    mission_status.current_action_type = 0xFFU;
    mission_status.action_step_count = 0U;
}

static void mission_finish_start_reanchor(uint8 result)
{
    mission_status.start_reanchor_result = result;
    motion_heading_lock_rebase_position();
    if(mission_status.plan_phase == SOLVER_PLAN_BOMB_P1 &&
       !roi_camera_link_session_ready())
    {
        mission_set_state(MISSION_FP_SESSION_WAIT, MISSION_RESULT_OK);
        return;
    }

    mission_select_current_action();
    mission_set_state(mission_status.action_cursor < mission_status.action_count ?
                          MISSION_STEP_WAIT : MISSION_COMPLETE,
                      MISSION_RESULT_OK);
}

static void mission_begin_start_reanchor(uint8 target_x, uint8 target_y,
                                         uint8 prior_result)
{
    vision_link_snapshot_t pose;

    motion_heading_lock_stop();
    vision_link_get_snapshot(&pose);
    mission_start_reanchor_start_tick = pit_count;
    mission_start_reanchor_cluster_tick = pit_count;
    mission_start_reanchor_last_packets = pose.pos_packets;
    mission_start_reanchor_last_frame = pose.frame_id;
    mission_start_reanchor_anchor_x10 = pose.car_x_mm;
    mission_start_reanchor_anchor_y10 = pose.car_y_mm;
    mission_status.start_target_x = target_x;
    mission_status.start_target_y = target_y;
    mission_status.start_reanchor_frames = 0U;
    mission_status.start_reanchor_bad_frames = 0U;
    mission_status.start_reanchor_result = prior_result;
    mission_status.start_reanchor_age_ms = 0U;
    mission_status.start_reanchor_x10 = pose.car_x_mm;
    mission_status.start_reanchor_y10 = pose.car_y_mm;
    mission_status.start_reanchor_error_x10 =
        (int16)((int)target_x * 10 - (int)pose.car_x_mm);
    mission_status.start_reanchor_error_y10 =
        (int16)((int)target_y * 10 - (int)pose.car_y_mm);
    mission_set_state(MISSION_START_POSE_WAIT, MISSION_RESULT_OK);
}

static void mission_poll_start_reanchor(void)
{
    vision_link_snapshot_t pose;
    point_test_snapshot_t point;
    uint32 elapsed =
        (uint32)(pit_count - mission_start_reanchor_start_tick);
    uint8 cluster_ready;
    uint8 target_cell;
    int dx_x10;
    int dy_x10;
    int dx_mm;
    int dy_mm;
    uint32 distance_sq;

    mission_status.start_reanchor_age_ms = elapsed * 5U;
    point_test_get_snapshot(&point);
    vision_link_get_snapshot(&pose);
    mission_status.start_reanchor_x10 = pose.car_x_mm;
    mission_status.start_reanchor_y10 = pose.car_y_mm;
    mission_status.start_reanchor_error_x10 = (int16)(
        (int)mission_status.start_target_x * 10 - (int)pose.car_x_mm);
    mission_status.start_reanchor_error_y10 = (int16)(
        (int)mission_status.start_target_y * 10 - (int)pose.car_y_mm);

    if(mission_observe_pose_fresh(&pose) && point.vision_position_stable &&
       pose.pos_packets != mission_start_reanchor_last_packets)
    {
        mission_start_reanchor_last_packets = pose.pos_packets;
        if(pose.frame_id != mission_start_reanchor_last_frame)
        {
            mission_start_reanchor_last_frame = pose.frame_id;
            if(mission_status.start_reanchor_frames == 0U ||
               (abs((int)pose.car_x_mm -
                    (int)mission_start_reanchor_anchor_x10) <=
                    MISSION_START_CLUSTER_X10 &&
                abs((int)pose.car_y_mm -
                    (int)mission_start_reanchor_anchor_y10) <=
                    MISSION_START_CLUSTER_X10))
            {
                if(mission_status.start_reanchor_frames == 0U)
                {
                    mission_start_reanchor_anchor_x10 = pose.car_x_mm;
                    mission_start_reanchor_anchor_y10 = pose.car_y_mm;
                    mission_start_reanchor_cluster_tick = pit_count;
                }
                if(mission_status.start_reanchor_frames < 255U)
                    mission_status.start_reanchor_frames++;
            }
            else
            {
                mission_start_reanchor_anchor_x10 = pose.car_x_mm;
                mission_start_reanchor_anchor_y10 = pose.car_y_mm;
                mission_start_reanchor_cluster_tick = pit_count;
                mission_status.start_reanchor_frames = 1U;
                if(mission_status.start_reanchor_bad_frames < 255U)
                    mission_status.start_reanchor_bad_frames++;
            }
        }
    }

    cluster_ready = (uint8)(
        mission_status.start_reanchor_frames >= MISSION_START_STABLE_FRAMES &&
        (uint32)(pit_count - mission_start_reanchor_cluster_tick) >=
            MISSION_START_STABLE_TICKS);
    if(cluster_ready)
    {
        dx_x10 = (int)mission_status.start_target_x * 10 -
                 (int)mission_start_reanchor_anchor_x10;
        dy_x10 = (int)mission_status.start_target_y * 10 -
                 (int)mission_start_reanchor_anchor_y10;
        target_cell = (uint8)(
            mission_round_grid_x10(mission_start_reanchor_anchor_x10) ==
                mission_status.start_target_x &&
            mission_round_grid_x10(mission_start_reanchor_anchor_y10) ==
                mission_status.start_target_y);
        if(!target_cell)
        {
            mission_hard_stop(MISSION_FAULT,
                              MISSION_RESULT_START_POSE_CHANGED);
            return;
        }
        if(abs(dx_x10) <= MISSION_START_TARGET_X10 &&
           abs(dy_x10) <= MISSION_START_TARGET_X10)
        {
            mission_finish_start_reanchor(
                mission_status.start_reanchor_result ==
                    MISSION_START_ALIGNED ?
                    MISSION_START_ALIGNED : MISSION_START_CENTERED);
            return;
        }

        dx_mm = dx_x10 * 20;
        dy_mm = dy_x10 * 20;
        distance_sq = (uint32)(dx_mm * dx_mm + dy_mm * dy_mm);
        if(distance_sq <= (uint32)MISSION_START_ALIGN_MAX_MM *
                              MISSION_START_ALIGN_MAX_MM &&
           action_follower_start_pose_reanchor(
               0U, 0U,
               mission_status.start_target_x,
               mission_status.start_target_y,
               mission_start_reanchor_anchor_x10,
               mission_start_reanchor_anchor_y10,
               MISSION_START_ALIGN_MAX_MM, 1U))
        {
            mission_set_state(MISSION_START_POSE_ALIGN, MISSION_RESULT_OK);
            return;
        }
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_START_POSE_CHANGED);
        return;
    }

    if(elapsed >= MISSION_START_WAIT_TICKS)
        mission_hard_stop(MISSION_FAULT, MISSION_RESULT_POSE_INVALID);
}

static uint8 mission_get_free_step_at(const action_t *action, uint8 substep,
                                      uint8 *from_x, uint8 *from_y,
                                      uint8 *to_x, uint8 *to_y)
{
    const waypoint_t *from;
    const waypoint_t *to;

    if(action == 0 || action->type != ACTION_FREE_MOVE ||
       substep >= mission_action_step_count(action))
        return 0U;

    from = &action->waypoints[substep];
    to = &action->waypoints[substep + 1U];
    if((from->x_mm % SOLVER_GRID_SIZE_MM) != 0 ||
       (from->y_mm % SOLVER_GRID_SIZE_MM) != 0 ||
       (to->x_mm % SOLVER_GRID_SIZE_MM) != 0 ||
       (to->y_mm % SOLVER_GRID_SIZE_MM) != 0)
        return 0U;

    *from_x = (uint8)(from->x_mm / SOLVER_GRID_SIZE_MM);
    *from_y = (uint8)(from->y_mm / SOLVER_GRID_SIZE_MM);
    *to_x = (uint8)(to->x_mm / SOLVER_GRID_SIZE_MM);
    *to_y = (uint8)(to->y_mm / SOLVER_GRID_SIZE_MM);
    return (uint8)(abs((int)*to_x - (int)*from_x) +
                   abs((int)*to_y - (int)*from_y) == 1);
}

static uint8 mission_get_current_free_step(uint8 *from_x, uint8 *from_y,
                                           uint8 *to_x, uint8 *to_y)
{
    if(mission_plan == 0 ||
       mission_status.action_cursor >= mission_status.action_count)
        return 0U;
    return mission_get_free_step_at(
        &mission_plan->actions[mission_status.action_cursor],
        mission_status.substep_cursor, from_x, from_y, to_x, to_y);
}

static uint8 mission_context_compatible(const mission_exec_context_t *left,
                                        const mission_exec_context_t *right)
{
    return (uint8)(left != 0 && right != 0 && left->merge_eligible &&
        right->merge_eligible &&
        left->follower.gear == right->follower.gear &&
        left->follower.wall_axis_mask == right->follower.wall_axis_mask &&
        left->follower.corridor_axis_mask ==
            right->follower.corridor_axis_mask &&
        left->follower.single_wall_correction_sign ==
            right->follower.single_wall_correction_sign &&
        left->wall_relation == right->wall_relation);
}

static uint8 mission_push_cruise_cells(const action_t *action)
{
    const push_meta_t *push;
    uint8 remaining;
    uint8 desired;
    uint8 accepted = 0U;
    uint8 index;
    int dx;
    int dy;

    if(!APP_MISSION_MULTI_PUSH_ENABLE || action == 0 ||
       (action->type != ACTION_PUSH_BOX &&
        action->type != ACTION_PUSH_BOMB))
        return 0U;
    push = &action->push_meta;
    if(push->push_dir > DIR_RIGHT ||
       mission_status.substep_cursor >= push->n_steps)
        return 0U;
    remaining = (uint8)(push->n_steps - mission_status.substep_cursor);
    if(remaining < MISSION_MULTI_PUSH_MIN_STEPS) return 0U;

    desired = (uint8)(remaining - 1U);
    if(desired > MISSION_MULTI_PUSH_MAX_CRUISE)
        desired = MISSION_MULTI_PUSH_MAX_CRUISE;
    if(desired > MISSION_FAST_MAX_SEGMENT_CELLS)
        desired = MISSION_FAST_MAX_SEGMENT_CELLS;
    dx = (int)DIR_DX[push->push_dir];
    dy = (int)DIR_DY[push->push_dir];

    for(index = 0U; index < desired; index++)
    {
        mission_exec_context_t edge_context;
        uint8 substep = (uint8)(mission_status.substep_cursor + index);
        uint8 from_x = (uint8)((int)push->car_target_x + dx * substep);
        uint8 from_y = (uint8)((int)push->car_target_y + dy * substep);
        uint8 to_x = (uint8)((int)from_x + dx);
        uint8 to_y = (uint8)((int)from_y + dy);

        if(!mission_build_exec_context(&mission_expected_map,
                                       from_x, from_y, to_x, to_y, 1U,
                                       &edge_context))
            break;
        /* 墙边和双边通道允许连续推；墙关系发生变化的出口格单独执行，
           由现有 transition 强矫正落在出口后的理想格心。 */
        if(edge_context.transition) break;
        accepted++;
    }
    return accepted >= 2U ? accepted : 0U;
}

static float mission_normalize_180(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static float mission_observe_map_heading(uint8 direction)
{
    if(direction == DIR_RIGHT) return 90.0f;
    if(direction == DIR_DOWN) return 180.0f;
    if(direction == DIR_LEFT) return 270.0f;
    return 0.0f;
}

static uint8 mission_start_rotation(uint16 angle_deg, uint8 clockwise)
{
    if(angle_deg == 0U) return 1U;
    if(!point_test_set_sensor_mode(POINT_SENSOR_ENCODER_IMU) ||
       !point_test_set_rotation_speed(MISSION_OBSERVE_ROTATE_SPEED) ||
       !point_test_set_rotation_angle(angle_deg) ||
       !point_test_set_rotation_clockwise(clockwise) ||
       !point_test_set_rotation_stop(POINT_ROTATE_STOP_IMU) ||
       !point_test_capture_origin() ||
       !point_test_start_rotation())
        return 0U;
    return 1U;
}

static void mission_advance_special_action(void)
{
    mission_status.action_cursor++;
    mission_status.substep_cursor = 0U;
    mission_select_current_action();
    if(mission_status.action_cursor >= mission_status.action_count)
    {
        mission_status.auto_run = 0U;
        mission_set_state(MISSION_COMPLETE, MISSION_RESULT_OK);
    }
    else
    {
        mission_set_state(MISSION_STEP_WAIT, MISSION_RESULT_OK);
        if(mission_status.auto_run) mission_start_current_step(1U);
    }
}

static uint8 mission_observation_id_unique(const int8 ids[MAX_BOXES],
                                           uint8 mask, uint8 slot,
                                           int8 value)
{
    uint8 index;
    for(index = 0U; index < MAX_BOXES; index++)
    {
        if(index != slot && (mask & (uint8)(1U << index)) &&
           ids[index] == value)
            return 0U;
    }
    return 1U;
}

static uint8 mission_store_observation(
    const roi_camera_link_result_t *result)
{
    uint8 count = mission_expected_map.box_count;

    if(result == 0 || result->status != ROI_CAMERA_LINK_STATUS_OK ||
       result->result_id >= count || result->object_slot >= count)
        return 0U;
    if(result->object_type == ROI_CAMERA_LINK_OBJECT_BOX)
    {
        if(!mission_observation_id_unique(mission_box_ids,
                                         mission_status.observed_box_mask,
                                         result->object_slot,
                                         (int8)result->result_id))
            return 0U;
        mission_box_ids[result->object_slot] = (int8)result->result_id;
        mission_status.observed_box_mask |=
            (uint8)(1U << result->object_slot);
    }
    else if(result->object_type == ROI_CAMERA_LINK_OBJECT_GOAL)
    {
        if(!mission_observation_id_unique(mission_goal_ids,
                                         mission_status.observed_goal_mask,
                                         result->object_slot,
                                         (int8)result->result_id))
            return 0U;
        mission_goal_ids[result->object_slot] = (int8)result->result_id;
        mission_status.observed_goal_mask |=
            (uint8)(1U << result->object_slot);
    }
    else
    {
        return 0U;
    }
    memcpy(mission_status.observed_box_ids, mission_box_ids,
           sizeof(mission_box_ids));
    memcpy(mission_status.observed_goal_ids, mission_goal_ids,
           sizeof(mission_goal_ids));
    return 1U;
}

static uint8 mission_send_observe_request(uint8 object_type,
                                          uint8 object_slot)
{
    mission_observe_pending_type = object_type;
    mission_observe_pending_slot = object_slot;
    mission_status.observe_object_type = object_type;
    mission_status.observe_object_slot = object_slot;
    if(!roi_camera_link_request(object_type, object_slot)) return 0U;
    mission_set_state(MISSION_OBSERVE_WAIT_RESULT, MISSION_RESULT_OK);
    return 1U;
}

static void mission_begin_observe_return(void)
{
    if(mission_observe_rotation_deg == 0U)
    {
        mission_advance_special_action();
        return;
    }
    if(!mission_start_rotation(mission_observe_rotation_deg,
                               (uint8)!mission_observe_rotation_clockwise))
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_OBSERVE_ROTATION_FAILED);
        return;
    }
    mission_set_state(MISSION_OBSERVE_ROTATE_BACK, MISSION_RESULT_OK);
}

static mission_result_t mission_begin_observe(const action_t *action)
{
    vision_link_snapshot_t pose;
    float imu_heading;
    float map_heading_ref;
    float delta;
    float abs_delta;
    uint8 logical_pose_ok;

    if(action == 0 || action->type != ACTION_OBSERVE ||
       !roi_camera_link_session_ready())
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_FP_SESSION_FAILED);
        return mission_status.last_result;
    }
    if(!action_follower_get_heading_frame(&imu_heading, &map_heading_ref))
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_POSE_INVALID);
        return mission_status.last_result;
    }
    vision_link_get_snapshot(&pose);
    logical_pose_ok = (uint8)(mission_status.logical_origin_override &&
        mission_status.logical_origin_x == action->target_x &&
        mission_status.logical_origin_y == action->target_y);
    if(!logical_pose_ok &&
       (!mission_observe_pose_fresh(&pose) ||
        abs((int)pose.car_x_mm - (int)action->target_x * 10) >
            MISSION_ARM_POSE_TOLERANCE_X10 ||
        abs((int)pose.car_y_mm - (int)action->target_y * 10) >
            MISSION_ARM_POSE_TOLERANCE_X10))
    {
        mission_wait_for_observe_pose();
        return mission_status.last_result;
    }
    (void)imu_heading;

    delta = mission_normalize_180(
        mission_observe_map_heading(action->observe_meta.direction) -
        map_heading_ref);
    abs_delta = delta < 0.0f ? -delta : delta;
    if(abs_delta < 45.0f) mission_observe_rotation_deg = 0U;
    else if(abs_delta < 135.0f) mission_observe_rotation_deg = 90U;
    else mission_observe_rotation_deg = 180U;
    mission_observe_rotation_clockwise = delta > 0.0f ? 1U : 0U;
    mission_observe_second_type = ROI_CAMERA_LINK_OBJECT_NONE;
    mission_observe_second_slot = 0xFFU;

    if(action->observe_meta.object_type == OBSERVE_OBJECT_BOX)
    {
        mission_observe_pending_type = ROI_CAMERA_LINK_OBJECT_BOX;
        mission_observe_pending_slot = action->observe_meta.box_index;
    }
    else if(action->observe_meta.object_type == OBSERVE_OBJECT_GOAL)
    {
        mission_observe_pending_type = ROI_CAMERA_LINK_OBJECT_GOAL;
        mission_observe_pending_slot = action->observe_meta.goal_index;
    }
    else if(action->observe_meta.object_type == OBSERVE_OBJECT_BOTH)
    {
        mission_observe_pending_type = ROI_CAMERA_LINK_OBJECT_BOX;
        mission_observe_pending_slot = action->observe_meta.box_index;
        mission_observe_second_type = ROI_CAMERA_LINK_OBJECT_GOAL;
        mission_observe_second_slot = action->observe_meta.goal_index;
    }
    else
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_RECOGNITION_REJECTED);
        return mission_status.last_result;
    }
    if(mission_observe_pending_slot >= mission_expected_map.box_count ||
       (mission_observe_second_type != ROI_CAMERA_LINK_OBJECT_NONE &&
        mission_observe_second_slot >= mission_expected_map.goal_count))
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_RECOGNITION_REJECTED);
        return mission_status.last_result;
    }

    mission_status.observe_rotation_deg = mission_observe_rotation_deg;
    mission_status.observe_rotation_clockwise =
        mission_observe_rotation_clockwise;
    mission_observe_state_tick = pit_count;
    if(mission_observe_rotation_deg == 0U)
    {
        motion_heading_lock_stop();
        mission_set_state(MISSION_OBSERVE_SETTLE, MISSION_RESULT_OK);
    }
    else if(mission_start_rotation(mission_observe_rotation_deg,
                                   mission_observe_rotation_clockwise))
    {
        mission_set_state(MISSION_OBSERVE_ROTATE_OUT, MISSION_RESULT_OK);
    }
    else
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_OBSERVE_ROTATION_FAILED);
    }
    return mission_status.last_result;
}

static mission_result_t mission_run_phase2_solver(void)
{
    uint8 count = mission_expected_map.box_count;
    planner_status_t planner_status;

    mission_set_state(MISSION_PHASE2_SOLVING, MISSION_RESULT_OK);
    if(!bomb_solver_resolve_n2(count, mission_box_ids,
                               mission_status.observed_box_mask,
                               mission_goal_ids,
                               mission_status.observed_goal_mask))
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_N2_RESOLVE_FAILED);
        return mission_status.last_result;
    }
    memcpy(mission_status.observed_box_ids, mission_box_ids,
           sizeof(mission_box_ids));
    memcpy(mission_status.observed_goal_ids, mission_goal_ids,
           sizeof(mission_goal_ids));
    planner_status = planner_service_solve_phase2(mission_box_ids,
                                                  mission_goal_ids);
    if(planner_status != PLANNER_STATUS_OK)
    {
        mission_hard_stop(MISSION_FAULT, MISSION_RESULT_PHASE2_FAILED);
        return mission_status.last_result;
    }
    status_buzzer_request(STATUS_BUZZER_EVENT_SOLVE_DONE);

    mission_plan = planner_service_get_plan();
    mission_status.plan_generation =
        planner_service_get_info()->plan_generation;
    mission_status.plan_phase = mission_plan->plan_phase;
    mission_status.action_cursor = 0U;
    mission_status.substep_cursor = 0U;
    mission_status.step_cursor = 0U;
    mission_status.action_count = mission_plan->action_count;
    mission_status.step_count = mission_manager_count_plan_steps(mission_plan);
    mission_select_current_action();
    mission_set_state(mission_status.action_cursor < mission_status.action_count ?
                      MISSION_STEP_WAIT : MISSION_COMPLETE,
                      MISSION_RESULT_OK);
    if(mission_status.auto_run &&
       mission_status.state == MISSION_STEP_WAIT)
        mission_start_current_step(1U);
    return mission_status.last_result;
}

static mission_result_t mission_start_current_step(uint8 allow_push)
{
    const action_t *action;
    mission_exec_context_t context;
    mission_exec_context_t terminal_context;
    uint8 from_x;
    uint8 from_y;
    uint8 to_x;
    uint8 to_y;
    uint8 fast_finish = 0U;
    uint8 segment_cells = 1U;
    uint8 use_logical_origin;
    uint8 use_post_push_ramp;
    uint8 push_cruise_cells = 0U;

    mission_select_current_action();
    if(mission_status.action_cursor >= mission_status.action_count)
    {
        mission_set_state(MISSION_COMPLETE, MISSION_RESULT_OK);
        return MISSION_RESULT_OK;
    }

    action = &mission_plan->actions[mission_status.action_cursor];
    if(action->type == ACTION_OBSERVE)
        return mission_begin_observe(action);
    if(action->type == ACTION_PHASE_END)
        return mission_run_phase2_solver();
    if(action->type == ACTION_WAIT)
    {
        motion_heading_lock_stop();
        mission_observe_state_tick = pit_count;
        mission_status.observe_wait_ms =
            (uint32)(action->wait_duration * 1000.0f + 0.5f);
        mission_set_state(MISSION_WAIT_ACTION, MISSION_RESULT_OK);
        return mission_status.last_result;
    }
    mission_clear_segment_runtime();
    mission_status.push_step = 0U;
    mission_status.push_is_bomb = 0U;
    mission_status.push_verify_pending = 0U;
    mission_status.push_verify_requests = 0U;
    mission_status.push_verify_bad_maps = 0U;
    mission_status.push_verify_mismatch_mask = 0U;
    mission_status.push_box_on_goal = 0U;
    mission_status.push_box_consumed = 0U;
    mission_status.push_car_box_filtered = 0U;
    mission_status.push_verify_age_ms = 0U;
    if(action->type == ACTION_FREE_MOVE)
    {
        if(!mission_get_current_free_step(&from_x, &from_y, &to_x, &to_y))
        {
            mission_hard_stop(MISSION_FAULT, MISSION_RESULT_BAD_STATE);
            return mission_status.last_result;
        }
        if(!mission_build_exec_context(&mission_expected_map,
                                       from_x, from_y, to_x, to_y, 0U,
                                       &context))
        {
            mission_hard_stop(MISSION_FAULT, MISSION_RESULT_BAD_STATE);
            return mission_status.last_result;
        }
        context.follower.terminal_node = 1U;
        terminal_context = context;

        if(mission_status.auto_run &&
           mission_status.run_profile == MISSION_PROFILE_FAST_SAFE &&
           context.merge_eligible)
        {
            uint8 candidate_substep = mission_status.substep_cursor;
            uint8 segment_to_x = to_x;
            uint8 segment_to_y = to_y;
            while(segment_cells < MISSION_FAST_MAX_SEGMENT_CELLS &&
                  candidate_substep + 1U <
                      mission_status.action_step_count)
            {
                mission_exec_context_t next_context;
                uint8 next_from_x;
                uint8 next_from_y;
                uint8 next_to_x;
                uint8 next_to_y;

                candidate_substep++;
                if(!mission_get_free_step_at(action, candidate_substep,
                                             &next_from_x, &next_from_y,
                                             &next_to_x, &next_to_y) ||
                   next_from_x != segment_to_x ||
                   next_from_y != segment_to_y ||
                   !mission_build_exec_context(&mission_expected_map,
                                               next_from_x, next_from_y,
                                               next_to_x, next_to_y, 0U,
                                               &next_context) ||
                   !mission_context_compatible(&context, &next_context))
                {
                    break;
                }
                if((int)next_to_x - (int)next_from_x !=
                       (int)to_x - (int)from_x ||
                   (int)next_to_y - (int)next_from_y !=
                       (int)to_y - (int)from_y)
                {
                    break;
                }
                segment_to_x = next_to_x;
                segment_to_y = next_to_y;
                terminal_context = next_context;
                segment_cells++;
            }
            mission_status.segment_total_cells = segment_cells;
            mission_status.segment_node_x = segment_to_x;
            mission_status.segment_node_y = segment_to_y;
            if(segment_cells >= 3U)
            {
                int step_x = (int)to_x - (int)from_x;
                int step_y = (int)to_y - (int)from_y;
                mission_terminal_from_x = (uint8)(
                    (int)segment_to_x - step_x);
                mission_terminal_from_y = (uint8)(
                    (int)segment_to_y - step_y);
                mission_terminal_to_x = segment_to_x;
                mission_terminal_to_y = segment_to_y;
                mission_terminal_context = terminal_context;
                mission_terminal_context.follower.segment_cells = 1U;
                mission_terminal_context.follower.cruise_only = 0U;
                mission_terminal_context.follower.terminal_node = 1U;
                mission_terminal_context.follower.strong_reanchor =
                    (uint8)(mission_terminal_context.transition ||
                            mission_terminal_context.wall_relation == 0U);
                mission_terminal_context.follower.start_tolerance_mm =
                    MISSION_TERMINAL_START_TOLERANCE_MM;
                mission_terminal_context.follower.align_max_distance_mm =
                    MISSION_TERMINAL_ALIGN_MAX_MM;
                mission_terminal_pending = 1U;

                to_x = mission_terminal_from_x;
                to_y = mission_terminal_from_y;
                context.follower.segment_cells =
                    (uint8)(segment_cells - 1U);
                context.follower.cruise_only = 1U;
                context.follower.terminal_node = 0U;
                context.follower.strong_reanchor = 0U;
                mission_status.segment_phase = MISSION_SEGMENT_CRUISE;
                mission_status.segment_cruise_cells =
                    (uint8)(segment_cells - 1U);
            }
            else
            {
                to_x = segment_to_x;
                to_y = segment_to_y;
                context.follower.segment_cells = segment_cells;
                context.follower.terminal_node = 1U;
                context.follower.strong_reanchor =
                    (uint8)(context.transition ||
                            context.wall_relation == 0U);
                mission_status.segment_phase = segment_cells > 1U ?
                    MISSION_SEGMENT_TERMINAL : MISSION_SEGMENT_SINGLE;
            }
        }
    }
    else if((action->type == ACTION_PUSH_BOX ||
             action->type == ACTION_PUSH_BOMB) && allow_push)
    {
        if(mission_status.auto_run &&
           mission_status.run_profile == MISSION_PROFILE_FAST_SAFE)
            push_cruise_cells = mission_push_cruise_cells(action);
        if(push_cruise_cells > 0U)
            segment_cells = push_cruise_cells;
        if(!mission_prepare_push_segment(action, segment_cells,
                                         &from_x, &from_y, &to_x, &to_y))
        {
            mission_pause_for_pose_replan();
            return mission_status.last_result;
        }
        if(!mission_build_exec_context(&mission_expected_map,
                                       from_x, from_y, to_x, to_y, 1U,
                                       &context))
        {
            mission_hard_stop(MISSION_FAULT, MISSION_RESULT_BAD_STATE);
            return mission_status.last_result;
        }
        if(push_cruise_cells > 0U)
        {
            context.follower.segment_cells = push_cruise_cells;
            context.follower.cruise_only = 1U;
            context.follower.terminal_node = 0U;
            context.follower.vision_drain_ms = MISSION_PUSH_DRAIN_MS;
            context.follower.vision_stable_frames =
                MISSION_PUSH_STABLE_FRAMES;
            context.follower.vision_stable_ms = MISSION_PUSH_STABLE_MS;
            mission_status.segment_phase = MISSION_SEGMENT_PUSH_CRUISE;
            mission_status.segment_total_cells = (uint8)(
                action->push_meta.n_steps - mission_status.substep_cursor);
            mission_status.segment_cruise_cells = push_cruise_cells;
            mission_status.segment_node_x = to_x;
            mission_status.segment_node_y = to_y;
        }
        else
        {
            context.follower.terminal_node = 1U;
        }
    }
    else
    {
        mission_status.auto_run = 0U;
        mission_set_state(MISSION_STEP_WAIT,
                          MISSION_RESULT_UNSUPPORTED_ACTION);
        return mission_status.last_result;
    }

    use_logical_origin = (uint8)(mission_force_logical_origin_once &&
        from_x == mission_status.logical_origin_x &&
        from_y == mission_status.logical_origin_y);
    use_post_push_ramp = mission_status.post_push_ramp_pending;
    context.follower.logical_origin_override = use_logical_origin;
    context.follower.pwm_ramp_ms = use_post_push_ramp ?
        MISSION_POST_PUSH_RAMP_MS : 0U;
    context.follower.pwm_ramp_max_delta = use_post_push_ramp ?
        MISSION_POST_PUSH_RAMP_DELTA : 0U;

    mission_status.from_x = from_x;
    mission_status.from_y = from_y;
    mission_status.to_x = to_x;
    mission_status.to_y = to_y;
    mission_status.segment_cells = segment_cells;
    if(mission_status.segment_total_cells == 1U)
    {
        mission_status.segment_node_x = to_x;
        mission_status.segment_node_y = to_y;
    }
    mission_status.exec_gear = context.follower.gear;
    mission_status.context_box_near = context.box_near;
    mission_status.context_bomb_near = context.bomb_near;
    mission_status.context_object_near = context.object_near;
    mission_status.context_goal_near = context.goal_near;
    mission_status.context_strict_position =
        context.follower.strict_position;
    mission_status.context_transition = context.transition;
    mission_status.context_wall_sign =
        context.follower.single_wall_correction_sign;
    fast_finish = (uint8)(
        mission_status.segment_phase == MISSION_SEGMENT_CRUISE ||
        mission_status.segment_phase == MISSION_SEGMENT_PUSH_CRUISE ||
        (mission_status.push_step && mission_status.push_box_on_goal));
    if(!action_follower_start_grid_step(
           mission_status.action_cursor,
           mission_status.substep_cursor,
           mission_status.action_step_count,
           mission_status.step_cursor,
           from_x, from_y, to_x, to_y, fast_finish,
           &context.follower))
    {
        mission_latch_follower_fault();
        return mission_status.last_result;
    }

    if(use_post_push_ramp) mission_status.post_push_ramp_pending = 0U;
    if(use_logical_origin)
    {
        mission_force_logical_origin_once = 0U;
        mission_status.logical_origin_override = 0U;
    }

    mission_set_state(MISSION_ACTION_RUNNING, MISSION_RESULT_OK);
    return mission_status.last_result;
}

static uint8 mission_start_terminal_node(void)
{
    if(!mission_terminal_pending) return 0U;

    mission_status.from_x = mission_terminal_from_x;
    mission_status.from_y = mission_terminal_from_y;
    mission_status.to_x = mission_terminal_to_x;
    mission_status.to_y = mission_terminal_to_y;
    mission_status.segment_phase = MISSION_SEGMENT_TERMINAL;
    mission_status.exec_gear = mission_terminal_context.follower.gear;
    mission_status.context_box_near = mission_terminal_context.box_near;
    mission_status.context_bomb_near = mission_terminal_context.bomb_near;
    mission_status.context_object_near =
        mission_terminal_context.object_near;
    mission_status.context_goal_near = mission_terminal_context.goal_near;
    mission_status.context_strict_position =
        mission_terminal_context.follower.strict_position;
    mission_status.context_transition = mission_terminal_context.transition;
    mission_status.context_wall_sign =
        mission_terminal_context.follower.single_wall_correction_sign;

    mission_terminal_pending = 0U;
    if(!action_follower_start_grid_step(
           mission_status.action_cursor,
           (uint8)(mission_status.substep_cursor +
                   mission_status.segment_cruise_cells),
           mission_status.action_step_count,
           (uint16)(mission_status.step_cursor +
                    mission_status.segment_cruise_cells),
           mission_terminal_from_x, mission_terminal_from_y,
           mission_terminal_to_x, mission_terminal_to_y, 0U,
           &mission_terminal_context.follower))
    {
        mission_latch_follower_fault();
        return 0U;
    }
    return 1U;
}

static void mission_commit_current_step(void)
{
    uint8 committed = mission_status.segment_cells > 0U ?
        mission_status.segment_cells : 1U;
    mission_status.step_cursor += committed;
    mission_status.substep_cursor += committed;
    mission_status.push_verify_pending = 0U;
    status_buzzer_request(STATUS_BUZZER_EVENT_NODE_REACHED);
    mission_clear_segment_runtime();
    if(mission_status.substep_cursor >= mission_status.action_step_count)
    {
        mission_status.action_cursor++;
        mission_status.substep_cursor = 0U;
    }
    mission_select_current_action();
    if(mission_status.action_cursor >= mission_status.action_count)
    {
        mission_status.auto_run = 0U;
        mission_set_state(MISSION_COMPLETE, MISSION_RESULT_OK);
    }
    else
    {
        mission_set_state(MISSION_STEP_WAIT, MISSION_RESULT_OK);
        if(mission_status.auto_run) mission_start_current_step(1U);
    }
}

static void mission_finish_post_push(uint8 result,
                                     uint8 logical_origin_override)
{
    mission_status.post_push_result = result;
    mission_status.post_push_ramp_pending = 1U;
    motion_heading_lock_rebase_position();
    if(logical_origin_override)
    {
        mission_force_logical_origin_once = 1U;
        mission_status.logical_origin_override = 1U;
        mission_status.logical_origin_x = mission_status.to_x;
        mission_status.logical_origin_y = mission_status.to_y;
    }
    mission_commit_current_step();
}

static void mission_begin_post_push_reanchor(void)
{
    vision_link_snapshot_t pose;

    motion_heading_lock_stop();
    motion_heading_lock_rebase_position();
    vision_link_get_snapshot(&pose);
    mission_post_push_start_tick = pit_count;
    mission_post_push_cluster_tick = pit_count;
    mission_post_push_last_packets = pose.pos_packets;
    mission_post_push_last_frame = pose.frame_id;
    mission_post_push_anchor_x10 = pose.car_x_mm;
    mission_post_push_anchor_y10 = pose.car_y_mm;
    mission_status.post_push_frames = 0U;
    mission_status.post_push_bad_frames = 0U;
    mission_status.post_push_result = MISSION_POST_PUSH_NONE;
    mission_status.post_push_ramp_pending = 0U;
    mission_status.post_push_age_ms = 0U;
    mission_status.post_push_x10 = pose.car_x_mm;
    mission_status.post_push_y10 = pose.car_y_mm;
    mission_status.post_push_error_x10 =
        (int16)((int)mission_status.to_x * 10 - (int)pose.car_x_mm);
    mission_status.post_push_error_y10 =
        (int16)((int)mission_status.to_y * 10 - (int)pose.car_y_mm);
    mission_set_state(MISSION_POST_PUSH_REANCHOR, MISSION_RESULT_OK);
}

static void mission_poll_post_push_reanchor(void)
{
    vision_link_snapshot_t pose;
    uint32 elapsed = (uint32)(pit_count - mission_post_push_start_tick);
    uint8 cluster_ready;
    uint8 target_cell;

    mission_status.post_push_age_ms = elapsed * 5U;
    vision_link_get_snapshot(&pose);
    mission_status.post_push_x10 = pose.car_x_mm;
    mission_status.post_push_y10 = pose.car_y_mm;
    mission_status.post_push_error_x10 =
        (int16)((int)mission_status.to_x * 10 - (int)pose.car_x_mm);
    mission_status.post_push_error_y10 =
        (int16)((int)mission_status.to_y * 10 - (int)pose.car_y_mm);

    if(mission_observe_pose_fresh(&pose) &&
       pose.pos_packets != mission_post_push_last_packets)
    {
        mission_post_push_last_packets = pose.pos_packets;
        if(pose.frame_id != mission_post_push_last_frame)
        {
            mission_post_push_last_frame = pose.frame_id;
            if(mission_status.post_push_frames == 0U ||
               (abs((int)pose.car_x_mm -
                    (int)mission_post_push_anchor_x10) <=
                    MISSION_POST_PUSH_CLUSTER_X10 &&
                abs((int)pose.car_y_mm -
                    (int)mission_post_push_anchor_y10) <=
                    MISSION_POST_PUSH_CLUSTER_X10))
            {
                if(mission_status.post_push_frames == 0U)
                {
                    mission_post_push_anchor_x10 = pose.car_x_mm;
                    mission_post_push_anchor_y10 = pose.car_y_mm;
                    mission_post_push_cluster_tick = pit_count;
                }
                if(mission_status.post_push_frames < 255U)
                    mission_status.post_push_frames++;
                /* Track the newest member of a stable cluster.  Keeping the
                   first sample forever can leave the anchor at 8.8 cells
                   while later frames have settled at the 9.0-cell target. */
                mission_post_push_anchor_x10 = pose.car_x_mm;
                mission_post_push_anchor_y10 = pose.car_y_mm;
            }
            else
            {
                mission_post_push_anchor_x10 = pose.car_x_mm;
                mission_post_push_anchor_y10 = pose.car_y_mm;
                mission_post_push_cluster_tick = pit_count;
                mission_status.post_push_frames = 1U;
                mission_status.post_push_bad_frames = 0U;
            }
        }
    }

    cluster_ready = (uint8)(
        mission_status.post_push_frames >= MISSION_POST_PUSH_STABLE_FRAMES &&
        (uint32)(pit_count - mission_post_push_cluster_tick) >=
            MISSION_POST_PUSH_STABLE_TICKS);
    target_cell = (uint8)(
        mission_round_grid_x10(mission_post_push_anchor_x10) ==
            mission_status.to_x &&
        mission_round_grid_x10(mission_post_push_anchor_y10) ==
            mission_status.to_y &&
        abs((int)mission_status.to_x * 10 -
            (int)mission_post_push_anchor_x10) <=
            MISSION_POST_PUSH_TARGET_X10 &&
        abs((int)mission_status.to_y * 10 -
            (int)mission_post_push_anchor_y10) <=
            MISSION_POST_PUSH_TARGET_X10);

    if(cluster_ready && target_cell &&
       elapsed >= MISSION_POST_PUSH_MIN_TICKS)
    {
        mission_status.post_push_bad_frames = 0U;
        mission_finish_post_push(MISSION_POST_PUSH_STABLE, 0U);
        return;
    }
    if(cluster_ready && !target_cell)
    {
        mission_status.post_push_bad_frames =
            mission_status.post_push_frames;
        if((mission_status.follower.wall_axis_mask != 0U ||
            mission_status.follower.corridor_axis_mask != 0U) &&
           elapsed >= MISSION_POST_PUSH_MIN_TICKS)
        {
            /* 上位机在贴墙和双边通道内会把坐标钳在墙线上，继续等待不会
               得到可校准位置。接受编码器+IMU逻辑原点，离墙出口再强矫正。 */
            mission_finish_post_push(MISSION_POST_PUSH_FALLBACK, 1U);
            return;
        }
    }

    if(elapsed < MISSION_POST_PUSH_TIMEOUT_TICKS) return;
    if(cluster_ready && !target_cell)
    {
        mission_status.post_push_result = MISSION_POST_PUSH_MISMATCH;
        mission_pause_for_pose_replan();
        return;
    }
    mission_finish_post_push(MISSION_POST_PUSH_FALLBACK, 1U);
}

static void mission_commit_auto_push_cruise(void)
{
    uint8 committed = mission_status.segment_cruise_cells;

    if(committed == 0U || mission_status.push_box_on_goal ||
       mission_status.substep_cursor + committed >=
           mission_status.action_step_count)
    {
        mission_hard_stop(MISSION_FAULT, MISSION_RESULT_BAD_STATE);
        return;
    }

    mission_expected_map = mission_push_expected_map;
    mission_expected_map_valid = 1U;
    mission_ignored_car_box_valid = 0U;
    mission_status.step_cursor += committed;
    mission_status.substep_cursor += committed;
    mission_status.push_step = 0U;
    mission_status.push_verify_pending = 0U;
    mission_status.push_verify_requests = 0U;
    mission_status.push_verify_bad_maps = 0U;
    mission_status.push_verify_mismatch_mask = 0U;
    mission_status.push_box_consumed = 0U;
    mission_status.post_push_ramp_pending = 1U;
    status_buzzer_request(STATUS_BUZZER_EVENT_NODE_REACHED);

    mission_clear_segment_runtime();
    mission_select_current_action();
    mission_set_state(MISSION_STEP_WAIT, MISSION_RESULT_OK);
    if(mission_status.auto_run) mission_start_current_step(1U);
}

static void mission_commit_auto_push_step(void)
{
    mission_expected_map = mission_push_expected_map;
    mission_status.push_box_consumed = 0U;
    if(!mission_status.push_is_bomb && mission_status.push_box_on_goal)
    {
        if(mission_remove_box_at(&mission_expected_map,
                                 mission_status.box_to_x,
                                 mission_status.box_to_y))
        {
            mission_status.push_box_consumed = 1U;
            mission_remove_goal_at(&mission_expected_map,
                                   mission_status.box_to_x,
                                   mission_status.box_to_y);
        }
    }
    mission_expected_map_valid = 1U;
    mission_ignored_car_box_valid = 0U;
    mission_status.push_verify_pending = 0U;
    mission_status.push_verify_requests = 0U;
    mission_status.push_verify_bad_maps = 0U;
    mission_status.push_verify_mismatch_mask = 0U;
    mission_begin_post_push_reanchor();
}

static void mission_begin_push_verify(void)
{
    vision_link_map_t latest;

    motion_fast_brake();
    motion_emergency_stop();
    mission_status.push_verify_pending = 1U;
    mission_status.push_verify_requests = 1U;
    mission_status.push_verify_bad_maps = 0U;
    mission_status.push_verify_mismatch_mask = 0U;
    mission_status.push_verify_age_ms = 0U;
    mission_status.push_map_version_after = 0U;
    if(vision_link_get_map(&latest))
        mission_status.push_map_version_before = latest.map_version;
    else
        mission_status.push_map_version_before = 0U;
    mission_push_verify_start_tick = pit_count;
    mission_push_verify_request_tick = pit_count;
    mission_post_push_start_tick = pit_count;
    mission_post_push_cluster_tick = pit_count;
    mission_post_push_last_packets = 0U;
    mission_post_push_last_frame = 0U;
    mission_post_push_anchor_x10 = 0;
    mission_post_push_anchor_y10 = 0;
    mission_force_logical_origin_once = 0U;
    vision_link_request_full_map();
    status_buzzer_request(STATUS_BUZZER_EVENT_MAP_REQUEST);
    mission_set_state(MISSION_PUSH_VERIFY, MISSION_RESULT_OK);
}

static void mission_poll_push_verify(void)
{
    vision_link_map_t latest;
    vision_link_map_t accepted;
    uint8 consumed;
    uint8 ignored_car_box;
    uint32 elapsed = (uint32)(pit_count - mission_push_verify_start_tick);

    mission_status.push_verify_age_ms = elapsed * 5U;
    if(elapsed >= MISSION_PUSH_VERIFY_TIMEOUT_TICKS)
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_PUSH_MAP_TIMEOUT);
        return;
    }
    if(mission_status.push_verify_bad_maps >= MISSION_PUSH_VERIFY_MAX_MAPS &&
       elapsed >= MISSION_PUSH_VERIFY_MIN_MISMATCH_TICKS)
    {
        mission_hard_stop(MISSION_FAULT,
                          MISSION_RESULT_PUSH_MAP_MISMATCH);
        return;
    }
    if(!vision_link_get_map(&latest) ||
       latest.map_version <= mission_status.push_map_version_before)
    {
        if(mission_status.push_verify_bad_maps > 0U &&
           mission_status.push_verify_requests <
               MISSION_PUSH_VERIFY_MAX_MAPS &&
           (uint32)(pit_count - mission_push_verify_request_tick) >=
               MISSION_PUSH_VERIFY_RETRY_TICKS)
        {
            mission_status.push_verify_requests++;
            mission_push_verify_request_tick = pit_count;
            vision_link_request_full_map();
        }
        return;
    }

    mission_status.push_map_version_after = latest.map_version;
    if(mission_push_result_matches(&latest, &consumed, &accepted,
                                   &ignored_car_box))
    {
        uint8 i;
        mission_status.push_box_consumed = consumed;
        mission_status.push_car_box_filtered = ignored_car_box;
        if(mission_status.push_is_bomb)
        {
            mission_expected_map = accepted;
            mission_expected_map.map_version = latest.map_version;
            mission_expected_map_valid = 1U;
            mission_ignored_car_box_valid = ignored_car_box;
            mission_ignored_car_box_x = mission_status.to_x;
            mission_ignored_car_box_y = mission_status.to_y;
            mission_ignored_car_box_map_version = latest.map_version;
            mission_commit_current_step();
            return;
        }
        mission_expected_map.map_version = latest.map_version;
        mission_expected_map.box_count = accepted.box_count;
        for(i = 0U; i < VISION_LINK_MAX_BOXES; i++)
            mission_expected_map.boxes[i] = accepted.boxes[i];
        if(consumed &&
           !mission_cell_list_has(accepted.boxes, accepted.box_count,
                                  mission_status.box_to_x,
                                  mission_status.box_to_y) &&
           !mission_cell_list_has(accepted.goals, accepted.goal_count,
                                  mission_status.box_to_x,
                                  mission_status.box_to_y))
        {
            mission_remove_goal_at(&mission_expected_map,
                                   mission_status.box_to_x,
                                   mission_status.box_to_y);
        }
        mission_ignored_car_box_valid = ignored_car_box;
        mission_ignored_car_box_x = mission_status.to_x;
        mission_ignored_car_box_y = mission_status.to_y;
        mission_ignored_car_box_map_version = latest.map_version;
        mission_expected_map_valid = 1U;
        mission_commit_current_step();
        return;
    }

    mission_status.push_verify_bad_maps++;
    mission_status.push_verify_mismatch_mask =
        mission_map_diff_mask(&mission_push_expected_map, &accepted);
    mission_status.push_map_version_before = latest.map_version;
}

void mission_manager_init(void)
{
    memset(&mission_status, 0, sizeof(mission_status));
    memset(&mission_expected_map, 0, sizeof(mission_expected_map));
    memset(&mission_push_expected_map, 0,
           sizeof(mission_push_expected_map));
    mission_plan = 0;
    mission_expected_map_valid = 0U;
    mission_ignored_car_box_valid = 0U;
    mission_ignored_car_box_x = 0U;
    mission_ignored_car_box_y = 0U;
    mission_ignored_car_box_map_version = 0U;
    memset(mission_box_ids, -1, sizeof(mission_box_ids));
    memset(mission_goal_ids, -1, sizeof(mission_goal_ids));
    mission_observe_pending_type = ROI_CAMERA_LINK_OBJECT_NONE;
    mission_observe_pending_slot = 0xFFU;
    mission_observe_second_type = ROI_CAMERA_LINK_OBJECT_NONE;
    mission_observe_second_slot = 0xFFU;
    mission_observe_rotation_deg = 0U;
    mission_observe_rotation_clockwise = 0U;
    mission_observe_state_tick = pit_count;
    mission_start_reanchor_start_tick = pit_count;
    mission_start_reanchor_cluster_tick = pit_count;
    mission_start_reanchor_last_packets = 0U;
    mission_start_reanchor_last_frame = 0U;
    mission_start_reanchor_anchor_x10 = 0;
    mission_start_reanchor_anchor_y10 = 0;
    mission_push_verify_start_tick = pit_count;
    mission_push_verify_request_tick = pit_count;
    mission_status.run_profile = MISSION_PROFILE_STANDARD;
    mission_status.exec_gear = ACTION_GEAR_STANDARD;
    mission_status.segment_cells = 1U;
    mission_clear_segment_runtime();
    action_follower_init();
    mission_hard_stop(MISSION_SAFE_IDLE, MISSION_RESULT_OK);
}

static mission_result_t mission_manager_arm_plan_internal(
    uint8 prevalidated_pose)
{
    const planner_info_t *info = planner_service_get_info();
    const solver_output_t *plan = planner_service_get_plan();
    const vision_world_snapshot_t *world = planner_service_get_world();
    vision_link_snapshot_t pose;

    if(mission_status.armed ||
       mission_status.state == MISSION_START_POSE_WAIT ||
       mission_status.state == MISSION_START_POSE_ALIGN ||
       mission_status.state == MISSION_ACTION_RUNNING ||
       mission_status.state == MISSION_POST_PUSH_REANCHOR ||
       mission_status.state == MISSION_PUSH_VERIFY ||
       mission_status.state == MISSION_PAUSED)
    {
        mission_status.last_result = MISSION_RESULT_BUSY;
        return mission_status.last_result;
    }
    if(plan == 0 || !info->plan_valid)
    {
        mission_status.last_result = MISSION_RESULT_NO_PLAN;
        return mission_status.last_result;
    }
    if(plan->plan_phase != SOLVER_PLAN_STANDARD &&
       plan->plan_phase != SOLVER_PLAN_BOMB_P1)
    {
        mission_status.last_result = MISSION_RESULT_UNSUPPORTED_ACTION;
        return mission_status.last_result;
    }
    if(!planner_service_plan_is_current())
    {
        mission_status.last_result = MISSION_RESULT_PLAN_STALE;
        return mission_status.last_result;
    }
    vision_link_get_snapshot(&pose);
    if(!pose.pose_valid)
    {
        mission_status.last_result = MISSION_RESULT_POSE_INVALID;
        return mission_status.last_result;
    }
    if(abs((int)pose.car_x_mm - (int)world->car_x10) >
           MISSION_ARM_POSE_TOLERANCE_X10 ||
       abs((int)pose.car_y_mm - (int)world->car_y10) >
           MISSION_ARM_POSE_TOLERANCE_X10)
    {
        mission_status.last_result = MISSION_RESULT_START_POSE_CHANGED;
        return mission_status.last_result;
    }
    if(!vision_link_get_map(&mission_expected_map) ||
       !mission_expected_map.valid)
    {
        mission_expected_map_valid = 0U;
        mission_status.last_result = MISSION_RESULT_PLAN_STALE;
        return mission_status.last_result;
    }
    mission_expected_map_valid = 1U;
    mission_ignored_car_box_valid = 0U;

    if(!action_follower_heading_frame_valid() &&
       !action_follower_begin_mission_heading())
    {
        mission_status.last_result = MISSION_RESULT_BAD_STATE;
        return mission_status.last_result;
    }

    mission_plan = plan;
    mission_status.plan_generation = info->plan_generation;
    mission_status.action_cursor = 0U;
    mission_status.substep_cursor = 0U;
    mission_status.step_cursor = 0U;
    mission_status.action_count = plan->action_count;
    mission_status.step_count = mission_manager_count_plan_steps(plan);
    mission_status.plan_phase = plan->plan_phase;
    mission_status.auto_run = 0U;
    mission_status.armed = 1U;
    mission_status.push_step = 0U;
    mission_status.push_is_bomb = 0U;
    mission_status.push_verify_pending = 0U;
    mission_status.push_verify_requests = 0U;
    mission_status.push_verify_bad_maps = 0U;
    mission_status.push_verify_mismatch_mask = 0U;
    mission_status.push_box_on_goal = 0U;
    mission_status.push_box_consumed = 0U;
    mission_status.push_car_box_filtered = 0U;
    mission_status.push_map_version_before = mission_expected_map.map_version;
    mission_status.push_map_version_after = 0U;
    mission_status.push_verify_age_ms = 0U;
    mission_status.post_push_frames = 0U;
    mission_status.post_push_bad_frames = 0U;
    mission_status.post_push_result = MISSION_POST_PUSH_NONE;
    mission_status.post_push_ramp_pending = 0U;
    mission_status.post_push_age_ms = 0U;
    mission_status.post_push_x10 = 0;
    mission_status.post_push_y10 = 0;
    mission_status.post_push_error_x10 = 0;
    mission_status.post_push_error_y10 = 0;
    mission_status.logical_origin_override = 0U;
    mission_status.logical_origin_x = 0U;
    mission_status.logical_origin_y = 0U;
    mission_status.wall_snap_accept_count = 0U;
    mission_force_logical_origin_once = 0U;
    mission_status.exec_gear = ACTION_GEAR_STANDARD;
    mission_status.segment_cells = 1U;
    mission_clear_segment_runtime();
    mission_status.context_box_near = 0U;
    mission_status.context_bomb_near = 0U;
    mission_status.context_object_near = 0U;
    mission_status.context_goal_near = 0U;
    mission_status.context_strict_position = 0U;
    mission_status.context_transition = 0U;
    mission_status.context_wall_sign = 0;
    mission_status.fp_state = ROI_CAMERA_LINK_NO_SESSION;
    mission_status.fp_status = ROI_CAMERA_LINK_STATUS_OK;
    mission_status.observe_object_type = ROI_CAMERA_LINK_OBJECT_NONE;
    mission_status.observe_object_slot = 0xFFU;
    mission_status.observed_box_mask = 0U;
    mission_status.observed_goal_mask = 0U;
    memset(mission_box_ids, -1, sizeof(mission_box_ids));
    memset(mission_goal_ids, -1, sizeof(mission_goal_ids));
    memcpy(mission_status.observed_box_ids, mission_box_ids,
           sizeof(mission_box_ids));
    memcpy(mission_status.observed_goal_ids, mission_goal_ids,
           sizeof(mission_goal_ids));
    mission_status.observe_rotation_deg = 0U;
    mission_status.observe_rotation_clockwise = 0U;
    mission_status.observe_wait_ms = 0U;
    mission_status.start_target_x = world->solver_map.car_x;
    mission_status.start_target_y = world->solver_map.car_y;
    mission_status.start_reanchor_frames = 0U;
    mission_status.start_reanchor_bad_frames = 0U;
    mission_status.start_reanchor_result = MISSION_START_NONE;
    mission_status.start_reanchor_age_ms = 0U;
    mission_status.start_reanchor_x10 = pose.car_x_mm;
    mission_status.start_reanchor_y10 = pose.car_y_mm;
    mission_status.start_reanchor_error_x10 = (int16)(
        (int)world->solver_map.car_x * 10 - (int)pose.car_x_mm);
    mission_status.start_reanchor_error_y10 = (int16)(
        (int)world->solver_map.car_y * 10 - (int)pose.car_y_mm);
    device_init_flag = 1;
    if(plan->plan_phase == SOLVER_PLAN_BOMB_P1)
    {
        roi_camera_link_reset_session();
        if(!roi_camera_link_begin_session(mission_expected_map.box_count,
                                          mission_expected_map.goal_count))
        {
            mission_hard_stop(MISSION_FAULT,
                              MISSION_RESULT_FP_SESSION_FAILED);
            return mission_status.last_result;
        }
    }
    if(prevalidated_pose)
    {
        /* The complete-match layer already accepted a new non-empty map and
           six post-map pose frames in the planner start cell.  Rebase the
           logical/encoder origin without commanding a second physical move. */
        mission_status.start_reanchor_frames = 6U;
        mission_status.start_reanchor_result = MISSION_START_CENTERED;
        mission_finish_start_reanchor(MISSION_START_CENTERED);
    }
    else
    {
        mission_begin_start_reanchor(world->solver_map.car_x,
                                     world->solver_map.car_y,
                                     MISSION_START_NONE);
    }
    return mission_status.last_result;
}

mission_result_t mission_manager_arm_plan(void)
{
    return mission_manager_arm_plan_internal(0U);
}

mission_result_t mission_manager_arm_plan_prevalidated_pose(void)
{
    return mission_manager_arm_plan_internal(1U);
}

mission_result_t mission_manager_run_next_step(void)
{
    if(!mission_status.armed || mission_plan == 0)
    {
        mission_status.last_result = MISSION_RESULT_NOT_ARMED;
        return mission_status.last_result;
    }
    if(mission_status.state != MISSION_STEP_WAIT)
    {
        mission_status.last_result =
            (mission_status.state == MISSION_START_POSE_WAIT ||
             mission_status.state == MISSION_START_POSE_ALIGN ||
             mission_status.state == MISSION_ACTION_RUNNING ||
             mission_status.state == MISSION_POST_PUSH_REANCHOR ||
             mission_status.state == MISSION_PUSH_VERIFY) ?
                MISSION_RESULT_BUSY : MISSION_RESULT_BAD_STATE;
        return mission_status.last_result;
    }
    if(!mission_plan_environment_current())
    {
        mission_hard_stop(MISSION_FAULT, MISSION_RESULT_PLAN_STALE);
        return mission_status.last_result;
    }
    mission_status.auto_run = 0U;
    return mission_start_current_step(1U);
}

mission_result_t mission_manager_run_one_grid(void)
{
    mission_run_profile_t saved_profile;
    mission_result_t result;

    if(!mission_status.armed || mission_plan == 0)
    {
        mission_status.last_result = MISSION_RESULT_NOT_ARMED;
        return mission_status.last_result;
    }
    if(mission_status.state != MISSION_STEP_WAIT)
    {
        mission_status.last_result =
            (mission_status.state == MISSION_START_POSE_WAIT ||
             mission_status.state == MISSION_START_POSE_ALIGN ||
             mission_status.state == MISSION_ACTION_RUNNING ||
             mission_status.state == MISSION_POST_PUSH_REANCHOR ||
             mission_status.state == MISSION_PUSH_VERIFY) ?
                MISSION_RESULT_BUSY : MISSION_RESULT_BAD_STATE;
        return mission_status.last_result;
    }
    if(!mission_plan_environment_current())
    {
        mission_hard_stop(MISSION_FAULT, MISSION_RESULT_PLAN_STALE);
        return mission_status.last_result;
    }

    saved_profile = mission_status.run_profile;
    mission_status.run_profile = MISSION_PROFILE_STANDARD;
    mission_status.auto_run = 0U;
    result = mission_start_current_step(1U);
    mission_status.run_profile = saved_profile;
    return result;
}

mission_result_t mission_manager_run_all_steps(void)
{
    if(!mission_status.armed || mission_plan == 0)
    {
        mission_status.last_result = MISSION_RESULT_NOT_ARMED;
        return mission_status.last_result;
    }
    if(mission_status.state != MISSION_STEP_WAIT)
    {
        mission_status.last_result = MISSION_RESULT_BUSY;
        return mission_status.last_result;
    }
    if(!mission_plan_environment_current())
    {
        mission_hard_stop(MISSION_FAULT, MISSION_RESULT_PLAN_STALE);
        return mission_status.last_result;
    }
    mission_status.auto_run = 1U;
    return mission_start_current_step(1U);
}

mission_result_t mission_manager_toggle_run_profile(void)
{
    if(mission_status.state == MISSION_START_POSE_WAIT ||
       mission_status.state == MISSION_START_POSE_ALIGN ||
       mission_status.state == MISSION_ACTION_RUNNING ||
       mission_status.state == MISSION_POST_PUSH_REANCHOR ||
       mission_status.state == MISSION_PUSH_VERIFY)
    {
        mission_status.last_result = MISSION_RESULT_BUSY;
        return mission_status.last_result;
    }
    if(mission_status.run_profile == MISSION_PROFILE_STANDARD)
        mission_status.run_profile = MISSION_PROFILE_NORMAL;
    else if(mission_status.run_profile == MISSION_PROFILE_NORMAL)
        mission_status.run_profile = MISSION_PROFILE_FAST_SAFE;
    else
        mission_status.run_profile = MISSION_PROFILE_STANDARD;
    mission_status.last_result = MISSION_RESULT_OK;
    mission_status.event_counter++;
    return mission_status.last_result;
}

mission_result_t mission_manager_set_run_profile(mission_run_profile_t profile)
{
    if(profile > MISSION_PROFILE_FAST_SAFE)
    {
        mission_status.last_result = MISSION_RESULT_BAD_STATE;
        return mission_status.last_result;
    }
    if(mission_status.state == MISSION_START_POSE_WAIT ||
       mission_status.state == MISSION_START_POSE_ALIGN ||
       mission_status.state == MISSION_ACTION_RUNNING ||
       mission_status.state == MISSION_POST_PUSH_REANCHOR ||
       mission_status.state == MISSION_PUSH_VERIFY)
    {
        mission_status.last_result = MISSION_RESULT_BUSY;
        return mission_status.last_result;
    }
    mission_status.run_profile = profile;
    mission_status.last_result = MISSION_RESULT_OK;
    mission_status.event_counter++;
    return mission_status.last_result;
}

void mission_manager_poll(void)
{
    action_follower_state_t follower_state;
    roi_camera_link_snapshot_t fp;
    point_test_snapshot_t point;
    const action_t *action;
    uint32 elapsed_ticks;

    action_follower_poll();
    roi_camera_link_get_snapshot(&fp);
    mission_status.fp_state = fp.state;
    mission_status.fp_status = fp.last_status;

    if(mission_status.state == MISSION_START_POSE_WAIT)
    {
        mission_poll_start_reanchor();
        return;
    }
    if(mission_status.state == MISSION_START_POSE_ALIGN)
    {
        follower_state = action_follower_state();
        if(follower_state == ACTION_FOLLOWER_DONE)
        {
            motion_heading_lock_stop();
            mission_begin_start_reanchor(mission_status.start_target_x,
                                         mission_status.start_target_y,
                                         MISSION_START_ALIGNED);
        }
        else if(follower_state == ACTION_FOLLOWER_REPLAN ||
                follower_state == ACTION_FOLLOWER_FAULT)
        {
            mission_pause_for_pose_replan();
        }
        return;
    }
    if(mission_status.state == MISSION_FP_SESSION_WAIT)
    {
        if(roi_camera_link_session_ready())
        {
            mission_select_current_action();
            mission_set_state(mission_status.action_cursor <
                                  mission_status.action_count ?
                              MISSION_STEP_WAIT : MISSION_COMPLETE,
                              MISSION_RESULT_OK);
        }
        else if(fp.state == ROI_CAMERA_LINK_FAULT)
        {
            mission_hard_stop(MISSION_FAULT,
                              MISSION_RESULT_FP_SESSION_FAILED);
        }
        return;
    }
    if(mission_status.state == MISSION_POST_PUSH_REANCHOR)
    {
        mission_poll_post_push_reanchor();
        return;
    }
    if(mission_status.state == MISSION_OBSERVE_POSE_WAIT)
    {
        vision_link_snapshot_t pose;
        int dx_x10;
        int dy_x10;
        int dx_mm;
        int dy_mm;
        uint32 distance_sq;

        if(mission_plan == 0 ||
           mission_status.action_cursor >= mission_status.action_count)
        {
            mission_hard_stop(MISSION_FAULT, MISSION_RESULT_BAD_STATE);
            return;
        }
        action = &mission_plan->actions[mission_status.action_cursor];
        if(action->type != ACTION_OBSERVE)
        {
            mission_hard_stop(MISSION_FAULT, MISSION_RESULT_BAD_STATE);
            return;
        }

        elapsed_ticks = (uint32)(pit_count - mission_observe_state_tick);
        mission_status.observe_wait_ms =
            elapsed_ticks >= MISSION_OBSERVE_POSE_WAIT_TICKS ? 0U :
            (MISSION_OBSERVE_POSE_WAIT_TICKS - elapsed_ticks) * 5U;
        point_test_get_snapshot(&point);
        vision_link_get_snapshot(&pose);
        if(mission_observe_pose_fresh(&pose) &&
           point.vision_position_stable)
        {
            dx_x10 = (int)action->target_x * 10 - (int)pose.car_x_mm;
            dy_x10 = (int)action->target_y * 10 - (int)pose.car_y_mm;
            if(abs(dx_x10) <= MISSION_ARM_POSE_TOLERANCE_X10 &&
               abs(dy_x10) <= MISSION_ARM_POSE_TOLERANCE_X10)
            {
                mission_status.observe_wait_ms = 0U;
                mission_begin_observe(action);
                return;
            }

            dx_mm = dx_x10 * 20;
            dy_mm = dy_x10 * 20;
            distance_sq = (uint32)(dx_mm * dx_mm + dy_mm * dy_mm);
            if(distance_sq <=
                   (uint32)MISSION_OBSERVE_ALIGN_MAX_MM *
                   MISSION_OBSERVE_ALIGN_MAX_MM &&
               action_follower_start_pose_reanchor(
                   mission_status.action_cursor,
                   mission_status.step_cursor,
                   action->target_x, action->target_y,
                   pose.car_x_mm, pose.car_y_mm,
                   MISSION_OBSERVE_ALIGN_MAX_MM, 0U))
            {
                mission_status.observe_wait_ms = 0U;
                mission_set_state(MISSION_OBSERVE_POSE_ALIGN,
                                  MISSION_RESULT_OK);
                return;
            }
        }

        if(elapsed_ticks >= MISSION_OBSERVE_POSE_WAIT_TICKS)
            mission_pause_for_pose_replan();
        return;
    }
    if(mission_status.state == MISSION_OBSERVE_POSE_ALIGN)
    {
        follower_state = action_follower_state();
        if(follower_state == ACTION_FOLLOWER_DONE)
        {
            motion_heading_lock_stop();
            mission_wait_for_observe_pose();
        }
        else if(follower_state == ACTION_FOLLOWER_REPLAN ||
                follower_state == ACTION_FOLLOWER_FAULT)
        {
            mission_pause_for_pose_replan();
        }
        return;
    }
    if(mission_status.state == MISSION_WAIT_ACTION)
    {
        elapsed_ticks = (uint32)(pit_count - mission_observe_state_tick);
        if(elapsed_ticks * 5U >= mission_status.observe_wait_ms)
        {
            mission_status.observe_wait_ms = 0U;
            mission_advance_special_action();
        }
        else
        {
            mission_status.observe_wait_ms -= elapsed_ticks * 5U;
            mission_observe_state_tick = pit_count;
        }
        return;
    }
    if(mission_status.state == MISSION_OBSERVE_ROTATE_OUT ||
       mission_status.state == MISSION_OBSERVE_ROTATE_BACK)
    {
        point_test_get_snapshot(&point);
        if(point.state == POINT_TEST_FAULT || point.state == POINT_TEST_LOCKED)
        {
            mission_hard_stop(MISSION_FAULT,
                              MISSION_RESULT_OBSERVE_ROTATION_FAILED);
            return;
        }
        if(point.state == POINT_TEST_DONE)
        {
            motion_heading_lock_stop();
            mission_observe_state_tick = pit_count;
            if(mission_status.state == MISSION_OBSERVE_ROTATE_OUT)
                mission_set_state(MISSION_OBSERVE_SETTLE,
                                  MISSION_RESULT_OK);
            else
                mission_advance_special_action();
        }
        return;
    }
    if(mission_status.state == MISSION_OBSERVE_SETTLE)
    {
        uint32 required_ticks = MISSION_OBSERVE_SETTLE_MIN_TICKS;
        if(mission_plan == 0 ||
           mission_status.action_cursor >= mission_status.action_count)
        {
            mission_hard_stop(MISSION_FAULT, MISSION_RESULT_BAD_STATE);
            return;
        }
        action = &mission_plan->actions[mission_status.action_cursor];
        if((uint32)action->observe_meta.dwell_ms / 5U > required_ticks)
            required_ticks = (uint32)action->observe_meta.dwell_ms / 5U;
        elapsed_ticks = (uint32)(pit_count - mission_observe_state_tick);
        mission_status.observe_wait_ms = elapsed_ticks >= required_ticks ? 0U :
            (required_ticks - elapsed_ticks) * 5U;
        if(elapsed_ticks >= required_ticks)
        {
            mission_status.observe_wait_ms = 0U;
            if(!mission_send_observe_request(
                   mission_observe_pending_type,
                   mission_observe_pending_slot))
                mission_hard_stop(MISSION_FAULT,
                                  MISSION_RESULT_FP_SESSION_FAILED);
        }
        return;
    }
    if(mission_status.state == MISSION_OBSERVE_WAIT_RESULT)
    {
        roi_camera_link_result_t result;
        if(!roi_camera_link_take_result(&result)) return;
        mission_status.fp_status = result.status;
        if(result.status != ROI_CAMERA_LINK_STATUS_OK)
        {
            mission_hard_stop(
                MISSION_FAULT,
                result.status == ROI_CAMERA_LINK_STATUS_TIMEOUT ?
                    MISSION_RESULT_RECOGNITION_TIMEOUT :
                    MISSION_RESULT_RECOGNITION_REJECTED);
            return;
        }
        if(!mission_store_observation(&result))
        {
            mission_hard_stop(MISSION_FAULT,
                              MISSION_RESULT_RECOGNITION_REJECTED);
            return;
        }
        if(mission_observe_second_type != ROI_CAMERA_LINK_OBJECT_NONE)
        {
            uint8 second_type = mission_observe_second_type;
            uint8 second_slot = mission_observe_second_slot;
            mission_observe_second_type = ROI_CAMERA_LINK_OBJECT_NONE;
            mission_observe_second_slot = 0xFFU;
            if(!mission_send_observe_request(second_type, second_slot))
                mission_hard_stop(MISSION_FAULT,
                                  MISSION_RESULT_FP_SESSION_FAILED);
        }
        else
        {
            mission_begin_observe_return();
        }
        return;
    }
    if(mission_status.state == MISSION_PUSH_VERIFY ||
       mission_status.state == MISSION_POST_PUSH_REANCHOR)
    {
        mission_poll_push_verify();
        return;
    }
    if(mission_status.state != MISSION_ACTION_RUNNING) return;
    if(!mission_plan_environment_current())
    {
        mission_hard_stop(MISSION_FAULT, MISSION_RESULT_PLAN_STALE);
        return;
    }

    follower_state = action_follower_state();
    if(follower_state == ACTION_FOLLOWER_DONE)
    {
        action_follower_get_debug(&mission_status.follower);
        if(mission_status.segment_phase == MISSION_SEGMENT_PUSH_CRUISE)
        {
            mission_commit_auto_push_cruise();
            return;
        }
        if(mission_status.segment_phase == MISSION_SEGMENT_CRUISE &&
           mission_terminal_pending)
        {
            mission_start_terminal_node();
            return;
        }
        if(mission_status.follower.wall_snap_accepted)
        {
            mission_force_logical_origin_once = 1U;
            mission_status.logical_origin_override = 1U;
            mission_status.logical_origin_x = mission_status.to_x;
            mission_status.logical_origin_y = mission_status.to_y;
            mission_status.wall_snap_accept_count =
                mission_status.follower.wall_snap_accept_count;
        }
        if(mission_status.push_step && mission_status.auto_run)
            mission_commit_auto_push_step();
        else if(mission_status.push_step)
            mission_begin_push_verify();
        else mission_commit_current_step();
    }
    else if(follower_state == ACTION_FOLLOWER_FAULT)
    {
        mission_latch_follower_fault();
    }
    else if(follower_state == ACTION_FOLLOWER_REPLAN)
    {
        motion_heading_lock_release();
        pentagram_enable = 0U;
        device_init_flag = 1;
        motion_fast_brake();
        motion_emergency_stop();
        mission_status.auto_run = 0U;
        mission_status.armed = 0U;
        mission_clear_segment_runtime();
        mission_set_state(MISSION_PAUSED, MISSION_RESULT_REPLAN_REQUIRED);
        status_buzzer_request(STATUS_BUZZER_EVENT_LOCKED);
    }
}

mission_result_t mission_manager_pause(void)
{
    if(mission_status.state == MISSION_START_POSE_WAIT ||
       mission_status.state == MISSION_START_POSE_ALIGN)
    {
        mission_hard_stop(MISSION_PAUSED,
                          MISSION_RESULT_REPLAN_REQUIRED);
        return mission_status.last_result;
    }
    if(mission_status.state == MISSION_PUSH_VERIFY)
    {
        mission_status.auto_run = 0U;
        mission_status.push_verify_pending = 0U;
        mission_status.post_push_ramp_pending = 0U;
        mission_set_state(MISSION_PAUSED, MISSION_RESULT_REPLAN_REQUIRED);
        return mission_status.last_result;
    }
    if(mission_status.state != MISSION_ACTION_RUNNING)
    {
        mission_status.last_result = MISSION_RESULT_BAD_STATE;
        return mission_status.last_result;
    }
    mission_status.auto_run = 0U;
    action_follower_pause();
    mission_set_state(MISSION_PAUSED, MISSION_RESULT_REPLAN_REQUIRED);
    return mission_status.last_result;
}

mission_result_t mission_manager_continue(void)
{
    mission_status.last_result = MISSION_RESULT_REPLAN_REQUIRED;
    return mission_status.last_result;
}

mission_result_t mission_manager_reset_cursor(void)
{
    if(mission_status.state == MISSION_START_POSE_WAIT ||
       mission_status.state == MISSION_START_POSE_ALIGN ||
       mission_status.state == MISSION_ACTION_RUNNING ||
       mission_status.state == MISSION_POST_PUSH_REANCHOR ||
       mission_status.state == MISSION_PUSH_VERIFY ||
       mission_status.state == MISSION_PAUSED)
    {
        mission_status.last_result = MISSION_RESULT_BUSY;
        return mission_status.last_result;
    }
    if(!mission_status.armed || mission_plan == 0)
    {
        mission_status.last_result = MISSION_RESULT_NOT_ARMED;
        return mission_status.last_result;
    }
    if(mission_status.step_cursor != 0U)
    {
        mission_status.last_result = MISSION_RESULT_REPLAN_REQUIRED;
        return mission_status.last_result;
    }
    mission_status.action_cursor = 0U;
    mission_status.substep_cursor = 0U;
    mission_status.step_cursor = 0U;
    mission_status.auto_run = 0U;
    mission_status.push_verify_pending = 0U;
    mission_status.segment_cells = 1U;
    mission_clear_segment_runtime();
    mission_status.exec_gear = ACTION_GEAR_STANDARD;
    mission_select_current_action();
    mission_set_state(mission_status.action_cursor < mission_status.action_count ?
                      MISSION_STEP_WAIT : MISSION_COMPLETE,
                      MISSION_RESULT_OK);
    return mission_status.last_result;
}

void mission_manager_disarm(void)
{
    mission_hard_stop(MISSION_SAFE_IDLE, MISSION_RESULT_OK);
    roi_camera_link_reset_session();
    mission_plan = 0;
    mission_status.action_cursor = 0U;
    mission_status.action_count = 0U;
    mission_status.substep_cursor = 0U;
    mission_status.action_step_count = 0U;
    mission_status.step_cursor = 0U;
    mission_status.step_count = 0U;
    mission_status.current_action_type = 0xFFU;
    mission_status.push_step = 0U;
    mission_status.push_verify_pending = 0U;
    mission_expected_map_valid = 0U;
}

void mission_manager_emergency_stop(void)
{
    mission_hard_stop(MISSION_SAFE_IDLE, MISSION_RESULT_OK);
}

uint8 mission_manager_planner_locked(void)
{
    return mission_status.armed;
}

void mission_manager_get_status(mission_status_t *out)
{
    if(out == 0) return;
    *out = mission_status;
    out->map_current = mission_plan_environment_current();
    action_follower_get_debug(&out->follower);
}

const char *mission_state_name(mission_state_t state)
{
    switch(state)
    {
        case MISSION_SAFE_IDLE: return "SAFE_IDLE";
        case MISSION_PLAN_READY: return "PLAN_READY";
        case MISSION_START_POSE_WAIT: return "START_POSE_WAIT";
        case MISSION_START_POSE_ALIGN: return "START_POSE_ALIGN";
        case MISSION_STEP_WAIT: return "STEP_WAIT";
        case MISSION_ACTION_RUNNING: return "GRID_STEP_RUNNING";
        case MISSION_POST_PUSH_REANCHOR: return "POST_PUSH_REANCHOR";
        case MISSION_PUSH_VERIFY: return "PUSH_MAP_VERIFY";
        case MISSION_FP_SESSION_WAIT: return "FP_SESSION_WAIT";
        case MISSION_OBSERVE_POSE_WAIT: return "OBS_POSE_WAIT";
        case MISSION_OBSERVE_POSE_ALIGN: return "OBS_POSE_ALIGN";
        case MISSION_OBSERVE_ROTATE_OUT: return "OBS_ROTATE_OUT";
        case MISSION_OBSERVE_SETTLE: return "OBS_SETTLE";
        case MISSION_OBSERVE_WAIT_RESULT: return "OBS_WAIT_RESULT";
        case MISSION_OBSERVE_ROTATE_BACK: return "OBS_ROTATE_BACK";
        case MISSION_WAIT_ACTION: return "WAIT_ACTION";
        case MISSION_PHASE2_SOLVING: return "PHASE2_SOLVING";
        case MISSION_PAUSED: return "PAUSED_REPLAN";
        case MISSION_COMPLETE: return "COMPLETE";
        case MISSION_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char *mission_result_name(mission_result_t result)
{
    switch(result)
    {
        case MISSION_RESULT_OK: return "OK";
        case MISSION_RESULT_NO_PLAN: return "NO_PLAN";
        case MISSION_RESULT_PLAN_STALE: return "PLAN_STALE";
        case MISSION_RESULT_NOT_ARMED: return "NOT_ARMED";
        case MISSION_RESULT_BUSY: return "BUSY";
        case MISSION_RESULT_UNSUPPORTED_ACTION: return "UNSUPPORTED_ACTION";
        case MISSION_RESULT_FOLLOWER_FAULT: return "FOLLOWER_FAULT";
        case MISSION_RESULT_START_POSE_CHANGED: return "START_POSE_CHANGED";
        case MISSION_RESULT_POSE_INVALID: return "POSE_INVALID";
        case MISSION_RESULT_REPLAN_REQUIRED: return "REPLAN_REQUIRED";
        case MISSION_RESULT_PUSH_MAP_TIMEOUT: return "PUSH_MAP_TIMEOUT";
        case MISSION_RESULT_PUSH_MAP_MISMATCH: return "PUSH_MAP_MISMATCH";
        case MISSION_RESULT_FP_SESSION_FAILED: return "FP_SESSION_FAILED";
        case MISSION_RESULT_RECOGNITION_TIMEOUT: return "RECOGNITION_TIMEOUT";
        case MISSION_RESULT_RECOGNITION_REJECTED: return "RECOGNITION_REJECTED";
        case MISSION_RESULT_OBSERVE_ROTATION_FAILED: return "OBS_ROTATION_FAILED";
        case MISSION_RESULT_N2_RESOLVE_FAILED: return "N2_RESOLVE_FAILED";
        case MISSION_RESULT_PHASE2_FAILED: return "PHASE2_FAILED";
        case MISSION_RESULT_BAD_STATE: return "BAD_STATE";
        default: return "UNKNOWN";
    }
}

const char *mission_run_profile_name(mission_run_profile_t profile)
{
    if(profile == MISSION_PROFILE_NORMAL) return "NORMAL_STABLE";
    return profile == MISSION_PROFILE_FAST_SAFE ? "FAST_SAFE" : "STANDARD";
}
