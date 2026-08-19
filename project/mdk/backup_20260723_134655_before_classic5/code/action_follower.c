#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "blue.h"
#include "motion_control.h"
#include "point_test.h"
#include "vision_link.h"
#include "status_buzzer.h"
#include "action_follower.h"
#include "app_config.h"

#define FOLLOWER_POSE_STALE_TICKS       (60U)      /* 300 ms */
#define FOLLOWER_START_TOLERANCE_MM     (120.0f)
#define FOLLOWER_ALREADY_THERE_MM       (60.0f)
#define FOLLOWER_SPEED_DEFAULT          ((float)APP_MATCH_EXIT_SPEED)
#define FOLLOWER_SPEED_MAX              (150.0f)
#define FOLLOWER_MAP_HEADING_MAX_ERR_DEG (30.0f)
#define FOLLOWER_CROSS_STALE_TICKS       (60U)      /* 300 ms; position frames arrive in bursts */
#define FOLLOWER_CROSS_START_TICKS       (60U)      /* 300 ms */
#define FOLLOWER_CROSS_EARLY_START_MM    (40.0f)
#define FOLLOWER_CROSS_DISABLE_MM        (40.0f)
#define FOLLOWER_CROSS_DEADBAND_MM       (20.0f)
#define FOLLOWER_CROSS_JUMP_MM           (80.0f)
#define FOLLOWER_CROSS_FILTER_ALPHA      (0.5f)
#define FOLLOWER_CROSS_KP                (0.4f)
#define FOLLOWER_CROSS_MAX_SPEED         (24.0f)
#define FOLLOWER_CORRIDOR_CROSS_KP       (0.8f)
#define FOLLOWER_CORRIDOR_DEADBAND_MM     (5.0f)
#define FOLLOWER_CORRIDOR_MAX_SPEED       (24.0f)
#define FOLLOWER_SINGLE_WALL_CROSS_KP     (0.30f)
#define FOLLOWER_SINGLE_WALL_MAX_SPEED    (18.0f)
#define FOLLOWER_PUSH_WALL_CROSS_KP       (0.20f)
#define FOLLOWER_PUSH_WALL_MAX_SPEED      (12.0f)
#define FOLLOWER_DEAD_RECKON_MAX_STEPS   (4U)
#define FOLLOWER_ALIGN_PASS_AXIS_MM       (40.0f)
#define FOLLOWER_ALIGN_PASS_DIST_MM       (55.0f)
#define FOLLOWER_ALIGN_MAX_DIST_MM        (120.0f)
#define FOLLOWER_ALIGN_MAX_ATTEMPTS       (2U)
#define FOLLOWER_STRICT_PASS_AXIS_MM       (APP_FOLLOWER_STRICT_AXIS_MM)
#define FOLLOWER_STRICT_PASS_DIST_MM       (APP_FOLLOWER_STRICT_DIST_MM)
#define FOLLOWER_STRICT_MAX_ATTEMPTS        (APP_FOLLOWER_STRICT_ATTEMPTS)
#define FOLLOWER_ALIGN_RECHECK_TICKS      (100U)    /* 500 ms */
#define FOLLOWER_WALL_MASK_X              (0x01U)
#define FOLLOWER_WALL_MASK_Y              (0x02U)
#define FOLLOWER_CROSS_SOURCE_NONE        (0U)
#define FOLLOWER_CROSS_SOURCE_VISION      (1U)
#define FOLLOWER_CROSS_SOURCE_CORRIDOR    (2U)
#define FOLLOWER_CROSS_SOURCE_SINGLE_WALL (3U)
#define FOLLOWER_POSE_SUSPECT_FRAMES      (6U)
#define FOLLOWER_POSE_SUSPECT_TICKS       (40U)      /* 200 ms */
#define FOLLOWER_POSE_SUSPECT_CLUSTER_X10 (2)        /* 40 mm */
#define FOLLOWER_WALL_SNAP_CROSS_MM       (60.0f)
#define FOLLOWER_WALL_SNAP_ALONG_MIN_MM  (140.0f)
#define FOLLOWER_WALL_SNAP_ALONG_MAX_MM  (260.0f)
#define FOLLOWER_WALL_SNAP_MAX_YAW_DEG    (15.0f)
#define FOLLOWER_RAD_TO_DEG              (57.2957795f)
#define FOLLOWER_DEG_TO_RAD              (0.0174532925f)
#define FOLLOWER_MAX_SEGMENT_CELLS       (APP_FOLLOWER_NORMAL_MAX_CELLS)

static action_follower_debug_t follower_debug;
static float follower_speed_command = FOLLOWER_SPEED_DEFAULT;
static uint32 follower_start_tick;
static uint32 follower_stop_tick;
static uint8 follower_mission_heading_valid;
static float follower_mission_heading_target_deg;
static float follower_map_heading_ref_deg;
static int16 follower_visual_heading_ref_x10;
static uint32 follower_cross_last_pos_packets;
static float follower_cross_raw_mm;
static float follower_cross_filtered_mm;
static uint8 follower_cross_filter_valid;
static uint8 follower_cross_reject_count;
static uint8 follower_dead_reckon_steps;
static uint8 follower_current_dead_reckon;
static uint8 follower_align_active;
static uint8 follower_align_attempts;
static float follower_align_max_distance_mm;
static float follower_start_tolerance_mm;
static uint8 follower_strong_align_reported;
static uint8 follower_wait_recheck;
static uint32 follower_wait_tick;
static uint8 follower_fast_finish;
static uint16 follower_vision_drain_ms;
static uint8 follower_vision_stable_frames;
static uint16 follower_vision_stable_ms;
static uint32 follower_suspect_last_packets;
static uint32 follower_suspect_start_tick;
static int16 follower_suspect_x10;
static int16 follower_suspect_y10;
static uint8 follower_wall_snap_accept_count;

static void follower_stop(action_follower_state_t state,
                          action_follower_fault_t fault);

static float follower_align_pass_axis_mm(void)
{
    return follower_debug.strict_position ?
        FOLLOWER_STRICT_PASS_AXIS_MM : FOLLOWER_ALIGN_PASS_AXIS_MM;
}

static float follower_align_pass_dist_mm(void)
{
    return follower_debug.strict_position ?
        FOLLOWER_STRICT_PASS_DIST_MM : FOLLOWER_ALIGN_PASS_DIST_MM;
}

static uint8 follower_align_max_attempts(void)
{
    return follower_debug.strict_position ?
        FOLLOWER_STRICT_MAX_ATTEMPTS : FOLLOWER_ALIGN_MAX_ATTEMPTS;
}

static float follower_normalize_360(float angle)
{
    while(angle < 0.0f) angle += 360.0f;
    while(angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static float follower_abs_angle_error(float a, float b)
{
    float error = follower_normalize_360(a) - follower_normalize_360(b);
    while(error > 180.0f) error -= 360.0f;
    while(error < -180.0f) error += 360.0f;
    return fabsf(error);
}

static float follower_quantize_cardinal(float angle)
{
    float normalized = follower_normalize_360(angle);
    uint8 quadrant = (uint8)((normalized + 45.0f) / 90.0f) & 3U;
    return (float)quadrant * 90.0f;
}

static uint16 follower_u16(float value)
{
    if(value <= 0.0f) return 0U;
    if(value >= 65535.0f) return 65535U;
    return (uint16)(value + 0.5f);
}

static uint16 follower_speed_u16(void)
{
    if(follower_speed_command < 110.0f) return 100U;
    if(follower_speed_command < 135.0f) return 120U;
    return 150U;
}

static float follower_clampf(float value, float low, float high)
{
    if(value < low) return low;
    if(value > high) return high;
    return value;
}

static int16 follower_float_x10(float value)
{
    if(value >= 0.0f) return (int16)(value * 10.0f + 0.5f);
    return (int16)(value * 10.0f - 0.5f);
}

static uint8 follower_wall_at(const vision_link_map_t *map,
                              uint8 x, uint8 y)
{
    uint16 index;

    if(map == 0 || !map->valid || x >= map->width || y >= map->height)
        return 0U;
    index = (uint16)y * VISION_LINK_GRID_W + x;
    return (uint8)((map->wall_bits[index >> 3] >> (index & 7U)) & 1U);
}

static uint8 follower_target_wall_mask(void)
{
    vision_link_map_t map;
    uint8 x = follower_debug.to_x;
    uint8 y = follower_debug.to_y;
    uint8 mask = 0U;

    if(!vision_link_get_map(&map)) return 0U;
    if((x > 0U && follower_wall_at(&map, (uint8)(x - 1U), y)) ||
       (x + 1U < map.width && follower_wall_at(&map, (uint8)(x + 1U), y)))
    {
        mask |= FOLLOWER_WALL_MASK_X;
    }
    if((y > 0U && follower_wall_at(&map, x, (uint8)(y - 1U))) ||
       (y + 1U < map.height && follower_wall_at(&map, x, (uint8)(y + 1U))))
    {
        mask |= FOLLOWER_WALL_MASK_Y;
    }
    return mask;
}

static uint8 follower_target_corridor_mask(void)
{
    vision_link_map_t map;
    uint8 mask = 0U;
    uint8 x[2] = {follower_debug.from_x, follower_debug.to_x};
    uint8 y[2] = {follower_debug.from_y, follower_debug.to_y};
    uint8 i;

    if(!vision_link_get_map(&map)) return 0U;
    for(i = 0U; i < 2U; i++)
    {
        if(follower_debug.from_x == follower_debug.to_x && x[i] > 0U &&
           x[i] + 1U < map.width &&
           follower_wall_at(&map, (uint8)(x[i] - 1U), y[i]) &&
           follower_wall_at(&map, (uint8)(x[i] + 1U), y[i]))
        {
            mask |= FOLLOWER_WALL_MASK_X;
        }
        if(follower_debug.from_y == follower_debug.to_y && y[i] > 0U &&
           y[i] + 1U < map.height &&
           follower_wall_at(&map, x[i], (uint8)(y[i] - 1U)) &&
           follower_wall_at(&map, x[i], (uint8)(y[i] + 1U)))
        {
            mask |= FOLLOWER_WALL_MASK_Y;
        }
    }
    return mask;
}

static uint8 follower_pose_fresh(const vision_link_snapshot_t *pose)
{
    return (vision_link_is_online() && pose->pose_valid &&
            pose->last_packet_tick != 0U &&
            (uint32)(pit_count - pose->last_packet_tick) <=
                FOLLOWER_POSE_STALE_TICKS) ? 1U : 0U;
}

static uint8 follower_round_grid_x10(int16 value)
{
    if(value <= 0) return 0U;
    return (uint8)(((uint16)value + 5U) / 10U);
}

static uint8 follower_accept_wall_snap(int16 position_x10,
                                       int16 position_y10)
{
    point_test_snapshot_t point;
    int step_x = (int)follower_debug.to_x - (int)follower_debug.from_x;
    int step_y = (int)follower_debug.to_y - (int)follower_debug.from_y;
    float error_x = (float)follower_debug.target_x_mm -
                    (float)position_x10 * 20.0f;
    float error_y = (float)follower_debug.target_y_mm -
                    (float)position_y10 * 20.0f;
    float along;
    float cross;
    uint8 wall_context;

    if(follower_debug.segment_cells != 1U ||
       follower_debug.interaction_locked || follower_debug.strong_reanchor ||
       follower_round_grid_x10(position_x10) != follower_debug.from_x ||
       follower_round_grid_x10(position_y10) != follower_debug.from_y)
    {
        return 0U;
    }
    wall_context = (uint8)(
        follower_debug.gear == ACTION_GEAR_SINGLE_WALL_FAST ||
        follower_debug.gear == ACTION_GEAR_CORRIDOR_FAST ||
        follower_debug.wall_axis_mask != 0U ||
        follower_debug.corridor_axis_mask != 0U);
    if(!wall_context) return 0U;

    if(step_x != 0)
    {
        along = error_x * (float)step_x;
        cross = error_y;
    }
    else
    {
        along = error_y * (float)step_y;
        cross = error_x;
    }
    follower_debug.wall_snap_candidate = 1U;
    follower_debug.wall_snap_along_mm = (int16)along;
    follower_debug.wall_snap_cross_mm = (int16)cross;
    point_test_get_snapshot(&point);
    if(point.state != POINT_TEST_DONE || point.fault != POINT_FAULT_NONE ||
       fabsf(point.imu_relative_deg) > FOLLOWER_WALL_SNAP_MAX_YAW_DEG ||
       fabsf(cross) > FOLLOWER_WALL_SNAP_CROSS_MM ||
       along < FOLLOWER_WALL_SNAP_ALONG_MIN_MM ||
       along > FOLLOWER_WALL_SNAP_ALONG_MAX_MM)
    {
        return 0U;
    }

    if(follower_wall_snap_accept_count < 255U)
        follower_wall_snap_accept_count++;
    follower_debug.wall_snap_accepted = 1U;
    follower_debug.wall_snap_accept_count = follower_wall_snap_accept_count;
    follower_debug.align_dx_mm = 0;
    follower_debug.align_dy_mm = 0;
    follower_debug.dx_mm = 0;
    follower_debug.dy_mm = 0;
    follower_debug.distance_mm = 0U;
    follower_dead_reckon_steps = 0U;
    follower_current_dead_reckon = 0U;
    follower_align_active = 0U;
    follower_wait_recheck = 0U;
    follower_stop(ACTION_FOLLOWER_DONE, ACTION_FOLLOWER_FAULT_NONE);
    return 1U;
}

static float follower_cross_error_mm(const vision_link_snapshot_t *pose)
{
    float map_rad = (float)follower_debug.map_direction_x10 * 0.1f *
                    FOLLOWER_DEG_TO_RAD;
    float dx = (float)follower_debug.target_x_mm -
               (float)pose->car_x_mm * 20.0f;
    float dy = (float)follower_debug.target_y_mm -
               (float)pose->car_y_mm * 20.0f;

    return dx * cosf(map_rad) + dy * sinf(map_rad);
}

static void follower_cross_reset(const vision_link_snapshot_t *pose)
{
    follower_cross_last_pos_packets = pose->pos_packets;
    follower_cross_raw_mm = follower_cross_error_mm(pose);
    follower_cross_filtered_mm = follower_cross_raw_mm;
    follower_cross_filter_valid = 1U;
    follower_cross_reject_count = 0U;
    follower_debug.cross_error_raw_x10 =
        follower_float_x10(follower_cross_raw_mm);
    follower_debug.cross_error_filtered_x10 =
        follower_float_x10(follower_cross_filtered_mm);
    follower_debug.cross_correction_speed_x10 = 0;
    follower_debug.cross_correction_active = 0U;
    follower_debug.cross_reject_count = 0U;
    follower_debug.cross_source = FOLLOWER_CROSS_SOURCE_VISION;
    follower_debug.corrected_body_command_x10 =
        follower_debug.body_command_x10;
}

static void follower_cross_reset_dead_reckon(void)
{
    follower_cross_last_pos_packets = 0U;
    follower_cross_raw_mm = 0.0f;
    follower_cross_filtered_mm = 0.0f;
    follower_cross_filter_valid = 0U;
    follower_cross_reject_count = 0U;
    follower_debug.cross_error_raw_x10 = 0;
    follower_debug.cross_error_filtered_x10 = 0;
    follower_debug.cross_correction_speed_x10 = 0;
    follower_debug.cross_correction_active = 0U;
    follower_debug.cross_reject_count = 0U;
    follower_debug.cross_source = FOLLOWER_CROSS_SOURCE_NONE;
    follower_debug.corrected_body_command_x10 =
        follower_debug.body_command_x10;
}

static void follower_update_cross_track(const point_test_snapshot_t *point)
{
    vision_link_snapshot_t pose;
    float base_body_deg = (float)follower_debug.body_command_x10 * 0.1f;
    float map_deg;
    float map_rad;
    float cross_delta;
    float correction_speed;
    float along_speed;
    float command_x;
    float command_y;
    float corrected_map_deg;
    float corrected_body_deg;
    float cross_deadband = FOLLOWER_CROSS_DEADBAND_MM;
    float cross_kp = FOLLOWER_CROSS_KP;
    float cross_max_speed = FOLLOWER_CROSS_MAX_SPEED;
    uint8 early_start_allowed;
    uint8 corridor_active;
    uint8 single_wall_active = 0U;

    follower_debug.cross_correction_active = 0U;
    follower_debug.cross_correction_speed_x10 = 0;
    follower_debug.corrected_body_command_x10 =
        follower_debug.body_command_x10;
    follower_debug.cross_source = FOLLOWER_CROSS_SOURCE_NONE;

    if(point->state != POINT_TEST_RUNNING ||
       point->kind != POINT_TEST_KIND_TRANSLATE ||
       follower_align_active)
    {
        return;
    }

    corridor_active = (uint8)(
        ((follower_debug.from_x == follower_debug.to_x) &&
         (follower_debug.corridor_axis_mask & FOLLOWER_WALL_MASK_X)) ||
        ((follower_debug.from_y == follower_debug.to_y) &&
         (follower_debug.corridor_axis_mask & FOLLOWER_WALL_MASK_Y)));

    if(corridor_active)
    {
        /* Vision is wall-clamped in a one-cell corridor. Keep the local
           encoder cross displacement near zero instead of chasing it. */
        follower_cross_raw_mm = -point->encoder_cross_mm;
        if(!follower_cross_filter_valid)
        {
            follower_cross_filtered_mm = follower_cross_raw_mm;
            follower_cross_filter_valid = 1U;
        }
        else
        {
            follower_cross_filtered_mm +=
                (follower_cross_raw_mm - follower_cross_filtered_mm) *
                FOLLOWER_CROSS_FILTER_ALPHA;
        }
        follower_debug.cross_source = FOLLOWER_CROSS_SOURCE_CORRIDOR;
        cross_deadband = FOLLOWER_CORRIDOR_DEADBAND_MM;
        cross_kp = FOLLOWER_CORRIDOR_CROSS_KP;
        cross_max_speed = FOLLOWER_CORRIDOR_MAX_SPEED;
    }
    else
    {
        single_wall_active = (uint8)(
            ((follower_debug.from_x == follower_debug.to_x) &&
             (follower_debug.wall_axis_mask & FOLLOWER_WALL_MASK_X)) ||
            ((follower_debug.from_y == follower_debug.to_y) &&
             (follower_debug.wall_axis_mask & FOLLOWER_WALL_MASK_Y)));
        vision_link_get_snapshot(&pose);
        if(!vision_link_is_online() || !pose.pose_valid ||
           pose.last_packet_tick == 0U ||
           (uint32)(pit_count - pose.last_packet_tick) >
               FOLLOWER_CROSS_STALE_TICKS)
        {
            point_test_set_runtime_direction_deg(base_body_deg);
            return;
        }

        if(pose.pos_packets != follower_cross_last_pos_packets)
        {
            follower_cross_last_pos_packets = pose.pos_packets;
            follower_cross_raw_mm = follower_cross_error_mm(&pose);
            if(!follower_cross_filter_valid)
            {
                follower_cross_filtered_mm = follower_cross_raw_mm;
                follower_cross_filter_valid = 1U;
            }
            else
            {
                cross_delta = follower_cross_raw_mm -
                              follower_cross_filtered_mm;
                if(fabsf(cross_delta) <= FOLLOWER_CROSS_JUMP_MM)
                {
                    follower_cross_filtered_mm +=
                        cross_delta * FOLLOWER_CROSS_FILTER_ALPHA;
                }
                else if(follower_cross_reject_count < 255U)
                {
                    follower_cross_reject_count++;
                }
            }
        }
        if(single_wall_active)
        {
            follower_debug.cross_source =
                FOLLOWER_CROSS_SOURCE_SINGLE_WALL;
            cross_kp = follower_debug.interaction_locked ?
                FOLLOWER_PUSH_WALL_CROSS_KP : FOLLOWER_SINGLE_WALL_CROSS_KP;
            cross_max_speed = follower_debug.interaction_locked ?
                FOLLOWER_PUSH_WALL_MAX_SPEED : FOLLOWER_SINGLE_WALL_MAX_SPEED;
        }
        else
        {
            follower_debug.cross_source = FOLLOWER_CROSS_SOURCE_VISION;
        }
    }

    follower_debug.cross_error_raw_x10 =
        follower_float_x10(follower_cross_raw_mm);
    follower_debug.cross_error_filtered_x10 =
        follower_float_x10(follower_cross_filtered_mm);
    follower_debug.cross_reject_count = follower_cross_reject_count;

    /* The step is armed only after a stable pose. Large cross errors need
       correction immediately, including the first 200 mm step. */
    early_start_allowed =
        (fabsf(follower_cross_filtered_mm) >=
             FOLLOWER_CROSS_EARLY_START_MM) ? 1U : 0U;

    if(!follower_cross_filter_valid ||
       (point->elapsed_ms < FOLLOWER_CROSS_START_TICKS * 5U &&
        !early_start_allowed) ||
       point->remaining <= FOLLOWER_CROSS_DISABLE_MM ||
       fabsf(follower_cross_filtered_mm) <= cross_deadband ||
       point->commanded_speed == 0U)
    {
        point_test_set_runtime_direction_deg(base_body_deg);
        return;
    }

    /* A wall-clamped global coordinate is trustworthy only when it says the
       car drifted away from the wall. Never command farther into the wall. */
    if(single_wall_active &&
       (follower_debug.single_wall_correction_sign == 0 ||
        follower_cross_filtered_mm *
            (float)follower_debug.single_wall_correction_sign <=
            cross_deadband))
    {
        point_test_set_runtime_direction_deg(base_body_deg);
        return;
    }

    correction_speed = follower_clampf(
        follower_cross_filtered_mm * cross_kp,
        -cross_max_speed,
        cross_max_speed);
    along_speed = (float)point->commanded_speed;
    map_deg = (float)follower_debug.map_direction_x10 * 0.1f;
    map_rad = map_deg * FOLLOWER_DEG_TO_RAD;
    command_x = along_speed * sinf(map_rad) +
                correction_speed * cosf(map_rad);
    command_y = -along_speed * cosf(map_rad) +
                correction_speed * sinf(map_rad);
    corrected_map_deg = follower_normalize_360(
        atan2f(command_x, -command_y) * FOLLOWER_RAD_TO_DEG);
    corrected_body_deg = follower_normalize_360(
        corrected_map_deg - follower_map_heading_ref_deg);

    if(point_test_set_runtime_direction_deg(corrected_body_deg))
    {
        follower_debug.cross_correction_active = 1U;
        follower_debug.cross_correction_speed_x10 =
            follower_float_x10(correction_speed);
        follower_debug.corrected_body_command_x10 =
            follower_float_x10(corrected_body_deg);
    }
}

static action_follower_fault_t follower_point_fault(point_test_fault_t fault)
{
    switch(fault)
    {
        case POINT_FAULT_VISION_NOT_READY: return ACTION_FOLLOWER_FAULT_POSE_INVALID;
        case POINT_FAULT_TIMEOUT: return ACTION_FOLLOWER_FAULT_TIMEOUT;
        case POINT_FAULT_OVERTRAVEL: return ACTION_FOLLOWER_FAULT_STALLED;
        case POINT_FAULT_VIS_CONFIRM_TIMEOUT: return ACTION_FOLLOWER_FAULT_VIS_CONFIRM_TIMEOUT;
        case POINT_FAULT_GRID_MISMATCH: return ACTION_FOLLOWER_FAULT_GRID_MISMATCH;
        case POINT_FAULT_STALL: return ACTION_FOLLOWER_FAULT_STALLED;
        default: return ACTION_FOLLOWER_FAULT_BAD_ACTION;
    }
}

static void follower_stop(action_follower_state_t state,
                          action_follower_fault_t fault)
{
    follower_debug.speed_command = 0;
    follower_debug.state = state;
    follower_debug.fault = fault;
    follower_debug.cross_correction_active = 0U;
    follower_debug.cross_correction_speed_x10 = 0;
    follower_stop_tick = pit_count;
}

static void follower_update_debug(const point_test_snapshot_t *point)
{
    vision_link_snapshot_t pose;
    float dx;
    float dy;

    vision_link_get_snapshot(&pose);
    follower_debug.pose_valid = pose.pose_valid;
    follower_debug.car_x10 = pose.car_x_mm;
    follower_debug.car_y10 = pose.car_y_mm;
    follower_debug.car_theta_x10 = pose.car_theta_x10;
    follower_debug.pose_age_ms = pose.last_packet_tick == 0U ? 99999UL :
        (uint32)(pit_count - pose.last_packet_tick) * 5UL;
    dx = (float)follower_debug.target_x_mm - (float)pose.car_x_mm * 20.0f;
    dy = (float)follower_debug.target_y_mm - (float)pose.car_y_mm * 20.0f;
    follower_debug.dx_mm = (int16)dx;
    follower_debug.dy_mm = (int16)dy;
    follower_debug.distance_mm = follower_u16(sqrtf(dx * dx + dy * dy));
    follower_debug.speed_command = (int16)point->commanded_speed;
    follower_debug.waypoint_hold_remaining_ms = point->settle_remaining_ms;
    if(point->state == POINT_TEST_VIS_DRAIN)
        follower_debug.waypoint_hold_remaining_ms = point->vision_drain_remaining_ms;
    follower_debug.visual_confirm_frames = point->vision_confirm_frames;
    follower_debug.visual_confirm_required = point->vision_confirm_required;
    follower_debug.visual_confirm_bad_frames = point->vision_confirm_bad_frames;
    follower_debug.visual_confirm_final = 1U;
    follower_debug.visual_fallback_used = point->vision_fallback_used;
    follower_debug.dead_reckon_active = follower_current_dead_reckon;
    follower_debug.dead_reckon_steps = follower_dead_reckon_steps;
    follower_debug.align_active = follower_align_active;
    follower_debug.align_attempts = follower_align_attempts;
    follower_debug.vision_confirm_valid = point->vision_confirm_valid;
    follower_debug.vision_confirm_x10 = point->vision_confirm_x10;
    follower_debug.vision_confirm_y10 = point->vision_confirm_y10;
    follower_debug.visual_confirm_age_ms = point->vision_confirm_age_ms;
    follower_debug.visual_confirm_pos_packets = point->vision_confirm_pos_packets;
    follower_debug.elapsed_ms = point->elapsed_ms;
    follower_debug.progress_age_ms = 0U;

    if(point->state == POINT_TEST_RUNNING)
        follower_debug.phase = follower_align_active ?
            ACTION_FOLLOWER_PHASE_ALIGN_MOVE : ACTION_FOLLOWER_PHASE_MOVE;
    else if(point->state == POINT_TEST_SETTLING)
        follower_debug.phase = follower_align_active ?
            ACTION_FOLLOWER_PHASE_ALIGN_RECHECK :
            ACTION_FOLLOWER_PHASE_SETTLE;
    else if(point->state == POINT_TEST_VIS_DRAIN)
        follower_debug.phase = follower_align_active ?
            ACTION_FOLLOWER_PHASE_ALIGN_RECHECK :
            ACTION_FOLLOWER_PHASE_VIS_DRAIN;
    else if(point->state == POINT_TEST_VIS_STABLE)
        follower_debug.phase = follower_align_active ?
            ACTION_FOLLOWER_PHASE_ALIGN_RECHECK :
            ACTION_FOLLOWER_PHASE_VIS_STABLE;
}

static void follower_enter_visual_wait(void)
{
    if(!follower_wait_recheck)
    {
        point_test_emergency_stop();
        follower_wait_tick = pit_count;
    }
    follower_wait_recheck = 1U;
    follower_align_active = 0U;
    follower_debug.align_active = 0U;
    follower_debug.speed_command = 0;
    follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_VISION;
}

static void follower_enter_pose_suspect(int16 position_x10,
                                        int16 position_y10)
{
    vision_link_snapshot_t pose;

    follower_enter_visual_wait();
    vision_link_get_snapshot(&pose);
    follower_suspect_x10 = position_x10;
    follower_suspect_y10 = position_y10;
    follower_suspect_last_packets = pose.pos_packets;
    follower_suspect_start_tick = pit_count;
    follower_debug.pose_suspect = 1U;
    follower_debug.pose_suspect_frames = 1U;
    follower_debug.pose_suspect_grid_x =
        follower_round_grid_x10(position_x10);
    follower_debug.pose_suspect_grid_y =
        follower_round_grid_x10(position_y10);
}

static uint8 follower_start_alignment(float correction_x_mm,
                                      float correction_y_mm)
{
    point_test_snapshot_t point;
    float correction_distance;
    float map_direction;
    float body_direction;
    uint16 command_distance;

    correction_distance = sqrtf(correction_x_mm * correction_x_mm +
                                correction_y_mm * correction_y_mm);
    if(correction_distance < follower_align_pass_dist_mm() ||
       correction_distance > follower_align_max_distance_mm ||
       follower_align_attempts >= follower_align_max_attempts())
    {
        return 0U;
    }

    map_direction = follower_normalize_360(
        atan2f(correction_x_mm, -correction_y_mm) * FOLLOWER_RAD_TO_DEG);
    body_direction = follower_normalize_360(
        map_direction - follower_map_heading_ref_deg);

    if(!point_test_set_sensor_mode(POINT_SENSOR_FUSION_LOCKED) ||
       !point_test_set_speed(100U) ||
       !point_test_set_direction_deg(body_direction) ||
       !point_test_set_fast_finish(0U))
    {
        follower_enter_visual_wait();
        return 0U;
    }
    point_test_get_snapshot(&point);
    command_distance = (uint16)(correction_distance +
                                (float)point.brake_lead_mm + 0.5f);
    if(command_distance > 300U) command_distance = 300U;

    if(!point_test_set_distance_mm(command_distance) ||
       !point_test_set_heading_target(follower_mission_heading_target_deg) ||
       !point_test_capture_origin() ||
       !point_test_start_translation())
    {
        follower_enter_visual_wait();
        return 0U;
    }

    follower_align_active = 1U;
    if(follower_debug.strong_reanchor && !follower_strong_align_reported)
    {
        follower_strong_align_reported = 1U;
        status_buzzer_request(STATUS_BUZZER_EVENT_STRONG_ALIGN);
    }
    follower_align_attempts++;
    follower_wait_recheck = 0U;
    follower_current_dead_reckon = 0U;
    follower_debug.align_active = 1U;
    follower_debug.align_attempts = follower_align_attempts;
    follower_debug.map_direction_x10 = follower_float_x10(map_direction);
    follower_debug.body_command_x10 = follower_float_x10(body_direction);
    follower_debug.corrected_body_command_x10 =
        follower_debug.body_command_x10;
    follower_debug.phase = ACTION_FOLLOWER_PHASE_ALIGN_MOVE;
    follower_debug.speed_command = 100;
    follower_debug.position_rebase_count++;
    follower_cross_reset_dead_reckon();
    return 1U;
}

static uint8 follower_accept_or_align_position(int16 position_x10,
                                               int16 position_y10)
{
    float dx = (float)follower_debug.target_x_mm -
               (float)position_x10 * 20.0f;
    float dy = (float)follower_debug.target_y_mm -
               (float)position_y10 * 20.0f;
    float distance;
    uint8 same_cell_x =
        (follower_round_grid_x10(position_x10) == follower_debug.to_x) ? 1U : 0U;
    uint8 same_cell_y =
        (follower_round_grid_x10(position_y10) == follower_debug.to_y) ? 1U : 0U;

    if(follower_accept_wall_snap(position_x10, position_y10)) return 1U;

    if(same_cell_x &&
       (follower_debug.wall_axis_mask & FOLLOWER_WALL_MASK_X) &&
       !follower_debug.strong_reanchor &&
       fabsf(dx) <= follower_align_max_distance_mm)
    {
        dx = 0.0f;
    }
    if(same_cell_y &&
       (follower_debug.wall_axis_mask & FOLLOWER_WALL_MASK_Y) &&
       !follower_debug.strong_reanchor &&
       fabsf(dy) <= follower_align_max_distance_mm)
    {
        dy = 0.0f;
    }

    distance = sqrtf(dx * dx + dy * dy);
    follower_debug.align_dx_mm = (int16)dx;
    follower_debug.align_dy_mm = (int16)dy;
    follower_debug.dx_mm = (int16)dx;
    follower_debug.dy_mm = (int16)dy;
    follower_debug.distance_mm = follower_u16(distance);

    if((fabsf(dx) <= follower_align_pass_axis_mm() &&
        fabsf(dy) <= follower_align_pass_axis_mm() &&
        distance <= follower_align_pass_dist_mm()) ||
       (same_cell_x && same_cell_y &&
         !follower_debug.strict_position &&
         follower_align_attempts >= follower_align_max_attempts()))
    {
        follower_dead_reckon_steps = 0U;
        follower_current_dead_reckon = 0U;
        follower_align_active = 0U;
        follower_wait_recheck = 0U;
        follower_debug.align_active = 0U;
        follower_stop(ACTION_FOLLOWER_DONE, ACTION_FOLLOWER_FAULT_NONE);
        return 1U;
    }

    if(distance <= follower_align_max_distance_mm &&
       follower_align_attempts < follower_align_max_attempts())
    {
        return follower_start_alignment(dx, dy);
    }

    if(follower_debug.strict_position && same_cell_x && same_cell_y)
    {
        follower_stop(ACTION_FOLLOWER_REPLAN,
                      ACTION_FOLLOWER_FAULT_GRID_MISMATCH);
        return 0U;
    }

    follower_enter_pose_suspect(position_x10, position_y10);
    return 0U;
}

static uint8 follower_try_begin(void)
{
    point_test_snapshot_t point;
    vision_link_snapshot_t pose;
    float current_x_mm;
    float current_y_mm;
    float from_dx;
    float from_dy;
    float target_dx;
    float target_dy;
    float target_distance;
    float target_euclidean;
    float map_direction;
    float body_direction;
    int step_dx;
    int step_dy;
    int step_unit_x;
    int step_unit_y;
    uint8 vision_ready;
    uint8 dead_reckon_start = 0U;

    point_test_get_snapshot(&point);
    vision_link_get_snapshot(&pose);
    vision_ready =
        (follower_pose_fresh(&pose) &&
         point.vision_position_stable) ? 1U : 0U;

    if(follower_debug.logical_origin_override)
    {
        if(!follower_mission_heading_valid) return 0U;
        current_x_mm =
            (float)follower_debug.from_x * SOLVER_GRID_SIZE_MM;
        current_y_mm =
            (float)follower_debug.from_y * SOLVER_GRID_SIZE_MM;
        dead_reckon_start = 1U;
    }
    else if(vision_ready)
    {
        current_x_mm = (float)pose.car_x_mm * 20.0f;
        current_y_mm = (float)pose.car_y_mm * 20.0f;
        from_dx = current_x_mm -
                  (float)follower_debug.from_x * SOLVER_GRID_SIZE_MM;
        from_dy = current_y_mm -
                  (float)follower_debug.from_y * SOLVER_GRID_SIZE_MM;
        if(sqrtf(from_dx * from_dx + from_dy * from_dy) >
           follower_start_tolerance_mm)
        {
            follower_debug.dx_mm = (int16)(-from_dx);
            follower_debug.dy_mm = (int16)(-from_dy);
            if(!follower_debug.strict_position &&
               follower_round_grid_x10(pose.car_x_mm) ==
                   follower_debug.from_x &&
               follower_round_grid_x10(pose.car_y_mm) ==
                   follower_debug.from_y)
            {
                /* The host may clamp a wall-adjacent coordinate near a cell
                   corner. The stable cell identity is still a valid origin. */
                current_x_mm =
                    (float)follower_debug.from_x * SOLVER_GRID_SIZE_MM;
                current_y_mm =
                    (float)follower_debug.from_y * SOLVER_GRID_SIZE_MM;
            }
            else
            {
                follower_enter_pose_suspect(pose.car_x_mm, pose.car_y_mm);
                return 0U;
            }
        }
        follower_dead_reckon_steps = 0U;
    }
    else
    {
        if(!follower_mission_heading_valid ||
           follower_dead_reckon_steps == 0U ||
           follower_dead_reckon_steps >= FOLLOWER_DEAD_RECKON_MAX_STEPS)
        {
            return 0U;
        }
        current_x_mm =
            (float)follower_debug.from_x * SOLVER_GRID_SIZE_MM;
        current_y_mm =
            (float)follower_debug.from_y * SOLVER_GRID_SIZE_MM;
        dead_reckon_start = 1U;
    }

    target_dx = (float)follower_debug.target_x_mm - current_x_mm;
    target_dy = (float)follower_debug.target_y_mm - current_y_mm;
    target_euclidean = sqrtf(target_dx * target_dx + target_dy * target_dy);
    if(target_euclidean <= FOLLOWER_ALREADY_THERE_MM)
    {
        follower_stop(ACTION_FOLLOWER_DONE, ACTION_FOLLOWER_FAULT_NONE);
        return 1U;
    }

    if(!follower_mission_heading_valid &&
       !action_follower_begin_mission_heading())
    {
        follower_stop(ACTION_FOLLOWER_FAULT,
                      ACTION_FOLLOWER_FAULT_BAD_ACTION);
        return 0U;
    }

    /* Keep the nominal move cardinal; bounded vision correction is lateral only. */
    step_dx = (int)follower_debug.to_x - (int)follower_debug.from_x;
    step_dy = (int)follower_debug.to_y - (int)follower_debug.from_y;
    step_unit_x = step_dx > 0 ? 1 : (step_dx < 0 ? -1 : 0);
    step_unit_y = step_dy > 0 ? 1 : (step_dy < 0 ? -1 : 0);
    target_distance = target_dx * (float)step_unit_x +
                      target_dy * (float)step_unit_y;
    if(target_distance < 50.0f ||
       target_distance >
           (float)follower_debug.segment_cells * SOLVER_GRID_SIZE_MM +
               follower_start_tolerance_mm)
    {
        /* A multi-cell cruise may coast slightly beyond its final node.  The
           terminal phase is explicitly allowed to pull back in two axes so
           it cannot wait forever on a negative forward projection. */
        if(follower_debug.terminal_node && vision_ready &&
           target_euclidean <= follower_align_max_distance_mm &&
           follower_align_attempts < follower_align_max_attempts())
        {
            return follower_start_alignment(target_dx, target_dy);
        }
        if(vision_ready) return 0U;
        follower_stop(ACTION_FOLLOWER_FAULT,
                      ACTION_FOLLOWER_FAULT_START_MISMATCH);
        return 0U;
    }

    if(step_dx > 0) map_direction = 90.0f;
    else if(step_dy > 0) map_direction = 180.0f;
    else if(step_dx < 0) map_direction = 270.0f;
    else map_direction = 0.0f;
    body_direction = follower_normalize_360(
        map_direction - follower_map_heading_ref_deg);
    follower_debug.map_direction_x10 = (int16)(map_direction * 10.0f + 0.5f);
    follower_debug.body_command_x10 = (int16)(body_direction * 10.0f + 0.5f);
    follower_debug.corrected_body_command_x10 =
        follower_debug.body_command_x10;
    follower_current_dead_reckon = dead_reckon_start;
    if(vision_ready && !dead_reckon_start) follower_cross_reset(&pose);
    else follower_cross_reset_dead_reckon();

    if(!point_test_set_sensor_mode(dead_reckon_start ?
                                      POINT_SENSOR_ENCODER_IMU :
                                      POINT_SENSOR_FUSION_LOCKED) ||
       !point_test_set_speed(follower_speed_u16()) ||
       !point_test_set_distance_mm((uint16)(target_distance + 0.5f)) ||
       !point_test_set_direction_deg(body_direction) ||
       !point_test_set_fast_finish(follower_fast_finish) ||
       !point_test_set_fusion_profile(follower_vision_drain_ms,
                                       follower_vision_stable_frames,
                                       follower_vision_stable_ms) ||
       !point_test_set_heading_target(follower_mission_heading_target_deg) ||
       !point_test_capture_origin())
    {
        point_test_get_snapshot(&point);
        if(point.fault == POINT_FAULT_VISION_NOT_READY)
        {
            follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_ORIGIN;
            return 0U;
        }
        follower_stop(ACTION_FOLLOWER_FAULT,
                      follower_point_fault(point.fault));
        return 0U;
    }

    if(follower_debug.pwm_ramp_ms > 0U &&
       follower_debug.pwm_ramp_max_delta > 0U)
    {
        motion_pwm_ramp_begin(follower_debug.pwm_ramp_ms,
                              follower_debug.pwm_ramp_max_delta);
    }
    if(!point_test_start_translation())
    {
        motion_pwm_ramp_cancel();
        point_test_get_snapshot(&point);
        if(point.fault == POINT_FAULT_VISION_NOT_READY)
        {
            follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_ORIGIN;
            return 0U;
        }
        follower_stop(ACTION_FOLLOWER_FAULT,
                      follower_point_fault(point.fault));
        return 0U;
    }

    follower_debug.phase = ACTION_FOLLOWER_PHASE_MOVE;
    follower_debug.speed_command = (int16)follower_speed_u16();
    follower_debug.position_rebase_count++;
    return 1U;
}

void action_follower_init(void)
{
    memset(&follower_debug, 0, sizeof(follower_debug));
    follower_debug.state = ACTION_FOLLOWER_IDLE;
    follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_ORIGIN;
    follower_debug.fault = ACTION_FOLLOWER_FAULT_NONE;
    follower_debug.nominal_speed_command = (int16)follower_speed_command;
    follower_start_tick = pit_count;
    follower_stop_tick = pit_count;
    follower_mission_heading_valid = 0U;
    follower_mission_heading_target_deg = 0.0f;
    follower_map_heading_ref_deg = 0.0f;
    follower_visual_heading_ref_x10 = 0;
    follower_cross_last_pos_packets = 0U;
    follower_cross_raw_mm = 0.0f;
    follower_cross_filtered_mm = 0.0f;
    follower_cross_filter_valid = 0U;
    follower_cross_reject_count = 0U;
    follower_dead_reckon_steps = 0U;
    follower_current_dead_reckon = 0U;
    follower_align_active = 0U;
    follower_align_attempts = 0U;
    follower_align_max_distance_mm = FOLLOWER_ALIGN_MAX_DIST_MM;
    follower_start_tolerance_mm = FOLLOWER_START_TOLERANCE_MM;
    follower_strong_align_reported = 0U;
    follower_wait_recheck = 0U;
    follower_wait_tick = pit_count;
    follower_fast_finish = 0U;
    follower_vision_drain_ms = 1000U;
    follower_vision_stable_frames = 6U;
    follower_vision_stable_ms = 200U;
    follower_suspect_last_packets = 0U;
    follower_suspect_start_tick = pit_count;
    follower_suspect_x10 = 0;
    follower_suspect_y10 = 0;
    follower_wall_snap_accept_count = 0U;
    point_test_init();
    point_test_set_sensor_mode(POINT_SENSOR_FUSION_LOCKED);
}

uint8 action_follower_begin_mission_heading(void)
{
    point_test_snapshot_t point;
    vision_link_snapshot_t pose;
    float visual_heading;
    float quantized_heading;

    point_test_get_snapshot(&point);
    vision_link_get_snapshot(&pose);
    if(!imu963ra_ready || !vision_link_is_online() || !pose.pose_valid ||
       !point.vision_input_stable)
    {
        return 0U;
    }

    /* Camera heading chooses the map quadrant once; IMU holds the physical heading. */
    visual_heading = follower_normalize_360(
        (float)pose.car_theta_x10 * 0.1f);
    quantized_heading = follower_quantize_cardinal(visual_heading);
    if(follower_abs_angle_error(visual_heading, quantized_heading) >
       FOLLOWER_MAP_HEADING_MAX_ERR_DEG)
    {
        return 0U;
    }

    if(!action_follower_set_mission_heading_frame(
           imu963ra_yaw_angle, quantized_heading, pose.car_theta_x10))
        return 0U;
    follower_debug.pose_valid = 1U;
    follower_debug.car_x10 = pose.car_x_mm;
    follower_debug.car_y10 = pose.car_y_mm;
    follower_debug.car_theta_x10 = pose.car_theta_x10;
    return 1U;
}

uint8 action_follower_begin_mission_heading_from_visual(
    int16 visual_heading_x10)
{
    float visual_heading;
    float quantized_heading;

    if(!imu963ra_ready) return 0U;
    visual_heading = follower_normalize_360(
        (float)visual_heading_x10 * 0.1f);
    quantized_heading = follower_quantize_cardinal(visual_heading);
    if(follower_abs_angle_error(visual_heading, quantized_heading) >
       FOLLOWER_MAP_HEADING_MAX_ERR_DEG)
        return 0U;
    return action_follower_set_mission_heading_frame(
        imu963ra_yaw_angle, quantized_heading, visual_heading_x10);
}

uint8 action_follower_set_mission_heading_frame(float imu_heading_deg,
                                                 float map_heading_deg,
                                                 int16 visual_heading_x10)
{
    float quantized_heading = follower_quantize_cardinal(map_heading_deg);

    if(!imu963ra_ready ||
       follower_abs_angle_error(map_heading_deg, quantized_heading) > 1.0f)
        return 0U;

    follower_mission_heading_target_deg =
        follower_normalize_360(imu_heading_deg);
    follower_map_heading_ref_deg = quantized_heading;
    follower_visual_heading_ref_x10 = visual_heading_x10;
    follower_mission_heading_valid = 1U;
    follower_dead_reckon_steps = 0U;
    follower_current_dead_reckon = 0U;
    follower_align_active = 0U;
    follower_align_attempts = 0U;
    follower_wait_recheck = 0U;
    follower_debug.visual_heading_ref_x10 = visual_heading_x10;
    follower_debug.physical_heading_x10 =
        follower_float_x10(follower_map_heading_ref_deg);
    return 1U;
}

uint8 action_follower_heading_frame_valid(void)
{
    return follower_mission_heading_valid;
}

void action_follower_end_mission_heading(void)
{
    follower_mission_heading_valid = 0U;
    follower_mission_heading_target_deg = 0.0f;
    follower_map_heading_ref_deg = 0.0f;
    follower_visual_heading_ref_x10 = 0;
    follower_dead_reckon_steps = 0U;
    follower_current_dead_reckon = 0U;
    follower_fast_finish = 0U;
    follower_debug.pose_suspect = 0U;
    point_test_clear_heading_target();
}

uint8 action_follower_get_heading_frame(float *imu_heading_deg,
                                        float *map_heading_deg)
{
    if(!follower_mission_heading_valid || imu_heading_deg == 0 ||
       map_heading_deg == 0)
        return 0U;
    *imu_heading_deg = follower_mission_heading_target_deg;
    *map_heading_deg = follower_map_heading_ref_deg;
    return 1U;
}

void action_follower_set_speed(float speed)
{
    if(speed < 100.0f) speed = 100.0f;
    if(speed > FOLLOWER_SPEED_MAX) speed = FOLLOWER_SPEED_MAX;
    follower_speed_command = speed;
}

float action_follower_get_speed(void)
{
    return follower_speed_command;
}

uint8 action_follower_start_grid_step(uint8 action_index,
                                      uint8 substep_index,
                                      uint8 action_step_count,
                                      uint16 global_step_index,
                                      uint8 from_x,
                                      uint8 from_y,
                                      uint8 to_x,
                                      uint8 to_y,
                                      uint8 fast_finish,
                                      const action_follower_step_context_t *context)
{
    int dx = abs((int)to_x - (int)from_x);
    int dy = abs((int)to_y - (int)from_y);
    uint8 segment_cells = (uint8)(dx + dy);

    if(follower_debug.state == ACTION_FOLLOWER_RUNNING ||
       (dx != 0 && dy != 0) || segment_cells < 1U ||
       segment_cells > FOLLOWER_MAX_SEGMENT_CELLS ||
       to_x >= MAP_MAX_W || to_y >= MAP_MAX_H ||
       (context != 0 && context->segment_cells != segment_cells))
    {
        follower_stop(ACTION_FOLLOWER_FAULT,
                      ACTION_FOLLOWER_FAULT_BAD_ACTION);
        return 0U;
    }

    point_test_emergency_stop();
    memset(&follower_debug, 0, sizeof(follower_debug));
    follower_debug.state = ACTION_FOLLOWER_RUNNING;
    follower_debug.fault = ACTION_FOLLOWER_FAULT_NONE;
    follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_ORIGIN;
    follower_debug.action_index = action_index;
    follower_debug.waypoint_index = substep_index;
    follower_debug.next_waypoint_index =
        (uint8)(substep_index + segment_cells);
    follower_debug.waypoint_count = action_step_count;
    follower_debug.step_index = global_step_index;
    follower_debug.from_x = from_x;
    follower_debug.from_y = from_y;
    follower_debug.to_x = to_x;
    follower_debug.to_y = to_y;
    follower_debug.target_x_mm = (int16)to_x * SOLVER_GRID_SIZE_MM;
    follower_debug.target_y_mm = (int16)to_y * SOLVER_GRID_SIZE_MM;
    follower_debug.visual_heading_ref_x10 =
        follower_visual_heading_ref_x10;
    follower_debug.physical_heading_x10 =
        (int16)(follower_map_heading_ref_deg * 10.0f + 0.5f);
    follower_debug.nominal_speed_command = (int16)follower_speed_u16();
    follower_debug.segment_cells = segment_cells;
    follower_debug.gear = context != 0 ? context->gear : ACTION_GEAR_STANDARD;
    follower_debug.interaction_locked =
        context != 0 ? context->interaction_locked : 0U;
    follower_debug.strong_reanchor =
        context != 0 ? context->strong_reanchor : 0U;
    follower_debug.strict_position =
        context != 0 ? context->strict_position : 0U;
    follower_debug.cruise_only =
        context != 0 ? context->cruise_only : 0U;
    follower_debug.terminal_node =
        context != 0 ? context->terminal_node : 0U;
    follower_debug.logical_origin_override =
        context != 0 ? context->logical_origin_override : 0U;
    follower_debug.pwm_ramp_ms =
        context != 0 ? context->pwm_ramp_ms : 0U;
    follower_debug.pwm_ramp_max_delta =
        context != 0 ? context->pwm_ramp_max_delta : 0U;
    follower_debug.wall_snap_accept_count =
        follower_wall_snap_accept_count;
    follower_debug.visual_confirm_required =
        context != 0 ? context->vision_stable_frames : 6U;
    follower_debug.dead_reckon_steps = follower_dead_reckon_steps;
    follower_debug.wall_axis_mask = context != 0 ?
        context->wall_axis_mask : follower_target_wall_mask();
    follower_debug.corridor_axis_mask = context != 0 ?
        context->corridor_axis_mask : follower_target_corridor_mask();
    follower_debug.single_wall_correction_sign = context != 0 ?
        context->single_wall_correction_sign : 0;
    follower_debug.fast_finish = fast_finish ? 1U : 0U;
    follower_debug.align_attempts = 0U;
    follower_debug.align_active = 0U;
    follower_align_active = 0U;
    follower_align_attempts = 0U;
    follower_align_max_distance_mm =
        context != 0 && context->align_max_distance_mm > 0U ?
        (float)context->align_max_distance_mm : FOLLOWER_ALIGN_MAX_DIST_MM;
    follower_start_tolerance_mm =
        context != 0 && context->start_tolerance_mm > 0U ?
        (float)context->start_tolerance_mm : FOLLOWER_START_TOLERANCE_MM;
    follower_debug.align_max_distance_mm =
        (uint16)(follower_align_max_distance_mm + 0.5f);
    follower_debug.start_tolerance_mm =
        (uint16)(follower_start_tolerance_mm + 0.5f);
    follower_strong_align_reported = 0U;
    follower_wait_recheck = 0U;
    follower_wait_tick = pit_count;
    follower_current_dead_reckon = 0U;
    follower_fast_finish = fast_finish ? 1U : 0U;
    follower_vision_drain_ms = context != 0 ? context->vision_drain_ms : 1000U;
    follower_vision_stable_frames = context != 0 ?
        context->vision_stable_frames : 6U;
    follower_vision_stable_ms = context != 0 ?
        context->vision_stable_ms : 200U;
    follower_suspect_last_packets = 0U;
    follower_suspect_start_tick = pit_count;
    follower_start_tick = pit_count;
    follower_stop_tick = pit_count;
    follower_try_begin();
    return follower_debug.state != ACTION_FOLLOWER_FAULT;
}

uint8 action_follower_start_pose_reanchor(uint8 action_index,
                                          uint16 global_step_index,
                                          uint8 target_x,
                                          uint8 target_y,
                                          int16 position_x10,
                                          int16 position_y10,
                                          uint16 max_distance_mm,
                                          uint8 strict_position)
{
    float dx;
    float dy;
    float distance;

    if(follower_debug.state == ACTION_FOLLOWER_RUNNING ||
       !follower_mission_heading_valid || target_x >= MAP_MAX_W ||
       target_y >= MAP_MAX_H ||
       max_distance_mm < (uint16)FOLLOWER_ALIGN_MAX_DIST_MM ||
       max_distance_mm > 300U)
        return 0U;

    dx = (float)target_x * SOLVER_GRID_SIZE_MM -
         (float)position_x10 * 20.0f;
    dy = (float)target_y * SOLVER_GRID_SIZE_MM -
         (float)position_y10 * 20.0f;
    distance = sqrtf(dx * dx + dy * dy);
    if(distance < (strict_position ? FOLLOWER_STRICT_PASS_DIST_MM :
                                      FOLLOWER_ALIGN_PASS_DIST_MM) ||
       distance > (float)max_distance_mm)
        return 0U;

    point_test_emergency_stop();
    memset(&follower_debug, 0, sizeof(follower_debug));
    follower_debug.state = ACTION_FOLLOWER_RUNNING;
    follower_debug.fault = ACTION_FOLLOWER_FAULT_NONE;
    follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_VISION;
    follower_debug.action_index = action_index;
    follower_debug.step_index = global_step_index;
    follower_debug.from_x = follower_round_grid_x10(position_x10);
    follower_debug.from_y = follower_round_grid_x10(position_y10);
    follower_debug.to_x = target_x;
    follower_debug.to_y = target_y;
    follower_debug.target_x_mm = (int16)target_x * SOLVER_GRID_SIZE_MM;
    follower_debug.target_y_mm = (int16)target_y * SOLVER_GRID_SIZE_MM;
    follower_debug.car_x10 = position_x10;
    follower_debug.car_y10 = position_y10;
    follower_debug.visual_heading_ref_x10 = follower_visual_heading_ref_x10;
    follower_debug.physical_heading_x10 =
        (int16)(follower_map_heading_ref_deg * 10.0f + 0.5f);
    follower_debug.nominal_speed_command = 100;
    follower_debug.segment_cells = 1U;
    follower_debug.gear = ACTION_GEAR_TRANSITION_REANCHOR;
    follower_debug.strong_reanchor = 1U;
    follower_debug.strict_position = strict_position ? 1U : 0U;
    follower_debug.visual_confirm_required = 6U;

    follower_align_active = 0U;
    follower_align_attempts = 0U;
    follower_align_max_distance_mm = (float)max_distance_mm;
    follower_start_tolerance_mm = FOLLOWER_START_TOLERANCE_MM;
    follower_strong_align_reported = 0U;
    follower_debug.align_max_distance_mm = max_distance_mm;
    follower_debug.start_tolerance_mm =
        (uint16)FOLLOWER_START_TOLERANCE_MM;
    follower_wait_recheck = 0U;
    follower_wait_tick = pit_count;
    follower_current_dead_reckon = 0U;
    follower_fast_finish = 0U;
    follower_vision_drain_ms = 1000U;
    follower_vision_stable_frames = 6U;
    follower_vision_stable_ms = 200U;
    follower_suspect_last_packets = 0U;
    follower_suspect_start_tick = pit_count;
    follower_start_tick = pit_count;
    follower_stop_tick = pit_count;
    follower_cross_reset_dead_reckon();

    if(follower_start_alignment(dx, dy)) return 1U;
    follower_stop(ACTION_FOLLOWER_FAULT,
                  ACTION_FOLLOWER_FAULT_BAD_ACTION);
    return 0U;
}

void action_follower_poll(void)
{
    point_test_snapshot_t point;
    vision_link_snapshot_t pose;

    point_test_poll();
    point_test_get_snapshot(&point);
    if(follower_debug.state != ACTION_FOLLOWER_RUNNING) return;

    follower_update_cross_track(&point);
    point_test_get_snapshot(&point);
    follower_update_debug(&point);
    if(follower_debug.phase == ACTION_FOLLOWER_PHASE_WAIT_ORIGIN)
    {
        follower_try_begin();
        return;
    }

    if(follower_debug.phase == ACTION_FOLLOWER_PHASE_WAIT_VISION)
    {
        vision_link_get_snapshot(&pose);
        if(follower_debug.pose_suspect)
        {
            if(follower_pose_fresh(&pose) &&
               pose.pos_packets != follower_suspect_last_packets)
            {
                follower_suspect_last_packets = pose.pos_packets;
                if(abs((int)pose.car_x_mm - (int)follower_suspect_x10) <=
                       FOLLOWER_POSE_SUSPECT_CLUSTER_X10 &&
                   abs((int)pose.car_y_mm - (int)follower_suspect_y10) <=
                       FOLLOWER_POSE_SUSPECT_CLUSTER_X10)
                {
                    if(follower_debug.pose_suspect_frames < 255U)
                        follower_debug.pose_suspect_frames++;
                }
                else
                {
                    follower_suspect_x10 = pose.car_x_mm;
                    follower_suspect_y10 = pose.car_y_mm;
                    follower_suspect_start_tick = pit_count;
                    follower_debug.pose_suspect_frames = 1U;
                    follower_debug.pose_suspect_grid_x =
                        follower_round_grid_x10(pose.car_x_mm);
                    follower_debug.pose_suspect_grid_y =
                        follower_round_grid_x10(pose.car_y_mm);
                }
            }

            if(follower_debug.pose_suspect_frames >=
                   FOLLOWER_POSE_SUSPECT_FRAMES &&
               (uint32)(pit_count - follower_suspect_start_tick) >=
                   FOLLOWER_POSE_SUSPECT_TICKS)
            {
                uint8 gx = follower_debug.pose_suspect_grid_x;
                uint8 gy = follower_debug.pose_suspect_grid_y;
                follower_debug.pose_suspect = 0U;
                if(gx == follower_debug.to_x && gy == follower_debug.to_y)
                {
                    /* A stable target-cell identity is topologically valid
                       even when the host clamps the coordinate to a wall. */
                    follower_align_attempts = follower_align_max_attempts();
                    follower_accept_or_align_position(follower_suspect_x10,
                                                      follower_suspect_y10);
                }
                else if(!follower_debug.interaction_locked &&
                        gx == follower_debug.from_x &&
                        gy == follower_debug.from_y)
                {
                    follower_wait_recheck = 0U;
                    follower_align_active = 0U;
                    follower_align_attempts = 0U;
                    follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_ORIGIN;
                    follower_try_begin();
                }
                else
                {
                    follower_stop(ACTION_FOLLOWER_REPLAN,
                                  ACTION_FOLLOWER_FAULT_GRID_MISMATCH);
                }
            }
            return;
        }
        if((uint32)(pit_count - follower_wait_tick) >=
               FOLLOWER_ALIGN_RECHECK_TICKS &&
           follower_pose_fresh(&pose) && point.vision_position_stable)
        {
            follower_accept_or_align_position(pose.car_x_mm,
                                              pose.car_y_mm);
        }
        return;
    }

    if(point.state == POINT_TEST_DONE)
    {
        if(point.fast_finish_used)
        {
            follower_debug.fast_finish = 1U;
            follower_align_active = 0U;
            follower_wait_recheck = 0U;
            follower_update_debug(&point);
            follower_stop(ACTION_FOLLOWER_DONE,
                          ACTION_FOLLOWER_FAULT_NONE);
            return;
        }
        if(point.vision_confirm_valid)
        {
            follower_debug.vision_confirm_valid = 1U;
            follower_debug.vision_confirm_x10 = point.vision_confirm_x10;
            follower_debug.vision_confirm_y10 = point.vision_confirm_y10;
            follower_accept_or_align_position(point.vision_confirm_x10,
                                              point.vision_confirm_y10);
            return;
        }

        /* The point layer may time out its fresh-frame collector while the
           latest stable pose is still usable. Recheck that pose before
           accepting an encoder-only finish, otherwise a large open-field
           cross error can be committed as a successful grid step. */
        vision_link_get_snapshot(&pose);
        if(follower_pose_fresh(&pose) && point.vision_position_stable)
        {
            follower_accept_or_align_position(pose.car_x_mm,
                                              pose.car_y_mm);
            return;
        }

        if((point.vision_fallback_used || follower_current_dead_reckon) &&
           !follower_debug.strict_position)
        {
            if(follower_dead_reckon_steps < 255U)
                follower_dead_reckon_steps++;
            follower_debug.visual_fallback_used = 1U;
            follower_debug.dead_reckon_active = 1U;
            follower_debug.dead_reckon_steps = follower_dead_reckon_steps;
            follower_align_active = 0U;
            follower_wait_recheck = 0U;
            follower_update_debug(&point);
            follower_stop(ACTION_FOLLOWER_DONE,
                          ACTION_FOLLOWER_FAULT_NONE);
            return;
        }

        follower_enter_visual_wait();
    }
    else if(point.state == POINT_TEST_FAULT || point.state == POINT_TEST_LOCKED)
    {
        if(point.fault == POINT_FAULT_VISION_NOT_READY ||
           point.fault == POINT_FAULT_VIS_CONFIRM_TIMEOUT ||
           point.fault == POINT_FAULT_GRID_MISMATCH)
        {
            follower_enter_visual_wait();
            return;
        }
        follower_stop(ACTION_FOLLOWER_FAULT,
                      follower_point_fault(point.fault));
    }
}

void action_follower_pause(void)
{
    if(follower_debug.state == ACTION_FOLLOWER_RUNNING)
    {
        point_test_emergency_stop();
        follower_stop(ACTION_FOLLOWER_PAUSED, ACTION_FOLLOWER_FAULT_NONE);
    }
}

uint8 action_follower_resume(void)
{
    return 0U;
}

void action_follower_abort(void)
{
    point_test_emergency_stop();
    action_follower_end_mission_heading();
    follower_debug.speed_command = 0;
    follower_debug.state = ACTION_FOLLOWER_IDLE;
    follower_debug.fault = ACTION_FOLLOWER_FAULT_NONE;
    follower_debug.pose_valid = 0U;
    follower_debug.phase = ACTION_FOLLOWER_PHASE_WAIT_ORIGIN;
    follower_cross_filter_valid = 0U;
    follower_cross_reject_count = 0U;
    follower_dead_reckon_steps = 0U;
    follower_current_dead_reckon = 0U;
    follower_align_active = 0U;
    follower_align_attempts = 0U;
    follower_align_max_distance_mm = FOLLOWER_ALIGN_MAX_DIST_MM;
    follower_wait_recheck = 0U;
    follower_fast_finish = 0U;
    follower_debug.pose_suspect = 0U;
    point_test_set_fast_finish(0U);
}

action_follower_state_t action_follower_state(void)
{
    return follower_debug.state;
}

action_follower_fault_t action_follower_fault(void)
{
    return follower_debug.fault;
}

void action_follower_get_debug(action_follower_debug_t *out)
{
    uint32 end;

    if(out == 0) return;
    *out = follower_debug;
    end = (follower_debug.state == ACTION_FOLLOWER_DONE ||
           follower_debug.state == ACTION_FOLLOWER_REPLAN ||
           follower_debug.state == ACTION_FOLLOWER_FAULT) ?
          follower_stop_tick : pit_count;
    out->elapsed_ms = (uint32)(end - follower_start_tick) * 5U;
}

const char *action_follower_state_name(action_follower_state_t state)
{
    switch(state)
    {
        case ACTION_FOLLOWER_IDLE: return "IDLE";
        case ACTION_FOLLOWER_RUNNING: return "RUNNING";
        case ACTION_FOLLOWER_PAUSED: return "PAUSED_ABORTED";
        case ACTION_FOLLOWER_DONE: return "DONE";
        case ACTION_FOLLOWER_REPLAN: return "POSE_REPLAN";
        case ACTION_FOLLOWER_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char *action_follower_fault_name(action_follower_fault_t fault)
{
    switch(fault)
    {
        case ACTION_FOLLOWER_FAULT_NONE: return "NONE";
        case ACTION_FOLLOWER_FAULT_BAD_ACTION: return "BAD_ACTION";
        case ACTION_FOLLOWER_FAULT_VISION_OFFLINE: return "VISION_OFFLINE";
        case ACTION_FOLLOWER_FAULT_POSE_INVALID: return "POSE_INVALID";
        case ACTION_FOLLOWER_FAULT_POSE_STALE: return "POSE_STALE";
        case ACTION_FOLLOWER_FAULT_START_MISMATCH: return "START_MISMATCH";
        case ACTION_FOLLOWER_FAULT_STALLED: return "STALLED";
        case ACTION_FOLLOWER_FAULT_TIMEOUT: return "TIMEOUT";
        case ACTION_FOLLOWER_FAULT_VIS_CONFIRM_TIMEOUT: return "VIS_CONFIRM_TIMEOUT";
        case ACTION_FOLLOWER_FAULT_GRID_MISMATCH: return "GRID_MISMATCH";
        default: return "UNKNOWN";
    }
}

const char *action_follower_phase_name(action_follower_phase_t phase)
{
    switch(phase)
    {
        case ACTION_FOLLOWER_PHASE_WAIT_ORIGIN: return "WAIT_VIS_ORIGIN";
        case ACTION_FOLLOWER_PHASE_MOVE: return "ENC_IMU_MOVE";
        case ACTION_FOLLOWER_PHASE_SETTLE: return "HOLD_500MS";
        case ACTION_FOLLOWER_PHASE_VIS_DRAIN: return "VIS_DRAIN";
        case ACTION_FOLLOWER_PHASE_VIS_STABLE: return "VIS_STABLE";
        case ACTION_FOLLOWER_PHASE_WAIT_VISION: return "WAIT_VIS_STABLE";
        case ACTION_FOLLOWER_PHASE_ALIGN_MOVE: return "ALIGN_ENC_IMU";
        case ACTION_FOLLOWER_PHASE_ALIGN_RECHECK: return "ALIGN_RECHECK";
        default: return "UNKNOWN";
    }
}

const char *action_follower_gear_name(action_follower_gear_t gear)
{
    switch(gear)
    {
        case ACTION_GEAR_STANDARD: return "STANDARD";
        case ACTION_GEAR_OPEN_FAST: return "OPEN_FAST";
        case ACTION_GEAR_SINGLE_WALL_FAST: return "SINGLE_WALL_FAST";
        case ACTION_GEAR_CORRIDOR_FAST: return "CORRIDOR_FAST";
        case ACTION_GEAR_BOX_NEAR_PRECISE: return "BOX_NEAR_PRECISE";
        case ACTION_GEAR_PUSH_PRECISE: return "PUSH_PRECISE";
        case ACTION_GEAR_TRANSITION_REANCHOR: return "TRANSITION_REANCHOR";
        default: return "UNKNOWN";
    }
}
