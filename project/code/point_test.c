#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "blue.h"
#include "motion_control.h"
#include "vision_link.h"
#include "point_test.h"
#include "app_config.h"

#define POINT_UPDATE_TICKS              (4U)
#define POINT_ROTATE_UPDATE_TICKS       (1U)
#define POINT_SETTLE_TICKS              (100U)
#define POINT_VISION_STALE_TICKS        (60U)
#define POINT_TRANSLATE_TIMEOUT_TICKS   (2000U)
#define POINT_ROTATE_TIMEOUT_TICKS      (4000U)
#define POINT_ROTATE_APPROACH_DEG         (APP_POINT_ROTATE_APPROACH_DEG)
#define POINT_ROTATE_APPROACH_SPEED       (APP_POINT_ROTATE_APPROACH_SPEED)
#define POINT_ROTATE_BRAKE_LEAD_DEG       (APP_POINT_ROTATE_BRAKE_LEAD_DEG)
#define POINT_ROTATE_HOLD_MIN_TICKS       (100U) /* 500 ms */
#define POINT_ROTATE_HOLD_MAX_TICKS       (400U) /* 2 s */
#define POINT_ROTATE_HOLD_STABLE_TICKS     (40U) /* 200 ms */
#define POINT_ROTATE_HOLD_ERROR_DEG         (1.5f)
#define POINT_ROTATE_HOLD_GYRO_DPS          (3.0f)
#define POINT_OVERTRAVEL_MARGIN_MM      (300.0f)
#define POINT_CROSS_LIMIT_MM            (400.0f)
#define POINT_CELL_MM                   ((float)APP_GRID_CELL_MM)
#define POINT_MAX_TRANSLATE_DISTANCE_MM (2720U)
#define POINT_WHEEL_CENTER_RADIUS_MM    (APP_WHEEL_CENTER_RADIUS_MM)
#define POINT_APPROACH_SPEED            (APP_POINT_APPROACH_SPEED)
#define POINT_DECEL_REMAINING_MM         (80.0f)
#define POINT_STARTUP_ASSIST_MS           (APP_POINT_STARTUP_ASSIST_MS)
#define POINT_STALL_GRACE_TICKS           (60U)  /* 300 ms */
#define POINT_STALL_WINDOW_TICKS          (APP_POINT_STALL_WINDOW_MS / 5U)
#define POINT_STALL_MIN_PROGRESS_MM       (APP_POINT_STALL_MIN_PROGRESS_MM)
#define POINT_VIS_CONFIRM_FRAMES         (3U)
#define POINT_VIS_CONFIRM_TIMEOUT_TICKS  (400U)
#define POINT_VIS_CONFIRM_ALONG_MM       (100.0f)
#define POINT_VIS_CONFIRM_CROSS_MM       (100.0f)
#define POINT_FUSION_DRAIN_TICKS         (200U)
#define POINT_FUSION_STABLE_FRAMES       (6U)
#define POINT_FUSION_STABLE_TICKS        (40U)
#define POINT_FUSION_NO_FRAME_TICKS      (100U) /* 500 ms after drain */
#define POINT_FUSION_TIMEOUT_TICKS       (1200U)
#define POINT_FUSION_CLUSTER_X10         (1)
#define POINT_INPUT_STABLE_FRAMES        (6U)
#define POINT_INPUT_STABLE_TICKS         (40U)
#define POINT_INPUT_CLUSTER_X10          (1)
#define POINT_INPUT_THETA_X10            (120)
#define POINT_RAD_TO_DEG                (57.2957795f)
#define POINT_DEG_TO_RAD                (0.0174532925f)

static const float point_direction_deg[8] =
{
    0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f
};

static point_test_snapshot_t point_status;
static int32 point_wheel_origin[3];
static uint32 point_start_tick;
static uint32 point_settle_tick;
static uint32 point_last_update_tick;
static int16 point_vision_origin_x10;
static int16 point_vision_origin_y10;
static int16 point_vision_origin_theta_x10;
static uint32 point_confirm_pos_packets;
static uint8 point_confirm_frame_id;
static int16 point_confirm_anchor_x10;
static int16 point_confirm_anchor_y10;
static int32 point_confirm_sum_x10;
static int32 point_confirm_sum_y10;
static uint32 point_confirm_stable_tick;
static uint32 point_vision_phase_tick;
static uint32 point_input_pos_packets;
static uint8 point_input_frame_id;
static int16 point_input_anchor_x10;
static int16 point_input_anchor_y10;
static int16 point_input_anchor_theta_x10;
static uint32 point_input_stable_tick;
static int16 point_position_anchor_x10;
static int16 point_position_anchor_y10;
static uint32 point_position_stable_tick;
static uint8 point_heading_target_valid;
static float point_heading_target_deg;
static uint8 point_rotation_heading_target_valid;
static float point_rotation_heading_target_deg;
static uint32 point_rotation_hold_stable_tick;
static uint32 point_stall_anchor_tick;
static float point_stall_anchor_progress_mm;
static uint32 point_fusion_drain_ticks = POINT_FUSION_DRAIN_TICKS;
static uint8 point_fusion_stable_frames = POINT_FUSION_STABLE_FRAMES;
static uint32 point_fusion_stable_ticks = POINT_FUSION_STABLE_TICKS;

static float point_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static const app_point_brake_profile_t *point_brake_profile_for_speed(uint16 speed)
{
    const app_point_brake_profile_t *nearest = &app_point_brake_profiles[0];
    uint16 nearest_delta = (uint16)abs((int)speed - (int)nearest->speed);
    uint8 index;

    for(index = 0U;
        index < app_point_brake_profile_count;
        index++)
    {
        uint16 delta;
        if(app_point_brake_profiles[index].speed == speed)
            return &app_point_brake_profiles[index];
        delta = (uint16)abs((int)speed -
                           (int)app_point_brake_profiles[index].speed);
        if(delta < nearest_delta)
        {
            nearest = &app_point_brake_profiles[index];
            nearest_delta = delta;
        }
    }
    return nearest;
}

static float point_brake_lead_for_direction(uint8 direction_index,
                                             uint16 speed)
{
    const app_point_brake_profile_t *profile =
        point_brake_profile_for_speed(speed);

    switch(direction_index & 7U)
    {
        case 0U: return (float)profile->forward_mm;
        case 2U: return (float)profile->right_mm;
        case 4U: return (float)profile->back_mm;
        case 6U: return (float)profile->left_mm;
        default: return (float)profile->forward_mm;
    }
}

static int32 point_angle_abs_delta_x10(int16 a, int16 b)
{
    int32 delta = (int32)a - (int32)b;

    while(delta > 1800) delta -= 3600;
    while(delta < -1800) delta += 3600;
    return delta < 0 ? -delta : delta;
}

static void point_update_input_stability(const vision_link_snapshot_t *vision)
{
    uint32 stable_ticks;

    if(!point_status.vision_live)
    {
        point_status.vision_input_stable = 0U;
        point_status.vision_input_stable_frames = 0U;
        point_status.vision_input_stable_ms = 0U;
        point_status.vision_position_stable = 0U;
        point_status.vision_position_stable_frames = 0U;
        point_status.vision_position_stable_ms = 0U;
        point_input_pos_packets = vision->pos_packets;
        point_input_frame_id = vision->frame_id;
        return;
    }

    if(vision->pos_packets != point_input_pos_packets)
    {
        point_input_pos_packets = vision->pos_packets;
        if(vision->frame_id != point_input_frame_id)
        {
            point_input_frame_id = vision->frame_id;

            if(point_status.vision_position_stable_frames == 0U ||
               (abs((int)vision->car_x_mm -
                    (int)point_position_anchor_x10) <=
                    POINT_INPUT_CLUSTER_X10 &&
                abs((int)vision->car_y_mm -
                    (int)point_position_anchor_y10) <=
                    POINT_INPUT_CLUSTER_X10))
            {
                if(point_status.vision_position_stable_frames == 0U)
                {
                    point_position_anchor_x10 = vision->car_x_mm;
                    point_position_anchor_y10 = vision->car_y_mm;
                    point_position_stable_tick = pit_count;
                }
                if(point_status.vision_position_stable_frames <
                   POINT_INPUT_STABLE_FRAMES)
                {
                    point_status.vision_position_stable_frames++;
                }
            }
            else
            {
                point_position_anchor_x10 = vision->car_x_mm;
                point_position_anchor_y10 = vision->car_y_mm;
                point_position_stable_tick = pit_count;
                point_status.vision_position_stable_frames = 1U;
            }

            if(point_status.vision_input_stable_frames == 0U ||
               (abs((int)vision->car_x_mm - (int)point_input_anchor_x10) <=
                    POINT_INPUT_CLUSTER_X10 &&
                abs((int)vision->car_y_mm - (int)point_input_anchor_y10) <=
                    POINT_INPUT_CLUSTER_X10 &&
                point_angle_abs_delta_x10(vision->car_theta_x10,
                                           point_input_anchor_theta_x10) <=
                    POINT_INPUT_THETA_X10))
            {
                if(point_status.vision_input_stable_frames == 0U)
                {
                    point_input_anchor_x10 = vision->car_x_mm;
                    point_input_anchor_y10 = vision->car_y_mm;
                    point_input_anchor_theta_x10 = vision->car_theta_x10;
                    point_input_stable_tick = pit_count;
                }
                if(point_status.vision_input_stable_frames <
                   POINT_INPUT_STABLE_FRAMES)
                {
                    point_status.vision_input_stable_frames++;
                }
            }
            else
            {
                point_input_anchor_x10 = vision->car_x_mm;
                point_input_anchor_y10 = vision->car_y_mm;
                point_input_anchor_theta_x10 = vision->car_theta_x10;
                point_input_stable_tick = pit_count;
                point_status.vision_input_stable_frames = 1U;
            }
        }
    }

    point_status.vision_input_frame_id = point_input_frame_id;
    stable_ticks = (uint32)(pit_count - point_input_stable_tick);
    point_status.vision_input_stable_ms = stable_ticks * 5U;
    point_status.vision_input_stable =
        (point_status.vision_input_stable_frames >=
             POINT_INPUT_STABLE_FRAMES &&
         stable_ticks >= POINT_INPUT_STABLE_TICKS) ? 1U : 0U;

    stable_ticks = (uint32)(pit_count - point_position_stable_tick);
    point_status.vision_position_stable_ms = stable_ticks * 5U;
    point_status.vision_position_stable =
        (point_status.vision_position_stable_frames >=
             POINT_INPUT_STABLE_FRAMES &&
         stable_ticks >= POINT_INPUT_STABLE_TICKS) ? 1U : 0U;
}

static uint8 point_is_busy(void)
{
    return (point_status.state == POINT_TEST_RUNNING ||
            point_status.state == POINT_TEST_SETTLING ||
            point_status.state == POINT_TEST_VIS_DRAIN ||
            point_status.state == POINT_TEST_VIS_STABLE) ? 1U : 0U;
}

static void point_set_state(point_test_state_t state)
{
    if(point_status.state != state)
    {
        point_status.state = state;
        point_status.event_counter++;
    }
}

static void point_hard_stop(point_test_fault_t fault, point_test_state_t state)
{
    motion_startup_assist_cancel();
    motion_emergency_stop();
    motion_set_yaw_hold_enable(0U);
    device_init_flag = 1;
    point_status.commanded_speed = 0U;
    point_status.fault = fault;
    point_set_state(state);
}

static void point_update_measurements(void)
{
    vision_link_snapshot_t vision;
    motion_startup_assist_status_t assist;
    int32 wheel_total[3];
    float yaw0_rad;
    float cos_yaw0;
    float sin_yaw0;
    float direction_rad;
    float cos_direction;
    float sin_direction;
    float vision_heading_rad;
    float vision_dx_mm;
    float vision_dy_mm;
    float encoder_world_x;
    float encoder_world_y;

    motion_get_wheel_total_counts(wheel_total);
    for(uint8 i = 0U; i < 3U; i++)
    {
        point_status.wheel_count[i] = wheel_total[i] - point_wheel_origin[i];
    }
    point_status.encoder_rotate_count =
        (point_status.wheel_count[0] + point_status.wheel_count[1] +
         point_status.wheel_count[2]) / 3;
    point_status.encoder_yaw_deg =
        (float)point_status.encoder_rotate_count * point_status.mm_per_count /
        point_status.wheel_center_radius_mm * POINT_RAD_TO_DEG;

    point_status.imu_relative_deg = imu963ra_yaw_angle - point_status.imu_origin_deg;
    encoder_world_x = odom_world_x_mm;
    encoder_world_y = odom_world_y_mm;
    yaw0_rad = point_status.imu_origin_deg * POINT_DEG_TO_RAD;
    cos_yaw0 = cosf(yaw0_rad);
    sin_yaw0 = sinf(yaw0_rad);
    point_status.encoder_forward_mm =
        encoder_world_x * cos_yaw0 + encoder_world_y * sin_yaw0;
    point_status.encoder_right_mm =
        -encoder_world_x * sin_yaw0 + encoder_world_y * cos_yaw0;

    direction_rad = point_status.direction_deg * POINT_DEG_TO_RAD;
    cos_direction = cosf(direction_rad);
    sin_direction = sinf(direction_rad);
    point_status.encoder_along_mm =
        point_status.encoder_forward_mm * cos_direction +
        point_status.encoder_right_mm * sin_direction;
    point_status.encoder_cross_mm =
        -point_status.encoder_forward_mm * sin_direction +
        point_status.encoder_right_mm * cos_direction;

    vision_link_get_snapshot(&vision);
    point_status.vision_x10 = vision.car_x_mm;
    point_status.vision_y10 = vision.car_y_mm;
    point_status.vision_theta_x10 = vision.car_theta_x10;
    point_status.vision_age_ms = (vision.last_packet_tick == 0U) ? 99999UL :
        (uint32)(pit_count - vision.last_packet_tick) * 5UL;
    point_status.vision_live = (vision_link_is_online() && vision.pose_valid &&
        vision.last_packet_tick != 0U &&
        (uint32)(pit_count - vision.last_packet_tick) <= POINT_VISION_STALE_TICKS) ? 1U : 0U;
    point_update_input_stability(&vision);

    if(point_status.vision_origin_valid && point_status.vision_live)
    {
        vision_dx_mm = (float)(vision.car_x_mm - point_vision_origin_x10) * 20.0f;
        vision_dy_mm = (float)(vision.car_y_mm - point_vision_origin_y10) * 20.0f;
        vision_heading_rad = (float)point_vision_origin_theta_x10 * 0.1f *
                             POINT_DEG_TO_RAD;
        point_status.vision_forward_mm =
            vision_dx_mm * sinf(vision_heading_rad) -
            vision_dy_mm * cosf(vision_heading_rad);
        point_status.vision_right_mm =
            vision_dx_mm * cosf(vision_heading_rad) +
            vision_dy_mm * sinf(vision_heading_rad);
        point_status.vision_along_mm =
            point_status.vision_forward_mm * cos_direction +
            point_status.vision_right_mm * sin_direction;
        point_status.vision_cross_mm =
            -point_status.vision_forward_mm * sin_direction +
            point_status.vision_right_mm * cos_direction;
    }

    motion_startup_assist_get_status(&assist);
    point_status.startup_assist_active = assist.active;
    point_status.startup_assist_applied_mask = assist.applied_mask;
    point_status.startup_assist_remaining_ms = assist.remaining_ms;
}

static void point_update_progress(void)
{
    float abs_yaw;
    float abs_cross;

    if(point_status.kind == POINT_TEST_KIND_ROTATE)
    {
        if(point_status.rotate_stop == POINT_ROTATE_STOP_IMU)
        {
            point_status.active_progress = point_status.rotate_clockwise ?
                point_status.imu_relative_deg : -point_status.imu_relative_deg;
            point_status.remaining = point_status.rotate_clockwise ?
                point_status.rotation_target_heading_deg - imu963ra_yaw_angle :
                imu963ra_yaw_angle - point_status.rotation_target_heading_deg;
        }
        else
        {
            point_status.active_progress = point_absf(point_status.encoder_yaw_deg);
            point_status.remaining =
                (float)point_status.target_rotation_deg - point_status.active_progress;
        }
        point_status.rotation_target_error_deg =
            point_status.rotation_target_heading_deg - imu963ra_yaw_angle;
        point_status.rotation_overshoot_deg =
            point_status.active_progress - (float)point_status.target_rotation_deg;
        abs_cross = 0.0f;
    }
    else
    {
        point_status.active_progress =
            (point_status.sensor_mode == POINT_SENSOR_VISION_IMU) ?
            point_status.vision_along_mm : point_status.encoder_along_mm;
        point_status.remaining =
            (float)point_status.target_distance_mm - point_status.active_progress;
        abs_cross = (point_status.sensor_mode == POINT_SENSOR_VISION_IMU) ?
            point_absf(point_status.vision_cross_mm) :
            point_absf(point_status.encoder_cross_mm);
    }

    abs_yaw = point_absf(point_status.imu_relative_deg);
    if(abs_yaw > point_status.max_abs_yaw_deg)
    {
        point_status.max_abs_yaw_deg = abs_yaw;
    }
    if(abs_cross > point_status.max_abs_cross_mm)
    {
        point_status.max_abs_cross_mm = abs_cross;
    }
}

static void point_begin_settle(void)
{
    vision_link_snapshot_t vision;

    point_status.commanded_speed = 0U;
    motion_startup_assist_cancel();
    if(point_status.kind == POINT_TEST_KIND_ROTATE)
    {
        point_status.rotation_stop_heading_deg = imu963ra_yaw_angle;
        point_status.rotation_overshoot_deg = point_status.active_progress -
            (float)point_status.target_rotation_deg;
        point_rotation_hold_stable_tick = 0U;
        motion_stop_manual_rotation_at(point_status.rotation_target_heading_deg);
    }
    else
    {
        motion_heading_lock_stop();
        // Hold the measured stop position, not the unreachable position target
        // accumulated while the wheels were below their commanded speed.
        motion_heading_lock_rebase_position();
    }
    point_settle_tick = pit_count;
    point_status.settle_remaining_ms = POINT_SETTLE_TICKS * 5U;
    point_status.vision_confirm_frames = 0U;
    point_status.vision_confirm_required = POINT_VIS_CONFIRM_FRAMES;
    point_status.vision_confirm_bad_frames = 0U;
    point_status.vision_fallback_used = 0U;
    point_status.fast_finish_used = 0U;
    point_status.vision_confirm_valid = 0U;
    point_status.vision_confirm_x10 = 0;
    point_status.vision_confirm_y10 = 0;
    point_status.vision_confirm_age_ms = 0U;
    point_status.vision_drain_remaining_ms = 0U;
    point_status.vision_target_error_along_mm = 0.0f;
    point_status.vision_target_error_cross_mm = 0.0f;
    point_status.vision_encoder_delta_along_mm = 0.0f;
    point_status.vision_encoder_delta_cross_mm = 0.0f;
    vision_link_get_snapshot(&vision);
    point_confirm_pos_packets = vision.pos_packets;
    point_confirm_frame_id = vision.frame_id;
    point_confirm_sum_x10 = 0;
    point_confirm_sum_y10 = 0;
    point_status.vision_confirm_frame_id = vision.frame_id;
    point_status.vision_confirm_pos_packets = vision.pos_packets;
    point_set_state(POINT_TEST_SETTLING);
}

void point_test_init(void)
{
    memset(&point_status, 0, sizeof(point_status));
    point_fusion_drain_ticks = POINT_FUSION_DRAIN_TICKS;
    point_fusion_stable_frames = POINT_FUSION_STABLE_FRAMES;
    point_fusion_stable_ticks = POINT_FUSION_STABLE_TICKS;
    point_status.state = POINT_TEST_LOCKED;
    point_status.kind = POINT_TEST_KIND_NONE;
    point_status.fault = POINT_FAULT_NONE;
    point_status.sensor_mode = POINT_SENSOR_ENCODER_IMU;
    point_status.rotate_stop = POINT_ROTATE_STOP_IMU;
    point_status.direction_index = 0U;
    point_status.direction_deg = 0.0f;
    point_status.command_direction_deg = 0.0f;
    point_status.cell_count = 1U;
    point_status.speed = 100U;
    point_status.commanded_speed = 0U;
    point_status.brake_lead_mm = (uint16)
        point_brake_lead_for_direction(0U, point_status.speed);
    point_status.startup_assist_enabled = 1U;
    point_status.stall_watch_ms = 0U;
    point_status.stall_window_progress_mm = 0.0f;
    point_status.target_distance_mm = 200U;
    point_status.target_rotation_deg = 90U;
    point_status.vision_confirm_required = POINT_VIS_CONFIRM_FRAMES;
    point_status.rotate_clockwise = 1U;
    point_status.rotate_approach_active = 0U;
    point_status.mm_per_count = motion_get_mm_per_count();
    point_status.wheel_center_radius_mm = POINT_WHEEL_CENTER_RADIUS_MM;
    point_heading_target_valid = 0U;
    point_heading_target_deg = 0.0f;
    point_rotation_heading_target_valid = 0U;
    point_rotation_heading_target_deg = 0.0f;
    point_rotation_hold_stable_tick = 0U;
    point_last_update_tick = pit_count;
    motion_get_wheel_total_counts(point_wheel_origin);
    point_hard_stop(POINT_FAULT_NONE, POINT_TEST_LOCKED);
}

uint8 point_test_capture_origin(void)
{
    vision_link_snapshot_t vision;

    if(point_is_busy()) return 0U;
    point_update_measurements();
    if(!imu963ra_ready)
    {
        point_status.fault = POINT_FAULT_IMU_NOT_READY;
        point_set_state(POINT_TEST_FAULT);
        return 0U;
    }
    if(point_status.sensor_mode == POINT_SENSOR_FUSION_LOCKED &&
       (!point_status.vision_live || !point_status.vision_position_stable))
    {
        point_status.origin_valid = 0U;
        point_status.vision_origin_valid = 0U;
        point_hard_stop(POINT_FAULT_VISION_NOT_READY, POINT_TEST_LOCKED);
        return 0U;
    }

    point_hard_stop(POINT_FAULT_NONE, POINT_TEST_LOCKED);
    motion_get_wheel_total_counts(point_wheel_origin);
    odometry_reset();
    point_status.imu_origin_deg = imu963ra_yaw_angle;
    point_status.imu_relative_deg = 0.0f;
    point_status.encoder_yaw_deg = 0.0f;
    point_status.encoder_forward_mm = 0.0f;
    point_status.encoder_right_mm = 0.0f;
    point_status.encoder_along_mm = 0.0f;
    point_status.encoder_cross_mm = 0.0f;
    point_status.vision_forward_mm = 0.0f;
    point_status.vision_right_mm = 0.0f;
    point_status.vision_along_mm = 0.0f;
    point_status.vision_cross_mm = 0.0f;
    point_status.active_progress = 0.0f;
    point_status.remaining = (float)point_status.target_distance_mm;
    point_status.max_abs_yaw_deg = 0.0f;
    point_status.max_abs_cross_mm = 0.0f;
    point_status.elapsed_ms = 0U;
    point_status.settle_remaining_ms = 0U;
    point_status.vision_confirm_frames = 0U;
    point_status.vision_confirm_required = POINT_VIS_CONFIRM_FRAMES;
    point_status.vision_confirm_bad_frames = 0U;
    point_status.vision_fallback_used = 0U;
    point_status.vision_confirm_valid = 0U;
    point_status.vision_confirm_x10 = 0;
    point_status.vision_confirm_y10 = 0;
    point_status.vision_confirm_frame_id = 0U;
    point_status.vision_confirm_age_ms = 0U;
    point_status.vision_drain_remaining_ms = 0U;
    point_status.vision_target_error_along_mm = 0.0f;
    point_status.vision_target_error_cross_mm = 0.0f;
    point_status.vision_encoder_delta_along_mm = 0.0f;
    point_status.vision_encoder_delta_cross_mm = 0.0f;
    point_status.commanded_speed = 0U;
    point_status.stall_watch_ms = 0U;
    point_status.stall_window_progress_mm = 0.0f;
    point_status.kind = POINT_TEST_KIND_NONE;
    point_status.fault = POINT_FAULT_NONE;
    point_status.origin_valid = 1U;

    vision_link_get_snapshot(&vision);
    point_status.vision_origin_valid = (vision_link_is_online() &&
        vision.pose_valid &&
        vision.last_packet_tick != 0U &&
        (uint32)(pit_count - vision.last_packet_tick) <= POINT_VISION_STALE_TICKS) ? 1U : 0U;
    if(point_status.vision_origin_valid)
    {
        point_vision_origin_x10 =
            point_status.sensor_mode == POINT_SENSOR_FUSION_LOCKED ?
                point_position_anchor_x10 : vision.car_x_mm;
        point_vision_origin_y10 =
            point_status.sensor_mode == POINT_SENSOR_FUSION_LOCKED ?
                point_position_anchor_y10 : vision.car_y_mm;
        point_vision_origin_theta_x10 = vision.car_theta_x10;
    }
    point_update_measurements();
    point_set_state(POINT_TEST_READY);
    return 1U;
}

void point_test_clear_vision_origin(void)
{
    if(point_is_busy()) return;
    point_status.vision_origin_valid = 0U;
    point_status.vision_forward_mm = 0.0f;
    point_status.vision_right_mm = 0.0f;
    point_status.vision_along_mm = 0.0f;
    point_status.vision_cross_mm = 0.0f;
}

uint8 point_test_start_translation(void)
{
    if(point_status.state != POINT_TEST_READY || !point_status.origin_valid)
    {
        point_status.fault = POINT_FAULT_NO_ORIGIN;
        return 0U;
    }
    if((point_status.sensor_mode == POINT_SENSOR_VISION_IMU ||
        point_status.sensor_mode == POINT_SENSOR_FUSION_LOCKED) &&
       (!point_status.vision_origin_valid || !point_status.vision_live))
    {
        point_status.fault = POINT_FAULT_VISION_NOT_READY;
        return 0U;
    }
    if(point_status.sensor_mode == POINT_SENSOR_FUSION_LOCKED &&
       !point_status.vision_position_stable)
    {
        point_status.fault = POINT_FAULT_VISION_NOT_READY;
        return 0U;
    }

    point_status.kind = POINT_TEST_KIND_TRANSLATE;
    point_status.fault = POINT_FAULT_NONE;
    point_status.active_progress = 0.0f;
    point_status.remaining = (float)point_status.target_distance_mm;
    point_status.max_abs_yaw_deg = 0.0f;
    point_status.max_abs_cross_mm = 0.0f;
    point_status.commanded_speed = point_status.speed;
    point_status.command_direction_deg = point_status.direction_deg;
    point_status.brake_lead_mm = (uint16)
        point_brake_lead_for_direction(point_status.direction_index,
                                       point_status.speed);
    point_start_tick = pit_count;
    point_stall_anchor_tick = pit_count;
    point_stall_anchor_progress_mm = 0.0f;
    point_status.stall_watch_ms = 0U;
    point_status.stall_window_progress_mm = 0.0f;
    point_last_update_tick = pit_count - POINT_UPDATE_TICKS;
    if(point_heading_target_valid)
        motion_heading_lock_begin_at(point_heading_target_deg);
    else
        motion_heading_lock_begin();
    motion_heading_lock_update(point_status.command_direction_deg,
                               (float)point_status.speed);
    if(point_status.startup_assist_enabled)
        motion_startup_assist_begin(POINT_STARTUP_ASSIST_MS);
    else
        motion_startup_assist_cancel();
    if(point_status.sensor_mode == POINT_SENSOR_ENCODER_OPEN_YAW)
    {
        motion_set_yaw_hold_enable(0U);
    }
    device_init_flag = 0;
    point_set_state(POINT_TEST_RUNNING);
    return 1U;
}

uint8 point_test_start_rotation(void)
{
    float signed_speed;

    if(point_status.state != POINT_TEST_READY || !point_status.origin_valid)
    {
        point_status.fault = POINT_FAULT_NO_ORIGIN;
        return 0U;
    }
    if(point_status.rotate_stop == POINT_ROTATE_STOP_IMU && !imu963ra_ready)
    {
        point_status.fault = POINT_FAULT_IMU_NOT_READY;
        return 0U;
    }

    point_status.kind = POINT_TEST_KIND_ROTATE;
    point_status.fault = POINT_FAULT_NONE;
    motion_startup_assist_cancel();
    point_status.active_progress = 0.0f;
    point_status.remaining = (float)point_status.target_rotation_deg;
    point_status.max_abs_yaw_deg = 0.0f;
    point_status.max_abs_cross_mm = 0.0f;
    point_status.commanded_speed = point_status.speed;
    point_status.rotate_approach_active = 0U;
    point_status.rotation_target_heading_deg =
        point_rotation_heading_target_valid ? point_rotation_heading_target_deg :
        (point_status.imu_origin_deg + (point_status.rotate_clockwise ?
            (float)point_status.target_rotation_deg :
            -(float)point_status.target_rotation_deg));
    point_status.rotation_target_error_deg =
        point_status.rotation_target_heading_deg - imu963ra_yaw_angle;
    point_status.rotation_stop_heading_deg = imu963ra_yaw_angle;
    point_status.rotation_overshoot_deg = 0.0f;
    point_status.rotation_hold_stable_ms = 0U;
    point_start_tick = pit_count;
    point_last_update_tick = pit_count - POINT_UPDATE_TICKS;
    signed_speed = point_status.rotate_clockwise ?
        (float)point_status.speed : -(float)point_status.speed;
    motion_set_manual_rotation(signed_speed);
    device_init_flag = 0;
    point_set_state(POINT_TEST_RUNNING);
    return 1U;
}

void point_test_poll(void)
{
    vision_link_snapshot_t vision;
    uint32 elapsed_ticks;
    uint32 timeout_ticks;
    uint32 new_pos_packets;
    uint32 update_ticks;

    update_ticks = (point_status.state == POINT_TEST_RUNNING &&
                    point_status.kind == POINT_TEST_KIND_ROTATE) ?
        POINT_ROTATE_UPDATE_TICKS : POINT_UPDATE_TICKS;
    if((uint32)(pit_count - point_last_update_tick) < update_ticks) return;
    point_last_update_tick = pit_count;
    point_update_measurements();
    point_update_progress();

    if(point_status.state == POINT_TEST_RUNNING)
    {
        elapsed_ticks = (uint32)(pit_count - point_start_tick);
        point_status.elapsed_ms = elapsed_ticks * 5U;
        timeout_ticks = (point_status.kind == POINT_TEST_KIND_ROTATE) ?
            POINT_ROTATE_TIMEOUT_TICKS :
            POINT_TRANSLATE_TIMEOUT_TICKS * (uint32)point_status.cell_count;

        if(point_status.kind == POINT_TEST_KIND_TRANSLATE &&
           point_status.sensor_mode == POINT_SENSOR_VISION_IMU &&
           !point_status.vision_live)
        {
            point_hard_stop(POINT_FAULT_VISION_NOT_READY, POINT_TEST_FAULT);
            return;
        }
        if(elapsed_ticks > timeout_ticks)
        {
            point_hard_stop(POINT_FAULT_TIMEOUT, POINT_TEST_FAULT);
            return;
        }
        if(point_status.kind == POINT_TEST_KIND_TRANSLATE)
        {
            float stall_delta;
            uint32 stall_ticks;

            if(elapsed_ticks < POINT_STALL_GRACE_TICKS)
            {
                point_stall_anchor_tick = pit_count;
                point_stall_anchor_progress_mm = point_status.active_progress;
                point_status.stall_watch_ms = 0U;
                point_status.stall_window_progress_mm = 0.0f;
            }
            else
            {
                stall_delta = point_absf(point_status.active_progress -
                                         point_stall_anchor_progress_mm);
                if(stall_delta >= POINT_STALL_MIN_PROGRESS_MM)
                {
                    point_stall_anchor_tick = pit_count;
                    point_stall_anchor_progress_mm = point_status.active_progress;
                    point_status.stall_watch_ms = 0U;
                    point_status.stall_window_progress_mm = stall_delta;
                }
                else
                {
                    stall_ticks = (uint32)(pit_count - point_stall_anchor_tick);
                    point_status.stall_watch_ms = stall_ticks * 5U;
                    point_status.stall_window_progress_mm = stall_delta;
                    if(stall_ticks >= POINT_STALL_WINDOW_TICKS)
                    {
                        point_hard_stop(POINT_FAULT_STALL, POINT_TEST_FAULT);
                        return;
                    }
                }
            }
        }
        if(point_status.kind == POINT_TEST_KIND_TRANSLATE &&
           (point_status.encoder_along_mm >
                (float)point_status.target_distance_mm + POINT_OVERTRAVEL_MARGIN_MM ||
            point_absf(point_status.encoder_cross_mm) > POINT_CROSS_LIMIT_MM))
        {
            point_hard_stop(POINT_FAULT_OVERTRAVEL, POINT_TEST_FAULT);
            return;
        }

        if(point_status.kind == POINT_TEST_KIND_TRANSLATE)
        {
            uint16 commanded_speed = point_status.speed;

            if(point_status.remaining <= POINT_DECEL_REMAINING_MM &&
               commanded_speed > POINT_APPROACH_SPEED)
            {
                commanded_speed = POINT_APPROACH_SPEED;
            }
            if(point_status.commanded_speed != commanded_speed)
            {
                point_status.commanded_speed = commanded_speed;
                point_status.event_counter++;
            }
            motion_heading_lock_update(point_status.command_direction_deg,
                                       (float)commanded_speed);
        }
        else if(point_status.kind == POINT_TEST_KIND_ROTATE &&
                point_status.remaining <= POINT_ROTATE_APPROACH_DEG &&
                point_status.remaining > 0.0f &&
                point_status.commanded_speed > POINT_ROTATE_APPROACH_SPEED)
        {
            float signed_speed;

            point_status.commanded_speed = POINT_ROTATE_APPROACH_SPEED;
            point_status.rotate_approach_active = 1U;
            point_status.event_counter++;
            signed_speed = point_status.rotate_clockwise ?
                (float)POINT_ROTATE_APPROACH_SPEED :
                -(float)POINT_ROTATE_APPROACH_SPEED;
            motion_set_manual_rotation(signed_speed);
        }

        if((point_status.kind == POINT_TEST_KIND_TRANSLATE &&
            point_status.remaining <= (float)point_status.brake_lead_mm) ||
        (point_status.kind == POINT_TEST_KIND_ROTATE &&
            point_status.remaining <= POINT_ROTATE_BRAKE_LEAD_DEG))
        {
            point_begin_settle();
        }
    }
    else if(point_status.state == POINT_TEST_SETTLING)
    {
        elapsed_ticks = (uint32)(pit_count - point_settle_tick);
        point_status.elapsed_ms = (uint32)(pit_count - point_start_tick) * 5U;
        point_status.vision_confirm_age_ms = elapsed_ticks * 5U;

        if(point_status.kind == POINT_TEST_KIND_ROTATE)
        {
            uint8 hold_stable =
                (point_absf(point_status.rotation_target_error_deg) <=
                    POINT_ROTATE_HOLD_ERROR_DEG &&
                 point_absf(imu963ra_gyro_z_dps) <=
                    POINT_ROTATE_HOLD_GYRO_DPS) ? 1U : 0U;

            if(hold_stable)
            {
                if(point_rotation_hold_stable_tick == 0U)
                    point_rotation_hold_stable_tick = pit_count;
                point_status.rotation_hold_stable_ms =
                    (uint32)(pit_count - point_rotation_hold_stable_tick) * 5U;
            }
            else
            {
                point_rotation_hold_stable_tick = 0U;
                point_status.rotation_hold_stable_ms = 0U;
            }
            point_status.settle_remaining_ms =
                elapsed_ticks >= POINT_ROTATE_HOLD_MAX_TICKS ? 0U :
                (POINT_ROTATE_HOLD_MAX_TICKS - elapsed_ticks) * 5U;
            if((elapsed_ticks >= POINT_ROTATE_HOLD_MIN_TICKS &&
                point_status.rotation_hold_stable_ms >=
                    POINT_ROTATE_HOLD_STABLE_TICKS * 5U) ||
               elapsed_ticks >= POINT_ROTATE_HOLD_MAX_TICKS)
            {
                point_hard_stop(POINT_FAULT_NONE, POINT_TEST_DONE);
            }
            return;
        }

        if(point_status.kind == POINT_TEST_KIND_TRANSLATE &&
           point_status.sensor_mode == POINT_SENSOR_VISION_IMU)
        {
            if(!point_status.vision_live)
            {
                point_hard_stop(POINT_FAULT_VISION_NOT_READY,
                                POINT_TEST_FAULT);
                return;
            }

            vision_link_get_snapshot(&vision);
            if(vision.pos_packets != point_confirm_pos_packets)
            {
                new_pos_packets = vision.pos_packets - point_confirm_pos_packets;
                point_confirm_pos_packets = vision.pos_packets;
                point_status.vision_confirm_pos_packets = vision.pos_packets;
                if(new_pos_packets > POINT_VIS_CONFIRM_FRAMES)
                    new_pos_packets = POINT_VIS_CONFIRM_FRAMES;

                if(point_absf(point_status.remaining) <=
                       POINT_VIS_CONFIRM_ALONG_MM &&
                   point_absf(point_status.vision_cross_mm) <=
                       POINT_VIS_CONFIRM_CROSS_MM)
                {
                    point_status.vision_confirm_bad_frames = 0U;
                    while(new_pos_packets-- > 0U &&
                          point_status.vision_confirm_frames <
                              POINT_VIS_CONFIRM_FRAMES)
                    {
                        point_status.vision_confirm_frames++;
                    }
                }
                else
                {
                    point_status.vision_confirm_frames = 0U;
                    while(new_pos_packets-- > 0U &&
                          point_status.vision_confirm_bad_frames <
                              POINT_VIS_CONFIRM_FRAMES)
                    {
                        point_status.vision_confirm_bad_frames++;
                    }
                }
            }

            if(elapsed_ticks < POINT_SETTLE_TICKS)
            {
                point_status.settle_remaining_ms =
                    (POINT_SETTLE_TICKS - elapsed_ticks) * 5U;
                return;
            }
            point_status.settle_remaining_ms = 0U;
            if(point_status.vision_confirm_bad_frames >=
               POINT_VIS_CONFIRM_FRAMES)
            {
                point_hard_stop(POINT_FAULT_GRID_MISMATCH,
                                POINT_TEST_FAULT);
                return;
            }
            if(point_status.vision_confirm_frames >=
               POINT_VIS_CONFIRM_FRAMES)
            {
                point_hard_stop(POINT_FAULT_NONE, POINT_TEST_DONE);
                return;
            }
            if(elapsed_ticks >= POINT_VIS_CONFIRM_TIMEOUT_TICKS)
            {
                point_hard_stop(POINT_FAULT_VIS_CONFIRM_TIMEOUT,
                                POINT_TEST_FAULT);
            }
            return;
        }

        if(elapsed_ticks >= POINT_SETTLE_TICKS)
        {
            point_status.settle_remaining_ms = 0U;
            if(point_status.kind == POINT_TEST_KIND_TRANSLATE &&
               point_status.sensor_mode == POINT_SENSOR_FUSION_LOCKED)
            {
                if(point_status.fast_finish_enabled)
                {
                    point_status.fast_finish_used = 1U;
                    point_hard_stop(POINT_FAULT_NONE, POINT_TEST_DONE);
                    return;
                }
                vision_link_get_snapshot(&vision);
                point_confirm_pos_packets = vision.pos_packets;
                point_confirm_frame_id = vision.frame_id;
                point_status.vision_confirm_pos_packets = vision.pos_packets;
                point_status.vision_confirm_frame_id = vision.frame_id;
                point_status.vision_confirm_frames = 0U;
                point_status.vision_confirm_required =
                    point_fusion_stable_frames;
                point_status.vision_confirm_bad_frames = 0U;
                point_status.vision_confirm_valid = 0U;
                point_confirm_sum_x10 = 0;
                point_confirm_sum_y10 = 0;
                point_vision_phase_tick = pit_count;
                point_status.vision_drain_remaining_ms =
                    point_fusion_drain_ticks * 5U;
                point_set_state(POINT_TEST_VIS_DRAIN);
            }
            else
            {
                point_hard_stop(POINT_FAULT_NONE, POINT_TEST_DONE);
            }
        }
        else
        {
            point_status.settle_remaining_ms =
                (POINT_SETTLE_TICKS - elapsed_ticks) * 5U;
        }
    }
    else if(point_status.state == POINT_TEST_VIS_DRAIN)
    {
        elapsed_ticks = (uint32)(pit_count - point_settle_tick);
        point_status.elapsed_ms = (uint32)(pit_count - point_start_tick) * 5U;
        point_status.vision_confirm_age_ms = elapsed_ticks * 5U;
        vision_link_get_snapshot(&vision);
        point_confirm_pos_packets = vision.pos_packets;
        point_confirm_frame_id = vision.frame_id;
        point_status.vision_confirm_pos_packets = vision.pos_packets;
        point_status.vision_confirm_frame_id = vision.frame_id;

        if(elapsed_ticks >= POINT_FUSION_TIMEOUT_TICKS)
        {
            point_hard_stop(POINT_FAULT_VIS_CONFIRM_TIMEOUT,
                            POINT_TEST_FAULT);
            return;
        }
        elapsed_ticks = (uint32)(pit_count - point_vision_phase_tick);
        if(elapsed_ticks < point_fusion_drain_ticks)
        {
            point_status.vision_drain_remaining_ms =
                (point_fusion_drain_ticks - elapsed_ticks) * 5U;
            return;
        }

        point_status.vision_drain_remaining_ms = 0U;
        point_status.vision_confirm_frames = 0U;
        point_confirm_sum_x10 = 0;
        point_confirm_sum_y10 = 0;
        point_confirm_stable_tick = pit_count;
        point_set_state(POINT_TEST_VIS_STABLE);
    }
    else if(point_status.state == POINT_TEST_VIS_STABLE)
    {
        elapsed_ticks = (uint32)(pit_count - point_settle_tick);
        point_status.elapsed_ms = (uint32)(pit_count - point_start_tick) * 5U;
        point_status.vision_confirm_age_ms = elapsed_ticks * 5U;

        /* A missing edge-of-map pose must not deadlock grid execution.
           Stable but contradictory vision still follows the strict timeout. */
        if(point_status.vision_confirm_bad_frames == 0U &&
           (uint32)(pit_count - point_confirm_stable_tick) >=
               POINT_FUSION_NO_FRAME_TICKS)
        {
            point_status.vision_fallback_used = 1U;
            point_hard_stop(POINT_FAULT_NONE, POINT_TEST_DONE);
            return;
        }

        if(elapsed_ticks >= POINT_FUSION_TIMEOUT_TICKS)
        {
            point_status.vision_fallback_used = 1U;
            point_hard_stop(POINT_FAULT_NONE, POINT_TEST_DONE);
            return;
        }
        if(!point_status.vision_live) return;

        vision_link_get_snapshot(&vision);
        if(vision.pos_packets == point_confirm_pos_packets) return;
        point_confirm_pos_packets = vision.pos_packets;
        point_status.vision_confirm_pos_packets = vision.pos_packets;
        if(vision.frame_id == point_confirm_frame_id) return;
        point_confirm_frame_id = vision.frame_id;
        point_status.vision_confirm_frame_id = vision.frame_id;

        if(point_status.vision_confirm_frames == 0U ||
           (abs((int)vision.car_x_mm - (int)point_confirm_anchor_x10) <=
                POINT_FUSION_CLUSTER_X10 &&
            abs((int)vision.car_y_mm - (int)point_confirm_anchor_y10) <=
                POINT_FUSION_CLUSTER_X10))
        {
            if(point_status.vision_confirm_frames == 0U)
            {
                point_confirm_anchor_x10 = vision.car_x_mm;
                point_confirm_anchor_y10 = vision.car_y_mm;
                point_confirm_sum_x10 = 0;
                point_confirm_sum_y10 = 0;
                point_confirm_stable_tick = pit_count;
            }
            if(point_status.vision_confirm_frames <
               point_fusion_stable_frames)
            {
                point_confirm_sum_x10 += vision.car_x_mm;
                point_confirm_sum_y10 += vision.car_y_mm;
                point_status.vision_confirm_frames++;
            }
        }
        else
        {
            point_confirm_anchor_x10 = vision.car_x_mm;
            point_confirm_anchor_y10 = vision.car_y_mm;
            point_confirm_sum_x10 = vision.car_x_mm;
            point_confirm_sum_y10 = vision.car_y_mm;
            point_confirm_stable_tick = pit_count;
            point_status.vision_confirm_frames = 1U;
        }

        point_status.vision_target_error_along_mm =
            (float)point_status.target_distance_mm -
            point_status.vision_along_mm;
        point_status.vision_target_error_cross_mm =
            -point_status.vision_cross_mm;
        point_status.vision_encoder_delta_along_mm =
            point_status.vision_along_mm - point_status.encoder_along_mm;
        point_status.vision_encoder_delta_cross_mm =
            point_status.vision_cross_mm - point_status.encoder_cross_mm;

        if(point_status.vision_confirm_frames >=
               point_fusion_stable_frames &&
           (uint32)(pit_count - point_confirm_stable_tick) >=
               point_fusion_stable_ticks)
        {
            point_status.vision_confirm_x10 =
                (int16)(point_confirm_sum_x10 /
                        (int32)point_status.vision_confirm_frames);
            point_status.vision_confirm_y10 =
                (int16)(point_confirm_sum_y10 /
                        (int32)point_status.vision_confirm_frames);
            point_status.vision_confirm_valid = 1U;
            point_hard_stop(POINT_FAULT_NONE, POINT_TEST_DONE);
            return;
        }
    }
}

void point_test_emergency_stop(void)
{
    point_status.origin_valid = 0U;
    point_status.kind = POINT_TEST_KIND_NONE;
    point_hard_stop(POINT_FAULT_ABORTED, POINT_TEST_LOCKED);
}

uint8 point_test_set_heading_target(float heading_target_deg)
{
    if(point_is_busy()) return 0U;
    point_heading_target_deg = heading_target_deg;
    point_heading_target_valid = 1U;
    return 1U;
}

void point_test_clear_heading_target(void)
{
    point_heading_target_valid = 0U;
    point_heading_target_deg = 0.0f;
}

uint8 point_test_set_direction(uint8 index)
{
    if(point_is_busy() || index >= 8U) return 0U;
    point_status.direction_index = index;
    point_status.direction_deg = point_direction_deg[index];
    point_status.command_direction_deg = point_status.direction_deg;
    point_status.brake_lead_mm = (uint16)
        point_brake_lead_for_direction(index, point_status.speed);
    return 1U;
}

uint8 point_test_set_direction_deg(float direction_deg)
{
    if(point_is_busy()) return 0U;
    while(direction_deg < 0.0f) direction_deg += 360.0f;
    while(direction_deg >= 360.0f) direction_deg -= 360.0f;
    point_status.direction_deg = direction_deg;
    point_status.command_direction_deg = direction_deg;
    point_status.direction_index = (uint8)(((uint16)(direction_deg + 22.5f) / 45U) & 7U);
    point_status.brake_lead_mm = (uint16)
        point_brake_lead_for_direction(point_status.direction_index,
                                       point_status.speed);
    return 1U;
}

uint8 point_test_set_runtime_direction_deg(float direction_deg)
{
    if(point_status.state != POINT_TEST_RUNNING ||
       point_status.kind != POINT_TEST_KIND_TRANSLATE)
    {
        return 0U;
    }

    while(direction_deg < 0.0f) direction_deg += 360.0f;
    while(direction_deg >= 360.0f) direction_deg -= 360.0f;
    point_status.command_direction_deg = direction_deg;
    return 1U;
}

uint8 point_test_set_cells(uint8 cells)
{
    if(point_is_busy() || cells < 1U || cells > 4U) return 0U;
    point_status.cell_count = cells;
    point_status.target_distance_mm = (uint16)((float)cells * POINT_CELL_MM + 0.5f);
    return 1U;
}

uint8 point_test_set_distance_mm(uint16 distance_mm)
{
    if(point_is_busy() || distance_mm < 50U ||
       distance_mm > POINT_MAX_TRANSLATE_DISTANCE_MM) return 0U;
    point_status.cell_count = (uint8)((distance_mm + 199U) / 200U);
    point_status.target_distance_mm = distance_mm;
    return 1U;
}

uint8 point_test_set_speed(uint16 speed)
{
    if(point_is_busy()) return 0U;
    if(speed != 100U && speed != 120U && speed != 150U) return 0U;
    point_status.speed = speed;
    point_status.brake_lead_mm = (uint16)
        point_brake_lead_for_direction(point_status.direction_index,
                                       point_status.speed);
    return 1U;
}

uint8 point_test_set_rotation_speed(uint16 speed)
{
    if(point_is_busy() || speed < 60U || speed > 150U) return 0U;
    point_status.speed = speed;
    return 1U;
}

uint8 point_test_set_startup_assist(uint8 enable)
{
    if(point_is_busy()) return 0U;
    point_status.startup_assist_enabled = enable ? 1U : 0U;
    if(!point_status.startup_assist_enabled) motion_startup_assist_cancel();
    return 1U;
}

uint8 point_test_set_fast_finish(uint8 enable)
{
    if(point_is_busy()) return 0U;
    point_status.fast_finish_enabled = enable ? 1U : 0U;
    point_status.fast_finish_used = 0U;
    return 1U;
}

uint8 point_test_set_fusion_profile(uint16 drain_ms, uint8 stable_frames,
                                    uint16 stable_ms)
{
    if(point_is_busy() || drain_ms > 3000U || stable_frames < 2U ||
       stable_frames > 12U || stable_ms < 50U || stable_ms > 1000U)
    {
        return 0U;
    }
    point_fusion_drain_ticks = (uint32)(drain_ms + 4U) / 5U;
    point_fusion_stable_frames = stable_frames;
    point_fusion_stable_ticks = (uint32)(stable_ms + 4U) / 5U;
    point_status.vision_confirm_required = stable_frames;
    return 1U;
}

uint8 point_test_set_sensor_mode(point_sensor_mode_t mode)
{
    if(point_is_busy() || mode < POINT_SENSOR_ENCODER_IMU ||
       mode > POINT_SENSOR_FUSION_LOCKED) return 0U;
    point_status.sensor_mode = mode;
    return 1U;
}

uint8 point_test_set_rotation_angle(uint16 angle_deg)
{
    if(point_is_busy()) return 0U;
    /* Menus expose the standard calibration angles, while mission startup
       also needs small visual-heading corrections such as 3-12 degrees. */
    if(angle_deg < 1U || angle_deg > 360U) return 0U;
    point_status.target_rotation_deg = angle_deg;
    return 1U;
}

uint8 point_test_set_rotation_clockwise(uint8 clockwise)
{
    if(point_is_busy()) return 0U;
    point_status.rotate_clockwise = clockwise ? 1U : 0U;
    return 1U;
}

uint8 point_test_set_rotation_stop(point_rotate_stop_t source)
{
    if(point_is_busy() || (source != POINT_ROTATE_STOP_IMU &&
       source != POINT_ROTATE_STOP_ENCODER)) return 0U;
    point_status.rotate_stop = source;
    return 1U;
}

uint8 point_test_set_rotation_heading_target(float heading_target_deg)
{
    if(point_is_busy()) return 0U;
    point_rotation_heading_target_deg = heading_target_deg;
    point_rotation_heading_target_valid = 1U;
    return 1U;
}

void point_test_clear_rotation_heading_target(void)
{
    point_rotation_heading_target_valid = 0U;
    point_rotation_heading_target_deg = 0.0f;
}

void point_test_get_snapshot(point_test_snapshot_t *out)
{
    if(out == 0) return;
    point_update_measurements();
    point_update_progress();
    *out = point_status;
}

const char *point_test_state_name(point_test_state_t state)
{
    switch(state)
    {
        case POINT_TEST_LOCKED: return "LOCKED";
        case POINT_TEST_READY: return "READY";
        case POINT_TEST_RUNNING: return "RUNNING";
        case POINT_TEST_SETTLING: return "SETTLING";
        case POINT_TEST_VIS_DRAIN: return "VIS_DRAIN";
        case POINT_TEST_VIS_STABLE: return "VIS_STABLE";
        case POINT_TEST_DONE: return "DONE";
        case POINT_TEST_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char *point_test_kind_name(point_test_kind_t kind)
{
    if(kind == POINT_TEST_KIND_TRANSLATE) return "TRANSLATE";
    if(kind == POINT_TEST_KIND_ROTATE) return "ROTATE";
    return "NONE";
}

const char *point_test_fault_name(point_test_fault_t fault)
{
    switch(fault)
    {
        case POINT_FAULT_NONE: return "NONE";
        case POINT_FAULT_NO_ORIGIN: return "NO_ORIGIN";
        case POINT_FAULT_IMU_NOT_READY: return "IMU_NOT_READY";
        case POINT_FAULT_VISION_NOT_READY: return "VISION_NOT_READY";
        case POINT_FAULT_TIMEOUT: return "TIMEOUT";
        case POINT_FAULT_OVERTRAVEL: return "OVERTRAVEL";
        case POINT_FAULT_BAD_CONFIG: return "BAD_CONFIG";
        case POINT_FAULT_ABORTED: return "ABORTED";
        case POINT_FAULT_VIS_CONFIRM_TIMEOUT: return "VIS_CONFIRM_TIMEOUT";
        case POINT_FAULT_GRID_MISMATCH: return "GRID_MISMATCH";
        case POINT_FAULT_STALL: return "STALL";
        default: return "UNKNOWN";
    }
}

const char *point_test_sensor_name(point_sensor_mode_t mode)
{
    switch(mode)
    {
        case POINT_SENSOR_ENCODER_IMU: return "ENC_STOP+IMU_HOLD";
        case POINT_SENSOR_ENCODER_OPEN_YAW: return "ENC_STOP+YAW_OFF";
        case POINT_SENSOR_VISION_IMU: return "VIS_STOP+IMU_HOLD";
        case POINT_SENSOR_FUSION_LOCKED: return "ENC_MOVE+DELAY_VIS_CHECK";
        default: return "UNKNOWN";
    }
}

const char *point_test_rotation_stop_name(point_rotate_stop_t source)
{
    return source == POINT_ROTATE_STOP_ENCODER ? "ENCODER" : "IMU";
}

const char *point_test_direction_name(uint8 index)
{
    static const char *const name[8] =
    {
        "FORWARD", "FRONT_RIGHT", "RIGHT", "BACK_RIGHT",
        "BACK", "BACK_LEFT", "LEFT", "FRONT_LEFT"
    };
    return index < 8U ? name[index] : "UNKNOWN";
}
