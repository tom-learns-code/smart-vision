#ifndef __POINT_TEST_H
#define __POINT_TEST_H

#include "zf_common_typedef.h"

typedef enum
{
    POINT_TEST_LOCKED = 0,
    POINT_TEST_READY,
    POINT_TEST_RUNNING,
    POINT_TEST_SETTLING,
    POINT_TEST_VIS_DRAIN,
    POINT_TEST_VIS_STABLE,
    POINT_TEST_DONE,
    POINT_TEST_FAULT
} point_test_state_t;

typedef enum
{
    POINT_TEST_KIND_NONE = 0,
    POINT_TEST_KIND_TRANSLATE,
    POINT_TEST_KIND_ROTATE
} point_test_kind_t;

typedef enum
{
    POINT_SENSOR_ENCODER_IMU = 1,
    POINT_SENSOR_ENCODER_OPEN_YAW,
    POINT_SENSOR_VISION_IMU,
    POINT_SENSOR_FUSION_LOCKED
} point_sensor_mode_t;

typedef enum
{
    POINT_ROTATE_STOP_IMU = 1,
    POINT_ROTATE_STOP_ENCODER
} point_rotate_stop_t;

typedef enum
{
    POINT_FAULT_NONE = 0,
    POINT_FAULT_NO_ORIGIN,
    POINT_FAULT_IMU_NOT_READY,
    POINT_FAULT_VISION_NOT_READY,
    POINT_FAULT_TIMEOUT,
    POINT_FAULT_OVERTRAVEL,
    POINT_FAULT_BAD_CONFIG,
    POINT_FAULT_ABORTED,
    POINT_FAULT_VIS_CONFIRM_TIMEOUT,
    POINT_FAULT_GRID_MISMATCH,
    POINT_FAULT_STALL
} point_test_fault_t;

typedef struct
{
    point_test_state_t state;
    point_test_kind_t kind;
    point_test_fault_t fault;
    point_sensor_mode_t sensor_mode;
    point_rotate_stop_t rotate_stop;
    uint8 origin_valid;
    uint8 vision_origin_valid;
    uint8 vision_live;
    uint8 vision_input_stable;
    uint8 vision_input_stable_frames;
    uint8 vision_position_stable;
    uint8 vision_position_stable_frames;
    uint8 vision_input_frame_id;
    uint8 direction_index;
    uint8 cell_count;
    uint8 rotate_clockwise;
    uint8 rotate_approach_active;
    uint16 speed;
    uint16 commanded_speed;
    uint16 brake_lead_mm;
    uint8 startup_assist_enabled;
    uint8 startup_assist_active;
    uint8 startup_assist_applied_mask;
    uint16 startup_assist_remaining_ms;
    uint32 stall_watch_ms;
    float stall_window_progress_mm;
    uint16 target_distance_mm;
    uint16 target_rotation_deg;
    float rotation_target_heading_deg;
    float rotation_target_error_deg;
    float rotation_stop_heading_deg;
    float rotation_overshoot_deg;
    uint32 rotation_hold_stable_ms;
    float direction_deg;
    float command_direction_deg;
    float imu_origin_deg;
    float imu_relative_deg;
    float encoder_yaw_deg;
    float encoder_forward_mm;
    float encoder_right_mm;
    float encoder_along_mm;
    float encoder_cross_mm;
    float vision_forward_mm;
    float vision_right_mm;
    float vision_along_mm;
    float vision_cross_mm;
    float active_progress;
    float remaining;
    float max_abs_yaw_deg;
    float max_abs_cross_mm;
    float mm_per_count;
    float wheel_center_radius_mm;
    int32 wheel_count[3];
    int32 encoder_rotate_count;
    int16 vision_x10;
    int16 vision_y10;
    int16 vision_theta_x10;
    uint32 vision_age_ms;
    uint32 vision_input_stable_ms;
    uint32 vision_position_stable_ms;
    uint32 elapsed_ms;
    uint32 settle_remaining_ms;
    uint8 vision_confirm_frames;
    uint8 vision_confirm_required;
    uint8 vision_confirm_bad_frames;
    uint8 vision_fallback_used;
    uint8 fast_finish_enabled;
    uint8 fast_finish_used;
    uint8 vision_confirm_valid;
    uint8 vision_confirm_frame_id;
    int16 vision_confirm_x10;
    int16 vision_confirm_y10;
    uint32 vision_confirm_age_ms;
    uint32 vision_confirm_pos_packets;
    uint32 vision_drain_remaining_ms;
    float vision_target_error_along_mm;
    float vision_target_error_cross_mm;
    float vision_encoder_delta_along_mm;
    float vision_encoder_delta_cross_mm;
    uint32 event_counter;
} point_test_snapshot_t;

void point_test_init(void);
void point_test_poll(void);
uint8 point_test_capture_origin(void);
void point_test_clear_vision_origin(void);
uint8 point_test_start_translation(void);
uint8 point_test_start_rotation(void);
void point_test_emergency_stop(void);
uint8 point_test_set_heading_target(float heading_target_deg);
void point_test_clear_heading_target(void);
uint8 point_test_set_direction(uint8 index);
uint8 point_test_set_direction_deg(float direction_deg);
uint8 point_test_set_runtime_direction_deg(float direction_deg);
uint8 point_test_set_cells(uint8 cells);
uint8 point_test_set_distance_mm(uint16 distance_mm);
uint8 point_test_set_speed(uint16 speed);
uint8 point_test_set_rotation_speed(uint16 speed);
uint8 point_test_set_startup_assist(uint8 enable);
uint8 point_test_set_fast_finish(uint8 enable);
uint8 point_test_set_fusion_profile(uint16 drain_ms, uint8 stable_frames,
                                    uint16 stable_ms);
uint8 point_test_set_sensor_mode(point_sensor_mode_t mode);
uint8 point_test_set_rotation_angle(uint16 angle_deg);
uint8 point_test_set_rotation_clockwise(uint8 clockwise);
uint8 point_test_set_rotation_stop(point_rotate_stop_t source);
uint8 point_test_set_rotation_heading_target(float heading_target_deg);
void point_test_clear_rotation_heading_target(void);
void point_test_get_snapshot(point_test_snapshot_t *out);
const char *point_test_state_name(point_test_state_t state);
const char *point_test_kind_name(point_test_kind_t kind);
const char *point_test_fault_name(point_test_fault_t fault);
const char *point_test_sensor_name(point_sensor_mode_t mode);
const char *point_test_rotation_stop_name(point_rotate_stop_t source);
const char *point_test_direction_name(uint8 index);

#endif
