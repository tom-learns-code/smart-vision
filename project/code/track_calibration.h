#ifndef __TRACK_CALIBRATION_H
#define __TRACK_CALIBRATION_H

#include "zf_common_typedef.h"
#include "point_test.h"

typedef enum
{
    TRACK_CAL_LOCKED = 0,
    TRACK_CAL_IDLE,
    TRACK_CAL_WAIT_ORIGIN,
    TRACK_CAL_RUNNING,
    TRACK_CAL_INTER_STEP,
    TRACK_CAL_PAUSED,
    TRACK_CAL_COMPLETE,
    TRACK_CAL_FAULT,
    TRACK_CAL_STATIONARY,
    TRACK_CAL_MAP_SCAN,
    TRACK_CAL_ROT_WAIT,
    TRACK_CAL_ROT_RUNNING,
    TRACK_CAL_ROT_VIS_DRAIN,
    TRACK_CAL_ROT_VIS_STABLE,
    TRACK_CAL_ROT_INTER_STEP,
    TRACK_CAL_ROT_HAND
} track_cal_state_t;

typedef enum
{
    TRACK_CAL_PROFILE_MANUAL = 0,
    TRACK_CAL_PROFILE_QUICK,
    TRACK_CAL_PROFILE_GRID2,
    TRACK_CAL_PROFILE_GRID3,
    TRACK_CAL_PROFILE_LENGTH_SWEEP,
    TRACK_CAL_PROFILE_STANDARD,
    TRACK_CAL_PROFILE_FULL
} track_cal_profile_t;

typedef enum
{
    TRACK_CAL_LOAD_FREE = 0,
    TRACK_CAL_LOAD_BOX,
    TRACK_CAL_LOAD_BOMB
} track_cal_load_t;

typedef enum
{
    TRACK_CAL_RESULT_NONE = 0,
    TRACK_CAL_RESULT_OK,
    TRACK_CAL_RESULT_POINT_FAULT,
    TRACK_CAL_RESULT_VISION_WAIT_TIMEOUT,
    TRACK_CAL_RESULT_ABORTED,
    TRACK_CAL_RESULT_SKIPPED
} track_cal_result_t;

typedef enum
{
    TRACK_ROT_ACTIVE = 0,
    TRACK_ROT_HAND
} track_rotation_mode_t;

typedef enum
{
    TRACK_ROT_SEQUENCE_SINGLE = 0,
    TRACK_ROT_SEQUENCE_PAIR,
    TRACK_ROT_SEQUENCE_AUTO16
} track_rotation_sequence_t;

typedef struct
{
    uint32 sequence;
    uint32 session;
    track_rotation_mode_t mode;
    track_rotation_sequence_t sequence_type;
    track_cal_result_t result;
    point_test_fault_t point_fault;
    uint8 valid;
    uint8 clockwise;
    uint8 auto_index;
    uint8 auto_count;
    uint16 angle_deg;
    uint16 speed;
    int32 target_heading_x10;
    int32 start_imu_x10;
    int32 stop_imu_x10;
    int32 end_imu_x10;
    int32 imu_delta_x10;
    int32 final_error_x10;
    int32 overshoot_x10;
    int32 encoder_yaw_x10;
    int32 wheel_count[3];
    int32 effective_radius_x10;
    int16 start_x10;
    int16 start_y10;
    int16 start_theta_x10;
    int16 end_x10;
    int16 end_y10;
    int16 end_theta_x10;
    int32 vision_delta_theta_x10;
    uint8 vision_valid;
    uint8 vision_stable_frames;
    int16 vision_theta_spread_x10;
    uint32 vision_age_ms;
    uint32 elapsed_ms;
    uint32 hold_stable_ms;
    int32 speed_abs_avg[3];
    int32 duty_abs_avg[3];
    int16 duty_abs_peak[3];
    uint32 drive_samples;
} track_rotation_record_t;

typedef struct
{
    uint32 sequence;
    uint32 session;
    uint16 step_index;
    uint16 step_count;
    track_cal_profile_t profile;
    track_cal_load_t load;
    track_cal_result_t result;
    point_test_fault_t point_fault;
    uint8 valid;
    uint8 direction;
    uint8 cells;
    uint16 speed;
    uint16 brake_lead_mm;
    uint16 voltage_x10;
    int16 start_x10;
    int16 start_y10;
    int16 start_theta_x10;
    int16 end_x10;
    int16 end_y10;
    int16 end_theta_x10;
    uint8 start_frame;
    uint8 end_frame;
    uint32 start_pos_packets;
    uint32 end_pos_packets;
    int32 encoder_along_x10;
    int32 encoder_cross_x10;
    int32 vision_along_x10;
    int32 vision_cross_x10;
    int32 vision_target_error_along_x10;
    int32 vision_target_error_cross_x10;
    int32 vision_encoder_delta_along_x10;
    int32 vision_encoder_delta_cross_x10;
    int32 imu_yaw_x10;
    int32 encoder_yaw_x10;
    int32 max_yaw_x10;
    int32 max_cross_x10;
    int32 wheel_count[3];
    int32 speed_abs_avg[3];
    int32 duty_abs_avg[3];
    int16 duty_abs_peak[3];
    uint32 drive_samples;
    uint8 startup_assist_mask;
    uint16 startup_assist_samples;
    uint32 stall_watch_ms;
    int32 stall_window_progress_x10;
    uint32 elapsed_ms;
    uint8 stable_frames;
    uint8 bad_clusters;
    uint32 vision_age_ms;
} track_cal_record_t;

typedef struct
{
    track_cal_state_t state;
    track_cal_profile_t selected_profile;
    track_cal_profile_t active_profile;
    track_cal_load_t load;
    track_cal_result_t last_result;
    point_test_fault_t point_fault;
    uint8 manual_direction;
    uint8 manual_cells;
    uint8 manual_repeats;
    uint8 auto_active;
    uint8 paused;
    uint8 telemetry_enabled;
    uint16 speed;
    uint16 voltage_x10;
    uint16 route_index;
    uint16 route_count;
    uint16 valid_records;
    uint16 invalid_records;
    uint32 session;
    uint32 record_sequence;
    uint32 event_counter;
    uint32 wait_remaining_ms;
    uint32 stationary_duration_ms;
    uint32 stationary_elapsed_ms;
    uint32 stationary_frames;
    int16 stationary_start_x10;
    int16 stationary_start_y10;
    int16 stationary_min_x10;
    int16 stationary_max_x10;
    int16 stationary_min_y10;
    int16 stationary_max_y10;
    int32 stationary_yaw_min_x10;
    int32 stationary_yaw_max_x10;
    uint8 map_scan_received;
    uint8 map_scan_target;
    uint16 map_scan_cell_disagreements;
    uint16 map_scan_max_frame_disagreements;
    uint32 map_scan_reference_version;
    uint32 map_scan_last_version;
    track_rotation_mode_t rotation_mode;
    track_rotation_sequence_t rotation_sequence;
    uint8 rotation_clockwise;
    uint8 rotation_repeats;
    uint8 rotation_index;
    uint8 rotation_count;
    uint16 rotation_angle_deg;
    uint16 rotation_speed;
    uint8 rotation_vision_frames;
    uint8 rotation_vision_required;
    uint32 rotation_wait_ms;
    uint32 rotation_record_sequence;
    point_test_snapshot_t point;
    track_cal_record_t last_record;
    track_rotation_record_t last_rotation_record;
} track_cal_snapshot_t;

void track_calibration_init(void);
void track_calibration_poll(void);
void track_calibration_emergency_stop(void);
uint8 track_calibration_verify_origin(void);
uint8 track_calibration_start_manual(uint8 use_repeats);
uint8 track_calibration_start_auto(void);
uint8 track_calibration_pause(void);
uint8 track_calibration_resume(void);
uint8 track_calibration_skip(void);
uint8 track_calibration_start_stationary(uint16 seconds);
uint8 track_calibration_start_map_scan(void);
uint8 track_calibration_start_rotation_single(void);
uint8 track_calibration_start_rotation_pair(void);
uint8 track_calibration_start_rotation_auto(void);
uint8 track_calibration_start_rotation_hand(void);
uint8 track_calibration_finish_rotation_hand(void);
uint8 track_calibration_set_rotation_angle(uint16 angle_deg);
uint8 track_calibration_set_rotation_clockwise(uint8 clockwise);
uint8 track_calibration_set_rotation_speed(uint16 speed);
uint8 track_calibration_set_rotation_repeats(uint8 repeats);
uint8 track_calibration_set_direction(uint8 direction);
uint8 track_calibration_set_cells(uint8 cells);
uint8 track_calibration_set_speed(uint16 speed);
uint8 track_calibration_set_repeats(uint8 repeats);
uint8 track_calibration_set_load(track_cal_load_t load);
uint8 track_calibration_set_voltage_x10(uint16 voltage_x10);
uint8 track_calibration_set_profile(track_cal_profile_t profile);
void track_calibration_set_telemetry(uint8 enabled);
void track_calibration_get_snapshot(track_cal_snapshot_t *out);
const char *track_calibration_state_name(track_cal_state_t state);
const char *track_calibration_profile_name(track_cal_profile_t profile);
const char *track_calibration_load_name(track_cal_load_t load);
const char *track_calibration_result_name(track_cal_result_t result);
const char *track_calibration_rotation_mode_name(track_rotation_mode_t mode);
const char *track_calibration_rotation_sequence_name(track_rotation_sequence_t sequence);

#endif
