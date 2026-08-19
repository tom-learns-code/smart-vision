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
    TRACK_CAL_MAP_SCAN
} track_cal_state_t;

typedef enum
{
    TRACK_CAL_PROFILE_MANUAL = 0,
    TRACK_CAL_PROFILE_QUICK,
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
    point_test_snapshot_t point;
    track_cal_record_t last_record;
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

#endif
