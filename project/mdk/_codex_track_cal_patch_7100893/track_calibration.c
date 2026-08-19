#include <stdlib.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "blue.h"
#include "motion_control.h"
#include "vision_link.h"
#include "point_test.h"
#include "track_calibration.h"

#define TRACK_CAL_INTER_STEP_TICKS       (100U)  /* 500 ms */
#define TRACK_CAL_ORIGIN_TIMEOUT_TICKS   (1000U) /* 5 s */
#define TRACK_CAL_SAMPLE_TICKS           (4U)    /* 20 ms */
#define TRACK_CAL_MAP_TIMEOUT_TICKS      (600U)  /* 3 s */
#define TRACK_CAL_MAP_FRAMES             (5U)

static track_cal_snapshot_t track_status;
static uint32 track_state_tick;
static uint32 track_sample_tick;
static uint32 track_last_stationary_pos_packets;
static uint32 track_map_request_tick;
static uint32 track_map_version_before_request;
static vision_link_map_t track_map_reference;
static int32 track_speed_abs_sum[3];
static int32 track_duty_abs_sum[3];
static int16 track_duty_abs_peak[3];
static uint32 track_drive_samples;
static float track_session_heading_deg;

static int32 track_round_x10(float value)
{
    return (int32)(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
}

static int16 track_abs16(int16 value)
{
    if(value == (int16)-32768) return 32767;
    return value < 0 ? (int16)-value : value;
}

static void track_set_state(track_cal_state_t state)
{
    if(track_status.state != state)
    {
        track_status.state = state;
        track_status.event_counter++;
    }
}

static uint8 track_is_active(void)
{
    return (track_status.state == TRACK_CAL_WAIT_ORIGIN ||
            track_status.state == TRACK_CAL_RUNNING ||
            track_status.state == TRACK_CAL_INTER_STEP ||
            track_status.state == TRACK_CAL_STATIONARY ||
            track_status.state == TRACK_CAL_MAP_SCAN) ? 1U : 0U;
}

static void track_reset_drive_stats(void)
{
    memset(track_speed_abs_sum, 0, sizeof(track_speed_abs_sum));
    memset(track_duty_abs_sum, 0, sizeof(track_duty_abs_sum));
    memset(track_duty_abs_peak, 0, sizeof(track_duty_abs_peak));
    track_drive_samples = 0U;
    track_sample_tick = pit_count;
}

static void track_sample_drive(void)
{
    uint8 i;

    if((uint32)(pit_count - track_sample_tick) < TRACK_CAL_SAMPLE_TICKS) return;
    track_sample_tick = pit_count;
    for(i = 0U; i < 3U; i++)
    {
        int16 speed_abs = track_abs16(encoder_speed[i]);
        int16 duty_abs = track_abs16(motor_output_duty[i]);
        track_speed_abs_sum[i] += speed_abs;
        track_duty_abs_sum[i] += duty_abs;
        if(duty_abs > track_duty_abs_peak[i]) track_duty_abs_peak[i] = duty_abs;
    }
    track_drive_samples++;
}

static uint8 track_route_direction(track_cal_profile_t profile,
                                   uint16 index, uint16 *speed)
{
    static const uint8 quick_route[12] =
    {
        0U, 4U, 2U, 6U,
        0U, 2U, 4U, 6U,
        0U, 6U, 4U, 2U
    };
    uint16 route_index = index;

    if(profile == TRACK_CAL_PROFILE_MANUAL)
    {
        return track_status.manual_direction;
    }
    if(profile == TRACK_CAL_PROFILE_QUICK)
    {
        return quick_route[index % 12U];
    }
    if(profile == TRACK_CAL_PROFILE_FULL)
    {
        uint16 phase = index / 60U;
        route_index = index % 60U;
        *speed = phase == 0U ? 100U : (phase == 1U ? 120U : 150U);
    }

    if(route_index < 10U) return (route_index & 1U) ? 4U : 0U;
    if(route_index < 20U) return (route_index & 1U) ? 6U : 2U;
    if(route_index < 32U)
    {
        uint8 phase = (uint8)((route_index - 20U) & 3U);
        return phase < 2U ? 0U : 4U;
    }
    if(route_index < 44U)
    {
        uint8 phase = (uint8)((route_index - 32U) & 3U);
        return phase < 2U ? 2U : 6U;
    }
    if(route_index < 52U)
    {
        static const uint8 cw_square[4] = {0U, 2U, 4U, 6U};
        return cw_square[(route_index - 44U) & 3U];
    }
    {
        static const uint8 ccw_square[4] = {0U, 6U, 4U, 2U};
        return ccw_square[(route_index - 52U) & 3U];
    }
}

static uint16 track_route_count(track_cal_profile_t profile)
{
    if(profile == TRACK_CAL_PROFILE_QUICK) return 12U;
    if(profile == TRACK_CAL_PROFILE_STANDARD) return 60U;
    if(profile == TRACK_CAL_PROFILE_FULL) return 180U;
    return track_status.manual_repeats;
}

static void track_capture_record_start(track_cal_record_t *record,
                                       const vision_link_snapshot_t *vision,
                                       uint8 direction, uint8 cells,
                                       uint16 speed)
{
    memset(record, 0, sizeof(*record));
    record->session = track_status.session;
    record->step_index = (uint16)(track_status.route_index + 1U);
    record->step_count = track_status.route_count;
    record->profile = track_status.active_profile;
    record->load = track_status.load;
    record->direction = direction;
    record->cells = cells;
    record->speed = speed;
    record->voltage_x10 = track_status.voltage_x10;
    record->start_x10 = vision->car_x_mm;
    record->start_y10 = vision->car_y_mm;
    record->start_theta_x10 = vision->car_theta_x10;
    record->start_frame = vision->frame_id;
    record->start_pos_packets = vision->pos_packets;
}

static void track_finalize_record(track_cal_result_t result,
                                  const point_test_snapshot_t *point)
{
    vision_link_snapshot_t vision;
    track_cal_record_t *record = &track_status.last_record;
    uint8 i;

    vision_link_get_snapshot(&vision);
    track_status.record_sequence++;
    record->sequence = track_status.record_sequence;
    record->result = result;
    record->point_fault = point->fault;
    record->valid = result == TRACK_CAL_RESULT_OK ? 1U : 0U;
    record->end_x10 = vision.car_x_mm;
    record->end_y10 = vision.car_y_mm;
    record->end_theta_x10 = vision.car_theta_x10;
    record->end_frame = vision.frame_id;
    record->end_pos_packets = vision.pos_packets;
    record->encoder_along_x10 = track_round_x10(point->encoder_along_mm);
    record->encoder_cross_x10 = track_round_x10(point->encoder_cross_mm);
    record->vision_along_x10 = track_round_x10(point->vision_along_mm);
    record->vision_cross_x10 = track_round_x10(point->vision_cross_mm);
    record->vision_target_error_along_x10 =
        track_round_x10(point->vision_target_error_along_mm);
    record->vision_target_error_cross_x10 =
        track_round_x10(point->vision_target_error_cross_mm);
    record->vision_encoder_delta_along_x10 =
        track_round_x10(point->vision_encoder_delta_along_mm);
    record->vision_encoder_delta_cross_x10 =
        track_round_x10(point->vision_encoder_delta_cross_mm);
    record->imu_yaw_x10 = track_round_x10(point->imu_relative_deg);
    record->encoder_yaw_x10 = track_round_x10(point->encoder_yaw_deg);
    record->max_yaw_x10 = track_round_x10(point->max_abs_yaw_deg);
    record->max_cross_x10 = track_round_x10(point->max_abs_cross_mm);
    record->drive_samples = track_drive_samples;
    record->elapsed_ms = point->elapsed_ms;
    record->stable_frames = point->vision_confirm_frames;
    record->bad_clusters = point->vision_confirm_bad_frames;
    record->vision_age_ms = point->vision_age_ms;
    for(i = 0U; i < 3U; i++)
    {
        record->wheel_count[i] = point->wheel_count[i];
        record->speed_abs_avg[i] = track_drive_samples == 0U ? 0 :
            track_speed_abs_sum[i] / (int32)track_drive_samples;
        record->duty_abs_avg[i] = track_drive_samples == 0U ? 0 :
            track_duty_abs_sum[i] / (int32)track_drive_samples;
        record->duty_abs_peak[i] = track_duty_abs_peak[i];
    }
    if(record->valid) track_status.valid_records++;
    else track_status.invalid_records++;
    track_status.last_result = result;
    track_status.point_fault = point->fault;
    track_status.event_counter++;
}

static uint8 track_prepare_step(void)
{
    point_test_snapshot_t point;
    vision_link_snapshot_t vision;
    uint16 speed = track_status.speed;
    uint8 cells = track_status.active_profile == TRACK_CAL_PROFILE_MANUAL ?
        track_status.manual_cells : 1U;
    uint8 direction = track_route_direction(track_status.active_profile,
                                             track_status.route_index,
                                             &speed);

    point_test_get_snapshot(&point);
    if(!point.vision_live || !point.vision_input_stable || !imu963ra_ready)
    {
        return 0U;
    }
    if(!point_test_set_sensor_mode(POINT_SENSOR_FUSION_LOCKED) ||
       !point_test_set_direction(direction) ||
       !point_test_set_cells(cells) ||
       !point_test_set_speed(speed) ||
       !point_test_set_heading_target(track_session_heading_deg) ||
       !point_test_capture_origin())
    {
        return 0U;
    }

    vision_link_get_snapshot(&vision);
    track_capture_record_start(&track_status.last_record, &vision,
                               direction, cells, speed);
    track_reset_drive_stats();
    if(!point_test_start_translation())
    {
        point_test_get_snapshot(&point);
        track_finalize_record(TRACK_CAL_RESULT_POINT_FAULT, &point);
        track_set_state(TRACK_CAL_PAUSED);
        track_status.paused = 1U;
        return 0U;
    }
    track_set_state(TRACK_CAL_RUNNING);
    return 1U;
}

static uint8 track_start_sequence(track_cal_profile_t profile, uint16 count)
{
    point_test_snapshot_t point;

    if(track_is_active()) return 0U;
    point_test_get_snapshot(&point);
    if(!imu963ra_ready || !point.vision_live || !point.vision_input_stable)
        return 0U;

    point_test_emergency_stop();
    track_status.session++;
    track_status.active_profile = profile;
    track_status.route_index = 0U;
    track_status.route_count = count;
    track_status.valid_records = 0U;
    track_status.invalid_records = 0U;
    track_status.last_result = TRACK_CAL_RESULT_NONE;
    track_status.point_fault = POINT_FAULT_NONE;
    track_status.auto_active = count > 1U || profile != TRACK_CAL_PROFILE_MANUAL;
    track_status.paused = 0U;
    track_session_heading_deg = imu963ra_yaw_angle;
    track_state_tick = pit_count;
    track_set_state(TRACK_CAL_WAIT_ORIGIN);
    return 1U;
}

static uint8 track_map_cell_type(const vision_link_map_t *map, uint8 x, uint8 y)
{
    uint16 index = (uint16)y * VISION_LINK_GRID_W + x;
    uint8 i;

    if((map->wall_bits[index >> 3] >> (index & 7U)) & 1U) return 1U;
    for(i = 0U; i < map->box_count; i++)
        if(map->boxes[i].gx == (int8)x && map->boxes[i].gy == (int8)y) return 2U;
    for(i = 0U; i < map->goal_count; i++)
        if(map->goals[i].gx == (int8)x && map->goals[i].gy == (int8)y) return 3U;
    for(i = 0U; i < map->bomb_count; i++)
        if(map->bombs[i].gx == (int8)x && map->bombs[i].gy == (int8)y) return 4U;
    return 0U;
}

static uint16 track_compare_maps(const vision_link_map_t *a,
                                 const vision_link_map_t *b)
{
    uint8 x;
    uint8 y;
    uint16 differences = 0U;

    if(a->width != b->width || a->height != b->height)
        return VISION_LINK_GRID_W * VISION_LINK_GRID_H;
    for(y = 0U; y < a->height; y++)
    {
        for(x = 0U; x < a->width; x++)
        {
            if(track_map_cell_type(a, x, y) != track_map_cell_type(b, x, y))
                differences++;
        }
    }
    return differences;
}

void track_calibration_init(void)
{
    memset(&track_status, 0, sizeof(track_status));
    track_status.state = TRACK_CAL_LOCKED;
    track_status.selected_profile = TRACK_CAL_PROFILE_QUICK;
    track_status.active_profile = TRACK_CAL_PROFILE_MANUAL;
    track_status.load = TRACK_CAL_LOAD_FREE;
    track_status.manual_direction = 0U;
    track_status.manual_cells = 1U;
    track_status.manual_repeats = 3U;
    track_status.speed = 120U;
    track_status.voltage_x10 = 123U;
    track_status.telemetry_enabled = 1U;
    track_status.map_scan_target = TRACK_CAL_MAP_FRAMES;
    point_test_get_snapshot(&track_status.point);
    motion_emergency_stop();
    device_init_flag = 1;
}

void track_calibration_poll(void)
{
    point_test_snapshot_t point;

    point_test_get_snapshot(&point);
    track_status.point = point;

    if(track_status.state == TRACK_CAL_WAIT_ORIGIN)
    {
        if(track_prepare_step()) return;
        if((uint32)(pit_count - track_state_tick) >= TRACK_CAL_ORIGIN_TIMEOUT_TICKS)
        {
            track_status.last_result = TRACK_CAL_RESULT_VISION_WAIT_TIMEOUT;
            track_status.paused = 1U;
            track_set_state(TRACK_CAL_PAUSED);
        }
    }
    else if(track_status.state == TRACK_CAL_RUNNING)
    {
        track_sample_drive();
        if(point.state == POINT_TEST_DONE)
        {
            track_finalize_record(TRACK_CAL_RESULT_OK, &point);
            track_status.route_index++;
            track_state_tick = pit_count;
            track_set_state(TRACK_CAL_INTER_STEP);
        }
        else if(point.state == POINT_TEST_FAULT || point.state == POINT_TEST_LOCKED)
        {
            track_finalize_record(TRACK_CAL_RESULT_POINT_FAULT, &point);
            track_status.paused = 1U;
            track_set_state(TRACK_CAL_PAUSED);
        }
    }
    else if(track_status.state == TRACK_CAL_INTER_STEP)
    {
        uint32 elapsed = (uint32)(pit_count - track_state_tick);
        track_status.wait_remaining_ms = elapsed >= TRACK_CAL_INTER_STEP_TICKS ? 0U :
            (TRACK_CAL_INTER_STEP_TICKS - elapsed) * 5U;
        if(elapsed >= TRACK_CAL_INTER_STEP_TICKS)
        {
            if(track_status.route_index >= track_status.route_count)
            {
                track_status.auto_active = 0U;
                point_test_clear_heading_target();
                track_set_state(TRACK_CAL_COMPLETE);
            }
            else
            {
                track_state_tick = pit_count;
                track_set_state(TRACK_CAL_WAIT_ORIGIN);
            }
        }
    }
    else if(track_status.state == TRACK_CAL_STATIONARY)
    {
        vision_link_snapshot_t vision;
        int32 yaw_x10 = track_round_x10(imu963ra_yaw_angle);
        uint32 elapsed = (uint32)(pit_count - track_state_tick) * 5U;

        track_status.stationary_elapsed_ms = elapsed;
        if(yaw_x10 < track_status.stationary_yaw_min_x10)
            track_status.stationary_yaw_min_x10 = yaw_x10;
        if(yaw_x10 > track_status.stationary_yaw_max_x10)
            track_status.stationary_yaw_max_x10 = yaw_x10;
        vision_link_get_snapshot(&vision);
        if(vision.pose_valid && vision.pos_packets != track_last_stationary_pos_packets)
        {
            track_last_stationary_pos_packets = vision.pos_packets;
            track_status.stationary_frames++;
            if(vision.car_x_mm < track_status.stationary_min_x10)
                track_status.stationary_min_x10 = vision.car_x_mm;
            if(vision.car_x_mm > track_status.stationary_max_x10)
                track_status.stationary_max_x10 = vision.car_x_mm;
            if(vision.car_y_mm < track_status.stationary_min_y10)
                track_status.stationary_min_y10 = vision.car_y_mm;
            if(vision.car_y_mm > track_status.stationary_max_y10)
                track_status.stationary_max_y10 = vision.car_y_mm;
        }
        if(elapsed >= track_status.stationary_duration_ms)
        {
            track_status.last_result = TRACK_CAL_RESULT_OK;
            track_set_state(TRACK_CAL_COMPLETE);
        }
    }
    else if(track_status.state == TRACK_CAL_MAP_SCAN)
    {
        vision_link_snapshot_t vision;
        vision_link_map_t map;

        vision_link_get_snapshot(&vision);
        if(vision.map_version != track_map_version_before_request &&
           vision_link_get_map(&map) && map.valid)
        {
            uint16 differences = 0U;
            if(track_status.map_scan_received == 0U)
            {
                track_map_reference = map;
                track_status.map_scan_reference_version = map.map_version;
            }
            else
            {
                differences = track_compare_maps(&track_map_reference, &map);
                track_status.map_scan_cell_disagreements += differences;
                if(differences > track_status.map_scan_max_frame_disagreements)
                    track_status.map_scan_max_frame_disagreements = differences;
            }
            track_status.map_scan_received++;
            track_status.map_scan_last_version = map.map_version;
            track_status.event_counter++;
            if(track_status.map_scan_received >= track_status.map_scan_target)
            {
                track_status.last_result = TRACK_CAL_RESULT_OK;
                track_set_state(TRACK_CAL_COMPLETE);
            }
            else
            {
                track_map_version_before_request = map.map_version;
                track_map_request_tick = pit_count;
                vision_link_request_full_map();
            }
        }
        else if((uint32)(pit_count - track_map_request_tick) >=
                TRACK_CAL_MAP_TIMEOUT_TICKS)
        {
            track_status.last_result = TRACK_CAL_RESULT_VISION_WAIT_TIMEOUT;
            track_set_state(TRACK_CAL_FAULT);
        }
    }
}

void track_calibration_emergency_stop(void)
{
    point_test_emergency_stop();
    point_test_clear_heading_target();
    track_status.auto_active = 0U;
    track_status.paused = 0U;
    track_status.last_result = TRACK_CAL_RESULT_ABORTED;
    track_set_state(TRACK_CAL_LOCKED);
}

uint8 track_calibration_verify_origin(void)
{
    if(track_is_active()) return 0U;
    if(!point_test_set_sensor_mode(POINT_SENSOR_FUSION_LOCKED)) return 0U;
    if(!point_test_capture_origin()) return 0U;
    point_test_get_snapshot(&track_status.point);
    track_status.last_result = TRACK_CAL_RESULT_NONE;
    track_set_state(TRACK_CAL_IDLE);
    return 1U;
}

uint8 track_calibration_start_manual(uint8 use_repeats)
{
    return track_start_sequence(TRACK_CAL_PROFILE_MANUAL,
        use_repeats ? track_status.manual_repeats : 1U);
}

uint8 track_calibration_start_auto(void)
{
    return track_start_sequence(track_status.selected_profile,
                                track_route_count(track_status.selected_profile));
}

uint8 track_calibration_pause(void)
{
    point_test_snapshot_t point;

    if(track_status.state == TRACK_CAL_PAUSED) return 1U;
    if(track_status.state != TRACK_CAL_WAIT_ORIGIN &&
       track_status.state != TRACK_CAL_RUNNING &&
       track_status.state != TRACK_CAL_INTER_STEP) return 0U;
    point_test_get_snapshot(&point);
    if(track_status.state == TRACK_CAL_RUNNING)
    {
        point_test_emergency_stop();
        point_test_get_snapshot(&point);
        track_finalize_record(TRACK_CAL_RESULT_ABORTED, &point);
    }
    track_status.paused = 1U;
    track_set_state(TRACK_CAL_PAUSED);
    return 1U;
}

uint8 track_calibration_resume(void)
{
    if(track_status.state != TRACK_CAL_PAUSED ||
       track_status.route_index >= track_status.route_count) return 0U;
    track_status.paused = 0U;
    track_status.last_result = TRACK_CAL_RESULT_NONE;
    track_state_tick = pit_count;
    track_set_state(TRACK_CAL_WAIT_ORIGIN);
    return 1U;
}

uint8 track_calibration_skip(void)
{
    if(track_status.state != TRACK_CAL_PAUSED) return 0U;
    track_status.last_result = TRACK_CAL_RESULT_SKIPPED;
    track_status.invalid_records++;
    track_status.route_index++;
    track_status.event_counter++;
    if(track_status.route_index >= track_status.route_count)
    {
        track_status.auto_active = 0U;
        track_status.paused = 0U;
        point_test_clear_heading_target();
        track_set_state(TRACK_CAL_COMPLETE);
    }
    else
    {
        track_status.paused = 0U;
        track_state_tick = pit_count;
        track_set_state(TRACK_CAL_WAIT_ORIGIN);
    }
    return 1U;
}

uint8 track_calibration_start_stationary(uint16 seconds)
{
    vision_link_snapshot_t vision;
    int32 yaw_x10;

    if(track_is_active() || (seconds != 10U && seconds != 30U)) return 0U;
    point_test_emergency_stop();
    vision_link_get_snapshot(&vision);
    if(!vision_link_is_online() || !vision.pose_valid) return 0U;
    yaw_x10 = track_round_x10(imu963ra_yaw_angle);
    track_status.stationary_duration_ms = (uint32)seconds * 1000U;
    track_status.stationary_elapsed_ms = 0U;
    track_status.stationary_frames = 0U;
    track_status.stationary_start_x10 = vision.car_x_mm;
    track_status.stationary_start_y10 = vision.car_y_mm;
    track_status.stationary_min_x10 = vision.car_x_mm;
    track_status.stationary_max_x10 = vision.car_x_mm;
    track_status.stationary_min_y10 = vision.car_y_mm;
    track_status.stationary_max_y10 = vision.car_y_mm;
    track_status.stationary_yaw_min_x10 = yaw_x10;
    track_status.stationary_yaw_max_x10 = yaw_x10;
    track_last_stationary_pos_packets = vision.pos_packets;
    track_state_tick = pit_count;
    track_status.last_result = TRACK_CAL_RESULT_NONE;
    track_set_state(TRACK_CAL_STATIONARY);
    return 1U;
}

uint8 track_calibration_start_map_scan(void)
{
    vision_link_snapshot_t vision;

    if(track_is_active() || !vision_link_is_online()) return 0U;
    point_test_emergency_stop();
    vision_link_get_snapshot(&vision);
    memset(&track_map_reference, 0, sizeof(track_map_reference));
    track_status.map_scan_received = 0U;
    track_status.map_scan_target = TRACK_CAL_MAP_FRAMES;
    track_status.map_scan_cell_disagreements = 0U;
    track_status.map_scan_max_frame_disagreements = 0U;
    track_status.map_scan_reference_version = 0U;
    track_status.map_scan_last_version = vision.map_version;
    track_map_version_before_request = vision.map_version;
    track_map_request_tick = pit_count;
    track_status.last_result = TRACK_CAL_RESULT_NONE;
    vision_link_request_full_map();
    track_set_state(TRACK_CAL_MAP_SCAN);
    return 1U;
}

uint8 track_calibration_set_direction(uint8 direction)
{
    if(track_is_active() || (direction != 0U && direction != 2U &&
       direction != 4U && direction != 6U)) return 0U;
    track_status.manual_direction = direction;
    track_status.event_counter++;
    return 1U;
}

uint8 track_calibration_set_cells(uint8 cells)
{
    if(track_is_active() || cells < 1U || cells > 4U) return 0U;
    track_status.manual_cells = cells;
    track_status.event_counter++;
    return 1U;
}

uint8 track_calibration_set_speed(uint16 speed)
{
    if(track_is_active() || (speed != 100U && speed != 120U && speed != 150U))
        return 0U;
    track_status.speed = speed;
    track_status.event_counter++;
    return 1U;
}

uint8 track_calibration_set_repeats(uint8 repeats)
{
    if(track_is_active() || (repeats != 1U && repeats != 3U &&
       repeats != 5U && repeats != 10U)) return 0U;
    track_status.manual_repeats = repeats;
    track_status.event_counter++;
    return 1U;
}

uint8 track_calibration_set_load(track_cal_load_t load)
{
    if(track_is_active() || load > TRACK_CAL_LOAD_BOMB) return 0U;
    track_status.load = load;
    track_status.event_counter++;
    return 1U;
}

uint8 track_calibration_set_voltage_x10(uint16 voltage_x10)
{
    if(track_is_active() || voltage_x10 < 100U || voltage_x10 > 140U) return 0U;
    track_status.voltage_x10 = voltage_x10;
    track_status.event_counter++;
    return 1U;
}

uint8 track_calibration_set_profile(track_cal_profile_t profile)
{
    if(track_is_active() || profile < TRACK_CAL_PROFILE_QUICK ||
       profile > TRACK_CAL_PROFILE_FULL) return 0U;
    track_status.selected_profile = profile;
    track_status.event_counter++;
    return 1U;
}

void track_calibration_set_telemetry(uint8 enabled)
{
    track_status.telemetry_enabled = enabled ? 1U : 0U;
    track_status.event_counter++;
}

void track_calibration_get_snapshot(track_cal_snapshot_t *out)
{
    if(out == 0) return;
    point_test_get_snapshot(&track_status.point);
    *out = track_status;
}

const char *track_calibration_state_name(track_cal_state_t state)
{
    switch(state)
    {
        case TRACK_CAL_LOCKED: return "LOCKED";
        case TRACK_CAL_IDLE: return "IDLE";
        case TRACK_CAL_WAIT_ORIGIN: return "WAIT_ORIGIN";
        case TRACK_CAL_RUNNING: return "RUNNING";
        case TRACK_CAL_INTER_STEP: return "INTER_STEP";
        case TRACK_CAL_PAUSED: return "PAUSED";
        case TRACK_CAL_COMPLETE: return "COMPLETE";
        case TRACK_CAL_FAULT: return "FAULT";
        case TRACK_CAL_STATIONARY: return "STATIONARY";
        case TRACK_CAL_MAP_SCAN: return "MAP_SCAN";
        default: return "UNKNOWN";
    }
}

const char *track_calibration_profile_name(track_cal_profile_t profile)
{
    switch(profile)
    {
        case TRACK_CAL_PROFILE_MANUAL: return "MANUAL";
        case TRACK_CAL_PROFILE_QUICK: return "QUICK12";
        case TRACK_CAL_PROFILE_STANDARD: return "STANDARD60";
        case TRACK_CAL_PROFILE_FULL: return "FULL180";
        default: return "UNKNOWN";
    }
}

const char *track_calibration_load_name(track_cal_load_t load)
{
    switch(load)
    {
        case TRACK_CAL_LOAD_FREE: return "FREE";
        case TRACK_CAL_LOAD_BOX: return "BOX";
        case TRACK_CAL_LOAD_BOMB: return "BOMB";
        default: return "UNKNOWN";
    }
}

const char *track_calibration_result_name(track_cal_result_t result)
{
    switch(result)
    {
        case TRACK_CAL_RESULT_NONE: return "NONE";
        case TRACK_CAL_RESULT_OK: return "OK";
        case TRACK_CAL_RESULT_POINT_FAULT: return "POINT_FAULT";
        case TRACK_CAL_RESULT_VISION_WAIT_TIMEOUT: return "VISION_TIMEOUT";
        case TRACK_CAL_RESULT_ABORTED: return "ABORTED";
        case TRACK_CAL_RESULT_SKIPPED: return "SKIPPED";
        default: return "UNKNOWN";
    }
}
