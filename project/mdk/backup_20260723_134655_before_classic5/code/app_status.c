#include <math.h>
#include <stdio.h>
#include "zf_common_headfile.h"
#include "vofa.h"
#include "motion_control.h"
#include "app_status.h"
#include "vision_link.h"

#define APP_STATUS_UART          (DEBUG_UART_INDEX)
#define MAG_TEST_ENABLE          (1U)
#define APP_STATUS_MAG_SCREEN_ONLY (1U)
#define MAG_TEST_RAD_TO_DEG      (57.2957795f)
#define VISION_NAV_DRY_RUN       (0U)
#define VISION_NAV_TARGET_X10    (20)
#define VISION_NAV_TARGET_Y10    (60)
#define VISION_NAV_SPEED_CMD     (200)
#define VISION_NAV_STOP_DIST     (0.15f)

static char ai_tuner_csv_buf[128];
static char mag_test_csv_buf[192];
static char vision_link_text_buf[480];
static char vision_link_raw_buf[32];

static uint8 vision_nav_link_ok = 0;
static uint8 vision_nav_map_ok = 0;
static uint8 vision_nav_ready = 0;
static int32 vision_nav_cmd_angle_x10 = 0;
static int32 vision_nav_cmd_speed = 0;

static float mag_test_x_g = 0.0f;
static float mag_test_y_g = 0.0f;
static float mag_test_z_g = 0.0f;
static float mag_test_norm_g = 0.0f;
static float mag_test_heading_raw_deg = 0.0f;
static float mag_test_heading_cal_deg = 0.0f;
static float mag_test_min_x_g = 9999.0f;
static float mag_test_max_x_g = -9999.0f;
static float mag_test_min_y_g = 9999.0f;
static float mag_test_max_y_g = -9999.0f;
static float mag_test_min_z_g = 9999.0f;
static float mag_test_max_z_g = -9999.0f;
static uint8 mag_test_valid = 0;

static float app_status_normalize_angle(float angle_deg)
{
    while(angle_deg < 0.0f) angle_deg += 360.0f;
    while(angle_deg >= 360.0f) angle_deg -= 360.0f;
    return angle_deg;
}

static void app_status_update_vision_nav(void)
{
    vision_link_snapshot_t s;
    vision_link_map_t map;
    int32 car_x10;
    int32 car_y10;
    int32 dx10;
    int32 dy10;
    float dx_grid;
    float dy_grid;
    float dist_grid;
    float cmd_angle_f;

    vision_link_get_snapshot(&s);
    car_x10 = (int32)s.car_x_mm;
    car_y10 = (int32)s.car_y_mm;

    vision_nav_link_ok = (uint8)(vision_link_is_online() && s.pos_packets > 0U &&
                                 s.checksum_errors == 0U && s.format_errors == 0U);
    vision_nav_map_ok = (uint8)(s.map_packets > 0U && s.map_valid &&
                                s.map_width == VISION_LINK_GRID_W &&
                                s.map_height == VISION_LINK_GRID_H &&
                                (s.wall_count > 0U || s.goal_count > 0U ||
                                 s.box_count > 0U || s.bomb_count > 0U));
    vision_nav_ready = (uint8)(vision_nav_link_ok && vision_nav_map_ok &&
                               vision_link_get_map(&map));

    if(vision_nav_ready)
    {
        dx10 = VISION_NAV_TARGET_X10 - car_x10;
        dy10 = VISION_NAV_TARGET_Y10 - car_y10;
        dx_grid = (float)dx10 * 0.1f;
        dy_grid = (float)dy10 * 0.1f;
        dist_grid = sqrtf(dx_grid * dx_grid + dy_grid * dy_grid);
        cmd_angle_f = app_status_normalize_angle(atan2f(dy_grid, dx_grid) * MAG_TEST_RAD_TO_DEG);
        vision_nav_cmd_angle_x10 = (int32)(cmd_angle_f * 10.0f + 0.5f);
        vision_nav_cmd_speed = (dist_grid < VISION_NAV_STOP_DIST) ? 0 : VISION_NAV_SPEED_CMD;
    }
    else
    {
        vision_nav_cmd_angle_x10 = 0;
        vision_nav_cmd_speed = 0;
    }

#if VISION_NAV_DRY_RUN
    (void)cmd_angle_f;
#else
    if(vision_nav_ready && vision_nav_cmd_speed > 0)
    {
        motion_set_velocity(((float)vision_nav_cmd_angle_x10) * 0.1f,
                            (float)vision_nav_cmd_speed);
    }
    else
    {
        motion_fast_brake();
    }
#endif
}

static void mag_test_update(void)
{
#if MAG_TEST_ENABLE
    float offset_x;
    float offset_y;
    float span_x;
    float span_y;
    float cal_x;
    float cal_y;

    if(!imu963ra_ready)
    {
        mag_test_valid = 0;
        return;
    }

    if(target_linear_speed > 1.0f || target_linear_speed < -1.0f)
    {
        return;
    }

    imu963ra_get_mag();
    mag_test_valid = 1;
    mag_test_x_g = imu963ra_mag_transition(imu963ra_mag_x);
    mag_test_y_g = imu963ra_mag_transition(imu963ra_mag_y);
    mag_test_z_g = imu963ra_mag_transition(imu963ra_mag_z);

    if(mag_test_x_g < mag_test_min_x_g) mag_test_min_x_g = mag_test_x_g;
    if(mag_test_x_g > mag_test_max_x_g) mag_test_max_x_g = mag_test_x_g;
    if(mag_test_y_g < mag_test_min_y_g) mag_test_min_y_g = mag_test_y_g;
    if(mag_test_y_g > mag_test_max_y_g) mag_test_max_y_g = mag_test_y_g;
    if(mag_test_z_g < mag_test_min_z_g) mag_test_min_z_g = mag_test_z_g;
    if(mag_test_z_g > mag_test_max_z_g) mag_test_max_z_g = mag_test_z_g;

    mag_test_norm_g = sqrtf(mag_test_x_g * mag_test_x_g +
                            mag_test_y_g * mag_test_y_g +
                            mag_test_z_g * mag_test_z_g);
    mag_test_heading_raw_deg = app_status_normalize_angle(atan2f(mag_test_y_g, mag_test_x_g) * MAG_TEST_RAD_TO_DEG);

    offset_x = (mag_test_max_x_g + mag_test_min_x_g) * 0.5f;
    offset_y = (mag_test_max_y_g + mag_test_min_y_g) * 0.5f;
    span_x = mag_test_max_x_g - mag_test_min_x_g;
    span_y = mag_test_max_y_g - mag_test_min_y_g;

    cal_x = mag_test_x_g - offset_x;
    cal_y = mag_test_y_g - offset_y;
    if(span_x > 0.02f && span_y > 0.02f)
    {
        mag_test_heading_cal_deg = app_status_normalize_angle(atan2f(cal_y, cal_x) * MAG_TEST_RAD_TO_DEG);
    }
    else
    {
        mag_test_heading_cal_deg = mag_test_heading_raw_deg;
    }
#endif
}



static void app_status_send_vision_link(void)
{
    static uint32 last_vision_link_send_tick = 0;
    vision_link_snapshot_t s;
    uint32 age_ms;
    int32 car_x10;
    int32 car_y10;
    int32 target_x10 = VISION_NAV_TARGET_X10;
    int32 target_y10 = VISION_NAV_TARGET_Y10;
    uint8 link_ok;
    uint8 map_content_ok;
    uint8 nav_ready;

    app_status_update_vision_nav();

    if((pit_count - last_vision_link_send_tick) < 200U)
    {
        return;
    }
    last_vision_link_send_tick = pit_count;

    vision_link_get_snapshot(&s);
    age_ms = (s.last_packet_tick == 0U) ? 99999UL : ((pit_count - s.last_packet_tick) * 5UL);
    car_x10 = (int32)s.car_x_mm;
    car_y10 = (int32)s.car_y_mm;
    link_ok = vision_nav_link_ok;
    map_content_ok = vision_nav_map_ok;
    nav_ready = vision_nav_ready;

    snprintf(vision_link_raw_buf, sizeof(vision_link_raw_buf),
             "%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned int)(s.raw_count > 0U ? s.raw_bytes[0] : 0U),
             (unsigned int)(s.raw_count > 1U ? s.raw_bytes[1] : 0U),
             (unsigned int)(s.raw_count > 2U ? s.raw_bytes[2] : 0U),
             (unsigned int)(s.raw_count > 3U ? s.raw_bytes[3] : 0U),
             (unsigned int)(s.raw_count > 4U ? s.raw_bytes[4] : 0U),
             (unsigned int)(s.raw_count > 5U ? s.raw_bytes[5] : 0U),
             (unsigned int)(s.raw_count > 6U ? s.raw_bytes[6] : 0U),
             (unsigned int)(s.raw_count > 7U ? s.raw_bytes[7] : 0U));

    snprintf(vision_link_text_buf, sizeof(vision_link_text_buf),
             "VIS id=7100868 %s %s online=%u age=%lu rx=%lu poll=%lu parse=%lu drop=%lu pos=%lu map=%lu ver=%lu req=%lu err=%lu/%lu rawn=%u raw=%s frame=%u car=(%ld.%ld,%ld.%ld) theta=%d.%d box=%u goal=%u bomb=%u wall=%u nav=%s tgt=(%ld.%ld,%ld.%ld) cmd=(%ld.%ld,%ld)\r\n",
             link_ok ? "LINK_OK" : "LINK_WAIT",
             map_content_ok ? "MAP_OK" : "MAP_EMPTY",
             (unsigned int)vision_link_is_online(),
             (unsigned long)age_ms,
             (unsigned long)s.rx_bytes,
             (unsigned long)s.poll_bytes,
             (unsigned long)s.parsed_bytes,
             (unsigned long)s.ring_drops,
             (unsigned long)s.pos_packets,
             (unsigned long)s.map_packets,
             (unsigned long)s.map_version,
             (unsigned long)s.map_request_count,
             (unsigned long)s.checksum_errors,
             (unsigned long)s.format_errors,
             (unsigned int)s.raw_count,
             vision_link_raw_buf,
             (unsigned int)s.frame_id,
             (long)(car_x10 / 10L),
             (long)((car_x10 < 0L ? -car_x10 : car_x10) % 10L),
             (long)(car_y10 / 10L),
             (long)((car_y10 < 0L ? -car_y10 : car_y10) % 10L),
             (int)(s.car_theta_x10 / 10),
             (int)((s.car_theta_x10 < 0 ? -s.car_theta_x10 : s.car_theta_x10) % 10),
             (unsigned int)s.box_count,
             (unsigned int)s.goal_count,
             (unsigned int)s.bomb_count,
             (unsigned int)s.wall_count,
             nav_ready ? (VISION_NAV_DRY_RUN ? "DRY" : "RUN") : "WAIT",
             (long)(target_x10 / 10L),
             (long)((target_x10 < 0L ? -target_x10 : target_x10) % 10L),
             (long)(target_y10 / 10L),
             (long)((target_y10 < 0L ? -target_y10 : target_y10) % 10L),
             (long)(vision_nav_cmd_angle_x10 / 10L),
             (long)((vision_nav_cmd_angle_x10 < 0L ? -vision_nav_cmd_angle_x10 : vision_nav_cmd_angle_x10) % 10L),
             (long)vision_nav_cmd_speed);
    uart_write_string(APP_STATUS_UART, vision_link_text_buf);
}
void app_status_send_vofa(void)
{
#if VISION_LINK_SCREEN_ENABLE
    app_status_send_vision_link();
    return;
#endif
    mag_test_update();

    JF_Data.data[0] = position_error[2];
    JF_Data.data[1] = position_speed_comp[2];
    JF_Data.data[2] = (float)motor2_speed_pid.Target;
    JF_Data.data[3] = (float)encoder_speed[2];
    JF_Data.data[4] = position_actual[2];
    JF_Data.data[5] = (float)motor_output_duty[2];

    snprintf(ai_tuner_csv_buf, sizeof(ai_tuner_csv_buf),
             "%lu,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\r\n",
             (unsigned long)(pit_count * 5UL),
             motor2_speed_pid.Target,
             (float)encoder_speed[2],
             (float)motor_output_duty[2],
             motor2_speed_pid.Error0,
             motor2_speed_pid.Kp,
             motor2_speed_pid.Ki,
             motor2_speed_pid.Kd);
    uart_write_string(APP_STATUS_UART, ai_tuner_csv_buf);

#if MAG_TEST_ENABLE
    if(!mag_test_valid)
    {
        snprintf(mag_test_csv_buf, sizeof(mag_test_csv_buf),
                 "#MAG,%lu,NOT_READY\r\n",
                 (unsigned long)(pit_count * 5UL));
        uart_write_string(APP_STATUS_UART, mag_test_csv_buf);
        return;
    }

    snprintf(mag_test_csv_buf, sizeof(mag_test_csv_buf),
             "#MAG,%lu,X,%.4f,Y,%.4f,Z,%.4f,N,%.4f,Hraw,%.1f,Hcal,%.1f,DX,%.4f,DY,%.4f,DZ,%.4f\r\n",
             (unsigned long)(pit_count * 5UL),
             mag_test_x_g,
             mag_test_y_g,
             mag_test_z_g,
             mag_test_norm_g,
             mag_test_heading_raw_deg,
             mag_test_heading_cal_deg,
             mag_test_max_x_g - mag_test_min_x_g,
             mag_test_max_y_g - mag_test_min_y_g,
             mag_test_max_z_g - mag_test_min_z_g);
    uart_write_string(APP_STATUS_UART, mag_test_csv_buf);
#endif
}

void app_status_show_ips200(void)
{
#if VISION_LINK_SCREEN_ENABLE
    vision_link_show_ips200();
    return;
#endif
#if MAG_TEST_ENABLE && APP_STATUS_MAG_SCREEN_ONLY
    static uint8 screen_cleared = 0;

    ips200_set_font(IPS200_6X8_FONT);
    mag_test_update();
    if(!screen_cleared)
    {
        ips200_clear();
        screen_cleared = 1;
    }

    ips200_show_string(0, 0, "MAG");
    ips200_show_string(36, 0, "IMU:");
    ips200_show_string(60, 0, imu963ra_ready ? "OK " : "ERR");
    ips200_show_string(90, 0, "PIT:");
    ips200_show_uint(120, 0, pit_count, 6);
    ips200_show_string(168, 0, "V:");
    ips200_show_int(184, 0, (int32)target_linear_speed, 4);

    ips200_show_string(0, 10, "YAW:");
    ips200_show_float(30, 10, imu963ra_yaw_angle, 4, 1);

    if(!mag_test_valid)
    {
        ips200_show_string(84, 10, "RAW: ---- ");
        ips200_show_string(162, 10, "CAL: ---- ");
    }
    else
    {
        ips200_show_string(84, 10, "RAW:");
        ips200_show_float(114, 10, mag_test_heading_raw_deg, 3, 1);
        ips200_show_string(162, 10, "CAL:");
        ips200_show_float(192, 10, mag_test_heading_cal_deg, 3, 1);
    }

    ips200_show_string(0, 20, "X:");
    ips200_show_float(18, 20, mag_test_x_g, 2, 4);
    ips200_show_string(80, 20, "Y:");
    ips200_show_float(98, 20, mag_test_y_g, 2, 4);
    ips200_show_string(160, 20, "Z:");
    ips200_show_float(178, 20, mag_test_z_g, 2, 4);

    ips200_show_string(0, 30, "N:");
    ips200_show_float(18, 30, mag_test_norm_g, 1, 4);
    ips200_show_string(80, 30, "DX:");
    ips200_show_float(104, 30, mag_test_max_x_g - mag_test_min_x_g, 1, 4);
    ips200_show_string(160, 30, "DY:");
    ips200_show_float(184, 30, mag_test_max_y_g - mag_test_min_y_g, 1, 4);

    ips200_show_string(0, 40, "DZ:");
    ips200_show_float(24, 40, mag_test_max_z_g - mag_test_min_z_g, 1, 4);

    ips200_show_string(0, 50, "T0:");
    ips200_show_int(24, 50, wheel_target_speed[0], 5);
    ips200_show_string(72, 50, "D0:");
    ips200_show_int(96, 50, motor_output_duty[0], 5);

    ips200_show_string(0, 60, "T1:");
    ips200_show_int(24, 60, wheel_target_speed[1], 5);
    ips200_show_string(72, 60, "D1:");
    ips200_show_int(96, 60, motor_output_duty[1], 5);

    ips200_show_string(0, 70, "T2:");
    ips200_show_int(24, 70, wheel_target_speed[2], 5);
    ips200_show_string(72, 70, "D2:");
    ips200_show_int(96, 70, motor_output_duty[2], 5);

    ips200_show_string(0, 80, "E2:");
    ips200_show_int(24, 80, (int32)position_error[2], 5);
    ips200_show_string(72, 80, "C2:");
    ips200_show_int(96, 80, (int32)position_speed_comp[2], 5);

    // ---- 五角星轨迹进度 ----
    {
        static const float penta_dir_disp[5] = { 288.0f, 144.0f, 0.0f, 216.0f, 72.0f };

        ips200_show_string(0, 92, "PENTA:");
        if(pentagram_state == PENTA_IDLE)
        {
            ips200_show_string(48, 92, pentagram_enable ? "WAIT" : "OFF ");
        }
        else if(pentagram_state >= PENTA_EDGE_0 && pentagram_state <= PENTA_EDGE_4)
        {
            ips200_show_string(48, 92, "EDGE ");
            ips200_show_uint(84, 92, (uint32)(pentagram_state - PENTA_EDGE_0 + 1), 1);
            ips200_show_string(92, 92, "/5");
            ips200_show_string(120, 92, "D:");
            ips200_show_float(136, 92, penta_dir_disp[pentagram_state - PENTA_EDGE_0], 3, 1);
        }
        else if(pentagram_state >= PENTA_PAUSE_0 && pentagram_state <= PENTA_PAUSE_4)
        {
            ips200_show_string(48, 92, "TURN ");
            ips200_show_uint(84, 92, (uint32)(pentagram_state - PENTA_PAUSE_0 + 1), 1);
        }
        else if(pentagram_state == PENTA_DONE)
        {
            ips200_show_string(48, 92, "DONE");
        }

        // 显示里程计位移，观察是否回到原点
        ips200_show_string(0, 102, "X:");
        ips200_show_float(16, 102, odom_world_x_mm / 10.0f, 4, 1);
        ips200_show_string(80, 102, "Y:");
        ips200_show_float(96, 102, odom_world_y_mm / 10.0f, 4, 1);
        ips200_show_string(0, 112, "DIS:");
        ips200_show_float(32, 112, odom_displacement_mm / 10.0f, 4, 1);
        ips200_show_string(100, 112, "SUM:");
        ips200_show_float(128, 112, odom_total_distance_mm / 10.0f, 4, 1);
    }
#else
    ips200_show_string(0, 0, "MAG SCREEN OFF");
#endif
}
