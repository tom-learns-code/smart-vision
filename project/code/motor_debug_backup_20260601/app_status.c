#include <math.h>
#include <stdio.h>
#include "zf_common_headfile.h"
#include "vofa.h"
#include "motion_control.h"
#include "app_status.h"

#define APP_STATUS_UART          (DEBUG_UART_INDEX)
#define MAG_TEST_ENABLE          (1U)
#define APP_STATUS_MAG_SCREEN_ONLY (1U)
#define MAG_TEST_RAD_TO_DEG      (57.2957795f)

static char ai_tuner_csv_buf[128];
static char mag_test_csv_buf[192];

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

void app_status_send_vofa(void)
{
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
#if MAG_TEST_ENABLE && APP_STATUS_MAG_SCREEN_ONLY
    static uint8 screen_cleared = 0;

    mag_test_update();
    if(!screen_cleared)
    {
        ips200_clear();
        screen_cleared = 1;
    }

    ips200_show_string(0, 0, "MAG TEST");

    ips200_show_string(0, 16, "IMU:");
    ips200_show_string(32, 16, imu963ra_ready ? "OK " : "ERR");
    ips200_show_string(80, 16, "YAW:");
    ips200_show_float(112, 16, imu963ra_yaw_angle, 4, 1);

    if(!mag_test_valid)
    {
        ips200_show_string(0, 32, "MAG NOT READY");
        return;
    }

    ips200_show_string(0, 32, "RAW:");
    ips200_show_float(40, 32, mag_test_heading_raw_deg, 3, 1);
    ips200_show_string(112, 32, "CAL:");
    ips200_show_float(152, 32, mag_test_heading_cal_deg, 3, 1);

    ips200_show_string(0, 48, "N:");
    ips200_show_float(24, 48, mag_test_norm_g, 1, 4);

    ips200_show_string(0, 64, "X:");
    ips200_show_float(24, 64, mag_test_x_g, 2, 4);
    ips200_show_string(120, 64, "Y:");
    ips200_show_float(144, 64, mag_test_y_g, 2, 4);

    ips200_show_string(0, 80, "Z:");
    ips200_show_float(24, 80, mag_test_z_g, 2, 4);

    ips200_show_string(0, 96, "DX:");
    ips200_show_float(32, 96, mag_test_max_x_g - mag_test_min_x_g, 1, 4);
    ips200_show_string(120, 96, "DY:");
    ips200_show_float(152, 96, mag_test_max_y_g - mag_test_min_y_g, 1, 4);

    ips200_show_string(0, 112, "DZ:");
    ips200_show_float(32, 112, mag_test_max_z_g - mag_test_min_z_g, 1, 4);
#else
    ips200_show_string(0, 0, "MAG SCREEN OFF");
#endif
}