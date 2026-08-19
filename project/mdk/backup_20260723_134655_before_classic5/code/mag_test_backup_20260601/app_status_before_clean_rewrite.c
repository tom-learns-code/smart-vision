#include "zf_common_headfile.h"
#include "vofa.h"
#include "motion_control.h"
#include "app_status.h"

#define APP_STATUS_UART          (DEBUG_UART_INDEX)`r`n#define MAG_TEST_ENABLE          (1U)`r`n#define MAG_TEST_PI              (3.1415926f)`r`n#define MAG_TEST_RAD_TO_DEG      (57.2957795f)

static char ai_tuner_csv_buf[128];`r`nstatic char mag_test_csv_buf[192];`r`n`r`nstatic float mag_test_x_g = 0.0f;`r`nstatic float mag_test_y_g = 0.0f;`r`nstatic float mag_test_z_g = 0.0f;`r`nstatic float mag_test_norm_g = 0.0f;`r`nstatic float mag_test_heading_raw_deg = 0.0f;`r`nstatic float mag_test_heading_cal_deg = 0.0f;`r`nstatic float mag_test_min_x_g = 9999.0f;`r`nstatic float mag_test_max_x_g = -9999.0f;`r`nstatic float mag_test_min_y_g = 9999.0f;`r`nstatic float mag_test_max_y_g = -9999.0f;`r`nstatic float mag_test_min_z_g = 9999.0f;`r`nstatic float mag_test_max_z_g = -9999.0f;`r`n`r`nstatic float app_status_normalize_angle(float angle_deg)`r`n{`r`n    while(angle_deg < 0.0f) angle_deg += 360.0f;`r`n    while(angle_deg >= 360.0f) angle_deg -= 360.0f;`r`n    return angle_deg;`r`n}`r`n`r`nstatic void mag_test_update(void)`r`n{`r`n#if MAG_TEST_ENABLE`r`n    float offset_x;`r`n    float offset_y;`r`n    float span_x;`r`n    float span_y;`r`n    float cal_x;`r`n    float cal_y;`r`n`r`n    if(!imu963ra_ready)`r`n    {`r`n        return;`r`n    }`r`n`r`n    imu963ra_get_mag();`r`n    mag_test_x_g = imu963ra_mag_transition(imu963ra_mag_x);`r`n    mag_test_y_g = imu963ra_mag_transition(imu963ra_mag_y);`r`n    mag_test_z_g = imu963ra_mag_transition(imu963ra_mag_z);`r`n`r`n    if(mag_test_x_g < mag_test_min_x_g) mag_test_min_x_g = mag_test_x_g;`r`n    if(mag_test_x_g > mag_test_max_x_g) mag_test_max_x_g = mag_test_x_g;`r`n    if(mag_test_y_g < mag_test_min_y_g) mag_test_min_y_g = mag_test_y_g;`r`n    if(mag_test_y_g > mag_test_max_y_g) mag_test_max_y_g = mag_test_y_g;`r`n    if(mag_test_z_g < mag_test_min_z_g) mag_test_min_z_g = mag_test_z_g;`r`n    if(mag_test_z_g > mag_test_max_z_g) mag_test_max_z_g = mag_test_z_g;`r`n`r`n    mag_test_norm_g = sqrtf(mag_test_x_g * mag_test_x_g +`r`n                            mag_test_y_g * mag_test_y_g +`r`n                            mag_test_z_g * mag_test_z_g);`r`n    mag_test_heading_raw_deg = app_status_normalize_angle(atan2f(mag_test_y_g, mag_test_x_g) * MAG_TEST_RAD_TO_DEG);`r`n`r`n    offset_x = (mag_test_max_x_g + mag_test_min_x_g) * 0.5f;`r`n    offset_y = (mag_test_max_y_g + mag_test_min_y_g) * 0.5f;`r`n    span_x = mag_test_max_x_g - mag_test_min_x_g;`r`n    span_y = mag_test_max_y_g - mag_test_min_y_g;`r`n`r`n    cal_x = mag_test_x_g - offset_x;`r`n    cal_y = mag_test_y_g - offset_y;`r`n    if(span_x > 0.02f && span_y > 0.02f)`r`n    {`r`n        mag_test_heading_cal_deg = app_status_normalize_angle(atan2f(cal_y, cal_x) * MAG_TEST_RAD_TO_DEG);`r`n    }`r`n    else`r`n    {`r`n        mag_test_heading_cal_deg = mag_test_heading_raw_deg;`r`n    }`r`n#endif`r`n}

void app_status_send_vofa(void)`r`n{`r`n    mag_test_update();
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
    uart_write_string(APP_STATUS_UART, ai_tuner_csv_buf);`r`n`r`n#if MAG_TEST_ENABLE`r`n    snprintf(mag_test_csv_buf, sizeof(mag_test_csv_buf),`r`n             "#MAG,%lu,X,%.4f,Y,%.4f,Z,%.4f,N,%.4f,Hraw,%.1f,Hcal,%.1f,DX,%.4f,DY,%.4f,DZ,%.4f\r\n",`r`n             (unsigned long)(pit_count * 5UL),`r`n             mag_test_x_g,`r`n             mag_test_y_g,`r`n             mag_test_z_g,`r`n             mag_test_norm_g,`r`n             mag_test_heading_raw_deg,`r`n             mag_test_heading_cal_deg,`r`n             mag_test_max_x_g - mag_test_min_x_g,`r`n             mag_test_max_y_g - mag_test_min_y_g,`r`n             mag_test_max_z_g - mag_test_min_z_g);`r`n    uart_write_string(APP_STATUS_UART, mag_test_csv_buf);`r`n#endif
}

void app_status_show_ips200(void)
{
    ips200_show_string(0, 0, "IMU:");
    ips200_show_string(32, 0, imu963ra_ready ? "OK " : "ERR");
    ips200_show_string(64, 0, "H:");
    ips200_show_string(80, 0, imu963ra_yaw_hold_enable ? "ON " : "OFF");
    ips200_show_string(120, 0, "Y:");
    ips200_show_float(136, 0, imu963ra_yaw_angle, 4, 1);

    ips200_show_string(0, 16, "GZ:");
    ips200_show_float(24, 16, imu963ra_gyro_z_dps, 4, 1);
    ips200_show_string(104, 16, "OUT:");
    ips200_show_float(136, 16, imu963ra_yaw_correction, 4, 1);

    ips200_show_string(0, 32, "ERR:");
    ips200_show_float(32, 32, imu963ra_yaw_error, 4, 1);
    ips200_show_string(104, 32, "OFS:");
    ips200_show_float(136, 32, imu963ra_gyro_z_offset, 4, 2);

    ips200_show_string(0, 48, "DST:");
    ips200_show_float(32, 48, odom_displacement_mm / 10.0f, 4, 1);
    ips200_show_string(104, 48, "SUM:");
    ips200_show_float(136, 48, odom_total_distance_mm / 10.0f, 4, 1);

    ips200_show_string(0, 64, "DIR:");
    ips200_show_float(32, 64, odom_move_direction_deg, 3, 1);
    ips200_show_string(104, 64, "NOW:");
    ips200_show_float(136, 64, odom_instant_direction_deg, 3, 1);

    ips200_show_string(0, 80, "X:");
    ips200_show_float(16, 80, odom_world_x_mm / 10.0f, 4, 1);
    ips200_show_string(104, 80, "Y:");
    ips200_show_float(120, 80, odom_world_y_mm / 10.0f, 4, 1);

    ips200_show_string(0, 96, "V:");
    ips200_show_float(16, 96, target_linear_speed, 3, 1);
    ips200_show_string(88, 96, "A:");
    ips200_show_float(104, 96, target_angle, 3, 1);
    ips200_show_string(168, 96, back_forth_mode ? "BF:ON " : "BF:OFF");

    ips200_show_string(0, 112, "S0:");
    ips200_show_int(24, 112, encoder_speed[0], 5);
    ips200_show_string(88, 112, "T0:");
    ips200_show_int(112, 112, (int16)motor0_speed_pid.Target, 5);

    ips200_show_string(0, 128, "S1:");
    ips200_show_int(24, 128, encoder_speed[1], 5);
    ips200_show_string(88, 128, "T1:");
    ips200_show_int(112, 128, (int16)motor1_speed_pid.Target, 5);

    ips200_show_string(0, 144, "S2:");
    ips200_show_int(24, 144, encoder_speed[2], 5);
    ips200_show_string(88, 144, "T2:");
    ips200_show_int(112, 144, (int16)motor2_speed_pid.Target, 5);

    ips200_show_string(0, 160, "C0:");
    ips200_show_int(24, 160, (int16)position_speed_comp[0], 5);
    ips200_show_string(88, 160, "C1:");
    ips200_show_int(112, 160, (int16)position_speed_comp[1], 5);

    ips200_show_string(0, 176, "C2:");
    ips200_show_int(24, 176, (int16)position_speed_comp[2], 5);
    ips200_show_string(88, 176, "D2:");
    ips200_show_int(112, 176, (int32)motor_output_duty[2], 5);

    ips200_show_string(0, 192, "SK:");
    ips200_show_float(24, 192, motor2_speed_pid.Kp, 2, 1);
    ips200_show_string(72, 192, "/");
    ips200_show_float(80, 192, motor2_speed_pid.Ki, 1, 3);
    ips200_show_string(128, 192, "/");
    ips200_show_float(136, 192, motor2_speed_pid.Kd, 1, 1);

    ips200_show_string(0, 208, "PK:");`r`n    ips200_show_float(24, 208, motor0_position_pid.Kp, 1, 1);`r`n    ips200_show_string(72, 208, "/");`r`n    ips200_show_float(80, 208, motor0_position_pid.Ki, 1, 3);`r`n`r`n#if MAG_TEST_ENABLE`r`n    ips200_show_string(0, 224, "MAG:");`r`n    ips200_show_float(32, 224, mag_test_heading_raw_deg, 3, 1);`r`n    ips200_show_string(88, 224, "N:");`r`n    ips200_show_float(104, 224, mag_test_norm_g, 1, 3);`r`n    ips200_show_string(168, 224, "C:");`r`n    ips200_show_float(184, 224, mag_test_heading_cal_deg, 3, 0);`r`n#endif`r`n}