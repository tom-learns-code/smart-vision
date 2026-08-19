#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "zf_common_headfile.h"
#include "pid.h"

extern int16 encoder_speed[3];
extern int16 wheel_target_speed[3];

extern PID_t motor0_speed_pid;
extern PID_t motor1_speed_pid;
extern PID_t motor2_speed_pid;
extern PID_t motor0_position_pid;
extern PID_t motor1_position_pid;
extern PID_t motor2_position_pid;

extern float position_error[3];
extern float position_speed_comp[3];
extern float position_target[3];
extern float position_actual[3];
extern int16 motor_output_duty[3];

extern float target_angle;
extern float target_linear_speed;
extern volatile uint32 pit_count;
extern int back_forth_mode;
extern float back_forth_speed;
extern uint32 back_forth_period;

extern uint8 imu963ra_ready;
extern uint8 imu963ra_yaw_hold_enable;
extern float imu963ra_gyro_z_offset;
extern float imu963ra_gyro_z_dps;
extern float imu963ra_yaw_angle;
extern float imu963ra_yaw_target;
extern float imu963ra_yaw_error;
extern float imu963ra_yaw_correction;

extern float odom_body_delta_x_mm;
extern float odom_body_delta_y_mm;
extern float odom_world_x_mm;
extern float odom_world_y_mm;
extern float odom_total_distance_mm;
extern float odom_displacement_mm;
extern float odom_body_direction_deg;
extern float odom_move_direction_deg;
extern float odom_instant_direction_deg;

void motor_pwm_init(void);
void motor_control_5ms(void);
void position_reset(void);
void p_set(uint8 wheel, int16 duty);

void ai_tuner_update_pid(float kp, float ki, float kd);
void ai_tuner_poll_command(void);
void ai_tuner_uart1_rx_handler(void);

uint8 imu963ra_heading_init(void);
void imu963ra_heading_reset(void);
void imu963ra_heading_update(void);
void odometry_reset(void);

#endif