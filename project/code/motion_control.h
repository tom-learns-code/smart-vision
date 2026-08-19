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
extern uint8 control_verify_mode;
extern uint8 control_verify_mode5_limit_enable;

extern float odom_body_delta_x_mm;
extern float odom_body_delta_y_mm;
extern float odom_world_x_mm;
extern float odom_world_y_mm;
extern float odom_total_distance_mm;
extern float odom_displacement_mm;
extern float odom_body_direction_deg;
extern float odom_move_direction_deg;
extern float odom_instant_direction_deg;

#define MOTION_POS_LIMIT_RATIO   (0x01U)
#define MOTION_POS_LIMIT_MAX     (0x02U)
#define MOTION_POS_LIMIT_CLIPPED (0x04U)
#define MOTION_POS_LIMIT_SLEW    (0x08U)

#define MOTION_DUTY_SOURCE_SPEED_PID (0U)
#define MOTION_DUTY_SOURCE_POS_HOLD  (1U)

typedef struct {
    float target_angle_deg;
    float target_speed;
    float yaw_deg;
    float yaw_target_deg;
    float yaw_error_deg;
    float yaw_correction;
    float gyro_z_dps;
    float visual_sync_error_deg;
    float visual_sync_step_deg;
    int16 wheel_target[3];
    int16 wheel_pid_target[3];
    int16 wheel_encoder[3];
    int16 encoder_rotate;
    int16 wheel_pwm[3];
    float position_target[3];
    float position_actual[3];
    float position_error[3];
    float position_raw_output[3];
    float position_limit[3];
    float position_comp[3];
    uint8 position_limit_flags[3];
    float speed_error[3];
    float speed_error_int[3];
    float speed_p_output[3];
    float speed_i_output[3];
    float speed_d_output[3];
    float speed_raw_output[3];
    uint8 duty_source[3];
    float yaw_p_output;
    float yaw_d_output;
    float yaw_raw_output;
    uint8 yaw_saturated;
    uint32 window_samples;
    float window_speed_error_abs_avg[3];
    float window_speed_error_abs_max[3];
    int16 window_pwm_min[3];
    int16 window_pwm_max[3];
    uint32 window_position_clip_count[3];
    uint32 window_position_slew_count[3];
    uint32 window_integral_cap_count[3];
    uint32 window_output_cap_count[3];
    uint32 window_position_hold_count[3];
    uint8 pwm_ramp_active;
    uint16 pwm_ramp_remaining_ms;
    uint16 pwm_ramp_max_delta;
    uint16 pwm_ramp_scale_x1000;
    int16 pwm_ramp_desired[3];
} motion_debug_snapshot_t;

typedef struct {
    uint8 active;
    uint8 applied_mask;
    uint16 remaining_ms;
} motion_startup_assist_status_t;

void motor_pwm_init(void);
void motor_control_5ms(void);
void motion_set_velocity(float angle_deg, float speed);
void motion_heading_lock_begin(void);
void motion_heading_lock_begin_at(float heading_target_deg);
void motion_heading_lock_update(float angle_deg, float speed);
void motion_heading_lock_correct_from_visual(float relative_heading_deg);
void motion_heading_lock_stop(void);
void motion_heading_lock_rebase_position(void);
void motion_heading_lock_release(void);
void motion_startup_assist_begin(uint16 duration_ms);
void motion_startup_assist_cancel(void);
void motion_startup_assist_get_status(motion_startup_assist_status_t *out);
void motion_pwm_ramp_begin(uint16 duration_ms, uint16 max_delta_per_5ms);
void motion_pwm_ramp_cancel(void);
void motion_set_yaw_hold_enable(uint8 enable);
void motion_set_manual_rotation(float speed);
void motion_stop_manual_rotation(void);
void motion_stop_manual_rotation_at(float heading_target_deg);
void motion_get_wheel_total_counts(int32 out[3]);
float motion_get_mm_per_count(void);
float motion_get_wheel_cpr(void);
float motion_get_wheel_diameter_mm(void);
float motion_get_odometry_scale(void);
void motion_get_debug_snapshot(motion_debug_snapshot_t *out);
void motion_fast_brake(void);
void motion_emergency_stop(void);
void position_reset(void);
void p_set(uint8 wheel, int16 duty);
void control_verify_init(uint8 mode);
void control_verify_set_mode5_speed(float speed);
void control_verify_set_mode5_limit(uint8 enable);

void ai_tuner_update_pid(float kp, float ki, float kd);
void ai_tuner_poll_command(void);
void ai_tuner_uart1_rx_handler(void);

uint8 imu963ra_heading_init(void);
void imu963ra_heading_reset(void);
void imu963ra_heading_update(void);
void odometry_reset(void);

extern uint8 pentagram_enable;
extern uint8 pentagram_state;
extern uint8 pentagram_edge_current;
void pentagram_run_task(void);

// 五角星状态机状态（供外部模块读取显示）
#define PENTA_IDLE      0
#define PENTA_EDGE_0    1
#define PENTA_EDGE_1    2
#define PENTA_EDGE_2    3
#define PENTA_EDGE_3    4
#define PENTA_EDGE_4    5
#define PENTA_PAUSE_0   6
#define PENTA_PAUSE_1   7
#define PENTA_PAUSE_2   8
#define PENTA_PAUSE_3   9
#define PENTA_PAUSE_4   10
#define PENTA_DONE      11

#endif
