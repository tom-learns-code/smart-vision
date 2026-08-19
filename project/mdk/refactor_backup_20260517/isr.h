/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/
 


#ifndef _isr_h
#define _isr_h

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
//extern float feedforward_total_out[3];  // 前馈已注释
//extern float feedforward_K;             // 前馈已注释
//extern float feedforward_K3;            // 前馈已注释
//extern float feedforward_bias;          // 前馈已注释
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

// 函数声明
void motor_control_5ms(void);
void position_reset(void);
void p_set(uint8 wheel, int16 duty);
void ai_tuner_poll_command(void);
uint8 imu963ra_heading_init(void);
void imu963ra_heading_reset(void);
void imu963ra_heading_update(void);
void odometry_reset(void);

#endif
