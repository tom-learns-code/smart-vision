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
* 许可申明副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          motor_driver_test
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2025-03-19        User                电机驱动测试程序
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "pid.h"
#include "vofa.h"
#include "isr.h"
#include "blue.h"

//========================================================= 电机驱动引脚定义 =========================================================
//C6是后轮，C8是左C10是右
// 电机PWM引脚（根据推荐的引脚分配）
#define PWM_C6              (PWM2_MODULE0_CHA_C6)
#define PWM_C10             (PWM2_MODULE2_CHA_C10)
#define PWM_C8              (PWM2_MODULE1_CHA_C8)

// 串口引脚（使用DEBUG串口）
#define TEST_UART           (DEBUG_UART_INDEX)

//========================================================= 测试参数 =========================================================

#define PWM_FREQUENCY       (17000)     // PWM频率 17kHz
#define PWM_DUTY_MAX        (10000)    // PWM占空比最大值
#define CIRCLE_LINEAR_SPEED (180.0f)   // 圆圈平移速度，速度太低时轨迹会发涩

//========================================================= 辅助函数 =========================================================

//========================================================= 电机驱动测试函数 =========================================================

// 初始化电机PWM
void motor_pwm_init(void)
{
    pwm_init(PWM_C6, PWM_FREQUENCY, 0);
    pwm_init(PWM_C10, PWM_FREQUENCY, 0);
    pwm_init(PWM_C8, PWM_FREQUENCY, 0);
}

//========================================================= Y车模PID速度环+位置环闭环控制 =========================================================

// 目标角度和速度（在isr.c中定义）
extern float target_angle;//extern 表示在另一个文件里面定义的
extern float target_linear_speed;//linear表示直线，线性


// 编码器速度数据（在isr.c中定义，用于显示）
extern int16 encoder_speed[3];
// 速度环PID输出（用于显示）
extern PID_t motor0_speed_pid;
extern PID_t motor1_speed_pid;
extern PID_t motor2_speed_pid;
// 位置环PID（用于显示）
extern PID_t motor0_position_pid;
extern PID_t motor1_position_pid;
extern PID_t motor2_position_pid;
// 运动学解算后的目标速度（用于显示）
extern int16 wheel_target_speed[3];
// 位置环数据
extern float position_error[3];//含义：目标位置 - 实际位置 = 应走但没走到的差距
extern float position_speed_comp[3];//位置环速度补偿,位置环PID输出的速度修正量，叠加到速度环目标上
extern float position_target[3];//含义：三轮各自"应该走过的总脉冲数"
                                //单位：编码器5ms脉冲数
extern float position_actual[3];//三轮各自"实际走过的总脉冲数"
extern int16 motor_output_duty[3];//经过"静止回正"判断后，实际下发给电机的PWM占空比

// 中断运行计数器（验证PIT中断是否正常触发）
extern volatile uint32 pit_count;

static char ai_tuner_csv_buf[128];






//========================================================= 主函数 =========================================================

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    // Y车模相关变量
    uart_write_string(TEST_UART, "#AI_TUNER_BOOT\r\n");
    target_angle = 0.0f;            // 转圈模式起始方向角
    back_forth_mode = 0;            // 关闭开机自动前后循环跑
    target_linear_speed = CIRCLE_LINEAR_SPEED;

    // 初始化方向控制引脚
    gpio_init(C7, GPO, 0, GPO_PUSH_PULL);
    gpio_init(C11, GPO, 0, GPO_PUSH_PULL);
    gpio_init(C9, GPO, 0, GPO_PUSH_PULL);
    motor_pwm_init();
	  wireless_uart_init();
    if(imu963ra_heading_init() == 0)
    {
        uart_write_string(TEST_UART, "#IMU963RA_READY\r\n");
    }
    else
    {
        uart_write_string(TEST_UART, "#IMU963RA_INIT_FAIL\r\n");
    }
    odometry_reset();

    // 初始化蓝牙串口（UART4, 9600波特率, C16TX/C17RX）
    // 注意：如果没接蓝牙模块，UART4 RX悬空会收到乱码导致参数被意外修改！
    // 没接蓝牙时请注释掉下面3行，接了蓝牙后再取消注释
    //uart_init(UART_4, 9600, UART4_TX_C16, UART4_RX_C17);
    //uart_rx_interrupt(UART_4, 1);
    //interrupt_set_priority(LPUART4_IRQn, 2);

    //system_delay_ms(300);  // 去掉多余延时，加快启动
	
    ips200_set_dir(IPS200_PORTAIT);//设置屏幕方向
    ips200_set_font(IPS200_8X16_FONT);//设置字体
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);//设置前景/背景色
    ips200_init(IPS200_TYPE_SPI);


    // 初始化编码器
    encoder_quad_init(QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C4);
    encoder_quad_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
    encoder_quad_init(QTIMER2_ENCODER2, QTIMER2_ENCODER2_CH1_C5, QTIMER2_ENCODER2_CH2_C25);

    // 初始化PIT定时器（5ms周期，用于速度环控制）
    pit_ms_init(PIT_CH0, 5);

    // 使能全局中断
    interrupt_global_enable(0);

    // 用GPT定时器精确计时（比system_delay_ms更准确）
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);//启动GPT定时器,启动后 timer_get() 才能读到有效值
    uint32 last_tick = timer_get(GPT_TIM_1);//记录起始时刻
    uint32 last_vofa_tick = 0;// VOFA发送时刻
    uint32 last_display_tick = 0;//屏幕刷新时刻
		//motor_control_5ms() 必须严格5ms间隔，用硬件定时器保证精度。VOFA和屏幕刷新不需要那么精确，用 pit_count 计数就够了。
    //  wireless_uart_send_string("\n");
    //   printf("%d\n",motor0_speed_pid.Out);
    //system_delay_ms(500);  // 去掉多余延时，加快启动

    // 确保启动时未停车（防止UART4初始化时收到乱码触发停车）
    device_init_flag = 0;//停车标志，定义在 blue.c 第6行：

    while(1)
    {
//			 gpio_set_level(C7, 1);
//			  pwm_set_duty(PWM2_MODULE0_CHA_C6, 3000);
//			gpio_set_level(C9, 0);pwm_set_duty(PWM2_MODULE1_CHA_C8, 3000);
//			gpio_set_level(C11, 0);pwm_set_duty(PWM2_MODULE2_CHA_C10, 3000);//右轮不一样
			
			
        ai_tuner_poll_command();

        uint32 now = timer_get(GPT_TIM_1);// 读取当前时刻(ms)
        while(now - last_tick >= 5)// 距上次执行已≥5ms？
        {
            last_tick += 5;// 推进基准时刻5ms
            // motor_control_5ms() now runs from PIT_IRQHandler.
        }

        if((pit_count - last_vofa_tick) >= 10)
        {
            last_vofa_tick = pit_count;

            // VOFA每50ms发一次，避免串口发送阻塞motor_control的5ms周期
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
            uart_write_string(TEST_UART, ai_tuner_csv_buf);
        }

        // 每20次（100ms）刷新屏幕
        if((pit_count - last_display_tick) >= 20)
        {
            last_display_tick = pit_count;

            // 速度环参数  SK:Kp/Ki/Kd
#if 0
            ips200_show_string(0, 0, "SK:");
            ips200_show_float(24, 0, motor1_speed_pid.Kp, 2, 1);
            ips200_show_string(72, 0, "/");
            ips200_show_float(80, 0, motor1_speed_pid.Ki, 1, 3);
            ips200_show_string(120, 0, "/");
            ips200_show_float(128, 0, motor1_speed_pid.Kd, 1, 1);

            // 前馈参数（已注释）
            //ips200_show_string(0, 18, "FK:");
            //ips200_show_float(24, 18, feedforward_K, 2, 1);
            //ips200_show_string(80, 18, "/");
            //ips200_show_float(88, 18, feedforward_bias, 2, 1);

            // 位置环参数  PK:Kp/Ki
            ips200_show_string(0, 36, "PK:");
            ips200_show_float(24, 36, motor0_position_pid.Kp, 1, 1);
            ips200_show_string(72, 36, "/");
            ips200_show_float(80, 36, motor0_position_pid.Ki, 1, 3);

            // 编码器速度 / 目标速度（后轮0）
            ips200_show_string(0, 54, "S0:");
            ips200_show_int(24, 54, encoder_speed[0], 5);
            ips200_show_string(80, 54, "/");
            ips200_show_int(88, 54, (int16)motor0_speed_pid.Target, 5);

            // 编码器速度 / 目标速度（右前轮2）
            ips200_show_string(0, 72, "S2:");
            ips200_show_int(24, 72, encoder_speed[2], 5);
            ips200_show_string(80, 72, "/");
            ips200_show_int(88, 72, (int16)motor2_speed_pid.Target, 5);

            // 位置误差（后轮0 / 右前轮2）
            ips200_show_string(0, 88, "E0:");
            ips200_show_int(24, 88, (int32)position_error[0], 7);
            ips200_show_string(0, 106, "E2:");
            ips200_show_int(24, 106, (int32)position_error[2], 7);

            // 位置环速度补偿（后轮0 / 右前轮2）
            ips200_show_string(0, 122, "C0:");
            ips200_show_int(24, 122, (int16)position_speed_comp[0], 5);
            ips200_show_string(0, 140, "C2:");
            ips200_show_int(24, 140, (int16)position_speed_comp[2], 5);

            // 目标速度和目标角度
            ips200_show_string(0, 156, "V:");
            ips200_show_float(16, 156, target_linear_speed, 3, 1);
            ips200_show_string(80, 156, "A:");
            ips200_show_float(96, 156, target_angle, 3, 1);

            // 来回跑模式状态
            ips200_show_string(0, 174, back_forth_mode ? "BF:ON " : "BF:OFF");
            if(back_forth_mode)
            {
                ips200_show_float(50, 174, back_forth_speed, 3, 1);
            }

            ips200_show_string(0, 192, "IMU963RA:");
            ips200_show_string(72, 192, imu963ra_ready ? "OK " : "ERR");
            ips200_show_string(112, 192, "H:");
            ips200_show_string(128, 192, imu963ra_yaw_hold_enable ? "ON " : "OFF");

            ips200_show_string(0, 210, "YAW:");
            ips200_show_float(32, 210, imu963ra_yaw_angle, 5, 1);
            ips200_show_string(104, 210, "GZ:");
            ips200_show_float(128, 210, imu963ra_gyro_z_dps, 4, 1);

            ips200_show_string(0, 228, "ERR:");
            ips200_show_float(32, 228, imu963ra_yaw_error, 4, 1);
            ips200_show_string(104, 228, "OUT:");
            ips200_show_float(136, 228, imu963ra_yaw_correction, 4, 1);

            ips200_show_string(0, 246, "OFS:");
            ips200_show_float(32, 246, imu963ra_gyro_z_offset, 4, 2);

            ips200_show_string(0, 264, "DST:");
            ips200_show_float(32, 264, odom_displacement_mm / 10.0f, 4, 1);
            ips200_show_string(104, 264, "SUM:");
            ips200_show_float(136, 264, odom_total_distance_mm / 10.0f, 4, 1);

            ips200_show_string(0, 282, "DIR:");
            ips200_show_float(32, 282, odom_move_direction_deg, 3, 1);
            ips200_show_string(104, 282, "NOW:");
            ips200_show_float(136, 282, odom_instant_direction_deg, 3, 1);

            ips200_show_string(0, 300, "X:");
            ips200_show_float(16, 300, odom_world_x_mm / 10.0f, 4, 1);
            ips200_show_string(104, 300, "Y:");
            ips200_show_float(120, 300, odom_world_y_mm / 10.0f, 4, 1);
#endif
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

            ips200_show_string(0, 208, "PK:");
            ips200_show_float(24, 208, motor0_position_pid.Kp, 1, 1);
            ips200_show_string(72, 208, "/");
            ips200_show_float(80, 208, motor0_position_pid.Ki, 1, 3);
        }
    }
}
