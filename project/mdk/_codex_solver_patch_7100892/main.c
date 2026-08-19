// 读这个工程时，先不要背函数名，按这个顺序想：
// 1. 我要控制什么硬件？
// 2. 这个硬件需要什么信号？GPIO / PWM / UART / SPI / I2C / ADC / 编码器？
// 3. 逐飞库里对应哪个头文件？
// 4. 这个头文件里有哪些 init / read / write / set 函数？
// 5. 工程里有没有类似写法？
// 6. 参数里的引脚、模式、电平，是否和硬件接线一致？

// zf_common_headfile.h 是逐飞库的“总头文件”。
// 包含它以后，GPIO、PWM、UART、PIT、编码器、屏幕、IMU 等常用函数基本都能直接用。
#include <stdio.h>
#include "zf_common_headfile.h"

// 下面三个是你自己项目里的模块，不是逐飞官方库：
// blue.h            蓝牙按键调参/启停
// motion_control.h  电机、编码器、PID、IMU 航向、里程计等运动控制
// app_status.h      串口曲线数据和 IPS200 屏幕显示
#include "blue.h"
#include "motion_control.h"
#include "app_status.h"
#include "vision_link.h"
#include "planner_service.h"
#include "mission_manager.h"
#include "action_follower.h"
#include "pc_console.h"
#include "point_test.h"

#define TEST_UART              (DEBUG_UART_INDEX)   // 调试串口：用来给电脑/上位机发提示和数据
#define DISPLAY_PERIOD_TICKS   (20U)                // 20 次约 100ms 刷一次屏幕
#define BOOT_TRACE_ENABLE      (0U)

// ==================== 用户常用运行配置 ====================
// 总模式开关：0=正式赛道，1=六模式控制验证，2=点对点串口菜单标定。
#define CONTROL_DEBUG_ENABLE   (1U)

// 调试子模式：1=单电机开环，2=三电机开环，3=位置保持，4=航向保持，5=向右平移，6=左前轮方向切换。
#define CONTROL_VERIFY_MODE    (3U)

// 正式赛道 FREE_MOVE 的平移速度，单位是“编码器脉冲/5ms”；当前保持原值100，可手动调大。
#define RACE_FREE_MOVE_SPEED   (120.0f)

// mode1使用：选择单独测试的电机，0=后轮，1=左前轮，2=右前轮。
#define CONTROL_VERIFY_MOTOR   (0U)

// mode1、mode2、mode6使用：开环PWM占空比，范围建议0~10000；测试时应抬起车轮。
#define CONTROL_VERIFY_PWM     (2000U)

// mode5使用：向右平移的闭环目标速度，单位同样是“编码器脉冲/5ms”。
#define CONTROL_VERIFY_MODE5_SPEED (300.0f)

// mode5最终PWM限幅开关：0=关闭限幅，1=启用2500限幅。
#define CONTROL_VERIFY_MODE5_LIMIT_ENABLE (0U)
// ==========================================================
#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x) STRINGIFY_INNER(x)

// 设置小车的初始目标状态。
// 这里不是直接给 PWM，而是设置“运动方向 + 目标速度”。
// 后面 5ms 控制中断会做运动学解算、速度闭环和快速制动。
static void app_target_init(void)
{
    uart_write_string(TEST_UART, "#AI_TUNER_BOOT\r\n");
#if CONTROL_DEBUG_ENABLE == 1U
    uart_write_string(TEST_UART, "#MCU_BOOT id=7100892 stage=CONTROL_DEBUG mode=" STRINGIFY(CONTROL_VERIFY_MODE) " screen=IPS200\r\n");
#elif CONTROL_DEBUG_ENABLE == 2U
    uart_write_string(TEST_UART, "#POINT_TEST_PREBOOT id=7100892 safe=1 menu=UART8_D16_D17 display=OFF telemetry=3s cpr=4096 odom_scale=0.915 pos_type=0x12 car_valid=ON\r\n");
#else
    uart_write_string(TEST_UART, "#UART8_D16_D17_RX id=7100892 safe=1 control=GRID_STEP_MENU display=OFF cpr=4096 odom_scale=0.915 pos_type=0x12 car_valid=ON tx=IRQ_QUEUE\r\n");
#endif
    motion_set_velocity(0.0f, 0.0f);
    pentagram_enable = 0;  // Vision navigation owns motion_set_velocity in run mode.
}

static void boot_mark(const char *text)
{
#if BOOT_TRACE_ENABLE
    uart_write_string(TEST_UART, text);
    uart_write_string(TEST_UART, "\r\n");
#else
    (void)text;
#endif
}

// 初始化电机相关 IO。
// 控制一个普通直流电机驱动，通常需要两类信号：
// 1. DIR 方向脚：GPIO 输出高/低电平，决定正转/反转
// 2. PWM 速度脚：PWM 占空比，决定电机输出大小
static void motor_io_init(void)
{
    // C7/C11/C9 是三个电机的方向控制脚。
    // 它们需要主动输出明确的 0 或 1，所以配成普通 GPIO 输出 + 推挽输出。
    gpio_init(C7,  GPO, 0, GPO_PUSH_PULL);//后
    gpio_init(C11, GPO, 0, GPO_PUSH_PULL);//右
    gpio_init(C9,  GPO, 0, GPO_PUSH_PULL);//左
    // PWM 通道在 motion_control.c 的 motor_pwm_init() 里配置。
    // 方向脚在这里初始化，速度脚在那里初始化，合起来才能控制电机。
    motor_pwm_init();
}

// 初始化 IMU963RA 姿态传感器，并把结果通过串口告诉电脑。
// 这个传感器用于估计小车的航向角，后面做“走直线不偏航”和里程计坐标转换会用到。
static void imu_init_and_report(void)
{
    if(imu963ra_heading_init() == 0)
    {
        uart_write_string(TEST_UART, "#IMU963RA_READY\r\n");
    }
    else
    {
        uart_write_string(TEST_UART, "#IMU963RA_INIT_FAIL\r\n");
    }
}

// 初始化 IPS200 屏幕。
// 你想在屏幕上显示文字/数字，就要先设置方向、字体、颜色，然后 init。
static void display_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);                  // 屏幕方向
    ips200_set_font(IPS200_8X16_FONT);               // 字体大小
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    // 前景色/背景色
    ips200_init(IPS200_TYPE_SPI);                    // 这个屏幕通过 SPI 驱动
}

// 初始化三个轮子的编码器。
// 编码器用于测量轮子实际转了多少，是 PID 速度闭环的反馈来源。
// 每个编码器一般有 A/B 两相信号，所以函数里要填两个通道引脚。
static void encoder_init_all(void)
{
    encoder_quad_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
    encoder_quad_init(QTIMER2_ENCODER2, QTIMER2_ENCODER2_CH1_C5, QTIMER2_ENCODER2_CH2_C25);
    encoder_quad_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);
}

#if CONTROL_DEBUG_ENABLE == 1U
static const char *control_debug_motor_name(uint8 motor)
{
    if(motor == 0U) return "REAR";
    if(motor == 1U) return "LEFT-FRONT";
    if(motor == 2U) return "RIGHT-FRONT";
    return "INVALID";
}

static void control_debug_open_loop_set(uint8 mode, uint8 motor)
{
    pwm_set_duty(PWM2_MODULE0_CHA_C6, 0);
    pwm_set_duty(PWM2_MODULE1_CHA_C8, 0);
    pwm_set_duty(PWM2_MODULE2_CHA_C10, 0);

    // Current verified physical-positive levels: rear=1, LF=0, RF=0.
    gpio_set_level(C11, 1);
    gpio_set_level(C9, 0);
    gpio_set_level(C7, 0);

    if(mode == 2U)
    {
        pwm_set_duty(PWM2_MODULE2_CHA_C10, CONTROL_VERIFY_PWM);
        pwm_set_duty(PWM2_MODULE1_CHA_C8, CONTROL_VERIFY_PWM);
        pwm_set_duty(PWM2_MODULE0_CHA_C6, CONTROL_VERIFY_PWM);
    }
    else if(motor == 0U)
    {
        pwm_set_duty(PWM2_MODULE2_CHA_C10, CONTROL_VERIFY_PWM);
    }
    else if(motor == 1U)
    {
        pwm_set_duty(PWM2_MODULE1_CHA_C8, CONTROL_VERIFY_PWM);
    }
    else if(motor == 2U)
    {
        pwm_set_duty(PWM2_MODULE0_CHA_C6, CONTROL_VERIFY_PWM);
    }
}

static void control_debug_open_loop_task(void)
{
    int16 rear_raw;
    int16 left_raw;
    int16 right_raw;

    control_debug_open_loop_set(CONTROL_VERIFY_MODE, CONTROL_VERIFY_MOTOR);

    rear_raw = encoder_get_count(QTIMER1_ENCODER2);
    left_raw = encoder_get_count(QTIMER2_ENCODER2);
    right_raw = encoder_get_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER2);
    encoder_clear_count(QTIMER1_ENCODER1);

    ips200_clear();
    ips200_show_string(0, 0, CONTROL_VERIFY_MODE == 1U ? "DEBUG M1 SINGLE" : "DEBUG M2 ALL POS");
    ips200_show_string(0, 16, "ID:7100892 PWM:");
    ips200_show_uint(128, 16, CONTROL_VERIFY_PWM, 4);
    if(CONTROL_VERIFY_MODE == 1U)
    {
        ips200_show_string(0, 32, "SELECT:");
        ips200_show_uint(64, 32, CONTROL_VERIFY_MOTOR, 1);
        ips200_show_string(80, 32, control_debug_motor_name(CONTROL_VERIFY_MOTOR));
    }
    else
    {
        ips200_show_string(0, 32, "DIR REAR=1 LF=0 RF=0");
    }
    ips200_show_string(0, 56, "REAR  C2/C24 RAW:");
    ips200_show_int(152, 56, rear_raw, 6);
    ips200_show_string(0, 72, "LEFT  C5/C25 RAW:");
    ips200_show_int(152, 72, left_raw, 6);
    ips200_show_string(0, 88, "RIGHT C0/C1  RAW:");
    ips200_show_int(152, 88, right_raw, 6);
    ips200_show_string(0, 112, "OPEN LOOP - LIFT CAR");

    system_delay_ms(100);
}

static void control_debug_mode6_task(void)
{
    static uint8 dir_level = 0U;
    static uint32 step_count = 0U;
    int16 rear_raw;
    int16 left_raw;
    int16 right_raw;

    if(step_count >= 30U)
    {
        step_count = 0U;
        dir_level = (uint8)!dir_level;
    }
    step_count++;

    pwm_set_duty(PWM2_MODULE0_CHA_C6, 0);
    pwm_set_duty(PWM2_MODULE2_CHA_C10, 0);
    gpio_set_level(C7, 0);
    gpio_set_level(C11, 1);
    gpio_set_level(C9, dir_level);
    pwm_set_duty(PWM2_MODULE1_CHA_C8, CONTROL_VERIFY_PWM);

    rear_raw = encoder_get_count(QTIMER1_ENCODER2);
    left_raw = encoder_get_count(QTIMER2_ENCODER2);
    right_raw = encoder_get_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER2);
    encoder_clear_count(QTIMER1_ENCODER1);

    ips200_clear();
    ips200_show_string(0, 0, "DEBUG M6 LF DIR TOGGLE");
    ips200_show_string(0, 16, "ID:7100892 PWM:");
    ips200_show_uint(128, 16, CONTROL_VERIFY_PWM, 4);
    ips200_show_string(0, 32, "LEFT PWM=C8 DIR=C9");
    ips200_show_string(0, 48, "C9 LEVEL:");
    ips200_show_uint(80, 48, dir_level, 1);
    ips200_show_string(112, 48, "3S SWITCH");
    ips200_show_string(0, 72, "LEFT  C5/C25 RAW:");
    ips200_show_int(152, 72, left_raw, 6);
    ips200_show_string(0, 88, "REAR  C2/C24 RAW:");
    ips200_show_int(152, 88, rear_raw, 6);
    ips200_show_string(0, 104, "RIGHT C0/C1  RAW:");
    ips200_show_int(152, 104, right_raw, 6);
    ips200_show_string(0, 128, "C9 0=CW, 1=CCW");
    ips200_show_string(0, 144, "ONLY LEFT SHOULD TURN");

    system_delay_ms(100);
}

static void control_debug_show_closed_loop(void)
{
    static uint8 screen_cleared = 0U;

    ips200_set_font(IPS200_8X16_FONT);
    if(!screen_cleared)
    {
        ips200_clear();
        screen_cleared = 1U;
    }

    if(CONTROL_VERIFY_MODE == 3U)
    {
        ips200_show_string(0, 0, "DEBUG M3 POS HOLD");
        ips200_show_string(0, 16, "ID:7100892 PWM CAP 1800");
        ips200_show_string(0, 40, "WHEEL   ERR   SPD  DUTY");
        ips200_show_string(0, 56, "REAR");
        ips200_show_int(48, 56, (int32)position_error[0], 5);
        ips200_show_int(104, 56, encoder_speed[0], 5);
        ips200_show_int(160, 56, motor_output_duty[0], 5);
        ips200_show_string(0, 72, "LEFT");
        ips200_show_int(48, 72, (int32)position_error[1], 5);
        ips200_show_int(104, 72, encoder_speed[1], 5);
        ips200_show_int(160, 72, motor_output_duty[1], 5);
        ips200_show_string(0, 88, "RIGHT");
        ips200_show_int(48, 88, (int32)position_error[2], 5);
        ips200_show_int(104, 88, encoder_speed[2], 5);
        ips200_show_int(160, 88, motor_output_duty[2], 5);
        ips200_show_string(0, 112, "HAND TURN: PULL BACK");
        return;
    }

    ips200_show_string(0, 0, CONTROL_VERIFY_MODE == 4U ? "DEBUG M4 YAW HOLD" : "DEBUG M5 RIGHT RUN");
    ips200_show_string(0, 16, "ID:7100892 ZERO=BOOT YAW");
    ips200_show_string(0, 40, "TGT:");
    ips200_show_float(40, 40, imu963ra_yaw_target, 4, 1);
    ips200_show_string(112, 40, "YAW:");
    ips200_show_float(152, 40, imu963ra_yaw_angle, 4, 1);
    ips200_show_string(0, 56, "ERR:");
    ips200_show_float(40, 56, imu963ra_yaw_error, 4, 1);
    ips200_show_string(112, 56, "GYR:");
    ips200_show_float(152, 56, imu963ra_gyro_z_dps, 4, 1);
    ips200_show_string(0, 72, "ROT CORR:");
    ips200_show_float(80, 72, imu963ra_yaw_correction, 4, 1);
    ips200_show_string(0, 96, "WHEEL  TGT  SPD DUTY");
    ips200_show_string(0, 112, "REAR");
    ips200_show_int(48, 112, wheel_target_speed[0], 4);
    ips200_show_int(96, 112, encoder_speed[0], 5);
    ips200_show_int(152, 112, motor_output_duty[0], 5);
    ips200_show_string(0, 128, "LEFT");
    ips200_show_int(48, 128, wheel_target_speed[1], 4);
    ips200_show_int(96, 128, encoder_speed[1], 5);
    ips200_show_int(152, 128, motor_output_duty[1], 5);
    ips200_show_string(0, 144, "RIGHT");
    ips200_show_int(48, 144, wheel_target_speed[2], 4);
    ips200_show_int(96, 144, encoder_speed[2], 5);
    ips200_show_int(152, 144, motor_output_duty[2], 5);
    if(CONTROL_VERIFY_MODE == 5U)
    {
        ips200_show_string(0, 160, "M5 SPEED:");
        ips200_show_int(80, 160, (int32)CONTROL_VERIFY_MODE5_SPEED, 4);
        ips200_show_string(0, 176, control_verify_mode5_limit_enable ? "PWM CAP=ON 2500" : "PWM CAP=OFF");
    }
}
#endif

int main(void)
{
#if CONTROL_DEBUG_ENABLE != 1U
    uint32 last_tick;//辅助维护软件时间
    uint32 last_loop_report_ms = 0;
    uint8 first_loop_trace = 1;
    uint32 now;//当前时间
    char loop_report_buf[64];
#endif
#if CONTROL_DEBUG_ENABLE == 1U
    uint32 last_display_tick = 0;//上次刷新屏幕的时间
#endif

    // 1. 最基础的系统初始化。
    // clock_init 设置芯片主频；debug_init 初始化调试串口。
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    boot_mark("#BOOT debug");

    // 2. 初始化你真正用到的外设/模块。
    // 新手读工程时，可以把这里当“硬件清单”：下面每一行都对应一个硬件或功能模块。
    app_target_init();       // 初始目标速度/方向
    boot_mark("#BOOT target");
    motor_io_init();         // 电机方向 GPIO + 电机 PWM
    boot_mark("#BOOT motor");
#if CONTROL_DEBUG_ENABLE != 1U
    if(wireless_uart_init() == 0U)
    {
        uart_write_string(TEST_UART, "#WIRELESS_UART8_READY id=7100892 baud=115200 tx=D16 rx=D17 display=OFF\r\n");
    }
    else
    {
        uart_write_string(TEST_UART, "#WIRELESS_UART8_INIT_FAIL id=7100892\r\n");
    }
#endif
    imu_init_and_report();   // IMU 姿态传感器
    boot_mark("#BOOT imu");
    odometry_reset();        // 里程计清零
    boot_mark("#BOOT odom");
#if CONTROL_DEBUG_ENABLE == 1U
    display_init();          // 旧六模式不启用无线串口，继续使用IPS200
    boot_mark("#BOOT display");
#else
    boot_mark("#BOOT display_off_uart_only");
#endif
#if CONTROL_DEBUG_ENABLE != 1U
    vision_link_init();      // OpenART UART5 visual link test
    boot_mark("#BOOT vision");
#endif
    encoder_init_all();      // 三个轮子的编码器
    boot_mark("#BOOT encoder");
#if CONTROL_DEBUG_ENABLE == 1U
    control_verify_init(CONTROL_VERIFY_MODE);
    control_verify_set_mode5_speed(CONTROL_VERIFY_MODE5_SPEED);
    control_verify_set_mode5_limit(CONTROL_VERIFY_MODE5_LIMIT_ENABLE);

#if (CONTROL_VERIFY_MODE == 1U) || (CONTROL_VERIFY_MODE == 2U)
    while(1)
    {
        control_debug_open_loop_task();
    }
#endif

#if CONTROL_VERIFY_MODE == 6U
    while(1)
    {
        control_debug_mode6_task();
    }
#endif
#elif CONTROL_DEBUG_ENABLE == 2U
    point_test_init();       // Standalone local-frame calibration state machine.
    pc_console_init(2U);     // UART8 menu owns every point-test command.
#else
    planner_service_init();  // Dry-run planner cache; it never enables motors.
    mission_manager_init();  // Guarded Action execution; starts hard-stopped.
    action_follower_set_speed(RACE_FREE_MOVE_SPEED);
    pc_console_init(0U);     // UART8 menu owns PC commands; motors stay stopped.
#endif

    // 3. 开一个 5ms 周期定时器。
    // 真正的电机控制不在 while(1) 里跑，而是在 PIT 中断里每 5ms 调一次 motor_control_5ms()。
    // 对应代码在 user/src/isr.c 的 PIT_IRQHandler。
    pit_ms_init(PIT_CH0, 5);
    boot_mark("#BOOT pit");
    interrupt_global_enable(0);
    boot_mark("#BOOT irq");

    // 4. GPT_TIM_1 是普通计时器，这里主要用来辅助主循环计时。
#if CONTROL_DEBUG_ENABLE != 1U
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
    last_tick = timer_get(GPT_TIM_1);
    boot_mark("#BOOT timer");
#endif

    // device_init_flag = 1 时表示停车/保护；0 表示允许运行。
#if CONTROL_DEBUG_ENABLE == 1U
    device_init_flag = 0;
#else
    device_init_flag = 1;
#endif
    boot_mark("#BOOT loop");

    while(1)
    {
#if CONTROL_DEBUG_ENABLE == 1U
        if((CONTROL_VERIFY_MODE >= 3U) && (CONTROL_VERIFY_MODE <= 5U) &&
           ((pit_count - last_display_tick) >= DISPLAY_PERIOD_TICKS))
        {
            last_display_tick = pit_count;
            control_debug_show_closed_loop();
        }
#elif CONTROL_DEBUG_ENABLE == 2U
        vision_link_poll();
        point_test_poll();
        pc_console_poll();
#else
        // 主循环不要放太重的电机控制逻辑。
        // 这里主要处理串口来的调参命令，比如修改 PID、修改目标速度、请求状态等。
        if(first_loop_trace) boot_mark("#L enter");
        if(first_loop_trace) boot_mark("#L vis0");
        vision_link_poll();
        if(first_loop_trace) boot_mark("#L vis1");
        if(first_loop_trace) boot_mark("#L mission0");
        mission_manager_poll();
        if(first_loop_trace) boot_mark("#L mission1");
        if(first_loop_trace) boot_mark("#L menu0");
        pc_console_poll();
        if(first_loop_trace) boot_mark("#L menu1");
        if(first_loop_trace) first_loop_trace = 0;

        // 这段只是维护一个 5ms 软件时间基准，真正周期控制靠 PIT 中断。
        now = timer_get(GPT_TIM_1);
        if(BOOT_TRACE_ENABLE && (now - last_loop_report_ms) >= 500U)
        {
            last_loop_report_ms = now;
            snprintf(loop_report_buf, sizeof(loop_report_buf),
                     "#LOOP ms=%lu pit=%lu\r\n",
                     (unsigned long)now,
                     (unsigned long)pit_count);
            uart_write_string(TEST_UART, loop_report_buf);
        }

        while(now - last_tick >= 5)
        {
            last_tick += 5;
        }

#endif
    }
}
