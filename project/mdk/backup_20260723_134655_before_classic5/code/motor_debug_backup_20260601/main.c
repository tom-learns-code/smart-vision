// 读这个工程时，先不要背函数名，按这个顺序想：
// 1. 我要控制什么硬件？
// 2. 这个硬件需要什么信号？GPIO / PWM / UART / SPI / I2C / ADC / 编码器？
// 3. 逐飞库里对应哪个头文件？
// 4. 这个头文件里有哪些 init / read / write / set 函数？
// 5. 工程里有没有类似写法？
// 6. 参数里的引脚、模式、电平，是否和硬件接线一致？

// zf_common_headfile.h 是逐飞库的“总头文件”。
// 包含它以后，GPIO、PWM、UART、PIT、编码器、屏幕、IMU 等常用函数基本都能直接用。
#include "zf_common_headfile.h"

// 下面三个是你自己项目里的模块，不是逐飞官方库：
// blue.h            蓝牙按键调参/启停
// motion_control.h  电机、编码器、PID、IMU 航向、里程计等运动控制
// app_status.h      串口曲线数据和 IPS200 屏幕显示
#include "blue.h"
#include "motion_control.h"
#include "app_status.h"

#define TEST_UART              (DEBUG_UART_INDEX)   // 调试串口：用来给电脑/上位机发提示和数据
#define DEFAULT_LINEAR_SPEED   (100.0f)             // 默认全向移动速度，单位为编码器脉冲/5ms量级
#define VOFA_PERIOD_TICKS      (10U)                // pit_count 每 5ms 加 1，10 次约 50ms 发一次串口数据
#define DISPLAY_PERIOD_TICKS   (20U)                // 20 次约 100ms 刷一次屏幕

// 设置小车的初始目标状态。
// 这里不是直接给 PWM，而是设置“运动方向 + 目标速度”。
// 后面 5ms 控制中断会做运动学解算、速度闭环和快速制动。
static void app_target_init(void)
{
    uart_write_string(TEST_UART, "#AI_TUNER_BOOT\r\n");
    motion_set_velocity(0.0f, DEFAULT_LINEAR_SPEED);
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
    encoder_quad_init(QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C4);
    encoder_quad_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
    encoder_quad_init(QTIMER2_ENCODER2, QTIMER2_ENCODER2_CH1_C5, QTIMER2_ENCODER2_CH2_C25);
}

int main(void)
{
    uint32 last_tick;//辅助维护软件时间
    uint32 last_vofa_tick = 0;//上次发送 VOFA 的时间
    uint32 last_display_tick = 0;//上次刷新屏幕的时间
    uint32 now;//当前时间

    // 1. 最基础的系统初始化。
    // clock_init 设置芯片主频；debug_init 初始化调试串口。
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // 2. 初始化你真正用到的外设/模块。
    // 新手读工程时，可以把这里当“硬件清单”：下面每一行都对应一个硬件或功能模块。
    app_target_init();       // 初始目标速度/方向
    motor_io_init();         // 电机方向 GPIO + 电机 PWM
    wireless_uart_init();    // 无线串口/蓝牙相关通信模块
    imu_init_and_report();   // IMU 姿态传感器
    odometry_reset();        // 里程计清零
    display_init();          // IPS200 屏幕
    encoder_init_all();      // 三个轮子的编码器

    // 3. 开一个 5ms 周期定时器。
    // 真正的电机控制不在 while(1) 里跑，而是在 PIT 中断里每 5ms 调一次 motor_control_5ms()。
    // 对应代码在 user/src/isr.c 的 PIT_IRQHandler。
    pit_ms_init(PIT_CH0, 5);
    interrupt_global_enable(0);

    // 4. GPT_TIM_1 是普通计时器，这里主要用来辅助主循环计时。
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
    last_tick = timer_get(GPT_TIM_1);

    // device_init_flag = 1 时表示停车/保护；0 表示允许运行。
    device_init_flag = 0;

    while(1)
    {
        // 主循环不要放太重的电机控制逻辑。
        // 这里主要处理串口来的调参命令，比如修改 PID、修改目标速度、请求状态等。
        ai_tuner_poll_command();

        // 这段只是维护一个 5ms 软件时间基准，真正周期控制靠 PIT 中断。
        now = timer_get(GPT_TIM_1);
        while(now - last_tick >= 5)
        {
            last_tick += 5;
        }

        // 每约 50ms 通过串口发一次数据，方便 VOFA/上位机画曲线观察 PID 效果。
        if((pit_count - last_vofa_tick) >= VOFA_PERIOD_TICKS)
        {
            last_vofa_tick = pit_count;
            app_status_send_vofa();
        }

        // 每约 100ms 刷一次 IPS200 屏幕。
        // 屏幕刷新没必要 5ms 一次，太频繁会浪费时间。
        if((pit_count - last_display_tick) >= DISPLAY_PERIOD_TICKS)
        {
            last_display_tick = pit_count;
            app_status_show_ips200();
        }
    }
}
