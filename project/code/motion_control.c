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

#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "motion_control.h"
#include "pid.h"
#include "blue.h"
#include "app_config.h"

#define WHEEL_COUNT                     (3U)      // 轮子数量：0后轮、1左前轮、2右前轮
#define POSITION_LOOP_DIV               (1U)      // 位置环分频：1表示每个5ms周期都执行
#define POSITION_ERR_LIMIT              (3000.0f) // 位置误差绝对值上限，单位为编码器脉冲
#define POSITION_ERR_DEADBAND           (1.0f)    // 位置误差死区，小于该值按0处理
#define POSITION_TARGET_DEADBAND        (5.0f)    // 轮速目标死区，用于区分运动与停车保持
#define POSITION_COMP_STEP_LIMIT        (6.0f)    // 位置补偿每周期最大变化量，抑制突变
#define POSITION_COMP_MOVING_RATIO      (0.45f)   // 运动中补偿最多占基础轮速目标的45%
#define POSITION_COMP_MOVING_MAX        (50.0f)   // 运动中位置环速度补偿的绝对上限
#define POSITION_HOLD_ERR_START         (8.0f)    // 停车位置误差超过该值时进入位置保持
#define POSITION_HOLD_ERR_STOP          (3.0f)    // 停车位置误差低于该值时允许退出保持
#define POSITION_HOLD_SPEED_STOP        (5.0f)    // 轮速低于该值才判定基本停稳
#define POSITION_HOLD_MIN_DUTY          (0)       // 停车保持的最小PWM
#define POSITION_HOLD_MAX_DUTY          (1000)    // 正常停车保持的最大PWM
#define CONTROL_VERIFY_MODE3_DUTY_LIMIT (1800)    // 调试模式3的位置保持PWM上限
#define POSITION_HOLD_DUTY_KP           (5.0f)    // 停车保持的位置误差比例系数
#define POSITION_HOLD_DUTY_KD           (30.0f)   // 停车保持的轮速阻尼系数
#define WHEEL_SPEED_LOOP_TEST_ENABLE     (0U)      // 单轮速度环测试开关：0关闭、1开启
#define WHEEL_SPEED_LOOP_TEST_TARGET     (100)     // 单轮速度环测试目标速度
#define WHEEL_SPEED_LOOP_TEST_DUTY_LIMIT (2500)    // 单轮速度环测试PWM上限
#define CONTROL_VERIFY_MODE5_DUTY_LIMIT  (2500)    // 调试模式5启用限幅时的PWM上限
#define STARTUP_ASSIST_PERIOD_MS          (5U)      // 起步补偿状态按5ms控制周期计时
#define STARTUP_ASSIST_TARGET_MIN         (5.0f)    // 仅对确实有运动目标的轮子补偿
#define STARTUP_ASSIST_SPEED_MAX          (5.0f)    // 轮速超过该值即认为已经克服静摩擦
#define PWM_VECTOR_RAMP_PERIOD_MS         (5U)
#define AI_TUNER_UART                    (DEBUG_UART_INDEX) // AI调参所使用的串口
#define CONTROL_PERIOD_S                 (0.005f)   // 控制周期，单位为秒
#define IMU963RA_ZERO_BIAS_SAMPLES       (1500U)    // 上电陀螺零偏采样次数
#define IMU963RA_ZERO_BIAS_DELAY_MS      (2U)       // 零偏采样之间的延时，单位为毫秒
#define IMU963RA_ZERO_BIAS_MAX_DPS       (5.0f)     // 零偏校准允许的最大角速度
#define IMU963RA_GYRO_DEADBAND_DPS       (1.20f)    // 陀螺角速度死区，单位为度每秒
#define IMU963RA_BIAS_TRACK_SPEED        (2.0f)     // 低于该轮速时允许在线跟踪零偏
#define IMU963RA_BIAS_TRACK_MAX_DPS      (2.50f)    // 在线跟踪零偏允许的最大角速度
#define IMU963RA_BIAS_TRACK_ALPHA        (0.002f)   // 在线零偏低通更新系数
#define YAW_HOLD_MIN_SPEED               (5.0f)     // 高于该平移速度才启用航向保持
#define YAW_HOLD_DEADBAND_DEG            (0.50f)    // 航向误差死区，单位为度
#define YAW_HOLD_KP                      (5.0f)     // 航向保持比例系数
#define YAW_HOLD_KD                      (0.40f)    // 航向保持角速度阻尼系数
#define YAW_HOLD_OUT_LIMIT               (50.0f)    // 航向旋转修正量绝对上限
#define YAW_HOLD_ROT_SIGN                (1.0f)     // 当前三轮/IMU方向：公共正轮速增大IMU航向，target-now直接形成负反馈
#define VISUAL_YAW_SYNC_GAIN             (0.35f)    // 视觉角度同步增益
#define VISUAL_YAW_SYNC_DEADBAND         (2.0f)     // 视觉角度同步死区，单位为度
#define VISUAL_YAW_SYNC_MAX_STEP         (4.0f)     // 单次视觉角度校正的最大步长
#define CIRCLE_MODE_ENABLE               (0U)       // 圆轨迹模式开关：0关闭、1开启
#define CIRCLE_RADIUS_MM                 (350.0f)   // 圆轨迹目标半径，单位为毫米
#define CIRCLE_POS_KP                    (0.35f)    // 圆轨迹径向误差比例系数
#define CIRCLE_CORR_LIMIT_RATIO          (0.80f)    // 径向修正量相对基础速度的比例上限
#define CIRCLE_SPEED_MAX_RATIO           (1.30f)    // 圆轨迹合成速度相对目标速度的上限
#define BACK_FORTH_FORWARD_ANGLE         (0.0f)     // 往返模式前进方向角，单位为度
#define BACK_FORTH_BACKWARD_ANGLE        (180.0f)   // 往返模式后退方向角，单位为度
#ifndef ODOMETRY_QUICK_SCALE
#define ODOMETRY_QUICK_SCALE            (APP_ODOMETRY_SCALE) // 实测有效里程比例集中配置
#endif
#ifndef WHEEL_DIAMETER_MM
#define WHEEL_DIAMETER_MM               (APP_WHEEL_DIAMETER_MM) // 轮胎直径集中配置
#endif
#ifndef WHEEL_CPR
#define WHEEL_CPR                       (APP_WHEEL_CPR) // 编码器CPR集中配置
#endif
#define ODOM_PULSE_TO_MM                ((3.1415926f * WHEEL_DIAMETER_MM / WHEEL_CPR) * ODOMETRY_QUICK_SCALE) // 单脉冲对应毫米数
#define ODOM_DIRECTION_MIN_MM           (0.5f)     // 更新运动方向所需的最小位移
#define ODOM_YAW_SIGN                   (1.0f)     // 里程计坐标旋转方向符号

typedef struct
{
    encoder_index_enum encoder;
    int8 encoder_sign;
    pwm_channel_enum pwm;
    gpio_pin_enum dir_pin;
    uint8 forward_level;
} wheel_hw_t;

static const wheel_hw_t wheel_hw[WHEEL_COUNT] =
{
    {QTIMER1_ENCODER2, APP_REAR_ENCODER_SIGN,  PWM2_MODULE2_CHA_C10, C11, APP_REAR_FORWARD_LEVEL},  // 后轮：编码器C2/C24，PWM C10，方向C11
    {QTIMER2_ENCODER2, APP_LEFT_ENCODER_SIGN,  PWM2_MODULE1_CHA_C8,  C9,  APP_LEFT_FORWARD_LEVEL},  // 左前轮：编码器C5/C25，PWM C8，方向C9
    {QTIMER1_ENCODER1, APP_RIGHT_ENCODER_SIGN, PWM2_MODULE0_CHA_C6,  C7,  APP_RIGHT_FORWARD_LEVEL},  // 右前轮：编码器C0/C1，PWM C6，方向C7
};

#define MOTOR_PWM_FREQUENCY       (17000) // 电机PWM频率，单位为Hz

void motor_pwm_init(void)
{
    pwm_init(PWM2_MODULE0_CHA_C6, MOTOR_PWM_FREQUENCY, 0);
    pwm_init(PWM2_MODULE2_CHA_C10, MOTOR_PWM_FREQUENCY, 0);
    pwm_init(PWM2_MODULE1_CHA_C8, MOTOR_PWM_FREQUENCY, 0);
}
static const float wheel_install_angle[WHEEL_COUNT] =
{
    -90.0f,  // 后轮安装角度
    30.0f,   // 左前轮安装角度
    150.0f,  // 右前轮安装角度
};




//========================================================= Y车模PID速度环参数 =========================================================

// 编码器数据（每5ms读取一次脉冲差值）
int16 encoder_speed[3] = {0, 0, 0};
static volatile int32 wheel_total_count[3] = {0, 0, 0};

// 三个轮子速度目标值（由运动学解算得到）
int16 wheel_target_speed[3] = {0, 0, 0};

PID_t motor0_speed_pid = {
    .Kp = 18,               // 后轮速度环比例系数
    .Ki = 0.03f,            // 后轮速度环积分系数
    .Kd = 0,                // 后轮速度环微分系数
    .ErrorIntMax = 1500.0f, // 后轮积分累计限值，最大积分输出为45
    .OutMax = 10000,        // 后轮PID正向输出上限
    .OutMin = -10000,       // 后轮PID反向输出下限
};
PID_t motor1_speed_pid = {
    .Kp = 18,               // 左前轮速度环比例系数
    .Ki = 0.03,             // 左前轮速度环积分系数
    .Kd = 0,                // 左前轮速度环微分系数
    .ErrorIntMax = 1500.0f, // 左前轮积分累计限值，最大积分输出为45
    .OutMax = 10000,        // 左前轮PID正向输出上限
    .OutMin = -10000,       // 左前轮PID反向输出下限
};
PID_t motor2_speed_pid = {
    .Kp = 20,               // 右前轮速度环比例系数
    .Ki = 0.03f,            // 右前轮速度环积分系数
    .Kd = 0.0,              // 右前轮速度环微分系数
    .ErrorIntMax = 1500.0f, // 右前轮积分累计限值，最大积分输出为45
    .OutMax = 10000,        // 右前轮PID正向输出上限
    .OutMin = -10000,       // 右前轮PID反向输出下限
};

//========================================================= Y车模位置环参数 =========================================================

PID_t motor0_position_pid = {
    .Kp = 0.7f,           // 后轮位置环比例系数
    .Ki = 0.0f,           // 后轮位置环积分系数，当前关闭
    .Kd = 0.0f,           // 后轮位置环微分系数，当前关闭
    .ErrorIntMax = 0.0f,  // 后轮位置环积分限值，Ki为0时无效
    .OutMax = 90.0f,      // 后轮位置PID原始输出上限
    .OutMin = -90.0f,     // 后轮位置PID原始输出下限
};
PID_t motor1_position_pid = {
    .Kp = 0.7f,           // 左前轮位置环比例系数
    .Ki = 0.0f,           // 左前轮位置环积分系数，当前关闭
    .Kd = 0.0f,           // 左前轮位置环微分系数，当前关闭
    .ErrorIntMax = 0.0f,  // 左前轮位置环积分限值，Ki为0时无效
    .OutMax = 90.0f,      // 左前轮位置PID原始输出上限
    .OutMin = -90.0f,     // 左前轮位置PID原始输出下限
};
PID_t motor2_position_pid = {
    .Kp = 0.7f,           // 右前轮位置环比例系数
    .Ki = 0.0f,           // 右前轮位置环积分系数，当前关闭
    .Kd = 0.0f,           // 右前轮位置环微分系数，当前关闭
    .ErrorIntMax = 0.0f,  // 右前轮位置环积分限值，Ki为0时无效
    .OutMax = 90.0f,      // 右前轮位置PID原始输出上限
    .OutMin = -90.0f,     // 右前轮位置PID原始输出下限
};

// 位置误差：目标位置 - 实际位置
float position_error[3] = {0, 0, 0};

// 位置环输出的速度补偿量
float position_speed_comp[3] = {0, 0, 0};

// 位置环显式累计量：目标位置与实际位置均以编码器5ms脉冲为单位
float position_target[3] = {0, 0, 0};
float position_actual[3] = {0, 0, 0};

// 实际下发给电机的占空比，便于VOFA观察静止回正是否真的出力
int16 motor_output_duty[3] = {0, 0, 0};

uint8 imu963ra_ready = 0;                   // IMU初始化成功标志
uint8 imu963ra_yaw_hold_enable = 1;         // 航向保持开关
float imu963ra_gyro_z_offset = 0.0f;        // 陀螺仪Z轴零偏
float imu963ra_gyro_z_dps = 0.0f;           // 当前Z轴角速度，单位为度每秒
float imu963ra_yaw_angle = 0.0f;            // 当前估算航向角
float imu963ra_yaw_target = 0.0f;           // 航向保持目标角
float imu963ra_yaw_error = 0.0f;            // 当前航向误差
float imu963ra_yaw_correction = 0.0f;       // 航向环输出的旋转修正量
uint8 control_verify_mode = 0U;
uint8 control_verify_mode5_limit_enable = 1U;
static float control_verify_mode5_speed = 300.0f;
static uint8 motion_heading_lock_active = 0U;
static volatile uint16 startup_assist_ticks_remaining = 0U;
static volatile uint8 startup_assist_applied_mask = 0U;
static const int16 startup_assist_min_duty[WHEEL_COUNT] = {3000, 2100, 2200};
static volatile uint16 pwm_ramp_ticks_remaining = 0U;
static volatile uint16 pwm_ramp_max_delta = 0U;
static volatile uint16 pwm_ramp_scale_x1000 = 1000U;
static int16 pwm_ramp_previous[WHEEL_COUNT] = {0, 0, 0};
static int16 pwm_ramp_desired[WHEEL_COUNT] = {0, 0, 0};
static uint8 manual_rotation_active = 0U;
static float manual_rotation_speed = 0.0f;
static float visual_yaw_sync_error_deg = 0.0f;
static float visual_yaw_sync_step_deg = 0.0f;
static float pentagram_heading_target = 0.0f;
// odom是odometry（里程计），根据编码器估算车辆位移
float odom_body_delta_x_mm = 0.0f;
float odom_body_delta_y_mm = 0.0f;
// 车体坐标系下的单周期位移


float odom_world_x_mm = 0.0f;
float odom_world_y_mm = 0.0f;
// 世界坐标系下的累计位置


float odom_total_distance_mm = 0.0f;//累计行驶路程
float odom_displacement_mm = 0.0f;//距离起点的直线距离
float odom_body_direction_deg = 0.0f;//
float odom_move_direction_deg = 0.0f;//
float odom_instant_direction_deg = 0.0f;//

// 前馈模型参数（已注释，当前不使用前馈）
//float feedforward_K = 12.0f;       // 前馈斜率系数（轮0、轮1）
//float feedforward_K3 = 12.1f;      // 前馈斜率系数（轮2，略有差异）
//float feedforward_bias = 60.20f;   // 前馈偏置（克服静摩擦和死区）

// 前馈模型函数（已注释）
//static float feedforward_model(float speed, uint8 wheel_idx)
//{
//    if(speed > 0)
//    {
//        if(wheel_idx == 2)
//            return feedforward_K3 * speed + feedforward_bias;
//        else
//            return feedforward_K * speed + feedforward_bias;
//    }
//    else if(speed < 0)
//    {
//        if(wheel_idx == 2)
//            return feedforward_K3 * speed - feedforward_bias;
//        else
//            return feedforward_K * speed - feedforward_bias;
//    }
//    else
//        return 0;
//}

// 增量式PID累加输出（已注释）
//static float pid_out_accum[3] = {0, 0, 0};

// 前馈+PID总输出（已注释）
//float feedforward_total_out[3] = {0, 0, 0};

// 运动学角度和速度目标（由main.c设置）
float target_angle = 90.0f;
float target_linear_speed = 0.0f;

// 来回跑模式
int back_forth_mode = 0;           // 0=手动速度模式，1=来回跑模式
float back_forth_speed = 100.0f;   // 来回跑的速度大小（蓝牙可调）
uint32 back_forth_period = 1000;   // 来回跑半周期，1000个5ms周期等于5秒

// PIT中断计数器（用于验证中断是否运行）
volatile uint32 pit_count = 0;
// pit_count在motor_control_5ms()中每次加1，用于观察5ms中断是否正常运行

// AI调参串口缓冲区
static char ai_tuner_cmd_buf[80];
static char ai_tuner_ready_buf[80];
static uint8 ai_tuner_cmd_len = 0;
static volatile uint8 ai_tuner_cmd_ready = 0;



// 度转弧度
#define DEG2RAD(deg)    ((deg) * 3.1415926f / 180.0f)//角度 × π / 180 = 弧度

static float last_wheel_target_speed[3] = {0.0f, 0.0f, 0.0f};//记录三个轮子的上一次目标速度
static float circle_phase_rad = 0.0f;//圆轨迹当前相位，单位为弧度
static uint8 position_loop_divider = 0;//位置环分频，作用：让位置环跑得比速度环慢
static uint8 position_hold_active[3] = {0, 0, 0};//标记每个轮子是否处于位置保持模式
static float position_raw_output[3] = {0.0f, 0.0f, 0.0f};
static float position_dynamic_limit[3] = {0.0f, 0.0f, 0.0f};
static uint8 position_limit_flags[3] = {0U, 0U, 0U};
static uint8 motor_duty_source[3] = {MOTION_DUTY_SOURCE_SPEED_PID,
                                     MOTION_DUTY_SOURCE_SPEED_PID,
                                     MOTION_DUTY_SOURCE_SPEED_PID};
static float yaw_p_output = 0.0f;
static float yaw_d_output = 0.0f;
static float yaw_raw_output = 0.0f;
static uint8 yaw_output_saturated = 0U;

typedef struct
{
    uint32 samples;
    float speed_error_abs_sum[3];
    float speed_error_abs_max[3];
    int16 pwm_min[3];
    int16 pwm_max[3];
    uint32 position_clip_count[3];
    uint32 position_slew_count[3];
    uint32 integral_cap_count[3];
    uint32 output_cap_count[3];
    uint32 position_hold_count[3];
} motion_pid_window_t;

static motion_pid_window_t pid_window;
//0 = 正常速度控制
// 1表示位置保持模式：停车后用PD控制锁住位置


// 三个局部工具函数
static float clampf_local(float value, float min_value, float max_value)//浮点限幅函数
{
    if(value > max_value) return max_value;
    if(value < min_value) return min_value;
    return value;
}

static float absf_local(float value)//求浮点绝对值
{
    return (value >= 0.0f) ? value : -value;
}

static float normalize_angle_360(float angle)//把角度归一化到0至360度
{
    while(angle < 0.0f) angle += 360.0f;
    while(angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static float normalize_angle_180(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static void pid_window_reset(void)
{
    memset(&pid_window, 0, sizeof(pid_window));
}

static void pid_window_update(void)
{
    PID_t *speed_pid[WHEEL_COUNT] = {&motor0_speed_pid, &motor1_speed_pid, &motor2_speed_pid};

    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        float error_abs = absf_local(speed_pid[i]->Error0);

        if(pid_window.samples == 0U)
        {
            pid_window.pwm_min[i] = motor_output_duty[i];
            pid_window.pwm_max[i] = motor_output_duty[i];
        }
        else
        {
            if(motor_output_duty[i] < pid_window.pwm_min[i]) pid_window.pwm_min[i] = motor_output_duty[i];
            if(motor_output_duty[i] > pid_window.pwm_max[i]) pid_window.pwm_max[i] = motor_output_duty[i];
        }

        pid_window.speed_error_abs_sum[i] += error_abs;
        if(error_abs > pid_window.speed_error_abs_max[i])
        {
            pid_window.speed_error_abs_max[i] = error_abs;
        }

        if(position_limit_flags[i] & MOTION_POS_LIMIT_CLIPPED) pid_window.position_clip_count[i]++;
        if(position_limit_flags[i] & MOTION_POS_LIMIT_SLEW) pid_window.position_slew_count[i]++;
        if(speed_pid[i]->ErrorIntMax > 0.0f &&
           absf_local(speed_pid[i]->ErrorInt) >= (speed_pid[i]->ErrorIntMax - 0.5f))
        {
            pid_window.integral_cap_count[i]++;
        }
        if(speed_pid[i]->Out >= (speed_pid[i]->OutMax - 0.5f) ||
           speed_pid[i]->Out <= (speed_pid[i]->OutMin + 0.5f))
        {
            pid_window.output_cap_count[i]++;
        }
        if(motor_duty_source[i] == MOTION_DUTY_SOURCE_POS_HOLD)
        {
            pid_window.position_hold_count[i]++;
        }
    }

    pid_window.samples++;
}
// -30度 -> 330度
// 370度 -> 10度
// 720度 -> 0度

// 主要全局状态说明
// wheel_hw              三个轮子的电气接线
// wheel_install_angle   三个轮子的机械安装角度

//target_angle          整车目标方向
//target_linear_speed   整车目标速度

//wheel_target_speed    三个轮子的目标速度
//encoder_speed         三个轮子的实际速度
// motor_output_duty     三个电机最终PWM

// position_xxx          位置环和快速制动状态
//imu963ra_xxx          车头角度保持
// odom_xxx              里程计状态




// 接下来的部分包含：
// 1. 里程计清零
// 2. 里程计更新
// 3. IMU初始化与更新
//4. 车头保持
//5. 运动解算、PWM 输出、编码器读取
void odometry_reset(void)//里程计清零
{
    odom_body_delta_x_mm = 0.0f;
    odom_body_delta_y_mm = 0.0f;
    odom_world_x_mm = 0.0f;
    odom_world_y_mm = 0.0f;
    odom_total_distance_mm = 0.0f;
    odom_displacement_mm = 0.0f;
    odom_body_direction_deg = 0.0f;
    odom_move_direction_deg = 0.0f;
    odom_instant_direction_deg = 0.0f;
}

static void odometry_update(void)//根据编码器估算车辆位移
{
    float body_x_pulse = 0.0f;//车体坐标系下X方向的编码器等效脉冲
    float body_y_pulse = 0.0f;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float world_dx;
    float world_dy;
    float step_distance;

    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        float rad = DEG2RAD(wheel_install_angle[i]);//将第i个轮子的安装角度由度转为弧度
        body_x_pulse += (float)encoder_speed[i] * cosf(rad);
        body_y_pulse += (float)encoder_speed[i] * sinf(rad);
        // 将单个轮子的运动贡献分解到X、Y方向
    }

    body_x_pulse *= (2.0f / 3.0f);
    body_y_pulse *= (2.0f / 3.0f);
    // 前面已累加三个轮子的运动贡献
    // 这里乘系数，将结果缩放到车体实际位移量级

    odom_body_delta_x_mm = body_x_pulse * ODOM_PULSE_TO_MM;
    odom_body_delta_y_mm = body_y_pulse * ODOM_PULSE_TO_MM;
		//编码器脉冲换算成毫米
    // body_x_pulse和body_y_pulse是等效脉冲数
    //ODOM_PULSE_TO_MM 是：
    // 每个编码器脉冲对应的毫米数由ODOM_PULSE_TO_MM给出
    //所以相乘之后得到：
    // 计算每个5ms周期内车体X、Y方向的毫米位移
		
		
		
		// 从车体坐标系转换到世界坐标系
    yaw_rad = DEG2RAD(imu963ra_yaw_angle * ODOM_YAW_SIGN);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    world_dx = odom_body_delta_x_mm * cos_yaw - odom_body_delta_y_mm * sin_yaw;
    world_dy = odom_body_delta_x_mm * sin_yaw + odom_body_delta_y_mm * cos_yaw;

    odom_world_x_mm += world_dx;
    odom_world_y_mm += world_dy;


    // 计算累计行驶路程
    step_distance = sqrtf(world_dx * world_dx + world_dy * world_dy);
    odom_total_distance_mm += step_distance;
    odom_displacement_mm = sqrtf(odom_world_x_mm * odom_world_x_mm + odom_world_y_mm * odom_world_y_mm);
    //计算运动方向
    if(step_distance >= ODOM_DIRECTION_MIN_MM)
    {
        odom_body_direction_deg = normalize_angle_360(atan2f(odom_body_delta_y_mm, odom_body_delta_x_mm) * 180.0f / 3.1415926f);
        odom_instant_direction_deg = normalize_angle_360(atan2f(world_dy, world_dx) * 180.0f / 3.1415926f);
    }

    if(odom_displacement_mm >= ODOM_DIRECTION_MIN_MM)
    {
        odom_move_direction_deg = normalize_angle_360(atan2f(odom_world_y_mm, odom_world_x_mm) * 180.0f / 3.1415926f);
    }
		// 计算起点指向当前位置的方向角
}

uint8 imu963ra_heading_init(void)
{
    float gyro_sum = 0.0f;
    uint16 valid_count = 0;

    imu963ra_ready = (imu963ra_init() == 0);
    imu963ra_gyro_z_offset = 0.0f;
    imu963ra_gyro_z_dps = 0.0f;
    imu963ra_yaw_angle = 0.0f;
    imu963ra_yaw_target = 0.0f;
    imu963ra_yaw_error = 0.0f;
    imu963ra_yaw_correction = 0.0f;

    if(!imu963ra_ready)
    {
        return 1;
    }

    for(uint16 i = 0; i < IMU963RA_ZERO_BIAS_SAMPLES; i++)
    {
        float gyro_z;

        imu963ra_get_gyro();
        gyro_z = imu963ra_gyro_transition(imu963ra_gyro_z);

        if(absf_local(gyro_z) < IMU963RA_ZERO_BIAS_MAX_DPS)
        {
            gyro_sum += gyro_z;
            valid_count++;
        }

        system_delay_ms(IMU963RA_ZERO_BIAS_DELAY_MS);
    }

    if(valid_count > 0)
    {
        imu963ra_gyro_z_offset = gyro_sum / (float)valid_count;
    }

    return 0;
}

void imu963ra_heading_reset(void)
{
    imu963ra_gyro_z_dps = 0.0f;
    imu963ra_yaw_angle = 0.0f;
    imu963ra_yaw_target = 0.0f;
    imu963ra_yaw_error = 0.0f;
    imu963ra_yaw_correction = 0.0f;
}

void imu963ra_heading_update(void)
{
    float gyro_z;
    float encoder_rotate_speed;

    if(!imu963ra_ready)
    {
        imu963ra_gyro_z_dps = 0.0f;
        return;
    }

    imu963ra_get_gyro();
    gyro_z = imu963ra_gyro_transition(imu963ra_gyro_z);
    imu963ra_gyro_z_dps = gyro_z - imu963ra_gyro_z_offset;
    encoder_rotate_speed = ((float)encoder_speed[0] +
                            (float)encoder_speed[1] +
                            (float)encoder_speed[2]) / 3.0f;

    // 平移时三轮速度和约为0，自转时会同向叠加；用实际旋转分量跟踪陀螺零偏，避免误差缓慢累积
    if(absf_local(encoder_rotate_speed) < POSITION_HOLD_SPEED_STOP &&
       absf_local(imu963ra_gyro_z_dps) < IMU963RA_BIAS_TRACK_MAX_DPS)
    {
        imu963ra_gyro_z_offset += imu963ra_gyro_z_dps * IMU963RA_BIAS_TRACK_ALPHA;
        imu963ra_gyro_z_dps = gyro_z - imu963ra_gyro_z_offset;
    }

    if(absf_local(imu963ra_gyro_z_dps) < IMU963RA_GYRO_DEADBAND_DPS)
    {
        imu963ra_gyro_z_dps = 0.0f;
    }

    imu963ra_yaw_angle += imu963ra_gyro_z_dps * CONTROL_PERIOD_S;
}

static float imu963ra_yaw_hold_calculate(void)
{
    if(!imu963ra_ready || !imu963ra_yaw_hold_enable)
    {
        imu963ra_yaw_error = 0.0f;
        imu963ra_yaw_correction = 0.0f;
        yaw_p_output = 0.0f;
        yaw_d_output = 0.0f;
        yaw_raw_output = 0.0f;
        yaw_output_saturated = 0U;
        return 0.0f;
    }

    if(absf_local(target_linear_speed) < YAW_HOLD_MIN_SPEED)
    {
        imu963ra_yaw_target = imu963ra_yaw_angle;
        imu963ra_yaw_error = 0.0f;
        imu963ra_yaw_correction = 0.0f;
        yaw_p_output = 0.0f;
        yaw_d_output = 0.0f;
        yaw_raw_output = 0.0f;
        yaw_output_saturated = 0U;
        return 0.0f;
    }

    imu963ra_yaw_error = normalize_angle_180(
        imu963ra_yaw_target - imu963ra_yaw_angle);
    if(absf_local(imu963ra_yaw_error) < YAW_HOLD_DEADBAND_DEG)
    {
        imu963ra_yaw_error = 0.0f;
    }

    yaw_p_output = imu963ra_yaw_error * YAW_HOLD_KP * YAW_HOLD_ROT_SIGN;
    yaw_d_output = -imu963ra_gyro_z_dps * YAW_HOLD_KD * YAW_HOLD_ROT_SIGN;
    yaw_raw_output = yaw_p_output + yaw_d_output;
    yaw_output_saturated = (absf_local(yaw_raw_output) > YAW_HOLD_OUT_LIMIT) ? 1U : 0U;
    imu963ra_yaw_correction = clampf_local(yaw_raw_output,
                                           -YAW_HOLD_OUT_LIMIT,
                                           YAW_HOLD_OUT_LIMIT);

    return imu963ra_yaw_correction;
}

static float imu963ra_yaw_hold_calculate_stationary(void)
{
    if(!imu963ra_ready || !imu963ra_yaw_hold_enable)
    {
        imu963ra_yaw_error = 0.0f;
        imu963ra_yaw_correction = 0.0f;
        yaw_p_output = 0.0f;
        yaw_d_output = 0.0f;
        yaw_raw_output = 0.0f;
        yaw_output_saturated = 0U;
        return 0.0f;
    }

    imu963ra_yaw_error = normalize_angle_180(
        imu963ra_yaw_target - imu963ra_yaw_angle);
    if(absf_local(imu963ra_yaw_error) < YAW_HOLD_DEADBAND_DEG)
    {
        imu963ra_yaw_error = 0.0f;
    }

    yaw_p_output = imu963ra_yaw_error * YAW_HOLD_KP * YAW_HOLD_ROT_SIGN;
    yaw_d_output = -imu963ra_gyro_z_dps * YAW_HOLD_KD * YAW_HOLD_ROT_SIGN;
    yaw_raw_output = yaw_p_output + yaw_d_output;
    yaw_output_saturated = (absf_local(yaw_raw_output) > YAW_HOLD_OUT_LIMIT) ? 1U : 0U;
    imu963ra_yaw_correction = clampf_local(yaw_raw_output,
                                           -YAW_HOLD_OUT_LIMIT,
                                           YAW_HOLD_OUT_LIMIT);
    return imu963ra_yaw_correction;
}

static float slew_limit(float current, float target, float step)
{
    float delta = target - current;

    if(delta > step)  return current + step;
    if(delta < -step) return current - step;
    return target;
}

static void reset_pid_state(PID_t *pid)
{
    pid->Error0 = 0.0f;
    pid->Error1 = 0.0f;
    pid->Error2 = 0.0f;
    pid->ErrorInt = 0.0f;
    pid->Out = 0.0f;
    pid->OutLast = 0.0f;
}

static void reset_position_channel(uint8 index, PID_t *position_pid)
{
    position_error[index] = 0.0f;
    position_speed_comp[index] = 0.0f;
    position_raw_output[index] = 0.0f;
    position_dynamic_limit[index] = 0.0f;
    position_limit_flags[index] = 0U;
    position_target[index] = 0.0f;
    position_actual[index] = 0.0f;
    last_wheel_target_speed[index] = 0.0f;
    position_hold_active[index] = 0;
    motor_duty_source[index] = MOTION_DUTY_SOURCE_SPEED_PID;
    reset_pid_state(position_pid);
}

// 运动学解算：给定角度和线速度，计算三轮目标速度
static void k_calculate(float angle, float speed, float rotate_speed, int16 motor_speed[3])//仅供当前C文件调用的三轮运动学解算
{
    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        float rad = DEG2RAD(angle - wheel_install_angle[i]);
        motor_speed[i] = (int16)(speed * cosf(rad) + rotate_speed);
    }
}

// 设置单个轮子PWM（沿用速度环工程的引脚定义）
void p_set(uint8 wheel, int16 duty)
{
    uint16 pwm_duty;
    uint8 dir_level;

    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    if(duty > 10000) duty = 10000;
    if(duty < -10000) duty = -10000;

    if(duty < 0)
    {
        pwm_duty = (uint16)(-duty);
        dir_level = (uint8)!wheel_hw[wheel].forward_level;
    }
    else
    {
        pwm_duty = (uint16)duty;
        dir_level = wheel_hw[wheel].forward_level;
    }

    gpio_set_level(wheel_hw[wheel].dir_pin, dir_level);
    pwm_set_duty(wheel_hw[wheel].pwm, (uint32)pwm_duty);
}

static void read_encoder_speed(void)
{
    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        int16 count = encoder_get_count(wheel_hw[i].encoder);
        encoder_clear_count(wheel_hw[i].encoder);
        encoder_speed[i] = (int16)(wheel_hw[i].encoder_sign * count);
        wheel_total_count[i] += (int32)encoder_speed[i];
    }
}

static void handle_target_change(PID_t *position_pid[WHEEL_COUNT])
{
    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        float current_target = (float)wheel_target_speed[i];
        float last_target = last_wheel_target_speed[i];
        uint8 was_stopped = (absf_local(last_target) < POSITION_TARGET_DEADBAND);
        uint8 is_running = (absf_local(current_target) >= POSITION_TARGET_DEADBAND);
        uint8 direction_changed =
            (current_target > POSITION_TARGET_DEADBAND && last_target < -POSITION_TARGET_DEADBAND) ||
            (current_target < -POSITION_TARGET_DEADBAND && last_target > POSITION_TARGET_DEADBAND);

        if((was_stopped && is_running) || direction_changed)
        {
            reset_position_channel(i, position_pid[i]);
        }

        last_wheel_target_speed[i] = current_target;
    }
}

static void integrate_position_counter(void)
{
    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        if(absf_local((float)wheel_target_speed[i]) >= POSITION_TARGET_DEADBAND)
        {
            position_target[i] += (float)wheel_target_speed[i];
        }

        position_actual[i] += (float)encoder_speed[i];
    }
}

static void update_position_loop(PID_t *position_pid[WHEEL_COUNT])
{
    if(++position_loop_divider < POSITION_LOOP_DIV)
    {
        return;
    }

    position_loop_divider = 0;

    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        float previous_comp = position_speed_comp[i];

        position_limit_flags[i] = 0U;
        position_error[i] = position_target[i] - position_actual[i];
        position_error[i] = clampf_local(position_error[i], -POSITION_ERR_LIMIT, POSITION_ERR_LIMIT);

        if(absf_local(position_error[i]) < POSITION_ERR_DEADBAND)
        {
            position_error[i] = 0.0f;
            if(absf_local((float)wheel_target_speed[i]) >= POSITION_TARGET_DEADBAND)
            {
                position_target[i] = position_actual[i];
            }
            reset_pid_state(position_pid[i]);
            position_speed_comp[i] = slew_limit(position_speed_comp[i], 0.0f, POSITION_COMP_STEP_LIMIT);
            position_raw_output[i] = 0.0f;
            position_dynamic_limit[i] = 0.0f;
            if(absf_local(position_speed_comp[i] - previous_comp) > 0.01f)
            {
                position_limit_flags[i] |= MOTION_POS_LIMIT_SLEW;
            }
            continue;
        }

        position_pid[i]->Target = position_error[i];
        position_pid[i]->Actual = 0.0f;
        position_pid_update(position_pid[i]);
        {
            float comp = position_pid[i]->Out;
            float base_target = absf_local((float)wheel_target_speed[i]);

            position_raw_output[i] = comp;
            position_dynamic_limit[i] = position_pid[i]->OutMax;
            if(base_target >= POSITION_TARGET_DEADBAND)
            {
                float comp_limit = base_target * POSITION_COMP_MOVING_RATIO;
                if(comp_limit > POSITION_COMP_MOVING_MAX)
                {
                    comp_limit = POSITION_COMP_MOVING_MAX;
                    position_limit_flags[i] |= MOTION_POS_LIMIT_MAX;
                }
                else
                {
                    position_limit_flags[i] |= MOTION_POS_LIMIT_RATIO;
                }
                if(absf_local(comp) > comp_limit)
                {
                    position_limit_flags[i] |= MOTION_POS_LIMIT_CLIPPED;
                }
                position_dynamic_limit[i] = comp_limit;
                comp = clampf_local(comp, -comp_limit, comp_limit);
            }

            position_speed_comp[i] = slew_limit(position_speed_comp[i], comp, POSITION_COMP_STEP_LIMIT);
            if(absf_local(position_speed_comp[i] - comp) > 0.01f)
            {
                position_limit_flags[i] |= MOTION_POS_LIMIT_SLEW;
            }
        }
    }
}

static int16 get_motor_duty(uint8 wheel, float pid_out)
{
    float hold_duty;
    float abs_error = absf_local(position_error[wheel]);
    float abs_speed = absf_local((float)encoder_speed[wheel]);

    motor_duty_source[wheel] = MOTION_DUTY_SOURCE_SPEED_PID;

    if(absf_local(target_linear_speed) < POSITION_TARGET_DEADBAND &&
       absf_local((float)wheel_target_speed[wheel]) < POSITION_TARGET_DEADBAND)
    {
        if(!position_hold_active[wheel] && abs_error > POSITION_HOLD_ERR_START)
        {
            position_hold_active[wheel] = 1;
        }

        if(position_hold_active[wheel] && abs_error < POSITION_HOLD_ERR_STOP && abs_speed < POSITION_HOLD_SPEED_STOP)
        {
            position_hold_active[wheel] = 0;
        }

        if(position_hold_active[wheel])
        {
            motor_duty_source[wheel] = MOTION_DUTY_SOURCE_POS_HOLD;
            hold_duty = position_error[wheel] * POSITION_HOLD_DUTY_KP
                      - (float)encoder_speed[wheel] * POSITION_HOLD_DUTY_KD;

            if(abs_error < POSITION_HOLD_ERR_STOP && abs_speed < POSITION_HOLD_SPEED_STOP)
            {
                return 0;
            }

            if(control_verify_mode == 3U)
            {
                hold_duty = clampf_local(hold_duty,
                                         -(float)CONTROL_VERIFY_MODE3_DUTY_LIMIT,
                                         (float)CONTROL_VERIFY_MODE3_DUTY_LIMIT);
            }
            else if(hold_duty > 0.0f)
            {
                hold_duty = clampf_local(hold_duty, (float)POSITION_HOLD_MIN_DUTY, (float)POSITION_HOLD_MAX_DUTY);
            }
            else if(hold_duty < 0.0f)
            {
                hold_duty = clampf_local(hold_duty, -(float)POSITION_HOLD_MAX_DUTY, -(float)POSITION_HOLD_MIN_DUTY);
            }

            return (int16)hold_duty;
        }

        if(abs_error < POSITION_HOLD_ERR_STOP && abs_speed < POSITION_HOLD_SPEED_STOP)
        {
            return 0;
        }
    }
    else
    {
        position_hold_active[wheel] = 0;
    }

    return (int16)pid_out;
}

static void apply_startup_assist(PID_t *speed_pid[WHEEL_COUNT],
                                 float rotate_speed)
{
    if(startup_assist_ticks_remaining == 0U) return;

    for(uint8 i = 0U; i < WHEEL_COUNT; i++)
    {
        float target = speed_pid[i]->Target;
        float translation_target = (float)wheel_target_speed[i] - rotate_speed;

        /* Do not amplify a small yaw-only correction into a hard wheel kick. */
        if(absf_local(translation_target) < STARTUP_ASSIST_TARGET_MIN ||
           absf_local(target) < STARTUP_ASSIST_TARGET_MIN ||
           absf_local((float)encoder_speed[i]) > STARTUP_ASSIST_SPEED_MAX)
        {
            continue;
        }

        if(target > 0.0f && motor_output_duty[i] < startup_assist_min_duty[i])
        {
            motor_output_duty[i] = startup_assist_min_duty[i];
            startup_assist_applied_mask |= (uint8)(1U << i);
        }
        else if(target < 0.0f && motor_output_duty[i] > -startup_assist_min_duty[i])
        {
            motor_output_duty[i] = (int16)-startup_assist_min_duty[i];
            startup_assist_applied_mask |= (uint8)(1U << i);
        }
    }

    startup_assist_ticks_remaining--;
}

static void apply_pwm_vector_ramp(void)
{
    int32 delta[WHEEL_COUNT];
    int32 max_abs_delta = 0;
    uint8 i;

    for(i = 0U; i < WHEEL_COUNT; i++)
    {
        int32 abs_delta;
        pwm_ramp_desired[i] = motor_output_duty[i];
        delta[i] = (int32)pwm_ramp_desired[i] -
                   (int32)pwm_ramp_previous[i];
        abs_delta = delta[i] < 0 ? -delta[i] : delta[i];
        if(abs_delta > max_abs_delta) max_abs_delta = abs_delta;
    }

    if(pwm_ramp_ticks_remaining == 0U || pwm_ramp_max_delta == 0U)
    {
        pwm_ramp_scale_x1000 = 1000U;
        for(i = 0U; i < WHEEL_COUNT; i++)
            pwm_ramp_previous[i] = motor_output_duty[i];
        return;
    }

    if(max_abs_delta > (int32)pwm_ramp_max_delta)
    {
        pwm_ramp_scale_x1000 =
            (uint16)(((uint32)pwm_ramp_max_delta * 1000U) /
                     (uint32)max_abs_delta);
        if(pwm_ramp_scale_x1000 == 0U) pwm_ramp_scale_x1000 = 1U;
        for(i = 0U; i < WHEEL_COUNT; i++)
        {
            int32 step = delta[i] * (int32)pwm_ramp_scale_x1000;
            step += step >= 0 ? 500 : -500;
            step /= 1000;
            motor_output_duty[i] =
                (int16)((int32)pwm_ramp_previous[i] + step);
        }
    }
    else
    {
        pwm_ramp_scale_x1000 = 1000U;
    }

    for(i = 0U; i < WHEEL_COUNT; i++)
        pwm_ramp_previous[i] = motor_output_duty[i];
    pwm_ramp_ticks_remaining--;
}

static void reset_speed_pid_all(void)
{
    reset_pid_state(&motor0_speed_pid);
    reset_pid_state(&motor1_speed_pid);
    reset_pid_state(&motor2_speed_pid);
}

// 位置复位：清零位置误差及位置环PID内部状态
// 用于重新开始一次运动；快速制动不能调用，否则会丢失刹停位置误差
void position_reset(void)
{
    position_loop_divider = 0;

    reset_position_channel(0, &motor0_position_pid);
    reset_position_channel(1, &motor1_position_pid);
    reset_position_channel(2, &motor2_position_pid);

    reset_speed_pid_all();
    pid_window_reset();
}

void control_verify_init(uint8 mode)
{
    control_verify_mode = mode;
    back_forth_mode = 0;
    target_angle = 0.0f;
    target_linear_speed = 0.0f;
    wheel_target_speed[0] = 0;
    wheel_target_speed[1] = 0;
    wheel_target_speed[2] = 0;
    position_reset();
    imu963ra_yaw_hold_enable = 1U;
    imu963ra_yaw_target = imu963ra_yaw_angle;
    imu963ra_yaw_error = 0.0f;
    imu963ra_yaw_correction = 0.0f;
    manual_rotation_active = 0U;
    manual_rotation_speed = 0.0f;
}

void control_verify_set_mode5_limit(uint8 enable)
{
    control_verify_mode5_limit_enable = enable ? 1U : 0U;
}

void control_verify_set_mode5_speed(float speed)
{
    if(speed < 0.0f) speed = 0.0f;
    if(speed > 2000.0f) speed = 2000.0f;
    control_verify_mode5_speed = speed;
}

// 设置全向移动目标
// angle_deg是车体坐标系运动方向角，speed单位为编码器脉冲/5ms量级
void motion_set_velocity(float angle_deg, float speed)
{
    back_forth_mode = 0;
    target_angle = normalize_angle_360(angle_deg);
    target_linear_speed = clampf_local(speed, -2000.0f, 2000.0f);

    // 新运动从当前位置累计目标，避免沿用上一段运动的位置误差
    position_reset();
    imu963ra_yaw_target = imu963ra_yaw_angle;
    manual_rotation_active = 0U;
    manual_rotation_speed = 0.0f;
}

void motion_heading_lock_begin_at(float heading_target_deg)
{
    back_forth_mode = 0;
    target_linear_speed = 0.0f;
    wheel_target_speed[0] = 0;
    wheel_target_speed[1] = 0;
    wheel_target_speed[2] = 0;
    position_reset();
    imu963ra_yaw_hold_enable = 1U;
    imu963ra_yaw_target = heading_target_deg;
    imu963ra_yaw_error = 0.0f;
    imu963ra_yaw_correction = 0.0f;
    visual_yaw_sync_error_deg = 0.0f;
    visual_yaw_sync_step_deg = 0.0f;
    motion_heading_lock_active = 1U;
    manual_rotation_active = 0U;
    manual_rotation_speed = 0.0f;
}

void motion_heading_lock_begin(void)
{
    motion_heading_lock_begin_at(imu963ra_yaw_angle);
}

void motion_heading_lock_update(float angle_deg, float speed)
{
    back_forth_mode = 0;
    target_angle = normalize_angle_360(angle_deg);
    target_linear_speed = clampf_local(speed, -2000.0f, 2000.0f);
    imu963ra_yaw_hold_enable = 1U;
    motion_heading_lock_active = 1U;
}

void motion_heading_lock_correct_from_visual(float relative_heading_deg)
{
    uint32 primask;
    float expected_yaw;
    float sync_error;
    float sync_step;

    primask = interrupt_global_disable();
    if(!motion_heading_lock_active || !imu963ra_ready)
    {
        visual_yaw_sync_error_deg = 0.0f;
        visual_yaw_sync_step_deg = 0.0f;
        interrupt_global_enable(primask);
        return;
    }

    expected_yaw = imu963ra_yaw_target + relative_heading_deg;
    sync_error = normalize_angle_180(expected_yaw - imu963ra_yaw_angle);
    visual_yaw_sync_error_deg = sync_error;
    if(absf_local(sync_error) <= VISUAL_YAW_SYNC_DEADBAND)
    {
        sync_step = 0.0f;
    }
    else
    {
        sync_step = clampf_local(sync_error * VISUAL_YAW_SYNC_GAIN,
                                 -VISUAL_YAW_SYNC_MAX_STEP,
                                 VISUAL_YAW_SYNC_MAX_STEP);
        imu963ra_yaw_angle += sync_step;
    }
    visual_yaw_sync_step_deg = sync_step;
    interrupt_global_enable(primask);
}

void motion_heading_lock_stop(void)
{
    float heading_target = imu963ra_yaw_target;

    motion_fast_brake();
    imu963ra_yaw_hold_enable = 1U;
    imu963ra_yaw_target = heading_target;
    motion_heading_lock_active = 1U;
}

void motion_heading_lock_rebase_position(void)
{
    position_reset();
}

void motion_heading_lock_release(void)
{
    motion_heading_lock_active = 0U;
    imu963ra_yaw_target = imu963ra_yaw_angle;
    imu963ra_yaw_error = 0.0f;
    imu963ra_yaw_correction = 0.0f;
    visual_yaw_sync_error_deg = 0.0f;
    visual_yaw_sync_step_deg = 0.0f;
}

void motion_set_yaw_hold_enable(uint8 enable)
{
    uint32 primask = interrupt_global_disable();

    imu963ra_yaw_hold_enable = enable ? 1U : 0U;
    if(!imu963ra_yaw_hold_enable)
    {
        imu963ra_yaw_error = 0.0f;
        imu963ra_yaw_correction = 0.0f;
    }
    interrupt_global_enable(primask);
}

void motion_set_manual_rotation(float speed)
{
    uint32 primask = interrupt_global_disable();

    manual_rotation_speed = clampf_local(speed, -2000.0f, 2000.0f);
    manual_rotation_active = 1U;
    motion_heading_lock_active = 0U;
    imu963ra_yaw_hold_enable = 0U;
    target_angle = 0.0f;
    target_linear_speed = 0.0f;
    position_reset();
    interrupt_global_enable(primask);
}

void motion_stop_manual_rotation(void)
{
    motion_stop_manual_rotation_at(imu963ra_yaw_angle);
}

void motion_stop_manual_rotation_at(float heading_target_deg)
{
    uint32 primask = interrupt_global_disable();

    manual_rotation_active = 0U;
    manual_rotation_speed = 0.0f;
    motion_fast_brake();
    imu963ra_yaw_hold_enable = 1U;
    imu963ra_yaw_target = heading_target_deg;
    motion_heading_lock_active = 1U;
    interrupt_global_enable(primask);
}

void motion_get_wheel_total_counts(int32 out[3])
{
    uint32 primask;

    if(out == 0) return;
    primask = interrupt_global_disable();
    out[0] = wheel_total_count[0];
    out[1] = wheel_total_count[1];
    out[2] = wheel_total_count[2];
    interrupt_global_enable(primask);
}

float motion_get_mm_per_count(void)
{
    return ODOM_PULSE_TO_MM;
}

void motion_startup_assist_begin(uint16 duration_ms)
{
    uint32 primask = interrupt_global_disable();
    uint32 ticks = ((uint32)duration_ms + STARTUP_ASSIST_PERIOD_MS - 1U) /
                   STARTUP_ASSIST_PERIOD_MS;

    if(ticks > 65535U) ticks = 65535U;
    startup_assist_ticks_remaining = (uint16)ticks;
    startup_assist_applied_mask = 0U;
    interrupt_global_enable(primask);
}

void motion_startup_assist_cancel(void)
{
    uint32 primask = interrupt_global_disable();

    startup_assist_ticks_remaining = 0U;
    startup_assist_applied_mask = 0U;
    interrupt_global_enable(primask);
}

void motion_startup_assist_get_status(motion_startup_assist_status_t *out)
{
    uint32 primask;

    if(out == 0) return;
    primask = interrupt_global_disable();
    out->active = startup_assist_ticks_remaining > 0U ? 1U : 0U;
    out->applied_mask = startup_assist_applied_mask;
    out->remaining_ms = (uint16)(startup_assist_ticks_remaining *
                                 STARTUP_ASSIST_PERIOD_MS);
    interrupt_global_enable(primask);
}

void motion_pwm_ramp_begin(uint16 duration_ms, uint16 max_delta_per_5ms)
{
    uint32 primask = interrupt_global_disable();
    uint32 ticks = ((uint32)duration_ms + PWM_VECTOR_RAMP_PERIOD_MS - 1U) /
                   PWM_VECTOR_RAMP_PERIOD_MS;
    uint8 i;

    if(ticks > 65535U) ticks = 65535U;
    pwm_ramp_ticks_remaining = (uint16)ticks;
    pwm_ramp_max_delta = max_delta_per_5ms;
    pwm_ramp_scale_x1000 = 1000U;
    for(i = 0U; i < WHEEL_COUNT; i++)
    {
        pwm_ramp_previous[i] = 0;
        pwm_ramp_desired[i] = 0;
    }
    interrupt_global_enable(primask);
}

void motion_pwm_ramp_cancel(void)
{
    uint32 primask = interrupt_global_disable();
    uint8 i;

    pwm_ramp_ticks_remaining = 0U;
    pwm_ramp_max_delta = 0U;
    pwm_ramp_scale_x1000 = 1000U;
    for(i = 0U; i < WHEEL_COUNT; i++)
    {
        pwm_ramp_previous[i] = 0;
        pwm_ramp_desired[i] = 0;
    }
    interrupt_global_enable(primask);
}

float motion_get_wheel_cpr(void)
{
    return WHEEL_CPR;
}

float motion_get_wheel_diameter_mm(void)
{
    return WHEEL_DIAMETER_MM;
}

float motion_get_odometry_scale(void)
{
    return ODOMETRY_QUICK_SCALE;
}

void motion_get_debug_snapshot(motion_debug_snapshot_t *out)
{
    uint32 primask;
    PID_t *speed_pid[WHEEL_COUNT] = {&motor0_speed_pid, &motor1_speed_pid, &motor2_speed_pid};

    if(out == 0) return;
    primask = interrupt_global_disable();
    out->target_angle_deg = target_angle;
    out->target_speed = target_linear_speed;
    out->yaw_deg = imu963ra_yaw_angle;
    out->yaw_target_deg = imu963ra_yaw_target;
    out->yaw_error_deg = imu963ra_yaw_error;
    out->yaw_correction = imu963ra_yaw_correction;
    out->gyro_z_dps = imu963ra_gyro_z_dps;
    out->visual_sync_error_deg = visual_yaw_sync_error_deg;
    out->visual_sync_step_deg = visual_yaw_sync_step_deg;
    out->wheel_target[0] = wheel_target_speed[0];
    out->wheel_target[1] = wheel_target_speed[1];
    out->wheel_target[2] = wheel_target_speed[2];
    out->wheel_pid_target[0] = (int16)motor0_speed_pid.Target;
    out->wheel_pid_target[1] = (int16)motor1_speed_pid.Target;
    out->wheel_pid_target[2] = (int16)motor2_speed_pid.Target;
    out->wheel_encoder[0] = encoder_speed[0];
    out->wheel_encoder[1] = encoder_speed[1];
    out->wheel_encoder[2] = encoder_speed[2];
    out->encoder_rotate = (int16)(((int32)encoder_speed[0] +
                                   (int32)encoder_speed[1] +
                                   (int32)encoder_speed[2]) / 3);
    out->wheel_pwm[0] = motor_output_duty[0];
    out->wheel_pwm[1] = motor_output_duty[1];
    out->wheel_pwm[2] = motor_output_duty[2];

    out->yaw_p_output = yaw_p_output;
    out->yaw_d_output = yaw_d_output;
    out->yaw_raw_output = yaw_raw_output;
    out->yaw_saturated = yaw_output_saturated;
    out->pwm_ramp_active = pwm_ramp_ticks_remaining > 0U ? 1U : 0U;
    out->pwm_ramp_remaining_ms =
        (uint16)(pwm_ramp_ticks_remaining * PWM_VECTOR_RAMP_PERIOD_MS);
    out->pwm_ramp_max_delta = pwm_ramp_max_delta;
    out->pwm_ramp_scale_x1000 = pwm_ramp_scale_x1000;
    out->window_samples = pid_window.samples;

    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        out->position_target[i] = position_target[i];
        out->position_actual[i] = position_actual[i];
        out->position_error[i] = position_error[i];
        out->position_raw_output[i] = position_raw_output[i];
        out->position_limit[i] = position_dynamic_limit[i];
        out->position_comp[i] = position_speed_comp[i];
        out->position_limit_flags[i] = position_limit_flags[i];
        out->speed_error[i] = speed_pid[i]->Error0;
        out->speed_error_int[i] = speed_pid[i]->ErrorInt;
        out->speed_p_output[i] = speed_pid[i]->Kp * speed_pid[i]->Error0;
        out->speed_i_output[i] = speed_pid[i]->Ki * speed_pid[i]->ErrorInt;
        out->speed_d_output[i] = speed_pid[i]->Kd * (speed_pid[i]->Error0 - speed_pid[i]->Error1);
        out->speed_raw_output[i] = speed_pid[i]->Out;
        out->duty_source[i] = motor_duty_source[i];
        out->pwm_ramp_desired[i] = pwm_ramp_desired[i];
        out->window_speed_error_abs_avg[i] = (pid_window.samples > 0U) ?
            (pid_window.speed_error_abs_sum[i] / (float)pid_window.samples) : 0.0f;
        out->window_speed_error_abs_max[i] = pid_window.speed_error_abs_max[i];
        out->window_pwm_min[i] = (pid_window.samples > 0U) ? pid_window.pwm_min[i] : 0;
        out->window_pwm_max[i] = (pid_window.samples > 0U) ? pid_window.pwm_max[i] : 0;
        out->window_position_clip_count[i] = pid_window.position_clip_count[i];
        out->window_position_slew_count[i] = pid_window.position_slew_count[i];
        out->window_integral_cap_count[i] = pid_window.integral_cap_count[i];
        out->window_output_cap_count[i] = pid_window.output_cap_count[i];
        out->window_position_hold_count[i] = pid_window.position_hold_count[i];
    }

    pid_window_reset();
    interrupt_global_enable(primask);
}

// 快速制动：目标速度立即归零，但保留位置累计误差
// 下一次5ms控制会由速度环和静止位置保持共同制动，而不是断PWM滑行
void motion_fast_brake(void)
{
    back_forth_mode = 0;
    target_linear_speed = 0.0f;
    wheel_target_speed[0] = 0;
    wheel_target_speed[1] = 0;
    wheel_target_speed[2] = 0;
    imu963ra_yaw_target = imu963ra_yaw_angle;
    imu963ra_yaw_correction = 0.0f;
    startup_assist_ticks_remaining = 0U;
    startup_assist_applied_mask = 0U;
    pwm_ramp_ticks_remaining = 0U;
    pwm_ramp_max_delta = 0U;
    pwm_ramp_scale_x1000 = 1000U;
    reset_speed_pid_all();
}

// 硬停：直接清状态并关闭PWM，适合保护和调试，不用于追求最短制动距离
void motion_emergency_stop(void)
{
    manual_rotation_active = 0U;
    manual_rotation_speed = 0.0f;
    target_linear_speed = 0.0f;
    startup_assist_ticks_remaining = 0U;
    startup_assist_applied_mask = 0U;
    pwm_ramp_ticks_remaining = 0U;
    pwm_ramp_max_delta = 0U;
    pwm_ramp_scale_x1000 = 1000U;
    position_reset();
    p_set(0, 0);
    p_set(1, 0);
    p_set(2, 0);
}

// 5ms中断：位置环（外环）和速度环（内环）串级控制
void ai_tuner_update_pid(float kp, float ki, float kd)
{
    motor1_speed_pid.Kp = kp;
    motor1_speed_pid.Ki = ki;
    motor1_speed_pid.Kd = kd;
    reset_pid_state(&motor1_speed_pid);
}

static void ai_tuner_send_status(void)
{
    char status[128];

    snprintf(status, sizeof(status),
             "# STATUS: P=%.3f I=%.3f D=%.3f SP=%.1f ACT=%d OUT=%.1f\r\n",
             motor1_speed_pid.Kp,
             motor1_speed_pid.Ki,
             motor1_speed_pid.Kd,
             motor1_speed_pid.Target,
             (int)encoder_speed[1],
             motor1_speed_pid.Out);
    uart_write_string(AI_TUNER_UART, status);
}

static void ai_tuner_process_command(char *cmd)
{
    float new_kp = motor1_speed_pid.Kp;
    float new_ki = motor1_speed_pid.Ki;
    float new_kd = motor1_speed_pid.Kd;
    float new_sp = 0.0f;
    float new_angle = target_angle;
    char *value_text;

    while(*cmd == ' ')
    {
        cmd++;
    }

    if(*cmd == '\0')
    {
        return;
    }

    if(strncmp(cmd, "SET ", 4) == 0)
    {
        value_text = cmd + 4;

        if(sscanf(value_text, "P:%f I:%f D:%f", &new_kp, &new_ki, &new_kd) != 3)
        {
            if(sscanf(value_text, "KP:%f KI:%f KD:%f", &new_kp, &new_ki, &new_kd) != 3)
            {
                uart_write_string(AI_TUNER_UART, "# ERROR: SET P:x I:y D:z\r\n");
                return;
            }
        }

        if(new_kp > 0.0f && new_kp <= 100.0f &&
           new_ki >= 0.0f && new_ki <= 50.0f &&
           new_kd >= 0.0f && new_kd <= 50.0f)
        {
            ai_tuner_update_pid(new_kp, new_ki, new_kd);
            ai_tuner_send_status();
        }
        else
        {
            uart_write_string(AI_TUNER_UART, "# ERROR: PID out of range\r\n");
        }
    }
    else if(strncmp(cmd, "PID ", 4) == 0)
    {
        if(sscanf(cmd + 4, "%f %f %f", &new_kp, &new_ki, &new_kd) == 3)
        {
            ai_tuner_update_pid(new_kp, new_ki, new_kd);
            ai_tuner_send_status();
        }
        else
        {
            uart_write_string(AI_TUNER_UART, "# ERROR: PID p i d\r\n");
        }
    }
    else if(strncmp(cmd, "SETPOINT", 8) == 0)
    {
        value_text = cmd + 8;
        while(*value_text == ' ' || *value_text == ':' || *value_text == '=')
        {
            value_text++;
        }

        if(sscanf(value_text, "%f", &new_sp) == 1 && new_sp >= -2000.0f && new_sp <= 2000.0f)
        {
            if(absf_local(new_sp) < POSITION_TARGET_DEADBAND)
            {
                motion_fast_brake();
                uart_write_string(AI_TUNER_UART, "# BRAKE OK\r\n");
            }
            else
            {
                motion_set_velocity(target_angle, new_sp);
                uart_write_string(AI_TUNER_UART, "# SETPOINT OK\r\n");
            }
        }
        else
        {
            uart_write_string(AI_TUNER_UART, "# ERROR: SETPOINT value\r\n");
        }
    }
    else if(strncmp(cmd, "MOVE ", 5) == 0)
    {
        value_text = cmd + 5;
        if(sscanf(value_text, "A:%f V:%f", &new_angle, &new_sp) != 2 &&
           sscanf(value_text, "%f %f", &new_angle, &new_sp) != 2)
        {
            uart_write_string(AI_TUNER_UART, "# ERROR: MOVE A:angle V:speed\r\n");
            return;
        }

        if(new_sp >= -2000.0f && new_sp <= 2000.0f)
        {
            if(absf_local(new_sp) < POSITION_TARGET_DEADBAND)
            {
                motion_fast_brake();
                uart_write_string(AI_TUNER_UART, "# BRAKE OK\r\n");
            }
            else
            {
                motion_set_velocity(new_angle, new_sp);
                uart_write_string(AI_TUNER_UART, "# MOVE OK\r\n");
            }
        }
        else
        {
            uart_write_string(AI_TUNER_UART, "# ERROR: MOVE speed\r\n");
        }
    }
    else if(strcmp(cmd, "BRAKE") == 0)
    {
        motion_fast_brake();
        uart_write_string(AI_TUNER_UART, "# BRAKE OK\r\n");
    }
    else if(strcmp(cmd, "RESET") == 0)
    {
        motion_emergency_stop();
        uart_write_string(AI_TUNER_UART, "# RESET OK\r\n");
    }
    else if(strcmp(cmd, "STATUS") == 0)
    {
        ai_tuner_send_status();
    }
}

void ai_tuner_uart1_rx_handler(void)
{
    uint8 data;

    while(uart_query_byte(AI_TUNER_UART, &data) != 0)
    {
        if(data == '\r')
        {
            continue;
        }

        if(data == '\n')
        {
            ai_tuner_cmd_buf[ai_tuner_cmd_len] = '\0';
            if(ai_tuner_cmd_len > 0 && !ai_tuner_cmd_ready)
            {
                strcpy(ai_tuner_ready_buf, ai_tuner_cmd_buf);
                ai_tuner_cmd_ready = 1;
            }
            ai_tuner_cmd_len = 0;
            continue;
        }

        if(ai_tuner_cmd_len < sizeof(ai_tuner_cmd_buf) - 1)
        {
            ai_tuner_cmd_buf[ai_tuner_cmd_len++] = (char)data;
        }
        else
        {
            ai_tuner_cmd_len = 0;
        }
    }
}

void ai_tuner_poll_command(void)
{
    char cmd[80];
    uint32 primask;

    if(!ai_tuner_cmd_ready)
    {
        return;
    }

    primask = interrupt_global_disable();
    strcpy(cmd, ai_tuner_ready_buf);
    ai_tuner_cmd_ready = 0;
    interrupt_global_enable(primask);

    ai_tuner_process_command(cmd);
}

void motor_control_5ms(void)
{
    PID_t *speed_pid[WHEEL_COUNT] = {&motor0_speed_pid, &motor1_speed_pid, &motor2_speed_pid};
#if !CIRCLE_MODE_ENABLE
    PID_t *position_pid[WHEEL_COUNT] = {&motor0_position_pid, &motor1_position_pid, &motor2_position_pid};
#endif
    float rotate_speed = 0.0f;
    float move_angle = target_angle;
    float move_speed = target_linear_speed;

    pit_count++;  // 中断计数
    read_encoder_speed();
    imu963ra_heading_update();
    odometry_update();

    // 停车检查：蓝牙0x09停车后保持PWM为0，直到0x0A恢复
    if(device_init_flag)
    {
        position_reset();
        imu963ra_yaw_target = imu963ra_yaw_angle;
        imu963ra_yaw_correction = 0.0f;
        p_set(0, 0);
        p_set(1, 0);
        p_set(2, 0);
        return;
    }

#if WHEEL_SPEED_LOOP_TEST_ENABLE
    wheel_target_speed[0] = WHEEL_SPEED_LOOP_TEST_TARGET;
    wheel_target_speed[1] = WHEEL_SPEED_LOOP_TEST_TARGET;
    wheel_target_speed[2] = WHEEL_SPEED_LOOP_TEST_TARGET;

    position_target[0] = position_actual[0];
    position_target[1] = position_actual[1];
    position_target[2] = position_actual[2];
    position_speed_comp[0] = 0.0f;
    position_speed_comp[1] = 0.0f;
    position_speed_comp[2] = 0.0f;
    position_hold_active[0] = 0;
    position_hold_active[1] = 0;
    position_hold_active[2] = 0;

    motor0_speed_pid.Target = (float)wheel_target_speed[0];
    motor1_speed_pid.Target = (float)wheel_target_speed[1];
    motor2_speed_pid.Target = (float)wheel_target_speed[2];
    motor0_speed_pid.Actual = encoder_speed[0];
    motor1_speed_pid.Actual = encoder_speed[1];
    motor2_speed_pid.Actual = encoder_speed[2];

    position_pid_update(&motor0_speed_pid);
    position_pid_update(&motor1_speed_pid);
    position_pid_update(&motor2_speed_pid);

    motor_output_duty[0] = get_motor_duty(0, motor0_speed_pid.Out);
    motor_output_duty[1] = get_motor_duty(1, motor1_speed_pid.Out);
    motor_output_duty[2] = get_motor_duty(2, motor2_speed_pid.Out);
    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        if(motor_output_duty[i] > WHEEL_SPEED_LOOP_TEST_DUTY_LIMIT)
        {
            motor_output_duty[i] = WHEEL_SPEED_LOOP_TEST_DUTY_LIMIT;
        }
        else if(motor_output_duty[i] < -WHEEL_SPEED_LOOP_TEST_DUTY_LIMIT)
        {
            motor_output_duty[i] = -WHEEL_SPEED_LOOP_TEST_DUTY_LIMIT;
        }
    }

    pid_window_update();
    p_set(0, motor_output_duty[0]);
    p_set(1, motor_output_duty[1]);
    p_set(2, motor_output_duty[2]);
    return;
#endif

#if CIRCLE_MODE_ENABLE
    // Circle mode: follow a fixed reference circle and pull position error back.
    static float circle_phase_rad = 0.0f;
    float path_step_rad;
    float target_x_mm;
    float target_y_mm;
    float error_x_mm;
    float error_y_mm;
    float corr_limit;
    float corr_x;
    float corr_y;
    float cmd_x;
    float cmd_y;
    float speed_limit;

    back_forth_mode = 0;

    path_step_rad = (absf_local(target_linear_speed) * ODOM_PULSE_TO_MM) / CIRCLE_RADIUS_MM;
    circle_phase_rad += path_step_rad;
    if(circle_phase_rad >= 6.2831852f)
    {
        circle_phase_rad -= 6.2831852f;
    }

    target_x_mm = CIRCLE_RADIUS_MM * sinf(circle_phase_rad);
    target_y_mm = CIRCLE_RADIUS_MM * (1.0f - cosf(circle_phase_rad));
    error_x_mm = target_x_mm - odom_world_x_mm;
    error_y_mm = target_y_mm - odom_world_y_mm;

    corr_limit = absf_local(target_linear_speed) * CIRCLE_CORR_LIMIT_RATIO;
    corr_x = clampf_local(error_x_mm * CIRCLE_POS_KP, -corr_limit, corr_limit);
    corr_y = clampf_local(error_y_mm * CIRCLE_POS_KP, -corr_limit, corr_limit);

    cmd_x = target_linear_speed * cosf(circle_phase_rad) + corr_x;
    cmd_y = target_linear_speed * sinf(circle_phase_rad) + corr_y;
    move_speed = sqrtf(cmd_x * cmd_x + cmd_y * cmd_y);
    speed_limit = absf_local(target_linear_speed) * CIRCLE_SPEED_MAX_RATIO;
    if(move_speed > speed_limit && move_speed > 1.0f)
    {
        float scale = speed_limit / move_speed;
        cmd_x *= scale;
        cmd_y *= scale;
        move_speed = speed_limit;
    }

    target_angle = normalize_angle_360(atan2f(cmd_y, cmd_x) * 180.0f / 3.1415926f);
    move_angle = target_angle - imu963ra_yaw_angle;
    rotate_speed = imu963ra_yaw_hold_calculate();
#else
    if(manual_rotation_active)
    {
        move_angle = 0.0f;
        move_speed = 0.0f;
        target_linear_speed = 0.0f;
        rotate_speed = manual_rotation_speed;
    }
    else if((control_verify_mode == 0U) && motion_heading_lock_active)
    {
        if(absf_local(target_linear_speed) < YAW_HOLD_MIN_SPEED)
        {
            rotate_speed = imu963ra_yaw_hold_calculate_stationary();
        }
        else
        {
            rotate_speed = imu963ra_yaw_hold_calculate();
        }
    }
    else if(control_verify_mode == 4U)
    {
        move_angle = 0.0f;
        move_speed = 0.0f;
        target_linear_speed = 0.0f;
        rotate_speed = imu963ra_yaw_hold_calculate_stationary();
    }
    else if(control_verify_mode == 5U)
    {
        move_angle = 90.0f;
        move_speed = control_verify_mode5_speed;
        target_linear_speed = control_verify_mode5_speed;
        rotate_speed = imu963ra_yaw_hold_calculate();
    }
    else
    {
        if(control_verify_mode == 3U)
        {
            move_angle = 0.0f;
            move_speed = 0.0f;
            target_linear_speed = 0.0f;
            imu963ra_yaw_target = imu963ra_yaw_angle;
        }
        rotate_speed = imu963ra_yaw_hold_calculate();
    }
#endif

    // 3. 运动学解算：由三轮公式统一分配平移和姿态纠偏
    k_calculate(move_angle, move_speed, rotate_speed, wheel_target_speed);

#if CIRCLE_MODE_ENABLE
    // 转圈时目标方向持续变化，不累计位置目标，防止位置误差不断增大
    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        position_error[i] = 0.0f;
        position_speed_comp[i] = 0.0f;
        position_target[i] = position_actual[i];
        last_wheel_target_speed[i] = (float)wheel_target_speed[i];
        position_hold_active[i] = 0;
    }
#else
    if(control_verify_mode == 4U)
    {
        for(uint8 i = 0; i < WHEEL_COUNT; i++)
        {
            position_error[i] = 0.0f;
            position_speed_comp[i] = 0.0f;
            position_target[i] = position_actual[i];
            last_wheel_target_speed[i] = (float)wheel_target_speed[i];
            position_hold_active[i] = 0;
        }
    }
    else
    {
        // 位置环：显式累计目标/实际位置，外环只输出受限速度补偿
        handle_target_change(position_pid);
        integrate_position_counter();
        update_position_loop(position_pid);
    }
#endif

    // 5. 速度环（内环）：目标 = 运动学解算 + 位置补偿
#if CIRCLE_MODE_ENABLE
    motor0_speed_pid.Target = (float)wheel_target_speed[0];
    motor1_speed_pid.Target = (float)wheel_target_speed[1];
    motor2_speed_pid.Target = (float)wheel_target_speed[2];
#else
    if(control_verify_mode == 4U)
    {
        motor0_speed_pid.Target = (float)wheel_target_speed[0];
        motor1_speed_pid.Target = (float)wheel_target_speed[1];
        motor2_speed_pid.Target = (float)wheel_target_speed[2];
    }
    else
    {
        motor0_speed_pid.Target = (float)wheel_target_speed[0] + position_speed_comp[0];
        motor1_speed_pid.Target = (float)wheel_target_speed[1] + position_speed_comp[1];
        motor2_speed_pid.Target = (float)wheel_target_speed[2] + position_speed_comp[2];
    }
#endif

    motor0_speed_pid.Actual = encoder_speed[0];
    motor1_speed_pid.Actual = encoder_speed[1];
    motor2_speed_pid.Actual = encoder_speed[2];

    position_pid_update(&motor0_speed_pid);
    position_pid_update(&motor1_speed_pid);
    position_pid_update(&motor2_speed_pid);

    // 6. PID输出驱动电机；静止保持时直接根据位置误差输出回正PWM
    motor_output_duty[0] = get_motor_duty(0, motor0_speed_pid.Out);
    motor_output_duty[1] = get_motor_duty(1, motor1_speed_pid.Out);
    motor_output_duty[2] = get_motor_duty(2, motor2_speed_pid.Out);
    apply_startup_assist(speed_pid, rotate_speed);
    apply_pwm_vector_ramp();

    if((control_verify_mode == 5U) && control_verify_mode5_limit_enable)
    {
        for(uint8 i = 0; i < WHEEL_COUNT; i++)
        {
            if(motor_output_duty[i] > CONTROL_VERIFY_MODE5_DUTY_LIMIT)
            {
                motor_output_duty[i] = CONTROL_VERIFY_MODE5_DUTY_LIMIT;
            }
            else if(motor_output_duty[i] < -CONTROL_VERIFY_MODE5_DUTY_LIMIT)
            {
                motor_output_duty[i] = -CONTROL_VERIFY_MODE5_DUTY_LIMIT;
            }
        }
    }

    pid_window_update();
    p_set(0, motor_output_duty[0]);
    p_set(1, motor_output_duty[1]);
    p_set(2, motor_output_duty[2]);
}

//======================================================================
// 五角星轨迹自动运行（{5/2} 星形多边形）
//======================================================================
// 上电后依次沿5个方向平移，画出五角星，用于验证全向移动方向
//
// 几何原理：五个顶点均匀分布在圆周上，每次跨过一个顶点连接
// 每条边的移动方向依次为288、144、0、216、72度，相邻方向差144度
// 机器人全向移动时无需改变车身航向，只改变平移方向
//======================================================================

#define PENTAGRAM_SPEED            300.0f    // wheel-speed command for each edge
#define PENTAGRAM_TICKS_PER_EDGE   200U     // 200 * 5ms = 1000ms per edge
#define PENTAGRAM_VERTEX_PAUSE     100U      // 顶点停顿时间：100个5ms周期等于0.5秒
#define PENTAGRAM_EDGE_COUNT       5U        // 五角星边数
#define PENTAGRAM_STARTUP_GUARD    200U      // 上电保护期：200个5ms周期等于1秒

// 五角星5条边的移动方向（标准化到0至360度）
static const float pentagram_dir[PENTAGRAM_EDGE_COUNT] =
{
    288.0f,  // 第1边：右下
    144.0f,  // 第2边：左上
    0.0f,    // 第3边：正右
    216.0f,  // 第4边：左下
    72.0f,   // 第5边：右上
};

uint8 pentagram_enable = 0;
uint8 pentagram_state = PENTA_IDLE;
uint8 pentagram_edge_current = 0;

void pentagram_run_task(void)
{
    static uint8  state = PENTA_IDLE;
    static uint32 tick_start = 0;
    static uint8  edge_index = 0;
    static uint8  started = 0;

    uint8  current_edge;
    uint8  is_pause_state;

    // 同步供app_status、VOFA和屏幕读取的全局状态
    pentagram_state = state;
    pentagram_edge_current = edge_index;

    // 上电保护期：pit_count 未满 PENTAGRAM_STARTUP_GUARD 时不启动
    if(pit_count < PENTAGRAM_STARTUP_GUARD)
    {
        return;
    }

    // 未使能且尚未开始时不执行任何动作
    if(!pentagram_enable && state == PENTA_IDLE)
    {
        return;
    }

    // 首次启动
    if(!started)
    {
        started = 1;
        state = PENTA_EDGE_0;
        edge_index = 0;
        tick_start = pit_count;

        // 开启IMU航向保持并锁定当前航向
        imu963ra_yaw_hold_enable = 1;
        pentagram_heading_target = imu963ra_yaw_angle;
        motion_heading_lock_active = 1U;
        imu963ra_yaw_target = pentagram_heading_target;

        // 里程计清零，方便观察轨迹
        odometry_reset();
        position_reset();

        // 设置第一条边的方向
        motion_set_velocity(pentagram_dir[0], PENTAGRAM_SPEED);
        imu963ra_yaw_target = pentagram_heading_target;
        return;
    }

    // 判断当前状态类型
    is_pause_state = (state >= PENTA_PAUSE_0 && state <= PENTA_PAUSE_4);

    if(is_pause_state)
    {
        // 顶点停顿：等待PENTAGRAM_VERTEX_PAUSE个控制周期
        if((pit_count - tick_start) >= PENTAGRAM_VERTEX_PAUSE)
        {
            // 停顿结束后进入下一条边
            edge_index++;
            if(edge_index >= PENTAGRAM_EDGE_COUNT)
            {
                // 所有边完成后强制刹停，清零PWM且不保留位置保持
                motion_emergency_stop();
                imu963ra_yaw_target = pentagram_heading_target;
                state = PENTA_DONE;
                return;
            }

            // 开始下一条边
            state = PENTA_EDGE_0 + edge_index;
            tick_start = pit_count;
            motion_set_velocity(pentagram_dir[edge_index], PENTAGRAM_SPEED);
            imu963ra_yaw_target = pentagram_heading_target;
        }
        return;
    }

    if(state == PENTA_DONE)
    {
        // 已完成，保持停止
        return;
    }

    // 当前在边上运行：等待达到持续时间
    current_edge = state - PENTA_EDGE_0;
    if(current_edge < PENTAGRAM_EDGE_COUNT)
    {
        if((pit_count - tick_start) >= PENTAGRAM_TICKS_PER_EDGE)
        {
            // 到达顶点后刹车并短暂停顿
            motion_fast_brake();
            imu963ra_yaw_target = pentagram_heading_target;
            tick_start = pit_count;
            state = PENTA_PAUSE_0 + current_edge;
        }
    }
}

