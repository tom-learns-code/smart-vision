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

#define WHEEL_COUNT                (3U)//3U = 无符号整数3
#define POSITION_LOOP_DIV          (1U)
#define POSITION_ERR_LIMIT         (3000.0f)
#define POSITION_ERR_DEADBAND      (1.0f)
#define POSITION_TARGET_DEADBAND   (5.0f)
#define POSITION_COMP_STEP_LIMIT   (6.0f)
#define POSITION_COMP_MOVING_RATIO (0.45f)
#define POSITION_COMP_MOVING_MAX   (30.0f)
#define POSITION_HOLD_ERR_START    (8.0f)
#define POSITION_HOLD_ERR_STOP     (3.0f)
#define POSITION_HOLD_SPEED_STOP   (5.0f)
#define POSITION_HOLD_MIN_DUTY     (0)
#define POSITION_HOLD_MAX_DUTY     (1000)
#define POSITION_HOLD_DUTY_KP      (5.0f)
#define POSITION_HOLD_DUTY_KD      (30.0f)
#define AI_TUNER_UART              (DEBUG_UART_INDEX)
#define CONTROL_PERIOD_S           (0.005f)
#define IMU963RA_ZERO_BIAS_SAMPLES (1500U)
#define IMU963RA_ZERO_BIAS_DELAY_MS (2U)
#define IMU963RA_ZERO_BIAS_MAX_DPS (5.0f)
#define IMU963RA_GYRO_DEADBAND_DPS (1.20f)
#define IMU963RA_BIAS_TRACK_SPEED  (2.0f)
#define IMU963RA_BIAS_TRACK_MAX_DPS (2.50f)
#define IMU963RA_BIAS_TRACK_ALPHA  (0.002f)
#define YAW_HOLD_MIN_SPEED         (5.0f)
#define YAW_HOLD_DEADBAND_DEG      (0.20f)
#define YAW_HOLD_KP                (6.00f)
#define YAW_HOLD_KD                (0.80f)
#define YAW_HOLD_OUT_LIMIT         (160.0f)
#define YAW_HOLD_ROT_SIGN          (-1.0f)
#define CIRCLE_MODE_ENABLE         (1U)
#define CIRCLE_RADIUS_MM           (350.0f)
#define CIRCLE_POS_KP              (0.35f)
#define CIRCLE_CORR_LIMIT_RATIO    (0.80f)
#define CIRCLE_SPEED_MAX_RATIO     (1.30f)
#define BACK_FORTH_FORWARD_ANGLE   (0.0f)
#define BACK_FORTH_BACKWARD_ANGLE  (180.0f)
#ifndef ODOMETRY_QUICK_SCALE
#define ODOMETRY_QUICK_SCALE       (1.0f)
#endif
#ifndef WHEEL_DIAMETER_MM
#define WHEEL_DIAMETER_MM          (65.0f)
#endif
#ifndef WHEEL_CPR
#define WHEEL_CPR                  (2048.0f)
#endif
#define ODOM_PULSE_TO_MM           ((3.1415926f * WHEEL_DIAMETER_MM / WHEEL_CPR) * ODOMETRY_QUICK_SCALE)
#define ODOM_DIRECTION_MIN_MM      (0.5f)
#define ODOM_YAW_SIGN              (1.0f)

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
    
		 {QTIMER1_ENCODER2, -1, PWM2_MODULE0_CHA_C6, C7,  1},  // rear uses former left encoder
    {QTIMER2_ENCODER1,  -1, PWM2_MODULE1_CHA_C8, C9,  0},  // left front uses former rear encoder, flip feedback sign
    {QTIMER2_ENCODER2, -1, PWM2_MODULE2_CHA_C10, C11, 1},  // right front motor wiring is mirrored
};
//定义了一个叫 wheel_hw_t 的类型（5个属性的盒子），创建了3个这样的盒子组成数组， 分别存了后轮、左前轮、右前轮的硬件接线信息

#define MOTOR_PWM_FREQUENCY       (17000)

void motor_pwm_init(void)
{
    pwm_init(PWM2_MODULE0_CHA_C6, MOTOR_PWM_FREQUENCY, 0);
    pwm_init(PWM2_MODULE2_CHA_C10, MOTOR_PWM_FREQUENCY, 0);
    pwm_init(PWM2_MODULE1_CHA_C8, MOTOR_PWM_FREQUENCY, 0);
}
static const float wheel_install_angle[WHEEL_COUNT] =
{
    -90.0f,  // rear
    30.0f,   // left front
    150.0f,  // right front
};




//========================================================= Y车模PID速度环参数 =========================================================

// 编码器数据（每5ms读取一次脉冲差值）
int16 encoder_speed[3] = {0, 0, 0};

// 三个轮子速度目标值（由运动学解算得到）
int16 wheel_target_speed[3] = {0, 0, 0};

// 速度环PID参数 - 标准位置式PID（前馈已注释）50 0.02 3.5
PID_t motor0_speed_pid = {
    .Kp = 18,
    .Ki = 0.03f,
    .Kd = 0,
    .ErrorIntMax = 1500.0f,  // 积分抗饱和：Ki*ErrorIntMax=1245
    .OutMax = 10000,
    .OutMin = -10000,
};//. 指定给哪个字段赋值
PID_t motor1_speed_pid = {
    .Kp = 18,
    .Ki = 0.03,
    .Kd = 0,
    .ErrorIntMax = 1500.0f,
    .OutMax = 10000,
    .OutMin = -10000,
};
PID_t motor2_speed_pid = {
    .Kp = 20,
    .Ki = 0.03f,   // match left wheel first; raise only if steady-state error remains
    .Kd = 0.0,
    .ErrorIntMax = 1500.0f,
    .OutMax = 10000,
    .OutMin = -10000,
};

//========================================================= Y车模位置环参数 =========================================================

// 位置环PID参数（外环，输出为速度补偿量，叠加到速度环目标上）
// 位置环是慢环，参数应比速度环温和
PID_t motor0_position_pid = {
    .Kp = 0.7f,    // 位置比例
    .Ki = 0.0f,    // 不用积分，避免超调
    .Kd = 0.0f,    // 位置环Kd=0：位置误差的微分≈速度误差，和速度环冲突
    .ErrorIntMax = 0.0f,  // Ki=0，此项无效
    .OutMax = 90.0f,   // 补偿上限
    .OutMin = -90.0f,
};
PID_t motor1_position_pid = {
    .Kp = 0.7f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .ErrorIntMax = 0.0f,
    .OutMax = 90.0f,
    .OutMin = -90.0f,
};
PID_t motor2_position_pid = {
    .Kp = 0.7f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .ErrorIntMax = 0.0f,
    .OutMax = 90.0f,
    .OutMin = -90.0f,
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

uint8 imu963ra_ready = 0;
uint8 imu963ra_yaw_hold_enable = 1;
float imu963ra_gyro_z_offset = 0.0f;
float imu963ra_gyro_z_dps = 0.0f;
float imu963ra_yaw_angle = 0.0f;
float imu963ra_yaw_target = 0.0f;
float imu963ra_yaw_error = 0.0f;
float imu963ra_yaw_correction = 0.0f;
float odom_body_delta_x_mm = 0.0f;
float odom_body_delta_y_mm = 0.0f;
float odom_world_x_mm = 0.0f;
float odom_world_y_mm = 0.0f;
float odom_total_distance_mm = 0.0f;
float odom_displacement_mm = 0.0f;
float odom_body_direction_deg = 0.0f;
float odom_move_direction_deg = 0.0f;
float odom_instant_direction_deg = 0.0f;

// 前馈模型参数（已注释，不使用前馈）
//float feedforward_K = 12.0f;       // 前馈斜率系数（轮0、轮1）
//float feedforward_K3 = 12.1f;      // 前馈斜率系数（轮2，略有差异）
//float feedforward_bias = 60.20f;   // 前馈偏置（克服静摩擦/死区补偿）

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
int back_forth_mode = 0;           // 0=手动速度模式, 1=来回跑模式
float back_forth_speed = 100.0f;   // 来回跑的速度大小（蓝牙可调）
uint32 back_forth_period = 1000;   // 来回跑半周期（单位：5ms次数，1000=5秒）

// PIT中断计数器（用于验证中断是否运行）
volatile uint32 pit_count = 0;

static char ai_tuner_cmd_buf[80];
static char ai_tuner_ready_buf[80];
static uint8 ai_tuner_cmd_len = 0;
static volatile uint8 ai_tuner_cmd_ready = 0;

// 度转弧度
#define DEG2RAD(deg)    ((deg) * 3.1415926f / 180.0f)

static float last_wheel_target_speed[3] = {0.0f, 0.0f, 0.0f};
static float circle_phase_rad = 0.0f;
static uint8 position_loop_divider = 0;//位置环分频，作用：让位置环跑得比速度环慢
static uint8 position_hold_active[3] = {0, 0, 0};//作用：标记每个轮子是否处于**"位置保持模式"**
//0 = 正常速度控制
//1 = 位置保持模式（车停了，用PD控制锁住位置不动）
static float clampf_local(float value, float min_value, float max_value)
{
    if(value > max_value) return max_value;
    if(value < min_value) return min_value;
    return value;
}

static float absf_local(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float normalize_angle_360(float angle)
{
    while(angle < 0.0f) angle += 360.0f;
    while(angle >= 360.0f) angle -= 360.0f;
    return angle;
}

void odometry_reset(void)
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

static void odometry_update(void)
{
    float body_x_pulse = 0.0f;
    float body_y_pulse = 0.0f;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float world_dx;
    float world_dy;
    float step_distance;

    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        float rad = DEG2RAD(wheel_install_angle[i]);
        body_x_pulse += (float)encoder_speed[i] * cosf(rad);
        body_y_pulse += (float)encoder_speed[i] * sinf(rad);
    }

    body_x_pulse *= (2.0f / 3.0f);
    body_y_pulse *= (2.0f / 3.0f);

    odom_body_delta_x_mm = body_x_pulse * ODOM_PULSE_TO_MM;
    odom_body_delta_y_mm = body_y_pulse * ODOM_PULSE_TO_MM;

    yaw_rad = DEG2RAD(imu963ra_yaw_angle * ODOM_YAW_SIGN);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    world_dx = odom_body_delta_x_mm * cos_yaw - odom_body_delta_y_mm * sin_yaw;
    world_dy = odom_body_delta_x_mm * sin_yaw + odom_body_delta_y_mm * cos_yaw;

    odom_world_x_mm += world_dx;
    odom_world_y_mm += world_dy;

    step_distance = sqrtf(world_dx * world_dx + world_dy * world_dy);
    odom_total_distance_mm += step_distance;
    odom_displacement_mm = sqrtf(odom_world_x_mm * odom_world_x_mm + odom_world_y_mm * odom_world_y_mm);

    if(step_distance >= ODOM_DIRECTION_MIN_MM)
    {
        odom_body_direction_deg = normalize_angle_360(atan2f(odom_body_delta_y_mm, odom_body_delta_x_mm) * 180.0f / 3.1415926f);
        odom_instant_direction_deg = normalize_angle_360(atan2f(world_dy, world_dx) * 180.0f / 3.1415926f);
    }

    if(odom_displacement_mm >= ODOM_DIRECTION_MIN_MM)
    {
        odom_move_direction_deg = normalize_angle_360(atan2f(odom_world_y_mm, odom_world_x_mm) * 180.0f / 3.1415926f);
    }
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

    // 平移时三轮速度求和约为0；只有自转时才会同向叠加。用实际旋转分量跟踪陀螺零偏，避免ERR慢慢自增。
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
        return 0.0f;
    }

    if(absf_local(target_linear_speed) < YAW_HOLD_MIN_SPEED)
    {
        imu963ra_yaw_target = imu963ra_yaw_angle;
        imu963ra_yaw_error = 0.0f;
        imu963ra_yaw_correction = 0.0f;
        return 0.0f;
    }

    imu963ra_yaw_error = imu963ra_yaw_target - imu963ra_yaw_angle;
    if(absf_local(imu963ra_yaw_error) < YAW_HOLD_DEADBAND_DEG)
    {
        imu963ra_yaw_error = 0.0f;
    }

    imu963ra_yaw_correction = (imu963ra_yaw_error * YAW_HOLD_KP)
                             - (imu963ra_gyro_z_dps * YAW_HOLD_KD);
    imu963ra_yaw_correction *= YAW_HOLD_ROT_SIGN;
    imu963ra_yaw_correction = clampf_local(imu963ra_yaw_correction,
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
    position_target[index] = 0.0f;
    position_actual[index] = 0.0f;
    last_wheel_target_speed[index] = 0.0f;
    position_hold_active[index] = 0;
    reset_pid_state(position_pid);
}

// 运动学解算：给定角度和线速度，计算三轮目标速度
static void k_calculate(float angle, float speed, float rotate_speed, int16 motor_speed[3])//static表示其被限制在当前.c文件
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
            continue;
        }

        position_pid[i]->Target = position_error[i];
        position_pid[i]->Actual = 0.0f;
        position_pid_update(position_pid[i]);
        {
            float comp = position_pid[i]->Out;
            float base_target = absf_local((float)wheel_target_speed[i]);

            if(base_target >= POSITION_TARGET_DEADBAND)
            {
                float comp_limit = base_target * POSITION_COMP_MOVING_RATIO;
                if(comp_limit > POSITION_COMP_MOVING_MAX)
                {
                    comp_limit = POSITION_COMP_MOVING_MAX;
                }
                comp = clampf_local(comp, -comp_limit, comp_limit);
            }

            position_speed_comp[i] = slew_limit(position_speed_comp[i], comp, POSITION_COMP_STEP_LIMIT);
        }
    }
}

static int16 get_motor_duty(uint8 wheel, float pid_out)
{
    float hold_duty;
    float abs_error = absf_local(position_error[wheel]);
    float abs_speed = absf_local((float)encoder_speed[wheel]);

    if(absf_local((float)wheel_target_speed[wheel]) < POSITION_TARGET_DEADBAND)
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
            hold_duty = position_error[wheel] * POSITION_HOLD_DUTY_KP
                      - (float)encoder_speed[wheel] * POSITION_HOLD_DUTY_KD;

            if(abs_error < POSITION_HOLD_ERR_STOP && abs_speed < POSITION_HOLD_SPEED_STOP)
            {
                return 0;
            }

            if(hold_duty > 0.0f)
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

// 位置复位：清零位置误差及位置环PID内部状态
void position_reset(void)
{
    position_loop_divider = 0;

    reset_position_channel(0, &motor0_position_pid);
    reset_position_channel(1, &motor1_position_pid);
    reset_position_channel(2, &motor2_position_pid);

    // 清零速度环PID内部累积
    reset_pid_state(&motor0_speed_pid);
    reset_pid_state(&motor1_speed_pid);
    reset_pid_state(&motor2_speed_pid);
}

// 5ms中断：位置环(外环) + 速度环(内环) 串级控制
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
            back_forth_mode = 0;
            target_linear_speed = new_sp;
            position_reset();
            uart_write_string(AI_TUNER_UART, "# SETPOINT OK\r\n");
        }
        else
        {
            uart_write_string(AI_TUNER_UART, "# ERROR: SETPOINT value\r\n");
        }
    }
    else if(strcmp(cmd, "RESET") == 0)
    {
        target_linear_speed = 0.0f;
        position_reset();
        p_set(0, 0);
        p_set(1, 0);
        p_set(2, 0);
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

    // 停车检查：蓝牙发0x09停车后，保持PWM=0直到发0x0A恢复
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
    rotate_speed = imu963ra_yaw_hold_calculate();
#endif

    // 3. 运动学解算：平移与姿态纠偏统一由三轮公式分配
    k_calculate(move_angle, move_speed, rotate_speed, wheel_target_speed);

#if CIRCLE_MODE_ENABLE
    // 转圈时目标方向不断变化，不做位置目标累计，防止位置ERR越积越大。
    for(uint8 i = 0; i < WHEEL_COUNT; i++)
    {
        position_error[i] = 0.0f;
        position_speed_comp[i] = 0.0f;
        position_target[i] = position_actual[i];
        last_wheel_target_speed[i] = (float)wheel_target_speed[i];
        position_hold_active[i] = 0;
    }
#else
    // 位置环：显式累计目标/实际位置，外环只输出受限速度补偿
    handle_target_change(position_pid);
    integrate_position_counter();
    update_position_loop(position_pid);
#endif

    // 5. 速度环（内环）：目标 = 运动学解算 + 位置补偿
#if CIRCLE_MODE_ENABLE
    motor0_speed_pid.Target = (float)wheel_target_speed[0];
    motor1_speed_pid.Target = (float)wheel_target_speed[1];
    motor2_speed_pid.Target = (float)wheel_target_speed[2];
#else
    motor0_speed_pid.Target = (float)wheel_target_speed[0] + position_speed_comp[0];
    motor1_speed_pid.Target = (float)wheel_target_speed[1] + position_speed_comp[1];
    motor2_speed_pid.Target = (float)wheel_target_speed[2] + position_speed_comp[2];
#endif

    motor0_speed_pid.Actual = encoder_speed[0];
    motor1_speed_pid.Actual = encoder_speed[1];
    motor2_speed_pid.Actual = encoder_speed[2];

    position_pid_update(&motor0_speed_pid);
    position_pid_update(&motor1_speed_pid);
    position_pid_update(&motor2_speed_pid);

    // 6. PID输出驱动电机；静止位置保持时直接用位置误差给回正占空比
    motor_output_duty[0] = get_motor_duty(0, motor0_speed_pid.Out);
    motor_output_duty[1] = get_motor_duty(1, motor1_speed_pid.Out);
    motor_output_duty[2] = get_motor_duty(2, motor2_speed_pid.Out);

    p_set(0, motor_output_duty[0]);
    p_set(1, motor_output_duty[1]);
    p_set(2, motor_output_duty[2]);
}
