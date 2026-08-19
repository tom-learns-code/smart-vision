#include "zf_common_headfile.h"
#include "blue.h"
#include "pid.h"
#include "isr.h"

int device_init_flag = 0;

// 蓝牙启动保护：初始化后前2秒（400次5ms）内忽略蓝牙命令，防止上电乱码误触发停车
static uint32 bt_start_tick = 0;
#define BT_STARTUP_GUARD_MS  2000  // 保护期2秒
#define BT_5MS_COUNT         400   // 2秒 = 400次5ms

// 停车/初始化：清零所有PID积分项、增量累加输出和电机输出
void bt_device_init(void)
{
    motor0_speed_pid.ErrorInt = 0; motor0_speed_pid.Error0 = 0; motor0_speed_pid.Error1 = 0; motor0_speed_pid.Out = 0;
    motor1_speed_pid.ErrorInt = 0; motor1_speed_pid.Error0 = 0; motor1_speed_pid.Error1 = 0; motor1_speed_pid.Out = 0;
    motor2_speed_pid.ErrorInt = 0; motor2_speed_pid.Error0 = 0; motor2_speed_pid.Error1 = 0; motor2_speed_pid.Out = 0;

    motor0_position_pid.ErrorInt = 0; motor0_position_pid.Error0 = 0; motor0_position_pid.Error1 = 0; motor0_position_pid.Out = 0;
    motor1_position_pid.ErrorInt = 0; motor1_position_pid.Error0 = 0; motor1_position_pid.Error1 = 0; motor1_position_pid.Out = 0;
    motor2_position_pid.ErrorInt = 0; motor2_position_pid.Error0 = 0; motor2_position_pid.Error1 = 0; motor2_position_pid.Out = 0;

    position_reset();

    p_set(0, 0);
    p_set(1, 0);
    p_set(2, 0);
}

// 三轮速度环PID同步宏
#define SYNC_SPEED_PID() do { \
    motor1_speed_pid.Kp = motor0_speed_pid.Kp; \
    motor1_speed_pid.Ki = motor0_speed_pid.Ki; \
    motor1_speed_pid.Kd = motor0_speed_pid.Kd; \
    motor2_speed_pid.Kp = motor0_speed_pid.Kp; \
    motor2_speed_pid.Ki = motor0_speed_pid.Ki; \
    motor2_speed_pid.Kd = motor0_speed_pid.Kd; \
} while(0)

// 三轮位置环PID同步宏
#define SYNC_POS_PID() do { \
    motor1_position_pid.Kp = motor0_position_pid.Kp; \
    motor1_position_pid.Ki = motor0_position_pid.Ki; \
    motor1_position_pid.Kd = motor0_position_pid.Kd; \
    motor2_position_pid.Kp = motor0_position_pid.Kp; \
    motor2_position_pid.Ki = motor0_position_pid.Ki; \
    motor2_position_pid.Kd = motor0_position_pid.Kd; \
} while(0)

// 蓝牙命令处理
void bt_cmd_handler(unsigned char cmd)
{
    // 启动保护期内忽略蓝牙命令，防止上电乱码误触发停车
    if(pit_count < BT_5MS_COUNT)
        return;

    switch(cmd)
    {
        // ======== 加 ========
        case BT_SPEED_KP_ADD:
            motor0_speed_pid.Kp += STEP_SPEED_KP;
            if(motor0_speed_pid.Kp > 50.0f) motor0_speed_pid.Kp = 50.0f;  // 上限保护
            SYNC_SPEED_PID();
            break;
        case BT_SPEED_KI_ADD:
            motor0_speed_pid.Ki += STEP_SPEED_KI;
            if(motor0_speed_pid.Ki > 5.0f) motor0_speed_pid.Ki = 5.0f;
            SYNC_SPEED_PID();
            break;
        case BT_SPEED_KD_ADD:
            motor0_speed_pid.Kd += STEP_SPEED_KD;
            if(motor0_speed_pid.Kd > 20.0f) motor0_speed_pid.Kd = 20.0f;
            SYNC_SPEED_PID();
            break;
        case BT_FF_K_ADD:
            //feedforward_K += STEP_FF_K;  // 前馈已注释
            //if(feedforward_K > 50.0f) feedforward_K = 50.0f;
            break;
        case BT_FF_BIAS_ADD:
            //feedforward_bias += STEP_FF_BIAS;  // 前馈已注释
            //if(feedforward_bias > 200.0f) feedforward_bias = 200.0f;
            break;
        case BT_POS_KP_ADD:
            motor0_position_pid.Kp += STEP_POS_KP;
            if(motor0_position_pid.Kp > 10.0f) motor0_position_pid.Kp = 10.0f;
            SYNC_POS_PID();
            break;
        case BT_POS_KI_ADD:
            motor0_position_pid.Ki += STEP_POS_KI;
            if(motor0_position_pid.Ki > 1.0f) motor0_position_pid.Ki = 1.0f;
            SYNC_POS_PID();
            break;
        case BT_SPEED_ADD:
            target_linear_speed += STEP_SPEED;
            if(target_linear_speed > 500.0f) target_linear_speed = 500.0f;
            break;
        case BT_BACK_FORTH:
            back_forth_mode = 0;
            break;
        case BT_BF_SPEED_ADD:
            back_forth_speed += 10.0f;
            if(back_forth_speed > 500.0f) back_forth_speed = 500.0f;
            break;

        // ======== 减 ========
        case BT_SPEED_KP_SUB:
            motor0_speed_pid.Kp -= STEP_SPEED_KP;
            if(motor0_speed_pid.Kp < 0) motor0_speed_pid.Kp = 0;
            SYNC_SPEED_PID();
            break;
        case BT_SPEED_KI_SUB:
            motor0_speed_pid.Ki -= STEP_SPEED_KI;
            if(motor0_speed_pid.Ki < 0) motor0_speed_pid.Ki = 0;
            SYNC_SPEED_PID();
            break;
        case BT_SPEED_KD_SUB:
            motor0_speed_pid.Kd -= STEP_SPEED_KD;
            if(motor0_speed_pid.Kd < 0) motor0_speed_pid.Kd = 0;
            SYNC_SPEED_PID();
            break;
        case BT_FF_K_SUB:
            //feedforward_K -= STEP_FF_K;  // 前馈已注释
            //if(feedforward_K < 0) feedforward_K = 0;
            break;
        case BT_FF_BIAS_SUB:
            //feedforward_bias -= STEP_FF_BIAS;  // 前馈已注释
            //if(feedforward_bias < 0) feedforward_bias = 0;
            break;
        case BT_POS_KP_SUB:
            motor0_position_pid.Kp -= STEP_POS_KP;
            if(motor0_position_pid.Kp < 0) motor0_position_pid.Kp = 0;
            SYNC_POS_PID();
            break;
        case BT_POS_KI_SUB:
            motor0_position_pid.Ki -= STEP_POS_KI;
            if(motor0_position_pid.Ki < 0) motor0_position_pid.Ki = 0;
            SYNC_POS_PID();
            break;
        case BT_SPEED_SUB:
            target_linear_speed -= STEP_SPEED;
            break;
        case BT_BF_SPEED_SUB:
            back_forth_speed -= 10.0f;
            if(back_forth_speed < 0) back_forth_speed = 0;
            break;

        // ======== 停车/恢复 ========
        case BT_STOP:
            device_init_flag = 1;
            bt_device_init();
            break;
        case BT_START:
            device_init_flag = 0;
            break;

        default:
            break;
    }
}
