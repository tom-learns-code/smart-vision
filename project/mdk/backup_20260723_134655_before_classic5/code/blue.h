#ifndef __BLUE_H
#define __BLUE_H

#include "zf_common_headfile.h"

//=========================================================
// 蓝牙按键调参协议（单字节命令）
//=========================================================
// 使用方法：手机蓝牙串口APP设置按钮，点一下发对应HEX字节
//
// 按键   发送HEX   功能
// -----------------------------------------------
//  1      0x01    速度环Kp + 0.5
//  2      0x02    速度环Ki + 0.005
//  3      0x03    速度环Kd + 0.1
//  4      0x04    前馈K(斜率) + 0.5
//  5      0x05    前馈BIAS(偏置) + 1.0
//  6      0x06    位置环Kp + 0.1
//  7      0x07    位置环Ki + 0.001
//  8      0x08    目标速度 + 10
// -----------------------------------------------
//  1长按   0x11    速度环Kp - 0.5
//  2长按   0x12    速度环Ki - 0.005
//  3长按   0x13    速度环Kd - 0.1
//  4长按   0x14    前馈K(斜率) - 0.5
//  5长按   0x15    前馈BIAS(偏置) - 1.0
//  6长按   0x16    位置环Kp - 0.1
//  7长按   0x17    位置环Ki - 0.001
//  8长按   0x18    目标速度 - 10
// -----------------------------------------------
//  9      0x09    停车，清零所有积分和输出
//  0      0x0A    恢复运行
//  A      0x0B    开关来回跑模式
//  B      0x0C    来回跑速度 +10
//  B长按   0x1C    来回跑速度 -10
//=========================================================

// 命令字节定义
#define BT_SPEED_KP_ADD    0x01
#define BT_SPEED_KI_ADD    0x02
#define BT_SPEED_KD_ADD    0x03
#define BT_FF_K_ADD        0x04
#define BT_FF_BIAS_ADD     0x05
#define BT_POS_KP_ADD      0x06
#define BT_POS_KI_ADD      0x07
#define BT_SPEED_ADD       0x08
#define BT_STOP            0x09
#define BT_START           0x0A
#define BT_BACK_FORTH      0x0B
#define BT_BF_SPEED_ADD    0x0C

#define BT_SPEED_KP_SUB    0x11
#define BT_SPEED_KI_SUB    0x12
#define BT_SPEED_KD_SUB    0x13
#define BT_FF_K_SUB        0x14
#define BT_FF_BIAS_SUB     0x15
#define BT_POS_KP_SUB      0x16
#define BT_POS_KI_SUB      0x17
#define BT_SPEED_SUB       0x18
#define BT_BF_SPEED_SUB    0x1C

// 步进量
#define STEP_SPEED_KP      0.5f
#define STEP_SPEED_KI      0.005f
#define STEP_SPEED_KD      0.1f
#define STEP_FF_K          0.5f
#define STEP_FF_BIAS       1.0f
#define STEP_POS_KP        0.1f
#define STEP_POS_KI        0.001f
#define STEP_SPEED         10.0f
#define STEP_ANGLE         15.0f

// 蓝牙命令处理函数（在UART4中断中调用）
void bt_cmd_handler(unsigned char cmd);

// 停车/初始化
void bt_device_init(void);

// 停车标志（isr.c中可查询）
extern int device_init_flag;

#endif
