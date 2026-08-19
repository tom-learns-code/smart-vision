#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "zf_common_headfile.h"

/*
 * 智能视觉小车集中配置（UTF-8）
 *
 * 调比赛行为时优先只改本文件。算法内部阈值仍留在各模块中，避免把
 * “普通比赛配置”和“底层控制器参数”混在一起造成误改。
 */

/* ============================== 构建标识 ============================== */
#define APP_MCU_BUILD_ID                    (7100969UL) /* 每次正式修改后递增，屏幕/串口据此确认固件版本 */
#define APP_MCU_STAGE_NAME                  "MULTI_PUSH_WALL_RECOVERY" /* 多格推物、墙边连续段、稳定帧恢复 */

/* ============================== 离线三键界面参数 ============================== */
/* 运行模式和串口/屏幕界面开关集中放在main.c“用户常用运行配置”中。 */
#define APP_BUTTON_HW_ENABLE                (1U) /* 1启用真实按键GPIO，0仅保留软件菜单状态机 */
#define APP_BUTTON_PREV_PIN                 (C12) /* 屏幕模式上一个选项；UART模式忽略 */
#define APP_BUTTON_NEXT_PIN                 (C13) /* 屏幕模式下一个选项；两种界面长按均紧急停车 */
#define APP_BUTTON_OK_PIN                   (C14) /* 屏幕模式短按确认；两种界面长按均准备或启动 */
#define APP_BUTTON_ACTIVE_LEVEL             (GPIO_LOW) /* 三个按键均为低电平按下，内部上拉 */
#define APP_BUTTON_DEBOUNCE_MS              (30U) /* 硬件按键消抖时间 */
#define APP_BUTTON_SHORT_MAX_MS             (450U) /* 短按最长时间 */
#define APP_BUTTON_LONG_MIN_MS              (900U) /* 长按判定时间 */
#define APP_BUTTON_DOUBLE_GAP_MS            (350U) /* 旧双按键兼容参数，三键离线菜单不使用双击 */
#define APP_OFFLINE_MENU_REFRESH_MS         (500U) /* 无事件时状态刷新周期；按键事件仍立即刷新 */
#define APP_MATCH_REQUIRE_PHYSICAL_START    (1U) /* 1禁止串口Y直接发车，必须长按C14完成最后启动 */

/* ============================== 已验证硬件与运动物理量 ============================== */
#define APP_WHEEL_DIAMETER_MM               (58.0f) /* 轮胎直径 */
#define APP_WHEEL_CPR                       (4096.0f) /* 编码器四倍频后每轮一圈脉冲数 */
#define APP_ODOMETRY_SCALE                  (0.915f) /* 实测里程比例 */
#define APP_WHEEL_CENTER_RADIUS_MM          (100.0f) /* 轮子到车体中心距离 */
#define APP_GRID_CELL_MM                    (200U) /* 赛道单格边长 */
#define APP_REAR_ENCODER_SIGN               (-1) /* 后轮编码器逻辑方向 */
#define APP_LEFT_ENCODER_SIGN               (-1) /* 左前轮编码器逻辑方向 */
#define APP_RIGHT_ENCODER_SIGN              (-1) /* 右前轮编码器逻辑方向 */
#define APP_REAR_FORWARD_LEVEL              (1U) /* 后轮正向DIR电平，C11 */
#define APP_LEFT_FORWARD_LEVEL              (0U) /* 左前轮正向DIR电平，C9；C9=0顺时针 */
#define APP_RIGHT_FORWARD_LEVEL             (0U) /* 右前轮正向DIR电平，C7 */

/* ============================== 三张连续地图配置 ============================== */
#define APP_RACE_ROUND_COUNT                (3U) /* 比赛固定连续三张图 */
#define APP_RACE_PRESET_MAX                 (6U) /* 预设结构体硬上限，新增时不能超过6套 */
#define APP_RACE_DEFAULT_PRESET             (0U) /* 上电默认预设下标 */

typedef enum
{
    APP_RACE_STRATEGY_SAFE = 0,             /* 每格强校准，最稳 */
    APP_RACE_STRATEGY_NORMAL,               /* 自由路段适度合并，物体附近强校准 */
    APP_RACE_STRATEGY_SPRINT                /* 直线节点合并，冲刺使用 */
} app_race_strategy_t;

typedef enum
{
    APP_RACE_ALGO_CLASSIC = 0,              /* 图一：纯箱子，无图片/数字识别，无炸弹 */
    APP_RACE_ALGO_IMAGE_ONLY,               /* 图二：箱子和目标需识别，但地图无炸弹 */
    APP_RACE_ALGO_BOMB_IMAGE                /* 图三：炸弹、箱子和目标识别 */
} app_race_algorithm_t;

typedef struct
{
    uint8 run;                              /* 1执行本图，0出基地加载地图后立即返航 */
    app_race_strategy_t strategy;           /* 本图运动策略 */
    uint16 speed;                           /* 本图速度，只建议100/120/150 */
    app_race_algorithm_t algorithm;         /* 本图显式算法，不再根据炸弹数量猜测 */
} app_race_round_config_t;

typedef struct
{
    const char *name;                       /* 串口/屏幕显示的预设名 */
    app_race_round_config_t round[APP_RACE_ROUND_COUNT]; /* 三张图独立配置 */
} app_race_preset_t;

/* 预设数组只在app_config.c定义；数量由数组长度自动计算。 */
extern const app_race_preset_t app_race_presets[];
extern const uint8 app_race_preset_count;

/* ============================== 完整比赛状态机 ============================== */
#define APP_MATCH_EXIT_DISTANCE_MM          (450U) /* 发车固定向车头平移45cm，完全忽略基地视觉 */
#define APP_MATCH_EXIT_SPEED                (120U) /* 发车速度 */
#define APP_MATCH_BASE_MAP_HEADING_DEG      (90.0f) /* 开局车头默认地图角度，基地视觉角度禁用 */
#define APP_MATCH_SKIP_RETURN_DIRECTION_DEG (270.0f) /* SKIP返航地图方向：保持原航向向车后平移 */
#define APP_MATCH_MAP_WAIT_MS               (10000U) /* 非空全图帧等待窗口，超时后继续请求而非锁死 */
#define APP_MATCH_BETWEEN_ROUND_MS          (2000U) /* 每两张图之间固定等待2秒 */
#define APP_MATCH_FINISH_SCAN_SAMPLES       (3U) /* 完赛后FULL_MAP复核帧数 */
#define APP_MATCH_RETURN_RETRIES            (2U) /* 返航动作重试次数 */
#define APP_MATCH_MISSION_ARM_RETRIES       (6U) /* 任务启动恢复次数 */
#define APP_MATCH_RUNTIME_REPLANS           (6U) /* 运行中动态重规划次数 */
#define APP_MATCH_TELEMETRY_LEVEL           (1U) /* 0静默 1事件 2动作 3控制 4全部 */

/* ============================== 点到点与赛道采集 ============================== */
#define APP_POINT_DEFAULT_SPEED             (100U) /* 点到点菜单默认速度 */
#define APP_POINT_APPROACH_SPEED            (120U) /* 平移接近终点时速度 */
#define APP_POINT_ROTATE_APPROACH_DEG        (20.0f) /* 旋转剩余该角度时进入慢速 */
#define APP_POINT_ROTATE_APPROACH_SPEED      (40U) /* 旋转接近阶段速度 */
#define APP_POINT_ROTATE_BRAKE_LEAD_DEG      (7.0f) /* 旋转制动提前角 */
#define APP_POINT_STARTUP_ASSIST_MS          (300U) /* 静摩擦起步辅助时长 */
#define APP_POINT_STALL_WINDOW_MS            (1000U) /* 堵转判定窗口 */
#define APP_POINT_STALL_MIN_PROGRESS_MM      (5.0f) /* 堵转窗口内最小位移 */

typedef struct
{
    uint16 speed;                            /* 速度档，单位为编码器脉冲/5ms */
    uint16 forward_mm;                       /* 向前制动提前量，数值越大越早刹车 */
    uint16 right_mm;                         /* 向右制动提前量 */
    uint16 back_mm;                          /* 向后制动提前量 */
    uint16 left_mm;                          /* 向左制动提前量 */
} app_point_brake_profile_t;

extern const app_point_brake_profile_t app_point_brake_profiles[];
extern const uint8 app_point_brake_profile_count;

#define APP_TRACK_INTER_STEP_MS              (500U) /* 自动采集相邻动作间隔 */
#define APP_TRACK_MAP_FRAMES                 (5U) /* 地图一致性采样帧数 */
#define APP_TRACK_TELEMETRY_MS               (3000U) /* 赛道采集遥测周期 */

/* ============================== 串口与视觉链路 ============================== */
#define APP_PC_LINK_REPORT_MS                (5000U) /* 视觉离线状态重复报告周期 */
#define APP_PC_RUN_TELEMETRY_MS              (250U) /* 比赛运行遥测周期 */
#define APP_PC_BASE_TELEMETRY_MS             (125U) /* 发车阶段双倍频率遥测 */
#define APP_PC_POINT_TELEMETRY_MS            (3000U) /* 点到点遥测周期 */
#define APP_PC_TX_RING_SIZE                  (8192U) /* UART8非阻塞发送队列 */
#define APP_PC_RX_RING_SIZE                  (128U) /* UART8接收队列 */
#define APP_PC_FP2_PERIODIC_REPORT_ENABLE    (0U) /* 第二摄像头仅事件上报，关闭周期刷屏 */
#define APP_GLOBAL_VISION_TIMEOUT_MS         (500U) /* 全局视觉在线超时 */
#define APP_FP2_RESULT_TIMEOUT_MS           (10000U) /* 三帧均值、双图片模型、复位及结果取回总等待 */
#define APP_FP2_REQUEST_RETRY_MS             (1000U) /* 等待识别时使用同一请求号重发，兼顾首次丢包和结果取回 */
#define APP_FP2_REQUEST_MAX_RETRIES          (8U) /* 10秒总超时内最多重发次数 */

/* ============================== 蜂鸣器事件提示 ============================== */
#define APP_BUZZER_ENABLE                    (1U) /* 总开关 */
#define APP_BUZZER_PIN                       (B11) /* 推挽输出，高电平响 */
#define APP_BUZZER_BOOT_MS                   (1000U) /* 上电提示 */
#define APP_BUZZER_COMMAND_MS                (200U) /* 菜单收到指令 */
#define APP_BUZZER_PREPARE_START_MS          (1000U) /* 准备启动车 */
#define APP_BUZZER_MAP_REQUEST_MS            (500U) /* 请求全图帧 */
#define APP_BUZZER_SOLVE_DONE_MS             (500U) /* 路径解算完成 */
#define APP_BUZZER_NODE_REACHED_MS           (200U) /* 到达运动节点 */
#define APP_BUZZER_LOCKED_MS                 (1000U) /* 锁止或严重故障 */
#define APP_BUZZER_ALIGN_PULSE_MS            (200U) /* 强矫正双响单次时长 */
#define APP_BUZZER_ALIGN_GAP_MS              (200U) /* 强矫正双响间隔 */

/* ============================== 运动策略常用调参入口 ============================== */
#define APP_FOLLOWER_NORMAL_MAX_CELLS        (13U) /* NORMAL/SPRINT自由直线最大合并格数 */
#define APP_FOLLOWER_STRICT_AXIS_MM          (20.0f) /* 箱子/炸弹八领域强校准单轴阈值 */
#define APP_FOLLOWER_STRICT_DIST_MM          (30.0f) /* 箱子/炸弹八领域强校准距离阈值 */
#define APP_FOLLOWER_STRICT_ATTEMPTS         (3U) /* 强校准最大尝试次数 */
#define APP_FOLLOWER_RECOVERY_MAX_MM         (300U) /* 稳定位置与预期不一致时，允许主动恢复的最大距离 */
#define APP_FOLLOWER_EXIT_ALIGN_MAX_MM       (300U) /* 离开单边墙或双边通道后的强校准最大距离 */
#define APP_MISSION_MULTI_PUSH_ENABLE        (1U) /* 1允许SPRINT把同方向推物拆成前n-1格连续推和末格精推 */
#define APP_MISSION_MULTI_PUSH_MIN_STEPS     (3U) /* 至少推3格才启用，2格拆分没有提速收益 */
#define APP_MISSION_MULTI_PUSH_MAX_CRUISE    (12U) /* 单次连续推最大格数，覆盖最长合法n-1段 */
#define APP_MISSION_PUSH_DRAIN_MS            (600U) /* 推物体后等待旧视觉数据排空 */
#define APP_MISSION_PUSH_STABLE_FRAMES       (4U) /* 推物体后稳定视觉帧数 */
#define APP_MISSION_OBSERVE_SPEED            (80U) /* 第一人称识别转向速度 */
#define APP_MISSION_OBSERVE_TIMEOUT_MS       (5000U) /* 识别姿态等待上限 */

#endif
