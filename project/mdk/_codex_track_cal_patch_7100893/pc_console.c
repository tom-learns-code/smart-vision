#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "blue.h"
#include "motion_control.h"
#include "vision_link.h"
#include "planner_service.h"
#include "mission_manager.h"
#include "point_test.h"
#include "track_calibration.h"
#include "pc_console.h"

#define PC_CONSOLE_BUILD_ID             (7100893UL)
#define PC_CONSOLE_UART                 (DEBUG_UART_INDEX)
#define PC_CONSOLE_UART_BASE            (LPUART8)
#define PC_CONSOLE_LINK_REPORT_TICKS    (1000U)
#define PC_CONSOLE_VERBOSE_TICKS        (200U)
#define PC_CONSOLE_MAP_TIMEOUT_TICKS    (600U)
#define PC_CONSOLE_DIRECT_POLL_LIMIT    (32U)
#define PC_CONSOLE_RX_RING_SIZE         (128U)
#define PC_CONSOLE_RX_RING_MASK         (PC_CONSOLE_RX_RING_SIZE - 1U)
#define PC_CONSOLE_TX_RING_SIZE         (8192U)
#define PC_CONSOLE_TX_RING_MASK         (PC_CONSOLE_TX_RING_SIZE - 1U)
#define PC_CONSOLE_TX_ISR_BUDGET        (16U)
#define PC_CONSOLE_RUN_TELEM_TICKS      (50U)  /* 250 ms */
#define PC_CONSOLE_POINT_TELEM_TICKS    (600U) /* 3 s */
#define PC_CONSOLE_TRACK_TELEM_TICKS    (600U) /* 3 s */

typedef enum
{
    PC_MENU_MAIN = 0,
    PC_MENU_PLANNER,
    PC_MENU_RUN,
    PC_MENU_POINT_MAIN,
    PC_MENU_POINT_MOVE,
    PC_MENU_POINT_ROTATE,
    PC_MENU_POINT_SENSOR,
    PC_MENU_POINT_SETTINGS,
    PC_MENU_TRACK_MAIN,
    PC_MENU_TRACK_MANUAL,
    PC_MENU_TRACK_AUTO,
    PC_MENU_TRACK_STATIONARY,
    PC_MENU_TRACK_VISION,
    PC_MENU_TRACK_SETTINGS
} pc_menu_state_t;

static uint8 g_app_mode = 0U;
static uint8 g_verbose = 0U;
static pc_menu_state_t g_menu_state = PC_MENU_MAIN;
static uint8 g_link_known = 0U;
static uint8 g_last_online = 0U;
static uint8 g_map_request_pending = 0U;
static uint32 g_map_request_tick = 0U;
static uint32 g_map_version_before_request = 0U;
static uint32 g_last_link_report_tick = 0U;
static uint32 g_last_verbose_tick = 0U;
static uint32 g_last_mission_event = 0U;
static uint32 g_last_run_report_tick = 0U;
static uint32 g_run_telem_sequence = 0U;
static uint8 g_run_telemetry = 1U;
static uint8 g_last_telem_action = 0xFFU;
static uint8 g_last_telem_waypoint = 0xFFU;
static uint8 g_last_telem_phase = 0xFFU;
static uint8 g_point_telemetry = 1U;
static uint32 g_last_point_report_tick = 0U;
static uint32 g_point_telem_sequence = 0U;
static uint32 g_last_point_event = 0U;
static uint32 g_last_track_event = 0U;
static uint32 g_last_track_record = 0U;
static uint32 g_last_track_report_tick = 0U;
static volatile uint8 g_uart8_rx_ring[PC_CONSOLE_RX_RING_SIZE];
static volatile uint16 g_uart8_rx_head = 0U;
static volatile uint16 g_uart8_rx_tail = 0U;
static volatile uint32 g_uart8_rx_drops = 0U;
static volatile uint8 g_uart8_tx_ring[PC_CONSOLE_TX_RING_SIZE];
static volatile uint16 g_uart8_tx_head = 0U;
static volatile uint16 g_uart8_tx_tail = 0U;
static volatile uint16 g_uart8_tx_peak = 0U;
static volatile uint32 g_uart8_tx_drops = 0U;

static uint16 pc_console_tx_queue_used(void)
{
    return (uint16)((g_uart8_tx_head - g_uart8_tx_tail) & PC_CONSOLE_TX_RING_MASK);
}

static void pc_console_tx_kick(void)
{
    if(g_uart8_tx_tail != g_uart8_tx_head)
    {
        LPUART_EnableInterrupts(PC_CONSOLE_UART_BASE,
                               (uint32)kLPUART_TxDataRegEmptyInterruptEnable);
    }
}

void pc_console_uart8_tx_isr(void)
{
    uint16 tail = g_uart8_tx_tail;
    uint16 budget = PC_CONSOLE_TX_ISR_BUDGET;

    while(budget-- > 0U && tail != g_uart8_tx_head &&
          (LPUART_GetStatusFlags(PC_CONSOLE_UART_BASE) &
           (uint32)kLPUART_TxDataRegEmptyFlag) != 0U)
    {
        LPUART_WriteByte(PC_CONSOLE_UART_BASE, g_uart8_tx_ring[tail]);
        tail = (uint16)((tail + 1U) & PC_CONSOLE_TX_RING_MASK);
    }
    g_uart8_tx_tail = tail;

    if(tail == g_uart8_tx_head)
    {
        LPUART_DisableInterrupts(PC_CONSOLE_UART_BASE,
                                (uint32)kLPUART_TxDataRegEmptyInterruptEnable);
    }
}

static void pc_console_write(const char *text)
{
    size_t length;
    uint16 head;
    uint16 free_space;
    uint16 used;
    size_t i;

    if(text == 0)
    {
        return;
    }

    length = strlen(text);
    if(length == 0U)
    {
        return;
    }

    head = g_uart8_tx_head;
    free_space = (uint16)((g_uart8_tx_tail - head - 1U) & PC_CONSOLE_TX_RING_MASK);
    if(length > (size_t)free_space)
    {
        g_uart8_tx_drops++;
        return;
    }

    for(i = 0U; i < length; i++)
    {
        g_uart8_tx_ring[head] = (uint8)text[i];
        head = (uint16)((head + 1U) & PC_CONSOLE_TX_RING_MASK);
    }
    g_uart8_tx_head = head;

    used = pc_console_tx_queue_used();
    if(used > g_uart8_tx_peak)
    {
        g_uart8_tx_peak = used;
    }

    pc_console_tx_kick();
}

static int32 pc_console_float_x10(float value)
{
    return (int32)(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
}

static void pc_console_format_x10(char *out, size_t size, int32 value_x10)
{
    uint32 magnitude;

    if(value_x10 < 0)
    {
        magnitude = (uint32)(-value_x10);
        snprintf(out, size, "-%lu.%lu",
                 (unsigned long)(magnitude / 10U),
                 (unsigned long)(magnitude % 10U));
    }
    else
    {
        magnitude = (uint32)value_x10;
        snprintf(out, size, "%lu.%lu",
                 (unsigned long)(magnitude / 10U),
                 (unsigned long)(magnitude % 10U));
    }
}

static int32 pc_console_float_round(float value)
{
    return (int32)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

static const char *pc_console_position_limit_name(uint8 flags)
{
    if(flags & MOTION_POS_LIMIT_MAX) return "MAX";
    if(flags & MOTION_POS_LIMIT_RATIO) return "RATIO";
    return "PID";
}

static const char *pc_console_duty_source_name(uint8 source)
{
    return (source == MOTION_DUTY_SOURCE_POS_HOLD) ? "POS_HOLD" : "SPD_PID";
}

static void pc_console_report_rx(const char *source, uint8 value)
{
    char text[96];
    const char *name = ".";

    if(value == '\r')
    {
        name = "CR";
    }
    else if(value == '\n')
    {
        name = "LF";
    }
    else if(value == '\t')
    {
        name = "TAB";
    }
    else if(value == ' ')
    {
        name = "SPACE";
    }
    else if(value >= 0x21U && value <= 0x7EU)
    {
        snprintf(text, sizeof(text),
                 "UART8_RX id=7100893 tx=D16 rx=D17 src=%s hex=%02X ascii='%c'\r\n",
                 source,
                 (unsigned int)value, (char)value);
        pc_console_write(text);
        return;
    }

    snprintf(text, sizeof(text),
             "UART8_RX id=7100893 tx=D16 rx=D17 src=%s hex=%02X ascii=%s\r\n",
             source,
             (unsigned int)value, name);
    pc_console_write(text);
}

void pc_console_uart8_rx_isr(void)
{
    uint8 value;

    while(uart_query_byte(PC_CONSOLE_UART, &value))
    {
        uint16 next = (uint16)((g_uart8_rx_head + 1U) & PC_CONSOLE_RX_RING_MASK);
        if(next == g_uart8_rx_tail)
        {
            g_uart8_rx_drops++;
            continue;
        }
        g_uart8_rx_ring[g_uart8_rx_head] = value;
        g_uart8_rx_head = next;
    }
}

static uint8 pc_console_uart8_ring_read(uint8 *value)
{
    uint16 tail = g_uart8_rx_tail;

    if(tail == g_uart8_rx_head)
    {
        return 0U;
    }
    *value = g_uart8_rx_ring[tail];
    g_uart8_rx_tail = (uint16)((tail + 1U) & PC_CONSOLE_RX_RING_MASK);
    return 1U;
}

static void pc_console_safe_stop(void)
{
    if(g_app_mode == 3U)
    {
        track_calibration_emergency_stop();
    }
    else if(g_app_mode == 2U)
    {
        point_test_emergency_stop();
    }
    else
    {
        mission_manager_emergency_stop();
    }
}

static void pc_console_print_main_menu(void)
{
    pc_console_write("\r\n=== SMART CAR MENU id=7100893 state=SAFE_IDLE_UART8_RX ===\r\n");
    pc_console_write("1  Request one FULL_MAP and print it\r\n");
    pc_console_write("2  Open planner/Action menu (dry run)\r\n");
    pc_console_write("3  Open guarded step-run menu\r\n");
    pc_console_write("4  Print vision-link status\r\n");
    pc_console_write("5  Print cached map\r\n");
    pc_console_write("V  Toggle quiet/verbose link reports\r\n");
    pc_console_write("X  Emergency stop\r\n");
    pc_console_write("H  Print this menu\r\n");
    pc_console_write("> ");
}

static void pc_console_print_planner_menu(void)
{
    pc_console_write("\r\n=== PLANNER MENU id=7100893 state=GRID_PLAN_MOTORS_LOCKED ===\r\n");
    pc_console_write("S  Solve cached map and print summary\r\n");
    pc_console_write("D  Solve cached map and print detailed Actions\r\n");
    pc_console_write("P  Print cached plan summary\r\n");
    pc_console_write("A  Print cached detailed Actions\r\n");
    pc_console_write("B  Back to main menu\r\n");
    pc_console_write("X  Emergency stop\r\n");
    pc_console_write("H  Print this menu\r\n");
    pc_console_write("> ");
}

static void pc_console_print_run_menu(void)
{
    char text[80];

    snprintf(text, sizeof(text),
             "\r\n=== RUN MENU id=7100893 mode=GRID_STEP_M4 speed=%d ===\r\n",
             (int)action_follower_get_speed());
    pc_console_write(text);
    pc_console_write("E  Arm current cached plan (motors remain stopped)\r\n");
    pc_console_write("N  Execute one FREE or PUSH grid step\r\n");
    pc_console_write("A  Auto-run all FREE/PUSH grids using pose+IMU checks\r\n");
    pc_console_write("P  Abort current grid step (replan required)\r\n");
    pc_console_write("C  Disabled after abort; solve and arm again\r\n");
    pc_console_write("R  Reset cursor (only before the first grid step)\r\n");
    pc_console_write("K  Disarm and hard stop\r\n");
    pc_console_write("Note: N uses FULL_MAP push verify; A has no push-map gate\r\n");
    pc_console_write("S  Print mission/follower status\r\n");
    snprintf(text, sizeof(text),
             "T  Toggle run telemetry (now %s, period=250ms)\r\n",
             g_run_telemetry ? "ON" : "OFF");
    pc_console_write(text);
    pc_console_write("B  Disarm and return to main menu\r\n");
    pc_console_write("X  Emergency stop\r\n");
    pc_console_write("H  Print this menu\r\n");
    pc_console_write("> ");
}

static void pc_console_print_point_main_menu(void)
{
    point_test_snapshot_t point;
    char text[256];

    point_test_get_snapshot(&point);
    snprintf(text, sizeof(text),
             "\r\n=== POINT TEST MENU id=7100893 state=%s origin=%u ===\r\n",
             point_test_state_name(point.state),
             (unsigned int)point.origin_valid);
    pc_console_write(text);
    pc_console_write("1  Translation point-to-point test\r\n");
    pc_console_write("2  In-place rotation calibration\r\n");
    pc_console_write("3  Locked sensor/encoder monitor\r\n");
    pc_console_write("4  Parameters and speed settings\r\n");
    pc_console_write("5  Print current/last result\r\n");
    pc_console_write("Z  Capture origin (M4 requires stable visual input)\r\n");
    snprintf(text, sizeof(text),
             "T  Toggle 3s telemetry (now %s)\r\n",
             g_point_telemetry ? "ON" : "OFF");
    pc_console_write(text);
    pc_console_write("X  Emergency stop and invalidate origin\r\n");
    pc_console_write("H  Print this menu\r\n> ");
}

static void pc_console_print_point_move_menu(void)
{
    point_test_snapshot_t point;
    char text[320];

    point_test_get_snapshot(&point);
    snprintf(text, sizeof(text),
             "\r\n=== TRANSLATION TEST state=%s dir=%s/%ddeg cells=%u dist=%umm speed=%u sensor=%s ===\r\n",
             point_test_state_name(point.state),
             point_test_direction_name(point.direction_index),
             (int)point.direction_deg,
             (unsigned int)point.cell_count,
             (unsigned int)point.target_distance_mm,
             (unsigned int)point.speed,
             point_test_sensor_name(point.sensor_mode));
    pc_console_write(text);
    pc_console_write("1 FWD  2 FRONT_RIGHT  3 RIGHT  4 BACK_RIGHT\r\n");
    pc_console_write("5 BACK 6 BACK_LEFT    7 LEFT   8 FRONT_LEFT\r\n");
    pc_console_write("G  Cycle distance 1/2/3/4 cells\r\n");
    pc_console_write("M  Cycle sensor M1/M2/M3/M4\r\n");
    pc_console_write("   M1=ENC+IMU  M2=ENC+YAW_OFF  M3=LIVE_VIS_STOP  M4=ENC+IMU_THEN_DELAY_VIS_CHECK\r\n");
    pc_console_write("V  Open speed/settings menu\r\n");
    pc_console_write("Z  Capture origin (M4 accepts only stable visual input)\r\n");
    pc_console_write("R  Run selected translation\r\n");
    pc_console_write("P  Print current/last result\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_point_rotate_menu(void)
{
    point_test_snapshot_t point;
    char text[256];

    point_test_get_snapshot(&point);
    snprintf(text, sizeof(text),
             "\r\n=== ROTATION TEST state=%s target=%udeg dir=%s stop=%s speed=%u ===\r\n",
             point_test_state_name(point.state),
             (unsigned int)point.target_rotation_deg,
             point.rotate_clockwise ? "CW" : "CCW",
             point_test_rotation_stop_name(point.rotate_stop),
             (unsigned int)point.speed);
    pc_console_write(text);
    pc_console_write("1  Target 90deg\r\n2  Target 180deg\r\n3  Target 360deg\r\n");
    pc_console_write("D  Toggle CW/CCW\r\n");
    pc_console_write("M  Toggle stop source IMU/ENCODER\r\n");
    pc_console_write("V  Open speed/settings menu\r\n");
    pc_console_write("Z  Capture origin (required before every run)\r\n");
    pc_console_write("R  Run selected rotation\r\n");
    pc_console_write("P  Print current/last result\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_point_sensor_menu(void)
{
    point_test_snapshot_t point;
    char text[224];

    point_test_get_snapshot(&point);
    snprintf(text, sizeof(text),
             "\r\n=== LOCKED SENSOR MONITOR state=%s vision=%s age=%lums ===\r\n",
             point_test_state_name(point.state),
             point.vision_live ? "LIVE" : "STALE",
             (unsigned long)point.vision_age_ms);
    pc_console_write(text);
    pc_console_write("Z  Zero local odometry and wheel accumulators\r\n");
    pc_console_write("P  Print one sensor snapshot\r\n");
    pc_console_write("T  Toggle 3s live monitor\r\n");
    pc_console_write("Use Z, then hand-turn one wheel exactly one revolution.\r\n");
    pc_console_write("Use Z, then hand-rotate chassis about 90deg.\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_point_settings_menu(void)
{
    point_test_snapshot_t point;
    char text[256];

    point_test_get_snapshot(&point);
    snprintf(text, sizeof(text),
             "\r\n=== POINT SETTINGS speed=%u cells=%u sensor=%s ===\r\n",
             (unsigned int)point.speed,
             (unsigned int)point.cell_count,
             point_test_sensor_name(point.sensor_mode));
    pc_console_write(text);
    pc_console_write("1  Set speed=100 counts/5ms\r\n");
    pc_console_write("2  Set speed=120 counts/5ms\r\n");
    pc_console_write("3  Set speed=150 counts/5ms\r\n");
    pc_console_write("G  Cycle distance 1/2/3/4 cells\r\n");
    pc_console_write("M  Cycle sensor M1/M2/M3/M4\r\n");
    pc_console_write("   M4 is the delayed-vision test; M3 remains legacy and must not be used for this test.\r\n");
    snprintf(text, sizeof(text),
             "P  Constants: cell=200mm wheel=%umm mm/count=%d.%05d center=100mm CPR=%u\r\n",
             (unsigned int)(motion_get_wheel_diameter_mm() + 0.5f),
             (int)point.mm_per_count,
             (int)(point.mm_per_count * 100000.0f + 0.5f),
             (unsigned int)(motion_get_wheel_cpr() + 0.5f));
    pc_console_write(text);
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_track_main_menu(void)
{
    track_cal_snapshot_t track;
    char text[256];

    track_calibration_get_snapshot(&track);
    snprintf(text, sizeof(text),
             "\r\n=== TRACK CAL MENU id=7100893 state=%s session=%lu record=%lu ===\r\n",
             track_calibration_state_name(track.state),
             (unsigned long)track.session,
             (unsigned long)track.record_sequence);
    pc_console_write(text);
    pc_console_write("1  Manual direction/distance acquisition\r\n");
    pc_console_write("2  One-key automatic route acquisition\r\n");
    pc_console_write("3  Stationary vision/IMU stability test\r\n");
    pc_console_write("4  Five-frame FULL_MAP consistency scan\r\n");
    pc_console_write("5  Speed/load/voltage settings\r\n");
    pc_console_write("6  Print session summary and last record\r\n");
    pc_console_write("Z  Verify stable visual origin (motors remain locked)\r\n");
    snprintf(text, sizeof(text),
             "T  Toggle 3s telemetry (now %s)\r\n",
             track.telemetry_enabled ? "ON" : "OFF");
    pc_console_write(text);
    pc_console_write("X  Emergency stop and abort acquisition\r\n");
    pc_console_write("H  Print this menu\r\n> ");
}

static void pc_console_print_track_manual_menu(void)
{
    track_cal_snapshot_t track;
    char text[320];

    track_calibration_get_snapshot(&track);
    snprintf(text, sizeof(text),
             "\r\n=== MANUAL ACQUIRE state=%s dir=%s cells=%u speed=%u repeat=%u load=%s V=%u.%u ===\r\n",
             track_calibration_state_name(track.state),
             point_test_direction_name(track.manual_direction),
             (unsigned int)track.manual_cells,
             (unsigned int)track.speed,
             (unsigned int)track.manual_repeats,
             track_calibration_load_name(track.load),
             (unsigned int)(track.voltage_x10 / 10U),
             (unsigned int)(track.voltage_x10 % 10U));
    pc_console_write(text);
    pc_console_write("1 FORWARD  2 RIGHT  3 BACK  4 LEFT\r\n");
    pc_console_write("G  Cycle distance 1/2/3/4 cells\r\n");
    pc_console_write("S  Cycle speed 100/120/150 counts/5ms\r\n");
    pc_console_write("N  Cycle repeats 1/3/5/10\r\n");
    pc_console_write("L  Cycle load FREE/BOX/BOMB\r\n");
    pc_console_write("R  Run one segment and record it\r\n");
    pc_console_write("A  Auto-repeat selected segment, stopping every segment\r\n");
    pc_console_write("C  Resume paused step  K  Skip failed step\r\n");
    pc_console_write("P  Print acquisition status and last record\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_track_auto_menu(void)
{
    track_cal_snapshot_t track;
    char text[320];

    track_calibration_get_snapshot(&track);
    snprintf(text, sizeof(text),
             "\r\n=== AUTO ACQUIRE state=%s profile=%s step=%u/%u speed=%u load=%s ===\r\n",
             track_calibration_state_name(track.state),
             track_calibration_profile_name(track.selected_profile),
             (unsigned int)track.route_index,
             (unsigned int)track.route_count,
             (unsigned int)track.speed,
             track_calibration_load_name(track.load));
    pc_console_write(text);
    pc_console_write("1  QUICK12: F/B, R/L, CW and CCW squares\r\n");
    pc_console_write("2  STANDARD60: repeated 1/2-grid-equivalent patterns\r\n");
    pc_console_write("3  FULL180: STANDARD60 at speed 100/120/150\r\n");
    pc_console_write("R  Start selected one-key route (every move is exactly 1 grid)\r\n");
    pc_console_write("P  Pause safely  C  Resume same step  K  Skip failed step\r\n");
    pc_console_write("S  Print acquisition status and last record\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_track_stationary_menu(void)
{
    track_cal_snapshot_t track;
    char text[224];

    track_calibration_get_snapshot(&track);
    snprintf(text, sizeof(text),
             "\r\n=== STATIONARY TEST state=%s elapsed=%lums/%lums frames=%lu ===\r\n",
             track_calibration_state_name(track.state),
             (unsigned long)track.stationary_elapsed_ms,
             (unsigned long)track.stationary_duration_ms,
             (unsigned long)track.stationary_frames);
    pc_console_write(text);
    pc_console_write("1  Start 10-second stability sample\r\n");
    pc_console_write("2  Start 30-second stability sample\r\n");
    pc_console_write("P  Print current/result\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_track_vision_menu(void)
{
    track_cal_snapshot_t track;
    char text[224];

    track_calibration_get_snapshot(&track);
    snprintf(text, sizeof(text),
             "\r\n=== VISION QUALITY state=%s map=%u/%u diff_total=%u diff_max=%u ===\r\n",
             track_calibration_state_name(track.state),
             (unsigned int)track.map_scan_received,
             (unsigned int)track.map_scan_target,
             (unsigned int)track.map_scan_cell_disagreements,
             (unsigned int)track.map_scan_max_frame_disagreements);
    pc_console_write(text);
    pc_console_write("1  Request and compare five FULL_MAP frames\r\n");
    pc_console_write("P  Print current/result\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_track_settings_menu(void)
{
    track_cal_snapshot_t track;
    char text[256];

    track_calibration_get_snapshot(&track);
    snprintf(text, sizeof(text),
             "\r\n=== TRACK SETTINGS speed=%u cells=%u repeat=%u load=%s voltage=%u.%uV ===\r\n",
             (unsigned int)track.speed,
             (unsigned int)track.manual_cells,
             (unsigned int)track.manual_repeats,
             track_calibration_load_name(track.load),
             (unsigned int)(track.voltage_x10 / 10U),
             (unsigned int)(track.voltage_x10 % 10U));
    pc_console_write(text);
    pc_console_write("S  Cycle speed 100/120/150 counts/5ms\r\n");
    pc_console_write("G  Cycle distance 1/2/3/4 cells\r\n");
    pc_console_write("N  Cycle repeats 1/3/5/10\r\n");
    pc_console_write("L  Cycle load FREE/BOX/BOMB\r\n");
    pc_console_write("V  Cycle battery label 12.0/12.2/12.4/12.6V\r\n");
    pc_console_write("P  Constants: wheel=58mm CPR=4096 odom_scale=0.915 center=100mm cell=200mm\r\n");
    pc_console_write("B  Back  X  Emergency stop  H  Menu\r\n> ");
}

static void pc_console_print_menu(void)
{
    if(g_app_mode == 3U)
    {
        if(g_menu_state == PC_MENU_TRACK_MANUAL)
            pc_console_print_track_manual_menu();
        else if(g_menu_state == PC_MENU_TRACK_AUTO)
            pc_console_print_track_auto_menu();
        else if(g_menu_state == PC_MENU_TRACK_STATIONARY)
            pc_console_print_track_stationary_menu();
        else if(g_menu_state == PC_MENU_TRACK_VISION)
            pc_console_print_track_vision_menu();
        else if(g_menu_state == PC_MENU_TRACK_SETTINGS)
            pc_console_print_track_settings_menu();
        else
            pc_console_print_track_main_menu();
    }
    else if(g_app_mode == 2U)
    {
        if(g_menu_state == PC_MENU_POINT_MOVE)
        {
            pc_console_print_point_move_menu();
        }
        else if(g_menu_state == PC_MENU_POINT_ROTATE)
        {
            pc_console_print_point_rotate_menu();
        }
        else if(g_menu_state == PC_MENU_POINT_SENSOR)
        {
            pc_console_print_point_sensor_menu();
        }
        else if(g_menu_state == PC_MENU_POINT_SETTINGS)
        {
            pc_console_print_point_settings_menu();
        }
        else
        {
            pc_console_print_point_main_menu();
        }
    }
    else if(g_menu_state == PC_MENU_RUN)
    {
        pc_console_print_run_menu();
    }
    else if(g_menu_state == PC_MENU_PLANNER)
    {
        pc_console_print_planner_menu();
    }
    else
    {
        pc_console_print_main_menu();
    }
}

static uint8 pc_console_cell_matches(const vision_link_cell_t *cells,
                                     uint8 count, uint8 x, uint8 y)
{
    uint8 i;

    for(i = 0U; i < count; i++)
    {
        if(cells[i].gx == (int8)x && cells[i].gy == (int8)y)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8 pc_console_wall_at(const vision_link_map_t *map, uint8 x, uint8 y)
{
    uint16 index = (uint16)y * VISION_LINK_GRID_W + x;
    return (uint8)((map->wall_bits[index >> 3] >> (index & 7U)) & 1U);
}

static uint16 pc_console_wall_count(const vision_link_map_t *map)
{
    uint8 x;
    uint8 y;
    uint16 count = 0U;

    for(y = 0U; y < map->height; y++)
    {
        for(x = 0U; x < map->width; x++)
        {
            count += pc_console_wall_at(map, x, y);
        }
    }
    return count;
}

static uint8 pc_console_print_cached_map(void)
{
    vision_link_map_t map;
    char line[VISION_LINK_GRID_W + 10U];
    char summary[128];
    uint8 x;
    uint8 y;

    if(!vision_link_get_map(&map) || !map.valid)
    {
        pc_console_write("MAP unavailable: request a FULL_MAP first.\r\n");
        return 0U;
    }
    if(map.width == 0U || map.width > VISION_LINK_GRID_W ||
       map.height == 0U || map.height > VISION_LINK_GRID_H)
    {
        pc_console_write("MAP rejected: invalid dimensions.\r\n");
        return 0U;
    }

    snprintf(summary, sizeof(summary),
             "MAP_BEGIN ver=%lu size=%ux%u wall=%u box=%u goal=%u bomb=%u\r\n",
             (unsigned long)map.map_version,
             (unsigned int)map.width,
             (unsigned int)map.height,
             (unsigned int)pc_console_wall_count(&map),
             (unsigned int)map.box_count,
             (unsigned int)map.goal_count,
             (unsigned int)map.bomb_count);
    pc_console_write(summary);

    for(y = 0U; y < map.height; y++)
    {
        line[0] = 'M';
        line[1] = (char)('0' + (y / 10U));
        line[2] = (char)('0' + (y % 10U));
        line[3] = ' ';
        for(x = 0U; x < map.width; x++)
        {
            char cell = '-';
            if(pc_console_wall_at(&map, x, y)) cell = '#';
            if(pc_console_cell_matches(map.goals, map.goal_count, x, y)) cell = '.';
            if(pc_console_cell_matches(map.boxes, map.box_count, x, y)) cell = '$';
            if(pc_console_cell_matches(map.bombs, map.bomb_count, x, y)) cell = '*';
            line[4U + x] = cell;
        }
        line[4U + map.width] = '\r';
        line[5U + map.width] = '\n';
        line[6U + map.width] = '\0';
        pc_console_write(line);
    }
    pc_console_write("MAP_END\r\n");
    return 1U;
}

static void pc_console_write_keep_vision(const char *text)
{
    pc_console_write(text);
    vision_link_poll();
}

static uint8 pc_console_print_plan_summary(void)
{
    const planner_info_t *info = planner_service_get_info();
    const solver_output_t *plan = planner_service_get_plan();
    const vision_world_snapshot_t *world = planner_service_get_world();
    char text[256];
    uint8 i;

    snprintf(text, sizeof(text),
             "PLAN_SUMMARY id=7100893 valid=%u current=%u status=%s world=%s solver=%s map_ver=%lu gen=%lu solve_ms=%lu\r\n",
             (unsigned int)info->plan_valid,
             (unsigned int)planner_service_plan_is_current(),
             planner_status_name(info->status),
             vision_world_status_name(info->world_status),
             solver_status_name(info->solver_status),
             (unsigned long)info->map_version,
             (unsigned long)info->plan_generation,
             (unsigned long)info->solve_ms);
    pc_console_write_keep_vision(text);

    if(info->world_status == VISION_WORLD_OK)
    {
        snprintf(text, sizeof(text),
                 "PLAN_WORLD car_raw=(%d.%d,%d.%d) car_cell=(%u,%u) adjusted=%u source_wall=%u solver_blocked=%u bomb=%u\r\n",
                 (int)(world->car_x10 / 10),
                 (int)((world->car_x10 < 0 ? -world->car_x10 : world->car_x10) % 10),
                 (int)(world->car_y10 / 10),
                 (int)((world->car_y10 < 0 ? -world->car_y10 : world->car_y10) % 10),
                 (unsigned int)world->solver_map.car_x,
                 (unsigned int)world->solver_map.car_y,
                 (unsigned int)world->car_cell_adjusted,
                 (unsigned int)world->source_wall_count,
                 (unsigned int)world->solver_blocked_count,
                 (unsigned int)world->solver_map.bomb_count);
        pc_console_write_keep_vision(text);
        if(world->solver_map.bomb_count > 0U)
        {
            pc_console_write_keep_vision(
                "PLAN_NOTE bombs are static obstacles in this dry-run stage; bomb actions come later.\r\n");
        }
    }

    if(plan == 0)
    {
        pc_console_write("PLAN unavailable: inspect world/solver status above.\r\n");
        return 0U;
    }

    snprintf(text, sizeof(text),
             "PLAN_CHECK ok=%u code=%s action=%u step=%u cell=(%u,%u) policy=GRID4+BOX_DISAPPEAR\r\n",
             (unsigned int)plan->plan_check_ok,
             solver_plan_check_name(plan->plan_check),
             (unsigned int)plan->plan_check_action,
             (unsigned int)plan->plan_check_step,
             (unsigned int)plan->plan_check_x,
             (unsigned int)plan->plan_check_y);
    pc_console_write_keep_vision(text);

    snprintf(text, sizeof(text),
             "PLAN_COUNTS actions=%u grid_steps=%u free=%u push=%u push_steps=%u waypoints=%u assign_cost=%ld fallback=%u\r\n",
             (unsigned int)plan->action_count,
             (unsigned int)mission_manager_count_plan_steps(plan),
             (unsigned int)plan->total_free_moves,
             (unsigned int)plan->total_pushes,
             (unsigned int)plan->total_push_steps,
             (unsigned int)plan->total_waypoints,
             (long)plan->assignment_cost,
             (unsigned int)plan->used_fallback);
    pc_console_write_keep_vision(text);
    for(i = 0U; i < world->solver_map.box_count; i++)
    {
        snprintf(text, sizeof(text),
                 "ASSIGN box%u=(%u,%u) -> goal%u=(%u,%u)\r\n",
                 (unsigned int)i,
                 (unsigned int)world->solver_map.boxes[i][0],
                 (unsigned int)world->solver_map.boxes[i][1],
                 (unsigned int)plan->assignment[i],
                 (unsigned int)world->solver_map.goals[plan->assignment[i]][0],
                 (unsigned int)world->solver_map.goals[plan->assignment[i]][1]);
        pc_console_write_keep_vision(text);
    }
    return 1U;
}

static void pc_console_print_plan_detail(void)
{
    const planner_info_t *info = planner_service_get_info();
    const solver_output_t *plan = planner_service_get_plan();
    char text[256];
    uint8 i;
    uint16 step = 0U;

    if(plan == 0)
    {
        pc_console_write("ACTION_PLAN unavailable: use S or D first.\r\n");
        return;
    }

    snprintf(text, sizeof(text),
             "ACTION_PLAN_BEGIN map_ver=%lu gen=%lu count=%u current=%u\r\n",
             (unsigned long)info->map_version,
             (unsigned long)info->plan_generation,
             (unsigned int)plan->action_count,
             (unsigned int)planner_service_plan_is_current());
    pc_console_write_keep_vision(text);

    for(i = 0U; i < plan->action_count; i++)
    {
        const action_t *action = &plan->actions[i];
        if(action->type == ACTION_FREE_MOVE)
        {
            uint8 w;
            snprintf(text, sizeof(text),
                     "A%02u FREE target=(%u,%u) wp=%u heading=AUTO\r\n",
                     (unsigned int)i,
                     (unsigned int)action->target_x,
                     (unsigned int)action->target_y,
                     (unsigned int)action->wp_count);
            pc_console_write_keep_vision(text);
            for(w = 0U; w < action->wp_count; w++)
            {
                snprintf(text, sizeof(text),
                         "  W%02u grid=(%d,%d) mm=(%d,%d)\r\n",
                         (unsigned int)w,
                         (int)(action->waypoints[w].x_mm / SOLVER_GRID_SIZE_MM),
                         (int)(action->waypoints[w].y_mm / SOLVER_GRID_SIZE_MM),
                         (int)action->waypoints[w].x_mm,
                         (int)action->waypoints[w].y_mm);
                pc_console_write_keep_vision(text);
            }
        }
        else if(action->type == ACTION_PUSH_BOX)
        {
            const push_meta_t *push = &action->push_meta;
            snprintf(text, sizeof(text),
                     "A%02u PUSH_BOX box=%u from=(%u,%u) to=(%u,%u) dir=%s steps=%u car=(%u,%u)\r\n",
                     (unsigned int)i,
                     (unsigned int)push->box_id,
                     (unsigned int)push->box_start_x,
                     (unsigned int)push->box_start_y,
                     (unsigned int)push->box_target_x,
                     (unsigned int)push->box_target_y,
                     push->push_dir_str,
                     (unsigned int)push->n_steps,
                     (unsigned int)push->car_target_x,
                     (unsigned int)push->car_target_y);
            pc_console_write_keep_vision(text);
        }
        else if(action->type == ACTION_WAIT)
        {
            snprintf(text, sizeof(text), "A%02u WAIT\r\n", (unsigned int)i);
            pc_console_write_keep_vision(text);
        }
        else
        {
            snprintf(text, sizeof(text),
                     "A%02u UNSUPPORTED type=%u\r\n",
                     (unsigned int)i, (unsigned int)action->type);
            pc_console_write_keep_vision(text);
        }
    }
    pc_console_write_keep_vision("ACTION_PLAN_END\r\n");

    snprintf(text, sizeof(text),
             "GRID_EXEC_BEGIN total=%u policy=ONE_CELL+M4_VIS_CHECK push=AUTO_POSE_ONLY\r\n",
             (unsigned int)mission_manager_count_plan_steps(plan));
    pc_console_write_keep_vision(text);
    for(i = 0U; i < plan->action_count; i++)
    {
        const action_t *action = &plan->actions[i];
        if(action->type == ACTION_FREE_MOVE && action->wp_count > 1U)
        {
            uint8 w;
            for(w = 0U; (uint8)(w + 1U) < action->wp_count; w++)
            {
                snprintf(text, sizeof(text),
                         "S%03u FREE A%02u.%02u (%d,%d)->(%d,%d)\r\n",
                         (unsigned int)step++, (unsigned int)i, (unsigned int)w,
                         (int)(action->waypoints[w].x_mm / SOLVER_GRID_SIZE_MM),
                         (int)(action->waypoints[w].y_mm / SOLVER_GRID_SIZE_MM),
                         (int)(action->waypoints[w + 1U].x_mm / SOLVER_GRID_SIZE_MM),
                         (int)(action->waypoints[w + 1U].y_mm / SOLVER_GRID_SIZE_MM));
                pc_console_write_keep_vision(text);
            }
        }
        else if(action->type == ACTION_PUSH_BOX)
        {
            const push_meta_t *push = &action->push_meta;
            uint8 p;
            for(p = 0U; p < push->n_steps; p++)
            {
                int car_from_x = (int)push->car_target_x + (int)DIR_DX[push->push_dir] * p;
                int car_from_y = (int)push->car_target_y + (int)DIR_DY[push->push_dir] * p;
                int car_to_x = (int)push->box_start_x + (int)DIR_DX[push->push_dir] * p;
                int car_to_y = (int)push->box_start_y + (int)DIR_DY[push->push_dir] * p;
                int box_to_x = car_to_x + (int)DIR_DX[push->push_dir];
                int box_to_y = car_to_y + (int)DIR_DY[push->push_dir];
                snprintf(text, sizeof(text),
                         "S%03u PUSH_AUTO A%02u.%02u car(%d,%d)->(%d,%d) box(%d,%d)->(%d,%d)\r\n",
                         (unsigned int)step++, (unsigned int)i, (unsigned int)p,
                         car_from_x, car_from_y, car_to_x, car_to_y,
                         car_to_x, car_to_y, box_to_x, box_to_y);
                pc_console_write_keep_vision(text);
            }
        }
    }
    pc_console_write_keep_vision("GRID_EXEC_END\r\n");
}

static void pc_console_solve(uint8 print_detail)
{
    planner_status_t status;

    if(mission_manager_planner_locked())
    {
        pc_console_write("SOLVER locked: disarm the run menu before replacing the plan.\r\n");
        return;
    }
    pc_console_safe_stop();
    pc_console_write("SOLVER start: motors locked, capturing one immutable map.\r\n");
    status = planner_service_solve();
    pc_console_print_plan_summary();
    if(status == PLANNER_STATUS_OK && print_detail)
    {
        pc_console_print_plan_detail();
    }
}

static void pc_console_print_mission_status(void)
{
    mission_status_t mission;
    char text[448];
    char car_x[16];
    char car_y[16];
    char theta[16];
    char heading_ref[16];
    char physical_heading[16];
    char map_dir[16];
    char body_cmd[16];

    mission_manager_get_status(&mission);
    snprintf(text, sizeof(text),
             "MISSION id=7100893 state=%s result=%s armed=%u auto=%u action=%u/%u sub=%u/%u step=%u/%u type=%u move=(%u,%u)->(%u,%u) gen=%lu plan_current=%u event=%lu telemetry=%s\r\n",
             mission_state_name(mission.state),
             mission_result_name(mission.last_result),
             (unsigned int)mission.armed,
             (unsigned int)mission.auto_run,
             (unsigned int)mission.action_cursor,
             (unsigned int)mission.action_count,
             (unsigned int)mission.substep_cursor,
             (unsigned int)mission.action_step_count,
             (unsigned int)mission.step_cursor,
             (unsigned int)mission.step_count,
             (unsigned int)mission.current_action_type,
             (unsigned int)mission.from_x,
             (unsigned int)mission.from_y,
             (unsigned int)mission.to_x,
             (unsigned int)mission.to_y,
             (unsigned long)mission.plan_generation,
             (unsigned int)mission.map_current,
             (unsigned long)mission.event_counter,
             g_run_telemetry ? "ON" : "OFF");
    pc_console_write(text);

    if(mission.push_step || mission.push_verify_pending ||
       mission.last_result == MISSION_RESULT_PUSH_MAP_TIMEOUT ||
       mission.last_result == MISSION_RESULT_PUSH_MAP_MISMATCH)
    {
        snprintf(text, sizeof(text),
                 "PUSH car=(%u,%u)->(%u,%u) box=(%u,%u)->(%u,%u) goal=%u consumed=%u car_filter=%u verify=%u req=%u bad=%u map=%lu->%lu age=%lums\r\n",
                 (unsigned int)mission.from_x,
                 (unsigned int)mission.from_y,
                 (unsigned int)mission.to_x,
                 (unsigned int)mission.to_y,
                 (unsigned int)mission.box_from_x,
                 (unsigned int)mission.box_from_y,
                 (unsigned int)mission.box_to_x,
                 (unsigned int)mission.box_to_y,
                 (unsigned int)mission.push_box_on_goal,
                 (unsigned int)mission.push_box_consumed,
                 (unsigned int)mission.push_car_box_filtered,
                 (unsigned int)mission.push_verify_pending,
                 (unsigned int)mission.push_verify_requests,
                 (unsigned int)mission.push_verify_bad_maps,
                 (unsigned long)mission.push_map_version_before,
                 (unsigned long)mission.push_map_version_after,
                 (unsigned long)mission.push_verify_age_ms);
        pc_console_write(text);
    }

    if(!mission.follower.pose_valid)
    {
        snprintf(text, sizeof(text),
                 "FOLLOW state=%s fault=%s pose=NOT_CAPTURED action=%u wp=%u/%u phase=%s speed=%d/%d\r\n",
                 action_follower_state_name(mission.follower.state),
                 action_follower_fault_name(mission.follower.fault),
                 (unsigned int)mission.follower.action_index,
                 (unsigned int)mission.follower.waypoint_index,
                 (unsigned int)mission.follower.waypoint_count,
                 action_follower_phase_name(mission.follower.phase),
                 (int)mission.follower.speed_command,
                 (int)mission.follower.nominal_speed_command);
        pc_console_write(text);
        return;
    }

    pc_console_format_x10(car_x, sizeof(car_x), mission.follower.car_x10);
    pc_console_format_x10(car_y, sizeof(car_y), mission.follower.car_y10);
    pc_console_format_x10(theta, sizeof(theta), mission.follower.car_theta_x10);
    pc_console_format_x10(heading_ref, sizeof(heading_ref),
                          mission.follower.visual_heading_ref_x10);
    pc_console_format_x10(physical_heading, sizeof(physical_heading),
                          mission.follower.physical_heading_x10);
    pc_console_format_x10(map_dir, sizeof(map_dir),
                          mission.follower.map_direction_x10);
    pc_console_format_x10(body_cmd, sizeof(body_cmd),
                          mission.follower.body_command_x10);
    snprintf(text, sizeof(text),
             "FOLLOW state=%s fault=%s action=%u wp=%u/%u phase=%s car=(%s,%s) theta=%s ref=%s used=%s age=%lums target_mm=(%d,%d) delta_mm=(%d,%d) dist=%u map_dir=%s body_cmd=%s speed=%d/%d hold=%lums rebase=%lu elapsed=%lums progress_age=%lums\r\n",
             action_follower_state_name(mission.follower.state),
             action_follower_fault_name(mission.follower.fault),
             (unsigned int)mission.follower.action_index,
             (unsigned int)mission.follower.waypoint_index,
             (unsigned int)mission.follower.waypoint_count,
             action_follower_phase_name(mission.follower.phase),
             car_x, car_y, theta, heading_ref, physical_heading,
             (unsigned long)mission.follower.pose_age_ms,
             (int)mission.follower.target_x_mm,
             (int)mission.follower.target_y_mm,
             (int)mission.follower.dx_mm,
             (int)mission.follower.dy_mm,
             (unsigned int)mission.follower.distance_mm,
             map_dir, body_cmd,
             (int)mission.follower.speed_command,
             (int)mission.follower.nominal_speed_command,
             (unsigned long)mission.follower.waypoint_hold_remaining_ms,
             (unsigned long)mission.follower.position_rebase_count,
             (unsigned long)mission.follower.elapsed_ms,
             (unsigned long)mission.follower.progress_age_ms);
    pc_console_write(text);

    snprintf(text, sizeof(text),
             "VIS_CONFIRM phase=%s frames=%u/%u bad=%u final=%u age=%lums pos=%lu dist=%u rel_accept=100mm abs_accept=120mm timeout=6000ms\r\n",
             action_follower_phase_name(mission.follower.phase),
             (unsigned int)mission.follower.visual_confirm_frames,
             (unsigned int)mission.follower.visual_confirm_required,
             (unsigned int)mission.follower.visual_confirm_bad_frames,
             (unsigned int)mission.follower.visual_confirm_final,
             (unsigned long)mission.follower.visual_confirm_age_ms,
             (unsigned long)mission.follower.visual_confirm_pos_packets,
             (unsigned int)mission.follower.distance_mm);
    pc_console_write(text);
}

static void pc_console_print_pid_wheel(uint32 sequence,
                                       const motion_debug_snapshot_t *motion,
                                       uint8 wheel)
{
    static const char *const wheel_name[3] = {"R", "LF", "RF"};
    char text[384];
    uint8 flags = motion->position_limit_flags[wheel];

    snprintf(text, sizeof(text),
             "PID_W seq=%lu w=%s src=%s POS[t=%ld a=%ld e=%ld raw=%ld lim=%ld comp=%ld mode=%s clip=%u slew=%u] SPD[t=%d a=%d e=%ld int=%ld P=%ld I=%ld D=%ld out=%ld pwm=%d]\r\n",
             (unsigned long)sequence,
             wheel_name[wheel],
             pc_console_duty_source_name(motion->duty_source[wheel]),
             (long)pc_console_float_round(motion->position_target[wheel]),
             (long)pc_console_float_round(motion->position_actual[wheel]),
             (long)pc_console_float_round(motion->position_error[wheel]),
             (long)pc_console_float_round(motion->position_raw_output[wheel]),
             (long)pc_console_float_round(motion->position_limit[wheel]),
             (long)pc_console_float_round(motion->position_comp[wheel]),
             pc_console_position_limit_name(flags),
             (unsigned int)((flags & MOTION_POS_LIMIT_CLIPPED) ? 1U : 0U),
             (unsigned int)((flags & MOTION_POS_LIMIT_SLEW) ? 1U : 0U),
             (int)motion->wheel_pid_target[wheel],
             (int)motion->wheel_encoder[wheel],
             (long)pc_console_float_round(motion->speed_error[wheel]),
             (long)pc_console_float_round(motion->speed_error_int[wheel]),
             (long)pc_console_float_round(motion->speed_p_output[wheel]),
             (long)pc_console_float_round(motion->speed_i_output[wheel]),
             (long)pc_console_float_round(motion->speed_d_output[wheel]),
             (long)pc_console_float_round(motion->speed_raw_output[wheel]),
             (int)motion->wheel_pwm[wheel]);
    pc_console_write(text);

    snprintf(text, sizeof(text),
             "PID_WIN seq=%lu w=%s n=%lu eavg=%ld emax=%ld pwm=%d..%d pcap=%lu slew=%lu icap=%lu ocap=%lu hold=%lu\r\n",
             (unsigned long)sequence,
             wheel_name[wheel],
             (unsigned long)motion->window_samples,
             (long)pc_console_float_round(motion->window_speed_error_abs_avg[wheel]),
             (long)pc_console_float_round(motion->window_speed_error_abs_max[wheel]),
             (int)motion->window_pwm_min[wheel],
             (int)motion->window_pwm_max[wheel],
             (unsigned long)motion->window_position_clip_count[wheel],
             (unsigned long)motion->window_position_slew_count[wheel],
             (unsigned long)motion->window_integral_cap_count[wheel],
             (unsigned long)motion->window_output_cap_count[wheel],
             (unsigned long)motion->window_position_hold_count[wheel]);
    pc_console_write(text);
}

static void pc_console_print_pid_yaw(uint32 sequence,
                                     const motion_debug_snapshot_t *motion)
{
    char text[224];
    char error[16];
    char gyro[16];
    char p_output[16];
    char d_output[16];
    char raw_output[16];
    char final_output[16];

    pc_console_format_x10(error, sizeof(error),
                          pc_console_float_x10(motion->yaw_error_deg));
    pc_console_format_x10(gyro, sizeof(gyro),
                          pc_console_float_x10(motion->gyro_z_dps));
    pc_console_format_x10(p_output, sizeof(p_output),
                          pc_console_float_x10(motion->yaw_p_output));
    pc_console_format_x10(d_output, sizeof(d_output),
                          pc_console_float_x10(motion->yaw_d_output));
    pc_console_format_x10(raw_output, sizeof(raw_output),
                          pc_console_float_x10(motion->yaw_raw_output));
    pc_console_format_x10(final_output, sizeof(final_output),
                          pc_console_float_x10(motion->yaw_correction));

    snprintf(text, sizeof(text),
             "PID_YAW seq=%lu err=%s gyro=%s P=%s D=%s raw=%s out=%s sat=%u\r\n",
             (unsigned long)sequence,
             error, gyro, p_output, d_output, raw_output, final_output,
             (unsigned int)motion->yaw_saturated);
    pc_console_write(text);
}

static void pc_console_print_run_telemetry(const mission_status_t *mission)
{
    motion_debug_snapshot_t motion;
    char text[448];
    char car_x[16];
    char car_y[16];
    char visual_theta[16];
    char heading_ref[16];
    char physical_heading[16];
    char map_dir[16];
    char body_cmd[16];
    char applied_angle[16];
    char applied_speed[16];
    char yaw[16];
    char yaw_target[16];
    char yaw_error[16];
    char yaw_correction[16];
    char gyro_z[16];
    char visual_sync_error[16];
    char visual_sync_step[16];

    motion_get_debug_snapshot(&motion);
    pc_console_format_x10(car_x, sizeof(car_x), mission->follower.car_x10);
    pc_console_format_x10(car_y, sizeof(car_y), mission->follower.car_y10);
    pc_console_format_x10(visual_theta, sizeof(visual_theta),
                          mission->follower.car_theta_x10);
    pc_console_format_x10(heading_ref, sizeof(heading_ref),
                          mission->follower.visual_heading_ref_x10);
    pc_console_format_x10(physical_heading, sizeof(physical_heading),
                          mission->follower.physical_heading_x10);
    pc_console_format_x10(map_dir, sizeof(map_dir),
                          mission->follower.map_direction_x10);
    pc_console_format_x10(body_cmd, sizeof(body_cmd),
                          mission->follower.body_command_x10);
    pc_console_format_x10(applied_angle, sizeof(applied_angle),
                          pc_console_float_x10(motion.target_angle_deg));
    pc_console_format_x10(applied_speed, sizeof(applied_speed),
                          pc_console_float_x10(motion.target_speed));
    pc_console_format_x10(yaw, sizeof(yaw),
                          pc_console_float_x10(motion.yaw_deg));
    pc_console_format_x10(yaw_target, sizeof(yaw_target),
                          pc_console_float_x10(motion.yaw_target_deg));
    pc_console_format_x10(yaw_error, sizeof(yaw_error),
                          pc_console_float_x10(motion.yaw_error_deg));
    pc_console_format_x10(yaw_correction, sizeof(yaw_correction),
                          pc_console_float_x10(motion.yaw_correction));
    pc_console_format_x10(gyro_z, sizeof(gyro_z),
                          pc_console_float_x10(motion.gyro_z_dps));
    pc_console_format_x10(visual_sync_error, sizeof(visual_sync_error),
                          pc_console_float_x10(motion.visual_sync_error_deg));
    pc_console_format_x10(visual_sync_step, sizeof(visual_sync_step),
                          pc_console_float_x10(motion.visual_sync_step_deg));

    g_run_telem_sequence++;
    snprintf(text, sizeof(text),
             "RUN_TELEM id=7100893 seq=%lu ms=%lu state=%s motor=%s plan=%u action=%u wp=%u/%u phase=%s speed=%d/%d hold=%lums rebase=%lu pose=%s car=(%s,%s) age=%lums target=(%d,%d) delta=(%d,%d) dist=%u vtheta=%s ref=%s used=%s elapsed=%lums progress=%lums\r\n",
             (unsigned long)g_run_telem_sequence,
             (unsigned long)pit_count * 5UL,
             mission_state_name(mission->state),
             device_init_flag ? "LOCKED" : "ENABLED",
             (unsigned int)planner_service_plan_is_current(),
             (unsigned int)mission->follower.action_index,
             (unsigned int)mission->follower.waypoint_index,
             (unsigned int)mission->follower.waypoint_count,
             action_follower_phase_name(mission->follower.phase),
             (int)mission->follower.speed_command,
             (int)mission->follower.nominal_speed_command,
             (unsigned long)mission->follower.waypoint_hold_remaining_ms,
             (unsigned long)mission->follower.position_rebase_count,
             mission->follower.pose_valid ? "LIVE" : "NONE",
             car_x, car_y,
             (unsigned long)mission->follower.pose_age_ms,
             (int)mission->follower.target_x_mm,
             (int)mission->follower.target_y_mm,
             (int)mission->follower.dx_mm,
             (int)mission->follower.dy_mm,
             (unsigned int)mission->follower.distance_mm,
             visual_theta, heading_ref, physical_heading,
             (unsigned long)mission->follower.elapsed_ms,
             (unsigned long)mission->follower.progress_age_ms);
    pc_console_write(text);

    snprintf(text, sizeof(text),
             "RUN_VIS seq=%lu frames=%u/%u bad=%u final=%u confirm_age=%lums pos=%lu dist=%umm rel_accept=100mm abs_accept=120mm\r\n",
             (unsigned long)g_run_telem_sequence,
             (unsigned int)mission->follower.visual_confirm_frames,
             (unsigned int)mission->follower.visual_confirm_required,
             (unsigned int)mission->follower.visual_confirm_bad_frames,
             (unsigned int)mission->follower.visual_confirm_final,
             (unsigned long)mission->follower.visual_confirm_age_ms,
             (unsigned long)mission->follower.visual_confirm_pos_packets,
             (unsigned int)mission->follower.distance_mm);
    pc_console_write(text);

    snprintf(text, sizeof(text),
             "RUN_CTRL seq=%lu map_dir=%s body_cmd=%s applied=(%s,%s) yaw=(now=%s target=%s err=%s gyro=%s corr=%s vsync=%s/%s) wheel_t[R,LF,RF]=(%d,%d,%d) pid_t=(%d,%d,%d) enc=(%d,%d,%d) enc_rot=%d pwm=(%d,%d,%d)\r\n",
             (unsigned long)g_run_telem_sequence,
             map_dir, body_cmd, applied_angle, applied_speed,
             yaw, yaw_target, yaw_error, gyro_z, yaw_correction,
             visual_sync_error, visual_sync_step,
             (int)motion.wheel_target[0],
             (int)motion.wheel_target[1],
             (int)motion.wheel_target[2],
             (int)motion.wheel_pid_target[0],
             (int)motion.wheel_pid_target[1],
             (int)motion.wheel_pid_target[2],
             (int)motion.wheel_encoder[0],
             (int)motion.wheel_encoder[1],
             (int)motion.wheel_encoder[2],
             (int)motion.encoder_rotate,
             (int)motion.wheel_pwm[0],
             (int)motion.wheel_pwm[1],
             (int)motion.wheel_pwm[2]);
    pc_console_write(text);

    pc_console_print_pid_wheel(g_run_telem_sequence, &motion, 0U);
    pc_console_print_pid_wheel(g_run_telem_sequence, &motion, 1U);
    pc_console_print_pid_wheel(g_run_telem_sequence, &motion, 2U);
    pc_console_print_pid_yaw(g_run_telem_sequence, &motion);
}

static void pc_console_print_point_status(uint8 detailed)
{
    point_test_snapshot_t point;
    motion_debug_snapshot_t motion;
    char text[448];
    char imu_relative[16];
    char encoder_yaw[16];
    char active_progress[16];
    char remaining[16];

    point_test_get_snapshot(&point);
    pc_console_format_x10(imu_relative, sizeof(imu_relative),
                          pc_console_float_x10(point.imu_relative_deg));
    pc_console_format_x10(encoder_yaw, sizeof(encoder_yaw),
                          pc_console_float_x10(point.encoder_yaw_deg));
    pc_console_format_x10(active_progress, sizeof(active_progress),
                          pc_console_float_x10(point.active_progress));
    pc_console_format_x10(remaining, sizeof(remaining),
                          pc_console_float_x10(point.remaining));

    snprintf(text, sizeof(text),
             "POINT_STATUS id=7100893 state=%s kind=%s fault=%s motor=%s origin=%u vis0=%u vis=%u age=%lums dir=%s/%d cells=%u speed=%u/%u sensor=%s rot=%udeg/%s/%s progress=%s remain=%s elapsed=%lums settle=%lums event=%lu\r\n",
             point_test_state_name(point.state),
             point_test_kind_name(point.kind),
             point_test_fault_name(point.fault),
             device_init_flag ? "LOCKED" : "ENABLED",
             (unsigned int)point.origin_valid,
             (unsigned int)point.vision_origin_valid,
             (unsigned int)point.vision_live,
             (unsigned long)point.vision_age_ms,
             point_test_direction_name(point.direction_index),
             (int)point.direction_deg,
             (unsigned int)point.cell_count,
             (unsigned int)point.speed,
             (unsigned int)point.commanded_speed,
             point_test_sensor_name(point.sensor_mode),
             (unsigned int)point.target_rotation_deg,
             point.rotate_clockwise ? "CW" : "CCW",
             point_test_rotation_stop_name(point.rotate_stop),
             active_progress, remaining,
             (unsigned long)point.elapsed_ms,
             (unsigned long)point.settle_remaining_ms,
             (unsigned long)point.event_counter);
    pc_console_write(text);

    snprintf(text, sizeof(text),
             "POINT_VIS_INPUT ready=%u frames=%u/6 stable=%lums frame=%u gate=pos20mm+theta12deg+200ms\r\n",
             (unsigned int)point.vision_input_stable,
             (unsigned int)point.vision_input_stable_frames,
             (unsigned long)point.vision_input_stable_ms,
             (unsigned int)point.vision_input_frame_id);
    pc_console_write(text);

    snprintf(text, sizeof(text),
             "POINT_VIS_PIPE phase=%s frames=%u/%u bad_clusters=%u frame=%u age=%lums pos=%lu drain=%lums vis_err=(%ld,%ld)mm vis-enc=(%ld,%ld)mm policy=%s\r\n",
             point_test_state_name(point.state),
             (unsigned int)point.vision_confirm_frames,
             (unsigned int)point.vision_confirm_required,
             (unsigned int)point.vision_confirm_bad_frames,
             (unsigned int)point.vision_confirm_frame_id,
             (unsigned long)point.vision_confirm_age_ms,
             (unsigned long)point.vision_confirm_pos_packets,
             (unsigned long)point.vision_drain_remaining_ms,
             (long)pc_console_float_round(point.vision_target_error_along_mm),
             (long)pc_console_float_round(point.vision_target_error_cross_mm),
             (long)pc_console_float_round(point.vision_encoder_delta_along_mm),
             (long)pc_console_float_round(point.vision_encoder_delta_cross_mm),
             point.sensor_mode == POINT_SENSOR_FUSION_LOCKED ?
                 "hold500+drain1000+stable6/200ms,tol60mm,total6s" :
                 "legacy-M1/M2/M3");
    pc_console_write(text);

    snprintf(text, sizeof(text),
             "POINT_POSE enc_mm[F,R,A,C]=(%ld,%ld,%ld,%ld) vis_mm[F,R,A,C]=(%ld,%ld,%ld,%ld) yaw[imu,enc]=(%s,%s) max[yaw,cross]=(%ld,%ld) wheel_count[R,LF,RF]=(%ld,%ld,%ld) rot_count=%ld vision=(%d.%d,%d.%d,%d.%d)\r\n",
             (long)pc_console_float_round(point.encoder_forward_mm),
             (long)pc_console_float_round(point.encoder_right_mm),
             (long)pc_console_float_round(point.encoder_along_mm),
             (long)pc_console_float_round(point.encoder_cross_mm),
             (long)pc_console_float_round(point.vision_forward_mm),
             (long)pc_console_float_round(point.vision_right_mm),
             (long)pc_console_float_round(point.vision_along_mm),
             (long)pc_console_float_round(point.vision_cross_mm),
             imu_relative, encoder_yaw,
             (long)pc_console_float_round(point.max_abs_yaw_deg),
             (long)pc_console_float_round(point.max_abs_cross_mm),
             (long)point.wheel_count[0],
             (long)point.wheel_count[1],
             (long)point.wheel_count[2],
             (long)point.encoder_rotate_count,
             (int)(point.vision_x10 / 10),
             (int)((point.vision_x10 < 0 ? -point.vision_x10 : point.vision_x10) % 10),
             (int)(point.vision_y10 / 10),
             (int)((point.vision_y10 < 0 ? -point.vision_y10 : point.vision_y10) % 10),
             (int)(point.vision_theta_x10 / 10),
             (int)((point.vision_theta_x10 < 0 ? -point.vision_theta_x10 : point.vision_theta_x10) % 10));
    pc_console_write(text);

    if(!detailed) return;
    motion_get_debug_snapshot(&motion);
    g_point_telem_sequence++;
    snprintf(text, sizeof(text),
             "POINT_CTRL seq=%lu target=(angle=%ld speed=%ld) wheel_t=(%d,%d,%d) pid_t=(%d,%d,%d) enc=(%d,%d,%d) pwm=(%d,%d,%d) txq=%u txdrop=%lu\r\n",
             (unsigned long)g_point_telem_sequence,
             (long)pc_console_float_round(motion.target_angle_deg),
             (long)pc_console_float_round(motion.target_speed),
             (int)motion.wheel_target[0],
             (int)motion.wheel_target[1],
             (int)motion.wheel_target[2],
             (int)motion.wheel_pid_target[0],
             (int)motion.wheel_pid_target[1],
             (int)motion.wheel_pid_target[2],
             (int)motion.wheel_encoder[0],
             (int)motion.wheel_encoder[1],
             (int)motion.wheel_encoder[2],
             (int)motion.wheel_pwm[0],
             (int)motion.wheel_pwm[1],
             (int)motion.wheel_pwm[2],
             (unsigned int)pc_console_tx_queue_used(),
             (unsigned long)g_uart8_tx_drops);
    pc_console_write(text);
    pc_console_print_pid_wheel(g_point_telem_sequence, &motion, 0U);
    pc_console_print_pid_wheel(g_point_telem_sequence, &motion, 1U);
    pc_console_print_pid_wheel(g_point_telem_sequence, &motion, 2U);
    pc_console_print_pid_yaw(g_point_telem_sequence, &motion);
}

static void pc_console_print_track_record(const track_cal_record_t *record)
{
    char text[384];

    if(record->sequence == 0U)
    {
        pc_console_write("CAL_REC none: no completed acquisition segment yet.\r\n");
        return;
    }
    snprintf(text, sizeof(text),
             "CAL_REC id=7100893 schema=1 seq=%lu session=%lu profile=%s step=%u/%u valid=%u result=%s point=%s\r\n",
             (unsigned long)record->sequence,
             (unsigned long)record->session,
             track_calibration_profile_name(record->profile),
             (unsigned int)record->step_index,
             (unsigned int)record->step_count,
             (unsigned int)record->valid,
             track_calibration_result_name(record->result),
             point_test_fault_name(record->point_fault));
    pc_console_write(text);
    snprintf(text, sizeof(text),
             "CAL_CFG dir=%s/%u cells=%u speed=%u load=%s voltage=%u.%uV elapsed=%lums samples=%lu\r\n",
             point_test_direction_name(record->direction),
             (unsigned int)record->direction,
             (unsigned int)record->cells,
             (unsigned int)record->speed,
             track_calibration_load_name(record->load),
             (unsigned int)(record->voltage_x10 / 10U),
             (unsigned int)(record->voltage_x10 % 10U),
             (unsigned long)record->elapsed_ms,
             (unsigned long)record->drive_samples);
    pc_console_write(text);
    snprintf(text, sizeof(text),
             "CAL_VISION start=(%d.%d,%d.%d,%d.%d) end=(%d.%d,%d.%d,%d.%d) frame=%u->%u pos=%lu->%lu age=%lums stable=%u bad=%u\r\n",
             (int)(record->start_x10 / 10),
             (int)abs(record->start_x10 % 10),
             (int)(record->start_y10 / 10),
             (int)abs(record->start_y10 % 10),
             (int)(record->start_theta_x10 / 10),
             (int)abs(record->start_theta_x10 % 10),
             (int)(record->end_x10 / 10),
             (int)abs(record->end_x10 % 10),
             (int)(record->end_y10 / 10),
             (int)abs(record->end_y10 % 10),
             (int)(record->end_theta_x10 / 10),
             (int)abs(record->end_theta_x10 % 10),
             (unsigned int)record->start_frame,
             (unsigned int)record->end_frame,
             (unsigned long)record->start_pos_packets,
             (unsigned long)record->end_pos_packets,
             (unsigned long)record->vision_age_ms,
             (unsigned int)record->stable_frames,
             (unsigned int)record->bad_clusters);
    pc_console_write(text);
    snprintf(text, sizeof(text),
             "CAL_DELTA enc_mm=(%ld.%ld,%ld.%ld) vis_mm=(%ld.%ld,%ld.%ld) vis_target_mm=(%ld.%ld,%ld.%ld) vis_enc_mm=(%ld.%ld,%ld.%ld)\r\n",
             (long)(record->encoder_along_x10 / 10),
             (long)labs(record->encoder_along_x10 % 10),
             (long)(record->encoder_cross_x10 / 10),
             (long)labs(record->encoder_cross_x10 % 10),
             (long)(record->vision_along_x10 / 10),
             (long)labs(record->vision_along_x10 % 10),
             (long)(record->vision_cross_x10 / 10),
             (long)labs(record->vision_cross_x10 % 10),
             (long)(record->vision_target_error_along_x10 / 10),
             (long)labs(record->vision_target_error_along_x10 % 10),
             (long)(record->vision_target_error_cross_x10 / 10),
             (long)labs(record->vision_target_error_cross_x10 % 10),
             (long)(record->vision_encoder_delta_along_x10 / 10),
             (long)labs(record->vision_encoder_delta_along_x10 % 10),
             (long)(record->vision_encoder_delta_cross_x10 / 10),
             (long)labs(record->vision_encoder_delta_cross_x10 % 10));
    pc_console_write(text);
    snprintf(text, sizeof(text),
             "CAL_ODOM wheel[R,LF,RF]=(%ld,%ld,%ld) yaw10[imu,enc,max]=(%ld,%ld,%ld) cross_max_x10=%ld\r\n",
             (long)record->wheel_count[0],
             (long)record->wheel_count[1],
             (long)record->wheel_count[2],
             (long)record->imu_yaw_x10,
             (long)record->encoder_yaw_x10,
             (long)record->max_yaw_x10,
             (long)record->max_cross_x10);
    pc_console_write(text);
    snprintf(text, sizeof(text),
             "CAL_DRIVE speed_abs_avg[R,LF,RF]=(%ld,%ld,%ld) duty_abs_avg=(%ld,%ld,%ld) duty_peak=(%d,%d,%d)\r\n",
             (long)record->speed_abs_avg[0],
             (long)record->speed_abs_avg[1],
             (long)record->speed_abs_avg[2],
             (long)record->duty_abs_avg[0],
             (long)record->duty_abs_avg[1],
             (long)record->duty_abs_avg[2],
             (int)record->duty_abs_peak[0],
             (int)record->duty_abs_peak[1],
             (int)record->duty_abs_peak[2]);
    pc_console_write(text);
}

static void pc_console_print_track_status(uint8 include_record)
{
    track_cal_snapshot_t track;
    char text[384];

    track_calibration_get_snapshot(&track);
    snprintf(text, sizeof(text),
             "CAL_STATUS id=7100893 state=%s motor=%s session=%lu profile=%s selected=%s step=%u/%u valid=%u invalid=%u paused=%u result=%s point=%s txq=%u txdrop=%lu\r\n",
             track_calibration_state_name(track.state),
             device_init_flag ? "LOCKED" : "ENABLED",
             (unsigned long)track.session,
             track_calibration_profile_name(track.active_profile),
             track_calibration_profile_name(track.selected_profile),
             (unsigned int)track.route_index,
             (unsigned int)track.route_count,
             (unsigned int)track.valid_records,
             (unsigned int)track.invalid_records,
             (unsigned int)track.paused,
             track_calibration_result_name(track.last_result),
             point_test_fault_name(track.point_fault),
             (unsigned int)pc_console_tx_queue_used(),
             (unsigned long)g_uart8_tx_drops);
    pc_console_write(text);
    snprintf(text, sizeof(text),
             "CAL_POINT state=%s origin=%u vis=%u stable=%u/%u age=%lums dir=%s cells=%u speed=%u/%u progress=%ld remain=%ld wait=%lums event=%lu\r\n",
             point_test_state_name(track.point.state),
             (unsigned int)track.point.origin_valid,
             (unsigned int)track.point.vision_live,
             (unsigned int)track.point.vision_input_stable,
             (unsigned int)track.point.vision_input_stable_frames,
             (unsigned long)track.point.vision_age_ms,
             point_test_direction_name(track.point.direction_index),
             (unsigned int)track.point.cell_count,
             (unsigned int)track.point.speed,
             (unsigned int)track.point.commanded_speed,
             (long)pc_console_float_round(track.point.active_progress),
             (long)pc_console_float_round(track.point.remaining),
             (unsigned long)track.wait_remaining_ms,
             (unsigned long)track.event_counter);
    pc_console_write(text);
    if(track.stationary_duration_ms != 0U)
    {
        snprintf(text, sizeof(text),
                 "CAL_STILL elapsed=%lums/%lums frames=%lu pos_start=(%d.%d,%d.%d) span_x10=(%d..%d,%d..%d) yaw_span_x10=(%ld..%ld)\r\n",
                 (unsigned long)track.stationary_elapsed_ms,
                 (unsigned long)track.stationary_duration_ms,
                 (unsigned long)track.stationary_frames,
                 (int)(track.stationary_start_x10 / 10),
                 (int)abs(track.stationary_start_x10 % 10),
                 (int)(track.stationary_start_y10 / 10),
                 (int)abs(track.stationary_start_y10 % 10),
                 (int)track.stationary_min_x10,
                 (int)track.stationary_max_x10,
                 (int)track.stationary_min_y10,
                 (int)track.stationary_max_y10,
                 (long)track.stationary_yaw_min_x10,
                 (long)track.stationary_yaw_max_x10);
        pc_console_write(text);
    }
    if(track.map_scan_received != 0U || track.state == TRACK_CAL_MAP_SCAN)
    {
        snprintf(text, sizeof(text),
                 "CAL_MAP frames=%u/%u ref_ver=%lu last_ver=%lu cell_diff_total=%u cell_diff_max=%u result=%s\r\n",
                 (unsigned int)track.map_scan_received,
                 (unsigned int)track.map_scan_target,
                 (unsigned long)track.map_scan_reference_version,
                 (unsigned long)track.map_scan_last_version,
                 (unsigned int)track.map_scan_cell_disagreements,
                 (unsigned int)track.map_scan_max_frame_disagreements,
                 track_calibration_result_name(track.last_result));
        pc_console_write(text);
    }
    if(include_record) pc_console_print_track_record(&track.last_record);
}

static void pc_console_print_run_result(const char *command,
                                        mission_result_t result)
{
    char text[112];

    snprintf(text, sizeof(text),
             "RUN_CMD %s result=%s\r\n",
             command, mission_result_name(result));
    pc_console_write(text);
    if(result == MISSION_RESULT_UNSUPPORTED_ACTION)
    {
        if(strcmp(command, "AUTO_ALL") == 0)
            pc_console_write("RUN_LOCKED: an unsupported action stopped AUTO_ALL.\r\n");
        else
            pc_console_write("RUN_LOCKED: this action type is not executable.\r\n");
    }
    else if(result == MISSION_RESULT_START_POSE_CHANGED ||
            result == MISSION_RESULT_POSE_INVALID ||
            result == MISSION_RESULT_REPLAN_REQUIRED)
    {
        pc_console_write(
            "RUN_REPLAN: disarm, solve again at the current car position, then arm.\r\n");
    }
    pc_console_print_mission_status();
}

static void pc_console_print_status(void)
{
    vision_link_snapshot_t s;
    mission_status_t mission;
    uint32 age_ms;
    char text[320];

    if(g_app_mode == 3U)
    {
        pc_console_print_track_status(0U);
        return;
    }
    if(g_app_mode == 2U)
    {
        pc_console_print_point_status(0U);
        return;
    }

    vision_link_get_snapshot(&s);
    mission_manager_get_status(&mission);
    age_ms = (s.last_packet_tick == 0U) ? 99999UL :
             (unsigned long)(pit_count - s.last_packet_tick) * 5UL;

    snprintf(text, sizeof(text),
             "STATUS id=7100893 motor=%s mission=%s online=%u pose_valid=%u age=%lu rx=%lu pos=%lu map=%lu ver=%lu req=%lu err=%lu/%lu drop=%lu txq=%u txpeak=%u txdrop=%lu car=(%d.%d,%d.%d) theta=%d.%d\r\n",
             device_init_flag ? "LOCKED" : "ENABLED",
             mission_state_name(mission.state),
             (unsigned int)vision_link_is_online(),
             (unsigned int)s.pose_valid,
             (unsigned long)age_ms,
             (unsigned long)s.rx_bytes,
             (unsigned long)s.pos_packets,
             (unsigned long)s.map_packets,
             (unsigned long)s.map_version,
             (unsigned long)s.map_request_count,
             (unsigned long)s.checksum_errors,
             (unsigned long)s.format_errors,
             (unsigned long)s.ring_drops,
             (unsigned int)pc_console_tx_queue_used(),
             (unsigned int)g_uart8_tx_peak,
             (unsigned long)g_uart8_tx_drops,
             (int)(s.car_x_mm / 10),
             (int)((s.car_x_mm < 0 ? -s.car_x_mm : s.car_x_mm) % 10),
             (int)(s.car_y_mm / 10),
             (int)((s.car_y_mm < 0 ? -s.car_y_mm : s.car_y_mm) % 10),
             (int)(s.car_theta_x10 / 10),
             (int)((s.car_theta_x10 < 0 ? -s.car_theta_x10 : s.car_theta_x10) % 10));
    pc_console_write(text);
}

static uint8 pc_console_point_cycle_cells(void)
{
    point_test_snapshot_t point;

    point_test_get_snapshot(&point);
    return point_test_set_cells(
        point.cell_count >= 4U ? 1U : (uint8)(point.cell_count + 1U));
}

static uint8 pc_console_point_cycle_sensor(void)
{
    point_test_snapshot_t point;
    point_sensor_mode_t next;

    point_test_get_snapshot(&point);
    next = (point.sensor_mode >= POINT_SENSOR_FUSION_LOCKED) ?
        POINT_SENSOR_ENCODER_IMU : (point_sensor_mode_t)(point.sensor_mode + 1);
    return point_test_set_sensor_mode(next);
}

static void pc_console_handle_point_command(uint8 command)
{
    point_test_snapshot_t point;
    uint8 ok = 1U;

    point_test_get_snapshot(&point);
    if(command == 'X')
    {
        point_test_emergency_stop();
        pc_console_write("POINT EMERGENCY STOP: motors locked; origin invalidated.\r\n");
        pc_console_print_point_status(1U);
        point_test_get_snapshot(&point);
        g_last_point_event = point.event_counter;
        pc_console_print_menu();
        return;
    }
    if(command == 'T')
    {
        g_point_telemetry = (uint8)!g_point_telemetry;
        g_last_point_report_tick = pit_count;
        pc_console_write(g_point_telemetry ?
            "POINT telemetry ON: 3s frames enabled; sensor menu uses compact frames.\r\n" :
            "POINT telemetry OFF.\r\n");
        pc_console_print_menu();
        return;
    }
    if(command == 'Z')
    {
        ok = point_test_capture_origin();
        pc_console_write(ok ?
            "POINT ORIGIN captured from stable visual input; encoder/IMU zeroed.\r\n" :
            "POINT ORIGIN rejected: M4 requires 6 stable visual frames for at least 200ms; keep still and retry Z.\r\n");
        pc_console_print_point_status(0U);
        point_test_get_snapshot(&point);
        g_last_point_event = point.event_counter;
        pc_console_print_menu();
        return;
    }

    if(g_menu_state == PC_MENU_POINT_MOVE)
    {
        if(command >= '1' && command <= '8')
        {
            ok = point_test_set_direction((uint8)(command - '1'));
        }
        else if(command == 'G')
        {
            ok = pc_console_point_cycle_cells();
        }
        else if(command == 'M')
        {
            ok = pc_console_point_cycle_sensor();
        }
        else if(command == 'V')
        {
            g_menu_state = PC_MENU_POINT_SETTINGS;
        }
        else if(command == 'R')
        {
            ok = point_test_start_translation();
            pc_console_write(ok ?
                "POINT RUN translation started. M4 moves on encoder+IMU, then waits for delayed vision while stopped.\r\n" :
                "POINT RUN rejected: capture Z origin and check selected sensor.\r\n");
            pc_console_print_point_status(1U);
        }
        else if(command == 'P')
        {
            pc_console_print_point_status(1U);
        }
        else if(command == 'B')
        {
            g_menu_state = PC_MENU_POINT_MAIN;
        }
        else if(command != 'H' && command != '?')
        {
            pc_console_write("Unknown translation command.\r\n");
        }
    }
    else if(g_menu_state == PC_MENU_POINT_ROTATE)
    {
        if(command == '1') ok = point_test_set_rotation_angle(90U);
        else if(command == '2') ok = point_test_set_rotation_angle(180U);
        else if(command == '3') ok = point_test_set_rotation_angle(360U);
        else if(command == 'D')
        {
            ok = point_test_set_rotation_clockwise((uint8)!point.rotate_clockwise);
        }
        else if(command == 'M')
        {
            ok = point_test_set_rotation_stop(
                point.rotate_stop == POINT_ROTATE_STOP_IMU ?
                POINT_ROTATE_STOP_ENCODER : POINT_ROTATE_STOP_IMU);
        }
        else if(command == 'V')
        {
            g_menu_state = PC_MENU_POINT_SETTINGS;
        }
        else if(command == 'R')
        {
            ok = point_test_start_rotation();
            pc_console_write(ok ?
                "POINT RUN rotation started. Keep clear; X stops immediately.\r\n" :
                "POINT RUN rejected: capture Z origin and check stop source.\r\n");
            pc_console_print_point_status(1U);
        }
        else if(command == 'P')
        {
            pc_console_print_point_status(1U);
        }
        else if(command == 'B')
        {
            g_menu_state = PC_MENU_POINT_MAIN;
        }
        else if(command != 'H' && command != '?')
        {
            pc_console_write("Unknown rotation command.\r\n");
        }
    }
    else if(g_menu_state == PC_MENU_POINT_SENSOR)
    {
        if(command == 'P')
        {
            pc_console_print_point_status(1U);
        }
        else if(command == 'B')
        {
            g_menu_state = PC_MENU_POINT_MAIN;
        }
        else if(command != 'H' && command != '?')
        {
            pc_console_write("Unknown sensor-monitor command.\r\n");
        }
    }
    else if(g_menu_state == PC_MENU_POINT_SETTINGS)
    {
        if(command == '1') ok = point_test_set_speed(100U);
        else if(command == '2') ok = point_test_set_speed(120U);
        else if(command == '3') ok = point_test_set_speed(150U);
        else if(command == 'G') ok = pc_console_point_cycle_cells();
        else if(command == 'M') ok = pc_console_point_cycle_sensor();
        else if(command == 'P') pc_console_print_point_status(1U);
        else if(command == 'B') g_menu_state = PC_MENU_POINT_MAIN;
        else if(command != 'H' && command != '?')
        {
            pc_console_write("Unknown settings command.\r\n");
        }
    }
    else
    {
        if(command == '1') g_menu_state = PC_MENU_POINT_MOVE;
        else if(command == '2') g_menu_state = PC_MENU_POINT_ROTATE;
        else if(command == '3') g_menu_state = PC_MENU_POINT_SENSOR;
        else if(command == '4') g_menu_state = PC_MENU_POINT_SETTINGS;
        else if(command == '5') pc_console_print_point_status(1U);
        else if(command != 'H' && command != '?')
        {
            pc_console_write("Unknown point-test command.\r\n");
        }
    }

    if(!ok)
    {
        pc_console_write("POINT setting rejected while busy or value invalid.\r\n");
    }
    point_test_get_snapshot(&point);
    g_last_point_event = point.event_counter;
    pc_console_print_menu();
}

static uint8 pc_console_track_cycle_speed(void)
{
    track_cal_snapshot_t track;
    track_calibration_get_snapshot(&track);
    return track_calibration_set_speed(track.speed == 100U ? 120U :
        (track.speed == 120U ? 150U : 100U));
}

static uint8 pc_console_track_cycle_cells(void)
{
    track_cal_snapshot_t track;
    track_calibration_get_snapshot(&track);
    return track_calibration_set_cells(track.manual_cells >= 4U ? 1U :
        (uint8)(track.manual_cells + 1U));
}

static uint8 pc_console_track_cycle_repeats(void)
{
    track_cal_snapshot_t track;
    uint8 next;
    track_calibration_get_snapshot(&track);
    next = track.manual_repeats == 1U ? 3U :
        (track.manual_repeats == 3U ? 5U :
        (track.manual_repeats == 5U ? 10U : 1U));
    return track_calibration_set_repeats(next);
}

static uint8 pc_console_track_cycle_load(void)
{
    track_cal_snapshot_t track;
    track_calibration_get_snapshot(&track);
    return track_calibration_set_load(track.load >= TRACK_CAL_LOAD_BOMB ?
        TRACK_CAL_LOAD_FREE : (track_cal_load_t)(track.load + 1));
}

static uint8 pc_console_track_cycle_voltage(void)
{
    track_cal_snapshot_t track;
    uint16 next;
    track_calibration_get_snapshot(&track);
    next = track.voltage_x10 < 120U || track.voltage_x10 >= 126U ?
        120U : (uint16)(track.voltage_x10 + 2U);
    return track_calibration_set_voltage_x10(next);
}

static void pc_console_handle_track_command(uint8 command)
{
    track_cal_snapshot_t track;
    uint8 ok = 1U;
    uint8 handled = 1U;

    track_calibration_get_snapshot(&track);
    if(command == 'X')
    {
        track_calibration_emergency_stop();
        pc_console_write("CAL EMERGENCY STOP: motors locked and acquisition aborted.\r\n");
        pc_console_print_track_status(0U);
        pc_console_print_menu();
        return;
    }
    if(command == 'Z')
    {
        ok = track_calibration_verify_origin();
        pc_console_write(ok ?
            "CAL ORIGIN verified from stable visual frames; motors remain locked.\r\n" :
            "CAL ORIGIN rejected: wait for LIVE stable=1, then retry Z.\r\n");
        pc_console_print_track_status(0U);
        pc_console_print_menu();
        return;
    }
    if(command == 'T')
    {
        track_calibration_set_telemetry((uint8)!track.telemetry_enabled);
        pc_console_write(track.telemetry_enabled ?
            "CAL telemetry OFF; event and final record output remains enabled.\r\n" :
            "CAL telemetry ON; active-state status prints every 3 seconds.\r\n");
        pc_console_print_menu();
        return;
    }

    if(g_menu_state == PC_MENU_TRACK_MANUAL)
    {
        if(command == '1') ok = track_calibration_set_direction(0U);
        else if(command == '2') ok = track_calibration_set_direction(2U);
        else if(command == '3') ok = track_calibration_set_direction(4U);
        else if(command == '4') ok = track_calibration_set_direction(6U);
        else if(command == 'G') ok = pc_console_track_cycle_cells();
        else if(command == 'S') ok = pc_console_track_cycle_speed();
        else if(command == 'N') ok = pc_console_track_cycle_repeats();
        else if(command == 'L') ok = pc_console_track_cycle_load();
        else if(command == 'R')
        {
            ok = track_calibration_start_manual(0U);
            pc_console_write(ok ?
                "CAL manual segment armed; waiting for stable visual origin.\r\n" :
                "CAL start rejected: stop current task and wait for stable vision.\r\n");
        }
        else if(command == 'A')
        {
            ok = track_calibration_start_manual(1U);
            pc_console_write(ok ?
                "CAL manual repeat route armed; every segment stops and confirms vision.\r\n" :
                "CAL start rejected: stop current task and wait for stable vision.\r\n");
        }
        else if(command == 'C') ok = track_calibration_resume();
        else if(command == 'K') ok = track_calibration_skip();
        else if(command == 'P') pc_console_print_track_status(1U);
        else if(command == 'B') g_menu_state = PC_MENU_TRACK_MAIN;
        else if(command != 'H' && command != '?') handled = 0U;
    }
    else if(g_menu_state == PC_MENU_TRACK_AUTO)
    {
        if(command == '1') ok = track_calibration_set_profile(TRACK_CAL_PROFILE_QUICK);
        else if(command == '2') ok = track_calibration_set_profile(TRACK_CAL_PROFILE_STANDARD);
        else if(command == '3') ok = track_calibration_set_profile(TRACK_CAL_PROFILE_FULL);
        else if(command == 'R')
        {
            ok = track_calibration_start_auto();
            pc_console_write(ok ?
                "CAL automatic route armed; keep a clear area around the car.\r\n" :
                "CAL start rejected: stable visual pose and ready IMU are required.\r\n");
        }
        else if(command == 'P')
        {
            ok = track_calibration_pause();
            pc_console_write(ok ? "CAL route paused and motors locked.\r\n" :
                                  "CAL pause rejected: no route is active.\r\n");
        }
        else if(command == 'C')
        {
            ok = track_calibration_resume();
            pc_console_write(ok ? "CAL route resume requested; retrying current step.\r\n" :
                                  "CAL resume rejected: route is not paused.\r\n");
        }
        else if(command == 'K')
        {
            ok = track_calibration_skip();
            pc_console_write(ok ? "CAL failed step marked SKIPPED; route continues.\r\n" :
                                  "CAL skip rejected: pause/fault the route first.\r\n");
        }
        else if(command == 'S') pc_console_print_track_status(1U);
        else if(command == 'B') g_menu_state = PC_MENU_TRACK_MAIN;
        else if(command != 'H' && command != '?') handled = 0U;
    }
    else if(g_menu_state == PC_MENU_TRACK_STATIONARY)
    {
        if(command == '1') ok = track_calibration_start_stationary(10U);
        else if(command == '2') ok = track_calibration_start_stationary(30U);
        else if(command == 'P') pc_console_print_track_status(0U);
        else if(command == 'B') g_menu_state = PC_MENU_TRACK_MAIN;
        else if(command != 'H' && command != '?') handled = 0U;
    }
    else if(g_menu_state == PC_MENU_TRACK_VISION)
    {
        if(command == '1') ok = track_calibration_start_map_scan();
        else if(command == 'P') pc_console_print_track_status(0U);
        else if(command == 'B') g_menu_state = PC_MENU_TRACK_MAIN;
        else if(command != 'H' && command != '?') handled = 0U;
    }
    else if(g_menu_state == PC_MENU_TRACK_SETTINGS)
    {
        if(command == 'S') ok = pc_console_track_cycle_speed();
        else if(command == 'G') ok = pc_console_track_cycle_cells();
        else if(command == 'N') ok = pc_console_track_cycle_repeats();
        else if(command == 'L') ok = pc_console_track_cycle_load();
        else if(command == 'V') ok = pc_console_track_cycle_voltage();
        else if(command == 'P') pc_console_print_track_status(0U);
        else if(command == 'B') g_menu_state = PC_MENU_TRACK_MAIN;
        else if(command != 'H' && command != '?') handled = 0U;
    }
    else
    {
        if(command == '1') g_menu_state = PC_MENU_TRACK_MANUAL;
        else if(command == '2') g_menu_state = PC_MENU_TRACK_AUTO;
        else if(command == '3') g_menu_state = PC_MENU_TRACK_STATIONARY;
        else if(command == '4') g_menu_state = PC_MENU_TRACK_VISION;
        else if(command == '5') g_menu_state = PC_MENU_TRACK_SETTINGS;
        else if(command == '6') pc_console_print_track_status(1U);
        else if(command != 'H' && command != '?') handled = 0U;
    }

    if(!handled) pc_console_write("Unknown track-calibration command.\r\n");
    else if(!ok) pc_console_write("CAL command rejected while busy or value invalid.\r\n");
    track_calibration_get_snapshot(&track);
    g_last_track_event = track.event_counter;
    g_last_track_record = track.record_sequence;
    pc_console_print_menu();
}

static void pc_console_request_map(void)
{
    vision_link_snapshot_t s;

    vision_link_get_snapshot(&s);
    g_map_version_before_request = s.map_version;
    g_map_request_tick = pit_count;
    g_map_request_pending = 1U;
    vision_link_request_full_map();
    pc_console_write("FULL_MAP request sent; waiting for a newer map version.\r\n");
}

static void pc_console_handle_command(uint8 command)
{
    mission_result_t result;
    mission_status_t mission;

    if(command >= 'a' && command <= 'z')
    {
        command = (uint8)(command - 'a' + 'A');
    }

    if(g_app_mode == 3U)
    {
        pc_console_handle_track_command(command);
        return;
    }
    if(g_app_mode == 2U)
    {
        pc_console_handle_point_command(command);
        return;
    }

    if(g_menu_state == PC_MENU_PLANNER)
    {
        switch(command)
        {
            case 'S':
                pc_console_solve(0U);
                break;
            case 'D':
                pc_console_solve(1U);
                break;
            case 'P':
                pc_console_print_plan_summary();
                break;
            case 'A':
                pc_console_print_plan_detail();
                break;
            case 'B':
                g_menu_state = PC_MENU_MAIN;
                break;
            case 'X':
                pc_console_safe_stop();
                pc_console_write("EMERGENCY STOP: motor output disabled.\r\n");
                break;
            case 'H':
            case '?':
                break;
            default:
                pc_console_write("Unknown planner command.\r\n");
                break;
        }
        pc_console_print_menu();
        return;
    }

    if(g_menu_state == PC_MENU_RUN)
    {
        switch(command)
        {
            case 'E':
                result = mission_manager_arm_plan();
                pc_console_print_run_result("ARM", result);
                break;
            case 'N':
                result = mission_manager_run_next_step();
                pc_console_print_run_result("NEXT", result);
                if(result == MISSION_RESULT_OK)
                {
                    g_last_run_report_tick = pit_count;
                    g_last_telem_action = 0xFFU;
                    g_last_telem_waypoint = 0xFFU;
                }
                break;
            case 'P':
                result = mission_manager_pause();
                pc_console_print_run_result("PAUSE", result);
                break;
            case 'C':
                result = mission_manager_continue();
                pc_console_print_run_result("CONTINUE", result);
                break;
            case 'R':
                result = mission_manager_reset_cursor();
                pc_console_print_run_result("RESET_CURSOR", result);
                break;
            case 'K':
                mission_manager_disarm();
                pc_console_write("RUN_CMD DISARM result=OK; motors hard-stopped.\r\n");
                pc_console_print_mission_status();
                break;
            case 'A':
                result = mission_manager_run_all_steps();
                pc_console_print_run_result("AUTO_ALL", result);
                if(result == MISSION_RESULT_OK)
                {
                    g_last_run_report_tick = pit_count;
                    g_last_telem_action = 0xFFU;
                    g_last_telem_waypoint = 0xFFU;
                }
                break;
            case 'S':
                pc_console_print_mission_status();
                break;
            case 'T':
                g_run_telemetry = (uint8)!g_run_telemetry;
                g_last_run_report_tick = pit_count;
                g_last_telem_action = 0xFFU;
                g_last_telem_waypoint = 0xFFU;
                pc_console_write(g_run_telemetry ?
                    "RUN telemetry ON: 250 ms periodic frames enabled.\r\n" :
                    "RUN telemetry OFF: periodic frames disabled.\r\n");
                break;
            case 'B':
                mission_manager_disarm();
                g_menu_state = PC_MENU_MAIN;
                pc_console_write("Run session disarmed; motors hard-stopped.\r\n");
                break;
            case 'X':
                pc_console_safe_stop();
                pc_console_write("EMERGENCY STOP: motor output disabled.\r\n");
                pc_console_print_mission_status();
                break;
            case 'H':
            case '?':
                break;
            default:
                pc_console_write("Unknown run command.\r\n");
                break;
        }
        mission_manager_get_status(&mission);
        g_last_mission_event = mission.event_counter;
        pc_console_print_menu();
        return;
    }

    switch(command)
    {
        case '1':
            pc_console_request_map();
            break;
        case '2':
            g_menu_state = PC_MENU_PLANNER;
            break;
        case '3':
            g_menu_state = PC_MENU_RUN;
            break;
        case '4':
            pc_console_print_status();
            break;
        case '5':
            pc_console_print_cached_map();
            break;
        case 'V':
            g_verbose = (uint8)!g_verbose;
            pc_console_write(g_verbose ? "Link report mode: VERBOSE.\r\n" :
                                           "Link report mode: QUIET.\r\n");
            break;
        case 'X':
            pc_console_safe_stop();
            pc_console_write("EMERGENCY STOP: motor output disabled.\r\n");
            break;
        case 'H':
        case '?':
            break;
        default:
            pc_console_write("Unknown command.\r\n");
            break;
    }
    pc_console_print_menu();
}

static void pc_console_poll_input(void)
{
    uint8 value;
    uint32 direct_limit = PC_CONSOLE_DIRECT_POLL_LIMIT;

    while(direct_limit-- && pc_console_uart8_ring_read(&value))
    {
        pc_console_report_rx("isr", value);
        if(value == 0U || value == '\r' || value == '\n' ||
           value == ' ' || value == '\t')
        {
            continue;
        }
        pc_console_handle_command(value);
    }

    direct_limit = PC_CONSOLE_DIRECT_POLL_LIMIT;
    while(direct_limit-- && uart_query_byte(PC_CONSOLE_UART, &value))
    {
        pc_console_report_rx("poll", value);
        if(value == 0U || value == '\r' || value == '\n' ||
           value == ' ' || value == '\t')
        {
            continue;
        }
        pc_console_handle_command(value);
    }
}

static void pc_console_poll_map_request(void)
{
    vision_link_snapshot_t s;

    if(!g_map_request_pending)
    {
        return;
    }

    vision_link_get_snapshot(&s);
    if(s.map_valid && s.map_version > g_map_version_before_request)
    {
        g_map_request_pending = 0U;
        pc_console_write("FULL_MAP received.\r\n");
        pc_console_print_cached_map();
        pc_console_print_menu();
    }
    else if((uint32)(pit_count - g_map_request_tick) >= PC_CONSOLE_MAP_TIMEOUT_TICKS)
    {
        g_map_request_pending = 0U;
        pc_console_write("FULL_MAP timeout: no newer map received within 3 s.\r\n");
        pc_console_print_status();
        pc_console_print_menu();
    }
}

static void pc_console_poll_link_report(void)
{
    uint8 online = vision_link_is_online();

    if(!g_link_known || online != g_last_online)
    {
        g_link_known = 1U;
        g_last_online = online;
        g_last_link_report_tick = pit_count;
        pc_console_write(online ? "VISION CONNECTED.\r\n" : "VISION DISCONNECTED.\r\n");
        pc_console_print_status();
        return;
    }

    if(!online && (uint32)(pit_count - g_last_link_report_tick) >= PC_CONSOLE_LINK_REPORT_TICKS)
    {
        g_last_link_report_tick = pit_count;
        pc_console_write("VISION still disconnected.\r\n");
        pc_console_print_status();
    }

    if(g_verbose && (uint32)(pit_count - g_last_verbose_tick) >= PC_CONSOLE_VERBOSE_TICKS)
    {
        g_last_verbose_tick = pit_count;
        pc_console_print_status();
    }
}

static void pc_console_poll_mission_report(void)
{
    mission_status_t mission;
    char text[192];

    mission_manager_get_status(&mission);
    if(mission.event_counter != g_last_mission_event)
    {
        g_last_mission_event = mission.event_counter;
        g_last_run_report_tick = pit_count;
        pc_console_write("MISSION_EVENT\r\n");
        pc_console_print_mission_status();
        if(g_run_telemetry && mission.follower.pose_valid &&
           (mission.state == MISSION_STEP_WAIT ||
            mission.state == MISSION_COMPLETE ||
            mission.state == MISSION_FAULT))
        {
            pc_console_print_run_telemetry(&mission);
        }
        if(g_menu_state == PC_MENU_RUN)
        {
            pc_console_print_menu();
        }
        return;
    }

    if(mission.state == MISSION_PUSH_VERIFY && g_run_telemetry)
    {
        if((uint32)(pit_count - g_last_run_report_tick) >=
           PC_CONSOLE_RUN_TELEM_TICKS)
        {
            g_last_run_report_tick = pit_count;
            snprintf(text, sizeof(text),
                     "PUSH_WAIT box=(%u,%u)->(%u,%u) req=%u bad=%u map=%lu->%lu age=%lums motor=LOCKED\r\n",
                     (unsigned int)mission.box_from_x,
                     (unsigned int)mission.box_from_y,
                     (unsigned int)mission.box_to_x,
                     (unsigned int)mission.box_to_y,
                     (unsigned int)mission.push_verify_requests,
                     (unsigned int)mission.push_verify_bad_maps,
                     (unsigned long)mission.push_map_version_before,
                     (unsigned long)mission.push_map_version_after,
                     (unsigned long)mission.push_verify_age_ms);
            pc_console_write(text);
        }
    }
    else if((mission.state == MISSION_ACTION_RUNNING ||
        mission.state == MISSION_PAUSED) && g_run_telemetry)
    {
        if(mission.follower.action_index != g_last_telem_action ||
           mission.follower.waypoint_index != g_last_telem_waypoint)
        {
            if(mission.follower.action_index != g_last_telem_action)
            {
                g_last_telem_phase = 0xFFU;
            }
            if(g_last_telem_action == 0xFFU)
            {
                snprintf(text, sizeof(text),
                         "RUN_EVENT START action=%u wp=%u/%u\r\n",
                         (unsigned int)mission.follower.action_index,
                         (unsigned int)mission.follower.waypoint_index,
                         (unsigned int)mission.follower.waypoint_count);
            }
            else
            {
                snprintf(text, sizeof(text),
                         "RUN_EVENT WAYPOINT action=%u old=%u new=%u total=%u\r\n",
                         (unsigned int)mission.follower.action_index,
                         (unsigned int)g_last_telem_waypoint,
                         (unsigned int)mission.follower.waypoint_index,
                         (unsigned int)mission.follower.waypoint_count);
            }
            pc_console_write(text);
            g_last_telem_action = mission.follower.action_index;
            g_last_telem_waypoint = mission.follower.waypoint_index;
        }

        if((uint8)mission.follower.phase != g_last_telem_phase)
        {
            snprintf(text, sizeof(text),
                     "RUN_EVENT PHASE action=%u sub=%u step=%u phase=%s wait=%lums frames=%u/%u\r\n",
                     (unsigned int)mission.follower.action_index,
                     (unsigned int)mission.follower.waypoint_index,
                     (unsigned int)mission.follower.step_index,
                     action_follower_phase_name(mission.follower.phase),
                     (unsigned long)mission.follower.waypoint_hold_remaining_ms,
                     (unsigned int)mission.follower.visual_confirm_frames,
                     (unsigned int)mission.follower.visual_confirm_required);
            pc_console_write(text);
            g_last_telem_phase = (uint8)mission.follower.phase;
        }

        if((uint32)(pit_count - g_last_run_report_tick) >=
           PC_CONSOLE_RUN_TELEM_TICKS)
        {
            g_last_run_report_tick = pit_count;
            pc_console_print_run_telemetry(&mission);
        }
    }
}

static void pc_console_poll_point_report(void)
{
    point_test_snapshot_t point;

    point_test_get_snapshot(&point);
    if(point.event_counter != g_last_point_event)
    {
        g_last_point_event = point.event_counter;
        g_last_point_report_tick = pit_count;
        pc_console_write("POINT_EVENT state changed.\r\n");
        pc_console_print_point_status(1U);
        if(point.state == POINT_TEST_DONE || point.state == POINT_TEST_FAULT ||
           point.state == POINT_TEST_LOCKED)
        {
            pc_console_print_menu();
        }
        return;
    }

    if(g_point_telemetry &&
       (point.state == POINT_TEST_RUNNING ||
        point.state == POINT_TEST_SETTLING ||
        point.state == POINT_TEST_VIS_DRAIN ||
        point.state == POINT_TEST_VIS_STABLE ||
        g_menu_state == PC_MENU_POINT_SENSOR) &&
       (uint32)(pit_count - g_last_point_report_tick) >=
           PC_CONSOLE_POINT_TELEM_TICKS)
    {
        g_last_point_report_tick = pit_count;
        pc_console_print_point_status(
            g_menu_state == PC_MENU_POINT_SENSOR ? 0U : 1U);
    }
}

static void pc_console_poll_track_report(void)
{
    track_cal_snapshot_t track;
    uint8 terminal;

    track_calibration_get_snapshot(&track);
    terminal = (track.state == TRACK_CAL_PAUSED ||
                track.state == TRACK_CAL_COMPLETE ||
                track.state == TRACK_CAL_FAULT ||
                track.state == TRACK_CAL_LOCKED) ? 1U : 0U;

    if(track.record_sequence != g_last_track_record)
    {
        g_last_track_record = track.record_sequence;
        g_last_track_event = track.event_counter;
        g_last_track_report_tick = pit_count;
        pc_console_print_track_record(&track.last_record);
        pc_console_print_track_status(0U);
        if(terminal) pc_console_print_menu();
        return;
    }
    if(track.event_counter != g_last_track_event)
    {
        g_last_track_event = track.event_counter;
        g_last_track_report_tick = pit_count;
        pc_console_print_track_status(0U);
        if(terminal) pc_console_print_menu();
        return;
    }
    if(track.telemetry_enabled &&
       (track.state == TRACK_CAL_WAIT_ORIGIN ||
        track.state == TRACK_CAL_RUNNING ||
        track.state == TRACK_CAL_INTER_STEP ||
        track.state == TRACK_CAL_STATIONARY ||
        track.state == TRACK_CAL_MAP_SCAN) &&
       (uint32)(pit_count - g_last_track_report_tick) >=
           PC_CONSOLE_TRACK_TELEM_TICKS)
    {
        g_last_track_report_tick = pit_count;
        pc_console_print_track_status(0U);
    }
}

void pc_console_init(uint8 app_mode)
{
    char text[512];
    mission_status_t mission;

    g_app_mode = app_mode;
    pc_console_safe_stop();
    g_menu_state = g_app_mode == 3U ? PC_MENU_TRACK_MAIN :
        (g_app_mode == 2U ? PC_MENU_POINT_MAIN : PC_MENU_MAIN);
    g_last_online = vision_link_is_online();
    g_link_known = 0U;
    g_last_link_report_tick = pit_count;
    g_last_verbose_tick = pit_count;
    if(g_app_mode == 3U)
    {
        track_cal_snapshot_t track;
        track_calibration_get_snapshot(&track);
        g_last_track_event = track.event_counter;
        g_last_track_record = track.record_sequence;
    }
    else if(g_app_mode == 2U)
    {
        point_test_snapshot_t point;
        point_test_get_snapshot(&point);
        g_last_point_event = point.event_counter;
    }
    else
    {
        mission_manager_get_status(&mission);
        g_last_mission_event = mission.event_counter;
    }
    g_last_run_report_tick = pit_count;
    g_run_telem_sequence = 0U;
    g_last_point_report_tick = pit_count;
    g_last_track_report_tick = pit_count;
    g_point_telem_sequence = 0U;
    g_last_telem_action = 0xFFU;
    g_last_telem_waypoint = 0xFFU;
    g_last_telem_phase = 0xFFU;
    g_uart8_tx_head = 0U;
    g_uart8_tx_tail = 0U;
    g_uart8_tx_peak = 0U;
    g_uart8_tx_drops = 0U;
    if(g_app_mode == 3U)
    {
        pc_console_write("\r\n#MCU_BOOT id=7100893 stage=TRACK_CAL_AUTO_UART safe=1 debug=3 display=OFF\r\n");
        snprintf(text, sizeof(text),
                 "UART8_CONFIG id=7100893 baud=115200 tx=D16 rx=D17 rx_drop=%lu tx=IRQ_QUEUE tx_size=%u tx_isr_budget=%u telemetry=ON/3s display=OFF mode3=TRACK_CAL manual=ON auto=QUICK12+STANDARD60+FULL180 step=GRID1 stop=HOLD500+DRAIN1000+STABLE6_200MS gate=100mm wheel=%umm cpr=%u odom_scale_x1000=%u center=100mm cell=200mm\r\n",
                 (unsigned long)g_uart8_rx_drops,
                 (unsigned int)PC_CONSOLE_TX_RING_SIZE,
                 (unsigned int)PC_CONSOLE_TX_ISR_BUDGET,
                 (unsigned int)(motion_get_wheel_diameter_mm() + 0.5f),
                 (unsigned int)(motion_get_wheel_cpr() + 0.5f),
                 (unsigned int)(motion_get_odometry_scale() * 1000.0f + 0.5f));
    }
    else if(g_app_mode == 2U)
    {
        pc_console_write("\r\n#MCU_BOOT id=7100893 stage=TRACK_CAL_AUTO_UART safe=1 debug=2 display=OFF\r\n");
        snprintf(text, sizeof(text),
                 "UART8_CONFIG id=7100893 baud=115200 tx=D16 rx=D17 rx_drop=%lu tx=IRQ_QUEUE tx_size=%u tx_isr_budget=%u telemetry=ON/3s sensor_frame=COMPACT display=OFF pos_type=0x12 wheel=%umm cpr=%u odom_scale_x1000=%u center=100mm cell=200mm solver=GRID4+PLAN_CHECK heading=LOCK_ON_ARM m4=CAR_VALID+Z_STABLE6_200MS+ENC_IMU_MOVE+HOLD500+DRAIN1000+STABLE6_200MS+VIS_CHECK100 push=AUTO_POSE_ONLY+N_MAP_VERIFY\r\n",
                 (unsigned long)g_uart8_rx_drops,
                 (unsigned int)PC_CONSOLE_TX_RING_SIZE,
                 (unsigned int)PC_CONSOLE_TX_ISR_BUDGET,
                 (unsigned int)(motion_get_wheel_diameter_mm() + 0.5f),
                 (unsigned int)(motion_get_wheel_cpr() + 0.5f),
                 (unsigned int)(motion_get_odometry_scale() * 1000.0f + 0.5f));
    }
    else
    {
        pc_console_write("\r\n#MCU_BOOT id=7100893 stage=TRACK_CAL_AUTO_UART safe=1 debug=0 display=OFF\r\n");
        snprintf(text, sizeof(text),
                 "UART8_CONFIG id=7100893 baud=115200 tx=D16 rx=D17 rx_drop=%lu tx=IRQ_QUEUE tx_size=%u tx_isr_budget=%u telemetry=ON/250ms display=OFF pos_type=0x12 cpr=%u odom_scale_x1000=%u race=ONE_GRID_PER_STEP solver=GRID4+PLAN_CHECK heading=LOCK_ON_ARM m4=CAR_VALID+ENC_IMU+HOLD500+DRAIN1000+STABLE6+VIS_CHECK100 push=AUTO_POSE_ONLY+N_MAP_VERIFY\r\n",
                 (unsigned long)g_uart8_rx_drops,
                 (unsigned int)PC_CONSOLE_TX_RING_SIZE,
                 (unsigned int)PC_CONSOLE_TX_ISR_BUDGET,
                 (unsigned int)(motion_get_wheel_cpr() + 0.5f),
                 (unsigned int)(motion_get_odometry_scale() * 1000.0f + 0.5f));
    }
    pc_console_write(text);
    pc_console_print_menu();
}

void pc_console_poll(void)
{
    pc_console_tx_kick();
    pc_console_poll_input();
    if(g_app_mode == 0U)
    {
        pc_console_poll_map_request();
    }
    pc_console_poll_link_report();
    if(g_app_mode == 3U)
    {
        pc_console_poll_track_report();
    }
    else if(g_app_mode == 2U)
    {
        pc_console_poll_point_report();
    }
    else
    {
        pc_console_poll_mission_report();
    }
    pc_console_tx_kick();
}
