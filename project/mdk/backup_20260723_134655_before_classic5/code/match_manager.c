#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "blue.h"
#include "motion_control.h"
#include "vision_link.h"
#include "vision_world.h"
#include "point_test.h"
#include "action_follower.h"
#include "solver.h"
#include "status_buzzer.h"
#include "match_manager.h"

#define MATCH_TICK_MS                       (5U)
#define MATCH_MAP_REQUEST_PERIOD_TICKS      (200U)
#define MATCH_FINISH_REQUEST_PERIOD_TICKS   (120U)
#define MATCH_RETURN_PREP_TIMEOUT_TICKS     (600U)
#define MATCH_PLAN_START_SNAP_MAX_X10       (4)
#define MATCH_MAX_RETURN_PATH               (MAP_CELLS)
#define MATCH_DEFAULT_BASE_X                (1U)
#define MATCH_DEFAULT_BASE_Y                (5U)
#define MATCH_DEFAULT_BASE_ALT_Y            (6U)
#define MATCH_DEFAULT_EXIT_DIRECTION        (MATCH_EXIT_ROUTE_RIGHT_UP)
#define MATCH_DEFAULT_EXIT_DISTANCE_MM      (APP_MATCH_EXIT_DISTANCE_MM)
#define MATCH_DEFAULT_SPEED                 (APP_MATCH_EXIT_SPEED)
#define MATCH_DEFAULT_MAP_WAIT_MS           (APP_MATCH_MAP_WAIT_MS)
#define MATCH_DEFAULT_BETWEEN_ROUND_MS      (APP_MATCH_BETWEEN_ROUND_MS)
#define MATCH_DEFAULT_FINISH_SAMPLES        (APP_MATCH_FINISH_SCAN_SAMPLES)
#define MATCH_DEFAULT_RETURN_RETRIES        (APP_MATCH_RETURN_RETRIES)
#define MATCH_MISSION_ARM_RETRY_MAX          (APP_MATCH_MISSION_ARM_RETRIES)
#define MATCH_MISSION_RUNTIME_REPLAN_MAX     (APP_MATCH_RUNTIME_REPLANS)
#define MATCH_MISSION_ARM_RETRY_TIMEOUT_MS   (15000U)
#define MATCH_MISSION_ARM_STABLE_FRAMES      (6U)
#define MATCH_MISSION_ARM_STABLE_MS          (200U)
#define MATCH_PLANNER_REJECT_RETRY_MAX         (3U)
#define MATCH_POST_MAP_STABLE_FRAMES          (6U)
#define MATCH_POST_MAP_STABLE_MS              (200U)
#define MATCH_POST_MAP_POS_CLUSTER_X10         (2)
#define MATCH_POST_MAP_HEADING_CLUSTER_X10   (120)
#define MATCH_BASE_MAP_HEADING_DEG            (APP_MATCH_BASE_MAP_HEADING_DEG)
#define MATCH_BASE_ALIGN_PASS_X10              (1)
#define MATCH_BASE_ALIGN_RELAXED_X10           (2)
#define MATCH_BASE_ALIGN_MAX_ATTEMPTS          (3U)
#define MATCH_BASE_ALIGN_MAX_MM              (360.0f)
#define MATCH_BASE_ALIGN_BRAKE_ALLOWANCE_MM   (50.0f)
#define MATCH_BASE_VIS_DRAIN_TICKS            (200U)
#define MATCH_BASE_RAW_STABLE_FRAMES            (3U)
#define MATCH_BASE_RAW_CLUSTER_X10               (2)
#define MATCH_BASE_RAW_ANCHOR_RADIUS_X10        (12)
#define MATCH_BASE_RAW_MAX_AGE_TICKS           (100U)

typedef struct {
    uint8 x;
    uint8 y;
} match_cell_t;

static match_status_t g_match;
static uint32 g_state_tick;
static uint32 g_match_start_tick;
static uint32 g_round_start_tick;
static uint32 g_last_map_request_tick;
static uint32 g_finish_last_version;
static uint8 g_point_command_started;
static float g_base_imu_heading_deg;
static uint8 g_base_heading_valid;
static uint8 g_base_recheck_pending;
static uint32 g_base_recheck_tick;
static uint32 g_base_recheck_packets;
static uint32 g_base_raw_last_packets;
static uint8 g_base_raw_stable_frames;
static int16 g_base_raw_anchor_x10;
static int16 g_base_raw_anchor_y10;
static uint32 g_post_map_pos_packets;
static uint8 g_post_map_frame_id;
static uint8 g_post_map_stable_frames;
static uint32 g_post_map_stable_tick;
static int16 g_post_map_anchor_x10;
static int16 g_post_map_anchor_y10;
static int16 g_post_map_anchor_theta_x10;
static uint8 g_round_heading_valid;
static float g_round_imu_heading_deg;
static float g_round_map_heading_deg;
static int16 g_round_visual_heading_x10;
static uint8 g_mission_auto_started;
static uint8 g_mission_retry_stable_active;
static uint32 g_mission_retry_anchor_packets;
static uint32 g_mission_retry_stable_tick;
static uint32 g_mission_retry_map_version;
static uint8 g_mission_retry_runtime;
static uint8 g_mission_retry_need_map;
static uint8 g_return_direct_started;
static uint8 g_return_step_started;
static uint8 g_return_force_planned;
static vision_link_map_t g_finish_map;
static match_cell_t g_return_path[MATCH_MAX_RETURN_PATH];

static void match_base_raw_stability_reset(void)
{
    g_base_raw_last_packets = 0U;
    g_base_raw_stable_frames = 0U;
    g_base_raw_anchor_x10 = 0;
    g_base_raw_anchor_y10 = 0;
}

static uint8 match_base_raw_position_stable(
    const vision_link_snapshot_t *vision)
{
    int anchor_x10;
    int anchor_y10;

    if(vision == 0 || !vision_link_is_online() ||
       vision->pos_packets == 0U || vision->last_packet_tick == 0U ||
       (uint32)(pit_count - vision->last_packet_tick) >
           MATCH_BASE_RAW_MAX_AGE_TICKS)
    {
        match_base_raw_stability_reset();
        return 0U;
    }

    anchor_x10 = (int)g_match.base_anchor_x * 10;
    anchor_y10 = (int)g_match.base_anchor_y * 10;
    if(abs((int)vision->car_x_mm - anchor_x10) >
           MATCH_BASE_RAW_ANCHOR_RADIUS_X10 ||
       abs((int)vision->car_y_mm - anchor_y10) >
           MATCH_BASE_RAW_ANCHOR_RADIUS_X10)
    {
        match_base_raw_stability_reset();
        return 0U;
    }

    if(vision->pos_packets == g_base_raw_last_packets)
        return (uint8)(g_base_raw_stable_frames >=
                       MATCH_BASE_RAW_STABLE_FRAMES);

    g_base_raw_last_packets = vision->pos_packets;
    if(g_base_raw_stable_frames == 0U ||
       (abs((int)vision->car_x_mm - (int)g_base_raw_anchor_x10) <=
            MATCH_BASE_RAW_CLUSTER_X10 &&
        abs((int)vision->car_y_mm - (int)g_base_raw_anchor_y10) <=
            MATCH_BASE_RAW_CLUSTER_X10))
    {
        if(g_base_raw_stable_frames < MATCH_BASE_RAW_STABLE_FRAMES)
            g_base_raw_stable_frames++;
    }
    else
    {
        g_base_raw_stable_frames = 1U;
    }
    g_base_raw_anchor_x10 = vision->car_x_mm;
    g_base_raw_anchor_y10 = vision->car_y_mm;
    return (uint8)(g_base_raw_stable_frames >=
                   MATCH_BASE_RAW_STABLE_FRAMES);
}

static uint8 match_preflight_mask(void)
{
    uint8 mask = 0U;
    if(imu963ra_ready) mask |= MATCH_PREFLIGHT_IMU_READY;
    if(vision_link_is_online()) mask |= MATCH_PREFLIGHT_VISION_LINK;
    return mask;
}

static uint8 match_preflight_required(void)
{
    uint8 i;
    uint8 required = MATCH_PREFLIGHT_IMU_READY;

    /* 三轮全SKIP只运行编码器+IMU原路往返；只要存在RUN轮次，
       启动前仍必须确认全局视觉链路。 */
    for(i = 0U; i < APP_RACE_ROUND_COUNT; i++)
    {
        if(g_match.round_config[i].run)
        {
            required |= MATCH_PREFLIGHT_VISION_LINK;
            break;
        }
    }
    return required;
}

static float match_normalize_360(float angle)
{
    while(angle < 0.0f) angle += 360.0f;
    while(angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static float match_quantize_cardinal_x10(int16 heading_x10)
{
    int32 normalized = heading_x10;
    uint8 quadrant;

    while(normalized < 0) normalized += 3600;
    while(normalized >= 3600) normalized -= 3600;
    quadrant = (uint8)(((normalized + 450) / 900) & 3);
    return (float)quadrant * 90.0f;
}

static uint8 match_apply_round_heading_frame(void)
{
    if(!g_round_heading_valid) return 0U;
    return action_follower_set_mission_heading_frame(
        g_round_imu_heading_deg, g_round_map_heading_deg,
        g_round_visual_heading_x10);
}

static int16 match_abs_angle_delta_x10(int16 a, int16 b)
{
    int16 delta = (int16)(a - b);
    while(delta > 1800) delta = (int16)(delta - 3600);
    while(delta < -1800) delta = (int16)(delta + 3600);
    return delta < 0 ? (int16)-delta : delta;
}

static void match_enter_state(match_state_t state)
{
    if(g_match.state != state)
    {
        g_match.state = state;
        g_match.event_counter++;
    }
    g_state_tick = pit_count;
    g_match.state_elapsed_ms = 0U;
}

static void match_set_gate(match_gate_t gate)
{
    if(g_match.gate != gate)
    {
        g_match.gate = gate;
        g_match.event_counter++;
    }
}

static void match_stop_motion(void)
{
    mission_manager_emergency_stop();
    action_follower_abort();
    point_test_emergency_stop();
    motion_emergency_stop();
    device_init_flag = 1U;
}

static void match_latch_fault(match_fault_t fault)
{
    match_stop_motion();
    match_set_gate(MATCH_GATE_NONE);
    g_match.fault = fault;
    g_match.running = 0U;
    g_match.armed = 0U;
    status_buzzer_request(STATUS_BUZZER_EVENT_LOCKED);
    match_enter_state(MATCH_STATE_FAULT);
}

static uint8 match_result_recoverable(mission_result_t result)
{
    switch(result)
    {
        case MISSION_RESULT_POSE_INVALID:
        case MISSION_RESULT_START_POSE_CHANGED:
        case MISSION_RESULT_PLAN_STALE:
        case MISSION_RESULT_BAD_STATE:
        case MISSION_RESULT_REPLAN_REQUIRED:
        case MISSION_RESULT_FOLLOWER_FAULT:
        case MISSION_RESULT_PUSH_MAP_TIMEOUT:
        case MISSION_RESULT_PUSH_MAP_MISMATCH:
            return 1U;
        default:
            return 0U;
    }
}

static uint8 match_result_needs_map(mission_result_t result)
{
    return (uint8)(result != MISSION_RESULT_BAD_STATE &&
                   result != MISSION_RESULT_POSE_INVALID);
}

static uint8 match_map_wall(const vision_link_map_t *map, uint8 x, uint8 y)
{
    uint16 index;
    if(map == 0 || x >= map->width || y >= map->height) return 1U;
    index = (uint16)y * VISION_LINK_GRID_W + x;
    return (uint8)((map->wall_bits[index >> 3] >> (index & 7U)) & 1U);
}

static uint8 match_cell_in_list(const vision_link_cell_t *cells,
                                uint8 count, uint8 x, uint8 y)
{
    uint8 i;
    for(i = 0U; i < count; i++)
    {
        if(cells[i].gx == (int8)x && cells[i].gy == (int8)y) return 1U;
    }
    return 0U;
}

static uint8 match_map_blocked(const vision_link_map_t *map,
                               uint8 x, uint8 y,
                               uint8 start_x, uint8 start_y)
{
    if(x == start_x && y == start_y) return 0U;
    return (uint8)(match_map_wall(map, x, y) ||
        match_cell_in_list(map->boxes, map->box_count, x, y) ||
        match_cell_in_list(map->bombs, map->bomb_count, x, y));
}

static uint8 match_round_grid_x10(int16 value, uint8 max_value)
{
    int rounded = value >= 0 ? ((int)value + 5) / 10 : 0;
    if(rounded < 0) rounded = 0;
    if(rounded > (int)max_value) rounded = max_value;
    return (uint8)rounded;
}

static uint8 match_build_return_path(const vision_link_map_t *map,
                                     uint8 start_x, uint8 start_y)
{
    int16 parent[MAP_CELLS];
    uint8 queue[MAP_CELLS];
    uint8 reverse[MAP_CELLS];
    uint16 head = 0U;
    uint16 tail = 0U;
    uint16 start;
    int16 target = -1;
    uint16 index;
    uint16 length = 0U;
    uint8 target_x[4];
    uint8 target_y[4];
    uint8 target_count = 0U;
    uint8 i;

    if(map == 0 || !map->valid || start_x >= map->width ||
       start_y >= map->height) return 0U;

    target_x[target_count] = g_match.config.base_target_x;
    target_y[target_count++] = g_match.config.base_target_y;
    target_x[target_count] = g_match.config.base_target_x;
    target_y[target_count++] = g_match.config.base_alternate_y;
    if(g_match.config.base_target_x + 1U < map->width)
    {
        target_x[target_count] = g_match.config.base_target_x + 1U;
        target_y[target_count++] = g_match.config.base_target_y;
        target_x[target_count] = g_match.config.base_target_x + 1U;
        target_y[target_count++] = g_match.config.base_alternate_y;
    }

    for(index = 0U; index < MAP_CELLS; index++) parent[index] = -2;
    start = (uint16)start_y * MAP_MAX_W + start_x;
    parent[start] = -1;
    queue[tail++] = (uint8)start;

    while(head < tail)
    {
        uint16 current = queue[head++];
        uint8 cx = (uint8)(current % MAP_MAX_W);
        uint8 cy = (uint8)(current / MAP_MAX_W);
        static const int8 dx[4] = {0, 0, -1, 1};
        static const int8 dy[4] = {-1, 1, 0, 0};

        for(i = 0U; i < target_count; i++)
        {
            if(cx == target_x[i] && cy == target_y[i])
            {
                target = (int16)current;
                break;
            }
        }
        if(target >= 0) break;

        for(i = 0U; i < 4U; i++)
        {
            int nx = (int)cx + dx[i];
            int ny = (int)cy + dy[i];
            uint16 next;
            if(nx < 0 || ny < 0 || nx >= (int)map->width ||
               ny >= (int)map->height) continue;
            if(match_map_blocked(map, (uint8)nx, (uint8)ny,
                                 start_x, start_y)) continue;
            next = (uint16)ny * MAP_MAX_W + (uint16)nx;
            if(parent[next] != -2) continue;
            parent[next] = (int16)current;
            queue[tail++] = (uint8)next;
        }
    }

    if(target < 0) return 0U;
    while(target >= 0 && (uint16)target != start && length < MAP_CELLS)
    {
        reverse[length++] = (uint8)target;
        target = parent[(uint16)target];
    }
    if(target < 0) return 0U;
    if(length == 0U)
    {
        g_match.return_path_length = 0U;
        g_match.return_path_index = 0U;
        return 2U;
    }

    g_match.return_path_length = (uint8)length;
    g_match.return_path_index = 0U;
    for(index = 0U; index < length; index++)
    {
        uint8 cell = reverse[length - 1U - index];
        g_return_path[index].x = (uint8)(cell % MAP_MAX_W);
        g_return_path[index].y = (uint8)(cell / MAP_MAX_W);
    }
    return 1U;
}

static void match_copy_preset(uint8 preset_index)
{
    uint8 i;
    if(preset_index >= app_race_preset_count)
        preset_index = APP_RACE_DEFAULT_PRESET;
    g_match.preset_index = preset_index;
    for(i = 0U; i < APP_RACE_ROUND_COUNT; i++)
        g_match.round_config[i] = app_race_presets[preset_index].round[i];
}

static void match_apply_current_round_config(void)
{
    uint8 index = g_match.round_index == 0U ? 0U :
        (uint8)(g_match.round_index - 1U);
    const app_race_round_config_t *round;

    if(index >= APP_RACE_ROUND_COUNT)
        index = APP_RACE_ROUND_COUNT - 1U;
    round = &g_match.round_config[index];
    g_match.current_round_run = round->run ? 1U : 0U;
    g_match.current_round_strategy = (uint8)round->strategy;
    g_match.current_round_speed = round->speed;
    g_match.current_round_algorithm = (uint8)round->algorithm;
    g_match.profile = (match_profile_t)round->strategy;
    g_match.config.mission_speed = round->speed;
}

static void match_apply_profile(void)
{
    mission_run_profile_t mission_profile = MISSION_PROFILE_STANDARD;

    if(g_match.profile == MATCH_PROFILE_NORMAL)
        mission_profile = MISSION_PROFILE_NORMAL;
    else if(g_match.profile == MATCH_PROFILE_FAST)
        mission_profile = MISSION_PROFILE_FAST_SAFE;
    mission_manager_set_run_profile(mission_profile);
    action_follower_set_speed((float)g_match.config.mission_speed);
}

static void match_configure_base_route(void)
{
    g_match.base_anchor_x = 1U;
    g_match.base_align_attempts = 0U;
    g_match.base_error_x10 = 0;
    g_match.base_error_y10 = 0;

    switch(g_match.config.exit_direction_index)
    {
        case MATCH_EXIT_ROUTE_TOP:
            g_match.base_anchor_y = 5U;
            g_match.base_exit_x = 1U;
            g_match.base_exit_y = 4U;
            break;
        case MATCH_EXIT_ROUTE_RIGHT_DOWN:
            g_match.base_anchor_y = 6U;
            g_match.base_exit_x = 2U;
            g_match.base_exit_y = 6U;
            break;
        case MATCH_EXIT_ROUTE_BOTTOM:
            g_match.base_anchor_y = 6U;
            g_match.base_exit_x = 1U;
            g_match.base_exit_y = 7U;
            break;
        case MATCH_EXIT_ROUTE_RIGHT_UP:
        default:
            g_match.config.exit_direction_index = MATCH_EXIT_ROUTE_RIGHT_UP;
            g_match.base_anchor_y = 5U;
            g_match.base_exit_x = 2U;
            g_match.base_exit_y = 5U;
            break;
    }
    g_match.config.base_target_x = g_match.base_anchor_x;
    g_match.config.base_target_y = g_match.base_anchor_y;
    g_match.config.base_alternate_y =
        g_match.base_anchor_y == 5U ? 6U : 5U;
}

static float match_base_exit_map_direction(void)
{
    /* 比赛发车固定沿车头平移；地图角90度对应车体方向0度。 */
    return MATCH_BASE_MAP_HEADING_DEG;
}

static uint8 match_start_base_map_translation(float map_direction_deg,
                                              uint16 distance_mm,
                                              uint8 add_brake_allowance)
{
    float body_direction_deg;
    uint16 command_distance = distance_mm;

    if(!g_base_heading_valid || !imu963ra_ready) return 0U;
    if(add_brake_allowance)
    {
        uint16 allowance = (uint16)MATCH_BASE_ALIGN_BRAKE_ALLOWANCE_MM;
        command_distance = (uint16)(command_distance + allowance);
    }
    body_direction_deg = match_normalize_360(
        map_direction_deg - MATCH_BASE_MAP_HEADING_DEG);

    point_test_emergency_stop();
    if(!point_test_set_sensor_mode(POINT_SENSOR_ENCODER_IMU) ||
       !point_test_set_speed(g_match.config.exit_speed) ||
       !point_test_set_direction_deg(body_direction_deg) ||
       !point_test_set_distance_mm(command_distance) ||
       !point_test_set_startup_assist(1U) ||
       !point_test_set_fast_finish(0U) ||
       !point_test_set_heading_target(g_base_imu_heading_deg) ||
       !point_test_capture_origin())
    {
        point_test_emergency_stop();
        return 0U;
    }
    point_test_clear_vision_origin();
    if(!point_test_start_translation())
    {
        point_test_emergency_stop();
        return 0U;
    }
    return 1U;
}

static uint8 match_start_base_anchor_alignment(int16 error_x10,
                                               int16 error_y10)
{
    float dx_mm = (float)error_x10 * 20.0f;
    float dy_mm = (float)error_y10 * 20.0f;
    float distance_mm = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);
    float map_direction_deg;

    if(distance_mm < 1.0f || distance_mm > MATCH_BASE_ALIGN_MAX_MM)
        return 0U;
    map_direction_deg = match_normalize_360(
        atan2f(dx_mm, -dy_mm) * 57.2957795f);
    if(!match_start_base_map_translation(
           map_direction_deg, (uint16)(distance_mm + 0.5f), 1U))
        return 0U;
    status_buzzer_request(STATUS_BUZZER_EVENT_STRONG_ALIGN);
    return 1U;
}

static void match_begin_map_wait(void)
{
    vision_link_snapshot_t vision;
    vision_link_get_snapshot(&vision);
    g_match.map_version_before = vision.map_version;
    g_match.map_version_current = vision.map_version;
    g_match.map_loaded = 0U;
    g_match.map_request_count++;
    g_last_map_request_tick = pit_count;
    vision_link_request_full_map();
    status_buzzer_request(STATUS_BUZZER_EVENT_MAP_REQUEST);
    match_set_gate(MATCH_GATE_MAP);
    match_enter_state(MATCH_STATE_MAP_WAIT);
}

static uint8 match_map_looks_loaded(const vision_link_map_t *map)
{
    return (uint8)(map != 0 && map->valid &&
        map->width == VISION_LINK_GRID_W &&
        map->height == VISION_LINK_GRID_H &&
        map->box_count <= MAX_BOXES && map->goal_count <= MAX_BOXES &&
        map->bomb_count <= MAX_BOXES &&
        (map->box_count > 0U || map->goal_count > 0U ||
         map->bomb_count > 0U));
}

static void match_select_solver(const vision_link_map_t *map)
{
    solver_mode_t mode = SOLVER_MODE_CLASSIC;
    (void)map;
    g_match.auto_label_degraded = 0U;
    if(g_match.current_round_algorithm == APP_RACE_ALGO_IMAGE_ONLY)
        mode = SOLVER_MODE_IMAGE_ONLY;
    else if(g_match.current_round_algorithm == APP_RACE_ALGO_BOMB_IMAGE)
        mode = SOLVER_MODE_BOMB_IMAGE;
    solver_set_mode(mode);
    g_match.selected_solver_mode = (uint8)mode;
}

static void match_begin_mission_arm_retry(
    const vision_link_snapshot_t *vision,
    uint8 runtime_replan,
    mission_result_t reason)
{
    g_match.recovery_reason = reason;
    mission_manager_disarm();
    action_follower_abort();
    motion_emergency_stop();
    device_init_flag = 1U;
    g_mission_auto_started = 0U;
    g_mission_retry_stable_active = 0U;
    g_mission_retry_anchor_packets =
        vision != 0 ? vision->pos_packets : 0U;
    g_mission_retry_map_version =
        vision != 0 ? vision->map_version : 0U;
    g_mission_retry_stable_tick = pit_count;
    g_mission_retry_runtime = runtime_replan ? 1U : 0U;
    g_mission_retry_need_map = match_result_needs_map(reason);
    if(g_mission_retry_runtime)
    {
        if(g_match.mission_runtime_replans <
           MATCH_MISSION_RUNTIME_REPLAN_MAX)
            g_match.mission_runtime_replans++;
        else if(g_match.gate_wait_cycles < 255U)
            g_match.gate_wait_cycles++;
    }
    else
    {
        if(g_match.mission_arm_retries < MATCH_MISSION_ARM_RETRY_MAX)
            g_match.mission_arm_retries++;
        else if(g_match.gate_wait_cycles < 255U)
            g_match.gate_wait_cycles++;
    }
    g_match.mission_arm_stable_frames = 0U;
    if(g_mission_retry_need_map)
    {
        vision_link_request_full_map();
        g_match.map_request_count++;
        g_last_map_request_tick = pit_count;
    }
    match_set_gate(runtime_replan ? MATCH_GATE_RUNTIME_REPLAN :
                   (reason == MISSION_RESULT_BAD_STATE ?
                       MATCH_GATE_HEADING_STABLE : MATCH_GATE_PLAN_REFRESH));
    match_enter_state(MATCH_STATE_MISSION_RETRY_WAIT);
}

static uint8 match_mission_retry_available(void)
{
    return g_mission_retry_runtime ?
        (uint8)(g_match.mission_runtime_replans <
                MATCH_MISSION_RUNTIME_REPLAN_MAX) :
        (uint8)(g_match.mission_arm_retries <
                MATCH_MISSION_ARM_RETRY_MAX);
}

static uint8 match_finish_map_is_clear(const vision_link_map_t *map)
{
    if(map == 0 || !map->valid || g_match.initial_box_count == 0U)
        return 0U;
    return (uint8)(map->box_count == 0U && map->bomb_count == 0U);
}

static void match_begin_finish_scan(void)
{
    vision_link_snapshot_t vision;
    memset(&g_finish_map, 0, sizeof(g_finish_map));
    vision_link_get_snapshot(&vision);
    g_finish_last_version = vision.map_version;
    g_match.finish_samples = 0U;
    g_match.finish_clear_samples = 0U;
    g_match.finish_remaining_samples = g_match.config.finish_scan_samples;
    g_match.map_clear = 0U;
    g_match.map_request_count++;
    g_last_map_request_tick = pit_count;
    vision_link_request_full_map();
    status_buzzer_request(STATUS_BUZZER_EVENT_MAP_REQUEST);
    match_set_gate(MATCH_GATE_FINISH_SCAN);
    match_enter_state(MATCH_STATE_FINISH_SCAN);
}

static void match_begin_return(void)
{
    g_return_direct_started = 0U;
    g_return_step_started = 0U;
    g_return_force_planned = 0U;
    g_match.return_path_length = 0U;
    g_match.return_path_index = 0U;
    g_match.return_retries = 0U;
    match_set_gate(MATCH_GATE_RETURN_POSE);
    match_enter_state(MATCH_STATE_RETURN_PREP);
}

static void match_finish_round(void)
{
    match_stop_motion();
    g_match.rounds_completed++;
    match_set_gate(MATCH_GATE_NONE);
    match_enter_state(MATCH_STATE_ROUND_DONE);
}

static uint8 match_planner_start_pose_plausible(void)
{
    const vision_world_snapshot_t *world = planner_service_get_world();
    int16 dx10;
    int16 dy10;

    if(world == 0 || !world->car_cell_adjusted) return 1U;
    dx10 = (int16)world->solver_map.car_x * 10 - world->car_x10;
    dy10 = (int16)world->solver_map.car_y * 10 - world->car_y10;
    if(dx10 < 0) dx10 = (int16)-dx10;
    if(dy10 < 0) dy10 = (int16)-dy10;
    return (uint8)(dx10 <= MATCH_PLAN_START_SNAP_MAX_X10 &&
                   dy10 <= MATCH_PLAN_START_SNAP_MAX_X10);
}

static void match_prepare_next_round(void)
{
    g_match.round_index++;
    g_round_start_tick = pit_count;
    g_match.round_success = 0U;
    g_match.map_clear = 0U;
    g_match.map_loaded = 0U;
    g_match.fault = MATCH_FAULT_NONE;
    g_point_command_started = 0U;
    g_post_map_pos_packets = 0U;
    g_post_map_frame_id = 0U;
    g_post_map_stable_frames = 0U;
    g_post_map_stable_tick = pit_count;
    g_post_map_anchor_x10 = 0;
    g_post_map_anchor_y10 = 0;
    g_post_map_anchor_theta_x10 = 0;
    g_round_heading_valid = 0U;
    g_round_imu_heading_deg = 0.0f;
    g_round_map_heading_deg = 0.0f;
    g_round_visual_heading_x10 = 0;
    g_match.round_heading_valid = 0U;
    g_match.round_imu_heading_x10 = 0;
    g_match.round_map_heading_x10 = 0;
    g_match.round_visual_heading_x10 = 0;
    g_mission_auto_started = 0U;
    g_mission_retry_stable_active = 0U;
    g_mission_retry_anchor_packets = 0U;
    g_mission_retry_stable_tick = pit_count;
    g_match.mission_arm_retries = 0U;
    g_match.mission_runtime_replans = 0U;
    g_match.mission_arm_stable_frames = 0U;
    g_match.gate_wait_cycles = 0U;
    g_match.planner_reject_streak = 0U;
    g_match.initial_counts_valid = 0U;
    g_match.initial_box_count = 0U;
    g_match.initial_goal_count = 0U;
    g_match.initial_bomb_count = 0U;
    g_match.recovery_reason = MISSION_RESULT_OK;
    g_match.gate = MATCH_GATE_NONE;
    match_configure_base_route();
    match_apply_current_round_config();
    g_base_imu_heading_deg = imu963ra_yaw_angle;
    g_base_heading_valid = imu963ra_ready ? 1U : 0U;
    g_base_recheck_pending = 0U;
    g_base_recheck_tick = pit_count;
    g_base_recheck_packets = 0U;
    match_base_raw_stability_reset();
    g_mission_retry_map_version = 0U;
    g_mission_retry_runtime = 0U;
    g_mission_retry_need_map = 0U;
    mission_manager_disarm();
    planner_service_init();
    if(g_match.skip_base_once)
    {
        g_match.skip_base_once = 0U;
        match_begin_map_wait();
    }
    else
    {
        /* 基地阶段完全屏蔽视觉位置与角度，只用编码器和开局IMU向前发车。 */
        match_set_gate(MATCH_GATE_NONE);
        match_enter_state(MATCH_STATE_EXIT_PREP);
    }
}

static uint8 match_start_direct_return(const vision_link_snapshot_t *pose)
{
    float imu_heading;
    float map_heading;
    float dx_mm;
    float dy_mm;
    float distance;
    float map_direction;
    float body_direction;

    if(pose == 0 || !pose->pose_valid ||
       (!action_follower_get_heading_frame(&imu_heading, &map_heading) &&
        (!match_apply_round_heading_frame() ||
         !action_follower_get_heading_frame(&imu_heading, &map_heading))))
        return 0U;
    dx_mm = ((float)g_match.config.base_target_x * 10.0f -
             (float)pose->car_x_mm) * 20.0f;
    dy_mm = ((float)g_match.config.base_target_y * 10.0f -
             (float)pose->car_y_mm) * 20.0f;
    distance = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);
    if(distance < 50.0f) return 2U;
    if(distance > 2720.0f) return 0U;
    map_direction = match_normalize_360(
        atan2f(dx_mm, -dy_mm) * 57.2957795f);
    body_direction = match_normalize_360(map_direction - map_heading);

    point_test_emergency_stop();
    if(!point_test_set_sensor_mode(POINT_SENSOR_ENCODER_IMU) ||
       !point_test_set_speed(g_match.config.exit_speed) ||
       !point_test_set_direction_deg(body_direction) ||
       !point_test_set_distance_mm((uint16)(distance + 0.5f)) ||
       !point_test_set_startup_assist(1U) ||
       !point_test_set_fast_finish(0U) ||
       !point_test_set_heading_target(imu_heading) ||
       !point_test_capture_origin() ||
       !point_test_start_translation())
    {
        point_test_emergency_stop();
        return 0U;
    }
    return 1U;
}

static uint8 match_start_return_grid_step(uint8 from_x, uint8 from_y,
                                          uint8 to_x, uint8 to_y)
{
    action_follower_step_context_t context;
    memset(&context, 0, sizeof(context));
    context.segment_cells = 1U;
    context.gear = ACTION_GEAR_TRANSITION_REANCHOR;
    context.strong_reanchor = 1U;
    context.strict_position = 1U;
    context.vision_drain_ms = 600U;
    context.vision_stable_frames = 4U;
    context.vision_stable_ms = 100U;
    context.align_max_distance_mm = 120U;
    return action_follower_start_grid_step(
        0xFEU, g_match.return_path_index,
        g_match.return_path_length, g_match.return_path_index,
        from_x, from_y, to_x, to_y, 0U, &context);
}

void match_manager_init(void)
{
    memset(&g_match, 0, sizeof(g_match));
    memset(&g_finish_map, 0, sizeof(g_finish_map));
    memset(g_return_path, 0, sizeof(g_return_path));
    g_match.state = MATCH_STATE_IDLE;
    g_match.kind = MATCH_KIND_FULL_THREE_ROUNDS;
    g_match.profile = MATCH_PROFILE_SAFE;
    g_match.label_policy = MATCH_LABEL_ROUND_SEQUENCE;
    g_match.next_policy = MATCH_NEXT_MAP;
    g_match.telemetry_level = (match_telemetry_level_t)APP_MATCH_TELEMETRY_LEVEL;
    g_match.config.base_target_x = MATCH_DEFAULT_BASE_X;
    g_match.config.base_target_y = MATCH_DEFAULT_BASE_Y;
    g_match.config.base_alternate_y = MATCH_DEFAULT_BASE_ALT_Y;
    g_match.config.exit_direction_index = MATCH_DEFAULT_EXIT_DIRECTION;
    g_match.config.exit_distance_mm = MATCH_DEFAULT_EXIT_DISTANCE_MM;
    g_match.config.exit_speed = MATCH_DEFAULT_SPEED;
    g_match.config.mission_speed = MATCH_DEFAULT_SPEED;
    g_match.config.map_wait_ms = MATCH_DEFAULT_MAP_WAIT_MS;
    g_match.config.between_round_ms = MATCH_DEFAULT_BETWEEN_ROUND_MS;
    g_match.config.finish_scan_samples = MATCH_DEFAULT_FINISH_SAMPLES;
    g_match.config.return_max_retries = MATCH_DEFAULT_RETURN_RETRIES;
    g_match.config.return_policy = MATCH_RETURN_AUTO;
    g_match.config_round_cursor = 0U;
    match_copy_preset(APP_RACE_DEFAULT_PRESET);
    match_configure_base_route();
    g_match.last_key_event = 0xFFU;
    g_match.recovery_reason = MISSION_RESULT_OK;
    g_match.gate = MATCH_GATE_NONE;
    g_match.preflight_mask = match_preflight_mask();
    g_state_tick = pit_count;
    g_match_start_tick = pit_count;
    g_round_start_tick = pit_count;
    g_base_imu_heading_deg = 0.0f;
    g_base_heading_valid = 0U;
    g_base_recheck_pending = 0U;
    g_base_recheck_tick = pit_count;
    g_base_recheck_packets = 0U;
}

uint8 match_manager_select_kind(match_kind_t kind)
{
    if(match_manager_is_active() || kind > MATCH_KIND_SCORE_GUARD) return 0U;
#if !MATCH_SCORE_GUARD_ENABLE
    if(kind == MATCH_KIND_SCORE_GUARD) return 0U;
#endif
    g_match.kind = kind;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_arm(void)
{
    if(match_manager_is_active()) return 0U;
    match_stop_motion();
    planner_service_init();
    g_match.fault = MATCH_FAULT_NONE;
    g_match.armed = 1U;
    g_match.running = 0U;
    g_match.remote_start = 0U;
    g_match.skip_base_once = 0U;
    g_match.round_index = 0U;
    g_match.round_target =
        g_match.kind == MATCH_KIND_SINGLE_ROUND ? 1U : 3U;
    g_match.rounds_completed = 0U;
    g_match.round_success = 0U;
    g_match.map_clear = 0U;
    g_match.map_loaded = 0U;
    g_match.planner_status = PLANNER_STATUS_NO_PLAN;
    g_match.mission_state = MISSION_SAFE_IDLE;
    g_match.mission_result = MISSION_RESULT_OK;
    g_match.mission_arm_retries = 0U;
    g_match.mission_runtime_replans = 0U;
    g_match.mission_arm_stable_frames = 0U;
    g_match.gate_wait_cycles = 0U;
    g_match.planner_reject_streak = 0U;
    g_match.initial_counts_valid = 0U;
    g_match.initial_box_count = 0U;
    g_match.initial_goal_count = 0U;
    g_match.initial_bomb_count = 0U;
    g_match.recovery_reason = MISSION_RESULT_OK;
    g_match.gate = MATCH_GATE_NONE;
    g_mission_retry_stable_active = 0U;
    g_mission_retry_anchor_packets = 0U;
    g_mission_retry_stable_tick = pit_count;
    g_mission_retry_map_version = 0U;
    g_mission_retry_runtime = 0U;
    g_mission_retry_need_map = 0U;
    match_enter_state(MATCH_STATE_ARMED);
    return 1U;
}

uint8 match_manager_arm_loaded_map_debug(void)
{
    if(!match_manager_arm()) return 0U;
    g_match.skip_base_once = 1U;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_start_remote(void)
{
    uint8 preflight_required;
    if(g_match.state != MATCH_STATE_ARMED || !g_match.armed) return 0U;
    g_match.preflight_mask = match_preflight_mask();
    preflight_required = match_preflight_required();
    if((g_match.preflight_mask & preflight_required) != preflight_required)
    {
        match_set_gate(MATCH_GATE_PREFLIGHT);
        g_match.event_counter++;
        return 0U;
    }
    match_set_gate(MATCH_GATE_NONE);
    g_match.running = 1U;
    g_match.remote_start = 1U;
    g_match_start_tick = pit_count;
    status_buzzer_request(STATUS_BUZZER_EVENT_PREPARE_START);
    match_prepare_next_round();
    return 1U;
}

uint8 match_manager_continue_next_round(void)
{
    if(g_match.state != MATCH_STATE_WAIT_OPERATOR || !g_match.armed)
        return 0U;
    match_prepare_next_round();
    return 1U;
}

uint8 match_manager_withdraw_round(void)
{
    if(!g_match.running || !g_match.armed)
        return 0U;

    switch(g_match.state)
    {
        case MATCH_STATE_SOLVING:
        case MATCH_STATE_MISSION_ARMING:
        case MATCH_STATE_MISSION_RETRY_WAIT:
        case MATCH_STATE_MISSION_RUNNING:
            break;
        default:
            return 0U;
    }

    match_stop_motion();
    g_mission_auto_started = 0U;
    g_mission_retry_stable_active = 0U;
    g_mission_retry_runtime = 0U;
    g_mission_retry_need_map = 0U;
    g_match.round_success = 0U;
    g_match.mission_result = MISSION_RESULT_REPLAN_REQUIRED;
    g_match.recovery_reason = MISSION_RESULT_REPLAN_REQUIRED;
    match_begin_finish_scan();
    return 1U;
}

void match_manager_cancel(void)
{
    match_stop_motion();
    g_match.running = 0U;
    g_match.armed = 0U;
    g_match.remote_start = 0U;
    g_match.fault = MATCH_FAULT_OPERATOR_STOP;
    match_enter_state(MATCH_STATE_IDLE);
}

void match_manager_emergency_stop(void)
{
    match_stop_motion();
    g_match.running = 0U;
    g_match.armed = 0U;
    g_match.fault = MATCH_FAULT_OPERATOR_STOP;
    status_buzzer_request(STATUS_BUZZER_EVENT_LOCKED);
    match_enter_state(MATCH_STATE_FAULT);
}

uint8 match_manager_is_active(void)
{
    return (uint8)(g_match.state != MATCH_STATE_IDLE &&
        g_match.state != MATCH_STATE_COMPLETE &&
        g_match.state != MATCH_STATE_FAULT);
}

void match_manager_poll(void)
{
    vision_link_snapshot_t vision;
    vision_link_map_t map;
    point_test_snapshot_t point;
    mission_status_t mission;
    action_follower_state_t follower;
    uint32 elapsed_ticks;
    float return_imu_heading;
    float return_map_heading;
    uint8 return_path_result;

#if !MATCH_MANAGER_ENABLE
    return;
#endif
    elapsed_ticks = (uint32)(pit_count - g_state_tick);
    g_match.state_elapsed_ms = elapsed_ticks * MATCH_TICK_MS;
    if(g_match.running)
    {
        g_match.match_elapsed_ms =
            (uint32)(pit_count - g_match_start_tick) * MATCH_TICK_MS;
        g_match.round_elapsed_ms =
            (uint32)(pit_count - g_round_start_tick) * MATCH_TICK_MS;
    }
    vision_link_get_snapshot(&vision);
    g_match.preflight_mask = match_preflight_mask();
    g_match.map_version_current = vision.map_version;
    mission_manager_get_status(&mission);
    g_match.mission_state = mission.state;
    g_match.mission_result = mission.last_result;

    switch(g_match.state)
    {
        case MATCH_STATE_BASE_POSE_WAIT:
            point_test_get_snapshot(&point);
            if(!g_base_heading_valid || !imu963ra_ready)
            {
                match_set_gate(MATCH_GATE_PREFLIGHT);
                break;
            }
            /* At the screen edge car_valid may flicker although the decoded
               center remains fresh and stable.  Base alignment deliberately
               uses position only; camera heading stays blocked. */
            if(!match_base_raw_position_stable(&vision))
            {
                match_set_gate(MATCH_GATE_POSITION_STABLE);
                break;
            }
            if(g_base_recheck_pending &&
               ((uint32)(pit_count - g_base_recheck_tick) <
                    MATCH_BASE_VIS_DRAIN_TICKS ||
                vision.pos_packets == g_base_recheck_packets))
            {
                match_set_gate(MATCH_GATE_POSITION_STABLE);
                break;
            }

            g_base_recheck_pending = 0U;
            g_match.base_error_x10 = (int16)(
                (int)g_match.base_anchor_x * 10 - (int)vision.car_x_mm);
            g_match.base_error_y10 = (int16)(
                (int)g_match.base_anchor_y * 10 - (int)vision.car_y_mm);

            if((abs((int)g_match.base_error_x10) <=
                    MATCH_BASE_ALIGN_PASS_X10 &&
                abs((int)g_match.base_error_y10) <=
                    MATCH_BASE_ALIGN_PASS_X10) ||
               (g_match.base_align_attempts >=
                    MATCH_BASE_ALIGN_MAX_ATTEMPTS &&
                abs((int)g_match.base_error_x10) <=
                    MATCH_BASE_ALIGN_RELAXED_X10 &&
                abs((int)g_match.base_error_y10) <=
                    MATCH_BASE_ALIGN_RELAXED_X10))
            {
                point_test_emergency_stop();
                status_buzzer_request(STATUS_BUZZER_EVENT_NODE_REACHED);
                match_set_gate(MATCH_GATE_NONE);
                g_point_command_started = 0U;
                match_enter_state(MATCH_STATE_EXIT_PREP);
                break;
            }

            if(g_match.base_align_attempts >=
                   MATCH_BASE_ALIGN_MAX_ATTEMPTS ||
               !match_start_base_anchor_alignment(
                   g_match.base_error_x10, g_match.base_error_y10))
            {
                match_latch_fault(MATCH_FAULT_BASE_EXIT);
                break;
            }
            g_match.base_align_attempts++;
            g_point_command_started = 1U;
            match_set_gate(MATCH_GATE_NONE);
            match_enter_state(MATCH_STATE_BASE_ALIGN_RUNNING);
            break;

        case MATCH_STATE_BASE_ALIGN_RUNNING:
            point_test_get_snapshot(&point);
            if(point.state == POINT_TEST_DONE)
            {
                g_point_command_started = 0U;
                g_base_recheck_pending = 1U;
                g_base_recheck_tick = pit_count;
                g_base_recheck_packets = vision.pos_packets;
                match_base_raw_stability_reset();
                match_set_gate(MATCH_GATE_POSITION_STABLE);
                match_enter_state(MATCH_STATE_BASE_POSE_WAIT);
            }
            else if(point.state == POINT_TEST_FAULT ||
                    point.state == POINT_TEST_LOCKED)
            {
                match_latch_fault(MATCH_FAULT_BASE_EXIT);
            }
            break;

        case MATCH_STATE_EXIT_PREP:
            if(!g_point_command_started)
            {
                /* Position was aligned inside the base.  The final grid is
                   encoder+IMU only: camera heading remains completely
                   blocked and map heading is fixed at 90 degrees. */
                match_set_gate(MATCH_GATE_NONE);
                g_point_command_started = 1U;
                if(!match_start_base_map_translation(
                       match_base_exit_map_direction(),
                       g_match.config.exit_distance_mm, 0U))
                {
                    match_latch_fault(MATCH_FAULT_POINT_START);
                    return;
                }
                match_enter_state(MATCH_STATE_EXIT_RUNNING);
            }
            break;

        case MATCH_STATE_EXIT_RUNNING:
            point_test_get_snapshot(&point);
            if(point.state == POINT_TEST_DONE)
            {
                g_point_command_started = 0U;
                status_buzzer_request(STATUS_BUZZER_EVENT_NODE_REACHED);
                if(!g_match.current_round_run)
                {
                    /* point_test在报告DONE前已原航向停车保持500ms。
                       SKIP不请求地图、不等待视觉，立即按相同距离向后平移。 */
                    if(!match_start_base_map_translation(
                           APP_MATCH_SKIP_RETURN_DIRECTION_DEG,
                           g_match.config.exit_distance_mm, 0U))
                    {
                        match_latch_fault(MATCH_FAULT_RETURN_MOVE);
                        break;
                    }
                    g_point_command_started = 1U;
                    match_set_gate(MATCH_GATE_NONE);
                    match_enter_state(MATCH_STATE_SKIP_RETURN);
                }
                else
                {
                    match_begin_map_wait();
                }
            }
            else if(point.state == POINT_TEST_FAULT ||
                    point.state == POINT_TEST_LOCKED)
            {
                match_latch_fault(MATCH_FAULT_BASE_EXIT);
            }
            break;

        case MATCH_STATE_SKIP_RETURN:
            point_test_get_snapshot(&point);
            if(point.state == POINT_TEST_DONE)
            {
                g_point_command_started = 0U;
                g_match.round_success = 0U;
                status_buzzer_request(STATUS_BUZZER_EVENT_NODE_REACHED);
                match_finish_round();
            }
            else if(point.state == POINT_TEST_FAULT ||
                    point.state == POINT_TEST_LOCKED)
            {
                match_latch_fault(MATCH_FAULT_RETURN_MOVE);
            }
            break;

        case MATCH_STATE_MAP_WAIT:
            if((uint32)(pit_count - g_last_map_request_tick) >=
               MATCH_MAP_REQUEST_PERIOD_TICKS)
            {
                g_last_map_request_tick = pit_count;
                g_match.map_request_count++;
                vision_link_request_full_map();
            }
            if(vision_link_get_map(&map) &&
               map.map_version > g_match.map_version_before &&
               match_map_looks_loaded(&map))
            {
                g_match.map_loaded = 1U;
                if(!g_match.initial_counts_valid)
                {
                    g_match.initial_box_count = map.box_count;
                    g_match.initial_goal_count = map.goal_count;
                    g_match.initial_bomb_count = map.bomb_count;
                    g_match.initial_counts_valid = 1U;
                }
                g_finish_map = map;
                if(g_match.kind == MATCH_KIND_SCORE_GUARD)
                    match_begin_return();
                else
                {
                    /* Only pose packets newer than this accepted map may
                       establish the planning pose and heading. */
                    g_post_map_pos_packets = vision.pos_packets;
                    g_post_map_frame_id = vision.frame_id;
                    g_post_map_stable_frames = 0U;
                    g_post_map_stable_tick = pit_count;
                    g_match.mission_arm_stable_frames = 0U;
                    match_set_gate(MATCH_GATE_POSITION_STABLE);
                    match_enter_state(MATCH_STATE_POST_MAP_POSE_WAIT);
                }
            }
            else if(g_match.state_elapsed_ms >= g_match.config.map_wait_ms)
            {
                /* Missing or half-loaded maps are external/transient.  Keep
                   requesting while exposing the gate instead of ending the
                   match after the car has already left the base. */
                if(g_match.gate_wait_cycles < 255U)
                    g_match.gate_wait_cycles++;
                g_match.event_counter++;
                g_state_tick = pit_count;
                g_last_map_request_tick = pit_count;
                g_match.map_request_count++;
                vision_link_request_full_map();
                match_set_gate(MATCH_GATE_MAP);
            }
            break;

        case MATCH_STATE_POST_MAP_POSE_WAIT:
            if(!vision_link_is_online() || !vision.pose_valid)
            {
                g_post_map_stable_frames = 0U;
                g_match.mission_arm_stable_frames = 0U;
                match_set_gate(MATCH_GATE_POSE);
                break;
            }
            if(vision.pos_packets == g_post_map_pos_packets)
            {
                match_set_gate(MATCH_GATE_POSITION_STABLE);
                break;
            }
            g_post_map_pos_packets = vision.pos_packets;
            if(vision.frame_id == g_post_map_frame_id)
                break;
            g_post_map_frame_id = vision.frame_id;

            if(g_post_map_stable_frames == 0U ||
               (abs((int)vision.car_x_mm -
                    (int)g_post_map_anchor_x10) <=
                    MATCH_POST_MAP_POS_CLUSTER_X10 &&
                abs((int)vision.car_y_mm -
                    (int)g_post_map_anchor_y10) <=
                    MATCH_POST_MAP_POS_CLUSTER_X10 &&
                match_abs_angle_delta_x10(
                    vision.car_theta_x10,
                    g_post_map_anchor_theta_x10) <=
                    MATCH_POST_MAP_HEADING_CLUSTER_X10))
            {
                if(g_post_map_stable_frames == 0U)
                {
                    g_post_map_anchor_x10 = vision.car_x_mm;
                    g_post_map_anchor_y10 = vision.car_y_mm;
                    g_post_map_anchor_theta_x10 = vision.car_theta_x10;
                    g_post_map_stable_tick = pit_count;
                }
                if(g_post_map_stable_frames <
                   MATCH_POST_MAP_STABLE_FRAMES)
                    g_post_map_stable_frames++;
            }
            else
            {
                g_post_map_anchor_x10 = vision.car_x_mm;
                g_post_map_anchor_y10 = vision.car_y_mm;
                g_post_map_anchor_theta_x10 = vision.car_theta_x10;
                g_post_map_stable_frames = 1U;
                g_post_map_stable_tick = pit_count;
            }
            g_match.mission_arm_stable_frames =
                g_post_map_stable_frames;
            match_set_gate(MATCH_GATE_POSITION_STABLE);
            if(g_post_map_stable_frames >=
                   MATCH_POST_MAP_STABLE_FRAMES &&
               (uint32)(pit_count - g_post_map_stable_tick) *
                   MATCH_TICK_MS >= MATCH_POST_MAP_STABLE_MS)
            {
                if(!g_round_heading_valid)
                {
                    g_round_imu_heading_deg = g_base_heading_valid ?
                        g_base_imu_heading_deg : imu963ra_yaw_angle;
                    g_round_map_heading_deg = match_quantize_cardinal_x10(
                        g_post_map_anchor_theta_x10);
                    g_round_visual_heading_x10 =
                        g_post_map_anchor_theta_x10;
                    g_round_heading_valid = 1U;
                    g_match.round_heading_valid = 1U;
                    g_match.round_imu_heading_x10 = (int16)(
                        g_round_imu_heading_deg * 10.0f + 0.5f);
                    g_match.round_map_heading_x10 = (int16)(
                        g_round_map_heading_deg * 10.0f + 0.5f);
                    g_match.round_visual_heading_x10 =
                        g_round_visual_heading_x10;
                }
                if(!match_apply_round_heading_frame())
                {
                    match_latch_fault(MATCH_FAULT_MISSION_ARM);
                    break;
                }
                match_set_gate(MATCH_GATE_NONE);
                if(g_match.current_round_run)
                {
                    match_enter_state(MATCH_STATE_SOLVING);
                }
                else
                {
                    /* 跳过本图仍需先出基地并确认非空地图，随后立即返航。 */
                    g_match.round_success = 0U;
                    match_begin_return();
                }
            }
            break;

        case MATCH_STATE_SOLVING:
            if(!vision_link_get_map(&map) || !match_map_looks_loaded(&map))
            {
                g_match.recovery_reason = MISSION_RESULT_PLAN_STALE;
                match_begin_map_wait();
                break;
            }
            match_select_solver(&map);
            planner_service_init();
            g_match.planner_status = planner_service_solve();
            if(g_match.planner_status != PLANNER_STATUS_OK)
            {
                g_match.recovery_reason = MISSION_RESULT_PLAN_STALE;
                if(g_match.planner_reject_streak < 255U)
                    g_match.planner_reject_streak++;
                if(g_match.gate_wait_cycles < 255U)
                    g_match.gate_wait_cycles++;

                /* A transient capture race gets fresh maps.  Repeated world
                   rejection means this round is no longer plannable; finish
                   it as incomplete and use the existing map-aware return. */
                if(g_match.planner_reject_streak >=
                   MATCH_PLANNER_REJECT_RETRY_MAX)
                {
                    match_stop_motion();
                    g_match.round_success = 0U;
                    g_match.mission_result = MISSION_RESULT_PLAN_STALE;
                    g_match.event_counter++;
                    match_begin_finish_scan();
                    break;
                }
                match_set_gate(MATCH_GATE_PLAN_REFRESH);
                match_begin_map_wait();
                break;
            }
            g_match.planner_reject_streak = 0U;
            status_buzzer_request(STATUS_BUZZER_EVENT_SOLVE_DONE);
            if(!match_planner_start_pose_plausible())
            {
                /* Do not start from a distant cell silently selected by the
                   planner when the live pose lies on a wall/boundary. */
                g_match.recovery_reason = MISSION_RESULT_START_POSE_CHANGED;
                match_begin_mission_arm_retry(
                    &vision, g_mission_retry_runtime,
                    MISSION_RESULT_START_POSE_CHANGED);
                break;
            }
            match_apply_profile();
            match_set_gate(MATCH_GATE_NONE);
            match_enter_state(MATCH_STATE_MISSION_ARMING);
            break;

        case MATCH_STATE_MISSION_ARMING:
            if(!g_mission_auto_started)
            {
                mission_result_t result;
                if(!match_apply_round_heading_frame())
                {
                    match_latch_fault(MATCH_FAULT_MISSION_ARM);
                    break;
                }
                result = mission_manager_arm_plan_prevalidated_pose();
                if(result != MISSION_RESULT_OK)
                {
                    g_match.mission_result = result;
                    if(match_mission_retry_available() &&
                       match_result_recoverable(result))
                    {
                        match_begin_mission_arm_retry(
                            &vision, g_mission_retry_runtime, result);
                    }
                    else if(match_result_recoverable(result))
                    {
                        g_match.round_success = 0U;
                        match_begin_finish_scan();
                    }
                    else
                    {
                        match_latch_fault(MATCH_FAULT_MISSION_ARM);
                    }
                    break;
                }
                g_mission_auto_started = 1U;
                match_set_gate(MATCH_GATE_NONE);
            }
            mission_manager_get_status(&mission);
            if(mission.state == MISSION_STEP_WAIT)
            {
                mission_result_t run_result = mission_manager_run_all_steps();
                if(run_result != MISSION_RESULT_OK)
                {
                    g_match.mission_result = run_result;
                    if(match_result_recoverable(run_result))
                        match_begin_mission_arm_retry(
                            &vision, g_mission_retry_runtime, run_result);
                    else
                        match_latch_fault(MATCH_FAULT_MISSION);
                    break;
                }
                match_enter_state(MATCH_STATE_MISSION_RUNNING);
                g_mission_retry_runtime = 0U;
                match_set_gate(MATCH_GATE_NONE);
            }
            else if(mission.state == MISSION_PAUSED &&
                    mission.last_result == MISSION_RESULT_REPLAN_REQUIRED &&
                    match_mission_retry_available())
            {
                    match_begin_mission_arm_retry(
                        &vision, g_mission_retry_runtime,
                        mission.last_result);
            }
            else if((mission.state == MISSION_FAULT ||
                     mission.state == MISSION_PAUSED) &&
                    match_result_recoverable(mission.last_result) &&
                    match_mission_retry_available())
            {
                match_begin_mission_arm_retry(
                    &vision, g_mission_retry_runtime,
                    mission.last_result);
            }
            else if(mission.state == MISSION_FAULT ||
                    mission.state == MISSION_PAUSED)
            {
                if(match_result_recoverable(mission.last_result))
                {
                    g_match.round_success = 0U;
                    match_begin_finish_scan();
                }
                else match_latch_fault(MATCH_FAULT_MISSION_ARM);
            }
            break;

        case MATCH_STATE_MISSION_RETRY_WAIT:
            if(g_match.state_elapsed_ms >=
               MATCH_MISSION_ARM_RETRY_TIMEOUT_MS)
            {
                /* Stay visible and retry the external gate.  Timeouts here
                   are not proof of an internal software or motor failure. */
                if(g_match.gate_wait_cycles < 255U)
                    g_match.gate_wait_cycles++;
                g_match.event_counter++;
                g_state_tick = pit_count;
                g_mission_retry_stable_active = 0U;
                g_match.mission_arm_stable_frames = 0U;
                if(g_mission_retry_need_map)
                {
                    g_mission_retry_map_version = vision.map_version;
                    g_last_map_request_tick = pit_count;
                    g_match.map_request_count++;
                    vision_link_request_full_map();
                }
                break;
            }
            if(!vision_link_is_online() || !vision.pose_valid)
            {
                match_set_gate(MATCH_GATE_POSE);
                g_mission_retry_stable_active = 0U;
                g_match.mission_arm_stable_frames = 0U;
                break;
            }
            if(g_mission_retry_need_map && (!vision.map_valid ||
               vision.map_version <= g_mission_retry_map_version)
              )
            {
                match_set_gate(MATCH_GATE_MAP);
                if((uint32)(pit_count - g_last_map_request_tick) >=
                   MATCH_MAP_REQUEST_PERIOD_TICKS)
                {
                    vision_link_request_full_map();
                    g_match.map_request_count++;
                    g_last_map_request_tick = pit_count;
                }
                break;
            }
            point_test_get_snapshot(&point);
            if(!point.vision_position_stable)
            {
                match_set_gate(MATCH_GATE_POSITION_STABLE);
                g_mission_retry_stable_active = 0U;
                g_match.mission_arm_stable_frames = 0U;
                break;
            }
            if(g_match.recovery_reason == MISSION_RESULT_BAD_STATE &&
               !point.vision_input_stable)
            {
                match_set_gate(MATCH_GATE_HEADING_STABLE);
                g_mission_retry_stable_active = 0U;
                g_match.mission_arm_stable_frames = 0U;
                break;
            }
            if(!g_mission_retry_stable_active)
            {
                g_mission_retry_stable_active = 1U;
                g_mission_retry_anchor_packets = vision.pos_packets;
                g_mission_retry_stable_tick = pit_count;
                g_match.mission_arm_stable_frames = 0U;
                break;
            }
            elapsed_ticks =
                (uint32)(vision.pos_packets -
                         g_mission_retry_anchor_packets);
            g_match.mission_arm_stable_frames =
                elapsed_ticks > 255U ? 255U : (uint8)elapsed_ticks;
            if(elapsed_ticks >= MATCH_MISSION_ARM_STABLE_FRAMES &&
               (uint32)(pit_count - g_mission_retry_stable_tick) *
                   MATCH_TICK_MS >= MATCH_MISSION_ARM_STABLE_MS)
            {
                g_mission_auto_started = 0U;
                g_mission_retry_stable_active = 0U;
                if(g_mission_retry_need_map)
                {
                    planner_service_init();
                    match_set_gate(MATCH_GATE_PLAN_REFRESH);
                    match_enter_state(MATCH_STATE_SOLVING);
                }
                else
                {
                    match_set_gate(MATCH_GATE_NONE);
                    match_enter_state(MATCH_STATE_MISSION_ARMING);
                }
            }
            break;

        case MATCH_STATE_MISSION_RUNNING:
            mission_manager_get_status(&mission);
            if(mission.state == MISSION_COMPLETE)
            {
                g_match.round_success = 1U;
#if MATCH_FINISH_MAP_RECHECK_ENABLE
                match_begin_finish_scan();
#else
                g_match.map_clear = 1U;
                match_begin_return();
#endif
            }
            else if(mission.state == MISSION_PAUSED &&
                    mission.last_result == MISSION_RESULT_REPLAN_REQUIRED &&
                    g_match.mission_runtime_replans <
                        MATCH_MISSION_RUNTIME_REPLAN_MAX)
            {
                match_begin_mission_arm_retry(
                    &vision, 1U, mission.last_result);
            }
            else if((mission.state == MISSION_FAULT ||
                     mission.state == MISSION_PAUSED) &&
                    match_result_recoverable(mission.last_result) &&
                    g_match.mission_runtime_replans <
                        MATCH_MISSION_RUNTIME_REPLAN_MAX)
            {
                match_begin_mission_arm_retry(
                    &vision, 1U, mission.last_result);
            }
            else if(mission.state == MISSION_FAULT ||
                    mission.state == MISSION_PAUSED)
            {
                g_match.round_success = 0U;
                match_begin_finish_scan();
            }
            break;

        case MATCH_STATE_FINISH_SCAN:
            if(vision_link_get_map(&map) &&
               map.map_version > g_finish_last_version)
            {
                g_finish_last_version = map.map_version;
                g_finish_map = map;
                g_match.finish_samples++;
                if(match_finish_map_is_clear(&map))
                    g_match.finish_clear_samples++;
                g_match.finish_remaining_samples =
                    g_match.finish_samples >= g_match.config.finish_scan_samples ?
                    0U : (uint8)(g_match.config.finish_scan_samples -
                                  g_match.finish_samples);
                if(g_match.finish_samples >= g_match.config.finish_scan_samples)
                {
                    g_match.map_clear = (uint8)(
                        g_match.finish_clear_samples ==
                        g_match.config.finish_scan_samples);
                    if(!g_match.map_clear) g_match.round_success = 0U;
                    match_begin_return();
                    break;
                }
            }
            if((uint32)(pit_count - g_last_map_request_tick) >=
               MATCH_FINISH_REQUEST_PERIOD_TICKS)
            {
                g_last_map_request_tick = pit_count;
                g_match.map_request_count++;
                vision_link_request_full_map();
            }
            if(g_match.state_elapsed_ms >= g_match.config.map_wait_ms)
            {
                /* Finishing evidence is optional for returning home. */
                g_match.map_clear = 0U;
                g_match.round_success = 0U;
                match_begin_return();
            }
            break;

        case MATCH_STATE_RETURN_PREP:
#if !MATCH_RETURN_HOME_ENABLE
            match_finish_round();
            break;
#endif
            vision_link_get_snapshot(&vision);
            if(!vision.pose_valid || !vision_link_is_online())
            {
                if(elapsed_ticks >= MATCH_RETURN_PREP_TIMEOUT_TICKS)
                {
                    if(g_match.gate_wait_cycles < 255U)
                        g_match.gate_wait_cycles++;
                    g_match.event_counter++;
                    g_state_tick = pit_count;
                    g_last_map_request_tick = pit_count;
                    g_match.map_request_count++;
                    vision_link_request_full_map();
                }
                match_set_gate(MATCH_GATE_RETURN_POSE);
                break;
            }
            if(!g_return_force_planned &&
               (g_match.config.return_policy == MATCH_RETURN_DIRECT ||
               (g_match.config.return_policy == MATCH_RETURN_AUTO &&
                g_match.map_clear)))
            {
                uint8 result = match_start_direct_return(&vision);
                if(result == 2U)
                    match_finish_round();
                else if(result == 1U)
                {
                    g_return_direct_started = 1U;
                    match_set_gate(MATCH_GATE_NONE);
                    match_enter_state(MATCH_STATE_RETURN_DIRECT);
                }
                else
                    g_return_force_planned = 1U;
                break;
            }
            if(!vision_link_get_map(&map)) map = g_finish_map;
            if(!action_follower_get_heading_frame(
                   &return_imu_heading, &return_map_heading) &&
               !match_apply_round_heading_frame())
            {
                if(elapsed_ticks >= MATCH_RETURN_PREP_TIMEOUT_TICKS)
                {
                    if(g_match.gate_wait_cycles < 255U)
                        g_match.gate_wait_cycles++;
                    g_match.event_counter++;
                    g_state_tick = pit_count;
                    g_match.map_request_count++;
                    vision_link_request_full_map();
                }
                match_set_gate(MATCH_GATE_RETURN_POSE);
                break;
            }
            return_path_result = match_build_return_path(
                &map,
                match_round_grid_x10(vision.car_x_mm,
                                     (uint8)(map.width - 1U)),
                match_round_grid_x10(vision.car_y_mm,
                                     (uint8)(map.height - 1U)));
            if(return_path_result == 2U)
            {
                match_finish_round();
                break;
            }
            if(return_path_result == 0U)
            {
                match_latch_fault(MATCH_FAULT_RETURN_NO_PATH);
                break;
            }
            match_set_gate(MATCH_GATE_NONE);
            match_enter_state(MATCH_STATE_RETURN_PATH);
            break;

        case MATCH_STATE_RETURN_DIRECT:
            point_test_get_snapshot(&point);
            if(point.state == POINT_TEST_DONE)
                match_finish_round();
            else if(point.state == POINT_TEST_FAULT ||
                    point.state == POINT_TEST_LOCKED)
            {
                point_test_emergency_stop();
                g_return_direct_started = 0U;
                g_return_force_planned = 1U;
                match_set_gate(MATCH_GATE_RETURN_POSE);
                match_enter_state(MATCH_STATE_RETURN_PREP);
            }
            break;

        case MATCH_STATE_RETURN_PATH:
            if(g_match.return_path_index >= g_match.return_path_length)
            {
                match_finish_round();
                break;
            }
            follower = action_follower_state();
            if(!g_return_step_started)
            {
                uint8 from_x;
                uint8 from_y;
                if(g_match.return_path_index == 0U)
                {
                    vision_link_get_snapshot(&vision);
                    from_x = match_round_grid_x10(
                        vision.car_x_mm, VISION_LINK_GRID_W - 1U);
                    from_y = match_round_grid_x10(
                        vision.car_y_mm, VISION_LINK_GRID_H - 1U);
                }
                else
                {
                    from_x = g_return_path[g_match.return_path_index - 1U].x;
                    from_y = g_return_path[g_match.return_path_index - 1U].y;
                }
                if(!match_start_return_grid_step(
                       from_x, from_y,
                       g_return_path[g_match.return_path_index].x,
                       g_return_path[g_match.return_path_index].y))
                {
                    action_follower_abort();
                    if(g_match.return_retries <
                       g_match.config.return_max_retries)
                    {
                        g_match.return_retries++;
                        g_return_step_started = 0U;
                        match_set_gate(MATCH_GATE_RETURN_POSE);
                        match_enter_state(MATCH_STATE_RETURN_PREP);
                    }
                    else
                        match_latch_fault(MATCH_FAULT_RETURN_MOVE);
                    break;
                }
                g_return_step_started = 1U;
            }
            else if(follower == ACTION_FOLLOWER_DONE)
            {
                g_match.return_path_index++;
                g_return_step_started = 0U;
                status_buzzer_request(STATUS_BUZZER_EVENT_NODE_REACHED);
                g_match.event_counter++;
            }
            else if(follower == ACTION_FOLLOWER_REPLAN ||
                    follower == ACTION_FOLLOWER_FAULT)
            {
                if(g_match.return_retries <
                   g_match.config.return_max_retries)
                {
                    g_match.return_retries++;
                    g_return_step_started = 0U;
                    action_follower_abort();
                    match_enter_state(MATCH_STATE_RETURN_PREP);
                }
                else
                    match_latch_fault(MATCH_FAULT_RETURN_MOVE);
            }
            break;

        case MATCH_STATE_ROUND_DONE:
            if(g_match.rounds_completed >= g_match.round_target)
            {
                g_match.running = 0U;
                g_match.armed = 0U;
                match_enter_state(MATCH_STATE_COMPLETE);
            }
            else if(g_match.next_policy == MATCH_WAIT_OPERATOR)
            {
                match_enter_state(MATCH_STATE_WAIT_OPERATOR);
            }
            else
            {
                match_enter_state(MATCH_STATE_BETWEEN_ROUNDS);
            }
            break;

        case MATCH_STATE_BETWEEN_ROUNDS:
            if(g_match.state_elapsed_ms >= g_match.config.between_round_ms)
                match_prepare_next_round();
            break;

        default:
            break;
    }
}

uint8 match_manager_cycle_profile(void)
{
    if(match_manager_is_active()) return 0U;
    g_match.profile = (match_profile_t)(((uint8)g_match.profile + 1U) % 3U);
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_label_policy(void)
{
    if(match_manager_is_active()) return 0U;
    g_match.label_policy = (match_label_policy_t)(
        ((uint8)g_match.label_policy + 1U) % 4U);
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_next_policy(void)
{
    if(match_manager_is_active()) return 0U;
    g_match.next_policy = (match_next_policy_t)(
        ((uint8)g_match.next_policy + 1U) % 3U);
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_return_policy(void)
{
    if(match_manager_is_active()) return 0U;
    g_match.config.return_policy = (match_return_policy_t)(
        ((uint8)g_match.config.return_policy + 1U) % 3U);
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_exit_distance(void)
{
    if(match_manager_is_active()) return 0U;
    g_match.config.exit_distance_mm = MATCH_DEFAULT_EXIT_DISTANCE_MM;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_exit_direction(void)
{
    if(match_manager_is_active()) return 0U;
    g_match.config.exit_direction_index = (uint8)(
        (g_match.config.exit_direction_index + 1U) % 4U);
    match_configure_base_route();
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_set_exit_direction(uint8 direction_index)
{
    if(match_manager_is_active() ||
       direction_index > MATCH_EXIT_ROUTE_BOTTOM)
    {
        return 0U;
    }
    g_match.config.exit_direction_index = direction_index;
    match_configure_base_route();
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_base_lane(void)
{
    if(match_manager_is_active()) return 0U;
    return match_manager_cycle_exit_direction();
}

uint8 match_manager_cycle_finish_samples(void)
{
    if(match_manager_is_active()) return 0U;
    if(g_match.config.finish_scan_samples == 3U)
        g_match.config.finish_scan_samples = 5U;
    else if(g_match.config.finish_scan_samples == 5U)
        g_match.config.finish_scan_samples = 7U;
    else g_match.config.finish_scan_samples = 3U;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_map_wait(void)
{
    if(match_manager_is_active()) return 0U;
    if(g_match.config.map_wait_ms == 10000U)
        g_match.config.map_wait_ms = 15000U;
    else if(g_match.config.map_wait_ms == 15000U)
        g_match.config.map_wait_ms = 30000U;
    else g_match.config.map_wait_ms = 10000U;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_telemetry(void)
{
    g_match.telemetry_level = (match_telemetry_level_t)(
        ((uint8)g_match.telemetry_level + 1U) % 5U);
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_set_speed(uint16 speed)
{
    if(match_manager_is_active() ||
       (speed != 100U && speed != 120U && speed != 150U)) return 0U;
    g_match.config.exit_speed = speed;
    g_match.config.mission_speed = speed;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_select_preset(uint8 preset_index)
{
    if(match_manager_is_active() || preset_index >= app_race_preset_count)
        return 0U;
    match_copy_preset(preset_index);
    g_match.kind = MATCH_KIND_FULL_THREE_ROUNDS;
    g_match.next_policy = MATCH_NEXT_MAP;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_preset(void)
{
    uint8 next;
    if(match_manager_is_active()) return 0U;
    next = g_match.preset_index >= app_race_preset_count ? 0U :
        (uint8)((g_match.preset_index + 1U) % app_race_preset_count);
    return match_manager_select_preset(next);
}

uint8 match_manager_select_config_round(uint8 round_index)
{
    if(match_manager_is_active() || round_index >= APP_RACE_ROUND_COUNT)
        return 0U;
    g_match.config_round_cursor = round_index;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_config_round(void)
{
    if(match_manager_is_active()) return 0U;
    g_match.config_round_cursor = (uint8)(
        (g_match.config_round_cursor + 1U) % APP_RACE_ROUND_COUNT);
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_toggle_round_run(void)
{
    app_race_round_config_t *round;
    if(match_manager_is_active()) return 0U;
    round = &g_match.round_config[g_match.config_round_cursor];
    round->run = round->run ? 0U : 1U;
    g_match.preset_index = 0xFFU;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_round_strategy(void)
{
    app_race_round_config_t *round;
    if(match_manager_is_active()) return 0U;
    round = &g_match.round_config[g_match.config_round_cursor];
    round->strategy = (app_race_strategy_t)(
        ((uint8)round->strategy + 1U) % 3U);
    g_match.preset_index = 0xFFU;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_round_speed(void)
{
    app_race_round_config_t *round;
    if(match_manager_is_active()) return 0U;
    round = &g_match.round_config[g_match.config_round_cursor];
    if(round->speed == 100U) round->speed = 120U;
    else if(round->speed == 120U) round->speed = 150U;
    else round->speed = 100U;
    g_match.preset_index = 0xFFU;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_cycle_round_algorithm(void)
{
    app_race_round_config_t *round;
    if(match_manager_is_active()) return 0U;
    round = &g_match.round_config[g_match.config_round_cursor];
    round->algorithm = (app_race_algorithm_t)(
        ((uint8)round->algorithm + 1U) % 3U);
    g_match.preset_index = 0xFFU;
    g_match.event_counter++;
    return 1U;
}

uint8 match_manager_handle_key(match_key_event_t event)
{
#if !MATCH_BUTTON_MENU_ENABLE
    (void)event;
    return 0U;
#else
    uint8 result = 1U;
    g_match.last_key_event = (uint8)event;
    if(event == MATCH_KEY1_SHORT) result = match_manager_cycle_preset();
    else if(event == MATCH_KEY1_DOUBLE)
        result = match_manager_cycle_config_round();
    else if(event == MATCH_KEY1_LONG)
    {
        if(g_match.state == MATCH_STATE_ARMED)
            result = match_manager_start_remote();
        else result = match_manager_arm();
    }
    else if(event == MATCH_KEY2_SHORT)
        result = match_manager_cycle_round_strategy();
    else if(event == MATCH_KEY2_DOUBLE)
        result = match_manager_toggle_round_run();
    else if(event == MATCH_KEY2_LONG)
        match_manager_emergency_stop();
    else result = 0U;
    return result;
#endif
}

uint8 match_manager_inject_fault(uint8 fault_code)
{
#if MATCH_FAULT_INJECTION_ENABLE
    if(g_match.running || fault_code == 0U) return 0U;
    g_match.fault_injection = fault_code;
    g_match.event_counter++;
    return 1U;
#else
    (void)fault_code;
    return 0U;
#endif
}

void match_manager_get_status(match_status_t *out)
{
    if(out != 0) *out = g_match;
}

const char *match_state_name(match_state_t state)
{
    switch(state)
    {
        case MATCH_STATE_IDLE: return "IDLE";
        case MATCH_STATE_ARMED: return "ARMED_WAIT_CONFIRM";
        case MATCH_STATE_BASE_POSE_WAIT: return "BASE_POS_ONLY_ALIGN";
        case MATCH_STATE_BASE_ALIGN_RUNNING: return "BASE_ALIGN_ENC_IMU";
        case MATCH_STATE_EXIT_PREP: return "EXIT_PREP";
        case MATCH_STATE_EXIT_RUNNING: return "EXIT_RUNNING";
        case MATCH_STATE_SKIP_RETURN: return "SKIP_RETURN";
        case MATCH_STATE_MAP_WAIT: return "WAIT_NEW_MAP";
        case MATCH_STATE_POST_MAP_POSE_WAIT: return "WAIT_POST_MAP_POSE";
        case MATCH_STATE_SOLVING: return "SOLVING";
        case MATCH_STATE_MISSION_ARMING: return "MISSION_ARMING";
        case MATCH_STATE_MISSION_RETRY_WAIT: return "MISSION_RETRY_WAIT";
        case MATCH_STATE_MISSION_RUNNING: return "MISSION_RUNNING";
        case MATCH_STATE_FINISH_SCAN: return "FINISH_MULTI_MAP";
        case MATCH_STATE_RETURN_PREP: return "RETURN_PREP";
        case MATCH_STATE_RETURN_DIRECT: return "RETURN_DIRECT";
        case MATCH_STATE_RETURN_PATH: return "RETURN_PATH";
        case MATCH_STATE_ROUND_DONE: return "ROUND_DONE";
        case MATCH_STATE_BETWEEN_ROUNDS: return "BETWEEN_ROUNDS";
        case MATCH_STATE_WAIT_OPERATOR: return "WAIT_OPERATOR";
        case MATCH_STATE_COMPLETE: return "MATCH_COMPLETE";
        case MATCH_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char *match_fault_name(match_fault_t fault)
{
    switch(fault)
    {
        case MATCH_FAULT_NONE: return "NONE";
        case MATCH_FAULT_BAD_STATE: return "BAD_STATE";
        case MATCH_FAULT_POINT_START: return "POINT_START";
        case MATCH_FAULT_BASE_EXIT: return "BASE_EXIT";
        case MATCH_FAULT_MAP_TIMEOUT: return "MAP_TIMEOUT";
        case MATCH_FAULT_MAP_INVALID: return "MAP_INVALID";
        case MATCH_FAULT_SOLVER: return "SOLVER";
        case MATCH_FAULT_MISSION_ARM: return "MISSION_ARM";
        case MATCH_FAULT_MISSION: return "MISSION";
        case MATCH_FAULT_FINISH_SCAN: return "FINISH_SCAN";
        case MATCH_FAULT_RETURN_NO_POSE: return "RETURN_NO_POSE";
        case MATCH_FAULT_RETURN_NO_PATH: return "RETURN_NO_PATH";
        case MATCH_FAULT_RETURN_MOVE: return "RETURN_MOVE";
        case MATCH_FAULT_OPERATOR_STOP: return "OPERATOR_STOP";
        default: return "UNKNOWN";
    }
}

const char *match_kind_name(match_kind_t kind)
{
    if(kind == MATCH_KIND_SINGLE_ROUND) return "SINGLE";
    if(kind == MATCH_KIND_SCORE_GUARD) return "SCORE_GUARD_3";
    return "FULL_3_ROUND";
}

const char *match_profile_name(match_profile_t profile)
{
    if(profile == MATCH_PROFILE_NORMAL) return "NORMAL";
    if(profile == MATCH_PROFILE_FAST) return "FAST";
    return "SAFE";
}

const char *match_label_policy_name(match_label_policy_t policy)
{
    if(policy == MATCH_LABEL_FORCE_UNLABELED) return "FORCE_UNLABELED";
    if(policy == MATCH_LABEL_FORCE_LABELED) return "FORCE_LABELED";
    if(policy == MATCH_LABEL_ROUND_SEQUENCE) return "ROUND1_CLASSIC_ROUND2_3_N2";
    return "AUTO";
}

const char *match_next_policy_name(match_next_policy_t policy)
{
    if(policy == MATCH_RETRY_SAME) return "RETRY_SAME";
    if(policy == MATCH_WAIT_OPERATOR) return "WAIT_OPERATOR";
    return "NEXT_MAP";
}

const char *match_return_policy_name(match_return_policy_t policy)
{
    if(policy == MATCH_RETURN_DIRECT) return "DIRECT";
    if(policy == MATCH_RETURN_PLANNED) return "PLANNED";
    return "AUTO_CLEAR_DIRECT_ELSE_PLAN";
}

const char *match_telemetry_name(match_telemetry_level_t level)
{
    if(level == MATCH_TELEMETRY_QUIET) return "QUIET";
    if(level == MATCH_TELEMETRY_ACTION) return "ACTION";
    if(level == MATCH_TELEMETRY_CONTROL) return "CONTROL";
    if(level == MATCH_TELEMETRY_FULL) return "FULL";
    return "EVENT";
}

const char *match_gate_name(match_gate_t gate)
{
    switch(gate)
    {
        case MATCH_GATE_NONE: return "NONE";
        case MATCH_GATE_PREFLIGHT: return "PREFLIGHT";
        case MATCH_GATE_MAP: return "WAIT_MAP";
        case MATCH_GATE_POSE: return "WAIT_POSE";
        case MATCH_GATE_POSITION_STABLE: return "WAIT_POS_STABLE";
        case MATCH_GATE_HEADING_STABLE: return "WAIT_HEADING_STABLE";
        case MATCH_GATE_PLAN_REFRESH: return "REFRESH_PLAN";
        case MATCH_GATE_RUNTIME_REPLAN: return "RUNTIME_REPLAN";
        case MATCH_GATE_FINISH_SCAN: return "FINISH_SCAN";
        case MATCH_GATE_RETURN_POSE: return "RETURN_POSE";
        default: return "UNKNOWN";
    }
}

const char *match_key_event_name(match_key_event_t event)
{
    switch(event)
    {
        case MATCH_KEY1_SHORT: return "KEY1_SHORT";
        case MATCH_KEY1_LONG: return "KEY1_LONG";
        case MATCH_KEY1_DOUBLE: return "KEY1_DOUBLE";
        case MATCH_KEY2_SHORT: return "KEY2_SHORT";
        case MATCH_KEY2_LONG: return "KEY2_LONG";
        case MATCH_KEY2_DOUBLE: return "KEY2_DOUBLE";
        default: return "UNKNOWN";
    }
}

const char *match_exit_direction_name(uint8 direction_index)
{
    if(direction_index == MATCH_EXIT_ROUTE_TOP) return "TOP_1_4";
    if(direction_index == MATCH_EXIT_ROUTE_RIGHT_UP) return "RIGHT_UP_2_5";
    if(direction_index == MATCH_EXIT_ROUTE_RIGHT_DOWN) return "RIGHT_DOWN_2_6";
    if(direction_index == MATCH_EXIT_ROUTE_BOTTOM) return "BOTTOM_1_7";
    return "INVALID";
}

const char *match_round_strategy_name(uint8 strategy)
{
    if(strategy == APP_RACE_STRATEGY_NORMAL) return "NORMAL";
    if(strategy == APP_RACE_STRATEGY_SPRINT) return "SPRINT";
    return "SAFE";
}

const char *match_round_algorithm_name(uint8 algorithm)
{
    if(algorithm == APP_RACE_ALGO_IMAGE_ONLY) return "IMAGE_ONLY";
    if(algorithm == APP_RACE_ALGO_BOMB_IMAGE) return "BOMB_IMAGE";
    return "CLASSIC";
}

const char *match_preset_name(uint8 preset_index)
{
    if(preset_index >= app_race_preset_count) return "CUSTOM";
    return app_race_presets[preset_index].name;
}
