#ifndef __MATCH_MANAGER_H
#define __MATCH_MANAGER_H

#include "zf_common_typedef.h"
#include "app_config.h"
#include "mission_manager.h"
#include "planner_service.h"

#define MATCH_MANAGER_ENABLE              (1U)
#define MATCH_BUTTON_MENU_ENABLE          (1U)
#define MATCH_SCORE_GUARD_ENABLE          (0U)
#define MATCH_RETURN_HOME_ENABLE          (1U)
#define MATCH_FINISH_MAP_RECHECK_ENABLE    (1U)
#define MATCH_FP2_AUTO_MODE_ENABLE         (0U)
#define MATCH_VISION_LAG_SHADOW_ENABLE     (0U)
#define MATCH_FAULT_INJECTION_ENABLE       (0U)

#define MATCH_EXIT_ROUTE_TOP                (0U) /* (1,5) -> (1,4) */
#define MATCH_EXIT_ROUTE_RIGHT_UP           (1U) /* (1,5) -> (2,5) */
#define MATCH_EXIT_ROUTE_RIGHT_DOWN         (2U) /* (1,6) -> (2,6) */
#define MATCH_EXIT_ROUTE_BOTTOM             (3U) /* (1,6) -> (1,7) */

#define MATCH_PREFLIGHT_IMU_READY           (0x01U)
#define MATCH_PREFLIGHT_VISION_LINK         (0x02U)
#define MATCH_PREFLIGHT_REQUIRED            (MATCH_PREFLIGHT_IMU_READY | \
                                             MATCH_PREFLIGHT_VISION_LINK)

typedef enum {
    MATCH_STATE_IDLE = 0,
    MATCH_STATE_ARMED,
    MATCH_STATE_BASE_POSE_WAIT,
    MATCH_STATE_BASE_ALIGN_RUNNING,
    MATCH_STATE_EXIT_PREP,
    MATCH_STATE_EXIT_RUNNING,
    MATCH_STATE_SKIP_RETURN,
    MATCH_STATE_MAP_WAIT,
    MATCH_STATE_POST_MAP_POSE_WAIT,
    MATCH_STATE_SOLVING,
    MATCH_STATE_MISSION_ARMING,
    MATCH_STATE_MISSION_RETRY_WAIT,
    MATCH_STATE_MISSION_RUNNING,
    MATCH_STATE_FINISH_SCAN,
    MATCH_STATE_RETURN_PREP,
    MATCH_STATE_RETURN_DIRECT,
    MATCH_STATE_RETURN_PATH,
    MATCH_STATE_ROUND_DONE,
    MATCH_STATE_BETWEEN_ROUNDS,
    MATCH_STATE_WAIT_OPERATOR,
    MATCH_STATE_COMPLETE,
    MATCH_STATE_FAULT
} match_state_t;

typedef enum {
    MATCH_KIND_SINGLE_ROUND = 0,
    MATCH_KIND_FULL_THREE_ROUNDS,
    MATCH_KIND_SCORE_GUARD
} match_kind_t;

typedef enum {
    MATCH_PROFILE_SAFE = 0,
    MATCH_PROFILE_NORMAL,
    MATCH_PROFILE_FAST
} match_profile_t;

typedef enum {
    MATCH_LABEL_AUTO = 0,
    MATCH_LABEL_FORCE_UNLABELED,
    MATCH_LABEL_FORCE_LABELED,
    MATCH_LABEL_ROUND_SEQUENCE
} match_label_policy_t;

typedef enum {
    MATCH_NEXT_MAP = 0,
    MATCH_RETRY_SAME,
    MATCH_WAIT_OPERATOR
} match_next_policy_t;

typedef enum {
    MATCH_RETURN_AUTO = 0,
    MATCH_RETURN_DIRECT,
    MATCH_RETURN_PLANNED
} match_return_policy_t;

typedef enum {
    MATCH_TELEMETRY_QUIET = 0,
    MATCH_TELEMETRY_EVENT,
    MATCH_TELEMETRY_ACTION,
    MATCH_TELEMETRY_CONTROL,
    MATCH_TELEMETRY_FULL
} match_telemetry_level_t;

typedef enum {
    MATCH_GATE_NONE = 0,
    MATCH_GATE_PREFLIGHT,
    MATCH_GATE_MAP,
    MATCH_GATE_POSE,
    MATCH_GATE_POSITION_STABLE,
    MATCH_GATE_HEADING_STABLE,
    MATCH_GATE_PLAN_REFRESH,
    MATCH_GATE_RUNTIME_REPLAN,
    MATCH_GATE_FINISH_SCAN,
    MATCH_GATE_RETURN_POSE
} match_gate_t;

typedef enum {
    MATCH_FAULT_NONE = 0,
    MATCH_FAULT_BAD_STATE,
    MATCH_FAULT_POINT_START,
    MATCH_FAULT_BASE_EXIT,
    MATCH_FAULT_MAP_TIMEOUT,
    MATCH_FAULT_MAP_INVALID,
    MATCH_FAULT_SOLVER,
    MATCH_FAULT_MISSION_ARM,
    MATCH_FAULT_MISSION,
    MATCH_FAULT_FINISH_SCAN,
    MATCH_FAULT_RETURN_NO_POSE,
    MATCH_FAULT_RETURN_NO_PATH,
    MATCH_FAULT_RETURN_MOVE,
    MATCH_FAULT_OPERATOR_STOP
} match_fault_t;

typedef enum {
    MATCH_KEY1_SHORT = 0,
    MATCH_KEY1_LONG,
    MATCH_KEY1_DOUBLE,
    MATCH_KEY2_SHORT,
    MATCH_KEY2_LONG,
    MATCH_KEY2_DOUBLE
} match_key_event_t;

typedef struct {
    uint8 base_target_x;
    uint8 base_target_y;
    uint8 base_alternate_y;
    uint8 exit_direction_index;
    uint16 exit_distance_mm;
    uint16 exit_speed;
    uint16 mission_speed;
    uint16 map_wait_ms;
    uint16 between_round_ms;
    uint8 finish_scan_samples;
    uint8 return_max_retries;
    match_return_policy_t return_policy;
} match_config_t;

typedef struct {
    match_state_t state;
    match_fault_t fault;
    match_kind_t kind;
    match_profile_t profile;
    match_label_policy_t label_policy;
    match_next_policy_t next_policy;
    match_telemetry_level_t telemetry_level;
    match_config_t config;
    app_race_round_config_t round_config[APP_RACE_ROUND_COUNT];
    uint8 preset_index;
    uint8 config_round_cursor;
    uint8 current_round_run;
    uint8 current_round_strategy;
    uint16 current_round_speed;
    uint8 current_round_algorithm;
    uint8 armed;
    uint8 running;
    uint8 remote_start;
    uint8 skip_base_once;
    uint8 preflight_mask;
    uint8 round_index;
    uint8 round_target;
    uint8 rounds_completed;
    uint8 round_success;
    uint8 map_clear;
    uint8 map_loaded;
    uint8 auto_label_degraded;
    uint8 finish_samples;
    uint8 finish_clear_samples;
    uint8 finish_remaining_samples;
    uint8 return_path_length;
    uint8 return_path_index;
    uint8 return_retries;
    uint8 mission_arm_retries;
    uint8 mission_runtime_replans;
    uint8 mission_arm_stable_frames;
    uint8 gate_wait_cycles;
    uint8 planner_reject_streak;
    uint8 initial_counts_valid;
    uint8 base_anchor_x;
    uint8 base_anchor_y;
    uint8 base_exit_x;
    uint8 base_exit_y;
    uint8 base_align_attempts;
    int16 base_error_x10;
    int16 base_error_y10;
    uint8 round_heading_valid;
    int16 round_imu_heading_x10;
    int16 round_map_heading_x10;
    int16 round_visual_heading_x10;
    uint8 selected_solver_mode;
    uint8 last_key_event;
    uint8 fault_injection;
    uint16 initial_box_count;
    uint16 initial_goal_count;
    uint16 initial_bomb_count;
    uint32 map_version_before;
    uint32 map_version_current;
    uint32 state_elapsed_ms;
    uint32 match_elapsed_ms;
    uint32 round_elapsed_ms;
    uint32 event_counter;
    uint32 map_request_count;
    planner_status_t planner_status;
    mission_state_t mission_state;
    mission_result_t mission_result;
    mission_result_t recovery_reason;
    match_gate_t gate;
} match_status_t;

void match_manager_init(void);
void match_manager_poll(void);
uint8 match_manager_select_kind(match_kind_t kind);
uint8 match_manager_arm(void);
uint8 match_manager_arm_loaded_map_debug(void);
uint8 match_manager_start_remote(void);
uint8 match_manager_continue_next_round(void);
uint8 match_manager_withdraw_round(void);
void match_manager_cancel(void);
void match_manager_emergency_stop(void);
uint8 match_manager_is_active(void);

uint8 match_manager_cycle_profile(void);
uint8 match_manager_cycle_label_policy(void);
uint8 match_manager_cycle_next_policy(void);
uint8 match_manager_cycle_return_policy(void);
uint8 match_manager_cycle_exit_distance(void);
uint8 match_manager_cycle_exit_direction(void);
uint8 match_manager_set_exit_direction(uint8 direction_index);
uint8 match_manager_cycle_base_lane(void);
uint8 match_manager_cycle_finish_samples(void);
uint8 match_manager_cycle_map_wait(void);
uint8 match_manager_cycle_telemetry(void);
uint8 match_manager_set_speed(uint16 speed);
uint8 match_manager_select_preset(uint8 preset_index);
uint8 match_manager_cycle_preset(void);
uint8 match_manager_select_config_round(uint8 round_index);
uint8 match_manager_cycle_config_round(void);
uint8 match_manager_toggle_round_run(void);
uint8 match_manager_cycle_round_strategy(void);
uint8 match_manager_cycle_round_speed(void);
uint8 match_manager_cycle_round_algorithm(void);
uint8 match_manager_handle_key(match_key_event_t event);
uint8 match_manager_inject_fault(uint8 fault_code);

void match_manager_get_status(match_status_t *out);
const char *match_state_name(match_state_t state);
const char *match_fault_name(match_fault_t fault);
const char *match_kind_name(match_kind_t kind);
const char *match_profile_name(match_profile_t profile);
const char *match_label_policy_name(match_label_policy_t policy);
const char *match_next_policy_name(match_next_policy_t policy);
const char *match_return_policy_name(match_return_policy_t policy);
const char *match_telemetry_name(match_telemetry_level_t level);
const char *match_gate_name(match_gate_t gate);
const char *match_key_event_name(match_key_event_t event);
const char *match_exit_direction_name(uint8 direction_index);
const char *match_round_strategy_name(uint8 strategy);
const char *match_round_algorithm_name(uint8 algorithm);
const char *match_preset_name(uint8 preset_index);

#endif
