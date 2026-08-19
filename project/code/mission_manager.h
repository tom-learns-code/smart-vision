#ifndef __MISSION_MANAGER_H
#define __MISSION_MANAGER_H

#include "action_follower.h"

typedef enum {
    MISSION_SAFE_IDLE = 0,
    MISSION_PLAN_READY,
    MISSION_START_POSE_WAIT,
    MISSION_START_POSE_ALIGN,
    MISSION_START_HEADING_WAIT,
    MISSION_START_HEADING_ROTATE,
    MISSION_STEP_WAIT,
    MISSION_ACTION_RUNNING,
    MISSION_POST_PUSH_REANCHOR,
    MISSION_PUSH_VERIFY,
    MISSION_FP_SESSION_WAIT,
    MISSION_OBSERVE_POSE_WAIT,
    MISSION_OBSERVE_POSE_ALIGN,
    MISSION_OBSERVE_ROTATE_OUT,
    MISSION_OBSERVE_SETTLE,
    MISSION_OBSERVE_WAIT_RESULT,
    MISSION_OBSERVE_ROTATE_BACK,
    MISSION_WAIT_ACTION,
    MISSION_PHASE2_SOLVING,
    MISSION_PAUSED,
    MISSION_COMPLETE,
    MISSION_FAULT
} mission_state_t;

typedef enum {
    MISSION_RESULT_OK = 0,
    MISSION_RESULT_NO_PLAN,
    MISSION_RESULT_PLAN_STALE,
    MISSION_RESULT_NOT_ARMED,
    MISSION_RESULT_BUSY,
    MISSION_RESULT_UNSUPPORTED_ACTION,
    MISSION_RESULT_FOLLOWER_FAULT,
    MISSION_RESULT_START_POSE_CHANGED,
    MISSION_RESULT_POSE_INVALID,
    MISSION_RESULT_REPLAN_REQUIRED,
    MISSION_RESULT_PUSH_MAP_TIMEOUT,
    MISSION_RESULT_PUSH_MAP_MISMATCH,
    MISSION_RESULT_FP_SESSION_FAILED,
    MISSION_RESULT_RECOGNITION_TIMEOUT,
    MISSION_RESULT_RECOGNITION_REJECTED,
    MISSION_RESULT_OBSERVE_ROTATION_FAILED,
    MISSION_RESULT_N2_RESOLVE_FAILED,
    MISSION_RESULT_PHASE2_FAILED,
    MISSION_RESULT_BAD_STATE
} mission_result_t;

typedef enum {
    MISSION_PROFILE_STANDARD = 0,
    MISSION_PROFILE_NORMAL,
    MISSION_PROFILE_FAST_SAFE
} mission_run_profile_t;

typedef enum {
    MISSION_SEGMENT_SINGLE = 0,
    MISSION_SEGMENT_CRUISE,
    MISSION_SEGMENT_TERMINAL,
    MISSION_SEGMENT_PUSH_CRUISE
} mission_segment_phase_t;

typedef struct {
    mission_state_t state;
    mission_result_t last_result;
    uint8 armed;
    uint8 auto_run;
    mission_run_profile_t run_profile;
    action_follower_gear_t exec_gear;
    uint8 segment_cells;
    mission_segment_phase_t segment_phase;
    uint8 segment_total_cells;
    uint8 segment_cruise_cells;
    uint8 segment_node_x;
    uint8 segment_node_y;
    uint8 context_box_near;
    uint8 context_bomb_near;
    uint8 context_object_near;
    uint8 context_goal_near;
    uint8 context_strict_position;
    uint8 context_transition;
    int8 context_wall_sign;
    int16 context_clearance_x_mm;
    int16 context_clearance_y_mm;
    uint8 map_current;
    uint8 action_cursor;
    uint8 action_count;
    uint8 substep_cursor;
    uint8 action_step_count;
    uint8 current_action_type;
    uint8 from_x;
    uint8 from_y;
    uint8 to_x;
    uint8 to_y;
    uint8 push_step;
    uint8 push_verify_pending;
    uint8 push_verify_requests;
    uint8 push_verify_bad_maps;
    uint8 push_verify_mismatch_mask;
    uint8 push_box_on_goal;
    uint8 push_box_consumed;
    uint8 push_car_box_filtered;
    uint8 push_is_bomb;
    uint8 post_push_frames;
    uint8 post_push_bad_frames;
    uint8 post_push_result;
    uint8 post_push_ramp_pending;
    uint8 logical_origin_override;
    uint8 logical_origin_x;
    uint8 logical_origin_y;
    uint8 wall_snap_accept_count;
    uint8 start_target_x;
    uint8 start_target_y;
    uint8 start_reanchor_frames;
    uint8 start_reanchor_bad_frames;
    uint8 start_reanchor_result;
    uint32 start_reanchor_age_ms;
    int16 start_reanchor_x10;
    int16 start_reanchor_y10;
    int16 start_reanchor_error_x10;
    int16 start_reanchor_error_y10;
    uint8 start_heading_frames;
    uint8 start_heading_bad_frames;
    uint8 start_heading_corrected;
    int16 start_heading_visual_x10;
    int16 start_heading_target_x10;
    int16 start_heading_correction_x10;
    int16 start_heading_spread_x10;
    uint8 box_from_x;
    uint8 box_from_y;
    uint8 box_to_x;
    uint8 box_to_y;
    uint16 step_cursor;
    uint16 step_count;
    uint32 push_map_version_before;
    uint32 push_map_version_after;
    uint32 push_verify_age_ms;
    uint32 post_push_age_ms;
    int16 post_push_x10;
    int16 post_push_y10;
    int16 post_push_error_x10;
    int16 post_push_error_y10;
    uint32 plan_generation;
    uint8 plan_phase;
    uint8 fp_state;
    uint8 fp_status;
    uint8 observe_object_type;
    uint8 observe_object_slot;
    uint8 observed_box_mask;
    uint8 observed_goal_mask;
    int8 observed_box_ids[MAX_BOXES];
    int8 observed_goal_ids[MAX_BOXES];
    uint16 observe_rotation_deg;
    uint8 observe_rotation_clockwise;
    uint32 observe_wait_ms;
    uint32 event_counter;
    action_follower_debug_t follower;
} mission_status_t;

void mission_manager_init(void);
void mission_manager_poll(void);
mission_result_t mission_manager_arm_plan(void);
mission_result_t mission_manager_arm_plan_prevalidated_pose(void);
mission_result_t mission_manager_run_next_step(void);
mission_result_t mission_manager_run_one_grid(void);
mission_result_t mission_manager_run_all_steps(void);
mission_result_t mission_manager_toggle_run_profile(void);
mission_result_t mission_manager_set_run_profile(mission_run_profile_t profile);
mission_result_t mission_manager_pause(void);
mission_result_t mission_manager_continue(void);
mission_result_t mission_manager_reset_cursor(void);
void mission_manager_disarm(void);
void mission_manager_emergency_stop(void);
uint8 mission_manager_planner_locked(void);
void mission_manager_get_status(mission_status_t *out);
uint16 mission_manager_count_plan_steps(const solver_output_t *plan);
const char *mission_state_name(mission_state_t state);
const char *mission_result_name(mission_result_t result);
const char *mission_run_profile_name(mission_run_profile_t profile);

#endif
