#ifndef __ACTION_FOLLOWER_H
#define __ACTION_FOLLOWER_H

#include "solver.h"

typedef enum {
    ACTION_FOLLOWER_IDLE = 0,
    ACTION_FOLLOWER_RUNNING,
    ACTION_FOLLOWER_PAUSED,
    ACTION_FOLLOWER_DONE,
    ACTION_FOLLOWER_REPLAN,
    ACTION_FOLLOWER_FAULT
} action_follower_state_t;

typedef enum {
    ACTION_GEAR_STANDARD = 0,
    ACTION_GEAR_OPEN_FAST,
    ACTION_GEAR_SINGLE_WALL_FAST,
    ACTION_GEAR_CORRIDOR_FAST,
    ACTION_GEAR_BOX_NEAR_PRECISE,
    ACTION_GEAR_PUSH_PRECISE,
    ACTION_GEAR_TRANSITION_REANCHOR
} action_follower_gear_t;

typedef struct {
    uint8 segment_cells;
    action_follower_gear_t gear;
    int16 target_offset_x_mm;
    int16 target_offset_y_mm;
    uint8 wall_axis_mask;
    uint8 corridor_axis_mask;
    int8 single_wall_correction_sign;
    uint8 interaction_locked;
    uint8 strong_reanchor;
    uint8 strict_position;
    uint16 vision_drain_ms;
    uint8 vision_stable_frames;
    uint16 vision_stable_ms;
    uint16 align_max_distance_mm;
    uint16 start_tolerance_mm;
    uint8 cruise_only;
    uint8 terminal_node;
    uint8 logical_origin_override;
    uint16 pwm_ramp_ms;
    uint16 pwm_ramp_max_delta;
} action_follower_step_context_t;

typedef enum {
    ACTION_FOLLOWER_PHASE_WAIT_ORIGIN = 0,
    ACTION_FOLLOWER_PHASE_MOVE,
    ACTION_FOLLOWER_PHASE_SETTLE,
    ACTION_FOLLOWER_PHASE_VIS_DRAIN,
    ACTION_FOLLOWER_PHASE_VIS_STABLE,
    ACTION_FOLLOWER_PHASE_WAIT_VISION,
    ACTION_FOLLOWER_PHASE_ALIGN_MOVE,
    ACTION_FOLLOWER_PHASE_ALIGN_RECHECK,
    ACTION_FOLLOWER_PHASE_WAYPOINT_HOLD = ACTION_FOLLOWER_PHASE_SETTLE
} action_follower_phase_t;

typedef enum {
    ACTION_FOLLOWER_FAULT_NONE = 0,
    ACTION_FOLLOWER_FAULT_BAD_ACTION,
    ACTION_FOLLOWER_FAULT_VISION_OFFLINE,
    ACTION_FOLLOWER_FAULT_POSE_INVALID,
    ACTION_FOLLOWER_FAULT_POSE_STALE,
    ACTION_FOLLOWER_FAULT_START_MISMATCH,
    ACTION_FOLLOWER_FAULT_STALLED,
    ACTION_FOLLOWER_FAULT_TIMEOUT,
    ACTION_FOLLOWER_FAULT_VIS_CONFIRM_TIMEOUT,
    ACTION_FOLLOWER_FAULT_GRID_MISMATCH
} action_follower_fault_t;

typedef struct {
    action_follower_state_t state;
    action_follower_fault_t fault;
    uint8 action_index;
    uint8 waypoint_index;
    uint8 waypoint_count;
    uint8 pose_valid;
    uint8 from_x;
    uint8 from_y;
    uint8 to_x;
    uint8 to_y;
    uint16 step_index;
    int16 car_x10;
    int16 car_y10;
    int16 car_theta_x10;
    int16 visual_heading_ref_x10;
    int16 physical_heading_x10;
    int16 target_x_mm;
    int16 target_y_mm;
    int16 target_offset_x_mm;
    int16 target_offset_y_mm;
    int16 dx_mm;
    int16 dy_mm;
    uint16 distance_mm;
    int16 map_direction_x10;
    int16 body_command_x10;
    int16 corrected_body_command_x10;
    int16 cross_error_raw_x10;
    int16 cross_error_filtered_x10;
    int16 cross_correction_speed_x10;
    uint8 cross_correction_active;
    uint8 cross_reject_count;
    int16 speed_command;
    int16 nominal_speed_command;
    action_follower_phase_t phase;
    uint8 next_waypoint_index;
    uint32 waypoint_hold_remaining_ms;
    uint8 visual_confirm_frames;
    uint8 visual_confirm_required;
    uint8 visual_confirm_bad_frames;
    uint8 visual_confirm_final;
    uint8 visual_fallback_used;
    uint8 dead_reckon_active;
    uint8 dead_reckon_steps;
    uint8 align_active;
    uint8 align_attempts;
    uint8 wall_axis_mask;
    uint8 corridor_axis_mask;
    int8 single_wall_correction_sign;
    uint8 cross_source;
    uint8 fast_finish;
    uint8 segment_cells;
    action_follower_gear_t gear;
    uint8 interaction_locked;
    uint8 strong_reanchor;
    uint8 strict_position;
    uint8 cruise_only;
    uint8 terminal_node;
    uint16 align_max_distance_mm;
    uint16 start_tolerance_mm;
    uint8 logical_origin_override;
    uint8 wall_snap_candidate;
    uint8 wall_snap_accepted;
    uint8 wall_snap_accept_count;
    int16 wall_snap_along_mm;
    int16 wall_snap_cross_mm;
    uint16 pwm_ramp_ms;
    uint16 pwm_ramp_max_delta;
    uint8 pose_suspect;
    uint8 pose_suspect_frames;
    uint8 pose_suspect_grid_x;
    uint8 pose_suspect_grid_y;
    uint8 push_pre_align_active;
    uint8 push_pre_align_attempts;
    uint8 push_finish_active;
    uint8 push_finish_attempts;
    int16 push_finish_along_mm;
    int16 push_finish_cross_mm;
    uint8 vision_confirm_valid;
    int16 vision_confirm_x10;
    int16 vision_confirm_y10;
    int16 align_dx_mm;
    int16 align_dy_mm;
    uint32 visual_confirm_age_ms;
    uint32 visual_confirm_pos_packets;
    uint32 position_rebase_count;
    uint32 pose_age_ms;
    uint32 elapsed_ms;
    uint32 progress_age_ms;
} action_follower_debug_t;

void action_follower_init(void);
uint8 action_follower_begin_mission_heading(void);
uint8 action_follower_begin_mission_heading_from_visual(int16 visual_heading_x10);
uint8 action_follower_set_mission_heading_frame(float imu_heading_deg,
                                                 float map_heading_deg,
                                                 int16 visual_heading_x10);
uint8 action_follower_heading_frame_valid(void);
void action_follower_end_mission_heading(void);
uint8 action_follower_get_heading_frame(float *imu_heading_deg,
                                        float *map_heading_deg);
void action_follower_set_speed(float speed);
float action_follower_get_speed(void);
uint8 action_follower_start_grid_step(uint8 action_index,
                                      uint8 substep_index,
                                      uint8 action_step_count,
                                      uint16 global_step_index,
                                      uint8 from_x,
                                      uint8 from_y,
                                      uint8 to_x,
                                      uint8 to_y,
                                      uint8 fast_finish,
                                      const action_follower_step_context_t *context);
uint8 action_follower_start_pose_reanchor(uint8 action_index,
                                          uint16 global_step_index,
                                          uint8 target_x,
                                          uint8 target_y,
                                          int16 position_x10,
                                          int16 position_y10,
                                          uint16 max_distance_mm,
                                          uint8 strict_position);
void action_follower_poll(void);
void action_follower_pause(void);
uint8 action_follower_resume(void);
void action_follower_abort(void);
action_follower_state_t action_follower_state(void);
action_follower_fault_t action_follower_fault(void);
void action_follower_get_debug(action_follower_debug_t *out);
const char *action_follower_state_name(action_follower_state_t state);
const char *action_follower_fault_name(action_follower_fault_t fault);
const char *action_follower_phase_name(action_follower_phase_t phase);
const char *action_follower_gear_name(action_follower_gear_t gear);

#endif
