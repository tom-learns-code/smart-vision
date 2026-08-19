#ifndef __SOLVER_H
#define __SOLVER_H

#include "zf_common_headfile.h"
#include <math.h>

#define MAP_MAX_W   16
#define MAP_MAX_H   12
#define MAP_CELLS   (MAP_MAX_W * MAP_MAX_H)
#define MAX_BOXES       5
#define N2_MAX_BOXES    3
#define MAX_ACTIONS     128
#define MAX_PATH    256
#define MAX_WP      50
#define SOLVER_GRID_SIZE_MM 200

#define DIR_UP    0
#define DIR_DOWN  1
#define DIR_LEFT  2
#define DIR_RIGHT 3

extern const int8 DIR_DX[4];
extern const int8 DIR_DY[4];

typedef struct {
    uint8  width;
    uint8  height;
    uint8  car_x;
    uint8  car_y;
    uint8  box_count;
    uint8  boxes[MAX_BOXES][2];
    uint8  goal_count;
    uint8  goals[MAX_BOXES][2];
    uint8  wall_cells[MAP_CELLS];
    uint8  bomb_count;
    uint8  bombs[MAX_BOXES][2];
} map_input_t;

#define WALL_BIT(map, x, y)  ((map)->wall_cells[(y) * MAP_MAX_W + (x)])
#define SET_WALL(map, x, y)  ((map)->wall_cells[(y) * MAP_MAX_W + (x)] = 1)
#define IS_WALL(map, x, y)   (WALL_BIT(map, x, y) != 0)

typedef struct {
    uint8  box_x;
    uint8  box_y;
    uint8  dir;
} push_step_t;

typedef struct {
    int16  steps;
    int16  turns;
    uint16 path_len;
    push_step_t path[MAX_PATH];
} push_result_t;

typedef struct {
    uint8  box_id;
    uint8  task_idx;
    uint8  box_start_x;
    uint8  box_start_y;
    uint8  box_end_x;
    uint8  box_end_y;
    uint8  push_dir;
    uint8  n_steps;
    uint8  car_target_x;
    uint8  car_target_y;
    uint8  car_end_x;
    uint8  car_end_y;
} micro_task_t;

typedef struct {
    uint8  box_id;
    uint8  task_idx;
    int16  car_dist;
} schedule_entry_t;

#define PUSH_DIR_UP    "UP"
#define PUSH_DIR_DOWN  "DOWN"
#define PUSH_DIR_LEFT  "LEFT"
#define PUSH_DIR_RIGHT "RIGHT"

typedef struct {
    uint8  box_id;
    uint8  box_start_x;
    uint8  box_start_y;
    uint8  box_target_x;
    uint8  box_target_y;
    uint8  push_dir;
    uint8  n_steps;
    uint8  car_target_x;
    uint8  car_target_y;
    uint8  wall_target_x;
    uint8  wall_target_y;
    char   push_dir_str[6];
} push_meta_t;

typedef struct {
    uint8 direction;
    uint8 object_type;
    uint8 box_index;
    uint8 goal_index;
    uint16 dwell_ms;
} observe_meta_t;

typedef struct {
    int16  x_mm;
    int16  y_mm;
} waypoint_t;

#define ACTION_FREE_MOVE  0
#define ACTION_PUSH_BOX   1
#define ACTION_PUSH_BOMB  2
#define ACTION_WAIT       3
#define ACTION_OBSERVE    4
#define ACTION_PHASE_END  5

#define OBSERVE_OBJECT_NONE  0
#define OBSERVE_OBJECT_BOX   1
#define OBSERVE_OBJECT_GOAL  2
#define OBSERVE_OBJECT_BOTH  3

typedef enum {
    SOLVER_PLAN_STANDARD = 0,
    SOLVER_PLAN_BOMB_P1,
    SOLVER_PLAN_BOMB_P2
} solver_plan_phase_t;

typedef enum {
    SOLVER_MODE_CLASSIC = 0,
    SOLVER_MODE_IMAGE_ONLY,
    SOLVER_MODE_BOMB_IMAGE
} solver_mode_t;

typedef struct {
    uint8  type;
    uint8  target_x;
    uint8  target_y;
    float  theta;
    uint8  wp_count;
    waypoint_t waypoints[MAX_WP];
    push_meta_t push_meta;
    observe_meta_t observe_meta;
    float  wait_duration;
    uint8  narrow_passage;
} action_t;

typedef enum {
    SOLVER_STATUS_OK = 0,
    SOLVER_STATUS_BAD_ARGUMENT,
    SOLVER_STATUS_BAD_MAP,
    SOLVER_STATUS_NO_ASSIGNMENT,
    SOLVER_STATUS_SCHEDULE_FAILED,
    SOLVER_STATUS_NO_SOLUTION
} solver_status_t;

typedef enum {
    SOLVER_PLAN_CHECK_NOT_RUN = 0,
    SOLVER_PLAN_CHECK_OK,
    SOLVER_PLAN_CHECK_BAD_ACTION,
    SOLVER_PLAN_CHECK_BAD_WAYPOINT,
    SOLVER_PLAN_CHECK_NON_ADJACENT,
    SOLVER_PLAN_CHECK_CAR_COLLISION,
    SOLVER_PLAN_CHECK_BAD_PUSH_META,
    SOLVER_PLAN_CHECK_BOX_COLLISION,
    SOLVER_PLAN_CHECK_EARLY_GOAL,
    SOLVER_PLAN_CHECK_GOAL_MISSING,
    SOLVER_PLAN_CHECK_BOXES_REMAIN
} solver_plan_check_t;

typedef struct {
    uint8   success;
    solver_status_t status;
    uint8   action_count;
    action_t actions[MAX_ACTIONS];
    uint8   assignment[MAX_BOXES];
    uint8   total_pushes;
    uint8   total_free_moves;
    uint16  total_waypoints;
    uint16  total_push_steps;
    int32   assignment_cost;
    uint8   used_fallback;
    uint8   plan_check_ok;
    solver_plan_check_t plan_check;
    uint8   plan_check_action;
    uint8   plan_check_step;
    uint8   plan_check_x;
    uint8   plan_check_y;
    solver_plan_phase_t plan_phase;
    uint8   requires_observation_ids;
    uint8   observed_box_mask;
    uint8   observed_goal_mask;
    uint8   required_box_observations;
    uint8   required_goal_observations;
    uint8   blast_count;
    uint8   blast_x[MAX_BOXES];
    uint8   blast_y[MAX_BOXES];
} solver_output_t;

typedef struct {
    uint8  width;
    uint8  height;
    uint8  component[MAP_CELLS];
    uint8  num_components;
    uint8  distances[MAP_CELLS][MAP_CELLS];
} map_preprocess_t;

typedef struct {
    int16  dist;
    uint16 path_len;
    uint8  px[MAX_PATH];
    uint8  py[MAX_PATH];
} car_path_t;

solver_status_t solver_solve(map_input_t *map, solver_output_t *out);
void solver_set_mode(solver_mode_t mode);
solver_mode_t solver_get_mode(void);
const char *solver_mode_name(solver_mode_t mode);
const char *solver_status_name(solver_status_t status);
const char *solver_plan_check_name(solver_plan_check_t check);
void preprocess_init(map_preprocess_t *pp, map_input_t *map);
int16 preprocess_distance(map_preprocess_t *pp, uint8 x1, uint8 y1,
                          uint8 x2, uint8 y2);

#endif
