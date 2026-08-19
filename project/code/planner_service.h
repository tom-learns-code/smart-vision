#ifndef __PLANNER_SERVICE_H
#define __PLANNER_SERVICE_H

#include "vision_world.h"

typedef enum {
    PLANNER_STATUS_OK = 0,
    PLANNER_STATUS_WORLD_REJECTED,
    PLANNER_STATUS_SOLVER_FAILED,
    PLANNER_STATUS_MAP_CHANGED,
    PLANNER_STATUS_NO_PLAN
} planner_status_t;

typedef struct {
    planner_status_t status;
    vision_world_status_t world_status;
    solver_status_t solver_status;
    uint32 map_version;
    uint32 map_hash;
    uint32 plan_generation;
    uint32 solve_ms;
    uint8 plan_valid;
} planner_info_t;

void planner_service_init(void);
planner_status_t planner_service_solve(void);
planner_status_t planner_service_solve_phase2(
    const int8 box_ids[MAX_BOXES], const int8 goal_ids[MAX_BOXES]);
const planner_info_t *planner_service_get_info(void);
const solver_output_t *planner_service_get_plan(void);
const vision_world_snapshot_t *planner_service_get_world(void);
uint8 planner_service_plan_is_current(void);
const char *planner_status_name(planner_status_t status);

#endif
