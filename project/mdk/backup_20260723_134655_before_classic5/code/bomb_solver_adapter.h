#ifndef __BOMB_SOLVER_ADAPTER_H
#define __BOMB_SOLVER_ADAPTER_H

#include "solver.h"

solver_status_t bomb_solver_plan_phase1(const map_input_t *map,
                                        solver_output_t *out);
solver_status_t bomb_solver_plan_phase2(const int8 box_ids[MAX_BOXES],
                                        const int8 goal_ids[MAX_BOXES],
                                        solver_output_t *out);
uint8 bomb_solver_resolve_n2(uint8 count,
                             int8 box_ids[MAX_BOXES], uint8 box_mask,
                             int8 goal_ids[MAX_BOXES], uint8 goal_mask);

#endif
