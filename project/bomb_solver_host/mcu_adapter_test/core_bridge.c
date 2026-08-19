#include "bomb_solver_core.h"

extern bool run_phase1(const char raw_map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS],
                       bomb_phase_result_t *result, int mode, int skip_flag);
extern bool run_phase2(int8_t *box_ids, int8_t *goal_ids,
                       bomb_phase_result_t *result);

bool bomb_solver_run_phase1(
    const char raw_map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS],
    bomb_phase_result_t *result, int skip_observation)
{
    return run_phase1(raw_map, result, 4, skip_observation);
}

bool bomb_solver_run_phase2(const int8_t *box_ids, const int8_t *goal_ids,
                            bomb_phase_result_t *result)
{
    return run_phase2((int8_t *)box_ids, (int8_t *)goal_ids, result);
}

void bomb_solver_reset(void)
{
}
