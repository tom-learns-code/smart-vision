#ifndef BOMB_TRACK_H
#define BOMB_TRACK_H

#include <stdbool.h>
#include <stdint.h>

#define BOMB_TRACK_ROWS       12
#define BOMB_TRACK_COLS       16
#define BOMB_TRACK_MAX_ITEMS  3
#define BOMB_TRACK_MAX_STEPS  400

typedef struct {
    uint8_t r;
    uint8_t c;
    uint8_t mask;
    uint8_t bombs[BOMB_TRACK_MAX_ITEMS];
    uint8_t boxes[BOMB_TRACK_MAX_ITEMS];
} BombRawState;

typedef struct {
    int8_t path[BOMB_TRACK_MAX_STEPS];
    BombRawState states[BOMB_TRACK_MAX_STEPS + 1];
    int path_length;
    int end_px;
    int end_py;
    uint8_t bomb_count;
    uint8_t box_count;
    uint8_t goal_count;
    uint8_t observed_box_mask;
    uint8_t observed_goal_mask;
    uint8_t blast_x[BOMB_TRACK_MAX_ITEMS];
    uint8_t blast_y[BOMB_TRACK_MAX_ITEMS];
    uint8_t assignment[BOMB_TRACK_MAX_ITEMS];
    char updated_map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS];
} PhaseResult;

bool run_phase1(const char raw_map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS],
                PhaseResult *p1_res, int mode, int skip_flag);
bool run_phase2(int8_t *box_ids, int8_t *target_ids, PhaseResult *p2_res);

#endif
