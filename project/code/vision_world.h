#ifndef __VISION_WORLD_H
#define __VISION_WORLD_H

#include "solver.h"

typedef enum {
    VISION_WORLD_OK = 0,
    VISION_WORLD_NO_MAP,
    VISION_WORLD_LINK_OFFLINE,
    VISION_WORLD_POSE_INVALID,
    VISION_WORLD_VERSION_MISMATCH,
    VISION_WORLD_BAD_SIZE,
    VISION_WORLD_BAD_COUNTS,
    VISION_WORLD_CELL_OUT_OF_RANGE,
    VISION_WORLD_CELL_OVERLAP,
    VISION_WORLD_OPEN_BOUNDARY,
    VISION_WORLD_NO_CAR_CELL
} vision_world_status_t;

typedef struct {
    map_input_t solver_map;
    uint32 map_version;
    int16 car_x10;
    int16 car_y10;
    int16 car_theta_x10;
    uint16 source_wall_count;
    uint16 solver_blocked_count;
    uint8 car_cell_adjusted;
} vision_world_snapshot_t;

vision_world_status_t vision_world_capture(vision_world_snapshot_t *out);
const char *vision_world_status_name(vision_world_status_t status);

#endif
