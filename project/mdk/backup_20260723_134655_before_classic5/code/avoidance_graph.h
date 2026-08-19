#ifndef AVOIDANCE_GRAPH_H
#define AVOIDANCE_GRAPH_H

#include "zf_common_headfile.h"
#include "solver.h"

void find_avoidance_nodes(map_input_t *map, uint8 *obstacles, uint8 obs_count,
                          uint8 *out_nodes);
int check_safe_connection(uint8 ax, uint8 ay, uint8 bx, uint8 by,
                          map_input_t *map, uint8 *obstacles, uint8 obs_count,
                          uint8 *avoidance_nodes);
uint8 avoidance_graph_get_waypoints(
    uint8 sx, uint8 sy, uint8 tx, uint8 ty,
    map_input_t *map, uint8 *obstacles, uint8 obs_count,
    uint8 *wp_x, uint8 *wp_y, uint8 max_wp);

#endif
