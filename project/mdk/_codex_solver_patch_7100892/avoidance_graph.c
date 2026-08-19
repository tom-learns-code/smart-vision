#include "avoidance_graph.h"
#include <string.h>

/*
 * Avoidance graph optimization is intentionally disabled for now.
 * micro_scheduler.c falls back to car_bfs_path() whenever this module returns
 * no waypoints, so the stable grid-BFS path remains the active behavior.
 */

void find_avoidance_nodes(map_input_t *map, uint8 *obstacles, uint8 obs_count,
                          uint8 *out_nodes)
{
    (void)map;
    (void)obstacles;
    (void)obs_count;
    memset(out_nodes, 0, MAP_CELLS);
}

int check_safe_connection(uint8 ax, uint8 ay, uint8 bx, uint8 by,
                          map_input_t *map, uint8 *obstacles, uint8 obs_count,
                          uint8 *avoidance_nodes)
{
    (void)ax;
    (void)ay;
    (void)bx;
    (void)by;
    (void)map;
    (void)obstacles;
    (void)obs_count;
    (void)avoidance_nodes;
    return 0;
}

uint8 avoidance_graph_get_waypoints(
    uint8 sx, uint8 sy, uint8 tx, uint8 ty,
    map_input_t *map, uint8 *obstacles, uint8 obs_count,
    uint8 *wp_x, uint8 *wp_y, uint8 max_wp)
{
    (void)sx;
    (void)sy;
    (void)tx;
    (void)ty;
    (void)map;
    (void)obstacles;
    (void)obs_count;
    (void)wp_x;
    (void)wp_y;
    (void)max_wp;
    return 0;
}
