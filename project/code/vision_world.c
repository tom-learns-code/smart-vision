#include <stdlib.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "vision_link.h"
#include "vision_world.h"

#define CELL_WALL  (0x01U)
#define CELL_BOX   (0x02U)
#define CELL_GOAL  (0x04U)
#define CELL_BOMB  (0x08U)

static uint8 world_cells[MAP_CELLS];

static uint8 source_wall_at(const vision_link_map_t *map, uint8 x, uint8 y)
{
    uint16 index = (uint16)y * VISION_LINK_GRID_W + x;
    return (uint8)((map->wall_bits[index >> 3] >> (index & 7U)) & 1U);
}

static uint8 cell_in_range(int8 x, int8 y, uint8 width, uint8 height)
{
    return (uint8)(x >= 0 && y >= 0 && x < (int8)width && y < (int8)height);
}

static vision_world_status_t add_cells(const vision_link_cell_t *source,
                                       uint8 count,
                                       uint8 flag,
                                       uint8 destination[MAX_BOXES][2],
                                       uint8 width,
                                       uint8 height)
{
    uint8 i;

    for(i = 0U; i < count; i++)
    {
        uint16 index;
        int8 x = source[i].gx;
        int8 y = source[i].gy;

        if(!cell_in_range(x, y, width, height))
        {
            return VISION_WORLD_CELL_OUT_OF_RANGE;
        }
        index = (uint16)y * MAP_MAX_W + (uint8)x;
        if(world_cells[index] & (CELL_WALL | CELL_BOMB))
        {
            return VISION_WORLD_CELL_OVERLAP;
        }
        if((flag == CELL_BOX && (world_cells[index] & CELL_BOX)) ||
           (flag == CELL_GOAL && (world_cells[index] & CELL_GOAL)) ||
           (flag == CELL_BOMB && (world_cells[index] & (CELL_BOX | CELL_GOAL))))
        {
            return VISION_WORLD_CELL_OVERLAP;
        }

        world_cells[index] |= flag;
        destination[i][0] = (uint8)x;
        destination[i][1] = (uint8)y;
    }
    return VISION_WORLD_OK;
}

static uint8 car_cell_blocked(const map_input_t *map, uint8 x, uint8 y)
{
    uint8 i;

    if(IS_WALL(map, x, y)) return 1U;
    for(i = 0U; i < map->box_count; i++)
    {
        if(map->boxes[i][0] == x && map->boxes[i][1] == y) return 1U;
    }
    return 0U;
}

static uint8 nearest_car_cell(map_input_t *map, int16 x10, int16 y10,
                              uint8 *out_x, uint8 *out_y, uint8 *adjusted)
{
    int16 rounded_x = (x10 >= 0) ? (int16)((x10 + 5) / 10) : 0;
    int16 rounded_y = (y10 >= 0) ? (int16)((y10 + 5) / 10) : 0;
    uint8 radius;

    if(rounded_x >= map->width) rounded_x = (int16)map->width - 1;
    if(rounded_y >= map->height) rounded_y = (int16)map->height - 1;

    for(radius = 0U; radius < (MAP_MAX_W + MAP_MAX_H); radius++)
    {
        uint8 y;
        for(y = 0U; y < map->height; y++)
        {
            uint8 x;
            for(x = 0U; x < map->width; x++)
            {
                int distance = abs((int)x - rounded_x) + abs((int)y - rounded_y);
                if(distance != radius || car_cell_blocked(map, x, y)) continue;
                *out_x = x;
                *out_y = y;
                *adjusted = (uint8)(x != rounded_x || y != rounded_y);
                return 1U;
            }
        }
    }
    return 0U;
}

vision_world_status_t vision_world_capture(vision_world_snapshot_t *out)
{
    vision_link_map_t source;
    vision_link_snapshot_t pose;
    map_input_t *map;
    vision_world_status_t status;
    uint8 x;
    uint8 y;
    uint8 i;

    if(out == 0) return VISION_WORLD_NO_MAP;
    memset(out, 0, sizeof(*out));
    memset(world_cells, 0, sizeof(world_cells));
    if(!vision_link_get_map(&source) || !source.valid) return VISION_WORLD_NO_MAP;
    if(!vision_link_is_online()) return VISION_WORLD_LINK_OFFLINE;
    vision_link_get_snapshot(&pose);
    if(!pose.pose_valid) return VISION_WORLD_POSE_INVALID;
    if(source.map_version != pose.map_version) return VISION_WORLD_VERSION_MISMATCH;
    if(source.width != MAP_MAX_W || source.height != MAP_MAX_H)
    {
        return VISION_WORLD_BAD_SIZE;
    }
    if(source.box_count > MAX_BOXES || source.goal_count > MAX_BOXES ||
       source.bomb_count > MAX_BOXES || source.box_count != source.goal_count)
    {
        return VISION_WORLD_BAD_COUNTS;
    }

    map = &out->solver_map;
    map->width = source.width;
    map->height = source.height;
    map->box_count = source.box_count;
    map->goal_count = source.goal_count;
    map->bomb_count = source.bomb_count;

    for(y = 0U; y < map->height; y++)
    {
        for(x = 0U; x < map->width; x++)
        {
            uint16 index = (uint16)y * MAP_MAX_W + x;
            if(source_wall_at(&source, x, y))
            {
                map->wall_cells[index] = 1U;
                world_cells[index] = CELL_WALL;
                out->source_wall_count++;
            }
        }
    }

    for(x = 0U; x < map->width; x++)
    {
        if(!IS_WALL(map, x, 0U) || !IS_WALL(map, x, map->height - 1U))
            return VISION_WORLD_OPEN_BOUNDARY;
    }
    for(y = 0U; y < map->height; y++)
    {
        if(!IS_WALL(map, 0U, y) || !IS_WALL(map, map->width - 1U, y))
            return VISION_WORLD_OPEN_BOUNDARY;
    }

    status = add_cells(source.goals, source.goal_count, CELL_GOAL,
                       map->goals, map->width, map->height);
    if(status != VISION_WORLD_OK) return status;
    status = add_cells(source.boxes, source.box_count, CELL_BOX,
                       map->boxes, map->width, map->height);
    if(status != VISION_WORLD_OK) return status;
    status = add_cells(source.bombs, source.bomb_count, CELL_BOMB,
                       map->bombs, map->width, map->height);
    if(status != VISION_WORLD_OK) return status;

    for(i = 0U; i < map->bomb_count; i++)
    {
        SET_WALL(map, map->bombs[i][0], map->bombs[i][1]);
    }
    for(i = 0U; i < MAP_CELLS; i++)
    {
        if(map->wall_cells[i]) out->solver_blocked_count++;
    }

    out->map_version = source.map_version;
    out->car_x10 = pose.car_x_mm;
    out->car_y10 = pose.car_y_mm;
    out->car_theta_x10 = pose.car_theta_x10;
    if(!nearest_car_cell(map, out->car_x10, out->car_y10,
                         &map->car_x, &map->car_y,
                         &out->car_cell_adjusted))
    {
        return VISION_WORLD_NO_CAR_CELL;
    }
    return VISION_WORLD_OK;
}

const char *vision_world_status_name(vision_world_status_t status)
{
    switch(status)
    {
        case VISION_WORLD_OK: return "OK";
        case VISION_WORLD_NO_MAP: return "NO_MAP";
        case VISION_WORLD_LINK_OFFLINE: return "LINK_OFFLINE";
        case VISION_WORLD_POSE_INVALID: return "POSE_INVALID";
        case VISION_WORLD_VERSION_MISMATCH: return "VERSION_MISMATCH";
        case VISION_WORLD_BAD_SIZE: return "BAD_SIZE";
        case VISION_WORLD_BAD_COUNTS: return "BAD_COUNTS";
        case VISION_WORLD_CELL_OUT_OF_RANGE: return "CELL_OUT_OF_RANGE";
        case VISION_WORLD_CELL_OVERLAP: return "CELL_OVERLAP";
        case VISION_WORLD_OPEN_BOUNDARY: return "OPEN_BOUNDARY";
        case VISION_WORLD_NO_CAR_CELL: return "NO_CAR_CELL";
        default: return "UNKNOWN";
    }
}
