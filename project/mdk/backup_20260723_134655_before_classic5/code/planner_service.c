#include <string.h>
#include "zf_common_headfile.h"
#include "motion_control.h"
#include "vision_link.h"
#include "bomb_solver_adapter.h"
#include "planner_service.h"

static planner_info_t planner_info;
static vision_world_snapshot_t planner_world;
static solver_output_t planner_plan;

static uint32 planner_hash_byte(uint32 hash, uint8 value)
{
    return (hash ^ value) * 16777619UL;
}

static uint32 planner_map_hash(const vision_link_map_t *map)
{
    uint32 hash = 2166136261UL;
    uint8 i;

    hash = planner_hash_byte(hash, map->width);
    hash = planner_hash_byte(hash, map->height);
    for(i = 0U; i < VISION_LINK_WALL_BYTES; i++)
        hash = planner_hash_byte(hash, map->wall_bits[i]);
    hash = planner_hash_byte(hash, map->box_count);
    for(i = 0U; i < map->box_count; i++)
    {
        hash = planner_hash_byte(hash, (uint8)map->boxes[i].gx);
        hash = planner_hash_byte(hash, (uint8)map->boxes[i].gy);
    }
    hash = planner_hash_byte(hash, map->goal_count);
    for(i = 0U; i < map->goal_count; i++)
    {
        hash = planner_hash_byte(hash, (uint8)map->goals[i].gx);
        hash = planner_hash_byte(hash, (uint8)map->goals[i].gy);
    }
    hash = planner_hash_byte(hash, map->bomb_count);
    for(i = 0U; i < map->bomb_count; i++)
    {
        hash = planner_hash_byte(hash, (uint8)map->bombs[i].gx);
        hash = planner_hash_byte(hash, (uint8)map->bombs[i].gy);
    }
    return hash;
}

void planner_service_init(void)
{
    memset(&planner_info, 0, sizeof(planner_info));
    memset(&planner_world, 0, sizeof(planner_world));
    memset(&planner_plan, 0, sizeof(planner_plan));
    planner_info.status = PLANNER_STATUS_NO_PLAN;
    planner_info.world_status = VISION_WORLD_NO_MAP;
    planner_info.solver_status = SOLVER_STATUS_BAD_ARGUMENT;
}

planner_status_t planner_service_solve(void)
{
    vision_link_map_t latest_map;
    uint32 start_tick = pit_count;

    planner_info.plan_valid = 0U;
    planner_info.map_hash = 0U;
    planner_info.world_status = vision_world_capture(&planner_world);
    if(planner_info.world_status != VISION_WORLD_OK)
    {
        planner_info.status = PLANNER_STATUS_WORLD_REJECTED;
        planner_info.solve_ms = (uint32)(pit_count - start_tick) * 5U;
        return planner_info.status;
    }

    planner_info.map_version = planner_world.map_version;
    if(vision_link_get_map(&latest_map))
    {
        planner_info.map_hash = planner_map_hash(&latest_map);
    }
    planner_info.solver_status = solver_solve(&planner_world.solver_map,
                                              &planner_plan);
    planner_info.solve_ms = (uint32)(pit_count - start_tick) * 5U;
    if(planner_info.solver_status != SOLVER_STATUS_OK || !planner_plan.success)
    {
        planner_info.status = PLANNER_STATUS_SOLVER_FAILED;
        return planner_info.status;
    }

    if(!vision_link_get_map(&latest_map) ||
       planner_map_hash(&latest_map) != planner_info.map_hash)
    {
        planner_info.status = PLANNER_STATUS_MAP_CHANGED;
        return planner_info.status;
    }

    planner_info.plan_generation++;
    planner_info.plan_valid = 1U;
    planner_info.status = PLANNER_STATUS_OK;
    return planner_info.status;
}

planner_status_t planner_service_solve_phase2(
    const int8 box_ids[MAX_BOXES], const int8 goal_ids[MAX_BOXES])
{
    uint32 start_tick = pit_count;

    if(!planner_info.plan_valid ||
       planner_plan.plan_phase != SOLVER_PLAN_BOMB_P1)
    {
        planner_info.status = PLANNER_STATUS_NO_PLAN;
        return planner_info.status;
    }
    planner_info.plan_valid = 0U;
    planner_info.solver_status =
        bomb_solver_plan_phase2(box_ids, goal_ids, &planner_plan);
    planner_info.solve_ms = (uint32)(pit_count - start_tick) * 5U;
    if(planner_info.solver_status != SOLVER_STATUS_OK ||
       !planner_plan.success ||
       planner_plan.plan_phase != SOLVER_PLAN_BOMB_P2)
    {
        planner_info.status = PLANNER_STATUS_SOLVER_FAILED;
        return planner_info.status;
    }
    planner_info.plan_generation++;
    planner_info.plan_valid = 1U;
    planner_info.status = PLANNER_STATUS_OK;
    return planner_info.status;
}

const planner_info_t *planner_service_get_info(void)
{
    return &planner_info;
}

const solver_output_t *planner_service_get_plan(void)
{
    return planner_info.plan_valid ? &planner_plan : 0;
}

const vision_world_snapshot_t *planner_service_get_world(void)
{
    return &planner_world;
}

uint8 planner_service_plan_is_current(void)
{
    vision_link_map_t latest_map;

    if(!planner_info.plan_valid || !vision_link_is_online()) return 0U;
    if(!vision_link_get_map(&latest_map)) return 0U;
    return (uint8)(planner_map_hash(&latest_map) == planner_info.map_hash);
}

const char *planner_status_name(planner_status_t status)
{
    switch(status)
    {
        case PLANNER_STATUS_OK: return "OK";
        case PLANNER_STATUS_WORLD_REJECTED: return "WORLD_REJECTED";
        case PLANNER_STATUS_SOLVER_FAILED: return "SOLVER_FAILED";
        case PLANNER_STATUS_MAP_CHANGED: return "MAP_CHANGED";
        case PLANNER_STATUS_NO_PLAN: return "NO_PLAN";
        default: return "UNKNOWN";
    }
}
