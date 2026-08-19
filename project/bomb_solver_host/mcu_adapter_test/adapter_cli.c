#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bomb_solver_adapter.h"

static int load_map(const char *path, map_input_t *map)
{
    FILE *fp = NULL;
    char line[128];
    int y = 0;
    memset(map, 0, sizeof(*map));
    map->width = MAP_MAX_W;
    map->height = MAP_MAX_H;
    if(fopen_s(&fp, path, "rb") != 0 || fp == NULL) return 0;
    while(y < MAP_MAX_H && fgets(line, sizeof(line), fp) != NULL) {
        int x;
        size_t len = strcspn(line, "\r\n");
        if(len == 0U) continue;
        if(len != MAP_MAX_W) { fclose(fp); return 0; }
        for(x = 0; x < MAP_MAX_W; x++) {
            char ch = line[x];
            if(ch == '#') SET_WALL(map, x, y);
            else if(ch == '@' || ch == '+') {
                map->car_x = (uint8)x;
                map->car_y = (uint8)y;
            }
            if(ch == '.' || ch == '+') {
                map->goals[map->goal_count][0] = (uint8)x;
                map->goals[map->goal_count++][1] = (uint8)y;
            } else if(ch == '$') {
                map->boxes[map->box_count][0] = (uint8)x;
                map->boxes[map->box_count++][1] = (uint8)y;
            } else if(ch == '*') {
                map->bombs[map->bomb_count][0] = (uint8)x;
                map->bombs[map->bomb_count++][1] = (uint8)y;
                SET_WALL(map, x, y);
            }
        }
        y++;
    }
    fclose(fp);
    return y == MAP_MAX_H;
}

static int validate_actions(const solver_output_t *out,
                            uint8 start_x, uint8 start_y)
{
    uint8 x = start_x;
    uint8 y = start_y;
    uint8 ai;
    for(ai = 0U; ai < out->action_count; ai++) {
        const action_t *action = &out->actions[ai];
        if(action->type == ACTION_FREE_MOVE) {
            uint8 wi;
            if(action->wp_count < 2U ||
               action->waypoints[0].x_mm / SOLVER_GRID_SIZE_MM != x ||
               action->waypoints[0].y_mm / SOLVER_GRID_SIZE_MM != y) return 0;
            for(wi = 1U; wi < action->wp_count; wi++) {
                uint8 nx = (uint8)(action->waypoints[wi].x_mm /
                                   SOLVER_GRID_SIZE_MM);
                uint8 ny = (uint8)(action->waypoints[wi].y_mm /
                                   SOLVER_GRID_SIZE_MM);
                if(abs((int)nx - x) + abs((int)ny - y) != 1) return 0;
                x = nx;
                y = ny;
            }
        } else if(action->type == ACTION_PUSH_BOX ||
                  action->type == ACTION_PUSH_BOMB) {
            int dx;
            int dy;
            if(action->push_meta.car_target_x != x ||
               action->push_meta.car_target_y != y ||
               action->push_meta.push_dir > DIR_RIGHT ||
               action->push_meta.n_steps == 0U) return 0;
            dx = action->push_meta.push_dir == DIR_LEFT ? -1 :
                 (action->push_meta.push_dir == DIR_RIGHT ? 1 : 0);
            dy = action->push_meta.push_dir == DIR_UP ? -1 :
                 (action->push_meta.push_dir == DIR_DOWN ? 1 : 0);
            if((int)action->target_x !=
                   (int)x + dx * action->push_meta.n_steps ||
               (int)action->target_y !=
                   (int)y + dy * action->push_meta.n_steps ||
               (int)action->push_meta.box_target_x !=
                   (int)action->push_meta.box_start_x +
                       dx * action->push_meta.n_steps ||
               (int)action->push_meta.box_target_y !=
                   (int)action->push_meta.box_start_y +
                       dy * action->push_meta.n_steps) return 0;
            x = action->target_x;
            y = action->target_y;
        } else if(action->type == ACTION_WAIT ||
                  action->type == ACTION_OBSERVE ||
                  action->type == ACTION_PHASE_END) {
            if(action->target_x != x || action->target_y != y) return 0;
        } else return 0;
    }
    return 1;
}

static int validate_n2_resolver(void)
{
    uint8 count;
    for(count = 1U; count <= MAX_BOXES; count++) {
        uint8 missing_box;
        for(missing_box = 0U; missing_box < count; missing_box++) {
            uint8 missing_goal;
            for(missing_goal = 0U; missing_goal < count; missing_goal++) {
                int8 box_ids[MAX_BOXES] = {-1, -1, -1};
                int8 goal_ids[MAX_BOXES] = {-1, -1, -1};
                uint8 valid_mask = (uint8)((1U << count) - 1U);
                uint8 box_mask = (uint8)(valid_mask &
                    (uint8)~(1U << missing_box));
                uint8 goal_mask = (uint8)(valid_mask &
                    (uint8)~(1U << missing_goal));
                uint8 i;
                for(i = 0U; i < count; i++) {
                    if(i != missing_box) box_ids[i] = (int8)i;
                    if(i != missing_goal) goal_ids[i] = (int8)i;
                }
                if(!bomb_solver_resolve_n2(count, box_ids, box_mask,
                                           goal_ids, goal_mask) ||
                   box_ids[missing_box] != (int8)missing_box ||
                   goal_ids[missing_goal] != (int8)missing_goal) return 0;
            }
        }
    }
    {
        int8 duplicate_box[MAX_BOXES] = {0, 0, -1};
        int8 valid_goal[MAX_BOXES] = {0, 1, -1};
        if(bomb_solver_resolve_n2(3U, duplicate_box, 0x03U,
                                  valid_goal, 0x03U)) return 0;
    }
    return 1;
}

int main(int argc, char **argv)
{
    map_input_t map;
    solver_output_t p1;
    solver_output_t p2;
    int8 box_ids[MAX_BOXES] = {0, 1, 2};
    static const uint8 permutations[6][3] = {
        {0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0}
    };
    uint8 permutation_count = 0U;
    uint8 max_p2_actions = 0U;
    uint8 p1_multi_pushes = 0U;
    uint8 p2_multi_pushes = 0U;
    uint8 max_push_cells = 1U;
    uint8 pi;
    if(argc != 2 || !load_map(argv[1], &map)) return 2;
    if(!validate_n2_resolver()) {
        printf("ADAPTER_FAIL N2\n");
        return 1;
    }
    if(bomb_solver_plan_phase1(&map, &p1) != SOLVER_STATUS_OK ||
       !validate_actions(&p1, map.car_x, map.car_y)) {
        printf("ADAPTER_FAIL P1\n");
        return 1;
    }
    for(pi = 0U; pi < p1.action_count; pi++) {
        const action_t *action = &p1.actions[pi];
        if((action->type == ACTION_PUSH_BOX ||
            action->type == ACTION_PUSH_BOMB) &&
           action->push_meta.n_steps > 1U) {
            p1_multi_pushes++;
            if(action->push_meta.n_steps > max_push_cells)
                max_push_cells = action->push_meta.n_steps;
        }
    }
    for(pi = 0U; pi < 6U; pi++) {
        int8 goal_ids[MAX_BOXES] = {0, 1, 2};
        uint8 j;
        uint8 valid = 1U;
        for(j = 0U; j < map.goal_count; j++) {
            if(permutations[pi][j] >= map.goal_count) valid = 0U;
            goal_ids[j] = (int8)permutations[pi][j];
        }
        if(!valid) continue;
        if(bomb_solver_plan_phase2(box_ids, goal_ids, &p2) != SOLVER_STATUS_OK ||
           !validate_actions(&p2,
                             p1.actions[p1.action_count - 1U].target_x,
                             p1.actions[p1.action_count - 1U].target_y)) {
            printf("ADAPTER_FAIL P2 permutation=%u\n", pi);
            return 1;
        }
        if(p2.action_count > max_p2_actions) max_p2_actions = p2.action_count;
        for(j = 0U; j < p2.action_count; j++) {
            const action_t *action = &p2.actions[j];
            if((action->type == ACTION_PUSH_BOX ||
                action->type == ACTION_PUSH_BOMB) &&
               action->push_meta.n_steps > 1U) {
                p2_multi_pushes++;
                if(action->push_meta.n_steps > max_push_cells)
                    max_push_cells = action->push_meta.n_steps;
            }
        }
        permutation_count++;
    }
    printf("ADAPTER_OK p1_actions=%u p2_actions_max=%u permutations=%u p1_steps=%u p2_steps_last=%u multi_push=%u/%u max_push_cells=%u\n",
           p1.action_count, max_p2_actions, permutation_count,
           p1.total_waypoints + p1.total_push_steps,
           p2.total_waypoints + p2.total_push_steps,
           p1_multi_pushes, p2_multi_pushes, max_push_cells);
    return 0;
}
