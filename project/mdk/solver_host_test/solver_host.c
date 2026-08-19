#include "solver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int load_map(const char *path, map_input_t *map)
{
    FILE *fp = fopen(path, "rb");
    char line[128];
    uint8 y = 0U;

    if(fp == NULL) return 0;
    memset(map, 0, sizeof(*map));
    while(fgets(line, sizeof(line), fp) != NULL) {
        size_t len = strcspn(line, "\r\n");
        if(len == 0U) continue;
        if(len > MAP_MAX_W || y >= MAP_MAX_H) {
            fclose(fp);
            return 0;
        }
        if(map->width == 0U) map->width = (uint8)len;
        if(len != map->width) {
            fclose(fp);
            return 0;
        }
        for(uint8 x = 0U; x < map->width; x++) {
            char c = line[x];
            if(c == '#') {
                SET_WALL(map, x, y);
            } else if(c == '$' && map->box_count < MAX_BOXES) {
                map->boxes[map->box_count][0] = x;
                map->boxes[map->box_count][1] = y;
                map->box_count++;
            } else if(c == '.' && map->goal_count < MAX_BOXES) {
                map->goals[map->goal_count][0] = x;
                map->goals[map->goal_count][1] = y;
                map->goal_count++;
            } else if(c == '@') {
                map->car_x = x;
                map->car_y = y;
            }
        }
        y++;
    }
    fclose(fp);
    map->height = y;
    return map->width > 0U && map->height > 0U;
}

int main(int argc, char **argv)
{
    map_input_t map;
    solver_output_t output;

    if(argc != 2 && argc != 4) {
        fprintf(stderr, "usage: solver_host MAP [CAR_X CAR_Y]\n");
        return 2;
    }
    if(!load_map(argv[1], &map)) {
        fprintf(stderr, "map load failed: %s\n", argv[1]);
        return 3;
    }
    if(argc == 4) {
        map.car_x = (uint8)atoi(argv[2]);
        map.car_y = (uint8)atoi(argv[3]);
    }

    (void)solver_solve(&map, &output);
    printf("RESULT success=%u status=%s check=%s fallback=%u actions=%u boxes=%u goals=%u car=%u,%u\n",
           output.success, solver_status_name(output.status),
           solver_plan_check_name(output.plan_check), output.used_fallback,
           output.action_count, map.box_count, map.goal_count,
           map.car_x, map.car_y);
    printf("ASSIGN");
    for(uint8 i = 0U; i < map.box_count; i++) printf(" %u", output.assignment[i]);
    printf("\n");

    for(uint8 ai = 0U; ai < output.action_count; ai++) {
        action_t *action = &output.actions[ai];
        if(action->type == ACTION_FREE_MOVE) {
            printf("FREE %u %u %u", action->target_x, action->target_y,
                   action->wp_count);
            for(uint8 wi = 0U; wi < action->wp_count; wi++) {
                printf(" %d,%d",
                       action->waypoints[wi].x_mm / SOLVER_GRID_SIZE_MM,
                       action->waypoints[wi].y_mm / SOLVER_GRID_SIZE_MM);
            }
            printf("\n");
        } else if(action->type == ACTION_PUSH_BOX) {
            push_meta_t *push = &action->push_meta;
            printf("PUSH %u %u,%u %u,%u %u %u %u,%u\n",
                   push->box_id, push->box_start_x, push->box_start_y,
                   push->box_target_x, push->box_target_y,
                   push->push_dir, push->n_steps,
                   push->car_target_x, push->car_target_y);
        } else {
            printf("OTHER %u\n", action->type);
        }
    }
    return output.success ? 0 : 1;
}
