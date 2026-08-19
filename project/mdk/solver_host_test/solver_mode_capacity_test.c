#include "solver.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
    map_input_t map;
    solver_output_t output;
    solver_status_t status;

    memset(&map, 0, sizeof(map));
    map.width = MAP_MAX_W;
    map.height = MAP_MAX_H;
    map.box_count = 4U;
    map.goal_count = 4U;

    solver_set_mode(SOLVER_MODE_IMAGE_ONLY);
    status = solver_solve(&map, &output);
    if(status != SOLVER_STATUS_BAD_MAP) {
        printf("FAIL image4 status=%s\n", solver_status_name(status));
        return 1;
    }

    map.bomb_count = 4U;
    solver_set_mode(SOLVER_MODE_BOMB_IMAGE);
    status = solver_solve(&map, &output);
    if(status != SOLVER_STATUS_BAD_MAP) {
        printf("FAIL bomb4 status=%s\n", solver_status_name(status));
        return 1;
    }

    printf("PASS image_max=%u bomb_image_max=%u classic_max=%u\n",
           N2_MAX_BOXES, N2_MAX_BOXES, MAX_BOXES);
    return 0;
}
