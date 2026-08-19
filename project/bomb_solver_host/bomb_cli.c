#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bomb_track.h"

static int read_map(const char *path,
                    char map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS])
{
    FILE *fp;
    char line[128];
    int row = 0;

    if(fopen_s(&fp, path, "rb") != 0 || fp == NULL) return 0;
    while(row < BOMB_TRACK_ROWS && fgets(line, sizeof(line), fp) != NULL)
    {
        size_t len = strcspn(line, "\r\n");
        if(len == 0U) continue;
        if(len != BOMB_TRACK_COLS)
        {
            fclose(fp);
            return 0;
        }
        memcpy(map[row], line, BOMB_TRACK_COLS);
        row++;
    }
    fclose(fp);
    return row == BOMB_TRACK_ROWS;
}

static void print_path(const char *name, const PhaseResult *result)
{
    int i;
    printf("%s_LEN %d\n", name, result->path_length);
    printf("%s_PATH", name);
    for(i = 0; i < result->path_length; i++) printf(" %d", result->path[i]);
    printf("\n");
}

int main(int argc, char **argv)
{
    char map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS];
    PhaseResult p1;
    PhaseResult p2;
    int8_t box_ids[BOMB_TRACK_MAX_ITEMS] = {1, 2, 3};
    int8_t goal_ids[BOMB_TRACK_MAX_ITEMS] = {1, 2, 3};
    int i;

    if(argc < 2)
    {
        fprintf(stderr, "usage: bomb_cli MAP [box ids...] [goal ids...]\n");
        return 2;
    }
    if(!read_map(argv[1], map))
    {
        fprintf(stderr, "bad map: %s\n", argv[1]);
        return 2;
    }
    memset(&p1, 0, sizeof(p1));
    memset(&p2, 0, sizeof(p2));
    if(!run_phase1(map, &p1, 4, 0))
    {
        printf("BOMB_RESULT P1_FAIL\n");
        return 1;
    }

    for(i = 0; i < p1.box_count && 2 + i < argc; i++)
        box_ids[i] = (int8_t)atoi(argv[2 + i]);
    for(i = 0; i < p1.goal_count && 2 + p1.box_count + i < argc; i++)
        goal_ids[i] = (int8_t)atoi(argv[2 + p1.box_count + i]);

    printf("BOMB_RESULT P1_OK bombs=%u boxes=%u goals=%u obs_box=%u obs_goal=%u end=%d,%d\n",
           p1.bomb_count, p1.box_count, p1.goal_count,
           p1.observed_box_mask, p1.observed_goal_mask,
           p1.end_px, p1.end_py);
    printf("P1_BLAST");
    for(i = 0; i < p1.bomb_count; i++)
        printf(" %u,%u", p1.blast_x[i], p1.blast_y[i]);
    printf("\n");
    print_path("P1", &p1);
    for(i = 0; i <= p1.path_length; i++)
    {
        const BombRawState *s = &p1.states[i];
        printf("P1_STATE %d %u %u %u %u %u %u %u %u %u\n",
               i, s->r, s->c, s->mask,
               s->bombs[0], s->bombs[1], s->bombs[2],
               s->boxes[0], s->boxes[1], s->boxes[2]);
    }

    if(!run_phase2(box_ids, goal_ids, &p2))
    {
        printf("BOMB_RESULT P2_FAIL\n");
        return 1;
    }
    printf("BOMB_RESULT P2_OK end=%d,%d\n", p2.end_px, p2.end_py);
    print_path("P2", &p2);
    return 0;
}
