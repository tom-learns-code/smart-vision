#include "zf_common_headfile.h"
#include "solver.h"
#include <stdlib.h>
#include <string.h>

#define HEAP_MAX        512
#define PATH_TABLE_MAX  1024
#define DIR_NONE        255

typedef struct {
    int16  cost;
    int16  steps;
    int16  turns;
    int16  est_car;
    uint8  x;
    uint8  y;
    uint8  prev_dir;
    uint16 parent_idx;
    uint8  push_dir;
} heap_node_t;

typedef struct {
    uint8  x;
    uint8  y;
    uint8  dir;
    uint16 parent;
} path_node_t;

static heap_node_t heap[HEAP_MAX];
static uint16 heap_size;
static path_node_t path_table[PATH_TABLE_MAX];
static uint16 path_count;
static int32 best_table[MAP_CELLS * 5];
static uint8 reach_visited[MAP_CELLS];
static uint16 reach_queue[MAP_CELLS];
static uint16 car_queue[MAP_CELLS];
static uint8 car_rev_x[MAX_PATH];
static uint8 car_rev_y[MAX_PATH];

static int heap_less(const heap_node_t *a, const heap_node_t *b)
{
    if(a->cost != b->cost) return a->cost < b->cost;
    if(a->steps != b->steps) return a->steps < b->steps;
    if(a->turns != b->turns) return a->turns < b->turns;
    if(a->est_car != b->est_car) return a->est_car < b->est_car;
    if(a->y != b->y) return a->y < b->y;
    if(a->x != b->x) return a->x < b->x;
    return a->prev_dir < b->prev_dir;
}

static void heap_push(heap_node_t n)
{
    if(heap_size >= HEAP_MAX) return;

    uint16 i = heap_size++;
    while(i > 0) {
        uint16 p = (i - 1) / 2;
        if(!heap_less(&n, &heap[p])) break;
        heap[i] = heap[p];
        i = p;
    }
    heap[i] = n;
}

static heap_node_t heap_pop(void)
{
    heap_node_t top = heap[0];
    heap_node_t last = heap[--heap_size];
    uint16 i = 0;

    while(heap_size > 0) {
        uint16 l = i * 2 + 1;
        uint16 r = i * 2 + 2;
        uint16 smallest;
        if(l >= heap_size) break;
        smallest = l;
        if(r < heap_size && heap_less(&heap[r], &heap[l])) smallest = r;
        if(!heap_less(&heap[smallest], &last)) break;
        heap[i] = heap[smallest];
        i = smallest;
    }

    if(heap_size > 0) heap[i] = last;
    return top;
}

static inline uint8 best_dir_index(uint8 dir)
{
    return (dir == DIR_NONE) ? 4 : dir;
}

static inline int32 best_get(uint8 x, uint8 y, uint8 dir)
{
    return best_table[((uint16)y * MAP_MAX_W + x) * 5 + best_dir_index(dir)];
}

static inline void best_set(uint8 x, uint8 y, uint8 dir, int16 steps, int16 turns)
{
    best_table[((uint16)y * MAP_MAX_W + x) * 5 + best_dir_index(dir)]
        = ((int32)steps << 16) | (uint16)turns;
}

static uint8 is_goal_cell(uint8 x, uint8 y, map_input_t *map)
{
    uint8 i;

    for(i = 0U; i < map->goal_count; i++)
    {
        if(map->goals[i][0] == x && map->goals[i][1] == y)
        {
            return 1U;
        }
    }
    return 0U;
}

static int is_deadlock(uint8 x, uint8 y, map_input_t *map)
{
    if(is_goal_cell(x, y, map)) return 0;
    if(x == 0 || x == map->width - 1 || y == 0 || y == map->height - 1)
        return 1;
    if(IS_WALL(map, x - 1, y) && IS_WALL(map, x, y - 1)) return 1;
    if(IS_WALL(map, x + 1, y) && IS_WALL(map, x, y - 1)) return 1;
    if(IS_WALL(map, x - 1, y) && IS_WALL(map, x, y + 1)) return 1;
    if(IS_WALL(map, x + 1, y) && IS_WALL(map, x, y + 1)) return 1;
    return 0;
}

static int16 car_reachable_distance(uint8 sx, uint8 sy,
                                    uint8 tx, uint8 ty,
                                    uint8 box_x, uint8 box_y,
                                    map_input_t *map,
                                    const uint8 *base_obstacles)
{
    uint16 head = 0U;
    uint16 tail = 0U;
    uint16 start;
    uint16 target;

    if(sx >= map->width || sy >= map->height ||
       tx >= map->width || ty >= map->height)
    {
        return -1;
    }
    start = (uint16)sy * MAP_MAX_W + sx;
    target = (uint16)ty * MAP_MAX_W + tx;
    if(base_obstacles[start] || base_obstacles[target] ||
       (sx == box_x && sy == box_y) ||
       (tx == box_x && ty == box_y))
    {
        return -1;
    }

    memset(reach_visited, 0, sizeof(reach_visited));
    reach_visited[start] = 1U;
    reach_queue[tail++] = start;

    while(head < tail)
    {
        uint16 cur = reach_queue[head++];
        uint8 cx = (uint8)(cur % MAP_MAX_W);
        uint8 cy = (uint8)(cur / MAP_MAX_W);
        uint8 d;

        if(cur == target)
        {
            return (int16)(reach_visited[cur] - 1U);
        }

        for(d = 0U; d < 4U; d++)
        {
            uint8 nx = (uint8)(cx + DIR_DX[d]);
            uint8 ny = (uint8)(cy + DIR_DY[d]);
            uint16 next;

            if(nx >= map->width || ny >= map->height) continue;
            if(nx == box_x && ny == box_y) continue;
            next = (uint16)ny * MAP_MAX_W + nx;
            if(base_obstacles[next] || reach_visited[next]) continue;
            reach_visited[next] = (uint8)(reach_visited[cur] + 1U);
            if(tail < MAP_CELLS) reach_queue[tail++] = next;
        }
    }
    return -1;
}

push_result_t push_box_bfs(
    uint8 box_x, uint8 box_y, uint8 goal_x, uint8 goal_y,
    map_input_t *map,
    uint8 *obstacles, uint8 obs_count,
    uint8 *blocked,
    uint8 car_x, uint8 car_y,
    map_preprocess_t *pp,
    int16 max_steps)
{
    push_result_t result;
    static uint8 obs_bm[MAP_CELLS];

    memset(&result, 0, sizeof(result));
    result.steps = -1;
    (void)pp;

    if(box_x == goal_x && box_y == goal_y) {
        result.steps = 0;
        return result;
    }

    heap_size = 0;
    path_count = 0;
    memset(best_table, 0xFF, sizeof(best_table));

    path_table[0].x = box_x;
    path_table[0].y = box_y;
    path_table[0].dir = DIR_NONE;
    path_table[0].parent = 65535;
    path_count = 1;

    {
        heap_node_t start;
        start.cost = 0;
        start.steps = 0;
        start.turns = 0;
        start.est_car = 0;
        start.x = box_x;
        start.y = box_y;
        start.prev_dir = DIR_NONE;
        start.parent_idx = 0;
        start.push_dir = DIR_NONE;
        heap_push(start);
    }

    best_set(box_x, box_y, DIR_NONE, 0, 0);

    memcpy(obs_bm, map->wall_cells, MAP_CELLS);
    for(uint8 i = 0; i < obs_count; i++) {
        uint8 ox = obstacles[i * 2];
        uint8 oy = obstacles[i * 2 + 1];
        if(ox < map->width && oy < map->height)
            obs_bm[(uint16)oy * MAP_MAX_W + ox] = 1;
    }

    while(heap_size > 0) {
        heap_node_t cur = heap_pop();
        int32 prev_best = best_get(cur.x, cur.y, cur.prev_dir);
        int16 prev_steps = (int16)(prev_best >> 16);
        int16 prev_turns = (int16)prev_best;

        if(prev_best != -1 &&
           (cur.steps > prev_steps ||
            (cur.steps == prev_steps && cur.turns > prev_turns))) {
            continue;
        }

        if(cur.x == goal_x && cur.y == goal_y) {
            uint16 idx = cur.parent_idx;
            result.steps = cur.steps;
            result.turns = cur.turns;
            result.path_len = 0;

            while(idx > 0 && result.path_len < MAX_PATH) {
                result.path[result.path_len].box_x = path_table[idx].x;
                result.path[result.path_len].box_y = path_table[idx].y;
                result.path[result.path_len].dir = path_table[idx].dir;
                result.path_len++;
                idx = path_table[idx].parent;
            }

            for(uint16 i = 0; i < result.path_len / 2; i++) {
                push_step_t tmp = result.path[i];
                result.path[i] = result.path[result.path_len - 1 - i];
                result.path[result.path_len - 1 - i] = tmp;
            }
            return result;
        }

        if(cur.steps >= max_steps) continue;

        for(uint8 d = 0; d < 4; d++) {
            uint8 new_x = (uint8)(cur.x + DIR_DX[d]);
            uint8 new_y = (uint8)(cur.y + DIR_DY[d]);
            uint8 car_tx = (uint8)(cur.x - DIR_DX[d]);
            uint8 car_ty = (uint8)(cur.y - DIR_DY[d]);
            uint16 car_tidx;
            uint16 new_idx;
            int16 delta_car;
            int16 new_turns;
            int16 ns;
            int32 existing;

            if(new_x >= map->width || new_y >= map->height) continue;
            if(car_tx >= map->width || car_ty >= map->height) continue;

            car_tidx = (uint16)car_ty * MAP_MAX_W + car_tx;
            new_idx = (uint16)new_y * MAP_MAX_W + new_x;
            if(obs_bm[car_tidx] || obs_bm[new_idx]) continue;
            if(is_deadlock(new_x, new_y, map)) continue;

            if(blocked) {
                uint16 block_key = ((uint16)cur.y * MAP_MAX_W + cur.x) * 4 + d;
                if(blocked[block_key]) continue;
            }

            new_turns = cur.turns;
            if(cur.prev_dir != DIR_NONE && cur.prev_dir != d)
                new_turns++;

            if(cur.prev_dir != DIR_NONE) {
                uint8 last_car_x = (uint8)(cur.x - DIR_DX[cur.prev_dir]);
                uint8 last_car_y = (uint8)(cur.y - DIR_DY[cur.prev_dir]);
                delta_car = car_reachable_distance(last_car_x, last_car_y,
                                                   car_tx, car_ty,
                                                   cur.x, cur.y,
                                                   map, obs_bm);
            } else {
                delta_car = car_reachable_distance(car_x, car_y,
                                                   car_tx, car_ty,
                                                   cur.x, cur.y,
                                                   map, obs_bm);
            }
            if(delta_car < 0) continue;

            existing = best_get(new_x, new_y, d);
            ns = cur.steps + 1;
            if(existing != -1 &&
               (ns > (int16)(existing >> 16) ||
                (ns == (int16)(existing >> 16) &&
                 new_turns >= (int16)existing)))
            {
                continue;
            }
            if(path_count >= PATH_TABLE_MAX) continue;
            {
                uint16 this_idx = path_count++;
                path_table[this_idx].x = new_x;
                path_table[this_idx].y = new_y;
                path_table[this_idx].dir = d;
                path_table[this_idx].parent = cur.parent_idx;

                heap_node_t next;
                best_set(new_x, new_y, d, ns, new_turns);
                next.steps = ns;
                next.turns = new_turns;
                next.est_car = cur.est_car + delta_car;
                next.cost = ns + new_turns * 2 + next.est_car;
                next.x = new_x;
                next.y = new_y;
                next.prev_dir = d;
                next.parent_idx = this_idx;
                next.push_dir = d;
                heap_push(next);
            }
        }
    }

    return result;
}

car_path_t car_bfs_path(uint8 sx, uint8 sy, uint8 tx, uint8 ty,
                        map_input_t *map, uint8 *extra_obs, uint8 obs_count)
{
    car_path_t result;
    static uint8 obs_bm[MAP_CELLS];
    static uint8 visited[MAP_CELLS];
    static uint16 dists[MAP_CELLS];
    static uint16 parent[MAP_CELLS];
    uint16 head = 0;
    uint16 tail = 0;
    uint16 tidx;
    uint8 found = 0;

    memset(&result, 0, sizeof(result));
    result.dist = -1;

    if(sx == tx && sy == ty) {
        result.dist = 0;
        result.px[0] = sx;
        result.py[0] = sy;
        result.path_len = 1;
        return result;
    }

    memcpy(obs_bm, map->wall_cells, MAP_CELLS);
    for(uint8 i = 0; i < obs_count; i++) {
        uint8 ox = extra_obs[i * 2];
        uint8 oy = extra_obs[i * 2 + 1];
        if(ox < map->width && oy < map->height)
            obs_bm[(uint16)oy * MAP_MAX_W + ox] = 1;
    }
    if(tx >= map->width || ty >= map->height) return result;
    if(obs_bm[(uint16)ty * MAP_MAX_W + tx]) return result;

    memset(visited, 0, sizeof(visited));
    memset(parent, 0xFF, sizeof(parent));
    {
        uint16 sidx = (uint16)sy * MAP_MAX_W + sx;
        visited[sidx] = 1;
        dists[sidx] = 0;
        parent[sidx] = 65535;
        car_queue[tail++] = sidx;
    }

    tidx = (uint16)ty * MAP_MAX_W + tx;

    while(head != tail) {
        uint16 cur = car_queue[head++];
        uint8 cx = (uint8)(cur % MAP_MAX_W);
        uint8 cy = (uint8)(cur / MAP_MAX_W);
        if(cur == tidx) {
            found = 1;
            break;
        }

        for(uint8 d = 0; d < 4; d++) {
            uint8 nx = (uint8)(cx + DIR_DX[d]);
            uint8 ny = (uint8)(cy + DIR_DY[d]);
            uint16 nidx;
            if(nx >= map->width || ny >= map->height) continue;
            nidx = (uint16)ny * MAP_MAX_W + nx;
            if(obs_bm[nidx] || visited[nidx]) continue;
            visited[nidx] = 1;
            dists[nidx] = dists[cur] + 1;
            parent[nidx] = cur;
            if(tail < MAP_CELLS) car_queue[tail++] = nidx;
        }
    }

    if(!found) return result;

    result.dist = (int16)dists[tidx];
    {
        uint16 idx = tidx;
        result.path_len = 0;

        while(idx != 65535 && result.path_len < MAX_PATH) {
            car_rev_x[result.path_len] = (uint8)(idx % MAP_MAX_W);
            car_rev_y[result.path_len] = (uint8)(idx / MAP_MAX_W);
            result.path_len++;
            idx = parent[idx];
        }

        for(int i = (int)result.path_len - 1; i >= 0; i--) {
            uint8 pi = (uint8)(result.path_len - 1 - (uint16)i);
            result.px[pi] = car_rev_x[i];
            result.py[pi] = car_rev_y[i];
        }
    }

    return result;
}
