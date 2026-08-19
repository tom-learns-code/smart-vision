#include "solver.h"
#include "avoidance_graph.h"
#include <string.h>

#define SCHED_HEAP_MAX       2048
#define SCHED_PATH_MAX       2048
#define MAX_TASKS_PER_BOX    8
#define SCHED_MAX_ENTRIES    48
#define LAST_DIR_NONE        4
#define BEST_HASH_SIZE       4096

extern push_result_t push_box_bfs(uint8 bx, uint8 by, uint8 gx, uint8 gy,
    map_input_t *map, uint8 *obstacles, uint8 obs_count,
    uint8 *blocked, uint8 cx, uint8 cy, map_preprocess_t *pp, int16 max_steps);
extern car_path_t car_bfs_path(uint8 sx, uint8 sy, uint8 tx, uint8 ty,
    map_input_t *map, uint8 *extra_obs, uint8 obs_count);

typedef struct {
    int32  cost;
    uint8  car_idx;
    uint8  progress[MAX_BOXES];
    uint8  last_dir;
    uint16 parent;
    uint8  box_id;
    uint8  task_idx;
} sched_node_t;

typedef struct {
    uint8  box_id;
    uint8  task_idx;
    int16  car_dist;
    uint16 parent;
} sched_path_t;

static sched_node_t sched_heap[SCHED_HEAP_MAX];
static uint16 sched_heap_size;
static sched_path_t sched_path[SCHED_PATH_MAX];
static uint16 sched_path_count;
static micro_task_t all_tasks[MAX_BOXES][MAX_TASKS_PER_BOX];
static uint8 task_counts[MAX_BOXES];
static uint32 best_keys[BEST_HASH_SIZE];
static uint16 best_vals[BEST_HASH_SIZE];
static schedule_entry_t generated_schedule[SCHED_MAX_ENTRIES];

static const char *dir_names[4] = {"UP", "DOWN", "LEFT", "RIGHT"};

static int sched_less(const sched_node_t *a, const sched_node_t *b)
{
    uint8 i;

    if(a->cost != b->cost) return a->cost < b->cost;
    if(a->car_idx != b->car_idx) return a->car_idx < b->car_idx;
    for(i = 0U; i < MAX_BOXES; i++) {
        if(a->progress[i] != b->progress[i])
            return a->progress[i] < b->progress[i];
    }
    return a->last_dir < b->last_dir;
}

static void sched_push(sched_node_t n)
{
    if(sched_heap_size >= SCHED_HEAP_MAX) return;

    uint16 i = sched_heap_size++;
    while(i > 0) {
        uint16 p = (i - 1) / 2;
        if(!sched_less(&n, &sched_heap[p])) break;
        sched_heap[i] = sched_heap[p];
        i = p;
    }
    sched_heap[i] = n;
}

static sched_node_t sched_pop(void)
{
    sched_node_t top = sched_heap[0];
    sched_node_t last = sched_heap[--sched_heap_size];
    uint16 i = 0;

    while(sched_heap_size > 0) {
        uint16 l = i * 2 + 1;
        uint16 r = i * 2 + 2;
        uint16 s;
        if(l >= sched_heap_size) break;
        s = l;
        if(r < sched_heap_size && sched_less(&sched_heap[r], &sched_heap[l])) s = r;
        if(!sched_less(&sched_heap[s], &last)) break;
        sched_heap[i] = sched_heap[s];
        i = s;
    }

    if(sched_heap_size > 0) sched_heap[i] = last;
    return top;
}

static uint8 goal_at(map_input_t *map, uint8 x, uint8 y)
{
    for(uint8 i = 0; i < map->goal_count; i++) {
        if(map->goals[i][0] == x && map->goals[i][1] == y) return 1U;
    }
    return 0U;
}

static int task_push_is_clear(
    const micro_task_t *task, uint8 bid, uint8 box_count,
    const uint8 prog[MAX_BOXES], uint8 cur_boxes[][2], map_input_t *map)
{
    uint8 bx;
    uint8 by;

    if(task->push_dir > DIR_RIGHT || task->n_steps == 0U) return 0;
    if(cur_boxes[bid][0] != task->box_start_x ||
       cur_boxes[bid][1] != task->box_start_y) return 0;

    bx = task->box_start_x;
    by = task->box_start_y;
    for(uint8 step = 0; step < task->n_steps; step++) {
        uint8 nx = (uint8)(bx + DIR_DX[task->push_dir]);
        uint8 ny = (uint8)(by + DIR_DY[task->push_dir]);

        if(nx >= map->width || ny >= map->height || IS_WALL(map, nx, ny)) return 0;
        for(uint8 j = 0; j < box_count; j++) {
            if(j == bid || prog[j] >= task_counts[j]) continue;
            if(cur_boxes[j][0] == nx && cur_boxes[j][1] == ny) return 0;
        }
        if(goal_at(map, nx, ny) && (uint8)(step + 1U) < task->n_steps) return 0;
        bx = nx;
        by = ny;
    }

    return bx == task->box_end_x && by == task->box_end_y;
}

static int add_task(uint8 bid, uint8 start_x, uint8 start_y,
                    uint8 end_x, uint8 end_y, uint8 dir, uint8 steps)
{
    if(bid >= MAX_BOXES || dir > DIR_RIGHT) return -1;
    if(task_counts[bid] >= MAX_TASKS_PER_BOX) return -1;

    {
        uint8 ti = task_counts[bid]++;
        micro_task_t *t = &all_tasks[bid][ti];
        t->box_id = bid;
        t->task_idx = ti;
        t->box_start_x = start_x;
        t->box_start_y = start_y;
        t->box_end_x = end_x;
        t->box_end_y = end_y;
        t->push_dir = dir;
        t->n_steps = steps;
        t->car_target_x = (uint8)(start_x - DIR_DX[dir]);
        t->car_target_y = (uint8)(start_y - DIR_DY[dir]);
        t->car_end_x = (uint8)(end_x - DIR_DX[dir]);
        t->car_end_y = (uint8)(end_y - DIR_DY[dir]);
    }

    return 0;
}

static int decompose_boxes(push_result_t *pushes, uint8 box_count,
                           uint8 boxes[][2])
{
    if(box_count > MAX_BOXES) return -1;
    memset(task_counts, 0, sizeof(task_counts));

    for(uint8 bid = 0; bid < box_count; bid++) {
        uint8 cur_dir;
        uint8 seg_start_x;
        uint8 seg_start_y;
        uint8 seg_end_x;
        uint8 seg_end_y;
        uint8 seg_steps = 0;

        if(pushes[bid].steps < 0) return -1;
        if(pushes[bid].path_len == 0) {
            task_counts[bid] = 0;
            continue;
        }

        cur_dir = pushes[bid].path[0].dir;
        seg_start_x = boxes[bid][0];
        seg_start_y = boxes[bid][1];
        seg_end_x = seg_start_x;
        seg_end_y = seg_start_y;

        for(uint16 i = 0; i < pushes[bid].path_len; i++) {
            push_step_t *ps = &pushes[bid].path[i];
            if(ps->dir == cur_dir) {
                seg_steps++;
                seg_end_x = ps->box_x;
                seg_end_y = ps->box_y;
            } else {
                if(seg_steps > 0 &&
                   add_task(bid, seg_start_x, seg_start_y,
                            seg_end_x, seg_end_y, cur_dir, seg_steps) < 0) {
                    return -1;
                }
                seg_start_x = seg_end_x;
                seg_start_y = seg_end_y;
                cur_dir = ps->dir;
                seg_steps = 1;
                seg_end_x = ps->box_x;
                seg_end_y = ps->box_y;
            }
        }

        if(seg_steps > 0 &&
           add_task(bid, seg_start_x, seg_start_y,
                    seg_end_x, seg_end_y, cur_dir, seg_steps) < 0) {
            return -1;
        }
    }

    return 0;
}

static uint32 best_make_key(uint8 car,
                            const uint8 progress[MAX_BOXES], uint8 ld)
{
    uint8 i;
    uint32 key = car;

    for(i = 0U; i < MAX_BOXES; i++)
        key = key * (MAX_TASKS_PER_BOX + 1U) + progress[i];
    return key * 5U + ld + 1U;
}

static uint16 best_get(uint8 car,
                       const uint8 progress[MAX_BOXES], uint8 ld)
{
    uint32 k = best_make_key(car, progress, ld);
    uint16 h = (uint16)(k % BEST_HASH_SIZE);
    for(uint16 i = 0; i < BEST_HASH_SIZE; i++) {
        uint16 idx = (uint16)((h + i) % BEST_HASH_SIZE);
        if(best_keys[idx] == k) return best_vals[idx];
        if(best_keys[idx] == 0) return 65535;
    }
    return 65535;
}

static void best_set(uint8 car, const uint8 progress[MAX_BOXES],
                     uint8 ld, uint16 val)
{
    uint32 k = best_make_key(car, progress, ld);
    uint16 h = (uint16)(k % BEST_HASH_SIZE);
    for(uint16 i = 0; i < BEST_HASH_SIZE; i++) {
        uint16 idx = (uint16)((h + i) % BEST_HASH_SIZE);
        if(best_keys[idx] == 0 || best_keys[idx] == k) {
            best_keys[idx] = k;
            best_vals[idx] = val;
            return;
        }
    }
}

static int schedule_micro_tasks(
    uint8 box_count, uint8 car_x, uint8 car_y,
    map_input_t *map, uint8 boxes[][2],
    schedule_entry_t *out_schedule, uint8 *out_len)
{
    uint8 car_idx;
    uint8 total_tasks = 0;
    uint8 zero_progress[MAX_BOXES];

    if(box_count > MAX_BOXES) return -1;
    *out_len = 0;
    memset(best_keys, 0, sizeof(best_keys));
    memset(best_vals, 0xFF, sizeof(best_vals));
    sched_heap_size = 0;
    sched_path_count = 0;
    memset(zero_progress, 0, sizeof(zero_progress));

    for(uint8 b = 0; b < box_count; b++) total_tasks += task_counts[b];
    if(total_tasks >= SCHED_MAX_ENTRIES) return -1;

    car_idx = (uint8)(car_y * MAP_MAX_W + car_x);
    best_set(car_idx, zero_progress, LAST_DIR_NONE, 0);

    {
        sched_node_t start;
        memset(&start, 0, sizeof(start));
        start.car_idx = car_idx;
        start.last_dir = LAST_DIR_NONE;
        start.parent = 65535;
        start.box_id = 255;
        start.task_idx = 255;
        sched_push(start);
    }

    sched_path[0].parent = 65535;
    sched_path_count = 1;

    while(sched_heap_size > 0) {
        sched_node_t cur = sched_pop();
        uint8 *prog = cur.progress;
        uint8 done = 1;

        if((uint16)cur.cost >
           best_get(cur.car_idx, cur.progress, cur.last_dir))
            continue;

        for(uint8 b = 0; b < box_count; b++) {
            if(prog[b] < task_counts[b]) {
                done = 0;
                break;
            }
        }

        if(done) {
            uint16 idx = cur.parent;
            uint8 pos = total_tasks;
            out_schedule[total_tasks].box_id = 255;

            while(idx != 65535 && pos > 0) {
                pos--;
                out_schedule[pos].box_id = sched_path[idx].box_id;
                out_schedule[pos].task_idx = sched_path[idx].task_idx;
                out_schedule[pos].car_dist = sched_path[idx].car_dist;
                idx = sched_path[idx].parent;
            }
            *out_len = total_tasks;
            return 0;
        }

        {
            uint8 cur_boxes[MAX_BOXES][2];
            memcpy(cur_boxes, boxes, box_count * 2U * sizeof(uint8));

            for(uint8 b = 0; b < box_count; b++) {
                if(prog[b] > 0) {
                    micro_task_t *last = &all_tasks[b][prog[b] - 1];
                    cur_boxes[b][0] = last->box_end_x;
                    cur_boxes[b][1] = last->box_end_y;
                }
            }

            for(uint8 bid = 0; bid < box_count; bid++) {
                uint8 p = prog[bid];
                micro_task_t *task;
                uint8 obs[MAX_BOXES * 2];
                uint8 obs_cnt = 0;
                uint8 cur_x;
                uint8 cur_y;
                car_path_t cp;
                int32 new_cost;
                uint8 new_car_idx;
                uint8 new_p[MAX_BOXES];
                uint16 old;

                if(p >= task_counts[bid]) continue;
                task = &all_tasks[bid][p];

                if(!task_push_is_clear(task, bid, box_count, prog, cur_boxes, map))
                    continue;

                for(uint8 j = 0; j < box_count; j++) {
                    if(prog[j] >= task_counts[j]) continue;
                    obs[obs_cnt * 2] = cur_boxes[j][0];
                    obs[obs_cnt * 2 + 1] = cur_boxes[j][1];
                    obs_cnt++;
                }

                cur_x = (uint8)(cur.car_idx % MAP_MAX_W);
                cur_y = (uint8)(cur.car_idx / MAP_MAX_W);
                cp = car_bfs_path(cur_x, cur_y,
                                  task->car_target_x, task->car_target_y,
                                  map, obs, obs_cnt);
                if(cp.dist < 0) continue;

                new_cost = cur.cost + cp.dist + task->n_steps;
                if(cur.last_dir != LAST_DIR_NONE && cur.last_dir != task->push_dir)
                    new_cost += 2;

                new_car_idx = (uint8)(task->car_end_y * MAP_MAX_W + task->car_end_x);
                memcpy(new_p, prog, sizeof(new_p));
                new_p[bid] = p + 1;

                old = best_get(new_car_idx, new_p, task->push_dir);
                if((uint16)new_cost < old) {
                    uint16 path_idx;
                    if(sched_path_count >= SCHED_PATH_MAX) continue;
                    path_idx = sched_path_count++;

                    best_set(new_car_idx, new_p, task->push_dir,
                             (uint16)new_cost);
                    sched_path[path_idx].box_id = bid;
                    sched_path[path_idx].task_idx = p;
                    sched_path[path_idx].car_dist = cp.dist;
                    sched_path[path_idx].parent = cur.parent;

                    {
                        sched_node_t next;
                        memset(&next, 0, sizeof(next));
                        next.cost = new_cost;
                        next.car_idx = new_car_idx;
                        memcpy(next.progress, new_p, sizeof(next.progress));
                        next.last_dir = task->push_dir;
                        next.parent = path_idx;
                        next.box_id = bid;
                        next.task_idx = p;
                        sched_push(next);
                    }
                }
            }
        }
    }

    return -1;
}

static int generate_actions(
    schedule_entry_t *schedule, uint8 sched_len,
    uint8 car_x, uint8 car_y, map_input_t *map,
    uint8 boxes[][2], map_preprocess_t *pp,
    action_t *actions, uint8 *act_count)
{
    uint8 car_cx = car_x;
    uint8 car_cy = car_y;
    uint8 cur_boxes[MAX_BOXES][2];
    uint8 box_done[MAX_BOXES];
    (void)pp;

    *act_count = 0;
    memset(box_done, 0, sizeof(box_done));
    memcpy(cur_boxes, boxes, map->box_count * 2U * sizeof(uint8));
    for(uint8 j = 0; j < map->box_count; j++) {
        if(task_counts[j] == 0U) box_done[j] = 1U;
    }

    for(uint8 si = 0; si < sched_len; si++) {
        uint8 bid = schedule[si].box_id;
        uint8 tidx = schedule[si].task_idx;
        micro_task_t *task;
        uint8 obs[MAX_BOXES * 2];
        uint8 obs_cnt = 0;

        if(bid >= map->box_count || tidx >= task_counts[bid]) return -1;
        task = &all_tasks[bid][tidx];

        for(uint8 j = 0; j < map->box_count; j++) {
            if(box_done[j]) continue;
            obs[obs_cnt * 2] = cur_boxes[j][0];
            obs[obs_cnt * 2 + 1] = cur_boxes[j][1];
            obs_cnt++;
        }

        if(car_cx != task->car_target_x || car_cy != task->car_target_y) {
            action_t *a;
            uint8 wp_x[MAX_WP];
            uint8 wp_y[MAX_WP];
            uint8 wp_n;

            if(*act_count >= MAX_ACTIONS) return -1;
            a = &actions[(*act_count)++];
            memset(a, 0, sizeof(action_t));
            a->type = ACTION_FREE_MOVE;
            a->target_x = task->car_target_x;
            a->target_y = task->car_target_y;
            a->theta = NAN;

            wp_n = avoidance_graph_get_waypoints(
                car_cx, car_cy, task->car_target_x, task->car_target_y,
                map, obs, obs_cnt, wp_x, wp_y, MAX_WP);

            if(wp_n > 0) {
                a->wp_count = wp_n;
                for(uint8 w = 0; w < wp_n; w++) {
                    a->waypoints[w].x_mm = (int16)wp_x[w] * SOLVER_GRID_SIZE_MM;
                    a->waypoints[w].y_mm = (int16)wp_y[w] * SOLVER_GRID_SIZE_MM;
                }
            } else {
                car_path_t cp = car_bfs_path(car_cx, car_cy,
                    task->car_target_x, task->car_target_y, map, obs, obs_cnt);
                if(cp.dist < 0) return -1;
                if(cp.path_len > MAX_WP) return -1;
                a->wp_count = (uint8)cp.path_len;
                for(uint8 w = 0; w < a->wp_count; w++) {
                    a->waypoints[w].x_mm = (int16)cp.px[w] * SOLVER_GRID_SIZE_MM;
                    a->waypoints[w].y_mm = (int16)cp.py[w] * SOLVER_GRID_SIZE_MM;
                }
            }
        }

        if(*act_count >= MAX_ACTIONS || task->push_dir > DIR_RIGHT) return -1;
        {
            action_t *pa = &actions[(*act_count)++];
            push_meta_t *pm;
            memset(pa, 0, sizeof(action_t));
            pa->type = ACTION_PUSH_BOX;
            pa->target_x = task->car_target_x;
            pa->target_y = task->car_target_y;
            pa->theta = NAN;
            pm = &pa->push_meta;
            pm->box_id = bid;
            pm->box_start_x = task->box_start_x;
            pm->box_start_y = task->box_start_y;
            pm->box_target_x = task->box_end_x;
            pm->box_target_y = task->box_end_y;
            pm->push_dir = task->push_dir;
            pm->n_steps = task->n_steps;
            pm->car_target_x = task->car_target_x;
            pm->car_target_y = task->car_target_y;
            strcpy(pm->push_dir_str, dir_names[task->push_dir]);
        }

        car_cx = task->car_end_x;
        car_cy = task->car_end_y;
        cur_boxes[bid][0] = task->box_end_x;
        cur_boxes[bid][1] = task->box_end_y;
        if((uint8)(tidx + 1U) >= task_counts[bid]) box_done[bid] = 1U;

    }

    return 0;
}

int micro_schedule_and_generate(
    push_result_t *pushes, uint8 box_count,
    uint8 car_x, uint8 car_y,
    map_input_t *map, uint8 boxes[][2],
    map_preprocess_t *pp,
    action_t *actions, uint8 *act_count,
    int8 *assignment)
{
    schedule_entry_t *schedule = generated_schedule;
    uint8 sched_len = 0;
    (void)assignment;

    if(box_count > MAX_BOXES) return -1;
    if(decompose_boxes(pushes, box_count, boxes) < 0) return -1;
    if(schedule_micro_tasks(box_count, car_x, car_y,
                            map, boxes, schedule, &sched_len) < 0) return -2;
    if(generate_actions(schedule, sched_len, car_x, car_y,
                        map, boxes, pp, actions, act_count) < 0) return -3;

    return 0;
}
