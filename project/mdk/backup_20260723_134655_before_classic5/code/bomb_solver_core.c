#include "zf_common_headfile.h"
#include "motion_control.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "bomb_solver_core.h"

// ==================== 求解器核心参数 ====================
#define MAX_R 12
#define MAX_C 16
#define MAX_ITEMS 3
#define MAX_NUDGES 3

#define MAX_NODES_1 1000     // 第一轨：常规搜索节点限制（极速）
#define MAX_NODES 3000       // 第二轨：无解时的极限兜底节点限制
#define HASH_SIZE 8192       
#define MAX_P2_Q 10000        
#define TRACK_NUM 4
#define WEIGHT_FACTOR 4      
#define MAX_STEPS 400      

#define GET_CURRENT_MS() ((uint32_t)pit_count * 5U)
#define MAX_SOLVE_TIME_MS 10500

#ifndef BOMB_SOLVER_TRACE_ENABLE
#define BOMB_SOLVER_TRACE_ENABLE 0
#endif

#if !BOMB_SOLVER_TRACE_ENABLE
#define printf(...) ((void)0)
#endif

// ==================== ★ 极速坐标转换宏 ====================
#define GET_R(pos) ((pos) >> 4)      
#define GET_C(pos) ((pos) & 15)      
#define MAKE_POS(r, c) (((r) << 4) | (c)) 

// 全局视觉绑定变量，必须在调用 run_phase1 前赋值好！
int BOX_TARGET_MAP[3] = {0, 1, 2}; 
char original_map[MAX_R][MAX_C + 1];

static uint32_t pos_hash_table[256]; 
static bool is_hash_table_init = false;

static int16_t q_f[MAX_NODES]; // 用于单独缓存 f = g + h
static int16_t q_g[MAX_NODES];
static int16_t relaxed_dist_b[3][MAX_R][MAX_C];
// ==================== 性能剖析监控变量 ====================
static double t_setup_and_hull = 0;
static double t_filter1_bfs = 0;
static double t_filter3_dist = 0;
static double t_filter2_prune = 0;
static double t_p1_astar = 0;
static double t_p2_bfs = 0;
static int p1_exec_cnt = 0;
static int p2_exec_cnt = 0;

// 辅助工具：将 timer_get 的差值转为秒
static inline double get_time_s(uint32_t start_ms, uint32_t end_ms) {
    return (double)(end_ms - start_ms) / 1000.0;
}

#pragma pack(push, 1)
struct State {
    uint16_t r, c;
    uint16_t b[MAX_ITEMS];      
    uint16_t bx[MAX_ITEMS];
    uint16_t mask;   
    uint16_t nudges; 
};
#pragma pack(pop)

struct Node {
    struct State s;
    int16_t g, h;
    int16_t parent;
    int32_t action;
};

// ==================== 极速静态内存池 ====================
AT_OCRAM_SECTION(static uint32_t hash_keys[HASH_SIZE]);
AT_OCRAM_SECTION(static uint16_t hash_gen[HASH_SIZE]);
static uint16_t current_hash_gen = 0;

AT_OCRAM_SECTION(static struct Node q[MAX_NODES]);
AT_OCRAM_SECTION(static int16_t heap[MAX_NODES]);
static int16_t heap_size = 0, node_cnt = 0;

static int16_t dr[] = {-1, 1, 0, 0};
static int16_t dc[] = {0, 0, -1, 1};
static int16_t p_dir[] = {0, 2, 3, 1}; 

static int16_t x_pos[MAX_ITEMS], dot_pos[MAX_ITEMS], dot_cnt = 0;
static int16_t bfs_dist_x[MAX_ITEMS][MAX_R][MAX_C], bfs_dist_dot[MAX_ITEMS][MAX_R][MAX_C];
static int16_t init_b[MAX_ITEMS][2], init_b_cnt = 0;
static int16_t init_bx[MAX_ITEMS][2], init_bx_cnt = 0;
static int16_t init_dot[MAX_ITEMS][2], init_dot_cnt = 0;
static int16_t init_robot_r = 0, init_robot_c = 0;
static int16_t perms[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};

struct SimState { uint16_t r, c, mask; uint16_t b[3]; uint16_t bx[3]; };
static struct SimState p1_states[MAX_STEPS + 1];
static int16_t p1_acts[MAX_STEPS]; 
static int p1_len = 0;
static uint16_t obs_b = 0, obs_t = 0;

// 二阶段队列，携带真实距离 dist
struct QN { int8_t rr, rc, br, bc; int16_t parent; int16_t dist; int8_t action; uint8_t pad; };
AT_OCRAM_SECTION(static struct QN rq_p2[MAX_P2_Q]);

static struct SimState g_final_p1_state; 
static int16_t g_best_p1_acts[MAX_STEPS]; 
static struct SimState g_best_p1_states[MAX_STEPS + 1];
static int g_best_p1_len = 0;
static uint16_t g_best_obs_b = 0;
static uint16_t g_best_obs_t = 0;
static int16_t g_best_bombs[3][2];
static int16_t g_best_p2_acts[MAX_STEPS]; 
static int g_best_p2_len = 0;

// ★ 临时 P2 缓冲
static int16_t temp_p2_acts[MAX_STEPS];
static int temp_p2_len = 0;

AT_OCRAM_SECTION(static uint16_t p2_vis[MAX_R][MAX_C][MAX_R][MAX_C]);
static uint16_t p2_vis_id = 0;

static uint16_t p1_dist_vis[MAX_R][MAX_C] = {0};
static uint16_t p1_dist_vis_id = 0;
static int16_t  global_dist[MAX_R][MAX_C];
static int16_t  global_parent[MAX_R][MAX_C];
static int16_t  global_action[MAX_R][MAX_C];

static uint16_t r_vis[MAX_R][MAX_C] = {0};
static uint16_t r_vis_id = 0;

static uint16_t reach_vis[MAX_R][MAX_C] = {0};
static uint16_t current_reach_id = 0;

static bool global_base_wall[MAX_R][MAX_C];
static bool base_wall_initialized = false;
static uint8_t global_exp_cov[MAX_R][MAX_C];
static bool deadlock_map[MAX_R][MAX_C];

AT_OCRAM_SECTION(static uint16_t global_alive_token[MAX_R][MAX_C][4]);
static uint16_t current_alive_token = 0;
static bool global_can_be_first[MAX_R][MAX_C] = {false};

// ★ 新增：O(1) 掩码墙壁预计算表
static bool fast_wall[8][MAX_R][MAX_C];

typedef struct { int r, c; } Coord;
static bool is_dist_ok(int r1, int c1, int r2, int c2) { return ((r1-r2)*(r1-r2) + (c1-c2)*(c1-c2)) >= 4; }

static long long cross_product_2x(int r1, int c1, int r2, int c2, int r3, int c3) { 
    return (long long)(r2 - r1) * (c3 - c1) - (long long)(c2 - c1) * (r3 - r1);
}

static bool is_point_in_hull_2x(int r, int c, Coord* hull, int hull_size) {
    bool has_pos = false, has_neg = false;
    for (int i = 0; i < hull_size; i++) {
        long long cp = cross_product_2x(hull[i].r * 2, hull[i].c * 2, hull[(i + 1) % hull_size].r * 2, hull[(i + 1) % hull_size].c * 2, r, c);
        if (cp > 0) has_pos = true; if (cp < 0) has_neg = true;
    } 
    return !(has_pos && has_neg);
}

static bool is_wall_intersect_hull(int r, int c, Coord* hull, int hull_size) {
    int corners[4][2] = {
        {(int)(2*r - 1.5), (int)(2*c - 1.5)}, {(int)(2*r - 1.5), (int)(2*c + 1.5)}, 
        {(int)(2*r + 1.5), (int)(2*c - 1.5)}, {(int)(2*r + 1.5), (int)(2*c + 1.5)}
    };
    for (int i = 0; i < 4; i++) if (is_point_in_hull_2x(corners[i][0], corners[i][1], hull, hull_size)) return true;
    return false;
}

static int get_convex_hull(Coord* pts, int n, Coord* hull) {
    if (n < 3) return 0;
    int l = 0; for (int i = 1; i < n; i++) if (pts[i].c < pts[l].c) l = i;
    int p = l, q, cnt = 0;
    do { 
        hull[cnt++] = pts[p]; q = (p + 1) % n;
        for (int i = 0; i < n; i++) {
            long long cp = (long long)(pts[i].r - pts[p].r) * (pts[q].c - pts[p].c) - (long long)(pts[i].c - pts[p].c) * (pts[q].r - pts[p].r);
            if (cp > 0) q = i;
        } 
        p = q;
    } while (p != l); 
    return cnt;
}

static inline bool compare_nodes(int idx1, int idx2) {
    if (q_f[idx1] != q_f[idx2]) return q_f[idx1] < q_f[idx2];
    return q_g[idx1] > q_g[idx2]; 
}

static void push_heap(int idx) {
    int i = heap_size++;
    while (i > 0) {
        int p = (i - 1) / 2; if (compare_nodes(heap[p], idx)) break;
        heap[i] = heap[p]; i = p;
    }
    heap[i] = idx;
}

static int pop_heap() {
    int res = heap[0], idx = heap[--heap_size], i = 0;
    while (i * 2 + 1 < heap_size) {
        int left = i * 2 + 1, right = i * 2 + 2, min_c = left;
        if (right < heap_size && compare_nodes(heap[right], heap[left])) min_c = right;
        if (compare_nodes(idx, heap[min_c])) break;
        heap[i] = heap[min_c]; i = min_c;
    }
    heap[i] = idx; return res;
}

// ★ 优化 1：寄存器级展开极速编码
static uint32_t encode(struct State *s) {
    if (!is_hash_table_init) {
        uint32_t seed = 0x12345678;
        for (int i = 0; i < 256; i++) {
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
            pos_hash_table[i] = seed;
        }
        is_hash_table_init = true;
    }

    uint32_t h = 2166136261u; 
    h ^= (s->r << 8) | s->c; 
    h *= 16777619u;
    h ^= s->mask;            
    h *= 16777619u;
    h ^= s->nudges;          
    h *= 16777619u;

    uint32_t b_hash = 0, bx_hash = 0;
    if (s->b[0] != 255) b_hash ^= pos_hash_table[s->b[0]];
    if (s->b[1] != 255) b_hash ^= pos_hash_table[s->b[1]];
    if (s->b[2] != 255) b_hash ^= pos_hash_table[s->b[2]];
    
    if (s->bx[0] != 255) bx_hash ^= pos_hash_table[s->bx[0]];
    if (s->bx[1] != 255) bx_hash ^= pos_hash_table[s->bx[1]];
    if (s->bx[2] != 255) bx_hash ^= pos_hash_table[s->bx[2]];

    h ^= b_hash;
    h *= 16777619u;
    h ^= bx_hash;
    h *= 16777619u;

    return h;
}

static inline bool hash_insert(uint32_t key) {
    uint32_t mixed_key = key ^ (key >> 16) ^ (key >> 8); 
    uint32_t idx = mixed_key & 8191; 
    while (hash_gen[idx] == current_hash_gen) {
        if (hash_keys[idx] == key) return false;
        idx = (idx + 1) & 8191; 
    }
    hash_keys[idx] = key; hash_gen[idx] = current_hash_gen; return true;
}

static int get_h_p1(struct State *s) {
    int mask = s->mask;
    int16_t b0 = s->b[0], b1 = s->b[1], b2 = s->b[2];
    
    // 预检：遇到 255 即认为是虚拟炸弹/缺失炸弹，不再计算代价
    bool v0 = (b0 != 255);
    bool v1 = (b1 != 255);
    bool v2 = (b2 != 255);
    
    int c0[3] = {0}, c1[3] = {0}, c2[3] = {0};
    int unexploded = 0;
    int min_p_dist = 9999;
    int sr = s->r, sc = s->c;

    if (v0) {
        unexploded++;
        int r = GET_R(b0), c = GET_C(b0);
        int d = abs(sr - r) + abs(sc - c);
        if (d < min_p_dist) min_p_dist = d;
        if (!(mask & 1)) c0[0] = bfs_dist_x[0][r][c];
        if (!(mask & 2)) c1[0] = bfs_dist_x[1][r][c];
        if (!(mask & 4)) c2[0] = bfs_dist_x[2][r][c];
    }
    if (v1) {
        unexploded++;
        int r = GET_R(b1), c = GET_C(b1);
        int d = abs(sr - r) + abs(sc - c);
        if (d < min_p_dist) min_p_dist = d;
        if (!(mask & 1)) c0[1] = bfs_dist_x[0][r][c];
        if (!(mask & 2)) c1[1] = bfs_dist_x[1][r][c];
        if (!(mask & 4)) c2[1] = bfs_dist_x[2][r][c];
    }
    if (v2) {
        unexploded++;
        int r = GET_R(b2), c = GET_C(b2);
        int d = abs(sr - r) + abs(sc - c);
        if (d < min_p_dist) min_p_dist = d;
        if (!(mask & 1)) c0[2] = bfs_dist_x[0][r][c];
        if (!(mask & 2)) c1[2] = bfs_dist_x[1][r][c];
        if (!(mask & 4)) c2[2] = bfs_dist_x[2][r][c];
    }

    if (unexploded == 0) return 0;

    int min_b_cost = c0[0] + c1[1] + c2[2];
    int p1 = c0[0] + c2[1] + c1[2]; if (p1 < min_b_cost) min_b_cost = p1;
    int p2 = c1[0] + c0[1] + c2[2]; if (p2 < min_b_cost) min_b_cost = p2;
    int p3 = c1[0] + c2[1] + c0[2]; if (p3 < min_b_cost) min_b_cost = p3;
    int p4 = c2[0] + c0[1] + c1[2]; if (p4 < min_b_cost) min_b_cost = p4;
    int p5 = c2[0] + c1[1] + c0[2]; if (p5 < min_b_cost) min_b_cost = p5;

    return (unexploded + min_b_cost + min_p_dist) * WEIGHT_FACTOR;
}

static inline bool check_robot_reach_fast(int sr, int sc, int tr, int tc, int br, int bc, bool wall[MAX_R][MAX_C]) {
    if (sr == tr && sc == tc) return true;
    int dist = abs(sr - tr) + abs(sc - tc);
    if (dist == 1) return true; 

    // O(1) 几何贴身绕行短路判定
    if (dist == 2) {
        if (!wall[sr][tc]) return true;
    } 
    else if (dist == 4) {
        if (sr == tr) { 
            if (br > 0 && !wall[br-1][sc] && !wall[br-1][bc] && !wall[br-1][tc]) return true;
            if (br < MAX_R-1 && !wall[br+1][sc] && !wall[br+1][bc] && !wall[br+1][tc]) return true;
        } else if (sc == tc) { 
            if (bc > 0 && !wall[sr][bc-1] && !wall[br][bc-1] && !wall[tr][bc-1]) return true;
            if (bc < MAX_C-1 && !wall[sr][bc+1] && !wall[br][bc+1] && !wall[tr][bc+1]) return true;
        }
    }

    r_vis_id++; 
    if (r_vis_id == 0xFFFF) { memset(r_vis, 0, sizeof(r_vis)); r_vis_id = 1; }
    
    uint8_t fast_q[256]; 
    int h = 0, t = 0; 
    fast_q[t++] = (sr << 4) | sc; 
    r_vis[sr][sc] = r_vis_id;
    
    uint8_t target_pos = (tr << 4) | tc;
    uint8_t box_pos = (br << 4) | bc;

    while(h < t) {
        uint8_t curr = fast_q[h++];
        if (curr == target_pos) return true;
        int r = curr >> 4, c = curr & 15;
        
        int nr, nc; uint8_t npos;
        nr = r - 1; nc = c;
        if (nr >= 0 && !wall[nr][nc]) { npos = (nr << 4) | nc; if (npos != box_pos && r_vis[nr][nc] != r_vis_id) { r_vis[nr][nc] = r_vis_id; fast_q[t++] = npos; } }
        nr = r + 1; nc = c;
        if (nr < MAX_R && !wall[nr][nc]) { npos = (nr << 4) | nc; if (npos != box_pos && r_vis[nr][nc] != r_vis_id) { r_vis[nr][nc] = r_vis_id; fast_q[t++] = npos; } }
        nr = r; nc = c - 1;
        if (nc >= 0 && !wall[nr][nc]) { npos = (nr << 4) | nc; if (npos != box_pos && r_vis[nr][nc] != r_vis_id) { r_vis[nr][nc] = r_vis_id; fast_q[t++] = npos; } }
        nr = r; nc = c + 1;
        if (nc < MAX_C && !wall[nr][nc]) { npos = (nr << 4) | nc; if (npos != box_pos && r_vis[nr][nc] != r_vis_id) { r_vis[nr][nc] = r_vis_id; fast_q[t++] = npos; } }
    }
    return false;
}

static void build_opt_wall_and_rigid_bodies(int16_t exploded_sites[][2], int exp_cnt, int16_t pushable_targets[][2], int pt_cnt, bool out_wall[MAX_R][MAX_C]) {
    bool temp_wall[MAX_R][MAX_C];
    if (!base_wall_initialized) {
        for(int r=0; r<MAX_R; r++) for(int c=0; c<MAX_C; c++) global_base_wall[r][c] = (original_map[r][c] == '#');
        base_wall_initialized = true;
    }
    memcpy(temp_wall, global_base_wall, sizeof(global_base_wall));

    for(int i=0; i<exp_cnt; i++) {
        int xr = exploded_sites[i][0], xc = exploded_sites[i][1];
        for(int dr_i=-1; dr_i<=1; dr_i++) for(int dc_i=-1; dc_i<=1; dc_i++) {
            int nr = xr+dr_i, nc = xc+dc_i;
            if(nr>0 && nr<MAX_R-1 && nc>0 && nc<MAX_C-1) temp_wall[nr][nc] = false;
        }
    }
    bool item_movable[MAX_ITEMS + MAX_ITEMS]; memset(item_movable, false, sizeof(item_movable));
    bool changed = true;
    while(changed) {
        changed = false;
        for(int i=0; i<init_b_cnt + init_bx_cnt; i++) {
            if(item_movable[i]) continue;
            int r = (i < init_b_cnt) ? init_b[i][0] : init_bx[i - init_b_cnt][0];
            int c = (i < init_b_cnt) ? init_b[i][1] : init_bx[i - init_b_cnt][1];
            bool can_push = false;
            if (i < init_b_cnt) {
                for (int d = 0; d < 4; d++) {
                    int tr = r + dr[d], tc = c + dc[d], rr = r - dr[d], rc = c - dc[d];
                    for(int k = 0; k < pt_cnt; k++) {
                        if (tr == pushable_targets[k][0] && tc == pushable_targets[k][1]) {
                            if (rr >= 0 && rr < MAX_R && rc >= 0 && rc < MAX_C && !temp_wall[rr][rc]) { can_push = true; break; }
                        }
                    }
                    if (can_push) break;
                }
            }
            if (!can_push) {
                for(int d=0; d<4; d++) {
                    int tr = r+dr[d], tc = c+dc[d], rr = r-dr[d], rc = c-dc[d];
                    if (tr < 0 || tr >= MAX_R || tc < 0 || tc >= MAX_C || rr < 0 || rr >= MAX_R || rc < 0 || rc >= MAX_C) continue;
                    if(temp_wall[tr][tc] || temp_wall[rr][rc]) continue;
                    bool blocked = false;
                    for(int j=0; j<init_b_cnt + init_bx_cnt; j++) {
                        if(i == j || item_movable[j]) continue;
                        int jr = (j < init_b_cnt) ? init_b[j][0] : init_bx[j - init_b_cnt][0];
                        int jc = (j < init_b_cnt) ? init_b[j][1] : init_bx[j - init_b_cnt][1];
                        if((jr == tr && jc == tc) || (jr == rr && jc == rc)) { blocked = true; break; }
                    }
                    if(!blocked) { can_push = true; break; }
                }
            }
            if(can_push) { item_movable[i] = true; changed = true; }
        }
    }
    memcpy(out_wall, temp_wall, sizeof(temp_wall));
    for(int i=0; i<init_b_cnt + init_bx_cnt; i++) {
        if(!item_movable[i]) {
            int r = (i < init_b_cnt) ? init_b[i][0] : init_bx[i - init_b_cnt][0];
            int c = (i < init_b_cnt) ? init_b[i][1] : init_bx[i - init_b_cnt][1];
            out_wall[r][c] = true;
        }
    }
}
static uint16_t flat_alive_token[1024] = {0};
AT_OCRAM_SECTION(static uint16_t reach_cache_id[MAX_R][MAX_C][4][4]);
AT_OCRAM_SECTION(static bool reach_cache_val[MAX_R][MAX_C][4][4]);
static uint16_t cur_reach_id = 0;
static uint16_t run_macro_pull(int16_t targets[][2], int target_cnt, bool wall[MAX_R][MAX_C], int16_t items[][2], int item_cnt) {
    current_alive_token++;
    if (current_alive_token == 0xFFFF) { memset(flat_alive_token, 0, sizeof(flat_alive_token)); current_alive_token = 1; }
    
    cur_reach_id++; 
    if (cur_reach_id == 0xFFFF) { memset(reach_cache_id, 0, sizeof(reach_cache_id)); cur_reach_id = 1; }
    
    uint16_t fast_macro_q[1024]; 
    int h=0, t=0; 
    
    for(int i=0; i<target_cnt; i++) {
        for(int d=0; d<4; d++) {
            int pr = targets[i][0] - dr[d], pc = targets[i][1] - dc[d], rr = pr - dr[d], rc = pc - dc[d];
            if (pr>=0 && pr<MAX_R && pc>=0 && pc<MAX_C && rr>=0 && rr<MAX_R && rc>=0 && rc<MAX_C) {
                if (!wall[pr][pc] && !wall[rr][rc]) { 
                    int flat_idx = (pr << 6) | (pc << 2) | d;
                    flat_alive_token[flat_idx] = current_alive_token; 
                    fast_macro_q[t++] = flat_idx;
                }
            }
        }
    }
    
    while(h < t) {
        uint16_t val = fast_macro_q[h++];
        int r = val >> 6, c = (val >> 2) & 15, d = val & 3;
        
        int nr = r - dr[d], nc = c - dc[d], rr = nr - dr[d], rc = nc - dc[d];
        if (nr>=0 && nr<MAX_R && nc>=0 && nc<MAX_C && rr>=0 && rr<MAX_R && rc>=0 && rc<MAX_C) {
            int flat_nr = (nr << 6) | (nc << 2) | d;
            if (!wall[nr][nc] && !wall[rr][rc] && flat_alive_token[flat_nr] != current_alive_token) { 
                flat_alive_token[flat_nr] = current_alive_token; 
                fast_macro_q[t++] = flat_nr; 
            }
        }
        int curr_rr = r - dr[d], curr_rc = c - dc[d];
        for(int nd=0; nd<4; nd++) {
            int flat_nd = (r << 6) | (c << 2) | nd;
            if (nd == d || flat_alive_token[flat_nd] == current_alive_token) continue;
            int next_rr = r - dr[nd], next_rc = c - dc[nd];
            if (next_rr>=0 && next_rr<MAX_R && next_rc>=0 && next_rc<MAX_C && !wall[next_rr][next_rc]) {
                bool can_reach = false;
                if (reach_cache_id[r][c][d][nd] == cur_reach_id) {
                    can_reach = reach_cache_val[r][c][d][nd]; 
                } else {
                    can_reach = check_robot_reach_fast(curr_rr, curr_rc, next_rr, next_rc, r, c, wall);
                    reach_cache_id[r][c][d][nd] = cur_reach_id;
                    reach_cache_val[r][c][d][nd] = can_reach;
                    reach_cache_id[r][c][nd][d] = cur_reach_id;
                    reach_cache_val[r][c][nd][d] = can_reach;
                }
                if (can_reach) { 
                    flat_alive_token[flat_nd] = current_alive_token; 
                    fast_macro_q[t++] = flat_nd; 
                }
            }
        }
    }
    uint16_t reached_mask = 0;
    for(int k=0; k<item_cnt; k++) {
        int bx = items[k][0], by = items[k][1];
        bool already_there = false;
        for(int i=0; i<target_cnt; i++) { if(bx == targets[i][0] && by == targets[i][1]) { already_there = true; break; } }
        if(already_there) { reached_mask |= (1 << k); continue; }
        for(int d=0; d<4; d++) {
            int flat_bx = (bx << 6) | (by << 2) | d;
            if (flat_alive_token[flat_bx] == current_alive_token && check_robot_reach_fast(init_robot_r, init_robot_c, bx - dr[d], by - dc[d], bx, by, wall)) { 
                reached_mask |= (1 << k); break; 
            }
        }
    }
    return reached_mask;
}

static bool fast_prune_check(int16_t bombs[3][2], int b_cnt) {
    if (b_cnt == 0) return false;

    // 1. 【通用拦截】先手判定：无论几个炸弹，至少得有一颗雷是机器人初始可以触达并推动的
    bool has_first = false;
    for (int i = 0; i < b_cnt; i++) {
        if (global_can_be_first[bombs[i][0]][bombs[i][1]]) {
            has_first = true; break;
        }
    }
    if (!has_first) return true; // 全都够不着，直接死刑拦截！

    // 2. 【2炸弹特有拦截】双炸弹时序因果链条判定
    if (b_cnt == 2) {
        bool order1_ok = global_can_be_first[bombs[0][0]][bombs[0][1]];
        bool order2_ok = global_can_be_first[bombs[1][0]][bombs[1][1]];
        
        // 如果只有炸弹0能作为先手，检查炸完雷0后，机器人能否绕过去摸到炸弹1
        if (order1_ok && !order2_ok) {
            bool temp_wall[MAX_R][MAX_C];
            int16_t exp1[1][2] = {{bombs[0][0], bombs[0][1]}};
            build_opt_wall_and_rigid_bodies(exp1, 1, NULL, 0, temp_wall);
            if (!check_robot_reach_fast(init_robot_r, init_robot_c, bombs[1][0], bombs[1][1], 255, 255, temp_wall)) {
                return true; // 炸完雷0也够不着雷1，剪枝！
            }
        }
        // 如果只有炸弹1能作为先手
        else if (!order1_ok && order2_ok) {
            bool temp_wall[MAX_R][MAX_C];
            int16_t exp1[1][2] = {{bombs[1][0], bombs[1][1]}};
            build_opt_wall_and_rigid_bodies(exp1, 1, NULL, 0, temp_wall);
            if (!check_robot_reach_fast(init_robot_r, init_robot_c, bombs[0][0], bombs[0][1], 255, 255, temp_wall)) {
                return true; // 炸完雷1也够不着雷0，剪枝！
            }
        }
    }

    // 3. 【通用拦截】绝对兜底防御：炸完之后，利用刚体逆向拉回函数测试所有目标点
    bool ult_wall[MAX_R][MAX_C];
    build_opt_wall_and_rigid_bodies(bombs, b_cnt, NULL, 0, ult_wall);
    if (run_macro_pull(init_dot, init_dot_cnt, ult_wall, init_bx, init_bx_cnt) != (1 << init_bx_cnt) - 1) {
        return true; // 刚体锁死或无法被拉回目标，剪枝！
    }

    // 4. 如果是1或2颗炸弹，通过上述三道拦截后直接放行给 A*
    if (b_cnt < 3) return false; 

    // 5. 针对 3 颗炸弹保留你原本原汁原味的时序推演逻辑
    bool reach_computed[3] = {false, false, false};
    uint16_t b_mask[3] = {0, 0, 0};

    for(int k=0; k<6; k++) {
        int p0 = perms[k][0];
        if (!reach_computed[0]) {
            int16_t exp2[2][2] = { {bombs[1][0], bombs[1][1]}, {bombs[2][0], bombs[2][1]} };
            bool opt_wall[MAX_R][MAX_C]; int16_t target[1][2] = {{bombs[0][0], bombs[0][1]}};
            build_opt_wall_and_rigid_bodies(exp2, 2, target, 1, opt_wall);
            b_mask[0] = run_macro_pull(target, 1, opt_wall, init_b, init_b_cnt);
            reach_computed[0] = true;
        }
        if (!(b_mask[0] & (1 << p0))) continue; 

        int p1 = perms[k][1];
        if (!reach_computed[1]) {
            int16_t exp2[2][2] = { {bombs[0][0], bombs[0][1]}, {bombs[2][0], bombs[2][1]} };
            bool opt_wall[MAX_R][MAX_C]; int16_t target[1][2] = {{bombs[1][0], bombs[1][1]}};
            build_opt_wall_and_rigid_bodies(exp2, 2, target, 1, opt_wall);
            b_mask[1] = run_macro_pull(target, 1, opt_wall, init_b, init_b_cnt);
            reach_computed[1] = true;
        }
        if (!(b_mask[1] & (1 << p1))) continue;

        int p2 = perms[k][2];
        if (!reach_computed[2]) {
            int16_t exp2[2][2] = { {bombs[0][0], bombs[0][1]}, {bombs[1][0], bombs[1][1]} };
            bool opt_wall[MAX_R][MAX_C]; int16_t target[1][2] = {{bombs[2][0], bombs[2][1]}};
            build_opt_wall_and_rigid_bodies(exp2, 2, target, 1, opt_wall);
            b_mask[2] = run_macro_pull(target, 1, opt_wall, init_b, init_b_cnt);
            reach_computed[2] = true;
        }
        if (b_mask[2] & (1 << p2)) {
            return false; 
        }
    }
    return true; 
}

static bool fast_forward_bfs_check(int16_t bombs[3][2], int b_cnt) {
    bool wall[MAX_R][MAX_C];
    for(int r=0; r<MAX_R; r++) for(int c=0; c<MAX_C; c++) wall[r][c] = (original_map[r][c] == '#');
    for(int i=0; i<b_cnt; i++) {
        int br = bombs[i][0], bc = bombs[i][1];
        for(int dr_i=-1; dr_i<=1; dr_i++) for(int dc_i=-1; dc_i<=1; dc_i++) {
            int nr = br + dr_i, nc = bc + dc_i;
            if(nr > 0 && nr < MAX_R-1 && nc > 0 && nc < MAX_C-1) wall[nr][nc] = false;
        }
    }
    for(int i=0; i<init_bx_cnt; i++) {
        bool vis[MAX_R][MAX_C] = {false};
        uint16_t qr[256], qc[256]; int head = 0, tail = 0;
        qr[tail] = init_bx[i][0]; qc[tail] = init_bx[i][1]; tail++; vis[init_bx[i][0]][init_bx[i][1]] = true;
        bool reached_target = false; int target_r = init_dot[i][0], target_c = init_dot[i][1];
        while(head < tail) {
            int r = qr[head], c = qc[head]; head++;
            if (r == target_r && c == target_c) { reached_target = true; break; }
            for(int d=0; d<4; d++) {
                int nr = r + dr[d], nc = c + dc[d], pr = r - dr[d], pc = c - dc[d];
                if (nr>=0 && nr<MAX_R && nc>=0 && nc<MAX_C && pr>=0 && pr<MAX_R && pc>=0 && pc<MAX_C) {
                    if (!wall[nr][nc] && !wall[pr][pc] && !vis[nr][nc]) { vis[nr][nc] = true; qr[tail] = nr; qc[tail] = nc; tail++; }
                }
            }
        }
        if (!reached_target) return true;
    }
    return false;
}

static int get_manhattan_dist(int a_cnt, int16_t a[MAX_ITEMS][2], int b_cnt, int16_t b[MAX_ITEMS][2]) {
    int min_total = 99999;
    // 兼容数量不足的曼哈顿距离下界计算
    for(int m=0; m<6; m++) {
        int cost = 0;
        for (int i=0; i<a_cnt; i++) {
            cost += abs(a[i][0] - b[perms[m][i]][0]) + abs(a[i][1] - b[perms[m][i]][1]);
        }
        if (cost < min_total) min_total = cost;
    }
    return min_total;
}

static int count_bits(uint16_t n) { int c=0; while(n){c+=n&1; n>>=1;} return c; }

static void observation_requirements(int req_b, int req_t,
                                     int *need_b, int *need_t)
{
    if(TRACK_NUM == 4)
    {
        *need_b = req_b > 0 ? req_b - 1 : 0;
        *need_t = req_t > 0 ? req_t - 1 : 0;
    }
    else if(TRACK_NUM == 5 && req_b == 3 && req_t == 3)
    {
        *need_b = 2;
        *need_t = 3;
    }
    else
    {
        *need_b = req_b;
        *need_t = req_t;
    }
}

static bool observations_satisfied(uint16_t box_mask, uint16_t goal_mask,
                                   int req_b, int req_t)
{
    int need_b, need_t;
    int cb = count_bits(box_mask);
    int ct = count_bits(goal_mask);
    observation_requirements(req_b, req_t, &need_b, &need_t);
    if(TRACK_NUM == 5 && req_b == 3 && req_t == 3)
        return (cb >= 2 && ct >= 3) || (cb >= 3 && ct >= 2);
    return cb >= need_b && ct >= need_t;
}

static uint16_t limit_observation_bits(uint16_t current_mask,
                                       uint16_t candidate_mask,
                                       int required_count)
{
    int room = required_count - count_bits(current_mask);
    uint16_t selected = 0;
    uint16_t unseen = (uint16_t)(candidate_mask & ~current_mask);
    int bit;
    if(room <= 0) return 0;
    for(bit = 0; bit < MAX_ITEMS && room > 0; bit++)
    {
        uint16_t flag = (uint16_t)(1U << bit);
        if(unseen & flag)
        {
            selected |= flag;
            room--;
        }
    }
    return selected;
}

static int count_path_turns(int16_t* acts, int len) {
    int turns = 0;
    int last_move_dir = -1;
    for (int i = 0; i < len; i++) {
        int current_act = acts[i];
        if ((current_act >= 0 && current_act <= 4) || (current_act >= 6 && current_act <= 8)) {
            if (last_move_dir != -1 && current_act != last_move_dir) {
                turns++;
            }
            last_move_dir = current_act;
        }
    }
    return turns;
}

static void try_free_look(struct SimState* cur, int* cur_look) {
    int need_b, need_t;
    observation_requirements(init_bx_cnt, init_dot_cnt, &need_b, &need_t);
    for(int i=0; i<4; i++) {
        if(count_bits(obs_b) >= need_b && count_bits(obs_t) >= need_t) break;
        int d = p_dir[i]; 
        int nr = cur->r + dr[d], nc = cur->c + dc[d];
        if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
        
        int npos = (nr << 4) | nc; 
        bool saw = false;
        for(int k=0; k<3 && count_bits(obs_b)<need_b; k++) if(cur->bx[k]!=255 && cur->bx[k]==npos && !(obs_b & (1<<k))) { obs_b |= (1<<k); saw = true; }
        for(int k=0; k<3 && count_bits(obs_t)<need_t; k++) if(dot_pos[k]!=255 && dot_pos[k]==npos && !(obs_t & (1<<k))) { obs_t |= (1<<k); saw = true; }
        if (saw) { p1_states[p1_len] = *cur; p1_acts[p1_len++] = 5+d; *cur_look = d; }
    }
}

static void remove_duplicate_looks(int16_t* path, int* len) {
    if (*len <= 1) return;
    int write_idx = 1;
    for (int read_idx = 1; read_idx < *len; read_idx++) {
        int current_act = path[read_idx];
        int previous_act = path[write_idx - 1];
        if (current_act >= 5 && current_act <= 8 && current_act == previous_act) continue;
        path[write_idx++] = current_act;
    }
    *len = write_idx;
}

// ==================== ★ 无炸弹专用极速 TSP 求解器组件 ====================
struct POI { int r, c, b_mask, t_mask; };
static struct POI tsp_pois[64];
static int tsp_poi_cnt;
static int16_t tsp_dist_matrix[64][64];
static int tsp_min_cost;
static int tsp_best_path[32];
static int tsp_best_len;
static bool tsp_visited[64];

static void dfs_tsp_0bomb(int u, int current_cost, int cur_b, int cur_t, int path_len, int* path, int req_b, int req_t, struct SimState* cur_sim, int current_look) {
    if (current_cost >= tsp_min_cost) return; // 最优性剪枝

    if (observations_satisfied(cur_b, cur_t, req_b, req_t)) {
        tsp_min_cost = current_cost;
        tsp_best_len = path_len;
        memcpy(tsp_best_path, path, path_len * sizeof(int));
        return;
    }

    for (int v = 1; v < tsp_poi_cnt; v++) {
        if (!tsp_visited[v] && tsp_dist_matrix[u][v] != 9999) {
            int need_b, need_t;
            observation_requirements(req_b, req_t, &need_b, &need_t);
            int add_b = limit_observation_bits((uint16_t)cur_b,
                                               (uint16_t)tsp_pois[v].b_mask,
                                               need_b);
            int add_t = limit_observation_bits((uint16_t)cur_t,
                                               (uint16_t)tsp_pois[v].t_mask,
                                               need_t);
            if (add_b || add_t) {
                int req_looks[4], req_cnt = 0;
                // 检测到了这个观测点需要看向哪些方向
                for(int d=0; d<4; d++) {
                    int nr = tsp_pois[v].r + dr[d], nc = tsp_pois[v].c + dc[d];
                    int npos = MAKE_POS(nr, nc);
                    bool need_look = false;
                    for(int k=0; k<3; k++) {
                        if ((add_b & (1<<k)) && cur_sim->bx[k] == npos) need_look = true;
                        if ((add_t & (1<<k)) && dot_pos[k] == npos) need_look = true;
                    }
                    if (need_look) req_looks[req_cnt++] = d;
                }
                
                if (req_cnt > 0) {
                    // 检查需要的观测方向中是否包含当前朝向（利用当前朝向可以免除一次转向惩罚）
                    bool has_curr = false;
                    for(int i=0; i<req_cnt; i++) if(req_looks[i] == current_look) has_curr = true;
                    
                    // 计算带转向惩罚的真实 Cost (+5 惩罚基数)
                    int base_penalty = has_curr ? (req_cnt - 1) * 5 : req_cnt * 5;
                    int step_cost = tsp_dist_matrix[u][v] + req_cnt + base_penalty;
                    
                    tsp_visited[v] = true;
                    path[path_len] = v;
                    
                    if (req_cnt == 1) {
                        dfs_tsp_0bomb(v, current_cost + step_cost, cur_b | add_b, cur_t | add_t, path_len + 1, path, req_b, req_t, cur_sim, req_looks[0]);
                    } else {
                        // 尝试所有可能的最终滞留朝向，确保 DFS 全局最优（必定最后执行的不是 current_look）
                        for(int i=0; i<req_cnt; i++) {
                            if (has_curr && req_looks[i] == current_look) continue; 
                            dfs_tsp_0bomb(v, current_cost + step_cost, cur_b | add_b, cur_t | add_t, path_len + 1, path, req_b, req_t, cur_sim, req_looks[i]);
                        }
                    }
                    
                    tsp_visited[v] = false;
                }
            }
        }
    }
}
// ========================================================================
// ==================== P2 专用极速 A* 优先队列 ====================
static int16_t p2_heap[MAX_P2_Q];
static int16_t p2_heap_size = 0;
static int16_t p2_f[MAX_P2_Q]; // 缓存 f = g + h 的值

static inline bool cmp_p2(int a, int b) {
    return p2_f[a] < p2_f[b]; // 最小堆
}

static inline void push_p2(int idx) {
    int i = p2_heap_size++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (cmp_p2(p2_heap[p], idx)) break;
        p2_heap[i] = p2_heap[p];
        i = p;
    }
    p2_heap[i] = idx;
}

static inline int pop_p2() {
    int res = p2_heap[0];
    int idx = p2_heap[--p2_heap_size];
    int i = 0;
    while (i * 2 + 1 < p2_heap_size) {
        int left = i * 2 + 1, right = i * 2 + 2, min_c = left;
        if (right < p2_heap_size && cmp_p2(p2_heap[right], p2_heap[left])) min_c = right;
        if (cmp_p2(idx, p2_heap[min_c])) break;
        p2_heap[i] = p2_heap[min_c];
        i = min_c;
    }
    p2_heap[i] = idx;
    return res;
}
// ==================== 核心统合求解器 ====================
static float solve_with_bombs(int16_t bombs[3][2], int *out_p1_steps, struct SimState* final_st, uint32_t start_time, int active_max_nodes, float current_best_metric, int skip_flag) {
    uint32_t start_p1 = GET_CURRENT_MS(); 
    p1_exec_cnt++;
    for (int i = 0; i < 3; i++) x_pos[i] = MAKE_POS(bombs[i][0], bombs[i][1]);

    for (int m = 0; m < 8; m++) {
        for (int r = 0; r < MAX_R; r++) {
            for (int c = 0; c < MAX_C; c++) {
                if (r == 0 || r == MAX_R - 1 || c == 0 || c == MAX_C - 1) {
                    fast_wall[m][r][c] = true; continue;
                }
                bool in_exp = false;
                for (int i = 0; i < 3; i++) {
                    if (i >= init_b_cnt) continue; 
                    if (m & (1 << i)) {
                        if (abs(r - bombs[i][0]) <= 1 && abs(c - bombs[i][1]) <= 1) { in_exp = true; break; }
                    }
                }
                fast_wall[m][r][c] = !in_exp && (original_map[r][c] == '#');
            }
        }
    }

    for (int r = 0; r < MAX_R; r++) {
        for (int c = 0; c < MAX_C; c++) {
            global_exp_cov[r][c] = 0;
            if (r > 0 && r < MAX_R - 1 && c > 0 && c < MAX_C - 1) {
                for (int i = 0; i < 3; i++) {
                    if (i >= init_b_cnt) continue; 
                    if (abs(r - bombs[i][0]) <= 1 && abs(c - bombs[i][1]) <= 1) {
                        global_exp_cov[r][c] |= (1 << i);
                    }
                }
            }
        }
    }

    for (int r = 1; r < MAX_R - 1; r++) {
        for (int c = 1; c < MAX_C - 1; c++) {
            bool up = global_base_wall[r-1][c] && !(7 & global_exp_cov[r-1][c]);
            bool down = global_base_wall[r+1][c] && !(7 & global_exp_cov[r+1][c]);
            bool left = global_base_wall[r][c-1] && !(7 & global_exp_cov[r][c-1]);
            bool right = global_base_wall[r][c+1] && !(7 & global_exp_cov[r][c+1]);
            deadlock_map[r][c] = ((up && left) || (up && right) || (down && left) || (down && right));
        }
    }

    for(int i=0; i<3; i++) {
        for(int r=0; r<MAX_R; r++) for(int c=0; c<MAX_C; c++) bfs_dist_x[i][r][c] = 9999;
        if (i < init_b_cnt) {
            uint16_t qr[512], qc[512]; int head=0, tail=0; 
            qr[tail] = GET_R(x_pos[i]); qc[tail] = GET_C(x_pos[i]); tail++; bfs_dist_x[i][qr[0]][qc[0]] = 0;
            while(head < tail) {
                int r = qr[head], c = qc[head]; head++;
                for(int d=0; d<4; d++) {
                    int nr = r+dr[d], nc = c+dc[d]; 
                    if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                    if(global_base_wall[nr][nc] && !(7 & global_exp_cov[nr][nc])) continue; 
                    if(bfs_dist_x[i][nr][nc] == 9999) { bfs_dist_x[i][nr][nc] = bfs_dist_x[i][r][c] + 1; qr[tail]=nr; qc[tail]=nc; tail++; }
                }
            }
        }
        
        for(int r=0; r<MAX_R; r++) for(int c=0; c<MAX_C; c++) bfs_dist_dot[i][r][c] = 9999;
        if (i < init_dot_cnt) {
            int head=0, tail=0; uint16_t qr[512], qc[512];
            qr[tail] = GET_R(dot_pos[i]); qc[tail] = GET_C(dot_pos[i]); tail++; bfs_dist_dot[i][qr[0]][qc[0]] = 0;
            while(head < tail) {
                int r = qr[head], c = qc[head]; head++;
                for(int d=0; d<4; d++) {
                    int nr = r+dr[d], nc = c+dc[d]; 
                    if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                    if(global_base_wall[nr][nc] && !(7 & global_exp_cov[nr][nc])) continue;
                    if(bfs_dist_dot[i][nr][nc] == 9999) { bfs_dist_dot[i][nr][nc] = bfs_dist_dot[i][r][c] + 1; qr[tail]=nr; qc[tail]=nc; tail++; }
                }
            }
        }
    }

    struct State start; memset(&start, 0, sizeof(struct State));
    int b_idx = 0, bx_idx = 0;
    for (int r = 0; r < MAX_R; r++) for (int c = 0; c < MAX_C; c++) {
        char ch = original_map[r][c];
        if (ch == '@' || ch == '+') { start.r = r; start.c = c; }
        if (ch == '*') { start.b[b_idx++] = MAKE_POS(r, c); }
        if (ch == '$') { start.bx[bx_idx++] = MAKE_POS(r, c); }
    }
    
    for (int i = init_b_cnt; i < 3; i++) { start.b[i] = 255; start.mask |= (1 << i); }
    for (int i = init_bx_cnt; i < 3; i++) { start.bx[i] = 255; }
    
    current_hash_gen++; if (current_hash_gen == 0xFFFF) { memset(hash_gen, 0, sizeof(hash_gen)); current_hash_gen = 1; }
    heap_size = 0; node_cnt = 0;
    
    q[node_cnt].s = start; 
    q[node_cnt].g = 0; 
    q[node_cnt].h = get_h_p1(&start); 
    q[node_cnt].parent = -1; 
    q[node_cnt].action = -1;

    int unex = 0;
    for(int i=0; i<3; i++) { if(!(start.mask & (1<<i))) unex++; }
    int unweighted_h = (q[node_cnt].h / WEIGHT_FACTOR) - unex;
    
    if (current_best_metric < 999999.0f && (float)unweighted_h >= current_best_metric) { return -1.0f; }

    q_g[node_cnt] = 0;
    q_f[node_cnt] = q[node_cnt].h;
    hash_insert(encode(&start)); 
    push_heap(node_cnt++);
    int goal_idx = -1;
        
    while (heap_size > 0 && node_cnt < active_max_nodes - 10) {
        if ((node_cnt & 127) == 0) {
            if (GET_CURRENT_MS() - start_time > MAX_SOLVE_TIME_MS) return -1.0f; 
        }

        int curr_idx = pop_heap();
        struct State curr = q[curr_idx].s;
        if (curr.mask == 7) { goal_idx = curr_idx; break; }

        uint16_t solid_mask[MAX_R] = {0};
        for (int k = 0; k < 3; k++) {
            if (curr.b[k] != 255) solid_mask[curr.b[k] >> 4] |= (1 << (curr.b[k] & 15));
            if (curr.bx[k] != 255) solid_mask[curr.bx[k] >> 4] |= (1 << (curr.bx[k] & 15));
        }

        current_reach_id++;
        if (current_reach_id == 0xFFFF) { memset(reach_vis, 0, sizeof(reach_vis)); current_reach_id = 1; }

        uint8_t fast_q[256]; int head = 0, tail = 0; 
        fast_q[tail++] = (curr.r << 4) | curr.c; 
        reach_vis[curr.r][curr.c] = current_reach_id; global_dist[curr.r][curr.c] = 0;
        
        while(head < tail) {
            uint8_t val = fast_q[head++]; 
            int r = val >> 4, c = val & 15;
            bool (*cur_wall)[MAX_C] = fast_wall[curr.mask];

            for(int d=0; d<4; d++) {
                int nr = r+dr[d], nc = c+dc[d];
                if (cur_wall[nr][nc]) continue; 
                if (!(solid_mask[nr] & (1 << nc)) && reach_vis[nr][nc] != current_reach_id) { 
                    reach_vis[nr][nc] = current_reach_id;
                    global_dist[nr][nc] = global_dist[r][c] + 1; fast_q[tail++] = (nr << 4) | nc;
                }
            }
        }

        for (int item_idx = 0; item_idx < 6; item_idx++) {
            int item_pos = (item_idx < 3) ? curr.b[item_idx] : curr.bx[item_idx - 3];
            if (item_pos == 255) continue;
            int nr = GET_R(item_pos), nc = GET_C(item_pos);

            for (int i = 0; i < 4; i++) {
                int r = nr - dr[i], c = nc - dc[i];
                if (r < 0 || r >= MAX_R || c < 0 || c >= MAX_C || reach_vis[r][c] != current_reach_id) continue;

                int nnr = nr + dr[i], nnc = nc + dc[i];
                if (nnr < 0 || nnr >= MAX_R || nnc < 0 || nnc >= MAX_C) continue;
                int nnpos = MAKE_POS(nnr, nnc);

                struct State next = curr; next.r = nr; next.c = nc;
                int action_penalty = 0;
                
                if (item_idx < 3) { 
                    int hit_b = item_idx; bool hit_x = false;
                    for(int k=0; k<3; k++) {
                        if(nnpos == x_pos[k] && !(curr.mask & (1<<k))) { 
                            next.b[hit_b]=255; next.mask|=(1<<k); hit_x=true; break; 
                        }
                    }
                    bool hit_other = (solid_mask[nnr] & (1 << nnc)) != 0;
                    if (hit_other || (!hit_x && fast_wall[curr.mask][nnr][nnc])) continue;
                    
                    if (!hit_x) {
                        bool can_reach = false;
                        for (int tk = 0; tk < 3; tk++) {
                            if (!(curr.mask & (1 << tk))) { if (bfs_dist_x[tk][nnr][nnc] != 9999) { can_reach = true; break; } }
                        }
                        if (!can_reach) continue;
                        next.b[hit_b] = nnpos;
                    }
                    action_penalty = 0;
                } else {
                    int hit_bx = item_idx - 3;
                    if (curr.nudges >= MAX_NUDGES) continue; 
                    next.nudges++;
                    bool hit_other = (solid_mask[nnr] & (1 << nnc)) != 0;
                    if (hit_other || fast_wall[curr.mask][nnr][nnc] || deadlock_map[nnr][nnc]) continue;
                    next.bx[hit_bx] = nnpos; action_penalty = 10; 
                }
                
                uint32_t key = encode(&next);
                if (hash_insert(key)) {
                    if (node_cnt >= active_max_nodes) continue;
                    q[node_cnt].s = next;
                    q[node_cnt].parent = curr_idx; q[node_cnt].action = (r << 16) | (c << 8) | i;
                    
                    int new_g = q_g[curr_idx] + global_dist[r][c] + 1 + action_penalty;
                    int new_h = get_h_p1(&next);
                    q_g[node_cnt] = new_g;               
                    q_f[node_cnt] = new_g + new_h;       
                    push_heap(node_cnt++);
                }
            }
        }
    }
    t_p1_astar += get_time_s(start_p1, GET_CURRENT_MS());

    uint32_t start_p2 = GET_CURRENT_MS();
    if (goal_idx == -1) return -1.0f;

    int16_t raw_acts[MAX_STEPS]; int raw_len = 0;
    int16_t p1_path[MAX_STEPS]; int p1_macro_len=0, curr_node=goal_idx;
    while(q[curr_node].parent != -1) { p1_path[p1_macro_len++] = curr_node; curr_node = q[curr_node].parent; }
    
    for(int i = p1_macro_len - 1; i >= 0; i--) {
        int act = q[p1_path[i]].action;
        int walk_r = act>>16, walk_c=(act>>8)&0xFF, push_dir=act&0xFF;
        struct State prev = q[q[p1_path[i]].parent].s;
        
        p1_dist_vis_id++;
        if(p1_dist_vis_id == 0xFFFF) { memset(p1_dist_vis, 0, sizeof(p1_dist_vis)); p1_dist_vis_id = 1; }
        
        uint16_t qr[512], qc[512]; int h=0, t=0; qr[t]=prev.r; qc[t++]=prev.c; 
        global_dist[prev.r][prev.c] = 0; p1_dist_vis[prev.r][prev.c] = p1_dist_vis_id;
        
        while(h<t) {
            int r=qr[h], c=qc[h++]; if(r==walk_r && c==walk_c) break;
            for(int d=0; d<4; d++) {
                int nr=r+dr[d], nc=c+dc[d];
                if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                if(!fast_wall[prev.mask][nr][nc] && p1_dist_vis[nr][nc] != p1_dist_vis_id) {
                    bool block=false; for(int k=0;k<3;k++) if((prev.b[k]!=255 && prev.b[k]==MAKE_POS(nr,nc)) || (prev.bx[k]!=255 && prev.bx[k]==MAKE_POS(nr,nc))) block=true;
                    if(!block) { 
                        p1_dist_vis[nr][nc] = p1_dist_vis_id;
                        global_dist[nr][nc] = global_dist[r][c]+1; 
                        global_parent[nr][nc] = MAKE_POS(r,c); global_action[nr][nc] = d; 
                        qr[t]=nr; qc[t++]=nc; 
                    }
                }
            }
        }
        int cr=walk_r, cc=walk_c; int16_t steps[512]; int step_cnt=0;
        while(cr!=prev.r || cc!=prev.c) { steps[step_cnt++]=global_action[cr][cc]; int p=global_parent[cr][cc]; cr=GET_R(p); cc=GET_C(p); }
        for(int x=step_cnt-1; x>=0; x--) raw_acts[raw_len++] = steps[x];
        raw_acts[raw_len++] = push_dir;
        if (q[p1_path[i]].s.mask != prev.mask) raw_acts[raw_len++] = 4;
    }

    struct SimState cur_sim; cur_sim.r=start.r; cur_sim.c=start.c; cur_sim.mask=start.mask; 
    for(int i=0;i<3;i++){ cur_sim.b[i]=start.b[i]; cur_sim.bx[i]=start.bx[i]; }
    
    obs_b = 0; obs_t = 0; p1_len = 0;
    
    // ★ 严格开局控制：默认朝向为正(0方向)
    int current_look_dir = 0; 
    if (skip_flag == 0) {
        try_free_look(&cur_sim, &current_look_dir);
    }

    for(int i=0; i<raw_len; i++) {
        p1_states[p1_len] = cur_sim; p1_acts[p1_len++] = raw_acts[i];
        int act = raw_acts[i];
        if (act >= 0 && act <= 3) {
            int nr=cur_sim.r+dr[act], nc=cur_sim.c+dc[act], npos=MAKE_POS(nr,nc);
            for(int k=0; k<3; k++) {
                if (cur_sim.b[k] == npos) {
                    int nnpos = MAKE_POS(nr+dr[act], nc+dc[act]); bool exp=false;
                    for(int x=0;x<3;x++) if(x_pos[x]==nnpos && !(cur_sim.mask&(1<<x))) { cur_sim.mask|=(1<<x); cur_sim.b[k]=255; exp=true; break; }
                    if(!exp) cur_sim.b[k]=nnpos;
                }
                if (cur_sim.bx[k] == npos) cur_sim.bx[k] = MAKE_POS(nr+dr[act], nc+dc[act]);
            }
            cur_sim.r = nr; cur_sim.c = nc;
        }
    }
    p1_states[p1_len] = cur_sim;

    // ==================== ★ 新增 skip_flag 控制逻辑 ====================
    if (skip_flag == 0) {
        // ==================== ★ 无炸弹走带严格惩罚的 TSP，有炸弹走原贪心逻辑 ★ ====================
        if (init_b_cnt == 0) {
            tsp_poi_cnt = 1;
            tsp_pois[0].r = cur_sim.r; tsp_pois[0].c = cur_sim.c;
            tsp_pois[0].b_mask = obs_b; tsp_pois[0].t_mask = obs_t;

            for (int r = 0; r < MAX_R; r++) {
                for (int c = 0; c < MAX_C; c++) {
                    if (fast_wall[cur_sim.mask][r][c]) continue;
                    bool is_solid = false;
                    for(int k=0; k<3; k++) if(cur_sim.bx[k] != 255 && cur_sim.bx[k] == MAKE_POS(r,c)) is_solid = true;
                    if (is_solid) continue;

                    int b_mask = 0, t_mask = 0;
                    for (int d = 0; d < 4; d++) {
                        int nr = r + dr[d], nc = c + dc[d];
                        int npos = MAKE_POS(nr, nc);
                        for(int k=0; k<3; k++) {
                            if (!(obs_b & (1<<k)) && cur_sim.bx[k] == npos) b_mask |= (1 << k);
                            if (!(obs_t & (1<<k)) && dot_pos[k] == npos) t_mask |= (1 << k);
                        }
                    }
                    if (b_mask || t_mask) {
                        bool dup = false;
                        if (r == tsp_pois[0].r && c == tsp_pois[0].c) {
                            tsp_pois[0].b_mask |= b_mask; tsp_pois[0].t_mask |= t_mask; dup = true;
                        }
                        if (!dup && tsp_poi_cnt < 64) {
                            tsp_pois[tsp_poi_cnt].r = r; tsp_pois[tsp_poi_cnt].c = c;
                            tsp_pois[tsp_poi_cnt].b_mask = b_mask; tsp_pois[tsp_poi_cnt].t_mask = t_mask;
                            tsp_poi_cnt++;
                        }
                    }
                }
            }

            for (int i = 0; i < tsp_poi_cnt; i++) {
                for (int j = 0; j < tsp_poi_cnt; j++) tsp_dist_matrix[i][j] = 9999;
                tsp_dist_matrix[i][i] = 0;

                p1_dist_vis_id++;
                if (p1_dist_vis_id == 0xFFFF) { memset(p1_dist_vis, 0, sizeof(p1_dist_vis)); p1_dist_vis_id = 1; }

                uint8_t qr[256], qc[256]; int head = 0, tail = 0;
                qr[tail] = tsp_pois[i].r; qc[tail++] = tsp_pois[i].c;
                p1_dist_vis[tsp_pois[i].r][tsp_pois[i].c] = p1_dist_vis_id;
                global_dist[tsp_pois[i].r][tsp_pois[i].c] = 0;

                while(head < tail) {
                    int r = qr[head], c = qc[head++];
                    for(int j=0; j<tsp_poi_cnt; j++) {
                        if (tsp_pois[j].r == r && tsp_pois[j].c == c) tsp_dist_matrix[i][j] = global_dist[r][c];
                    }
                    for(int d=0; d<4; d++) {
                        int nr = r + dr[d], nc = c + dc[d];
                        if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                        if (fast_wall[cur_sim.mask][nr][nc]) continue;

                        bool solid = false;
                        for(int k=0;k<3;k++) if(cur_sim.bx[k]!=255 && cur_sim.bx[k]==MAKE_POS(nr,nc)) solid = true;
                        if(!solid && p1_dist_vis[nr][nc] != p1_dist_vis_id) {
                            p1_dist_vis[nr][nc] = p1_dist_vis_id;
                            global_dist[nr][nc] = global_dist[r][c] + 1;
                            qr[tail]=nr; qc[tail++]=nc;
                        }
                    }
                }
            }

            tsp_min_cost = 999999;
            tsp_best_len = 0;
            memset(tsp_visited, 0, sizeof(tsp_visited));
            tsp_visited[0] = true;
            int temp_path[32];
            
            // ★ 将初始 current_look_dir 传入作为初始状态
            dfs_tsp_0bomb(0, 0, obs_b, obs_t, 0, temp_path, init_bx_cnt, init_dot_cnt, &cur_sim, current_look_dir);

            if (tsp_min_cost == 999999) return -1.0f; 

            int curr_node = 0;
            for (int i = 0; i < tsp_best_len; i++) {
                int next_node = tsp_best_path[i];

                p1_dist_vis_id++;
                if (p1_dist_vis_id == 0xFFFF) { memset(p1_dist_vis, 0, sizeof(p1_dist_vis)); p1_dist_vis_id = 1; }
                uint8_t qr[256], qc[256]; int h = 0, t = 0;
                qr[t] = tsp_pois[curr_node].r; qc[t++] = tsp_pois[curr_node].c;
                p1_dist_vis[tsp_pois[curr_node].r][tsp_pois[curr_node].c] = p1_dist_vis_id;

                while(h < t) {
                    int r = qr[h], c = qc[h++];
                    if (r == tsp_pois[next_node].r && c == tsp_pois[next_node].c) break;
                    for(int d=0; d<4; d++) {
                        int nr = r + dr[d], nc = c + dc[d];
                        if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                        if (fast_wall[cur_sim.mask][nr][nc]) continue;
                        bool solid = false;
                        for(int k=0;k<3;k++) if(cur_sim.bx[k]!=255 && cur_sim.bx[k]==MAKE_POS(nr,nc)) solid = true;
                        if(!solid && p1_dist_vis[nr][nc] != p1_dist_vis_id) {
                            p1_dist_vis[nr][nc] = p1_dist_vis_id;
                            global_parent[nr][nc] = MAKE_POS(r, c);
                            global_action[nr][nc] = d;
                            qr[t]=nr; qc[t++]=nc;
                        }
                    }
                }

                int16_t walk_path[256]; int walk_len = 0;
                int cr = tsp_pois[next_node].r, cc = tsp_pois[next_node].c;
                while(cr != tsp_pois[curr_node].r || cc != tsp_pois[curr_node].c) {
                    walk_path[walk_len++] = global_action[cr][cc];
                    int p = global_parent[cr][cc]; cr = GET_R(p); cc = GET_C(p);
                }

                for (int j = walk_len - 1; j >= 0; j--) {
                    p1_states[p1_len] = cur_sim;
                    p1_acts[p1_len++] = walk_path[j];
                    cur_sim.r += dr[walk_path[j]];
                    cur_sim.c += dc[walk_path[j]];
                }

                // ★ 还原执行动作：巧妙地根据是否有当前方向来决定先后记录顺序
                int need_b, need_t;
                observation_requirements(init_bx_cnt, init_dot_cnt,
                                         &need_b, &need_t);
                int add_b = limit_observation_bits(obs_b,
                                                   (uint16_t)tsp_pois[next_node].b_mask,
                                                   need_b);
                int add_t = limit_observation_bits(obs_t,
                                                   (uint16_t)tsp_pois[next_node].t_mask,
                                                   need_t);
                
                int req_looks[4], req_cnt = 0;
                for(int d=0; d<4; d++) {
                    int nr = cur_sim.r + dr[d], nc = cur_sim.c + dc[d];
                    int npos = MAKE_POS(nr, nc);
                    bool need_look = false;
                    for(int k=0; k<3; k++) {
                        if ((add_b & (1<<k)) && cur_sim.bx[k] == npos) need_look = true;
                        if ((add_t & (1<<k)) && dot_pos[k] == npos) need_look = true;
                    }
                    if (need_look) req_looks[req_cnt++] = d;
                }
                
                if (req_cnt > 0) {
                    bool has_curr = false;
                    for(int x=0; x<req_cnt; x++) if(req_looks[x] == current_look_dir) has_curr = true;
                    
                    // 如果刚好能用上现在的朝向，则直接先看，避免惩罚
                    if (has_curr) {
                        p1_states[p1_len] = cur_sim;
                        p1_acts[p1_len++] = 5 + current_look_dir;
                    }
                    
                    // 然后再逐一转头看其他的，把最后的视角赋值给 current_look_dir
                    for(int x=0; x<req_cnt; x++) {
                        if (req_looks[x] == current_look_dir) continue;
                        p1_states[p1_len] = cur_sim;
                        p1_acts[p1_len++] = 5 + req_looks[x];
                        current_look_dir = req_looks[x]; // 更新物理底盘方向
                    }
                    obs_b |= add_b;
                    obs_t |= add_t;
                }
                curr_node = next_node;
            }
            p1_states[p1_len] = cur_sim;
        } else {
            // ==================== 原有的贪心算法 (保护带炸弹时的既有逻辑，自带惩罚处理) ====================
            while(1) {
                int req_b = init_bx_cnt, req_t = init_dot_cnt;
                if (observations_satisfied(obs_b, obs_t, req_b, req_t)) break;

                int best_cost = 999999, best_idx = -1, best_path_len = 0;
                int best_see_type = -1, best_see_id = -1; int16_t best_look_act = -1;
                int16_t best_path[512];
                int max_quota_b, max_quota_t;
                observation_requirements(req_b, req_t,
                                         &max_quota_b, &max_quota_t);
                int cur_cb = count_bits(obs_b), cur_ct = count_bits(obs_t);

                for(int i=0; i<=p1_len; i++) { 
                    struct SimState st = p1_states[i];
                    p1_dist_vis_id++;
                    if(p1_dist_vis_id == 0xFFFF) { memset(p1_dist_vis, 0, sizeof(p1_dist_vis)); p1_dist_vis_id = 1; }
                    
                    uint16_t qr[512], qc[512]; int h=0, t=0; qr[t]=st.r; qc[t++]=st.c; 
                    global_dist[st.r][st.c]=0; p1_dist_vis[st.r][st.c] = p1_dist_vis_id;
                    while(h < t) {
                        int cr=qr[h], cc=qc[h++];
                        for(int d=0; d<4; d++) {
                            int nr=cr+dr[d], nc=cc+dc[d];
                            if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                            if (fast_wall[st.mask][nr][nc]) continue;
                            bool solid = false;
                            for(int k=0;k<3;k++) if((st.b[k]!=255 && st.b[k]==MAKE_POS(nr,nc)) || (st.bx[k]!=255 && st.bx[k]==MAKE_POS(nr,nc))) solid = true;
                            if(!solid && p1_dist_vis[nr][nc] != p1_dist_vis_id) { 
                                p1_dist_vis[nr][nc] = p1_dist_vis_id;
                                global_dist[nr][nc] = global_dist[cr][cc]+1; 
                                global_parent[nr][nc] = MAKE_POS(cr,cc); qr[t]=nr; qc[t++]=nc; 
                            }
                        }
                    }
                    
                    for(int k=0; k<3; k++) {
                        int check_pts[2][2] = {{-1,-1}, {-1,-1}}; 
                        int check_type[2] = {-1, -1}; 
                        int pts_cnt = 0;
                        
                        if(st.bx[k]!=255 && !(obs_b & (1<<k)) && cur_cb < max_quota_b) { 
                            check_pts[pts_cnt][0] = GET_R(st.bx[k]); check_pts[pts_cnt][1] = GET_C(st.bx[k]); check_type[pts_cnt] = 0; pts_cnt++; 
                        }
                        if(dot_pos[k]!=255 && !(obs_t & (1<<k)) && cur_ct < max_quota_t) { 
                            check_pts[pts_cnt][0] = GET_R(dot_pos[k]); check_pts[pts_cnt][1] = GET_C(dot_pos[k]); check_type[pts_cnt] = 1; pts_cnt++; 
                        }
                        
                        for(int pt=0; pt<pts_cnt; pt++) {
                            int tr = check_pts[pt][0], tc = check_pts[pt][1];
                            for(int look=0; look<4; look++) {
                                int ld = p_dir[look], ar=tr-dr[ld], ac=tc-dc[ld];
                                if(ar>=0 && ar<MAX_R && ac>=0 && ac<MAX_C && p1_dist_vis[ar][ac] == p1_dist_vis_id) {
                                    // ★ 这里自带了惩罚机制！
                                    int posture_penalty = (ld == current_look_dir) ? 0 : 5;
                                    int back_dist = (i == p1_len) ? 0 : global_dist[ar][ac];
                                    int cost = (global_dist[ar][ac] + posture_penalty + 1 + back_dist) * 1000 - i;
                                                                                                                    
                                    if(cost < best_cost) {
                                        best_cost = cost; best_idx = i; best_see_type = check_type[pt]; best_see_id = k; best_look_act = 5+ld;
                                        best_path_len = 0; int cr=ar, cc=ac;
                                        while(cr!=st.r || cc!=st.c) {
                                            int p = global_parent[cr][cc], pr=GET_R(p), pc=GET_C(p);
                                            for(int dir=0; dir<4; dir++) if(pr+dr[dir]==cr && pc+dc[dir]==cc) { best_path[best_path_len++]=dir; break; }
                                            cr=pr; cc=pc;
                                        }
                                        for(int x=0; x<best_path_len/2; x++) { int16_t tmp=best_path[x]; best_path[x]=best_path[best_path_len-1-x]; best_path[best_path_len-1-x]=tmp; }
                                    }
                                }
                            }
                        }
                    }
                }
                if (best_idx == -1) break;
                
                struct SimState det_st = p1_states[best_idx]; 
                int jump_r = det_st.r, jump_c = det_st.c; 
                for(int j=0; j<best_path_len; j++) { det_st.r+=dr[best_path[j]]; det_st.c+=dc[best_path[j]]; }
                
                int16_t back_path[512]; int back_len = 0;
                if (best_idx < p1_len) {
                    p1_dist_vis_id++;
                    if (p1_dist_vis_id == 0xFFFF) { memset(p1_dist_vis, 0, sizeof(p1_dist_vis)); p1_dist_vis_id = 1; }
                    uint16_t qr[256], qc[256]; int h=0, t=0; qr[t]=det_st.r; qc[t++]=det_st.c; 
                    global_dist[det_st.r][det_st.c] = 0; p1_dist_vis[det_st.r][det_st.c] = p1_dist_vis_id;
                    while(h < t) {
                        int r=qr[h], c=qc[h++];
                        if(r==jump_r && c==jump_c) {
                            back_len = global_dist[jump_r][jump_c]; int cr=jump_r, cc=jump_c;
                            for(int i=back_len-1; i>=0; i--) { back_path[i] = global_action[cr][cc]; int p = global_parent[cr][cc]; cr=GET_R(p); cc=GET_C(p); }
                            break;
                        }
                        for(int d=0; d<4; d++) {
                            int nr=r+dr[d], nc=c+dc[d];
                            if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                            if (fast_wall[det_st.mask][nr][nc]) continue; 
                            bool blocked = false; 
                            for(int k=0; k<3; k++) if((det_st.b[k]!=255 && det_st.b[k]==MAKE_POS(nr,nc)) || (det_st.bx[k]!=255 && det_st.bx[k]==MAKE_POS(nr,nc))) blocked = true;
                            if(!blocked && p1_dist_vis[nr][nc] != p1_dist_vis_id) { 
                                p1_dist_vis[nr][nc] = p1_dist_vis_id;
                                global_dist[nr][nc] = global_dist[r][c]+1; 
                                global_parent[nr][nc] = MAKE_POS(r,c); global_action[nr][nc] = d; 
                                qr[t]=nr; qc[t++]=nc; 
                            }
                        }
                    }
                }
                
                int inject_len = best_path_len + 1 + back_len;
                for(int j=p1_len; j>=best_idx; j--) { p1_acts[j+inject_len]=p1_acts[j]; p1_states[j+inject_len]=p1_states[j]; }
                
                int p_idx = best_idx; det_st = p1_states[best_idx]; 
                for(int j=0; j<best_path_len; j++) { p1_states[p_idx]=det_st; p1_acts[p_idx++]=best_path[j]; det_st.r+=dr[best_path[j]]; det_st.c+=dc[best_path[j]]; }
                p1_states[p_idx] = det_st; p1_acts[p_idx++] = best_look_act;
                
                current_look_dir = best_look_act - 5; // 更新物理朝向
                
                for(int j=0; j<back_len; j++) { p1_states[p_idx]=det_st; p1_acts[p_idx++]=back_path[j]; det_st.r+=dr[back_path[j]]; det_st.c+=dc[back_path[j]]; }
                
                if (best_idx == p1_len) p1_states[p1_len + inject_len] = det_st;
                p1_len += inject_len;
                if(best_see_type==0) obs_b |= (1<<best_see_id); else obs_t |= (1<<best_see_id);
            }
        }
    }

    if (final_st) *final_st = p1_states[p1_len];

    int p1_turns = count_path_turns(p1_acts, p1_len);
    float p1_score = (float)p1_len + 3.0f * p1_turns;
    
    int cur_assign[3] = {BOX_TARGET_MAP[0], BOX_TARGET_MAP[1], BOX_TARGET_MAP[2]};
    float best_score_for_assign = 999999.0f; 
    int acts_len_for_assign = 0;
    static int16_t best_acts_for_assign[MAX_STEPS];

    for (int ord = 0; ord < 6; ord++) {
        struct SimState s2 = p1_states[p1_len];
        static int16_t cur_acts[MAX_STEPS]; int cur_len = 0; bool possible = true;
        
        for (int step = 0; step < 3; step++) {
            int b_id = perms[ord][step];
            if (s2.bx[b_id] == 255) continue; 
            
            int t_id = cur_assign[b_id]; 
            int tr = GET_R(dot_pos[t_id]), tc = GET_C(dot_pos[t_id]);
            
            p2_vis_id++;
            if (p2_vis_id == 0xFFFF) { memset(p2_vis, 0, sizeof(p2_vis)); p2_vis_id = 1; }
            
            // ★ 初始化 P2 A* 队列
            p2_heap_size = 0;
            int t = 0; 
            
            rq_p2[t].rr = s2.r; rq_p2[t].rc = s2.c; 
            rq_p2[t].br = GET_R(s2.bx[b_id]); rq_p2[t].bc = GET_C(s2.bx[b_id]); 
            rq_p2[t].parent = -1; rq_p2[t].action = -1; 
            rq_p2[t].dist = 0; 
            
            // 初始节点的 f 值
            p2_f[t] = 0; 
            p2_vis[s2.r][s2.c][rq_p2[t].br][rq_p2[t].bc] = p2_vis_id;
            
            push_p2(t++); // 压入起点
            
            int goal = -1;
            
            // ★ 核心替换：从 BFS 变成 A*
            while (p2_heap_size > 0) {
                int curr_idx = pop_p2(); // 弹出 f 值最小的节点
                struct QN curr = rq_p2[curr_idx];
                
                if (curr.br == tr && curr.bc == tc) { goal = curr_idx; break; }
                
                for (int d = 0; d < 4; d++) {
                    int nrr = curr.rr + dr[d], nrc = curr.rc + dc[d], nbr = curr.br, nbc = curr.bc;
                    
                    if (nrr < 0 || nrr >= MAX_R || nrc < 0 || nrc >= MAX_C) continue;
                    if (global_base_wall[nrr][nrc] && !(7 & global_exp_cov[nrr][nrc])) continue;
                    
                    bool blocked = false; 
                    for (int k = 0; k < 3; k++) {
                        if (k != b_id && s2.bx[k] != 255 && s2.bx[k] == MAKE_POS(nrr, nrc)) blocked = true;
                    }
                    if (blocked) continue;
                    
                    // 机器人碰到了箱子，发生推动
                    if (nrr == curr.br && nrc == curr.bc) {
                        nbr += dr[d]; nbc += dc[d]; 
                        if (nbr < 0 || nbr >= MAX_R || nbc < 0 || nbc >= MAX_C) continue;
                        if (global_base_wall[nbr][nbc] && !(7 & global_exp_cov[nbr][nbc])) continue;
                        for (int k = 0; k < 3; k++) {
                            if (k != b_id && s2.bx[k] != 255 && s2.bx[k] == MAKE_POS(nbr, nbc)) blocked = true;
                        }
                        if (blocked) continue;
                    }
                    
                    if (p2_vis[nrr][nrc][nbr][nbc] != p2_vis_id) { 
                        int new_dist = curr.dist + 1;
                        if (cur_len + new_dist > best_score_for_assign) continue; // 全局最优剪枝

                        p2_vis[nrr][nrc][nbr][nbc] = p2_vis_id; 
                        
                        if (t < MAX_P2_Q) {
                            rq_p2[t].rr = nrr; rq_p2[t].rc = nrc; rq_p2[t].br = nbr; rq_p2[t].bc = nbc;
                            rq_p2[t].parent = curr_idx; rq_p2[t].action = d; 
                            rq_p2[t].dist = new_dist; 
                            
                            // ==================== ★ A* 启发式计算 ★ ====================
                            // 1. 获取预计算的箱子到目标的真实走廊距离（完美屏蔽了死胡同）
                            int h_box = bfs_dist_dot[t_id][nbr][nbc];
                            if (h_box == 9999) continue; // 如果箱子被推到了死角，直接剪枝！
                            
                            // 2. 计算机器人到箱子的曼哈顿距离 (破局点：引导机器人快速贴近箱子)
                            int h_robot = abs(nrr - nbr) + abs(nrc - nbc);
                            
                            // f = g + h_box * 权值 + h_robot
                            // 给 h_box 乘 3 是一种贪心倾向，让 A* 更像 Best-First Search，速度极快
                            p2_f[t] = new_dist + h_box * 1 + h_robot;
                            
                            push_p2(t++);
                        }
                    }
                }
            }
            if (goal == -1) { possible = false; break; } 
            
            int16_t temp_p[1000]; int t_len = 0, c_ptr = goal;
            while (rq_p2[c_ptr].parent != -1) { temp_p[t_len++] = rq_p2[c_ptr].action; c_ptr = rq_p2[c_ptr].parent; }
            for (int x = t_len - 1; x >= 0; x--) cur_acts[cur_len++] = temp_p[x];
            s2.r = rq_p2[goal].rr; s2.c = rq_p2[goal].rc; s2.bx[b_id] = 255; 
        }
        
        if (possible) {
            int cur_turns = count_path_turns(cur_acts, cur_len);
            float cur_score = (float)cur_len + 4.0f * cur_turns;
            if (cur_score < best_score_for_assign) { 
                best_score_for_assign = cur_score; 
                acts_len_for_assign = cur_len; 
                memcpy(best_acts_for_assign, cur_acts, sizeof(int16_t) * cur_len); 
            }
        }
    }

    if (best_score_for_assign >= 999999.0f) return -1.0f; 

    remove_duplicate_looks(p1_acts, &p1_len);
    *out_p1_steps = p1_len;

    temp_p2_len = acts_len_for_assign;
    memcpy(temp_p2_acts, best_acts_for_assign, sizeof(int16_t) * acts_len_for_assign);
    t_p2_bfs += get_time_s(start_p2, GET_CURRENT_MS());
    return p1_score + best_score_for_assign; 
}

static int get_dynamic_bx_dist(int16_t bombs[3][2], int b_cnt) {
    bool wall[MAX_R][MAX_C];
    for(int r=0; r<MAX_R; r++) for(int c=0; c<MAX_C; c++) wall[r][c] = (original_map[r][c] == '#');
    for(int i=0; i<b_cnt; i++) {
        int br = bombs[i][0], bc = bombs[i][1];
        for(int dr_i=-1; dr_i<=1; dr_i++) for(int dc_i=-1; dc_i<=1; dc_i++) {
            int nr = br + dr_i, nc = bc + dc_i;
            if(nr > 0 && nr < MAX_R-1 && nc > 0 && nc < MAX_C-1) wall[nr][nc] = false;
        }
    }

    int total_dist = 0;
    for(int i=0; i<init_bx_cnt; i++) {
        bool vis[MAX_R][MAX_C] = {false};
        uint16_t qr[256], qc[256]; int16_t dist[256]; int head = 0, tail = 0;
        
        qr[tail] = init_bx[i][0]; qc[tail] = init_bx[i][1]; dist[tail] = 0; 
        vis[qr[tail]][qc[tail]] = true; tail++;
        
        int target_r = init_dot[i][0], target_c = init_dot[i][1];
        int min_d = 9999;

        while(head < tail) {
            int r = qr[head], c = qc[head]; int16_t d = dist[head]; head++;
            if (r == target_r && c == target_c) { min_d = d; break; }
            for(int dir=0; dir<4; dir++) {
                int nr = r + dr[dir], nc = c + dc[dir];
                if (nr>=0 && nr<MAX_R && nc>=0 && nc<MAX_C && !wall[nr][nc] && !vis[nr][nc]) { 
                    vis[nr][nc] = true; qr[tail] = nr; qc[tail] = nc; dist[tail] = d + 1; tail++; 
                }
            }
        }
        if (min_d == 9999) return 9999;
        total_dist += min_d;
    }
    return total_dist;
}

void bomb_solver_reset(void)
{
    t_setup_and_hull = 0.0;
    t_filter1_bfs = 0.0;
    t_filter3_dist = 0.0;
    t_filter2_prune = 0.0;
    t_p1_astar = 0.0;
    t_p2_bfs = 0.0;
    p1_exec_cnt = 0;
    p2_exec_cnt = 0;
    g_best_p1_len = 0;
    g_best_p2_len = 0;
    g_best_obs_b = 0;
    g_best_obs_t = 0;
    temp_p2_len = 0;
    obs_b = 0;
    obs_t = 0;
    p1_len = 0;
    base_wall_initialized = false;
}

bool bomb_solver_run_phase1(
    const char raw_map[BOMB_TRACK_ROWS][BOMB_TRACK_COLS],
    bomb_phase_result_t *p1_res, int skip_flag) {
    if(p1_res == NULL) return false;
    memset(p1_res, 0, sizeof(*p1_res));
    bomb_solver_reset();
    uint32_t t0 = GET_CURRENT_MS();  
    Coord key_pts[30], hull[30]; int key_cnt = 0;
    init_b_cnt = 0; init_bx_cnt = 0; init_dot_cnt = 0;
    
    base_wall_initialized = false;
    for (int r = 0; r < MAX_R; r++) {
        for (int c = 0; c < MAX_C; c++) {
            char ch = raw_map[r][c]; original_map[r][c] = ch;
            global_base_wall[r][c] = (ch == '#');
            if (ch == '@' || ch == '+') { init_robot_r = r; init_robot_c = c; }
            // 防越界加载并真实计数
            if (ch == '*' && init_b_cnt < MAX_ITEMS) { init_b[init_b_cnt][0] = r; init_b[init_b_cnt++][1] = c; }
            if (ch == '$' && init_bx_cnt < MAX_ITEMS) { init_bx[init_bx_cnt][0] = r; init_bx[init_bx_cnt++][1] = c; }
            if ((ch == '.' || ch == '+') && init_dot_cnt < MAX_ITEMS) { init_dot[init_dot_cnt][0] = r; init_dot[init_dot_cnt++][1] = c; }
            if (ch == '@' || ch == '+' || ch == '*' || ch == '$' || ch == '.') key_pts[key_cnt++] = (Coord){r, c};
        }
    }
    
    for(int i=0; i<init_b_cnt; i++) {
        for(int r=0; r<MAX_R; r++) for(int c=0; c<MAX_C; c++) relaxed_dist_b[i][r][c] = 9999;
        uint16_t qr[256], qc[256]; int head=0, tail=0;
        qr[tail] = init_b[i][0]; qc[tail] = init_b[i][1]; tail++;
        relaxed_dist_b[i][init_b[i][0]][init_b[i][1]] = 0;
        
        while(head < tail) {
            int r = qr[head], c = qc[head]; head++;
            for(int d=0; d<4; d++) {
                int nr = r+dr[d], nc = c+dc[d];
                if (nr>=0 && nr<MAX_R && nc>=0 && nc<MAX_C) {
                    bool is_border_wall = (nr==0 || nr==MAX_R-1 || nc==0 || nc==MAX_C-1) && global_base_wall[nr][nc];
                    if (!is_border_wall && relaxed_dist_b[i][nr][nc] == 9999) {
                        relaxed_dist_b[i][nr][nc] = relaxed_dist_b[i][r][c] + 1;
                        qr[tail]=nr; qc[tail]=nc; tail++;
                    }
                }
            }
        }
    }
    base_wall_initialized = true;

    for (int i = 0; i < init_dot_cnt; i++) dot_pos[i] = MAKE_POS(init_dot[i][0], init_dot[i][1]);
    for (int i = init_dot_cnt; i < 3; i++) dot_pos[i] = 255;
    dot_cnt = init_dot_cnt;
    
    int hull_size = get_convex_hull(key_pts, key_cnt, hull);
    int16_t pool[192][2]; int pool_size = 0;
    for (int r = 1; r < MAX_R - 1; r++) {
        for (int c = 1; c < MAX_C - 1; c++) {
            if (original_map[r][c] == '#') {
                if (is_wall_intersect_hull(r, c, hull, hull_size)) { pool[pool_size][0] = r; pool[pool_size][1] = c; pool_size++; }
            }
        }
    }

    /* Degenerate hulls can omit a wall immediately pushable by a bomb. */
    for(int bi = 0; bi < init_b_cnt; bi++) {
        for(int d = 0; d < 4; d++) {
            int nr = init_b[bi][0] + dr[d];
            int nc = init_b[bi][1] + dc[d];
            int sr = init_b[bi][0] - dr[d];
            int sc = init_b[bi][1] - dc[d];
            bool exists = false;
            if(nr < 1 || nr >= MAX_R - 1 || nc < 1 || nc >= MAX_C - 1) continue;
            if(original_map[nr][nc] != '#') continue;
            if(sr < 0 || sr >= MAX_R || sc < 0 || sc >= MAX_C) continue;
            if(original_map[sr][sc] == '#') continue;
            for(int pi = 0; pi < pool_size; pi++) {
                if(pool[pi][0] == nr && pool[pi][1] == nc) { exists = true; break; }
            }
            if(!exists && pool_size < 192) {
                pool[pool_size][0] = (int16_t)nr;
                pool[pool_size][1] = (int16_t)nc;
                pool_size++;
            }
        }
    }

    /* Try walls near bombs/boxes/goals first so a useful upper bound appears early. */
    for(int i = 1; i < pool_size; i++) {
        int16_t wr = pool[i][0], wc = pool[i][1];
        int key_score = 9999;
        int j;
        for(int bi = 0; bi < init_b_cnt; bi++) {
            int v = (abs(wr - init_b[bi][0]) + abs(wc - init_b[bi][1])) * 3;
            if(v < key_score) key_score = v;
        }
        for(int bi = 0; bi < init_bx_cnt; bi++) {
            int v = abs(wr - init_bx[bi][0]) + abs(wc - init_bx[bi][1]);
            if(v < key_score) key_score = v;
        }
        for(int gi = 0; gi < init_dot_cnt; gi++) {
            int v = abs(wr - init_dot[gi][0]) + abs(wc - init_dot[gi][1]);
            if(v < key_score) key_score = v;
        }
        for(j = i - 1; j >= 0; j--) {
            int prev_score = 9999;
            int pr = pool[j][0], pc = pool[j][1];
            for(int bi = 0; bi < init_b_cnt; bi++) {
                int v = (abs(pr - init_b[bi][0]) + abs(pc - init_b[bi][1])) * 3;
                if(v < prev_score) prev_score = v;
            }
            for(int bi = 0; bi < init_bx_cnt; bi++) {
                int v = abs(pr - init_bx[bi][0]) + abs(pc - init_bx[bi][1]);
                if(v < prev_score) prev_score = v;
            }
            for(int gi = 0; gi < init_dot_cnt; gi++) {
                int v = abs(pr - init_dot[gi][0]) + abs(pc - init_dot[gi][1]);
                if(v < prev_score) prev_score = v;
            }
            if(prev_score <= key_score) break;
            pool[j + 1][0] = pool[j][0];
            pool[j + 1][1] = pool[j][1];
        }
        pool[j + 1][0] = wr;
        pool[j + 1][1] = wc;
    }
    t_setup_and_hull = get_time_s(t0, GET_CURRENT_MS());
    memset(global_can_be_first, 0, sizeof(global_can_be_first));
        
    for (int i = 0; i < pool_size; i++) {
        int16_t target[1][2] = {{pool[i][0], pool[i][1]}}; 
        bool base_wall[MAX_R][MAX_C];
        build_opt_wall_and_rigid_bodies(NULL, 0, target, 1, base_wall);
        if (run_macro_pull(target, 1, base_wall, init_b, init_b_cnt) > 0) {
            global_can_be_first[pool[i][0]][pool[i][1]] = true;
        }
    }

    float global_best_metric = 999999.0f; int best_p1_steps = 0;
    int total_combos = 0, pruned_combos = 0, tested_combos = 0, success_cnt = 0;
    int base_bx_dist = get_manhattan_dist(init_bx_cnt, init_bx, init_dot_cnt, init_dot);

    printf("====== 启动联合求解器 (双轨自适应搜索 + 极速查表Warp) ======\r\n");
    uint32_t start_time = GET_CURRENT_MS();
    bool timeout_triggered = false;

    // ==================== ★ 双轨自适应机制 ====================
    int node_limits[2] = {MAX_NODES_1, MAX_NODES};
    
    for (int lim_idx = 0; lim_idx < 2; lim_idx++) {
        int active_max_nodes = node_limits[lim_idx];
        printf("\r\n>> [双轨搜索] 启动第 %d 轨，当前节点上限: MAX_NODES = %d ...\r\n", lim_idx + 1, active_max_nodes);
        
        total_combos = 0; pruned_combos = 0; tested_combos = 0; success_cnt = 0;
        global_best_metric = 999999.0f;
        
        // ★ 完全修复的严格组合生成约束边界
        int lim_i = (init_b_cnt >= 1) ? pool_size : 1;
        for (int i = 0; i < lim_i; i++) {
            
            int start_j = (init_b_cnt >= 2) ? (i + 1) : 0;
            int lim_j   = (init_b_cnt >= 2) ? pool_size : 1;
            for (int j = start_j; j < lim_j; j++) {
                
                int start_k = (init_b_cnt >= 3) ? (j + 1) : 0;
                int lim_k   = (init_b_cnt >= 3) ? pool_size : 1;
                for (int k = start_k; k < lim_k; k++) {
                    
                    if (init_b_cnt >= 2 && !is_dist_ok(pool[i][0], pool[i][1], pool[j][0], pool[j][1])) continue;
                    if (init_b_cnt >= 3 && !is_dist_ok(pool[i][0], pool[i][1], pool[k][0], pool[k][1])) continue;
                    if (init_b_cnt >= 3 && !is_dist_ok(pool[j][0], pool[j][1], pool[k][0], pool[k][1])) continue;
                    
                    total_combos++;
                    int16_t bombs[3][2];
                    bombs[0][0] = (init_b_cnt >= 1) ? pool[i][0] : 0;
                    bombs[0][1] = (init_b_cnt >= 1) ? pool[i][1] : 0;
                    bombs[1][0] = (init_b_cnt >= 2) ? pool[j][0] : 0;
                    bombs[1][1] = (init_b_cnt >= 2) ? pool[j][1] : 0;
                    bombs[2][0] = (init_b_cnt >= 3) ? pool[k][0] : 0;
                    bombs[2][1] = (init_b_cnt >= 3) ? pool[k][1] : 0;
                    
                    uint32_t t_tmp = GET_CURRENT_MS();
                    if (fast_forward_bfs_check(bombs, init_b_cnt)) { t_filter1_bfs += get_time_s(t_tmp, GET_CURRENT_MS());pruned_combos++; continue; }
                    t_filter1_bfs += get_time_s(t_tmp, GET_CURRENT_MS());
                    
                    t_tmp = GET_CURRENT_MS();
                    if (global_best_metric < 999999.0f) {
                        int b_dist = 99999;
                        if (init_b_cnt == 3) {
                            for (int m = 0; m < 6; m++) {
                                int cost = relaxed_dist_b[0][bombs[perms[m][0]][0]][bombs[perms[m][0]][1]] +
                                           relaxed_dist_b[1][bombs[perms[m][1]][0]][bombs[perms[m][1]][1]] +
                                           relaxed_dist_b[2][bombs[perms[m][2]][0]][bombs[perms[m][2]][1]];
                                if (cost < b_dist) b_dist = cost;
                            }
                        } else if (init_b_cnt == 2) {
                            for (int m = 0; m < 2; m++) {
                                int cost = relaxed_dist_b[0][bombs[m==0?0:1][0]][bombs[m==0?0:1][1]] +
                                           relaxed_dist_b[1][bombs[m==0?1:0][0]][bombs[m==0?1:0][1]];
                                if (cost < b_dist) b_dist = cost;
                            }
                        } else if (init_b_cnt == 1) {
                            b_dist = relaxed_dist_b[0][bombs[0][0]][bombs[0][1]];
                        } else {
                            b_dist = 0;
                        }

                        int dynamic_box_dist = get_dynamic_bx_dist(bombs, init_b_cnt);

                        if ((float)(b_dist + dynamic_box_dist) >= global_best_metric) { 
                            t_filter3_dist += get_time_s(t_tmp, GET_CURRENT_MS());
                            pruned_combos++; continue; 
                        }
                    }
                    t_filter3_dist += get_time_s(t_tmp, GET_CURRENT_MS());
                    
                    t_tmp = GET_CURRENT_MS();
                    if (fast_prune_check(bombs, init_b_cnt)) { t_filter2_prune += get_time_s(t_tmp, GET_CURRENT_MS());pruned_combos++; continue; }
                    t_filter2_prune += get_time_s(t_tmp, GET_CURRENT_MS());
                    
                    tested_combos++; int temp_p1_len = 0; struct SimState temp_final_st;
                    // ★ 传入 skip_flag
                    float metric = solve_with_bombs(bombs, &temp_p1_len, &temp_final_st, start_time, active_max_nodes, global_best_metric, skip_flag); 
                    
                    if (metric >= 0.0f) {
                        success_cnt++;
                        if (metric < global_best_metric) {
                            global_best_metric = metric; best_p1_steps = temp_p1_len;
                            g_best_p1_len = temp_p1_len;
                            g_final_p1_state = temp_final_st;
                            for(int bx=0; bx<3; bx++) { g_best_bombs[bx][0] = bombs[bx][0]; g_best_bombs[bx][1] = bombs[bx][1]; }
                            memcpy(g_best_p1_acts, p1_acts, sizeof(int16_t) * best_p1_steps);
                            memcpy(g_best_p1_states, p1_states,
                                   sizeof(struct SimState) * (best_p1_steps + 1));
                            g_best_obs_b = obs_b;
                            g_best_obs_t = obs_t;
                            g_best_p2_len = temp_p2_len;
                            memcpy(g_best_p2_acts, temp_p2_acts, sizeof(int16_t) * temp_p2_len);
                        }
                    }
                    if (GET_CURRENT_MS() - start_time > MAX_SOLVE_TIME_MS) {
                        timeout_triggered = true;
                        goto ESCAPE_SEARCH;
                    }
                }
            }
        }
        
        if (success_cnt > 0) {
            printf(">> [双轨搜索] 在 MAX_NODES = %d 档位已找到完美通关解，提前终止放宽！\r\n", active_max_nodes);
            break;
        }
        if (timeout_triggered) {
            break;
        }
    }

ESCAPE_SEARCH: 
    printf("\r\n======================================================\r\n");
    double total_time = t_setup_and_hull + t_filter1_bfs + t_filter3_dist + t_filter2_prune + t_p1_astar + t_p2_bfs;
    printf("\n================ 性能剖析报告 ================\n");
    printf("1. 地图预处理与凸包提取  : %8.4f 秒\n", t_setup_and_hull);
    printf("2. 拦截网1 (宏观连通BFS) : %8.4f 秒\n", t_filter1_bfs);
    printf("3. 拦截网3 (曼哈顿距下界): %8.4f 秒\n", t_filter3_dist);
    printf("4. 拦截网2 (宏观回拉剪枝): %8.4f 秒\n", t_filter2_prune);
    printf("5. 核心引擎 P1 (A*主寻路): %8.4f 秒 (执行 %d 次)\n", t_p1_astar, p1_exec_cnt);
    printf("6. 核心引擎 P2 (A*极速版) : %8.4f 秒 (执行 %d 次)\n", t_p2_bfs, p2_exec_cnt);
    printf("----------------------------------------------\n");
    printf("总系统运行时间           : %8.4f 秒\n", total_time);
    printf("\r\n======================================================\r\n");
    if (timeout_triggered && success_cnt > 0) { printf(">> 警告：触发解算超时上限，截断并输出保底最优解！\r\n"); } else { printf(">> 正常解算完毕！\r\n"); }
    printf("生成有效组合 (间距>=2): %d 组\r\n", total_combos);
    if(total_combos > 0) printf("安检拦截: %d 组\r\n", pruned_combos);
    printf("引擎放行: %d 组 | 跑通解: %d 组\r\n", tested_combos, success_cnt);
    
    if (global_best_metric >= 999999.0f) return false; 
    
    printf("\r\n【全局最优解】");
    for (int i = 0; i < init_b_cnt; i++) {
        printf("最强炸点%d: (%d,%d) ", i+1, g_best_bombs[i][1], 11 - g_best_bombs[i][0]);
    }
    printf("\r\n");
    
    p1_res->path_length = best_p1_steps;
    for (int i = 0; i < best_p1_steps; i++) p1_res->path[i] = (int8_t)g_best_p1_acts[i];
    p1_res->end_px = g_final_p1_state.c; p1_res->end_py = 11 - g_final_p1_state.r; 
    p1_res->bomb_count = (uint8_t)init_b_cnt;
    p1_res->box_count = (uint8_t)init_bx_cnt;
    p1_res->goal_count = (uint8_t)init_dot_cnt;
    p1_res->observed_box_mask = (uint8_t)g_best_obs_b;
    p1_res->observed_goal_mask = (uint8_t)g_best_obs_t;
    for(int i = 0; i < MAX_ITEMS; i++) {
        p1_res->blast_x[i] = (uint8_t)g_best_bombs[i][1];
        p1_res->blast_y[i] = (uint8_t)g_best_bombs[i][0];
    }
    for(int i = 0; i <= best_p1_steps; i++) {
        p1_res->states[i].r = (uint8_t)g_best_p1_states[i].r;
        p1_res->states[i].c = (uint8_t)g_best_p1_states[i].c;
        p1_res->states[i].mask = (uint8_t)g_best_p1_states[i].mask;
        for(int k = 0; k < MAX_ITEMS; k++) {
            p1_res->states[i].bombs[k] = (uint8_t)g_best_p1_states[i].b[k];
            p1_res->states[i].boxes[k] = (uint8_t)g_best_p1_states[i].bx[k];
        }
    }

    for (int r = 0; r < MAX_R; r++) {
        for (int c = 0; c < MAX_C; c++) {
            char ch = original_map[r][c]; int y = 11 - r; int x = c;
            p1_res->updated_map[y][x] = (ch == '#' || ch == '.') ? ch : ' ';
            for (int b = 0; b < init_b_cnt; b++) {
                if (abs(r - g_best_bombs[b][0]) <= 1 && abs(c - g_best_bombs[b][1]) <= 1) {
                    if (r > 0 && r < MAX_R - 1 && c > 0 && c < MAX_C - 1) p1_res->updated_map[y][x] = ' ';
                }
            }
        }
    }
    p1_res->updated_map[11 - g_final_p1_state.r][g_final_p1_state.c] = '@';
    for (int i = 0; i < 3; i++) {
        if (g_final_p1_state.bx[i] != 255) {
            int bx_r = GET_R(g_final_p1_state.bx[i]); int bx_c = GET_C(g_final_p1_state.bx[i]);
            p1_res->updated_map[11 - bx_r][bx_c] = '$';
        }
    }
    return true;
}

bool bomb_solver_run_phase2(const int8_t* box_ids, const int8_t* target_ids,
                            bomb_phase_result_t* p2_res) {
    if(box_ids == NULL || target_ids == NULL || p2_res == NULL ||
       g_best_p1_len <= 0) return false;
    memset(p2_res, 0, sizeof(*p2_res));
    int cur_assign[3] = {-1, -1, -1};
    bool target_used[3] = {false, false, false};
    for (int i = 0; i < init_bx_cnt; i++) {
        for (int j = 0; j < init_dot_cnt; j++) {
            if (!target_used[j] && box_ids[i] == target_ids[j]) {
                cur_assign[i] = j; 
                target_used[j] = true;
                break;
            }
        }
        if(cur_assign[i] < 0) return false;
    }
    for (int r = 0; r < MAX_R; r++) {
        for (int c = 0; c < MAX_C; c++) {
            global_exp_cov[r][c] = 0;
            if (r > 0 && r < MAX_R - 1 && c > 0 && c < MAX_C - 1) {
                for (int i = 0; i < init_b_cnt; i++) {
                    if (abs(r - g_best_bombs[i][0]) <= 1 && abs(c - g_best_bombs[i][1]) <= 1) {
                        global_exp_cov[r][c] |= (1 << i);
                    }
                }
            }
        }
    }

    for(int i = 0; i < 3; i++) {
        for(int r = 0; r < MAX_R; r++) for(int c = 0; c < MAX_C; c++) bfs_dist_dot[i][r][c] = 9999;
        if (i < init_dot_cnt) {
            int head = 0, tail = 0; uint16_t qr[512], qc[512];
            qr[tail] = GET_R(dot_pos[i]); qc[tail] = GET_C(dot_pos[i]); tail++; 
            bfs_dist_dot[i][qr[0]][qc[0]] = 0;
            while(head < tail) {
                int r = qr[head], c = qc[head]; head++;
                for(int d = 0; d < 4; d++) {
                    int nr = r + dr[d], nc = c + dc[d]; 
                    if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
                    if(global_base_wall[nr][nc] && !(7 & global_exp_cov[nr][nc])) continue;
                    if(bfs_dist_dot[i][nr][nc] == 9999) { 
                        bfs_dist_dot[i][nr][nc] = bfs_dist_dot[i][r][c] + 1; 
                        qr[tail] = nr; qc[tail] = nc; tail++; 
                    }
                }
            }
        }
    }

    float best_score = 999999.0f;
    int16_t best_acts[MAX_STEPS];
    int best_len = 0;
    struct SimState final_p2_state = g_final_p1_state; 

    int box_orders[6][3] = {
        {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
    };
    int order_count = 6;
    if(init_bx_cnt == 1) {
        order_count = 1;
        box_orders[0][0] = 0;
    } else if(init_bx_cnt == 2) {
        order_count = 2;
        box_orders[0][0] = 0; box_orders[0][1] = 1;
        box_orders[1][0] = 1; box_orders[1][1] = 0;
    }

    for (int ord = 0; ord < order_count; ord++) {
        struct SimState s2 = g_final_p1_state; 
        int16_t cur_acts[MAX_STEPS]; 
        int cur_len = 0; 
        bool possible = true;
        
        for (int step = 0; step < init_bx_cnt; step++) {
            int b_id = box_orders[ord][step];
            if (s2.bx[b_id] == 255) continue; 
            
            int t_id = cur_assign[b_id]; 
            int tr = GET_R(dot_pos[t_id]), tc = GET_C(dot_pos[t_id]);
            
            p2_vis_id++;
            if (p2_vis_id == 0xFFFF) { memset(p2_vis, 0, sizeof(p2_vis)); p2_vis_id = 1; }
            
            p2_heap_size = 0;
            int t = 0; 
            
            rq_p2[t].rr = s2.r; rq_p2[t].rc = s2.c; 
            rq_p2[t].br = GET_R(s2.bx[b_id]); rq_p2[t].bc = GET_C(s2.bx[b_id]); 
            rq_p2[t].parent = -1; rq_p2[t].action = -1; 
            rq_p2[t].dist = 0; 
            p2_f[t] = 0; 

            push_p2(t++); 
            
            int goal = -1;
            
            while (p2_heap_size > 0) {
                int curr_idx = pop_p2(); 
                struct QN curr = rq_p2[curr_idx];

                if (p2_vis[curr.rr][curr.rc][curr.br][curr.bc] == p2_vis_id) continue;
                p2_vis[curr.rr][curr.rc][curr.br][curr.bc] = p2_vis_id;
                
                if (curr.br == tr && curr.bc == tc) { goal = curr_idx; break; }
                
                for (int d = 0; d < 4; d++) {
                    int nrr = curr.rr + dr[d], nrc = curr.rc + dc[d], nbr = curr.br, nbc = curr.bc;
                    
                    if (nrr < 0 || nrr >= MAX_R || nrc < 0 || nrc >= MAX_C) continue;
                    if (global_base_wall[nrr][nrc] && !(7 & global_exp_cov[nrr][nrc])) continue;
                    
                    bool blocked = false; 
                    for (int k = 0; k < 3; k++) {
                        if (k != b_id && s2.bx[k] != 255 && s2.bx[k] == MAKE_POS(nrr, nrc)) blocked = true;
                    }
                    if (blocked) continue;
                    
                    if (nrr == curr.br && nrc == curr.bc) {
                        nbr += dr[d]; nbc += dc[d]; 
                        if (nbr < 0 || nbr >= MAX_R || nbc < 0 || nbc >= MAX_C) continue;
                        if (global_base_wall[nbr][nbc] && !(7 & global_exp_cov[nbr][nbc])) continue;
                        for (int k = 0; k < 3; k++) {
                            if (k != b_id && s2.bx[k] != 255 && s2.bx[k] == MAKE_POS(nbr, nbc)) blocked = true;
                        }
                        if (blocked) continue;
                    }

                    if (p2_vis[nrr][nrc][nbr][nbc] == p2_vis_id) continue;
                    
                    int new_dist = curr.dist + 1;
                    if (cur_len + new_dist > best_score) continue; 

                    if (t < MAX_P2_Q) {
                        rq_p2[t].rr = nrr; rq_p2[t].rc = nrc; rq_p2[t].br = nbr; rq_p2[t].bc = nbc;
                        rq_p2[t].parent = curr_idx; rq_p2[t].action = d; 
                        rq_p2[t].dist = new_dist; 
                        int h_box = bfs_dist_dot[t_id][nbr][nbc];
                        if (h_box == 9999) continue; 
                        
                        int h_robot = abs(nrr - nbr) + abs(nrc - nbc);
                        p2_f[t] = new_dist + h_box * 1 + h_robot;
                        push_p2(t++);
                    }
                }
            }
            if (goal == -1) { possible = false; break; } 
            int16_t temp_p[1000]; int t_len = 0, c_ptr = goal;
            while (rq_p2[c_ptr].parent != -1) { temp_p[t_len++] = rq_p2[c_ptr].action; c_ptr = rq_p2[c_ptr].parent; }
            for (int x = t_len - 1; x >= 0; x--) cur_acts[cur_len++] = temp_p[x];
            
            s2.r = rq_p2[goal].rr; s2.c = rq_p2[goal].rc; s2.bx[b_id] = 255; 
        }
        
        if (possible) {
            int cur_turns = count_path_turns(cur_acts, cur_len);
            float cur_score = (float)cur_len + 4.0f * cur_turns; // 转向惩罚
            if (cur_score < best_score) { 
                best_score = cur_score; 
                best_len = cur_len; 
                memcpy(best_acts, cur_acts, sizeof(int16_t) * cur_len); 
                final_p2_state = s2; 
            }
        }
    }

    if (best_score >= 999999.0f) return false; 
    p2_res->path_length = best_len;
    for (int i = 0; i < best_len; i++) p2_res->path[i] = (int8_t)best_acts[i];
    p2_res->bomb_count = (uint8_t)init_b_cnt;
    p2_res->box_count = (uint8_t)init_bx_cnt;
    p2_res->goal_count = (uint8_t)init_dot_cnt;
    for(int i = 0; i < MAX_ITEMS; i++)
        p2_res->assignment[i] = (uint8_t)(cur_assign[i] < 0 ? 255 : cur_assign[i]);
    int rr = final_p2_state.r, rc = final_p2_state.c;
    int base1_r = 11 - 5, base1_c = 1; int base2_r = 11 - 5, base2_c = 14; 
    
    static int16_t dist[MAX_R][MAX_C]; static int16_t parent[MAX_R][MAX_C]; static int8_t  action[MAX_R][MAX_C];
    for(int r = 0; r < MAX_R; r++) for(int c = 0; c < MAX_C; c++) dist[r][c] = -1;
    
    static uint8_t qr[256], qc[256]; int h = 0, t = 0;
    qr[t] = rr; qc[t++] = rc; dist[rr][rc] = 0;
    
    while (h < t) {
        int r = qr[h], c = qc[h++];
        for (int d = 0; d < 4; d++) {
            int nr = r + dr[d], nc = c + dc[d];
            if (nr < 0 || nr >= MAX_R || nc < 0 || nc >= MAX_C) continue;
            if (global_base_wall[nr][nc] && !(7 & global_exp_cov[nr][nc])) continue;
            
            if (dist[nr][nc] == -1) { 
                dist[nr][nc] = dist[r][c] + 1; 
                parent[nr][nc] = MAKE_POS(r, c); 
                action[nr][nc] = d; 
                qr[t] = nr; qc[t++] = nc; 
            }
        }
    }
    
    int d1 = dist[base1_r][base1_c], d2 = dist[base2_r][base2_c];
    int target_r = -1, target_c = -1;
    if (d1 != -1 && d2 != -1) { if (d1 <= d2) { target_r = base1_r; target_c = base1_c; } else { target_r = base2_r; target_c = base2_c; } } 
    else if (d1 != -1) { target_r = base1_r; target_c = base1_c; } else if (d2 != -1) { target_r = base2_r; target_c = base2_c; }
    
    if (target_r != -1) {
        int back_len = dist[target_r][target_c]; int cr = target_r, cc = target_c; int16_t temp_path[256];
        for (int i = back_len - 1; i >= 0; i--) { temp_path[i] = action[cr][cc]; int p = parent[cr][cc]; cr = GET_R(p); cc = GET_C(p); }
        for (int i = 0; i < back_len; i++) { if(p2_res->path_length < MAX_STEPS) p2_res->path[p2_res->path_length++] = (int8_t)temp_path[i]; }
        p2_res->end_px = target_c; p2_res->end_py = 11 - target_r;
    } else {
        p2_res->end_px = rc; p2_res->end_py = 11 - rr;
    }
    
    return true;
}
