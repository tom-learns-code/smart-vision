# -*- coding: utf-8 -*-
"""
track.c 的逐行 Python 翻译（约1946行C代码）

翻译原则：
  1. 每一处计算逻辑和C代码完全一致
  2. 整数溢出严格模拟（uint32_t 用 & 0xFFFFFFFF，vis_id 代清除保留）
  3. 手写二叉堆保持和C一致的比较逻辑
  4. 所有注释用中文，标注对应C代码行号
  5. 模块级全局变量 + reset_globals() 方案
"""

import time
import math
from typing import List, Tuple, Optional

# 导入常量
from .constants import (
    MAX_R, MAX_C, MAX_ITEMS, MAX_NUDGES,
    MAX_NODES_1, MAX_NODES, HASH_SIZE, MAX_P2_Q,
    TRACK_NUM, WEIGHT_FACTOR, MAX_STEPS, MAX_SOLVE_TIME_MS,
    GET_R, GET_C, MAKE_POS,
)
from .types import State, Node, SimState, QN, Coord, POI, PhaseResult

# ==================== 时间工具 ====================

_solve_start_time_ms: float = 0.0  # 求解开始时间（用于超时控制）


def _get_current_ms() -> float:
    """C: GET_CURRENT_MS() → timer_get(GPT_TIM_1)
    Python用 perf_counter 替代硬件定时器，返回毫秒"""
    return time.perf_counter() * 1000.0


def _get_time_s(start_ms: float, end_ms: float) -> float:
    """C: static inline get_time_s (约53-55行)
    将定时器差值转换为秒"""
    return (end_ms - start_ms) / 1000.0


# ==================== 全局变量（和C代码 static 变量逐行对应） ====================

# C: line 33 — int BOX_TARGET_MAP[3] = {0, 1, 2};
BOX_TARGET_MAP = [0, 1, 2]

# C: line 34 — char original_map[MAX_R][MAX_C + 1];
original_map = [[' '] * (MAX_C + 1) for _ in range(MAX_R)]

# C: line 36-37 — Zobrist哈希表
pos_hash_table = [0] * 256        # uint32_t[256]
is_hash_table_init = False        # bool

# C: line 39-41 — A* 的 f/g 值辅助数组
q_f = [0] * MAX_NODES             # int16_t[MAX_NODES]
q_g = [0] * MAX_NODES             # int16_t[MAX_NODES]
relaxed_dist_b = [[[9999] * MAX_C for _ in range(MAX_R)] for __ in range(3)]  # int16_t[3][MAX_R][MAX_C]

# C: line 43-49 — 性能统计变量
t_setup_and_hull = 0.0
t_filter1_bfs = 0.0
t_filter3_dist = 0.0
t_filter2_prune = 0.0
t_p1_astar = 0.0
t_p2_bfs = 0.0
p1_exec_cnt = 0
p2_exec_cnt = 0

# C: line 75-81 — A*搜索数据结构
hash_keys = [0] * HASH_SIZE       # uint32_t[HASH_SIZE]
hash_gen = [0] * HASH_SIZE        # uint16_t[HASH_SIZE]
current_hash_gen = 0              # uint16_t

q = [Node() for _ in range(MAX_NODES)]   # struct Node[MAX_NODES] (79行)
heap = [0] * MAX_NODES                    # int16_t[MAX_NODES] (80行)
heap_size = 0                             # int16_t (81行)
node_cnt = 0                              # int16_t (81行)

# C: line 83-85 — 方向定义
dr = [-1, 1, 0, 0]               # int16_t[4]
dc = [0, 0, -1, 1]               # int16_t[4]
p_dir = [0, 2, 3, 1]             # int16_t[4]: 下=0,右=1(通过p_dir映射)

# C: line 87-93 — 地图解析结果
x_pos = [0, 0, 0]                # int16_t[3]: 炸弹箱目标位置（编码后）
dot_pos = [0, 0, 0]              # int16_t[3]: 目标点位置（编码后）
dot_cnt = 0                      # int16_t
bfs_dist_x = [[[9999] * MAX_C for _ in range(MAX_R)] for __ in range(3)]   # int16_t[3][MAX_R][MAX_C]
bfs_dist_dot = [[[9999] * MAX_C for _ in range(MAX_R)] for __ in range(3)] # int16_t[3][MAX_R][MAX_C]
init_b = [[0, 0] for _ in range(3)]      # int16_t[3][2]: 初始炸弹箱位置
init_b_cnt = 0                           # int16_t
init_bx = [[0, 0] for _ in range(3)]     # int16_t[3][2]: 初始推箱位置
init_bx_cnt = 0                          # int16_t
init_dot = [[0, 0] for _ in range(3)]    # int16_t[3][2]: 初始目标点位置
init_dot_cnt = 0                         # int16_t
init_robot_r = 0                         # int16_t
init_robot_c = 0                         # int16_t
perms = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]]  # int16_t[6][3] (93行)

# C: line 95-99 — P1模拟执行相关
p1_states = [SimState() for _ in range(MAX_STEPS)]  # struct SimState[MAX_STEPS]
p1_acts = [0] * MAX_STEPS                             # int16_t[MAX_STEPS]
p1_len = 0                                            # int
obs_b = 0                                             # uint16_t: 已观察到的箱子掩码
obs_t = 0                                             # uint16_t: 已观察到的目标掩码

# C: line 102 — P2队列
rq_p2 = [QN() for _ in range(MAX_P2_Q)]  # struct QN[MAX_P2_Q]

# C: line 105-110 — 全局最优解缓存
g_final_p1_state = SimState()            # struct SimState
g_best_p1_acts = [0] * MAX_STEPS         # int16_t[MAX_STEPS]
g_best_p1_len = 0                        # int
g_best_bombs = [[0, 0] for _ in range(3)]  # int16_t[3][2]
g_best_p2_acts = [0] * MAX_STEPS         # int16_t[MAX_STEPS]
g_best_p2_len = 0                        # int

# C: line 113-114 — 临时P2路径
temp_p2_acts = [0] * MAX_STEPS           # int16_t[MAX_STEPS]
temp_p2_len = 0                          # int

# C: line 116-123 — P2/P1 访问标记数组
p2_vis = [[[[0] * MAX_C for _ in range(MAX_R)] for __ in range(MAX_R)] for ___ in range(MAX_C)]
# 用1D列表方式表达 p2_vis[MAX_R][MAX_C][MAX_R][MAX_C]
# 实际用展平方式: p2_vis_flat[rr][rc][br][bc]
p2_vis_flat = None  # 在reset中重建
p2_vis_id = 0       # uint16_t

p1_dist_vis = [[0] * MAX_C for _ in range(MAX_R)]  # uint16_t[MAX_R][MAX_C]
p1_dist_vis_id = 0                                  # uint16_t
global_dist = [[0] * MAX_C for _ in range(MAX_R)]   # int16_t[MAX_R][MAX_C]
global_parent = [[0] * MAX_C for _ in range(MAX_R)] # int16_t[MAX_R][MAX_C]
global_action = [[0] * MAX_C for _ in range(MAX_R)] # int16_t[MAX_R][MAX_C]

# C: line 125-126 — 机器人可达性检查
r_vis = [[0] * MAX_C for _ in range(MAX_R)]  # uint16_t[MAX_R][MAX_C]
r_vis_id = 0                                  # uint16_t

# C: line 128-129 — reach可达性
reach_vis = [[0] * MAX_C for _ in range(MAX_R)]  # uint16_t[MAX_R][MAX_C]
current_reach_id = 0                              # uint16_t

# C: line 131-138 — 墙体/死锁预计算
global_base_wall = [[False] * MAX_C for _ in range(MAX_R)]  # bool[MAX_R][MAX_C]
base_wall_initialized = False                                # bool
global_exp_cov = [[0] * MAX_C for _ in range(MAX_R)]         # uint8_t[MAX_R][MAX_C]
deadlock_map = [[False] * MAX_C for _ in range(MAX_R)]       # bool[MAX_R][MAX_C]

global_alive_token = [[[0] * 4 for _ in range(MAX_C)] for __ in range(MAX_R)]  # uint16_t[MAX_R][MAX_C][4]
current_alive_token = 0                                                          # uint16_t
global_can_be_first = [[False] * MAX_C for _ in range(MAX_R)]                    # bool[MAX_R][MAX_C]

# C: line 141 — 快速墙表: fast_wall[8][MAX_R][MAX_C]
fast_wall = [[[False] * MAX_C for _ in range(MAX_R)] for __ in range(8)]

# C: line 418-421 — run_macro_pull 相关
flat_alive_token = [0] * 1024                    # uint16_t[1024]
reach_cache_id = [[[[0] * 4 for _ in range(4)] for __ in range(MAX_C)] for ___ in range(MAX_R)]
# uint16_t[MAX_R][MAX_C][4][4]
reach_cache_val = [[[[False] * 4 for _ in range(4)] for __ in range(MAX_C)] for ___ in range(MAX_R)]
# bool[MAX_R][MAX_C][4][4]
cur_reach_id = 0                                 # uint16_t

# C: 第669-677行 — TSP求解相关
tsp_pois: List[POI] = []
tsp_dist_matrix: List[List[int]] = []
tsp_min_cost = 0
tsp_best_path: List[int] = []
tsp_best_len = 0
tsp_visited: List[bool] = []

# C: 第747-748行 — P2 A* 优先队列
p2_heap = [0] * MAX_P2_Q       # int16_t[MAX_P2_Q]
p2_heap_size = 0                # int16_t
p2_f_array = [0] * MAX_P2_Q    # int16_t[MAX_P2_Q] (C中叫p2_f，Python中避开冲突)


# ==================== 全局变量重置函数 ====================

def reset_globals():
    """每次求解前重置所有模块级全局变量，和C代码的程序重启行为一致"""
    global BOX_TARGET_MAP, original_map, pos_hash_table, is_hash_table_init
    global q_f, q_g, relaxed_dist_b
    global t_setup_and_hull, t_filter1_bfs, t_filter3_dist, t_filter2_prune
    global t_p1_astar, t_p2_bfs, p1_exec_cnt, p2_exec_cnt
    global hash_keys, hash_gen, current_hash_gen
    global q, heap, heap_size, node_cnt
    global x_pos, dot_pos, dot_cnt, bfs_dist_x, bfs_dist_dot
    global init_b, init_b_cnt, init_bx, init_bx_cnt, init_dot, init_dot_cnt
    global init_robot_r, init_robot_c
    global p1_states, p1_acts, p1_len, obs_b, obs_t
    global rq_p2, p2_vis_flat, p2_vis_id
    global p1_dist_vis, p1_dist_vis_id, global_dist, global_parent, global_action
    global r_vis, r_vis_id, reach_vis, current_reach_id
    global global_base_wall, base_wall_initialized, global_exp_cov, deadlock_map
    global global_alive_token, current_alive_token, global_can_be_first
    global fast_wall
    global flat_alive_token, reach_cache_id, reach_cache_val, cur_reach_id
    global g_final_p1_state, g_best_p1_acts, g_best_p1_len
    global g_best_bombs, g_best_p2_acts, g_best_p2_len
    global temp_p2_acts, temp_p2_len
    global tsp_pois, tsp_dist_matrix, tsp_min_cost, tsp_best_path, tsp_best_len, tsp_visited
    global p2_heap, p2_heap_size, p2_f_array

    BOX_TARGET_MAP = [0, 1, 2]
    original_map = [[' '] * (MAX_C + 1) for _ in range(MAX_R)]

    pos_hash_table = [0] * 256
    is_hash_table_init = False

    q_f = [0] * MAX_NODES
    q_g = [0] * MAX_NODES
    relaxed_dist_b = [[[9999] * MAX_C for _ in range(MAX_R)] for __ in range(3)]

    t_setup_and_hull = 0.0
    t_filter1_bfs = 0.0
    t_filter3_dist = 0.0
    t_filter2_prune = 0.0
    t_p1_astar = 0.0
    t_p2_bfs = 0.0
    p1_exec_cnt = 0
    p2_exec_cnt = 0

    hash_keys = [0] * HASH_SIZE
    hash_gen = [0] * HASH_SIZE
    current_hash_gen = 0

    q = [Node() for _ in range(MAX_NODES)]
    heap = [0] * MAX_NODES
    heap_size = 0
    node_cnt = 0

    x_pos = [0, 0, 0]
    dot_pos = [0, 0, 0]
    dot_cnt = 0
    bfs_dist_x = [[[9999] * MAX_C for _ in range(MAX_R)] for __ in range(3)]
    bfs_dist_dot = [[[9999] * MAX_C for _ in range(MAX_R)] for __ in range(3)]

    init_b = [[0, 0] for _ in range(3)]
    init_b_cnt = 0
    init_bx = [[0, 0] for _ in range(3)]
    init_bx_cnt = 0
    init_dot = [[0, 0] for _ in range(3)]
    init_dot_cnt = 0
    init_robot_r = 0
    init_robot_c = 0

    p1_states = [SimState() for _ in range(MAX_STEPS)]
    p1_acts = [0] * MAX_STEPS
    p1_len = 0
    obs_b = 0
    obs_t = 0

    rq_p2 = [QN() for _ in range(MAX_P2_Q)]
    # p2_vis 用展平4D列表重建
    _rebuild_p2_vis()
    p2_vis_id = 0

    p1_dist_vis = [[0] * MAX_C for _ in range(MAX_R)]
    p1_dist_vis_id = 0
    global_dist = [[0] * MAX_C for _ in range(MAX_R)]
    global_parent = [[0] * MAX_C for _ in range(MAX_R)]
    global_action = [[0] * MAX_C for _ in range(MAX_R)]

    r_vis = [[0] * MAX_C for _ in range(MAX_R)]
    r_vis_id = 0

    reach_vis = [[0] * MAX_C for _ in range(MAX_R)]
    current_reach_id = 0

    global_base_wall = [[False] * MAX_C for _ in range(MAX_R)]
    base_wall_initialized = False
    global_exp_cov = [[0] * MAX_C for _ in range(MAX_R)]
    deadlock_map = [[False] * MAX_C for _ in range(MAX_R)]

    global_alive_token = [[[0] * 4 for _ in range(MAX_C)] for __ in range(MAX_R)]
    current_alive_token = 0
    global_can_be_first = [[False] * MAX_C for _ in range(MAX_R)]

    fast_wall = [[[False] * MAX_C for _ in range(MAX_R)] for __ in range(8)]

    flat_alive_token = [0] * 1024
    reach_cache_id = [[[[0] * 4 for _ in range(4)] for __ in range(MAX_C)] for ___ in range(MAX_R)]
    reach_cache_val = [[[[False] * 4 for _ in range(4)] for __ in range(MAX_C)] for ___ in range(MAX_R)]
    cur_reach_id = 0

    g_final_p1_state = SimState()
    g_best_p1_acts = [0] * MAX_STEPS
    g_best_p1_len = 0
    g_best_bombs = [[0, 0] for _ in range(3)]
    g_best_p2_acts = [0] * MAX_STEPS
    g_best_p2_len = 0

    temp_p2_acts = [0] * MAX_STEPS
    temp_p2_len = 0

    tsp_pois = []
    tsp_dist_matrix = []
    tsp_min_cost = 0
    tsp_best_path = []
    tsp_best_len = 0
    tsp_visited = []

    p2_heap = [0] * MAX_P2_Q
    p2_heap_size = 0
    p2_f_array = [0] * MAX_P2_Q


def _rebuild_p2_vis():
    """重建4维p2访问标记数组 p2_vis[MAX_R][MAX_C][MAX_R][MAX_C]"""
    global p2_vis_flat
    # 展平为1D: [rr * MAX_C * MAX_R * MAX_C + rc * MAX_R * MAX_C + br * MAX_C + bc]
    # 但为了和C代码语义一致，使用嵌套列表重建
    # 注：为了方便索引，保持嵌套结构但在reset时整体重建
    p2_vis_flat = [[[[0] * MAX_C for _ in range(MAX_R)] for __ in range(MAX_C)] for ___ in range(MAX_R)]


def _p2_vis_get(rr, rc, br, bc):
    """p2_vis[rr][rc][br][bc] 读取"""
    return p2_vis_flat[rr][rc][br][bc]


def _p2_vis_set(rr, rc, br, bc, val):
    """p2_vis[rr][rc][br][bc] 写入"""
    p2_vis_flat[rr][rc][br][bc] = val


# ==================== Zobrist 哈希（C: 第209-252行） ====================

def encode(s: State) -> int:
    """C: encode(struct State *s) (约209-242行)
    Zobrist哈希编码, uint32_t 溢出严格模拟"""
    global pos_hash_table, is_hash_table_init

    # C: 第210-217行 — 延迟初始化随机数表
    if not is_hash_table_init:
        seed = 0x12345678
        for i in range(256):
            seed ^= (seed << 13) & 0xFFFFFFFF
            seed ^= (seed >> 17) & 0xFFFFFFFF
            seed ^= (seed << 5) & 0xFFFFFFFF
            pos_hash_table[i] = seed & 0xFFFFFFFF
        is_hash_table_init = True

    # C: 第219行 — FNV-1a offset basis
    h = 0x811C9DC5  # 2166136261u
    # C: 第220行 — r<<8 | c
    h ^= ((s.r << 8) | s.c) & 0xFFFFFFFF
    h = (h * 0x01000193) & 0xFFFFFFFF   # * 16777619u
    # C: 第222行 — mask
    h ^= s.mask & 0xFFFFFFFF
    h = (h * 0x01000193) & 0xFFFFFFFF
    # C: 第224行 — nudges
    h ^= s.nudges & 0xFFFFFFFF
    h = (h * 0x01000193) & 0xFFFFFFFF

    # C: 第227-233行 — b_hash 和 bx_hash
    b_hash = 0
    bx_hash = 0
    if s.b[0] != 255:
        b_hash ^= pos_hash_table[s.b[0]]
    if s.b[1] != 255:
        b_hash ^= pos_hash_table[s.b[1]]
    if s.b[2] != 255:
        b_hash ^= pos_hash_table[s.b[2]]

    if s.bx[0] != 255:
        bx_hash ^= pos_hash_table[s.bx[0]]
    if s.bx[1] != 255:
        bx_hash ^= pos_hash_table[s.bx[1]]
    if s.bx[2] != 255:
        bx_hash ^= pos_hash_table[s.bx[2]]

    # C: 第236-239行
    h ^= b_hash & 0xFFFFFFFF
    h = (h * 0x01000193) & 0xFFFFFFFF
    h ^= bx_hash & 0xFFFFFFFF
    h = (h * 0x01000193) & 0xFFFFFFFF

    return h & 0xFFFFFFFF


def hash_insert(key: int) -> bool:
    """C: hash_insert(uint32_t key) (约244-252行)
    线性探测哈希表插入，返回True表示新键（插入成功），False表示已存在"""
    global hash_keys, hash_gen, current_hash_gen

    # C: 第245-246行 — 混合键
    mixed_key = (key ^ (key >> 16) ^ (key >> 8)) & 0xFFFFFFFF
    idx = mixed_key & 8191  # & (HASH_SIZE-1)

    # C: 第247-250行 — 线性探测
    while hash_gen[idx] == current_hash_gen:
        if hash_keys[idx] == key:
            return False  # 已存在
        idx = (idx + 1) & 8191

    # C: 第251行 — 插入
    hash_keys[idx] = key
    hash_gen[idx] = current_hash_gen
    return True


# ==================== A* 优先队列（C: 第183-206行，手写二叉堆） ====================

def _compare_nodes(idx1: int, idx2: int) -> bool:
    """C: compare_nodes (约183-186行)
    返回True表示idx1应该排在idx2前面（idx1更优）
    优先f值小的；f相同时选g值大的（更接近目标）"""
    if q_f[idx1] != q_f[idx2]:
        return q_f[idx1] < q_f[idx2]
    return q_g[idx1] > q_g[idx2]  # f相同，g大的更优（tie-breaking）


def push_heap(idx: int):
    """C: push_heap(int idx) (约188-195行)
    将节点索引推入最小堆"""
    global heap, heap_size

    i = heap_size
    heap_size += 1
    while i > 0:
        p = (i - 1) // 2
        if _compare_nodes(heap[p], idx):
            break
        heap[i] = heap[p]
        i = p
    heap[i] = idx


def pop_heap() -> int:
    """C: pop_heap() (约197-206行)
    弹出最优节点索引"""
    global heap, heap_size

    res = heap[0]
    heap_size -= 1
    idx = heap[heap_size]
    i = 0
    while i * 2 + 1 < heap_size:
        left = i * 2 + 1
        right = i * 2 + 2
        min_c = left
        if right < heap_size and _compare_nodes(heap[right], heap[left]):
            min_c = right
        if _compare_nodes(idx, heap[min_c]):
            break
        heap[i] = heap[min_c]
        i = min_c
    heap[i] = idx
    return res


# ==================== 启发式函数（C: 第254-306行） ====================

def get_h_p1(s: State) -> int:
    """C: get_h_p1(struct State *s) (约254-306行)
    阶段1的A*启发式函数：
    未爆炸弹数 + 最优BFS分配代价 + 机器人到最近炸弹距离"""
    mask = s.mask
    b0, b1, b2 = s.b[0], s.b[1], s.b[2]

    # C: 第259-261行 — 检查各炸弹是否有效（255表示已爆炸/不存在）
    v0 = (b0 != 255)
    v1 = (b1 != 255)
    v2 = (b2 != 255)

    c0 = [0, 0, 0]  # 对应x_pos[0]的BFS距离
    c1 = [0, 0, 0]  # 对应x_pos[1]的BFS距离
    c2 = [0, 0, 0]  # 对应x_pos[2]的BFS距离
    unexploded = 0
    min_p_dist = 9999
    sr, sc = s.r, s.c

    # C: 第268-294行 — 对每个未爆的炸弹箱计算距离
    if v0:
        unexploded += 1
        r, c = GET_R(b0), GET_C(b0)
        d = abs(sr - r) + abs(sc - c)
        if d < min_p_dist:
            min_p_dist = d
        if not (mask & 1):
            c0[0] = bfs_dist_x[0][r][c]
        if not (mask & 2):
            c1[0] = bfs_dist_x[1][r][c]
        if not (mask & 4):
            c2[0] = bfs_dist_x[2][r][c]

    if v1:
        unexploded += 1
        r, c = GET_R(b1), GET_C(b1)
        d = abs(sr - r) + abs(sc - c)
        if d < min_p_dist:
            min_p_dist = d
        if not (mask & 1):
            c0[1] = bfs_dist_x[0][r][c]
        if not (mask & 2):
            c1[1] = bfs_dist_x[1][r][c]
        if not (mask & 4):
            c2[1] = bfs_dist_x[2][r][c]

    if v2:
        unexploded += 1
        r, c = GET_R(b2), GET_C(b2)
        d = abs(sr - r) + abs(sc - c)
        if d < min_p_dist:
            min_p_dist = d
        if not (mask & 1):
            c0[2] = bfs_dist_x[0][r][c]
        if not (mask & 2):
            c1[2] = bfs_dist_x[1][r][c]
        if not (mask & 4):
            c2[2] = bfs_dist_x[2][r][c]

    # C: 第296行 — 全爆了
    if unexploded == 0:
        return 0

    # C: 第298-303行 — 枚举6种排列的最小分配代价
    min_b_cost = c0[0] + c1[1] + c2[2]
    p1_val = c0[0] + c2[1] + c1[2]
    if p1_val < min_b_cost:
        min_b_cost = p1_val
    p2_val = c1[0] + c0[1] + c2[2]
    if p2_val < min_b_cost:
        min_b_cost = p2_val
    p3_val = c1[0] + c2[1] + c0[2]
    if p3_val < min_b_cost:
        min_b_cost = p3_val
    p4_val = c2[0] + c0[1] + c1[2]
    if p4_val < min_b_cost:
        min_b_cost = p4_val
    p5_val = c2[0] + c1[1] + c0[2]
    if p5_val < min_b_cost:
        min_b_cost = p5_val

    # C: 第305行 — h = (unexploded + min_b_cost + min_p_dist) * WEIGHT_FACTOR
    return (unexploded + min_b_cost + min_p_dist) * WEIGHT_FACTOR


# ==================== 机器人快速可达性检查（C: 第308-354行） ====================

def check_robot_reach_fast(sr: int, sc: int, tr: int, tc: int,
                           br: int, bc: int, wall) -> bool:
    """C: check_robot_reach_fast (约308-354行)
    快速检查机器人能否从(sr,sc)到达(tr,tc)，避开箱子位置(br,bc)
    利用O(1)捷径判断 + 轻量BFS"""
    global r_vis, r_vis_id

    # C: 第309行 — 已经在目标
    if sr == tr and sc == tc:
        return True

    # C: 第310-311行 — 曼哈顿距离
    dist = abs(sr - tr) + abs(sc - tc)
    if dist == 1:
        return True

    # C: 第313-325行 — O(1)几何捷径判断
    if dist == 2:
        if not wall[sr][tc]:
            return True
    elif dist == 4:
        if sr == tr:
            if br > 0 and not wall[br - 1][sc] and not wall[br - 1][bc] and not wall[br - 1][tc]:
                return True
            if br < MAX_R - 1 and not wall[br + 1][sc] and not wall[br + 1][bc] and not wall[br + 1][tc]:
                return True
        elif sc == tc:
            if bc > 0 and not wall[sr][bc - 1] and not wall[br][bc - 1] and not wall[tr][bc - 1]:
                return True
            if bc < MAX_C - 1 and not wall[sr][bc + 1] and not wall[br][bc + 1] and not wall[tr][bc + 1]:
                return True

    # C: 第327-352行 — vis_id代清除 + 轻量BFS
    r_vis_id += 1
    if r_vis_id == 0xFFFF:
        for rr in range(MAX_R):
            for cc in range(MAX_C):
                r_vis[rr][cc] = 0
        r_vis_id = 1

    fast_q = [0] * 256  # uint8_t[256]
    h = 0
    t = 0
    fast_q[t] = (sr << 4) | sc
    t += 1
    r_vis[sr][sc] = r_vis_id

    target_pos = (tr << 4) | tc
    box_pos = (br << 4) | bc

    while h < t:
        curr = fast_q[h]
        h += 1
        if curr == target_pos:
            return True
        r = curr >> 4
        c = curr & 15

        # 上
        nr, nc = r - 1, c
        if nr >= 0 and not wall[nr][nc]:
            npos = (nr << 4) | nc
            if npos != box_pos and r_vis[nr][nc] != r_vis_id:
                r_vis[nr][nc] = r_vis_id
                fast_q[t] = npos
                t += 1
        # 下
        nr, nc = r + 1, c
        if nr < MAX_R and not wall[nr][nc]:
            npos = (nr << 4) | nc
            if npos != box_pos and r_vis[nr][nc] != r_vis_id:
                r_vis[nr][nc] = r_vis_id
                fast_q[t] = npos
                t += 1
        # 左
        nr, nc = r, c - 1
        if nc >= 0 and not wall[nr][nc]:
            npos = (nr << 4) | nc
            if npos != box_pos and r_vis[nr][nc] != r_vis_id:
                r_vis[nr][nc] = r_vis_id
                fast_q[t] = npos
                t += 1
        # 右
        nr, nc = r, c + 1
        if nc < MAX_C and not wall[nr][nc]:
            npos = (nr << 4) | nc
            if npos != box_pos and r_vis[nr][nc] != r_vis_id:
                r_vis[nr][nc] = r_vis_id
                fast_q[t] = npos
                t += 1

    return False


# ==================== 优化墙体与刚体构建（C: 第356-417行） ====================

def build_opt_wall_and_rigid_bodies(exploded_sites, exp_cnt: int,
                                     pushable_targets, pt_cnt: int,
                                     out_wall):
    """C: build_opt_wall_and_rigid_bodies (约356-417行)
    构建优化墙体：考虑爆炸清除 + 不可移动物体标记为墙"""
    global global_base_wall, base_wall_initialized

    # C: 第357-362行 — 延迟初始化全局基础墙
    temp_wall = [[False] * MAX_C for _ in range(MAX_R)]
    if not base_wall_initialized:
        for r in range(MAX_R):
            for c in range(MAX_C):
                global_base_wall[r][c] = (original_map[r][c] == '#')
        base_wall_initialized = True

    # C: 第361行 — memcpy基础墙
    for r in range(MAX_R):
        for c in range(MAX_C):
            temp_wall[r][c] = global_base_wall[r][c]

    # C: 第364-369行 — 模拟爆炸清除3x3墙体
    for i in range(exp_cnt):
        xr, xc = exploded_sites[i][0], exploded_sites[i][1]
        for dr_i in range(-1, 2):
            for dc_i in range(-1, 2):
                nr, nc = xr + dr_i, xc + dc_i
                if nr > 0 and nr < MAX_R - 1 and nc > 0 and nc < MAX_C - 1:
                    temp_wall[nr][nc] = False

    # C: 第371-408行 — 迭代标记不可移动的物体为墙
    total_items = init_b_cnt + init_bx_cnt
    item_movable = [False] * (MAX_ITEMS + MAX_ITEMS)
    changed = True
    while changed:
        changed = False
        for i in range(total_items):
            if item_movable[i]:
                continue
            r = init_b[i][0] if i < init_b_cnt else init_bx[i - init_b_cnt][0]
            c = init_b[i][1] if i < init_b_cnt else init_bx[i - init_b_cnt][1]
            can_push = False

            # C: 第380-389行 — 炸弹箱只能推到pushable_targets方向
            if i < init_b_cnt:
                for d in range(4):
                    tr = r + dr[d]
                    tc = c + dc[d]
                    rr = r - dr[d]
                    rc = c - dc[d]
                    for k in range(pt_cnt):
                        if tr == pushable_targets[k][0] and tc == pushable_targets[k][1]:
                            if (rr >= 0 and rr < MAX_R and rc >= 0 and rc < MAX_C
                                    and not temp_wall[rr][rc]):
                                can_push = True
                                break
                    if can_push:
                        break

            # C: 第391-405行 — 通用推物检查
            if not can_push:
                for d in range(4):
                    tr = r + dr[d]
                    tc = c + dc[d]
                    rr = r - dr[d]
                    rc = c - dc[d]
                    if (tr < 0 or tr >= MAX_R or tc < 0 or tc >= MAX_C
                            or rr < 0 or rr >= MAX_R or rc < 0 or rc >= MAX_C):
                        continue
                    if temp_wall[tr][tc] or temp_wall[rr][rc]:
                        continue
                    blocked = False
                    for j in range(total_items):
                        if i == j or item_movable[j]:
                            continue
                        jr = init_b[j][0] if j < init_b_cnt else init_bx[j - init_b_cnt][0]
                        jc = init_b[j][1] if j < init_b_cnt else init_bx[j - init_b_cnt][1]
                        if (jr == tr and jc == tc) or (jr == rr and jc == rc):
                            blocked = True
                            break
                    if not blocked:
                        can_push = True
                        break

            if can_push:
                item_movable[i] = True
                changed = True

    # C: 第409-416行 — 输出墙体：不可移动物体标为墙
    for r in range(MAX_R):
        for c in range(MAX_C):
            out_wall[r][c] = temp_wall[r][c]

    for i in range(total_items):
        if not item_movable[i]:
            r = init_b[i][0] if i < init_b_cnt else init_bx[i - init_b_cnt][0]
            c = init_b[i][1] if i < init_b_cnt else init_bx[i - init_b_cnt][1]
            out_wall[r][c] = True


# ==================== 宏拉取 (Macro Pull)（C: 第422-494行） ====================

def run_macro_pull(targets, target_cnt: int, wall,
                   items, item_cnt: int) -> int:
    """C: run_macro_pull (约422-494行)
    反向BFS传播推方向，返回哪些物品可以被推到目标位置
    返回值为位掩码：bit k 表示 items[k] 可被推到某个target"""
    global current_alive_token, flat_alive_token, cur_reach_id
    global reach_cache_id, reach_cache_val

    # C: 第423-424行 — alive_token 代清除
    current_alive_token += 1
    if current_alive_token == 0xFFFF:
        for i in range(1024):
            flat_alive_token[i] = 0
        current_alive_token = 1

    # C: 第426-427行 — reach_cache 代清除
    cur_reach_id += 1
    if cur_reach_id == 0xFFFF:
        for r in range(MAX_R):
            for c in range(MAX_C):
                for d in range(4):
                    for nd in range(4):
                        reach_cache_id[r][c][d][nd] = 0
        cur_reach_id = 1

    # C: 第429行 — 固定大小队列
    fast_macro_q = [0] * 1024  # uint16_t[1024]
    h, t = 0, 0

    # C: 第432-443行 — 从目标出发初始化
    for i in range(target_cnt):
        for d in range(4):
            pr = targets[i][0] - dr[d]
            pc = targets[i][1] - dc[d]
            rr = pr - dr[d]
            rc = pc - dc[d]
            if (pr >= 0 and pr < MAX_R and pc >= 0 and pc < MAX_C
                    and rr >= 0 and rr < MAX_R and rc >= 0 and rc < MAX_C):
                if not wall[pr][pc] and not wall[rr][rc]:
                    flat_idx = (pr << 6) | (pc << 2) | d
                    flat_alive_token[flat_idx] = current_alive_token
                    fast_macro_q[t] = flat_idx
                    t += 1

    # C: 第445-479行 — BFS传播
    while h < t:
        val = fast_macro_q[h]
        h += 1
        r_pos = val >> 6
        c_pos = (val >> 2) & 15
        d = val & 3

        # 同向传播（直推）
        nr = r_pos - dr[d]
        nc = c_pos - dc[d]
        rr = nr - dr[d]
        rc = nc - dc[d]
        if (nr >= 0 and nr < MAX_R and nc >= 0 and nc < MAX_C
                and rr >= 0 and rr < MAX_R and rc >= 0 and rc < MAX_C):
            flat_nr = (nr << 6) | (nc << 2) | d
            if (not wall[nr][nc] and not wall[rr][rc]
                    and flat_alive_token[flat_nr] != current_alive_token):
                flat_alive_token[flat_nr] = current_alive_token
                fast_macro_q[t] = flat_nr
                t += 1

        # 换向传播（车绕过来从另一侧推）
        curr_rr = r_pos - dr[d]
        curr_rc = c_pos - dc[d]
        for nd in range(4):
            flat_nd = (r_pos << 6) | (c_pos << 2) | nd
            if nd == d or flat_alive_token[flat_nd] == current_alive_token:
                continue
            next_rr = r_pos - dr[nd]
            next_rc = c_pos - dc[nd]
            if (next_rr >= 0 and next_rr < MAX_R and next_rc >= 0 and next_rc < MAX_C
                    and not wall[next_rr][next_rc]):
                can_reach = False
                if reach_cache_id[r_pos][c_pos][d][nd] == cur_reach_id:
                    can_reach = reach_cache_val[r_pos][c_pos][d][nd]
                else:
                    can_reach = check_robot_reach_fast(curr_rr, curr_rc,
                                                       next_rr, next_rc, r_pos, c_pos, wall)
                    reach_cache_id[r_pos][c_pos][d][nd] = cur_reach_id
                    reach_cache_val[r_pos][c_pos][d][nd] = can_reach
                    reach_cache_id[r_pos][c_pos][nd][d] = cur_reach_id
                    reach_cache_val[r_pos][c_pos][nd][d] = can_reach
                if can_reach:
                    flat_alive_token[flat_nd] = current_alive_token
                    fast_macro_q[t] = flat_nd
                    t += 1

    # C: 第480-493行 — 检查每个物品是否可被推到目标
    reached_mask = 0
    for k in range(item_cnt):
        bx, by = items[k][0], items[k][1]
        already_there = False
        for i in range(target_cnt):
            if bx == targets[i][0] and by == targets[i][1]:
                already_there = True
                break
        if already_there:
            reached_mask |= (1 << k)
            continue

        for d in range(4):
            flat_bx = (bx << 6) | (by << 2) | d
            if (flat_alive_token[flat_bx] == current_alive_token
                    and check_robot_reach_fast(init_robot_r, init_robot_c,
                                               bx - dr[d], by - dc[d], bx, by, wall)):
                reached_mask |= (1 << k)
                break

    return reached_mask


# ==================== 快速剪枝检查（C: 第496-581行） ====================

def fast_prune_check(bombs, b_cnt: int) -> bool:
    """C: fast_prune_check (约496-581行)
    4层剪枝检查，返回True表示应该剪掉（不可行）"""
    global global_can_be_first

    if b_cnt == 0:
        return False

    # C: 第500-506行 — 检查1：至少一个炸弹是"可以做第一个的"
    has_first = False
    for i in range(b_cnt):
        if global_can_be_first[bombs[i][0]][bombs[i][1]]:
            has_first = True
            break
    if not has_first:
        return True

    # C: 第509-531行 — 检查2：双炸弹顺序依赖
    if b_cnt == 2:
        order1_ok = global_can_be_first[bombs[0][0]][bombs[0][1]]
        order2_ok = global_can_be_first[bombs[1][0]][bombs[1][1]]

        if order1_ok and not order2_ok:
            temp_wall = [[False] * MAX_C for _ in range(MAX_R)]
            exp1 = [[bombs[0][0], bombs[0][1]]]
            build_opt_wall_and_rigid_bodies(exp1, 1, [], 0, temp_wall)
            if not check_robot_reach_fast(init_robot_r, init_robot_c,
                                          bombs[1][0], bombs[1][1], 255, 255, temp_wall):
                return True
        elif not order1_ok and order2_ok:
            temp_wall = [[False] * MAX_C for _ in range(MAX_R)]
            exp1 = [[bombs[1][0], bombs[1][1]]]
            build_opt_wall_and_rigid_bodies(exp1, 1, [], 0, temp_wall)
            if not check_robot_reach_fast(init_robot_r, init_robot_c,
                                          bombs[0][0], bombs[0][1], 255, 255, temp_wall):
                return True

    # C: 第534-537行 — 检查3：炸后宏推可达性
    ult_wall = [[False] * MAX_C for _ in range(MAX_R)]
    build_opt_wall_and_rigid_bodies(bombs, b_cnt, [], 0, ult_wall)
    full_mask = (1 << init_bx_cnt) - 1
    if run_macro_pull(init_dot, init_dot_cnt, ult_wall, init_bx, init_bx_cnt) != full_mask:
        return True

    # C: 第540行 — <3炸弹直接放行
    if b_cnt < 3:
        return False

    # C: 第543-580行 — 检查4：3炸弹排列可行性
    reach_computed = [False, False, False]
    b_mask_arr = [0, 0, 0]

    for k in range(6):
        p0 = perms[k][0]
        if not reach_computed[0]:
            exp2 = [[bombs[1][0], bombs[1][1]], [bombs[2][0], bombs[2][1]]]
            opt_wall = [[False] * MAX_C for _ in range(MAX_R)]
            target = [[bombs[0][0], bombs[0][1]]]
            build_opt_wall_and_rigid_bodies(exp2, 2, target, 1, opt_wall)
            b_mask_arr[0] = run_macro_pull(target, 1, opt_wall, init_b, init_b_cnt)
            reach_computed[0] = True
        if not (b_mask_arr[0] & (1 << p0)):
            continue

        p1 = perms[k][1]
        if not reach_computed[1]:
            exp2 = [[bombs[0][0], bombs[0][1]], [bombs[2][0], bombs[2][1]]]
            opt_wall = [[False] * MAX_C for _ in range(MAX_R)]
            target = [[bombs[1][0], bombs[1][1]]]
            build_opt_wall_and_rigid_bodies(exp2, 2, target, 1, opt_wall)
            b_mask_arr[1] = run_macro_pull(target, 1, opt_wall, init_b, init_b_cnt)
            reach_computed[1] = True
        if not (b_mask_arr[1] & (1 << p1)):
            continue

        p2 = perms[k][2]
        if not reach_computed[2]:
            exp2 = [[bombs[0][0], bombs[0][1]], [bombs[1][0], bombs[1][1]]]
            opt_wall = [[False] * MAX_C for _ in range(MAX_R)]
            target = [[bombs[2][0], bombs[2][1]]]
            build_opt_wall_and_rigid_bodies(exp2, 2, target, 1, opt_wall)
            b_mask_arr[2] = run_macro_pull(target, 1, opt_wall, init_b, init_b_cnt)
            reach_computed[2] = True
        if b_mask_arr[2] & (1 << p2):
            return False

    return True


# ==================== 快速前向BFS检查（C: 第583-611行） ====================

def fast_forward_bfs_check(bombs, b_cnt: int) -> bool:
    """C: fast_forward_bfs_check (约583-611行)
    模拟爆炸后检查每个推箱能否BFS到达目标（纯连通性，无视车）
    返回True表示不可行"""
    wall_temp = [[False] * MAX_C for _ in range(MAX_R)]
    for r in range(MAX_R):
        for c in range(MAX_C):
            wall_temp[r][c] = (original_map[r][c] == '#')

    for i in range(b_cnt):
        br, bc = bombs[i][0], bombs[i][1]
        for dr_i in range(-1, 2):
            for dc_i in range(-1, 2):
                nr, nc = br + dr_i, bc + dc_i
                if nr > 0 and nr < MAX_R - 1 and nc > 0 and nc < MAX_C - 1:
                    wall_temp[nr][nc] = False

    for i in range(init_bx_cnt):
        vis = [[False] * MAX_C for _ in range(MAX_R)]
        qr = [0] * 256
        qc = [0] * 256
        head, tail = 0, 0
        qr[tail] = init_bx[i][0]
        qc[tail] = init_bx[i][1]
        tail += 1
        vis[qr[0]][qc[0]] = True
        reached_target = False
        target_r, target_c = init_dot[i][0], init_dot[i][1]

        while head < tail:
            r = qr[head]
            c = qc[head]
            head += 1
            if r == target_r and c == target_c:
                reached_target = True
                break
            for d in range(4):
                nr = r + dr[d]
                nc = c + dc[d]
                pr = r - dr[d]
                pc = c - dc[d]
                if (nr >= 0 and nr < MAX_R and nc >= 0 and nc < MAX_C
                        and pr >= 0 and pr < MAX_R and pc >= 0 and pc < MAX_C):
                    if not wall_temp[nr][nc] and not wall_temp[pr][pc] and not vis[nr][nc]:
                        vis[nr][nc] = True
                        qr[tail] = nr
                        qc[tail] = nc
                        tail += 1

        if not reached_target:
            return True

    return False


# ==================== 工具函数（C: 第613-667行） ====================

def get_manhattan_dist(a_cnt: int, a, b_cnt: int, b) -> int:
    """C: get_manhattan_dist (约613-624行)"""
    min_total = 99999
    for m in range(6):
        cost = 0
        for i in range(a_cnt):
            cost += abs(a[i][0] - b[perms[m][i]][0]) + abs(a[i][1] - b[perms[m][i]][1])
        if cost < min_total:
            min_total = cost
    return min_total


def count_bits(n: int) -> int:
    """C: count_bits (约626行)"""
    c = 0
    while n:
        c += n & 1
        n >>= 1
    return c


def count_path_turns(acts, length: int) -> int:
    """C: count_path_turns (约628-641行)"""
    turns = 0
    last_move_dir = -1
    for i in range(length):
        current_act = acts[i]
        if (0 <= current_act <= 4) or (6 <= current_act <= 8):
            if last_move_dir != -1 and current_act != last_move_dir:
                turns += 1
            last_move_dir = current_act
    return turns


def _try_free_look(cur: SimState) -> int:
    """C: try_free_look (约643-655行)
    尝试从当前位置做自由观察。返回新的current_look值"""
    global obs_b, obs_t, p1_len

    current_look = 0
    for i in range(4):
        d = p_dir[i]
        nr = cur.r + dr[d]
        nc = cur.c + dc[d]
        if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
            continue

        npos = MAKE_POS(nr, nc)
        saw = False
        for k in range(3):
            if (cur.bx[k] != 255 and cur.bx[k] == npos
                    and not (obs_b & (1 << k))):
                obs_b |= (1 << k)
                saw = True
        for k in range(3):
            if (dot_pos[k] != 255 and dot_pos[k] == npos
                    and not (obs_t & (1 << k))):
                obs_t |= (1 << k)
                saw = True
        if saw:
            p1_states[p1_len] = SimState(r=cur.r, c=cur.c, mask=cur.mask,
                                          b=list(cur.b), bx=list(cur.bx))
            p1_acts[p1_len] = 5 + d
            p1_len += 1
            current_look = d
    return current_look


def remove_duplicate_looks(path, length_ptr: list):
    """C: remove_duplicate_looks (约657-667行)
    移除连续的重复观察动作。length_ptr = [length] 以便修改"""
    length_val = length_ptr[0]
    if length_val <= 1:
        return
    write_idx = 1
    for read_idx in range(1, length_val):
        current_act = path[read_idx]
        previous_act = path[write_idx - 1]
        if 5 <= current_act <= 8 and current_act == previous_act:
            continue
        path[write_idx] = current_act
        write_idx += 1
    length_ptr[0] = write_idx


# ==================== TSP DFS 观察顺序（C: 第669-745行） ====================

def _dfs_tsp_0bomb(u: int, current_cost: int, cur_b: int, cur_t: int,
                   path_length: int, path: list, req_b: int, req_t: int,
                   cur_sim: SimState, current_look: int):
    """C: dfs_tsp_0bomb (约679-744行)
    DFS + 分支定界搜索最优观察顺序"""
    global tsp_min_cost, tsp_best_len, tsp_best_path

    if current_cost >= tsp_min_cost:
        return

    cb = count_bits(cur_b)
    ct = count_bits(cur_t)
    satisfied = False
    if req_b == 3 and req_t == 3:
        if TRACK_NUM == 4:
            satisfied = (cb >= 2 and ct >= 2)
        elif TRACK_NUM == 5:
            satisfied = ((cb == 2 and ct == 3) or (cb == 3 and ct == 2))
        else:
            satisfied = (cb == 3 and ct == 3)
    else:
        satisfied = (cb == req_b and ct == req_t)

    if satisfied:
        tsp_min_cost = current_cost
        tsp_best_len = path_length
        tsp_best_path = list(path[:path_length])
        return

    for v in range(1, len(tsp_pois)):
        if tsp_visited[v]:
            continue
        if tsp_dist_matrix[u][v] == 9999:
            continue

        add_b = tsp_pois[v].b_mask & ~cur_b
        add_t = tsp_pois[v].t_mask & ~cur_t
        if add_b or add_t:
            req_looks = []
            for d in range(4):
                nr = tsp_pois[v].r + dr[d]
                nc = tsp_pois[v].c + dc[d]
                npos = MAKE_POS(nr, nc)
                need_look = False
                for k in range(3):
                    if (add_b & (1 << k)) and cur_sim.bx[k] == npos:
                        need_look = True
                    if (add_t & (1 << k)) and dot_pos[k] == npos:
                        need_look = True
                if need_look:
                    req_looks.append(d)

            req_cnt = len(req_looks)
            if req_cnt > 0:
                has_curr = any(rl == current_look for rl in req_looks)
                base_penalty = (req_cnt - 1) * 5 if has_curr else req_cnt * 5
                step_cost = tsp_dist_matrix[u][v] + req_cnt + base_penalty

                tsp_visited[v] = True
                path[path_length] = v

                if req_cnt == 1:
                    _dfs_tsp_0bomb(v, current_cost + step_cost,
                                   cur_b | add_b, cur_t | add_t,
                                   path_length + 1, path,
                                   req_b, req_t, cur_sim, req_looks[0])
                else:
                    for i in range(req_cnt):
                        if has_curr and req_looks[i] == current_look:
                            continue
                        _dfs_tsp_0bomb(v, current_cost + step_cost,
                                       cur_b | add_b, cur_t | add_t,
                                       path_length + 1, path,
                                       req_b, req_t, cur_sim, req_looks[i])

                tsp_visited[v] = False


# ==================== P2 A* 优先队列（C: 第747-779行） ====================

def _cmp_p2(a: int, b: int) -> bool:
    """C: cmp_p2 (约751-753行)"""
    return p2_f_array[a] < p2_f_array[b]


def _push_p2(idx: int):
    """C: push_p2 (约755-764行)"""
    global p2_heap, p2_heap_size

    i = p2_heap_size
    p2_heap_size += 1
    while i > 0:
        p = (i - 1) // 2
        if _cmp_p2(p2_heap[p], idx):
            break
        p2_heap[i] = p2_heap[p]
        i = p
    p2_heap[i] = idx


def _pop_p2() -> int:
    """C: pop_p2 (约766-779行)"""
    global p2_heap, p2_heap_size

    res = p2_heap[0]
    p2_heap_size -= 1
    idx = p2_heap[p2_heap_size]
    i = 0
    while i * 2 + 1 < p2_heap_size:
        left = i * 2 + 1
        right = i * 2 + 2
        min_c = left
        if right < p2_heap_size and _cmp_p2(p2_heap[right], p2_heap[left]):
            min_c = right
        if _cmp_p2(idx, p2_heap[min_c]):
            break
        p2_heap[i] = p2_heap[min_c]
        i = min_c
    p2_heap[i] = idx
    return res


# ==================== solve_with_bombs: 核心求解（C: 第781-1485行） ====================

def solve_with_bombs(bombs, out_p1_steps: list, final_st: SimState,
                     start_time: float, active_max_nodes: int,
                     current_best_metric: float, skip_flag: int) -> float:
    """C: solve_with_bombs (约781-1485行)
    给定炸弹位置组合，搜索完整P1+P2路径。
    返回综合评分（float），-1.0表示无解。
    out_p1_steps = [0] 通过列表修改P1步数
    final_st 通过对象属性修改最终状态"""
    global p1_exec_cnt, t_p1_astar, t_p2_bfs
    global current_hash_gen, hash_gen, hash_keys, heap_size, node_cnt
    global heap, q, q_f, q_g
    global bfs_dist_x, bfs_dist_dot, fast_wall
    global global_exp_cov, deadlock_map, global_base_wall
    global reach_vis, current_reach_id, global_dist, global_parent, global_action
    global p1_dist_vis, p1_dist_vis_id
    global obs_b, obs_t, p1_len, p1_states, p1_acts
    global temp_p2_acts, temp_p2_len
    global p2_vis_id, p2_heap_size, p2_f_array

    # C: 第782行 — 记录P1开始时间
    start_p1 = _get_current_ms()
    p1_exec_cnt += 1

    # C: 第783行 — 设置炸弹目标位置
    for i in range(3):
        x_pos[i] = MAKE_POS(bombs[i][0], bombs[i][1])

    # C: 第786-802行 — 预计算8种爆炸掩码的快速墙表
    for m in range(8):
        for r in range(MAX_R):
            for c in range(MAX_C):
                if r == 0 or r == MAX_R - 1 or c == 0 or c == MAX_C - 1:
                    fast_wall[m][r][c] = True
                    continue
                in_exp = False
                for i in range(3):
                    if i >= init_b_cnt:
                        continue
                    if m & (1 << i):
                        if abs(r - bombs[i][0]) <= 1 and abs(c - bombs[i][1]) <= 1:
                            in_exp = True
                            break
                fast_wall[m][r][c] = (not in_exp and (original_map[r][c] == '#'))

    # C: 第804-816行 — 爆炸覆盖掩码 + 墙基础
    for r in range(MAX_R):
        for c in range(MAX_C):
            global_exp_cov[r][c] = 0
            if r > 0 and r < MAX_R - 1 and c > 0 and c < MAX_C - 1:
                for i in range(3):
                    if i >= init_b_cnt:
                        continue
                    if abs(r - bombs[i][0]) <= 1 and abs(c - bombs[i][1]) <= 1:
                        global_exp_cov[r][c] |= (1 << i)

    # C: 第818-826行 — 死锁地图预计算（墙角检测）
    for r in range(1, MAX_R - 1):
        for c in range(1, MAX_C - 1):
            up = global_base_wall[r - 1][c] and not (7 & global_exp_cov[r - 1][c])
            down = global_base_wall[r + 1][c] and not (7 & global_exp_cov[r + 1][c])
            left = global_base_wall[r][c - 1] and not (7 & global_exp_cov[r][c - 1])
            right = global_base_wall[r][c + 1] and not (7 & global_exp_cov[r][c + 1])
            deadlock_map[r][c] = ((up and left) or (up and right) or (down and left) or (down and right))

    # C: 第828-858行 — BFS预计算每个炸弹到所有格的距离 + 每个目标到所有格的距离
    for i in range(3):
        for r in range(MAX_R):
            for c in range(MAX_C):
                bfs_dist_x[i][r][c] = 9999

        if i < init_b_cnt:
            qr_bfs = [0] * 512
            qc_bfs = [0] * 512
            head, tail = 0, 0
            qr_bfs[tail] = GET_R(x_pos[i])
            qc_bfs[tail] = GET_C(x_pos[i])
            tail += 1
            bfs_dist_x[i][qr_bfs[0]][qc_bfs[0]] = 0
            while head < tail:
                r = qr_bfs[head]
                c = qc_bfs[head]
                head += 1
                for d in range(4):
                    nr = r + dr[d]
                    nc = c + dc[d]
                    if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                        continue
                    if global_base_wall[nr][nc] and not (7 & global_exp_cov[nr][nc]):
                        continue
                    if bfs_dist_x[i][nr][nc] == 9999:
                        bfs_dist_x[i][nr][nc] = bfs_dist_x[i][r][c] + 1
                        qr_bfs[tail] = nr
                        qc_bfs[tail] = nc
                        tail += 1

        for r in range(MAX_R):
            for c in range(MAX_C):
                bfs_dist_dot[i][r][c] = 9999

        if i < init_dot_cnt:
            qr_bfs = [0] * 512
            qc_bfs = [0] * 512
            head, tail = 0, 0
            qr_bfs[tail] = GET_R(dot_pos[i])
            qc_bfs[tail] = GET_C(dot_pos[i])
            tail += 1
            bfs_dist_dot[i][qr_bfs[0]][qc_bfs[0]] = 0
            while head < tail:
                r = qr_bfs[head]
                c = qc_bfs[head]
                head += 1
                for d in range(4):
                    nr = r + dr[d]
                    nc = c + dc[d]
                    if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                        continue
                    if global_base_wall[nr][nc] and not (7 & global_exp_cov[nr][nc]):
                        continue
                    if bfs_dist_dot[i][nr][nc] == 9999:
                        bfs_dist_dot[i][nr][nc] = bfs_dist_dot[i][r][c] + 1
                        qr_bfs[tail] = nr
                        qc_bfs[tail] = nc
                        tail += 1

    # C: 第860-867行 — 构建A*初始状态
    start = State()
    b_idx, bx_idx = 0, 0
    for r in range(MAX_R):
        for c in range(MAX_C):
            ch = original_map[r][c]
            if ch == '@' or ch == '+':
                start.r = r
                start.c = c
            if ch == '*':
                if b_idx < 3:
                    start.b[b_idx] = MAKE_POS(r, c)
                    b_idx += 1
            if ch == '$':
                if bx_idx < 3:
                    start.bx[bx_idx] = MAKE_POS(r, c)
                    bx_idx += 1

    # C: 第869-870行 — 标记不存在的炸弹/推箱为255
    for i in range(init_b_cnt, 3):
        start.b[i] = 255
        start.mask |= (1 << i)
    for i in range(init_bx_cnt, 3):
        start.bx[i] = 255

    # C: 第872行 — 哈希代清除
    current_hash_gen += 1
    if current_hash_gen == 0xFFFF:
        for i in range(HASH_SIZE):
            hash_gen[i] = 0
        current_hash_gen = 1
    heap_size = 0
    node_cnt = 0

    # C: 第875-890行 — 初始化A*起始节点
    q[node_cnt].s = State(r=start.r, c=start.c,
                          b=list(start.b), bx=list(start.bx),
                          mask=start.mask, nudges=start.nudges)
    q[node_cnt].g = 0
    q[node_cnt].h = get_h_p1(q[node_cnt].s)
    q[node_cnt].parent = -1
    q[node_cnt].action = -1

    unex = 0
    for i in range(3):
        if not (start.mask & (1 << i)):
            unex += 1
    unweighted_h = (q[node_cnt].h // WEIGHT_FACTOR) - unex

    if current_best_metric < 999999.0 and float(unweighted_h) >= current_best_metric:
        return -1.0

    q_g[node_cnt] = 0
    q_f[node_cnt] = q[node_cnt].h
    hash_insert(encode(q[node_cnt].s))
    push_heap(node_cnt)
    node_cnt += 1
    goal_idx = -1

    # ===== C: 第893-988行 — A*主循环 =====
    while heap_size > 0 and node_cnt < active_max_nodes - 10:
        if (node_cnt & 127) == 0:  # 每128个节点检查一次超时
            if _get_current_ms() - start_time > MAX_SOLVE_TIME_MS:
                t_p1_astar += _get_time_s(start_p1, _get_current_ms())
                return -1.0

        curr_idx = pop_heap()
        curr = q[curr_idx].s
        if curr.mask == 7:  # 所有炸弹已引爆
            goal_idx = curr_idx
            break

        # C: 第902-906行 — 固体掩码（用于碰撞检测）
        solid_mask = [0] * MAX_R  # uint16_t[MAX_R]
        for k in range(3):
            if curr.b[k] != 255:
                solid_mask[curr.b[k] >> 4] |= (1 << (curr.b[k] & 15))
            if curr.bx[k] != 255:
                solid_mask[curr.bx[k] >> 4] |= (1 << (curr.bx[k] & 15))

        # C: 第908-909行 — 机器人可达性 vis_id
        current_reach_id += 1
        if current_reach_id == 0xFFFF:
            for rr in range(MAX_R):
                for cc in range(MAX_C):
                    reach_vis[rr][cc] = 0
            current_reach_id = 1

        # C: 第911-928行 — BFS计算从机器人当前位置到所有可达格的距离
        fast_q_bfs = [0] * 256
        head, tail = 0, 0
        fast_q_bfs[tail] = (curr.r << 4) | curr.c
        tail += 1
        reach_vis[curr.r][curr.c] = current_reach_id
        global_dist[curr.r][curr.c] = 0

        while head < tail:
            val = fast_q_bfs[head]
            head += 1
            r = val >> 4
            c = val & 15
            cur_wall = fast_wall[curr.mask]

            for d in range(4):
                nr = r + dr[d]
                nc = c + dc[d]
                if cur_wall[nr][nc]:
                    continue
                if not (solid_mask[nr] & (1 << nc)) and reach_vis[nr][nc] != current_reach_id:
                    reach_vis[nr][nc] = current_reach_id
                    global_dist[nr][nc] = global_dist[r][c] + 1
                    fast_q_bfs[tail] = (nr << 4) | nc
                    tail += 1

        # C: 第930-987行 — 对每个物品尝试推（6个物品：3炸弹箱+3推箱）
        for item_idx in range(6):
            item_pos = curr.b[item_idx] if item_idx < 3 else curr.bx[item_idx - 3]
            if item_pos == 255:
                continue
            nr = GET_R(item_pos)
            nc = GET_C(item_pos)

            # 尝试4个推方向
            for i_dir in range(4):
                r = nr - dr[i_dir]
                c = nc - dc[i_dir]
                if r < 0 or r >= MAX_R or c < 0 or c >= MAX_C:
                    continue
                if reach_vis[r][c] != current_reach_id:
                    continue

                nnr = nr + dr[i_dir]
                nnc = nc + dc[i_dir]
                if nnr < 0 or nnr >= MAX_R or nnc < 0 or nnc >= MAX_C:
                    continue
                nnpos = MAKE_POS(nnr, nnc)

                next_state = State(r=nr, c=nc,
                                   b=list(curr.b), bx=list(curr.bx),
                                   mask=curr.mask, nudges=curr.nudges)
                action_penalty = 0

                # C: 第946-963行 — 推炸弹箱
                if item_idx < 3:
                    hit_b = item_idx
                    hit_x = False
                    for k in range(3):
                        if nnpos == x_pos[k] and not (curr.mask & (1 << k)):
                            next_state.b[hit_b] = 255
                            next_state.mask |= (1 << k)
                            hit_x = True
                            break
                    hit_other = (solid_mask[nnr] & (1 << nnc)) != 0
                    if hit_other or (not hit_x and fast_wall[curr.mask][nnr][nnc]):
                        continue

                    if not hit_x:
                        can_reach = False
                        for tk in range(3):
                            if not (curr.mask & (1 << tk)):
                                if bfs_dist_x[tk][nnr][nnc] != 9999:
                                    can_reach = True
                                    break
                        if not can_reach:
                            continue
                        next_state.b[hit_b] = nnpos
                    action_penalty = 0

                # C: 第965-971行 — 推弹/微调（nudge）
                else:
                    hit_bx = item_idx - 3
                    if curr.nudges >= MAX_NUDGES:
                        continue
                    next_state.nudges += 1
                    hit_other = (solid_mask[nnr] & (1 << nnc)) != 0
                    if hit_other or fast_wall[curr.mask][nnr][nnc] or deadlock_map[nnr][nnc]:
                        continue
                    next_state.bx[hit_bx] = nnpos
                    action_penalty = 10

                # C: 第974-984行 — 哈希去重 + 入堆
                key = encode(next_state)
                if hash_insert(key):
                    if node_cnt >= active_max_nodes:
                        continue
                    q[node_cnt].s = next_state
                    q[node_cnt].parent = curr_idx
                    q[node_cnt].action = (r << 16) | (c << 8) | i_dir

                    new_g = q_g[curr_idx] + global_dist[r][c] + 1 + action_penalty
                    new_h = get_h_p1(next_state)
                    q_g[node_cnt] = new_g
                    q_f[node_cnt] = new_g + new_h
                    push_heap(node_cnt)
                    node_cnt += 1

    # C: 第989行 — P1结束时间
    t_p1_astar += _get_time_s(start_p1, _get_current_ms())

    # C: 第991-992行
    start_p2 = _get_current_ms()
    if goal_idx == -1:
        return -1.0

    # ===== C: 第994-1031行 — 从目标节点逆推P1路径 =====
    raw_acts = [0] * MAX_STEPS
    raw_len = 0
    p1_path = [0] * MAX_STEPS
    p1_macro_len = 0
    curr_node = goal_idx
    while q[curr_node].parent != -1:
        p1_path[p1_macro_len] = curr_node
        p1_macro_len += 1
        curr_node = q[curr_node].parent

    for i in range(p1_macro_len - 1, -1, -1):
        act = q[p1_path[i]].action
        walk_r = act >> 16
        walk_c = (act >> 8) & 0xFF
        push_dir = act & 0xFF
        prev = q[q[p1_path[i]].parent].s

        # BFS还原行走路径
        p1_dist_vis_id += 1
        if p1_dist_vis_id == 0xFFFF:
            for rr in range(MAX_R):
                for cc in range(MAX_C):
                    p1_dist_vis[rr][cc] = 0
            p1_dist_vis_id = 1

        qr_bfs = [0] * 512
        qc_bfs = [0] * 512
        h, t = 0, 0
        qr_bfs[t] = prev.r
        qc_bfs[t] = prev.c
        t += 1
        global_dist[prev.r][prev.c] = 0
        p1_dist_vis[prev.r][prev.c] = p1_dist_vis_id

        while h < t:
            r = qr_bfs[h]
            c = qc_bfs[h]
            h += 1
            if r == walk_r and c == walk_c:
                break
            for d in range(4):
                nr = r + dr[d]
                nc = c + dc[d]
                if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                    continue
                if not fast_wall[prev.mask][nr][nc] and p1_dist_vis[nr][nc] != p1_dist_vis_id:
                    block = False
                    for k in range(3):
                        if ((prev.b[k] != 255 and prev.b[k] == MAKE_POS(nr, nc))
                                or (prev.bx[k] != 255 and prev.bx[k] == MAKE_POS(nr, nc))):
                            block = True
                    if not block:
                        p1_dist_vis[nr][nc] = p1_dist_vis_id
                        global_dist[nr][nc] = global_dist[r][c] + 1
                        global_parent[nr][nc] = MAKE_POS(r, c)
                        global_action[nr][nc] = d
                        qr_bfs[t] = nr
                        qc_bfs[t] = nc
                        t += 1

        cr, cc = walk_r, walk_c
        steps = [0] * 512
        step_cnt = 0
        while cr != prev.r or cc != prev.c:
            steps[step_cnt] = global_action[cr][cc]
            step_cnt += 1
            p = global_parent[cr][cc]
            cr = GET_R(p)
            cc = GET_C(p)
        for x in range(step_cnt - 1, -1, -1):
            raw_acts[raw_len] = steps[x]
            raw_len += 1
        raw_acts[raw_len] = push_dir
        raw_len += 1
        if q[p1_path[i]].s.mask != prev.mask:
            raw_acts[raw_len] = 4  # 爆炸动作
            raw_len += 1

    # ===== C: 第1032-1058行 — 模拟执行P1动作序列 =====
    cur_sim = SimState(r=start.r, c=start.c, mask=start.mask,
                       b=list(start.b), bx=list(start.bx))
    obs_b = 0
    obs_t = 0
    p1_len = 0

    current_look_dir = 0
    if skip_flag == 0:
        current_look_dir = _try_free_look(cur_sim)

    for i in range(raw_len):
        p1_states[p1_len] = SimState(r=cur_sim.r, c=cur_sim.c, mask=cur_sim.mask,
                                      b=list(cur_sim.b), bx=list(cur_sim.bx))
        p1_acts[p1_len] = raw_acts[i]
        p1_len += 1
        act = raw_acts[i]
        if 0 <= act <= 3:  # 方向移动
            nr = cur_sim.r + dr[act]
            nc = cur_sim.c + dc[act]
            npos = MAKE_POS(nr, nc)
            for k in range(3):
                if cur_sim.b[k] == npos:
                    nnpos = MAKE_POS(nr + dr[act], nc + dc[act])
                    exp = False
                    for x in range(3):
                        if x_pos[x] == nnpos and not (cur_sim.mask & (1 << x)):
                            cur_sim.mask |= (1 << x)
                            cur_sim.b[k] = 255
                            exp = True
                            break
                    if not exp:
                        cur_sim.b[k] = nnpos
                if cur_sim.bx[k] == npos:
                    cur_sim.bx[k] = MAKE_POS(nr + dr[act], nc + dc[act])
            cur_sim.r = nr
            cur_sim.c = nc
    p1_states[p1_len] = SimState(r=cur_sim.r, c=cur_sim.c, mask=cur_sim.mask,
                                  b=list(cur_sim.b), bx=list(cur_sim.bx))

    # ===== C: 第1062-1356行 — 观察/识别阶段 =====
    if skip_flag == 0:
        if init_b_cnt == 0:
            # ===== C: 第1064-1223行 — 0炸弹TSP观察顺序 =====
            global tsp_pois, tsp_dist_matrix, tsp_min_cost, tsp_best_path, tsp_best_len, tsp_visited

            # 构建POI列表
            tsp_pois = [POI(r=cur_sim.r, c=cur_sim.c, b_mask=obs_b, t_mask=obs_t)]

            for r in range(MAX_R):
                for c in range(MAX_C):
                    if fast_wall[cur_sim.mask][r][c]:
                        continue
                    is_solid = False
                    for k in range(3):
                        if cur_sim.bx[k] != 255 and cur_sim.bx[k] == MAKE_POS(r, c):
                            is_solid = True
                    if is_solid:
                        continue

                    b_mask = 0
                    t_mask = 0
                    for d in range(4):
                        nr = r + dr[d]
                        nc = c + dc[d]
                        npos = MAKE_POS(nr, nc)
                        for k in range(3):
                            if not (obs_b & (1 << k)) and cur_sim.bx[k] == npos:
                                b_mask |= (1 << k)
                            if not (obs_t & (1 << k)) and dot_pos[k] == npos:
                                t_mask |= (1 << k)
                    if b_mask or t_mask:
                        dup = False
                        if r == tsp_pois[0].r and c == tsp_pois[0].c:
                            tsp_pois[0].b_mask |= b_mask
                            tsp_pois[0].t_mask |= t_mask
                            dup = True
                        if not dup and len(tsp_pois) < 64:
                            tsp_pois.append(POI(r=r, c=c, b_mask=b_mask, t_mask=t_mask))

            # 计算POI间距离矩阵
            n_pois = len(tsp_pois)
            tsp_dist_matrix = [[9999] * n_pois for _ in range(n_pois)]
            for i_poi in range(n_pois):
                tsp_dist_matrix[i_poi][i_poi] = 0

                p1_dist_vis_id += 1
                if p1_dist_vis_id == 0xFFFF:
                    for rr in range(MAX_R):
                        for cc in range(MAX_C):
                            p1_dist_vis[rr][cc] = 0
                    p1_dist_vis_id = 1

                qr_bfs = [0] * 256
                qc_bfs = [0] * 256
                h, t = 0, 0
                qr_bfs[t] = tsp_pois[i_poi].r
                qc_bfs[t] = tsp_pois[i_poi].c
                t += 1
                p1_dist_vis[tsp_pois[i_poi].r][tsp_pois[i_poi].c] = p1_dist_vis_id
                global_dist[tsp_pois[i_poi].r][tsp_pois[i_poi].c] = 0

                while h < t:
                    r = qr_bfs[h]
                    c = qc_bfs[h]
                    h += 1
                    for j in range(n_pois):
                        if tsp_pois[j].r == r and tsp_pois[j].c == c:
                            tsp_dist_matrix[i_poi][j] = global_dist[r][c]
                    for d in range(4):
                        nr = r + dr[d]
                        nc = c + dc[d]
                        if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                            continue
                        if fast_wall[cur_sim.mask][nr][nc]:
                            continue
                        solid = False
                        for k in range(3):
                            if cur_sim.bx[k] != 255 and cur_sim.bx[k] == MAKE_POS(nr, nc):
                                solid = True
                        if not solid and p1_dist_vis[nr][nc] != p1_dist_vis_id:
                            p1_dist_vis[nr][nc] = p1_dist_vis_id
                            global_dist[nr][nc] = global_dist[r][c] + 1
                            qr_bfs[t] = nr
                            qc_bfs[t] = nc
                            t += 1

            tsp_min_cost = 999999
            tsp_best_len = 0
            tsp_visited = [False] * n_pois
            tsp_visited[0] = True
            temp_path = [0] * 32
            _dfs_tsp_0bomb(0, 0, obs_b, obs_t, 0, temp_path,
                           init_bx_cnt, init_dot_cnt, cur_sim, current_look_dir)

            if tsp_min_cost == 999999:
                return -1.0

            # 执行TSP最优路径
            curr_node_tsp = 0
            for i_tsp in range(tsp_best_len):
                next_node = tsp_best_path[i_tsp]
                # BFS找路径
                p1_dist_vis_id += 1
                if p1_dist_vis_id == 0xFFFF:
                    for rr in range(MAX_R):
                        for cc in range(MAX_C):
                            p1_dist_vis[rr][cc] = 0
                    p1_dist_vis_id = 1
                qr_bfs = [0] * 256
                qc_bfs = [0] * 256
                h, t = 0, 0
                qr_bfs[t] = tsp_pois[curr_node_tsp].r
                qc_bfs[t] = tsp_pois[curr_node_tsp].c
                t += 1
                p1_dist_vis[tsp_pois[curr_node_tsp].r][tsp_pois[curr_node_tsp].c] = p1_dist_vis_id

                while h < t:
                    r = qr_bfs[h]
                    c = qc_bfs[h]
                    h += 1
                    if r == tsp_pois[next_node].r and c == tsp_pois[next_node].c:
                        break
                    for d in range(4):
                        nr = r + dr[d]
                        nc = c + dc[d]
                        if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                            continue
                        if fast_wall[cur_sim.mask][nr][nc]:
                            continue
                        solid = False
                        for k in range(3):
                            if cur_sim.bx[k] != 255 and cur_sim.bx[k] == MAKE_POS(nr, nc):
                                solid = True
                        if not solid and p1_dist_vis[nr][nc] != p1_dist_vis_id:
                            p1_dist_vis[nr][nc] = p1_dist_vis_id
                            global_parent[nr][nc] = MAKE_POS(r, c)
                            global_action[nr][nc] = d
                            qr_bfs[t] = nr
                            qc_bfs[t] = nc
                            t += 1

                walk_path = [0] * 256
                walk_len = 0
                cr = tsp_pois[next_node].r
                cc = tsp_pois[next_node].c
                while cr != tsp_pois[curr_node_tsp].r or cc != tsp_pois[curr_node_tsp].c:
                    walk_path[walk_len] = global_action[cr][cc]
                    walk_len += 1
                    p = global_parent[cr][cc]
                    cr = GET_R(p)
                    cc = GET_C(p)

                for j in range(walk_len - 1, -1, -1):
                    p1_states[p1_len] = SimState(r=cur_sim.r, c=cur_sim.c, mask=cur_sim.mask,
                                                  b=list(cur_sim.b), bx=list(cur_sim.bx))
                    p1_acts[p1_len] = walk_path[j]
                    p1_len += 1
                    cur_sim.r += dr[walk_path[j]]
                    cur_sim.c += dc[walk_path[j]]

                # 执行观察动作
                add_b = tsp_pois[next_node].b_mask & ~obs_b
                add_t = tsp_pois[next_node].t_mask & ~obs_t

                req_looks = []
                for d in range(4):
                    nr = cur_sim.r + dr[d]
                    nc = cur_sim.c + dc[d]
                    npos = MAKE_POS(nr, nc)
                    need_look = False
                    for k in range(3):
                        if (add_b & (1 << k)) and cur_sim.bx[k] == npos:
                            need_look = True
                        if (add_t & (1 << k)) and dot_pos[k] == npos:
                            need_look = True
                    if need_look:
                        req_looks.append(d)

                if req_looks:
                    has_curr = any(rl == current_look_dir for rl in req_looks)
                    if has_curr:
                        p1_states[p1_len] = SimState(r=cur_sim.r, c=cur_sim.c, mask=cur_sim.mask,
                                                      b=list(cur_sim.b), bx=list(cur_sim.bx))
                        p1_acts[p1_len] = 5 + current_look_dir
                        p1_len += 1

                    for rl in req_looks:
                        if rl == current_look_dir and has_curr:
                            has_curr = False  # 只执行一次当前方向的
                            continue
                        if rl == current_look_dir:
                            continue
                        p1_states[p1_len] = SimState(r=cur_sim.r, c=cur_sim.c, mask=cur_sim.mask,
                                                      b=list(cur_sim.b), bx=list(cur_sim.bx))
                        p1_acts[p1_len] = 5 + rl
                        p1_len += 1
                        current_look_dir = rl

                obs_b |= add_b
                obs_t |= add_t
                curr_node_tsp = next_node

            p1_states[p1_len] = SimState(r=cur_sim.r, c=cur_sim.c, mask=cur_sim.mask,
                                          b=list(cur_sim.b), bx=list(cur_sim.bx))

        else:
            # ===== C: 第1225-1355行 — 有炸弹时的贪心观察收集 =====
            while True:
                cb = count_bits(obs_b)
                ct = count_bits(obs_t)
                satisfied = False
                req_b = init_bx_cnt
                req_t = init_dot_cnt
                if req_b == 3 and req_t == 3:
                    if TRACK_NUM == 4:
                        satisfied = (cb >= 2 and ct >= 2)
                    elif TRACK_NUM == 5:
                        satisfied = ((cb == 2 and ct == 3) or (cb == 3 and ct == 2))
                    else:
                        satisfied = (cb == 3 and ct == 3)
                else:
                    satisfied = (cb == req_b and ct == req_t)
                if satisfied:
                    break

                best_cost = 999999
                best_idx = -1
                best_see_type = -1
                best_see_id = -1
                best_look_act = -1
                best_path = [0] * 512
                best_path_len_val = 0
                max_quota = 2 if TRACK_NUM == 4 else 3
                cur_cb = count_bits(obs_b)
                cur_ct = count_bits(obs_t)

                for i_state in range(p1_len + 1):
                    st = p1_states[i_state]
                    p1_dist_vis_id += 1
                    if p1_dist_vis_id == 0xFFFF:
                        for rr in range(MAX_R):
                            for cc in range(MAX_C):
                                p1_dist_vis[rr][cc] = 0
                        p1_dist_vis_id = 1

                    qr_bfs = [0] * 512
                    qc_bfs = [0] * 512
                    h, t = 0, 0
                    qr_bfs[t] = st.r
                    qc_bfs[t] = st.c
                    t += 1
                    global_dist[st.r][st.c] = 0
                    p1_dist_vis[st.r][st.c] = p1_dist_vis_id

                    while h < t:
                        cr = qr_bfs[h]
                        cc = qc_bfs[h]
                        h += 1
                        for d in range(4):
                            nr = cr + dr[d]
                            nc = cc + dc[d]
                            if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                                continue
                            if fast_wall[st.mask][nr][nc]:
                                continue
                            solid = False
                            for k in range(3):
                                if ((st.b[k] != 255 and st.b[k] == MAKE_POS(nr, nc))
                                        or (st.bx[k] != 255 and st.bx[k] == MAKE_POS(nr, nc))):
                                    solid = True
                            if not solid and p1_dist_vis[nr][nc] != p1_dist_vis_id:
                                p1_dist_vis[nr][nc] = p1_dist_vis_id
                                global_dist[nr][nc] = global_dist[cr][cc] + 1
                                global_parent[nr][nc] = MAKE_POS(cr, cc)
                                qr_bfs[t] = nr
                                qc_bfs[t] = nc
                                t += 1

                    for k in range(3):
                        check_pts = [[-1, -1], [-1, -1]]
                        check_type = [-1, -1]
                        pts_cnt = 0

                        if st.bx[k] != 255 and not (obs_b & (1 << k)) and cur_cb < max_quota:
                            check_pts[pts_cnt][0] = GET_R(st.bx[k])
                            check_pts[pts_cnt][1] = GET_C(st.bx[k])
                            check_type[pts_cnt] = 0
                            pts_cnt += 1
                        if dot_pos[k] != 255 and not (obs_t & (1 << k)) and cur_ct < max_quota:
                            check_pts[pts_cnt][0] = GET_R(dot_pos[k])
                            check_pts[pts_cnt][1] = GET_C(dot_pos[k])
                            check_type[pts_cnt] = 1
                            pts_cnt += 1

                        for pt in range(pts_cnt):
                            tr = check_pts[pt][0]
                            tc = check_pts[pt][1]
                            for look in range(4):
                                ld = p_dir[look]
                                ar = tr - dr[ld]
                                ac = tc - dc[ld]
                                if (ar >= 0 and ar < MAX_R and ac >= 0 and ac < MAX_C
                                        and p1_dist_vis[ar][ac] == p1_dist_vis_id):
                                    posture_penalty = 0 if ld == current_look_dir else 5
                                    back_dist = 0 if i_state == p1_len else global_dist[ar][ac]
                                    cost = (global_dist[ar][ac] + posture_penalty + 1 + back_dist) * 1000 - i_state

                                    if cost < best_cost:
                                        best_cost = cost
                                        best_idx = i_state
                                        best_see_type = check_type[pt]
                                        best_see_id = k
                                        best_look_act = 5 + ld
                                        best_path_len_val = 0
                                        cr_path = ar
                                        cc_path = ac
                                        while cr_path != st.r or cc_path != st.c:
                                            p = global_parent[cr_path][cc_path]
                                            pr = GET_R(p)
                                            pc = GET_C(p)
                                            for dir_i in range(4):
                                                if pr + dr[dir_i] == cr_path and pc + dc[dir_i] == cc_path:
                                                    best_path[best_path_len_val] = dir_i
                                                    best_path_len_val += 1
                                                    break
                                            cr_path = pr
                                            cc_path = pc
                                        # 翻转路径
                                        for x in range(best_path_len_val // 2):
                                            tmp = best_path[x]
                                            best_path[x] = best_path[best_path_len_val - 1 - x]
                                            best_path[best_path_len_val - 1 - x] = tmp

                if best_idx == -1:
                    break

                # 注入观察动作
                det_st = SimState(r=p1_states[best_idx].r, c=p1_states[best_idx].c,
                                  mask=p1_states[best_idx].mask,
                                  b=list(p1_states[best_idx].b),
                                  bx=list(p1_states[best_idx].bx))
                jump_r, jump_c = det_st.r, det_st.c
                for j in range(best_path_len_val):
                    det_st.r += dr[best_path[j]]
                    det_st.c += dc[best_path[j]]

                back_path = [0] * 512
                back_len = 0
                if best_idx < p1_len:
                    p1_dist_vis_id += 1
                    if p1_dist_vis_id == 0xFFFF:
                        for rr in range(MAX_R):
                            for cc in range(MAX_C):
                                p1_dist_vis[rr][cc] = 0
                        p1_dist_vis_id = 1
                    qr_bfs = [0] * 256
                    qc_bfs = [0] * 256
                    h, t = 0, 0
                    qr_bfs[t] = det_st.r
                    qc_bfs[t] = det_st.c
                    t += 1
                    global_dist[det_st.r][det_st.c] = 0
                    p1_dist_vis[det_st.r][det_st.c] = p1_dist_vis_id

                    while h < t:
                        r = qr_bfs[h]
                        c = qc_bfs[h]
                        h += 1
                        if r == jump_r and c == jump_c:
                            back_len = global_dist[jump_r][jump_c]
                            cr_path = jump_r
                            cc_path = jump_c
                            for ii in range(back_len - 1, -1, -1):
                                back_path[ii] = global_action[cr_path][cc_path]
                                p = global_parent[cr_path][cc_path]
                                cr_path = GET_R(p)
                                cc_path = GET_C(p)
                            break
                        for d in range(4):
                            nr = r + dr[d]
                            nc = c + dc[d]
                            if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                                continue
                            if fast_wall[det_st.mask][nr][nc]:
                                continue
                            blocked = False
                            for k in range(3):
                                if ((det_st.b[k] != 255 and det_st.b[k] == MAKE_POS(nr, nc))
                                        or (det_st.bx[k] != 255 and det_st.bx[k] == MAKE_POS(nr, nc))):
                                    blocked = True
                            if not blocked and p1_dist_vis[nr][nc] != p1_dist_vis_id:
                                p1_dist_vis[nr][nc] = p1_dist_vis_id
                                global_dist[nr][nc] = global_dist[r][c] + 1
                                global_parent[nr][nc] = MAKE_POS(r, c)
                                global_action[nr][nc] = d
                                qr_bfs[t] = nr
                                qc_bfs[t] = nc
                                t += 1

                inject_len = best_path_len_val + 1 + back_len
                for j in range(p1_len, best_idx - 1, -1):
                    if j + inject_len < MAX_STEPS:
                        p1_acts[j + inject_len] = p1_acts[j]
                        p1_states[j + inject_len] = SimState(r=p1_states[j].r, c=p1_states[j].c,
                                                              mask=p1_states[j].mask,
                                                              b=list(p1_states[j].b),
                                                              bx=list(p1_states[j].bx))

                p_idx = best_idx
                det_st2 = SimState(r=p1_states[best_idx].r, c=p1_states[best_idx].c,
                                   mask=p1_states[best_idx].mask,
                                   b=list(p1_states[best_idx].b),
                                   bx=list(p1_states[best_idx].bx))
                for j in range(best_path_len_val):
                    p1_states[p_idx] = SimState(r=det_st2.r, c=det_st2.c, mask=det_st2.mask,
                                                 b=list(det_st2.b), bx=list(det_st2.bx))
                    p1_acts[p_idx] = best_path[j]
                    p_idx += 1
                    det_st2.r += dr[best_path[j]]
                    det_st2.c += dc[best_path[j]]
                p1_states[p_idx] = SimState(r=det_st2.r, c=det_st2.c, mask=det_st2.mask,
                                             b=list(det_st2.b), bx=list(det_st2.bx))
                p1_acts[p_idx] = best_look_act
                p_idx += 1

                current_look_dir = best_look_act - 5

                for j in range(back_len):
                    p1_states[p_idx] = SimState(r=det_st2.r, c=det_st2.c, mask=det_st2.mask,
                                                 b=list(det_st2.b), bx=list(det_st2.bx))
                    p1_acts[p_idx] = back_path[j]
                    p_idx += 1
                    det_st2.r += dr[back_path[j]]
                    det_st2.c += dc[back_path[j]]

                if best_idx == p1_len:
                    p1_states[p1_len + inject_len] = SimState(r=det_st2.r, c=det_st2.c,
                                                               mask=det_st2.mask,
                                                               b=list(det_st2.b),
                                                               bx=list(det_st2.bx))
                p1_len += inject_len
                if best_see_type == 0:
                    obs_b |= (1 << best_see_id)
                else:
                    obs_t |= (1 << best_see_id)

    # ===== C: 第1358-1485行 — P2阶段：枚举6种推箱排列 =====
    if final_st:
        # 写回final_st（通过对象属性修改）
        final_st.r = p1_states[p1_len].r
        final_st.c = p1_states[p1_len].c
        final_st.mask = p1_states[p1_len].mask
        final_st.b = list(p1_states[p1_len].b)
        final_st.bx = list(p1_states[p1_len].bx)

    p1_turns = count_path_turns(p1_acts, p1_len)
    p1_score = float(p1_len) + 3.0 * p1_turns

    cur_assign = [BOX_TARGET_MAP[0], BOX_TARGET_MAP[1], BOX_TARGET_MAP[2]]
    best_score_for_assign = 999999.0
    acts_len_for_assign = 0
    best_acts_for_assign = [0] * MAX_STEPS

    for ord_i in range(6):
        s2 = SimState(r=p1_states[p1_len].r, c=p1_states[p1_len].c,
                      mask=p1_states[p1_len].mask,
                      b=list(p1_states[p1_len].b),
                      bx=list(p1_states[p1_len].bx))
        cur_acts = [0] * MAX_STEPS
        cur_len = 0
        possible = True

        for step in range(3):
            b_id = perms[ord_i][step]
            if s2.bx[b_id] == 255:
                continue

            t_id = cur_assign[b_id]
            tr = GET_R(dot_pos[t_id])
            tc = GET_C(dot_pos[t_id])

            p2_vis_id += 1
            if p2_vis_id == 0xFFFF:
                _rebuild_p2_vis()
                p2_vis_id = 1

            # 初始化P2 A*搜索
            p2_heap_size = 0
            t_idx = 0

            rq_p2[t_idx].rr = s2.r
            rq_p2[t_idx].rc = s2.c
            rq_p2[t_idx].br = GET_R(s2.bx[b_id])
            rq_p2[t_idx].bc = GET_C(s2.bx[b_id])
            rq_p2[t_idx].parent = -1
            rq_p2[t_idx].action = -1
            rq_p2[t_idx].dist = 0

            p2_f_array[t_idx] = 0
            _p2_vis_set(s2.r, s2.c, rq_p2[t_idx].br, rq_p2[t_idx].bc, p2_vis_id)

            _push_p2(t_idx)
            t_idx += 1

            goal = -1

            while p2_heap_size > 0:
                curr_p2 = _pop_p2()
                cqn = rq_p2[curr_p2]

                if cqn.br == tr and cqn.bc == tc:
                    goal = curr_p2
                    break

                for d in range(4):
                    nrr = cqn.rr + dr[d]
                    nrc = cqn.rc + dc[d]
                    nbr = cqn.br
                    nbc = cqn.bc

                    if nrr < 0 or nrr >= MAX_R or nrc < 0 or nrc >= MAX_C:
                        continue
                    if global_base_wall[nrr][nrc] and not (7 & global_exp_cov[nrr][nrc]):
                        continue

                    blocked = False
                    for k in range(3):
                        if k != b_id and s2.bx[k] != 255 and s2.bx[k] == MAKE_POS(nrr, nrc):
                            blocked = True
                    if blocked:
                        continue

                    # 推动箱子
                    if nrr == cqn.br and nrc == cqn.bc:
                        nbr += dr[d]
                        nbc += dc[d]
                        if nbr < 0 or nbr >= MAX_R or nbc < 0 or nbc >= MAX_C:
                            continue
                        if global_base_wall[nbr][nbc] and not (7 & global_exp_cov[nbr][nbc]):
                            continue
                        for k in range(3):
                            if k != b_id and s2.bx[k] != 255 and s2.bx[k] == MAKE_POS(nbr, nbc):
                                blocked = True
                        if blocked:
                            continue

                    if _p2_vis_get(nrr, nrc, nbr, nbc) != p2_vis_id:
                        new_dist = cqn.dist + 1
                        if cur_len + new_dist > best_score_for_assign:
                            continue

                        _p2_vis_set(nrr, nrc, nbr, nbc, p2_vis_id)

                        if t_idx < MAX_P2_Q:
                            rq_p2[t_idx].rr = nrr
                            rq_p2[t_idx].rc = nrc
                            rq_p2[t_idx].br = nbr
                            rq_p2[t_idx].bc = nbc
                            rq_p2[t_idx].parent = curr_p2
                            rq_p2[t_idx].action = d
                            rq_p2[t_idx].dist = new_dist

                            h_box = bfs_dist_dot[t_id][nbr][nbc]
                            if h_box == 9999:
                                continue

                            h_robot = abs(nrr - nbr) + abs(nrc - nbc)
                            p2_f_array[t_idx] = new_dist + h_box * 1 + h_robot

                            _push_p2(t_idx)
                            t_idx += 1

            if goal == -1:
                possible = False
                break

            temp_p = [0] * 1000
            t_len = 0
            c_ptr = goal
            while rq_p2[c_ptr].parent != -1:
                temp_p[t_len] = rq_p2[c_ptr].action
                t_len += 1
                c_ptr = rq_p2[c_ptr].parent
            for x in range(t_len - 1, -1, -1):
                cur_acts[cur_len] = temp_p[x]
                cur_len += 1
            s2.r = rq_p2[goal].rr
            s2.c = rq_p2[goal].rc
            s2.bx[b_id] = 255

        if possible:
            cur_turns = count_path_turns(cur_acts, cur_len)
            cur_score = float(cur_len) + 4.0 * cur_turns
            if cur_score < best_score_for_assign:
                best_score_for_assign = cur_score
                acts_len_for_assign = cur_len
                for i_copy in range(cur_len):
                    best_acts_for_assign[i_copy] = cur_acts[i_copy]

    if best_score_for_assign >= 999999.0:
        return -1.0

    len_ptr = [p1_len]
    remove_duplicate_looks(p1_acts, len_ptr)
    p1_len = len_ptr[0]
    out_p1_steps[0] = p1_len

    temp_p2_len = acts_len_for_assign
    for i_copy in range(acts_len_for_assign):
        temp_p2_acts[i_copy] = best_acts_for_assign[i_copy]

    t_p2_bfs += _get_time_s(start_p2, _get_current_ms())
    return p1_score + best_score_for_assign


# ==================== 动态推箱BFS距离（C: 第1487-1523行） ====================

def get_dynamic_bx_dist(bombs, b_cnt: int) -> int:
    """C: get_dynamic_bx_dist (约1487-1523行)
    模拟爆炸后每个推箱到各自目标的最短距离之和"""
    wall_dyn = [[False] * MAX_C for _ in range(MAX_R)]
    for r in range(MAX_R):
        for c in range(MAX_C):
            wall_dyn[r][c] = (original_map[r][c] == '#')
    for i in range(b_cnt):
        br, bc = bombs[i][0], bombs[i][1]
        for dr_i in range(-1, 2):
            for dc_i in range(-1, 2):
                nr, nc = br + dr_i, bc + dc_i
                if nr > 0 and nr < MAX_R - 1 and nc > 0 and nc < MAX_C - 1:
                    wall_dyn[nr][nc] = False

    total_dist = 0
    for i in range(init_bx_cnt):
        vis = [[False] * MAX_C for _ in range(MAX_R)]
        qr = [0] * 256
        qc = [0] * 256
        dist_arr = [0] * 256
        head, tail = 0, 0

        qr[tail] = init_bx[i][0]
        qc[tail] = init_bx[i][1]
        dist_arr[tail] = 0
        vis[qr[tail]][qc[tail]] = True
        tail += 1

        target_r, target_c = init_dot[i][0], init_dot[i][1]
        min_d = 9999

        while head < tail:
            r = qr[head]
            c = qc[head]
            d_val = dist_arr[head]
            head += 1
            if r == target_r and c == target_c:
                min_d = d_val
                break
            for dir_i in range(4):
                nr = r + dr[dir_i]
                nc = c + dc[dir_i]
                if (nr >= 0 and nr < MAX_R and nc >= 0 and nc < MAX_C
                        and not wall_dyn[nr][nc] and not vis[nr][nc]):
                    vis[nr][nc] = True
                    qr[tail] = nr
                    qc[tail] = nc
                    dist_arr[tail] = d_val + 1
                    tail += 1

        if min_d == 9999:
            return 9999
        total_dist += min_d
    return total_dist


# ==================== 凸包相关（C: 第146-181行） ====================

def cross_product_2x(r1: int, c1: int, r2: int, c2: int, r3: int, c3: int) -> int:
    """C: cross_product_2x (约146-148行)"""
    return (r2 - r1) * (c3 - c1) - (c2 - c1) * (r3 - r1)


def is_point_in_hull_2x(r: int, c: int, hull: list, hull_size: int) -> bool:
    """C: is_point_in_hull_2x (约150-157行)"""
    has_pos = False
    has_neg = False
    for i in range(hull_size):
        cp = cross_product_2x(hull[i].r * 2, hull[i].c * 2,
                              hull[(i + 1) % hull_size].r * 2,
                              hull[(i + 1) % hull_size].c * 2, r, c)
        if cp > 0:
            has_pos = True
        if cp < 0:
            has_neg = True
    return not (has_pos and has_neg)


def is_wall_intersect_hull(r: int, c: int, hull: list, hull_size: int) -> bool:
    """C: is_wall_intersect_hull (约159-166行)"""
    corners = [
        [int(2 * r - 1.5), int(2 * c - 1.5)],
        [int(2 * r - 1.5), int(2 * c + 1.5)],
        [int(2 * r + 1.5), int(2 * c - 1.5)],
        [int(2 * r + 1.5), int(2 * c + 1.5)],
    ]
    for i in range(4):
        if is_point_in_hull_2x(corners[i][0], corners[i][1], hull, hull_size):
            return True
    return False


def get_convex_hull(pts: list, n: int, hull: list) -> int:
    """C: get_convex_hull (约168-181行)
    Gift wrapping算法求凸包，返回凸包点数"""
    if n < 3:
        return 0
    # 找最左点
    l = 0
    for i in range(1, n):
        if pts[i].c < pts[l].c:
            l = i
    p = l
    cnt = 0
    while True:
        hull[cnt] = pts[p]
        cnt += 1
        q = (p + 1) % n
        for i in range(n):
            cp = ((pts[i].r - pts[p].r) * (pts[q].c - pts[p].c)
                  - (pts[i].c - pts[p].c) * (pts[q].r - pts[p].r))
            if cp > 0:
                q = i
        p = q
        if p == l:
            break
    return cnt


# ==================== is_dist_ok（C: 第144行） ====================

def is_dist_ok(r1: int, c1: int, r2: int, c2: int) -> bool:
    """C: is_dist_ok (约144行)
    两点欧氏距离平方 >= 4（即曼哈顿/欧氏距离 >= 2）"""
    return ((r1 - r2) * (r1 - r2) + (c1 - c2) * (c1 - c2)) >= 4


# ==================== run_phase1: 阶段一入口（C: 第1525-1754行） ====================

def run_phase1(raw_map, p1_res: PhaseResult, mode: int, skip_flag: int) -> bool:
    """C: run_phase1 (约1525-1754行)
    阶段一主入口：解析地图、枚举炸弹组合、搜索最优解

    Args:
        raw_map: 12×16 char数组（字符串列表或嵌套列表）
        p1_res: PhaseResult输出对象（通过属性修改）
        mode: 模式（实际被TRACK_NUM覆盖）
        skip_flag: 跳过标志（0=正常, 非0=跳过开局势头观察）

    Returns:
        bool: True=找到解, False=无解
    """
    global init_b_cnt, init_bx_cnt, init_dot_cnt
    global init_robot_r, init_robot_c
    global init_b, init_bx, init_dot
    global dot_pos, dot_cnt
    global t_setup_and_hull, t_filter1_bfs, t_filter3_dist, t_filter2_prune
    global t_p1_astar, t_p2_bfs, p1_exec_cnt, p2_exec_cnt
    global base_wall_initialized, global_base_wall
    global global_can_be_first, relaxed_dist_b
    global g_best_bombs, g_best_p1_acts, g_best_p1_len, g_best_p2_acts, g_best_p2_len
    global g_final_p1_state

    t0 = _get_current_ms()
    mode = TRACK_NUM  # C: 第1527行 — 强制覆盖

    # C: 第1528-1543行 — 解析地图
    key_pts = []
    init_b_cnt = 0
    init_bx_cnt = 0
    init_dot_cnt = 0
    base_wall_initialized = False

    for r in range(MAX_R):
        for c in range(MAX_C):
            ch = raw_map[r][c]
            original_map[r][c] = ch
            global_base_wall[r][c] = (ch == '#')
            if ch == '@' or ch == '+':
                init_robot_r = r
                init_robot_c = c
            if ch == '*' and init_b_cnt < MAX_ITEMS:
                init_b[init_b_cnt][0] = r
                init_b[init_b_cnt][1] = c
                init_b_cnt += 1
            if ch == '$' and init_bx_cnt < MAX_ITEMS:
                init_bx[init_bx_cnt][0] = r
                init_bx[init_bx_cnt][1] = c
                init_bx_cnt += 1
            if (ch == '.' or ch == '+') and init_dot_cnt < MAX_ITEMS:
                init_dot[init_dot_cnt][0] = r
                init_dot[init_dot_cnt][1] = c
                init_dot_cnt += 1
            if ch in ('@', '+', '*', '$', '.'):
                key_pts.append(Coord(r=r, c=c))

    # C: 第1545-1565行 — 炸弹箱宽松BFS距离（不用爆炸掩码）
    for i in range(init_b_cnt):
        for r in range(MAX_R):
            for c in range(MAX_C):
                relaxed_dist_b[i][r][c] = 9999
        qr = [0] * 256
        qc = [0] * 256
        head, tail = 0, 0
        qr[tail] = init_b[i][0]
        qc[tail] = init_b[i][1]
        tail += 1
        relaxed_dist_b[i][init_b[i][0]][init_b[i][1]] = 0

        while head < tail:
            r = qr[head]
            c = qc[head]
            head += 1
            for d in range(4):
                nr = r + dr[d]
                nc = c + dc[d]
                if nr >= 0 and nr < MAX_R and nc >= 0 and nc < MAX_C:
                    is_border_wall = ((nr == 0 or nr == MAX_R - 1 or nc == 0 or nc == MAX_C - 1)
                                      and global_base_wall[nr][nc])
                    if not is_border_wall and relaxed_dist_b[i][nr][nc] == 9999:
                        relaxed_dist_b[i][nr][nc] = relaxed_dist_b[i][r][c] + 1
                        qr[tail] = nr
                        qc[tail] = nc
                        tail += 1
    base_wall_initialized = True

    # C: 第1567-1569行 — 设置目标点编码
    for i in range(init_dot_cnt):
        dot_pos[i] = MAKE_POS(init_dot[i][0], init_dot[i][1])
    for i in range(init_dot_cnt, 3):
        dot_pos[i] = 255
    dot_cnt = init_dot_cnt

    # C: 第1571-1580行 — 凸包提取 + 候选墙池
    hull_temp = [Coord() for _ in range(30)]
    hull_size = get_convex_hull(key_pts, len(key_pts), hull_temp)
    pool = []
    for r in range(1, MAX_R - 1):
        for c in range(1, MAX_C - 1):
            if original_map[r][c] == '#':
                if is_wall_intersect_hull(r, c, hull_temp, hull_size):
                    pool.append([r, c])
    pool_size = len(pool)

    t_setup_and_hull = _get_time_s(t0, _get_current_ms())

    # C: 第1581-1590行 — 构建can_be_first标记
    for r in range(MAX_R):
        for c in range(MAX_C):
            global_can_be_first[r][c] = False

    for i in range(pool_size):
        target = [[pool[i][0], pool[i][1]]]
        base_wall = [[False] * MAX_C for _ in range(MAX_R)]
        build_opt_wall_and_rigid_bodies([], 0, target, 1, base_wall)
        if run_macro_pull(target, 1, base_wall, init_b, init_b_cnt) > 0:
            global_can_be_first[pool[i][0]][pool[i][1]] = True

    # C: 第1592-1594行 — 初始化最优解跟踪
    global_best_metric = 999999.0
    best_p1_steps = 0
    total_combos = 0
    pruned_combos = 0
    tested_combos = 0
    success_cnt = 0
    base_bx_dist = get_manhattan_dist(init_bx_cnt, init_bx, init_dot_cnt, init_dot)

    print("====== 开始暴力枚举 (双轮自适应放宽 + 快速剪枝Warp) ======")
    start_time = _get_current_ms()
    timeout_triggered = False

    # C: 第1601-1703行 — 双轮自适应搜索
    node_limits = [MAX_NODES_1, MAX_NODES]

    for lim_idx in range(2):
        active_max_nodes = node_limits[lim_idx]
        print(f"\r\n>> [双轮搜索] 第 {lim_idx + 1} 轮，当前节点限制: MAX_NODES = {active_max_nodes} ...")

        total_combos = 0
        pruned_combos = 0
        tested_combos = 0
        success_cnt = 0
        global_best_metric = 999999.0

        lim_i = pool_size if init_b_cnt >= 1 else 1
        for i in range(lim_i):
            start_j = i + 1 if init_b_cnt >= 2 else 0
            lim_j = pool_size if init_b_cnt >= 2 else 1
            for j in range(start_j, lim_j):
                start_k = j + 1 if init_b_cnt >= 3 else 0
                lim_k = pool_size if init_b_cnt >= 3 else 1
                for k in range(start_k, lim_k):

                    # C: 第1622-1624行 — 距离检查
                    if init_b_cnt >= 2 and not is_dist_ok(pool[i][0], pool[i][1],
                                                           pool[j][0], pool[j][1]):
                        continue
                    if init_b_cnt >= 3 and not is_dist_ok(pool[i][0], pool[i][1],
                                                           pool[k][0], pool[k][1]):
                        continue
                    if init_b_cnt >= 3 and not is_dist_ok(pool[j][0], pool[j][1],
                                                           pool[k][0], pool[k][1]):
                        continue

                    total_combos += 1
                    bombs = [[0, 0] for _ in range(3)]
                    bombs[0][0] = pool[i][0] if init_b_cnt >= 1 else 0
                    bombs[0][1] = pool[i][1] if init_b_cnt >= 1 else 0
                    bombs[1][0] = pool[j][0] if init_b_cnt >= 2 else 0
                    bombs[1][1] = pool[j][1] if init_b_cnt >= 2 else 0
                    bombs[2][0] = pool[k][0] if init_b_cnt >= 3 else 0
                    bombs[2][1] = pool[k][1] if init_b_cnt >= 3 else 0

                    # C: 第1635-1637行 — 过滤器1: 快速前向BFS
                    t_tmp = _get_current_ms()
                    if fast_forward_bfs_check(bombs, init_b_cnt):
                        t_filter1_bfs += _get_time_s(t_tmp, _get_current_ms())
                        pruned_combos += 1
                        continue
                    t_filter1_bfs += _get_time_s(t_tmp, _get_current_ms())

                    # C: 第1639-1668行 — 过滤器3: 距离下界剪枝
                    t_tmp = _get_current_ms()
                    if global_best_metric < 999999.0:
                        b_dist = 99999
                        if init_b_cnt == 3:
                            for m in range(6):
                                cost = (relaxed_dist_b[0][bombs[perms[m][0]][0]][bombs[perms[m][0]][1]]
                                        + relaxed_dist_b[1][bombs[perms[m][1]][0]][bombs[perms[m][1]][1]]
                                        + relaxed_dist_b[2][bombs[perms[m][2]][0]][bombs[perms[m][2]][1]])
                                if cost < b_dist:
                                    b_dist = cost
                        elif init_b_cnt == 2:
                            for m in range(2):
                                cost = (relaxed_dist_b[0][bombs[0 if m == 0 else 1][0]][bombs[0 if m == 0 else 1][1]]
                                        + relaxed_dist_b[1][bombs[1 if m == 0 else 0][0]][bombs[1 if m == 0 else 0][1]])
                                if cost < b_dist:
                                    b_dist = cost
                        elif init_b_cnt == 1:
                            b_dist = relaxed_dist_b[0][bombs[0][0]][bombs[0][1]]
                        else:
                            b_dist = 0

                        dynamic_box_dist = get_dynamic_bx_dist(bombs, init_b_cnt)

                        if float(b_dist + dynamic_box_dist) >= global_best_metric:
                            t_filter3_dist += _get_time_s(t_tmp, _get_current_ms())
                            pruned_combos += 1
                            continue
                    t_filter3_dist += _get_time_s(t_tmp, _get_current_ms())

                    # C: 第1670-1672行 — 过滤器2: 快速剪枝
                    t_tmp = _get_current_ms()
                    if fast_prune_check(bombs, init_b_cnt):
                        t_filter2_prune += _get_time_s(t_tmp, _get_current_ms())
                        pruned_combos += 1
                        continue
                    t_filter2_prune += _get_time_s(t_tmp, _get_current_ms())

                    # C: 第1674-1687行 — 核心求解
                    tested_combos += 1
                    temp_p1_len_ptr = [0]
                    temp_final_st = SimState()
                    metric = solve_with_bombs(bombs, temp_p1_len_ptr, temp_final_st,
                                              start_time, active_max_nodes,
                                              global_best_metric, skip_flag)

                    if metric >= 0.0:
                        success_cnt += 1
                        if metric < global_best_metric:
                            global_best_metric = metric
                            best_p1_steps = temp_p1_len_ptr[0]
                            g_final_p1_state = SimState(r=temp_final_st.r, c=temp_final_st.c,
                                                        mask=temp_final_st.mask,
                                                        b=list(temp_final_st.b),
                                                        bx=list(temp_final_st.bx))
                            for bx_i in range(3):
                                g_best_bombs[bx_i][0] = bombs[bx_i][0]
                                g_best_bombs[bx_i][1] = bombs[bx_i][1]
                            for ai in range(best_p1_steps):
                                g_best_p1_acts[ai] = p1_acts[ai]
                            g_best_p2_len = temp_p2_len
                            for ai in range(temp_p2_len):
                                g_best_p2_acts[ai] = temp_p2_acts[ai]

                    if _get_current_ms() - start_time > MAX_SOLVE_TIME_MS:
                        timeout_triggered = True
                        break
                if timeout_triggered:
                    break
            if timeout_triggered:
                break

        if success_cnt > 0:
            print(f">> [双轮搜索] 在 MAX_NODES = {active_max_nodes} 档次找到有效通关解，提前终止放宽。")
            break
        if timeout_triggered:
            break

    # C: 第1705-1753行 — 输出统计和结果
    print("")
    print("======================================================")
    total_time = (t_setup_and_hull + t_filter1_bfs + t_filter3_dist
                  + t_filter2_prune + t_p1_astar + t_p2_bfs)
    print("")
    print("================ 搜索性能统计 ================")
    print(f"1. 地图预处理和凸包提取  : {t_setup_and_hull:8.4f} 秒")
    print(f"2. 过滤器1 (基本连通BFS) : {t_filter1_bfs:8.4f} 秒")
    print(f"3. 过滤器3 (距离下界)   : {t_filter3_dist:8.4f} 秒")
    print(f"4. 过滤器2 (宏拉取剪枝) : {t_filter2_prune:8.4f} 秒")
    print(f"5. 核心求解 P1 (A*搜索): {t_p1_astar:8.4f} 秒 (执行 {p1_exec_cnt} 次)")
    print(f"6. 核心求解 P2 (A*搬运) : {t_p2_bfs:8.4f} 秒 (执行 {p2_exec_cnt} 次)")
    print("----------------------------------------------")
    print(f"总系统运行时间           : {total_time:8.4f} 秒")
    print("")
    print("======================================================")
    if timeout_triggered and success_cnt > 0:
        print(">> 警告：枚举计算超时限制，截断后保留最优解！")
    else:
        print(">> 搜索过程完成。")
    print(f"有效组合数 (距离>=2): {total_combos} 组")
    if total_combos > 0:
        print(f"被剪枝数: {pruned_combos} 组")
    print(f"被测试数: {tested_combos} 组 | 通过数: {success_cnt} 组")

    if global_best_metric >= 999999.0:
        return False

    print("")
    print("[全局最优解]")
    for i in range(init_b_cnt):
        print(f"  加强炸弹{i + 1}: ({g_best_bombs[i][1]}, {11 - g_best_bombs[i][0]})")

    print("")

    p1_res.path_length = best_p1_steps
    for i in range(best_p1_steps):
        p1_res.path[i] = int(g_best_p1_acts[i])
    p1_res.end_px = g_final_p1_state.c
    p1_res.end_py = 11 - g_final_p1_state.r

    # 构建updated_map
    for r in range(MAX_R):
        for c in range(MAX_C):
            ch = original_map[r][c]
            y = 11 - r
            x = c
            p1_res.updated_map[y][x] = ' ' if (ch == '#' or ch == '.') else ch
            for b in range(init_b_cnt):
                if abs(r - g_best_bombs[b][0]) <= 1 and abs(c - g_best_bombs[b][1]) <= 1:
                    if r > 0 and r < MAX_R - 1 and c > 0 and c < MAX_C - 1:
                        p1_res.updated_map[y][x] = ' '
    p1_res.updated_map[11 - g_final_p1_state.r][g_final_p1_state.c] = '@'
    for i in range(3):
        if g_final_p1_state.bx[i] != 255:
            bx_r = GET_R(g_final_p1_state.bx[i])
            bx_c = GET_C(g_final_p1_state.bx[i])
            p1_res.updated_map[11 - bx_r][bx_c] = '$'

    return True


# ==================== run_phase2: 阶段二入口（C: 第1756-1946行） ====================

def run_phase2(box_ids, target_ids, p2_res: PhaseResult) -> bool:
    """C: run_phase2 (约1756-1946行)
    阶段二主入口：根据box→target匹配，枚举6种推箱排列，A*求解

    Args:
        box_ids: 箱子ID数组 (如 [2, 3, 1])
        target_ids: 目标ID数组 (如 [1, 2, 3])
        p2_res: PhaseResult输出对象

    Returns:
        bool: True=找到解, False=无解
    """
    global global_exp_cov, bfs_dist_dot
    global p2_vis_id, p2_heap_size, p2_f_array
    global g_final_p1_state, g_best_bombs

    # C: 第1757-1765行 — 构建分配映射
    cur_assign = [0, 1, 2]
    for i in range(init_bx_cnt):
        for j in range(init_dot_cnt):
            if box_ids[i] == target_ids[j]:
                cur_assign[i] = j
                break

    # C: 第1766-1777行 — 重建爆炸覆盖掩码
    for r in range(MAX_R):
        for c in range(MAX_C):
            global_exp_cov[r][c] = 0
            if r > 0 and r < MAX_R - 1 and c > 0 and c < MAX_C - 1:
                for i in range(init_b_cnt):
                    if (abs(r - g_best_bombs[i][0]) <= 1
                            and abs(c - g_best_bombs[i][1]) <= 1):
                        global_exp_cov[r][c] |= (1 << i)

    # C: 第1779-1798行 — BFS预计算目标到所有格的距离
    for i in range(3):
        for r in range(MAX_R):
            for c in range(MAX_C):
                bfs_dist_dot[i][r][c] = 9999
        if i < init_dot_cnt:
            qr = [0] * 512
            qc = [0] * 512
            head, tail = 0, 0
            qr[tail] = GET_R(dot_pos[i])
            qc[tail] = GET_C(dot_pos[i])
            tail += 1
            bfs_dist_dot[i][qr[0]][qc[0]] = 0
            while head < tail:
                r = qr[head]
                c = qc[head]
                head += 1
                for d in range(4):
                    nr = r + dr[d]
                    nc = c + dc[d]
                    if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                        continue
                    if global_base_wall[nr][nc] and not (7 & global_exp_cov[nr][nc]):
                        continue
                    if bfs_dist_dot[i][nr][nc] == 9999:
                        bfs_dist_dot[i][nr][nc] = bfs_dist_dot[i][r][c] + 1
                        qr[tail] = nr
                        qc[tail] = nc
                        tail += 1

    # C: 第1800-1902行 — 枚举6种推箱排列
    best_score = 999999.0
    best_acts = [0] * MAX_STEPS
    best_len = 0
    final_p2_state = SimState(r=g_final_p1_state.r, c=g_final_p1_state.c,
                               mask=g_final_p1_state.mask,
                               b=list(g_final_p1_state.b),
                               bx=list(g_final_p1_state.bx))

    for ord_i in range(6):
        s2 = SimState(r=g_final_p1_state.r, c=g_final_p1_state.c,
                      mask=g_final_p1_state.mask,
                      b=list(g_final_p1_state.b),
                      bx=list(g_final_p1_state.bx))
        cur_acts = [0] * MAX_STEPS
        cur_len = 0
        possible = True

        for step in range(init_bx_cnt):
            b_id = perms[ord_i][step]
            if s2.bx[b_id] == 255:
                continue

            t_id = cur_assign[b_id]
            tr = GET_R(dot_pos[t_id])
            tc = GET_C(dot_pos[t_id])

            p2_vis_id += 1
            if p2_vis_id == 0xFFFF:
                _rebuild_p2_vis()
                p2_vis_id = 1

            p2_heap_size = 0
            t_idx = 0

            rq_p2[t_idx].rr = s2.r
            rq_p2[t_idx].rc = s2.c
            rq_p2[t_idx].br = GET_R(s2.bx[b_id])
            rq_p2[t_idx].bc = GET_C(s2.bx[b_id])
            rq_p2[t_idx].parent = -1
            rq_p2[t_idx].action = -1
            rq_p2[t_idx].dist = 0
            p2_f_array[t_idx] = 0

            _push_p2(t_idx)
            t_idx += 1

            goal = -1

            while p2_heap_size > 0:
                curr_p2 = _pop_p2()
                cqn = rq_p2[curr_p2]

                if _p2_vis_get(cqn.rr, cqn.rc, cqn.br, cqn.bc) == p2_vis_id:
                    continue
                _p2_vis_set(cqn.rr, cqn.rc, cqn.br, cqn.bc, p2_vis_id)

                if cqn.br == tr and cqn.bc == tc:
                    goal = curr_p2
                    break

                for d in range(4):
                    nrr = cqn.rr + dr[d]
                    nrc = cqn.rc + dc[d]
                    nbr = cqn.br
                    nbc = cqn.bc

                    if nrr < 0 or nrr >= MAX_R or nrc < 0 or nrc >= MAX_C:
                        continue
                    if global_base_wall[nrr][nrc] and not (7 & global_exp_cov[nrr][nrc]):
                        continue

                    blocked = False
                    for k in range(3):
                        if k != b_id and s2.bx[k] != 255 and s2.bx[k] == MAKE_POS(nrr, nrc):
                            blocked = True
                    if blocked:
                        continue

                    if nrr == cqn.br and nrc == cqn.bc:
                        nbr += dr[d]
                        nbc += dc[d]
                        if nbr < 0 or nbr >= MAX_R or nbc < 0 or nbc >= MAX_C:
                            continue
                        if global_base_wall[nbr][nbc] and not (7 & global_exp_cov[nbr][nbc]):
                            continue
                        for k in range(3):
                            if k != b_id and s2.bx[k] != 255 and s2.bx[k] == MAKE_POS(nbr, nbc):
                                blocked = True
                        if blocked:
                            continue

                    if _p2_vis_get(nrr, nrc, nbr, nbc) == p2_vis_id:
                        continue

                    new_dist = cqn.dist + 1
                    if cur_len + new_dist > best_score:
                        continue

                    if t_idx < MAX_P2_Q:
                        rq_p2[t_idx].rr = nrr
                        rq_p2[t_idx].rc = nrc
                        rq_p2[t_idx].br = nbr
                        rq_p2[t_idx].bc = nbc
                        rq_p2[t_idx].parent = curr_p2
                        rq_p2[t_idx].action = d
                        rq_p2[t_idx].dist = new_dist

                        h_box = bfs_dist_dot[t_id][nbr][nbc]
                        if h_box == 9999:
                            continue

                        h_robot = abs(nrr - nbr) + abs(nrc - nbc)
                        p2_f_array[t_idx] = new_dist + h_box * 1 + h_robot
                        _push_p2(t_idx)
                        t_idx += 1

            if goal == -1:
                possible = False
                break

            temp_p = [0] * 1000
            t_len = 0
            c_ptr = goal
            while rq_p2[c_ptr].parent != -1:
                temp_p[t_len] = rq_p2[c_ptr].action
                t_len += 1
                c_ptr = rq_p2[c_ptr].parent
            for x in range(t_len - 1, -1, -1):
                cur_acts[cur_len] = temp_p[x]
                cur_len += 1

            s2.r = rq_p2[goal].rr
            s2.c = rq_p2[goal].rc
            s2.bx[b_id] = 255

        if possible:
            cur_turns = count_path_turns(cur_acts, cur_len)
            cur_score = float(cur_len) + 4.0 * cur_turns
            if cur_score < best_score:
                best_score = cur_score
                best_len = cur_len
                for i_copy in range(cur_len):
                    best_acts[i_copy] = cur_acts[i_copy]
                final_p2_state = SimState(r=s2.r, c=s2.c, mask=s2.mask,
                                          b=list(s2.b), bx=list(s2.bx))

    if best_score >= 999999.0:
        return False

    # C: 第1904-1945行 — 输出结果 + 前往最近基地
    p2_res.path_length = best_len
    for i in range(best_len):
        p2_res.path[i] = int(best_acts[i])

    rr, rc = final_p2_state.r, final_p2_state.c
    base1_r, base1_c = 11 - 5, 1
    base2_r, base2_c = 11 - 5, 14

    # BFS计算到两基地的距离
    dist_p2 = [[-1] * MAX_C for _ in range(MAX_R)]
    parent_p2 = [[0] * MAX_C for _ in range(MAX_R)]
    action_p2 = [[0] * MAX_C for _ in range(MAX_R)]

    for r in range(MAX_R):
        for c in range(MAX_C):
            dist_p2[r][c] = -1

    qr = [0] * 256
    qc = [0] * 256
    h, t = 0, 0
    qr[t] = rr
    qc[t] = rc
    t += 1
    dist_p2[rr][rc] = 0

    while h < t:
        r = qr[h]
        c = qc[h]
        h += 1
        for d in range(4):
            nr = r + dr[d]
            nc = c + dc[d]
            if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                continue
            if global_base_wall[nr][nc] and not (7 & global_exp_cov[nr][nc]):
                continue
            if dist_p2[nr][nc] == -1:
                dist_p2[nr][nc] = dist_p2[r][c] + 1
                parent_p2[nr][nc] = MAKE_POS(r, c)
                action_p2[nr][nc] = d
                qr[t] = nr
                qc[t] = nc
                t += 1

    d1 = dist_p2[base1_r][base1_c]
    d2 = dist_p2[base2_r][base2_c]
    target_r, target_c = -1, -1
    if d1 != -1 and d2 != -1:
        if d1 <= d2:
            target_r, target_c = base1_r, base1_c
        else:
            target_r, target_c = base2_r, base2_c
    elif d1 != -1:
        target_r, target_c = base1_r, base1_c
    elif d2 != -1:
        target_r, target_c = base2_r, base2_c

    if target_r != -1:
        back_len = dist_p2[target_r][target_c]
        cr = target_r
        cc = target_c
        temp_path = [0] * 256
        for i in range(back_len - 1, -1, -1):
            temp_path[i] = action_p2[cr][cc]
            p = parent_p2[cr][cc]
            cr = GET_R(p)
            cc = GET_C(p)
        for i in range(back_len):
            if p2_res.path_length < MAX_STEPS:
                p2_res.path[p2_res.path_length] = int(temp_path[i])
                p2_res.path_length += 1
        p2_res.end_px = target_c
        p2_res.end_py = 11 - target_r
    else:
        p2_res.end_px = rc
        p2_res.end_py = 11 - rr

    return True
