# -*- coding: utf-8 -*-
"""
c_solver raw action codes → 结构化 action 转换器

将 track_port.py 内部的 p1_acts/p2_acts (整数序列) 转换为
PathFollower 可消费的结构化 action 字典列表。

动作码含义（来自 track_port.py）:
  0-3: 方向移动 (0=UP, 1=DOWN, 2=LEFT, 3=RIGHT)
         可能是 walk、push_box 或 push_bomb，需通过状态变化判断
  4:   爆炸 (mask 位变化)
  5-8: 观察 (5=UP观察, 6=LEFT观察, 7=RIGHT观察, 8=DOWN观察)

方向常量:
  dr = [-1, 1, 0, 0]  # 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
  dc = [0, 0, -1, 1]
  p_dir = [0, 2, 3, 1]  # 观察码 5+i 对应的实际方向

结构化 action 格式:
  free_move:  {type:'free_move', target:(c,r), theta:deg|None, waypoints:[...]}
  push_box:   {type:'push_box',  target:(c,r), theta:deg, push_meta:{box_id, box_start, box_target, push_dir}}
  push_bomb:  {type:'push_bomb', target:(c,r), theta:deg, push_meta:{bomb_id, bomb_start, bomb_target, wall_target, push_dir}}
  wait:       {type:'wait', duration:0.5, reason:'bomb_explosion'}
  observe:    {type:'observe', target:(c,r), theta:deg, observe_meta:{direction, object_type, object_index}}
  end_of_phase1: {type:'end_of_phase1'}
"""

from typing import List, Dict, Tuple, Optional
from .constants import MAX_R, MAX_C, MAKE_POS, GET_R, GET_C
from .types import SimState


# ===== 方向映射 =====
# track_port.py 的 dr/dc 定义
DR = [-1, 1, 0, 0]   # 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
DC = [0, 0, -1, 1]

# p_dir: 观察码 5+i → 实际方向
P_DIR = [0, 2, 3, 1]  # p_dir[0]=0(UP), p_dir[1]=2(LEFT), p_dir[2]=3(RIGHT), p_dir[3]=1(DOWN)

# 方向 → 字符串名称
DIR_NAMES = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}

# 方向 → theta 角度（度，0=右，90=下，180=左，270=上）
DIR_THETA = {0: -90.0, 1: 90.0, 2: 180.0, 3: 0.0}  # UP=-90, DOWN=90, LEFT=180, RIGHT=0

# 方向 → 相反方向
OPPOSITE = {0: 1, 1: 0, 2: 3, 3: 2}

# 方向 → 推方向（字符串）
PUSH_DIR_NAMES = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}


def _dir_name(d: int) -> str:
    return DIR_NAMES.get(d, 'UP')


def _theta(d: int) -> float:
    """方向 → theta 角度（度）"""
    return DIR_THETA.get(d, 0.0)


def _push_theta_from_dir(d: int) -> float:
    """推箱子时车需要站在物体的反方向，theta 指向推方向"""
    return _theta(d)


# ===== BFS 路径还原工具 =====

def _bfs_walk_path(start_r: int, start_c: int, end_r: int, end_c: int,
                   fast_wall, sim: SimState, vis_id: int, vis_array, dist_array,
                   parent_array, action_array) -> List[int]:
    """
    BFS 从 start 到 end，返回最短路径的步方向列表 [d0, d1, ...]
    使用 track_port.py 中已有的 vis/dist/parent/action 数组。
    返回路径方向序列（从 start 走到 end）。
    """
    qr = [0] * 512
    qc = [0] * 512
    head, tail = 0, 0
    qr[tail] = start_r
    qc[tail] = start_c
    tail += 1
    vis_array[start_r][start_c] = vis_id
    dist_array[start_r][start_c] = 0

    found = False
    while head < tail:
        r = qr[head]
        c = qc[head]
        head += 1
        if r == end_r and c == end_c:
            found = True
            break
        for d in range(4):
            nr = r + DR[d]
            nc = c + DC[d]
            if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
                continue
            if fast_wall[sim.mask][nr][nc]:
                continue
            # 检查固体障碍（箱子 + 炸弹）
            blocked = False
            for k in range(3):
                if ((sim.b[k] != 255 and sim.b[k] == MAKE_POS(nr, nc))
                        or (sim.bx[k] != 255 and sim.bx[k] == MAKE_POS(nr, nc))):
                    blocked = True
                    break
            if not blocked and vis_array[nr][nc] != vis_id:
                vis_array[nr][nc] = vis_id
                dist_array[nr][nc] = dist_array[r][c] + 1
                parent_array[nr][nc] = MAKE_POS(r, c)
                action_array[nr][nc] = d
                qr[tail] = nr
                qc[tail] = nc
                tail += 1

    if not found:
        return []  # 无法到达

    # 回溯路径
    steps = [0] * 512
    step_cnt = 0
    cr, cc = end_r, end_c
    while cr != start_r or cc != start_c:
        steps[step_cnt] = action_array[cr][cc]
        step_cnt += 1
        p = parent_array[cr][cc]
        cr = GET_R(p)
        cc = GET_C(p)

    # 翻转
    path = []
    for i in range(step_cnt - 1, -1, -1):
        path.append(steps[i])
    return path


def _find_object_in_direction(sim: SimState, look_dir: int, dot_pos: list):
    """
    在给定方向 (0-3) 上查找相邻格上的物体。

    Returns:
        (object_type, object_index) 或 (None, None)
        object_type: 'box', 'goal', 或 'both'
    """
    nr = sim.r + DR[look_dir]
    nc = sim.c + DC[look_dir]
    if nr < 0 or nr >= MAX_R or nc < 0 or nc >= MAX_C:
        return None, None

    npos = MAKE_POS(nr, nc)
    found_box = -1
    found_goal = -1

    for k in range(3):
        if sim.bx[k] != 255 and sim.bx[k] == npos:
            found_box = k
        # dot_pos[k] 为 255 表示该目标不存在
        if k < len(dot_pos) and dot_pos[k] != 255 and dot_pos[k] == npos:
            found_goal = k

    if found_box >= 0 and found_goal >= 0:
        return 'both', found_box  # 返回箱子索引，调用方知道也要标记目标
    elif found_box >= 0:
        return 'box', found_box
    elif found_goal >= 0:
        return 'goal', found_goal
    return None, None


# ===== theta 传导 =====

def _propagate_theta(actions: List[Dict]):
    """
    向后传导 theta：只把 observe 的朝向传递给前面的 free_move。

    推物体不需要朝向（全向推送），只有观察时需要车面对目标。
    从后向前扫描，遇到 observe 就记住 theta，向前面的 free_move 传导。
    """
    pending_theta = None
    for i in range(len(actions) - 1, -1, -1):
        a = actions[i]
        t = a['type']
        if t == 'observe':
            th = a.get('theta')
            if th is not None:
                pending_theta = th
        elif t == 'free_move':
            if a.get('theta') is None and pending_theta is not None:
                a['theta'] = pending_theta
            # free_move 阻断传导
            pending_theta = None
        elif t in ('push_bomb', 'push_box', 'wait'):
            # 推物/等待不需要转向，阻断传导
            pending_theta = None


# ===== 主转换函数 =====

def convert_p1_actions(
    p1_acts: List[int],
    p1_states: List[SimState],
    p1_len: int,
    init_b_cnt: int,
    init_bx_cnt: int,
    dot_pos_list: list,
    fast_wall: list,
    p1_dist_vis: list,
    global_dist: list,
    global_parent: list,
    global_action: list,
) -> Tuple[List[Dict], List[int], List[int]]:
    """
    将 P1 raw action 序列转换为结构化 action 列表。

    Args:
        p1_acts: raw 动作码序列
        p1_states: 对应状态序列 (长度 p1_len+1，p1_states[i] 是执行 p1_acts[i] 前的状态)
        p1_len: 动作数量
        init_b_cnt: 炸弹数量
        init_bx_cnt: 箱子数量
        dot_pos_list: 目标点编码列表（MAKE_POS 格式，dot_pos[k]=255 表示不存在）
        fast_wall: 墙体数组 fast_wall[mask][r][c]
        p1_dist_vis, global_dist, global_parent, global_action: BFS 工作数组

    Returns:
        (actions, observed_box_indices, observed_target_indices)
    """
    actions: List[Dict] = []
    observed_boxes = []
    observed_targets = []

    # BFS 用 vis_id
    vis_id = [p1_dist_vis[0][0] + 1]  # 从当前值开始递增

    def next_vis_id():
        vis_id[0] += 1
        if vis_id[0] >= 0xFFFF:
            for rr in range(MAX_R):
                for cc in range(MAX_C):
                    p1_dist_vis[rr][cc] = 0
            vis_id[0] = 1
        return vis_id[0]

    # 累积 walk steps，统一输出为 free_move
    walk_accum = []  # [(r, c, dir_code)]

    def flush_walk():
        """将累积的 walk steps 输出为一个 free_move action"""
        nonlocal walk_accum
        if not walk_accum:
            return
        # waypoints: 从起点开始，每一步的终点作为 waypoint
        waypoints = []
        # 起点是第一步之前的位置
        for r, c, _ in walk_accum:
            waypoints.append((c, r))  # (col, row)

        # target 是最后一步的终点
        final_r, final_c, _ = walk_accum[-1]
        target = (final_c, final_r)  # (col, row)

        # theta: 检查下一步是否是 observe 或 push，如果是则需要设置 theta
        # 这里先设为 None，在后续处理中如果发现需要朝向会由调用方设置
        actions.append({
            'type': 'free_move',
            'target': target,
            'theta': None,
            'waypoints': waypoints,
            'narrow_passage': False,
        })
        walk_accum = []

    def get_current_pos():
        """获取当前累积状态下的位置（最后一步终点或最后一个 action 的 target）"""
        if walk_accum:
            return walk_accum[-1][0], walk_accum[-1][1]
        if actions:
            last = actions[-1]
            if last['type'] == 'free_move':
                tc, tr = last['target']
                return tr, tc
            elif last['type'] == 'observe':
                tc, tr = last['target']
                return tr, tc
        # fallback: 从 p1_states[0] 获取
        return p1_states[0].r, p1_states[0].c

    last_exploded_bomb = -1  # raw 索引，传给 wait action

    i = 0
    while i < p1_len:
        act = p1_acts[i]
        cur_st = p1_states[i]
        next_st = p1_states[i + 1] if i + 1 <= p1_len else cur_st

        if 0 <= act <= 3:
            # 移动动作: 判断是 walk 还是 push
            nr = cur_st.r + DR[act]
            nc = cur_st.c + DC[act]
            npos = MAKE_POS(nr, nc)

            # 检查这个移动是否推动了物体
            pushed_box = -1
            pushed_bomb = -1

            for k in range(3):
                if cur_st.bx[k] != 255 and cur_st.bx[k] == npos:
                    # 箱子在这个位置，检查是否移动了
                    expected_new = MAKE_POS(nr + DR[act], nc + DC[act])
                    if next_st.bx[k] == expected_new:
                        pushed_box = k
                if cur_st.b[k] != 255 and cur_st.b[k] == npos:
                    # 炸弹在这个位置
                    expected_new = MAKE_POS(nr + DR[act], nc + DC[act])
                    if next_st.b[k] == expected_new:
                        pushed_bomb = k
                    elif (next_st.b[k] == 255
                          and next_st.mask != cur_st.mask):
                        # 炸弹被推入爆炸（bomb消失，mask变化）
                        pushed_bomb = k
                        last_exploded_bomb = k  # 记录 raw 索引

            if pushed_bomb >= 0:
                # push_bomb: 先清空累积的 walk
                flush_walk()

                push_dir = PUSH_DIR_NAMES[act]
                bomb_start = (GET_C(cur_st.b[pushed_bomb]), GET_R(cur_st.b[pushed_bomb]))
                if next_st.b[pushed_bomb] != 255:
                    bomb_target = (GET_C(next_st.b[pushed_bomb]), GET_R(next_st.b[pushed_bomb]))
                else:
                    # 炸弹消失（推入爆炸），真实目标是它被推到的墙格
                    bomb_target = (nc + DC[act], nr + DR[act])

                wall_target = bomb_target  # 炸弹推入的墙位置

                actions.append({
                    'type': 'push_bomb',
                    'target': (nc, nr),  # 车推完后停在炸弹原位置
                    'theta': None,  # 推物不需要朝向
                    'push_meta': {
                        'bomb_id': init_bx_cnt + pushed_bomb,  # bomb_id = box数量 + 炸弹索引
                        'bomb_start': bomb_start,
                        'bomb_target': bomb_target,
                        'wall_target': wall_target,
                        'push_dir': push_dir,
                    },
                    'narrow_passage': False,
                })

            elif pushed_box >= 0:
                # push_box
                flush_walk()

                push_dir = PUSH_DIR_NAMES[act]
                box_start = (GET_C(cur_st.bx[pushed_box]), GET_R(cur_st.bx[pushed_box]))
                box_target = (GET_C(next_st.bx[pushed_box]), GET_R(next_st.bx[pushed_box]))

                actions.append({
                    'type': 'push_box',
                    'target': (nc, nr),  # 车推完后停在箱子原位置
                    'theta': None,  # 推物不需要朝向
                    'push_meta': {
                        'box_id': pushed_box,
                        'box_start': box_start,
                        'box_target': box_target,
                        'push_dir': push_dir,
                    },
                    'narrow_passage': False,
                })

            else:
                # 普通 walk step
                walk_accum.append((nr, nc, act))

        elif act == 4:
            # 爆炸: 清空 walk，输出 wait
            flush_walk()

            # 查找哪个炸弹爆炸了（mask 变化）
            exploded_bomb = -1
            for k in range(3):
                if (cur_st.mask & (1 << k)) == 0 and (next_st.mask & (1 << k)):
                    exploded_bomb = k
                    break

            # 也可能在 act=4 时 mask 已经在之前的 push 中变了
            # 此时找哪个炸弹刚消失
            if exploded_bomb < 0:
                for k in range(3):
                    if cur_st.b[k] != 255 and next_st.b[k] == 255:
                        exploded_bomb = k
                        break

            actions.append({
                'type': 'wait',
                'target': None,
                'theta': None,
                'duration': 0.5,
                'reason': 'bomb_explosion',
                'bomb_id': last_exploded_bomb,  # raw 索引，_smooth_actions用
            })
            last_exploded_bomb = -1  # 重置

        elif 5 <= act <= 8:
            # 观察动作
            flush_walk()

            look_dir = act - 5  # 实际方向 (0-3): c_solver中观察码=5+d
            cur_r = cur_st.r
            cur_c = cur_st.c

            obj_type, obj_idx = _find_object_in_direction(cur_st, look_dir, dot_pos_list)

            if obj_type is not None:
                actions.append({
                    'type': 'observe',
                    'target': (cur_c, cur_r),
                    'theta': _theta(look_dir),
                    'observe_meta': {
                        'direction': _dir_name(look_dir),
                        'object_type': obj_type,
                        'object_index': obj_idx,
                    },
                })
                if obj_type in ('box', 'both') and obj_idx not in observed_boxes:
                    observed_boxes.append(obj_idx)
                if obj_type in ('goal', 'both'):
                    # 需要找到对应目标索引
                    nr_obs = cur_r + DR[look_dir]
                    nc_obs = cur_c + DC[look_dir]
                    npos_obs = MAKE_POS(nr_obs, nc_obs)
                    for k in range(3):
                        if k < len(dot_pos_list) and dot_pos_list[k] == npos_obs:
                            if k not in observed_targets:
                                observed_targets.append(k)

        i += 1

    # 清空剩余 walk
    flush_walk()

    # === theta 传导：把 push/observe 的方向往前面的 free_move 传递 ===
    _propagate_theta(actions)

    # 追加 end_of_phase1 标记
    actions.append({
        'type': 'end_of_phase1',
    })

    return actions, observed_boxes, observed_targets


def convert_p2_actions(
    p2_acts: List[int],
    p2_len: int,
    initial_state: SimState,
    init_bx_cnt: int,
    assigned_goal_positions: Optional[List[int]] = None,
) -> List[Dict]:
    """
    将 P2 raw action 序列转换为结构化 action 列表。

    通过前向模拟追踪状态变化，无需预先存储状态序列。
    P2 中没有观察动作（5-8），只有移动和推箱。

    Args:
        p2_acts: P2 raw 动作码序列
        p2_len: 动作数量
        initial_state: P2 起始状态（通常为P1结束状态）
        assigned_goal_positions: 每个箱子的指定目标编码位置；到达后箱子退场
    Returns:
        结构化 action 列表
    """
    actions: List[Dict] = []

    # 前向模拟状态
    cur = SimState(r=initial_state.r, c=initial_state.c, mask=initial_state.mask,
                   b=list(initial_state.b), bx=list(initial_state.bx))

    walk_accum = []  # [(r, c, dir_code)]

    def flush_walk():
        nonlocal walk_accum
        if not walk_accum:
            return
        waypoints = []
        for r, c, _ in walk_accum:
            waypoints.append((c, r))
        final_r, final_c, _ = walk_accum[-1]
        target = (final_c, final_r)
        actions.append({
            'type': 'free_move',
            'target': target,
            'theta': None,
            'waypoints': waypoints,
            'narrow_passage': False,
        })
        walk_accum = []

    for i in range(p2_len):
        act = p2_acts[i]
        if not (0 <= act <= 3):
            continue

        nr = cur.r + DR[act]
        nc = cur.c + DC[act]
        npos = MAKE_POS(nr, nc)

        # 检查此步是否推动了箱子
        pushed_box = -1
        for k in range(3):
            if cur.bx[k] != 255 and cur.bx[k] == npos:
                # 箱子在此方向，尝试推到下一格
                nnpos = MAKE_POS(nr + DR[act], nc + DC[act])
                nnr = GET_R(nnpos)
                nnc = GET_C(nnpos)
                # 检查目标格是否合法（不穿墙、不撞其他物）
                blocked = False
                for k2 in range(3):
                    if cur.bx[k2] != 255 and cur.bx[k2] == nnpos and k2 != k:
                        blocked = True
                    if cur.b[k2] != 255 and cur.b[k2] == nnpos:
                        blocked = True
                if not blocked:
                    pushed_box = k
                break

        if pushed_box >= 0:
            flush_walk()
            push_dir = PUSH_DIR_NAMES[act]
            box_start = (GET_C(cur.bx[pushed_box]), GET_R(cur.bx[pushed_box]))
            # 更新箱子位置
            nnpos = MAKE_POS(nr + DR[act], nc + DC[act])
            cur.bx[pushed_box] = nnpos
            box_target = (GET_C(cur.bx[pushed_box]), GET_R(cur.bx[pushed_box]))
            # 车位置更新到推箱前的位置
            # (车的推动位置 = 箱子原位置 - push_dir)
            actions.append({
                'type': 'push_box',
                'target': (nc, nr),  # 车推完后停在箱子原位置
                'theta': None,  # 推物不需要朝向
                'push_meta': {
                    'box_id': pushed_box,
                    'box_start': box_start,
                    'box_target': box_target,
                    'push_dir': push_dir,
                },
                'narrow_passage': False,
            })
            if (assigned_goal_positions is not None
                    and pushed_box < len(assigned_goal_positions)
                    and cur.bx[pushed_box] == assigned_goal_positions[pushed_box]):
                # run_phase2 removes a completed box before planning the next one.
                # Keep the converter's replay state identical, otherwise later
                # walking through the goal is misclassified as another push.
                cur.bx[pushed_box] = 255
            # 车走到 nr,nc（箱子原来位置）
            cur.r = nr
            cur.c = nc
        else:
            # 普通 walk step
            cur.r = nr
            cur.c = nc
            walk_accum.append((nr, nc, act))

    flush_walk()

    # theta 传导：让推箱方向传给前面的 free_move
    _propagate_theta(actions)

    return actions
