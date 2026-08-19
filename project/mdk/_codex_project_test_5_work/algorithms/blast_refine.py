"""
炸点精确优化模块 (步骤 5)

给定步骤 4 的分配结果，对每个炸弹对精确计算最优墙体坐标和推弹/推箱路径。
"""

from typing import List, Dict, Tuple, Optional, Set
from collections import deque

from core.map_analysis import (
    get_walls_in_rect, get_walls_in_3x3, is_boundary_wall,
    compute_narrow_cells,
)
from core.distance import manhattan_distance
from core.bfs import bfs_shortest_path

_DIR_VEC = {'UP': (0, -1), 'DOWN': (0, 1), 'LEFT': (-1, 0), 'RIGHT': (1, 0)}


# =============================================================================
# 目标周边墙
# =============================================================================

def _get_walls_near_goal(goal, walls, w, h, radius=2):
    gx, gy = goal
    result = set()
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            n = (gx + dx, gy + dy)
            if n in walls and not is_boundary_wall(n, w, h):
                result.add(n)
    return result


# =============================================================================
# 推弹 BFS (含车位置验证)
# =============================================================================

def _bfs_push_bomb_with_car(bomb, wall, walls, w, h, obstacles, max_steps=200, car_start=None):
    """推弹搜索: 两级优先(步数优先,等步数选车总移动少).

    返回 (steps, first_dir, turns, inter_car).
    inter_car: 推弹步间车移动曼哈顿累积(首次推弹的初始车距不计入).
    """
    if bomb == wall:
        return None
    import heapq
    # state: ((push_steps, car_total), steps, turns, bomb_pos, car_pos, prev_dir, first_dir, inter_car)
    # car_total = 初始车距 + 步间移动累积 (用于排序)
    # inter_car = 仅步间移动累积 (不含初始车距, 用于返回)
    heap = [((0, 0), 0, 0, bomb, None, None, None, 0)]
    best = {}

    while heap:
        (p_steps, car_total), steps, turns, pos, car_pos, prev_dir, first_dir, inter_car = heapq.heappop(heap)
        key = (pos, car_pos)
        if key in best:
            prev = best[key]
            if prev[0] < p_steps or (prev[0] == p_steps and prev[1] <= car_total):
                continue
        best[key] = (p_steps, car_total)

        if steps > max_steps:
            continue
        for dx, dy, dname in [(0,-1,'UP'),(0,1,'DOWN'),(-1,0,'LEFT'),(1,0,'RIGHT')]:
            nb = (pos[0]+dx, pos[1]+dy)
            cp = (pos[0]-dx, pos[1]-dy)
            if not (0 <= cp[0] < w and 0 <= cp[1] < h):
                continue
            if cp in walls or cp in obstacles:
                continue
            nd = first_dir if first_dir else dname
            nt = turns + (1 if prev_dir and prev_dir != dname else 0)
            # 车移动代价: 从前一车位到当前车位
            if car_pos is not None:
                car_move = abs(car_pos[0]-cp[0]) + abs(car_pos[1]-cp[1])
            elif car_start is not None:
                # 首次推弹: 从车起始位置到首推车位
                car_move = abs(car_start[0]-cp[0]) + abs(car_start[1]-cp[1])
            else:
                car_move = 0
            # car_total 包含所有车移动(初始+步间), inter_car 仅步间
            new_car_total = car_total + car_move
            new_inter_car = inter_car + (car_move if car_pos is not None else 0)
            if nb == wall:
                return steps + 1, nd, nt, new_inter_car
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in walls or nb in obstacles:
                continue
            heapq.heappush(heap, ((steps+1, new_car_total),
                                  steps+1, nt, nb, cp, dname, nd, new_inter_car))
    return None


# =============================================================================
# 推箱 BFS
# =============================================================================

def _bfs_push_box_path(box, goal, walls, w, h, obstacles, max_steps=200, narrow_set=None):
    """推箱 BFS: 返回 (steps, turns, narrow_count). turns 为全程累加."""
    if box == goal:
        return 0, 0, 0
    queue = deque([(box, 0, None, 0, 0)])
    visited = {box}
    while queue:
        pos, steps, prev_dir, ncount, turns = queue.popleft()
        if steps > max_steps:
            continue
        for dx, dy, dname in [(0,-1,'UP'),(0,1,'DOWN'),(-1,0,'LEFT'),(1,0,'RIGHT')]:
            nb = (pos[0]+dx, pos[1]+dy)
            cp = (pos[0]-dx, pos[1]-dy)
            if not (0 <= cp[0] < w and 0 <= cp[1] < h):
                continue
            if cp in walls or cp in obstacles:
                continue
            nn = ncount + 1 if (narrow_set and nb in narrow_set) else ncount
            nt = turns + (1 if prev_dir and prev_dir != dname else 0)
            if nb == goal:
                return steps + 1, nt, nn
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in walls or nb in obstacles:
                continue
            if nb in visited:
                continue
            visited.add(nb)
            queue.append((nb, steps+1, dname, nn, nt))
    return None


# =============================================================================
# 窄道检测
# =============================================================================

# =============================================================================
# 跨对收益检测 (Cross-Pair Benefit)
# =============================================================================

def _bfs_push_box_with_path(box, goal, walls, w, h, obstacles, max_steps=200):
    """推箱BFS+路径追踪. 返回 (steps, turns, path) 或 None.
    path: [(pos, (dx,dy)), ...] 从box到goal, 每项为(到达位置, 推的方向)
    """
    if box == goal:
        return 0, 0, []
    queue = deque([(box, 0, None)])
    visited = {box}
    parent = {}  # pos -> (prev_pos, (dx, dy))
    while queue:
        pos, steps, prev_dir = queue.popleft()
        if steps > max_steps:
            continue
        for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
            nb = (pos[0]+dx, pos[1]+dy)
            cp = (pos[0]-dx, pos[1]-dy)
            if not (0 <= cp[0] < w and 0 <= cp[1] < h):
                continue
            if cp in walls or cp in obstacles:
                continue
            if nb == goal:
                parent[nb] = (pos, (dx, dy))
                # 重建路径
                path = []
                cur = nb
                while cur != box:
                    ppos, pdir = parent[cur]
                    path.append((cur, pdir))
                    cur = ppos
                path.reverse()
                # 计算转弯
                turns = 0
                for i in range(len(path) - 1):
                    if path[i][1] != path[i+1][1]:
                        turns += 1
                return steps + 1, turns, path
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in walls or nb in obstacles:
                continue
            if nb in visited:
                continue
            visited.add(nb)
            parent[nb] = (pos, (dx, dy))
            queue.append((nb, steps+1, (dx, dy)))
    return None


def _group_connected_walls(wall_set, walls):
    """将wall_set中的墙按4邻接连通分组, 返回 [{walls}, ...]"""
    wall_set = wall_set & walls
    if not wall_set:
        return []
    groups = []
    remaining = set(wall_set)
    while remaining:
        seed = remaining.pop()
        group = {seed}
        q = deque([seed])
        while q:
            w = q.popleft()
            for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
                nb = (w[0]+dx, w[1]+dy)
                if nb in remaining:
                    remaining.discard(nb)
                    group.add(nb)
                    q.append(nb)
        groups.append(group)
    return groups


def _find_turn_wall_groups(box, goal, walls, w, h, obstacles):
    """方案1: 找出造成推箱转弯的墙体连通组"""
    result = _bfs_push_box_with_path(box, goal, walls, w, h, obstacles)
    if result is None:
        return []
    _, turns, path = result
    if turns == 0 or len(path) < 2:
        return []

    turn_walls = set()
    for i in range(len(path) - 1):
        pos_i, dir_i = path[i]
        _, dir_next = path[i+1]
        if dir_i != dir_next:
            # 转弯: 继续沿dir_i方向会撞到墙
            fwd = (pos_i[0] + dir_i[0], pos_i[1] + dir_i[1])
            if fwd in walls:
                turn_walls.add(fwd)

    return _group_connected_walls(turn_walls, walls)


def _mark_boundary_connected(walls, w, h):
    """标记所有与边界墙连通的墙. 返回 set of connected walls."""
    boundary_connected = set()
    q = deque()
    for wall in walls:
        if is_boundary_wall(wall, w, h):
            boundary_connected.add(wall)
            q.append(wall)
    while q:
        wpos = q.popleft()
        for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
            nb = (wpos[0]+dx, wpos[1]+dy)
            if nb in walls and nb not in boundary_connected:
                boundary_connected.add(nb)
                q.append(nb)
    return boundary_connected


def _detect_corridor_benefit(all_walls, original_walls, w, h):
    """方案2: 检测炸后行/列是否形成单段可移动区域.
    返回受益的行列列表 [(axis, idx), ...] axis='row'|'col'
    """
    boundary_conn = _mark_boundary_connected(all_walls, w, h)
    isolated = all_walls - boundary_conn  # 不与边界连通的墙

    old_boundary_conn = _mark_boundary_connected(original_walls, w, h)
    old_isolated = original_walls - old_boundary_conn

    corridors = []

    # 逐行检测
    for y in range(h):
        old_iso_in_row = {w for w in old_isolated if w[1] == y}
        new_iso_in_row = {w for w in isolated if w[1] == y}
        # 之前有孤立墙分割, 现在没了 → 通道形成
        if old_iso_in_row and not new_iso_in_row:
            # 确认可移动区域是单段: 数非墙格
            segments = 0
            in_seg = False
            for x in range(w):
                if (x, y) in all_walls:
                    if in_seg:
                        segments += 1
                        in_seg = False
                else:
                    in_seg = True
            if in_seg:
                segments += 1
            if segments == 1:
                corridors.append(('row', y))

    # 逐列检测
    for x in range(w):
        old_iso_in_col = {w for w in old_isolated if w[0] == x}
        new_iso_in_col = {w for w in isolated if w[0] == x}
        if old_iso_in_col and not new_iso_in_col:
            segments = 0
            in_seg = False
            for y in range(h):
                if (x, y) in all_walls:
                    if in_seg:
                        segments += 1
                        in_seg = False
                else:
                    in_seg = True
            if in_seg:
                segments += 1
            if segments == 1:
                corridors.append(('col', x))

    return corridors


def _blast_synergy_bonus(selected_walls, walls):
    """方案3: 检测炸点3x3区域是否相邻/重叠, 返回微量奖励分"""
    if len(selected_walls) < 2:
        return 0
    blasts = [get_walls_in_3x3(w, walls) for w in selected_walls]
    bonus = 0
    for i in range(len(blasts)):
        for j in range(i+1, len(blasts)):
            union = blasts[i] | blasts[j]
            overlap = blasts[i] & blasts[j]
            # 有重叠或相邻 (曼哈顿距离≤3)
            min_dist = min(abs(w1[0]-w2[0]) + abs(w1[1]-w2[1])
                          for w1 in blasts[i] for w2 in blasts[j])
            if min_dist <= 1:  # 相邻或重叠
                # 奖励 = (合并面积 - 各自面积)*0.5, 上限3分
                gain = max(0, len(union) - len(blasts[i]) - len(blasts[j]) + len(overlap))
                bonus += min(gain, 3)
    return bonus


def _goal_vicinity_clearance(selected_walls, goals, walls, w, h):
    """方案4: 检测目标近邻墙体清除. 返回 {goal: cleared_wall_count}"""
    vicinity = {}
    for gw in selected_walls:
        blast = get_walls_in_3x3(gw, walls)
        for goal in goals:
            gx, gy = goal
            # 检查goal周边5x5区域(排除goal本身)
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    if dx == 0 and dy == 0:
                        continue
                    n = (gx+dx, gy+dy)
                    if n in blast and 0 <= n[0] < w and 0 <= n[1] < h:
                        vicinity[goal] = vicinity.get(goal, 0) + 1
    return vicinity


# =============================================================================
# 粗筛
# =============================================================================

def _is_isolated_wall(wall, walls):
    x, y = wall
    for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
        if (x+dx, y+dy) in walls:
            return False
    return True


def _blast_affects_goal(wall, goal, w, h):
    """炸点 3x3 范围是否覆盖目标附近"""
    gx, gy = goal
    wx, wy = wall
    # 3x3范围: (wx-1,wy-1) 到 (wx+1,wy+1)
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            n = (gx+dx, gy+dy)
            if not (0 <= n[0] < w and 0 <= n[1] < h):
                continue
            if abs(dx) + abs(dy) > 2:
                continue
            # n 在 blast 3x3内?
            if abs(n[0] - wx) <= 1 and abs(n[1] - wy) <= 1:
                return True
    return False


def _coarse_rank(wall, bomb, goal, walls):
    w_bomb = 5 if manhattan_distance(bomb, wall) > 10 else 2
    w_goal = 1 if manhattan_distance(goal, wall) > 12 else 3
    blast_count = len(get_walls_in_3x3(wall, walls))
    return manhattan_distance(bomb, wall) * w_bomb + manhattan_distance(goal, wall) * w_goal - blast_count * 2


def _coarse_filter(walls_pool, bomb, goal, walls, w, h):
    filtered = []
    for wall in walls_pool:
        if _is_isolated_wall(wall, walls):
            continue
        if len(get_walls_in_3x3(wall, walls)) <= 1:
            continue
        filtered.append(wall)
    filtered.sort(key=lambda wall: _coarse_rank(wall, bomb, goal, walls))
    return filtered[:16]


# =============================================================================
# 精评
# =============================================================================

def _precise_evaluate(box, goal, bomb, wall, walls, w, h, obstacles, all_bombs,
                      car, car_obs, narrow_set):
    """精评: 推弹+车路径+炸后推箱总代价"""
    bomb_result = _bfs_push_bomb_with_car(bomb, wall, walls, w, h,
                                            (obstacles - walls) - {bomb},
                                            car_start=car)
    if bomb_result is None:
        return None
    push_cost, first_dir, bomb_turns, inter_car = bomb_result

    # 车 BFS: 从车当前位置到推弹起始位置
    fd = _DIR_VEC[first_dir]
    car_target = (bomb[0] - fd[0], bomb[1] - fd[1])
    car_dist = None
    result = bfs_shortest_path(car, car_target, walls | car_obs, w, h)
    if result is not None:
        car_dist = result[0]
    if car_dist is None:
        car_dist = 50  # 当前障碍物下车不可达, 给高代价但不排除

    temp_walls = walls - get_walls_in_3x3(wall, walls)
    post_obs = (obstacles - walls) - {box} - set(all_bombs)
    box_result = _bfs_push_box_path(box, goal, temp_walls, w, h, post_obs,
                                     narrow_set=narrow_set)
    if box_result is None:
        return None
    box_steps, box_turns, narrow_count = box_result

    # P3: 直线奖励 (推箱0转弯 + 步数>2)
    straight_bonus = 12 if box_turns == 0 and box_steps > 2 else 0
    # P4: 窄道惩罚 (每窄道格 ≈ 1步推箱代价)
    narrow_penalty = narrow_count * 6

    total = push_cost + box_steps + car_dist

    return {
        'wall': wall, 'bomb': bomb,
        'push_cost': push_cost, 'box_cost': box_steps,
        'box_turns': box_turns, 'bomb_turns': bomb_turns,
        'car_dist': car_dist, 'inter_car': inter_car,
        'narrow_count': narrow_count,
        'total': push_cost + box_steps,
        'eval_cost': (push_cost * 8 + box_steps * 6 + box_turns * 25
                      + bomb_turns * 20 + car_dist * 2 + inter_car * 2
                      - straight_bonus + narrow_penalty),
    }


# =============================================================================
# 主入口
# =============================================================================

def refine_blast_points(map_data, alloc, connectivity, channel_analysis, blast_preplan):
    walls = map_data['walls']
    boxes = map_data['boxes']
    bombs = map_data['bombs']
    goals = map_data['goals']
    w, h = map_data['width'], map_data['height']

    all_obstacles = walls | set(boxes) | set(bombs)
    car_obs = walls | set(boxes) | set(bombs)
    narrow_set = compute_narrow_cells(walls, w, h)
    car = map_data['car']
    dl_bombs_pos = {pos for _, pos, _ in channel_analysis.get('deadlocked_bombs', [])}

    refined_pairs = []
    for p in alloc.get('pairs', []):
        box = p['box']
        goal = p['goal']
        bp = p.get('bomb_plan')
        use_bomb = p.get('uses_bomb', False)

        # --- 直达对 ---
        if not use_bomb and not (bp and bp.get('chain')):
            post_obs = (all_obstacles - walls) - {box}
            result = _bfs_push_box_path(box, goal, walls, w, h, post_obs)
            if result:
                steps, turns, _ = result
                p['path_steps'] = steps
                p['path_turns'] = turns
                p['score'] = 1000 - steps * 10 - turns * 30
            # 若 BFS 失败, 保留步骤4原有的 path_steps
            refined_pairs.append(p)
            continue

        # --- 链救援 ---
        if bp and bp.get('chain'):
            chain = bp['chain']
            sim_walls = set(walls)
            sim_obs = all_obstacles - walls
            total_push = 0
            chain_ok = True

            for cbomb, cwall in chain:
                result = _bfs_push_bomb_with_car(cbomb, cwall, sim_walls, w, h,
                                                   sim_obs - {cbomb})
                if result is None:
                    chain_ok = False
                    break
                push_steps, _, _, _ = result
                total_push += push_steps
                sim_walls -= get_walls_in_3x3(cwall, sim_walls)
                sim_obs.discard(cbomb)

            if chain_ok:
                post_obs = sim_obs - {box}
                box_result = _bfs_push_box_path(box, goal, sim_walls, w, h, post_obs)
                if box_result:
                    box_steps, box_turns, _ = box_result
                    p['path_steps'] = box_steps
                    p['path_turns'] = box_turns
                    p['score'] = 1000 - box_steps*6 - box_turns*25 - bp.get('chain_depth',1)*300
                    bp['bomb_steps'] = total_push
                    bp['box_steps'] = box_steps
                    bp['refined_total'] = total_push + box_steps
                    p['bomb_plan'] = bp
                else:
                    bp['bomb_steps'] = total_push
                    p['bomb_plan'] = bp
            refined_pairs.append(p)
            continue

        # --- 单炸弹对 ---
        bomb = bp.get('bomb') if bp else None
        if bomb is None:
            # 不改步骤4的决定 — 没分配就是不分配
            refined_pairs.append(p)
            continue

        # 候选池
        pool = set()
        pool.update(get_walls_in_rect(box, goal, walls, expand=3))
        pool.update(_get_walls_near_goal(goal, walls, w, h, radius=2))
        for bc in blast_preplan.get('blast_candidates', []):
            bw = bc['wall']
            if manhattan_distance(bw, box) + manhattan_distance(bw, goal) <= manhattan_distance(box, goal) + 8:
                pool.add(bw)
        pool = {wall for wall in pool if not is_boundary_wall(wall, w, h)}

        if not pool:
            pool = get_walls_in_rect(box, goal, walls, expand=5)
            pool = {wall for wall in pool if not is_boundary_wall(wall, w, h)}

        if not pool:
            refined_pairs.append(p)
            continue

        top = _coarse_filter(pool, bomb, goal, walls, w, h)

        best = None
        no_improve = 0
        for wall in (top[:12] if len(top) >= 12 else top):
            result = _precise_evaluate(box, goal, bomb, wall, walls, w, h,
                                        all_obstacles, bombs, car, car_obs, narrow_set)
            if result is None:
                no_improve += 1
                if no_improve >= 3:
                    break
                continue
            if best is None or result['eval_cost'] < best['eval_cost']:
                best = result
                no_improve = 0

        if best:
            bp_new = {
                'bomb': best['bomb'],
                'wall': best['wall'],
                'bomb_steps': best['push_cost'],
                'bomb_turns': best.get('bomb_turns', 0),
                'box_steps': best['box_cost'],
                'box_turns': best.get('box_turns', 0),
                'chain_depth': 1,
                'refined_total': best['total'],
            }
            p['bomb_plan'] = bp_new
            p['path_steps'] = best['box_cost']
            p['path_turns'] = best.get('box_turns', 0)
            p['score'] = (1000 - best['box_cost']*6 - best['box_turns']*25
                          - best['push_cost']*8 - best.get('bomb_turns', 0)*20
                          - best.get('car_dist', 0)*2 - best.get('inter_car', 0)*2)
        else:
            p['needs_chain'] = True

        refined_pairs.append(p)

    # ---- 全局验证: 模拟全部炸墙, 捕捉跨对收益 ----
    all_walls = set(walls)
    all_disappeared = set()
    selected_walls = []
    for p in refined_pairs:
        bp = p.get('bomb_plan')
        if bp and bp.get('wall'):
            all_walls -= get_walls_in_3x3(bp['wall'], all_walls)
            selected_walls.append(bp['wall'])
            if bp.get('bomb'):
                all_disappeared.add(bp['bomb'])
        if bp and bp.get('chain'):
            for cbomb, cwall in bp['chain']:
                all_walls -= get_walls_in_3x3(cwall, all_walls)
                selected_walls.append(cwall)
                all_disappeared.add(cbomb)

    # ---- 方案2: 通道检测 ----
    corridors = _detect_corridor_benefit(all_walls, walls, w, h)

    # ---- 方案1: 转向墙检测 ----
    turn_wall_benefit = {}  # pair_idx -> bool
    for i, p in enumerate(refined_pairs):
        post_obs = (all_obstacles - walls) - {p['box']} - set(bombs)
        groups = _find_turn_wall_groups(p['box'], p['goal'], walls, w, h, post_obs)
        if groups:
            all_turn_walls = set.union(*groups)
            cleared_turn = all_turn_walls - all_walls  # all_walls已移除炸掉的墙
            if cleared_turn:
                turn_wall_benefit[i] = True

    # ---- 方案3: 炸点合力 ----
    synergy_bonus = _blast_synergy_bonus(selected_walls, walls)

    # ---- 方案4: 近邻清障 ----
    goal_clearance = _goal_vicinity_clearance(selected_walls, goals, walls, w, h)

    # ---- 合并重算 ----
    cross_benefit_pairs = set()
    # 方案2触发: 通道所在行/列附近的对
    for axis, idx in corridors:
        for i, p in enumerate(refined_pairs):
            if axis == 'row' and abs(p['box'][1] - idx) <= 3:
                cross_benefit_pairs.add(i)
            elif axis == 'col' and abs(p['box'][0] - idx) <= 3:
                cross_benefit_pairs.add(i)
    # 方案1触发
    cross_benefit_pairs.update(turn_wall_benefit.keys())

    for i in cross_benefit_pairs:
        p = refined_pairs[i]
        post_obs = (all_obstacles - walls) - {p['box']} - set(bombs)
        new_result = _bfs_push_box_path(p['box'], p['goal'], all_walls, w, h, post_obs)
        old_steps = p.get('path_steps', 999)
        old_turns = p.get('path_turns', 999)
        if new_result:
            new_steps, new_turns, _ = new_result
            if new_steps < old_steps:
                p['path_steps'] = new_steps
                p['path_turns'] = new_turns
                p['score'] = p.get('score', 0) + (old_steps - new_steps) * 10
            elif new_turns < old_turns:
                p['path_turns'] = new_turns
                p['score'] = p.get('score', 0) + (old_turns - new_turns) * 5
        elif old_steps == 999:
            p['path_steps'] = 0
            p['path_turns'] = 0

    # 方案3奖励
    if synergy_bonus:
        for p in refined_pairs:
            p['score'] = (p.get('score', 0) or 0) + synergy_bonus

    # 方案4标记 (写入pair, 供步骤7使用)
    for p in refined_pairs:
        goal = p['goal']
        if goal in goal_clearance:
            p['goal_vicinity_cleared'] = goal_clearance[goal]

    # 基础重算 (非跨对受益的pair也走一遍合并地图)
    for i, p in enumerate(refined_pairs):
        if i in cross_benefit_pairs:
            continue  # 已重算
        post_obs = (all_obstacles - walls) - {p['box']} - set(bombs)
        new_result = _bfs_push_box_path(p['box'], p['goal'], all_walls, w, h, post_obs)
        old_steps = p.get('path_steps', 999)
        old_turns = p.get('path_turns', 999)
        was_unreachable = (old_steps == 999 or old_steps <= 0)
        needs_chain = p.get('needs_chain', False)

        if new_result:
            new_steps, new_turns, _ = new_result
            # 之前不可达(needs_chain或无路径), 合并地图下可达 → 可能搭别的对炸墙的便车变直达
            if was_unreachable or needs_chain:
                # 关键: 降级判定必须排除"该对自己炸弹炸的墙"的增益,
                # 否则会用自己的炸弹证明自己不需要炸弹(循环论证), 丢失 bomb_plan.
                # 仅当扣除自身炸墙后(只剩别的对的炸墙)仍可达, 才允许降级为直达。
                own_bp = p.get('bomb_plan')
                walls_no_self = all_walls
                if own_bp:
                    own_walls = set()
                    if own_bp.get('wall'):
                        own_walls |= get_walls_in_3x3(own_bp['wall'], walls)
                    if own_bp.get('chain'):
                        for _cbomb, _cwall in own_bp['chain']:
                            own_walls |= get_walls_in_3x3(_cwall, walls)
                    # 把自己炸掉的墙加回去(保留别的对的炸墙增益)
                    walls_no_self = all_walls | own_walls

                downgrade_result = (new_result if walls_no_self is all_walls
                                    else _bfs_push_box_path(p['box'], p['goal'],
                                                            walls_no_self, w, h, post_obs))

                if downgrade_result:
                    # 不靠自己的炸弹也可达 → 真·搭便车, 降级为直达
                    d_steps, d_turns, _ = downgrade_result
                    p['needs_chain'] = False
                    p['uses_bomb'] = False
                    p['bomb_plan'] = None
                    p['path_steps'] = d_steps
                    p['path_turns'] = d_turns
                    p['score'] = 1000 - d_steps * 10 - d_turns * 30  # 直达评分
                    p['_merged_walls'] = walls_no_self  # 传递合并墙态给step6
                else:
                    # 必须靠自己的炸弹才可达 → 保留 bomb_plan, 不降级。
                    # 此时 path_steps 用含自身炸墙的可达步数更新(供调度评估)。
                    p['path_steps'] = new_steps
                    p['path_turns'] = new_turns
            elif new_steps < old_steps:
                p['path_steps'] = new_steps
                p['path_turns'] = new_turns
                p['score'] = p.get('score', 0) + (old_steps - new_steps) * 10
            elif new_turns < old_turns:
                p['path_turns'] = new_turns
                p['score'] = p.get('score', 0) + (old_turns - new_turns) * 5
        elif was_unreachable:
            p['path_steps'] = 0
            p['path_turns'] = 0

    # summary
    all_ok = all(p.get('path_steps', 0) > 0 for p in refined_pairs)
    has_chain = any(p.get('needs_chain', False) for p in refined_pairs)
    used = {p.get('bomb_plan', {}).get('bomb') for p in refined_pairs if p.get('bomb_plan')}
    used.discard(None)
    unused = [b for b in bombs if b not in used]
    total_score = sum(p.get('score', 0) for p in refined_pairs)

    return {
        'pairs': refined_pairs,
        'unused_bombs': unused,
        'unused_goals': alloc.get('unused_goals', set()),
        'summary': {
            'total_bombs_used': len(used),
            'is_solvable': all_ok,
            'needs_chain_rescue': has_chain,
            'total_score': total_score,
        },
    }
