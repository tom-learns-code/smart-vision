"""
推箱规划模块

包含:
  - push_box_bfs（支持 blocked_states）
  - car_bfs、路径分段合并、段间转换评分
  - generate_actions_for_pair（纯验证，不做修复）
  - generate_actions_with_retry（含扩展/绕行/阻塞三重修复）
"""

import math
import heapq
from collections import deque
from typing import List, Tuple, Optional, Set, Dict

from core.direction import DIRECTION_VECTORS
from core.map_preprocess import MapPreprocess, CellType


# =============================================================================
# 推箱 BFS
# =============================================================================

def push_box_bfs(
    box, goal, walls, width, height, obstacles,
    max_steps=200, blocked_states=None, preprocess=None,
    initial_car_pos=None,
):
    """
    推箱 BFS（代价 = 推箱步数 + 转向 + 段间车位移）。

    initial_car_pos: 初始时车的位置，用于正确估计第一条推箱指令的车位移。
    """
    if box == goal:
        return 0, 0, []
    if blocked_states is None:
        blocked_states = set()
    # heap: (primary_cost, steps, turns, est_car, pos, prev_dir, path)
    heap = [(0, 0, 0, 0, box, None, [])]
    best = {}

    while heap:
        _, steps, turns, est_car, pos, prev_dir, path = heapq.heappop(heap)

        # 目标检测必须在弹出时，保证 Dijkstra 最优性
        if pos == goal:
            return steps, turns, path

        key = (pos, prev_dir)
        if key in best:
            ps, pt = best[key]
            if ps < steps or (ps == steps and pt <= turns):
                continue
        best[key] = (steps, turns)
        if steps >= max_steps:
            continue

        for dname, (dx, dy) in DIRECTION_VECTORS.items():
            if (pos, dname) in blocked_states:
                continue
            car_pos = (pos[0] - dx, pos[1] - dy)  # 本次推箱车的站位
            new_pos = (pos[0] + dx, pos[1] + dy)  # 推完后箱子的位置
            if not (0 <= car_pos[0] < width and 0 <= car_pos[1] < height):
                continue
            if car_pos in walls or car_pos in obstacles:
                continue
            nt = turns + (1 if prev_dir is not None and prev_dir != dname else 0)

            # 段间车位移：车从上次推完的位置 → 本次推箱站位
            if prev_dir is not None:
                pdx, pdy = DIRECTION_VECTORS[prev_dir]
                car_after_prev = (pos[0] - pdx, pos[1] - pdy)
            elif initial_car_pos is not None:
                car_after_prev = initial_car_pos  # 用实际初始车位
            else:
                car_after_prev = pos

            if preprocess is not None:
                delta_car = preprocess.distance(car_after_prev, car_pos)
            else:
                delta_car = (abs(car_after_prev[0] - car_pos[0]) +
                             abs(car_after_prev[1] - car_pos[1]))

            # 箱子阻塞：如果箱子在车位移路径上，用实际 car_bfs 取准确距离
            if _box_blocks_path(car_after_prev, car_pos, pos):
                blocked_obs = obstacles | {pos}
                actual = car_bfs(car_after_prev, car_pos, blocked_obs,
                                 width, height)
                if actual is not None:
                    delta_car = actual[0]  # 用 BFS 实际步数
                else:
                    delta_car = 999  # 不可达

            new_est_car = est_car + delta_car

            if not (0 <= new_pos[0] < width and 0 <= new_pos[1] < height):
                continue
            if new_pos in walls or new_pos in obstacles:
                continue
            if _is_deadlock(new_pos, walls, width, height):
                continue

            ns = steps + 1
            cost = ns + nt * 2 + new_est_car
            heapq.heappush(heap, (cost, ns, nt, new_est_car,
                                   new_pos, dname, path + [(new_pos, dname)]))
    return None


def _is_deadlock(pos, walls, width, height):
    x, y = pos
    if x <= 0 or x >= width - 1 or y <= 0 or y >= height - 1:
        return True
    for w1, w2 in [((x-1,y),(x,y-1)), ((x+1,y),(x,y-1)),
                    ((x-1,y),(x,y+1)), ((x+1,y),(x,y+1))]:
        if w1 in walls and w2 in walls:
            return True
    return False


# =============================================================================
# LOS 路径平滑（阶段二：截弯取直）
# =============================================================================


def _point_to_segment_dist(px, py, ax, ay, bx, by):
    """点 (px, py) 到线段 AB 的最短欧氏距离（网格单位）。"""
    dx = bx - ax
    dy = by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0,
                     ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def _swept_width(ax, ay, bx, by):
    """
    轴对齐 1×1 正方形沿线段 AB 移动时的扫过宽度（垂直于路径方向）。

    公式: (|cosθ| + |sinθ|)
    水平/垂直: 1.0    45°对角: √2 ≈ 1.414
    """
    ddx = bx - ax
    ddy = by - ay
    seg_len = math.hypot(ddx, ddy)
    if seg_len < 0.001:
        return 1.0
    return (abs(ddx) + abs(ddy)) / seg_len


def _has_line_of_sight(a, b, walls, non_target_boxes, width, height):
    """
    基于扫过宽度的 LOS 检测。

    轴对齐 1×1 正方形车体沿线段 AB 移动时，扫过区域的宽度取决于路径角度。
    对于每个障碍格点计算垂直距离，与动态阈值比较。

    a, b:        网格坐标 (x, y)
    walls:       Set[Tuple[int,int]] 墙体格点
    non_target_boxes: Set[Tuple[int,int]] 不可触碰的箱子位置（扩展 8 邻域）
    """
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])

    # 动态阈值：车体扫过宽度 = 车半宽+障碍半宽刚好不重叠的最小距离
    threshold = _swept_width(ax, ay, bx, by)

    # ---- 扩展非目标物体的 8 邻域 ----
    box_expanded = set()
    for gx, gy in non_target_boxes:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    box_expanded.add((nx, ny))

    # ---- 搜索 AABB ----
    margin = int(math.ceil(threshold)) + 1
    min_x = max(0, int(min(ax, bx)) - margin)
    max_x = min(width - 1, int(max(ax, bx)) + margin)
    min_y = max(0, int(min(ay, by)) - margin)
    max_y = min(height - 1, int(max(ay, by)) + margin)

    for gy in range(min_y, max_y + 1):
        for gx in range(min_x, max_x + 1):
            p = (gx, gy)
            if p == a or p == b:
                continue

            if p in walls or p in box_expanded:
                d = _point_to_segment_dist(gx, gy, ax, ay, bx, by)
                if d < threshold:
                    return False

    return True


def _los_smooth(path, walls, non_target_boxes, width, height):
    """
    Line-of-sight 贪心平滑。

    给定一条网格路径（来自 car_bfs），从起点出发每次找直线可达的最远点，
    剔除中间冗余点，生成包含对角捷径的最简 waypoint 序列。

    path:            List[Tuple[int, int]] 原始网格路径（须包含起点和终点）
    walls:           Set[Tuple[int,int]] 墙体格点
    non_target_boxes: Set[Tuple[int,int]] 不可触碰的箱子位置
    """
    if len(path) <= 2:
        return list(path)

    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        furthest = i + 1
        for j in range(len(path) - 1, i, -1):
            if _has_line_of_sight(path[i], path[j], walls,
                                  non_target_boxes, width, height):
                furthest = j
                break
        result.append(path[furthest])
        i = furthest

    return result


# =============================================================================
# 车 BFS / waypoint 提取 / 段合并
# =============================================================================

def car_bfs(car, target, obstacles, width, height):
    if car == target:
        return 0, [car]
    queue = deque([car])
    visited = {car}
    parent = {car: None}
    while queue:
        pos = queue.popleft()
        for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
            np = (pos[0]+dx, pos[1]+dy)
            if np == target:
                path = [np, pos]
                cur = pos
                while parent[cur] is not None:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                return len(path)-1, path
            if not (0 <= np[0] < width and 0 <= np[1] < height):
                continue
            if np in obstacles:
                continue
            if np in visited:
                continue
            visited.add(np)
            parent[np] = pos
            queue.append(np)
    return None


def _extract_waypoints(path):
    if len(path) <= 2:
        return list(path)
    wps = [path[0]]
    for i in range(1, len(path)-1):
        pd = (path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])
        nd = (path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
        if pd != nd:
            wps.append(path[i])
    wps.append(path[-1])
    return wps


def _merge_push_segments(path, box_start):
    if not path:
        return []
    segs, i = [], 0
    while i < len(path):
        d = path[i][1]
        j = i
        while j < len(path) and path[j][1] == d:
            j += 1
        ss = box_start if i == 0 else path[i-1][0]
        se = path[j-1][0]
        n = j - i
        dx, dy = DIRECTION_VECTORS[d]
        segs.append({
            'box_start': ss, 'box_end': se, 'push_dir': d, 'n_steps': n,
            'car_target': (ss[0]-dx, ss[1]-dy),
            'car_end': (ss[0]+dx*(n-1), ss[1]+dy*(n-1)),
        })
        i = j
    return segs


# =============================================================================
# 评分 / Action 构建 / 段扩展 / 方向绕行
# =============================================================================

def score_transition(car_end, car_target, preprocess):
    s = 0
    if not preprocess.same_component(car_end, car_target):
        return 2
    ct = preprocess.cell_type(car_end)
    if ct == CellType.DEAD_END:
        s = 2
    elif ct == CellType.CHANNEL:
        s = max(s, 1)
    if preprocess.is_choke(car_target):
        s = max(s, 1)
    return s


def _mk_push(box_id, seg):
    return {"type": "push_box", "target": seg['car_end'], "theta": None,
            "push_meta": {"box_id": box_id, "box_start": seg['box_start'],
                          "box_target": seg['box_end'], "push_dir": seg['push_dir']},
            "narrow_passage": False}


def _mk_free(target, wps):
    return {"type": "free_move", "target": target, "theta": None,
            "waypoints": wps, "narrow_passage": False}


def _try_extend_segment(seg, extra):
    from core.direction import DIRECTION_VECTORS
    d = seg['push_dir']
    dx, dy = DIRECTION_VECTORS[d]
    n = seg['n_steps'] + extra
    return {
        'box_start': seg['box_start'],
        'box_end': (seg['box_start'][0]+dx*n, seg['box_start'][1]+dy*n),
        'push_dir': d, 'n_steps': n,
        'car_target': (seg['box_start'][0]-dx, seg['box_start'][1]-dy),
        'car_end': (seg['box_start'][0]+dx*(n-1), seg['box_start'][1]+dy*(n-1)),
    }


def _car_can(cp, tgt, obs, w, h):
    return cp == tgt or car_bfs(cp, tgt, obs, w, h) is not None


def _box_blocks_path(a, b, box):
    """检查箱子是否在 a→b 的任意曼哈顿最短路径上。"""
    if box == a or box == b:
        return False
    manhattan = abs(a[0] - b[0]) + abs(a[1] - b[1])
    via_box = (abs(a[0] - box[0]) + abs(a[1] - box[1]) +
               abs(box[0] - b[0]) + abs(box[1] - b[1]))
    return via_box == manhattan


# =============================================================================
# generate_actions_for_pair -- pure validation (no repair)
# =============================================================================

def generate_actions_for_pair(
    box_start, goal, walls, width, height, obstacles,
    car_start, box_id, preprocess=None, blocked_states=None,
    avoidance_graph=None,
):
    """
    Pure segment-by-segment validation. No repair.
    Returns (actions, final_car_pos, failure) or None.
    failure = (box_pos, push_dir, car_pos_before_failure) or None.

    avoidance_graph: optional AvoidanceGraph for composite Dijkstra shortcuts.
    """
    pr = push_box_bfs(box_start, goal, walls, width, height,
                      obstacles, blocked_states=blocked_states,
                      preprocess=preprocess, initial_car_pos=car_start)
    if pr is None:
        return None
    _, _, path = pr
    segs = _merge_push_segments(path, box_start)

    actions, cp = [], car_start
    for seg in segs:
        if cp == seg['car_target']:
            actions.append(_mk_push(box_id, seg))
            cp = seg['car_end']
            continue

        obs = obstacles | {seg['box_start']}
        wps = None

        # 阶段二：优先使用规避节点图（复合 Dijkstra）
        if avoidance_graph is not None:
            wps = avoidance_graph.get_waypoints(cp, seg['car_target'], obs)

        if wps is None:
            # 回退：car_bfs + LOS 平滑
            cr = car_bfs(cp, seg['car_target'], obs, width, height)
            if cr is not None:
                _, cpath = cr
                non_target = obstacles - walls
                los_walls = walls | {seg['box_start']}
                wps = _los_smooth(cpath, los_walls, non_target, width, height)
            else:
                return actions, cp, (seg['box_start'], seg['push_dir'], cp)

        actions.append(_mk_free(seg['car_target'], wps))
        actions.append(_mk_push(box_id, seg))
        cp = seg['car_end']

    return actions, cp, None


# =============================================================================
# 修复策略
# =============================================================================

def _try_extend_and_replan(
    fail_box_pos, fail_dir, fail_car_pos, prev_push_seg,
    goal, walls, obstacles, width, height, box_id, preprocess,
):
    """
    Strategy (a): extend the previous push segment (same direction, more steps),
    then re-plan from the new box position.

    Returns (extended_push_action, new_box_pos, new_car_pos) or None.
    """
    for extra in range(1, 11):
        ns = _try_extend_segment(prev_push_seg, extra)
        nbe = ns['box_end']
        if not (0 <= nbe[0] < width and 0 <= nbe[1] < height):
            break
        if nbe in walls or nbe in obstacles:
            break
        # Can car reach the next target from the extended position?
        # The car should be able to reach the first car_target of the re-planned path.
        # Just check that the new box/car position is valid and try re-plan.
        return _mk_push(box_id, ns), ns['box_end'], ns['car_end']
    return None


def _try_direction_detour(
    box_pos, original_dir, car_pos_before,
    walls, obstacles, width, height, box_id,
):
    """
    Strategy (b): push the box in an alternative direction to free the car.
    Returns [(inserted_actions, new_box_pos, new_car_pos), ...] sorted by steps.
    """
    from core.direction import DIRECTION_VECTORS
    results = []
    for alt_dir, (dx, dy) in DIRECTION_VECTORS.items():
        if alt_dir == original_dir:
            continue
        alt_car = (box_pos[0] - dx, box_pos[1] - dy)
        if not (0 <= alt_car[0] < width and 0 <= alt_car[1] < height):
            continue
        if alt_car in walls or alt_car in obstacles:
            continue
        step1 = car_bfs(car_pos_before, alt_car,
                        obstacles | {box_pos}, width, height)
        if step1 is None:
            continue
        for n_steps in range(1, 11):
            det_end = (box_pos[0] + dx*n_steps, box_pos[1] + dy*n_steps)
            if not (0 <= det_end[0] < width and 0 <= det_end[1] < height):
                break
            if det_end in walls or det_end in obstacles:
                break
            det_car = (box_pos[0] + dx*(n_steps-1), box_pos[1] + dy*(n_steps-1))
            _, sp1 = step1
            det_seg = {
                'box_start': box_pos, 'box_end': det_end,
                'push_dir': alt_dir, 'n_steps': n_steps,
                'car_target': alt_car, 'car_end': det_car,
            }
            inserted = [
                _mk_free(alt_car, _los_smooth(sp1, walls,
                         obstacles - walls, width, height)),
                _mk_push(box_id, det_seg),
            ]
            results.append((inserted, det_end, det_car, n_steps))
    results.sort(key=lambda x: x[3])
    return [(r[0], r[1], r[2]) for r in results]


# =============================================================================
# 辅助：计算 Action 序列的总车程
# =============================================================================

def compute_total_distance(actions, walls, width, height):
    """
    计算 Action 序列的总车程（推箱步数 + 自由移动步数）。

    代价模型：推 1 格 = 1，自由移动 1 格 = 1。
    不区分推箱和走路——车电机转一圈就是一格。
    """
    total = 0
    for a in actions:
        if a['type'] == 'push_box':
            m = a['push_meta']
            # 推箱：箱子只能单轴运动，依然是曼哈顿距离
            total += abs(m['box_target'][0] - m['box_start'][0]) + \
                     abs(m['box_target'][1] - m['box_start'][1])
        elif a['type'] == 'free_move':
            wps = a.get('waypoints')
            if wps and len(wps) >= 2:
                # 阶段二：平滑后的 waypoints 可包含对角段，用欧氏距离
                for i in range(len(wps) - 1):
                    dx = wps[i+1][0] - wps[i][0]
                    dy = wps[i+1][1] - wps[i][1]
                    total += math.hypot(dx, dy)
    return total


# =============================================================================
# generate_actions_with_retry -- full repair pipeline
# =============================================================================

def generate_actions_with_retry(
    box_start, goal, walls, width, height, obstacles,
    car_start, box_id, preprocess=None, max_retries=5,
    avoidance_graph=None,
):
    """
    Full action generation pipeline with repair strategies.

    Returns:
        (actions, final_car_pos) or None
    """
    blocked = set()

    for _ in range(max_retries):
        result = generate_actions_for_pair(
            box_start, goal, walls, width, height, obstacles,
            car_start, box_id, preprocess=preprocess,
            blocked_states=blocked if blocked else None,
            avoidance_graph=avoidance_graph,
        )
        if result is None:
            return None

        prefix_actions, car_before, failure = result
        if failure is None:
            return prefix_actions, car_before

        fail_pos, fail_dir, fail_car = failure

        # ---- Strategy a: extend previous push ----
        if prefix_actions:
            last = prefix_actions[-1]
            if last['type'] == 'push_box':
                prev_meta = last['push_meta']
                prev_seg = {
                    'box_start': prev_meta['box_start'],
                    'box_end': prev_meta['box_target'],
                    'push_dir': prev_meta['push_dir'],
                    'n_steps': 0,
                }
                from core.direction import DIRECTION_VECTORS
                dx, dy = DIRECTION_VECTORS[prev_meta['push_dir']]
                if dx != 0:
                    prev_seg['n_steps'] = abs(prev_meta['box_target'][0] -
                                              prev_meta['box_start'][0])
                else:
                    prev_seg['n_steps'] = abs(prev_meta['box_target'][1] -
                                              prev_meta['box_start'][1])

                ext = _try_extend_and_replan(
                    fail_pos, fail_dir, fail_car, prev_seg,
                    goal, walls, obstacles, width, height, box_id, preprocess,
                )
                if ext is not None:
                    ext_push, new_box, new_car = ext
                    new_prefix = prefix_actions[:-1] + [ext_push]
                    sub = generate_actions_for_pair(
                        new_box, goal, walls, width, height, obstacles,
                        new_car, box_id, preprocess=preprocess,
                        blocked_states=blocked if blocked else None,
                        avoidance_graph=avoidance_graph,
                    )
                    if sub is not None:
                        sub_a, sub_c, sub_f = sub
                        if sub_f is None:
                            return new_prefix + sub_a, sub_c

        # ---- Strategy b: direction detour ----
        detour_options = _try_direction_detour(
            box_pos=fail_pos, original_dir=fail_dir,
            car_pos_before=fail_car,
            walls=walls, obstacles=obstacles,
            width=width, height=height, box_id=box_id,
        )
        for det_actions, new_box, new_car in detour_options:
            sub = generate_actions_for_pair(
                new_box, goal, walls, width, height, obstacles,
                new_car, box_id, preprocess=preprocess,
                blocked_states=blocked if blocked else None,
                avoidance_graph=avoidance_graph,
            )
            if sub is not None:
                sub_a, sub_c, sub_f = sub
                if sub_f is None:
                    return prefix_actions + det_actions + sub_a, sub_c

        # ---- Strategy c: block and retry ----
        blocked.add((fail_pos, fail_dir))

    return None
