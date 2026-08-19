"""
路径搜索模块 (步骤 6)

将每对 (box, goal, bomb, wall) 分解为 MicroTask 序列。
"""

import heapq
from dataclasses import dataclass, field
from collections import deque
from typing import List, Tuple, Optional, Set, Dict

from core.map_analysis import get_walls_in_3x3
from core.bfs import bfs_shortest_path as car_bfs  # 统一 BFS 实现

_DIR_VEC = {'UP': (0, -1), 'DOWN': (0, 1), 'LEFT': (-1, 0), 'RIGHT': (1, 0)}
_DIR_ORDER = [(0, -1, 'UP'), (0, 1, 'DOWN'), (-1, 0, 'LEFT'), (1, 0, 'RIGHT')]


# =============================================================================
# 数据结构
# =============================================================================

@dataclass
class MicroTask:
    """一个不可分割的推物段 (连续同向推)"""
    pair_id: int
    seg_idx: int
    task_type: str               # 'push_box' | 'push_bomb'
    obj_start: Tuple[int, int]
    obj_end: Tuple[int, int]
    push_dir: str
    n_steps: int
    car_target: Tuple[int, int]  # 推之前车应站的位置
    car_end: Tuple[int, int]     # 推完后车的位置


@dataclass
class PairPlan:
    """一对 (box, goal) 的完整操作计划"""
    pair_id: int
    box: Tuple[int, int]
    goal: Tuple[int, int]
    is_direct: bool
    bomb: Optional[Tuple[int, int]] = None
    wall: Optional[Tuple[int, int]] = None
    bomb_tasks: List[MicroTask] = field(default_factory=list)
    blast_clears: List[Tuple[int, int]] = field(default_factory=list)
    pre_box_tasks: List[MicroTask] = field(default_factory=list)   # 炸前可推的箱段
    post_box_tasks: List[MicroTask] = field(default_factory=list)  # 炸后才能推的箱段
    box_tasks: List[MicroTask] = field(default_factory=list)       # 兼容: pre+post
    is_chain: bool = False
    chain: Optional[List] = None


# =============================================================================
# 死锁检测
# =============================================================================

def _is_deadlock(pos: Tuple[int, int], walls: Set[Tuple[int, int]],
                 width: int, height: int) -> bool:
    """角落死锁: 两邻墙形成直角"""
    x, y = pos
    if x <= 0 or x >= width - 1 or y <= 0 or y >= height - 1:
        return True
    corners = [
        ((x - 1, y), (x, y - 1)), ((x + 1, y), (x, y - 1)),
        ((x - 1, y), (x, y + 1)), ((x + 1, y), (x, y + 1)),
    ]
    for w1, w2 in corners:
        if w1 in walls and w2 in walls:
            return True
    return False


# =============================================================================
# 段合并
# =============================================================================

def _merge_push_segments(path: List[Tuple[Tuple[int, int], str]],
                         obj_start: Tuple[int, int]) -> List[Dict]:
    """将推物路径中连续同向的步骤合并为段"""
    if not path:
        return []
    segs = []
    i = 0
    while i < len(path):
        d = path[i][1]
        j = i
        while j < len(path) and path[j][1] == d:
            j += 1
        ss = obj_start if i == 0 else path[i - 1][0]
        se = path[j - 1][0]
        n = j - i
        dx, dy = _DIR_VEC[d]
        segs.append({
            'obj_start': ss, 'obj_end': se, 'push_dir': d, 'n_steps': n,
            'car_target': (ss[0] - dx, ss[1] - dy),
            'car_end': (ss[0] + dx * (n - 1), ss[1] + dy * (n - 1)),
        })
        i = j
    return segs


# =============================================================================
# 推箱 BFS (Dijkstra + 路径追踪)
# =============================================================================

def push_box_bfs(box: Tuple[int, int], goal: Tuple[int, int],
                 walls: Set[Tuple[int, int]], width: int, height: int,
                 obstacles: Set[Tuple[int, int]],
                 initial_car_pos: Optional[Tuple[int, int]] = None,
                 max_steps: int = 200):
    """
    Dijkstra 推箱搜索, 代价 = 步数 + 转向 + 段间车移动(曼哈顿估计).

    Returns: (steps, turns, path) or None
      path = [(pos, dir_name), ...]  从 box 出发到 goal, 含每步的推方向
    """
    if box == goal:
        return 0, 0, []
    _r2_base = walls | obstacles  # 在本次调用内固定, 预算一次避免内层循环反复 |
    _r2_cache: Dict = {}          # (car_after_prev, car_pos, pos) -> bool
    # heap: (cost, steps, turns, est_car, pos, prev_dir, path)
    heap = [(0, 0, 0, 0, box, None, [])]
    best: Dict[Tuple, int] = {}

    while heap:
        _, steps, turns, est_car, pos, prev_dir, path = heapq.heappop(heap)

        if pos == goal:
            return steps, turns, path

        key = (pos, prev_dir)
        if key in best:
            ps = best[key]
            if ps[0] < steps or (ps[0] == steps and ps[1] <= turns):
                continue
        best[key] = (steps, turns)
        if steps >= max_steps:
            continue

        for dx, dy, dname in _DIR_ORDER:
            car_pos = (pos[0] - dx, pos[1] - dy)
            nb = (pos[0] + dx, pos[1] + dy)
            if not (0 <= car_pos[0] < width and 0 <= car_pos[1] < height):
                continue
            if car_pos in walls or car_pos in obstacles:
                continue
            if not (0 <= nb[0] < width and 0 <= nb[1] < height):
                continue
            if nb in walls or nb in obstacles:
                continue
            if nb != goal and _is_deadlock(nb, walls, width, height):
                continue

            nt = turns + (1 if prev_dir is not None and prev_dir != dname else 0)

            # 车上一步推完所在的位置 (用于车间移动估计 + 可达性检查)
            if prev_dir is not None:
                pdx, pdy = _DIR_VEC[prev_dir]
                car_after_prev = (pos[0] - pdx, pos[1] - pdy)
            elif initial_car_pos is not None:
                car_after_prev = initial_car_pos
            else:
                car_after_prev = pos

            # 车可达性: 仅当车需要换位时检查 (同向连续推时 car_after_prev==car_pos, 车自然跟进)。
            # 防止 push_box_bfs 选中"车站位静态非墙但实际是死口袋/绕不进"的推法,
            # 否则该 plan 会在步骤7 scheduler 的 car_bfs 处失败而整体无解。
            # 障碍含当前箱子位置 pos (车必须绕过箱子才能到推动站位)。
            if car_pos != car_after_prev:
                _ck = (car_after_prev, car_pos, pos)
                _ok = _r2_cache.get(_ck)
                if _ok is None:
                    _ok = car_bfs(car_after_prev, car_pos,
                                  _r2_base | {pos}, width, height) is not None
                    _r2_cache[_ck] = _ok
                if not _ok:
                    continue

            # 车间移动估计 (曼哈顿)
            delta_car = abs(car_after_prev[0] - car_pos[0]) + \
                        abs(car_after_prev[1] - car_pos[1])

            ns = steps + 1
            cost = ns + nt * 2 + est_car + delta_car
            new_path = path + [(nb, dname)]
            heapq.heappush(heap, (cost, ns, nt, est_car + delta_car,
                                   nb, dname, new_path))
    return None


# =============================================================================
# 推弹 BFS (Dijkstra + 路径追踪)
# =============================================================================

def push_bomb_bfs(bomb: Tuple[int, int], wall: Tuple[int, int],
                  walls: Set[Tuple[int, int]], width: int, height: int,
                  obstacles: Set[Tuple[int, int]],
                  car_start: Optional[Tuple[int, int]] = None,
                  max_steps: int = 200):
    """
    Dijkstra 推弹搜索, 代价模型同推箱.

    Returns: (steps, turns, path) or None
    """
    if bomb == wall:
        return None
    _r2_base = walls | obstacles
    _r2_cache: Dict = {}
    # heap: (cost, steps, turns, total_car, pos, car_pos, prev_dir, path)
    heap = [(0, 0, 0, 0, bomb, None, None, [])]
    best: Dict[Tuple, int] = {}

    while heap:
        _, steps, turns, total_car, pos, car_pos, prev_dir, path = heapq.heappop(heap)

        key = (pos, car_pos)
        if key in best:
            ps, pt = best[key]
            if ps < steps or (ps == steps and pt <= total_car):
                continue
        best[key] = (steps, total_car)
        if steps >= max_steps:
            continue

        for dx, dy, dname in _DIR_ORDER:
            nb = (pos[0] + dx, pos[1] + dy)
            cp = (pos[0] - dx, pos[1] - dy)
            if not (0 <= cp[0] < width and 0 <= cp[1] < height):
                continue
            if cp in walls or cp in obstacles:
                continue

            # R2 车可达性: 推动方向改变需车换位时, 验证车能否从上一步推完的
            # 位置走到新站位 cp (障碍含当前炸弹位置 pos). 同向连续推车自然跟进不检查.
            # 与 push_box_bfs 的 R2 对称. 缺此验证会选中"车站位静态合法但实际绕不进"
            # 的引爆路径, 致步骤7 scheduler 的 car_bfs 失败而整体无解 (如 map8 pair2
            # 选 wall=(1,3) 的第二段需车站(3,3), 但推完首段车被炸弹封在左上角口袋).
            if prev_dir is not None:
                _pdx, _pdy = _DIR_VEC[prev_dir]
                _car_after_prev = (pos[0] - _pdx, pos[1] - _pdy)
            elif car_start is not None:
                _car_after_prev = car_start
            else:
                _car_after_prev = pos
            if cp != _car_after_prev:
                _ck = (_car_after_prev, cp, pos)
                _ok = _r2_cache.get(_ck)
                if _ok is None:
                    _ok = car_bfs(_car_after_prev, cp,
                                  _r2_base | {pos}, width, height) is not None
                    _r2_cache[_ck] = _ok
                if not _ok:
                    continue

            nt = turns + (1 if prev_dir is not None and prev_dir != dname else 0)

            if car_pos is not None:
                car_move = abs(car_pos[0] - cp[0]) + abs(car_pos[1] - cp[1])
            elif car_start is not None:
                car_move = abs(car_start[0] - cp[0]) + abs(car_start[1] - cp[1])
            else:
                car_move = 0

            if nb == wall:
                new_path = path + [(nb, dname)]
                return steps + 1, nt, new_path
            if not (0 <= nb[0] < width and 0 <= nb[1] < height):
                continue
            if nb in walls or nb in obstacles:
                continue

            ns = steps + 1
            cost = ns + nt * 2 + total_car + car_move
            new_path = path + [(nb, dname)]
            heapq.heappush(heap, (cost, ns, nt, total_car + car_move,
                                   nb, cp, dname, new_path))
    return None


# =============================================================================
# 炸前可达性检测 + 路径分割
# =============================================================================

def _find_pre_blast_reachable(box, walls, w, h, obstacles, max_steps=200):
    """BFS: 找出炸前(原墙)能到达的所有格子"""
    from collections import deque
    reachable = {box}
    q = deque([box])
    while q:
        pos = q.popleft()
        if len(reachable) > max_steps * 4:
            continue
        for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
            nb = (pos[0]+dx, pos[1]+dy)
            cp = (pos[0]-dx, pos[1]-dy)
            if not (0 <= cp[0] < w and 0 <= cp[1] < h):
                continue
            if cp in walls or cp in obstacles:
                continue
            if nb in walls or nb in obstacles:
                continue
            if nb in reachable:
                continue
            reachable.add(nb)
            q.append(nb)
    return reachable


def _segment_valid_pre_blast(task, walls_pre, w, h, obstacles):
    """验证单段推箱在炸前是否合法"""
    dx, dy = _DIR_VEC[task.push_dir]
    cur = task.obj_start
    for k in range(task.n_steps):
        nxt = (cur[0]+dx, cur[1]+dy)
        car_stand = (cur[0]-dx, cur[1]-dy)
        if not (0 <= car_stand[0] < w and 0 <= car_stand[1] < h):
            return False
        if car_stand in walls_pre or car_stand in obstacles:
            return False
        if nxt in walls_pre or nxt in obstacles:
            return False
        cur = nxt
    return True


def _split_box_tasks(box_tasks, walls_pre, walls_post, w, h, obstacles):
    """逐段验证: 第一段在炸前不合法的就是分割点"""
    pre_tasks = []
    post_tasks = []
    found_split = False
    for t in box_tasks:
        if not found_split and _segment_valid_pre_blast(t, walls_pre, w, h, obstacles):
            pre_tasks.append(t)
        else:
            found_split = True
            post_tasks.append(t)

    # 重编号
    for j, t in enumerate(pre_tasks):
        t.seg_idx = j
    for j, t in enumerate(post_tasks):
        t.seg_idx = len(pre_tasks) + j
    return pre_tasks, post_tasks


# =============================================================================
# 主入口: 生成所有 PairPlan
# =============================================================================

def generate_pair_plans(map_data: Dict, refined_pairs: List[Dict]) -> List[PairPlan]:
    """
    将步骤5的 refined_pairs 转为 PairPlan 列表。

    每个 PairPlan 包含 bomb_tasks (推弹段) 和 box_tasks (推箱段)。
    """
    walls: Set[Tuple[int, int]] = map_data['walls']
    boxes = map_data['boxes']
    bombs = map_data['bombs']
    w, h = map_data['width'], map_data['height']
    car_start = map_data['car']
    all_obstacles = walls | set(boxes) | set(bombs)

    # 已分配的炸弹: 会被推走→炸掉, 推箱路径中可以排除
    used_bombs = set()
    for p in refined_pairs:
        bp = p.get('bomb_plan')
        if bp and bp.get('bomb'):
            used_bombs.add(bp['bomb'])
        if bp and bp.get('chain'):
            for cbomb, _ in bp['chain']:
                used_bombs.add(cbomb)
    # 未分配的炸弹仍算障碍物
    unused_bombs = set(bombs) - used_bombs

    # 其他对的箱子也会被推走→腾出路, 推箱搜索阶段应像 used_bombs 一样豁免;
    # 它们的实际让路顺序由步骤7 scheduler 交叉执行各对微任务来保证。
    # (否则两个箱子互相挡路时会被误判无解, 如 map2(4) b(2,5)->g(12,5) 被 box(2,8) 挡)
    all_boxes_set = set(boxes)

    # 全局炸墙态: 所有炸弹对的3x3炸墙并集 (供直达对fallback使用)
    _global_blast: Set[Tuple[int, int]] = set()
    for _p in refined_pairs:
        _bp = _p.get('bomb_plan')
        if _bp and _bp.get('wall'):
            _global_blast |= get_walls_in_3x3(_bp['wall'], walls)
        if _bp and _bp.get('chain'):
            for _, _cwall in _bp['chain']:
                _global_blast |= get_walls_in_3x3(_cwall, walls)
    fallback_walls = walls - _global_blast if _global_blast else None

    plans = []
    for i, p in enumerate(refined_pairs):
        box = p['box']
        goal = p['goal']
        bp = p.get('bomb_plan')
        use_bomb = p.get('uses_bomb', False)

        # ---- 直达对 ----
        if not use_bomb and not (bp and bp.get('chain')):
            # 跨对收益: 使用合并墙态(其他pair的炸弹清除后的墙)
            effective_walls = p.get('_merged_walls', walls)
            post_obs = (all_obstacles - walls) - {box} - used_bombs - (all_boxes_set - {box})
            result = push_box_bfs(box, goal, effective_walls, w, h, post_obs)
            # fallback: 原始/合并墙态都不通时, 试全局炸墙态(依赖别对先炸墙).
            # scheduler 的 active_walls 是全局遍历, 会按各炸弹对进度放行,
            # 从而表达"本直达对依赖别对炸墙"的跨对时序依赖.
            if result is None and fallback_walls is not None \
                    and effective_walls != fallback_walls:
                _fb = push_box_bfs(box, goal, fallback_walls, w, h, post_obs)
                if _fb is not None:
                    result = _fb
                    effective_walls = fallback_walls
            if result is None:
                # ---- 第三级: clearance fallback ----
                # 豁免 unused_bombs 找路，若路径穿过某炸弹则顺路推到邻近墙格引爆清障，
                # pair 从直达对升级为炸弹对。只在路径确实穿弹时触发，零误伤其他图。
                if unused_bombs:
                    _cl_obs = ((all_obstacles - walls) - {box}
                               - used_bombs - unused_bombs - (all_boxes_set - {box}))
                    _cl_r = push_box_bfs(box, goal, walls, w, h, _cl_obs)
                    if _cl_r is not None:
                        _path_cells = {pos for pos, _ in _cl_r[2]}
                        _blocker = next((b for b in unused_bombs if b in _path_cells), None)
                        if _blocker is not None:
                            _clr_obs_b = (all_obstacles - walls) - {_blocker}
                            _clr_wall = None
                            for _ddx, _ddy, _ in _DIR_ORDER:
                                _nbc = (_blocker[0] + _ddx, _blocker[1] + _ddy)
                                _cpc = (_blocker[0] - _ddx, _blocker[1] - _ddy)
                                if (_nbc in walls
                                        and 0 <= _cpc[0] < w and 0 <= _cpc[1] < h
                                        and _cpc not in walls and _cpc not in _clr_obs_b):
                                    _clr_wall = _nbc
                                    break
                            if _clr_wall is not None:
                                _br = push_bomb_bfs(_blocker, _clr_wall, walls, w, h,
                                                    _clr_obs_b, car_start=car_start)
                                if _br is not None:
                                    _bsegs = _merge_push_segments(_br[2], _blocker)
                                    _bomb_tasks = [
                                        MicroTask(pair_id=i, seg_idx=j, task_type='push_bomb',
                                                  obj_start=s['obj_start'], obj_end=s['obj_end'],
                                                  push_dir=s['push_dir'], n_steps=s['n_steps'],
                                                  car_target=s['car_target'], car_end=s['car_end'])
                                        for j, s in enumerate(_bsegs)
                                    ]
                                    _blast = list(get_walls_in_3x3(_clr_wall, walls))
                                    _clr_w = walls - set(_blast)
                                    _clr_used = used_bombs | {_blocker}
                                    _post_obs = ((all_obstacles - walls) - {box}
                                                 - _clr_used - (all_boxes_set - {box}))
                                    _br2 = push_box_bfs(box, goal, _clr_w, w, h, _post_obs)
                                    if _br2 is not None:
                                        _bsegs2 = _merge_push_segments(_br2[2], box)
                                        _box_tasks = [
                                            MicroTask(pair_id=i, seg_idx=j, task_type='push_box',
                                                      obj_start=s['obj_start'], obj_end=s['obj_end'],
                                                      push_dir=s['push_dir'], n_steps=s['n_steps'],
                                                      car_target=s['car_target'], car_end=s['car_end'])
                                            for j, s in enumerate(_bsegs2)
                                        ]
                                        plans.append(PairPlan(
                                            pair_id=i, box=box, goal=goal, is_direct=False,
                                            bomb=_blocker, wall=_clr_wall,
                                            bomb_tasks=_bomb_tasks, blast_clears=_blast,
                                            pre_box_tasks=[], post_box_tasks=_box_tasks,
                                            box_tasks=_box_tasks))
                                        continue
                print(f'  [WARN] pair[{i}] DIRECT push_box_bfs failed, box{box}->goal{goal}')
                plans.append(PairPlan(pair_id=i, box=box, goal=goal,
                                       is_direct=True))
                continue
            steps, turns, path = result
            segs = _merge_push_segments(path, box)
            box_tasks = [
                MicroTask(pair_id=i, seg_idx=j, task_type='push_box',
                          obj_start=s['obj_start'], obj_end=s['obj_end'],
                          push_dir=s['push_dir'], n_steps=s['n_steps'],
                          car_target=s['car_target'], car_end=s['car_end'])
                for j, s in enumerate(segs)
            ]
            plans.append(PairPlan(pair_id=i, box=box, goal=goal,
                                   is_direct=True, box_tasks=box_tasks))
            continue

        # ---- 链救援 ----
        if bp and bp.get('chain'):
            chain = bp['chain']
            sim_walls = set(walls)
            sim_obs = all_obstacles - walls
            bomb_tasks = []
            all_blast = set()
            chain_car = car_start  # 追踪车在链中的位置

            chain_ok = True
            for ci, (cbomb, cwall) in enumerate(chain):
                result = push_bomb_bfs(cbomb, cwall, sim_walls, w, h,
                                        sim_obs - {cbomb}, car_start=chain_car)
                if result is None:
                    chain_ok = False
                    break
                # 推完后车在bomb起始位置(车从后方推弹)
                chain_car = cbomb  # 简化: 车停在炸弹原位置
                b_steps, b_turns, b_path = result
                segs = _merge_push_segments(b_path, cbomb)
                for j, s in enumerate(segs):
                    bomb_tasks.append(MicroTask(
                        pair_id=i, seg_idx=j, task_type='push_bomb',
                        obj_start=s['obj_start'], obj_end=s['obj_end'],
                        push_dir=s['push_dir'], n_steps=s['n_steps'],
                        car_target=s['car_target'], car_end=s['car_end']))
                blast = get_walls_in_3x3(cwall, sim_walls)
                sim_walls -= blast
                all_blast.update(blast)
                sim_obs.discard(cbomb)

            if not chain_ok:
                plans.append(PairPlan(pair_id=i, box=box, goal=goal,
                                       is_direct=False, is_chain=True, chain=chain))
                continue

            post_obs = sim_obs - {box} - used_bombs - (all_boxes_set - {box})
            result = push_box_bfs(box, goal, sim_walls, w, h, post_obs)
            box_tasks = []
            if result:
                _, _, bpath = result
                segs = _merge_push_segments(bpath, box)
                box_tasks = [
                    MicroTask(pair_id=i, seg_idx=j, task_type='push_box',
                              obj_start=s['obj_start'], obj_end=s['obj_end'],
                              push_dir=s['push_dir'], n_steps=s['n_steps'],
                              car_target=s['car_target'], car_end=s['car_end'])
                    for j, s in enumerate(segs)
                ]

            plans.append(PairPlan(
                pair_id=i, box=box, goal=goal, is_direct=False,
                bomb=chain[-1][0], wall=chain[-1][1],
                bomb_tasks=bomb_tasks, blast_clears=list(all_blast),
                pre_box_tasks=[], post_box_tasks=box_tasks,
                box_tasks=box_tasks, is_chain=True, chain=chain))
            continue

        # ---- 单炸弹对 ----
        bomb = bp.get('bomb') if bp else None
        wall_target = bp.get('wall') if bp else None
        if bomb is None or wall_target is None:
            plans.append(PairPlan(pair_id=i, box=box, goal=goal,
                                   is_direct=False))
            continue

        bomb_result = push_bomb_bfs(bomb, wall_target, walls, w, h,
                                      (all_obstacles - walls) - {bomb},
                                      car_start=car_start)
        bomb_tasks = []
        blast_clears = []
        if bomb_result is not None:
            _, _, bpath = bomb_result
            segs = _merge_push_segments(bpath, bomb)
            bomb_tasks = [
                MicroTask(pair_id=i, seg_idx=j, task_type='push_bomb',
                          obj_start=s['obj_start'], obj_end=s['obj_end'],
                          push_dir=s['push_dir'], n_steps=s['n_steps'],
                          car_target=s['car_target'], car_end=s['car_end'])
                for j, s in enumerate(segs)
            ]
            blast_clears = list(get_walls_in_3x3(wall_target, walls))

        temp_walls = walls - set(blast_clears)
        post_obs = (all_obstacles - walls) - {box} - used_bombs - (all_boxes_set - {box})
        result = push_box_bfs(box, goal, temp_walls, w, h, post_obs)
        box_tasks = []
        pre_box = []
        post_box = []
        if result:
            _, _, bpath = result
            segs = _merge_push_segments(bpath, box)
            box_tasks = [
                MicroTask(pair_id=i, seg_idx=j, task_type='push_box',
                          obj_start=s['obj_start'], obj_end=s['obj_end'],
                          push_dir=s['push_dir'], n_steps=s['n_steps'],
                          car_target=s['car_target'], car_end=s['car_end'])
                for j, s in enumerate(segs)
            ]
            # 分割: 炸前可达 vs 炸后
            # 已分配的炸弹会被推走, 不应阻止推箱段标记为pre
            # (调度器可通过交叉执行让其他pair先推走挡路的炸弹/箱子)
            pre_obs = (all_obstacles - walls) - {box} - used_bombs - (all_boxes_set - {box})
            pre_box, post_box = _split_box_tasks(box_tasks, walls, temp_walls, w, h, pre_obs)
            # bomb-first: pre_box 路径若穿过本对炸弹自身位置(或车站位落在炸弹上),
            # 则该箱段在炸弹推走前根本无法执行(箱会撞上自己要推的炸弹).
            # _split_box_tasks 因 used_bombs 豁免了炸弹, 漏判此冲突 → scheduler
            # 出现箱-弹路径争用死锁. 此处显式检测: 一旦 pre 段触及 bomb, 整个箱段
            # 改为炸后执行(bomb-first). (如 map5(4) pre (11,9)->(11,7) 过弹 (11,8))
            if pre_box and bomb is not None:
                _hits_bomb = False
                for t in pre_box:
                    if t.car_target == bomb:
                        _hits_bomb = True
                        break
                    dx, dy = _DIR_VEC[t.push_dir]
                    cur = t.obj_start
                    for _ in range(t.n_steps):
                        cur = (cur[0] + dx, cur[1] + dy)
                        if cur == bomb:
                            _hits_bomb = True
                            break
                    if _hits_bomb:
                        break
                # 车被炸弹困住（炸前 car_bfs 到 pre_box 首段车站位不可达）也强制bomb-first
                if not _hits_bomb and \
                        car_bfs(car_start, pre_box[0].car_target,
                                all_obstacles, w, h) is None:
                    _hits_bomb = True
                if _hits_bomb:
                    for j, t in enumerate(box_tasks):
                        t.seg_idx = j
                    pre_box, post_box = [], list(box_tasks)

        plans.append(PairPlan(
            pair_id=i, box=box, goal=goal, is_direct=False,
            bomb=bomb, wall=wall_target,
            bomb_tasks=bomb_tasks, blast_clears=blast_clears,
            pre_box_tasks=pre_box, post_box_tasks=post_box,
            box_tasks=box_tasks))
    return plans
