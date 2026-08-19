"""
炸点预规划模块 (步骤 3.5)

从小车主分量向外扩散，对所有候选墙体分类打分（P0/P1），
计算受益向量，查找连锁救援方案。
"""

from typing import List, Dict, Tuple, Optional, Set
from collections import deque

from core.map_analysis import (
    is_boundary_wall,
    get_walls_in_3x3,
    analyze_connectivity,
)
from core.distance import manhattan_distance


# 解锁型炸点全局预算: 整个 preplan_blasts 内 push_box_bfs 总调用上限,
# 守 0.5s 红线 (单次约5ms). 每对内再限 confirm 调用数与确认胜出数.
_UNLOCK_TOTAL_BUDGET = 40       # 整图 push_box_bfs 总预算
_UNLOCK_CONFIRM_PER_PAIR = 14   # 单对最多 confirm 次数
_UNLOCK_WINS_PER_PAIR = 3       # 单对确认胜出多少个就停
_UNLOCK_CENTERS_PER_SET = 2     # 每个墙集向 assignment 暴露的引爆中心数


# =============================================================================
# 工具: 推弹 BFS
# =============================================================================

def _bfs_push_bomb(
    bomb_start: tuple,
    target_wall: tuple,
    walls: set,
    width: int,
    height: int,
    obstacles: set,
    max_steps: int = 200
) -> Optional[int]:
    """轻量推弹 BFS: 返回推弹到目标墙的最小步数，不可达返回 None"""
    if bomb_start in obstacles:
        return None

    queue = deque([(bomb_start, 0)])
    visited = {bomb_start}

    while queue:
        pos, steps = queue.popleft()
        if steps > max_steps:
            continue

        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = pos[0] + dx, pos[1] + dy
            nxt = (nx, ny)

            if nxt == target_wall:
                return steps + 1

            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if nxt in walls or nxt in obstacles:
                continue
            if nxt in visited:
                continue

            visited.add(nxt)
            queue.append((nxt, steps + 1))

    return None


# =============================================================================
# 步骤 3.5 主入口
# =============================================================================

def preplan_blasts(
    map_data: dict,
    connectivity: dict,
    channel_analysis: dict,
) -> dict:
    """
    炸点预规划

    Args:
        map_data:         步骤 1 scan_map 输出
        connectivity:     步骤 2b compare_connectivity 输出
        channel_analysis:  步骤 3 analyze_channels 输出

    Returns:
        {
            'main_region': int,
            'blast_candidates': [{wall, benefit, chain_depth, unlocks_bomb, score}, ...],
            'rescue_chains': [{sequence, chain_depth, terminal_target, terminal_benefit_level}, ...],
            'region_adjacent_pairs': [{regions: (r1,r2), wall}, ...],
        }
    """
    w, h = map_data['width'], map_data['height']
    walls = map_data['walls']
    car = map_data['car']
    boxes = map_data['boxes']
    bombs = map_data['bombs']
    goals = map_data['goals']

    current = connectivity['current']
    element_regions = connectivity['element_regions']
    main_region = element_regions['car']

    deadlocked_boxes = channel_analysis.get('deadlocked_boxes', [])
    deadlocked_bombs = channel_analysis.get('deadlocked_bombs', [])

    boxes_set = set(boxes)
    bombs_set = set(bombs)
    all_obstacles = walls | boxes_set | bombs_set
    cars_goals = goals | {car}

    # 当前连通分量 components
    cur_result = analyze_connectivity(
        w, h, all_obstacles, exempt_positions=cars_goals
    )
    current_components = cur_result['components']

    # ---- P0/P1: 区域桥接墙 ----
    boundary_walls = _find_region_boundary_walls(
        walls, current_components, w, h
    )

    candidates = []
    for wall in boundary_walls:
        benefit = _evaluate_wall_benefit(
            wall, map_data, current_components,
        )
        # P0b: 分量中的单向运输（箱或目标在单侧）
        if benefit['p0a'] == 0:
            neigh_comps = _get_wall_neighbor_comps(wall, current_components)
            for nc in neigh_comps:
                b_in = sum(1 for b in boxes if current_components.get(b) == nc)
                g_in = sum(1 for g in goals if current_components.get(g) == nc)
                if b_in > 0 and g_in == 0:
                    benefit['p0b'] += b_in
                if g_in > 0 and b_in == 0:
                    benefit['p0b'] += g_in

        candidates.append({
            'wall': wall,
            'benefit': benefit,
            'chain_depth': 1,
            'unlocks_bomb': False,
            'score': 0.0,
        })

    # ---- P1/P0c: 死锁元素救援 ----
    rescue_chains = []

    for dl_idx, dpos, dl_types in deadlocked_boxes + deadlocked_bombs:
        rescue_walls = _find_walls_around(dpos, walls, w, h)
        for rw in rescue_walls:
            for bidx, bpos in enumerate(bombs):
                cost = _bfs_push_bomb(bpos, rw, walls, w, h,
                                      boxes_set | bombs_set - {bpos})
                if cost is not None:
                    benefit = {
                        'p0a': 0, 'p0b': 0, 'p0c': 1,
                        'p1a': 0, 'p1b': 0, 'p2': 0,
                    }
                    candidates.append({
                        'wall': rw,
                        'benefit': benefit,
                        'chain_depth': 1,
                        'unlocks_bomb': False,
                        'score': 0.0,
                        'frees_element': dpos,
                    })

    # ---- 连锁救援 ----
    all_dead_list = deadlocked_boxes + deadlocked_bombs
    for dl_idx, dpos, dl_types in all_dead_list:
        rescue_walls = _find_walls_around(dpos, walls, w, h)
        for rw in rescue_walls:
            chain = _find_rescue_chain(
                rw, bombs, walls, boxes_set, deadlocked_bombs,
                w, h, depth=1, max_depth=3
            )
            if chain and len(chain) >= 2:
                rescue_chains.append({
                    'sequence': chain,
                    'chain_depth': len(chain),
                    'terminal_target': str(dpos),
                    'terminal_benefit_level': 'P0c',
                })

    # ---- 算分 ----
    for c in candidates:
        b = c['benefit']
        c['score'] = (
            b['p0a'] * 1000 +
            b['p0b'] * 800 +
            b['p0c'] * 800 +
            b['p1a'] * 500 +
            b['p1b'] * 300 +
            b['p2'] * 100
        ) / c['chain_depth']
        if c.get('unlocks_bomb'):
            c['score'] *= 1.5

    candidates.sort(key=lambda c: c['score'], reverse=True)

    # ---- 解锁型炸点 (卡点定位预筛, 含空格引爆中心) ----
    # 对"直达推不动"的对补充引爆中心候选, 按 (box,goal) 分通道暴露给 assignment.
    # 关键: 不并入全局 blast_candidates(那会污染所有对的 walls_to_check),
    # 而是 per-pair 通道, 只让对应的"直达推不动"对消费.
    # 引爆中心可为空格: 炸弹推到空格引爆, 3x3溅射炸斜对角墙 — 当前墙格候选缺失的维度.
    unlock_centers_list = _preplan_unlock_blasts(map_data, connectivity)
    unlock_by_pair = {}
    for uc in unlock_centers_list:
        key = (uc['box'], uc['goal'])
        unlock_by_pair.setdefault(key, []).append(uc)

    # ---- 分量相邻对 ----
    region_adjacent_pairs = _find_region_adjacent_pairs(
        walls, connectivity['ideal']['regions'], w, h
    )

    return {
        'main_region': main_region,
        'blast_candidates': candidates,
        'unlock_by_pair': unlock_by_pair,
        'rescue_chains': rescue_chains,
        'region_adjacent_pairs': region_adjacent_pairs,
    }


# =============================================================================
# 解锁型炸点: 卡点定位预筛 (双向前沿割)
# =============================================================================

def _fwd_push_reachable(box, walls, w, h, obstacles, limit=400):
    """正向: 箱能被推到哪些格 (轻量, 不验R2车可达). 车站位仅查静态非墙非障碍."""
    reach = {box}
    q = deque([box])
    while q and len(reach) < limit:
        pos = q.popleft()
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nb = (pos[0] + dx, pos[1] + dy)
            cp = (pos[0] - dx, pos[1] - dy)  # 车站在箱后方
            if not (0 <= cp[0] < w and 0 <= cp[1] < h):
                continue
            if cp in walls or cp in obstacles:
                continue
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in walls or nb in obstacles:
                continue
            if nb in reach:
                continue
            reach.add(nb)
            q.append(nb)
    return reach


def _bwd_push_reachable(goal, walls, w, h, obstacles, limit=400):
    """反向: 从哪些格出发能把箱推到 goal (反推: 箱在pos, 上一步在prev=pos-d, 车在prev-d)."""
    reach = {goal}
    q = deque([goal])
    while q and len(reach) < limit:
        pos = q.popleft()
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            prev = (pos[0] - dx, pos[1] - dy)       # 箱原位置
            car = (pos[0] - 2 * dx, pos[1] - 2 * dy)  # 推动时车站位
            if not (0 <= prev[0] < w and 0 <= prev[1] < h):
                continue
            if not (0 <= car[0] < w and 0 <= car[1] < h):
                continue
            if prev in walls or prev in obstacles:
                continue
            if car in walls or car in obstacles:
                continue
            if prev in reach:
                continue
            reach.add(prev)
            q.append(prev)
    return reach


def _wall_touches(wall, region):
    """墙的4邻是否触及 region 中的某格."""
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        if (wall[0] + dx, wall[1] + dy) in region:
            return True
    return False


def _find_unlock_centers(box, goal, walls, w, h, obstacles, bombs, car,
                         push_box_bfs, push_bomb_bfs, budget,
                         max_wins=None, max_confirm=None, centers_per_set=None):
    """
    为"直达推不动"的一对找解锁型引爆中心 (含空格).

    预筛(零 push_box_bfs):
      1. 正向前沿 Rf (箱可推到) + 反向前沿 Rb (可推达goal).
      2. 引爆中心 c: 3x3内可破坏墙集 dset, 且 dset 同时触及 Rf 和 Rb (是"割墙").
      3. 按墙集去重, 按 (墙集大小, 离box-goal连线距离) 排序.
    确认(消耗预算):
      4. top 序逐个跑带R2的 push_box_bfs(炸后墙态); 胜出≤_UNLOCK_WINS_PER_PAIR 即停.
      5. 对胜出墙集, 选一个"炸弹可推达"的引爆中心(空格用push_box_bfs判, 墙格用push_bomb_bfs判).

    返回 ([{wall(=引爆中心), blast_set, box_steps, bomb}, ...], 消耗的push_box_bfs次数)
    """
    # 前沿BFS的障碍: 排除当前箱(它要动)+所有炸弹(炸弹会被推走腾路),
    # 与下游 push_box_bfs 的 post_obs 语义一致, 否则箱被自身/炸弹困住致前沿退化.
    _max_wins = _UNLOCK_WINS_PER_PAIR if max_wins is None else max_wins
    _max_confirm = _UNLOCK_CONFIRM_PER_PAIR if max_confirm is None else max_confirm
    _per_set = _UNLOCK_CENTERS_PER_SET if centers_per_set is None else centers_per_set
    frontier_obs = (obstacles - walls) - {box} - set(bombs)
    Rf = _fwd_push_reachable(box, walls, w, h, frontier_obs)
    Rb = _bwd_push_reachable(goal, walls, w, h, frontier_obs)
    if goal in Rf:
        return [], 0  # 其实直达可达, 不该走到这

    # 收集"割墙集" -> 候选引爆中心列表
    centers = {}
    for cx in range(w):
        for cy in range(h):
            c = (cx, cy)
            blast = get_walls_in_3x3(c, walls)
            dset = frozenset(wl for wl in blast if not is_boundary_wall(wl, w, h))
            if not dset:
                continue
            tf = any(_wall_touches(wl, Rf) for wl in dset)
            tb = any(_wall_touches(wl, Rb) for wl in dset)
            if tf and tb:
                centers.setdefault(dset, []).append(c)

    if not centers:
        return [], 0

    # 排序: 优先小墙集(溅射少, 更可能不破坏别处) + 中心靠近 box-goal 连线
    ranked = sorted(
        centers.items(),
        key=lambda kv: (len(kv[0]),
                        min(manhattan_distance(c, box) + manhattan_distance(c, goal)
                            for c in kv[1])))

    post_obs = (obstacles - walls) - {box} - set(bombs)
    results = []
    calls = 0
    for dset, clist in ranked:
        if calls >= _max_confirm or calls >= budget:
            break
        tw = walls - set(dset)
        calls += 1
        r = push_box_bfs(box, goal, tw, w, h, post_obs, initial_car_pos=car)
        if r is None:
            continue
        box_steps = r[0]
        # 在该墙集的候选中心里挑炸弹可推达的引爆点 (空格优先 - 这是关键缺失维度)
        chosen = []
        for c in clist:
            for bp in bombs:
                if c in walls:
                    rr = push_bomb_bfs(bp, c, walls, w, h,
                                       (obstacles - walls) - {bp}, car_start=car)
                else:
                    rr = push_box_bfs(bp, c, walls, w, h,
                                      (obstacles - walls) - {bp}, initial_car_pos=car)
                if rr is not None:
                    chosen.append((c, bp))
                    break
            if len(chosen) >= _per_set:
                break
        if not chosen:
            continue
        for c, bp in chosen:
            results.append({
                'wall': c, 'blast_set': dset,
                'box_steps': box_steps, 'bomb': bp,
            })
        if len({tuple(sorted(d['blast_set'])) for d in results}) >= _max_wins:
            break

    return results, calls


def find_unlock_centers_wide(map_data, box, goal):
    """为单对 (box,goal) 计算"放宽版"解锁型引爆中心.

    与步3.5预筛(_preplan_unlock_blasts)用同一套双向前沿割算法, 但放宽截断:
      - max_wins/confirm/centers_per_set 都放大, 让那些"墙集偏大但可能是唯一可调度解"
        的引爆中心(在常规预筛里被 _UNLOCK_WINS_PER_PAIR=3 截掉)也进入候选.
    仅在"默认方案 schedule 失败"的修复路径里调用(见 refine_blast_points 尾部),
    故不影响 20 张已 OK 图的常规预筛预算/排序.

    返回 [{'wall':引爆中心, 'blast_set':墙集, 'box_steps':, 'bomb':}, ...] (已按墙集大小排序).
    """
    from algorithms.path_search import push_box_bfs, push_bomb_bfs
    w, h = map_data['width'], map_data['height']
    walls = map_data['walls']
    boxes = map_data['boxes']
    bombs = list(map_data['bombs'])
    car = map_data['car']
    if not bombs:
        return []
    all_obs = walls | set(boxes) | set(bombs)
    res, _ = _find_unlock_centers(
        box, goal, walls, w, h, all_obs, bombs, car,
        push_box_bfs, push_bomb_bfs, budget=60,
        max_wins=12, max_confirm=40, centers_per_set=3)
    return res


def _preplan_unlock_blasts(map_data, connectivity):
    """
    步3.5 解锁型炸点主入口. 对每个"直达 push_box_bfs 推不动"的 (box,goal) 对,
    用双向前沿割预筛产出 ≤3-5 个高质量解锁引爆中心(可为空格), 全图总 push_box_bfs <_UNLOCK_TOTAL_BUDGET.

    返回 unlock_centers: [{'wall':引爆中心, 'blast_set':墙集, 'box':, 'goal':, 'box_steps':, 'bomb':}, ...]
    引爆中心即作 assignment 的候选墙(_compute_bomb_candidates 用它跑炸弹/推箱评分).
    """
    # 延迟 import 避免循环依赖 (path_search 不 import algorithms 模块, 安全)
    from algorithms.path_search import push_box_bfs, push_bomb_bfs

    w, h = map_data['width'], map_data['height']
    walls = map_data['walls']
    boxes = map_data['boxes']
    bombs = list(map_data['bombs'])
    goals = sorted(map_data['goals'])
    car = map_data['car']

    if not bombs:
        return []

    all_obs = walls | set(boxes) | set(bombs)

    unlock = []
    budget = _UNLOCK_TOTAL_BUDGET
    for box in boxes:
        for goal in goals:
            if budget <= 0:
                break
            # 直达预筛: 轻量正向前沿是否含 goal
            post_obs = (all_obs - walls) - {box} - set(bombs)
            Rf = _fwd_push_reachable(box, walls, w, h, post_obs)
            if goal in Rf:
                continue  # 直达可达, 不需解锁炸点
            res, used = _find_unlock_centers(
                box, goal, walls, w, h, all_obs, bombs, car,
                push_box_bfs, push_bomb_bfs, budget)
            budget -= used
            for d in res:
                d['box'] = box
                d['goal'] = goal
                unlock.append(d)
        if budget <= 0:
            break

    return unlock


# =============================================================================
# 内部函数
# =============================================================================

def _find_region_boundary_walls(
    walls: set, current_components: dict, width: int, height: int
) -> list:
    """找分隔当前连通分量的内部墙"""
    boundary = []
    for wall in walls:
        if is_boundary_wall(wall, width, height):
            continue
        neighbor_comps = set()
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            n = (wall[0] + dx, wall[1] + dy)
            if n in current_components:
                neighbor_comps.add(current_components[n])
        if len(neighbor_comps) >= 2:
            boundary.append(wall)
    return boundary


def _get_wall_neighbor_comps(wall, current_components) -> set:
    comps = set()
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        n = (wall[0] + dx, wall[1] + dy)
        if n in current_components:
            comps.add(current_components[n])
    return comps


def _evaluate_wall_benefit(
    wall: tuple, map_data: dict, current_components: dict,
) -> dict:
    """对一面候选墙计算受益向量（P0a 部分）"""
    w, h = map_data['width'], map_data['height']
    walls = map_data['walls']
    boxes = map_data['boxes']
    goals = map_data['goals']
    cars_goals = goals | {map_data['car']}

    # 模拟炸墙
    blast = get_walls_in_3x3(wall, walls)
    temp_walls = walls - blast

    # 炸后理想连通
    ideal_conn = analyze_connectivity(
        w, h, temp_walls,
        exempt_positions=set(boxes) | set(map_data['bombs']) | cars_goals
    )

    benefit = {'p0a': 0, 'p0b': 0, 'p0c': 0, 'p1a': 0, 'p1b': 0, 'p2': 0}

    # P0a: 炸后桥接完整配对链
    for reg in ideal_conn['regions']:
        b_in = sum(1 for b in boxes if b in reg)
        g_in = sum(1 for g in goals if g in reg)
        if b_in > 0 and g_in > 0:
            benefit['p0a'] += min(b_in, g_in)

    return benefit


def _find_walls_around(pos: tuple, walls: set, width: int, height: int) -> list:
    """找 pos 周围 4 邻域中的内部墙"""
    result = []
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        n = (pos[0] + dx, pos[1] + dy)
        if n in walls and not is_boundary_wall(n, width, height):
            result.append(n)
    return result


def _find_rescue_chain(
    target_wall: tuple,
    bombs: list,
    walls: set,
    boxes_set: set,
    deadlocked_bombs: list,
    width: int,
    height: int,
    depth: int = 1,
    max_depth: int = 3,
) -> Optional[list]:
    """递归找连锁救援链 [(bomb, wall), ...]"""
    if depth > max_depth:
        return None

    from core.distance import manhattan_distance

    dl_bomb_positions = {pos for _, pos, _ in deadlocked_bombs}
    all_obs = walls | boxes_set

    # 直接救援: 近距离 + 短路径 (曼哈顿≤8 且 BFS步数≤5)
    for bidx, bpos in enumerate(bombs):
        if bpos in dl_bomb_positions:
            continue
        if manhattan_distance(bpos, target_wall) > 8:
            continue
        cost = _bfs_push_bomb(bpos, target_wall, walls, width, height,
                              all_obs - {bpos})
        if cost is not None and cost <= 5:
            return [(bpos, target_wall)]

    # 连锁救援
    for bidx, bpos in enumerate(bombs):
        if bpos not in dl_bomb_positions:
            continue

        rescue_walls = _find_walls_around(bpos, walls, width, height)
        for rw in rescue_walls:
            if rw == target_wall:
                continue
            sub_chain = _find_rescue_chain(
                rw, bombs, walls, boxes_set, deadlocked_bombs,
                width, height, depth + 1, max_depth
            )
            if sub_chain:
                sim_walls = set(walls)
                for _, w in sub_chain:
                    sim_walls -= get_walls_in_3x3(w, sim_walls)
                cost = _bfs_push_bomb(bpos, target_wall, sim_walls, width, height,
                                      boxes_set - {bpos})
                if cost is not None:
                    return sub_chain + [(bpos, target_wall)]

    return None


def _find_region_adjacent_pairs(
    walls: set, ideal_regions: list, width: int, height: int
) -> list:
    """找理想分量间仅隔一面墙的相邻对"""
    region_of = {}
    for rid, reg in enumerate(ideal_regions):
        for pos in reg:
            region_of[pos] = rid

    pairs = []
    done = set()

    for wall in walls:
        if is_boundary_wall(wall, width, height):
            continue
        neighbor_regions = set()
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            n = (wall[0] + dx, wall[1] + dy)
            if n in region_of:
                neighbor_regions.add(region_of[n])
        if len(neighbor_regions) >= 2:
            nlist = sorted(neighbor_regions)
            for i in range(len(nlist)):
                for j in range(i + 1, len(nlist)):
                    key = (nlist[i], nlist[j])
                    if key not in done:
                        done.add(key)
                        pairs.append({'regions': key, 'wall': wall})

    return pairs
