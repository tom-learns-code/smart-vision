"""
箱子-目标分配模块 (步骤 4)

分级仲裁 + 多候选炸弹评分 + 全局枚举搜索
"""

from typing import List, Dict, Tuple, Optional, Set
from itertools import permutations

from core.distance import manhattan_distance
from core.map_analysis import get_walls_in_3x3, get_walls_in_rect, is_boundary_wall
from core.bfs import bfs_shortest_path


# =============================================================================
# 推箱/推弹 BFS
# =============================================================================

def _bfs_push_box(box, goal, walls, w, h, obstacles, max_steps=200):
    if box == goal:
        return 0, 0
    from collections import deque
    queue = deque([(box, 0, None, 0)])
    visited = {box}
    while queue:
        pos, steps, prev_dir, turns = queue.popleft()
        if steps > max_steps:
            continue
        for dx, dy, dname in [(0,-1,'UP'),(0,1,'DOWN'),(-1,0,'LEFT'),(1,0,'RIGHT')]:
            nb = (pos[0]+dx, pos[1]+dy)
            cp = (pos[0]-dx, pos[1]-dy)
            if not (0 <= cp[0] < w and 0 <= cp[1] < h):
                continue
            if cp in walls or cp in obstacles:
                continue
            nt = turns + (1 if prev_dir and prev_dir != dname else 0)
            if nb == goal:
                return steps + 1, nt
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in walls or nb in obstacles:
                continue
            if nb in visited:
                continue
            visited.add(nb)
            queue.append((nb, steps+1, dname, nt))
    return None


def _bfs_push_bomb(bomb, wall, walls, w, h, obstacles, max_steps=200, car_start=None):
    """推弹搜索: 含车位置+车间移动, 两级优先 (步数优先, 等步数选车移动少).

    返回 (steps, turns) — car_cost 内化在路径选择中, 不暴露给评分.
    """
    import heapq
    # 堆 key: (push_steps, car_total) — 两级优先, 先最小化推弹步数
    # car_total = sum(manhattan between consecutive car positions)
    heap = [((0, 0), 0, 0, bomb, None, None)]
    best = {}

    while heap:
        (p_steps, car_total), steps, turns, pos, car_pos, prev_dir = heapq.heappop(heap)
        key = (pos, car_pos)
        if key in best:
            prev = best[key]
            if prev[0] < p_steps or (prev[0] == p_steps and prev[1] <= car_total):
                continue
        best[key] = (p_steps, car_total)

        if steps > max_steps:
            continue
        for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
            n = (pos[0]+dx, pos[1]+dy)
            cp = (pos[0]-dx, pos[1]-dy)
            if not (0 <= cp[0] < w and 0 <= cp[1] < h):
                continue
            if cp in walls or cp in obstacles:
                continue
            new_turns = turns + (1 if prev_dir and prev_dir != (dx, dy) else 0)
            if car_pos is not None:
                car_move = abs(car_pos[0]-cp[0]) + abs(car_pos[1]-cp[1])
            elif car_start is not None:
                car_move = abs(car_start[0]-cp[0]) + abs(car_start[1]-cp[1])
            else:
                car_move = 0
            if n == wall:
                return steps + 1, new_turns
            if not (0 <= n[0] < w and 0 <= n[1] < h):
                continue
            if n in walls or n in obstacles:
                continue
            heapq.heappush(heap, ((steps+1, car_total + car_move),
                                  steps+1, new_turns, n, cp, (dx, dy)))
    return None


# =============================================================================
# 长墙惩罚
# =============================================================================

def _calculate_wall_penalty(box, goal, walls, w, h):
    bx, by = box; gx, gy = goal
    dx, dy = gx-bx, gy-by; total = 0
    check = []
    if dx < 0: check.append(('right', 1, 0))
    if dy < 0: check.append(('down', 0, 1))
    if dx > 0: check.append(('left', -1, 0))
    if dy > 0: check.append(('up', 0, -1))
    for _, wdx, wdy in check:
        adj = (bx+wdx, by+wdy)
        if adj not in walls: continue
        if wdx != 0:
            col = bx+wdx; mid = (gy+by)/2
            lo = hi = by
            y = by-1
            while y >= 0 and (col, y) in walls: lo = y; y -= 1
            y = by+1
            while y < h and (col, y) in walls: hi = y; y += 1
            if lo <= by <= hi:
                x = min(abs(mid-lo), abs(mid-hi))
                ld = (col-wdx, lo) in walls
                hd = (col-wdx, hi) in walls
                if ld and hd: continue
                elif ld: x = abs(mid-hi)
                elif hd: x = abs(mid-lo)
                import math; total += (math.ceil(x)+1)*2+5
        else:
            row = by+wdy; mid = (gx+bx)/2
            lo = hi = bx
            x = bx-1
            while x >= 0 and (x, row) in walls: lo = x; x -= 1
            x = bx+1
            while x < w and (x, row) in walls: hi = x; x += 1
            if lo <= bx <= hi:
                x = min(abs(mid-lo), abs(mid-hi))
                ld = (lo, row-wdy) in walls
                hd = (hi, row-wdy) in walls
                if ld and hd: continue
                elif ld: x = abs(mid-hi)
                elif hd: x = abs(mid-lo)
                import math; total += (math.ceil(x)+1)*2+5
    return total


# =============================================================================
# 评分
# =============================================================================

def _compute_pair_score(box, goal, walls, w, h, obstacles, same_region, car_start=None):
    manh = manhattan_distance(box, goal)
    result = _bfs_push_box(box, goal, walls, w, h, obstacles)
    if result is None:
        return {'reachable': False, 'path_steps': 0, 'turns': 0, 'manhattan': manh, 'score': -1}
    steps, turns = result
    wp = _calculate_wall_penalty(box, goal, walls, w, h)
    score = 1000 + (200 if same_region else 0) - steps*10 - turns*30 - manh*1 - wp*8
    return {'reachable': True, 'path_steps': steps, 'turns': turns, 'manhattan': manh, 'wall_penalty': wp, 'score': score}


# =============================================================================
# 多炸弹接力搜索
# =============================================================================

# 炸弹候选缓存 (box, goal, frozenset(bombs)) -> candidates
_seq_blast_cache = {}

def _search_sequential_blasts(box, goal, walls, w, h, obstacles, bombs,
                               car_start, dl_bomb_positions, walls_to_check,
                               max_depth=3):
    """
    DFS 搜索多炸弹接力清除方案。
    """
    # 缓存检查
    cache_key = (box, goal, frozenset(bombs))
    if cache_key in _seq_blast_cache:
        return _seq_blast_cache[cache_key]

    results = []
    active_bombs = list(bombs)
    # 限制搜索深度: 2面墙通常足够打通路径
    effective_depth = min(max_depth, 3)
    # 限制walls_to_check到路径附近的墙
    path_walls = [wl for wl in walls_to_check
                  if manhattan_distance(wl, box) + manhattan_distance(wl, goal) <=
                     manhattan_distance(box, goal) + 6]
    if not path_walls:
        path_walls = list(walls_to_check)

    def _bomb_reachable(bpos, cur_walls, cur_obs, cur_car):
        """检查炸弹的任一推方向是否可被车到达"""
        for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
            push_pos = (bpos[0] - dx, bpos[1] - dy)
            if not (0 <= push_pos[0] < w and 0 <= push_pos[1] < h):
                continue
            if push_pos in cur_walls or push_pos in cur_obs:
                continue
            if bfs_shortest_path(cur_car, push_pos, cur_walls | cur_obs, w, h) is not None:
                return True
        return False

    def _dfs(cur_walls, cur_obs, available_bombs, car_pos, chain, depth):
        nonlocal results
        if len(results) >= 2:  # 已找到足够的链方案, 停止深搜
            return

        # 1. 检查 box→goal 是否已可达
        box_result = _bfs_push_box(box, goal, cur_walls, w, h, cur_obs - {box})
        if box_result is not None:
            box_steps, box_turns = box_result
            total_bomb_steps = sum(s for _, _, s, _ in chain)
            total_bomb_turns = sum(t for _, _, _, t in chain)
            cd = len(chain)
            score = 1000 - box_steps*6 - box_turns*30 - total_bomb_steps*8 - total_bomb_turns*20 - cd*250
            if chain:
                last_bomb, last_wall, _, _ = chain[-1]
            else:
                last_bomb, last_wall = None, None
            results.append({
                'wall': last_wall, 'bomb': last_bomb,
                'bomb_steps': total_bomb_steps, 'bomb_turns': total_bomb_turns,
                'box_steps': box_steps, 'box_turns': box_turns,
                'chain_depth': max(cd, 1),
                'score': score,
                'chain': [(b, w) for b, w, _, _ in chain] if chain else None,
            })
            return  # 找到链就停止, 不继续深搜
        if depth >= max_depth:
            return

        # 2. 找当前可达的炸弹
        reachable = []
        for bpos in available_bombs:
            if bpos in cur_obs:
                if _bomb_reachable(bpos, cur_walls, cur_obs, car_pos):
                    reachable.append(bpos)

        if not reachable:
            return

        # 3. 剪枝: 每层最多试 3 面墙, 2个炸弹
        # 首层(depth=0)用全量墙: 需要远处墙来"解锁"新炸弹
        wall_pool = walls_to_check if depth == 0 else path_walls
        walls_priority = sorted(
            [wl for wl in wall_pool if wl in cur_walls],
            key=lambda wl: manhattan_distance(wl, car_pos) if depth == 0
            else manhattan_distance(wl, box) + manhattan_distance(wl, goal))
        walls_to_try = walls_priority[:8] if depth == 0 else walls_priority[:6]

        tried = 0
        for bpos in reachable[:2]:
            if tried >= 3:
                break
            for wall in walls_to_try:
                if tried >= 3:
                    break
                if wall not in cur_walls:
                    continue

                bomb_result = _bfs_push_bomb(
                    bpos, wall, cur_walls, w, h,
                    (cur_obs - cur_walls) - {bpos},
                    car_start=car_pos)
                if bomb_result is None:
                    continue

                tried += 1
                bomb_steps, bomb_turns = bomb_result
                dx = wall[0] - bpos[0]; dy = wall[1] - bpos[1]
                if abs(dx) >= abs(dy):
                    pdir = (1 if dx > 0 else -1, 0)
                else:
                    pdir = (0, 1 if dy > 0 else -1)
                new_car = (wall[0] - pdir[0], wall[1] - pdir[1])
                if not (0 <= new_car[0] < w and 0 <= new_car[1] < h):
                    new_car = car_pos

                blast = get_walls_in_3x3(wall, cur_walls)
                new_walls = cur_walls - blast
                new_obs = (cur_obs - cur_walls) - {bpos}
                new_available = [b for b in available_bombs if b != bpos]

                _dfs(new_walls, new_obs, new_available, new_car,
                     chain + [(bpos, wall, bomb_steps, bomb_turns)], depth + 1)

    # 初始障碍物(非墙): 箱+弹, 排除当前箱
    initial_obs = (obstacles - walls) - {box}
    _dfs(set(walls), initial_obs, list(active_bombs), car_start, [], 0)

    # 去重+排序
    seen = set()
    unique = []
    for c in results:
        key = (c['box_steps'], c['bomb_steps'], c['chain_depth'])
        if key not in seen:
            seen.add(key)
            unique.append(c)
    unique.sort(key=lambda c: (-c['score'], c['bomb_steps'] + c['box_steps']))
    result = unique[:3]
    # 缓存
    _seq_blast_cache[cache_key] = result
    return result


# 炸弹候选缓存 (box, goal, frozenset(available_bombs)) -> candidates
_bomb_candidate_cache = {}

def _compute_bomb_candidates(box, goal, walls, w, h, blast_candidates, bombs,
                             obstacles, rescue_chains, dl_bomb_positions,
                             car_start=None, extra_walls=None, other_boxes=None):
    """返回候选列表, car_start 传入时含车移动代价.

    extra_walls: 解锁型炸点引爆中心(可为空格), 仅本对(box,goal)专用,
    不污染其他对. 用于"直达推不动"对的精准炸点解锁.
    other_boxes: 其他对的箱(R2重验时排除, 与 generate_pair_plans 障碍语义一致).
    """
    # 检查缓存 (extra_walls/other_boxes 进 key, 避免跨对/跨图缓存污染)
    ew_key = frozenset(extra_walls) if extra_walls else frozenset()
    ob_key = frozenset(other_boxes) if other_boxes else frozenset()
    cache_key = (box, goal, frozenset(b for b in bombs if b not in (dl_bomb_positions or set())), ew_key, ob_key)
    if cache_key in _bomb_candidate_cache:
        return _bomb_candidate_cache[cache_key]

    cands = []
    seen = set()

    walls_to_check = set()
    for c in blast_candidates:
        walls_to_check.add(c['wall'])
    rect = get_walls_in_rect(box, goal, walls, expand=2)
    for rw in rect:
        if not is_boundary_wall(rw, w, h):
            walls_to_check.add(rw)

    # 软炸弹可推走, 不挡箱路
    soft_bombs_local = set(bombs) - (dl_bomb_positions or set())

    def _eval_centers(center_set):
        """对一组引爆中心(可含空格)评分, 追加到 cands."""
        for wall in center_set:
            if wall in seen:
                continue
            seen.add(wall)
            blast = get_walls_in_3x3(wall, walls)
            tw = walls - blast
            for bpos in bombs:
                if dl_bomb_positions and bpos in dl_bomb_positions:
                    continue
                bomb_result = _bfs_push_bomb(bpos, wall, walls, w, h, (obstacles-walls)-{bpos},
                                              car_start=car_start)
                if bomb_result is None:
                    continue
                bomb_steps, bomb_turns = bomb_result
                post_obs = (obstacles-walls) - {box} - soft_bombs_local
                result = _bfs_push_box(box, goal, tw, w, h, post_obs)
                if result is None:
                    continue
                box_steps, box_turns = result
                score = 1000 - box_steps*6 - box_turns*30 - bomb_steps*8 - bomb_turns*20 - manhattan_distance(wall, goal)*3
                cands.append({'wall': wall, 'bomb': bpos, 'bomb_steps': bomb_steps,
                              'bomb_turns': bomb_turns,
                              'box_steps': box_steps, 'box_turns': box_turns,
                              'chain_depth': 1, 'score': score})

    _eval_centers(walls_to_check)

    if rescue_chains:
        for rc in rescue_chains:
            cb = {b for b,_ in rc['sequence']}
            cw_list = [w for _,w in rc['sequence']]
            if not cw_list: continue
            # 计算链总推弹步数
            chain_push = 0
            chain_turns = 0
            sw = set(walls); so = (obstacles-walls) - soft_bombs_local
            for cbomb, cwall in rc['sequence']:
                result = _bfs_push_bomb(cbomb, cwall, sw, w, h, so-{cbomb})
                if result is None: chain_push = -1; break
                bomb_st, bomb_t = result
                chain_push += bomb_st
                chain_turns += bomb_t
                sw -= get_walls_in_3x3(cwall, sw)
                so.discard(cbomb)
            if chain_push < 0: continue
            result = _bfs_push_box(box, goal, sw, w, h, so-{box})
            if result is None: continue
            bs, bt = result; cd = rc['chain_depth']
            cands.append({'wall': cw_list[-1], 'bomb': list(cb)[-1] if cb else None,
                          'bomb_steps': chain_push, 'bomb_turns': chain_turns,
                          'box_steps': bs, 'box_turns': bt,
                          'chain_depth': cd, 'score': 1000-bs*6-bt*30-cd*300,
                          'chain': rc['sequence']})

    # 单炸弹和救援链都不够 → 尝试多炸弹接力清除
    if not cands and walls_to_check and car_start is not None:
        seq_chains = _search_sequential_blasts(
            box, goal, walls, w, h, obstacles, bombs,
            car_start, dl_bomb_positions, walls_to_check)
        cands.extend(seq_chains)

    cands.sort(key=lambda c: (-c['score'], c['bomb_steps'] + c['box_steps']))
    result = cands[:3]
    _bomb_candidate_cache[cache_key] = result
    return result


# =============================================================================
# 主分配
# =============================================================================

def solve_assignment(map_data, connectivity, channel_analysis, blast_preplan):
    walls = map_data['walls']; boxes = map_data['boxes']
    goals_list = sorted(map_data['goals']); bombs = map_data['bombs']
    w, h = map_data['width'], map_data['height']

    # 清空炸弹候选缓存: 其 key=(box,goal,frozenset(bombs)) 不含墙体,
    # 跨地图(相同坐标不同墙布局)会命中错误缓存致下游执行错乱(批量求解非确定性bug)。
    # 缓存仅用于单次求解内去重, 每次入口清空既正确又零额外开销。
    _seq_blast_cache.clear()
    _bomb_candidate_cache.clear()

    el = connectivity['element_regions']
    blc = blast_preplan['blast_candidates']
    unlock_by_pair = blast_preplan.get('unlock_by_pair', {})
    rcs = blast_preplan.get('rescue_chains', [])
    dl_bp = {pos for _,pos,_ in channel_analysis.get('deadlocked_bombs', [])}
    dl_gp = {pos for _,pos,_ in channel_analysis.get('deadlocked_goals', [])}
    soft_bombs = set(bombs) - dl_bp  # 非死锁弹可推走

    perm_obs = walls | set(boxes) | dl_bp  # 永久障碍
    all_obs = walls | set(boxes) | set(bombs)

    # 预计算每对分数
    pair_candidates = {}
    for bi, box in enumerate(boxes):
        for gi, goal in enumerate(goals_list):
            sr = (el['boxes'][bi] == el['goals'][gi])
            direct = _compute_pair_score(box, goal, walls, w, h, perm_obs-{box}, sr,
                                          car_start=map_data['car'])
            gdl = goal in dl_gp
            if gdl and direct['reachable']:
                direct['score'] -= 800; direct['goal_deadlocked'] = True
            bc_list = None
            # 本对专用的解锁型引爆中心(含空格), 仅供该对消费
            ew = [d['wall'] for d in unlock_by_pair.get((box, goal), [])]
            ob = set(boxes) - {box}  # 其他箱(R2重验时排除, 与步6障碍语义一致)
            if not direct['reachable'] or gdl:
                bc_list = _compute_bomb_candidates(box, goal, walls, w, h, blc, bombs, all_obs, rcs, dl_bp,
                                                    car_start=map_data['car'], extra_walls=ew, other_boxes=ob)
            elif direct['path_steps'] > direct['manhattan'] + 15:
                bc_list = _compute_bomb_candidates(box, goal, walls, w, h, blc, bombs, all_obs, rcs, dl_bp,
                                                    car_start=map_data['car'], extra_walls=ew, other_boxes=ob)
            pair_candidates[(bi, gi)] = {
                'box_idx': bi, 'goal_idx': gi, 'box': box, 'goal': goal,
                'same_region': sr, 'direct': direct, 'bomb_candidates': bc_list or [],
            }

    # 全局搜索 (分级仲裁 + 炸弹穷举分配)
    all_allocs = []  # [(vec, alloc_dict), ...] 收集所有候选
    n = len(boxes)

    for perm in permutations(range(n)):
        for mask in range(1 << n):
            unsolvable = 0

            # ---- 第一遍: 分离直达对和炸弹对 ----
            # direct_info: [(bi, gi, ps), ...]
            # bomb_info:  [(bi, gi, ps, options), ...]
            #   options: [(bomb_frozenset, candidate_dict), ...]
            direct_info = []
            bomb_info = []
            tot_direct = {'box': 0, 'turns': 0, 'score': 0}

            for bi, gi in enumerate(perm):
                ps = pair_candidates[(bi, gi)]
                use_bomb = bool(mask & (1 << bi))

                if not use_bomb:
                    if not ps['direct']['reachable']:
                        unsolvable += 1
                        tot_direct['score'] -= 10000
                    else:
                        tot_direct['box'] += ps['direct']['path_steps']
                        tot_direct['turns'] += ps['direct']['turns']
                        tot_direct['score'] += ps['direct']['score']
                    direct_info.append((bi, gi))
                else:
                    options = []
                    for bc in ps['bomb_candidates']:
                        if bc.get('chain'):
                            bs = frozenset(b for b, _ in bc['chain'])
                        else:
                            bs = frozenset([bc['bomb']])
                        options.append((bs, bc))
                    if options:
                        bomb_info.append((bi, gi, ps, options))
                    else:
                        unsolvable += 1
                        tot_direct['score'] -= 5000

            # ---- 第二遍: 穷举炸弹分配 ----
            from itertools import product

            best_bomb_score = None
            best_bomb_choice = None  # [(bs, bc), ...] 与 bomb_info 对齐

            if bomb_info:
                all_opts = [bi[3] for bi in bomb_info]  # 每个炸弹对的options列表
                for choices in product(*all_opts):
                    used = set()
                    valid = True
                    bt = {'bomb': 0, 'box': 0, 'turns': 0, 'score': 0}

                    for bs, bc in choices:
                        if bs & used:
                            valid = False
                            break
                        used |= bs
                        bt['bomb'] += bc['bomb_steps']
                        bt['box'] += bc['box_steps']
                        bt['turns'] += bc.get('box_turns', 0) + bc.get('bomb_turns', 0)
                        bt['score'] += bc['score']

                    if not valid:
                        continue

                    total_s = tot_direct['score'] + bt['score']
                    if best_bomb_score is None or total_s > best_bomb_score:
                        best_bomb_score = total_s
                        best_bomb_choice = choices
                        best_bomb_totals = bt
                        best_bomb_used = used

            # ---- 第三遍: 构建输出 ----
            if bomb_info and best_bomb_choice is None:
                unsolvable += len(bomb_info)

            pairs = []
            totals = {'bomb': 0, 'box': tot_direct['box'], 'turns': tot_direct['turns'],
                      'score': tot_direct['score']}
            used_bombs = set()

            # 炸弹分配映射
            bomb_assign = {}
            if best_bomb_choice:
                for idx, (bs, bc) in enumerate(best_bomb_choice):
                    bi, gi, ps, _ = bomb_info[idx]
                    bomb_assign[(bi, gi)] = bc
                    used_bombs |= bs
                totals['bomb'] += best_bomb_totals['bomb']
                totals['box'] += best_bomb_totals['box']
                totals['turns'] += best_bomb_totals['turns']
                totals['score'] += best_bomb_totals['score']

            for bi, gi in enumerate(perm):
                ps = pair_candidates[(bi, gi)]
                use_bomb = bool(mask & (1 << bi))

                if not use_bomb:
                    if ps['direct']['reachable']:
                        pairs.append({**ps, 'uses_bomb': False, 'bomb_plan': None})
                    else:
                        pairs.append({**ps, 'uses_bomb': False, 'bomb_plan': None})
                else:
                    bc = bomb_assign.get((bi, gi))
                    if bc:
                        pairs.append({**ps, 'uses_bomb': True, 'bomb_plan': bc})
                    else:
                        pairs.append({**ps, 'uses_bomb': True, 'bomb_plan': None, 'needs_chain': True})

            unused = len(bombs) - len(used_bombs)
            vec = (1, -unsolvable, -(totals['bomb']+totals['box']), -totals['turns'], unused, totals['score'])
            # 深拷贝: 每对的direct字段也是嵌套dict, 需要独立拷贝
            copied_pairs = []
            for p in pairs:
                cp = dict(p)
                if 'direct' in cp and isinstance(cp['direct'], dict):
                    cp['direct'] = dict(cp['direct'])
                if 'bomb_plan' in cp and cp['bomb_plan'] is not None:
                    cp['bomb_plan'] = dict(cp['bomb_plan'])
                copied_pairs.append(cp)
            all_allocs.append((vec, {
                'pairs': copied_pairs,
                'used_bombs': list(used_bombs), 'totals': dict(totals)
            }))

    if not all_allocs:
        return {'pairs': [], 'unused_bombs': bombs, 'unused_goals': set(goals_list),
                'summary': {'total_bombs_used': 0, 'is_solvable': False, 'needs_chain_rescue': True, 'total_score': -99999}}

    # 按评分排序
    all_allocs.sort(key=lambda x: x[0], reverse=True)

    # === 合并墙态可行性检查: top-5逐个尝试 ===
    # 改动1(方案A): 回验改用带R2车可达性的 push_box_bfs(与步6 generate_pair_plans 同源),
    # 不再用浅层 _bfs_push_box(不验车站位可达), 并传 initial_car_pos=map_data['car'] 验首段车可达。
    # 障碍语义与基线一致(其他箱仍当障碍, 让路由 scheduler + 下游 chain rescue 兜底),
    # 仅升级验证精度, 确保零回归。豁免其他箱/路径互斥留给改动2。
    from core.map_analysis import get_walls_in_3x3
    from algorithms.path_search import push_box_bfs, push_bomb_bfs

    car0 = map_data['car']

    best_alloc = None
    for _, alloc in all_allocs[:5]:
        # 1. 模拟所有炸弹爆炸的合并墙态
        merged_walls = set(walls)
        for p in alloc['pairs']:
            bp = p.get('bomb_plan')
            if not bp:
                continue
            chain = bp.get('chain')
            if chain:
                for cbomb, cwall in chain:
                    merged_walls -= get_walls_in_3x3(cwall, merged_walls)
            elif bp.get('wall'):
                merged_walls -= get_walls_in_3x3(bp['wall'], merged_walls)

        # 2. 仅重算"无炸弹/不可达"对(不碰已有炸弹的对)
        all_ok = True
        for p in alloc['pairs']:
            has_bomb = (p.get('uses_bomb') and p.get('bomb_plan') is not None)
            if has_bomb:
                # === 炸弹对R2回验 + 候选换挡(救 map3(4)/5(4)/8) ===
                # 病根: 全局搜索按浅层_bfs_push_box评分选中"R2幻影"炸点(如map3(4)的(10,8)),
                #   下游 generate_pair_plans 带R2 push_box_bfs 失败. 这里在选定allocation后,
                #   对炸弹对用R2回验其炸后推箱; 若失败, 从该对候选列表里换一个R2可行的炸点
                #   (优先同炸弹, 保持全局炸弹互斥不变). 仅在此层换挡, 不扰动全局搜索 → 防回归.
                bpn = p['bomb_plan']
                if bpn.get('chain'):
                    continue  # 链方案交下游处理
                cur_wall = bpn.get('wall')
                cur_bomb = bpn.get('bomb')
                if cur_wall is None:
                    continue
                pbox, pgoal = p['box'], p['goal']
                # R2回验障碍语义必须与 generate_pair_plans 单炸弹分支严格一致:
                #   post_obs = (all_obs-walls) - {box} - used_bombs - (other_boxes)
                # 即: 排除本箱+所有其他箱(让路scheduler兜底)+本对所用炸弹, 但"未使用的炸弹"仍是障碍!
                #   (此前误删全部炸弹 → R2过乐观, 选中被未用炸弹挡死的炸点, 下游box_tasks=0)
                # used_alloc_bombs: 本 allocation 已分配的所有炸弹(会被推走→不挡路)
                used_alloc_bombs = set()
                for q in alloc['pairs']:
                    qbp = q.get('bomb_plan')
                    if not qbp:
                        continue
                    if qbp.get('chain'):
                        for cb, _ in qbp['chain']:
                            used_alloc_bombs.add(cb)
                    elif qbp.get('bomb'):
                        used_alloc_bombs.add(qbp['bomb'])
                other_boxes_set = set(boxes) - {pbox}

                def _bomb_pair_r2(wall, use_bomb):
                    # 该炸点所用炸弹(use_bomb)会被推走→从障碍移除; 其余未用炸弹保留为障碍
                    free_bombs = (used_alloc_bombs | {use_bomb})
                    r2obs = (all_obs - walls) - {pbox} - free_bombs - other_boxes_set
                    tw = merged_walls - get_walls_in_3x3(wall, merged_walls)
                    return push_box_bfs(pbox, pgoal, tw, w, h, r2obs,
                                        initial_car_pos=car0) is not None
                if _bomb_pair_r2(cur_wall, cur_bomb):
                    continue  # 当前炸点R2可行, 保留
                # 换挡: 先在常规候选里找R2可行的同炸弹炸点(分高优先, 保持全局炸弹互斥)
                alt = None
                for c in sorted(p.get('bomb_candidates', []),
                                key=lambda c: -c.get('score', 0)):
                    if c.get('chain') or c.get('bomb') != cur_bomb:
                        continue
                    if c['wall'] == cur_wall:
                        continue
                    if _bomb_pair_r2(c['wall'], cur_bomb):
                        alt = {'bomb': cur_bomb, 'wall': c['wall'],
                               'bomb_steps': c.get('bomb_steps', 0),
                               'bomb_turns': c.get('bomb_turns', 0),
                               'chain_depth': 1}
                        break
                # 仍无解 → 用解锁型引爆中心(含空格): 炸弹推到空格引爆,3x3溅射炸斜对角墙.
                #   引爆中心已在 blast_select 经R2确认; 这里需确认"该中心炸弹能推达"且"R2推箱可行".
                if alt is None:
                    for uc in unlock_by_pair.get((pbox, pgoal), []):
                        center = uc['wall']
                        ucbomb = uc.get('bomb')
                        if not _bomb_pair_r2(center, ucbomb):
                            continue
                        # 推弹/推箱到引爆中心: 墙格用 push_bomb_bfs, 空格用 push_box_bfs
                        if center in walls:
                            rr = push_bomb_bfs(ucbomb, center, walls, w, h,
                                               (all_obs - walls) - {ucbomb}, car_start=car0)
                        else:
                            rr = push_box_bfs(ucbomb, center, walls, w, h,
                                              (all_obs - walls) - {ucbomb}, initial_car_pos=car0)
                        if rr is None:
                            continue
                        alt = {'bomb': ucbomb, 'wall': center,
                               'bomb_steps': rr[0], 'bomb_turns': 0, 'chain_depth': 1}
                        break
                if alt is not None:
                    p['bomb_plan'] = alt
                else:
                    all_ok = False
                continue  # 炸弹对处理完毕

            needs = p.get('needs_chain', False)
            ps = p.get('direct', {}).get('path_steps', 0) if 'direct' in p else p.get('path_steps', 0)
            if needs or ps <= 0:
                # 合并墙态下推箱(所有炸弹已消失), 带R2车可达性 + 首段车可达
                result = push_box_bfs(p['box'], p['goal'], merged_walls, w, h,
                                      (all_obs - walls) - {p['box']} - set(bombs),
                                      initial_car_pos=car0)
                if result is not None:
                    p['needs_chain'] = False
                    p['uses_bomb'] = False
                    p['bomb_plan'] = None
                    p['_merged_walls'] = merged_walls
                    if 'direct' in p and isinstance(p['direct'], dict):
                        p['direct']['reachable'] = True
                        p['direct']['path_steps'] = result[0]
                        p['direct']['turns'] = result[1]
                        p['direct']['score'] = 1000 - result[0]*10 - result[1]*30
                    else:
                        p['path_steps'] = result[0]
                        p['score'] = 1000 - result[0]*10 - result[1]*30
                else:
                    all_ok = False

        if all_ok:
            best_alloc = alloc
            break

    if best_alloc is None:
        # 无全可达候选, 回退原最优
        best_alloc = all_allocs[0][1]

    final_pairs = []
    for p in best_alloc['pairs']:
        bp = p.get('bomb_plan'); use = p.get('uses_bomb', False)
        entry = {'box': p['box'], 'goal': p['goal'], 'same_region': p['same_region'],
                 'uses_bomb': use, 'bomb_plan': None,
                 'needs_chain': p.get('needs_chain', False),
                 'score': (bp.get('score', 0) if use and bp else p['direct']['score']),
                 'path_steps': (bp.get('box_steps', 0) if use and bp else p['direct']['path_steps'])}
        if p.get('_merged_walls'):
            entry['_merged_walls'] = p['_merged_walls']
        if use and bp:
            entry['bomb_plan'] = {'bomb': bp.get('bomb'), 'wall': bp.get('wall'),
                                  'bomb_steps': bp.get('bomb_steps', 0),
                                  'bomb_turns': bp.get('bomb_turns', 0),
                                  'chain_depth': bp.get('chain_depth', 1)}
            if bp.get('chain'): entry['bomb_plan']['chain'] = bp['chain']
        final_pairs.append(entry)

    ubs = set(best_alloc['used_bombs']); unused = [b for b in bombs if b not in ubs]
    has_chain = any(p.get('needs_chain', False) for p in final_pairs)
    t = best_alloc['totals']

    return {'pairs': final_pairs, 'unused_bombs': unused, 'unused_goals': set(),
            'summary': {'total_bombs_used': len(best_alloc['used_bombs']),
                        'is_solvable': not has_chain, 'needs_chain_rescue': has_chain,
                        'total_score': t['score'], 'total_bomb_steps': t['bomb'],
                        'total_box_steps': t['box'], 'total_turns': t['turns']}}
