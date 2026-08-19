"""
路径分析与死锁检测模块

包含:
  - compare_connectivity: 理想 vs 当前连通分量对比 (步骤 2b)
  - analyze_channels: 管道/窄口分析与死锁检测 (步骤 3)
"""

from typing import List, Dict, Tuple, Optional, Set
from collections import deque

from core.map_analysis import analyze_connectivity
from core.bfs import bfs_shortest_path


# =============================================================================
# 步骤 2b: 连通分量对比分析
# =============================================================================

def compare_connectivity(map_data: dict) -> dict:
    """
    对比理想连通（仅墙）与当前连通（墙+箱+弹），
    找出阻塞点、区域失衡、元素归属。

    Args:
        map_data: scan_map 输出

    Returns:
        {
            'ideal':          {...},   # 理想连通结果
            'current':        {...},   # 当前连通结果
            'element_regions':{...},   # 每元素所属理想分量 ID
            'is_blocked':     bool,
            'blockers':       [{pos, type, blocks_regions}, ...],
            'region_imbalance':[{region_id, boxes_count, goals_count}, ...],
        }
    """
    cars = {map_data['car']}
    walls = map_data['walls']
    boxes_set = set(map_data['boxes'])
    bombs_set = set(map_data['bombs'])
    goals = map_data['goals']
    w, h = map_data['width'], map_data['height']

    # 1. 理想连通（仅墙为障碍，其他都豁免）
    ideal = analyze_connectivity(
        w, h, obstacles=walls,
        exempt_positions=boxes_set | bombs_set | goals | cars
    )

    # 2. 当前连通（墙+箱+弹为障碍，车+目标豁免）
    all_obstacles = walls | boxes_set | bombs_set
    current = analyze_connectivity(
        w, h, obstacles=all_obstacles,
        exempt_positions=goals | cars
    )

    # 3. 每个元素所在理想分量 ID
    ideal_comp = ideal['components']
    element_regions = {
        'car': ideal_comp.get(map_data['car'], -1),
        'boxes': [ideal_comp.get(b, -1) for b in map_data['boxes']],
        'goals': [ideal_comp.get(g, -1) for g in sorted(goals)],
        'bombs': [ideal_comp.get(b, -1) for b in map_data['bombs']],
    }

    # 4. 找阻塞点: 哪些箱子/炸弹在当前连通中造成了区域分割
    blockers = []
    is_blocked = not current['is_single']

    if is_blocked:
        # 模拟移除每个箱子/炸弹后区域是否合并
        for i, box in enumerate(map_data['boxes']):
            temp_obs = all_obstacles - {box}
            temp_conn = analyze_connectivity(w, h, temp_obs, exempt_positions=goals | cars)
            if temp_conn['component_count'] < current['component_count']:
                freed = _find_freed_regions(current, temp_conn)
                blockers.append({
                    'pos': box,
                    'type': 'box',
                    'index': i,
                    'blocks_regions': freed,
                })

        for i, bomb in enumerate(map_data['bombs']):
            temp_obs = all_obstacles - {bomb}
            temp_conn = analyze_connectivity(w, h, temp_obs, exempt_positions=goals | cars)
            if temp_conn['component_count'] < current['component_count']:
                freed = _find_freed_regions(current, temp_conn)
                blockers.append({
                    'pos': bomb,
                    'type': 'bomb',
                    'index': i,
                    'blocks_regions': freed,
                })

    # 5. 区域失衡（理想分量中箱 vs 目标数）
    imbalance = []
    for reg_id in range(ideal['component_count']):
        region = ideal['regions'][reg_id]
        boxes_in = sum(1 for b in map_data['boxes'] if b in region)
        goals_in = sum(1 for g in goals if g in region)
        imbalance.append({
            'region_id': reg_id,
            'size': len(region),
            'boxes_count': boxes_in,
            'goals_count': goals_in,
        })

    return {
        'ideal': {
            'regions': ideal['regions'],
            'component_count': ideal['component_count'],
            'is_single': ideal['is_single'],
        },
        'current': {
            'regions': current['regions'],
            'component_count': current['component_count'],
            'is_single': current['is_single'],
        },
        'element_regions': element_regions,
        'is_blocked': is_blocked,
        'blockers': blockers,
        'region_imbalance': imbalance,
    }


def _find_freed_regions(before: dict, after: dict) -> list:
    """找出移除某个障碍物后被"释放"（合并）的区域 ID"""
    before_sizes = {i: len(r) for i, r in enumerate(before['regions'])}
    after_sizes = {i: len(r) for i, r in enumerate(after['regions'])}
    freed = []
    for bid, bsize in before_sizes.items():
        matched = False
        for aid, asize in after_sizes.items():
            if bsize == asize:
                matched = True
                break
        if not matched:
            freed.append(bid)
    return freed


# =============================================================================
# 步骤 3: 管道/窄口分析
# =============================================================================

def _classify_grid_cell(pos: tuple, walkable: set) -> str:
    """
    对单个 walkable 格分类

    Returns:
        'dead' | 'dead_end' | 'channel' | 'corner' | 'open'
    """
    x, y = pos
    neighbors = []
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        n = (x + dx, y + dy)
        if n in walkable:
            neighbors.append(n)

    cnt = len(neighbors)
    if cnt == 0:
        return 'dead'
    if cnt == 1:
        return 'dead_end'

    if cnt == 2:
        d0 = (neighbors[0][0] - x, neighbors[0][1] - y)
        d1 = (neighbors[1][0] - x, neighbors[1][1] - y)
        # 方向相反 → 管道格；方向垂直 → L 转角
        if d0[0] + d1[0] == 0 and d0[1] + d1[1] == 0:
            return 'channel'
        else:
            return 'corner'

    return 'open'


def _segment_channel(pos: tuple, grid_types: dict, visited_channels: set) -> dict:
    """
    从 pos 出发 BFS 连接相邻 channel/corner 格，返回一个通道段。

    Returns:
        {tiles, orientation, endpoints, length}
    """
    queue = deque([pos])
    tiles = []
    visited_channels.add(pos)

    while queue:
        cur = queue.popleft()
        tiles.append(cur)
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nb = (cur[0] + dx, cur[1] + dy)
            if nb in visited_channels:
                continue
            t = grid_types.get(nb)
            if t in ('channel', 'corner'):
                visited_channels.add(nb)
                queue.append(nb)

    # 判走向
    if len(tiles) >= 2:
        dx = tiles[-1][0] - tiles[0][0]
        dy = tiles[-1][1] - tiles[0][1]
        orientation = 'h' if abs(dx) >= abs(dy) else 'v'
    else:
        orientation = 'h'

    # 按走向排序找端点
    if orientation == 'h':
        tiles.sort(key=lambda t: t[0])
    else:
        tiles.sort(key=lambda t: t[1])

    endpoints = [tiles[0], tiles[-1]]

    return {
        'tiles': tiles,
        'orientation': orientation,
        'endpoints': endpoints,
        'length': len(tiles),
    }


def _analyze_exit_state(endpoint: tuple, grid_types: dict, walkable: set, walls: set) -> dict:
    """
    分析管道出口状态

    核心：看端点邻接的"开阔区格子"本身有多少 walkable 邻居，
    如果开阔格子自身有 ≥3 方向开放 → free

    Returns:
        {'state': 'free'|'constrained'|'deadlock', 'reason': str}
    """
    x, y = endpoint
    open_adj = []      # 邻接的 walkable 格子（非墙）
    wall_adj = []      # 邻接的墙

    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        n = (x + dx, y + dy)
        if n in walkable:
            open_adj.append((n, (dx, dy)))
        elif n in walls:
            wall_adj.append((n, (dx, dy)))

    open_cnt = len(open_adj)
    wall_cnt = len(wall_adj)

    if open_cnt == 0:
        return {'state': 'deadlock', 'reason': '出口完全被封'}

    # 检查连接的是否是真正开阔区
    # — 看连接格在 walkable 中的邻居数
    truly_free = False
    for n, _ in open_adj:
        nx, ny = n
        free_neighbors = 0
        for ndx, ndy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            if (nx + ndx, ny + ndy) in walkable:
                free_neighbors += 1
        if free_neighbors >= 3:
            truly_free = True
            break

    if truly_free and open_cnt >= 2:
        return {'state': 'free', 'reason': ''}

    if wall_cnt >= 2:
        for wn, _ in wall_adj:
            wx, wy = wn
            corner_walls = 0
            for cdx, cdy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                if (wx + cdx, wy + cdy) in walls:
                    corner_walls += 1
            if corner_walls >= 3:
                return {'state': 'deadlock', 'reason': '出口侧墙角围堵'}
        return {'state': 'constrained', 'reason': '出口贴墙'}

    if open_cnt >= 3:
        return {'state': 'free', 'reason': ''}

    return {'state': 'constrained', 'reason': '出口受限'}


def _has_L_corner(channel: dict, grid_types: dict) -> bool:
    """检测管道内是否有 90° 转弯 — 通过检查相邻管道格的走向是否一致"""
    tiles = channel['tiles']
    if len(tiles) < 3:
        return False

    prev_dir = None
    for i in range(1, len(tiles)):
        dx = abs(tiles[i][0] - tiles[i-1][0])
        dy = abs(tiles[i][1] - tiles[i-1][1])
        cur_dir = 'h' if dx > dy else 'v'
        if prev_dir and cur_dir != prev_dir:
            return True
        prev_dir = cur_dir
    return False


def analyze_channels(map_data: dict, ideal_regions: list) -> dict:
    """
    对理想连通分量内部做管道/窄口分析 (步骤 3)

    Args:
        map_data: scan_map 输出
        ideal_regions: 步骤 2b ideal['regions']

    Returns:
        {
            'grid_types':       {(x,y): type},
            'channels':         [{tiles, orientation, endpoints, length,
                                  exit_states: [{state, reason},...],
                                  has_L_corner: bool,
                                  passable: bool}, ...],
            'sub_regions':      [{tiles, parent_region}, ...],
            'passable_pairs':   [(a,b,bool), ...],
            'deadlocked_boxes': [(idx, pos, [type...]), ...],
            'deadlocked_bombs': [(idx, pos, [type...]), ...],
        }
    """
    w, h = map_data['width'], map_data['height']
    walls = map_data['walls']
    walkable = map_data['walkable']
    boxes_set = set(map_data['boxes'])
    bombs_set = set(map_data['bombs'])

    # 3a. 全图格子分类
    grid_types = {}
    for pos in walkable:
        grid_types[pos] = _classify_grid_cell(pos, walkable)

    # 3b. 管道分段（过滤长度 < 2 的噪音）
    visited_channels = set()
    channels = []
    for pos, t in grid_types.items():
        if t in ('channel', 'corner') and pos not in visited_channels:
            ch = _segment_channel(pos, grid_types, visited_channels)
            if ch['length'] >= 2:
                channels.append(ch)

    # 3c. 管道死锁分析
    for ch in channels:
        exit_states = []
        for ep in ch['endpoints']:
            es = _analyze_exit_state(ep, grid_types, walkable, walls)
            exit_states.append(es)

        has_l = _has_L_corner(ch, grid_types)

        passable = True
        if has_l:
            passable = False
        for es in exit_states:
            if es['state'] == 'deadlock':
                passable = False

        ch['exit_states'] = exit_states
        ch['has_L_corner'] = has_l
        ch['passable'] = passable

    # 3d. 子区域分割（从 open 格子出发 BFS，不走 channel/corner）
    blocked = set()
    for ch in channels:
        blocked.update(ch['tiles'])

    sub_regions = []
    visited_sub = set()

    for pos in walkable:
        if pos in visited_sub or pos in blocked:
            continue
        if grid_types.get(pos) not in ('open', 'dead_end', 'dead'):
            continue

        # BFS 子区域
        queue = deque([pos])
        region = set()
        while queue:
            cur = queue.popleft()
            if cur in visited_sub:
                continue
            visited_sub.add(cur)
            region.add(cur)
            for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                nb = (cur[0] + dx, cur[1] + dy)
                if nb in visited_sub or nb in blocked:
                    continue
                if nb in walkable:
                    t = grid_types.get(nb, 'open')
                    if t in ('open', 'dead_end', 'dead'):
                        queue.append(nb)

        if region:
            parent_region = None
            for rid, reg in enumerate(ideal_regions):
                if pos in reg:
                    parent_region = rid
                    break
            sub_regions.append({
                'tiles': region,
                'parent_region': parent_region,
            })

    # 子区域间可达性（相邻子区域通过管道连接）
    passable_pairs = []
    for ch in channels:
        ep_sub_regions = []
        for ep in ch['endpoints']:
            for sid, sr in enumerate(sub_regions):
                for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                    adj = (ep[0] + dx, ep[1] + dy)
                    if adj in sr['tiles']:
                        ep_sub_regions.append(sid)
                        break
        if len(ep_sub_regions) == 2:
            a, b = ep_sub_regions[0], ep_sub_regions[1]
            passable_pairs.append((a, b, ch['passable']))

    # 3e. 死锁检测（静态 + 车可达性）

    boxes_list = map_data['boxes']
    bombs_list = map_data['bombs']
    all_obj = boxes_set | bombs_set
    car = map_data['car']

    deadlocked_boxes = []
    for idx, box in enumerate(boxes_list):
        other = (all_obj - {box})
        dl_types = _check_static_deadlock(box, walls, w, h, other)
        if not dl_types:
            # 静态不死锁 → 查车能否到达推方向（死锁 #12）
            if not _car_can_reach_any_push(box, car, walls, w, h, other):
                dl_types = ['car_unreachable']
        if dl_types:
            deadlocked_boxes.append((idx, box, dl_types))

    deadlocked_bombs = []
    for idx, bomb in enumerate(bombs_list):
        other = (all_obj - {bomb})
        dl_types = _check_static_deadlock(bomb, walls, w, h, other)
        if not dl_types:
            if not _car_can_reach_any_push(bomb, car, walls, w, h, other):
                dl_types = ['car_unreachable']
        if dl_types:
            deadlocked_bombs.append((idx, bomb, dl_types))

    # 3f. 目标死锁检测
    deadlocked_goals = []
    for idx, goal in enumerate(sorted(map_data['goals'])):
        dl_types = _check_static_deadlock(goal, walls, w, h, all_obj)
        if dl_types:
            deadlocked_goals.append((idx, goal, dl_types))

    return {
        'grid_types': grid_types,
        'channels': channels,
        'sub_regions': sub_regions,
        'passable_pairs': passable_pairs,
        'deadlocked_boxes': deadlocked_boxes,
        'deadlocked_bombs': deadlocked_bombs,
        'deadlocked_goals': deadlocked_goals,
    }


# =============================================================================
# 静态死锁检测
# =============================================================================

def _check_static_deadlock(pos: tuple, walls: set, width: int, height: int,
                           other_objects: set) -> list:
    """
    对单个位置做 5 类静态死锁检测 (死锁 #1-#5)

    Returns: 命中的死锁类型名列表
    """
    x, y = pos
    results = []
    obstacles = walls | other_objects

    # #1 角落死锁：两个垂直邻格都是墙/物
    if (((x - 1, y) in obstacles and (x, y - 1) in obstacles) or
        ((x + 1, y) in obstacles and (x, y - 1) in obstacles) or
        ((x - 1, y) in obstacles and (x, y + 1) in obstacles) or
        ((x + 1, y) in obstacles and (x, y + 1) in obstacles)):
        results.append('corner')

    # #2 边界死锁
    if x == 0 or x == width - 1 or y == 0 or y == height - 1:
        results.append('boundary')

    # #3 三面堵死
    blocked = 0
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        if (x + dx, y + dy) in obstacles:
            blocked += 1
    if blocked >= 3:
        results.append('three_side_blocked')

    # #4 点位围堵（4 方向各查 1-2 格）
    tight_count = 0
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        d1 = (x + dx, y + dy) in obstacles
        d2 = (x + dx * 2, y + dy * 2) in obstacles
        if d1 or d2:
            tight_count += 1
    if tight_count == 4:
        results.append('tight_surround')

    # #5 隧道死角：walkable 邻居 ≤ 1（排除障碍物后）
    free_neighbors = 0
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        n = (x + dx, y + dy)
        if n not in obstacles and 0 <= n[0] < width and 0 <= n[1] < height:
            free_neighbors += 1
    if free_neighbors <= 1:
        results.append('tunnel_dead_end')

    return results


def _car_can_reach_any_push(
    obj_pos: tuple, car_pos: tuple, walls: set,
    width: int, height: int, obstacles: set
) -> bool:
    """
    检查车能否到达 obj 的任一推方向后方位置（死锁 #12）

    推动 obj 向 dir 需要车在 (obj - dir) → 车 BFS
    """
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        # 推方向后方 = 车需要站的位置
        car_needed = (obj_pos[0] - dx, obj_pos[1] - dy)
        if not (0 <= car_needed[0] < width and 0 <= car_needed[1] < height):
            continue
        if car_needed in walls or car_needed in obstacles:
            continue
        # 轻量 BFS: 车→car_needed
        if bfs_shortest_path(car_pos, car_needed, walls | obstacles, width, height) is not None:
            return True
    return False


