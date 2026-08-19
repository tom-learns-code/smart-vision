"""
地图分析模块

提供地图相关的分析函数
"""

from typing import Optional, Set, Tuple
import random


def is_valid_coordinate(pos: tuple, width: int, height: int) -> bool:
    """
    检查坐标是否在地图范围内

    Args:
        pos: 坐标
        width: 地图宽度
        height: 地图高度

    Returns:
        bool: 是否有效
    """
    return 0 <= pos[0] < width and 0 <= pos[1] < height


def is_wall(pos: tuple, walls: set) -> bool:
    """
    检查位置是否为墙体

    Args:
        pos: 坐标
        walls: 墙体集合

    Returns:
        bool: 是否为墙体
    """
    return pos in walls


def is_boundary_wall(pos: tuple, width: int, height: int) -> bool:
    """
    判断墙体是否为边界墙

    边界墙：位于地图边缘的墙体，通常不能被推入

    Args:
        pos: 墙体位置
        width: 地图宽度
        height: 地图高度

    Returns:
        bool: 是否为边界墙
    """
    return pos[0] == 0 or pos[0] == width - 1 or pos[1] == 0 or pos[1] == height - 1


def is_valid_position(
    pos: tuple,
    walls: set,
    width: int,
    height: int,
    extra_obstacles: Optional[set] = None
) -> bool:
    """
    综合检查位置是否有效

    Args:
        pos: 坐标
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        extra_obstacles: 额外障碍物集合

    Returns:
        bool: 是否有效（不在任何障碍物中且在范围内）
    """
    if not is_valid_coordinate(pos, width, height):
        return False

    if pos in walls:
        return False

    if extra_obstacles and pos in extra_obstacles:
        return False

    return True


def get_walls_in_3x3(center_pos: tuple, walls: set) -> set:
    """
    获取3x3爆炸范围内的所有墙体

    Args:
        center_pos: 中心墙体位置
        walls: 所有墙体集合

    Returns:
        set: 在3x3范围内的墙体坐标集合
    """
    destroyed = set()

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            target = (center_pos[0] + dx, center_pos[1] + dy)
            if target in walls:
                destroyed.add(target)

    return destroyed


def get_walls_in_rect(
    pos1: tuple,
    pos2: tuple,
    walls: set,
    expand: int = 0
) -> set:
    """
    获取两点确定的矩形区域内的所有墙体

    Args:
        pos1: 位置1
        pos2: 位置2
        walls: 墙体集合
        expand: 扩展范围

    Returns:
        set: 矩形区域内的墙体
    """
    min_x = min(pos1[0], pos2[0]) - expand
    max_x = max(pos1[0], pos2[0]) + expand
    min_y = min(pos1[1], pos2[1]) - expand
    max_y = max(pos1[1], pos2[1]) + expand

    rect_walls = set()
    for wall in walls:
        if min_x <= wall[0] <= max_x and min_y <= wall[1] <= max_y:
            rect_walls.add(wall)

    return rect_walls


def get_walls_in_3x3_with_check(center_pos: tuple, walls: set,
                                 width: int, height: int) -> set:
    """
    获取3x3范围内的墙体，并考虑边界

    Args:
        center_pos: 中心位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度

    Returns:
        set: 3x3范围内的墙体
    """
    destroyed = set()

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            target = (center_pos[0] + dx, center_pos[1] + dy)
            if target in walls:
                destroyed.add(target)

    return destroyed


def find_wall_boundaries(
    wall_pos: tuple,
    direction: str,
    walls: set,
    map_width: int,
    map_height: int
) -> tuple:
    """
    搜索墙体的边界（上/下/左/右方向）

    Args:
        wall_pos: 墙体起始位置
        direction: 搜索方向 ('up', 'down', 'left', 'right')
        walls: 墙体集合
        map_width: 地图宽度
        map_height: 地图高度

    Returns:
        tuple: (start_bound, end_bound) 边界范围
    """
    x, y = wall_pos

    if direction == 'up':
        bound = y
        while bound >= 0:
            if (x, bound) in walls:
                bound -= 1
            else:
                break
        return (bound + 1, y)

    elif direction == 'down':
        bound = y
        while bound < map_height:
            if (x, bound) in walls:
                bound += 1
            else:
                break
        return (y, bound - 1)

    elif direction == 'left':
        bound = x
        while bound >= 0:
            if (bound, y) in walls:
                bound -= 1
            else:
                break
        return (bound + 1, x)

    elif direction == 'right':
        bound = x
        while bound < map_width:
            if (bound, y) in walls:
                bound += 1
            else:
                break
        return (x, bound - 1)

    return (0, 0)


def is_corner_deadlock(pos: tuple, walls: set) -> bool:
    """
    检查位置是否为墙角死锁

    墙角死锁：位于角落且相邻两边都是墙

    Args:
        pos: 位置
        walls: 墙体集合

    Returns:
        bool: 是否为墙角死锁
    """
    x, y = pos

    # 检查四个角落
    corners = [
        (x - 1, y) in walls and (x, y - 1) in walls,  # 左上角
        (x + 1, y) in walls and (x, y - 1) in walls,  # 右上角
        (x - 1, y) in walls and (x, y + 1) in walls,  # 左下角
        (x + 1, y) in walls and (x, y + 1) in walls,   # 右下角
    ]

    return any(corners)


def count_adjacent_walls(pos: tuple, walls: set) -> int:
    """
    计算相邻墙体的数量

    Args:
        pos: 位置
        walls: 墙体集合

    Returns:
        int: 相邻墙体数量（0-4）
    """
    x, y = pos
    count = 0

    if (x - 1, y) in walls:
        count += 1
    if (x + 1, y) in walls:
        count += 1
    if (x, y - 1) in walls:
        count += 1
    if (x, y + 1) in walls:
        count += 1

    return count


def get_free_neighbors(pos: tuple, walls: set, width: int, height: int,
                      obstacles: Optional[set] = None) -> list:
    """
    获取所有可通行的相邻位置

    Args:
        pos: 当前位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        obstacles: 额外障碍物集合

    Returns:
        list: 可通行的相邻位置列表
    """
    from .direction import DIRECTION_NAMES, get_neighbor

    neighbors = []
    for direction in DIRECTION_NAMES:
        neighbor = get_neighbor(pos, direction)

        if not is_valid_coordinate(neighbor, width, height):
            continue

        if neighbor in walls:
            continue

        if obstacles and neighbor in obstacles:
            continue

        neighbors.append(neighbor)

    return neighbors


def get_internal_walls(walls: set, width: int, height: int) -> set:
    """
    获取所有内部墙体（非边界墙）

    Args:
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度

    Returns:
        set: 内部墙体集合
    """
    internal = set()
    for wall in walls:
        if not is_boundary_wall(wall, width, height):
            internal.add(wall)
    return internal


def compute_narrow_cells(walls: set, w: int, h: int) -> set:
    """
    预计算窄道格: 两侧被墙夹住的单格宽通道。

    窄道判定: 水平方向两侧为墙（或边界），或垂直方向两侧为墙（或边界）

    Args:
        walls: 墙体坐标集合
        w:     地图宽度
        h:     地图高度

    Returns:
        窄道格坐标集合
    """
    narrow = set()
    for x in range(w):
        for y in range(h):
            if (x, y) in walls:
                continue
            h_walled = ((x-1, y) in walls or x == 0) and ((x+1, y) in walls or x == w-1)
            v_walled = ((x, y-1) in walls or y == 0) and ((x, y+1) in walls or y == h-1)
            if h_walled or v_walled:
                narrow.add((x, y))
    return narrow


def get_blast_candidates(walls: set, width: int, height: int) -> set:
    """
    获取所有可以作为炸点的墙体

    条件：不是边界墙

    Args:
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度

    Returns:
        set: 可作为炸点的墙体集合
    """
    candidates = set()
    for wall in walls:
        if not is_boundary_wall(wall, width, height):
            candidates.add(wall)
    return candidates


# =============================================================================
# 连通分量分析
# =============================================================================

def analyze_connectivity(
    width: int,
    height: int,
    obstacles: set,
    exempt_positions: set = None,
    start_pos: tuple = None,
):
    """
    BFS 连通分量分析

    所有格子默认视为障碍物，exempt_positions 中的格子强制通行。

    内部使用 1D 数组索引 `idx = y * width + x`，便于移植到 C。

    Args:
        width: 地图宽度
        height: 地图高度
        obstacles: 障碍物坐标集合
        exempt_positions: 豁免坐标集合（即使属于 obstacles 也视为可走）
        start_pos: 可选，若传入则额外返回该点所在分量

    Returns:
        {
            'components': dict,       # {(x,y): component_id}
            'component_count': int,   # 分量总数
            'is_single': bool,        # 全图是否单一分量
            'regions': list,          # 每个分量的点集 [{...}, {...}]
            'start_component': int,   # start_pos 所在分量 ID（若传入）
            'start_region': set,      # start_pos 所在分量的点集（若传入）
        }
    """
    total = width * height
    exempt = exempt_positions if exempt_positions else set()
    skip_start = start_pos is not None

    visited = [False] * total         # C: uint8 visited[192]
    component_id = [0] * total        # C: uint8 comp_id[192]
    queue = [0] * total               # C: uint16 queue[192]
    direction_offsets = (-width, width, -1, 1)  # 上下左右对应 1D 偏移

    regions = []
    start_comp = -1

    def pos_to_idx(x, y):
        return y * width + x

    def idx_to_pos(idx):
        return idx % width, idx // width

    for sy in range(height):
        for sx in range(width):
            idx = pos_to_idx(sx, sy)

            if visited[idx]:
                continue

            # 是否可走：豁免格总是可走，否则不在障碍物中才可走
            pos = (sx, sy)
            walkable = (pos in exempt) or (pos not in obstacles)
            if not walkable:
                visited[idx] = True
                continue

            # BFS 标记当前分量
            comp_id = len(regions)
            region = set()
            head, tail = 0, 0
            queue[tail] = idx
            tail += 1
            visited[idx] = True

            while head < tail:
                cur = queue[head]
                head += 1
                component_id[cur] = comp_id
                pos_cur = (cur % width, cur // width)
                region.add(pos_cur)

                for off in direction_offsets:
                    nxt = cur + off
                    # 水平移动需要检查是否越列（左移 -1 不能跨越行）
                    if off == -1 and cur % width == 0:
                        continue
                    if off == 1 and cur % width == width - 1:
                        continue
                    if nxt < 0 or nxt >= total:
                        continue
                    if visited[nxt]:
                        continue

                    nx, ny = nxt % width, nxt // width
                    pos_nxt = (nx, ny)
                    walkable_nxt = (pos_nxt in exempt) or (pos_nxt not in obstacles)
                    if walkable_nxt:
                        visited[nxt] = True
                        queue[tail] = nxt
                        tail += 1

            regions.append(region)
            if start_pos is not None and start_pos in region:
                start_comp = comp_id

    # 构建 (x,y) → component_id 映射（按需，小开销）
    components = {}
    for y in range(height):
        for x in range(width):
            idx = pos_to_idx(x, y)
            comp = component_id[idx]
            components[(x, y)] = comp

    result = {
        'components': components,
        'component_count': len(regions),
        'is_single': len(regions) == 1,
        'regions': regions,
    }
    if skip_start:
        if start_comp >= 0:
            result['start_component'] = start_comp
            result['start_region'] = regions[start_comp]
        else:
            result['start_component'] = -1
            result['start_region'] = set()
    return result


# =============================================================================
# 地图扫描
# =============================================================================

_MAP_SYMBOLS = {
    '#': 'wall',
    '@': 'car',
    '$': 'box',
    '.': 'goal',
    '*': 'bomb',
    '+': 'car_on_goal',
    '-': 'empty',
    ' ': 'empty',
}


def scan_map(map_data: dict) -> dict:
    """
    扫描地图，提取所有元素的位置和数量

    Args:
        map_data: 原始地图数据字典，至少包含:
            - walls: 墙体集合
            - boxes_start: 箱子位置列表
            - goals: 目标点集合
            - car_start: 车起始位置
            - bombs: 炸弹位置列表
            - width: 地图宽度
            - height: 地图高度

    Returns:
        dict: {
            'walls': set,         # 所有墙体位置 {(x,y), ...}
            'boxes': list,        # 箱子位置 [(x,y), ...]
            'goals': set,         # 目标点位置 {(x,y), ...}
            'bombs': list,        # 炸弹位置 [(x,y), ...]
            'car': tuple,         # 车起始位置 (x,y)
            'walkable': set,      # 所有非墙格子
            'counts': {
                'walls': int,
                'boxes': int,
                'goals': int,
                'bombs': int,
            },
            'width': int,
            'height': int,
        }
    """
    walls = set(map_data['walls'])
    boxes = list(map_data['boxes_start'])
    goals = set(map_data['goals'])
    bombs = list(map_data.get('bombs', []))
    car = map_data['car_start']
    width = map_data['width']
    height = map_data['height']

    # 可走区域 = 整个地图 - 墙
    walkable = set()
    for x in range(width):
        for y in range(height):
            if (x, y) not in walls:
                walkable.add((x, y))

    return {
        'walls': walls,
        'boxes': boxes,
        'goals': goals,
        'bombs': bombs,
        'car': car,
        'walkable': walkable,
        'counts': {
            'walls': len(walls),
            'boxes': len(boxes),
            'goals': len(goals),
            'bombs': len(bombs),
        },
        'width': width,
        'height': height,
    }


def load_map_from_file(filepath: str) -> dict:
    """
    从文件加载地图并返回 map_data

    Args:
        filepath: 地图文件路径

    Returns:
        dict: map_data 字典
    """
    import os

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"地图文件不存在: {filepath}")

    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    walls = set()
    boxes = []
    goals = set()
    bombs = []
    car = None
    width = 0
    height = len(lines)

    for y, line in enumerate(lines):
        line = line.rstrip('\n\r')
        width = max(width, len(line))
        for x, ch in enumerate(line):
            if ch == '#':
                walls.add((x, y))
            elif ch == '$':
                boxes.append((x, y))
            elif ch == '.':
                goals.add((x, y))
            elif ch == '*':
                bombs.append((x, y))
            elif ch == '@':
                car = (x, y)
            elif ch == '+':
                car = (x, y)
                goals.add((x, y))

    if car is None:
        raise ValueError("地图中未找到车的位置 (@)")

    return {
        'walls': walls,
        'boxes_start': boxes,
        'goals': goals,
        'bombs': bombs,
        'car_start': car,
        'width': width,
        'height': height,
    }


# =============================================================================
# ID 随机分配（模拟比赛模式②③中的图片识别）
# =============================================================================

def assign_random_ids(n_boxes: int, n_goals: int, digit_pool_size: int = 10):
    """
    为箱子和目标点随机分配 label_id。

    从 0~digit_pool_size-1 中随机抽取 n 个数字作为共享 ID 池，
    箱子和目标点各自从中独立随机排列。保证一一配对可能，
    但每次加载分配顺序可能不同。

    Args:
        n_boxes: 箱子数量
        n_goals: 目标点数量（应与 n_boxes 相同）
        digit_pool_size: 数字池大小，默认 10（0-9）
    Returns:
        (box_ids, goal_ids): 两个字典 {index: label_id}
    """
    if n_boxes == 0 or n_goals == 0:
        return {}, {}

    n = max(n_boxes, n_goals)
    # 从 0~digit_pool_size-1 随机抽 n 个不重复数字
    pool = random.sample(range(digit_pool_size), min(n, digit_pool_size))
    # 确保池大小至少为 n（如果 digit_pool_size < n，循环采样）
    while len(pool) < n:
        pool.append(random.randint(0, digit_pool_size - 1))

    # 箱子和目标点各取 n 个，各自独立 shuffle
    box_labels = pool[:n_boxes]
    goal_labels = pool[:n_goals]
    random.shuffle(box_labels)
    random.shuffle(goal_labels)

    box_ids = {i: box_labels[i] for i in range(n_boxes)}
    goal_ids = {i: goal_labels[i] for i in range(n_goals)}

    return box_ids, goal_ids
