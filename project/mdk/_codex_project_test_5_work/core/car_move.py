"""
车移动核心模块 - 可达路径计算

提供车从起点到终点的独立移动路径（不推动任何物体）
"""

from collections import deque
from typing import Optional


# 方向定义
DIRECTION_VECTORS = {
    'UP': (0, -1),
    'DOWN': (0, 1),
    'LEFT': (-1, 0),
    'RIGHT': (1, 0)
}

DIRECTION_NAMES = ['UP', 'DOWN', 'LEFT', 'RIGHT']
DIRECTION_OFFSETS = [(0, -1), (0, 1), (-1, 0), (1, 0)]


def get_neighbor(pos: tuple, direction: str) -> tuple:
    """
    获取相邻位置

    Args:
        pos: 当前位置
        direction: 方向 ('UP', 'DOWN', 'LEFT', 'RIGHT')

    Returns:
        tuple: 相邻位置坐标
    """
    dx, dy = DIRECTION_VECTORS[direction]
    return (pos[0] + dx, pos[1] + dy)


def bfs_car_path(
    car_start: tuple,
    car_end: tuple,
    walls: set,
    width: int,
    height: int,
    additional_obstacles: Optional[set] = None,
    exempt_positions: Optional[set] = None
) -> dict:
    """
    计算车从起点到终点的独立移动路径（不推动任何物体）

    游戏规则：
    - 车只能移动，不能推动物体
    - 车不能穿过墙体和其他障碍物
    - 爆炸后的区域（exempt_positions）即使原本是墙也可通行

    障碍物处理优先级：
    1. walls: 默认障碍物（墙体）
    2. additional_obstacles: 额外障碍物（箱子、其他物体）
    3. exempt_positions: 豁免位置（爆炸后的墙等，即使在walls中也视为可通行）

    Args:
        car_start: 车辆起始位置
        car_end: 车辆目标位置
        walls: 墙体坐标集合
        width: 地图宽度
        height: 地图高度
        additional_obstacles: 额外障碍物集合（可选）
            - 其他箱子：{(5,5), (7,7)}
            - 其他炸弹：{(3,3)}
        exempt_positions: 豁免位置集合（可选）
            - 例如：被炸弹炸毁的墙体 {(4,4), (4,5), (5,4)}

    Returns:
        dict: {
            'path': list,          # 移动路径 [('MOVE', 'UP'), ('MOVE', 'DOWN'), ...]
            'distance': int,       # 移动距离
            'success': bool       # 是否成功到达
        }
    """
    # 处理单点目标
    if isinstance(car_end, (set, list)):
        if car_end:
            first = list(car_end)[0] if isinstance(car_end, set) else car_end[0]
            if isinstance(first, tuple):
                car_end = first

    if car_start == car_end:
        return {'path': [], 'distance': 0, 'success': True}

    # 初始化障碍物集合
    obstacles = set(walls)
    if additional_obstacles:
        obstacles.update(additional_obstacles)

    # 豁免位置从障碍物中移除
    if exempt_positions:
        obstacles -= exempt_positions

    # BFS
    queue = deque([(car_start, [])])
    visited = {car_start}

    while queue:
        current, path = queue.popleft()

        for direction, (dx, dy) in zip(DIRECTION_NAMES, DIRECTION_OFFSETS):
            neighbor = (current[0] + dx, current[1] + dy)

            # 边界检查
            if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                continue

            # 障碍物检查（已排除豁免位置）
            if neighbor in obstacles:
                continue

            # 到达目标
            if neighbor == car_end:
                return {
                    'path': path + [('MOVE', direction)],
                    'distance': len(path) + 1,
                    'success': True
                }

            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [('MOVE', direction)]))

    return {'path': [], 'distance': float('inf'), 'success': False}


def bfs_car_reachable(
    car_pos: tuple,
    target_set: set,
    walls: set,
    width: int,
    height: int,
    additional_obstacles: Optional[set] = None,
    exempt_positions: Optional[set] = None
) -> dict:
    """
    计算车能否到达目标集合中的任意一个位置

    Args:
        car_pos: 车辆起始位置
        target_set: 目标位置集合
        walls: 墙体坐标集合
        width: 地图宽度
        height: 地图高度
        additional_obstacles: 额外障碍物集合
        exempt_positions: 豁免位置集合

    Returns:
        dict: {
            'reachable': bool,           # 是否可达
            'targets_reached': set,      # 实际到达的目标集合
            'distance': int or inf,     # 到最近目标的距离
            'path': list or None,       # 到最近目标的路径
            'target': tuple or None     # 到达的目标位置
        }
    """
    if not target_set:
        return {
            'reachable': False,
            'targets_reached': set(),
            'distance': float('inf'),
            'path': None,
            'target': None
        }

    # 处理单点目标
    if len(target_set) == 1:
        target = list(target_set)[0]
        result = bfs_car_path(car_pos, target, walls, width, height,
                              additional_obstacles, exempt_positions)
        return {
            'reachable': result['success'],
            'targets_reached': {target} if result['success'] else set(),
            'distance': result['distance'] if result['success'] else float('inf'),
            'path': result['path'] if result['success'] else None,
            'target': target if result['success'] else None
        }

    # 多目标BFS
    obstacles = set(walls)
    if additional_obstacles:
        obstacles.update(additional_obstacles)
    if exempt_positions:
        obstacles -= exempt_positions

    queue = deque([(car_pos, [], 0)])
    visited = {car_pos}
    targets_reached = set()

    while queue:
        current, path, dist = queue.popleft()

        # 检查是否在目标集合中
        if current in target_set:
            targets_reached.add(current)
            if len(targets_reached) == 1:
                # 返回第一个到达的目标
                return {
                    'reachable': True,
                    'targets_reached': targets_reached,
                    'distance': dist,
                    'path': path,
                    'target': current
                }

        for direction, (dx, dy) in zip(DIRECTION_NAMES, DIRECTION_OFFSETS):
            neighbor = (current[0] + dx, current[1] + dy)

            # 边界检查
            if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                continue

            # 障碍物检查
            if neighbor in obstacles:
                continue

            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [('MOVE', direction)], dist + 1))

    # 找到最近的到达目标
    return {
        'reachable': len(targets_reached) > 0,
        'targets_reached': targets_reached,
        'distance': float('inf'),
        'path': None,
        'target': None
    }


def bfs_car_to_push_position(
    car_pos: tuple,
    box_pos: tuple,
    push_direction: str,
    walls: set,
    width: int,
    height: int,
    additional_obstacles: Optional[set] = None
) -> dict:
    """
    计算车到达推动位置的最短路径

    推动位置 = 箱子位置的推动方向反方向一格
    例如：推动方向是 UP，推动位置是 box_pos 下方一格

    Args:
        car_pos: 车辆起始位置
        box_pos: 箱子位置
        push_direction: 推动方向 ('UP', 'DOWN', 'LEFT', 'RIGHT')
        walls: 墙体坐标集合
        width: 地图宽度
        height: 地图高度
        additional_obstacles: 额外障碍物集合

    Returns:
        dict: {
            'path': list,          # 移动路径
            'distance': int,       # 移动距离
            'push_position': tuple, # 推动位置
            'success': bool        # 是否成功到达
        }
    """
    # 计算推动位置（推动方向的反方向一格）
    opposite = {
        'UP': 'DOWN',
        'DOWN': 'UP',
        'LEFT': 'RIGHT',
        'RIGHT': 'LEFT'
    }
    push_pos = get_neighbor(box_pos, opposite[push_direction])

    # 检查推动位置是否有效
    if not (0 <= push_pos[0] < width and 0 <= push_pos[1] < height):
        return {
            'path': [],
            'distance': float('inf'),
            'push_position': push_pos,
            'success': False
        }

    # 推动位置不能是墙或障碍物
    obstacles = set(walls)
    if additional_obstacles:
        obstacles.update(additional_obstacles)

    if push_pos in obstacles:
        return {
            'path': [],
            'distance': float('inf'),
            'push_position': push_pos,
            'success': False
        }

    # BFS到推动位置
    result = bfs_car_path(car_pos, push_pos, walls, width, height,
                          additional_obstacles)

    return {
        'path': result['path'],
        'distance': result['distance'],
        'push_position': push_pos,
        'success': result['success']
    }
