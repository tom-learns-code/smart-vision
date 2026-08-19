"""
长墙惩罚模块

计算因箱子后方长墙体导致的附加距离（"解救"成本）
"""

import math
from typing import Tuple, Dict, Optional

from .direction import DIRECTION_NAMES, DIRECTION_OFFSETS
from .map_analysis import is_wall, find_wall_boundaries


def determine_back_direction(box_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> list:
    """
    确定需要检查的后方方向

    后方 = 目标方向的相反方向

    Args:
        box_pos: 箱子位置 (c, d)
        goal_pos: 目标位置 (a, b)

    Returns:
        list: 需要检查的方向 [('right', 1, 0), ('down', 0, 1), ...]
    """
    dx = goal_pos[0] - box_pos[0]
    dy = goal_pos[1] - box_pos[1]

    directions = []
    if dx < 0:  # 目标在左，检查右侧墙
        directions.append(('right', 1, 0))
    if dy < 0:  # 目标在上，检查下侧墙
        directions.append(('down', 0, 1))
    if dx > 0:  # 目标在右，检查左侧墙
        directions.append(('left', -1, 0))
    if dy > 0:  # 目标在下，检查上侧墙
        directions.append(('up', 0, -1))

    return directions


def check_deadlock_at_boundary(bound_pos: Tuple[int, int], direction: str, walls: set) -> bool:
    """
    检查边界位置是否是死锁（墙角）

    Args:
        bound_pos: 边界位置
        direction: 方向 ('up', 'down', 'left', 'right')
        walls: 墙体集合

    Returns:
        bool: 是否是死锁位置
    """
    dx, dy = 0, 0

    if direction == 'up':
        dy = -1
    elif direction == 'down':
        dy = 1
    elif direction == 'left':
        dx = -1
    elif direction == 'right':
        dx = 1

    check_pos = (bound_pos[0] - dx, bound_pos[1] - dy)
    return is_wall(check_pos, walls)


def calculate_single_direction_penalty(
    box_pos: Tuple[int, int],
    goal_pos: Tuple[int, int],
    direction: str,
    walls: set,
    map_width: int,
    map_height: int
) -> float:
    """
    计算单个方向的长墙惩罚

    Args:
        box_pos: 箱子位置
        goal_pos: 目标位置
        direction: 方向 ('right', 'down', 'left', 'up')
        walls: 墙体集合
        map_width: 地图宽度
        map_height: 地图高度

    Returns:
        float: 该方向的长墙惩罚值
    """
    c, d = box_pos
    wall_dx, wall_dy = 0, 0

    if direction == 'right':
        wall_dx, wall_dy = 1, 0
    elif direction == 'down':
        wall_dx, wall_dy = 0, 1
    elif direction == 'left':
        wall_dx, wall_dy = -1, 0
    elif direction == 'up':
        wall_dx, wall_dy = 0, -1

    wall_adjacent = (c + wall_dx, d + wall_dy)
    if not is_wall(wall_adjacent, walls):
        return 0

    if direction in ['left', 'right']:
        wall_col = c + wall_dx
        center_row = (goal_pos[1] + d) / 2

        up_bound = d
        row = d - 1
        while row >= 0:
            if is_wall((wall_col, row), walls):
                up_bound = row
                row -= 1
            else:
                break

        down_bound = d
        row = d + 1
        while row < map_height:
            if is_wall((wall_col, row), walls):
                down_bound = row
                row += 1
            else:
                break

        if not (up_bound <= d <= down_bound):
            return 0

        dist_up = abs(center_row - up_bound)
        dist_down = abs(center_row - down_bound)
        x = min(dist_up, dist_down)

        up_is_deadlock = check_deadlock_at_boundary((wall_col, up_bound), direction, walls)
        down_is_deadlock = check_deadlock_at_boundary((wall_col, down_bound), direction, walls)

        if up_is_deadlock and down_is_deadlock:
            return 0
        elif up_is_deadlock:
            x = dist_down
        elif down_is_deadlock:
            x = dist_up

        if x == int(x):
            penalty = (x + 1) * 2 + 5
        else:
            penalty = (math.ceil(x) + 1) * 2 + 5

        return penalty

    else:
        wall_row = d + wall_dy
        center_col = (goal_pos[0] + c) / 2

        left_bound = c
        col = c - 1
        while col >= 0:
            if is_wall((col, wall_row), walls):
                left_bound = col
                col -= 1
            else:
                break

        right_bound = c
        col = c + 1
        while col < map_width:
            if is_wall((col, wall_row), walls):
                right_bound = col
                col += 1
            else:
                break

        if not (left_bound <= c <= right_bound):
            return 0

        dist_left = abs(center_col - left_bound)
        dist_right = abs(center_col - right_bound)
        x = min(dist_left, dist_right)

        left_is_deadlock = check_deadlock_at_boundary((left_bound, wall_row), direction, walls)
        right_is_deadlock = check_deadlock_at_boundary((right_bound, wall_row), direction, walls)

        if left_is_deadlock and right_is_deadlock:
            return 0
        elif left_is_deadlock:
            x = dist_right
        elif right_is_deadlock:
            x = dist_left

        if x == int(x):
            penalty = (x + 1) * 2 + 5
        else:
            penalty = (math.ceil(x) + 1) * 2 + 5

        return penalty


def calculate_wall_penalty(
    box_pos: Tuple[int, int],
    goal_pos: Tuple[int, int],
    walls: set,
    map_width: int,
    map_height: int
) -> float:
    """
    计算因箱子后方长墙体导致的附加距离（"解救"成本）

    适用场景：长墙体在箱子后方（不穿过箱子-目标点连线）

    Args:
        box_pos: 箱子位置 (c, d)
        goal_pos: 目标位置 (a, b)
        walls: 墙壁集合
        map_width: 地图宽度
        map_height: 地图高度

    Returns:
        float: 附加距离（总惩罚值）
    """
    check_directions = determine_back_direction(box_pos, goal_pos)

    total_penalty = 0
    for direction, _, _ in check_directions:
        penalty = calculate_single_direction_penalty(
            box_pos, goal_pos, direction, walls, map_width, map_height
        )
        total_penalty += penalty

    return total_penalty


def calculate_wall_penalty_detailed(
    box_pos: Tuple[int, int],
    goal_pos: Tuple[int, int],
    walls: set,
    map_width: int,
    map_height: int
) -> dict:
    """
    计算长墙惩罚的详细信息（调试用）

    Args:
        box_pos: 箱子位置
        goal_pos: 目标位置
        walls: 墙壁集合
        map_width: 地图宽度
        map_height: 地图高度

    Returns:
        dict: 详细信息
    """
    check_directions = determine_back_direction(box_pos, goal_pos)

    details = {
        'box_pos': box_pos,
        'goal_pos': goal_pos,
        'directions_checked': [],
        'penalties': {},
        'total_penalty': 0
    }

    for direction, _, _ in check_directions:
        details['directions_checked'].append(direction)

        c, d = box_pos
        wall_dx, wall_dy = 0, 0

        if direction == 'right':
            wall_dx, wall_dy = 1, 0
        elif direction == 'down':
            wall_dx, wall_dy = 0, 1
        elif direction == 'left':
            wall_dx, wall_dy = -1, 0
        elif direction == 'up':
            wall_dx, wall_dy = 0, -1

        wall_adjacent = (c + wall_dx, d + wall_dy)
        has_adjacent_wall = is_wall(wall_adjacent, walls)

        details['penalties'][direction] = {
            'adjacent_wall': wall_adjacent,
            'has_adjacent_wall': has_adjacent_wall,
            'penalty': 0
        }

        if has_adjacent_wall:
            penalty = calculate_single_direction_penalty(
                box_pos, goal_pos, direction, walls, map_width, map_height
            )
            details['penalties'][direction]['penalty'] = penalty
            details['total_penalty'] += penalty

    return details
