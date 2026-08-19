"""
方向处理模块

提供方向相关的计算函数
"""

# 方向定义
DIRECTION_VECTORS = {
    'UP': (0, -1),
    'DOWN': (0, 1),
    'LEFT': (-1, 0),
    'RIGHT': (1, 0)
}

DIRECTION_NAMES = ['UP', 'DOWN', 'LEFT', 'RIGHT']
DIRECTION_OFFSETS = [(0, -1), (0, 1), (-1, 0), (1, 0)]

# 反方向映射
OPPOSITE_DIRECTION = {
    'UP': 'DOWN',
    'DOWN': 'UP',
    'LEFT': 'RIGHT',
    'RIGHT': 'LEFT'
}


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


def get_opposite_direction(direction: str) -> str:
    """
    获取相反方向

    Args:
        direction: 方向 ('UP', 'DOWN', 'LEFT', 'RIGHT')

    Returns:
        str: 相反方向
    """
    return OPPOSITE_DIRECTION[direction]


def get_all_neighbors(pos: tuple) -> list:
    """
    获取所有相邻位置

    Args:
        pos: 当前位置

    Returns:
        list: 相邻位置列表 [(pos, direction), ...]
    """
    result = []
    for direction in DIRECTION_NAMES:
        neighbor = get_neighbor(pos, direction)
        result.append((neighbor, direction))
    return result


def get_push_direction(from_pos: tuple, to_pos: tuple) -> str:
    """
    确定推动方向（从 from_pos 推到 to_pos）

    Args:
        from_pos: 起始位置
        to_pos: 目标位置

    Returns:
        str: 推动方向 ('UP', 'DOWN', 'LEFT', 'RIGHT')

    Raises:
        ValueError: 如果两个位置不相邻
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]

    if dx == 0 and dy == -1:
        return 'UP'
    elif dx == 0 and dy == 1:
        return 'DOWN'
    elif dx == -1 and dy == 0:
        return 'LEFT'
    elif dx == 1 and dy == 0:
        return 'RIGHT'
    else:
        raise ValueError(f"Positions {from_pos} and {to_pos} are not adjacent")


def get_car_position_for_push(box_pos: tuple, push_direction: str) -> tuple:
    """
    获取推动箱子时车应该所在的位置

    推动位置 = 箱子推动方向的反方向一格

    Args:
        box_pos: 箱子位置
        push_direction: 推动方向

    Returns:
        tuple: 车应该所在的位置
    """
    return get_neighbor(box_pos, get_opposite_direction(push_direction))


def is_adjacent(pos1: tuple, pos2: tuple) -> bool:
    """
    检查两个位置是否相邻

    Args:
        pos1: 位置1
        pos2: 位置2

    Returns:
        bool: 是否相邻
    """
    dx = abs(pos1[0] - pos2[0])
    dy = abs(pos1[1] - pos2[1])
    return (dx == 1 and dy == 0) or (dx == 0 and dy == 1)


def get_direction_from_to(from_pos: tuple, to_pos: tuple) -> str:
    """
    获取从 from_pos 到 to_pos 的方向

    Args:
        from_pos: 起始位置
        to_pos: 目标位置

    Returns:
        str: 方向 ('UP', 'DOWN', 'LEFT', 'RIGHT')

    Raises:
        ValueError: 如果两个位置不相邻
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]

    if dx == 0 and dy < 0:
        return 'UP'
    elif dx == 0 and dy > 0:
        return 'DOWN'
    elif dx < 0 and dy == 0:
        return 'LEFT'
    elif dx > 0 and dy == 0:
        return 'RIGHT'
    else:
        raise ValueError(f"Positions {from_pos} and {to_pos} are not adjacent in cardinal direction")


def get_back_directions(box_pos: tuple, goal_pos: tuple) -> list:
    """
    获取箱子后方需要检查的方向

    后方 = 目标方向的相反方向

    Args:
        box_pos: 箱子位置
        goal_pos: 目标位置

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
