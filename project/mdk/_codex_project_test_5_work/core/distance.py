"""
距离计算模块

提供各种距离计算函数
"""

import math
from typing import Optional, Callable


def manhattan_distance(pos1: tuple, pos2: tuple) -> int:
    """
    计算曼哈顿距离

    Args:
        pos1: 位置1
        pos2: 位置2

    Returns:
        int: 曼哈顿距离
    """
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


def euclidean_distance(pos1: tuple, pos2: tuple) -> float:
    """
    计算欧几里得距离

    Args:
        pos1: 位置1
        pos2: 位置2

    Returns:
        float: 欧几里得距离
    """
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    return math.sqrt(dx * dx + dy * dy)


def chebyshev_distance(pos1: tuple, pos2: tuple) -> int:
    """
    计算切比雪夫距离（棋盘距离）

    Args:
        pos1: 位置1
        pos2: 位置2

    Returns:
        int: 切比雪夫距离
    """
    return max(abs(pos1[0] - pos2[0]), abs(pos1[1] - pos2[1]))


def min_distance_to_set(pos: tuple, target_set: set,
                        distance_func: Optional[Callable] = None) -> tuple:
    """
    计算到集合中所有点的最小距离

    Args:
        pos: 当前位置
        target_set: 目标点集合
        distance_func: 距离函数（默认曼哈顿距离）

    Returns:
        tuple: (最小距离, 最近目标点)
    """
    if not target_set:
        return float('inf'), None

    if distance_func is None:
        distance_func = manhattan_distance

    min_dist = float('inf')
    nearest = None

    for target in target_set:
        dist = distance_func(pos, target)
        if dist < min_dist:
            min_dist = dist
            nearest = target

    return min_dist, nearest


def distance_matrix(points1: list, points2: list,
                   distance_func: Optional[Callable] = None) -> list:
    """
    计算两点集之间的距离矩阵

    Args:
        points1: 点集1
        points2: 点集2
        distance_func: 距离函数

    Returns:
        list: 距离矩阵 [points1 x points2]
    """
    if distance_func is None:
        distance_func = manhattan_distance

    matrix = []
    for p1 in points1:
        row = []
        for p2 in points2:
            row.append(distance_func(p1, p2))
        matrix.append(row)

    return matrix


def center_of_mass(positions: list) -> tuple:
    """
    计算多个位置的中心点

    Args:
        positions: 位置列表

    Returns:
        tuple: 中心点坐标 (x, y)
    """
    if not positions:
        return (0, 0)

    sum_x = sum(p[0] for p in positions)
    sum_y = sum(p[1] for p in positions)
    n = len(positions)

    return (sum_x / n, sum_y / n)


def bounding_box(positions: list) -> tuple:
    """
    计算多个位置的边界框

    Args:
        positions: 位置列表

    Returns:
        tuple: (min_x, min_y, max_x, max_y)
    """
    if not positions:
        return (0, 0, 0, 0)

    min_x = min(p[0] for p in positions)
    max_x = max(p[0] for p in positions)
    min_y = min(p[1] for p in positions)
    max_y = max(p[1] for p in positions)

    return (min_x, min_y, max_x, max_y)


def rectangle_area(pos1: tuple, pos2: tuple) -> int:
    """
    计算两点确定的矩形的面积

    Args:
        pos1: 位置1
        pos2: 位置2

    Returns:
        int: 矩形面积
    """
    width = abs(pos1[0] - pos2[0]) + 1
    height = abs(pos1[1] - pos2[1]) + 1
    return width * height


def manhattan_center(pos1: tuple, pos2: tuple) -> float:
    """
    计算曼哈顿几何中心

    Args:
        pos1: 位置1
        pos2: 位置2

    Returns:
        float: 中心点坐标（可能是小数）
    """
    return (pos1[0] + pos2[0]) / 2, (pos1[1] + pos2[1]) / 2
