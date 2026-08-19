"""
评分系统模块

提供箱子-目标点匹配的统一评分函数
"""

from typing import List, Tuple, Set, Optional, Dict

from .distance import manhattan_distance
from .map_analysis import get_walls_in_rect, is_boundary_wall
from .wall_penalty import calculate_wall_penalty
from .object_push import bfs_push_object, bfs_box_to_goal, simple_object_reachability


# =============================================================================
# 可达性缓存（用于加速分配算法）
# =============================================================================
_reachability_cache = {}

def clear_reachability_cache():
    """清空可达性缓存"""
    global _reachability_cache
    _reachability_cache = {}


# =============================================================================
# 子评分函数
# =============================================================================

def score_reachability(
    box: tuple,
    goal: tuple,
    walls: set,
    width: int,
    height: int,
    obstacles: Optional[set] = None,
    use_cache: bool = True,
    use_simple_check: bool = True
) -> Tuple[bool, Optional[list]]:
    """
    子函数1：可达性评分

    检查箱子能否通过推动到达目标

    Args:
        box: 箱子位置
        goal: 目标位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        obstacles: 额外障碍物
        use_cache: 是否使用缓存
        use_simple_check: 是否使用简化检查（只检查箱子位置，不检查车）

    Returns:
        tuple: (reachable: bool, path: list or None)
    """
    global _reachability_cache
    
    # 缓存键
    cache_key = (box, goal, frozenset(walls), frozenset(obstacles) if obstacles else None)
    
    if use_cache and cache_key in _reachability_cache:
        cached = _reachability_cache[cache_key]
        return cached, None
    
    # 底层调用：根据粒度选择不同函数
    if use_simple_check:
        # 底层1：只检查物体位置，最快
        reachable = simple_object_reachability(
            box, goal, walls, width, height, obstacles
        )
        result = (reachable, None)
    else:
        # 底层2：检查物体+车位置，标准版
        box_path = bfs_box_to_goal(
            box, goal, walls, width, height, obstacles or set()
        )
        result = (box_path is not None, box_path)
    
    if use_cache:
        _reachability_cache[cache_key] = result[0]
    
    return result


def score_manhattan(box: tuple, goal: tuple, weight: float = 1.0) -> float:
    """
    子函数2：曼哈顿距离评分

    距离越小越好，用负分表示

    Args:
        box: 箱子位置
        goal: 目标位置
        weight: 权重

    Returns:
        float: 负的距离 * 权重
    """
    dist = manhattan_distance(box, goal)
    return -dist * weight


def score_path_length(path: Optional[list], weight: float = 1.0) -> float:
    """
    子函数3：路径长度评分

    路径越短越好

    Args:
        path: 推动路径
        weight: 权重

    Returns:
        float: 1000(可达奖励) - 路径长度 * 权重
    """
    if path is None or len(path) == 0:
        return 0

    return 1000 - len(path) * weight


def score_push_direction(path: Optional[list]) -> float:
    """
    子函数4：推动方向评分

    垂直方向略优

    Args:
        path: 推动路径

    Returns:
        float: 垂直方向+10
    """
    if path is None or len(path) == 0:
        return 0

    direction = path[0][1]
    if direction in ['UP', 'DOWN']:
        return 10
    return 0


def score_wall_penalty_sub(
    box: tuple,
    goal: tuple,
    walls: set,
    width: int,
    height: int,
    weight: float = 1.0
) -> float:
    """
    子函数5：长墙惩罚（调用wall_penalty模块）

    解救成本作为负分

    Args:
        box: 箱子位置
        goal: 目标位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        weight: 权重

    Returns:
        float: -penalty * weight
    """
    try:
        penalty = calculate_wall_penalty(box, goal, walls, width, height)
        return -penalty * weight
    except Exception:
        return 0


def score_blast_reachable(
    box: tuple,
    goal: tuple,
    walls: set,
    width: int,
    height: int,
    bombs: List[tuple],
    obstacles: Optional[set] = None,
    weight: float = 1.0
) -> float:
    """
    子函数6：炸墙可达性评分

    检查炸墙后能否到达目标，以及炸弹能否到达炸点

    Args:
        box: 箱子位置
        goal: 目标位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        bombs: 炸弹列表
        obstacles: 额外障碍物
        weight: 权重

    Returns:
        float: 炸墙后可达且炸弹可达则加分，否则扣分
    """
    if not bombs:
        return 0

    # 找矩形区域内的墙
    from .map_analysis import get_walls_in_3x3
    rect_walls = get_walls_in_rect(box, goal, walls, expand=1)

    found_reachable_blast = False
    best_blast_score = 0

    for wall in rect_walls:
        if is_boundary_wall(wall, width, height):
            continue

        # 临时移除墙体
        temp_walls = walls - get_walls_in_3x3(wall, walls)

        # 检查炸墙后箱子能否到达目标
        temp_path = bfs_push_object(
            box, goal, temp_walls, width, height,
            additional_obstacles=obstacles
        )

        if temp_path['success']:
            # 检查炸弹能否到达这个墙
            blast_reachable = False
            for bomb in bombs:
                bomb_result = bfs_push_object(
                    bomb, wall, walls, width, height,
                    additional_obstacles=obstacles
                )
                if bomb_result['success']:
                    blast_reachable = True
                    break

            if blast_reachable:
                found_reachable_blast = True
                # 炸墙后可达且炸弹可达，大幅加分
                blast_score = 1500 - len(temp_path['object_path']) * 2
                if blast_score > best_blast_score:
                    best_blast_score = blast_score

    if found_reachable_blast:
        return best_blast_score * weight
    else:
        # 所有候选墙都不可达，大幅扣分
        return -500 * weight


# =============================================================================
# 综合评分函数
# =============================================================================

def calculate_pair_score(
    box: tuple,
    goal: tuple,
    walls: set,
    width: int,
    height: int,
    other_obstacles: Optional[set] = None,
    bombs: Optional[List[tuple]] = None,
    max_nodes: int = 50000,
    skip_blast_check: bool = False
) -> float:
    """
    计算箱子-目标配对的综合分数

    根据是否直达自适应调整权重：
    - 能直达：降低曼哈顿距离权重，提高路径和解救成本权重
    - 不能直达：保持原权重

    Args:
        box: 箱子位置
        goal: 目标位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        other_obstacles: 其他箱子作为障碍物
        bombs: 炸弹列表
        max_nodes: 最大节点数（保留参数）
        skip_blast_check: 是否跳过炸墙可达性检查（加速分配算法）

    Returns:
        float: 综合分数
    """
    # 检查可达性
    is_reachable, box_path = score_reachability(
        box, goal, walls, width, height, other_obstacles
    )

    # 根据是否直达调整权重
    if is_reachable:
        manhattan_weight = 1.0
        path_weight = 2.0
        rescue_weight = 2.0
    else:
        manhattan_weight = 2.0
        path_weight = 0.5
        rescue_weight = 0.5

    total_score = 0

    # 1. 曼哈顿距离
    total_score += score_manhattan(box, goal, manhattan_weight)

    # 2. 路径长度
    total_score += score_path_length(box_path, path_weight)

    # 3. 推动方向
    total_score += score_push_direction(box_path)

    # 4. 长墙惩罚
    total_score += score_wall_penalty_sub(box, goal, walls, width, height, rescue_weight)

    # 5. 炸墙可达性（默认跳过，加速分配算法）
    if bombs and not skip_blast_check:
        total_score += score_blast_reachable(
            box, goal, walls, width, height, bombs, other_obstacles
        )

    return total_score


def calculate_matching_score(
    pairs: List[Tuple[tuple, tuple]],
    walls: set,
    width: int,
    height: int,
    other_boxes_map: Dict[tuple, list],
    bombs: List[tuple],
    max_nodes: int = 50000
) -> float:
    """
    计算匹配的总分数

    分数越高表示匹配越好

    Args:
        pairs: 匹配 [(box, goal), ...]
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        other_boxes_map: 每个箱子对应的其他箱子列表
        bombs: 炸弹列表
        max_nodes: 最大节点数

    Returns:
        float: 总分数
    """
    total_score = 0

    for box, goal in pairs:
        other_boxes = other_boxes_map.get(box, [])
        score = calculate_pair_score(
            box, goal, walls, width, height,
            other_obstacles=set(other_boxes),
            bombs=bombs,
            max_nodes=max_nodes
        )
        total_score += score

    return total_score


def score_box_assignment(
    boxes: List[tuple],
    goals: List[tuple],
    walls: set,
    width: int,
    height: int,
    bombs: Optional[List[tuple]] = None,
    score_func: Optional[callable] = None
) -> Dict:
    """
    评估箱子分配方案

    Args:
        boxes: 箱子列表
        goals: 目标列表
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        bombs: 炸弹列表
        score_func: 评分函数（默认 calculate_pair_score）

    Returns:
        dict: {
            'assignment': {box: goal},
            'total_score': float,
            'individual_scores': [(box, goal, score), ...]
        }
    """
    if score_func is None:
        score_func = lambda b, g: calculate_pair_score(
            b, g, walls, width, height, bombs=bombs
        )

    # 构建障碍物映射
    other_boxes_map = {}
    for box in boxes:
        other_boxes_map[box] = [b for b in boxes if b != box]

    total_score = 0
    individual_scores = []

    for box, goal in zip(boxes, goals):
        score = calculate_pair_score(
            box, goal, walls, width, height,
            other_obstacles=set(other_boxes_map.get(box, [])),
            bombs=bombs
        )
        total_score += score
        individual_scores.append((box, goal, score))

    assignment = dict(zip(boxes, goals))

    return {
        'assignment': assignment,
        'total_score': total_score,
        'individual_scores': individual_scores
    }
