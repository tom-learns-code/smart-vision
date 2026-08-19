"""
Core 核心模块

提供路径搜索、物体推动、地图分析等底层功能
"""

from .car_move import (
    bfs_car_path,
    bfs_car_reachable,
    bfs_car_to_push_position,
    DIRECTION_VECTORS,
    DIRECTION_NAMES,
    DIRECTION_OFFSETS,
    get_neighbor
)

from .object_push import (
    bfs_push_object,
    bfs_push_object_optimized,
    check_object_can_reach,
    find_all_reachable_positions
)

from .direction import (
    get_opposite_direction,
    get_all_neighbors,
    get_push_direction,
    get_car_position_for_push,
    is_adjacent,
    get_direction_from_to,
    get_back_directions
)

from .distance import (
    manhattan_distance,
    euclidean_distance,
    chebyshev_distance,
    min_distance_to_set,
    distance_matrix,
    center_of_mass,
    bounding_box
)

from .map_analysis import (
    is_valid_coordinate,
    is_wall,
    is_boundary_wall,
    is_valid_position,
    get_walls_in_3x3,
    get_walls_in_rect,
    is_corner_deadlock,
    count_adjacent_walls,
    get_free_neighbors,
    get_internal_walls,
    get_blast_candidates,
    analyze_connectivity,
    scan_map,
    load_map_from_file
)

from .scoring import (
    calculate_pair_score,
    calculate_matching_score,
    score_box_assignment,
    score_reachability,
    score_manhattan,
    score_path_length,
    score_push_direction,
    score_wall_penalty_sub,
    score_blast_reachable
)

from .wall_penalty import (
    calculate_wall_penalty,
    calculate_wall_penalty_detailed,
    determine_back_direction,
    calculate_single_direction_penalty
)

__all__ = [
    # 车移动
    'bfs_car_path',
    'bfs_car_reachable',
    'bfs_car_to_push_position',
    'DIRECTION_VECTORS',
    'DIRECTION_NAMES',
    'DIRECTION_OFFSETS',
    'get_neighbor',

    # 物体推动
    'bfs_push_object',
    'bfs_push_object_optimized',
    'check_object_can_reach',
    'find_all_reachable_positions',

    # 方向
    'get_opposite_direction',
    'get_all_neighbors',
    'get_push_direction',
    'get_car_position_for_push',
    'is_adjacent',
    'get_direction_from_to',
    'get_back_directions',

    # 距离
    'manhattan_distance',
    'euclidean_distance',
    'chebyshev_distance',
    'min_distance_to_set',
    'distance_matrix',
    'center_of_mass',
    'bounding_box',

    # 地图分析
    'is_valid_coordinate',
    'is_wall',
    'is_boundary_wall',
    'is_valid_position',
    'get_walls_in_3x3',
    'get_walls_in_rect',
    'is_corner_deadlock',
    'count_adjacent_walls',
    'get_free_neighbors',
    'get_internal_walls',
    'get_blast_candidates',
    'analyze_connectivity',
    'scan_map',
    'load_map_from_file',

    # 评分
    'calculate_pair_score',
    'calculate_matching_score',
    'score_box_assignment',
    'score_reachability',
    'score_manhattan',
    'score_path_length',
    'score_push_direction',
    'score_wall_penalty_sub',
    'score_blast_reachable',

    # 长墙惩罚
    'calculate_wall_penalty',
    'calculate_wall_penalty_detailed',
    'determine_back_direction',
    'calculate_single_direction_penalty'
]
