"""
物体推动核心模块 - 可推路径计算

提供推动物体（箱子、炸弹）从起点到终点的完整路径
"""

from collections import deque
from typing import Optional

from .car_move import DIRECTION_VECTORS, DIRECTION_NAMES, DIRECTION_OFFSETS, get_neighbor


def bfs_push_object(
    object_start: tuple,
    object_end: tuple,
    walls: set,
    width: int,
    height: int,
    additional_obstacles: Optional[set] = None,
    initial_push_direction: Optional[str] = None
) -> dict:
    """
    计算推动物体从起点到终点的完整路径（车的推动路径）

    游戏规则：
    - 车只能推动一个物体，不能连续推动
    - 物体移动时，其他物体（箱子、炸弹）视为障碍物
    - 目标点自动豁免（即使在其他障碍物中）

    状态空间: (object_pos, push_direction)
    - object_pos: 物体当前位置
    - push_direction: 上一次推的方向（决定车所在的位置）
    - 第一步必须是PUSH操作

    障碍物处理：
    1. walls: 默认障碍物
    2. additional_obstacles: 额外障碍物（其他箱子、炸弹等）
    3. object_end: 自动豁免（目标点）

    Args:
        object_start: 被推物体起始位置
        object_end: 被推物体目标位置（享有豁免权）
        walls: 墙体坐标集合
        width: 地图宽度
        height: 地图高度
        additional_obstacles: 额外障碍物集合（可选）
            - 其他箱子：{(5,5), (7,7)}
            - 其他炸弹：{(3,3)}
        initial_push_direction:
            - 指定第一步推动方向（手动干预）
            - None: 自动选择最优方向（默认）

    Returns:
        dict: {
            'car_path': list,       # 车的完整推动路径 [('MOVE', 'UP'), ('PUSH', 'UP'), ...]
            'object_path': list,   # 被推物体的路径 [(x,y), (x,y), ...]
            'initial_car_pos': tuple, # 车的初始位置（第一步推动方向的反方向一格）
            'first_push_direction': str, # 第一步推动方向
            'total_cost': int,     # 总步数
            'success': bool
        }
    """
    # 目标点自动豁免
    exempt_positions = {object_end}

    # 初始化障碍物
    obstacles = set(walls)
    if additional_obstacles:
        obstacles.update(additional_obstacles)

    # 获取推动方向的反方向
    opposite_map = {
        'UP': 'DOWN',
        'DOWN': 'UP',
        'LEFT': 'RIGHT',
        'RIGHT': 'LEFT'
    }

    # 如果指定了初始方向，直接使用
    if initial_push_direction:
        push_dirs = [initial_push_direction]
    else:
        push_dirs = DIRECTION_NAMES

    # 尝试所有可能的初始推动方向
    best_result = None
    best_cost = float('inf')

    for push_dir in push_dirs:
        dx, dy = DIRECTION_VECTORS[push_dir]
        new_object_pos = (object_start[0] + dx, object_start[1] + dy)
        car_at_push_pos = object_start  # 推动后，车在物体原来的位置

        # 检查推动是否合法
        # 1. 物体新位置不能越界
        if not (0 <= new_object_pos[0] < width and 0 <= new_object_pos[1] < height):
            continue

        # 2. 物体新位置不能是墙体或额外障碍物（除非是目标点）
        if new_object_pos in obstacles and new_object_pos not in exempt_positions:
            continue

        # 3. 车必须能到达推动位置（物体原来的位置）
        #    但物体原来位置可能就是起点，所以车已经在那里了
        #    推动后，物体移动，车留在原来位置

        # 状态: (object_pos, push_direction)
        # 初始状态: (new_object_pos, push_dir)
        # 注意：这里我们把物体已经推了一步

        state = (new_object_pos, push_dir)
        queue = deque()
        queue.append((state, [('PUSH', push_dir)], [object_start, new_object_pos], car_at_push_pos))

        visited = {state}
        all_results = []

        while queue:
            (obj_pos, last_push_dir), path, obj_path, car_pos = queue.popleft()

            # 检查是否到达目标
            if obj_pos == object_end:
                all_results.append({
                    'path': path,
                    'object_path': obj_path,
                    'car_pos': car_pos,
                    'cost': len(path)
                })
                continue

            # 尝试继续推动
            for next_dir in DIRECTION_NAMES:
                ndx, ndy = DIRECTION_VECTORS[next_dir]
                next_obj_pos = (obj_pos[0] + ndx, obj_pos[1] + ndy)
                next_car_pos = obj_pos  # 推动后车在物体原来位置

                # 检查合法性
                if not (0 <= next_obj_pos[0] < width and 0 <= next_obj_pos[1] < height):
                    continue

                if next_obj_pos in obstacles and next_obj_pos not in exempt_positions:
                    continue

                # 车已经在这里了（上一推的位置），所以不需要移动
                next_state = (next_obj_pos, next_dir)
                if next_state not in visited:
                    visited.add(next_state)
                    queue.append((
                        next_state,
                        path + [('PUSH', next_dir)],
                        obj_path + [next_obj_pos],
                        next_car_pos
                    ))

            # 尝试在推动之间移动车（车绕行）
            # 车可以从当前位置移动到其他推动位置
            for move_dir in DIRECTION_NAMES:
                mdx, mdy = DIRECTION_VECTORS[move_dir]
                move_car_pos = (car_pos[0] + mdx, car_pos[1] + mdy)

                # 检查车移动是否合法
                if not (0 <= move_car_pos[0] < width and 0 <= move_car_pos[1] < height):
                    continue

                if move_car_pos in obstacles:
                    continue

                # 检查是否能从移动后的位置推动
                for push_dir_candidate in DIRECTION_NAMES:
                    pdx, pdy = DIRECTION_VECTORS[push_dir_candidate]
                    behind_car = (move_car_pos[0] + pdx, move_car_pos[1] + pdy)

                    # 检查推动是否指向物体
                    if behind_car != obj_pos:
                        continue

                    # 推动后物体位置
                    push_obj_pos = (obj_pos[0] + pdx, obj_pos[1] + pdy)

                    if not (0 <= push_obj_pos[0] < width and 0 <= push_obj_pos[1] < height):
                        continue

                    if push_obj_pos in obstacles and push_obj_pos not in exempt_positions:
                        continue

                    # 合法：车移动到move_car_pos，然后从那里推动
                    next_state = (push_obj_pos, push_dir_candidate)
                    if next_state not in visited:
                        visited.add(next_state)
                        queue.append((
                            next_state,
                            path + [('MOVE', move_dir), ('PUSH', push_dir_candidate)],
                            obj_path + [push_obj_pos],
                            push_obj_pos
                        ))

        # 从所有结果中选择最优
        for result in all_results:
            if result['cost'] < best_cost:
                best_cost = result['cost']
                best_result = {
                    'car_path': result['path'],
                    'object_path': result['object_path'],
                    'initial_car_pos': object_start,
                    'first_push_direction': push_dir,
                    'total_cost': result['cost'],
                    'success': True
                }

    if best_result:
        return best_result

    return {
        'car_path': [],
        'object_path': [],
        'initial_car_pos': None,
        'first_push_direction': None,
        'total_cost': float('inf'),
        'success': False
    }


def bfs_push_object_optimized(
    object_start: tuple,
    object_end: tuple,
    walls: set,
    width: int,
    height: int,
    additional_obstacles: Optional[set] = None,
    initial_push_direction: Optional[str] = None,
    max_iterations: int = 50000
) -> dict:
    """
    计算推动物体路径的优化版本（带迭代限制）

    Args:
        object_start: 被推物体起始位置
        object_end: 被推物体目标位置
        walls: 墙体坐标集合
        width: 地图宽度
        height: 地图高度
        additional_obstacles: 额外障碍物集合
        initial_push_direction: 指定初始推动方向
        max_iterations: 最大迭代次数

    Returns:
        dict: 同 bfs_push_object
    """
    # 目标点自动豁免
    exempt_positions = {object_end}

    # 初始化障碍物
    obstacles = set(walls)
    if additional_obstacles:
        obstacles.update(additional_obstacles)

    opposite_map = {
        'UP': 'DOWN',
        'DOWN': 'UP',
        'LEFT': 'RIGHT',
        'RIGHT': 'LEFT'
    }

    if initial_push_direction:
        push_dirs = [initial_push_direction]
    else:
        push_dirs = DIRECTION_NAMES

    best_result = None
    best_cost = float('inf')
    iterations = 0

    for push_dir in push_dirs:
        dx, dy = DIRECTION_VECTORS[push_dir]
        new_object_pos = (object_start[0] + dx, object_start[1] + dy)
        car_at_push_pos = object_start

        if not (0 <= new_object_pos[0] < width and 0 <= new_object_pos[1] < height):
            continue

        if new_object_pos in obstacles and new_object_pos not in exempt_positions:
            continue

        state = (new_object_pos, push_dir)
        queue = deque()
        queue.append((state, [('PUSH', push_dir)], [object_start, new_object_pos], car_at_push_pos))

        visited = {state}

        while queue:
            iterations += 1
            if iterations >= max_iterations:
                break

            (obj_pos, last_push_dir), path, obj_path, car_pos = queue.popleft()

            if obj_pos == object_end:
                cost = len(path)
                if cost < best_cost:
                    best_cost = cost
                    best_result = {
                        'car_path': path,
                        'object_path': obj_path,
                        'initial_car_pos': object_start,
                        'first_push_direction': push_dir,
                        'total_cost': cost,
                        'success': True
                    }
                continue

            # 尝试继续推动
            for next_dir in DIRECTION_NAMES:
                ndx, ndy = DIRECTION_VECTORS[next_dir]
                next_obj_pos = (obj_pos[0] + ndx, obj_pos[1] + ndy)
                next_car_pos = obj_pos

                if not (0 <= next_obj_pos[0] < width and 0 <= next_obj_pos[1] < height):
                    continue

                if next_obj_pos in obstacles and next_obj_pos not in exempt_positions:
                    continue

                next_state = (next_obj_pos, next_dir)
                if next_state not in visited:
                    visited.add(next_state)
                    queue.append((
                        next_state,
                        path + [('PUSH', next_dir)],
                        obj_path + [next_obj_pos],
                        next_car_pos
                    ))

        if iterations >= max_iterations:
            break

    if best_result:
        return best_result

    return {
        'car_path': [],
        'object_path': [],
        'initial_car_pos': None,
        'first_push_direction': None,
        'total_cost': float('inf'),
        'success': False
    }


def check_object_can_reach(
    object_pos: tuple,
    target_pos: tuple,
    walls: set,
    width: int,
    height: int,
    additional_obstacles: Optional[set] = None
) -> bool:
    """
    快速检查物体能否到达目标位置（不计算路径）

    Args:
        object_pos: 物体位置
        target_pos: 目标位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        additional_obstacles: 额外障碍物

    Returns:
        bool: 是否可达
    """
    result = bfs_push_object(
        object_pos, target_pos, walls, width, height,
        additional_obstacles=additional_obstacles
    )
    return result['success']


def find_all_reachable_positions(
    object_pos: tuple,
    walls: set,
    width: int,
    height: int,
    additional_obstacles: Optional[set] = None,
    max_distance: int = 20
) -> dict:
    """
    找出物体所有可达的位置

    Args:
        object_pos: 物体起始位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        additional_obstacles: 额外障碍物
        max_distance: 最大搜索距离

    Returns:
        dict: {
            'positions': set,  # 所有可达位置
            'distances': dict  # 到每个位置的最短距离
        }
    """
    obstacles = set(walls)
    if additional_obstacles:
        obstacles.update(additional_obstacles)

    # 使用BFS搜索可达位置
    queue = deque([(object_pos, 0)])
    visited = {object_pos: 0}

    while queue:
        pos, dist = queue.popleft()

        if dist >= max_distance:
            continue

        for direction in DIRECTION_NAMES:
            dx, dy = DIRECTION_VECTORS[direction]
            new_pos = (pos[0] + dx, pos[1] + dy)

            if not (0 <= new_pos[0] < width and 0 <= new_pos[1] < height):
                continue

            if new_pos in obstacles:
                continue

            if new_pos not in visited or dist + 1 < visited[new_pos]:
                visited[new_pos] = dist + 1
                queue.append((new_pos, dist + 1))

    return {
        'positions': set(visited.keys()),
        'distances': visited
    }


# =============================================================================
# 旧版简化 BFS（用于快速可达性判断）
# =============================================================================

def get_required_car_pos(box_pos, direction):
    """
    获取推动箱子到指定方向时，车应该在的位置
    
    Args:
        box_pos: 箱子当前位置
        direction: 推动方向
        
    Returns:
        tuple: 车应该到达的位置
    """
    dx, dy = DIRECTION_VECTORS[direction]
    # 车在推动方向的相反方向
    return (box_pos[0] - dx, box_pos[1] - dy)


def bfs_box_to_goal(
    box_start: tuple,
    goal: tuple,
    walls: set,
    width: int,
    height: int,
    obstacles: set
):
    """
    旧版 BFS：搜索箱子从起点到目标的推路径
    
    状态：(box_pos, path)
    只检查箱子位置和车能否到达推位置，不区分推动方向
    
    Args:
        box_start: 箱子起点
        goal: 目标位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        obstacles: 其他箱子（作为障碍）
        
    Returns:
        list: 路径 [(action, direction), ...] 或 None
    """
    if box_start == goal:
        return []
    
    queue = deque([(box_start, [])])
    visited = {box_start}
    
    while queue:
        box_pos, path = queue.popleft()
        
        if box_pos == goal:
            return path
        
        for direction in DIRECTION_NAMES:
            # 计算箱子移动后的位置
            new_box_pos = get_neighbor(box_pos, direction)
            
            # 检查越界
            if not (0 <= new_box_pos[0] < width and 0 <= new_box_pos[1] < height):
                continue
            
            # 检查箱子位置障碍物
            if new_box_pos in walls or new_box_pos in obstacles:
                continue
            
            # 计算车应该在的位置
            car_pos = get_required_car_pos(box_pos, direction)
            
            # 检查车位置越界
            if not (0 <= car_pos[0] < width and 0 <= car_pos[1] < height):
                continue
            
            # 检查车位置障碍物
            if car_pos in walls or car_pos in obstacles:
                continue
            
            # 车不能在箱子位置
            if car_pos == new_box_pos:
                continue
            
            if new_box_pos in visited:
                continue
            
            visited.add(new_box_pos)
            new_path = path + [('PUSH', direction)]
            queue.append((new_box_pos, new_path))
    
    return None


# =============================================================================
# 底层函数：简化可达性检查
# =============================================================================

def simple_object_reachability(
    object_pos: tuple,
    goal_pos: tuple,
    walls: set,
    width: int,
    height: int,
    obstacles: Optional[set] = None
) -> bool:
    """
    简化可达性检查：只检查物体位置，不检查车能否到达
    
    底层函数，最快速，用于排序/预判
    不检查车位置，只检查物体能否移动
    
    Args:
        object_pos: 物体当前位置
        goal_pos: 目标位置
        walls: 墙体集合
        width: 地图宽度
        height: 地图高度
        obstacles: 额外障碍物
        
    Returns:
        bool: 是否可达
    """
    if object_pos == goal_pos:
        return True
    
    all_obstacles = set(walls)
    if obstacles:
        all_obstacles.update(obstacles)
    all_obstacles.discard(goal_pos)  # 目标点豁免
    
    queue = deque([object_pos])
    visited = {object_pos}
    
    while queue:
        pos = queue.popleft()
        
        if pos == goal_pos:
            return True
        
        for direction in DIRECTION_NAMES:
            new_pos = get_neighbor(pos, direction)
            
            if not (0 <= new_pos[0] < width and 0 <= new_pos[1] < height):
                continue
            if new_pos in all_obstacles:
                continue
            if new_pos in visited:
                continue
            
            visited.add(new_pos)
            queue.append(new_pos)
    
    return False
