"""
统一 BFS 工具模块

提供项目中所有 BFS 最短路径和可达性检查的公共实现。
消除 algorithms/ 下多个文件中的重复 BFS 代码。
"""

from collections import deque
from typing import List, Optional, Set, Tuple


def bfs_shortest_path(
    start: Tuple[int, int],
    target: Tuple[int, int],
    obstacles: Set[Tuple[int, int]],
    width: int,
    height: int,
    max_steps: int = 300,
) -> Optional[Tuple[int, List[Tuple[int, int]]]]:
    """
    车/物体自由移动 BFS 最短路径。

    Args:
        start:     起始网格坐标
        target:    目标网格坐标
        obstacles: 障碍物坐标集合（墙+物体等）
        width:     地图宽度
        height:    地图高度
        max_steps: 最大步数限制

    Returns:
        (steps, path) 或 None（不可达）
        path 包含终点但不含起点: [start] 为仅一步可达时
    """
    if start == target:
        return 0, [start]

    queue = deque([start])
    visited = {start}
    parent: dict = {start: None}

    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    while queue:
        pos = queue.popleft()

        for dx, dy in directions:
            np = (pos[0] + dx, pos[1] + dy)

            if np == target:
                # 重建路径
                path = [np]
                cur = pos
                while cur is not None:
                    path.append(cur)
                    cur = parent.get(cur)
                path.reverse()
                return len(path) - 1, path

            if not (0 <= np[0] < width and 0 <= np[1] < height):
                continue
            if np in obstacles:
                continue
            if np in visited:
                continue
            visited.add(np)
            parent[np] = pos
            queue.append(np)

            # 步数限制（按展开节点数近似）
            if len(visited) > max_steps * 4:
                return None

    return None


def bfs_reachable_region(
    start: Tuple[int, int],
    obstacles: Set[Tuple[int, int]],
    width: int,
    height: int,
    max_steps: Optional[int] = None,
) -> Set[Tuple[int, int]]:
    """
    BFS 从起点可达的所有位置。

    Args:
        start:     起始网格坐标
        obstacles: 障碍物坐标集合
        width:     地图宽度
        height:    地图高度
        max_steps: 最大步数限制（None = 不限）

    Returns:
        可达位置集合（包含 start）
    """
    reachable: Set[Tuple[int, int]] = {start}
    queue = deque([start])

    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    steps = 0

    while queue:
        x, y = queue.popleft()
        steps += 1
        if max_steps is not None and steps > max_steps:
            break

        for dx, dy in directions:
            np = (x + dx, y + dy)
            if not (0 <= np[0] < width and 0 <= np[1] < height):
                continue
            if np in obstacles:
                continue
            if np in reachable:
                continue
            reachable.add(np)
            queue.append(np)

    return reachable
