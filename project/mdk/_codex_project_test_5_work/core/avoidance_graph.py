"""
规避节点 + 安全连线图（阶段二修正方案）

在推箱搜索之前对地图做一次性结构分析，提取所有"碰撞热点"（规避节点），
预计算其间的安全连线。在复合图（格线 + 捷径）上跑 Dijkstra 自动产生对角捷径。

核心概念:
  规避节点   — 墙凸角/物体凸角的对角格子，车斜穿时最易碰撞的位置
  安全连线   — 两个规避节点之间车体扫过区域不碰任何障碍物的直线
  复合图     — 所有 walkable 格点 + 4邻格线边 + 规避节点间捷径边

用法:
  ag = AvoidanceGraph(walls, width, height)   # 开局预分析
  ag.set_objects(box_positions)               # 放置物体
  waypoints = ag.get_waypoints(start, target) # 每次 free_move
  ag.move_object(old_pos, new_pos)            # 物体移动后更新
"""

import heapq
import math
from collections import deque
from typing import List, Tuple, Optional, Set, Dict


# =============================================================================
# 几何工具
# =============================================================================

def _point_to_segment_dist(px, py, ax, ay, bx, by):
    """点 (px, py) 到线段 AB 的最短欧氏距离（网格单位）。"""
    dx = bx - ax
    dy = by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0,
                     ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def _swept_width(ax, ay, bx, by):
    """轴对齐 1×1 正方形沿线段 AB 移动的扫过宽度。"""
    ddx = bx - ax
    ddy = by - ay
    seg_len = math.hypot(ddx, ddy)
    if seg_len < 0.001:
        return 1.0
    return (abs(ddx) + abs(ddy)) / seg_len


# =============================================================================
# 规避节点查找
# =============================================================================

def find_avoidance_nodes(obstacles: Set[Tuple[int, int]],
                         width: int, height: int) -> Set[Tuple[int, int]]:
    """
    查找所有规避节点。

    规则（三条件必须同时满足）:
      a) P 的 4 个对角邻域中至少 1 个是障碍物
      b) P 的 4 个相邻邻域（上下左右）中至少 2 个是无障碍的
      c) 对于某个满足(a)的对角障碍物 D，D 与 P 共享的 2 个相邻格
         （即 D 的水平和垂直邻格）都必须无障碍物 —— D 形成一个凸角
    """
    nodes = set()
    for y in range(height):
        for x in range(width):
            if (x, y) in obstacles:
                continue

            # 条件 b: 相邻至少 2 侧自由
            free_adj = 0
            for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if (nx, ny) not in obstacles:
                        free_adj += 1
            if free_adj < 2:
                continue

            # 条件 a + c: 存在一个对角障碍物，其两个共享相邻格都无障碍
            has_convex_corner = False
            for dx, dy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
                diag_x, diag_y = x + dx, y + dy
                if not (0 <= diag_x < width and 0 <= diag_y < height):
                    continue
                if (diag_x, diag_y) not in obstacles:
                    continue

                # 条件 c: D 的两个共享相邻格 = P的水平和垂直邻格
                shared1 = (diag_x, y)     # D 的垂直方向邻格（与P同列）
                shared2 = (x, diag_y)     # D 的水平方向邻格（与P同行）

                s1_ok = (0 <= shared1[0] < width and 0 <= shared1[1] < height
                         and shared1 not in obstacles)
                s2_ok = (0 <= shared2[0] < width and 0 <= shared2[1] < height
                         and shared2 not in obstacles)

                if s1_ok and s2_ok:
                    has_convex_corner = True
                    break

            if has_convex_corner:
                nodes.add((x, y))

    return nodes


# =============================================================================
# 安全连线判定
# =============================================================================

def _center_line_clear(a: Tuple[int, int], b: Tuple[int, int],
                       walls: Set[Tuple[int, int]],
                       avoidance_nodes: Set[Tuple[int, int]]) -> bool:
    """
    条件一: Bresenham 中心线不穿过任何墙体或其他规避节点。
    规避节点在中心连线上视为不安全 → 防止 A→C→B 和 A→B 共存。
    """
    x0, y0 = a
    x1, y1 = b

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x, y = x0, y0
    while True:
        if (x, y) != a and (x, y) != b:
            if (x, y) in walls:
                return False
            if (x, y) in avoidance_nodes:
                return False
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy

    return True


def _bresenham_cells(sx: float, sy: float, ex: float, ey: float,
                      width: int, height: int) -> List[Tuple[int, int]]:
    """
    浮点 Bresenham：从 (sx, sy) 到 (ex, ey) 线段穿过的所有网格单元格。

    使用 round() 映射到网格（格心在整数坐标，格边界在 ±0.5），
    而非 floor()（格心在半整数坐标的像素网格）。
    """
    x, y = int(round(sx)), int(round(sy))
    end_x, end_y = int(round(ex)), int(round(ey))

    dx = abs(ex - sx)
    dy = abs(ey - sy)
    step_x = 1 if ex > sx else -1
    step_y = 1 if ey > sy else -1

    if dx < 0.001 and dy < 0.001:
        if 0 <= x < width and 0 <= y < height:
            return [(x, y)]
        return []

    # 当前格边界（格心 ± 0.5）
    if step_x > 0:
        next_x = x + 0.5
    else:
        next_x = x - 0.5
    if step_y > 0:
        next_y = y + 0.5
    else:
        next_y = y - 0.5

    t_delta_x = 1.0 / dx if dx > 0.001 else float('inf')
    t_delta_y = 1.0 / dy if dy > 0.001 else float('inf')
    t_max_x = (next_x - sx) / (ex - sx) if dx > 0.001 else float('inf')
    t_max_y = (next_y - sy) / (ey - sy) if dy > 0.001 else float('inf')

    cells = []
    last = None
    if 0 <= x < width and 0 <= y < height:
        cells.append((x, y))
        last = (x, y)

    while x != end_x or y != end_y:
        if t_max_x < t_max_y:
            x += step_x
            t_max_x += t_delta_x
        else:
            y += step_y
            t_max_y += t_delta_y
        p = (x, y)
        if p != last and 0 <= x < width and 0 <= y < height:
            cells.append(p)
            last = p

    return cells


def _swept_area_clear(a: Tuple[int, int], b: Tuple[int, int],
                      walls: Set[Tuple[int, int]],
                      width: int, height: int) -> bool:
    """
    条件二: 4 角连线法。
    将 1×1 正方形车体从 A 中心移到 B 中心，扫过区域的 4 条角边界
    都不穿过任何墙体 → 车体不碰墙。
    """
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])

    # 4条角线（±0.49偏移，略内缩0.01避免格边界误判）
    corners = [
        (ax - 0.49, ay - 0.49, bx - 0.49, by - 0.49),
        (ax + 0.49, ay - 0.49, bx + 0.49, by - 0.49),
        (ax - 0.49, ay + 0.49, bx - 0.49, by + 0.49),
        (ax + 0.49, ay + 0.49, bx + 0.49, by + 0.49),
    ]

    for sx, sy, ex, ey in corners:
        cells = _bresenham_cells(sx, sy, ex, ey, width, height)
        for c in cells:
            if c != a and c != b and c in walls:
                return False

    return True


def check_safe_connection(a: Tuple[int, int], b: Tuple[int, int],
                          walls: Set[Tuple[int, int]],
                          avoidance_nodes: Set[Tuple[int, int]],
                          width: int, height: int) -> bool:
    """两重判定：中心线干净 + 车体不扫墙。"""
    if not _center_line_clear(a, b, walls, avoidance_nodes):
        return False
    if not _swept_area_clear(a, b, walls, width, height):
        return False
    return True


# =============================================================================
# 复合图
# =============================================================================

def _build_avoidance_adj(avoidance_nodes: Set[Tuple[int, int]],
                         walls: Set[Tuple[int, int]],
                         width: int, height: int) -> Dict[Tuple[int, int],
                                                          List[Tuple[Tuple[int, int], float]]]:
    """
    预计算所有规避节点对的安全连线，构建规避节点邻接表。
    """
    adj = {}
    node_list = list(avoidance_nodes)
    for i, n1 in enumerate(node_list):
        adj[n1] = []
        for j, n2 in enumerate(node_list):
            if i >= j:
                continue
            if check_safe_connection(n1, n2, walls, avoidance_nodes, width, height):
                d = math.hypot(n2[0] - n1[0], n2[1] - n1[1])
                adj[n1].append((n2, d))
                adj.setdefault(n2, []).append((n1, d))
    return adj


# =============================================================================
# 复合 Dijkstra
# =============================================================================

def composite_dijkstra(start: Tuple[int, int], target: Tuple[int, int],
                       walls: Set[Tuple[int, int]],
                       avoidance_adj: Dict[Tuple[int, int],
                                           List[Tuple[Tuple[int, int], float]]],
                       width: int, height: int) -> Optional[List[Tuple[int, int]]]:
    """
    复合图上 Dijkstra。
    节点: 所有 walkable 格点
    边1（格线）: 4 邻接，代价 = 1
    边2（捷径）: 规避节点间安全连线，代价 = 欧氏距离

    返回: 最短路径的 waypoint 序列（网格坐标），包含规避节点间的对角捷径点。
    """
    if start == target:
        return [start]

    heap = [(0.0, start, [start])]
    best: Dict[Tuple[int, int], float] = {start: 0.0}

    while heap:
        cost, pos, path = heapq.heappop(heap)

        if cost > best.get(pos, float('inf')):
            continue

        if pos == target:
            return path

        # ---- 格线边（4 邻接）----
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = pos[0] + dx, pos[1] + dy
            np = (nx, ny)
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if np in walls:
                continue
            new_cost = cost + 1.5
            if new_cost < best.get(np, float('inf')):
                best[np] = new_cost
                heapq.heappush(heap, (new_cost, np, path + [np]))

        # ---- 捷径边（仅从规避节点出发）----
        if pos in avoidance_adj:
            for other, edge_cost in avoidance_adj[pos]:
                new_cost = cost + edge_cost
                if new_cost < best.get(other, float('inf')):
                    best[other] = new_cost
                    heapq.heappush(heap, (new_cost, other, path + [other]))

    return None


# =============================================================================
# 主类
# =============================================================================

class AvoidanceGraph:
    """
    规避节点 + 安全连线图管理器。

    用法:
      ag = AvoidanceGraph(walls, width, height)   # 开局（仅墙体）
      ag.set_objects(box_positions)               # 放置物体
      wps = ag.get_waypoints(start, target)       # 每次 free_move
      ag.move_object(old, new)                    # 物体移动后增量更新
    """

    def __init__(self, walls: Set[Tuple[int, int]],
                 width: int, height: int):
        self.width = width
        self.height = height

        # 墙体（不变）
        self._walls = walls

        # 物体位置（可变）
        self._object_positions: Set[Tuple[int, int]] = set()

        # 当前全部障碍物 = 墙 + 物体
        self._all_obstacles: Set[Tuple[int, int]] = set(walls)

        # 规避节点（仅墙体）
        self._wall_avoidance_nodes: Set[Tuple[int, int]] = \
            find_avoidance_nodes(walls, width, height)

        # 当前全部规避节点 = 墙体规避节点 + 物体规避节点
        self.avoidance_nodes: Set[Tuple[int, int]] = set(self._wall_avoidance_nodes)

        # 规避节点间安全连线的邻接表
        self._avoidance_adj: Dict[Tuple[int, int],
                                  List[Tuple[Tuple[int, int], float]]] = {}

        # 预计算（仅墙体）
        self._rebuild_full()

    # -----------------------------------------------------------------
    # 公共接口
    # -----------------------------------------------------------------

    def set_objects(self, positions: List[Tuple[int, int]]):
        """一次性设置所有物体位置（取代旧位置）。"""
        old = self._object_positions
        new = set(positions)
        # 移除旧的
        to_remove = old - new
        to_add = new - old
        for pos in to_remove:
            self._remove_single_object(pos)
        for pos in to_add:
            self._add_single_object(pos)
        self._object_positions = new

        # 只对变化了的局部做增量重算
        if to_remove or to_add:
            self._rebuild_affected(to_remove | to_add)

    def remove_object(self, pos: Tuple[int, int]):
        """移除单个物体（箱子到达目标）。"""
        if pos in self._object_positions:
            self._remove_single_object(pos)
            self._object_positions.discard(pos)
            self._rebuild_affected({pos})

    def move_object(self, old_pos: Tuple[int, int], new_pos: Tuple[int, int]):
        """移动单个物体。"""
        self._remove_single_object(old_pos)
        self._add_single_object(new_pos)
        self._object_positions.discard(old_pos)
        self._object_positions.add(new_pos)
        self._rebuild_affected({old_pos, new_pos})

    def get_waypoints(self, start: Tuple[int, int],
                      target: Tuple[int, int],
                      obstacles: Set[Tuple[int, int]],
                      object_world_positions: Dict[Tuple[int, int],
                                                    Tuple[float, float]] = None
                      ) -> Optional[List[Tuple[int, int]]]:
        """
        统一混合图 Dijkstra。

        三种边混合:
          捷径边: 规避节点间安全连线，代价=欧氏距离（若两端在同一软障碍区则禁用）
          软区格线: 4邻接，两端在软障碍区，代价=1
          开放格线: 4邻接，至少一端不在软障碍区，代价=10

        obstacles:             当前全部障碍物（墙 + 所有物体格点位置）
        object_world_positions: 物体的实际世界坐标（浮点像素），用于精确计算碰撞盒覆盖格。
                               不传则默认物体在格心上。
        若失败返回 None，调用方回退 car_bfs。
        """
        w, h = self.width, self.height

        # ---- 扩展物体障碍物（基于实际连续位置）----
        expanded_obs = set(obstacles)
        for ox, oy in obstacles:
            if (ox, oy) in self._walls:
                continue
            # 物体的世界像素坐标：用传入的实际位置，否则取格心
            if object_world_positions and (ox, oy) in object_world_positions:
                wx, wy = object_world_positions[(ox, oy)]
            else:
                wx, wy = ox * 50 + 25, oy * 50 + 25  # 格心
            gx0, gx1 = (wx - 25) // 50, (wx + 24) // 50
            gy0, gy1 = (wy - 25) // 50, (wy + 24) // 50
            for gx in range(int(gx0), int(gx1) + 1):
                for gy in range(int(gy0), int(gy1) + 1):
                    if 0 <= gx < w and 0 <= gy < h and (gx, gy) not in self._walls:
                        expanded_obs.add((gx, gy))

        # ---- 规避节点（用扩展后的障碍物）----
        avoidance_nodes = find_avoidance_nodes(expanded_obs, w, h)

        # ---- 软障碍区 = 物体（原始格点）的 8 邻域（不含 obstacles 自身）----
        soft_zone: Set[Tuple[int, int]] = set()
        for ox, oy in obstacles:
            if (ox, oy) in self._walls:
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = ox + dx, oy + dy
                    if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in obstacles:
                        soft_zone.add((nx, ny))

        # ---- 临时将 start 加入规避节点 ----
        start_is_node = start in avoidance_nodes
        if not start_is_node:
            avoidance_nodes.add(start)

        # ---- 收集 walkable 格点（用原始 obstacles，车需要能走进软障碍区）----
        walkable = [(x, y) for y in range(h) for x in range(w)
                    if (x, y) not in obstacles]

        # ---- 构建邻接表 ----
        adj: Dict[Tuple[int, int], List[Tuple[Tuple[int, int], float]]] = {}
        for p in walkable:
            adj[p] = []

        # 捷径边: 规避节点间安全连线（无距离过滤）
        node_list = list(avoidance_nodes)
        for i, n1 in enumerate(node_list):
            for j in range(i + 1, len(node_list)):
                n2 = node_list[j]
                # 两端在同一软障碍区 → 禁用捷径（unfree）
                if n1 in soft_zone and n2 in soft_zone:
                    in_same = False
                    for ox, oy in obstacles:
                        if (ox, oy) in self._walls:
                            continue
                        if (abs(n1[0] - ox) <= 1 and abs(n1[1] - oy) <= 1 and
                            abs(n2[0] - ox) <= 1 and abs(n2[1] - oy) <= 1):
                            in_same = True
                            break
                    if in_same:
                        continue
                if check_safe_connection(n1, n2, expanded_obs, avoidance_nodes, w, h):
                    d = math.hypot(n2[0] - n1[0], n2[1] - n1[1])
                    adj[n1].append((n2, d))
                    adj[n2].append((n1, d))

        # 格线边: 4邻接
        for px, py in walkable:
            p = (px, py)
            in_soft = p in soft_zone
            for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                np = (px + dx, py + dy)
                if np not in adj:
                    continue
                np_soft = np in soft_zone
                cost = 1.5  # 统一格线代价，让斜线（≈1.41）天然更优
                adj[p].append((np, cost))

        # 还原
        if not start_is_node:
            avoidance_nodes.discard(start)

        return composite_dijkstra(start, target, obstacles, adj, w, h)

    # -----------------------------------------------------------------
    # 内部
    # -----------------------------------------------------------------

    def _rebuild_full(self):
        """全量重算规避节点和安全连线（开局时调用一次）。"""
        self.avoidance_nodes = set(self._wall_avoidance_nodes)
        # 物体规避节点
        for obj in self._object_positions:
            obj_nodes = find_avoidance_nodes({obj}, self.width, self.height)
            # 去掉与墙重叠的
            for n in obj_nodes:
                if n not in self._walls:
                    self.avoidance_nodes.add(n)

        self._all_obstacles = self._walls | self._object_positions
        self._avoidance_adj = _build_avoidance_adj(
            self.avoidance_nodes, self._all_obstacles, self.width, self.height)

    def _rebuild_affected(self, changed_positions: Set[Tuple[int, int]]):
        """
        增量重算：只更新受 changed_positions 附近影响的规避节点和连线。
        实际实现为全量重算（地图小，开销可忽略），保留接口供后续优化。
        """
        # 16×12 地图全量重算 < 10ms，直接偷懒全量
        self._rebuild_full()

    def _add_single_object(self, pos: Tuple[int, int]):
        """在障碍物集合中添加一个物体格点。"""
        self._all_obstacles.add(pos)

    def _remove_single_object(self, pos: Tuple[int, int]):
        """从障碍物集合中移除一个物体格点。"""
        self._all_obstacles.discard(pos)
