"""
地图预分析模块（阶段0）

在推箱搜索之前对地图做一次性分析，为后续阶段提供：
  1. 连通分量划分 → 快速判断两格点是否"绝对不可达"
  2. 格点分类 → 标注 dead_end/channel/corner/open
  3. 管道分段 → 识别连续狭窄通道及其出口
  4. 瓶颈识别 → 箱子放置后会切断连通的位置

设计目标：全部计算 O(W×H)，不包含 BFS 搜索。
"""

from enum import IntEnum
from collections import deque
from typing import List, Tuple, Set, Dict, Optional


# =============================================================================
# 格点分类
# =============================================================================

class CellType(IntEnum):
    """格点类型，按出口数分类"""
    DEAD = 0        # 无出口（不应出现在可行走格中）
    DEAD_END = 1    # 1 个出口 — 死胡同，车进去出不来
    CHANNEL = 2     # 2 个出口（对向）— 狭窄通道，只能直走
    CORNER = 3      # 2 个出口（L 形）— 转角
    OPEN = 4        # 3+ 出口 — 开阔区域，车可以自由移动


# 四个方向
_NEIGHBORS = [(0, -1), (0, 1), (-1, 0), (1, 0)]


# =============================================================================
# 主类
# =============================================================================

class MapPreprocess:
    """地图预分析结果"""

    def __init__(self, walls: Set[Tuple[int, int]],
                 width: int, height: int):
        self.walls = walls
        self.width = width
        self.height = height

        # 输出数据
        self.component: List[List[int]] = []        # component[y][x] = 分量ID
        self.num_components: int = 0
        self.component_cells: List[Set[Tuple[int, int]]] = []  # 每个分量的格点集

        self.cell_types: List[List[CellType]] = []  # cell_types[y][x]
        self.choke_points: Set[Tuple[int, int]] = set()
        self.channels: List[Dict] = []               # 管道段列表
        self.distances: Dict[Tuple[int, int],
                             Dict[Tuple[int, int], int]] = {}  # 全对最短距离

        # 执行分析
        self._analyze()

    # =========================================================================
    # 总入口
    # =========================================================================

    def _analyze(self):
        self._compute_components()
        self._classify_cells()
        self._segment_channels()
        self._find_choke_points()
        self._compute_distances()

    # =========================================================================
    # 1. 连通分量分析
    # =========================================================================

    def _compute_components(self):
        """BFS 全图划分连通分量"""
        w, h = self.width, self.height
        self.component = [[-1] * w for _ in range(h)]
        self.num_components = 0
        self.component_cells = []

        for y in range(h):
            for x in range(w):
                if (x, y) in self.walls:
                    continue
                if self.component[y][x] >= 0:
                    continue

                # 新分量：BFS
                cid = self.num_components
                self.num_components += 1
                cells = set()
                q = deque([(x, y)])
                self.component[y][x] = cid
                cells.add((x, y))

                while q:
                    cx, cy = q.popleft()
                    for dx, dy in _NEIGHBORS:
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            if (nx, ny) in self.walls:
                                continue
                            if self.component[ny][nx] >= 0:
                                continue
                            self.component[ny][nx] = cid
                            cells.add((nx, ny))
                            q.append((nx, ny))

                self.component_cells.append(cells)

    # =========================================================================
    # 2. 格点分类
    # =========================================================================

    def _classify_cells(self):
        """对每个可行走格点分类"""
        w, h = self.width, self.height
        self.cell_types = [[CellType.DEAD] * w for _ in range(h)]

        for y in range(h):
            for x in range(w):
                if (x, y) in self.walls:
                    continue
                self.cell_types[y][x] = self._classify_one(x, y)

    def _classify_one(self, x: int, y: int) -> CellType:
        """分类单个格点"""
        free_dirs = []
        for dx, dy in _NEIGHBORS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if (nx, ny) not in self.walls:
                    free_dirs.append((dx, dy))

        n = len(free_dirs)
        if n == 0:
            return CellType.DEAD
        if n == 1:
            return CellType.DEAD_END
        if n >= 3:
            return CellType.OPEN
        # n == 2: 判断是对向（CHANNEL）还是 L 形（CORNER）
        d1, d2 = free_dirs
        if d1[0] + d2[0] == 0 and d1[1] + d2[1] == 0:
            return CellType.CHANNEL  # 对向：水平和垂直都抵消
        # 另一种对向：比如 (0,1) 和 (0,-1)
        return CellType.CORNER  # L 形

    # =========================================================================
    # 3. 管道分段
    # =========================================================================

    def _segment_channels(self):
        """
        BFS 遍历所有 CHANNEL/CORNER 格点，聚合为管道段。

        每个管道段记录：
          - tiles: 包含的格点集
          - orientation: 'HORIZONTAL' | 'VERTICAL' | 'L'
          - endpoints: 两端出口格点（管道外的相邻格点）
          - length: 格点数
        """
        w, h = self.width, self.height
        visited = set()
        self.channels = []

        for y in range(h):
            for x in range(w):
                if (x, y) in self.walls:
                    continue
                if (x, y) in visited:
                    continue
                ct = self.cell_types[y][x]
                if ct not in (CellType.CHANNEL, CellType.CORNER):
                    continue

                # 从这个 CHANNEL/CORNER 开始 BFS 聚合
                seg_tiles = set()
                q = deque([(x, y)])
                seg_tiles.add((x, y))
                visited.add((x, y))

                while q:
                    cx, cy = q.popleft()
                    for dx, dy in _NEIGHBORS:
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            if (nx, ny) in self.walls:
                                continue
                            if (nx, ny) in visited:
                                continue
                            nct = self.cell_types[ny][nx]
                            if nct in (CellType.CHANNEL, CellType.CORNER):
                                visited.add((nx, ny))
                                seg_tiles.add((nx, ny))
                                q.append((nx, ny))

                # 分析管道方向
                orientation = self._channel_orientation(seg_tiles)
                endpoints = self._find_endpoints(seg_tiles)

                self.channels.append({
                    'tiles': seg_tiles,
                    'orientation': orientation,
                    'endpoints': endpoints,
                    'length': len(seg_tiles),
                })

    def _channel_orientation(self, tiles: Set[Tuple[int, int]]) -> str:
        """判断管道方向"""
        if len(tiles) <= 1:
            return 'SINGLE'
        xs = {t[0] for t in tiles}
        ys = {t[1] for t in tiles}
        if len(xs) == 1:
            return 'VERTICAL'
        if len(ys) == 1:
            return 'HORIZONTAL'
        # 检查是否有 CORNER（L 形）
        for tx, ty in tiles:
            if self.cell_types[ty][tx] == CellType.CORNER:
                return 'L'
        return 'COMPLEX'

    def _find_endpoints(self, tiles: Set[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        找到管道的出口格点（管道边缘相邻的非管道格点）。

        返回 [(出口格点, 连接方向), ...]
        """
        endpoints = []
        for tx, ty in tiles:
            for dx, dy in _NEIGHBORS:
                nx, ny = tx + dx, ty + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) in self.walls:
                        continue
                    if (nx, ny) in tiles:
                        continue
                    # nx,ny 是管道外相邻格点
                    endpoints.append((nx, ny))
        # 去重
        return list(set(endpoints))

    # =========================================================================
    # 4. 瓶颈识别
    # =========================================================================

    def _find_choke_points(self):
        """
        识别瓶颈格点：箱子放在该位置会使连通分量分裂。

        方法：对每个 CHANNEL/CORNER/DEAD_END 格点检查：
          如果它的非墙体邻居格点之间（排除该格点自身后）有不连通的，
          那该格点就是瓶颈。

        优化：只检查管道端点附近的格点（真正的瓶颈通常在管道出口）。
        """
        self.choke_points = set()

        # 收集候选：管道端点相邻的格点 + 管道内的格点
        candidates = set()
        for ch in self.channels:
            for ep in ch['endpoints']:
                candidates.add(ep)
            candidates.update(ch['tiles'])

        if not candidates:
            # 没有管道，检查所有 channel 和 dead_end
            for y in range(self.height):
                for x in range(self.width):
                    ct = self.cell_types[y][x]
                    if ct in (CellType.CHANNEL, CellType.DEAD_END):
                        candidates.add((x, y))

        for cx, cy in candidates:
            if (cx, cy) in self.walls:
                continue
            if self._is_choke(cx, cy):
                self.choke_points.add((cx, cy))

    def _is_choke(self, cx: int, cy: int) -> bool:
        """
        检查 (cx, cy) 是否为瓶颈。

        把该格点当作被箱子占据，看它的邻居们是否还在同一连通分量内。
        如果邻居分裂成多个分量 → 瓶颈。
        """
        # 收集该格点的非墙体邻居
        neighbors = []
        for dx, dy in _NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if (nx, ny) not in self.walls:
                    neighbors.append((nx, ny))

        if len(neighbors) <= 1:
            return False

        # 临时阻塞该格点，从第一个邻居 BFS 看能否到达其他邻居
        blocked = self.walls | {(cx, cy)}
        visited = set()
        q = deque([neighbors[0]])
        visited.add(neighbors[0])

        while q:
            px, py = q.popleft()
            for dx, dy in _NEIGHBORS:
                nx, ny = px + dx, py + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) in blocked:
                        continue
                    if (nx, ny) in visited:
                        continue
                    visited.add((nx, ny))
                    q.append((nx, ny))

        # 检查所有邻居是否都在 visited 中
        for nb in neighbors[1:]:
            if nb not in visited:
                return True
        return False

    # =========================================================================
    # 5. 全对最短路径距离预计算
    # =========================================================================

    def _compute_distances(self):
        """
        对每个可行走格点跑 BFS，预计算到所有其他格点的最短距离。

        复杂度 O(N^2) 其中 N = 可行走格点数。
        16×12 地图约 85 个格点 → ~7200 次操作，<10ms。
        """
        w, h = self.width, self.height
        # 收集所有可行走格点
        walkable = []
        for y in range(h):
            for x in range(w):
                if (x, y) not in self.walls:
                    walkable.append((x, y))

        for sx, sy in walkable:
            # BFS from (sx, sy)
            dist_map = {(sx, sy): 0}
            q = deque([(sx, sy)])
            while q:
                cx, cy = q.popleft()
                d = dist_map[(cx, cy)] + 1
                for dx, dy in _NEIGHBORS:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if (nx, ny) in self.walls:
                            continue
                        if (nx, ny) in dist_map:
                            continue
                        dist_map[(nx, ny)] = d
                        q.append((nx, ny))
            self.distances[(sx, sy)] = dist_map

    # =========================================================================
    # 查询接口
    # =========================================================================

    def distance(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> int:
        """两格点之间的静态 BFS 距离。如果 p1 就是 p2 返回 0。"""
        if p1 == p2:
            return 0
        return self.distances.get(p1, {}).get(p2, 999999)

    def same_component(self, p1: Tuple[int, int],
                       p2: Tuple[int, int]) -> bool:
        """两点是否在同一连通分量"""
        c1 = self.component[p1[1]][p1[0]]
        c2 = self.component[p2[1]][p2[0]]
        return c1 >= 0 and c1 == c2

    def is_choke(self, pos: Tuple[int, int]) -> bool:
        return pos in self.choke_points

    def cell_type(self, pos: Tuple[int, int]) -> CellType:
        return self.cell_types[pos[1]][pos[0]]

    def get_channels_at(self, pos: Tuple[int, int]) -> List[Dict]:
        """返回包含该位置的管道段列表"""
        result = []
        for ch in self.channels:
            if pos in ch['tiles']:
                result.append(ch)
        return result

    # =========================================================================
    # 打印（调试用）
    # =========================================================================

    def print_summary(self):
        """打印分析摘要"""
        type_names = {
            CellType.DEAD: 'D', CellType.DEAD_END: 'E',
            CellType.CHANNEL: 'C', CellType.CORNER: 'N',
            CellType.OPEN: 'O',
        }
        print(f"[预分析] 地图 {self.width}×{self.height}")
        print(f"  连通分量: {self.num_components}")
        print(f"  管道段: {len(self.channels)}")
        print(f"  瓶颈点: {len(self.choke_points)}")
        print(f"  格点分类分布:")
        counts = {t: 0 for t in CellType}
        for y in range(self.height):
            for x in range(self.width):
                if (x, y) not in self.walls:
                    counts[self.cell_types[y][x]] += 1
        for t in CellType:
            print(f"    {t.name}: {counts[t]}")
        print(f"  管道详情:")
        for i, ch in enumerate(self.channels):
            print(f"    CH{i}: {ch['orientation']} len={ch['length']} "
                  f"endpoints={ch['endpoints']}")

    def print_map(self):
        """打印分类地图"""
        type_chars = {
            CellType.DEAD: 'X', CellType.DEAD_END: 'E',
            CellType.CHANNEL: 'c', CellType.CORNER: 'n',
            CellType.OPEN: '.',
        }
        for y in range(self.height):
            line = ''
            for x in range(self.width):
                if (x, y) in self.walls:
                    line += '#'
                elif (x, y) in self.choke_points:
                    line += '!'
                else:
                    line += type_chars.get(self.cell_types[y][x], '?')
            print(f"  {line}")
