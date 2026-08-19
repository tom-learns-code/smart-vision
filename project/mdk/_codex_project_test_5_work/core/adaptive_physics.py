"""
自适应物理系统 - 帧率自适应子步和空间分区
"""
from collections import defaultdict


class AdaptivePhysics:
    """自适应物理系统"""

    def __init__(self, base_substeps=10, grid_size=64):
        self.base_substeps = base_substeps
        self.grid_size = grid_size
        self.spatial_grid = {}

    def adaptive_substeps(self, *objects):
        """
        帧率自适应子步（优化版）
        基于速度动态调整子步数，不受物体数量影响
        """
        max_speed = 0
        for obj in objects:
            if hasattr(obj, 'velocity'):
                max_speed = max(max_speed, obj.velocity.length())

        # 速度越快，子步越多，但有上限
        # 不再基于物体数量减少子步，保证稳定性
        speed_based = max(3, min(self.base_substeps, int(max_speed / 50)))

        return speed_based

    def update_spatial_grid(self, objects, walls):
        """更新空间哈希网格"""
        self.spatial_grid = defaultdict(list)

        # 处理动态物体
        for obj in objects:
            self._add_object_to_grid(obj)

        # 处理墙体
        for wall in walls:
            self._add_object_to_grid(wall)

        return self.spatial_grid

    def _add_object_to_grid(self, obj):
        """将物体添加到空间网格"""
        rect = obj.rect
        grid_x_start = int(rect.left / self.grid_size)
        grid_x_end = int(rect.right / self.grid_size)
        grid_y_start = int(rect.top / self.grid_size)
        grid_y_end = int(rect.bottom / self.grid_size)

        for x in range(grid_x_start, grid_x_end + 1):
            for y in range(grid_y_start, grid_y_end + 1):
                self.spatial_grid[(x, y)].append(obj)

    def optimized_collision_detection(self, obj):
        """
        惰性碰撞检测
        只检测相邻网格内的物体
        """
        potential = set()
        rect = obj.rect
        obj_grid_x = int(rect.centerx / self.grid_size)
        obj_grid_y = int(rect.centery / self.grid_size)

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                key = (obj_grid_x + dx, obj_grid_y + dy)
                potential.update(self.spatial_grid.get(key, []))

        return [o for o in potential if o != obj]
