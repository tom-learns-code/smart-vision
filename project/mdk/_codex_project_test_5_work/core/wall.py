"""
墙体类 - 静态碰撞对象
"""
import pygame
from .game_object import GameObject


class Wall(GameObject):
    """墙体对象：完全静态，质量无限大，绝对不可推动"""

    def __init__(self, x, y, width, height, color=(100, 100, 100), is_boundary=False, is_indestructible=False):
        super().__init__(x, y, width, color)
        self.width = width
        self.height = height
        self.type = "wall"
        self.is_boundary = is_boundary  # 是否为边界墙
        self.is_indestructible = is_indestructible  # 是否不可破坏
        self.mass = float('inf')  # 无限质量
        self.is_static = True  # 标记为静态
        self.velocity = pygame.Vector2(0, 0)  # 永不动
        self.target_velocity = pygame.Vector2(0, 0)  # 无目标速度

    def update(self, dt):
        """墙体完全不更新任何状态"""
        pass

    def predict_movement(self, dt):
        """墙体永远不移动"""
        return self.pos, 0

    def get_state(self):
        """获取墙体状态供算法使用"""
        return {
            'x': self.pos.x,
            'y': self.pos.y,
            'theta': 0,
            'width': self.width,
            'height': self.height,
            'type': self.type,
            'is_boundary': self.is_boundary,
            'is_indestructible': self.is_indestructible
        }

    @property
    def center(self):
        """获取中心点"""
        return self.pos

    def get_vertices(self):
        """获取矩形的四个顶点"""
        half_w = self.width / 2
        half_h = self.height / 2
        return [
            pygame.Vector2(self.pos.x - half_w, self.pos.y - half_h),
            pygame.Vector2(self.pos.x + half_w, self.pos.y - half_h),
            pygame.Vector2(self.pos.x + half_w, self.pos.y + half_h),
            pygame.Vector2(self.pos.x - half_w, self.pos.y + half_h)
        ]

    @property
    def rect(self):
        """获取pygame.Rect用于快速碰撞检测"""
        half_w = self.width / 2
        half_h = self.height / 2
        return pygame.Rect(
            self.pos.x - half_w,
            self.pos.y - half_h,
            self.width,
            self.height
        )

    def draw(self, surface):
        """绘制墙体"""
        pygame.draw.rect(surface, self.color, self.rect)
        # 边界墙和普通墙都用相同的边框
        pygame.draw.rect(surface, (150, 150, 150), self.rect, 2)
