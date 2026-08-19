"""
炸弹类 - 可被推入墙中并炸毁墙体
"""
import pygame
from .physics import Vector2
from .config import BOMB_EXPLOSION_DELAY


class Bomb:
    """炸弹对象：与箱子外形相同，可穿墙爆炸"""

    def __init__(self, x, y, size, color=(255, 100, 100)):
        self.pos = Vector2(x, y)
        self.prev_pos = Vector2(x, y)
        self.velocity = Vector2(0, 0)
        self.size = size
        self.color = color
        self.target_velocity = Vector2(0, 0)
        self.mass = size * size / 1000.0
        self.type = "bomb"
        self.is_static = False
        self.is_exploded = False
        self.friction_coefficient = 0.15
        self.is_in_wall = False
        self.in_wall_timer = 0.0
        self.explosion_delay = BOMB_EXPLOSION_DELAY
        self.is_being_pushed = False  # 是否正在被车推动

    def update(self, dt):
        """更新位置"""
        self.prev_pos = self.pos.copy()

        # 如果正在被推动，不应用摩擦力
        if not self.is_being_pushed:
            self.velocity *= (1.0 - self.friction_coefficient)

        # 炸弹只受碰撞影响，不使用目标速度插值
        self.pos += self.velocity * dt

        # 更新进入墙体的计时器
        if self.is_in_wall:
            self.in_wall_timer += dt
        else:
            self.in_wall_timer = 0.0

        # 重置推动状态，每帧需要重新检测
        self.is_being_pushed = False

    def get_state(self):
        """获取炸弹状态供算法使用"""
        return {
            'x': self.pos.x,
            'y': self.pos.y,
            'theta': 0,
            'size': self.size,
            'type': self.type,
            'is_exploded': self.is_exploded
        }

    def get_vertices(self):
        """获取正方形的四个顶点"""
        half_size = self.size / 2
        return [
            Vector2(self.pos.x - half_size, self.pos.y - half_size),
            Vector2(self.pos.x + half_size, self.pos.y - half_size),
            Vector2(self.pos.x + half_size, self.pos.y + half_size),
            Vector2(self.pos.x - half_size, self.pos.y + half_size)
        ]

    @property
    def rect(self):
        """获取pygame.Rect用于快速碰撞检测"""
        half_size = self.size / 2
        return pygame.Rect(
            self.pos.x - half_size,
            self.pos.y - half_size,
            self.size,
            self.size
        )

    @property
    def center(self):
        """获取中心点"""
        return self.pos

    @property
    def vertices(self):
        """获取顶点"""
        return self.get_vertices()

    def draw(self, surface):
        """绘制炸弹（与箱子相同的正方形）"""
        vertices = [(v.x, v.y) for v in self.get_vertices()]

        # 绘制主体（红色）
        pygame.draw.polygon(surface, self.color, vertices)
        pygame.draw.polygon(surface, (255, 255, 255), vertices, 2)

        # 如果在墙体中，显示爆炸倒计时
        if self.is_in_wall and not self.is_exploded:
            remaining_time = max(0, self.explosion_delay - self.in_wall_timer)
            from core.font_manager import FontManager
            font = FontManager.get_font(16)
            timer_text = font.render(f"{remaining_time:.1f}", True, (255, 255, 0))
            text_rect = timer_text.get_rect(center=(int(self.pos.x), int(self.pos.y)))
            surface.blit(timer_text, text_rect)
