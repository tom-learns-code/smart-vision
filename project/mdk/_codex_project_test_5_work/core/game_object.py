"""
游戏对象类
"""
import pygame
import math
from .physics import Vector2


class GameObject:
    """基础游戏对象类"""

    def __init__(self, x, y, size, color):
        self.pos = Vector2(x, y)
        self.prev_pos = Vector2(x, y)
        self.velocity = Vector2(0, 0)
        self.size = size
        self.color = color
        self.target_velocity = Vector2(0, 0)
        self.mass = size * size / 1000.0
        self.type = "box"
        self.friction_coefficient = 0.08
        self.is_static = False
        self.is_being_pushed = False  # 是否正在被车推动

        # ID识别系统
        self.label_id: int = -1           # 真实ID，-1=无ID（无识别模式）
        self.discovered_id: bool = False  # 是否已被车扫描发现

    def predict_movement(self, dt):
        """预测下一帧位置"""
        return self.pos + self.velocity * dt

    def get_state(self):
        """获取游戏对象状态供算法使用"""
        return {
            'x': self.pos.x,
            'y': self.pos.y,
            'theta': 0,
            'size': self.size,
            'type': self.type,
            'label_id': self.label_id,
            'discovered_id': self.discovered_id,
        }

    def update(self, dt):
        """更新位置"""
        self.prev_pos = self.pos.copy()
        # 如果正在被推动，不应用摩擦力
        if not self.is_being_pushed:
            self.velocity *= (1.0 - self.friction_coefficient)
        self.pos += self.velocity * dt
        # 重置推动状态，每帧需要重新检测
        self.is_being_pushed = False

    def get_vertices(self):
        """获取正方形的四个顶点（未旋转）"""
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
        """绘制对象（含ID标签）"""
        vertices = [(v.x, v.y) for v in self.get_vertices()]
        pygame.draw.polygon(surface, self.color, vertices)
        pygame.draw.polygon(surface, (255, 255, 255), vertices, 2)

        # ID标签（箱子和炸弹类型显示）
        if self.type in ('box', 'bomb') and self.label_id is not None and self.label_id >= 0:
            try:
                from .font_manager import FontManager
                font = FontManager.get_font(14)
                if self.discovered_id:
                    text = str(self.label_id)
                    color = (255, 255, 255)  # 白色数字
                else:
                    text = "?"
                    color = (150, 150, 150)  # 灰色问号
                txt = font.render(text, True, color)
                surface.blit(txt, (self.pos.x - txt.get_width() // 2,
                                   self.pos.y - self.size // 2 - 18))
            except Exception:
                pass


class Player(GameObject):
    """玩家方块：可旋转和平移"""

    def __init__(self, x, y, size, color):
        super().__init__(x, y, size, color)
        self.angle = 0
        self.angular_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.inertia = (size ** 2) / 12.0 * self.mass
        self.type = "player"
        self.prev_angle = 0

    def update(self, dt):
        """更新位置和旋转"""
        self.prev_pos = self.pos.copy()
        
        lerp_factor = 0.15
        self.velocity = self.velocity.lerp(self.target_velocity, lerp_factor)
        self.pos += self.velocity * dt

        self.angular_velocity = self.angular_velocity + (
            self.target_angular_velocity - self.angular_velocity
        ) * lerp_factor
        self.angle += self.angular_velocity * dt

    def get_rotated_vertices(self, pos=None):
        """获取旋转后的顶点"""
        if pos is None:
            pos = self.pos

        half_size = self.size / 2
        angle_rad = math.radians(self.angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        corners = [
            Vector2(-half_size, -half_size),
            Vector2(half_size, -half_size),
            Vector2(half_size, half_size),
            Vector2(-half_size, half_size)
        ]

        rotated = []
        for corner in corners:
            rotated_x = corner.x * cos_a - corner.y * sin_a
            rotated_y = corner.x * sin_a + corner.y * cos_a
            rotated.append(Vector2(pos.x + rotated_x, pos.y + rotated_y))

        return rotated

    def get_vertices(self):
        """获取旋转后的顶点"""
        return self.get_rotated_vertices()

    def draw(self, surface, show_debug=False):
        """绘制玩家方块（带旋转方向箭头）"""
        vertices = self.get_rotated_vertices()
        points = [(v.x, v.y) for v in vertices]

        # 绘制主体
        pygame.draw.polygon(surface, self.color, points)
        pygame.draw.polygon(surface, (255, 255, 255), points, 2)

        # 绘制旋转方向箭头
        center = self.pos
        arrow_length = self.size * 0.4
        angle_rad = math.radians(self.angle)

        arrow_end = Vector2(
            center.x + arrow_length * math.cos(angle_rad),
            center.y + arrow_length * math.sin(angle_rad)
        )

        pygame.draw.line(surface, (255, 255, 255),
                        (center.x, center.y),
                        (arrow_end.x, arrow_end.y), 3)

        # 箭头头部
        arrow_angle1 = angle_rad + math.radians(150)
        arrow_angle2 = angle_rad - math.radians(150)
        arrow_size = 10

        arrow_p1 = Vector2(
            arrow_end.x + arrow_size * math.cos(arrow_angle1),
            arrow_end.y + arrow_size * math.sin(arrow_angle1)
        )
        arrow_p2 = Vector2(
            arrow_end.x + arrow_size * math.cos(arrow_angle2),
            arrow_end.y + arrow_size * math.sin(arrow_angle2)
        )

        pygame.draw.polygon(surface, (255, 255, 255),
                           [(arrow_end.x, arrow_end.y),
                            (arrow_p1.x, arrow_p1.y),
                            (arrow_p2.x, arrow_p2.y)])

        if show_debug:
            # 显示速度向量
            if self.velocity.length() > 0:
                vel_end = center + self.velocity * 0.1
                pygame.draw.line(surface, (255, 255, 0),
                               (center.x, center.y),
                               (vel_end.x, vel_end.y), 2)
