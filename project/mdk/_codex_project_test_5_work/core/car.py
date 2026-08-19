"""
车辆类 - 可推物体的玩家角色
"""
import pygame
import math
from .physics import Vector2


class Car:
    """车辆：可自由旋转和平移的推动者"""

    def __init__(self, x, y, size, color):
        self.pos = Vector2(x, y)
        self.prev_pos = Vector2(x, y)
        self.velocity = Vector2(0, 0)
        self.size = size
        self.color = color
        self.target_velocity = Vector2(0, 0)
        self.angle = 0  # 角度（度）
        self.angular_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.mass = size * size / 800.0  # 车质量
        self.inertia = (size ** 2) / 12.0 * self.mass  # 转动惯量
        self.type = "car"
        self.is_static = False

        # 速度平滑滤波
        self.velocity_history = []  # 速度历史，用于平滑滤波
        self.smoothing_window = 5  # 平滑窗口大小

    def predict_movement(self, dt):
        """预测下一帧位置"""
        return self.pos + self.velocity * dt, self.angle + self.angular_velocity * dt

    def get_state(self):
        """获取车辆状态供算法使用"""
        return {
            'x': self.pos.x,
            'y': self.pos.y,
            'theta': math.radians(self.angle),
            'size': self.size,
            'type': self.type
        }

    def update(self, dt):
        """更新位置和旋转"""
        self.prev_pos = self.pos.copy()
        self.prev_angle = self.angle  # 保存前一个角度用于CCD

        # 速度插值 - 提高插值因子以快速达到目标速度
        lerp_factor = 0.8
        self.velocity = self.velocity.lerp(self.target_velocity, lerp_factor)

        # 不再使用速度平滑滤波，让速度直接响应目标值

        self.pos += self.velocity * dt

        # 角速度插值
        self.angular_velocity = self.angular_velocity + (
            self.target_angular_velocity - self.angular_velocity
        ) * lerp_factor
        self.angle += self.angular_velocity * dt

    def _smooth_velocity(self, velocity: Vector2) -> Vector2:
        """速度平滑滤波 - 使用移动平均减少抖动"""
        self.velocity_history.append(velocity)
        if len(self.velocity_history) > self.smoothing_window:
            self.velocity_history.pop(0)

        # 计算平均速度
        avg_x = sum(v.x for v in self.velocity_history) / len(self.velocity_history)
        avg_y = sum(v.y for v in self.velocity_history) / len(self.velocity_history)

        return Vector2(avg_x, avg_y)

    def get_rotated_vertices(self, pos=None, angle=None):
        """获取旋转后的顶点"""
        if pos is None:
            pos = self.pos
        if angle is None:
            angle = self.angle

        half_size = self.size / 2
        angle_rad = math.radians(angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # 未旋转的顶点
        corners = [
            Vector2(-half_size, -half_size),
            Vector2(half_size, -half_size),
            Vector2(half_size, half_size),
            Vector2(-half_size, half_size)
        ]

        # 旋转并平移
        rotated = []
        for corner in corners:
            rotated_x = corner.x * cos_a - corner.y * sin_a
            rotated_y = corner.x * sin_a + corner.y * cos_a
            rotated.append(Vector2(pos.x + rotated_x, pos.y + rotated_y))

        return rotated

    @property
    def vertices(self):
        """获取顶点（用于碰撞检测）"""
        return self.get_rotated_vertices()

    @property
    def rect(self):
        """获取旋转后的轴对齐包围盒（用于快速预检测）"""
        # 获取旋转后的顶点
        verts = self.get_rotated_vertices()

        # 计算包围盒
        min_x = min(v.x for v in verts)
        max_x = max(v.x for v in verts)
        min_y = min(v.y for v in verts)
        max_y = max(v.y for v in verts)

        return pygame.Rect(
            min_x,
            min_y,
            max_x - min_x,
            max_y - min_y
        )

    @property
    def center(self):
        """获取中心点"""
        return self.pos

    def draw(self, surface, show_debug=False):
        """绘制车辆（带方向指示）"""
        vertices = self.get_rotated_vertices()
        points = [(v.x, v.y) for v in vertices]

        # 绘制主体
        pygame.draw.polygon(surface, self.color, points)
        pygame.draw.polygon(surface, (255, 255, 255), points, 2)

        # 绘制方向箭头
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
