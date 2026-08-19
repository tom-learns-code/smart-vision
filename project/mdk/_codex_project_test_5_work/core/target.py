"""
目标点类 - 箱子推动到目标点时，两者一起消失
"""
import pygame
from .game_object import GameObject


class Target(GameObject):
    """目标点 - 当箱子到达时触发"""

    def __init__(self, x, y, size, color):
        super().__init__(x, y, size, color)
        self.type = "target"
        self.is_active = True  # 目标点是否活跃（未被收集）

    def get_state(self):
        """获取目标点状态供算法使用"""
        return {
            'x': self.pos.x,
            'y': self.pos.y,
            'theta': 0,
            'size': self.size,
            'type': self.type,
            'is_active': self.is_active
        }

    def draw(self, surface):
        """绘制目标点（含ID标签）"""
        if not self.is_active:
            return

        # 绘制实心正方形（与箱子大小相同）
        rect = pygame.Rect(
            self.pos.x - self.size / 2,
            self.pos.y - self.size / 2,
            self.size,
            self.size
        )

        # 半透明填充
        fill_surface = pygame.Surface((self.size, self.size), pygame.SRCALPHA)
        fill_color = (self.color[0], self.color[1], self.color[2], 100)  # 半透明
        fill_surface.fill(fill_color)
        surface.blit(fill_surface, rect)

        # 绘制边框
        pygame.draw.rect(surface, self.color, rect, 3, border_radius=3)

        # ID标签
        if self.label_id is not None and self.label_id >= 0:
            try:
                from .font_manager import FontManager
                font = FontManager.get_font(14)
                if self.discovered_id:
                    text = str(self.label_id)
                    color = (255, 255, 255)
                else:
                    text = "?"
                    color = (150, 150, 150)
                txt = font.render(text, True, color)
                surface.blit(txt, (self.pos.x - txt.get_width() // 2,
                                   self.pos.y - self.size // 2 - 18))
            except Exception:
                pass
