"""
编辑模式 - 自由放置箱子和墙体
"""
import pygame
from .font_manager import FontManager


class EditorMode:
    """编辑模式管理器"""

    def __init__(self, screen_width, screen_height, grid_size=50):
        self.enabled = False
        self.grid_size = grid_size
        self.screen_width = screen_width
        self.screen_height = screen_height

        # 编辑状态
        self.current_tool = "box"  # "box", "wall", "bomb", "target"
        self.boxes = []
        self.walls = []
        self.bombs = []
        self.targets = []
        self.car = None

        # UI状态
        self.show_menu = False
        self.buttons = []
        self._init_buttons()

    def _init_buttons(self):
        """初始化编辑模式按钮"""
        button_width = 100
        button_height = 40
        spacing = 10
        start_x = self.screen_width - button_width - 20

        self.buttons = [
            {'rect': pygame.Rect(start_x, 20, button_width, button_height),
             'text': '箱子', 'tool': 'box', 'color': (139, 69, 19)},
            {'rect': pygame.Rect(start_x, 20 + (button_height + spacing), button_width, button_height),
             'text': '墙体', 'tool': 'wall', 'color': (100, 100, 100)},
            {'rect': pygame.Rect(start_x, 20 + 2 * (button_height + spacing), button_width, button_height),
             'text': '炸弹', 'tool': 'bomb', 'color': (255, 100, 100)},
            {'rect': pygame.Rect(start_x, 20 + 3 * (button_height + spacing), button_width, button_height),
             'text': '目标点', 'tool': 'target', 'color': (0, 255, 100)},
            {'rect': pygame.Rect(start_x, 20 + 4 * (button_height + spacing), button_width, button_height),
             'text': '清除所有', 'tool': 'clear', 'color': (200, 50, 50)},
        ]

    def toggle(self):
        """切换编辑模式"""
        self.enabled = not self.enabled
        return self.enabled

    def handle_event(self, event, car, boxes, walls, bombs, targets):
        """处理编辑模式输入事件"""
        if not self.enabled:
            return

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # 左键点击
                mouse_pos = pygame.mouse.get_pos()

                # 检查按钮点击
                for button in self.buttons:
                    if button['rect'].collidepoint(mouse_pos):
                        self._handle_button_click(button['tool'], car, boxes, walls)
                        return

                # 放置物体
                self._place_object(mouse_pos, boxes, walls, bombs, targets)

            elif event.button == 3:  # 右键点击 - 删除物体
                mouse_pos = pygame.mouse.get_pos()
                self._remove_object(mouse_pos, boxes, walls, bombs, targets)

    def _handle_button_click(self, tool, car, boxes, walls):
        """处理按钮点击"""
        if tool == 'box':
            self.current_tool = 'box'
        elif tool == 'wall':
            self.current_tool = 'wall'
        elif tool == 'bomb':
            self.current_tool = 'bomb'
        elif tool == 'target':
            self.current_tool = 'target'
        elif tool == 'clear':
            boxes.clear()
            walls.clear()
            bombs.clear()
            targets.clear()
            # 保留边界墙
            self._create_boundary_walls(walls)

    def _place_object(self, mouse_pos, boxes, walls, bombs, targets):
        """在鼠标位置放置物体"""
        if self.current_tool == 'box':
            # 箱子自由放置（基于鼠标位置）
            from .game_object import GameObject
            from .physics import Vector2
            box = GameObject(mouse_pos[0], mouse_pos[1], 40, (139, 69, 19))
            boxes.append(box)

        elif self.current_tool == 'wall':
            # 墙体对齐到网格
            grid_x = round(mouse_pos[0] / self.grid_size) * self.grid_size
            grid_y = round(mouse_pos[1] / self.grid_size) * self.grid_size

            # 检查是否已经有墙体在这个位置
            from .wall import Wall
            # 墙体大小等于网格大小
            new_wall = Wall(grid_x, grid_y, self.grid_size, self.grid_size, (80, 80, 80))

            # 避免重复放置
            duplicate = False
            for wall in walls:
                if wall.pos == new_wall.pos:
                    duplicate = True
                    break

            if not duplicate:
                walls.append(new_wall)

        elif self.current_tool == 'bomb':
            # 炸弹自由放置（基于鼠标位置）
            from .bomb import Bomb
            from .physics import Vector2
            bomb = Bomb(mouse_pos[0], mouse_pos[1], 40, (255, 100, 100))
            bombs.append(bomb)

        elif self.current_tool == 'target':
            # 目标点自由放置（基于鼠标位置）
            from .target import Target
            from .physics import Vector2
            target = Target(mouse_pos[0], mouse_pos[1], 40, (0, 255, 100))
            targets.append(target)

    def _remove_object(self, mouse_pos, boxes, walls, bombs, targets):
        """删除鼠标位置的物体"""
        # 检查箱子
        for i, box in enumerate(boxes):
            if box.rect.collidepoint(mouse_pos):
                del boxes[i]
                return

        # 检查墙体
        for i, wall in enumerate(walls):
            if wall.rect.collidepoint(mouse_pos):
                del walls[i]
                return

        # 检查炸弹
        for i, bomb in enumerate(bombs):
            if bomb.rect.collidepoint(mouse_pos):
                del bombs[i]
                return

        # 检查目标点
        for i, target in enumerate(targets):
            if target.rect.collidepoint(mouse_pos):
                del targets[i]
                return

    def _create_boundary_walls(self, walls):
        """创建边界墙"""
        wall_thickness = 20
        color = (80, 80, 80)

        walls.append(Wall(self.screen_width / 2, wall_thickness / 2,
                         self.screen_width, wall_thickness, color))
        walls.append(Wall(self.screen_width / 2, self.screen_height - wall_thickness / 2,
                         self.screen_width, wall_thickness, color))
        walls.append(Wall(wall_thickness / 2, self.screen_height / 2,
                         wall_thickness, self.screen_height, color))
        walls.append(Wall(self.screen_width - wall_thickness / 2, self.screen_height / 2,
                         wall_thickness, self.screen_height, color))

    def draw(self, surface):
        """绘制编辑模式UI"""
        if not self.enabled:
            return

        # 绘制网格
        for x in range(0, self.screen_width, self.grid_size):
            pygame.draw.line(surface, (60, 60, 70), (x, 0), (x, self.screen_height), 1)
        for y in range(0, self.screen_height, self.grid_size):
            pygame.draw.line(surface, (60, 60, 70), (0, y), (self.screen_width, y), 1)

        # 绘制按钮
        font = FontManager.get_font(24)
        for button in self.buttons:
            # 按钮背景
            bg_color = button['color']
            if button['tool'] == self.current_tool:
                # 当前选中的工具加亮
                bg_color = (min(bg_color[0] + 50, 255),
                           min(bg_color[1] + 50, 255),
                           min(bg_color[2] + 50, 255))

            pygame.draw.rect(surface, bg_color, button['rect'])
            pygame.draw.rect(surface, (255, 255, 255), button['rect'], 2)

            # 按钮文字
            text = font.render(button['text'], True, (255, 255, 255))
            text_rect = text.get_rect(center=button['rect'].center)
            surface.blit(text, text_rect)

        # 绘制预览
        mouse_pos = pygame.mouse.get_pos()
        preview_color = (255, 255, 255, 100)

        if self.current_tool == 'box':
            # 箱子预览（跟随鼠标，使用OBJECT_SIZE）
            size = 40
            rect = pygame.Rect(mouse_pos[0] - size/2, mouse_pos[1] - size/2, size, size)
            s = pygame.Surface((size, size), pygame.SRCALPHA)
            s.fill((*preview_color[:3], 100))
            surface.blit(s, rect)

        elif self.current_tool == 'wall':
            # 墙体预览（对齐网格，大小等于网格）
            grid_x = round(mouse_pos[0] / self.grid_size) * self.grid_size
            grid_y = round(mouse_pos[1] / self.grid_size) * self.grid_size
            size = self.grid_size
            rect = pygame.Rect(grid_x - size/2, grid_y - size/2, size, size)
            s = pygame.Surface((size, size), pygame.SRCALPHA)
            s.fill((*preview_color[:3], 100))
            surface.blit(s, rect)

        elif self.current_tool == 'bomb':
            # 炸弹预览（跟随鼠标，使用OBJECT_SIZE）
            size = 40
            rect = pygame.Rect(mouse_pos[0] - size/2, mouse_pos[1] - size/2, size, size)
            s = pygame.Surface((size, size), pygame.SRCALPHA)
            s.fill((*preview_color[:3], 100))
            surface.blit(s, rect)

        elif self.current_tool == 'target':
            # 目标点预览（跟随鼠标，使用OBJECT_SIZE）
            size = 40
            rect = pygame.Rect(mouse_pos[0] - size/2, mouse_pos[1] - size/2, size, size)
            s = pygame.Surface((size, size), pygame.SRCALPHA)
            s.fill((0, 255, 100, 100))
            surface.blit(s, rect)

        # 绘制提示文字
        font = FontManager.get_font(28)
        info_text = "编辑模式 - 左键放置 | 右键删除 | TAB退出"
        text = font.render(info_text, True, (255, 255, 255))
        surface.blit(text, (20, 20))

        tool_text = font.render(f"当前工具: {self.current_tool}", True, (255, 200, 100))
        surface.blit(tool_text, (20, 55))
