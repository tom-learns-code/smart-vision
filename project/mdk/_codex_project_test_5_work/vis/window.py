"""
可视化窗口

打开游戏窗口显示完整游戏画面和算法标记
"""

import os
import json
from typing import Dict, Any, Optional

_MARKER_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "algorithm_markers.json")


def show_visualization(map_data: Dict, markers_data: Optional[Dict] = None) -> None:
    """显示可视化窗口（阻塞，直到用户退出 ESC）"""
    import pygame
    from core.font_manager import FontManager
    from vis.renderer import AlgorithmVisualizer
    from core.config import SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_BG
    from core.game_factory import GameFactory
    from core.physics import Vector2
    from core.wall import Wall
    from core.car import Car
    from core.bomb import Bomb
    from core.target import Target

    pygame.init()
    pygame.display.set_caption("算法可视化 - 多炸弹推箱子")
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()
    font = FontManager.get_font(16)

    cell_size = 50
    map_offset_x = (SCREEN_WIDTH - 16 * cell_size) // 2
    map_offset_y = (SCREEN_HEIGHT - 12 * cell_size) // 2

    def cell_to_screen(cx, cy):
        return (map_offset_x + cx * cell_size + cell_size // 2,
                map_offset_y + cy * cell_size + cell_size // 2)

    # 创建游戏对象
    car_pos = map_data.get('car_start', (1, 1))
    sp = cell_to_screen(car_pos[0], car_pos[1])
    car = GameFactory.create_car(sp[0], sp[1])
    car.pos = Vector2(sp[0], sp[1])
    car.prev_pos = Vector2(sp[0], sp[1])

    boxes = []
    for bp in map_data.get('boxes_start', []):
        sp = cell_to_screen(bp[0], bp[1])
        box = GameFactory.create_box()
        box.pos = Vector2(sp[0], sp[1])
        box.prev_pos = Vector2(sp[0], sp[1])
        boxes.append(box)

    bombs = []
    for bp in map_data.get('bombs', []):
        sp = cell_to_screen(bp[0], bp[1])
        bomb = GameFactory.create_bomb()
        bomb.pos = Vector2(sp[0], sp[1])
        bomb.prev_pos = Vector2(sp[0], sp[1])
        bombs.append(bomb)

    targets = []
    for gp in map_data.get('goals', []):
        sp = cell_to_screen(gp[0], gp[1])
        target = GameFactory.create_target()
        target.pos = Vector2(sp[0], sp[1])
        targets.append(target)

    walls = []
    for wp in map_data.get('walls', []):
        sp = cell_to_screen(wp[0], wp[1])
        is_boundary = (wp[0] == 0 or wp[0] == 15 or wp[1] == 0 or wp[1] == 11)
        wall = Wall(sp[0], sp[1], 50, 50, (80, 80, 80), is_indestructible=is_boundary)
        walls.append(wall)

    boundary_walls = GameFactory.create_boundary_walls()

    if markers_data is not None:
        try:
            with open(_MARKER_FILE, 'w', encoding='utf-8') as f:
                json.dump(markers_data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[可视化] 保存标记数据失败: {e}")

    visualizer = AlgorithmVisualizer(_MARKER_FILE)
    visualizer.load_markers()
    print("[可视化] 打开窗口，按 ESC 退出")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    visualizer.load_markers()
                    print("[可视化] 已刷新标记")

        screen.fill(COLOR_BG)
        map_rect = pygame.Rect(map_offset_x - 5, map_offset_y - 5,
                               16 * cell_size + 10, 12 * cell_size + 10)
        pygame.draw.rect(screen, (40, 40, 45), map_rect)

        for x in range(17):
            sx = map_offset_x + x * cell_size
            pygame.draw.line(screen, (60, 60, 70), (sx, map_offset_y),
                           (sx, map_offset_y + 12 * cell_size), 1)
        for y in range(13):
            sy = map_offset_y + y * cell_size
            pygame.draw.line(screen, (60, 60, 70), (map_offset_x, sy),
                           (map_offset_x + 16 * cell_size, sy), 1)

        for x in range(16):
            text = font.render(str(x), True, (150, 150, 150))
            text_rect = text.get_rect(center=(map_offset_x + x * cell_size + 25, map_offset_y - 15))
            screen.blit(text, text_rect)
        for y in range(12):
            text = font.render(str(y), True, (150, 150, 150))
            text_rect = text.get_rect(center=(map_offset_x - 20, map_offset_y + y * cell_size + 25))
            screen.blit(text, text_rect)

        for wall in boundary_walls + walls:
            wall.draw(screen)
        for t in targets:
            t.draw(screen)
        for box in boxes:
            box.draw(screen)
        for bomb in bombs:
            if not bomb.is_exploded:
                bomb.draw(screen)
        car.draw(screen, show_debug=False)

        title = font.render("算法可视化 - 多炸弹推箱子", True, (255, 255, 255))
        screen.blit(title, (20, 15))
        pygame.draw.line(screen, (100, 100, 100), (20, 40), (850, 40), 1)

        hints = ["ESC: 退出", "R: 刷新标记"]
        for i, hint in enumerate(hints):
            text = font.render(hint, True, (150, 150, 150))
            screen.blit(text, (20 + i * 150, 620))

        visualizer.render(screen, map_offset_x, map_offset_y)
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    print("[可视化] 窗口已关闭")


class VisualizationWindow:
    """可视化窗口管理器（单例兼容）"""
    _instance: Optional['VisualizationWindow'] = None

    @classmethod
    def get_instance(cls) -> 'VisualizationWindow':
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
