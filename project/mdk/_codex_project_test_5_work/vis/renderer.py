"""
可视化渲染模块

包含路径绘制和算法标记渲染
"""

import pygame
import math
import json
import os
import colorsys
from typing import List, Dict, Any, Optional, Tuple


# =============================================================================
# 路径绘制
# =============================================================================

def draw_path(screen, path):
    """绘制路径（绿色线段 + 红色方向箭头）"""
    if not path or len(path) < 2:
        return

    for i in range(1, len(path)):
        start_pos = (path[i-1]['x'], path[i-1]['y'])
        end_pos = (path[i]['x'], path[i]['y'])
        pygame.draw.line(screen, (0, 255, 0), start_pos, end_pos, 3)

        if i % 5 == 0:
            angle = path[i].get('theta', 0)
            arrow_length = 20
            end_x = path[i]['x'] + arrow_length * math.cos(angle)
            end_y = path[i]['y'] + arrow_length * math.sin(angle)
            pygame.draw.line(screen, (255, 0, 0),
                           (path[i]['x'], path[i]['y']),
                           (end_x, end_y), 2)


# =============================================================================
# 算法标记渲染器
# =============================================================================

class AlgorithmVisualizer:
    """算法可视化渲染器 — 在地图最顶层渲染算法输出的标记"""

    def __init__(self, output_file: str = "algorithm_markers.json"):
        self.output_file = output_file
        self.markers: Dict[str, Any] = {}
        self.enabled = True
        self.pair_colors: Dict[int, Tuple[int, int, int]] = {}

    def _generate_color(self, index: int, total: int) -> Tuple[int, int, int]:
        if total <= 1:
            return (255, 0, 0)
        hue = index / total
        r, g, b = colorsys.hsv_to_rgb(hue, 0.8, 0.9)
        return (int(r * 255), int(g * 255), int(b * 255))

    def load_markers(self) -> bool:
        if not os.path.exists(self.output_file):
            self.markers = {}
            return False
        try:
            with open(self.output_file, 'r', encoding='utf-8') as f:
                self.markers = json.load(f)
            return True
        except Exception as e:
            print(f"[算法可视化] 加载标记数据失败: {e}")
            self.markers = {}
            return False

    def render(self, surface: pygame.Surface, offset_x: int, offset_y: int, cell_size: int = 50):
        if not self.enabled or not self.markers:
            return
        if not self.load_markers():
            return

        if 'box_goal_pairs' in self.markers:
            num_pairs = len(self.markers['box_goal_pairs'])
            self.pair_colors = {}
            for idx, pair in enumerate(self.markers['box_goal_pairs']):
                self.pair_colors[idx] = self._generate_color(idx, max(num_pairs, 1))

        if 'box_goal_pairs' in self.markers:
            self._render_box_goal_pairs(surface, self.markers['box_goal_pairs'], offset_x, offset_y, cell_size)
        if 'walls' in self.markers:
            self._render_walls(surface, self.markers['walls'], offset_x, offset_y, cell_size)
        if 'blast_points' in self.markers:
            self._render_blast_points(surface, self.markers['blast_points'], offset_x, offset_y, cell_size)
        if 'paths' in self.markers:
            self._render_paths(surface, self.markers['paths'], offset_x, offset_y, cell_size)
        if 'points' in self.markers:
            self._render_points(surface, self.markers['points'], offset_x, offset_y, cell_size)
        if 'areas' in self.markers:
            self._render_areas(surface, self.markers['areas'], offset_x, offset_y, cell_size)
        if 'labels' in self.markers:
            self._render_labels(surface, self.markers['labels'], offset_x, offset_y, cell_size)
        if 'bomb_assignments' in self.markers:
            self._render_bomb_assignments(surface, self.markers['bomb_assignments'], offset_x, offset_y, cell_size)

    # ---- 内部渲染方法 ----

    def _render_box_goal_pairs(self, surface, pairs, offset_x, offset_y, cell_size):
        from core.font_manager import FontManager
        font = FontManager.get_font(14)
        for pair in pairs:
            box_pos = pair.get('box', [0, 0])
            goal_pos = pair.get('goal', [0, 0])
            reachable = pair.get('reachable', True)
            label = pair.get('label', '')
            color = self.pair_colors.get(pairs.index(pair), (255, 0, 0))
            self._render_pair_marker(surface, box_pos, color, reachable, offset_x, offset_y, cell_size, 'box', label)
            self._render_pair_marker(surface, goal_pos, color, reachable, offset_x, offset_y, cell_size, 'goal', label)

    def _render_pair_marker(self, surface, pos, color, reachable, offset_x, offset_y, cell_size, marker_type, label):
        from core.font_manager import FontManager
        screen_x = offset_x + pos[0] * cell_size + cell_size // 2
        screen_y = offset_y + pos[1] * cell_size + cell_size // 2
        radius = 12
        if reachable:
            pygame.draw.circle(surface, color, (screen_x, screen_y), radius)
            pygame.draw.circle(surface, (255, 255, 255), (screen_x, screen_y), radius, 2)
        else:
            triangle_points = [
                (screen_x - radius, screen_y - radius // 2),
                (screen_x + radius, screen_y - radius // 2),
                (screen_x, screen_y + radius)
            ]
            pygame.draw.polygon(surface, color, triangle_points)
            pygame.draw.polygon(surface, (255, 255, 255), triangle_points, 2)
        if label:
            font = FontManager.get_font(10)
            text = font.render(label, True, (255, 255, 255))
            text_rect = text.get_rect(center=(screen_x, screen_y - radius - 8))
            bg_rect = text_rect.inflate(4, 2)
            bg_surface = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
            bg_surface.fill((0, 0, 0, 180))
            surface.blit(bg_surface, bg_rect)
            surface.blit(text, text_rect)

    def _render_blast_points(self, surface, blast_points, offset_x, offset_y, cell_size):
        for bp in blast_points:
            bomb_pos = bp.get('bomb')
            wall_pos = bp.get('wall', [0, 0])
            if bomb_pos:
                bomb_x = offset_x + bomb_pos[0] * cell_size + cell_size // 2
                bomb_y = offset_y + bomb_pos[1] * cell_size + cell_size // 2
                wall_x = offset_x + wall_pos[0] * cell_size + cell_size // 2
                wall_y = offset_y + wall_pos[1] * cell_size + cell_size // 2
                pygame.draw.line(surface, (255, 100, 100), (bomb_x, bomb_y), (wall_x, wall_y), 2)
        wall_groups: Dict[Tuple[int, int], List[Dict]] = {}
        for bp in blast_points:
            wall_pos = tuple(bp.get('wall', [0, 0]))
            if wall_pos not in wall_groups:
                wall_groups[wall_pos] = []
            wall_groups[wall_pos].append(bp)
        for wall_pos, group in wall_groups.items():
            self._render_wall_markers(surface, wall_pos, group, offset_x, offset_y, cell_size)

    def _render_wall_markers(self, surface, wall_pos, group, offset_x, offset_y, cell_size):
        num_markers = len(group)
        marker_radius = 4
        marker_spacing = 12
        wall_screen_x = offset_x + wall_pos[0] * cell_size + cell_size // 2
        wall_screen_y = offset_y + wall_pos[1] * cell_size + cell_size // 2
        total_width = (num_markers - 1) * marker_spacing if num_markers > 1 else 0
        start_x = wall_screen_x - total_width // 2
        for i, bp in enumerate(group):
            color = self.pair_colors.get(bp.get('for_pair', 0), (255, 0, 0))
            label = bp.get('label', '')
            score = bp.get('score', 0)
            marker_x = start_x + i * marker_spacing
            marker_y = wall_screen_y
            pygame.draw.circle(surface, color, (marker_x, marker_y), marker_radius)
            pygame.draw.circle(surface, (255, 255, 255), (marker_x, marker_y), marker_radius, 1)
            if label and num_markers > 1:
                font = pygame.font.SysFont('Arial', 9)
                text = font.render(label, True, color)
                text_rect = text.get_rect(center=(marker_x, marker_y - marker_radius - 6))
                surface.blit(text, text_rect)
            from core.font_manager import FontManager
            score_font = FontManager.get_font(10)
            score_text = score_font.render(str(score), True, (255, 255, 255))
            score_rect = score_text.get_rect(center=(marker_x, marker_y + marker_radius + 8))
            bg_rect = score_rect.inflate(6, 3)
            bg_surface = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
            bg_surface.fill((0, 0, 0, 150))
            surface.blit(bg_surface, bg_rect)
            surface.blit(score_text, score_rect)

    def _render_walls(self, surface, walls, offset_x, offset_y, cell_size):
        font = pygame.font.SysFont('Arial', 12)
        for wall in walls:
            x, y = wall.get('x', 0), wall.get('y', 0)
            color = tuple(wall.get('color', [255, 0, 0]))
            width = wall.get('width', 3)
            label = wall.get('label')
            screen_x = offset_x + x * cell_size
            screen_y = offset_y + y * cell_size
            rect = pygame.Rect(screen_x, screen_y, cell_size, cell_size)
            pygame.draw.rect(surface, color, rect, width)
            if label:
                text = font.render(str(label), True, color)
                surface.blit(text, (screen_x + 5, screen_y + 5))

    def _render_paths(self, surface, paths, offset_x, offset_y, cell_size):
        for path in paths:
            points = path.get('points', [])
            color = tuple(path.get('color', [0, 255, 0]))
            width = path.get('width', 2)
            if len(points) < 2:
                continue
            screen_points = []
            for p in points:
                sx = offset_x + p[0] * cell_size + cell_size // 2
                sy = offset_y + p[1] * cell_size + cell_size // 2
                screen_points.append((sx, sy))
            pygame.draw.lines(surface, color, False, screen_points, width)
            for sp in screen_points:
                pygame.draw.circle(surface, color, sp, 4)

    def _render_points(self, surface, points, offset_x, offset_y, cell_size):
        font = pygame.font.SysFont('Arial', 10)
        for point in points:
            x, y = point.get('x', 0), point.get('y', 0)
            color = tuple(point.get('color', [255, 255, 0]))
            radius = point.get('radius', 8)
            label = point.get('label')
            screen_x = offset_x + x * cell_size + cell_size // 2
            screen_y = offset_y + y * cell_size + cell_size // 2
            pygame.draw.circle(surface, color, (screen_x, screen_y), radius)
            pygame.draw.circle(surface, (255, 255, 255), (screen_x, screen_y), radius, 1)
            if label:
                text = font.render(str(label), True, color)
                surface.blit(text, (screen_x + radius + 2, screen_y - radius))

    def _render_areas(self, surface, areas, offset_x, offset_y, cell_size):
        for area in areas:
            x, y = area.get('x', 0), area.get('y', 0)
            w = area.get('width', 1)
            h = area.get('height', 1)
            color = tuple(area.get('color', [255, 0, 255]))
            alpha = area.get('alpha', 100)
            screen_x = offset_x + x * cell_size
            screen_y = offset_y + y * cell_size
            screen_w = w * cell_size
            screen_h = h * cell_size
            s = pygame.Surface((screen_w, screen_h), pygame.SRCALPHA)
            s.fill((*color, alpha))
            surface.blit(s, (screen_x, screen_y))
            rect = pygame.Rect(screen_x, screen_y, screen_w, screen_h)
            pygame.draw.rect(surface, color, rect, 2)

    def _render_labels(self, surface, labels, offset_x, offset_y, cell_size):
        font = pygame.font.SysFont('Arial', 14)
        for label in labels:
            x, y = label.get('x', 0), label.get('y', 0)
            text = label.get('text', '')
            color = tuple(label.get('color', [255, 255, 255]))
            screen_x = offset_x + x * cell_size + cell_size // 2
            screen_y = offset_y + y * cell_size + cell_size // 2
            rendered_text = font.render(text, True, color)
            text_rect = rendered_text.get_rect(center=(screen_x, screen_y))
            bg_rect = text_rect.inflate(6, 4)
            bg_surface = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
            bg_surface.fill((0, 0, 0, 180))
            surface.blit(bg_surface, bg_rect)
            surface.blit(rendered_text, text_rect)

    def _render_bomb_assignments(self, surface, assignments, offset_x, offset_y, cell_size):
        from core.font_manager import FontManager
        for assignment in assignments:
            bomb_pos = assignment.get('bomb', [0, 0])
            walls = assignment.get('walls', [])
            pair_index = assignment.get('pair_index', 0)
            if walls:
                avg_x = sum(w[0] for w in walls) / len(walls)
                avg_y = sum(w[1] for w in walls) / len(walls)
            else:
                avg_x, avg_y = bomb_pos[0], bomb_pos[1]
            color = self.pair_colors.get(pair_index, (255, 215, 0))
            bomb_sx = offset_x + bomb_pos[0] * cell_size + cell_size // 2
            bomb_sy = offset_y + bomb_pos[1] * cell_size + cell_size // 2
            avg_sx = offset_x + avg_x * cell_size + cell_size // 2
            avg_sy = offset_y + avg_y * cell_size + cell_size // 2
            dx = avg_sx - bomb_sx
            dy = avg_sy - bomb_sy
            distance = (dx ** 2 + dy ** 2) ** 0.5
            if distance > 0:
                nx, ny = dx / distance, dy / distance
                start_x = bomb_sx + nx * cell_size * 0.3
                start_y = bomb_sy + ny * cell_size * 0.3
                end_x = bomb_sx + nx * (distance - cell_size * 0.3)
                end_y = bomb_sy + ny * (distance - cell_size * 0.3)
                pygame.draw.line(surface, color, (start_x, start_y), (end_x, end_y), 3)
                arrow_size = 10
                angle = math.atan2(ny, nx)
                left_angle = angle + 0.5
                right_angle = angle - 0.5
                left_x = end_x - arrow_size * math.cos(left_angle)
                left_y = end_y - arrow_size * math.sin(left_angle)
                right_x = end_x - arrow_size * math.cos(right_angle)
                right_y = end_y - arrow_size * math.sin(right_angle)
                arrow_points = [(end_x, end_y), (left_x, left_y), (right_x, right_y)]
                pygame.draw.polygon(surface, color, arrow_points)
                pygame.draw.circle(surface, color, (avg_sx, avg_sy), 6)
                pygame.draw.circle(surface, (255, 255, 255), (avg_sx, avg_sy), 6, 2)
                font = FontManager.get_font(10)
                label = f"P{pair_index + 1} ({avg_x:.1f},{avg_y:.1f})"
                text = font.render(label, True, color)
                text_rect = text.get_rect(center=(avg_sx, avg_sy - 12))
                bg_rect = text_rect.inflate(4, 2)
                bg_surface = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
                bg_surface.fill((0, 0, 0, 180))
                surface.blit(bg_surface, bg_rect)
                surface.blit(text, text_rect)

    def clear_markers(self):
        self.markers = {}
        if os.path.exists(self.output_file):
            try:
                os.remove(self.output_file)
            except Exception as e:
                print(f"[算法可视化] 清空标记文件失败: {e}")

    def toggle(self) -> bool:
        self.enabled = not self.enabled
        return self.enabled
