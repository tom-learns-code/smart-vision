"""
地图编辑器 - 16x12 网格地图编辑器
支持导入/导出，撤销/重做，拖拽绘制
"""
import pygame
import os
import copy
from .font_manager import FontManager
from .physics import Vector2
from .game_object import GameObject
from .bomb import Bomb
from .target import Target
from .wall import Wall


# 地图配置
MAP_WIDTH = 16  # 列数
MAP_HEIGHT = 12  # 行数
CELL_SIZE = 50  # 每个格子的像素大小

# 元素类型映射
ELEMENT_TYPES = {
    '#': {'name': '墙体', 'color': (100, 100, 100), 'key': '1'},
    '$': {'name': '箱子', 'color': (139, 69, 19), 'key': '2'},
    '.': {'name': '目的地', 'color': (0, 255, 100), 'key': '3'},
    '*': {'name': '炸弹', 'color': (255, 100, 100), 'key': '4'},
    '@': {'name': '小车位置', 'color': (50, 150, 255), 'key': '5'},
    '-': {'name': '空地', 'color': (50, 50, 60), 'key': '6'}
}


class MapEditor:
    """16x12 网格地图编辑器"""

    def __init__(self, screen_width, screen_height):
        self.enabled = False
        self.screen_width = screen_width
        self.screen_height = screen_height

        # 计算地图偏移量（居中显示）
        map_pixel_width = MAP_WIDTH * CELL_SIZE
        map_pixel_height = MAP_HEIGHT * CELL_SIZE
        self.offset_x = (screen_width - map_pixel_width) // 2
        self.offset_y = (screen_height - map_pixel_height) // 2

        # 地图网格 (16x12)
        self.grid = [['-' for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]

        # 初始化边界墙
        self._init_boundary_walls()

        # 当前选中的元素
        self.current_element = '#'

        # 拖拽绘制状态
        self.is_dragging = False
        self.drag_start_cell = None

        # 撤销/重做历史
        self.history = []
        self.history_index = -1
        self.max_history = 50

        # 车辆初始位置（保存网格坐标）
        self.car_spawn_cell = None  # (cell_x, cell_y)

        # 文件夹路径
        self.export_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'maps_export')
        self.import_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'maps_import')

        # 创建文件夹
        os.makedirs(self.export_dir, exist_ok=True)
        os.makedirs(self.import_dir, exist_ok=True)

        # 保存初始状态
        self._save_state()

    def _init_boundary_walls(self):
        """初始化边界墙"""
        # 第一行和最后一行
        for x in range(MAP_WIDTH):
            self.grid[0][x] = '#'
            self.grid[MAP_HEIGHT - 1][x] = '#'

        # 第一列和最后一列
        for y in range(MAP_HEIGHT):
            self.grid[y][0] = '#'
            self.grid[y][MAP_WIDTH - 1] = '#'

    def toggle(self):
        """切换编辑器"""
        self.enabled = not self.enabled
        return self.enabled

    def handle_event(self, event, car, boxes, walls, bombs, targets, boundary_walls):
        """处理编辑器输入事件"""
        if not self.enabled:
            return

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # 左键 - 放置
                self.is_dragging = True
                mouse_pos = pygame.mouse.get_pos()
                cell_pos = self._screen_to_cell(mouse_pos)
                if cell_pos:
                    need_update_car = self._place_element(cell_pos, boxes, walls, bombs, targets)
                    if need_update_car:
                        self.update_car_to_spawn_position(car)
                    self.drag_start_cell = cell_pos

            elif event.button == 3:  # 右键 - 删除
                mouse_pos = pygame.mouse.get_pos()
                cell_pos = self._screen_to_cell(mouse_pos)
                if cell_pos:
                    need_update_car = self._remove_element(cell_pos, boxes, walls, bombs, targets)
                    if need_update_car:
                        self.update_car_to_spawn_position(car)

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # 左键释放
                self.is_dragging = False
                self.drag_start_cell = None

        elif event.type == pygame.MOUSEMOTION:
            # 拖拽绘制（仅对墙体有效）
            if self.is_dragging and self.current_element == '#':
                mouse_pos = pygame.mouse.get_pos()
                cell_pos = self._screen_to_cell(mouse_pos)
                if cell_pos and cell_pos != self.drag_start_cell:
                    need_update_car = self._place_element(cell_pos, boxes, walls, bombs, targets)
                    if need_update_car:
                        self.update_car_to_spawn_position(car)
                    self.drag_start_cell = cell_pos

        elif event.type == pygame.KEYDOWN:
            # 快捷键切换元素
            if event.key == pygame.K_1:
                self.current_element = '#'
            elif event.key == pygame.K_2:
                self.current_element = '$'
            elif event.key == pygame.K_3:
                self.current_element = '.'
            elif event.key == pygame.K_4:
                self.current_element = '*'
            elif event.key == pygame.K_5:
                self.current_element = '@'
            elif event.key == pygame.K_6:
                self.current_element = '-'

            # 撤销/重做（Ctrl+Z, Ctrl+Y）
            elif event.key == pygame.K_z and (pygame.key.get_mods() & pygame.KMOD_CTRL):
                self.undo(boxes, walls, bombs, targets)
                self.update_car_to_spawn_position(car)
            elif event.key == pygame.K_y and (pygame.key.get_mods() & pygame.KMOD_CTRL):
                self.redo(boxes, walls, bombs, targets)
                self.update_car_to_spawn_position(car)

            # 清除所有
            elif event.key == pygame.K_c:
                self.clear_all(boxes, walls, bombs, targets, boundary_walls)
                self.update_car_to_spawn_position(car)

            # 导出地图（使用保存的车辆初始位置）
            elif event.key == pygame.K_s:
                self.export_map()

            # 导入地图
            elif event.key == pygame.K_l:
                self.import_map(boxes, walls, bombs, targets, boundary_walls)
                self.update_car_to_spawn_position(car)

    def _screen_to_cell(self, screen_pos):
        """屏幕坐标转网格坐标"""
        x, y = screen_pos
        cell_x = (x - self.offset_x) // CELL_SIZE
        cell_y = (y - self.offset_y) // CELL_SIZE

        if 0 <= cell_x < MAP_WIDTH and 0 <= cell_y < MAP_HEIGHT:
            return (cell_x, cell_y)
        return None

    def _cell_to_screen(self, cell_x, cell_y):
        """网格坐标转屏幕坐标（中心点）"""
        return (self.offset_x + cell_x * CELL_SIZE + CELL_SIZE // 2,
                self.offset_y + cell_y * CELL_SIZE + CELL_SIZE // 2)

    def _is_boundary(self, cell_x, cell_y):
        """检查是否为边界"""
        return (cell_x == 0 or cell_x == MAP_WIDTH - 1 or
                cell_y == 0 or cell_y == MAP_HEIGHT - 1)

    def _place_element(self, cell_pos, boxes, walls, bombs, targets):
        """放置元素，返回是否需要更新车辆位置"""
        cell_x, cell_y = cell_pos

        # 边界墙不可编辑
        if self._is_boundary(cell_x, cell_y):
            return False

        # 检查是否已有相同元素
        if self.grid[cell_y][cell_x] == self.current_element:
            return False

        # 标记是否需要更新车辆位置
        need_update_car = False

        # 如果是 @ 标记，先清除地图中已有的 @ 标记（一个地图只能有一个）
        if self.current_element == '@':
            self._clear_car_spawn_marker()
            # 保存车辆初始位置（网格坐标）
            self.car_spawn_cell = (cell_x, cell_y)
            need_update_car = True
        # 如果删除了之前的 @ 标记，清除保存的车辆初始位置
        elif self.grid[cell_y][cell_x] == '@':
            self.car_spawn_cell = None
            need_update_car = True

        # 先移除该位置现有元素
        self._remove_element(cell_pos, boxes, walls, bombs, targets, save_state=False)

        # 更新网格
        self.grid[cell_y][cell_x] = self.current_element

        # 创建对应的游戏对象（@ 标记不创建游戏对象）
        screen_pos = self._cell_to_screen(cell_x, cell_y)

        if self.current_element == '#':
            # 检查是否在地图最外一圈，如果是则设置不可破坏
            is_indestructible = (cell_x == 0 or cell_x == MAP_WIDTH - 1 or
                               cell_y == 0 or cell_y == MAP_HEIGHT - 1)
            wall = Wall(screen_pos[0], screen_pos[1], CELL_SIZE, CELL_SIZE,
                       (80, 80, 80), is_indestructible=is_indestructible)
            walls.append(wall)
        elif self.current_element == '$':
            box = GameObject(screen_pos[0], screen_pos[1], 40, (139, 69, 19))
            boxes.append(box)
        elif self.current_element == '.':
            target = Target(screen_pos[0], screen_pos[1], 40, (0, 255, 100))
            targets.append(target)
        elif self.current_element == '*':
            bomb = Bomb(screen_pos[0], screen_pos[1], 40, (255, 100, 100))
            bombs.append(bomb)
        # @ 标记不创建游戏对象，只记录在网格中

        # 保存状态（用于撤销/重做）
        self._save_state()

        return need_update_car

    def _clear_car_spawn_marker(self):
        """清除地图中所有小车初始位置标记"""
        for y in range(1, MAP_HEIGHT - 1):
            for x in range(1, MAP_WIDTH - 1):
                if self.grid[y][x] == '@':
                    self.grid[y][x] = '-'
        # 同时清除保存的车辆初始位置
        self.car_spawn_cell = None

    def _clear_car_spawn_marker_in_grid(self, grid):
        """清除指定网格中的所有小车初始位置标记"""
        for y in range(1, MAP_HEIGHT - 1):
            for x in range(1, MAP_WIDTH - 1):
                if grid[y][x] == '@':
                    grid[y][x] = '-'

    def _sync_car_spawn_from_grid(self):
        """从网格中同步车辆初始位置标记"""
        self.car_spawn_cell = None
        for y in range(1, MAP_HEIGHT - 1):
            for x in range(1, MAP_WIDTH - 1):
                if self.grid[y][x] == '@':
                    self.car_spawn_cell = (x, y)
                    return  # 只保存第一个找到的标记

    def _remove_element(self, cell_pos, boxes, walls, bombs, targets, save_state=True):
        """删除元素，返回是否需要更新车辆位置"""
        cell_x, cell_y = cell_pos

        # 边界墙不可删除
        if self._is_boundary(cell_x, cell_y):
            return False

        current_element = self.grid[cell_y][cell_x]

        if current_element == '-':
            return False

        # 标记是否需要更新车辆位置
        need_update_car = (current_element == '@')

        # 移除对应的游戏对象
        screen_pos = self._cell_to_screen(cell_x, cell_y)
        hitbox = pygame.Rect(screen_pos[0] - CELL_SIZE // 2,
                            screen_pos[1] - CELL_SIZE // 2,
                            CELL_SIZE, CELL_SIZE)

        if current_element == '#':
            for i, wall in enumerate(walls):
                if wall.rect.colliderect(hitbox):
                    del walls[i]
                    break
        elif current_element == '$':
            for i, box in enumerate(boxes):
                if box.rect.colliderect(hitbox):
                    del boxes[i]
                    break
        elif current_element == '.':
            for i, target in enumerate(targets):
                if target.rect.colliderect(hitbox):
                    del targets[i]
                    break
        elif current_element == '*':
            for i, bomb in enumerate(bombs):
                if bomb.rect.colliderect(hitbox):
                    del bombs[i]
                    break

        # 更新网格
        self.grid[cell_y][cell_x] = '-'

        # 保存状态
        if save_state:
            self._save_state()

        return need_update_car

    def clear_all(self, boxes, walls, bombs, targets, boundary_walls):
        """清除所有可编辑元素"""
        # 重置网格
        self.grid = [['-' for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]
        self._init_boundary_walls()

        # 根据网格重建游戏对象
        self._restore_state(self.grid, boxes, walls, bombs, targets, boundary_walls)

        # 重建外部边界墙（游戏对象）
        self._rebuild_boundary_walls(boundary_walls)

        # 清除车辆初始位置
        self.car_spawn_cell = None

        # 保存状态
        self._save_state()

    def _rebuild_boundary_walls(self, boundary_walls):
        """重建边界墙游戏对象"""
        boundary_walls.clear()

        # 顶部边界墙
        top_wall = Wall(self.offset_x + MAP_WIDTH * CELL_SIZE // 2,
                       self.offset_y - CELL_SIZE // 2,
                       MAP_WIDTH * CELL_SIZE, CELL_SIZE, (60, 60, 60))
        boundary_walls.append(top_wall)

        # 底部边界墙
        bottom_wall = Wall(self.offset_x + MAP_WIDTH * CELL_SIZE // 2,
                          self.offset_y + MAP_HEIGHT * CELL_SIZE + CELL_SIZE // 2,
                          MAP_WIDTH * CELL_SIZE, CELL_SIZE, (60, 60, 60))
        boundary_walls.append(bottom_wall)

        # 左侧边界墙
        left_wall = Wall(self.offset_x - CELL_SIZE // 2,
                        self.offset_y + MAP_HEIGHT * CELL_SIZE // 2,
                        CELL_SIZE, MAP_HEIGHT * CELL_SIZE, (60, 60, 60))
        boundary_walls.append(left_wall)

        # 右侧边界墙
        right_wall = Wall(self.offset_x + MAP_WIDTH * CELL_SIZE + CELL_SIZE // 2,
                         self.offset_y + MAP_HEIGHT * CELL_SIZE // 2,
                         CELL_SIZE, MAP_HEIGHT * CELL_SIZE, (60, 60, 60))
        boundary_walls.append(right_wall)

    def _save_state(self):
        """保存当前状态到历史记录"""
        # 如果当前位置在历史中间，删除后面的记录
        if self.history_index < len(self.history) - 1:
            self.history = self.history[:self.history_index + 1]

        # 保存网格深拷贝
        self.history.append(copy.deepcopy(self.grid))

        # 限制历史记录数量
        if len(self.history) > self.max_history:
            self.history.pop(0)
        else:
            self.history_index += 1

    def undo(self, boxes, walls, bombs, targets, boundary_walls):
        """撤销"""
        if self.history_index > 0:
            self.history_index -= 1
            self._restore_state(self.history[self.history_index], boxes, walls, bombs, targets, boundary_walls)
            print("[地图编辑器] 撤销")

    def redo(self, boxes, walls, bombs, targets, boundary_walls):
        """重做"""
        if self.history_index < len(self.history) - 1:
            self.history_index += 1
            self._restore_state(self.history[self.history_index], boxes, walls, bombs, targets, boundary_walls)
            print("[地图编辑器] 重做")

    def _restore_state(self, grid_state, boxes, walls, bombs, targets, boundary_walls):
        """恢复状态"""
        # 清空所有游戏对象
        boxes.clear()
        walls.clear()
        bombs.clear()
        targets.clear()

        # 恢复网格
        self.grid = copy.deepcopy(grid_state)

        # 同步车辆初始位置标记
        self._sync_car_spawn_from_grid()

        # 根据网格重建游戏对象
        for y in range(MAP_HEIGHT):
            for x in range(MAP_WIDTH):
                element = self.grid[y][x]
                if element == '-':
                    continue

                screen_pos = self._cell_to_screen(x, y)

                if element == '#':
                    # 检查是否在地图最外一圈，如果是则设置不可破坏
                    is_indestructible = (x == 0 or x == MAP_WIDTH - 1 or
                                       y == 0 or y == MAP_HEIGHT - 1)
                    wall = Wall(screen_pos[0], screen_pos[1], CELL_SIZE, CELL_SIZE,
                               (80, 80, 80), is_indestructible=is_indestructible)
                    walls.append(wall)
                elif element == '$':
                    box = GameObject(screen_pos[0], screen_pos[1], 40, (139, 69, 19))
                    boxes.append(box)
                elif element == '.':
                    target = Target(screen_pos[0], screen_pos[1], 40, (0, 255, 100))
                    targets.append(target)
                elif element == '*':
                    bomb = Bomb(screen_pos[0], screen_pos[1], 40, (255, 100, 100))
                    bombs.append(bomb)

    def export_map(self):
        """导出地图（使用保存的车辆初始位置）"""
        # 查找最后一个地图编号
        last_number = 0
        for filename in os.listdir(self.export_dir):
            if filename.startswith('map') and filename.endswith('.txt'):
                try:
                    number = int(filename[3:-4])
                    last_number = max(last_number, number)
                except ValueError:
                    pass

        # 生成新文件名
        new_number = last_number + 1
        filename = f'map{new_number}.txt'
        filepath = os.path.join(self.export_dir, filename)

        # 复制网格用于导出（不影响原始网格）
        export_grid = copy.deepcopy(self.grid)

        # 如果有保存的车辆初始位置，在网格中添加 @ 标记
        if self.car_spawn_cell is not None:
            cell_x, cell_y = self.car_spawn_cell
            # 检查是否在有效范围内（排除边界墙）
            if 1 <= cell_x < MAP_WIDTH - 1 and 1 <= cell_y < MAP_HEIGHT - 1:
                # 清除之前的车辆标记
                self._clear_car_spawn_marker_in_grid(export_grid)
                # 在车辆初始位置添加 @ 标记
                export_grid[cell_y][cell_x] = '@'
                print(f"[地图编辑器] 车辆初始位置已保存到网格 ({cell_x}, {cell_y})")

        # 导出为文本
        with open(filepath, 'w', encoding='utf-8') as f:
            for row in export_grid:
                line = ''.join(row) + '\n'
                f.write(line)

        print(f"[地图编辑器] 导出地图: {filepath}")
        return filepath

    def import_map(self, boxes, walls, bombs, targets, boundary_walls):
        """导入地图（默认第一个文件）"""
        try:
            # 获取第一个 .txt 文件
            files = [f for f in os.listdir(self.import_dir) if f.endswith('.txt')]
            if not files:
                print("[地图编辑器] 没有可导入的地图文件")
                return False

            # 排序后取第一个
            files.sort()
            filename = files[0]
            filepath = os.path.join(self.import_dir, filename)

            # 读取文件
            with open(filepath, 'r', encoding='utf-8') as f:
                lines = f.readlines()

            # 验证格式（12行，每行16字符）
            if len(lines) != MAP_HEIGHT:
                print(f"[地图编辑器] 导入失败: 行数不匹配 (应为{MAP_HEIGHT}, 实际{len(lines)})")
                return False

            new_grid = []
            for line in lines:
                line = line.strip('\r\n')
                if len(line) != MAP_WIDTH:
                    print(f"[地图编辑器] 导入失败: 列数不匹配 (应为{MAP_WIDTH}, 实际{len(line)})")
                    return False
                new_grid.append(list(line))

            # 确保边界墙正确
            for x in range(MAP_WIDTH):
                new_grid[0][x] = '#'
                new_grid[MAP_HEIGHT - 1][x] = '#'
            for y in range(MAP_HEIGHT):
                new_grid[y][0] = '#'
                new_grid[y][MAP_WIDTH - 1] = '#'

            # 清空并恢复
            boxes.clear()
            walls.clear()
            bombs.clear()
            targets.clear()
            self._rebuild_boundary_walls(boundary_walls)

            # 更新网格并重建对象
            self.grid = new_grid
            self._restore_state(self.grid, boxes, walls, bombs, targets, boundary_walls)

            # 查找并保存导入地图中的车辆初始位置标记
            self._sync_car_spawn_from_grid()

            # 保存到历史
            self._save_state()

            print(f"[地图编辑器] 导入地图: {filepath}")
            return True

        except Exception as e:
            print(f"[地图编辑器] 导入失败: {e}")
            return False

    def draw(self, surface, walls, boundary_walls):
        """绘制编辑器"""
        if not self.enabled:
            return

        # 绘制地图区域背景
        map_rect = pygame.Rect(self.offset_x - 5, self.offset_y - 5,
                               MAP_WIDTH * CELL_SIZE + 10,
                               MAP_HEIGHT * CELL_SIZE + 10)
        pygame.draw.rect(surface, (40, 40, 45), map_rect)

        # 绘制网格线
        for x in range(MAP_WIDTH + 1):
            screen_x = self.offset_x + x * CELL_SIZE
            pygame.draw.line(surface, (60, 60, 70),
                           (screen_x, self.offset_y),
                           (screen_x, self.offset_y + MAP_HEIGHT * CELL_SIZE), 1)

        for y in range(MAP_HEIGHT + 1):
            screen_y = self.offset_y + y * CELL_SIZE
            pygame.draw.line(surface, (60, 60, 70),
                           (self.offset_x, screen_y),
                           (self.offset_x + MAP_WIDTH * CELL_SIZE, screen_y), 1)

        # 绘制坐标（左上角为原点）
        self._draw_coordinates(surface)

        # 绘制UI面板
        self._draw_ui(surface)

    def _draw_coordinates(self, surface):
        """绘制地图坐标"""
        font = FontManager.get_font(16)

        # 在每个格子边框显示坐标
        # 显示X坐标（在顶部）
        for x in range(MAP_WIDTH):
            screen_x = self.offset_x + x * CELL_SIZE + CELL_SIZE // 2
            screen_y = self.offset_y - 10  # 在地图上方

            text = font.render(str(x), True, (180, 180, 180))
            text_rect = text.get_rect(center=(screen_x, screen_y))
            surface.blit(text, text_rect)

        # 显示Y坐标（在左侧）
        for y in range(MAP_HEIGHT):
            screen_x = self.offset_x - 12  # 在地图左侧
            screen_y = self.offset_y + y * CELL_SIZE + CELL_SIZE // 2

            text = font.render(str(y), True, (180, 180, 180))
            text_rect = text.get_rect(center=(screen_x, screen_y))
            surface.blit(text, text_rect)

    def _draw_element(self, surface, element, cell_x, cell_y):
        """绘制元素"""
        screen_pos = self._cell_to_screen(cell_x, cell_y)
        element_info = ELEMENT_TYPES.get(element, {'color': (100, 100, 100)})

        if element == '#':
            # 墙体
            rect = pygame.Rect(screen_pos[0] - CELL_SIZE // 2 + 2,
                             screen_pos[1] - CELL_SIZE // 2 + 2,
                             CELL_SIZE - 4, CELL_SIZE - 4)
            pygame.draw.rect(surface, element_info['color'], rect)
            pygame.draw.rect(surface, (120, 120, 120), rect, 2)

        elif element == '$':
            # 箱子
            rect = pygame.Rect(screen_pos[0] - 20, screen_pos[1] - 20, 40, 40)
            pygame.draw.rect(surface, element_info['color'], rect)
            pygame.draw.rect(surface, (160, 82, 45), rect, 3)
            # 箱子纹理
            pygame.draw.line(surface, (100, 50, 15),
                           (rect.left, rect.top), (rect.right, rect.bottom), 2)
            pygame.draw.line(surface, (100, 50, 15),
                           (rect.right, rect.top), (rect.left, rect.bottom), 2)

        elif element == '.':
            # 目标点
            pygame.draw.circle(surface, element_info['color'], screen_pos, 18)
            pygame.draw.circle(surface, (0, 200, 80), screen_pos, 18, 3)

        elif element == '*':
            # 炸弹
            pygame.draw.circle(surface, element_info['color'], screen_pos, 18)
            pygame.draw.circle(surface, (255, 150, 150), screen_pos, 18, 2)
            # 引信
            pygame.draw.line(surface, (200, 200, 200),
                           (screen_pos[0], screen_pos[1] - 18),
                           (screen_pos[0] + 8, screen_pos[1] - 25), 3)

        elif element == '@':
            # 小车初始位置（新版本：圆形）
            pygame.draw.circle(surface, element_info['color'], screen_pos, 18)
            pygame.draw.circle(surface, (100, 200, 255), screen_pos, 18, 3)
            # 绘制中心圆点
            pygame.draw.circle(surface, (255, 255, 255), screen_pos, 4)

    def _draw_preview(self, surface, cell_pos):
        """绘制鼠标悬停预览"""
        cell_x, cell_y = cell_pos
        screen_pos = self._cell_to_screen(cell_x, cell_y)

        # 边界墙不可预览编辑
        if self._is_boundary(cell_x, cell_y):
            return

        element_info = ELEMENT_TYPES.get(self.current_element, {'color': (100, 100, 100)})
        color = element_info['color']

        # 半透明预览
        s = pygame.Surface((CELL_SIZE - 4, CELL_SIZE - 4), pygame.SRCALPHA)
        s.fill((*color, 128))  # 50% 透明度

        rect = pygame.Rect(screen_pos[0] - CELL_SIZE // 2 + 2,
                         screen_pos[1] - CELL_SIZE // 2 + 2,
                         CELL_SIZE - 4, CELL_SIZE - 4)
        surface.blit(s, rect)

        # 绘制边框
        pygame.draw.rect(surface, (255, 255, 255), rect, 2)

    def _draw_ui(self, surface):
        """绘制UI面板"""
        font = FontManager.get_font(20)

        # 左上角：当前元素
        element_info = ELEMENT_TYPES.get(self.current_element, {'name': '未知', 'key': '?'})
        current_text = font.render(
            f"当前元素: [{self.current_element}] {element_info['name']} ({element_info['key']})",
            True, (255, 255, 255)
        )
        surface.blit(current_text, (20, 20))

        # 操作说明
        instructions = [
            "地图编辑器",
            f"1-6 - 切换元素 (墙# 箱$ 目. 炸* 车@ 空-)",
            "左键 - 放置 | 右键 - 删除",
            "墙体可拖拽绘制",
            "Ctrl+Z - 撤销 | Ctrl+Y - 重做",
            "C - 清除所有 | S - 导出地图 | L - 导入地图",
            "TAB - 退出编辑器"
        ]

        for i, line in enumerate(instructions):
            text = font.render(line, True, (200, 200, 200))
            surface.blit(text, (20, 55 + i * 25))

        # 右上角：元素选择
        start_x = self.screen_width - 180
        start_y = 20

        for i, (char, info) in enumerate(ELEMENT_TYPES.items()):
            y = start_y + i * 30

            # 背景
            bg_color = info['color']
            if char == self.current_element:
                bg_color = (min(bg_color[0] + 50, 255),
                           min(bg_color[1] + 50, 255),
                           min(bg_color[2] + 50, 255))

            rect = pygame.Rect(start_x, y, 140, 26)
            pygame.draw.rect(surface, bg_color, rect)
            pygame.draw.rect(surface, (255, 255, 255), rect, 1 if char != self.current_element else 2)

            # 文字
            text = font.render(f"{info['key']} - {char} {info['name']}", True, (255, 255, 255))
            surface.blit(text, (start_x + 10, y + 3))

    def get_car_spawn_position(self):
        """获取小车初始位置"""
        # 第一优先级：使用编辑器中保存的车辆初始位置
        if self.car_spawn_cell is not None:
            cell_x, cell_y = self.car_spawn_cell
            # 检查是否在有效范围内
            if 1 <= cell_x < MAP_WIDTH - 1 and 1 <= cell_y < MAP_HEIGHT - 1:
                screen_pos = self._cell_to_screen(cell_x, cell_y)
                return Vector2(screen_pos[0], screen_pos[1])

        # 第二优先级：查找地图中的 @ 标记（车辆）
        for y in range(1, MAP_HEIGHT - 1):
            for x in range(1, MAP_WIDTH - 1):
                if self.grid[y][x] == '@':
                    screen_pos = self._cell_to_screen(x, y)
                    return Vector2(screen_pos[0], screen_pos[1])

        # 第三优先级：最左列中间（第2列，第6行）
        middle_y = MAP_HEIGHT // 2  # 第6行（中间）
        left_x = 1  # 第2列（第1列是边界墙）
        if self.grid[middle_y][left_x] == '-':
            screen_pos = self._cell_to_screen(left_x, middle_y)
            return Vector2(screen_pos[0], screen_pos[1])

        # 第四优先级：最右侧中间（第15列，第6行）
        right_x = MAP_WIDTH - 2  # 第15列（第16列是边界墙）
        if self.grid[middle_y][right_x] == '-':
            screen_pos = self._cell_to_screen(right_x, middle_y)
            return Vector2(screen_pos[0], screen_pos[1])

        # 第五优先级：强制放在地图正中间（第8列，第6行），不论是否占用
        center_x = MAP_WIDTH // 2  # 第8列
        center_y = MAP_HEIGHT // 2  # 第6行
        screen_pos = self._cell_to_screen(center_x, center_y)
        return Vector2(screen_pos[0], screen_pos[1])

    def update_car_to_spawn_position(self, car):
        """在编辑器模式下，将车辆移动到保存的初始位置"""
        if self.car_spawn_cell is not None:
            cell_x, cell_y = self.car_spawn_cell
            if 1 <= cell_x < MAP_WIDTH - 1 and 1 <= cell_y < MAP_HEIGHT - 1:
                screen_pos = self._cell_to_screen(cell_x, cell_y)
                car.pos = Vector2(screen_pos[0], screen_pos[1])
                car.prev_pos = Vector2(screen_pos[0], screen_pos[1])
                car.velocity = Vector2(0, 0)
                car.target_velocity = Vector2(0, 0)
                car.angular_velocity = 0.0
                car.target_angular_velocity = 0.0
                return True
        return False
