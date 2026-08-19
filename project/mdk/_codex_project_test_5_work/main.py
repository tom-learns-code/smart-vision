"""
车推箱子物理引擎 - 主程序
"""
import pygame
import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.config import *
from core.game_factory import GameFactory
from core.game_logic import BombExplosion, BombDetection, TargetDetection, get_world_state
from core.physics import clamp_velocity, Vector2
from core.collision_pipeline import CollisionPipeline
from core.adaptive_physics import AdaptivePhysics
from core.font_manager import FontManager
from core.path_executor import PathExecutor
from vis.renderer import draw_path, AlgorithmVisualizer
from core.map_editor import MapEditor, MAP_WIDTH

# 使用共享配置（与 generate_algorithm_markers.py 同步）
from config_shared import MAP_FILE_PATH


def load_map_file(map_file_path, map_editor, boxes, walls, bombs, targets, boundary_walls):
    """从指定文件加载地图到地图编辑器"""
    if not os.path.exists(map_file_path):
        print(f"[地图加载] 文件不存在: {map_file_path}")
        return False

    with open(map_file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # 去除换行符
    new_grid = []
    for line in lines:
        line = line.rstrip('\n\r')
        # 补齐到 MAP_WIDTH
        cells = list(line)
        while len(cells) < MAP_WIDTH:
            cells.append(' ')
        new_grid.append(cells)

    # 确保边界墙正确
    for x in range(MAP_WIDTH):
        new_grid[0][x] = '#'
        new_grid[len(new_grid) - 1][x] = '#'
    for y in range(len(new_grid)):
        new_grid[y][0] = '#'
        new_grid[y][MAP_WIDTH - 1] = '#'

    # 更新到地图编辑器
    boxes.clear()
    walls.clear()
    bombs.clear()
    targets.clear()
    map_editor._rebuild_boundary_walls(boundary_walls)
    map_editor.grid = new_grid
    map_editor._restore_state(map_editor.grid, boxes, walls, bombs, targets, boundary_walls)
    map_editor._sync_car_spawn_from_grid()
    map_editor._save_state()

    print(f"[地图加载] 文件: {map_file_path}")
    return True


def reset_game():
    """重置游戏状态"""
    car = GameFactory.create_car()
    box = GameFactory.create_box()
    bomb = GameFactory.create_bomb()
    target = GameFactory.create_target()

    walls = GameFactory.create_default_walls()
    boundary_walls = GameFactory.create_boundary_walls()

    return car, [box], [bomb], [target], (walls, boundary_walls)


def reset_from_map_editor(map_editor):
    """从地图编辑器重置游戏状态"""
    from core.game_factory import GameFactory

    # 获取小车初始位置
    car_pos = map_editor.get_car_spawn_position()
    car = GameFactory.create_car(car_pos.x, car_pos.y)
    car.prev_pos = Vector2(car_pos.x, car_pos.y)

    boxes = []
    bombs = []
    targets = []
    walls = []

    # 根据地图编辑器的网格重建游戏对象
    for y in range(12):
        for x in range(16):
            element = map_editor.grid[y][x]
            if element == '-' or element == '+':
                continue

            screen_pos = map_editor._cell_to_screen(x, y)

            if element == '#':
                from core.wall import Wall
                # 检查是否在地图最外一圈，如果是则设置不可破坏
                is_indestructible = (x == 0 or x == 15 or y == 0 or y == 11)
                wall = Wall(screen_pos[0], screen_pos[1], 50, 50, (80, 80, 80), is_indestructible=is_indestructible)
                walls.append(wall)
            elif element == '$':
                box = GameFactory.create_box()
                box.pos = Vector2(screen_pos[0], screen_pos[1])
                box.prev_pos = Vector2(screen_pos[0], screen_pos[1])
                boxes.append(box)
            elif element == '.':
                target = GameFactory.create_target()
                target.pos = Vector2(screen_pos[0], screen_pos[1])
                targets.append(target)
            elif element == '*':
                bomb = GameFactory.create_bomb()
                bomb.pos = Vector2(screen_pos[0], screen_pos[1])
                bomb.prev_pos = Vector2(screen_pos[0], screen_pos[1])
                bombs.append(bomb)

    # 重建边界墙
    boundary_walls = GameFactory.create_boundary_walls()

    return car, boxes, bombs, targets, (walls, boundary_walls)


def reset_to_editor_center(map_editor):
    """重置车辆到地图编辑器的小车初始位置"""
    from core.game_factory import GameFactory

    car = GameFactory.create_car()
    # 使用地图编辑器的小车初始位置逻辑
    car_pos = map_editor.get_car_spawn_position()
    car.pos = Vector2(car_pos.x, car_pos.y)
    car.prev_pos = Vector2(car_pos.x, car_pos.y)

    return car


def get_actual_velocity_and_speed(car, dt):
    """计算实际速度和速度大小（位移变化率）"""
    if hasattr(car, 'prev_pos') and dt > 0:
        displacement = car.pos - car.prev_pos
        actual_velocity = displacement / dt
        actual_speed = actual_velocity.length()
    else:
        actual_velocity = car.velocity
        actual_speed = actual_velocity.length()
    return actual_velocity, actual_speed


def _draw_map_coordinates(screen, map_editor):
    """绘制地图坐标（左上角为原点）"""
    font = FontManager.get_font(16)

    # 显示X坐标（在顶部）
    for x in range(16):
        screen_x = map_editor.offset_x + x * 50 + 25
        screen_y = map_editor.offset_y - 10

        text = font.render(str(x), True, (180, 180, 180))
        text_rect = text.get_rect(center=(screen_x, screen_y))
        screen.blit(text, text_rect)

    # 显示Y坐标（在左侧）
    for y in range(12):
        screen_x = map_editor.offset_x - 12
        screen_y = map_editor.offset_y + y * 50 + 25

        text = font.render(str(y), True, (180, 180, 180))
        text_rect = text.get_rect(center=(screen_x, screen_y))
        screen.blit(text, text_rect)


def draw_debug_info(surface, car, dt, free_mode):
    """绘制调试信息"""
    font = FontManager.get_font(24)

    # 计算实际速度
    actual_velocity, actual_speed = get_actual_velocity_and_speed(car, dt)

    mode_text = "自由模式" if free_mode else "地图模式"
    info_lines = [
        f"当前模式: {mode_text}",
        f"车位置: ({car.pos.x:.1f}, {car.pos.y:.1f})",
        f"实际速度: {actual_speed:.1f} 像素/秒",
        f"实际方向: ({actual_velocity.x:.1f}, {actual_velocity.y:.1f})",
        f"车角度: {((car.angle + 180) % 360 - 180):.1f}°",
        f"角速度: {car.angular_velocity:.1f}°/s",
        "",
        f"当前档位: {SPEED_KEYS[CURRENT_SPEED_INDEX]} ({SPEED_PRESETS[SPEED_KEYS[CURRENT_SPEED_INDEX]]})",
        "",
        "控制说明:",
        "WASD - 移动车",
        "Q/E - 旋转车",
        "1/2 - 切换速度档位",
        "SPACE - 重置",
        "TAB - 切换地图编辑器",
        "F - 切换自由/地图模式",
        "I - 切换信息栏显示",
        "R - 算法生成路径轨迹",
        "T - 开始/暂停循迹执行",
        "Y - 重置路径",
    ]

    for i, line in enumerate(info_lines):
        text = font.render(line, True, COLOR_WHITE)
        surface.blit(text, (10, 10 + i * 25))


def main():
    """主游戏循环"""
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("车推箱子物理引擎 - 零穿模保证")
    clock = pygame.time.Clock()

    # 初始化游戏对象
    car, boxes, bombs, targets, (walls, boundary_walls) = reset_game()

    # 初始化地图编辑器（默认禁用，游戏启动进入地图模式）
    map_editor = MapEditor(SCREEN_WIDTH, SCREEN_HEIGHT)
    map_editor.enabled = False  # 禁用编辑模式，但会加载地图数据

    # 自由模式状态，默认关闭
    free_mode = False

    # 初始化物理管线
    pipeline = CollisionPipeline(
        friction=FRICTION,
        torque_factor=TORQUE_FACTOR,
        wall_torque_factor=WALL_TORQUE_FACTOR
    )

    # 初始化自适应物理系统
    adaptive = AdaptivePhysics(base_substeps=BASE_SUBSTEPS, grid_size=GRID_SIZE)

    # 添加路径执行模块
    path_executor = PathExecutor()

    # 加载演示算法
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    algorithm_path = os.path.join(script_dir, "algorithms/demo_path_algorithm.py")
    path_executor.load_algorithm(algorithm_path)

    # 初始化地图编辑器的边界墙
    map_editor._rebuild_boundary_walls(boundary_walls)

    # 初始化算法可视化渲染器
    # 检查是否从标记脚本启动（通过环境变量判断）
    enable_viz = os.environ.get('ENABLE_VISUALIZATION', '0') == '1'
    # 使用相对路径，文件保存在脚本所在目录
    markers_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'algorithm_markers.json')
    algorithm_visualizer = AlgorithmVisualizer(markers_file)
    algorithm_visualizer.load_markers()
    # 默认关闭可视化，只有从标记脚本启动时才打开
    algorithm_visualizer.enabled = enable_viz
    if enable_viz:
        print("[游戏] 算法可视化已启用")
    else:
        print("[游戏] 算法可视化已禁用（按 V 键启用）")

    running = True

    # 全局变量用于速度控制
    global CURRENT_SPEED_INDEX

    # 信息栏显示状态，默认关闭
    show_info_panel = False

    # 从文件加载地图
    if not load_map_file(MAP_FILE_PATH, map_editor, boxes, walls, bombs, targets, boundary_walls):
        # 加载失败，使用默认空地图
        car, boxes, bombs, targets, (walls, boundary_walls) = reset_from_map_editor(map_editor)
        print("[游戏] 未找到地图文件，使用默认地图")
    else:
        car, boxes, bombs, targets, (walls, boundary_walls) = reset_from_map_editor(map_editor)
        print("[游戏] 地图加载成功，进入游戏模式")

    while running:
        dt = clock.tick(FPS) / 1000.0  # 4秒

        # 处理输入
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if not map_editor.enabled:
                        if free_mode:
                            # 自由模式下重置到默认游戏状态
                            car, boxes, bombs, targets, (walls, boundary_walls) = reset_game()
                            path_executor.reset()
                            print("[重置] 自由模式已重置")
                        else:
                            # 地图模式下重置到地图编辑器状态
                            car, boxes, bombs, targets, (walls, boundary_walls) = reset_from_map_editor(map_editor)
                            path_executor.reset()
                            print("[重置] 地图模式已重置")
                    else:
                        # 编辑器模式下，重置车辆位置到中心
                        car = reset_to_editor_center(map_editor)
                elif event.key == pygame.K_TAB:
                    if map_editor.toggle():
                        # 进入编辑器模式，重置车辆位置
                        car = reset_to_editor_center(map_editor)
                        # 初始化边界墙
                        map_editor._rebuild_boundary_walls(boundary_walls)
                    else:
                        # 退出编辑器，使用地图编辑器的数据重置游戏
                        car, boxes, bombs, targets, (walls, boundary_walls) = reset_from_map_editor(map_editor)
                        path_executor.reset()
                        print("[地图编辑器] 退出编辑器，地图已加载")
                elif event.key == pygame.K_1 and not map_editor.enabled:  # 减速（非编辑器模式下）
                    CURRENT_SPEED_INDEX = (CURRENT_SPEED_INDEX - 1) % len(SPEED_KEYS)
                    print(f"[速度控制] 切换到: {SPEED_KEYS[CURRENT_SPEED_INDEX]} ({SPEED_PRESETS[SPEED_KEYS[CURRENT_SPEED_INDEX]]})")
                elif event.key == pygame.K_2 and not map_editor.enabled:  # 加速（非编辑器模式下）
                    CURRENT_SPEED_INDEX = (CURRENT_SPEED_INDEX + 1) % len(SPEED_KEYS)
                    print(f"[速度控制] 切换到: {SPEED_KEYS[CURRENT_SPEED_INDEX]} ({SPEED_PRESETS[SPEED_KEYS[CURRENT_SPEED_INDEX]]})")

                # 算法控制键（非编辑器模式下）
                elif event.key == pygame.K_r and not map_editor.enabled:  # R键 - 计算并显示路径
                    world_state = get_world_state(car, boxes, walls, bombs, targets)
                    path = path_executor.compute_path(world_state)
                    path_executor.load_path(path)
                    print(f"[路径计算] 生成路径，共 {len(path)} 个点")
                elif event.key == pygame.K_t and not map_editor.enabled:  # T键 - 开始/暂停循迹执行
                    path_executor.is_running = not path_executor.is_running
                    print(f"[路径循迹] {'开始' if path_executor.is_running else '暂停'}")
                elif event.key == pygame.K_y and not map_editor.enabled:  # Y键 - 重置路径
                    path_executor.reset()
                    print(f"[路径循迹] 重置")
                elif event.key == pygame.K_i and not map_editor.enabled:  # I键 - 切换信息栏显示
                    show_info_panel = not show_info_panel
                    print(f"[界面] 信息栏 {'显示' if show_info_panel else '隐藏'}")
                elif event.key == pygame.K_f and not map_editor.enabled:  # F键 - 切换自由模式
                    free_mode = not free_mode
                    if free_mode:
                        # 切换到自由模式，重置游戏
                        car, boxes, bombs, targets, (walls, boundary_walls) = reset_game()
                        path_executor.reset()
                        print(f"[模式] 切换到自由模式")
                    else:
                        # 切换到地图模式，从地图编辑器恢复
                        car, boxes, bombs, targets, (walls, boundary_walls) = reset_from_map_editor(map_editor)
                        path_executor.reset()
                        print(f"[模式] 切换到地图模式")
                elif event.key == pygame.K_v and not map_editor.enabled:  # V键 - 切换算法可视化
                    algorithm_visualizer.toggle()

            # 地图编辑器事件
            map_editor.handle_event(event, car, boxes, walls, bombs, targets, boundary_walls)

        # 获取键盘状态
        keys = pygame.key.get_pressed()

        # 根据当前档位获取移动速度
        move_speed = SPEED_PRESETS[SPEED_KEYS[CURRENT_SPEED_INDEX]]
        rotate_speed = 180  # 度/秒

        # 路径循迹模式，通过实时输入沿轨迹走
        if not map_editor.enabled:
            if not path_executor.is_running:
                # 手动控制模式
                car.target_velocity = Vector2(0, 0)
                if keys[pygame.K_w]:
                    car.target_velocity.y = -move_speed
                if keys[pygame.K_s]:
                    car.target_velocity.y = move_speed
                if keys[pygame.K_a]:
                    car.target_velocity.x = -move_speed
                if keys[pygame.K_d]:
                    car.target_velocity.x = move_speed

                car.target_angular_velocity = 0.0
                if keys[pygame.K_q]:
                    car.target_angular_velocity = -rotate_speed
                if keys[pygame.K_e]:
                    car.target_angular_velocity = rotate_speed
            else:
                # 路径循迹模式 - 通过实时输入沿轨迹走
                target = path_executor.get_current_target()
                if target:
                    # 计算到目标点的方向
                    dx = target['x'] - car.pos.x
                    dy = target['y'] - car.pos.y
                    dist = (dx**2 + dy**2)**0.5

                    # 计算实际速度（使用位移变化率，和左上角显示一致）
                    _, current_speed = get_actual_velocity_and_speed(car, dt)

                    # 检查是否卡住
                    if path_executor.check_stuck(current_speed, dt):
                        path_executor.is_running = False
                        car.target_velocity = Vector2(0, 0)
                        car.target_angular_velocity = 0.0
                        print(f"[路径循迹] 车辆卡住（1秒速度<5），停止循迹")
                    elif dist > 5:  # 如果距离足够远，设置目标速度向目标点移动
                        car.target_velocity = Vector2(dx / dist * move_speed, dy / dist * move_speed)
                        # 设置目标角度
                        target_angle = target['theta']
                        current_angle = math.radians(car.angle)

                        # 角度差
                        angle_diff = target_angle - current_angle
                        # 标准化到 [-pi, pi]
                        while angle_diff > math.pi:
                            angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi:
                            angle_diff += 2 * math.pi

                        # 设置目标角速度
                        if abs(angle_diff) > 0.1:
                            car.target_angular_velocity = angle_diff * 5.0  # 旋转到目标角度
                        else:
                            car.target_angular_velocity = 0.0

                        # 接近当前目标点时，切换到下一个点
                        if dist < 20:
                            path_executor.step()
                    else:
                        # 已到达当前点，移动到下一个
                        path_executor.step()
                else:
                    path_executor.is_running = False
                    car.target_velocity = Vector2(0, 0)
                    car.target_angular_velocity = 0.0
                    print(f"[路径循迹] 到达路径终点")

        # 物理更新（非编辑模式）
        if not map_editor.enabled:
            # 更新空间网格
            all_objects = [car] + boxes + bombs
            adaptive.update_spatial_grid(all_objects, walls + boundary_walls)

            # 自适应子步数
            substeps = adaptive.adaptive_substeps(*all_objects)

            # 物理子步处理
            for _ in range(substeps):
                sub_dt = dt / substeps

                # 更新所有箱子和车的预测位置
                car.update(sub_dt)
                for box in boxes:
                    box.update(sub_dt)
                for bomb in bombs:
                    if not bomb.is_exploded:
                        bomb.update(sub_dt)

                # 优化：使用空间哈希的碰撞检测

                # 0. 边界墙碰撞检测（绝对不可穿过，优先处理）
                for obj in [car] + boxes + bombs:
                    if not (hasattr(obj, 'is_exploded') and obj.is_exploded):
                        for boundary_wall in boundary_walls:
                            pipeline.process_single_collision(obj, boundary_wall, sub_dt)

                # 1. 箱子-墙碰撞（只检测相邻网格）
                for box in boxes:
                    potential_walls = adaptive.optimized_collision_detection(box)
                    for wall in potential_walls:
                        if wall.type == "wall":
                            pipeline.process_single_collision(box, wall, sub_dt)

                # 2. 车-墙碰撞（只检测相邻网格）
                potential_walls = adaptive.optimized_collision_detection(car)
                for wall in potential_walls:
                    if wall.type == "wall":
                        pipeline.process_single_collision(car, wall, sub_dt)

                # 2.5. 炸弹-墙碰撞（只检测相邻网格）
                for bomb in bombs:
                    if not bomb.is_exploded:
                        potential_walls = adaptive.optimized_collision_detection(bomb)
                        for wall in potential_walls:
                            if wall.type == "wall":
                                pipeline.process_single_collision(bomb, wall, sub_dt)

                # 3. 车-箱子碰撞
                for box in boxes:
                    pipeline.process_single_collision(car, box, sub_dt, no_resistance=True)

                # 4. 车-炸弹碰撞
                for bomb in bombs:
                    if not bomb.is_exploded:
                        pipeline.process_single_collision(car, bomb, sub_dt, no_resistance=True)

                # 5. 箱子-箱子碰撞（双向检测）
                for box in boxes:
                    potential_objs = adaptive.optimized_collision_detection(box)
                    for other in potential_objs:
                        if other.type == "box":
                            pipeline.process_single_collision(box, other, sub_dt)
                for box in boxes:
                    for other in boxes:
                        if id(box) != id(other):
                            pipeline.process_single_collision(box, other, sub_dt)

                # 6. 箱子-炸弹碰撞
                for box in boxes:
                    potential_objs = adaptive.optimized_collision_detection(box)
                    for bomb in potential_objs:
                        if bomb.type == "bomb" and not bomb.is_exploded:
                            pipeline.process_single_collision(box, bomb, sub_dt)

                # 7. 炸弹-箱子碰撞（双向检测）
                for bomb in bombs:
                    if not bomb.is_exploded:
                        for box in boxes:
                            pipeline.process_single_collision(bomb, box, sub_dt)

                # 8. 炸弹-炸弹碰撞（双向检测）
                for bomb in bombs:
                    if not bomb.is_exploded:
                        for other in bombs:
                            if not other.is_exploded and id(bomb) != id(other):
                                pipeline.process_single_collision(bomb, other, sub_dt)

                # 9. 最终位置对齐（SAT精确检测，轴对齐法线）
                from core.physics import sat_collision

                # 箱子-箱子对齐
                for box in boxes:
                    potential_objs = adaptive.optimized_collision_detection(box)
                    for other in potential_objs:
                        if other.type == "box" and id(box) < id(other):
                            collided, normal, overlap = sat_collision(box.get_vertices(), other.get_vertices(), box.center, other.center)
                            if collided and normal is not None and overlap > 0:
                                # 轴对齐法线
                                if abs(normal.x) > abs(normal.y):
                                    aligned_normal = Vector2(1, 0) if normal.x > 0 else Vector2(-1, 0)
                                else:
                                    aligned_normal = Vector2(0, 1) if normal.y > 0 else Vector2(0, -1)
                                ratio = box.mass / (box.mass + other.mass)
                                if overlap > 2.0:
                                    box.pos -= aligned_normal * overlap * ratio * 0.85
                                    other.pos += aligned_normal * overlap * (1 - ratio) * 0.85

                # 炸弹-炸弹对齐
                for bomb in bombs:
                    if not bomb.is_exploded:
                        for other in bombs:
                            if not other.is_exploded and id(bomb) < id(other):
                                collided, normal, overlap = sat_collision(bomb.get_vertices(), other.get_vertices(), bomb.center, other.center)
                                if collided and normal is not None and overlap > 0:
                                    if abs(normal.x) > abs(normal.y):
                                        aligned_normal = Vector2(1, 0) if normal.x > 0 else Vector2(-1, 0)
                                    else:
                                        aligned_normal = Vector2(0, 1) if normal.y > 0 else Vector2(0, -1)
                                    ratio = bomb.mass / (bomb.mass + other.mass)
                                    if overlap > 2.0:
                                        bomb.pos -= aligned_normal * overlap * ratio * 0.85
                                        other.pos += aligned_normal * overlap * (1 - ratio) * 0.85

                # 炸弹-箱子对齐
                for bomb in bombs:
                    if not bomb.is_exploded:
                        for box in boxes:
                            collided, normal, overlap = sat_collision(bomb.get_vertices(), box.get_vertices(), bomb.center, box.center)
                            if collided and normal is not None and overlap > 0:
                                if abs(normal.x) > abs(normal.y):
                                    aligned_normal = Vector2(1, 0) if normal.x > 0 else Vector2(-1, 0)
                                else:
                                    aligned_normal = Vector2(0, 1) if normal.y > 0 else Vector2(0, -1)
                                ratio = bomb.mass / (bomb.mass + box.mass)
                                if overlap > 2.0:
                                    bomb.pos -= aligned_normal * overlap * ratio * 0.85
                                    box.pos += aligned_normal * overlap * (1 - ratio) * 0.85

                # 10. 物体-墙对齐（SAT精确检测）
                for obj in [car] + boxes + bombs:
                    # 炸弹需要检查是否已爆炸
                    if hasattr(obj, 'is_exploded') and obj.is_exploded:
                        continue
                    potential_walls = adaptive.optimized_collision_detection(obj)
                    for wall in potential_walls:
                        if wall.type == "wall":
                            # 炸弹与可破坏的墙不需要对齐（允许穿入）
                            if obj.type == "bomb":
                                if hasattr(wall, 'is_indestructible') and not wall.is_indestructible:
                                    continue  # 可破坏的墙，跳过对齐，允许炸弹穿入

                            collided, _, _ = sat_collision(obj.get_vertices() if hasattr(obj, 'get_vertices') else obj.get_rotated_vertices(obj.pos), wall.get_vertices())
                            if collided:
                                normal = pipeline.calculate_wall_normal(obj, wall)
                                overlap = pipeline.calculate_sat_penetration(obj, wall)
                                if overlap > 0:
                                    obj.pos += normal * overlap
                                    wall.velocity = Vector2(0, 0)

                # 11. 物体-边界墙对齐（100%修正，清零法向速度）
                for obj in [car] + boxes + bombs:
                    if not (hasattr(obj, 'is_exploded') and obj.is_exploded):
                        for boundary_wall in boundary_walls:
                            collided, _, _ = sat_collision(obj.get_vertices() if hasattr(obj, 'get_vertices') else obj.get_rotated_vertices(obj.pos), boundary_wall.get_vertices())
                            if collided:
                                normal = pipeline.calculate_wall_normal(obj, boundary_wall)
                                overlap = pipeline.calculate_sat_penetration(obj, boundary_wall)
                                if overlap > 0:
                                    obj.pos += normal * overlap
                                    v_normal = normal * obj.velocity.dot(normal)
                                    obj.velocity -= v_normal

            # 检测炸弹爆炸
            for bomb in bombs:
                if not bomb.is_exploded:
                    bomb.is_in_wall = BombDetection.check_in_wall(bomb, walls)
                    if bomb.is_in_wall and bomb.in_wall_timer >= BOMB_EXPLOSION_DELAY:
                        destroyed_count = BombExplosion.explode(bomb, walls)
                        print(f"[炸弹] 炸弹爆炸! 炸毁 {destroyed_count} 个墙体")

            # 检测箱子到达目标点
            TargetDetection.check_box_target_collision(boxes, targets)

            # 检测链式碰撞（用于调试显示）
            chain_collision = pipeline.check_any_chain_collision(car, boxes, walls)

            # 速度钳制
            clamp_velocity(car, MAX_SPEED)
            for box in boxes:
                clamp_velocity(box, MAX_SPEED)
            for bomb in bombs:
                if not bomb.is_exploded:
                    clamp_velocity(bomb, MAX_SPEED)

        # 渲染
        screen.fill(COLOR_BG)

        # 绘制地图编辑器网格和UI
        map_editor.draw(screen, walls, boundary_walls)

        # 绘制鼠标悬停预览（仅编辑模式）
        if map_editor.enabled:
            mouse_pos = pygame.mouse.get_pos()
            cell_pos = map_editor._screen_to_cell(mouse_pos)
            if cell_pos:
                map_editor._draw_preview(screen, cell_pos)

        # 绘制网格（地图模式或自由模式都显示，但样式不同）
        if not map_editor.enabled:
            if not free_mode:
                # 地图模式：显示16x12地图区域的网格
                for x in range(map_editor.offset_x, map_editor.offset_x + 16 * 50 + 1, 50):
                    pygame.draw.line(screen, (50, 50, 60), (x, map_editor.offset_y), (x, map_editor.offset_y + 12 * 50), 1)
                for y in range(map_editor.offset_y, map_editor.offset_y + 12 * 50 + 1, 50):
                    pygame.draw.line(screen, (50, 50, 60), (map_editor.offset_x, y), (map_editor.offset_x + 16 * 50, y), 1)
            else:
                # 自由模式：全屏网格
                for x in range(0, SCREEN_WIDTH, 50):
                    pygame.draw.line(screen, (50, 50, 60), (x, 0), (x, SCREEN_HEIGHT), 1)
                for y in range(0, SCREEN_HEIGHT, 50):
                    pygame.draw.line(screen, (50, 50, 60), (0, y), (SCREEN_WIDTH, y), 1)

        # 绘制墙体（先画边界墙，再画普通墙）
        for wall in boundary_walls + walls:
            wall.draw(screen)

        # 绘制所有箱子、炸弹、目标点和车
        for box in boxes:
            box.draw(screen)
        for bomb in bombs:
            if not bomb.is_exploded:
                bomb.draw(screen)
        for target in targets:
            target.draw(screen)
        car.draw(screen, show_debug=True)

        # 绘制预计算路径
        if path_executor.path:
            draw_path(screen, path_executor.path)

        # 绘制路径执行状态
        if path_executor.path and path_executor.current_index < len(path_executor.path):
            target = path_executor.path[path_executor.current_index]
            pygame.draw.circle(screen, (255, 255, 0), (int(target['x']), int(target['y'])), 8)
            # 绘制从车到目标的连线
            pygame.draw.line(screen, (255, 255, 0),
                           (car.pos.x, car.pos.y),
                           (target['x'], target['y']), 2)

        # 绘制算法输入状态
        if path_executor.is_running:
            font = FontManager.get_font(20)
            status_text = font.render("循迹执行中", True, (0, 255, 0))
            screen.blit(status_text, (SCREEN_WIDTH - 120, 10))

        # 绘制地图坐标（地图模式或自由模式都显示）
        if not map_editor.enabled:
            _draw_map_coordinates(screen, map_editor)

        # 绘制算法可视化标记
        if not map_editor.enabled:
            algorithm_visualizer.render(screen, map_editor.offset_x, map_editor.offset_y)

        # 绘制链式碰撞力传递
        if SHOW_FORCE_CHAIN and chain_collision and boxes:
            box = boxes[0]  # 使用第一个箱子显示
            pygame.draw.line(screen, COLOR_FORCE_CHAIN,
                           (car.pos.x, car.pos.y),
                           (box.pos.x, box.pos.y), 3)
            # 找到阻挡的墙
            from core.physics import sat_collision
            for wall in walls:
                if wall.type == "wall":
                    collided, _, _ = sat_collision(box.get_vertices(), wall.get_vertices())
                    if collided:
                        pygame.draw.line(screen, COLOR_FORCE_CHAIN, (box.pos.x, box.pos.y), (wall.pos.x, wall.pos.y), 3)
                        font = FontManager.get_font(24)
                        chain_text = font.render("力传递链条", True, COLOR_FORCE_CHAIN)
                        screen.blit(chain_text, (box.pos.x + 20, box.pos.y - 40))
                        break

        # 绘制调试信息
        if SHOW_DEBUG_INFO and not map_editor.enabled and show_info_panel:
            draw_debug_info(screen, car, dt, free_mode)

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
