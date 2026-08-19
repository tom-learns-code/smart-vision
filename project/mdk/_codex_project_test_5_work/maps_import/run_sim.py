"""
模拟器入口 — 算法求解 + 可视化回放

用法:
    python run_sim.py [地图文件名]
    python run_sim.py              # 默认 map0.txt

键盘:
    T      — 开始/暂停 自动执行
    R      — 重新求解
    SPACE  — 重置
    LEFT/RIGHT — 切换地图
    L      — 地图选择器
    ESC    — 退出
"""

import pygame
import sys
import os
import math
import json
from typing import Tuple

sys.stdout.reconfigure(encoding='utf-8')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.config import (SCREEN_WIDTH, SCREEN_HEIGHT, FPS, COLOR_BG, COLOR_WHITE,
                         COLOR_YELLOW, COLOR_RED, OBJECT_SIZE, COLOR_BOX, COLOR_TARGET,
                         COLOR_CAR, COLOR_WALL, COLOR_BOMB,
                         BOMB_EXPLOSION_DELAY, MAX_SPEED, EDITOR_GRID_SIZE)
from core.map_analysis import load_map_from_file, scan_map, get_walls_in_3x3
from core.game_factory import GameFactory
from core.wall import Wall
from core.physics import Vector2, clamp_velocity
from core.collision_pipeline import CollisionPipeline
from core.adaptive_physics import AdaptivePhysics
from core.game_logic import BombDetection, BombExplosion, TargetDetection
from core.path_follower import PathFollower, FollowerState
from core.sim_controller import SimController, world_to_grid, grid_to_world, CELL_SIZE
from core.font_manager import FontManager

# ---- 地图尺寸 ----
MAP_W = 16
MAP_H = 12
MAP_PX_W = MAP_W * CELL_SIZE
MAP_PX_H = MAP_H * CELL_SIZE
OFFSET_X = (SCREEN_WIDTH - MAP_PX_W) // 2
OFFSET_Y = (SCREEN_HEIGHT - MAP_PX_H) // 2


# =============================================================================
# 从地图数据创建游戏对象
# =============================================================================

def create_objects_from_map(map_data, offset_x=OFFSET_X, offset_y=OFFSET_Y):
    walls_raw = map_data['walls']
    boxes_list = map_data.get('boxes_start', map_data.get('boxes', []))
    goals_set = map_data['goals']
    bombs_list = map_data.get('bombs', [])
    car_grid = map_data.get('car_start', map_data.get('car'))
    width = map_data['width']
    height = map_data['height']

    inner_walls = []
    boundary_walls = []
    for gx, gy in walls_raw:
        wx, wy = grid_to_world(gx, gy)
        is_boundary = (gx == 0 or gx == width - 1 or gy == 0 or gy == height - 1)
        w = Wall(wx + offset_x, wy + offset_y, CELL_SIZE, CELL_SIZE,
                COLOR_WALL, is_boundary=is_boundary, is_indestructible=is_boundary)
        if is_boundary:
            boundary_walls.append(w)
        else:
            inner_walls.append(w)
    walls = inner_walls + boundary_walls  # 保持兼容: walls = 所有墙

    boxes = []
    for gx, gy in boxes_list:
        wx, wy = grid_to_world(gx, gy)
        box = GameFactory.create_box()
        box.pos = Vector2(wx + offset_x, wy + offset_y)
        box.prev_pos = Vector2(wx + offset_x, wy + offset_y)
        box.velocity = Vector2(0, 0)
        boxes.append(box)

    bombs = []
    for gx, gy in bombs_list:
        wx, wy = grid_to_world(gx, gy)
        bomb = GameFactory.create_bomb()
        bomb.pos = Vector2(wx + offset_x, wy + offset_y)
        bomb.prev_pos = Vector2(wx + offset_x, wy + offset_y)
        bomb.velocity = Vector2(0, 0)
        bombs.append(bomb)

    targets = []
    for gx, gy in goals_set:
        wx, wy = grid_to_world(gx, gy)
        tgt = GameFactory.create_target()
        tgt.pos = Vector2(wx + offset_x, wy + offset_y)
        targets.append(tgt)

    wx_car, wy_car = grid_to_world(car_grid[0], car_grid[1])
    car = GameFactory.create_car()
    car.pos = Vector2(wx_car + offset_x, wy_car + offset_y)
    car.prev_pos = Vector2(wx_car + offset_x, wy_car + offset_y)
    car.velocity = Vector2(0, 0)
    car.angle = 0.0
    car.target_velocity = Vector2(0, 0)
    car.target_angular_velocity = 0.0

    return car, boxes, bombs, targets, walls, boundary_walls


# =============================================================================
# 简化物理
# =============================================================================

def _separate(obj1, obj2):
    """Action切换时的重叠分离"""
    if not obj1.rect.colliderect(obj2.rect):
        return
    dx = obj2.pos.x - obj1.pos.x
    dy = obj2.pos.y - obj1.pos.y
    dist = math.hypot(dx, dy) or 1.0
    nx, ny = dx / dist, dy / dist
    overlap = (obj1.size + obj2.size) / 2 - dist
    if overlap > 0:
        obj1.pos.x -= nx * overlap * 0.5
        obj1.pos.y -= ny * overlap * 0.5
        obj2.pos.x += nx * overlap * 0.5
        obj2.pos.y += ny * overlap * 0.5


# =============================================================================
# 渲染
# =============================================================================

def draw_grid(screen):
    for x in range(MAP_W + 1):
        px = OFFSET_X + x * CELL_SIZE
        pygame.draw.line(screen, (50, 50, 60), (px, OFFSET_Y),
                         (px, OFFSET_Y + MAP_PX_H), 1)
    for y in range(MAP_H + 1):
        py = OFFSET_Y + y * CELL_SIZE
        pygame.draw.line(screen, (50, 50, 60), (OFFSET_X, py),
                         (OFFSET_X + MAP_PX_W, py), 1)
    font = FontManager.get_font(16)
    for x in range(MAP_W):
        text = font.render(str(x), True, (150, 150, 150))
        screen.blit(text, (OFFSET_X + x * CELL_SIZE + 20, OFFSET_Y - 16))
    for y in range(MAP_H):
        text = font.render(str(y), True, (150, 150, 150))
        screen.blit(text, (OFFSET_X - 18, OFFSET_Y + y * CELL_SIZE + 18))


def draw_targets(screen, targets, occupied_goals=None):
    if occupied_goals is None:
        occupied_goals = set()
    for t in targets:
        cx, cy = t.pos.x, t.pos.y
        g = world_to_grid(cx - OFFSET_X, cy - OFFSET_Y)
        if g in occupied_goals:
            s = 18
            pygame.draw.rect(screen, COLOR_TARGET,
                             (cx - s // 2, cy - s // 2, s, s))
        else:
            s = 15
            pygame.draw.line(screen, COLOR_TARGET, (cx - s, cy), (cx + s, cy), 3)
            pygame.draw.line(screen, COLOR_TARGET, (cx, cy - s), (cx, cy + s), 3)


def draw_waypoints(screen, waypoints, color):
    valid = [w for w in waypoints if w is not None]
    if len(valid) < 2:
        return
    pts = [(px + OFFSET_X, py + OFFSET_Y) for px, py in valid]
    pygame.draw.lines(screen, color, False, pts, 2)
    for px, py in pts:
        pygame.draw.circle(screen, color, (int(px), int(py)), 4)


def draw_status(screen, pf, auto_mode, recording=False, rec_idx=0):
    font = FontManager.get_font(18)
    y = 10
    st = pf.get_status()
    lines = [
        f"状态: {pf.state.name}",
        f"自动: {'开' if auto_mode else '关'}  |  "
        f"耗时: {st.elapsed:.1f}s",
        f"Action: {st.action_idx}/{st.total}",
        f"阶段: {st.push_phase.name if st.push_phase else '-'}  |  "
        f"路点: {st.waypoint_at}/{st.waypoints_total}",
        "",
        "T=自动  R=求解  M=录制",
        "SPACE=重置  ←→=切图  WASD=手动",
        "", f"录制: {'●' if recording else '○'}  文件#{rec_idx+1}",
    ]
    for line in lines:
        text = font.render(line, True, COLOR_WHITE)
        screen.blit(text, (SCREEN_WIDTH - 240, y))
        y += 22


# =============================================================================
# 文件浏览器（复用4.5.1）
# =============================================================================

class FileBrowser:
    def __init__(self, import_dir):
        self.import_dir = import_dir
        self.active = False
        self.selected_idx = 0
        self.files = []
        self._refresh()

    def _refresh(self):
        self.files = sorted(f for f in os.listdir(self.import_dir) if f.endswith('.txt'))
        if self.selected_idx >= len(self.files):
            self.selected_idx = max(0, len(self.files) - 1)

    def open(self):
        self._refresh()
        self.active = bool(self.files)
        self.selected_idx = 0
        return self.active

    def close(self):
        self.active = False

    def get_selected(self):
        if 0 <= self.selected_idx < len(self.files):
            return os.path.join(self.import_dir, self.files[self.selected_idx])
        return None

    def handle_event(self, event):
        if not self.active:
            return None
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                self.selected_idx = (self.selected_idx - 1) % len(self.files)
            elif event.key == pygame.K_DOWN:
                self.selected_idx = (self.selected_idx + 1) % len(self.files)
            elif event.key == pygame.K_RETURN:
                path = self.get_selected()
                self.close()
                return path
            elif event.key == pygame.K_ESCAPE:
                self.close()
        return None

    def draw(self, screen):
        if not self.active:
            return
        font = FontManager.get_font(26)
        font_title = FontManager.get_font(32)
        w, h = screen.get_size()
        overlay = pygame.Surface((w, h), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        screen.blit(overlay, (0, 0))
        pw, ph = 500, 400
        px, py = (w - pw) // 2, (h - ph) // 2
        pygame.draw.rect(screen, (40, 40, 55), (px, py, pw, ph))
        pygame.draw.rect(screen, (100, 100, 130), (px, py, pw, ph), 2)
        title = font_title.render("选择地图 (maps_import/)", True, (220, 220, 255))
        screen.blit(title, (px + 20, py + 10))
        pygame.draw.line(screen, (80, 80, 100), (px + 15, py + 50), (px + pw - 15, py + 50), 1)
        list_y = py + 65
        visible = min(len(self.files), 8)
        start = max(0, min(self.selected_idx - 3, len(self.files) - visible))
        for i in range(start, min(start + visible, len(self.files))):
            y = list_y + (i - start) * 36
            if i == self.selected_idx:
                pygame.draw.rect(screen, (60, 60, 120), (px + 15, y - 2, pw - 30, 32))
                color = (255, 255, 150)
                prefix = "> "
            else:
                color = (200, 200, 200)
                prefix = "  "
            text = font.render(f"{prefix}{self.files[i]}", True, color)
            screen.blit(text, (px + 25, y))


# =============================================================================
# 操作录制器
# =============================================================================

class ActionRecorder:
    """网格吸附录制: 追踪车和物体的网格位置变化, 生成 Action 序列"""

    def __init__(self, car_start, boxes_start, bombs_start, walls, goals, w, h,
                 offset_x=OFFSET_X, offset_y=OFFSET_Y):
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.w = w
        self.h = h
        self.car_start_grid = car_start
        self.boxes_start = list(boxes_start)
        self.bombs_start = list(bombs_start)
        self.goals = goals
        self.walls = walls
        self.recording = False
        self.actions = []
        self._car_grid_prev = None
        self._box_grids_prev = None
        self._bomb_grids_prev = None
        self._bomb_alive = None
        self._free_wps = []       # 当前 free_move 的路径点
        self._car_wps_start = None  # free_move 开始时车的位置
        self._pending_push = None  # (obj_type, obj_id, start, push_dir, steps, car_end)

    def start(self):
        self.recording = True
        self.actions = []
        self._car_grid_prev = self.car_start_grid
        self._box_grids_prev = list(self.boxes_start)
        self._bomb_grids_prev = list(self.bombs_start)
        self._bomb_alive = [True] * len(self.bombs_start)
        self._free_wps = [self.car_start_grid]
        self._car_wps_start = self.car_start_grid
        self._pending_push = None
        from core.map_analysis import compute_narrow_cells
        self._narrow_set = compute_narrow_cells(self.walls, self.w, self.h)
        print("[录制] 开始录制")

    def stop(self):
        if not self.recording:
            return None
        self._flush_free_move()
        self._flush_push()
        self.recording = False
        print(f"[录制] 停止, 共 {len(self.actions)} 条 Action")
        return self.actions

    def tick(self, car_wx, car_wy, box_positions, bomb_positions):
        """每帧调用: car_wx/wy=车世界坐标(去偏移), box/bomb_positions=世界坐标(去偏移)列表"""
        if not self.recording:
            return

        car_grid = world_to_grid(car_wx, car_wy)
        box_grids = [world_to_grid(bx, by) for bx, by in box_positions]
        bomb_grids = [world_to_grid(bx, by) for bx, by in bomb_positions]

        # 检测炸弹爆炸 (炸弹进入墙格)
        for i, bg in enumerate(bomb_grids):
            if self._bomb_alive[i] and bg in self.walls:
                self._bomb_alive[i] = False
                self._flush_push()  # 先结束当前push
                self.actions.append({
                    "type": "wait", "target": None, "theta": None,
                    "duration": 0.5, "reason": "bomb_explosion",
                })
                print(f"[录制] 炸弹{i} 在{bg}爆炸")

        # 检测推物
        push_detected = None  # (obj_type, obj_id, direction)
        for i, (bg_now, bg_prev) in enumerate(zip(box_grids, self._box_grids_prev)):
            if bg_now != bg_prev and bg_now != bg_prev:  # 箱子移动了
                d = (bg_now[0] - bg_prev[0], bg_now[1] - bg_prev[1])
                if abs(d[0]) + abs(d[1]) == 1:
                    push_detected = ('push_box', i, bg_prev, bg_now, d)
                    break
        if push_detected is None:
            for i, (bg_now, bg_prev) in enumerate(zip(bomb_grids, self._bomb_grids_prev)):
                if bg_now != bg_prev and self._bomb_alive[i]:
                    d = (bg_now[0] - bg_prev[0], bg_now[1] - bg_prev[1])
                    if abs(d[0]) + abs(d[1]) == 1:
                        push_detected = ('push_bomb', i, bg_prev, bg_now, d)
                        break

        if push_detected:
            obj_type, obj_id, start, end, d = push_detected
            dname = {(0,-1):'UP',(0,1):'DOWN',(-1,0):'LEFT',(1,0):'RIGHT'}[d]

            if self._pending_push and self._pending_push[0] == obj_type and \
               self._pending_push[1] == obj_id and self._pending_push[3] == dname:
                # 继续同方向推
                self._pending_push = (obj_type, obj_id, self._pending_push[2],
                                       dname, self._pending_push[4] + 1, car_grid)
            else:
                # 新推送或换方向
                self._flush_free_move()
                self._flush_push()
                self._pending_push = (obj_type, obj_id, start, dname, 1, car_grid)
        elif car_grid != self._car_grid_prev:
            # 车自由移动
            if self._pending_push:
                # 推完了, 结束push段
                self._flush_push()
            # 记录waypoint
            if not self._free_wps:
                self._free_wps = [self._car_wps_start or car_grid]
                self._car_wps_start = self._car_wps_start or car_grid
            # 去重连续同格
            if car_grid != self._free_wps[-1]:
                self._free_wps.append(car_grid)

        self._car_grid_prev = car_grid
        self._box_grids_prev = box_grids
        self._bomb_grids_prev = bomb_grids

    def _flush_free_move(self):
        if len(self._free_wps) >= 2 and self._free_wps[-1] != self._free_wps[0]:
            is_narrow = any(wp in self._narrow_set for wp in self._free_wps)
            self.actions.append({
                "type": "free_move",
                "target": self._free_wps[-1],
                "theta": None,
                "waypoints": list(self._free_wps),
                "narrow_passage": is_narrow,
            })
        self._free_wps = []
        self._car_wps_start = None

    def _flush_push(self):
        if self._pending_push is None:
            return
        obj_type, obj_id, start, dname, steps, car_end = self._pending_push
        dx, dy = {'UP':(0,-1),'DOWN':(0,1),'LEFT':(-1,0),'RIGHT':(1,0)}[dname]
        end = (start[0] + dx * steps, start[1] + dy * steps)
        # 检查推动过程中车所在位置是否经过窄道
        push_narrow = False
        cur = start
        for _ in range(steps):
            car_p = (cur[0] - dx, cur[1] - dy)
            if car_p in self._narrow_set:
                push_narrow = True
                break
            cur = (cur[0] + dx, cur[1] + dy)

        if obj_type == 'push_box':
            self.actions.append({
                "type": "push_box",
                "target": car_end,
                "theta": None,
                "push_meta": {
                    "box_id": obj_id,
                    "box_start": start, "box_target": end,
                    "push_dir": dname,
                },
                "narrow_passage": push_narrow,
            })
        else:
            self.actions.append({
                "type": "push_bomb",
                "target": car_end,
                "theta": None,
                "push_meta": {
                    "bomb_id": len(self.boxes_start) + obj_id,
                    "bomb_start": start, "bomb_target": end,
                    "wall_target": end,
                    "push_dir": dname,
                },
                "narrow_passage": push_narrow,
            })
        self._pending_push = None


# =============================================================================
# 主循环
# =============================================================================

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("推箱子+炸弹 算法求解模拟器")
    clock = pygame.time.Clock()

    base = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(base, 'maps_export')
    import_dir = os.path.join(base, 'maps_import')
    os.makedirs(import_dir, exist_ok=True)

    if len(sys.argv) > 1:
        fname = sys.argv[1]
        filepath = os.path.join(maps_dir, fname) if not os.path.isabs(fname) else fname
    else:
        mdir = maps_dir if os.path.isdir(maps_dir) and os.listdir(maps_dir) else import_dir
        files = sorted(f for f in os.listdir(mdir) if f.endswith('.txt'))
        filepath = os.path.join(mdir, files[0]) if files else None

    if filepath is None or not os.path.exists(filepath):
        print("无可用地图文件")
        return

    print(f"[模拟] 加载地图: {filepath}")

    # 动态导入求解器
    from run_solver import solve as solver_solve

    def load_and_solve():
        raw = load_map_from_file(filepath)
        info = scan_map(raw)
        result = solver_solve(raw)
        return raw, info, result

    raw, info, result = load_and_solve()

    def reset_objects():
        return create_objects_from_map(raw)

    car, boxes, bombs, targets, walls, boundary_walls = reset_objects()
    file_browser = FileBrowser(import_dir)

    pf = PathFollower()
    sim = SimController()
    pipeline = CollisionPipeline()
    adaptive = AdaptivePhysics()
    auto_mode = False
    manual_mode = False     # WASD手动控制
    recording = False       # 正在录制
    recorder = ActionRecorder(
        info['car'], info['boxes'], info['bombs'],
        info['walls'], info['goals'], info['width'], info['height'])
    record_file_idx = 0
    all_walls_original = set(info['walls'])

    if result['success'] and result['actions']:
        pf.load_actions(result['actions'])
        print(f"[模拟] 求解成功, {len(result['actions'])} 条 Action 已加载")
    else:
        print(f"[模拟] 求解失败: {result.get('error', '未知')}")

    running = True
    prev_action_idx = -1
    completed_boxes = set()
    occupied_goals = set()
    detonated_bombs = set()

    while running:
        dt = clock.tick(FPS) / 1000.0

        st = pf.get_status()
        if st.action_idx != prev_action_idx:
            prev_action_idx = st.action_idx
            car.velocity = Vector2(0, 0)
            car.target_velocity = Vector2(0, 0)
            for i, box in enumerate(boxes):
                if i not in completed_boxes:
                    box.velocity = Vector2(0, 0)
            for b in bombs:
                if not b.is_exploded:
                    b.velocity = Vector2(0, 0)
            for _ in range(3):
                for box in boxes:
                    _separate(car, box)
                for b in bombs:
                    if not b.is_exploded:
                        _separate(car, b)
                for i in range(len(boxes)):
                    for j in range(i + 1, len(boxes)):
                        _separate(boxes[i], boxes[j])
            # 检测炸弹爆炸: 炸弹碰到墙体 → 炸毁
            current_action = None
            if st.action_idx < len(result.get('actions', [])):
                current_action = result['actions'][st.action_idx]
            if current_action and current_action['type'] == 'wait' and current_action.get('reason') == 'bomb_explosion':
                for i, b in enumerate(bombs):
                    if i in detonated_bombs or b.is_exploded:
                        continue
                    g = world_to_grid(b.pos.x - OFFSET_X, b.pos.y - OFFSET_Y)
                    if g in all_walls_original:
                        # 炸弹在墙上 → 引爆
                        blast = get_walls_in_3x3(g, all_walls_original)
                        for w in walls[:]:
                            wg = world_to_grid(w.pos.x - OFFSET_X, w.pos.y - OFFSET_Y)
                            if wg in blast and not w.is_indestructible:
                                walls.remove(w)
                        b.is_exploded = True
                        detonated_bombs.add(i)
                        print(f"[模拟] 炸弹{i} 在 {g} 爆炸, 清除 {len(blast)} 面墙")

            print(f"[模拟] -> Action[{st.action_idx}], state={pf.state.name}")

        # 事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if file_browser.active:
                r = file_browser.handle_event(event)
                if r:
                    filepath = r
                    raw, info, result = load_and_solve()
                    car, boxes, bombs, targets, walls, boundary_walls = reset_objects()
                    pf.reset()
                    completed_boxes.clear()
                    occupied_goals.clear()
                    detonated_bombs.clear()
                    all_walls_original = set(info['walls'])
                    if result['success'] and result['actions']:
                        pf.load_actions(result['actions'])
                    auto_mode = False
                continue
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_l:
                    if file_browser.open():
                        print("[文件] 打开地图选择器")
                elif event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                    direction = 1 if event.key == pygame.K_RIGHT else -1
                    search_dir = import_dir if os.path.isdir(import_dir) and os.listdir(import_dir) else maps_dir
                    files_sorted = sorted(f for f in os.listdir(search_dir) if f.endswith('.txt'))
                    if files_sorted:
                        cur_name = os.path.basename(filepath)
                        try:
                            idx = files_sorted.index(cur_name)
                        except ValueError:
                            idx = 0
                        new_idx = (idx + direction) % len(files_sorted)
                        filepath = os.path.join(search_dir, files_sorted[new_idx])
                        print(f"[模拟] 切换: {files_sorted[new_idx]}")
                        raw, info, result = load_and_solve()
                        car, boxes, bombs, targets, walls, boundary_walls = reset_objects()
                        pf.reset()
                        completed_boxes.clear()
                        occupied_goals.clear()
                        detonated_bombs.clear()
                        all_walls_original = set(info['walls'])
                        if result['success'] and result['actions']:
                            pf.load_actions(result['actions'])
                        auto_mode = False
                elif event.key == pygame.K_t:
                    auto_mode = not auto_mode
                    print(f"[控制] 自动: {'开' if auto_mode else '关'}")
                elif event.key == pygame.K_r:
                    print("[控制] 重新求解...")
                    raw, info, result = load_and_solve()
                    car, boxes, bombs, targets, walls, boundary_walls = reset_objects()
                    pf.reset()
                    completed_boxes.clear()
                    occupied_goals.clear()
                    detonated_bombs.clear()
                    all_walls_original = set(info['walls'])
                    if result['success'] and result['actions']:
                        pf.load_actions(result['actions'])
                elif event.key == pygame.K_m:
                    # 录制切换
                    if recording:
                        # 停止录制并保存
                        actions = recorder.stop()
                        if actions:
                            record_file_idx += 1
                            fname = os.path.join(base, f'recorded_{record_file_idx:03d}.json')
                            with open(fname, 'w', encoding='utf-8') as f:
                                json.dump(actions, f, indent=2, ensure_ascii=False, default=str)
                            print(f"[录制] 已保存: {fname} ({len(actions)} actions)")
                        manual_mode = False
                        recording = False
                    else:
                        # 开始录制: 重置+开始
                        car, boxes, bombs, targets, walls, boundary_walls = reset_objects()
                        completed_boxes.clear()
                        occupied_goals.clear()
                        detonated_bombs.clear()
                        all_walls_original = set(info['walls'])
                        recorder = ActionRecorder(
                            info['car'], info['boxes'], info['bombs'],
                            info['walls'], info['goals'], info['width'], info['height'])
                        recorder.start()
                        recording = True
                        manual_mode = True
                        auto_mode = False
                        pf.reset()
                        sim.stop(car)
                # WASD 手动控制
                elif manual_mode:
                    if event.key == pygame.K_w:
                        car.target_velocity.y = -200
                    elif event.key == pygame.K_s:
                        car.target_velocity.y = 200
                    elif event.key == pygame.K_a:
                        car.target_velocity.x = -200
                    elif event.key == pygame.K_d:
                        car.target_velocity.x = 200
                elif event.key == pygame.K_SPACE:
                    print("[控制] 重置")
                    car, boxes, bombs, targets, walls, boundary_walls = reset_objects()
                    pf.reset()
                    completed_boxes.clear()
                    occupied_goals.clear()
                    detonated_bombs.clear()
                    all_walls_original = set(info['walls'])
                    if result['success'] and result['actions']:
                        pf.load_actions(result['actions'])
                    auto_mode = False
                    manual_mode = False
                    recording = recorder.stop() is not None  # 如果正在录制则停止不保存

            elif event.type == pygame.KEYUP and manual_mode:
                if event.key in (pygame.K_w, pygame.K_s):
                    car.target_velocity.y = 0
                elif event.key in (pygame.K_a, pygame.K_d):
                    car.target_velocity.x = 0

        # 录制模式: 手动驾驶 + 记录
        if recording and manual_mode:
            sim.apply(car.target_velocity.x, car.target_velocity.y, 0, car)
            car_wx = car.pos.x - OFFSET_X
            car_wy = car.pos.y - OFFSET_Y
            box_poses = [(b.pos.x - OFFSET_X, b.pos.y - OFFSET_Y) for b in boxes]
            bomb_poses = [(b.pos.x - OFFSET_X, b.pos.y - OFFSET_Y) for b in bombs]
            recorder.tick(car_wx, car_wy, box_poses, bomb_poses)

            # 自动停止: 所有箱子到目标
            goals_set = set(info['goals'])
            all_done = all(
                world_to_grid(box.pos.x - OFFSET_X, box.pos.y - OFFSET_Y) in goals_set
                for box in boxes
            )
            if all_done and len(completed_boxes) < len(boxes):
                for i, box in enumerate(boxes):
                    g = world_to_grid(box.pos.x - OFFSET_X, box.pos.y - OFFSET_Y)
                    if g in goals_set and i not in completed_boxes:
                        completed_boxes.add(i)
                        occupied_goals.add(g)

            if all_done:
                actions = recorder.stop()
                if actions:
                    record_file_idx += 1
                    fname = os.path.join(base, f'recorded_{record_file_idx:03d}.json')
                    with open(fname, 'w', encoding='utf-8') as f:
                        json.dump(actions, f, indent=2, ensure_ascii=False, default=str)
                    print(f"[录制] 自动完成! 已保存: {fname} ({len(actions)} actions)")
                recording = False
                manual_mode = False
                sim.stop(car)

        # 中间层更新
        if not file_browser.active and auto_mode and not pf.is_done() and not pf.is_stuck():
            car_wx = car.pos.x - OFFSET_X
            car_wy = car.pos.y - OFFSET_Y
            boxes_grid = {}
            for i, box in enumerate(boxes):
                boxes_grid[i] = (box.pos.x - OFFSET_X, box.pos.y - OFFSET_Y)
            bomb_offset = len(boxes)
            for i, b in enumerate(bombs):
                boxes_grid[bomb_offset + i] = (b.pos.x - OFFSET_X, b.pos.y - OFFSET_Y)
            cmd = pf.update(car_wx, car_wy, car.angle, boxes_grid, dt)
            if pf.is_done():
                print(f"[模拟] 全部完成!")
                auto_mode = False
            elif pf.is_stuck():
                print(f"[模拟] 卡住了!")
                auto_mode = False
        else:
            cmd = pf.cmd

        # 模拟控制
        if auto_mode:
            sim.apply(cmd.vx, cmd.vy, cmd.omega, car)
        elif not manual_mode:
            sim.stop(car)

        # ====== 物理更新 (4.3 完整管线) ======
        active_boxes = [b for i, b in enumerate(boxes) if i not in completed_boxes]
        active_bombs = [b for b in bombs if not b.is_exploded]
        all_objects = [car] + active_boxes + active_bombs
        adaptive.update_spatial_grid(all_objects, walls + boundary_walls)
        substeps = adaptive.adaptive_substeps(*all_objects)

        for _ in range(substeps):
            sub_dt = dt / substeps

            car.update(sub_dt)
            for box in active_boxes:
                box.update(sub_dt)
            for b in active_bombs:
                b.update(sub_dt)

            # 0. 边界墙碰撞（绝对不可穿过）
            for obj in [car] + active_boxes + active_bombs:
                for bw in boundary_walls:
                    pipeline.process_single_collision(obj, bw, sub_dt)

            # 1. 箱子-墙
            for box in active_boxes:
                pwalls = adaptive.optimized_collision_detection(box)
                for w in pwalls:
                    if w.type == "wall":
                        pipeline.process_single_collision(box, w, sub_dt)

            # 2. 车-墙
            pwalls = adaptive.optimized_collision_detection(car)
            for w in pwalls:
                if w.type == "wall":
                    pipeline.process_single_collision(car, w, sub_dt)

            # 2.5 炸弹-墙
            for b in active_bombs:
                pwalls = adaptive.optimized_collision_detection(b)
                for w in pwalls:
                    if w.type == "wall":
                        pipeline.process_single_collision(b, w, sub_dt)

            # 3. 车-箱
            for box in active_boxes:
                pipeline.process_single_collision(car, box, sub_dt, no_resistance=True)

            # 4. 车-炸弹
            for b in active_bombs:
                pipeline.process_single_collision(car, b, sub_dt, no_resistance=True)

            # 5. 箱-箱
            for box in active_boxes:
                pothers = adaptive.optimized_collision_detection(box)
                for other in pothers:
                    if other.type == "box":
                        pipeline.process_single_collision(box, other, sub_dt)
            for i, b1 in enumerate(active_boxes):
                for b2 in active_boxes[i+1:]:
                    pipeline.process_single_collision(b1, b2, sub_dt)

            # 6-7. 箱-炸弹 双向
            for box in active_boxes:
                for b in active_bombs:
                    pipeline.process_single_collision(box, b, sub_dt)
                    pipeline.process_single_collision(b, box, b, sub_dt)

            # 8. 炸弹-炸弹
            for i, b1 in enumerate(active_bombs):
                for b2 in active_bombs[i+1:]:
                    pipeline.process_single_collision(b1, b2, sub_dt)

            # 9. SAT位置对齐
            from core.physics import sat_collision
            for i, b1 in enumerate(active_boxes):
                for b2 in active_boxes[i+1:]:
                    collided, normal, overlap = sat_collision(b1.get_vertices(), b2.get_vertices(), b1.center, b2.center)
                    if collided and normal is not None and overlap > 2.0:
                        if abs(normal.x) > abs(normal.y):
                            an = Vector2(1,0) if normal.x > 0 else Vector2(-1,0)
                        else:
                            an = Vector2(0,1) if normal.y > 0 else Vector2(0,-1)
                        r = b1.mass / (b1.mass + b2.mass)
                        b1.pos -= an * overlap * r * 0.85
                        b2.pos += an * overlap * (1-r) * 0.85
            for b in active_bombs:
                for box in active_boxes:
                    collided, normal, overlap = sat_collision(b.get_vertices(), box.get_vertices(), b.center, box.center)
                    if collided and normal is not None and overlap > 2.0:
                        if abs(normal.x) > abs(normal.y):
                            an = Vector2(1,0) if normal.x > 0 else Vector2(-1,0)
                        else:
                            an = Vector2(0,1) if normal.y > 0 else Vector2(0,-1)
                        r = b.mass / (b.mass + box.mass)
                        b.pos -= an * overlap * r * 0.85
                        box.pos += an * overlap * (1-r) * 0.85

            # 10. 物体-墙对齐（炸弹跳过可破坏墙）
            for obj in [car] + active_boxes + active_bombs:
                pwalls = adaptive.optimized_collision_detection(obj)
                for w in pwalls:
                    if w.type != "wall":
                        continue
                    if obj.type == "bomb" and not w.is_indestructible:
                        continue
                    collided, _, _ = sat_collision(
                        obj.get_vertices() if hasattr(obj, 'get_vertices') else obj.get_rotated_vertices(obj.pos),
                        w.get_vertices())
                    if collided:
                        normal = pipeline.calculate_wall_normal(obj, w)
                        overlap_val = pipeline.calculate_sat_penetration(obj, w)
                        if overlap_val > 0:
                            obj.pos += normal * overlap_val

            # 11. 边界墙对齐
            for obj in [car] + active_boxes + active_bombs:
                for bw in boundary_walls:
                    collided, _, _ = sat_collision(
                        obj.get_vertices() if hasattr(obj, 'get_vertices') else obj.get_rotated_vertices(obj.pos),
                        bw.get_vertices())
                    if collided:
                        normal = pipeline.calculate_wall_normal(obj, bw)
                        overlap_val = pipeline.calculate_sat_penetration(obj, bw)
                        if overlap_val > 0:
                            obj.pos += normal * overlap_val
                            vn = normal * obj.velocity.dot(normal)
                            obj.velocity -= vn

        # 炸弹爆炸检测
        for b in bombs:
            if not b.is_exploded:
                b.is_in_wall = BombDetection.check_in_wall(b, walls)
                if b.is_in_wall and b.in_wall_timer >= BOMB_EXPLOSION_DELAY:
                    destroyed = BombExplosion.explode(b, walls)
                    if destroyed > 0:
                        boundary_walls = [w for w in boundary_walls if w in walls]
                        print(f"[炸弹] 爆炸! 炸毁 {destroyed} 面墙")

        # 箱子到达目标
        goals_set = set(info['goals'])
        for i, box in enumerate(boxes):
            if i in completed_boxes:
                continue
            g = world_to_grid(box.pos.x - OFFSET_X, box.pos.y - OFFSET_Y)
            if g in goals_set and g not in occupied_goals:
                completed_boxes.add(i)
                occupied_goals.add(g)
                box.velocity = Vector2(0, 0)
                # 如果PathFollower正在推这个箱子, 通知跳转
                st = pf.get_status()
                act = result['actions'][st.action_idx] if st.action_idx < len(result['actions']) else None
                if act and act['type'] == 'push_box' and act['push_meta']['box_id'] == i:
                    pf.force_advance()
                print(f"[模拟] 箱{i} 到达目标 {g}")

        # 速度钳制
        for obj in [car] + boxes + bombs:
            if hasattr(obj, 'is_exploded') and obj.is_exploded:
                continue
            clamp_velocity(obj, MAX_SPEED)

        # 渲染
        screen.fill(COLOR_BG)
        draw_grid(screen)
        for wall in walls:
            wall.draw(screen)
        for b in bombs:
            if not b.is_exploded:
                b.draw(screen)
        for i, box in enumerate(boxes):
            if i not in completed_boxes:
                box.draw(screen)
        draw_targets(screen, targets, occupied_goals)
        car.draw(screen, show_debug=True)

        st = pf.get_status()
        if (st.state == FollowerState.MOVING and st.waypoints_total > 1 and pf._wps):
            remaining = pf._wps[st.waypoint_at:]
            if remaining:
                draw_waypoints(screen, remaining, COLOR_YELLOW)

        draw_status(screen, pf, auto_mode, recording, record_file_idx)

        if pf.is_done():
            font = FontManager.get_font(36)
            text = font.render("全部完成!", True, (0, 255, 0))
            screen.blit(text, (SCREEN_WIDTH // 2 - 80, SCREEN_HEIGHT - 40))

        map_font = FontManager.get_font(20)
        name_text = map_font.render(os.path.basename(filepath), True, (200, 200, 200))
        screen.blit(name_text, (SCREEN_WIDTH - name_text.get_width() - 15, 12))

        file_browser.draw(screen)
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == '__main__':
    main()
