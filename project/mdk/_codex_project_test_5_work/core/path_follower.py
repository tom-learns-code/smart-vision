"""
中间层 - 路径跟随器 (Path Follower)

状态机驱动的 Action 执行器。
消费算法层输出的 Action 序列，输出 (vx, vy, omega) 速度指令（世界坐标系）。

职责:
  1. 解析 Action 序列并逐条执行
  2. 位置闭环跟踪（P 控制器）
  3. 朝向闭环控制（P 控制器）
  4. 到达判定与 Action 切换

不负责:
  - 路径规划（→ 算法层）
  - 逆运动学 / 电机驱动（→ 控制层）
  - 环境感知 / 偏差检测（第三阶段再加）

接口:
  输入: Action 序列 + 每帧位姿 (x, y, theta) + 箱子位置
  输出: VelocityCmd(vx, vy, omega) 世界坐标系
"""

import math
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum, auto


# =============================================================================
# 参数
# =============================================================================

# 网格 → 世界坐标转换的格子边长（与 map_editor 一致）
CELL_SIZE = 50

# 方向 → 单位向量
DIRECTION_VECTORS = {
    'UP':    (0, -1),
    'DOWN':  (0,  1),
    'LEFT':  (-1,  0),
    'RIGHT': (1,   0),
}


def grid_to_world(grid_pos: Tuple[int, int]) -> Tuple[float, float]:
    """网格坐标 → 世界坐标（格点中心）"""
    return (
        grid_pos[0] * CELL_SIZE + CELL_SIZE / 2,
        grid_pos[1] * CELL_SIZE + CELL_SIZE / 2,
    )


# =============================================================================
# 数据结构
# =============================================================================

class FollowerState(Enum):
    IDLE    = auto()   # 无任务，等待 Action
    MOVING  = auto()   # 自由移动中 (free_move)
    PUSHING = auto()   # 推动中 (push_box / push_bomb)
    WAITING = auto()   # 等待中 (wait)
    OBSERVE = auto()   # 观察识别中 (observe)
    DONE    = auto()   # 全部完成
    STUCK   = auto()   # 卡住 / 超时


class PushPhase(Enum):
    APPROACH = auto()  # 阶段1: 车走到推动位置
    PUSH     = auto()  # 阶段2: 执行推动


@dataclass
class VelocityCmd:
    """速度指令（世界坐标系）"""
    vx:    float = 0.0   # 世界 X 方向速度（像素/秒）
    vy:    float = 0.0   # 世界 Y 方向速度（像素/秒）
    omega: float = 0.0   # 旋转角速度（度/秒）

    ZERO: 'VelocityCmd' = None  # type: ignore  # 类变量，下面补


VelocityCmd.ZERO = VelocityCmd(0, 0, 0)


@dataclass
class FollowerStatus:
    """供外部查询的当前状态快照"""
    state:       FollowerState = FollowerState.IDLE
    action_idx:  int = 0           # 当前是第几个 Action
    total:       int = 0           # Action 总数
    push_phase:  Optional[PushPhase] = None
    waypoint_at: int = 0           # 当前路径点下标
    waypoints_total: int = 0       # 当前段路径点总数
    elapsed:     float = 0.0       # 当前 Action 已耗时（秒）


# =============================================================================
# 路径跟随器
# =============================================================================

class PathFollower:
    """中间层 - 路径跟随器"""

    # ---------- 可调参数 ----------
    KP_POS:   float = 5.0       # 位置 P 增益
    KP_ANGLE: float = 6.0       # 角度 P 增益
    MAX_V:    float = 300.0     # 最大平移速度（像素/秒）
    MAX_OMEGA: float = 360.0    # 最大旋转速度（度/秒）
    PUSH_V:   float = 150.0     # 推箱子时平移速度
    ARRIVE:   float = 5.0       # 到达判定距离（像素）
    WP_ARRIVE: float = 6.0       # waypoint 切换距离（像素），平衡收敛速度与防切角
    TIMEOUT:  float = 30.0      # 单个 Action 超时（秒）

    # ---------- 构造 ----------

    def __init__(self):
        self.state = FollowerState.IDLE
        self.actions: List[Dict] = []
        self.idx: int = 0                     # 当前 Action 下标

        # 自由移动内部状态
        self._wps: List[Tuple[float, float]] = []   # waypoints 世界坐标
        self._wp_i: int = 0

        # 推动内部状态
        self._push_phase = PushPhase.APPROACH
        self._push_pos: Optional[Tuple[float, float]] = None   # 推动位置 世界坐标
        self._car_end: Optional[Tuple[float, float]] = None    # 推完后车位置 世界坐标
        self._push_vec: Optional[Tuple[float, float]] = None   # 推方向单位向量 (dx, dy)
        self._box_target_grid: Optional[Tuple[int, int]] = None # 箱子目标格点

        # 等待
        self._wait_left: float = 0.0

        # 计时
        self._elapsed: float = 0.0

        # 观察状态
        self._observe_left: float = 0.0  # 观察剩余时间（秒）

        # 推箱安全超时
        self._push_elapsed: float = 0.0  # PUSH 阶段累计时间
        self._push_speed: float = 150.0  # 当前推速（延续APPROACH减速）
        self._push_stop: float = -1.0   # 推达后停顿计时（-1=未触发）

        # 当前输出
        self.cmd = VelocityCmd.ZERO

    # =========================================================================
    # 公开接口
    # =========================================================================

    def load_actions(self, actions: List[Dict]):
        """加载算法层 Action 序列，从第一条开始执行"""
        self.actions = actions
        self.idx = 0
        if not actions:
            self.state = FollowerState.DONE
        else:
            self.state = FollowerState.IDLE
        self._elapsed = 0.0
        if actions:
            self._enter_action()

    def update(self,
               car_x: float, car_y: float, car_theta_deg: float,
               boxes: Dict[int, Tuple[int, int]],
               dt: float) -> VelocityCmd:
        """
        每帧调用。

        Args:
            car_x, car_y:      车世界坐标
            car_theta_deg:     车朝向（度）
            boxes:             {box_id: (grid_x, grid_y)} 各箱子当前位置
            dt:                帧间隔（秒）
        Returns:
            VelocityCmd: 世界坐标系速度指令
        """
        if self.state in (FollowerState.IDLE, FollowerState.DONE):
            return VelocityCmd.ZERO

        self._elapsed += dt

        # 超时
        if self._elapsed > self.TIMEOUT:
            self.state = FollowerState.STUCK
            return VelocityCmd.ZERO

        action = self.actions[self.idx]
        atype = action['type']

        # --- 分发 ---
        if atype == 'free_move':
            self.cmd = self._tick_free_move(action, car_x, car_y, car_theta_deg)
        elif atype == 'push_box':
            self.cmd = self._tick_push(action, car_x, car_y, car_theta_deg, boxes, dt)
        elif atype == 'push_bomb':
            self.cmd = self._tick_push(action, car_x, car_y, car_theta_deg, boxes, dt)
        elif atype == 'wait':
            self.cmd = self._tick_wait(dt)
        elif atype == 'observe':
            self.cmd = self._tick_observe(action, car_theta_deg, dt)
        else:
            # 未知类型，跳过
            self.cmd = VelocityCmd.ZERO
            self._advance()

        return self.cmd

    def get_status(self) -> FollowerStatus:
        return FollowerStatus(
            state=self.state,
            action_idx=self.idx,
            total=len(self.actions),
            push_phase=(self._push_phase
                        if self.state == FollowerState.PUSHING else None),
            waypoint_at=self._wp_i,
            waypoints_total=len(self._wps),
            elapsed=self._elapsed,
        )

    def current_action_type(self) -> str:
        """获取当前 action 类型（供外部检测 end_of_phase1 等特殊类型）"""
        if self.idx < len(self.actions):
            return self.actions[self.idx].get('type', '')
        return ''

    def is_done(self) -> bool:
        return self.state == FollowerState.DONE

    def is_stuck(self) -> bool:
        return self.state == FollowerState.STUCK

    def force_advance(self):
        """外部强制跳到下一个 Action (箱子被标记完成时调用)"""
        self._advance()

    def resume_after_phase1(self):
        """end_of_phase1 后恢复执行：跳过标记 action，进入 P2 动作序列"""
        self._advance()

    def reset(self):
        self.state = FollowerState.IDLE
        self.actions.clear()
        self.idx = 0
        self._wps.clear()
        self._wp_i = 0
        self._push_phase = PushPhase.APPROACH
        self._push_pos = None
        self._car_end = None
        self._push_vec = None
        self._box_target_grid = None
        self._wait_left = 0.0
        self._observe_left = 0.0
        self._push_elapsed = 0.0
        self._push_stop = -1.0
        self._elapsed = 0.0
        self.cmd = VelocityCmd.ZERO

    # =========================================================================
    # Action 切换
    # =========================================================================

    def _advance(self):
        """完成当前 Action，尝试切到下一条"""
        self.idx += 1
        self._elapsed = 0.0
        if self.idx >= len(self.actions):
            self.state = FollowerState.DONE
        else:
            self._enter_action()

    def _enter_action(self):
        """进入新 Action 时的初始化"""
        action = self.actions[self.idx]
        atype = action['type']

        if atype == 'free_move':
            self._init_free_move(action)
            self.state = FollowerState.MOVING
        elif atype in ('push_box', 'push_bomb'):
            self._init_push(action)
            self.state = FollowerState.PUSHING
        elif atype == 'wait':
            self._wait_left = action.get('duration', 0.0)
            self.state = FollowerState.WAITING
        elif atype == 'observe':
            self._init_observe(action)
            self.state = FollowerState.OBSERVE
        elif atype == 'end_of_phase1':
            # end_of_phase1: 标记动作，保持 IDLE 等待外部处理
            # 外部检测到 state==IDLE 且 action type 为 end_of_phase1 后:
            #   收集ID → 调用P2 → 扩展actions → pf.resume_after_phase1()
            self.state = FollowerState.IDLE
        else:
            self._advance()  # 未知类型跳过

    # =========================================================================
    # free_move 初始化
    # =========================================================================

    def _init_free_move(self, action: Dict):
        wps_raw = action.get('waypoints')
        target_grid = tuple(action['target'])

        if wps_raw and len(wps_raw) >= 2:
            # 使用算法层给的 waypoints
            self._wps = [grid_to_world(tuple(p)) for p in wps_raw]
        else:
            # fallback: 只有终点，车从当前位置直接开过去
            self._wps = [grid_to_world(target_grid)]

        self._wp_i = 0

    # =========================================================================
    # push 初始化
    # =========================================================================

    def _init_push(self, action: Dict):
        meta = action['push_meta']
        push_dir = meta['push_dir']
        dv = DIRECTION_VECTORS[push_dir]

        obj_start = tuple(meta.get('bomb_start', meta.get('box_start')))
        car_start_grid = (obj_start[0] - dv[0], obj_start[1] - dv[1])

        self._push_pos = grid_to_world(car_start_grid)
        self._push_vec = (float(dv[0]), float(dv[1]))
        target_grid = tuple(meta.get('bomb_target', meta.get('box_target')))
        self._push_target_world = grid_to_world(target_grid)
        self._car_end = grid_to_world(tuple(action['target']))
        self._push_is_bomb = (action['type'] == 'push_bomb')
        self._push_phase = PushPhase.APPROACH
        self._wps = [self._push_pos]

    # =========================================================================
    # 每帧：free_move
    # =========================================================================

    def _tick_free_move(self, action: Dict,
                        cx: float, cy: float, _theta: float) -> VelocityCmd:
        target_theta = action.get('theta')

        # 到达当前 waypoint → 前进
        while self._wp_i < len(self._wps):
            tx, ty = self._wps[self._wp_i]
            if _dist(cx, cy, tx, ty) < self.WP_ARRIVE:
                self._wp_i += 1
            else:
                break

        # 所有路点到达后，若需要转向则等角度到位再 advance
        if self._wp_i >= len(self._wps):
            if target_theta is not None:
                diff = target_theta - _theta
                # 归一化到 [-180, 180]
                while diff > 180:
                    diff -= 360
                while diff < -180:
                    diff += 360
                if abs(diff) < 5.0:  # 角度误差 < 5° 才算到位
                    self._advance()
                    return VelocityCmd.ZERO
                # 角度未到位：继续原地转向
                omega = self._steer_angle(_theta, target_theta)
                return VelocityCmd(vx=0.0, vy=0.0, omega=omega)
            self._advance()
            return VelocityCmd.ZERO

        tx, ty = self._wps[self._wp_i]
        vx, vy = self._steer_pos(cx, cy, tx, ty, self.MAX_V)
        omega = self._steer_angle(_theta, target_theta)
        return VelocityCmd(vx=vx, vy=vy, omega=omega)

    # =========================================================================
    # 每帧：push_box / push_bomb
    # =========================================================================

    def _tick_push(self, action: Dict,
                   cx: float, cy: float, _theta: float,
                   boxes: Dict[int, Tuple[int, int]],
                   dt: float = 0.016) -> VelocityCmd:
        meta = action['push_meta']
        obj_id = meta.get('bomb_id', meta.get('box_id'))

        # 阶段1: 走到推动位置
        if self._push_phase == PushPhase.APPROACH:
            px, py = self._push_pos
            if _dist(cx, cy, px, py) < self.ARRIVE:
                self._push_phase = PushPhase.PUSH
                self._push_elapsed = 0.0  # 进入PUSH时清零计时
                # 沿用减速时的速度，不平滑跳变
                ax, ay = self._steer_pos(cx, cy, px, py, self.MAX_V)
                self._push_speed = math.hypot(ax, ay) or self.PUSH_V * 0.3
            else:
                vx, vy = self._steer_pos(cx, cy, px, py, self.MAX_V)
                omega = self._steer_angle(_theta, action.get('theta'))
                return VelocityCmd(vx=vx, vy=vy, omega=omega)

        # 阶段2: 推动 — 直接沿推动方向匀速推进
        pvx, pvy = self._push_vec
        self._push_elapsed += dt  # 超时检测用

        # 单轴到达判定：只看推动方向的轴
        obj_world = boxes.get(obj_id)
        if obj_world is not None:
            if abs(pvx) > 0:
                axis_dist = abs(obj_world[0] - self._push_target_world[0])
            else:
                axis_dist = abs(obj_world[1] - self._push_target_world[1])
            if axis_dist < self.ARRIVE:
                # 推完停顿0.15s再进入下一步
                if self._push_stop < 0:
                    self._push_stop = 0.15
                self._push_stop -= dt
                if self._push_stop <= 0:
                    self._push_stop = -1
                    self._advance()
                return VelocityCmd.ZERO
            # 安全超时：推了 > 2s 且物体在 2 格内 → 强制 advance
            if self._push_elapsed > 2.0 and axis_dist < 1.5 * CELL_SIZE:
                self._advance()
                return VelocityCmd.ZERO

        # 沿 push_dir 推（从APPROACH速度平滑上升到PUSH_V）
        t = min(self._push_elapsed / 0.3, 1.0)  # 0.3秒内线性过渡
        spd = self._push_speed + (self.PUSH_V - self._push_speed) * t
        vx = spd * pvx
        vy = spd * pvy
        omega = self._steer_angle(_theta, action.get('theta'))
        return VelocityCmd(vx=vx, vy=vy, omega=omega)

    # =========================================================================
    # 每帧：wait
    # =========================================================================

    def _tick_wait(self, dt: float) -> VelocityCmd:
        self._wait_left -= dt
        if self._wait_left <= 0.0:
            self._advance()
        return VelocityCmd.ZERO

    # =========================================================================
    # observe 初始化和执行
    # =========================================================================

    def _init_observe(self, action: Dict):
        """初始化观察动作"""
        from .config import OBSERVE_DURATION
        self._observe_left = OBSERVE_DURATION

    def _tick_observe(self, action: Dict, theta_deg: float,
                      dt: float) -> VelocityCmd:
        """Turn toward the object, then hold still for the exposure window."""
        from .config import OBSERVE_ANGLE_TOLERANCE_DEG, OBSERVE_DURATION

        target_theta = action.get('theta')
        if target_theta is not None:
            diff = target_theta - theta_deg
            while diff > 180.0:
                diff -= 360.0
            while diff < -180.0:
                diff += 360.0
            if abs(diff) > OBSERVE_ANGLE_TOLERANCE_DEG:
                self._observe_left = OBSERVE_DURATION
                return VelocityCmd(0.0, 0.0,
                                   self._steer_angle(theta_deg, target_theta))

        self._observe_left -= dt
        if self._observe_left <= 0.0:
            self._advance()
        return VelocityCmd.ZERO

    # =========================================================================
    # P 控制器
    # =========================================================================

    def _steer_pos(self, cx: float, cy: float,
                   tx: float, ty: float,
                   vmax: float) -> Tuple[float, float]:
        """位置 P 控制: 当前 → 目标，输出 (vx, vy) 世界坐标"""
        dx = tx - cx
        dy = ty - cy
        dist = math.hypot(dx, dy)
        if dist < self.ARRIVE:
            return 0.0, 0.0
        # 比例 + 抑制远处过速
        v = min(self.KP_POS * dist, vmax)
        return v * dx / dist, v * dy / dist

    def _steer_angle(self, current_deg: float,
                     target_deg: Optional[float]) -> float:
        """朝向 P 控制。target_deg=None 时不控制朝向"""
        if target_deg is None:
            return 0.0
        diff = target_deg - current_deg
        # 归一化到 [-180, 180]
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        omega = self.KP_ANGLE * diff
        return max(-self.MAX_OMEGA, min(self.MAX_OMEGA, omega))


# =============================================================================
# 工具
# =============================================================================

def _dist(x1, y1, x2, y2) -> float:
    return math.hypot(x2 - x1, y2 - y1)
