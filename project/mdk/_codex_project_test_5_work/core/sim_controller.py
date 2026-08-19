"""
模拟控制层 + 视觉层模拟

在没有真实硬件和视觉模块时，用纯软件模拟替代：

  真实链路:  视觉层 → 算法层 → 中间层 → 控制层 → 电机

  模拟链路:  地图文件 → 算法层 → 中间层 → SimController → 物理引擎
                ↑                                                 │
                └─── VisionSim.get_state()（模拟视觉层输出）←─────┘

SimController: 接收 (vx, vy, omega) 世界坐标系速度指令，驱动车模
VisionSim:     从物理引擎读取当前状态，输出网格坐标（模拟 OpenART Plus 串口数据）
"""

import math
from typing import Dict, List, Tuple, Optional
from .physics import Vector2

# 与 map_editor / path_follower 一致的格点大小
CELL_SIZE = 50


# =============================================================================
# 坐标转换
# =============================================================================

def world_to_grid(wx: float, wy: float) -> Tuple[int, int]:
    """世界坐标 → 网格坐标"""
    return int(wx / CELL_SIZE), int(wy / CELL_SIZE)


def grid_to_world(gx, gy=None):
    """网格坐标 → 世界坐标（格点中心）。支持 grid_to_world((x,y)) 和 grid_to_world(x, y)"""
    if gy is None:
        gx, gy = gx
    return gx * CELL_SIZE + CELL_SIZE / 2, gy * CELL_SIZE + CELL_SIZE / 2


# =============================================================================
# SimController — 速度指令 → 车模
# =============================================================================

class SimController:
    """模拟控制层：将 (vx, vy, omega) 施加到车模"""

    def __init__(self):
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_omega = 0.0

    def apply(self, vx: float, vy: float, omega: float, car):
        """
        施加中间层输出的速度指令。

        Args:
            vx, vy: 世界坐标系平移速度（像素/秒）
            omega:  旋转角速度（度/秒）
            car:    Car 对象
        """
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_omega = omega
        car.target_velocity = Vector2(vx, vy)
        car.target_angular_velocity = omega

    def stop(self, car):
        """紧急停车"""
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_omega = 0.0
        car.target_velocity = Vector2(0, 0)
        car.target_angular_velocity = 0.0
        car.velocity = Vector2(0, 0)
        car.angular_velocity = 0.0


# =============================================================================
# VisionSim — 模拟视觉层输出
# =============================================================================

class VisionSim:
    """
    模拟 OpenART Plus 摄像头识别后通过串口发来的结构化数据。

    原地不动时输出与 scan_map 一致的初始数据；
    执行过程中每帧输出当前箱子位置（网格坐标）和车位姿。
    """

    @staticmethod
    def get_car_grid(car) -> Tuple[int, int]:
        """车当前位置 → 网格坐标"""
        return world_to_grid(car.pos.x, car.pos.y)

    @staticmethod
    def get_car_theta_deg(car) -> float:
        """车当前朝向（度）"""
        return car.angle

    @staticmethod
    def get_box_positions(boxes: List) -> Dict[int, Tuple[int, int]]:
        """
        所有箱子当前位置 → {box_id: (grid_x, grid_y)}

        box_id 按 boxes 列表顺序分配，与算法层的 box_id 对应。
        """
        return {i: world_to_grid(b.pos.x, b.pos.y) for i, b in enumerate(boxes)}

    @staticmethod
    def get_bomb_positions(bombs: List) -> Dict[int, Tuple[int, int]]:
        """所有炸弹当前位置 → {bomb_id: (grid_x, grid_y)}"""
        return {i: world_to_grid(b.pos.x, b.pos.y)
                for i, b in enumerate(bombs) if not b.is_exploded}

    @staticmethod
    def get_state(car, boxes, bombs, walls, goals) -> Dict:
        """
        获取完整视觉状态（模拟视觉层一帧输出）。

        返回格式与 scan_map 输出一致，可直接喂给算法层。
        """
        return {
            'car': VisionSim.get_car_grid(car),
            'boxes': [VisionSim.get_box_positions(boxes).get(i)
                      for i in range(len(boxes))],
            'box_positions': VisionSim.get_box_positions(boxes),
            'bomb_positions': VisionSim.get_bomb_positions(bombs),
            'walls': {world_to_grid(w.pos.x, w.pos.y)
                      for w in walls if w.type == "wall"},
        }

    @staticmethod
    def world_to_grid_pos(wx: float, wy: float) -> Tuple[int, int]:
        """快捷转换"""
        return world_to_grid(wx, wy)
