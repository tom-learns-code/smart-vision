# -*- coding: utf-8 -*-
"""
track.c 中所有结构体的 Python 翻译

C语言使用 #pragma pack(push, 1) 紧凑布局，Python中不需要。
但为了和C代码逐行对应，保持相同的字段名和语义。
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from .constants import MAX_STEPS, MAX_R, MAX_C


# C: struct State (约58-65行)
# 用列表模拟C的紧凑数组以提高性能，也可用dataclass
@dataclass
class State:
    """A*搜索中的状态（和C代码 struct State 完全对应）"""
    r: int = 0          # uint16_t: 机器人行
    c: int = 0          # uint16_t: 机器人列
    # b[3]: 炸弹箱位置（MAKE_POS编码），255表示不存在/已爆炸
    b: List[int] = field(default_factory=lambda: [255, 255, 255])
    # bx[3]: 推箱位置（MAKE_POS编码），255表示不存在/已完成
    bx: List[int] = field(default_factory=lambda: [255, 255, 255])
    mask: int = 0       # uint16_t: 已爆炸的炸弹位掩码（bit0/1/2分别对应b[0]/b[1]/b[2]）
    nudges: int = 0     # uint16_t: 微调(推弹)次数，不超过MAX_NUDGES


# C: struct Node (约67-72行)
@dataclass
class Node:
    """A*搜索节点"""
    s: State = field(default_factory=State)  # struct State
    g: int = 0          # int16_t: 实际代价
    h: int = 0          # int16_t: 启发式估计
    parent: int = -1    # int16_t: 父节点索引（-1表示无父节点）
    action: int = 0     # int32_t: 动作编码 (walk_r << 16) | (walk_c << 8) | push_dir


# C: struct SimState (约95行)
@dataclass
class SimState:
    """模拟执行状态（轻量版State，用于路径回放）"""
    r: int = 0          # uint16_t
    c: int = 0          # uint16_t
    mask: int = 0       # uint16_t: 爆炸掩码
    b: List[int] = field(default_factory=lambda: [255, 255, 255])   # uint16_t[3]
    bx: List[int] = field(default_factory=lambda: [255, 255, 255])  # uint16_t[3]


# C: struct QN (约102行)
@dataclass
class QN:
    """P2阶段A*队列节点（4维状态空间）"""
    rr: int = 0         # int8_t: 机器人行
    rc: int = 0         # int8_t: 机器人列
    br: int = 0         # int8_t: 箱子行
    bc: int = 0         # int8_t: 箱子列
    parent: int = -1    # int16_t: 父节点索引
    dist: int = 0       # int16_t: g值（已走步数）
    action: int = 0     # int8_t: 动作方向(0-3)
    pad: int = 0        # uint8_t: 对齐填充（Python不需要，保留字段名）


# C: Coord (约143行)
@dataclass
class Coord:
    """坐标"""
    r: int = 0
    c: int = 0


# C: struct POI (约670行)
@dataclass
class POI:
    """TSP兴趣点"""
    r: int = 0          # 行
    c: int = 0          # 列
    b_mask: int = 0     # 在此位置能观察到的箱子掩码
    t_mask: int = 0     # 在此位置能观察到的目标掩码


# ==================== PhaseResult 输出结构 ====================

@dataclass
class PhaseResult:
    """阶段求解结果（对应C代码中main.c里的PhaseResult）"""
    path_length: int = 0
    path: List[int] = field(default_factory=lambda: [0] * MAX_STEPS)  # int8_t[MAX_STEPS]
    end_px: int = 0     # 最终机器人x坐标（列）
    end_py: int = 0     # 最终机器人y坐标（翻转后的行=11-r）
    updated_map: List[List[str]] = field(default_factory=lambda: [[' '] * MAX_C for _ in range(MAX_R)])
    # P2阶段用
    bomb_wall_map: Optional[dict] = None  # 可选扩展字段
