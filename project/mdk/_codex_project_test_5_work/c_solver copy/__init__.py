# -*- coding: utf-8 -*-
"""
c_solver — track.c 的 Python 翻译

用法:
    from c_solver import run_phase1, run_phase2, reset_globals, read_map

    # 读取地图
    raw_map = read_map("maps_export/map0.txt")

    # 重置全局状态（每次求解前调用）
    reset_globals()

    # 阶段一
    p1_res = PhaseResult()
    success = run_phase1(raw_map, p1_res, mode=4, skip_flag=0)

    # 阶段二
    p2_res = PhaseResult()
    success2 = run_phase2([2, 3, 1], [1, 2, 3], p2_res)
"""

from .track_port import (
    run_phase1,
    run_phase2,
    reset_globals,
)
from .types import PhaseResult

from .constants import MAX_R, MAX_C


def read_map(filepath: str):
    """读取地图txt文件，返回12x16的字符串列表（和C版 raw_map 格式完全一致）

    地图文件格式：16列x12行的ASCII字符网格
    # = 墙
    @ = 机器人起始位置
    * = 炸弹箱（需要炸的墙的位置标记）
    $ = 推箱
    . = 目标点
    + = 机器人+目标重合
    - 或 空格 = 空地
    """
    raw_map = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.rstrip('\n\r')
            if len(line) >= MAX_C:
                raw_map.append(list(line[:MAX_C]))
            elif len(line) > 0:
                # 补齐到16列
                row = list(line)
                while len(row) < MAX_C:
                    row.append(' ')
                raw_map.append(row)
    # 确保有12行
    while len(raw_map) < MAX_R:
        raw_map.append([' '] * MAX_C)
    return raw_map[:MAX_R]
