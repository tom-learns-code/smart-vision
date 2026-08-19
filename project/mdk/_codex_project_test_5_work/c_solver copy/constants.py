# -*- coding: utf-8 -*-
"""
track.c 中的所有 #define 宏常量，一字不差地翻译
"""

# ==================== 地图尺寸参数 ====================
MAX_R = 12
MAX_C = 16
MAX_ITEMS = 3
MAX_NUDGES = 3

# ==================== 搜索规模限制 ====================
MAX_NODES_1 = 1000      # 第一轮：小规模搜索节点限制（快速筛选）
MAX_NODES = 3000        # 第二轮：无解时的极限二阶节点数
HASH_SIZE = 8192
MAX_P2_Q = 10000
TRACK_NUM = 4           # 赛道编号，控制"看几个就停"的阈值
WEIGHT_FACTOR = 4       # 启发式权重因子
MAX_STEPS = 400

# ==================== 超时控制 ====================
# GET_CURRENT_MS() 在Python中用 time.perf_counter() * 1000 替代
MAX_SOLVE_TIME_MS = 10500

# ==================== 位运算坐标宏 ====================
# C: #define GET_R(pos) ((pos) >> 4)
def GET_R(pos):
    """从编码位置提取行号"""
    return pos >> 4

# C: #define GET_C(pos) ((pos) & 15)
def GET_C(pos):
    """从编码位置提取列号"""
    return pos & 15

# C: #define MAKE_POS(r, c) (((r) << 4) | (c))
def MAKE_POS(r, c):
    """将(行,列)编码为单个整数"""
    return (r << 4) | c
