"""
游戏配置模块 - 集中管理所有配置参数
"""


# 屏幕配置
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
FPS = 60


# 物理参数
BASE_SUBSTEPS = 10  # 基础物理子步数
MAX_SPEED = 800  # 最大速度（像素/秒）
FRICTION = 0.7  # 摩擦系数（车与箱子/炸弹之间）
TORQUE_FACTOR = 0.15  # 扭矩系数
WALL_TORQUE_FACTOR = 0.08  # 墙体碰撞扭矩系数


# 速度档位配置
SPEED_PRESETS = {
    "慢速": 150,
    "普通": 300,
    "快速": 450,
    "极速": 600
}
SPEED_KEYS = list(SPEED_PRESETS.keys())
CURRENT_SPEED_INDEX = 1  # 默认使用"普通"档位


# 空间哈希参数
GRID_SIZE = 80  # 空间网格大小
EDITOR_GRID_SIZE = 50  # 编辑模式网格大小


# 物体大小
OBJECT_SIZE = 40  # 车和箱子大小（像素）
WALL_SIZE = 50  # 墙体大小（等于编辑网格大小）


# 调试显示开关
SHOW_DEBUG_INFO = True  # 显示调试信息面板
SHOW_FORCE_CHAIN = False  # 显示力传递链条（默认关闭）
NO_RESISTANCE_MODE = True  # 无阻力模式：车不受箱子阻力影响


# 颜色配置
COLOR_BG = (30, 30, 40)
COLOR_CAR = (50, 150, 255)
COLOR_BOX = (139, 69, 19)
COLOR_WALL = (80, 80, 80)
COLOR_WHITE = (255, 255, 255)
COLOR_YELLOW = (255, 255, 0)
COLOR_RED = (255, 100, 100)
COLOR_NORMAL = (100, 200, 255)
COLOR_FORCE_CHAIN = (255, 200, 50)
COLOR_BOMB = (255, 100, 100)
COLOR_TARGET = (0, 255, 100)


# 炸弹配置
BOMB_EXPLOSION_DELAY = 1.0  # 爆炸延时（秒）
BOMB_IN_WALL_COVERAGE = 4  # 需要覆盖的角点数量


# 目标点配置
TARGET_DISTANCE_THRESHOLD = 0.4  # 距离阈值（相对于箱子大小的比例）

# 观察/识别配置
OBSERVE_DURATION = 0.3  # 观察动作耗时（秒），模拟OpenART拍照识别
OBSERVE_ANGLE_TOLERANCE_DEG = 5.0  # 朝向误差进入此范围后才允许识别
