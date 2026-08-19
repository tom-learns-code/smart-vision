# ============================================================
# OpenART Plus 视觉层 — 完整集成版
#
# 合并内容:
#   - 识别代码.py : 完整视觉检测逻辑（车/箱子/炸弹/墙壁/目标）
#   - untitled_1.py : 二进制 UART 协议（FULL_MAP/POS_UPDATE/MAP_REQUEST）
#
# 协议与 MCU vision_parser.c 逐字段对齐。
# ============================================================

import sensor
import image
import time
import math
from machine import UART


# ============================================================
# 0. 协议常量（与 MCU vision_parser.h 对齐）
# ============================================================
VP_HEADER_0       = 0xA5
VP_HEADER_1       = 0x5A
VP_TYPE_FULL_MAP   = 0x01
VP_TYPE_POS_UPDATE = 0x02
VP_TYPE_MAP_REQUEST = 0x03
VP_TYPE_HEARTBEAT  = 0x04

UART_BAUDRATE      = 115200
HEARTBEAT_PERIOD_MS = 500
UART_CANDIDATES    = (5, 12, 11)       # 逐个尝试，连接上的那个就是对的
MCU_MAX_BOXES       = 3
VISION_MAX_BOMBS    = 3
MCU_MAX_BOMBS       = 2


# ============================================================
# 1. 摄像头 / 地图常量
# ============================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)       # 320×240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 地图四角在图像中的像素坐标（手动标定）
LT = (11, 9)
RT = (310, 7)
RB = (310, 233)
LB = (3, 220)

# 边缘控制点（对应绿色网格交叉点）。默认值按四角直线插值生成，
# 手动校准时只改这些像素坐标即可拟合弯曲边缘。
TOP_C04 = (82, 6)       # top edge:    col=4,  row=0
TOP_C08 = (155, 5)      # top edge:    col=8,  row=0
TOP_C12 = (233, 5)      # top edge:    col=12, row=0
BOT_C04 = (75, 225)     # bottom edge: col=4,  row=12
BOT_C08 = (150, 230)    # bottom edge: col=8,  row=12
BOT_C12 = (231, 232)    # bottom edge: col=12, row=12
LEFT_R03 = (7, 60)      # left edge:   col=0,  row=3
LEFT_R06 = (4, 112)     # left edge:   col=0,  row=6
LEFT_R09 = (3, 167)     # left edge:   col=0,  row=9
RIGHT_R03 = (312, 61)   # right edge:  col=16, row=3
RIGHT_R06 = (313, 118)  # right edge:  col=16, row=6
RIGHT_R09 = (313, 176)  # right edge:  col=16, row=9

TOP_EDGE_POINTS = (LT, TOP_C04, TOP_C08, TOP_C12, RT)
BOTTOM_EDGE_POINTS = (LB, BOT_C04, BOT_C08, BOT_C12, RB)
LEFT_EDGE_POINTS = (LT, LEFT_R03, LEFT_R06, LEFT_R09, LB)
RIGHT_EDGE_POINTS = (RT, RIGHT_R03, RIGHT_R06, RIGHT_R09, RB)

# Geometry profile selector. The original outer calibration remains above.
GEOMETRY_PROFILE_OUTER = 0
GEOMETRY_PROFILE_INNER = 1
GEOMETRY_PROFILE = GEOMETRY_PROFILE_INNER

OUTER_LT, OUTER_RT, OUTER_RB, OUTER_LB = LT, RT, RB, LB
OUTER_TOP_EDGE_POINTS = TOP_EDGE_POINTS
OUTER_BOTTOM_EDGE_POINTS = BOTTOM_EDGE_POINTS
OUTER_LEFT_EDGE_POINTS = LEFT_EDGE_POINTS
OUTER_RIGHT_EDGE_POINTS = RIGHT_EDGE_POINTS

# New-phone floor/border-wall boundary measured from 41.bmp.
# This maps map col=1..15 and row=1..11, not the invisible outer outline.
INNER_LT = (21, 29)
INNER_RT = (288, 22)
INNER_RB = (281, 219)
INNER_LB = (21, 199)
INNER_TOP_EDGE_POINTS = (
    INNER_LT, (80, 25), (146, 23), (218, 21), INNER_RT)
INNER_BOTTOM_EDGE_POINTS = (
    INNER_LB, (78, 209), (140, 212), (216, 218), INNER_RB)
INNER_LEFT_EDGE_POINTS = (
    INNER_LT, (19, 69), (18, 118), (19, 164), INNER_LB)
INNER_RIGHT_EDGE_POINTS = (
    INNER_RT, (290, 71), (286, 122), (284, 174), INNER_RB)

if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER:
    LT, RT, RB, LB = INNER_LT, INNER_RT, INNER_RB, INNER_LB
    TOP_EDGE_POINTS = INNER_TOP_EDGE_POINTS
    BOTTOM_EDGE_POINTS = INNER_BOTTOM_EDGE_POINTS
    LEFT_EDGE_POINTS = INNER_LEFT_EDGE_POINTS
    RIGHT_EDGE_POINTS = INNER_RIGHT_EDGE_POINTS

COLS = 16
ROWS = 12
GRID_SIZE_MM = 200

if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER:
    GEOMETRY_U_MIN = 1.0 / COLS
    GEOMETRY_U_MAX = (COLS - 1.0) / COLS
    GEOMETRY_V_MIN = 1.0 / ROWS
    GEOMETRY_V_MAX = (ROWS - 1.0) / ROWS
else:
    GEOMETRY_U_MIN = 0.0
    GEOMETRY_U_MAX = 1.0
    GEOMETRY_V_MIN = 0.0
    GEOMETRY_V_MAX = 1.0

SCALE = 10          # 10 采样点/格 → 0.1 格分辨率
MAP_W = COLS * SCALE
MAP_H = ROWS * SCALE

MIN_BOX_PIXELS  = 8
MIN_BOMB_PIXELS = 6
MIN_CAR_PIXELS  = 4
MAX_CAR_MARKER_PIXELS = 220

# 车头-车尾配对距离筛选（格，每格 = 200mm）
CAR_MARKER_MIN_DIST = 0.12
CAR_MARKER_MAX_DIST = 1.20
CAR_MARKER_TARGET_DIST = 0.35
CAR_MARKER_SIZE_SCORE_WEIGHT = 0.008

MAX_CAR_HOLD_FRAMES    = 5
POS_UPDATE_PERIOD_MS   = 50   # POS_UPDATE 周期 (20 Hz)
CAR_POSE_SMOOTHING = True
CAR_POSE_SMOOTH_ALPHA = 0.45
CAR_THETA_SMOOTH_ALPHA = 0.18
CAR_THETA_MAX_STEP_DEG = 6.0
CAR_POSE_SMOOTH_MAX_DIST = 1.2

# ----- 调试开关 -----
PRINT_DEBUG        = False       # 旧详细调试输出，保留但默认停用
DRAW_DEBUG_OVERLAY = False       # 图像上画标记（会降帧率）
DRAW_GRID_OVERLAY  = False       # 图传关闭时不绘制网格
DRAW_GRID_POINTS    = False      # 图传关闭时不绘制格点
DRAW_CELL_CENTERS   = False      # 图传关闭时不绘制采样中心
PERIODIC_FULL_MAP_DEBUG = False  # 旧周期全图保留但默认停用
PERIODIC_FULL_MAP_PERIOD_MS = 2000
PERIODIC_FULL_MAP_SEND_UART = False
IDE_EVENT_DEBUG = True           # 首帧 + 事件驱动IDE输出
IDE_INITIAL_FULL_MAP = True
IDE_EVENT_FULL_MAP = True
IDE_GOAL_TRACE = True            # 仅目标事件状态变化时输出
IDE_FPS_DEBUG = True             # 每秒输出一次实际平均帧率
IDE_FPS_PERIOD_MS = 1000
IDE_PERF_DEBUG = True            # 与FPS同周期输出各阶段平均/最大耗时
STARTUP_BANNER = False
USE_PRECOMPUTED_SAMPLE_COORDS = True
USE_MULTI_CLASS_COMPONENTS = True
USE_REUSABLE_REGION_BUFFERS = True
USE_ADAPTIVE_FAST_RADIUS = True
USE_SEPARATE_OBJECT_ROIS = True
USE_PREDICTED_CAR_RECOVERY = True

PERF_SNAPSHOT = 0
PERF_REGION = 1
PERF_COMPONENTS = 2
PERF_FALLBACK = 3
PERF_TRACKING = 4
PERF_FULL_MAP = 5
PERF_LOOP = 6
PERF_STAGE_COUNT = 7
STATIC_MAP_LOCK = True             # 首次高质量建图后锁定墙/目标/地面
STATIC_MAP_BUILD_SAMPLES = 5       # 每格 NxN 采样，仅首次建图使用
STATIC_INNER_BORDER_SAMPLE_INSET = 0.25
STATIC_MAP_VERIFY_FRAMES = 2
STATIC_MAP_VERIFY_FRAME_DELAY_MS = 10
STATIC_WALL_VERIFY_VOTE_LOW = 4
STATIC_WALL_VERIFY_VOTE_HIGH = 12
STATIC_WALL_SAMPLE_SCORE_MIN = 78
STATIC_WALL_STRONG_SAMPLE_SCORE_MIN = 88
STATIC_WALL_VOTE_MIN = 8
STATIC_WALL_STRONG_VOTE_MIN = 2
STATIC_WALL_HIGH_VOTE_MIN = 12
STATIC_WALL_NONWALL_VOTE_MAX = 2
STATIC_WALL_NONWALL_MARGIN = 4
STATIC_WALL_WEAK_VOTE_MIN = 4
STATIC_WALL_NEIGHBOR_VOTE_MIN = 8
STATIC_WALL_NEIGHBOR_MIN = 2
STATIC_WALL_POST_APPLY = False
STATIC_WALL_POST_DIAGNOSTIC = True
STATIC_WALL_POST_DEBUG_MAX = 8
STATIC_GOAL_VOTE_MIN = 5
STATIC_GOAL_STRONG_VOTE_MIN = 2
STATIC_GOAL_WALL_VOTE_MAX = 6
STATIC_FORCE_BORDER_WALLS = True
STATIC_CELL_DEBUG = False
STATIC_DEBUG_CELLS = ((2, 2), (9, 4), (10, 4), (9, 5), (10, 5), (9, 6), (10, 6), (11, 6), (13, 4))
CAR_CANDIDATE_SCORE_DEBUG = False
CAR_PAIR_DEBUG_MAX = 4
CAR_NEAR_BACK_RESCUE = True
CAR_RESCUE_CYAN_SCORE_MIN = 58
CAR_RESCUE_SCORE_MARGIN = 6
CAR_RESCUE_SAMPLE_STEP = 2

# ----- 近距物体跟踪 -----
NEAR_OBJECT_TRACKING           = True
NEAR_OBJECT_RADIUS_CELLS       = 3.0
NEAR_OBJECT_PREPARE_RADIUS_CELLS = 4.0
CAR_TRACK_RADIUS_CELLS         = 2.5
OBJECT_TRACK_RADIUS_CELLS      = 2.2
OBJECT_ROI_MAX_TRACKS          = 2
OBJECT_ROI_SECONDARY_PERIOD    = 2
OBJECT_ROI_WAKE_MARGIN_CELLS   = 1.0
CAR_PREDICTION_TIME_GAIN       = 1.15
CAR_PREDICTION_MIN_SPEED_PER_MS = 0.0008
CAR_PREDICTION_MAX_ELAPSED_MS  = 1500
CAR_FULL_FALLBACK_RETRY_MS     = 400
ENABLE_DYNAMIC_WALL_SUPPRESS = False
CAR_DYNAMIC_COVER_RADIUS_CELLS = 0.95
COMP_DYNAMIC_COVER_PAD_SAMPLES = 2
DYNAMIC_WALL_SUPPRESS_SCORE_MAX = 84

# ----- Layered world-state tracking -----
# Keep the current UART byte layout while the new state model is verified.
# The protocol adapter below exports these tracks through the legacy packets.
LAYERED_STATE_ENABLED = True
OBJECT_ENTER_ACTIVE_RADIUS = 1.5
OBJECT_EXIT_ACTIVE_RADIUS = 1.8
OBJECT_ACTIVATE_FRAMES = 2
OBJECT_FREEZE_FRAMES = 3
OBJECT_ASSOCIATE_MAX_DIST = 1.25
OBJECT_OCCLUDED_ASSOCIATE_MAX_DIST = 1.80
OBJECT_OCCLUDED_AFTER_MISSES = 2
OBJECT_LOST_AFTER_MISSES = 12
OBJECT_HISTORY_COUNT = 5
OBJECT_POSITION_ALPHA = 0.68
OBJECT_MOVE_MIN = 0.035
OBJECT_AXIS_LOCK_MIN_DELTA = 0.10
OBJECT_AXIS_LOCK_RATIO = 2.0
OBJECT_AXIS_LOCK_FRAMES = 2
OBJECT_AXIS_SWITCH_FRAMES = 2
OBJECT_AXIS_RELEASE_STILL_FRAMES = 4
OBJECT_SETTLED_SPEED = 0.025

BOX_GOAL_ENTRY_RADIUS = 0.42
BOX_GOAL_FORWARD_LOOKAHEAD = 3.20
BOX_GOAL_LATERAL_TOLERANCE = 0.38
BOX_GOAL_HISTORY_MIN_DELTA = 0.06
BOX_GOAL_CANDIDATE_KEEP_RADIUS = 3.50
BOX_GOAL_MISSING_FRAMES = 4
BOX_GOAL_VERIFY_FRAMES = 2
BOX_GOAL_PENDING_MAX_FRAMES = 24
BOX_GOAL_CAR_CLEAR_RADIUS = 0.45
GLOBAL_GOAL_VERIFY_FRAMES = 2
GLOBAL_GOAL_ASSIGN_MAX_DIST = 3.0
GLOBAL_GOAL_ASSIGN_FORWARD_LOOKAHEAD = 4.0
GLOBAL_GOAL_ASSIGN_LATERAL_TOLERANCE = 1.25
GLOBAL_GOAL_EXPLICIT_ASSIGN_RADIUS = 1.35
GLOBAL_GOAL_CONFIRMATION_ONLY = True

STATIC_AUDIT_ENABLED = True
STATIC_AUDIT_CANDIDATE_FRAMES = 3

EXPLOSION_EVENTS_ENABLED = True
EXPLOSION_MISSING_FRAMES = 2
EXPLOSION_VERIFY_FRAMES = 2
EXPLOSION_PENDING_MAX_FRAMES = 20
EXPLOSION_INNER_FLOOR_RATIO_Q8 = 154   # 0.60 * 256
EXPLOSION_OUTER_MATCH_RATIO_Q8 = 192   # 0.75 * 256

OBJ_TYPE_BOX = 1
OBJ_TYPE_BOMB = 2

OBJ_STATE_FROZEN = 0
OBJ_STATE_ACTIVE = 1
OBJ_STATE_PUSH_CANDIDATE = 2
OBJ_STATE_PUSHING_X = 3
OBJ_STATE_PUSHING_Y = 4
OBJ_STATE_OCCLUDED = 5
OBJ_STATE_SETTLING = 6
OBJ_STATE_BLOCKED = 7
OBJ_STATE_EXPLOSION_PENDING = 8
OBJ_STATE_REMOVED = 9

OBJ_OUTPUT_VALID = 0
OBJ_OUTPUT_OCCLUDED = 1
OBJ_OUTPUT_COMPLETED = 2
OBJ_OUTPUT_LOST = 3

AXIS_NONE = 0
AXIS_X = 1
AXIS_Y = 2

# DynamicObject is represented by a fixed list to keep MicroPython allocation
# predictable. Use these indexes instead of dictionaries/classes.
O_ID = 0
O_TYPE = 1
O_X = 2
O_Y = 3
O_STATE = 4
O_OUTPUT = 5
O_CONFIDENCE = 6
O_VISIBLE_Q8 = 7
O_AXIS = 8
O_DIRECTION = 9
O_VX = 10
O_VY = 11
O_LAST_SEEN = 12
O_MISSES = 13
O_ENTER_COUNT = 14
O_EXIT_COUNT = 15
O_AXIS_COUNT = 16
O_STILL_COUNT = 17
O_ANCHOR_X = 18
O_ANCHOR_Y = 19
O_HISTORY = 20
O_PIXELS = 21
O_GOAL_X = 22
O_GOAL_Y = 23
O_GOAL_FLOOR_COUNT = 24
O_GOAL_VISIBLE_COUNT = 25
O_GOAL_PENDING_AGE = 26
O_GOAL_RESULT = 27
O_GOAL_TRACE_STATE = 28
O_GOAL_LAST_EVIDENCE = 29
O_LAST_MOVE_AXIS = 30
O_LAST_MOVE_DIRECTION = 31

# CarState list indexes.
C_X = 0
C_Y = 1
C_THETA = 2
C_VX = 3
C_VY = 4
C_CONFIDENCE = 5
C_LAST_SEEN = 6

# ----- Sample-space correction -----
# Geometry correction is wired into the sampler, but the coefficient stays
# zero until the grid overlay is calibrated from a real camera frame.
SAMPLE_GEOMETRY_CORRECTION = True
SAMPLE_BARREL_K1 = 0.0
SAMPLE_BARREL_K2 = 0.0
SAMPLE_CENTER_U = 0.5
SAMPLE_CENTER_V = 0.5

# Fixed vignetting compensation, Q6 format: 64 = 1.00x.
# This is intentionally applied only to sampled pixels, not the whole image.
CALIBRATION_PROFILE_OLD = 0
CALIBRATION_PROFILE_NEW = 1
CALIBRATION_PROFILE = CALIBRATION_PROFILE_NEW

SAMPLE_BRIGHTNESS_CORRECTION = True
BRIGHTNESS_GAIN_BASE_Q6 = 64
BRIGHTNESS_GAIN_GRID_SIZE = 5
OLD_BRIGHTNESS_GAIN_GRID_Q6 = (
    (96, 84, 78, 84, 90),
    (88, 74, 68, 73, 82),
    (84, 70, 64, 70, 78),
    (90, 76, 70, 74, 84),
    (100, 88, 80, 86, 94),
)
NEW_BRIGHTNESS_GAIN_GRID_Q6 = (
    (99, 84, 66, 67, 89),
    (99, 69, 61, 61, 77),
    (99, 68, 61, 62, 76),
    (99, 70, 62, 64, 84),
    (99, 87, 74, 81, 99),
)

if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW:
    BRIGHTNESS_GAIN_GRID_Q6 = NEW_BRIGHTNESS_GAIN_GRID_Q6
else:
    BRIGHTNESS_GAIN_GRID_Q6 = OLD_BRIGHTNESS_GAIN_GRID_Q6

# ----- 分类常量 -----
CLS_NONE      = 0
CLS_BOX       = 1
CLS_BOMB_RED  = 2
CLS_BOMB_DARK = 3
CLS_CAR_FRONT = 4
CLS_CAR_BACK  = 5


# ============================================================
# 2. UART 初始化（CMM 配置 + 逐个尝试候选 UART）
# ============================================================

CMM_LINES = (
    "hw,-,-,rt117x,seekfree_art_plus,",
    "uart,5,TXD,-,AD_28,",
    "uart,5,RXD,-,AD_29,",
    "uart,11,TXD,-,LPSR_04,",
    "uart,11,RXD,-,LPSR_05,",
    "uart,12,TXD,-,LPSR_06,",
    "uart,12,RXD,-,LPSR_07,",
)

CMM_CFG = """fn,unit,signal,hint,pinobj,comments
hw,-,-,rt117x,seekfree_art_plus,
led,1,-,-,DISP_B2_03,
led,2,-,-,DISP_B1_07,
led,3,-,-,DISP_B1_05,
led,4,-,-,DISP_B2_08,
pin,-,J25,-,AD_26,
pin,-,J26,-,AD_27,
pin,-,J27,-,AD_28,
pin,-,J28,-,AD_29,
pin,-,M12,-,LPSR_12,
pin,-,M11,-,LPSR_11,
pin,-,M10,-,LPSR_10,
pin,-,M9,-,LPSR_09,
pin,-,M4,-,LPSR_04,
pin,-,M5,-,LPSR_05,
pin,-,J6,-,AD_07,
adc,-,A0,1,AD_26,
adc,-,A1,1,AD_27,
adc,-,A2,1,AD_28,
adc,-,A3,1,AD_29,
pwm,-,CH1,f2.a1,AD_26,
pwm,-,CH2,f2.b1,AD_27,
pwm,-,CH3,f2.a2,AD_28,
pwm,-,CH4,f2.b2,AD_29,
uart,5,TXD,-,AD_28,
uart,5,RXD,-,AD_29,
uart,11,TXD,-,LPSR_04,
uart,11,RXD,-,LPSR_05,
uart,12,TXD,-,LPSR_06,
uart,12,RXD,-,LPSR_07,
i2c,5,SDA,-,LPSR_04,
i2c,5,SCL,-,LPSR_05,
i2c,6,SDA,-,LPSR_06,
i2c,6,SCL,-,LPSR_07,
spi,6,SCK,-,LPSR_10,
spi,6,PCS0,-,LPSR_09,
spi,6,SDO,-,LPSR_11,
spi,6,SDI,-,LPSR_12,
"""


def load_cmm_config():
    try:
        from machine import Pin
        import cmm
        cmap = {}
        for sLine in CMM_LINES:
            lst = sLine.split(",")
            if lst[1] == "-":
                comboName = lst[0] + "." + lst[2]
            else:
                comboName = lst[0] + "." + lst[1] + "." + lst[2]
            try:
                pin = Pin(lst[4])
            except Exception:
                pin = None
            cmap[comboName] = (lst[3], lst[4], pin, None)
        cmm.add(cmap)
        if STARTUP_BANNER:
            print("cmm.add OK")
        return
    except Exception as e:
        print("direct cmm.add failed:", e)
    try:
        import cmm_load
        cmm_load.load()
        if STARTUP_BANNER:
            print("cmm_load OK")
        return
    except Exception as e:
        print("import cmm_load failed:", e)
    try:
        for path in ("/cmm_load.py", "cmm_load.py", "/flash/cmm_load.py"):
            try:
                exec(open(path).read(), globals())
                loader = globals().get("load", None)
                if loader is None:
                    raise Exception("load missing")
                loader()
                if STARTUP_BANNER:
                    print(path, "OK")
                return
            except Exception as ee:
                print(path, "failed:", ee)
    except Exception as e:
        print("cmm load failed:", e)


# 写 CMM 配置文件
for path in ("/cmm_cfg.csv", "cmm_cfg.csv", "/flash/cmm_cfg.csv"):
    try:
        with open(path, "w") as f:
            f.write(CMM_CFG)
        if STARTUP_BANNER:
            print("WROTE", path)
    except Exception as e:
        print("SKIP", path, e)

load_cmm_config()


# 尝试打开 UART
g_uart = None
g_uart_ports = []

for uart_id in UART_CANDIDATES:
    try:
        u = UART(uart_id, baudrate=UART_BAUDRATE)
        g_uart_ports.append([uart_id, u, bytearray()])
        if STARTUP_BANNER:
            print("UART%d OK -> added for MCU broadcast" % uart_id)
    except Exception as e:
        print("UART%d NO: %s" % (uart_id, e))

if not g_uart_ports:
    raise Exception("No UART opened - check wiring")

g_uart = g_uart_ports[0][1]


def uart_write_all(pkt):
    sent = 0
    for port in g_uart_ports:
        try:
            port[1].write(pkt)
            sent += 1
        except Exception as e:
            if PRINT_DEBUG:
                print("UART%d write failed: %s" % (port[0], e))
    return sent


# ============================================================
# 3. 二进制协议打包工具
# ============================================================

def xor_checksum(data):
    cs = 0
    for b in data:
        cs ^= b
    return cs & 0xFF


def perf_record(stage, elapsed_ms):
    if not IDE_PERF_DEBUG:
        return
    if elapsed_ms < 0:
        elapsed_ms = 0
    g_perf_sum_ms[stage] += elapsed_ms
    g_perf_count[stage] += 1
    if elapsed_ms > g_perf_max_ms[stage]:
        g_perf_max_ms[stage] = elapsed_ms


def perf_avg_x10(stage):
    count = g_perf_count[stage]
    if count <= 0:
        return 0
    return (g_perf_sum_ms[stage] * 10 + count // 2) // count


def perf_reset_window():
    global g_perf_fallback_count, g_perf_prediction_success_count
    global g_perf_full_fallback_count, g_perf_full_fallback_success_count
    global g_perf_full_fallback_skip_count
    for i in range(PERF_STAGE_COUNT):
        g_perf_sum_ms[i] = 0
        g_perf_max_ms[i] = 0
        g_perf_count[i] = 0
    g_perf_fallback_count = 0
    g_perf_prediction_success_count = 0
    g_perf_full_fallback_count = 0
    g_perf_full_fallback_success_count = 0
    g_perf_full_fallback_skip_count = 0


def perf_print_window():
    values = []
    for stage in range(PERF_STAGE_COUNT):
        avg_x10 = perf_avg_x10(stage)
        values.append((avg_x10 // 10, avg_x10 % 10,
                       g_perf_max_ms[stage], g_perf_count[stage]))
    print("PERF avg_ms snapshot=%d.%d region=%d.%d components=%d.%d fallback=%d.%d tracking=%d.%d fullmap=%d.%d loop=%d.%d" %
          (values[PERF_SNAPSHOT][0], values[PERF_SNAPSHOT][1],
           values[PERF_REGION][0], values[PERF_REGION][1],
           values[PERF_COMPONENTS][0], values[PERF_COMPONENTS][1],
           values[PERF_FALLBACK][0], values[PERF_FALLBACK][1],
           values[PERF_TRACKING][0], values[PERF_TRACKING][1],
           values[PERF_FULL_MAP][0], values[PERF_FULL_MAP][1],
           values[PERF_LOOP][0], values[PERF_LOOP][1]))
    print("PERF max_ms snapshot=%d region=%d components=%d fallback=%d tracking=%d fullmap=%d loop=%d prediction_count=%d prediction_success=%d full_fallback_count=%d full_fallback_success=%d full_fallback_skip=%d fullmap_count=%d" %
          (values[PERF_SNAPSHOT][2], values[PERF_REGION][2],
           values[PERF_COMPONENTS][2], values[PERF_FALLBACK][2],
           values[PERF_TRACKING][2], values[PERF_FULL_MAP][2],
           values[PERF_LOOP][2], g_perf_fallback_count,
           g_perf_prediction_success_count,
           g_perf_full_fallback_count,
           g_perf_full_fallback_success_count,
           g_perf_full_fallback_skip_count,
           values[PERF_FULL_MAP][3]))


def append_i16(buf, value):
    value &= 0xFFFF
    buf.append(value & 0xFF)
    buf.append((value >> 8) & 0xFF)


def append_i8(buf, value):
    buf.append(value & 0xFF)


def make_full_map_binary(grid, goals, boxes, bombs, car):
    """生成 FULL_MAP 二进制包（59 字节，与 MCU vision_parser 对齐）"""
    payload = bytearray()
    payload.append(COLS)     # map_width
    payload.append(ROWS)     # map_height

    # 墙体位图: 192 bits → 24 bytes, LSB 优先
    walls = bytearray(24)
    for row in range(ROWS):
        for col in range(COLS):
            if grid_get(grid, col, row) == "#":
                idx = row * COLS + col
                walls[idx >> 3] |= 1 << (idx & 7)
    payload.extend(walls)

    # 箱子: 最多 4 个, int8 格坐标, 空位填 -1
    box_count = len(boxes) if len(boxes) <= MCU_MAX_BOXES else MCU_MAX_BOXES
    payload.append(box_count)
    for i in range(4):
        if i < box_count:
            append_i8(payload, clamp_cell_x(boxes[i][0]))
            append_i8(payload, clamp_cell_y(boxes[i][1]))
        else:
            append_i8(payload, -1)
            append_i8(payload, -1)

    # 目标: 最多 4 个, int8 格坐标
    goal_count = len(goals) if len(goals) <= MCU_MAX_BOXES else MCU_MAX_BOXES
    payload.append(goal_count)
    for i in range(4):
        if i < goal_count:
            append_i8(payload, goals[i][0])
            append_i8(payload, goals[i][1])
        else:
            append_i8(payload, -1)
            append_i8(payload, -1)

    # The current MCU packet layout reserves exactly two bomb slots.
    bomb_count = len(bombs) if len(bombs) <= MCU_MAX_BOMBS else MCU_MAX_BOMBS
    payload.append(bomb_count)
    for i in range(MCU_MAX_BOMBS):
        if i < bomb_count:
            append_i8(payload, clamp_cell_x(bombs[i][0]))
            append_i8(payload, clamp_cell_y(bombs[i][1]))
        else:
            append_i8(payload, -1)
            append_i8(payload, -1)

    # 车位姿 (mm, deg×10 → int16 LE)
    if car is not None:
        append_i16(payload, grid_to_mm(car[0]))
        append_i16(payload, grid_to_mm(car[1]))
        append_i16(payload, deg_to_x10(car[2]))
    else:
        append_i16(payload, 0)
        append_i16(payload, 0)
        append_i16(payload, 0)

    payload.append(xor_checksum(payload))
    return bytearray([VP_HEADER_0, VP_HEADER_1, VP_TYPE_FULL_MAP]) + payload


def make_pos_update_binary(frame_id, car, near_boxes, near_bombs):
    """生成 POS_UPDATE 二进制包（与 MCU vision_parser 对齐）

    MCU parse_byte() 固定读 VP_POS_UPDATE_LEN-3 = 19 字节 payload。
    必须精确 19 字节，否则后续帧解析错位。
    """
    payload = bytearray()
    payload.append(frame_id & 0xFF)

    if car is not None:
        append_i16(payload, grid_to_mm(car[0]))
        append_i16(payload, grid_to_mm(car[1]))
        append_i16(payload, deg_to_x10(car[2]))
    else:
        append_i16(payload, 0)
        append_i16(payload, 0)
        append_i16(payload, 0)

    # 箱子: 最多 3 个，留出空间给 bomb_count + padding
    box_count = len(near_boxes) if len(near_boxes) <= 3 else 3
    payload.append(box_count)
    for i in range(box_count):
        append_i8(payload, clamp_cell_x(near_boxes[i][0]))
        append_i8(payload, clamp_cell_y(near_boxes[i][1]))

    # 炸弹: 填 0（如需传炸弹可改为 1，但需对应减少箱子数量）
    payload.append(0)

    # 填充到 18 字节（checksum 在第 19 字节即索引 18）
    while len(payload) < 18:
        payload.append(0)

    payload.append(xor_checksum(payload))
    return bytearray([VP_HEADER_0, VP_HEADER_1, VP_TYPE_POS_UPDATE]) + payload


def make_heartbeat_binary(seq):
    payload = bytearray()
    payload.append(seq & 0xFF)
    payload.append(xor_checksum(payload))
    return bytearray([VP_HEADER_0, VP_HEADER_1, VP_TYPE_HEARTBEAT]) + payload


# ============================================================
# 4. MAP_REQUEST 监听
# ============================================================

g_full_map_requested = False


def poll_request():
    """读取 UART，检测 0x03 请求包 → 标记需要回复 FULL_MAP"""
    global g_uart_ports, g_full_map_requested

    for port in g_uart_ports:
        uart_id = port[0]
        uart = port[1]
        rx_buf = port[2]

        if uart.any():
            data = uart.read()
            if data:
                for b in data:
                    rx_buf.append(b)

        while len(rx_buf) >= 5:
            if rx_buf[0] != VP_HEADER_0 or rx_buf[1] != VP_HEADER_1:
                rx_buf = rx_buf[1:]
                continue
            pkt = rx_buf[:5]
            if pkt[2] == VP_TYPE_MAP_REQUEST and xor_checksum(pkt[:4]) == pkt[4]:
                g_full_map_requested = True
                if PRINT_DEBUG:
                    print("RCVD MAP_REQUEST uart=%d reason=%d" % (uart_id, pkt[3]))
                rx_buf = rx_buf[5:]
            else:
                rx_buf = rx_buf[1:]

        if len(rx_buf) > 32:
            rx_buf = rx_buf[-16:]
        port[2] = rx_buf


# ============================================================
# 5. 坐标 / 颜色 / 工具函数（来自 识别代码.py，不变）
# ============================================================

def lerp(a, b, t):
    return a + (b - a) * t


def lerp_point(a, b, t):
    return lerp(a[0], b[0], t), lerp(a[1], b[1], t)


def edge_point(points, t):
    if t <= 0.0:
        return points[0]
    if t >= 1.0:
        return points[4]
    s = t * 4.0
    i = int(s)
    if i >= 4:
        return points[4]
    return lerp_point(points[i], points[i + 1], s - i)


def coons_point_from_uv(u, v):
    top = edge_point(TOP_EDGE_POINTS, u)
    bottom = edge_point(BOTTOM_EDGE_POINTS, u)
    left = edge_point(LEFT_EDGE_POINTS, v)
    right = edge_point(RIGHT_EDGE_POINTS, v)

    tb = lerp_point(top, bottom, v)
    lr = lerp_point(left, right, u)
    corner_top = lerp_point(LT, RT, u)
    corner_bottom = lerp_point(LB, RB, u)
    corner = lerp_point(corner_top, corner_bottom, v)
    return tb[0] + lr[0] - corner[0], tb[1] + lr[1] - corner[1]


def correct_sample_uv(u, v):
    if not SAMPLE_GEOMETRY_CORRECTION:
        return u, v

    dx = u - SAMPLE_CENTER_U
    dy = v - SAMPLE_CENTER_V
    edge_u = 1.0 - 4.0 * dy * dy
    edge_v = 1.0 - 4.0 * dx * dx
    if edge_u < 0.0:
        edge_u = 0.0
    if edge_v < 0.0:
        edge_v = 0.0
    u = u + dx * (SAMPLE_BARREL_K1 * edge_u + SAMPLE_BARREL_K2 * edge_u * edge_u)
    v = v + dy * (SAMPLE_BARREL_K1 * edge_v + SAMPLE_BARREL_K2 * edge_v * edge_v)
    return u, v


def geometry_local_uv(u, v):
    u = (u - GEOMETRY_U_MIN) / (GEOMETRY_U_MAX - GEOMETRY_U_MIN)
    v = (v - GEOMETRY_V_MIN) / (GEOMETRY_V_MAX - GEOMETRY_V_MIN)
    if u < 0.0:
        u = 0.0
    elif u > 1.0:
        u = 1.0
    if v < 0.0:
        v = 0.0
    elif v > 1.0:
        v = 1.0
    return u, v


def geometry_uv_visible(u, v):
    return (GEOMETRY_U_MIN <= u <= GEOMETRY_U_MAX and
            GEOMETRY_V_MIN <= v <= GEOMETRY_V_MAX)


def screen_point_from_uv(u, v):
    u, v = correct_sample_uv(u, v)
    u, v = geometry_local_uv(u, v)
    x, y = coons_point_from_uv(u, v)
    if x < 0:
        x = 0
    elif x > 319:
        x = 319
    if y < 0:
        y = 0
    elif y > 239:
        y = 239
    return int(x), int(y)


def screen_point_from_map(mx, my):
    return screen_point_from_uv(mx / COLS, my / ROWS)


def clamp_cell_x(x):
    if x < 0:        return 0
    if x >= COLS:    return COLS - 1
    return int(x)


def clamp_cell_y(y):
    if y < 0:        return 0
    if y >= ROWS:    return ROWS - 1
    return int(y)


def grid_to_mm(g):
    return int(g * GRID_SIZE_MM + 0.5)


def deg_to_x10(deg):
    return int(deg * 10 + 0.5)


def grid_to_x10(g):
    return int(g * 10 + 0.5)


def clamp_u8(value):
    if value < 0:
        return 0
    if value > 255:
        return 255
    return int(value)


def apply_brightness_gain_q6(r, g, b, gain_q6):
    if not SAMPLE_BRIGHTNESS_CORRECTION:
        return r, g, b
    r = (r * gain_q6 + 32) >> 6
    g = (g * gain_q6 + 32) >> 6
    b = (b * gain_q6 + 32) >> 6
    return clamp_u8(r), clamp_u8(g), clamp_u8(b)


def brightness_gain_q6_at_uv(u, v):
    if not SAMPLE_BRIGHTNESS_CORRECTION:
        return BRIGHTNESS_GAIN_BASE_Q6

    max_idx = BRIGHTNESS_GAIN_GRID_SIZE - 1
    gx = u * max_idx
    gy = v * max_idx
    ix = int(gx)
    iy = int(gy)
    if ix < 0:
        ix = 0
    if iy < 0:
        iy = 0
    if ix >= max_idx:
        ix = max_idx - 1
        fx = 1.0
    else:
        fx = gx - ix
    if iy >= max_idx:
        iy = max_idx - 1
        fy = 1.0
    else:
        fy = gy - iy

    g00 = BRIGHTNESS_GAIN_GRID_Q6[iy][ix]
    g10 = BRIGHTNESS_GAIN_GRID_Q6[iy][ix + 1]
    g01 = BRIGHTNESS_GAIN_GRID_Q6[iy + 1][ix]
    g11 = BRIGHTNESS_GAIN_GRID_Q6[iy + 1][ix + 1]
    top = g00 + (g10 - g00) * fx
    bottom = g01 + (g11 - g01) * fx
    return int(top + (bottom - top) * fy + 0.5)


def build_brightness_gain_map_q6():
    out = bytearray(MAP_W * MAP_H)
    for sy in range(MAP_H):
        v = (sy + 0.5) / MAP_H
        for sx in range(MAP_W):
            u = (sx + 0.5) / MAP_W
            out[sy * MAP_W + sx] = brightness_gain_q6_at_uv(u, v)
    return out


def build_sample_coordinate_maps():
    """Precompute the exact dynamic-grid sample coordinates.

    X needs 9 bits at QVGA width, so it is stored as low/high byte arrays.
    Y and visibility each fit in one byte. This keeps the lookup portable on
    MicroPython without relying on a specific array module implementation.
    """
    count = MAP_W * MAP_H
    x_lo = bytearray(count)
    x_hi = bytearray(count)
    y_map = bytearray(count)
    visible_map = bytearray(count)
    for sy in range(MAP_H):
        v = (sy + 0.5) / MAP_H
        row_base = sy * MAP_W
        for sx in range(MAP_W):
            u = (sx + 0.5) / MAP_W
            idx = row_base + sx
            if not geometry_uv_visible(u, v):
                continue
            x, y = screen_point_from_uv(u, v)
            x_lo[idx] = x & 0xFF
            x_hi[idx] = (x >> 8) & 0xFF
            y_map[idx] = y
            visible_map[idx] = 1
    return x_lo, x_hi, y_map, visible_map


# ----- 颜色判定 -----

YELLOW_L_MIN, YELLOW_L_MAX = 50, 100
YELLOW_A_MIN, YELLOW_A_MAX = -30, 20
YELLOW_B_MIN, YELLOW_B_MAX = 11, 127

MAGENTA_L_MIN, MAGENTA_L_MAX = 0, 100
MAGENTA_A_MIN, MAGENTA_A_MAX = 92, 127
MAGENTA_B_MIN, MAGENTA_B_MAX = -128, 127

RED_L_MIN, RED_L_MAX = 40, 70
RED_A_MIN, RED_A_MAX = 60, 100
RED_B_MIN, RED_B_MAX = -50, 127


def looks_magenta_goal_rgb(r, g, b):
    # A goal keeps both red and blue strong. Red bombs have a much lower
    # blue/red ratio, while blue floor has too little red.
    if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW:
        return (r >= 105 and b >= 80 and g <= 50 and
                r >= g + 70 and b >= g + 55 and
                b * 100 >= r * 45 and
                r * 100 >= b * 75)
    return (r >= 85 and b >= 100 and
            r >= g + 45 and b >= g + 65 and
            b * 100 >= r * 75 and
            r * 100 >= b * 55)


def is_strong_magenta_goal_rgb(r, g, b):
    if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW:
        return (r >= 135 and b >= 100 and g <= 35 and
                r >= g + 90 and b >= g + 70 and
                b * 100 >= r * 50 and
                r * 100 >= b * 75)
    return (r >= 105 and b >= 125 and g <= 65 and
            b * 100 >= r * 82 and
            r * 100 >= b * 62)


def is_yellow(r, g, b):
    # RGB fallback keeps yellow boxes detectable after screen brightness is
    # reduced. Requiring both red and green rejects red bombs.
    if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW:
        return (r >= 85 and g >= 65 and b <= 60 and
                r >= g + 5 and r <= g + 100 and
                r >= b + 55 and g >= b + 50)
    if (r >= 70 and g >= 55 and
            r >= b + 45 and g >= b + 35 and
            r <= g + 85):
        return True
    l, a_val, b_val = image.rgb_to_lab((r, g, b))
    return (YELLOW_L_MIN <= l <= YELLOW_L_MAX and
            YELLOW_A_MIN <= a_val <= YELLOW_A_MAX and
            YELLOW_B_MIN <= b_val <= YELLOW_B_MAX)


def is_red_or_orange(r, g, b):
    if looks_magenta_goal_rgb(r, g, b):
        return False
    if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW:
        return (r >= 85 and r >= g + 50 and r >= b + 50 and
                g * 100 <= r * 45 and
                b * 100 <= r * 45)
    # Red bombs remain red-dominant even when their absolute brightness
    # changes. This also covers the darker red bomb tiles.
    if r >= 75 and r >= g + 40 and r >= b + 45:
        return True
    l, a_val, b_val = image.rgb_to_lab((r, g, b))
    return (RED_L_MIN <= l <= RED_L_MAX and
            RED_A_MIN <= a_val <= RED_A_MAX and
            RED_B_MIN <= b_val <= RED_B_MAX)


def is_dark(r, g, b):
    return (r + g + b) < 70


# ----- 青色标记（车头）LAB 阈值（双阈值 OR，覆盖正常光照 + 图像边缘发黑）-----
CYAN1_L_MIN, CYAN1_L_MAX = 48, 83
CYAN1_A_MIN, CYAN1_A_MAX = -128, -7
CYAN1_B_MIN, CYAN1_B_MAX = -54, -1

CYAN2_L_MIN, CYAN2_L_MAX = 75, 100
CYAN2_A_MIN, CYAN2_A_MAX = -70, -25
CYAN2_B_MIN, CYAN2_B_MAX = -40, 39

# ----- 绿色标记（车尾）LAB 阈值（双阈值 OR，覆盖正常光照 + 图像边缘发黑）-----
GREEN1_L_MIN, GREEN1_L_MAX = 43, 73
GREEN1_A_MIN, GREEN1_A_MAX = -86, -16
GREEN1_B_MIN, GREEN1_B_MAX = -5, 70

GREEN2_L_MIN, GREEN2_L_MAX = 77, 100
GREEN2_A_MIN, GREEN2_A_MAX = -89, -33
GREEN2_B_MIN, GREEN2_B_MAX = 19, 115

CAR_MARKER_SCORE_MIN = 58
CAR_MARKER_SCORE_MARGIN = 6


def range_score(value, low, high, soft):
    if low <= value <= high:
        return 40
    if value < low:
        d = low - value
    else:
        d = value - high
    if d >= soft:
        return 0
    return 40 - int(d * 40 / soft)


def lab_cyan_score(l, a_val, b_val):
    s1 = (range_score(l, CYAN1_L_MIN, CYAN1_L_MAX, 18) +
          range_score(a_val, CYAN1_A_MIN, CYAN1_A_MAX, 28) +
          range_score(b_val, CYAN1_B_MIN, CYAN1_B_MAX, 28))
    s2 = (range_score(l, CYAN2_L_MIN, CYAN2_L_MAX, 18) +
          range_score(a_val, CYAN2_A_MIN, CYAN2_A_MAX, 28) +
          range_score(b_val, CYAN2_B_MIN, CYAN2_B_MAX, 28))
    return s1 if s1 > s2 else s2


def lab_green_score(l, a_val, b_val):
    s1 = (range_score(l, GREEN1_L_MIN, GREEN1_L_MAX, 18) +
          range_score(a_val, GREEN1_A_MIN, GREEN1_A_MAX, 28) +
          range_score(b_val, GREEN1_B_MIN, GREEN1_B_MAX, 28))
    s2 = (range_score(l, GREEN2_L_MIN, GREEN2_L_MAX, 18) +
          range_score(a_val, GREEN2_A_MIN, GREEN2_A_MAX, 28) +
          range_score(b_val, GREEN2_B_MIN, GREEN2_B_MAX, 28))
    return s1 if s1 > s2 else s2


def car_marker_scores(r, g, b):
    if looks_magenta_goal_rgb(r, g, b):
        return 0, 0

    total = r + g + b
    maxc = r
    if g > maxc: maxc = g
    if b > maxc: maxc = b
    minc = r
    if g < minc: minc = g
    if b < minc: minc = b
    if total < 55 or maxc - minc < 18:
        return 0, 0
    if b > g + 65 and b > r + 65:
        return 0, 0

    # These are hard acceptance conditions later in the original classifier.
    # Hoisting them before LAB is exactly equivalent: when both fail, both
    # scores would be forced to zero after all expensive calculations.
    cyan_rgb_ok = (g >= r + 18 and b >= r + 25 and b >= g - 20)
    green_rgb_ok = (g >= r + 25 and g >= b + 18)
    if not cyan_rgb_ok and not green_rgb_ok:
        return 0, 0

    l, a_val, b_val = image.rgb_to_lab((r, g, b))
    cyan = lab_cyan_score(l, a_val, b_val) if cyan_rgb_ok else 0
    green = lab_green_score(l, a_val, b_val) if green_rgb_ok else 0

    # RGB ratio hints. They are deliberately soft; LAB keeps the color anchor.
    if g > r:
        if cyan_rgb_ok:
            cyan += int((g - r) / 2)
        if green_rgb_ok:
            green += int((g - r) / 2)
    if cyan_rgb_ok and b > r:
        cyan += int((b - r) / 2)
    if green_rgb_ok and g > b:
        green += int((g - b) / 2)
    if cyan_rgb_ok and b >= g - 25:
        cyan += 18
    if green_rgb_ok and g >= b - 8:
        green += 14
    if green_rgb_ok and b > g + 55:
        green -= 22
    if cyan_rgb_ok and g > b + 45:
        cyan -= 18
    if cyan < 0:
        cyan = 0
    if green < 0:
        green = 0

    return cyan, green


def classify_car_marker_rgb(r, g, b):
    cyan, green = car_marker_scores(r, g, b)
    if cyan >= CAR_MARKER_SCORE_MIN and cyan >= green + CAR_MARKER_SCORE_MARGIN:
        return CLS_CAR_FRONT
    if green >= CAR_MARKER_SCORE_MIN and green >= cyan + CAR_MARKER_SCORE_MARGIN:
        return CLS_CAR_BACK
    return CLS_NONE


def is_cyan_marker(r, g, b):
    """Return True only when cyan clearly beats green."""
    return classify_car_marker_rgb(r, g, b) == CLS_CAR_FRONT


def is_green_marker(r, g, b):
    """Return True only when green clearly beats cyan."""
    return classify_car_marker_rgb(r, g, b) == CLS_CAR_BACK


def is_magenta_goal(r, g, b):
    return looks_magenta_goal_rgb(r, g, b)


# ----- 墙体 LAB 阈值 -----
OLD_WALL_L_MIN, OLD_WALL_L_MAX = 19, 100
OLD_WALL_A_MIN, OLD_WALL_A_MAX = -10, 65
OLD_WALL_B_MIN, OLD_WALL_B_MAX = -70, 6

NEW_WALL_L_MIN, NEW_WALL_L_MAX = 13, 100
NEW_WALL_A_MIN, NEW_WALL_A_MAX = -5, 35
NEW_WALL_B_MIN, NEW_WALL_B_MAX = 0, 30

if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW:
    WALL_L_MIN, WALL_L_MAX = NEW_WALL_L_MIN, NEW_WALL_L_MAX
    WALL_A_MIN, WALL_A_MAX = NEW_WALL_A_MIN, NEW_WALL_A_MAX
    WALL_B_MIN, WALL_B_MAX = NEW_WALL_B_MIN, NEW_WALL_B_MAX
else:
    WALL_L_MIN, WALL_L_MAX = OLD_WALL_L_MIN, OLD_WALL_L_MAX
    WALL_A_MIN, WALL_A_MAX = OLD_WALL_A_MIN, OLD_WALL_A_MAX
    WALL_B_MIN, WALL_B_MAX = OLD_WALL_B_MIN, OLD_WALL_B_MAX
WALL_STRONG_SCORE = 85


def is_gray_wall(r, g, b):
    """LAB 颜色空间判定墙体（替代原 RGB 阈值）

    阈值 (L:19~100, A:-10~65, B:-70~6) 对光照变化更鲁棒。
    """
    l, a_val, b_val = image.rgb_to_lab((r, g, b))
    return (WALL_L_MIN <= l <= WALL_L_MAX and
            WALL_A_MIN <= a_val <= WALL_A_MAX and
            WALL_B_MIN <= b_val <= WALL_B_MAX)


def wall_score_rgb(r, g, b):
    l, a_val, b_val = image.rgb_to_lab((r, g, b))
    score = (range_score(l, WALL_L_MIN, WALL_L_MAX, 24) +
             range_score(a_val, WALL_A_MIN, WALL_A_MAX, 28) +
             range_score(b_val, WALL_B_MIN, WALL_B_MAX, 24))
    avg = (r + g + b) // 3
    color_spread = abs(r - avg) + abs(g - avg) + abs(b - avg)
    if color_spread < 55:
        score += 18
    elif color_spread > 110:
        score -= 24
    if is_blue_floor(r, g, b):
        score -= 30
    if score < 0:
        score = 0
    return score


def is_blue_floor(r, g, b):
    return b >= 75 and b >= r + 45 and b >= g + 45


def classify_dynamic_rgb(r, g, b):
    if is_yellow(r, g, b):
        return CLS_BOX
    if is_red_or_orange(r, g, b):
        return CLS_BOMB_RED
    car_cls = classify_car_marker_rgb(r, g, b)
    if car_cls != CLS_NONE:
        return car_cls
    return CLS_NONE


def classify_cell_rgb(r, g, b):
    if is_yellow(r, g, b):
        return "$"
    if is_red_or_orange(r, g, b):
        return "!"
    if is_gray_wall(r, g, b):
        return "#"
    if is_magenta_goal(r, g, b):
        return "."
    if is_blue_floor(r, g, b):
        return "-"
    return "-"


# ----- RGB 采样 -----

def avg_rgb_at_uv(img, u, v):
    total_r, total_g, total_b, count = 0, 0, 0, 0
    offsets = ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1))
    cx, cy = screen_point_from_uv(u, v)
    for off in offsets:
        x = cx + off[0]
        y = cy + off[1]
        if x < 0 or x >= 320 or y < 0 or y >= 240:
            continue
        r, g, b = img.get_pixel(x, y)
        total_r += r; total_g += g; total_b += b
        count += 1
    if count == 0:
        return 0, 0, 0
    r = total_r // count
    g = total_g // count
    b = total_b // count
    return apply_brightness_gain_q6(r, g, b, brightness_gain_q6_at_uv(u, v))


def avg_rgb_at_cell(img, col, row):
    u = (col + 0.5) / COLS
    v = (row + 0.5) / ROWS
    return avg_rgb_at_uv(img, u, v)


# ============================================================
# 6. 分类图 + 连通分量（来自 识别代码.py，不变）
# ============================================================

def build_dynamic_class_map(img):
    if USE_REUSABLE_REGION_BUFFERS:
        cls_map = DYNAMIC_CLASS_BUFFER
    else:
        cls_map = bytearray(MAP_W * MAP_H)
    for sy in range(MAP_H):
        if not USE_PRECOMPUTED_SAMPLE_COORDS:
            v = (sy + 0.5) / MAP_H
        for sx in range(MAP_W):
            idx = sy * MAP_W + sx
            if USE_PRECOMPUTED_SAMPLE_COORDS:
                if SAMPLE_VISIBLE_MAP[idx] == 0:
                    cls_map[idx] = CLS_NONE
                    continue
                x = SAMPLE_X_LO[idx] | (SAMPLE_X_HI[idx] << 8)
                y = SAMPLE_Y_MAP[idx]
            else:
                u = (sx + 0.5) / MAP_W
                if not geometry_uv_visible(u, v):
                    cls_map[idx] = CLS_NONE
                    continue
                x, y = screen_point_from_uv(u, v)
            r, g, b = img.get_pixel(x, y)
            r, g, b = apply_brightness_gain_q6(
                r, g, b, GAIN_MAP_Q6[idx])
            cls_map[idx] = classify_dynamic_rgb(r, g, b)
    return cls_map


def build_region_class_map(img, center, radius_cells):
    if center is None:
        x0, y0 = 0, 0
        x1, y1 = MAP_W, MAP_H
    else:
        cx = int(center[0] * SCALE)
        cy = int(center[1] * SCALE)
        r  = int(radius_cells * SCALE)
        x0 = cx - r if cx - r > 0 else 0
        y0 = cy - r if cy - r > 0 else 0
        x1 = cx + r if cx + r < MAP_W else MAP_W
        y1 = cy + r if cy + r < MAP_H else MAP_H

    width  = x1 - x0
    height = y1 - y0
    if USE_REUSABLE_REGION_BUFFERS:
        cls_map = DYNAMIC_CLASS_BUFFER
    else:
        cls_map = bytearray(width * height)

    for sy in range(height):
        map_y = y0 + sy
        if not USE_PRECOMPUTED_SAMPLE_COORDS:
            v = (map_y + 0.5) / MAP_H
        for sx in range(width):
            map_x = x0 + sx
            src_idx = map_y * MAP_W + map_x
            dst_idx = sy * width + sx
            if USE_PRECOMPUTED_SAMPLE_COORDS:
                if SAMPLE_VISIBLE_MAP[src_idx] == 0:
                    cls_map[dst_idx] = CLS_NONE
                    continue
                x = SAMPLE_X_LO[src_idx] | (SAMPLE_X_HI[src_idx] << 8)
                y = SAMPLE_Y_MAP[src_idx]
            else:
                u = (map_x + 0.5) / MAP_W
                if not geometry_uv_visible(u, v):
                    cls_map[dst_idx] = CLS_NONE
                    continue
                x, y = screen_point_from_uv(u, v)
            r, g, b = img.get_pixel(x, y)
            r, g, b = apply_brightness_gain_q6(
                r, g, b, GAIN_MAP_Q6[src_idx])
            cls_map[dst_idx] = classify_dynamic_rgb(r, g, b)

    return cls_map, width, height, x0, y0


def find_components(cls_map, target_cls, min_pixels, ignore_border=False):
    visited = bytearray(MAP_W * MAP_H)
    comps = []

    for start in range(MAP_W * MAP_H):
        if visited[start] or cls_map[start] != target_cls:
            continue

        stack = [start]
        visited[start] = 1
        count, sum_x, sum_y = 0, 0, 0
        min_x, max_x = MAP_W, 0
        min_y, max_y = MAP_H, 0

        while stack:
            idx = stack.pop()
            x = idx % MAP_W
            y = idx // MAP_W
            count += 1
            sum_x += x; sum_y += y
            if x < min_x: min_x = x
            if x > max_x: max_x = x
            if y < min_y: min_y = y
            if y > max_y: max_y = y

            if x > 0:
                n = idx - 1
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)
            if x < MAP_W - 1:
                n = idx + 1
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)
            if y > 0:
                n = idx - MAP_W
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)
            if y < MAP_H - 1:
                n = idx + MAP_W
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)

        if count < min_pixels:
            continue

        cx = (sum_x / count + 0.5) / SCALE
        cy = (sum_y / count + 0.5) / SCALE

        if ignore_border:
            if cx < 0.4 or cx > COLS - 0.4 or cy < 0.4 or cy > ROWS - 0.4:
                continue

        comps.append((cx, cy, count, min_x, min_y, max_x, max_y))

    comps.sort(key=lambda item: item[2], reverse=True)
    return comps


def find_components_region(cls_map, width, height, origin_x, origin_y,
                           target_cls, min_pixels):
    visited = bytearray(width * height)
    comps = []

    for start in range(width * height):
        if visited[start] or cls_map[start] != target_cls:
            continue

        stack = [start]
        visited[start] = 1
        count, sum_x, sum_y = 0, 0, 0
        min_x, max_x = width, 0
        min_y, max_y = height, 0

        while stack:
            idx = stack.pop()
            x = idx % width
            y = idx // width
            count += 1
            sum_x += x; sum_y += y
            if x < min_x: min_x = x
            if x > max_x: max_x = x
            if y < min_y: min_y = y
            if y > max_y: max_y = y

            if x > 0:
                n = idx - 1
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)
            if x < width - 1:
                n = idx + 1
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)
            if y > 0:
                n = idx - width
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)
            if y < height - 1:
                n = idx + width
                if not visited[n] and cls_map[n] == target_cls:
                    visited[n] = 1; stack.append(n)

        if count < min_pixels:
            continue

        cx = (origin_x + sum_x / count + 0.5) / SCALE
        cy = (origin_y + sum_y / count + 0.5) / SCALE
        comps.append((cx, cy, count,
                       origin_x + min_x, origin_y + min_y,
                       origin_x + max_x, origin_y + max_y))

    comps.sort(key=lambda item: item[2], reverse=True)
    return comps


def find_all_dynamic_components_region(cls_map, width, height,
                                       origin_x, origin_y):
    """Extract all dynamic classes in one 4-neighbour traversal.

    Return order:
      car_front, car_back, boxes, bomb_red, bomb_dark
    Each list preserves the original component tuple and descending-size sort.
    """
    global DYNAMIC_VISIT_TOKEN, DYNAMIC_VISITED_BUFFER
    if USE_REUSABLE_REGION_BUFFERS:
        DYNAMIC_VISIT_TOKEN += 1
        if DYNAMIC_VISIT_TOKEN > 255:
            DYNAMIC_VISITED_BUFFER = bytearray(MAP_W * MAP_H)
            DYNAMIC_VISIT_TOKEN = 1
        visited = DYNAMIC_VISITED_BUFFER
        visit_token = DYNAMIC_VISIT_TOKEN
    else:
        visited = bytearray(width * height)
        visit_token = 1
    car_front = []
    car_back = []
    boxes = []
    bomb_red = []
    bomb_dark = []

    for start in range(width * height):
        if visited[start] == visit_token:
            continue
        target_cls = cls_map[start]
        if target_cls == CLS_NONE:
            visited[start] = visit_token
            continue

        stack = [start]
        visited[start] = visit_token
        count, sum_x, sum_y = 0, 0, 0
        min_x, max_x = width, 0
        min_y, max_y = height, 0

        while stack:
            idx = stack.pop()
            x = idx % width
            y = idx // width
            count += 1
            sum_x += x
            sum_y += y
            if x < min_x: min_x = x
            if x > max_x: max_x = x
            if y < min_y: min_y = y
            if y > max_y: max_y = y

            if x > 0:
                n = idx - 1
                if visited[n] != visit_token and cls_map[n] == target_cls:
                    visited[n] = visit_token
                    stack.append(n)
            if x < width - 1:
                n = idx + 1
                if visited[n] != visit_token and cls_map[n] == target_cls:
                    visited[n] = visit_token
                    stack.append(n)
            if y > 0:
                n = idx - width
                if visited[n] != visit_token and cls_map[n] == target_cls:
                    visited[n] = visit_token
                    stack.append(n)
            if y < height - 1:
                n = idx + width
                if visited[n] != visit_token and cls_map[n] == target_cls:
                    visited[n] = visit_token
                    stack.append(n)

        min_pixels = MIN_CAR_PIXELS
        target_list = None
        if target_cls == CLS_CAR_FRONT:
            target_list = car_front
        elif target_cls == CLS_CAR_BACK:
            target_list = car_back
        elif target_cls == CLS_BOX:
            min_pixels = MIN_BOX_PIXELS
            target_list = boxes
        elif target_cls == CLS_BOMB_RED:
            min_pixels = MIN_BOMB_PIXELS
            target_list = bomb_red
        elif target_cls == CLS_BOMB_DARK:
            min_pixels = MIN_BOMB_PIXELS
            target_list = bomb_dark

        if target_list is None or count < min_pixels:
            continue
        cx = (origin_x + sum_x / count + 0.5) / SCALE
        cy = (origin_y + sum_y / count + 0.5) / SCALE
        target_list.append((
            cx, cy, count,
            origin_x + min_x, origin_y + min_y,
            origin_x + max_x, origin_y + max_y))

    car_front.sort(key=lambda item: item[2], reverse=True)
    car_back.sort(key=lambda item: item[2], reverse=True)
    boxes.sort(key=lambda item: item[2], reverse=True)
    bomb_red.sort(key=lambda item: item[2], reverse=True)
    bomb_dark.sort(key=lambda item: item[2], reverse=True)
    return car_front, car_back, boxes, bomb_red, bomb_dark


def extract_dynamic_components_region(cls_map, width, height, origin_x, origin_y):
    if USE_MULTI_CLASS_COMPONENTS:
        return find_all_dynamic_components_region(
            cls_map, width, height, origin_x, origin_y)
    front = find_components_region(
        cls_map, width, height, origin_x, origin_y,
        CLS_CAR_FRONT, MIN_CAR_PIXELS)
    back = find_components_region(
        cls_map, width, height, origin_x, origin_y,
        CLS_CAR_BACK, MIN_CAR_PIXELS)
    boxes = find_components_region(
        cls_map, width, height, origin_x, origin_y,
        CLS_BOX, MIN_BOX_PIXELS)
    bomb_red = find_components_region(
        cls_map, width, height, origin_x, origin_y,
        CLS_BOMB_RED, MIN_BOMB_PIXELS)
    bomb_dark = find_components_region(
        cls_map, width, height, origin_x, origin_y,
        CLS_BOMB_DARK, MIN_BOMB_PIXELS)
    return front, back, boxes, bomb_red, bomb_dark


# ----- 辅助筛选 -----

def top_n(comps, n):
    return comps[:n] if len(comps) > n else comps


def sort_by_position(comps):
    comps.sort(key=lambda item: int(item[1] * 10) * 1000 + int(item[0] * 10))
    return comps


def largest(comps):
    return comps[0] if comps else None


def dist_cells(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def filter_near_components(comps, car, radius_cells, max_count):
    if car is None:
        return []
    out = []
    for comp in comps:
        d = dist_cells((comp[0], comp[1]), car)
        if d <= radius_cells:
            out.append((comp[0], comp[1], comp[2], comp[3], comp[4], comp[5], comp[6], d))
    out.sort(key=lambda item: item[7])
    return out[:max_count] if len(out) > max_count else out


# ============================================================
# 6b. Layered car/object state
# ============================================================

def object_state_name(state):
    names = (
        "FROZEN", "ACTIVE", "PUSH_CANDIDATE", "PUSHING_X",
        "PUSHING_Y", "OCCLUDED", "SETTLING", "BLOCKED",
        "EXPLOSION_PENDING", "REMOVED")
    if state >= 0 and state < len(names):
        return names[state]
    return "UNKNOWN"


def object_output_name(state):
    names = ("VALID", "OCCLUDED", "COMPLETED", "LOST")
    if state >= 0 and state < len(names):
        return names[state]
    return "UNKNOWN"


def make_object_track(object_id, object_type, comp, frame_seq):
    history = [(comp[0], comp[1])]
    confidence = 128 + comp[2]
    if confidence > 255:
        confidence = 255
    return [
        object_id, object_type, comp[0], comp[1],
        OBJ_STATE_FROZEN, OBJ_OUTPUT_VALID, confidence, 255,
        AXIS_NONE, 0, 0.0, 0.0, frame_seq, 0,
        0, 0, 0, 0, comp[0], comp[1], history, comp[2],
        -1, -1, 0, 0, 0, 0, 0, 0, AXIS_NONE, 0
    ]


def make_car_state(car, old_state, now_ms, observed):
    if car is None:
        return old_state
    if not observed and old_state is not None:
        return [
            old_state[C_X], old_state[C_Y], old_state[C_THETA],
            old_state[C_VX], old_state[C_VY], 128,
            old_state[C_LAST_SEEN]
        ]
    vx = 0.0
    vy = 0.0
    if old_state is not None:
        elapsed_ms = time.ticks_diff(now_ms, old_state[C_LAST_SEEN])
        if elapsed_ms > 0:
            vx = (car[0] - old_state[C_X]) / elapsed_ms
            vy = (car[1] - old_state[C_Y]) / elapsed_ms
    return [car[0], car[1], car[2], vx, vy, 255, now_ms]


def car_tuple_from_state(car_state):
    if car_state is None:
        return None
    return (car_state[C_X], car_state[C_Y], car_state[C_THETA])


def dedupe_components(comps, merge_distance=0.35):
    out = []
    for comp in comps:
        duplicate_index = -1
        for i in range(len(out)):
            if dist_cells(comp, out[i]) <= merge_distance:
                duplicate_index = i
                break
        if duplicate_index < 0:
            out.append(comp)
        elif comp[2] > out[duplicate_index][2]:
            out[duplicate_index] = comp
    return out


def object_distance_to_car(track, car):
    if car is None:
        return 999.0
    dx = track[O_X] - car[0]
    dy = track[O_Y] - car[1]
    return math.sqrt(dx * dx + dy * dy)


def choose_fast_search_radius(car, tracks):
    radius = CAR_TRACK_RADIUS_CELLS
    if not NEAR_OBJECT_TRACKING:
        return radius
    if not USE_ADAPTIVE_FAST_RADIUS:
        return max(radius, NEAR_OBJECT_PREPARE_RADIUS_CELLS)
    if car is None or tracks is None:
        return max(radius, NEAR_OBJECT_PREPARE_RADIUS_CELLS)

    wake_radius = (NEAR_OBJECT_PREPARE_RADIUS_CELLS +
                   OBJECT_ROI_WAKE_MARGIN_CELLS)
    wake_radius_sq = wake_radius * wake_radius
    for track in tracks:
        if track[O_STATE] == OBJ_STATE_REMOVED:
            continue
        dx = track[O_X] - car[0]
        dy = track[O_Y] - car[1]
        if dx * dx + dy * dy <= wake_radius_sq:
            return max(radius, NEAR_OBJECT_PREPARE_RADIUS_CELLS)
    return radius


def object_roi_wake_radius():
    return NEAR_OBJECT_PREPARE_RADIUS_CELLS + OBJECT_ROI_WAKE_MARGIN_CELLS


def track_needs_object_roi(track, car):
    if car is None or track[O_STATE] == OBJ_STATE_REMOVED:
        return False
    return object_distance_to_car(track, car) <= object_roi_wake_radius()


def object_roi_track_score(track, car):
    distance = object_distance_to_car(track, car)
    score = distance
    if track[O_STATE] in (
            OBJ_STATE_ACTIVE, OBJ_STATE_PUSH_CANDIDATE,
            OBJ_STATE_PUSHING_X, OBJ_STATE_PUSHING_Y):
        score -= 3.0
    return score


def predicted_car_search_center(car_state, now_ms):
    if not USE_PREDICTED_CAR_RECOVERY or car_state is None:
        return None
    vx = car_state[C_VX]
    vy = car_state[C_VY]
    min_speed_sq = (CAR_PREDICTION_MIN_SPEED_PER_MS *
                    CAR_PREDICTION_MIN_SPEED_PER_MS)
    if vx * vx + vy * vy < min_speed_sq:
        return None
    elapsed_ms = time.ticks_diff(now_ms, car_state[C_LAST_SEEN])
    if elapsed_ms <= 0:
        return None
    if elapsed_ms > CAR_PREDICTION_MAX_ELAPSED_MS:
        elapsed_ms = CAR_PREDICTION_MAX_ELAPSED_MS
    prediction_ms = elapsed_ms * CAR_PREDICTION_TIME_GAIN
    x = car_state[C_X] + vx * prediction_ms
    y = car_state[C_Y] + vy * prediction_ms
    if x < 0.0:
        x = 0.0
    elif x > COLS:
        x = COLS
    if y < 0.0:
        y = 0.0
    elif y > ROWS:
        y = ROWS
    return (x, y, car_state[C_THETA])


def recover_car_and_region_objects(img, center, radius_cells):
    cls_map, width, height, offset_x, offset_y = build_region_class_map(
        img, center, radius_cells)
    front, back, boxes, bomb_red, bomb_dark = extract_dynamic_components_region(
        cls_map, width, height, offset_x, offset_y)
    detected = find_best_car_pair(
        front, back, CAR_MARKER_MIN_DIST, CAR_MARKER_MAX_DIST)
    if detected is None:
        detected = rescue_car_near_backs(img, back, False)
    return detected, front, back, boxes, bomb_red, bomb_dark


def collect_object_roi_observations(img, tracks, car, frame_seq):
    boxes = []
    bombs = []
    region_ms = 0
    component_ms = 0
    if tracks is None or car is None:
        return boxes, bombs, region_ms, component_ms

    candidates = []
    for track in tracks:
        if not track_needs_object_roi(track, car):
            continue
        candidates.append((object_roi_track_score(track, car), track))
    candidates.sort(key=lambda item: item[0])
    limit = OBJECT_ROI_MAX_TRACKS
    if limit <= 0 or limit > len(candidates):
        limit = len(candidates)

    for i in range(limit):
        if i > 0 and OBJECT_ROI_SECONDARY_PERIOD > 1:
            if frame_seq % OBJECT_ROI_SECONDARY_PERIOD != 0:
                continue
        track = candidates[i][1]
        center = (track[O_X], track[O_Y], 0.0)
        start_ms = time.ticks_ms()
        cls_map, width, height, offset_x, offset_y = build_region_class_map(
            img, center, OBJECT_TRACK_RADIUS_CELLS)
        region_ms += time.ticks_diff(time.ticks_ms(), start_ms)

        start_ms = time.ticks_ms()
        _front, _back, roi_boxes, bomb_red, bomb_dark = (
            extract_dynamic_components_region(
                cls_map, width, height, offset_x, offset_y))
        component_ms += time.ticks_diff(time.ticks_ms(), start_ms)
        boxes += roi_boxes
        bombs += bomb_red
        bombs += bomb_dark
    return boxes, bombs, region_ms, component_ms


def track_observation_distance(track, observation):
    dx = track[O_X] - observation[0]
    dy = track[O_Y] - observation[1]
    return math.sqrt(dx * dx + dy * dy)


def freeze_track_at_stable_position(track):
    history = track[O_HISTORY]
    if not isinstance(history, list) or not history:
        track[O_HISTORY] = [(track[O_X], track[O_Y])]
        return
    if (track[O_VX] * track[O_VX] + track[O_VY] * track[O_VY] >
            OBJECT_SETTLED_SPEED * OBJECT_SETTLED_SPEED):
        return
    xs = []
    ys = []
    for pos in history:
        try:
            xs.append(pos[0])
            ys.append(pos[1])
        except Exception:
            track[O_HISTORY] = [(track[O_X], track[O_Y])]
            return
    xs.sort()
    ys.sort()
    mid = len(xs) // 2
    track[O_X] = xs[mid]
    track[O_Y] = ys[mid]
    track[O_ANCHOR_X] = track[O_X]
    track[O_ANCHOR_Y] = track[O_Y]


def update_track_activation(track, car):
    if track[O_STATE] == OBJ_STATE_REMOVED:
        return
    distance = object_distance_to_car(track, car)
    if distance <= OBJECT_ENTER_ACTIVE_RADIUS:
        track[O_ENTER_COUNT] += 1
        track[O_EXIT_COUNT] = 0
        if (track[O_ENTER_COUNT] >= OBJECT_ACTIVATE_FRAMES and
                track[O_STATE] == OBJ_STATE_FROZEN):
            track[O_STATE] = OBJ_STATE_ACTIVE
    elif distance >= OBJECT_EXIT_ACTIVE_RADIUS:
        track[O_EXIT_COUNT] += 1
        track[O_ENTER_COUNT] = 0
        if (track[O_EXIT_COUNT] >= OBJECT_FREEZE_FRAMES and
                track[O_STATE] not in (
                    OBJ_STATE_FROZEN, OBJ_STATE_OCCLUDED,
                    OBJ_STATE_EXPLOSION_PENDING, OBJ_STATE_REMOVED)):
            freeze_track_at_stable_position(track)
            track[O_STATE] = OBJ_STATE_FROZEN
            track[O_VX] = 0.0
            track[O_VY] = 0.0
    else:
        track[O_ENTER_COUNT] = 0
        track[O_EXIT_COUNT] = 0


def object_cell_is_blocked(track, x, y):
    if g_static_grid is None:
        return False
    col = clamp_cell_x(x)
    row = clamp_cell_y(y)
    is_border = (col == 0 or col == COLS - 1 or
                 row == 0 or row == ROWS - 1)
    cell = grid_get(g_static_grid, col, row)
    if track[O_TYPE] == OBJ_TYPE_BOX:
        return cell == "#"
    if track[O_TYPE] == OBJ_TYPE_BOMB:
        return is_border and cell == "#"
    return False


def object_hits_other_track(track, x, y, tracks):
    for other in tracks:
        if other is track or other[O_STATE] == OBJ_STATE_REMOVED:
            continue
        dx = x - other[O_X]
        dy = y - other[O_Y]
        if dx * dx + dy * dy < 0.45 * 0.45:
            return True
    return False


def update_track_axis(track, raw_dx, raw_dy):
    abs_dx = raw_dx if raw_dx >= 0 else -raw_dx
    abs_dy = raw_dy if raw_dy >= 0 else -raw_dy

    if track[O_AXIS] == AXIS_NONE:
        candidate = AXIS_NONE
        if (abs_dx >= OBJECT_AXIS_LOCK_MIN_DELTA and
                abs_dx >= abs_dy * OBJECT_AXIS_LOCK_RATIO):
            candidate = AXIS_X
        elif (abs_dy >= OBJECT_AXIS_LOCK_MIN_DELTA and
                abs_dy >= abs_dx * OBJECT_AXIS_LOCK_RATIO):
            candidate = AXIS_Y

        if candidate == AXIS_X:
            if track[O_AXIS_COUNT] < 0:
                track[O_AXIS_COUNT] = 0
            track[O_AXIS_COUNT] += 1
        elif candidate == AXIS_Y:
            if track[O_AXIS_COUNT] > 0:
                track[O_AXIS_COUNT] = 0
            track[O_AXIS_COUNT] -= 1
        else:
            track[O_AXIS_COUNT] = 0

        if track[O_AXIS_COUNT] >= OBJECT_AXIS_LOCK_FRAMES:
            track[O_AXIS] = AXIS_X
            track[O_ANCHOR_Y] = track[O_Y]
            track[O_AXIS_COUNT] = 0
        elif track[O_AXIS_COUNT] <= -OBJECT_AXIS_LOCK_FRAMES:
            track[O_AXIS] = AXIS_Y
            track[O_ANCHOR_X] = track[O_X]
            track[O_AXIS_COUNT] = 0
    elif track[O_AXIS] == AXIS_X:
        if (abs_dy >= OBJECT_AXIS_LOCK_MIN_DELTA and
                abs_dy >= abs_dx * OBJECT_AXIS_LOCK_RATIO):
            if track[O_AXIS_COUNT] > 0:
                track[O_AXIS_COUNT] = 0
            track[O_AXIS_COUNT] -= 1
            if track[O_AXIS_COUNT] <= -OBJECT_AXIS_SWITCH_FRAMES:
                track[O_AXIS] = AXIS_Y
                track[O_ANCHOR_X] = track[O_X]
                track[O_AXIS_COUNT] = 0
        elif abs_dx >= OBJECT_MOVE_MIN:
            track[O_AXIS_COUNT] = 0
    elif track[O_AXIS] == AXIS_Y:
        if (abs_dx >= OBJECT_AXIS_LOCK_MIN_DELTA and
                abs_dx >= abs_dy * OBJECT_AXIS_LOCK_RATIO):
            if track[O_AXIS_COUNT] < 0:
                track[O_AXIS_COUNT] = 0
            track[O_AXIS_COUNT] += 1
            if track[O_AXIS_COUNT] >= OBJECT_AXIS_SWITCH_FRAMES:
                track[O_AXIS] = AXIS_X
                track[O_ANCHOR_Y] = track[O_Y]
                track[O_AXIS_COUNT] = 0
        elif abs_dy >= OBJECT_MOVE_MIN:
            track[O_AXIS_COUNT] = 0

    if track[O_AXIS] == AXIS_X and abs_dx >= OBJECT_MOVE_MIN:
        track[O_DIRECTION] = 1 if raw_dx > 0 else -1
        track[O_LAST_MOVE_AXIS] = AXIS_X
        track[O_LAST_MOVE_DIRECTION] = track[O_DIRECTION]
    elif track[O_AXIS] == AXIS_Y and abs_dy >= OBJECT_MOVE_MIN:
        track[O_DIRECTION] = 1 if raw_dy > 0 else -1
        track[O_LAST_MOVE_AXIS] = AXIS_Y
        track[O_LAST_MOVE_DIRECTION] = track[O_DIRECTION]


def apply_track_observation(track, comp, tracks, frame_seq):
    old_x = track[O_X]
    old_y = track[O_Y]
    raw_x = comp[0]
    raw_y = comp[1]
    raw_dx = raw_x - old_x
    raw_dy = raw_y - old_y

    update_track_axis(track, raw_dx, raw_dy)
    if track[O_AXIS] == AXIS_X:
        raw_y = track[O_ANCHOR_Y]
    elif track[O_AXIS] == AXIS_Y:
        raw_x = track[O_ANCHOR_X]

    new_x = old_x + (raw_x - old_x) * OBJECT_POSITION_ALPHA
    new_y = old_y + (raw_y - old_y) * OBJECT_POSITION_ALPHA
    if (object_cell_is_blocked(track, new_x, new_y) or
            object_hits_other_track(track, new_x, new_y, tracks)):
        track[O_STATE] = OBJ_STATE_BLOCKED
        track[O_VX] = 0.0
        track[O_VY] = 0.0
        track[O_MISSES] = 0
        track[O_LAST_SEEN] = frame_seq
        return False

    track[O_X] = new_x
    track[O_Y] = new_y
    track[O_VX] = new_x - old_x
    track[O_VY] = new_y - old_y
    track[O_LAST_SEEN] = frame_seq
    track[O_MISSES] = 0
    track[O_OUTPUT] = OBJ_OUTPUT_VALID
    previous_pixels = track[O_PIXELS]
    if previous_pixels > 0 and comp[2] < previous_pixels:
        track[O_VISIBLE_Q8] = (comp[2] * 255) // previous_pixels
    else:
        track[O_VISIBLE_Q8] = 255
        track[O_PIXELS] = comp[2]
    confidence = 128 + comp[2]
    track[O_CONFIDENCE] = 255 if confidence > 255 else confidence

    speed_sq = track[O_VX] * track[O_VX] + track[O_VY] * track[O_VY]
    if speed_sq <= OBJECT_SETTLED_SPEED * OBJECT_SETTLED_SPEED:
        track[O_STILL_COUNT] += 1
        if track[O_STATE] not in (OBJ_STATE_FROZEN, OBJ_STATE_REMOVED):
            track[O_STATE] = OBJ_STATE_SETTLING
        if track[O_STILL_COUNT] >= OBJECT_AXIS_RELEASE_STILL_FRAMES:
            track[O_AXIS] = AXIS_NONE
            track[O_AXIS_COUNT] = 0
            track[O_DIRECTION] = 0
            track[O_ANCHOR_X] = track[O_X]
            track[O_ANCHOR_Y] = track[O_Y]
    else:
        track[O_STILL_COUNT] = 0
        if track[O_AXIS] == AXIS_X:
            track[O_STATE] = OBJ_STATE_PUSHING_X
        elif track[O_AXIS] == AXIS_Y:
            track[O_STATE] = OBJ_STATE_PUSHING_Y
        else:
            track[O_STATE] = OBJ_STATE_PUSH_CANDIDATE

    history = track[O_HISTORY]
    if not isinstance(history, list):
        history = [(old_x, old_y)]
        track[O_HISTORY] = history
    history.append((track[O_X], track[O_Y]))
    if len(history) > OBJECT_HISTORY_COUNT:
        history.pop(0)
    return True


def mark_track_missed(track, tracks, should_search):
    if not should_search or track[O_STATE] == OBJ_STATE_REMOVED:
        return
    track[O_MISSES] += 1
    if track[O_CONFIDENCE] > 8:
        track[O_CONFIDENCE] -= 8
    else:
        track[O_CONFIDENCE] = 0
    if track[O_MISSES] >= OBJECT_LOST_AFTER_MISSES:
        track[O_OUTPUT] = OBJ_OUTPUT_LOST
        track[O_VISIBLE_Q8] = 0
        return
    if track[O_MISSES] >= OBJECT_OCCLUDED_AFTER_MISSES:
        if track[O_STATE] != OBJ_STATE_EXPLOSION_PENDING:
            track[O_STATE] = OBJ_STATE_OCCLUDED
        track[O_OUTPUT] = OBJ_OUTPUT_OCCLUDED
        track[O_VISIBLE_Q8] = 0

        # Short, bounded extrapolation along the already confirmed axis.
        if track[O_MISSES] <= OBJECT_OCCLUDED_AFTER_MISSES + 2:
            next_x = track[O_X]
            next_y = track[O_Y]
            if track[O_AXIS] == AXIS_X:
                next_x += track[O_VX]
            elif track[O_AXIS] == AXIS_Y:
                next_y += track[O_VY]
            if (not object_cell_is_blocked(track, next_x, next_y) and
                    not object_hits_other_track(track, next_x, next_y, tracks)):
                track[O_X] = next_x
                track[O_Y] = next_y
            track[O_VX] *= 0.5
            track[O_VY] *= 0.5


def initialize_object_tracks(boxes, bombs, frame_seq):
    tracks = []
    boxes = sort_by_position(dedupe_components(boxes))
    bombs = sort_by_position(dedupe_components(bombs))
    max_boxes = MCU_MAX_BOXES if len(boxes) > MCU_MAX_BOXES else len(boxes)
    max_bombs = VISION_MAX_BOMBS if len(bombs) > VISION_MAX_BOMBS else len(bombs)
    for i in range(max_boxes):
        tracks.append(make_object_track(i, OBJ_TYPE_BOX, boxes[i], frame_seq))
    for i in range(max_bombs):
        tracks.append(make_object_track(i, OBJ_TYPE_BOMB, bombs[i], frame_seq))
    return tracks


def update_object_tracks(tracks, box_observations, bomb_observations,
                         car, frame_seq, full_scan=False):
    if tracks is None:
        return
    observations_by_type = (
        (OBJ_TYPE_BOX, dedupe_components(box_observations)),
        (OBJ_TYPE_BOMB, dedupe_components(bomb_observations)))

    for track in tracks:
        update_track_activation(track, car)

    for object_type, observations in observations_by_type:
        used = bytearray(len(observations))
        for track in tracks:
            if (track[O_TYPE] != object_type or
                    track[O_STATE] == OBJ_STATE_REMOVED):
                continue
            distance_to_car = object_distance_to_car(track, car)
            search_radius = OBJECT_EXIT_ACTIVE_RADIUS
            if USE_SEPARATE_OBJECT_ROIS:
                search_radius = object_roi_wake_radius()
            should_search = full_scan or distance_to_car <= search_radius
            should_mark_missed = (
                full_scan or distance_to_car <= OBJECT_EXIT_ACTIVE_RADIUS)
            if not should_search:
                continue

            best_i = -1
            best_d = 999.0
            max_dist = OBJECT_ASSOCIATE_MAX_DIST
            if track[O_OUTPUT] != OBJ_OUTPUT_VALID:
                max_dist = OBJECT_OCCLUDED_ASSOCIATE_MAX_DIST
            for i in range(len(observations)):
                if used[i]:
                    continue
                d = track_observation_distance(track, observations[i])
                if d <= max_dist and d < best_d:
                    best_i = i
                    best_d = d

            if best_i >= 0:
                used[best_i] = 1
                was_frozen = track[O_STATE] == OBJ_STATE_FROZEN
                apply_track_observation(
                    track, observations[best_i], tracks, frame_seq)
                if was_frozen:
                    track[O_STATE] = OBJ_STATE_FROZEN
                    track[O_VX] = 0.0
                    track[O_VY] = 0.0
            else:
                mark_track_missed(track, tracks, should_mark_missed)


def tracked_components(tracks, object_type, include_completed=False):
    out = []
    if tracks is None:
        return out
    for track in tracks:
        if track[O_TYPE] != object_type:
            continue
        if track[O_STATE] == OBJ_STATE_REMOVED and not include_completed:
            continue
        if track[O_OUTPUT] == OBJ_OUTPUT_COMPLETED and not include_completed:
            continue
        sample_x = int(track[O_X] * SCALE)
        sample_y = int(track[O_Y] * SCALE)
        out.append((track[O_X], track[O_Y], track[O_PIXELS],
                    sample_x, sample_y, sample_x, sample_y))
    return out


def debug_print_object_tracks(tracks):
    if not PRINT_DEBUG or tracks is None:
        return
    for track in tracks:
        kind = "box" if track[O_TYPE] == OBJ_TYPE_BOX else "bomb"
        print("  track %s%d pos=(%.2f,%.2f) state=%s out=%s axis=%d dir=%d miss=%d conf=%d visible=%d" %
              (kind, track[O_ID], track[O_X], track[O_Y],
               object_state_name(track[O_STATE]),
               object_output_name(track[O_OUTPUT]),
               track[O_AXIS], track[O_DIRECTION], track[O_MISSES],
               track[O_CONFIDENCE], track[O_VISIBLE_Q8]))


def queue_visual_event(name, track, col, row, send_uart_full_map=True):
    global g_event_queue
    object_type = 0
    object_id = -1
    if track is not None:
        object_type = track[O_TYPE]
        object_id = track[O_ID]
    event = (name, object_type, object_id, col, row,
             g_map_version, 1 if send_uart_full_map else 0)
    for queued in g_event_queue:
        if queued[0] == name and queued[1] == object_type and queued[2] == object_id:
            return
    g_event_queue.append(event)


def nearest_goal_cell(x, y):
    best = None
    best_dist_sq = BOX_GOAL_ENTRY_RADIUS * BOX_GOAL_ENTRY_RADIUS
    for goal in g_static_goals:
        gx = goal[0] + 0.5
        gy = goal[1] + 0.5
        dx = x - gx
        dy = y - gy
        dist_sq = dx * dx + dy * dy
        if dist_sq <= best_dist_sq:
            best_dist_sq = dist_sq
            best = goal
    return best


def goal_trace(track, trace_state, message):
    if not IDE_GOAL_TRACE:
        return
    if track[O_GOAL_TRACE_STATE] == trace_state:
        return
    track[O_GOAL_TRACE_STATE] = trace_state
    print("TRACE %s" % message)


def recent_motion_axis_direction(track):
    if track[O_AXIS] != AXIS_NONE and track[O_DIRECTION] != 0:
        return track[O_AXIS], track[O_DIRECTION]

    vx = track[O_VX]
    vy = track[O_VY]
    abs_vx = vx if vx >= 0 else -vx
    abs_vy = vy if vy >= 0 else -vy
    if abs_vx >= BOX_GOAL_HISTORY_MIN_DELTA and abs_vx >= abs_vy:
        return AXIS_X, 1 if vx > 0 else -1
    if abs_vy >= BOX_GOAL_HISTORY_MIN_DELTA:
        return AXIS_Y, 1 if vy > 0 else -1

    if (track[O_LAST_MOVE_AXIS] != AXIS_NONE and
            track[O_LAST_MOVE_DIRECTION] != 0):
        return track[O_LAST_MOVE_AXIS], track[O_LAST_MOVE_DIRECTION]

    history = track[O_HISTORY]
    if not isinstance(history, list) or len(history) < 2:
        track[O_HISTORY] = [(track[O_X], track[O_Y])]
        return AXIS_NONE, 0
    try:
        first = history[0]
        last = history[len(history) - 1]
        dx = last[0] - first[0]
        dy = last[1] - first[1]
    except Exception:
        track[O_HISTORY] = [(track[O_X], track[O_Y])]
        goal_trace(
            track, 30,
            "HISTORY_RESET box=%d invalid_history" % track[O_ID])
        return AXIS_NONE, 0
    abs_dx = dx if dx >= 0 else -dx
    abs_dy = dy if dy >= 0 else -dy
    if abs_dx >= BOX_GOAL_HISTORY_MIN_DELTA and abs_dx >= abs_dy:
        return AXIS_X, 1 if dx > 0 else -1
    if abs_dy >= BOX_GOAL_HISTORY_MIN_DELTA:
        return AXIS_Y, 1 if dy > 0 else -1
    return AXIS_NONE, 0


def forward_goal_cell(track):
    axis, direction = recent_motion_axis_direction(track)
    if axis == AXIS_NONE or direction == 0:
        return None
    best = None
    best_forward = BOX_GOAL_FORWARD_LOOKAHEAD + 1.0
    for goal in g_static_goals:
        gx = goal[0] + 0.5
        gy = goal[1] + 0.5
        if axis == AXIS_X:
            forward = (gx - track[O_X]) * direction
            lateral = gy - track[O_Y]
        else:
            forward = (gy - track[O_Y]) * direction
            lateral = gx - track[O_X]
        if lateral < 0:
            lateral = -lateral
        if (forward >= 0.0 and
                forward <= BOX_GOAL_FORWARD_LOOKAHEAD and
                lateral <= BOX_GOAL_LATERAL_TOLERANCE and
                forward < best_forward):
            best_forward = forward
            best = goal
    return best


def clear_track_goal_candidate(track):
    track[O_GOAL_X] = -1
    track[O_GOAL_Y] = -1
    track[O_GOAL_FLOOR_COUNT] = 0
    track[O_GOAL_VISIBLE_COUNT] = 0
    track[O_GOAL_PENDING_AGE] = 0
    track[O_GOAL_RESULT] = 0
    track[O_GOAL_TRACE_STATE] = 0
    track[O_GOAL_LAST_EVIDENCE] = 0


def existing_goal_candidate_still_near(track):
    if track[O_GOAL_X] < 0 or track[O_GOAL_Y] < 0:
        return False
    dx = track[O_X] - (track[O_GOAL_X] + 0.5)
    dy = track[O_Y] - (track[O_GOAL_Y] + 0.5)
    return dx * dx + dy * dy <= (
        BOX_GOAL_CANDIDATE_KEEP_RADIUS * BOX_GOAL_CANDIDATE_KEEP_RADIUS)


def track_is_on_goal_candidate(track):
    if track[O_GOAL_X] < 0 or track[O_GOAL_Y] < 0:
        return False
    dx = track[O_X] - (track[O_GOAL_X] + 0.5)
    dy = track[O_Y] - (track[O_GOAL_Y] + 0.5)
    radius = BOX_GOAL_ENTRY_RADIUS + 0.18
    return dx * dx + dy * dy <= radius * radius


def track_near_goal_candidate(track, radius):
    if track[O_GOAL_X] < 0 or track[O_GOAL_Y] < 0:
        return False
    dx = track[O_X] - (track[O_GOAL_X] + 0.5)
    dy = track[O_Y] - (track[O_GOAL_Y] + 0.5)
    return dx * dx + dy * dy <= radius * radius


def update_track_goal_candidate(track):
    if track[O_TYPE] != OBJ_TYPE_BOX or track[O_STATE] == OBJ_STATE_REMOVED:
        return
    goal = nearest_goal_cell(track[O_X], track[O_Y])
    if goal is None:
        goal = forward_goal_cell(track)
    if goal is None:
        if (track[O_OUTPUT] == OBJ_OUTPUT_VALID and
                not existing_goal_candidate_still_near(track)):
            clear_track_goal_candidate(track)
        return
    if track[O_GOAL_X] != goal[0] or track[O_GOAL_Y] != goal[1]:
        track[O_GOAL_X] = goal[0]
        track[O_GOAL_Y] = goal[1]
        track[O_GOAL_FLOOR_COUNT] = 0
        track[O_GOAL_VISIBLE_COUNT] = 0
        track[O_GOAL_PENDING_AGE] = 0
        track[O_GOAL_RESULT] = 0
        track[O_GOAL_LAST_EVIDENCE] = 0
        axis, direction = recent_motion_axis_direction(track)
        goal_trace(
            track, 1,
            "GOAL_CANDIDATE box=%d goal=(%d,%d) pos=(%.2f,%.2f) axis=%d dir=%d" %
            (track[O_ID], goal[0], goal[1], track[O_X], track[O_Y],
             axis, direction))


def goal_verification_block_reason(track, tracks, car):
    col = track[O_GOAL_X]
    row = track[O_GOAL_Y]
    if col < 0 or row < 0:
        return 3
    center_x = col + 0.5
    center_y = row + 0.5
    if car is not None:
        dx = car[0] - center_x
        dy = car[1] - center_y
        if dx * dx + dy * dy <= (
                BOX_GOAL_CAR_CLEAR_RADIUS * BOX_GOAL_CAR_CLEAR_RADIUS):
            return 1
    for other in tracks:
        if other is track or other[O_STATE] == OBJ_STATE_REMOVED:
            continue
        dx = other[O_X] - center_x
        dy = other[O_Y] - center_y
        if dx * dx + dy * dy <= 0.45 * 0.45:
            return 2
    return 0


def classify_goal_verification(img, col, row):
    stats = static_cell_stats(img, col, row)
    observed = classify_static_cell_from_stats(
        col, row, stats[0], stats[1], stats[2], stats[3], stats[4])
    if observed == "-" and static_observation_is_strong("-", stats):
        return "-"
    if observed == "." and static_observation_is_strong(".", stats):
        return "."
    return "?"


def reset_global_goal_evidence():
    global g_goal_floor_counts, g_goal_last_trace
    for i in range(COLS * ROWS):
        g_goal_floor_counts[i] = 0
        g_goal_last_trace[i] = 0


def commit_box_goal_match(track):
    global g_map_version, g_static_goals
    col = track[O_GOAL_X]
    row = track[O_GOAL_Y]
    set_grid_cell_safe(g_static_grid, col, row, "-")
    g_static_goals = detect_goals_from_grid(g_static_grid)
    reset_global_goal_evidence()
    g_map_version = (g_map_version + 1) & 0xFF
    if g_map_version == 0:
        g_map_version = 1
    track[O_X] = col + 0.5
    track[O_Y] = row + 0.5
    track[O_STATE] = OBJ_STATE_REMOVED
    track[O_OUTPUT] = OBJ_OUTPUT_COMPLETED
    track[O_GOAL_RESULT] = 1
    track[O_VX] = 0.0
    track[O_VY] = 0.0
    queue_visual_event("BOX_GOAL_MATCHED", track, col, row, True)


def commit_box_goal_mismatch(track):
    col = track[O_GOAL_X]
    row = track[O_GOAL_Y]
    track[O_X] = col + 0.5
    track[O_Y] = row + 0.5
    track[O_STATE] = OBJ_STATE_OCCLUDED
    track[O_OUTPUT] = OBJ_OUTPUT_OCCLUDED
    track[O_GOAL_RESULT] = 2
    track[O_VX] = 0.0
    track[O_VY] = 0.0
    queue_visual_event("BOX_GOAL_MISMATCHED", track, col, row, True)


def update_box_goal_events(img, tracks, car):
    if tracks is None or g_static_grid is None:
        return
    for track in tracks:
        if track[O_TYPE] != OBJ_TYPE_BOX or track[O_STATE] == OBJ_STATE_REMOVED:
            continue

        if track[O_OUTPUT] == OBJ_OUTPUT_VALID:
            update_track_goal_candidate(track)
            if not track_is_on_goal_candidate(track):
                continue
        if track[O_MISSES] >= BOX_GOAL_MISSING_FRAMES:
            update_track_goal_candidate(track)
        if track[O_GOAL_X] < 0 or track[O_GOAL_Y] < 0:
            if track[O_MISSES] >= BOX_GOAL_MISSING_FRAMES:
                axis, direction = recent_motion_axis_direction(track)
                goal_trace(
                    track, 2,
                    "BOX_MISSING_NO_GOAL box=%d pos=(%.2f,%.2f) axis=%d dir=%d misses=%d" %
                    (track[O_ID], track[O_X], track[O_Y],
                     axis, direction, track[O_MISSES]))
            continue
        if track[O_GOAL_RESULT] == 2:
            continue
        on_goal_candidate = track_is_on_goal_candidate(track)
        if (track[O_MISSES] < BOX_GOAL_MISSING_FRAMES and
                not on_goal_candidate):
            continue
        if track[O_GOAL_TRACE_STATE] not in (3, 11, 12, 13, 20):
            goal_trace(
                track, 3,
                "BOX_GOAL_VERIFY box=%d candidate=(%d,%d) pos=(%.2f,%.2f) misses=%d on_goal=%d" %
                (track[O_ID], track[O_GOAL_X], track[O_GOAL_Y],
                 track[O_X], track[O_Y], track[O_MISSES],
                 1 if on_goal_candidate else 0))
        block_reason = goal_verification_block_reason(track, tracks, car)
        if block_reason:
            reason_name = "CAR" if block_reason == 1 else (
                "OBJECT" if block_reason == 2 else "NO_GOAL")
            goal_trace(
                track, 10 + block_reason,
                "GOAL_VERIFY_BLOCKED box=%d goal=(%d,%d) reason=%s" %
                (track[O_ID], track[O_GOAL_X],
                 track[O_GOAL_Y], reason_name))
            continue

        track[O_GOAL_PENDING_AGE] += 1
        evidence = classify_goal_verification(
            img, track[O_GOAL_X], track[O_GOAL_Y])
        evidence_code = 1 if evidence == "-" else (2 if evidence == "." else 3)
        if evidence_code != track[O_GOAL_LAST_EVIDENCE]:
            track[O_GOAL_LAST_EVIDENCE] = evidence_code
            if IDE_GOAL_TRACE:
                evidence_name = "FLOOR" if evidence == "-" else (
                    "GOAL" if evidence == "." else "UNKNOWN")
                print("TRACE GOAL_VERIFY box=%d goal=(%d,%d) evidence=%s floor=%d goal_count=%d age=%d" %
                      (track[O_ID], track[O_GOAL_X], track[O_GOAL_Y],
                       evidence_name, track[O_GOAL_FLOOR_COUNT],
                       track[O_GOAL_VISIBLE_COUNT],
                       track[O_GOAL_PENDING_AGE]))
        if evidence == "-":
            track[O_GOAL_FLOOR_COUNT] += 1
            track[O_GOAL_VISIBLE_COUNT] = 0
        elif evidence == ".":
            track[O_GOAL_VISIBLE_COUNT] += 1
            track[O_GOAL_FLOOR_COUNT] = 0
        else:
            track[O_GOAL_FLOOR_COUNT] = 0
            track[O_GOAL_VISIBLE_COUNT] = 0

        if track[O_GOAL_FLOOR_COUNT] >= BOX_GOAL_VERIFY_FRAMES:
            if track_near_goal_candidate(
                    track, GLOBAL_GOAL_EXPLICIT_ASSIGN_RADIUS):
                commit_box_goal_match(track)
            else:
                goal_trace(
                    track, 21,
                    "GOAL_VERIFY_STALE box=%d goal=(%d,%d) pos=(%.2f,%.2f)" %
                    (track[O_ID], track[O_GOAL_X], track[O_GOAL_Y],
                     track[O_X], track[O_Y]))
                clear_track_goal_candidate(track)
        elif track[O_GOAL_VISIBLE_COUNT] >= BOX_GOAL_VERIFY_FRAMES:
            if track_near_goal_candidate(
                    track, GLOBAL_GOAL_EXPLICIT_ASSIGN_RADIUS):
                commit_box_goal_mismatch(track)
            else:
                goal_trace(
                    track, 21,
                    "GOAL_VERIFY_STALE box=%d goal=(%d,%d) pos=(%.2f,%.2f)" %
                    (track[O_ID], track[O_GOAL_X], track[O_GOAL_Y],
                     track[O_X], track[O_Y]))
                clear_track_goal_candidate(track)
        elif track[O_GOAL_PENDING_AGE] >= BOX_GOAL_PENDING_MAX_FRAMES:
            track[O_GOAL_RESULT] = 3
            goal_trace(
                track, 20,
                "GOAL_VERIFY_TIMEOUT box=%d goal=(%d,%d)" %
                (track[O_ID], track[O_GOAL_X], track[O_GOAL_Y]))


def global_goal_track_score(track, goal):
    gx = goal[0] + 0.5
    gy = goal[1] + 0.5
    dx = gx - track[O_X]
    dy = gy - track[O_Y]
    dist_sq = dx * dx + dy * dy
    max_dist_sq = (GLOBAL_GOAL_ASSIGN_MAX_DIST *
                   GLOBAL_GOAL_ASSIGN_MAX_DIST)

    axis, direction = recent_motion_axis_direction(track)
    if axis == AXIS_X and direction != 0:
        forward = dx * direction
        lateral = dy if dy >= 0 else -dy
        if (forward >= -0.35 and
                forward <= GLOBAL_GOAL_ASSIGN_FORWARD_LOOKAHEAD and
                lateral <= GLOBAL_GOAL_ASSIGN_LATERAL_TOLERANCE):
            return dist_sq + lateral * 2.0
    elif axis == AXIS_Y and direction != 0:
        forward = dy * direction
        lateral = dx if dx >= 0 else -dx
        if (forward >= -0.35 and
                forward <= GLOBAL_GOAL_ASSIGN_FORWARD_LOOKAHEAD and
                lateral <= GLOBAL_GOAL_ASSIGN_LATERAL_TOLERANCE):
            return dist_sq + lateral * 2.0

    if dist_sq <= max_dist_sq:
        return dist_sq + 32.0
    return None


def track_can_complete_any_global_goal(track):
    if track[O_GOAL_X] >= 0 and track[O_GOAL_Y] >= 0:
        return True
    axis, direction = recent_motion_axis_direction(track)
    if axis == AXIS_NONE or direction == 0:
        return False
    for goal in g_static_goals:
        if global_goal_track_score(track, goal) is not None:
            return True
    return False


def unresolved_goal_completion_boxes(tracks):
    out = []
    for track in tracks:
        if track[O_TYPE] != OBJ_TYPE_BOX:
            continue
        if track[O_STATE] == OBJ_STATE_REMOVED:
            continue
        if track[O_GOAL_RESULT] == 2:
            continue
        if GLOBAL_GOAL_CONFIRMATION_ONLY:
            if track_has_goal_candidate(track):
                out.append(track)
            continue
        if (track[O_MISSES] >= BOX_GOAL_MISSING_FRAMES or
                track_can_complete_any_global_goal(track)):
            out.append(track)
    return out


def track_has_goal_candidate(track):
    return track[O_GOAL_X] >= 0 and track[O_GOAL_Y] >= 0


def track_goal_candidate_matches(track, goal):
    return track[O_GOAL_X] == goal[0] and track[O_GOAL_Y] == goal[1]


def explicit_global_goal_assignment_allowed(track, goal):
    if not track_goal_candidate_matches(track, goal):
        return False
    if track_is_on_goal_candidate(track):
        return True
    gx = goal[0] + 0.5
    gy = goal[1] + 0.5
    dx = gx - track[O_X]
    dy = gy - track[O_Y]
    max_dist = GLOBAL_GOAL_EXPLICIT_ASSIGN_RADIUS
    return dx * dx + dy * dy <= max_dist * max_dist


def assign_goal_to_missing_box(missing, goal):
    # Global disappearance is confirmation-only: it may confirm an existing
    # box->goal candidate, but should not invent a pairing during fake moves.
    for track in missing:
        if explicit_global_goal_assignment_allowed(track, goal):
            return track

    if GLOBAL_GOAL_CONFIRMATION_ONLY:
        return None

    best = None
    best_score = 999999.0
    for track in missing:
        if track_has_goal_candidate(track):
            continue
        score = global_goal_track_score(track, goal)
        if score is None:
            continue
        if score < best_score:
            best_score = score
            best = track
    return best


def append_unique_goal(goals, goal):
    for item in goals:
        if item[0] == goal[0] and item[1] == goal[1]:
            return
    goals.append(goal)


def goal_in_list(goals, goal):
    for item in goals:
        if item[0] == goal[0] and item[1] == goal[1]:
            return True
    return False


def relevant_global_goals_for_candidates(candidates):
    goals = []
    for track in candidates:
        if track[O_GOAL_X] >= 0 and track[O_GOAL_Y] >= 0:
            for goal in g_static_goals:
                if goal[0] == track[O_GOAL_X] and goal[1] == track[O_GOAL_Y]:
                    append_unique_goal(goals, goal)
                    break
            continue
        for goal in g_static_goals:
            if global_goal_track_score(track, goal) is not None:
                append_unique_goal(goals, goal)
    return goals


def update_global_goal_disappearance(img, tracks):
    global g_goal_floor_counts, g_goal_last_trace
    if tracks is None or g_static_grid is None:
        return
    candidates = unresolved_goal_completion_boxes(tracks)
    if not candidates:
        return
    relevant_goals = relevant_global_goals_for_candidates(candidates)
    if not relevant_goals:
        return

    for goal in g_static_goals:
        if not goal_in_list(relevant_goals, goal):
            idx = grid_index(goal[0], goal[1])
            g_goal_floor_counts[idx] = 0
            g_goal_last_trace[idx] = 0

    confirmed = []
    for goal in relevant_goals:
        idx = grid_index(goal[0], goal[1])
        evidence = classify_goal_verification(img, goal[0], goal[1])
        if evidence == "-":
            if g_goal_floor_counts[idx] < 255:
                g_goal_floor_counts[idx] += 1
            if g_goal_last_trace[idx] != 1 and IDE_GOAL_TRACE:
                g_goal_last_trace[idx] = 1
                print("TRACE GLOBAL_GOAL goal=(%d,%d) evidence=FLOOR count=%d candidates=%d" %
                      (goal[0], goal[1], g_goal_floor_counts[idx], len(candidates)))
        else:
            g_goal_floor_counts[idx] = 0
            evidence_code = 2 if evidence == "." else 3
            if g_goal_last_trace[idx] != evidence_code and IDE_GOAL_TRACE:
                g_goal_last_trace[idx] = evidence_code
                evidence_name = "GOAL" if evidence == "." else "UNKNOWN"
                print("TRACE GLOBAL_GOAL goal=(%d,%d) evidence=%s candidates=%d" %
                      (goal[0], goal[1], evidence_name, len(candidates)))
        if g_goal_floor_counts[idx] >= GLOBAL_GOAL_VERIFY_FRAMES:
            confirmed.append(goal)

    for goal in confirmed:
        if not candidates:
            break
        track = assign_goal_to_missing_box(candidates, goal)
        if track is None:
            if IDE_GOAL_TRACE:
                print("TRACE GLOBAL_GOAL_NO_ASSIGN goal=(%d,%d) candidates=%d" %
                      (goal[0], goal[1], len(candidates)))
            g_goal_floor_counts[grid_index(goal[0], goal[1])] = 0
            continue
        track[O_GOAL_X] = goal[0]
        track[O_GOAL_Y] = goal[1]
        if IDE_GOAL_TRACE:
            print("TRACE GLOBAL_GOAL_ASSIGN box=%d goal=(%d,%d) pos=(%.2f,%.2f)" %
                  (track[O_ID], goal[0], goal[1], track[O_X], track[O_Y]))
        commit_box_goal_match(track)
        candidates.remove(track)
        g_goal_floor_counts[grid_index(goal[0], goal[1])] = 0


def compose_current_world(car):
    if g_static_grid is None:
        return None, [], [], []
    boxes = tracked_components(g_object_tracks, OBJ_TYPE_BOX)
    bombs = tracked_components(g_object_tracks, OBJ_TYPE_BOMB)
    grid, goals = compose_full_map_grid(
        g_static_grid, boxes, bombs, car)
    return grid, goals, boxes, bombs


def flush_visual_events(frame_seq, car):
    global g_event_queue
    if not g_event_queue:
        return
    grid, goals, boxes, bombs = compose_current_world(car)
    if grid is None:
        return
    while g_event_queue:
        event = g_event_queue.pop(0)
        if IDE_EVENT_DEBUG:
            print("EVENT name=%s type=%d object=%d cell=(%d,%d) map_version=%d" %
                  (event[0], event[1], event[2], event[3], event[4], event[5]))
            if IDE_EVENT_FULL_MAP:
                debug_print_full_map(
                    frame_seq, grid, goals, boxes, bombs, car)
        if event[6]:
            uart_write_all(make_full_map_binary(
                grid, goals, boxes, bombs, car))


# ============================================================
# 7. 车 / 目标检测（来自 识别代码.py，不变）
# ============================================================

def car_pose(front, back):
    if front is None or back is None:
        return None
    car_x = (front[0] + back[0]) / 2
    car_y = (front[1] + back[1]) / 2
    dx = front[0] - back[0]
    dy = front[1] - back[1]
    # 0° = 朝上（grid y 减小），90° = 朝右（grid x 增大）
    angle = math.atan2(dx, -dy) * 57.2957795
    if angle < 0:
        angle += 360
    return car_x, car_y, angle


def angle_diff_deg(a, b):
    d = a - b
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def smooth_car_pose(new_car, old_car):
    if not CAR_POSE_SMOOTHING or new_car is None or old_car is None:
        return new_car
    dx = new_car[0] - old_car[0]
    dy = new_car[1] - old_car[1]
    if math.sqrt(dx * dx + dy * dy) > CAR_POSE_SMOOTH_MAX_DIST:
        return new_car

    pos_a = CAR_POSE_SMOOTH_ALPHA
    x = old_car[0] + (new_car[0] - old_car[0]) * pos_a
    y = old_car[1] + (new_car[1] - old_car[1]) * pos_a

    theta_step = angle_diff_deg(new_car[2], old_car[2]) * CAR_THETA_SMOOTH_ALPHA
    if theta_step > CAR_THETA_MAX_STEP_DEG:
        theta_step = CAR_THETA_MAX_STEP_DEG
    elif theta_step < -CAR_THETA_MAX_STEP_DEG:
        theta_step = -CAR_THETA_MAX_STEP_DEG
    theta = old_car[2] + theta_step
    if theta < 0:
        theta += 360
    if theta >= 360:
        theta -= 360
    return x, y, theta


def marker_xy(comp):
    return comp[0], comp[1]


def marker_track_error(front, back, last_front, last_back):
    if last_front is None or last_back is None:
        return 999.0
    return dist_cells(marker_xy(front), last_front) + dist_cells(marker_xy(back), last_back)


def marker_component_touches_border(comp):
    return (comp[3] <= 0 or comp[4] <= 0 or
            comp[5] >= MAP_W - 1 or comp[6] >= MAP_H - 1)


def find_best_car_pair(fronts, backs, min_dist, max_dist):
    """在所有(车头,车尾)色块对中找距离最合理的一对，返回 car_pose 或 None

    替代原来的 largest() 分别取最大车头/车尾的方式。
    只有车头车尾空间距离在 [min_dist, max_dist] 范围内才认为有效。
    多对候选时选距离合理且与上一帧正反身份最连续的一对。
    """
    global g_last_front_marker, g_last_back_marker

    if not fronts or not backs:
        return None

    best_pair = None
    best_score = 999.0
    best_front = None
    best_back = None

    for f in fronts:
        if f[2] > MAX_CAR_MARKER_PIXELS:
            continue
        if marker_component_touches_border(f):
            continue
        fx, fy = f[0], f[1]
        for b in backs:
            if b[2] > MAX_CAR_MARKER_PIXELS:
                continue
            if marker_component_touches_border(b):
                continue
            bx, by = b[0], b[1]
            d = math.sqrt((fx - bx) ** 2 + (fy - by) ** 2)
            if not (min_dist <= d <= max_dist):
                continue

            track_error = 0.0
            if g_last_front_marker is not None and g_last_back_marker is not None:
                track_error = marker_track_error(
                    f, b, g_last_front_marker, g_last_back_marker)

            marker_pixels = f[2]
            if b[2] < marker_pixels:
                marker_pixels = b[2]
            if marker_pixels > 80:
                marker_pixels = 80
            score = (abs(d - CAR_MARKER_TARGET_DIST) +
                     track_error * 0.6 -
                     marker_pixels * CAR_MARKER_SIZE_SCORE_WEIGHT)
            if score < best_score:
                best_score = score
                best_pair = (f, b)
                best_front = marker_xy(f)
                best_back = marker_xy(b)

    if best_pair:
        g_last_front_marker = best_front
        g_last_back_marker = best_back
        return car_pose(best_pair[0], best_pair[1])
    return None


def rescue_car_near_backs(img, backs, print_debug=False):
    if not CAR_NEAR_BACK_RESCUE or img is None or not backs:
        return None

    best_front = None
    best_back = None
    best_score = -999
    best_rgb = (0, 0, 0)
    best_cyan = 0
    best_green = 0

    for b in backs:
        if b[2] > MAX_CAR_MARKER_PIXELS:
            continue

        bx = int(b[0] * SCALE)
        by = int(b[1] * SCALE)
        r = int(CAR_MARKER_MAX_DIST * SCALE)
        x0 = bx - r
        y0 = by - r
        x1 = bx + r
        y1 = by + r
        if x0 < 0: x0 = 0
        if y0 < 0: y0 = 0
        if x1 >= MAP_W: x1 = MAP_W - 1
        if y1 >= MAP_H: y1 = MAP_H - 1

        for sy in range(y0, y1 + 1, CAR_RESCUE_SAMPLE_STEP):
            fy = (sy + 0.5) / SCALE
            for sx in range(x0, x1 + 1, CAR_RESCUE_SAMPLE_STEP):
                fx = (sx + 0.5) / SCALE
                dx = fx - b[0]
                dy = fy - b[1]
                d = math.sqrt(dx * dx + dy * dy)
                if d < CAR_MARKER_MIN_DIST or d > CAR_MARKER_MAX_DIST:
                    continue
                u = (sx + 0.5) / MAP_W
                v = (sy + 0.5) / MAP_H
                px, py = screen_point_from_uv(u, v)
                rr, gg, bb = img.get_pixel(px, py)
                rr, gg, bb = apply_brightness_gain_q6(
                    rr, gg, bb, GAIN_MAP_Q6[sy * MAP_W + sx])
                cyan, green = car_marker_scores(rr, gg, bb)
                if cyan < CAR_RESCUE_CYAN_SCORE_MIN:
                    continue
                if cyan < green + CAR_RESCUE_SCORE_MARGIN:
                    continue

                # Prefer a strong cyan point close to a car-sized distance.
                score = (cyan - green -
                         int(abs(d - CAR_MARKER_TARGET_DIST) * 20))
                if score > best_score:
                    best_score = score
                    best_front = (fx, fy, 1, sx, sy, sx, sy)
                    best_back = b
                    best_rgb = (rr, gg, bb)
                    best_cyan = cyan
                    best_green = green

    if print_debug:
        if best_front is None:
            print("car_rescue: none")
        else:
            dx = best_front[0] - best_back[0]
            dy = best_front[1] - best_back[1]
            d = math.sqrt(dx * dx + dy * dy)
            print("car_rescue: front=(%.1f,%.1f) back=(%.1f,%.1f) d=%.2f rgb=(%d,%d,%d) score=(%d,%d)" %
                  (best_front[0], best_front[1], best_back[0], best_back[1],
                   d, best_rgb[0], best_rgb[1], best_rgb[2],
                   best_cyan, best_green))

    if best_front is not None:
        return car_pose(best_front, best_back)
    return None


def detect_goals_from_grid(grid):
    goals = []

    for row in range(1, ROWS - 1):
        for col in range(1, COLS - 1):
            if grid_get(grid, col, row) == ".":
                goals.append((col, row))
                if len(goals) >= MCU_MAX_BOXES:
                    return goals

    return goals


def detect_boxes_from_grid(grid):
    points = []
    for row in range(1, ROWS - 1):
        for col in range(1, COLS - 1):
            if grid_get(grid, col, row) != "$":
                continue

            merged = False
            for p in points:
                dx = p[0] - col
                dy = p[1] - row
                if dx < 0:
                    dx = -dx
                if dy < 0:
                    dy = -dy
                if dx <= 1 and dy <= 1:
                    merged = True
                    break

            if not merged:
                points.append((col, row))
                if len(points) >= MCU_MAX_BOXES:
                    return points
    return points


def component_covered_cell(col, row, comps):
    cell_x0 = col * SCALE
    cell_x1 = (col + 1) * SCALE - 1
    cell_y0 = row * SCALE
    cell_y1 = (row + 1) * SCALE - 1
    for comp in comps:
        min_x = comp[3] - COMP_DYNAMIC_COVER_PAD_SAMPLES
        min_y = comp[4] - COMP_DYNAMIC_COVER_PAD_SAMPLES
        max_x = comp[5] + COMP_DYNAMIC_COVER_PAD_SAMPLES
        max_y = comp[6] + COMP_DYNAMIC_COVER_PAD_SAMPLES
        if max_x >= cell_x0 and min_x <= cell_x1 and max_y >= cell_y0 and min_y <= cell_y1:
            return True
    return False


def car_covered_cell(col, row, car):
    if car is None:
        return False
    cx = col + 0.5
    cy = row + 0.5
    dx = cx - car[0]
    dy = cy - car[1]
    return dx * dx + dy * dy <= CAR_DYNAMIC_COVER_RADIUS_CELLS * CAR_DYNAMIC_COVER_RADIUS_CELLS


def classify_static_sample_rgb(r, g, b):
    if is_magenta_goal(r, g, b):
        return "."
    if wall_score_rgb(r, g, b) >= STATIC_WALL_SAMPLE_SCORE_MIN:
        return "#"
    return "-"


def static_sample_blocks_wall(r, g, b, wall_score):
    if is_magenta_goal(r, g, b):
        return True
    if is_yellow(r, g, b) or is_red_or_orange(r, g, b):
        return True
    if classify_car_marker_rgb(r, g, b) != CLS_NONE:
        return True
    if is_blue_floor(r, g, b) and wall_score < STATIC_WALL_STRONG_SAMPLE_SCORE_MIN:
        return True
    return False


def static_cell_stats(img, col, row):
    wall_votes = 0
    strong_wall_votes = 0
    goal_votes = 0
    strong_goal_votes = 0
    nonwall_votes = 0
    score_sum = 0
    score_max = 0
    sample_n = STATIC_MAP_BUILD_SAMPLES

    for sy in range(sample_n):
        sample_v = (sy + 0.5) / sample_n
        if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER:
            if row == 1:
                sample_v = (STATIC_INNER_BORDER_SAMPLE_INSET +
                            (1.0 - STATIC_INNER_BORDER_SAMPLE_INSET) * sample_v)
            elif row == ROWS - 2:
                sample_v = ((1.0 - STATIC_INNER_BORDER_SAMPLE_INSET) *
                            sample_v)
        v = (row + sample_v) / ROWS
        for sx in range(sample_n):
            sample_u = (sx + 0.5) / sample_n
            if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER:
                if col == 1:
                    sample_u = (STATIC_INNER_BORDER_SAMPLE_INSET +
                                (1.0 - STATIC_INNER_BORDER_SAMPLE_INSET) * sample_u)
                elif col == COLS - 2:
                    sample_u = ((1.0 - STATIC_INNER_BORDER_SAMPLE_INSET) *
                                sample_u)
            u = (col + sample_u) / COLS
            r, g, b = avg_rgb_at_uv(img, u, v)
            wall_score = wall_score_rgb(r, g, b)
            score_sum += wall_score
            if wall_score > score_max:
                score_max = wall_score
            blocks_wall = static_sample_blocks_wall(r, g, b, wall_score)
            if not blocks_wall and wall_score >= STATIC_WALL_SAMPLE_SCORE_MIN:
                wall_votes += 1
            if not blocks_wall and wall_score >= STATIC_WALL_STRONG_SAMPLE_SCORE_MIN:
                strong_wall_votes += 1
            if is_magenta_goal(r, g, b):
                goal_votes += 1
            if is_strong_magenta_goal_rgb(r, g, b):
                strong_goal_votes += 1
            if blocks_wall:
                nonwall_votes += 1

    if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW:
        center_r, center_g, center_b = avg_rgb_at_cell(img, col, row)
        if not is_strong_magenta_goal_rgb(center_r, center_g, center_b):
            strong_goal_votes = 0

    sample_count = sample_n * sample_n
    if sample_count <= 0:
        sample_count = 1
    return (wall_votes, strong_wall_votes, goal_votes, strong_goal_votes, nonwall_votes,
            score_sum // sample_count, score_max)


def static_cell_votes(img, col, row):
    wall_votes, strong_wall_votes, goal_votes, strong_goal_votes, nonwall_votes, score_avg, score_max = static_cell_stats(img, col, row)
    return wall_votes, goal_votes


def classify_static_cell_from_stats(col, row, wall_votes, strong_wall_votes,
                                    goal_votes, strong_goal_votes, nonwall_votes):
    if STATIC_FORCE_BORDER_WALLS:
        if row == 0 or row == ROWS - 1 or col == 0 or col == COLS - 1:
            return "#"

    if (goal_votes >= STATIC_GOAL_VOTE_MIN and
            strong_goal_votes >= STATIC_GOAL_STRONG_VOTE_MIN and
            wall_votes <= STATIC_GOAL_WALL_VOTE_MAX):
        return "."
    nonwall_wins = (
        wall_votes < STATIC_WALL_HIGH_VOTE_MIN and
        nonwall_votes >= wall_votes + STATIC_WALL_NONWALL_MARGIN)
    if wall_votes >= STATIC_WALL_VOTE_MIN and not nonwall_wins:
        return "#"
    return "-"


def classify_static_cell_from_votes(col, row, wall_votes, goal_votes):
    return classify_static_cell_from_stats(
        col, row, wall_votes, wall_votes, goal_votes, goal_votes, 0)


def classify_static_cell_vote(img, col, row):
    wall_votes, strong_wall_votes, goal_votes, strong_goal_votes, nonwall_votes, score_avg, score_max = static_cell_stats(img, col, row)
    return classify_static_cell_from_stats(
        col, row, wall_votes, strong_wall_votes, goal_votes,
        strong_goal_votes, nonwall_votes)


def static_neighbor_strong_wall(grid, wall_votes_map, col, row):
    if col < 0 or col >= COLS or row < 0 or row >= ROWS:
        return False
    if grid_get(grid, col, row) == "#":
        return True
    if grid_get(grid, col, row) == ".":
        return False
    return wall_votes_map[grid_index(col, row)] >= STATIC_WALL_NEIGHBOR_VOTE_MIN


def post_process_static_walls(grid, wall_votes_map, strong_wall_votes_map, nonwall_votes_map):
    suspects = 0
    applied = 0
    for row in range(1, ROWS - 1):
        for col in range(1, COLS - 1):
            idx = grid_index(col, row)
            if grid_get(grid, col, row) != "-":
                continue
            if wall_votes_map[idx] < STATIC_WALL_WEAK_VOTE_MIN:
                continue
            if nonwall_votes_map[idx] > STATIC_WALL_NONWALL_VOTE_MAX:
                continue

            support = 0
            left = static_neighbor_strong_wall(grid, wall_votes_map, col - 1, row)
            right = static_neighbor_strong_wall(grid, wall_votes_map, col + 1, row)
            up = static_neighbor_strong_wall(grid, wall_votes_map, col, row - 1)
            down = static_neighbor_strong_wall(grid, wall_votes_map, col, row + 1)
            if left:
                support += 1
            if right:
                support += 1
            if up:
                support += 1
            if down:
                support += 1
            bridge_gap = (left and right and not up and not down) or (up and down and not left and not right)

            if support >= STATIC_WALL_NEIGHBOR_MIN:
                suspects += 1
                can_apply = (STATIC_WALL_POST_APPLY and
                             strong_wall_votes_map[idx] >= STATIC_WALL_STRONG_VOTE_MIN and
                             not bridge_gap)
                if can_apply:
                    grid[idx] = ord("#")
                    applied += 1
                if (PRINT_DEBUG and STATIC_WALL_POST_DIAGNOSTIC and
                        suspects <= STATIC_WALL_POST_DEBUG_MAX):
                    print("STATIC_WALL_SUSPECT cell=(%d,%d) votes=%d strong=%d nonwall=%d support=%d bridge=%d apply=%d" %
                          (col, row, wall_votes_map[idx], strong_wall_votes_map[idx],
                           nonwall_votes_map[idx], support, 1 if bridge_gap else 0,
                           1 if can_apply else 0))

    if PRINT_DEBUG and suspects > 0:
        print("STATIC_WALL_POST suspects=%d applied=%d apply_enabled=%d" %
              (suspects, applied, 1 if STATIC_WALL_POST_APPLY else 0))


def grid_fill_cell_for_pos(col, row):
    if STATIC_FORCE_BORDER_WALLS:
        if row == 0 or row == ROWS - 1 or col == 0 or col == COLS - 1:
            return "#"
    return "-"


def grid_index(x, y):
    return y * COLS + x


def grid_get(grid, x, y):
    if grid is None:
        return "-"
    if x < 0 or x >= COLS or y < 0 or y >= ROWS:
        return "-"
    idx = grid_index(x, y)
    if idx < 0 or idx >= len(grid):
        return "-"
    val = grid[idx]
    if val == 35:
        return "#"
    if val == 46:
        return "."
    if val == 36:
        return "$"
    if val == 33:
        return "!"
    if val == 64:
        return "@"
    return "-"


def grid_row_text(grid, row):
    s = ""
    for col in range(COLS):
        s += grid_get(grid, col, row)
    return s


def debug_static_cells(img, grid):
    if not PRINT_DEBUG or not STATIC_CELL_DEBUG:
        return
    print("---- STATIC CELL DEBUG ----")
    for i in range(len(STATIC_DEBUG_CELLS)):
        col, row = STATIC_DEBUG_CELLS[i]
        wall_votes, strong_wall_votes, goal_votes, strong_goal_votes, nonwall_votes, score_avg, score_max = static_cell_stats(img, col, row)
        r, g, b = avg_rgb_at_cell(img, col, row)
        cell = grid_get(grid, col, row)
        print("  cell(%d,%d)=%s wall=%d strong=%d goal=%d goal_strong=%d nonwall=%d avg=%d max=%d rgb=(%d,%d,%d) wscore=%d mag=%d" %
              (col, row, cell, wall_votes, strong_wall_votes, goal_votes, strong_goal_votes,
               nonwall_votes, score_avg, score_max, r, g, b,
               wall_score_rgb(r, g, b), 1 if is_magenta_goal(r, g, b) else 0))


def make_empty_grid():
    grid = bytearray(ROWS * COLS)
    for row in range(ROWS):
        for col in range(COLS):
            grid[grid_index(col, row)] = ord(grid_fill_cell_for_pos(col, row))
    return grid


def build_dynamic_cell_mask(boxes, bombs, car):
    mask = bytearray(ROWS * COLS)

    groups = (boxes, bombs)
    for comps in groups:
        for comp in comps:
            x = clamp_cell_x(comp[0])
            y = clamp_cell_y(comp[1])
            mask[grid_index(x, y)] = 1

    if car is not None:
        x = clamp_cell_x(car[0])
        y = clamp_cell_y(car[1])
        mask[grid_index(x, y)] = 1
    return mask


def build_static_grid_high_quality(img, dynamic_mask=None):
    grid = bytearray(ROWS * COLS)
    verify_map = bytearray(ROWS * COLS)
    wall_frame_map = bytearray(ROWS * COLS)
    goal_frame_map = bytearray(ROWS * COLS)
    wall_votes_map = bytearray(ROWS * COLS)
    strong_wall_votes_map = bytearray(ROWS * COLS)
    nonwall_votes_map = bytearray(ROWS * COLS)
    verify_cells = 0

    for row in range(ROWS):
        for col in range(COLS):
            idx = grid_index(col, row)
            if (dynamic_mask is not None and dynamic_mask[idx] and
                    row != 0 and row != ROWS - 1 and
                    col != 0 and col != COLS - 1):
                grid[idx] = ord("-")
                continue

            wall_votes, strong_wall_votes, goal_votes, strong_goal_votes, nonwall_votes, score_avg, score_max = static_cell_stats(img, col, row)
            wall_votes_map[idx] = wall_votes
            strong_wall_votes_map[idx] = strong_wall_votes
            nonwall_votes_map[idx] = nonwall_votes
            cell = classify_static_cell_from_stats(
                col, row, wall_votes, strong_wall_votes, goal_votes,
                strong_goal_votes, nonwall_votes)
            grid[idx] = ord(cell)
            if cell == "#":
                wall_frame_map[idx] = 1
            elif cell == ".":
                goal_frame_map[idx] = 1

            if STATIC_FORCE_BORDER_WALLS:
                if row == 0 or row == ROWS - 1 or col == 0 or col == COLS - 1:
                    continue

            wall_near_threshold = (wall_votes >= STATIC_WALL_VERIFY_VOTE_LOW and
                                   wall_votes < STATIC_WALL_VERIFY_VOTE_HIGH)
            goal_needs_verify = (goal_votes > 0 or cell == ".")
            if wall_near_threshold or goal_needs_verify:
                verify_map[idx] = 1
                verify_cells += 1

    frame_count = 1
    if verify_cells > 0:
        for frame_i in range(STATIC_MAP_VERIFY_FRAMES):
            if STATIC_MAP_VERIFY_FRAME_DELAY_MS > 0:
                time.sleep_ms(STATIC_MAP_VERIFY_FRAME_DELAY_MS)
            frame_img = sensor.snapshot()
            frame_count += 1

            for row in range(1, ROWS - 1):
                for col in range(1, COLS - 1):
                    idx = grid_index(col, row)
                    if verify_map[idx] == 0:
                        continue
                    if dynamic_mask is not None and dynamic_mask[idx]:
                        continue
                    wall_votes, strong_wall_votes, goal_votes, strong_goal_votes, nonwall_votes, score_avg, score_max = static_cell_stats(frame_img, col, row)
                    wall_votes_map[idx] += wall_votes
                    strong_wall_votes_map[idx] += strong_wall_votes
                    nonwall_votes_map[idx] += nonwall_votes
                    cell = classify_static_cell_from_stats(
                        col, row, wall_votes, strong_wall_votes, goal_votes,
                        strong_goal_votes, nonwall_votes)
                    if cell == "#":
                        wall_frame_map[idx] += 1
                    elif cell == ".":
                        goal_frame_map[idx] += 1

        frame_vote_min = frame_count // 2 + 1
        for row in range(1, ROWS - 1):
            for col in range(1, COLS - 1):
                idx = grid_index(col, row)
                if verify_map[idx] == 0:
                    continue
                wall_votes_map[idx] = (wall_votes_map[idx] + frame_count // 2) // frame_count
                strong_wall_votes_map[idx] = (strong_wall_votes_map[idx] + frame_count // 2) // frame_count
                nonwall_votes_map[idx] = (nonwall_votes_map[idx] + frame_count // 2) // frame_count
                if goal_frame_map[idx] >= frame_vote_min:
                    grid[idx] = ord(".")
                elif wall_frame_map[idx] >= frame_vote_min:
                    grid[idx] = ord("#")
                else:
                    grid[idx] = ord("-")
    else:
        frame_vote_min = 1

    post_process_static_walls(grid, wall_votes_map, strong_wall_votes_map, nonwall_votes_map)
    goals = detect_goals_from_grid(grid)

    if PRINT_DEBUG:
        masked_cells = 0
        if dynamic_mask is not None:
            for value in dynamic_mask:
                if value:
                    masked_cells += 1
        print("STATIC_MAP_BUILT samples=%d verify_frames=%d verify_cells=%d masked=%d frame_vote=%d wall_vote=%d strong_vote=%d goal_vote=%d goal_strong=%d post_apply=%d" %
              (STATIC_MAP_BUILD_SAMPLES,
               frame_count - 1,
               verify_cells,
               masked_cells,
               frame_vote_min,
               STATIC_WALL_VOTE_MIN,
               STATIC_WALL_STRONG_VOTE_MIN,
               STATIC_GOAL_VOTE_MIN,
               STATIC_GOAL_STRONG_VOTE_MIN,
               1 if STATIC_WALL_POST_APPLY else 0))
        debug_static_cells(img, grid)
    return grid, goals


def copy_static_grid(grid):
    out = make_empty_grid()
    if grid is None:
        return out
    max_len = len(grid)
    if max_len > ROWS * COLS:
        max_len = ROWS * COLS
    for i in range(max_len):
        out[i] = grid[i]
    return out


def set_grid_cell_safe(grid, x, y, value):
    if grid is None:
        return
    if x < 0 or x >= COLS or y < 0 or y >= ROWS:
        return
    idx = grid_index(x, y)
    if idx < 0 or idx >= len(grid):
        return
    grid[idx] = ord(value)


def ensure_static_map(img, dynamic_mask=None):
    global g_static_grid
    if STATIC_MAP_LOCK and g_static_grid is None:
        grid, goals = build_static_grid_high_quality(img, dynamic_mask)
        initialize_static_layer(grid, goals)


def filter_bombs_on_static_floor(comps):
    if not STATIC_MAP_LOCK or g_static_grid is None:
        return comps
    out = []
    for comp in comps:
        x = clamp_cell_x(comp[0])
        y = clamp_cell_y(comp[1])
        if grid_get(g_static_grid, x, y) == "-":
            out.append(comp)
    return out


# ============================================================
# 7b. Static layer audit and explosion transactions
# ============================================================

def initialize_static_layer(grid, goals):
    global g_static_initial, g_static_grid, g_static_goals
    global g_static_candidate_class, g_static_candidate_count
    global g_map_version
    g_static_initial = copy_static_grid(grid)
    g_static_grid = copy_static_grid(grid)
    g_static_goals = list(goals)
    g_static_candidate_class = bytearray(ROWS * COLS)
    g_static_candidate_count = bytearray(ROWS * COLS)
    g_map_version = 1


def static_observation_is_strong(observed, stats):
    wall_votes = stats[0]
    strong_wall_votes = stats[1]
    goal_votes = stats[2]
    strong_goal_votes = stats[3]
    nonwall_votes = stats[4]
    if observed == "#":
        return (wall_votes >= STATIC_WALL_HIGH_VOTE_MIN and
                strong_wall_votes >= STATIC_WALL_STRONG_VOTE_MIN)
    if observed == ".":
        return (goal_votes >= STATIC_GOAL_VOTE_MIN and
                strong_goal_votes >= STATIC_GOAL_STRONG_VOTE_MIN)
    return (wall_votes <= STATIC_WALL_NONWALL_VOTE_MAX and
            goal_votes == 0 and
            nonwall_votes >= STATIC_WALL_HIGH_VOTE_MIN)


def audit_static_layer(img, dynamic_mask=None):
    if (not STATIC_AUDIT_ENABLED or g_static_grid is None or
            g_static_candidate_class is None):
        return 0
    confirmed_contradictions = 0
    for row in range(1, ROWS - 1):
        for col in range(1, COLS - 1):
            idx = grid_index(col, row)
            if dynamic_mask is not None and dynamic_mask[idx]:
                g_static_candidate_count[idx] = 0
                g_static_candidate_class[idx] = 0
                continue
            stats = static_cell_stats(img, col, row)
            observed = classify_static_cell_from_stats(
                col, row, stats[0], stats[1], stats[2], stats[3], stats[4])
            current = grid_get(g_static_grid, col, row)
            if observed == current or not static_observation_is_strong(
                    observed, stats):
                g_static_candidate_count[idx] = 0
                g_static_candidate_class[idx] = 0
                continue

            observed_code = ord(observed)
            if g_static_candidate_class[idx] == observed_code:
                if g_static_candidate_count[idx] < 255:
                    g_static_candidate_count[idx] += 1
            else:
                g_static_candidate_class[idx] = observed_code
                g_static_candidate_count[idx] = 1
            if (g_static_candidate_count[idx] >=
                    STATIC_AUDIT_CANDIDATE_FRAMES):
                confirmed_contradictions += 1

            # Normal audits deliberately do not rewrite known static cells.
            # WALL->FLOOR is committed only by a verified explosion event.
    return confirmed_contradictions


def dynamic_mask_from_tracks(tracks, car):
    boxes = tracked_components(tracks, OBJ_TYPE_BOX)
    bombs = tracked_components(tracks, OBJ_TYPE_BOMB)
    return build_dynamic_cell_mask(boxes, bombs, car)


def is_inner_map_cell(col, row):
    return (col > 0 and col < COLS - 1 and
            row > 0 and row < ROWS - 1)


def infer_explosion_center(track):
    if (track[O_TYPE] != OBJ_TYPE_BOMB or
            track[O_STATE] == OBJ_STATE_REMOVED or
            g_static_grid is None):
        return None
    if track[O_MISSES] < EXPLOSION_MISSING_FRAMES:
        return None

    col = clamp_cell_x(track[O_X])
    row = clamp_cell_y(track[O_Y])
    if is_inner_map_cell(col, row) and grid_get(g_static_grid, col, row) == "#":
        return (col, row)

    next_col = col
    next_row = row
    if track[O_AXIS] == AXIS_X and track[O_DIRECTION] != 0:
        next_col += track[O_DIRECTION]
    elif track[O_AXIS] == AXIS_Y and track[O_DIRECTION] != 0:
        next_row += track[O_DIRECTION]
    else:
        return None
    if (is_inner_map_cell(next_col, next_row) and
            grid_get(g_static_grid, next_col, next_row) == "#"):
        return (next_col, next_row)
    return None


def maybe_begin_explosion_transaction(tracks):
    global g_explosion_pending, g_pending_map
    if (not EXPLOSION_EVENTS_ENABLED or g_explosion_pending is not None or
            g_static_grid is None):
        return
    for track in tracks:
        if track[O_TYPE] != OBJ_TYPE_BOMB:
            continue
        center = infer_explosion_center(track)
        if center is None:
            continue
        track[O_STATE] = OBJ_STATE_EXPLOSION_PENDING
        track[O_OUTPUT] = OBJ_OUTPUT_OCCLUDED
        # [object_id, center_x, center_y, confirm_count, age]
        g_explosion_pending = [track[O_ID], center[0], center[1], 0, 0]
        g_pending_map = copy_static_grid(g_static_grid)
        for row in range(center[1] - 1, center[1] + 2):
            for col in range(center[0] - 1, center[0] + 2):
                if is_inner_map_cell(col, row):
                    set_grid_cell_safe(g_pending_map, col, row, "-")
        if PRINT_DEBUG:
            print("EXPLOSION_PENDING bomb=%d center=(%d,%d)" %
                  (track[O_ID], center[0], center[1]))
        queue_visual_event(
            "BOMB_EXPLOSION_PENDING", track, center[0], center[1], False)
        return


def find_track_by_id(tracks, object_type, object_id):
    for track in tracks:
        if track[O_TYPE] == object_type and track[O_ID] == object_id:
            return track
    return None


def explosion_verification_scores(img, center_x, center_y, dynamic_mask):
    inner_changed = 0
    inner_floor = 0
    outer_total = 0
    outer_match = 0

    for row in range(center_y - 2, center_y + 3):
        for col in range(center_x - 2, center_x + 3):
            if not is_inner_map_cell(col, row):
                continue
            idx = grid_index(col, row)
            if dynamic_mask is not None and dynamic_mask[idx]:
                continue
            dx = col - center_x
            if dx < 0:
                dx = -dx
            dy = row - center_y
            if dy < 0:
                dy = -dy
            stats = static_cell_stats(img, col, row)
            observed = classify_static_cell_from_stats(
                col, row, stats[0], stats[1], stats[2], stats[3], stats[4])
            current = grid_get(g_static_grid, col, row)

            if dx <= 1 and dy <= 1:
                if current == "#":
                    inner_changed += 1
                    if observed == "-" and static_observation_is_strong("-", stats):
                        inner_floor += 1
            else:
                outer_total += 1
                if observed == current:
                    outer_match += 1

    inner_ok = (inner_changed > 0 and
                inner_floor * 256 >=
                inner_changed * EXPLOSION_INNER_FLOOR_RATIO_Q8)
    outer_ok = (outer_total == 0 or
                outer_match * 256 >=
                outer_total * EXPLOSION_OUTER_MATCH_RATIO_Q8)
    return inner_ok, outer_ok, inner_floor, inner_changed, outer_match, outer_total


def commit_explosion_transaction(tracks):
    global g_explosion_pending, g_pending_map
    global g_map_version, g_static_goals
    if g_explosion_pending is None:
        return False
    object_id = g_explosion_pending[0]
    center_x = g_explosion_pending[1]
    center_y = g_explosion_pending[2]

    if g_pending_map is None:
        return False
    for i in range(len(g_pending_map)):
        g_static_grid[i] = g_pending_map[i]
    g_static_goals = detect_goals_from_grid(g_static_grid)
    g_map_version = (g_map_version + 1) & 0xFF
    if g_map_version == 0:
        g_map_version = 1

    track = find_track_by_id(tracks, OBJ_TYPE_BOMB, object_id)
    if track is not None:
        track[O_STATE] = OBJ_STATE_REMOVED
        track[O_OUTPUT] = OBJ_OUTPUT_COMPLETED
        track[O_VX] = 0.0
        track[O_VY] = 0.0
    if PRINT_DEBUG:
        print("EXPLOSION_COMMITTED bomb=%d center=(%d,%d) map_version=%d" %
              (object_id, center_x, center_y, g_map_version))
    queue_visual_event(
        "BOMB_EXPLOSION_COMMITTED", track, center_x, center_y, True)
    g_explosion_pending = None
    g_pending_map = None
    return True


def update_explosion_transaction(img, tracks, car):
    global g_explosion_pending, g_pending_map
    if not EXPLOSION_EVENTS_ENABLED:
        return False
    maybe_begin_explosion_transaction(tracks)
    if g_explosion_pending is None:
        return False

    pending_track = find_track_by_id(
        tracks, OBJ_TYPE_BOMB, g_explosion_pending[0])
    if (pending_track is None or
            pending_track[O_OUTPUT] == OBJ_OUTPUT_VALID or
            pending_track[O_MISSES] < EXPLOSION_MISSING_FRAMES):
        if pending_track is not None:
            pending_track[O_STATE] = OBJ_STATE_ACTIVE
        if PRINT_DEBUG:
            print("EXPLOSION_CANCEL visible_again bomb=%d" %
                  g_explosion_pending[0])
        queue_visual_event(
            "BOMB_EXPLOSION_CANCELLED", pending_track,
            g_explosion_pending[1], g_explosion_pending[2], False)
        g_explosion_pending = None
        g_pending_map = None
        return False

    g_explosion_pending[4] += 1
    dynamic_mask = dynamic_mask_from_tracks(tracks, car)
    dynamic_mask[grid_index(
        g_explosion_pending[1], g_explosion_pending[2])] = 0
    scores = explosion_verification_scores(
        img, g_explosion_pending[1], g_explosion_pending[2], dynamic_mask)
    if scores[0] and scores[1]:
        g_explosion_pending[3] += 1
    else:
        g_explosion_pending[3] = 0

    if PRINT_DEBUG and (scores[0] or g_explosion_pending[4] == 1):
        print("EXPLOSION_VERIFY inner=%d/%d outer=%d/%d confirm=%d age=%d" %
              (scores[2], scores[3], scores[4], scores[5],
               g_explosion_pending[3], g_explosion_pending[4]))

    if g_explosion_pending[3] >= EXPLOSION_VERIFY_FRAMES:
        return commit_explosion_transaction(tracks)
    if g_explosion_pending[4] >= EXPLOSION_PENDING_MAX_FRAMES:
        track = find_track_by_id(
            tracks, OBJ_TYPE_BOMB, g_explosion_pending[0])
        if track is not None and track[O_STATE] == OBJ_STATE_EXPLOSION_PENDING:
            track[O_STATE] = OBJ_STATE_OCCLUDED
        if PRINT_DEBUG:
            print("EXPLOSION_ROLLBACK bomb=%d" % g_explosion_pending[0])
        queue_visual_event(
            "BOMB_EXPLOSION_ROLLBACK", track,
            g_explosion_pending[1], g_explosion_pending[2], False)
        g_explosion_pending = None
        g_pending_map = None
    return False


# ============================================================
# 8. Pure output composition
# ============================================================

def build_live_static_grid(img, boxes, car):
    grid = make_empty_grid()
    for row in range(ROWS):
        for col in range(COLS):
            r, g, b = avg_rgb_at_cell(img, col, row)
            cell = classify_cell_rgb(r, g, b)
            if ENABLE_DYNAMIC_WALL_SUPPRESS and cell == "#":
                wall_score = wall_score_rgb(r, g, b)
                if (wall_score < DYNAMIC_WALL_SUPPRESS_SCORE_MAX and
                        car_covered_cell(col, row, car)):
                    cell = "-"
                elif (wall_score < DYNAMIC_WALL_SUPPRESS_SCORE_MAX and
                      component_covered_cell(col, row, boxes)):
                    cell = "-"
            set_grid_cell_safe(grid, col, row, cell)
    return grid


def compose_full_map_grid(static_grid, boxes, bombs, car):
    """Compose one output snapshot without mutating any source layer."""
    grid = copy_static_grid(static_grid)
    goals = detect_goals_from_grid(static_grid)

    for comp in bombs:
        x = clamp_cell_x(comp[0])
        y = clamp_cell_y(comp[1])
        set_grid_cell_safe(grid, x, y, "!")

    for comp in boxes:
        x = clamp_cell_x(comp[0])
        y = clamp_cell_y(comp[1])
        set_grid_cell_safe(grid, x, y, "$")

    if car is not None:
        x = clamp_cell_x(car[0])
        y = clamp_cell_y(car[1])
        set_grid_cell_safe(grid, x, y, "@")
    return grid, goals


# ============================================================
# 9. 调试绘制（来自 识别代码.py，不变）
# ============================================================

def draw_component(img, comp, color):
    x, y = screen_point_from_map(comp[0], comp[1])
    img.draw_cross(x, y, color=color, size=5)
    img.draw_circle(x, y, 4, color=color)


def draw_car(img, car):
    if car is None:
        return
    x, y = screen_point_from_map(car[0], car[1])
    img.draw_cross(x, y, color=(255, 255, 255), size=6)
    end_x = car[0] + math.sin(car[2] / 57.2957795) * 0.7
    end_y = car[1] - math.cos(car[2] / 57.2957795) * 0.7
    px, py = screen_point_from_map(end_x, end_y)
    img.draw_line(x, y, px, py, color=(255, 255, 255))


def draw_grid(img):
    img.draw_line(LT[0], LT[1], RT[0], RT[1], color=(0, 255, 0))
    img.draw_line(RT[0], RT[1], RB[0], RB[1], color=(0, 255, 0))
    img.draw_line(RB[0], RB[1], LB[0], LB[1], color=(0, 255, 0))
    img.draw_line(LB[0], LB[1], LT[0], LT[1], color=(0, 255, 0))
    col_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    col_end = COLS if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else COLS + 1
    row_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    row_end = ROWS if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else ROWS + 1
    for col in range(col_start, col_end):
        u = col / COLS
        x1, y1 = screen_point_from_uv(u, GEOMETRY_V_MIN)
        x2, y2 = screen_point_from_uv(u, GEOMETRY_V_MAX)
        img.draw_line(x1, y1, x2, y2, color=(0, 90, 0))
    for row in range(row_start, row_end):
        v = row / ROWS
        x1, y1 = screen_point_from_uv(GEOMETRY_U_MIN, v)
        x2, y2 = screen_point_from_uv(GEOMETRY_U_MAX, v)
        img.draw_line(x1, y1, x2, y2, color=(0, 90, 0))


def draw_grid_points(img, color=(0, 255, 0), size=2):
    """绘制所有网格交叉点（17×13 个点）

    格点即 (col, row) 的整数交叉处，共 (COLS+1)×(ROWS+1) 个。
    便于在 PC 端 IDE 上直观查看网格覆盖是否准确。
    """
    col_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    col_end = COLS if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else COLS + 1
    row_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    row_end = ROWS if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else ROWS + 1
    for col in range(col_start, col_end):
        u = col / COLS
        for row in range(row_start, row_end):
            v = row / ROWS
            x, y = screen_point_from_uv(u, v)
            img.draw_cross(x, y, color=color, size=size)


def draw_cell_centers(img, color=(255, 255, 0), size=2):
    col_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    col_end = COLS - 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else COLS
    row_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    row_end = ROWS - 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else ROWS
    for col in range(col_start, col_end):
        u = (col + 0.5) / COLS
        for row in range(row_start, row_end):
            v = (row + 0.5) / ROWS
            x, y = screen_point_from_uv(u, v)
            img.draw_cross(x, y, color=color, size=size)


# ============================================================
# 10. 调试打印（简化版）
# ============================================================

def debug_print_full_map(frame_id, grid, goals, boxes, bombs, car):
    print("==== FULL_MAP frame=%d ====" % frame_id)
    print("size=%dx%d" % (COLS, ROWS))
    for row in range(ROWS):
        print(grid_row_text(grid, row))
    print("boxes=%d" % len(boxes))
    for i in range(len(boxes)):
        b = boxes[i]
        print("  box%d: (%d,%d)" % (i, clamp_cell_x(b[0]), clamp_cell_y(b[1])))
    print("goals=%d" % len(goals))
    for i, g in enumerate(goals):
        print("  goal%d: (%d,%d)" % (i, g[0], g[1]))
    print("bombs=%d" % len(bombs))
    for i in range(len(bombs)):
        b = bombs[i]
        print("  bomb%d: (%d,%d)" % (i, clamp_cell_x(b[0]), clamp_cell_y(b[1])))
    if car is not None:
        print("car: grid=(%.1f,%.1f) mm=(%d,%d) theta=%.1f" %
              (car[0], car[1], grid_to_mm(car[0]), grid_to_mm(car[1]), car[2]))
    else:
        print("car: none")


def print_car_comp_debug(name, idx, comp, img):
    if img is None or not CAR_CANDIDATE_SCORE_DEBUG:
        print("  %s%d: grid=(%.1f,%.1f) pix=%d" %
              (name, idx, comp[0], comp[1], comp[2]))
        return
    r, g, b = avg_rgb_at_uv(img, comp[0] / COLS, comp[1] / ROWS)
    cyan, green = car_marker_scores(r, g, b)
    print("  %s%d: grid=(%.1f,%.1f) pix=%d rgb=(%d,%d,%d) score=(%d,%d)" %
          (name, idx, comp[0], comp[1], comp[2], r, g, b, cyan, green))


def debug_print_nearest_car_pairs(fronts, backs):
    if not fronts or not backs:
        return
    pairs = []
    for f in fronts:
        for b in backs:
            dx = f[0] - b[0]
            dy = f[1] - b[1]
            d = math.sqrt(dx * dx + dy * dy)
            fv = 1 if f[2] <= MAX_CAR_MARKER_PIXELS else 0
            bv = 1 if b[2] <= MAX_CAR_MARKER_PIXELS else 0
            pairs.append((d, f, b, fv, bv))
    pairs.sort(key=lambda item: item[0])
    max_n = CAR_PAIR_DEBUG_MAX if len(pairs) > CAR_PAIR_DEBUG_MAX else len(pairs)
    for i in range(max_n):
        d, f, b, fv, bv = pairs[i]
        print("  pair%d: d=%.2f fv=%d bv=%d f=(%.1f,%.1f,%d) b=(%.1f,%.1f,%d)" %
              (i, d, fv, bv, f[0], f[1], f[2], b[0], b[1], b[2]))


def debug_print_car_candidates(fronts, backs, img=None):
    valid_fronts = []
    valid_backs = []
    rejected_fronts = []
    rejected_backs = []
    for f in fronts:
        if f[2] <= MAX_CAR_MARKER_PIXELS:
            valid_fronts.append(f)
        else:
            rejected_fronts.append(f)
    for b in backs:
        if b[2] <= MAX_CAR_MARKER_PIXELS:
            valid_backs.append(b)
        else:
            rejected_backs.append(b)
    print("car_front_candidates=%d/%d car_back_candidates=%d/%d" %
          (len(valid_fronts), len(fronts), len(valid_backs), len(backs)))
    for i in range(len(valid_fronts) if len(valid_fronts) < 3 else 3):
        print_car_comp_debug("front", i, valid_fronts[i], img)
    for i in range(len(valid_backs) if len(valid_backs) < 3 else 3):
        print_car_comp_debug("back", i, valid_backs[i], img)
    for i in range(len(rejected_fronts) if len(rejected_fronts) < 2 else 2):
        print_car_comp_debug("front_rej", i, rejected_fronts[i], img)
    for i in range(len(rejected_backs) if len(rejected_backs) < 2 else 2):
        print_car_comp_debug("back_rej", i, rejected_backs[i], img)
    debug_print_nearest_car_pairs(fronts, backs)
    if g_last_front_marker is not None and g_last_back_marker is not None:
        dx = g_last_front_marker[0] - g_last_back_marker[0]
        dy = g_last_front_marker[1] - g_last_back_marker[1]
        print("  selected_markers front=(%.2f,%.2f) back=(%.2f,%.2f) d=%.2f" %
              (g_last_front_marker[0], g_last_front_marker[1],
               g_last_back_marker[0], g_last_back_marker[1],
               math.sqrt(dx * dx + dy * dy)))


# ============================================================
# 11. 主循环
# ============================================================

GAIN_MAP_Q6 = build_brightness_gain_map_q6()
if USE_PRECOMPUTED_SAMPLE_COORDS:
    SAMPLE_X_LO, SAMPLE_X_HI, SAMPLE_Y_MAP, SAMPLE_VISIBLE_MAP = (
        build_sample_coordinate_maps())
else:
    SAMPLE_X_LO = None
    SAMPLE_X_HI = None
    SAMPLE_Y_MAP = None
    SAMPLE_VISIBLE_MAP = None

if USE_REUSABLE_REGION_BUFFERS:
    DYNAMIC_CLASS_BUFFER = bytearray(MAP_W * MAP_H)
    DYNAMIC_VISITED_BUFFER = bytearray(MAP_W * MAP_H)
else:
    DYNAMIC_CLASS_BUFFER = None
    DYNAMIC_VISITED_BUFFER = None
DYNAMIC_VISIT_TOKEN = 0

if STARTUP_BANNER:
    print("openart integrated visual layer ready")
    print("geometry profile=%s" %
          ("INNER" if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else "OUTER"))
    print("calibration profile=%s" %
          ("NEW" if CALIBRATION_PROFILE == CALIBRATION_PROFILE_NEW else "OLD"))
    print("UART baud=%d  POS_UPDATE=%d ms  FULL_MAP on MAP_REQUEST" %
          (UART_BAUDRATE, POS_UPDATE_PERIOD_MS))

frame_id = 0
last_car = None
g_last_front_marker = None
g_last_back_marker = None
car_missing_frames = 0
last_pos_update_ms = 0
last_heartbeat_ms = 0
last_full_map_debug_ms = 0
heartbeat_seq = 100
g_full_map_requested = False
last_map_boxes = []
g_static_grid = None
g_static_initial = None
g_static_goals = []
g_static_candidate_class = None
g_static_candidate_count = None
g_map_version = 0
g_explosion_pending = None
g_pending_map = None
g_object_tracks = None
g_car_state = None
g_event_queue = []
g_goal_floor_counts = bytearray(ROWS * COLS)
g_goal_last_trace = bytearray(ROWS * COLS)
g_initial_full_map_pending = IDE_INITIAL_FULL_MAP
g_fps_window_start_ms = time.ticks_ms()
g_fps_window_frames = 0
g_perf_sum_ms = [0] * PERF_STAGE_COUNT
g_perf_max_ms = [0] * PERF_STAGE_COUNT
g_perf_count = [0] * PERF_STAGE_COUNT
g_perf_fallback_count = 0
g_perf_prediction_success_count = 0
g_perf_full_fallback_count = 0
g_perf_full_fallback_success_count = 0
g_perf_full_fallback_skip_count = 0
g_last_full_fallback_ms = None


while True:
    perf_loop_start_ms = time.ticks_ms()
    perf_stage_start_ms = perf_loop_start_ms
    img = sensor.snapshot()
    perf_record(
        PERF_SNAPSHOT,
        time.ticks_diff(time.ticks_ms(), perf_stage_start_ms))
    now_ms = time.ticks_ms()
    g_fps_window_frames += 1

    # ----- 11a. 监听 MCU 的 MAP_REQUEST -----
    poll_request()

    if time.ticks_diff(now_ms, last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS:
        last_heartbeat_ms = now_ms
        uart_write_all(make_heartbeat_binary(heartbeat_seq))
        heartbeat_seq = (heartbeat_seq + 1) & 0xFF

    # ----- 11b. 车 + 近距物体检测（区域搜索，快）-----
    if USE_SEPARATE_OBJECT_ROIS:
        search_radius = CAR_TRACK_RADIUS_CELLS
    else:
        search_radius = choose_fast_search_radius(last_car, g_object_tracks)
    region_elapsed_ms = 0
    component_elapsed_ms = 0

    perf_stage_start_ms = time.ticks_ms()
    region_map, rw, rh, ox, oy = build_region_class_map(img, last_car, search_radius)
    region_elapsed_ms += time.ticks_diff(time.ticks_ms(), perf_stage_start_ms)

    perf_stage_start_ms = time.ticks_ms()
    car_front, car_back, region_boxes, region_bomb_red, region_bomb_dark = (
        extract_dynamic_components_region(region_map, rw, rh, ox, oy))
    all_bombs = region_bomb_red + region_bomb_dark
    component_elapsed_ms += time.ticks_diff(time.ticks_ms(), perf_stage_start_ms)

    detected_car = find_best_car_pair(car_front, car_back,
                                       CAR_MARKER_MIN_DIST, CAR_MARKER_MAX_DIST)
    if detected_car is None:
        detected_car = rescue_car_near_backs(img, car_back, False)

    # 区域搜索丢失：有明确速度时先搜索预测位置，仍失败才全图回退。
    if detected_car is None and last_car is not None:
        perf_stage_start_ms = time.ticks_ms()
        prediction_center = predicted_car_search_center(g_car_state, now_ms)
        if prediction_center is not None:
            g_perf_fallback_count += 1
            (detected_car, car_front, car_back, region_boxes,
             region_bomb_red, region_bomb_dark) = (
                recover_car_and_region_objects(
                    img, prediction_center, CAR_TRACK_RADIUS_CELLS))
            if detected_car is not None:
                g_perf_prediction_success_count += 1

        if detected_car is None:
            fallback_now_ms = time.ticks_ms()
            allow_full_fallback = (
                g_last_full_fallback_ms is None or
                time.ticks_diff(
                    fallback_now_ms, g_last_full_fallback_ms) >=
                CAR_FULL_FALLBACK_RETRY_MS)
            if allow_full_fallback:
                g_perf_full_fallback_count += 1
                (detected_car, car_front, car_back, region_boxes,
                 region_bomb_red, region_bomb_dark) = (
                    recover_car_and_region_objects(img, None, search_radius))
                g_last_full_fallback_ms = time.ticks_ms()
                if detected_car is not None:
                    g_perf_full_fallback_success_count += 1
            else:
                g_perf_full_fallback_skip_count += 1

        all_bombs = region_bomb_red + region_bomb_dark
        perf_record(
            PERF_FALLBACK,
            time.ticks_diff(time.ticks_ms(), perf_stage_start_ms))

    if detected_car is not None:
        car = smooth_car_pose(detected_car, last_car)
        last_car = car
        car_missing_frames = 0
        g_last_full_fallback_ms = None
    else:
        car_missing_frames += 1
        if last_car is not None and car_missing_frames <= MAX_CAR_HOLD_FRAMES:
            car = last_car
        else:
            car = None
            g_last_front_marker = None
            g_last_back_marker = None
    g_car_state = make_car_state(
        car, g_car_state, now_ms, detected_car is not None)

    if (USE_SEPARATE_OBJECT_ROIS and LAYERED_STATE_ENABLED and
            g_object_tracks is not None and car is not None):
        (object_boxes, object_bombs, object_region_ms,
         object_component_ms) = collect_object_roi_observations(
            img, g_object_tracks, car, g_frame_seq)
        region_elapsed_ms += object_region_ms
        component_elapsed_ms += object_component_ms
        region_boxes = dedupe_components(region_boxes + object_boxes)
        all_bombs = dedupe_components(all_bombs + object_bombs)

    perf_record(PERF_REGION, region_elapsed_ms)
    perf_record(PERF_COMPONENTS, component_elapsed_ms)

    # ----- 11c. 近距物体过滤 -----
    perf_stage_start_ms = time.ticks_ms()
    near_boxes = []
    near_bombs = []
    if NEAR_OBJECT_TRACKING and car is not None:
        near_boxes = filter_near_components(region_boxes, car,
                                            NEAR_OBJECT_RADIUS_CELLS, 4)
        near_bombs = filter_near_components(all_bombs, car,
                                            NEAR_OBJECT_RADIUS_CELLS, 2)

    if LAYERED_STATE_ENABLED and g_object_tracks is not None:
        update_object_tracks(
            g_object_tracks, region_boxes, all_bombs,
            car, frame_id, False)
        update_box_goal_events(img, g_object_tracks, car)
        update_global_goal_disappearance(img, g_object_tracks)
        update_explosion_transaction(img, g_object_tracks, car)
        near_boxes = tracked_components(g_object_tracks, OBJ_TYPE_BOX)
        near_bombs = tracked_components(g_object_tracks, OBJ_TYPE_BOMB)
        flush_visual_events(frame_id, car)
    perf_record(
        PERF_TRACKING,
        time.ticks_diff(time.ticks_ms(), perf_stage_start_ms))

    # ----- 11d. 发送 POS_UPDATE（周期） -----
    if time.ticks_diff(now_ms, last_pos_update_ms) >= POS_UPDATE_PERIOD_MS:
        last_pos_update_ms = now_ms
        pos_boxes = near_boxes
        if (not LAYERED_STATE_ENABLED and
                len(last_map_boxes) > len(pos_boxes)):
            pos_boxes = last_map_boxes
        pkt = make_pos_update_binary(frame_id, car, pos_boxes, near_bombs)
        uart_write_all(pkt)

    # ----- 11e. 发送/打印 FULL_MAP（MCU 请求或周期调试） -----
    initial_full_map = g_initial_full_map_pending
    request_full_map = g_full_map_requested
    need_full_map = request_full_map or initial_full_map
    debug_full_map_only = False
    if (PERIODIC_FULL_MAP_DEBUG and
            time.ticks_diff(now_ms, last_full_map_debug_ms) >= PERIODIC_FULL_MAP_PERIOD_MS):
        last_full_map_debug_ms = now_ms
        if not need_full_map:
            need_full_map = True
            debug_full_map_only = True

    if need_full_map:
        perf_full_map_start_ms = time.ticks_ms()
        g_full_map_requested = False
        if initial_full_map:
            g_initial_full_map_pending = False

        # 全图扫描（较慢，但只偶发）
        cls_map = build_dynamic_class_map(img)

        boxes = top_n(sort_by_position(
            find_components(cls_map, CLS_BOX, MIN_BOX_PIXELS)), 4)
        bomb_red  = find_components(cls_map, CLS_BOMB_RED, MIN_BOMB_PIXELS,
                                    ignore_border=True)
        bomb_dark = find_components(cls_map, CLS_BOMB_DARK, MIN_BOMB_PIXELS,
                                    ignore_border=True)
        raw_bombs = bomb_red + bomb_dark

        if STATIC_MAP_LOCK and g_static_grid is None:
            dynamic_mask = build_dynamic_cell_mask(
                boxes, raw_bombs, car)
            ensure_static_map(img, dynamic_mask)

        initial_bombs = top_n(sort_by_position(
            filter_bombs_on_static_floor(raw_bombs)), VISION_MAX_BOMBS)

        debug_car_fronts = None
        debug_car_backs = None
        if PRINT_DEBUG and CAR_CANDIDATE_SCORE_DEBUG:
            debug_car_fronts = find_components(
                cls_map, CLS_CAR_FRONT, MIN_CAR_PIXELS)
            debug_car_backs = find_components(
                cls_map, CLS_CAR_BACK, MIN_CAR_PIXELS)

        # 全图扫描中如果车丢失，也尝试再检测一次
        if car is None:
            car_front2 = (debug_car_fronts if debug_car_fronts is not None else
                          find_components(cls_map, CLS_CAR_FRONT, MIN_CAR_PIXELS))
            car_back2 = (debug_car_backs if debug_car_backs is not None else
                         find_components(cls_map, CLS_CAR_BACK, MIN_CAR_PIXELS))
            detected_car2 = find_best_car_pair(car_front2, car_back2,
                                                CAR_MARKER_MIN_DIST, CAR_MARKER_MAX_DIST)
            if detected_car2 is None:
                detected_car2 = rescue_car_near_backs(img, car_back2, True)
            if detected_car2 is not None:
                car = smooth_car_pose(detected_car2, last_car)
                last_car = car
                car_missing_frames = 0

        if LAYERED_STATE_ENABLED:
            if g_object_tracks is None:
                g_object_tracks = initialize_object_tracks(
                    boxes, initial_bombs, frame_id)
                for track in g_object_tracks:
                    update_track_activation(track, car)
            else:
                update_object_tracks(
                    g_object_tracks, boxes, raw_bombs,
                    car, frame_id, True)

            boxes = tracked_components(g_object_tracks, OBJ_TYPE_BOX)
            bombs = tracked_components(g_object_tracks, OBJ_TYPE_BOMB)

            if STATIC_MAP_LOCK:
                static_base = g_static_grid
                audit_mask = dynamic_mask_from_tracks(
                    g_object_tracks, car)
                contradictions = audit_static_layer(img, audit_mask)
                if PRINT_DEBUG and contradictions > 0:
                    print("STATIC_AUDIT contradictions=%d map_version=%d" %
                          (contradictions, g_map_version))
            else:
                static_base = build_live_static_grid(img, boxes, car)
            grid, goals = compose_full_map_grid(
                static_base, boxes, bombs, car)
        else:
            bombs = initial_bombs
            if STATIC_MAP_LOCK:
                static_base = g_static_grid
            else:
                static_base = build_live_static_grid(img, boxes, car)
            grid, goals = compose_full_map_grid(
                static_base, boxes, bombs, car)

        if not STATIC_MAP_LOCK and not LAYERED_STATE_ENABLED:
            grid_boxes = detect_boxes_from_grid(grid)
            if len(grid_boxes) > len(boxes):
                boxes = grid_boxes
        last_map_boxes = boxes

        if request_full_map or PERIODIC_FULL_MAP_SEND_UART:
            pkt = make_full_map_binary(grid, goals, boxes, bombs, car)
            uart_write_all(pkt)

        if initial_full_map and IDE_EVENT_DEBUG:
            debug_print_full_map(frame_id, grid, goals, boxes, bombs, car)
        elif PRINT_DEBUG:
            if debug_full_map_only:
                print("---- PERIODIC FULL_MAP DEBUG ----")
            debug_print_full_map(frame_id, grid, goals, boxes, bombs, car)
            if LAYERED_STATE_ENABLED:
                print("layered map_version=%d pending_explosion=%d" %
                      (g_map_version,
                       1 if g_explosion_pending is not None else 0))
                debug_print_object_tracks(g_object_tracks)
            if debug_car_fronts is not None and debug_car_backs is not None:
                debug_print_car_candidates(
                    debug_car_fronts, debug_car_backs, img)
        perf_record(
            PERF_FULL_MAP,
            time.ticks_diff(time.ticks_ms(), perf_full_map_start_ms))

    # ----- 11f. 调试叠加 -----
    if DRAW_GRID_POINTS:
        draw_grid_points(img, color=(0, 255, 0), size=2)

    if DRAW_CELL_CENTERS:
        draw_cell_centers(img, color=(255, 255, 0), size=2)

    if DRAW_GRID_OVERLAY:
        draw_grid(img)

    if DRAW_DEBUG_OVERLAY:
        for comp in region_boxes:
            draw_component(img, comp, (255, 255, 0))
        for comp in region_bomb_red:
            draw_component(img, comp, (255, 0, 0))
        for comp in region_bomb_dark:
            draw_component(img, comp, (255, 255, 255))
        for comp in car_front:
            draw_component(img, comp, (0, 255, 255))
        for comp in car_back:
            draw_component(img, comp, (0, 255, 0))
        draw_car(img, car)

    perf_loop_end_ms = time.ticks_ms()
    perf_record(
        PERF_LOOP,
        time.ticks_diff(perf_loop_end_ms, perf_loop_start_ms))

    if (IDE_FPS_DEBUG and
            time.ticks_diff(perf_loop_end_ms, g_fps_window_start_ms) >=
            IDE_FPS_PERIOD_MS):
        elapsed_ms = time.ticks_diff(
            perf_loop_end_ms, g_fps_window_start_ms)
        fps_x10 = 0
        if elapsed_ms > 0:
            fps_x10 = (g_fps_window_frames * 10000 + elapsed_ms // 2) // elapsed_ms
        print("FPS avg=%d.%d frames=%d elapsed_ms=%d" %
              (fps_x10 // 10, fps_x10 % 10,
               g_fps_window_frames, elapsed_ms))
        if IDE_PERF_DEBUG:
            perf_print_window()
            perf_reset_window()
        g_fps_window_start_ms = perf_loop_end_ms
        g_fps_window_frames = 0

    frame_id = (frame_id + 1) & 0xFF
    time.sleep_ms(10)
