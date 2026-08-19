import sensor, image, time, math
from machine import UART

# =========================
# 固定参数
# =========================
GRID_W, GRID_H = 16, 12
FRAME_W, FRAME_H = 640, 480
DEBUG_BUILD_ID = 7100429

# 地图几何定位。当前 OUTER 标定点已经是 VGA 坐标，不再额外缩放。
GEOMETRY_SCALE = 1

def _pt(x, y):
    return (int(x * GEOMETRY_SCALE), int(y * GEOMETRY_SCALE))

LT = _pt(40, 25)
RT = _pt(598, 37)
RB = _pt(587, 430)
LB = _pt(33, 429)
TOP_EDGE_POINTS = (LT, _pt(180, 28), _pt(322, 31), _pt(460, 33), RT)
BOTTOM_EDGE_POINTS = (LB, _pt(172, 433), _pt(311, 433), _pt(449, 433), RB)
LEFT_EDGE_POINTS = (LT, _pt(38, 123), _pt(36, 226), _pt(35, 330), LB)
RIGHT_EDGE_POINTS = (RT, _pt(595, 130), _pt(593, 228), _pt(590, 329), RB)

GEOMETRY_PROFILE_OUTER = 0
GEOMETRY_PROFILE_INNER = 1
GEOMETRY_PROFILE = GEOMETRY_PROFILE_OUTER

INNER_LT = _pt(21, 29)
INNER_RT = _pt(288, 22)
INNER_RB = _pt(281, 219)
INNER_LB = _pt(21, 199)
INNER_TOP_EDGE_POINTS = (INNER_LT, _pt(80, 25), _pt(146, 23), _pt(218, 21), INNER_RT)
INNER_BOTTOM_EDGE_POINTS = (INNER_LB, _pt(78, 209), _pt(140, 212), _pt(216, 218), INNER_RB)
INNER_LEFT_EDGE_POINTS = (INNER_LT, _pt(19, 69), _pt(18, 118), _pt(19, 164), INNER_LB)
INNER_RIGHT_EDGE_POINTS = (INNER_RT, _pt(290, 71), _pt(286, 122), _pt(284, 174), INNER_RB)

if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER:
    LT, RT, RB, LB = INNER_LT, INNER_RT, INNER_RB, INNER_LB
    TOP_EDGE_POINTS = INNER_TOP_EDGE_POINTS
    BOTTOM_EDGE_POINTS = INNER_BOTTOM_EDGE_POINTS
    LEFT_EDGE_POINTS = INNER_LEFT_EDGE_POINTS
    RIGHT_EDGE_POINTS = INNER_RIGHT_EDGE_POINTS
    GEOMETRY_U_MIN = 1.0 / GRID_W
    GEOMETRY_U_MAX = (GRID_W - 1.0) / GRID_W
    GEOMETRY_V_MIN = 1.0 / GRID_H
    GEOMETRY_V_MAX = (GRID_H - 1.0) / GRID_H
else:
    GEOMETRY_U_MIN = 0.0
    GEOMETRY_U_MAX = 1.0
    GEOMETRY_V_MIN = 0.0
    GEOMETRY_V_MAX = 1.0

# 串口
UART_ID = 5
UART_BAUD = 115200
UART_CANDIDATES = (UART_ID, 12, 11)

VP_HEADER_0 = 0xA5
VP_HEADER_1 = 0x5A
VP_TYPE_FULL_MAP = 0x01
VP_TYPE_POS_UPDATE = 0x12  # v2: payload[0].bit7 is current-frame car_valid
VP_TYPE_MAP_REQUEST = 0x03

POS_UPDATE_PERIOD_MS = 25
FULL_MAP_REFRESH_PERIOD_MS = 5000
IDE_PERIODIC_MAP_DUMP_ENABLE = False
IDE_PERIODIC_MAP_DUMP_PERIOD_MS = 5000
MAX_BOXES = 4
MAX_GOALS = 4
MAX_BOMBS = 4
GRID_SIZE_MM = 200

# A freshly rebuilt map can classify the pixels hidden by the car as a wall.
# Only a car candidate continuous with the last live pose may override that
# interior-wall conflict; immutable boundary walls are never overridden.
CAR_WALL_OVERRIDE_ENABLE = True
CAR_WALL_OVERRIDE_CONTINUITY_X10 = 3

# LAB区间定义
# 每个物品对应一个或多个区间
# wall示例：需要同时满足两个L区间条件，且a、b在对应区间
# 格式: [(L_min, L_max), (a_min, a_max), (b_min, b_max)]
# PLAYER（地图）：16×12 格心建图时用 LAB_RANGES["PLAYER"]，可多段 OR。
# LAB_PLAYER_TRACK（追踪）：mode2 算车质心专用，可与地图不同（如两色车拆两条、避开黄；地图仍可用较宽单框把车格标出来）。
LAB_RANGES = {
    # 比赛屏幕四周明显变暗、中央略微过曝。各范围保留亮度余量，
    # 主要依靠a/b色度分离，避免暗蓝空地被误判成灰色墙体。
    "WALL": [
        {"L": (10, 90), "a": (-15, 20), "b": (-50, 15)}
    ],
    # 蓝色空地和紫色目标共用色域外框，再由a+b区分色相。
    "BLUE_FAMILY": [
        {"L": (10, 75), "a": (25, 115), "b": (-125, -30)}
    ],
    "BOX": [
        {"L": (25, 90), "a": (-45, -8), "b": (25, 100)}
    ],
    "BOMB": [
        {"L": (10, 75), "a": (20, 100), "b": (15, 85)}
    ],
    "PLAYER": [],
}

# a+b较大时颜色更偏紫红，判为目标；较小时偏蓝，判为空地。
BLUE_FAMILY_GOAL_AB_SPLIT = -20

LAB_PLAYER_TRACK = [
    # 车头：青色/蓝绿色
    {"L": (30, 80), "a": (-35, 2), "b": (-55, 4)},
    # 车尾：绿色
    {"L": (28, 80), "a": (-75, -18), "b": (5, 80)},
]

# =========================
# 显示控制
# =========================
SHOW_CELL_CODES = False
SHOW_FPS = False
# 统一测试开关
DEBUG_TRACK = False
DEBUG_PRINT_FPS = False
DEBUG_PRINT_MAP_ON_FULL = False
DEBUG_PRINT_CAR = True
DEBUG_PRINT_EVENTS = False
DEBUG_PROFILE_TIMING = False
DEBUG_FPS_PERIOD_MS = 1000
# =========================
# 地图采样策略（性能关键）
# =========================
SAMPLE_MODE = 2
SAMP_OFF = 1
_o = SAMP_OFF
# sample5 相对中心的 9 个偏移（与原先 pts 顺序一致）
_SAMPLE5_DXY = (
    (0, 0), (-_o, 0), (-_o, _o), (_o, 0), (_o, _o), (0, -_o), (-_o, -_o), (0, _o), (_o, -_o),
)
del _o
USE_TEMPORAL_FALLBACK = True
# 低延迟：mode2 不跑全图（地图仅在 mode1 刷新；需下位机接受「车位与地图非同帧」）
SKIP_GRID_ON_MODE2 = True
# 无请求时串口轮询休眠（微秒）；越小响应越快，0 为忙等
IDLE_UART_SLEEP_US = 100
# 编码
ENC = {"EMPTY":0, "WALL":1, "GOAL":2, "BOX":3, "BOMB":4, "PLAYER":5, "UNK":255}

def _lab_rng6(key):
    e = LAB_RANGES[key][0]
    return (e["L"][0], e["L"][1], e["a"][0], e["a"][1], e["b"][0], e["b"][1])

def _lab_rng6_entry(e):
    return (e["L"][0], e["L"][1], e["a"][0], e["a"][1], e["b"][0], e["b"][1])

_B_WALL = _lab_rng6("WALL")
_B_BLUE_FAMILY = _lab_rng6("BLUE_FAMILY")
_B_BOX = _lab_rng6("BOX")
_B_BOMB = _lab_rng6("BOMB")
_B_PLAYER_SEGS = tuple(_lab_rng6_entry(e) for e in LAB_RANGES["PLAYER"])
_B_PLAYER_TRACK_SEGS = tuple(_lab_rng6_entry(e) for e in LAB_PLAYER_TRACK)

# =========================
# PLAYER中心检测参数
# =========================
PLAYER_SCAN_STEP = 3
PLAYER_MIN_HIT = 40
# mode2 专用：步长大则采样少、延迟低（车体需足够大；丢车则略减小步长或降 PLAYER_MIN_HIT_MODE2）
PLAYER_SCAN_STEP_MODE2 = 4
PLAYER_MIN_HIT_MODE2 = 24
PLAYER_FAST_SCAN_STEP = 6
PLAYER_FAST_MIN_HIT = 6
PLAYER_FAST_RADIUS_PX = 42
PLAYER_FULL_SCAN_STEP = 8
PLAYER_FULL_MIN_HIT = 6
PLAYER_USE_BLOB_LOCAL_SEARCH = True
PLAYER_USE_BLOB_FULL_SEARCH = True
PLAYER_USE_PYTHON_LOCAL_SEARCH_FALLBACK = False
PLAYER_USE_PYTHON_FULL_SEARCH_FALLBACK = False
PLAYER_BLOB_X_STRIDE = 3
PLAYER_BLOB_Y_STRIDE = 3
PLAYER_BLOB_PIXELS_THRESHOLD = 8
PLAYER_BLOB_AREA_THRESHOLD = 8
PLAYER_BLOB_MAX_CANDIDATES = 8
PLAYER_PAIR_MIN_DIST_PX = 6
PLAYER_PAIR_MAX_DIST_PX = 48
PLAYER_PAIR_TARGET_DIST_PX = 15
USE_FAST_GRID_INVERSE = True
# mode2 未找到车时额外再采图识别次数（0=只试一次；2=首次失败后再拍 2 次，共最多 3 次）
PLAYER_FIND_EXTRA_SNAPSHOTS = 0
PLAYER_FULL_REACQUIRE_PERIOD_MS = 500
PLAYER_SEARCH_RING = 1   # 四周一圈 => 3x3格区域；如需更大可改2
# 车位：仅用 PLAYER 的 LAB 会把「与车颜色重叠的箱子」算进质心；改为走 classify 与地图一致（BOX 优先于 PLAYER）
# 为 True 时：不统计落在「上一帧地图为箱子」格内的采样点，减轻旁侧箱子拉偏（仅 mode1 会刷新 last_map 时可关 false 试效果）
PLAYER_SKIP_SAMPLES_IN_LASTMAP_BOX = False

TEXT_DX = 7
TEXT_DY = 10

# =========================
# 摄像头初始化
# =========================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_framerate(60)
sensor.set_auto_exposure(False, exposure_us=400)
sensor.skip_frames(time=300)


USE_WINDOWING = False

_CMM_LINES = (
    "hw,-,-,rt117x,seekfree_art_plus,",
    "uart,5,TXD,-,AD_28,",
    "uart,5,RXD,-,AD_29,",
    "uart,11,TXD,-,LPSR_04,",
    "uart,11,RXD,-,LPSR_05,",
    "uart,12,TXD,-,LPSR_06,",
    "uart,12,RXD,-,LPSR_07,",
)

_CMM_CFG = """fn,unit,signal,hint,pinobj,comments
hw,-,-,rt117x,seekfree_art_plus,
uart,5,TXD,-,AD_28,
uart,5,RXD,-,AD_29,
uart,11,TXD,-,LPSR_04,
uart,11,RXD,-,LPSR_05,
uart,12,TXD,-,LPSR_06,
uart,12,RXD,-,LPSR_07,
"""

def load_cmm_config():
    try:
        from machine import Pin
        import cmm
        cmap = {}
        for line in _CMM_LINES:
            parts = line.split(",")
            if parts[1] == "-":
                name = parts[0] + "." + parts[2]
            else:
                name = parts[0] + "." + parts[1] + "." + parts[2]
            try:
                pin = Pin(parts[4])
            except Exception:
                pin = None
            cmap[name] = (parts[3], parts[4], pin, None)
        cmm.add(cmap)
        print("cmm.add OK")
        return
    except Exception as e:
        print("direct cmm.add failed:", e)

    try:
        import cmm_load
        cmm_load.load()
        print("cmm_load OK")
        return
    except Exception as e:
        print("import cmm_load failed:", e)

for path in ("/cmm_cfg.csv", "cmm_cfg.csv", "/flash/cmm_cfg.csv"):
    try:
        with open(path, "w") as f:
            f.write(_CMM_CFG)
        print("WROTE", path)
    except Exception as e:
        print("SKIP", path, e)

load_cmm_config()

uarts = []
for uart_id in UART_CANDIDATES:
    try:
        u = UART(uart_id, UART_BAUD)
        uarts.append(u)
        print("UART%d OK" % uart_id)
    except Exception as e:
        print("UART%d NO: %s" % (uart_id, e))

if not uarts:
    raise Exception("No UART opened - check cmm config and wiring")

print("OPENART oa_id=%d LINK_READY" % DEBUG_BUILD_ID)

def uart_write_all(pkt):
    for u in uarts:
        try:
            u.write(pkt)
        except Exception as e:
            print("UART write failed:", e)

clock = time.clock()

# =========================
# 工具函数
# =========================
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def lerp(a, b, t):
    return a + (b - a) * t

def lerp_point(a, b, t):
    return lerp(a[0], b[0], t), lerp(a[1], b[1], t)

def edge_point(edge_points, t):
    if t <= 0.0:
        return edge_points[0]
    if t >= 1.0:
        return edge_points[4]
    s = t * 4.0
    i = int(s)
    if i >= 4:
        return edge_points[4]
    return lerp_point(edge_points[i], edge_points[i + 1], s - i)

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

def screen_point_from_uv(u, v):
    u, v = geometry_local_uv(u, v)
    x, y = coons_point_from_uv(u, v)
    max_x = FRAME_W - 1
    max_y = FRAME_H - 1
    if x < 0:
        x = 0
    elif x > max_x:
        x = max_x
    if y < 0:
        y = 0
    elif y > max_y:
        y = max_y
    return int(x), int(y)

def screen_point_from_map(mx, my):
    return screen_point_from_uv(mx / GRID_W, my / GRID_H)

def build_grid_points_geometry():
    pts = []
    for gy in range(GRID_H):
        for gx in range(GRID_W):
            pts.append(screen_point_from_map(gx + 0.5, gy + 0.5))
    return pts

def geometry_bounds():
    xs = []
    ys = []
    for gy in range(GRID_H + 1):
        for gx in range(GRID_W + 1):
            x, y = screen_point_from_map(gx, gy)
            xs.append(x)
            ys.append(y)
    x1 = max(0, min(xs))
    y1 = max(0, min(ys))
    x2 = min(FRAME_W - 1, max(xs))
    y2 = min(FRAME_H - 1, max(ys))
    return x1, y1, x2 - x1 + 1, y2 - y1 + 1

def cell_region_bounds(gx0, gx1, gy0, gy1):
    xs = []
    ys = []
    for gy in range(gy0, gy1 + 2):
        for gx in range(gx0, gx1 + 2):
            x, y = screen_point_from_map(gx, gy)
            xs.append(x)
            ys.append(y)
    pad = 3
    x0 = clamp(min(xs) - pad, 0, FRAME_W - 1)
    y0 = clamp(min(ys) - pad, 0, FRAME_H - 1)
    x1 = clamp(max(xs) + pad, 0, FRAME_W - 1)
    y1 = clamp(max(ys) + pad, 0, FRAME_H - 1)
    return x0, y0, x1 + 1, y1 + 1

def build_grid_lines_rect(x1, y1, w, h, gw, gh):
    x_lines = [int(round(x1 + i * w / gw)) for i in range(gw + 1)]
    y_lines = [int(round(y1 + i * h / gh)) for i in range(gh + 1)]
    return x_lines, y_lines

def build_grid_points_rect(x_lines, y_lines, use_windowing=False):
    pts = []
    for gy in range(GRID_H):
        for gx in range(GRID_W):
            cx = (x_lines[gx] + x_lines[gx + 1]) // 2
            cy = (y_lines[gy] + y_lines[gy + 1]) // 2

            if use_windowing:
                cx -= ROI_X1
                cy -= ROI_Y1

            pts.append((cx, cy))
    return pts

def sample1_lab(img, x, y):
    """
    单点采样并返回 LAB 值
    """
    r, g, b = img.get_pixel(x, y)
    L_val, A_val, B_val = rgb_to_lab(r, g, b)
    return L_val, A_val, B_val

def sample5_lab(img, x, y):
    w = img.width()
    h = img.height()
    wm1, hm1 = w - 1, h - 1
    sum_L = 0
    sum_a = 0
    sum_b = 0
    for dx, dy in _SAMPLE5_DXY:
        px = x + dx
        if px < 0:
            px = 0
        elif px > wm1:
            px = wm1
        py = y + dy
        if py < 0:
            py = 0
        elif py > hm1:
            py = hm1
        r, g, b = img.get_pixel(px, py)
        L, a, b_ = rgb_to_lab(r, g, b)
        sum_L += L
        sum_a += a
        sum_b += b_
    return sum_L // 9, sum_a // 9, sum_b // 9

def gray_score(r, g, b):
    avg = (r + g + b) // 3
    return abs(r - avg) + abs(g - avg) + abs(b - avg)

def l1_dist(r, g, b, p):
    return abs(r - p[0]) + abs(g - p[1]) + abs(b - p[2])

def _lab_xfer_f(t):
    return t ** (1/3) if t > 0.008856 else 7.787 * t + 16/116

def rgb_to_lab(r, g, b):
    # RGB -> XYZ
    r = r / 255.0
    g = g / 255.0
    b = b / 255.0

    r = r / 12.92 if r <= 0.04045 else ((r + 0.055) / 1.055) ** 2.4
    g = g / 12.92 if g <= 0.04045 else ((g + 0.055) / 1.055) ** 2.4
    b = b / 12.92 if b <= 0.04045 else ((b + 0.055) / 1.055) ** 2.4

    x = r * 0.4124 + g * 0.3576 + b * 0.1805
    y = r * 0.2126 + g * 0.7152 + b * 0.0722
    z = r * 0.0193 + g * 0.1192 + b * 0.9505

    # XYZ -> LAB
    x /= 0.95047
    y /= 1.0000
    z /= 1.08883

    fx = _lab_xfer_f(x)
    fy = _lab_xfer_f(y)
    fz = _lab_xfer_f(z)

    L = 116 * fy - 16
    a = 500 * (fx - fy)
    b = 200 * (fy - fz)

    return L, a, b

def classify_lab_code_with_player_segs(L_val, A_val, B_val, player_segs):
    """与 classify_lab_code 相同，但最后 PLAYER 判定使用传入的 player_segs（地图 / 追踪可分开）。"""
    lo, hi, alo, ahi, blo, bhi = _B_WALL
    if lo <= L_val <= hi and alo <= A_val <= ahi and blo <= B_val <= bhi:
        return 1
    lo, hi, alo, ahi, blo, bhi = _B_BOX
    if lo <= L_val <= hi and alo <= A_val <= ahi and blo <= B_val <= bhi:
        return 3
    lo, hi, alo, ahi, blo, bhi = _B_BOMB
    if lo <= L_val <= hi and alo <= A_val <= ahi and blo <= B_val <= bhi:
        return 4
    lo, hi, alo, ahi, blo, bhi = _B_BLUE_FAMILY
    if lo <= L_val <= hi and alo <= A_val <= ahi and blo <= B_val <= bhi:
        return 2 if A_val + B_val >= BLUE_FAMILY_GOAL_AB_SPLIT else 0
    for lo, hi, alo, ahi, blo, bhi in player_segs:
        if lo <= L_val <= hi and alo <= A_val <= ahi and blo <= B_val <= bhi:
            return 5
    return 255

def classify_lab_code(L_val, A_val, B_val):
    """地图格点分类：PLAYER 使用 LAB_RANGES["PLAYER"]。"""
    return classify_lab_code_with_player_segs(L_val, A_val, B_val, _B_PLAYER_SEGS)

rx_buf = bytearray()
full_map_requested = False

def xor_checksum(data):
    cs = 0
    for b in data:
        cs ^= b
    return cs & 0xFF

def poll_map_request():
    """MCU -> OpenART: [A5][5A][03][reason][xor first 4 bytes]."""
    global rx_buf, full_map_requested

    for u in uarts:
        try:
            n = u.any()
            if n:
                data = u.read(n)
                if data:
                    rx_buf.extend(data)
        except Exception:
            pass

    while len(rx_buf) >= 5:
        if rx_buf[0] != VP_HEADER_0 or rx_buf[1] != VP_HEADER_1:
            rx_buf = bytearray(rx_buf[1:])
            continue

        pkt = rx_buf[:5]
        if pkt[2] == VP_TYPE_MAP_REQUEST and xor_checksum(pkt[:4]) == pkt[4]:
            full_map_requested = True
            rx_buf = bytearray(rx_buf[5:])
        else:
            rx_buf = bytearray(rx_buf[1:])

    if len(rx_buf) > 32:
        rx_buf = bytearray(rx_buf[-16:])

def _track_seg_index(L_val, A_val, B_val):
    for i, (lo, hi, alo, ahi, blo, bhi) in enumerate(_B_PLAYER_TRACK_SEGS):
        if lo <= L_val <= hi and alo <= A_val <= ahi and blo <= B_val <= bhi:
            return i
    return -1

def find_player_in_rect(img, px0, py0, px1, py1, scan_step=None, min_hit=None):
    global debug_search_px0, debug_search_py0, debug_search_px1, debug_search_py1

    if scan_step is None:
        scan_step = PLAYER_SCAN_STEP
    if min_hit is None:
        min_hit = PLAYER_MIN_HIT

    px0 = clamp(px0, 0, img.width() - 1)
    px1 = clamp(px1, 0, img.width())
    py0 = clamp(py0, 0, img.height() - 1)
    py1 = clamp(py1, 0, img.height())

    debug_search_px0 = px0
    debug_search_py0 = py0
    debug_search_px1 = px1
    debug_search_py1 = py1

    cnt = 0
    sum_x = 0
    sum_y = 0
    seg_cnt = [0, 0]
    seg_sum_x = [0, 0]
    seg_sum_y = [0, 0]

    skip_box = PLAYER_SKIP_SAMPLES_IN_LASTMAP_BOX
    box_code = ENC["BOX"]

    for y in range(py0, py1, scan_step):
        for x in range(px0, px1, scan_step):
            if skip_box:
                gi = _grid_idx_at_full_pixel(x, y)
                if gi >= 0 and last_map[gi] == box_code:
                    continue
            L_val, A_val, B_val = sample1_lab(img, x, y)
            si = _track_seg_index(L_val, A_val, B_val)
            if si < 0:
                continue
            sum_x += x
            sum_y += y
            cnt += 1
            if si < 2:
                seg_cnt[si] += 1
                seg_sum_x[si] += x
                seg_sum_y[si] += y

    if cnt < min_hit:
        return None

    cx = sum_x // cnt
    cy = sum_y // cnt
    hx = hy = tx = ty = -1
    if seg_cnt[0] > 0:
        hx = seg_sum_x[0] // seg_cnt[0]
        hy = seg_sum_y[0] // seg_cnt[0]
    if seg_cnt[1] > 0:
        tx = seg_sum_x[1] // seg_cnt[1]
        ty = seg_sum_y[1] // seg_cnt[1]
    return (cx, cy, hx, hy, tx, ty)

def find_player(img, req_gx, req_gy, scan_step=None, min_hit=None):
    gx0 = clamp(req_gx - PLAYER_SEARCH_RING, 0, GRID_W - 1)
    gx1 = clamp(req_gx + PLAYER_SEARCH_RING, 0, GRID_W - 1)
    gy0 = clamp(req_gy - PLAYER_SEARCH_RING, 0, GRID_H - 1)
    gy1 = clamp(req_gy + PLAYER_SEARCH_RING, 0, GRID_H - 1)
    px0, py0, px1, py1 = cell_region_bounds(gx0, gx1, gy0, gy1)
    if PLAYER_USE_BLOB_LOCAL_SEARCH:
        pos = find_player_blobs_in_rect(img, px0, py0, px1, py1)
        if pos is not None:
            return pos
        if not PLAYER_USE_PYTHON_LOCAL_SEARCH_FALLBACK:
            return None
    return find_player_in_rect(img, px0, py0, px1, py1, scan_step, min_hit)

def find_player_near_pixel(img, cx, cy):
    r = PLAYER_FAST_RADIUS_PX
    px0 = int(cx) - r
    py0 = int(cy) - r
    px1 = int(cx) + r + 1
    py1 = int(cy) + r + 1
    if PLAYER_USE_BLOB_LOCAL_SEARCH:
        pos = find_player_blobs_in_rect(img, px0, py0, px1, py1)
        if pos is not None:
            return pos
        if not PLAYER_USE_PYTHON_LOCAL_SEARCH_FALLBACK:
            return None
    return find_player_in_rect(
        img,
        px0,
        py0,
        px1,
        py1,
        PLAYER_FAST_SCAN_STEP,
        PLAYER_FAST_MIN_HIT,
    )

def find_player_blobs_in_rect(img, px0, py0, px1, py1):
    px0 = clamp(px0, 0, img.width() - 1)
    px1 = clamp(px1, 0, img.width())
    py0 = clamp(py0, 0, img.height() - 1)
    py1 = clamp(py1, 0, img.height())
    if px1 <= px0 or py1 <= py0:
        return None

    candidates = [[], []]
    roi = (px0, py0, px1 - px0, py1 - py0)

    try:
        for si, th in enumerate(_B_PLAYER_TRACK_SEGS):
            blobs = img.find_blobs(
                [th],
                roi=roi,
                x_stride=PLAYER_BLOB_X_STRIDE,
                y_stride=PLAYER_BLOB_Y_STRIDE,
                pixels_threshold=PLAYER_BLOB_PIXELS_THRESHOLD,
                area_threshold=PLAYER_BLOB_AREA_THRESHOLD,
                merge=True,
            )
            for blob in blobs:
                pixels = blob.pixels()
                item = (pixels, blob.cx(), blob.cy())
                top = candidates[si]
                insert_at = 0
                while insert_at < len(top) and top[insert_at][0] >= pixels:
                    insert_at += 1
                top.insert(insert_at, item)
                if len(top) > PLAYER_BLOB_MAX_CANDIDATES:
                    top.pop()
    except Exception as e:
        if DEBUG_PRINT_EVENTS:
            print("EVENT CAR_BLOB_FAIL err=%s" % str(e))
        return None

    if not candidates[0] or not candidates[1]:
        return None

    min_d2 = PLAYER_PAIR_MIN_DIST_PX * PLAYER_PAIR_MIN_DIST_PX
    max_d2 = PLAYER_PAIR_MAX_DIST_PX * PLAYER_PAIR_MAX_DIST_PX
    target_d2 = PLAYER_PAIR_TARGET_DIST_PX * PLAYER_PAIR_TARGET_DIST_PX
    best = None
    best_score = None
    for head in candidates[0]:
        for tail in candidates[1]:
            dx = head[1] - tail[1]
            dy = head[2] - tail[2]
            d2 = dx * dx + dy * dy
            if d2 < min_d2 or d2 > max_d2:
                continue

            # 必须同时存在相邻的青色车头和绿色车尾；面积平衡可避免墙体拖走质心。
            min_pixels = head[0] if head[0] < tail[0] else tail[0]
            score = min_pixels * 10000 - abs(head[0] - tail[0]) * 100 - abs(d2 - target_d2)
            if best_score is None or score > best_score:
                best_score = score
                best = (head, tail)

    if best is None:
        return None

    head, tail = best
    total = head[0] + tail[0]
    cx = (head[1] * head[0] + tail[1] * tail[0]) // total
    cy = (head[2] * head[0] + tail[2] * tail[0]) // total
    return (cx, cy, head[1], head[2], tail[1], tail[2])

def split_3digits(v):
    # 未找到时发 999
    if v < 0:
        return 9, 9, 9

    if v > 999:
        v = 999

    b = v // 100
    s = (v // 10) % 10
    g = v % 10
    return b, s, g

# =========================
# 预计算比例网格
# =========================
ROI_X1, ROI_Y1, ROI_W, ROI_H = geometry_bounds()
ROI_X2 = ROI_X1 + ROI_W
ROI_Y2 = ROI_Y1 + ROI_H
ROI = (ROI_X1, ROI_Y1, ROI_W, ROI_H)
points = build_grid_points_geometry()

def build_grid_inverse_cache():
    cache = []
    for gy in range(GRID_H):
        for gx in range(GRID_W):
            idx = gy * GRID_W + gx
            cx, cy = points[idx]

            if gx <= 0:
                lx, ly = points[idx]
                rx, ry = points[idx + 1]
                ex_x = rx - lx
                ex_y = ry - ly
            elif gx >= GRID_W - 1:
                lx, ly = points[idx - 1]
                rx, ry = points[idx]
                ex_x = rx - lx
                ex_y = ry - ly
            else:
                lx, ly = points[idx - 1]
                rx, ry = points[idx + 1]
                ex_x = (rx - lx) * 0.5
                ex_y = (ry - ly) * 0.5

            if gy <= 0:
                tx, ty = points[idx]
                bx, by = points[idx + GRID_W]
                ey_x = bx - tx
                ey_y = by - ty
            elif gy >= GRID_H - 1:
                tx, ty = points[idx - GRID_W]
                bx, by = points[idx]
                ey_x = bx - tx
                ey_y = by - ty
            else:
                tx, ty = points[idx - GRID_W]
                bx, by = points[idx + GRID_W]
                ey_x = (bx - tx) * 0.5
                ey_y = (by - ty) * 0.5

            det = ex_x * ey_y - ey_x * ex_y
            if det == 0:
                inv = 0.0
            else:
                inv = 1.0 / det
            cache.append((cx, cy, gx + 0.5, gy + 0.5, ex_x, ex_y, ey_x, ey_y, inv))
    return cache

GRID_INV_CACHE = build_grid_inverse_cache()

last_map = bytearray([ENC["EMPTY"]] * (GRID_W * GRID_H))

def _grid_idx_at_full_pixel(ax_full, ay_full):
    """整图坐标落在哪一格；越界返回 -1。"""
    gx, gy = pixel_to_grid(ax_full, ay_full)
    if gx < 0 or gy < 0:
        return -1
    return gy * GRID_W + gx

def pixel_to_grid_float(px, py):
    best_mx = 0.5
    best_my = 0.5
    best_d2 = 1 << 30
    for gy in range(GRID_H):
        for gx in range(GRID_W):
            x, y = points[gy * GRID_W + gx]
            dx = x - px
            dy = y - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_mx = gx + 0.5
                best_my = gy + 0.5

    step = 0.5
    for _ in range(4):
        base_x = best_mx
        base_y = best_my
        for iy in range(-2, 3):
            my = base_y + iy * step
            if my < 0.0:
                my = 0.0
            elif my > GRID_H:
                my = GRID_H
            for ix in range(-2, 3):
                mx = base_x + ix * step
                if mx < 0.0:
                    mx = 0.0
                elif mx > GRID_W:
                    mx = GRID_W
                x, y = screen_point_from_map(mx, my)
                dx = x - px
                dy = y - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_mx = mx
                    best_my = my
        step *= 0.5
    return best_mx, best_my

def pixel_to_grid_float_fast(px, py):
    best = GRID_INV_CACHE[0]
    best_d2 = 1 << 30
    for item in GRID_INV_CACHE:
        cx, cy = item[0], item[1]
        dx = cx - px
        dy = cy - py
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best = item

    cx, cy, mx0, my0, ex_x, ex_y, ey_x, ey_y, inv = best
    if inv == 0.0:
        return mx0, my0

    dx = px - cx
    dy = py - cy
    du = (dx * ey_y - ey_x * dy) * inv
    dv = (ex_x * dy - dx * ex_y) * inv
    mx = mx0 + du
    my = my0 + dv
    if mx < 0.0:
        mx = 0.0
    elif mx > GRID_W:
        mx = GRID_W
    if my < 0.0:
        my = 0.0
    elif my > GRID_H:
        my = GRID_H
    return mx, my

def pixel_to_grid(px, py):
    if px < ROI_X1 - 4 or px > ROI_X2 + 4 or py < ROI_Y1 - 4 or py > ROI_Y2 + 4:
        return -1, -1
    mx, my = pixel_to_grid_float(px, py)
    gx = int(mx)
    gy = int(my)
    if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
        return -1, -1
    return gx, gy

def force_map_boundary_walls(out):
    wall = ENC["WALL"]
    top = 0
    bottom = (GRID_H - 1) * GRID_W

    for gx in range(GRID_W):
        out[top + gx] = wall
        out[bottom + gx] = wall

    for gy in range(GRID_H):
        row = gy * GRID_W
        out[row] = wall
        out[row + GRID_W - 1] = wall

def build_current_map(img):
    out = bytearray(GRID_W * GRID_H)
    for idx, (px, py) in enumerate(points):
        if SAMPLE_MODE == 1:
            L_val, A_val, B_val = sample1_lab(img, px, py)
        else:
            L_val, A_val, B_val = sample5_lab(img, px, py)
        code = classify_lab_code(L_val, A_val, B_val)
        if code == 255:
            out[idx] = last_map[idx] if USE_TEMPORAL_FALLBACK else ENC["EMPTY"]
        else:
            out[idx] = code
    force_map_boundary_walls(out)
    last_map[:] = out
    return out

def collect_cells(out, code, max_count):
    cells = []
    for idx, v in enumerate(out):
        if v == code:
            cells.append((idx % GRID_W, idx // GRID_W))
            if len(cells) >= max_count:
                break
    return cells

def local_player_grid_values(cx, cy):
    if cx < 0 or cy < 0:
        return 0, 0, -1, -1
    if USE_FAST_GRID_INVERSE:
        grid_x, grid_y = pixel_to_grid_float_fast(cx, cy)
    else:
        grid_x, grid_y = pixel_to_grid_float(cx, cy)
    x10 = int((grid_x - 0.5) * 10 + 0.5)
    y10 = int((grid_y - 0.5) * 10 + 0.5)
    gx = int(grid_x)
    gy = int(grid_y)
    if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
        gx = -1
        gy = -1
    return x10, y10, gx, gy

def car_candidate_map_conflict(out, gx, gy):
    # Boundary walls are immutable. Interior walls are also reliable on maps
    # without bombs; bomb maps may legitimately remove an interior wall later.
    if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
        return 1
    if gx == 0 or gx == GRID_W - 1 or gy == 0 or gy == GRID_H - 1:
        return 1
    has_bomb = False
    for idx in range(GRID_W * GRID_H):
        if out[idx] == ENC["BOMB"]:
            has_bomb = True
            break
    if (not has_bomb) and out[gy * GRID_W + gx] == ENC["WALL"]:
        return 2
    return 0

def local_player_to_mm(cx, cy):
    if cx < 0 or cy < 0:
        return 0, 0
    grid_x, grid_y = pixel_to_grid_float(cx, cy)
    x10 = int((grid_x - 0.5) * 10 + 0.5)
    y10 = int((grid_y - 0.5) * 10 + 0.5)
    return x10, y10

def local_player_to_x10(cx, cy):
    if cx < 0 or cy < 0:
        return -1, -1
    grid_x, grid_y = pixel_to_grid_float(cx, cy)
    return int((grid_x - 0.5) * 10 + 0.5), int((grid_y - 0.5) * 10 + 0.5)

def local_player_to_grid(cx, cy):
    if cx < 0 or cy < 0:
        return -1, -1
    return pixel_to_grid(cx, cy)

def pose_theta_x10(hx, hy, tx, ty):
    if hx < 0 or hy < 0 or tx < 0 or ty < 0:
        return None
    dx = hx - tx
    dy = hy - ty
    if abs(dx) + abs(dy) < 2:
        return None
    angle = math.atan2(dx, -dy) * 57.2957795
    if angle < 0:
        angle += 360.0
    return int(angle * 10 + 0.5)

def find_player_full_frame(img, scan_step=None, min_hit=None):
    global debug_full_search_source
    debug_full_search_source = "none"
    if PLAYER_USE_BLOB_FULL_SEARCH:
        pos = find_player_blobs_in_rect(img, ROI_X1, ROI_Y1, ROI_X2, ROI_Y2)
        if pos is not None:
            debug_full_search_source = "blob"
            return pos
    if not PLAYER_USE_PYTHON_FULL_SEARCH_FALLBACK:
        return None
    if scan_step is None:
        scan_step = PLAYER_FULL_SCAN_STEP
    if min_hit is None:
        min_hit = PLAYER_FULL_MIN_HIT
    pos = find_player_in_rect(img, ROI_X1, ROI_Y1, ROI_X2, ROI_Y2, scan_step, min_hit)
    if pos is not None:
        debug_full_search_source = "python"
    return pos

# =========================
# 调试状态
# =========================
player_found = False
player_cx = -1
player_cy = -1
last_player_gx = -1
last_player_gy = -1

head_cx = -1
head_cy = -1
tail_cx = -1
tail_cy = -1

debug_req_gx = -1
debug_req_gy = -1
debug_search_px0 = -1
debug_search_py0 = -1
debug_search_px1 = -1
debug_search_py1 = -1
debug_full_search_source = "none"

def draw_mapped_line(img, mx0, my0, mx1, my1, color, thickness=1, steps=32):
    last_x, last_y = screen_point_from_map(mx0, my0)
    for i in range(1, steps + 1):
        t = i / steps
        mx = mx0 + (mx1 - mx0) * t
        my = my0 + (my1 - my0) * t
        x, y = screen_point_from_map(mx, my)
        img.draw_line(last_x, last_y, x, y, color=color, thickness=thickness)
        last_x, last_y = x, y

def draw_cell_codes(img, out_bytes):
    col_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    col_end = GRID_W if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else GRID_W + 1
    row_start = 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else 0
    row_end = GRID_H if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else GRID_H + 1

    for gx in range(col_start, col_end):
        draw_mapped_line(img, gx, row_start, gx, row_end - 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else GRID_H,
                         color=(255, 255, 255), thickness=1)

    for gy in range(row_start, row_end):
        draw_mapped_line(img, col_start, gy, col_end - 1 if GEOMETRY_PROFILE == GEOMETRY_PROFILE_INNER else GRID_W, gy,
                         color=(255, 255, 255), thickness=1)

    for gy in range(GRID_H):
        for gx in range(GRID_W):
            idx = gy * GRID_W + gx
            v = out_bytes[idx]
            if v > 5:
                continue

            cx, cy = points[idx]
            img.draw_string(cx - TEXT_DX, cy - TEXT_DY, str(v), color=(0, 0, 0), scale=2, thickness=2)

def draw_player_center(img):
    if not DEBUG_TRACK:
        return
    if not player_found:
        return

    # 车中心（红）
    if player_cx >= 0 and player_cy >= 0:
        img.draw_cross(player_cx, player_cy, color=(255, 0, 0), size=4, thickness=1)
        img.draw_circle(player_cx, player_cy, 4, color=(255, 0, 0), thickness=1)
    if head_cx >= 0 and head_cy >= 0:
        img.draw_cross(head_cx, head_cy, color=(0, 255, 255), size=4, thickness=1)
    if tail_cx >= 0 and tail_cy >= 0:
        img.draw_cross(tail_cx, tail_cy, color=(0, 255, 0), size=4, thickness=1)
    if head_cx >= 0 and tail_cx >= 0:
        img.draw_line(tail_cx, tail_cy, head_cx, head_cy, color=(255, 255, 255), thickness=1)

def draw_search_region(img):
    if not DEBUG_TRACK:
        return
    if debug_search_px0 < 0:
        return

    x = debug_search_px0
    y = debug_search_py0
    w = debug_search_px1 - debug_search_px0
    h = debug_search_py1 - debug_search_py0

    if w > 0 and h > 0:
        img.draw_rectangle(x, y, w, h, color=(255, 255, 0), thickness=2)

def append_i16(buf, value):
    value = int(value) & 0xFFFF
    buf.append(value & 0xFF)
    buf.append((value >> 8) & 0xFF)

def append_i8(buf, value):
    buf.append(int(value) & 0xFF)

def append_cell_slots(buf, cells, max_count):
    count = len(cells)
    if count > max_count:
        count = max_count
    buf.append(count)
    for i in range(max_count):
        if i < count:
            append_i8(buf, cells[i][0])
            append_i8(buf, cells[i][1])
        else:
            append_i8(buf, -1)
            append_i8(buf, -1)

def send_full_map(out, car_x_mm, car_y_mm, car_theta_x10):
    payload = bytearray()
    payload.append(GRID_W)
    payload.append(GRID_H)

    walls = bytearray(24)
    for idx, v in enumerate(out):
        if v == ENC["WALL"]:
            walls[idx >> 3] |= 1 << (idx & 7)
    payload.extend(walls)

    append_cell_slots(payload, collect_cells(out, ENC["BOX"], MAX_BOXES), MAX_BOXES)
    append_cell_slots(payload, collect_cells(out, ENC["GOAL"], MAX_GOALS), MAX_GOALS)
    append_cell_slots(payload, collect_cells(out, ENC["BOMB"], MAX_BOMBS), MAX_BOMBS)

    append_i16(payload, car_x_mm)
    append_i16(payload, car_y_mm)
    append_i16(payload, car_theta_x10)
    payload.append(xor_checksum(payload))

    pkt = bytearray([VP_HEADER_0, VP_HEADER_1, VP_TYPE_FULL_MAP])
    pkt.extend(payload)
    uart_write_all(pkt)

def send_pos_update(frame_id, out, car_x_mm, car_y_mm, car_theta_x10, car_valid):
    payload = bytearray()
    # Bit7 explicitly marks whether this frame contains a current car pose.
    # The MCU must not mistake the retained coordinates for a live detection.
    payload.append((frame_id & 0x7F) | (0x80 if car_valid else 0x00))
    append_i16(payload, car_x_mm)
    append_i16(payload, car_y_mm)
    append_i16(payload, car_theta_x10)

    boxes = collect_cells(out, ENC["BOX"], MAX_BOXES)
    bombs = collect_cells(out, ENC["BOMB"], MAX_BOMBS)

    payload.append(len(boxes))
    for gx, gy in boxes:
        append_i8(payload, gx)
        append_i8(payload, gy)

    payload.append(len(bombs))
    for gx, gy in bombs:
        append_i8(payload, gx)
        append_i8(payload, gy)

    while len(payload) < 25:
        payload.append(0)

    if len(payload) > 25:
        payload = payload[:25]

    payload.append(xor_checksum(payload))

    pkt = bytearray([VP_HEADER_0, VP_HEADER_1, VP_TYPE_POS_UPDATE])
    pkt.extend(payload)
    uart_write_all(pkt)

def map_row_text(out, row):
    chars = "-#.$!@"
    s = ""
    for gx in range(GRID_W):
        v = out[row * GRID_W + gx]
        if 0 <= v < len(chars):
            s += chars[v]
        else:
            s += "?"
    return s

def print_map_debug(out):
    boxes = collect_cells(out, ENC["BOX"], MAX_BOXES)
    goals = collect_cells(out, ENC["GOAL"], MAX_GOALS)
    bombs = collect_cells(out, ENC["BOMB"], MAX_BOMBS)
    wall_count = 0
    for v in out:
        if v == ENC["WALL"]:
            wall_count += 1
    print("OA_MAP oa_id=%d box=%d goal=%d bomb=%d wall=%d" %
          (DEBUG_BUILD_ID, len(boxes), len(goals), len(bombs), wall_count))
    for row in range(GRID_H):
        print("OA%02d %s" % (row, map_row_text(out, row)))
    print("OA_MAP_END")

def print_car_debug(fps_value, car_x10, car_y10, car_theta_x10, car_valid):
    if car_valid:
        print("oa_id=%d FPS %.1f CAR (%.1f,%.1f) theta=%.1f deg" %
              (DEBUG_BUILD_ID, fps_value, car_x10 / 10.0, car_y10 / 10.0, car_theta_x10 / 10.0))
    else:
        print("oa_id=%d FPS %.1f CAR lost" % (DEBUG_BUILD_ID, fps_value))

# =========================
# 主循环
# =========================
frame_id = 0
last_pos_update_ms = 0
last_debug_ms = time.ticks_ms()
last_full_map_refresh_ms = 0
last_openmv_map_dump_ms = 0
last_player_full_search_ms = 0
map_cache_ready = False
debug_frame_count = 0
profile_snapshot_ms = 0
profile_map_ms = 0
profile_car_ms = 0
profile_send_ms = 0
last_car_x_mm = 0
last_car_y_mm = 0
last_car_x10 = -1
last_car_y10 = -1
last_car_theta_x10 = 0
car_known = False
player_track_lost = True
last_car_map_reject_ms = -10000

while True:
    poll_map_request()

    now_ms = time.ticks_ms()
    need_pos_update = time.ticks_diff(now_ms, last_pos_update_ms) >= POS_UPDATE_PERIOD_MS
    need_full_map = full_map_requested
    need_ide_map_dump = IDE_PERIODIC_MAP_DUMP_ENABLE and \
        time.ticks_diff(now_ms, last_openmv_map_dump_ms) >= IDE_PERIODIC_MAP_DUMP_PERIOD_MS
    need_map_refresh = need_full_map or need_ide_map_dump

    if not need_pos_update and not need_full_map and not need_map_refresh:
        if IDLE_UART_SLEEP_US > 0:
            time.sleep_us(IDLE_UART_SLEEP_US)
        continue

    clock.tick()
    profile_t0 = time.ticks_ms()
    img = sensor.snapshot()
    profile_snapshot_ms += time.ticks_diff(time.ticks_ms(), profile_t0)
    debug_frame_count += 1

    # 每帧先清空调试区域
    debug_search_px0 = -1
    debug_search_py0 = -1
    debug_search_px1 = -1
    debug_search_py1 = -1

    if need_map_refresh:
        if DEBUG_PRINT_EVENTS:
            if need_full_map:
                print("EVENT MAP_REFRESH start reason=request")
            elif need_ide_map_dump:
                print("EVENT MAP_REFRESH start reason=ide_periodic")
            elif not map_cache_ready:
                print("EVENT MAP_REFRESH start reason=request_first")
        event_t0 = time.ticks_ms()
        out = build_current_map(img)
        profile_map_ms += time.ticks_diff(time.ticks_ms(), event_t0)
        if DEBUG_PRINT_EVENTS:
            print("EVENT MAP_REFRESH done ms=%d" %
                  time.ticks_diff(time.ticks_ms(), event_t0))
        print_requested_map = DEBUG_PRINT_MAP_ON_FULL and need_full_map and \
            ((not map_cache_ready) or
             time.ticks_diff(now_ms, last_openmv_map_dump_ms) >= IDE_PERIODIC_MAP_DUMP_PERIOD_MS)
        if need_ide_map_dump or print_requested_map:
            print_map_debug(out)
            last_openmv_map_dump_ms = now_ms
        last_full_map_refresh_ms = now_ms
        map_cache_ready = True
    else:
        out = last_map

    player_found = False
    pos = None
    profile_t0 = time.ticks_ms()

    if (not player_track_lost) and player_cx >= 0 and player_cy >= 0:
        pos = find_player_near_pixel(img, player_cx, player_cy)
        if pos is None and DEBUG_PRINT_EVENTS:
            print("EVENT CAR_FAST_LOST px=%d py=%d" % (player_cx, player_cy))

    if pos is None and (not player_track_lost) and last_player_gx >= 0 and last_player_gy >= 0:
        pos = find_player(
            img, last_player_gx, last_player_gy,
            PLAYER_SCAN_STEP_MODE2,
            PLAYER_MIN_HIT_MODE2,
        )
        if pos is None and DEBUG_PRINT_EVENTS:
            print("EVENT CAR_GRID_LOST gx=%d gy=%d" % (last_player_gx, last_player_gy))
        if pos is None:
            player_track_lost = True
            player_cx = -1
            player_cy = -1
            last_player_gx = -1
            last_player_gy = -1

    need_full_player_search = player_track_lost and \
        time.ticks_diff(now_ms, last_player_full_search_ms) >= PLAYER_FULL_REACQUIRE_PERIOD_MS

    if pos is None and need_full_player_search:
        if DEBUG_PRINT_EVENTS:
            print("EVENT CAR_FULL_SEARCH start known=%d last_x10=%d last_y10=%d" %
                  (1 if car_known else 0, last_car_x10, last_car_y10))
        event_t0 = time.ticks_ms()
        pos = find_player_full_frame(img)
        if DEBUG_PRINT_EVENTS:
            if pos is None:
                print("EVENT CAR_FULL_SEARCH lost source=%s ms=%d" %
                      (debug_full_search_source,
                       time.ticks_diff(time.ticks_ms(), event_t0)))
            else:
                print("EVENT CAR_FULL_SEARCH hit source=%s ms=%d" %
                      (debug_full_search_source,
                       time.ticks_diff(time.ticks_ms(), event_t0)))
        last_player_full_search_ms = now_ms

    if pos is None:
        for attempt in range(PLAYER_FIND_EXTRA_SNAPSHOTS):
            img = sensor.snapshot()
            pos = find_player_full_frame(img)
            if pos is not None:
                break

    if pos is not None:
        candidate_cx, candidate_cy, candidate_head_cx, candidate_head_cy, \
            candidate_tail_cx, candidate_tail_cy = pos
        candidate_x10, candidate_y10, candidate_gx, candidate_gy = \
            local_player_grid_values(candidate_cx, candidate_cy)
        conflict = car_candidate_map_conflict(out, candidate_gx, candidate_gy) \
            if map_cache_ready else 0
        wall_override = conflict == 2 and CAR_WALL_OVERRIDE_ENABLE and car_known and \
            abs(candidate_x10 - last_car_x10) <= CAR_WALL_OVERRIDE_CONTINUITY_X10 and \
            abs(candidate_y10 - last_car_y10) <= CAR_WALL_OVERRIDE_CONTINUITY_X10
        if conflict and not wall_override:
            if DEBUG_PRINT_EVENTS and \
                    time.ticks_diff(now_ms, last_car_map_reject_ms) >= 1000:
                reason = "boundary" if conflict == 1 else "wall"
                print("EVENT CAR_MAP_REJECT reason=%s gx=%d gy=%d x10=%d y10=%d" %
                      (reason, candidate_gx, candidate_gy,
                       candidate_x10, candidate_y10))
                last_car_map_reject_ms = now_ms
            pos = None
            player_cx = -1
            player_cy = -1
            last_player_gx = -1
            last_player_gy = -1
            player_track_lost = True
        else:
            if wall_override:
                if DEBUG_PRINT_EVENTS and \
                        time.ticks_diff(now_ms, last_car_map_reject_ms) >= 1000:
                    print("EVENT CAR_WALL_OVERRIDE gx=%d gy=%d x10=%d y10=%d" %
                          (candidate_gx, candidate_gy,
                           candidate_x10, candidate_y10))
                    last_car_map_reject_ms = now_ms
            # A confirmed car cannot occupy an interior wall.  Remove only
            # that wall sample; goals, boxes and bombs keep their semantics.
            car_idx = candidate_gy * GRID_W + candidate_gx
            if out[car_idx] == ENC["WALL"]:
                out[car_idx] = ENC["EMPTY"]
                last_map[car_idx] = ENC["EMPTY"]
            player_cx = candidate_cx
            player_cy = candidate_cy
            head_cx = candidate_head_cx
            head_cy = candidate_head_cy
            tail_cx = candidate_tail_cx
            tail_cy = candidate_tail_cy
            player_found = True
            last_car_x10 = candidate_x10
            last_car_y10 = candidate_y10
            last_player_gx = candidate_gx
            last_player_gy = candidate_gy
            last_car_x_mm = last_car_x10
            last_car_y_mm = last_car_y10

    if player_found:
        theta_candidate = pose_theta_x10(head_cx, head_cy, tail_cx, tail_cy)
        if theta_candidate is not None:
            last_car_theta_x10 = theta_candidate
        car_known = True
        player_track_lost = False
        if DEBUG_TRACK:
            print("CENTER =", int(player_cx), int(player_cy), "grid", last_player_gx, last_player_gy,
                  "x10", last_car_x10, last_car_y10, "theta_x10", last_car_theta_x10)
    else:
        player_cx = -1
        player_cy = -1
        if DEBUG_TRACK:
            print("PLAYER not found")

    profile_car_ms += time.ticks_diff(time.ticks_ms(), profile_t0)

    if need_pos_update:
        profile_t0 = time.ticks_ms()
        send_pos_update(frame_id, out, last_car_x_mm, last_car_y_mm,
                        last_car_theta_x10, player_found)
        profile_send_ms += time.ticks_diff(time.ticks_ms(), profile_t0)
        frame_id = (frame_id + 1) & 0x7F
        last_pos_update_ms = now_ms

    if need_full_map:
        profile_t0 = time.ticks_ms()
        send_full_map(out, last_car_x_mm, last_car_y_mm, last_car_theta_x10)
        profile_send_ms += time.ticks_diff(time.ticks_ms(), profile_t0)
        full_map_requested = False

    if DEBUG_PRINT_FPS and time.ticks_diff(now_ms, last_debug_ms) >= DEBUG_FPS_PERIOD_MS:
        elapsed = time.ticks_diff(now_ms, last_debug_ms)
        fps_value = debug_frame_count * 1000.0 / elapsed if elapsed > 0 else 0.0
        if DEBUG_PRINT_CAR:
            print_car_debug(fps_value, last_car_x10, last_car_y10,
                            last_car_theta_x10, player_found)
        else:
            print("FPS %.1f" % fps_value)
        if DEBUG_PROFILE_TIMING and debug_frame_count > 0:
            print("PROFILE avg_ms snap=%.1f map=%.1f car=%.1f send=%.1f" %
                  (profile_snapshot_ms / debug_frame_count,
                   profile_map_ms / debug_frame_count,
                   profile_car_ms / debug_frame_count,
                   profile_send_ms / debug_frame_count))
        debug_frame_count = 0
        profile_snapshot_ms = 0
        profile_map_ms = 0
        profile_car_ms = 0
        profile_send_ms = 0
        last_debug_ms = now_ms

    # 可选显示
    if SHOW_CELL_CODES:
        draw_cell_codes(img, out)

    if DEBUG_TRACK:
        draw_search_region(img)
        draw_player_center(img)

    if SHOW_FPS:
        img.draw_string(2, 2, "FPS: %.1f" % clock.fps())
