# OpenART视觉识别交接说明

更新时间：2026-06-24

## 1. 项目背景

本项目使用 OpenART Plus mini（OpenMV 兼容）作为视觉层，识别 16x12 的格子地图，并通过 UART 向 MCU 发送：

- `FULL_MAP`：完整地图，包含墙、目标点、箱子、炸弹、车位姿。
- `POS_UPDATE`：周期性车位姿和近距离动态物体更新。
- `HEARTBEAT`：心跳。

当前重点是提升摄像头边缘畸变、边缘偏暗、颜色不稳定时的地图识别鲁棒性，尤其是墙体、目标点、车头车尾识别。

## 2. 重要路径

电脑端 OpenART 代码：

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart
```

OpenART / SD 卡运行代码：

```text
G:\
```

当前需要保持同步的 5 个入口文件：

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\boot.py
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\main.py
G:\boot.py
G:\main.py
G:\sd\main.py
```

当前这 5 份文件已完全一致，SHA256：

```text
B4E0C5827F6C8D04CE168CB7FAB34A3F717B98F7D2F80F3921524111BEBFD454
```

历史备份文件曾创建过：

```text
.bak_vignette_20260623_215359
```

不要修改这些备份文件。

## 3. 参考文件

设计规划文件：

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\design\中间层完整规划.md
```

该规划是目前架构参考中置信度最高的一版。它描述了：

- OpenART 视觉层。
- MCU `vision_parser`。
- solver。
- `path_follower`。
- motion control。

当前 OpenART 修改方向与该规划匹配：视觉层提供稳定 `FULL_MAP` 和高频 `POS_UPDATE`，静态地图尽量一次识别准确，后续用动态层叠加车、箱子、炸弹。

用户测试图片：

```text
C:\Users\L\Desktop\20.bmp
```

逐飞官方文档：

```text
C:\Users\L\Desktop\新建 文本文档 (3).txt
```

## 4. 当前视觉总体方案

当前采用“首次高质量静态建图 + 后续动态叠加”的方式。

首次 FULL_MAP：

1. 用几何校正后的采样坐标取色。
2. 应用固定亮度补偿蒙版。
3. 每个格子做 `5x5` 静态采样。
4. 锁定静态底图：墙 `#`、目标点 `.`、地面 `-`。
5. 做自动弱墙后处理，补救采样偏弱但处于连续墙结构中的墙格。

后续 FULL_MAP：

1. 复制静态底图。
2. 动态叠加箱子 `$`、炸弹 `!`、车 `@`。
3. 目标点始终来自静态底图，避免被车或箱子覆盖后丢失。

## 5. 几何与亮度处理

摄像头为 120 广角，边缘有桶状畸变和暗角。当前做法：

- 不直接整图矫正，避免算力开销。
- 只在采样点映射上做几何修正。
- 四边由手动标定的边缘控制点通过 Coons/edge interpolation 映射。
- 亮度补偿只作用于采样像素，不处理整张图。

当前关键边缘控制点在 `openart/boot.py` 顶部：

```python
LT = (11, 9)
RT = (310, 7)
RB = (310, 233)
LB = (3, 220)

TOP_C04 = (82, 6)
TOP_C08 = (155, 5)
TOP_C12 = (233, 5)
BOT_C04 = (75, 225)
BOT_C08 = (150, 230)
BOT_C12 = (231, 232)
LEFT_R03 = (7, 60)
LEFT_R06 = (4, 112)
LEFT_R09 = (3, 167)
RIGHT_R03 = (312, 61)
RIGHT_R06 = (313, 118)
RIGHT_R09 = (313, 176)
```

当前桶形参数保留接口但为 0：

```python
SAMPLE_GEOMETRY_CORRECTION = True
SAMPLE_BARREL_K1 = 0.0
SAMPLE_BARREL_K2 = 0.0
```

亮度补偿固定蒙版：

```python
SAMPLE_BRIGHTNESS_CORRECTION = True
BRIGHTNESS_GAIN_GRID_SIZE = 5
BRIGHTNESS_GAIN_GRID_Q6 = (
    (96, 84, 78, 84, 90),
    (88, 74, 68, 73, 82),
    (84, 70, 64, 70, 78),
    (90, 76, 70, 74, 84),
    (100, 88, 80, 86, 94),
)
```

## 6. 当前关键参数

静态建图：

```python
STATIC_MAP_LOCK = True
STATIC_MAP_BUILD_SAMPLES = 5
STATIC_WALL_SAMPLE_SCORE_MIN = 78
STATIC_WALL_VOTE_MIN = 8
STATIC_WALL_WEAK_VOTE_MIN = 4
STATIC_WALL_NEIGHBOR_VOTE_MIN = 8
STATIC_WALL_NEIGHBOR_MIN = 2
STATIC_GOAL_VOTE_MIN = 3
STATIC_GOAL_WALL_VOTE_MAX = 6
STATIC_FORCE_BORDER_WALLS = True
```

周期调试：

```python
PERIODIC_FULL_MAP_DEBUG = True
PERIODIC_FULL_MAP_PERIOD_MS = 2000
PERIODIC_FULL_MAP_SEND_UART = False
STATIC_CELL_DEBUG = False
CAR_CANDIDATE_SCORE_DEBUG = True
```

车识别与平滑：

```python
MIN_CAR_PIXELS = 4
MAX_CAR_MARKER_PIXELS = 220
CAR_MARKER_SCORE_MIN = 58
CAR_MARKER_SCORE_MARGIN = 6
CAR_NEAR_BACK_RESCUE = True
CAR_RESCUE_CYAN_SCORE_MIN = 58
CAR_RESCUE_SCORE_MARGIN = 6
CAR_RESCUE_SAMPLE_STEP = 2

CAR_POSE_SMOOTHING = True
CAR_POSE_SMOOTH_ALPHA = 0.45
CAR_THETA_SMOOTH_ALPHA = 0.18
CAR_THETA_MAX_STEP_DEG = 6.0
CAR_POSE_SMOOTH_MAX_DIST = 1.2
```

动态墙抑制目前关闭：

```python
ENABLE_DYNAMIC_WALL_SUPPRESS = False
```

原因：之前动态抑制会把车/箱子附近的真墙清掉，例如 `(13,6)、(13,7)、(12,7)` 一类墙体。

## 7. 历史修改思路

### 7.1 图像几何拟合

最初图像存在：

- 130 摄像头桶状畸变。
- 边缘暗角。
- 地图四角不正。

处理过程：

1. 添加绿色网格交点和黄色采样中心点调试。
2. 用户手动调整四角和边缘控制点。
3. 用四边控制点生成映射，替代简单矩形/四点透视。
4. 最终绿点边缘拟合较好，黄色采样点基本落在格子中心。

### 7.2 亮度补偿

用户提出固定亮度蒙版思路：摄像头和屏幕位置固定，边缘暗角也基本固定。

当前实现：

- 使用 `BRIGHTNESS_GAIN_GRID_Q6` 作为 5x5 固定增益表。
- 只对采样点像素补偿。
- 避免整图处理，节省 OpenART 算力。

### 7.3 RGB + LAB 混合置信度

车头/车尾原本容易因青色、绿色、边缘暗角互相混淆。

当前实现：

- 使用 `car_marker_scores()` 混合 LAB 和 RGB 比例。
- 青色、绿色分别打分。
- 加入 `CAR_MARKER_SCORE_MIN` 和 `CAR_MARKER_SCORE_MARGIN`。
- 明显品红目标直接排除出车标评分，避免目标点被误认为车头。

### 7.4 车正反稳定

已实现前后标记连续性：

- `g_last_front_marker`
- `g_last_back_marker`
- `marker_track_error()`
- `find_best_car_pair()`

当车头车尾候选可能互换时，会优先沿用上一帧前后关系，减少 180 度跳变。

### 7.5 静态地图锁定

曾经尝试动态墙抑制，但它会把车/箱子周围真墙误删。

后来改为：

- 首次 FULL_MAP 高质量静态建图。
- 后续直接复制静态底图。
- 只动态覆盖箱子、炸弹、车。

这符合设计规划：静态墙体不需要每帧重复识别，只要首次正确，后续应作为高置信度基准。

### 7.6 bytearray 网格结构

最初静态底图用二维 `list`，在 OpenMV MicroPython 上出现：

```text
TypeError: 'list' object isn't subscriptable
IndexError: list index out of range
```

已经改为一维 `bytearray`：

- `grid_get()`
- `set_grid_cell_safe()`
- `grid_row_text()`

所有 FULL_MAP 打包、目标扫描、箱子扫描、调试打印都走统一 helper。

### 7.7 目标点与炸弹误判

之前 `(2,2)` 目标点和左上附近墙体被误判成炸弹。

已修：

- 炸弹识别排除明显品红目标。
- 静态目标识别要求 `goal_votes >= 3` 且 `wall_votes <= 6`。
- 动态炸弹只允许叠加在静态地面 `-` 上，墙和目标上的假炸弹会被过滤。

当前测试中：

```text
goals=3: (2,2), (10,4), (10,5)
bombs=0
```

### 7.8 强品红目标兜底

`(10,5)` 中心颜色是强品红：

```text
rgb=(255,67,255)
```

但 LAB 判断一度没有通过。

已加 RGB 兜底：

- `looks_magenta_goal_rgb()`
- `is_strong_magenta_goal_rgb()`

强品红目标不再完全依赖 LAB。

### 7.9 车贴边识别

车在地图边缘时曾经完全检测不到。

当前已加：

- 更小 `MIN_CAR_PIXELS = 4`。
- 过大候选过滤 `MAX_CAR_MARKER_PIXELS = 220`。
- 品红排除，避免目标/粉墙变成假车头。
- `rescue_car_near_backs()`：当正常配对失败时，在绿色车尾附近搜索青色车头。

当前测试中车已能在边缘识别：

```text
car: grid=(约1.2~1.4, 约6.0~6.3)
```

### 7.10 车位姿平滑

车贴边时位置较稳，但 `theta` 曾在 0 到 50 度之间跳。

已加：

- 位置平滑：`CAR_POSE_SMOOTH_ALPHA = 0.45`
- 角度慢跟随：`CAR_THETA_SMOOTH_ALPHA = 0.18`
- 单帧角度最大变化：`CAR_THETA_MAX_STEP_DEG = 6.0`

注意：这会牺牲一点角度灵敏度，但对路径地图和低速控制通常更安全。若实车转向反馈变慢，可尝试：

```python
CAR_THETA_SMOOTH_ALPHA = 0.25
CAR_THETA_MAX_STEP_DEG = 10.0
```

### 7.11 弱墙自动后处理

用户指出不能手动补 `(13,4)`，比赛地图会变化。

已撤销手动强制坐标，改为自动弱墙补偿：

- 强墙：`wall_votes >= 8`
- 弱墙候选：`wall_votes >= 4`
- 弱墙需周围至少 2 个方向有强墙或强墙票支撑。
- 目标格不会被后处理覆盖。

相关函数：

- `classify_static_cell_from_votes()`
- `static_neighbor_strong_wall()`
- `post_process_static_walls()`

如果有自动补墙，会输出：

```text
STATIC_WALL_POST changes=N
```

## 8. 当前测试结果概况

最近一次稳定输出中：

```text
################
#-#------------#
#-.------#####-#
##$###---#---#-#
#----#---#.#---#
#----#####.#-#-#
#@------$--$-#-#
#-----------##-#
#--------------#
#-----####-----#
#--------------#
################
```

识别结果：

```text
boxes=3
  box0: (2,3)
  box1: (8,6)
  box2: (11,6)
goals=3
  goal0: (2,2)
  goal1: (10,4)
  goal2: (10,5)
bombs=0
car: 已能识别，边缘时 theta 已做限幅平滑
```

用户发现：

- `(13,4)` 墙体没有识别到。

针对该问题，当前最新方案是自动弱墙补偿，而不是手动补坐标。下一次测试应重点看：

1. `(13,4)` 是否能通过 `STATIC_WALL_POST` 自动补为 `#`。
2. 是否产生新的假墙。
3. 目标 `(10,4)、(10,5)` 是否仍保持稳定。

## 9. 下次对话建议流程

如果下次继续调试，建议按这个顺序：

1. 先确认 5 份入口文件是否仍同哈希。
2. 让用户运行 OpenART，贴出首次 `STATIC_MAP_BUILT` 后的前几帧 `FULL_MAP`。
3. 如果墙体有漏识别：
   - 不要手动补坐标。
   - 优先检查 `STATIC_WALL_POST changes=N`。
   - 如补墙不足，调小 `STATIC_WALL_WEAK_VOTE_MIN` 或 `STATIC_WALL_NEIGHBOR_MIN`。
   - 如假墙变多，调高 `STATIC_WALL_WEAK_VOTE_MIN` 或要求更严格的邻接支撑。
4. 如果目标点丢失：
   - 临时打开 `STATIC_CELL_DEBUG = True`。
   - 把目标周围格加入 `STATIC_DEBUG_CELLS`。
   - 看 `goal_votes`、`wall_votes`、`rgb`、`mag`。
5. 如果车丢失或角度跳：
   - 保留 `CAR_CANDIDATE_SCORE_DEBUG = True`。
   - 看 `front/back/front_rej/back_rej` 和 `pair` 输出。
   - 如车位置稳定但角度漂移，调 `CAR_THETA_SMOOTH_ALPHA` 和 `CAR_THETA_MAX_STEP_DEG`。

## 10. 常用验证命令

本地编译检查：

```powershell
python -m py_compile "C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\boot.py"
```

同步 5 份入口文件后验证哈希：

```powershell
$paths = @(
  "C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\boot.py",
  "C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\main.py",
  "G:\boot.py",
  "G:\main.py",
  "G:\sd\main.py"
)
Get-FileHash -Algorithm SHA256 -LiteralPath $paths
```

当前同步原则：

- 如果修改 `openart\boot.py`，必须同步到其余 4 份入口文件。
- 如果用户手动改了 `openart\boot.py`，默认以它为源同步。
- 不要修改备份文件。

## 11. 注意事项

- 不要再用手动固定地图坐标补墙，比赛地图会变化。
- 静态地图应靠采样、投票、邻接结构自动识别。
- 动态层不应反向污染静态墙和目标。
- 目标点和墙的优先级要谨慎：目标点可覆盖地面，但不应轻易覆盖强墙。
- 车头车尾在边缘最容易受暗角和屏幕颜色影响，调参时要看候选分数，不要只看最终 `theta`。
- 当前 `STATIC_CELL_DEBUG = False`，需要分析目标/墙采样时再打开。
