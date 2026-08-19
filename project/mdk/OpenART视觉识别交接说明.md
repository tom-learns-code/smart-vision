# OpenART视觉识别交接说明

> 更新时间：2026-06-26  
> 当前阶段：分层视觉主框架已实现，正在逐项优化快速帧性能；多箱ROI开销已下降，目标消失兜底已降级为确认路径  
> 当前首要目标：在不降低准确度和灵敏度的前提下，提高车辆与动态物体跟踪帧率  
> 本文以当前实际代码为准，覆盖旧版交接说明

---

## 0. 本次暂停节点

### 0.1 当前代码状态

本次暂停时，最新权威代码仍是：

```text
project\mdk\vision_staging\boot.py
project\mdk\vision_staging\main.py
```

两份入口已经同步，哈希一致：

```text
SHA256:
8E5B25151B6F963CAC626B251BA2183170866EF2B4F9FA30974141D961005121

文件大小:
148797 bytes
```

已执行：

```powershell
python -m py_compile .\vision_staging\boot.py .\vision_staging\main.py
```

语法检查通过，`__pycache__` 已清理。

注意：`openart\boot.py`、`openart\main.py`、`G:\boot.py`、`G:\main.py`、`G:\sd\main.py` 仍可能是旧副本。IDE直接运行 `vision_staging\boot.py` 才是当前最新版本。

### 0.2 本轮最后完成的修改

当前性能与跟踪优化走到 `6.14`：

```python
OBJECT_ROI_MAX_TRACKS = 2
OBJECT_ROI_SECONDARY_PERIOD = 2
OBJECT_AXIS_SWITCH_FRAMES = 2
GLOBAL_GOAL_CONFIRMATION_ONLY = True
GLOBAL_GOAL_EXPLICIT_ASSIGN_RADIUS = 1.35
```

本轮经验：

- `OBJECT_ROI_MAX_TRACKS=2` 比 `1` 更容易让多箱假动作场景建立候选，右侧箱子已经能出现 `GOAL_CANDIDATE goal=(13,3)` 并成功触发 `BOX_GOAL_MATCHED`。
- 但每帧扫2个对象ROI会把推动阶段 `region` 多次拉到约 `190～215 ms`，FPS回落到约 `3～4`。
- 因此当前不是退回1个ROI，而是保留2个ROI名额，但让第二对象ROI隔帧扫描：`OBJECT_ROI_SECONDARY_PERIOD=2`。
- 旧候选过远时不再直接提交 `BOX_GOAL_MATCHED` 或 `BOX_GOAL_MISMATCHED`，而是输出 `TRACE GOAL_VERIFY_STALE ...` 并清掉候选。
- 当前宁可暴露“候选没建立/连续跟踪丢失”，也不要靠全局目标消失去猜测完成。

### 0.3 下次首先测试

继续使用当前同一测试：

```text
空地图
3箱 + 3目标
目标点与箱子打乱对应
加入假动作：推到目标点旁边、推走、再推向其他目标点
IDE图传关闭，只保留串口/FPS/PERF
```

重点观察：

- 推动阶段 `region` 是否从上一轮常见 `190～215 ms` 降下来。
- FPS是否从约 `3～4` 回升，目标是尽量接近 `5～6`。
- 右侧箱子是否还能建立 `TRACE GOAL_CANDIDATE ... goal=(13,3)`。
- 出现 `TRACE GOAL_VERIFY_STALE ...` 时，后续是否能重新建立正确候选。
- 如果仍出现 `BOX_MISSING_NO_GOAL`，下一步应加对象ROI选择/关联trace，确认是ROI没扫到、组件没识别到，还是关联被旧轨迹抢走；不要立刻放宽事件提交。

### 0.4 下次不应走偏的点

- 不要把 `GLOBAL_GOAL_ASSIGN` 重新放宽成全局猜测。假动作测试已经证明这条路会误提交。
- 不要为了追求事件成功率而允许远距离旧候选提交完成。
- 不要降低采样密度或放宽颜色阈值来换FPS；当前问题主要是ROI调度和对象连续跟踪。
- 如果帧率仍不够，优先继续优化对象ROI选择、节流和关联，不要回到每帧高成本全图/多ROI扫描。
- 如果候选经常缺失，下一步加trace定位：候选对象是否被选中ROI、ROI里是否检测到组件、组件是否被正确关联到track。

### 0.5 已验证失败路线与注意事项

以下内容是已经实测或讨论确认过的坑，下次不要重新从这些方向开始：

#### 墙体与静态层

- 弱墙结构补偿不应重新作为正式结果启用。它容易把相隔一个空格、目标点或其他元素的两个墙体中间误补成墙。
- 不要按某张地图或某个坐标特判补墙。所有修改必须能适用于未知比赛地图。
- 当前墙体识别应优先依赖强墙采样、强票数和验证帧，而不是事后形态补丁。

#### 颜色、采样与亮度

- 当前新手机标定和亮度蒙版已经稳定，不要为了追帧率先改颜色阈值或降低采样密度。
- 质量包括灵敏度、连续性和延迟，不只是最终地图看起来对。降低采样密度属于最后手段。
- IDE图传会明显影响帧率；性能测试时应关闭图传，只保留串口/FPS/PERF输出。

#### 车辆与推动规则

- 不要用车头朝向判断推动方向。比赛规则确认：车辆任意朝向都可以推箱子或炸弹。
- 日志里的“大跳变”主要来自低帧率/漏帧，不是上位机瞬移；不要按瞬移假设写逻辑。
- 车辆失联时不要每帧做高成本全图回退；连续全图回退会制造“低帧率 -> 更大跨帧位移 -> 更容易漏检”的反馈链。

#### 对象ROI与帧率

- `OBJECT_ROI_MAX_TRACKS=1` 在多箱假动作场景偏窄，当前被推箱子可能抢不到对象ROI，候选建立不稳。
- 每帧扫描2个对象ROI能改善候选建立，但推动阶段 `region` 多次达到约 `190～215 ms`，FPS回落到约 `3～4`，因此当前改为第二对象ROI隔帧扫描。
- 如果后续仍漏候选，优先加对象ROI选择/关联trace，而不是继续盲目增加ROI数量或半径。

#### 目标事件与全局兜底

- 全局目标消失只能作为确认路径，不能作为“猜测哪个箱子完成”的主路径。假动作测试已经证明全局猜测会误提交。
- 没有显式 `GOAL_CANDIDATE` 的箱子不应靠 `GLOBAL_GOAL_ASSIGN` 完成。
- 有其他候选目标的箱子不应被强行分配到另一个目标。
- 远离旧候选目标时不应提交 `BOX_GOAL_MATCHED` 或 `BOX_GOAL_MISMATCHED`；当前会输出 `GOAL_VERIFY_STALE` 并清掉旧候选。
- 如果真实完成后没有事件，优先判断是否缺少候选或连续跟踪断掉，不要先放宽事件提交门槛。

#### 已撤回或低收益优化

- 旧位置4格局部恢复已实测失败：4次恢复成功率为0，还会增加单次回退耗时。
- 车辆颜色分类LAB短路虽然严格等价，但实机收益很小；保留即可，不继续围绕它派生优化。
- 缓冲复用主要减少分配和潜在GC压力，平均FPS收益不明显；保留即可。

#### 文件与同步

- `vision_staging\boot.py` 和 `vision_staging\main.py` 是权威源。
- `openart\boot.py`、`openart\main.py`、`G:\boot.py`、`G:\main.py`、`G:\sd\main.py` 可能落后；不要反向覆盖 staging。
- 每次改完至少检查：`boot.py/main.py` 哈希一致、语法检查通过、`__pycache__` 清理。

---

## 1. 当前结论速览

当前视觉代码已经不再是早期的“每隔一段时间重新扫描并输出整张地图”方案，而是：

```text
首帧高质量建图
├─ 静态层：墙、地面、目标点
├─ 动态层：箱子、炸弹及稳定对象ID
└─ 车辆层：连续坐标、角度和短期速度

正常运行
├─ 每帧静默抓图和分析
├─ 车辆附近执行快速ROI识别
├─ 动态对象按ACTIVE/FROZEN状态跟踪
├─ IDE每秒输出FPS/PERF诊断
├─ UART周期发送POS_UPDATE
└─ 只在首帧、事件或MAP_REQUEST时生成FULL_MAP
```

截至本次暂停：

- 首帧静态地图和新手机标定已经较稳定。
- 分层状态、箱子目标事件、爆炸事务框架已经写入代码。
- 空地图正常快速帧已经从约 `0.9 FPS` 提升到常见 `8～12 FPS`。
- 车辆连续失联时的全图回退反馈链已经通过限频切断。
- `单箱 + 单目标` 已验证能触发 `BOX_GOAL_MATCHED` 并正确移除箱子和目标点。
- 新增车辆ROI与对象ROI首版拆分，单箱单目标远近距离测试通过。
- 多箱误配已加清计数和合理性门槛；复测显示误配改善。关掉IDE图传后帧率明显提升；目前全局目标验证已局部化到候选相关目标，并进一步降级为“确认已有显式 `GOAL_CANDIDATE`”的路径，不再凭近距离或运动方向评分直接提交完成。
- 多箱假动作场景已从“全局误提交”收敛到“连续跟踪/候选建立是否稳定”的问题。
- 当前最新优化是 `6.14`：保留2个对象ROI名额，但第二对象ROI隔帧扫描，并阻止远距离旧候选提交事件。

---

## 2. 最重要的代码路径与同步状态

### 2.1 当前最新、应作为权威源的代码

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\project\mdk\vision_staging\boot.py
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\project\mdk\vision_staging\main.py
```

两份文件当前完全一致：

```text
SHA256:
8E5B25151B6F963CAC626B251BA2183170866EF2B4F9FA30974141D961005121

文件大小:
148797 bytes
```

本轮测试一直由IDE直接运行：

```text
project\mdk\vision_staging\boot.py
```

### 2.2 设备启动副本目前落后

以下五份文件仍是较早版本：

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\boot.py
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\main.py
G:\boot.py
G:\main.py
G:\sd\main.py
```

它们当前彼此一致，但不是最新 staging：

```text
SHA256:
EB6814CA4F0857377A9591C203AD5BA110C02CAA613692919C0DD226ACB6CACB

文件大小:
106630 bytes
```

因此下次必须注意：

- IDE直接运行 `vision_staging\boot.py`：使用最新版本。
- OpenART脱离IDE自行启动：仍可能运行旧版本。
- 在准备设备自主启动或上完整赛道前，应以 `vision_staging\boot.py` 为源同步五份入口。
- 同步前不要反向用 `openart\boot.py` 覆盖 staging。

### 2.3 当前稳定备份

在大规模分层重构前已保存稳定版本：

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\backup\视觉稳定版_20260625_020206
```

该备份包含：

```text
openart\boot.py
openart\main.py
sd\boot.py
sd\main.py
sd\sd_main.py
design\视觉分层识别与事件驱动更新设计.md
README.md
```

不要修改该目录。出现严重回归时可用于比较，但当前开发应继续以 `vision_staging` 为主。

---

## 3. 重要设计文档

当前设计依据：

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\design\视觉分层识别与事件驱动更新设计.md

C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\design\全系统分层视觉与中间层协同总体规划.md

C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\design\中间层完整规划.md
```

其中：

- `视觉分层识别与事件驱动更新设计.md` 保存视觉层规则和状态机依据。
- `全系统分层视觉与中间层协同总体规划.md` 保存视觉、协议、算法、中间层和控制层的整体协同方案。
- 当前优先目标仍是先打通可运行链路，不提前构建过度复杂的通信健壮性体系。

性能优化逐项记录：

```text
C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\project\mdk\vision_staging\PERFORMANCE_OPTIMIZATION_PROGRESS.md
```

---

## 4. 必须坚持的用户要求与比赛规则

### 4.1 修改原则

- 所有识别逻辑必须普遍适用，禁止按某张地图或某个固定坐标打补丁。
- 记忆文件和旧设计只能作为参考，不能机械照搬。
- 优先提高强墙和强证据识别，不再依赖弱墙形态补偿制造墙。
- 质量包括准确度、灵敏度、连续性和响应延迟。
- 提升帧率时，首先做严格等价或不降低采样密度的优化。
- 只有无损路线基本走完后，才考虑降低采样密度或简化分类器。
- 性能优化必须逐项实施、逐项实测，避免一次修改过多而无法归因。
- 某个优化被实测证明无效或负收益时，应撤回，不在错误路线继续打补丁。

### 4.2 已确认比赛规则

- 地图固定为 `16×12`。
- 第一行、最后一行、第一列、最后一列为不可破坏边界墙。
- 箱子不能进入任何墙体。
- 炸弹不能进入边界墙，但可以进入普通墙并触发爆炸。
- 箱子和炸弹只沿X或Y单轴运动，不会斜推。
- 不允许连锁推动。
- 车辆推动物体时不要求车头朝向物体。
- `car.theta` 不能用于判断推动轴、推动方向或是否正在推动。
- 推动关系主要由车辆中心位置、车辆速度、物体速度、相对位置和碰撞规则判断。
- 动态物体被目标点或墙体遮挡不等于消失。

### 4.3 关于车辆运动

当前上位机控制的车辆运动是连续线性运动，没有瞬间跳跃操作。

日志中的大幅跨帧位移来自：

```text
识别漏帧或全图回退耗时过高
→ 图像采样间隔变长
→ 相邻识别帧之间位置跨度变大
```

后续分析不得把该现象误归因于上位机瞬移。

---

## 5. 摄像头、几何和颜色标定

### 5.1 当前硬件与图像格式

```python
sensor.RGB565
sensor.QVGA       # 320×240
auto_gain = False
auto_whitebal = False
```

当前摄像头为 OpenART Plus mini：

- 主频约 1 GHz。
- 片内 SRAM 2 MB。
- SDRAM 64 MB。
- 外置 Flash 16 MB。

性能优化允许合理使用内存换取速度。

### 5.2 新旧几何标定均保留

当前选择：

```python
GEOMETRY_PROFILE = GEOMETRY_PROFILE_INNER
```

外坐标旧标定仍完整保留。新手机屏幕较大、地图外边缘不完全可见，因此新增内坐标标定：

```python
INNER_LT = (17, 25)
INNER_RT = (294, 22)
INNER_RB = (291, 221)
INNER_LB = (17, 208)

INNER_TOP_EDGE_POINTS = (
    INNER_LT, (81, 23), (151, 22), (221, 21), INNER_RT)

INNER_BOTTOM_EDGE_POINTS = (
    INNER_LB, (79, 217), (145, 220), (219, 223), INNER_RB)

INNER_LEFT_EDGE_POINTS = (
    INNER_LT, (15, 69), (15, 118), (15, 164), INNER_LB)

INNER_RIGHT_EDGE_POINTS = (
    INNER_RT, (295, 71), (293, 122), (292, 174), INNER_RB)
```

内坐标映射的是边界墙内侧，也就是地图 `col=1..15`、`row=1..11` 的边界，而不是画面外不可见的地图外轮廓。

桶形接口仍保留，但当前参数为0：

```python
SAMPLE_GEOMETRY_CORRECTION = True
SAMPLE_BARREL_K1 = 0.0
SAMPLE_BARREL_K2 = 0.0
```

当前主要依赖四边控制点插值，不做整图畸变变换。

### 5.3 新旧亮度与颜色配置均保留

当前选择：

```python
CALIBRATION_PROFILE = CALIBRATION_PROFILE_NEW
```

旧手机配置：

```python
OLD_BRIGHTNESS_GAIN_GRID_Q6 = (
    (96, 84, 78, 84, 90),
    (88, 74, 68, 73, 82),
    (84, 70, 64, 70, 78),
    (90, 76, 70, 74, 84),
    (100, 88, 80, 86, 94),
)
```

新手机配置：

```python
NEW_BRIGHTNESS_GAIN_GRID_Q6 = (
    (99, 84, 66, 67, 89),
    (99, 69, 61, 61, 77),
    (99, 68, 61, 62, 76),
    (99, 70, 62, 64, 84),
    (99, 87, 74, 81, 99),
)
```

补偿只作用于采样点，不处理整张图。

新配置还包含独立的：

- 墙体LAB范围。
- 黄色箱子RGB规则。
- 品红目标RGB规则。
- 红色炸弹RGB规则。
- 车头青色与车尾绿色混合RGB/LAB评分。

### 5.4 新标定素材

当前新手机素材目录：

```text
C:\Users\L\Desktop\图层蒙版
```

内容：

```text
0.bmp：空地
1.bmp：箱子
2.bmp：目标点
3.bmp：炸弹
4.bmp：墙体
a～e.bmp：车辆约顺时针135度
f～k.bmp：车辆约逆时针90度
l～n.bmp：车辆约顺时针45度
x.bmp：复杂地图无网格图传
y.bmp：复杂地图带网格图传
```

这些素材用于离线提取亮度分布、颜色范围和检查网格落点。

---

## 6. 降帧率优化前已经完成的视觉功能

以下内容是在性能专项优化开始前已完成或已写入代码的主框架。

### 6.1 首帧高质量静态建图

当前主要参数：

```python
STATIC_MAP_LOCK = True
STATIC_MAP_BUILD_SAMPLES = 5
STATIC_MAP_VERIFY_FRAMES = 2
STATIC_MAP_VERIFY_FRAME_DELAY_MS = 10

STATIC_WALL_SAMPLE_SCORE_MIN = 78
STATIC_WALL_STRONG_SAMPLE_SCORE_MIN = 88
STATIC_WALL_VOTE_MIN = 8
STATIC_WALL_STRONG_VOTE_MIN = 2

STATIC_GOAL_VOTE_MIN = 5
STATIC_GOAL_STRONG_VOTE_MIN = 2
STATIC_GOAL_WALL_VOTE_MAX = 6
STATIC_FORCE_BORDER_WALLS = True
```

静态建图流程：

1. 第一帧全图扫描动态物体。
2. 根据车、箱子、炸弹生成动态遮挡蒙版。
3. 静态格使用 `5×5` 高质量采样。
4. 对需要验证的格执行2帧补充验证。
5. 保存 `g_static_initial` 和 `g_static_grid`。
6. 边界墙强制为墙。
7. 后续常规快速帧不重复高质量全图静态建图。

### 6.2 弱墙补偿已经停止应用

旧方案曾根据邻接墙结构补弱墙，容易把间隔一个空格、目标或其他元素的区域误补成墙。

当前：

```python
STATIC_WALL_POST_APPLY = False
STATIC_WALL_POST_DIAGNOSTIC = True
```

即：

- 可以输出诊断。
- 不把后处理结果写回正式地图。
- 当前正式结果依赖强墙票数、强样本和验证帧。

不要重新开启弱墙应用，除非有新的普适证据和完整回归测试。

### 6.3 三层世界状态

已经实现：

```text
静态地图层
动态对象层
车辆状态层
```

动态对象使用固定长度列表保存，避免MicroPython对象分配不可控。

对象内部状态：

```text
FROZEN
ACTIVE
PUSH_CANDIDATE
PUSHING_X
PUSHING_Y
OCCLUDED
SETTLING
BLOCKED
EXPLOSION_PENDING
REMOVED
```

对象输出状态：

```text
VALID
OCCLUDED
COMPLETED
LOST
```

当前关键参数：

```python
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
OBJECT_AXIS_LOCK_MIN_DELTA = 0.10
OBJECT_AXIS_LOCK_RATIO = 2.0
OBJECT_AXIS_LOCK_FRAMES = 2
```

### 6.4 动态对象跟踪与物理约束

已经实现的基础能力：

- 首帧为箱子和炸弹分配稳定ID。
- 车辆附近对象激活，远离车辆后冻结。
- 按对象类型分别做最近邻关联。
- 单轴运动检测和X/Y轴锁定。
- 箱子进入墙体时拒绝更新。
- 炸弹进入边界墙时拒绝更新。
- 其他动态物体阻挡当前对象，禁止连锁推动。
- 对象漏检后转入OCCLUDED/LOST，而不是立即删除。
- 冻结位置使用历史和稳定速度判断。

这些逻辑已经写入代码，但仍需通过真实推箱子和炸弹测试继续验证。

### 6.5 箱子进入目标点事件

已经实现两条验证路径：

#### 跟踪候选路径

1. 根据箱子最近运动轴和方向推导前方目标。
2. 箱子丢失后保留目标候选。
3. 检查目标区域是否变为高置信地面。
4. 地面连续确认后判为匹配成功。
5. 目标仍存在则判为不匹配遮挡。

#### 全局目标消失兜底

当箱子运动过快、历史不足或局部候选没有建立时：

1. 收集当前丢失箱子。
2. 检查所有未完成目标区域。
3. 某目标连续出现地面证据时，将其分配给最合理的丢失箱子。
4. 提交匹配完成事件。

事件名称：

```text
BOX_GOAL_MATCHED
BOX_GOAL_MISMATCHED
```

匹配成功会：

- 将箱子设为 `REMOVED / COMPLETED`。
- 从静态目标列表移除该目标。
- 增加 `map_version`。
- 发送事件和一帧新的FULL_MAP。

性能优化前曾成功检测一次慢推箱子到目标：

```text
EVENT name=BOX_GOAL_MATCHED ...
```

但快速推箱子时曾因帧率约 `0.9 FPS` 漏失过程。当前帧率已提升，必须重新测试。

### 6.6 爆炸事务框架

当前：

```python
EXPLOSION_EVENTS_ENABLED = True
EXPLOSION_MISSING_FRAMES = 2
EXPLOSION_VERIFY_FRAMES = 2
EXPLOSION_PENDING_MAX_FRAMES = 20
EXPLOSION_INNER_FLOOR_RATIO_Q8 = 154
EXPLOSION_OUTER_MATCH_RATIO_Q8 = 192
```

已实现：

1. 根据炸弹位置、运动轴和墙体关系推导爆炸中心。
2. 复制当前静态地图为候选地图。
3. 候选中心 `3×3` 内清除非边界墙。
4. 对 `5×5` 范围检查内部地面变化和外圈保持。
5. 验证成功后原子提交地图。
6. `map_version` 增加。
7. 炸弹标记为完成。
8. 输出提交、取消或回滚事件。

相关事件：

```text
BOMB_EXPLOSION_PENDING
BOMB_EXPLOSION_COMMITTED
BOMB_EXPLOSION_CANCELLED
BOMB_EXPLOSION_ROLLBACK
```

注意：框架已写入，但尚未完成真实完整爆炸流程的系统性实测。

### 6.7 车头车尾与姿态

当前参数：

```python
MIN_CAR_PIXELS = 4
MAX_CAR_MARKER_PIXELS = 220

CAR_MARKER_SCORE_MIN = 58
CAR_MARKER_SCORE_MARGIN = 6
CAR_MARKER_MIN_DIST = 0.12
CAR_MARKER_MAX_DIST = 1.20
CAR_MARKER_TARGET_DIST = 0.35

CAR_NEAR_BACK_RESCUE = True
CAR_RESCUE_CYAN_SCORE_MIN = 58
CAR_RESCUE_SCORE_MARGIN = 6
```

已实现：

- 青色车头、绿色车尾混合RGB/LAB评分。
- 品红目标排除。
- 候选组件大小过滤。
- 车头车尾距离配对。
- 前后标记历史连续性，降低180度翻转。
- 绿色车尾附近的青色车头救援搜索。
- 位置和角度平滑。

平滑参数：

```python
CAR_POSE_SMOOTH_ALPHA = 0.45
CAR_THETA_SMOOTH_ALPHA = 0.18
CAR_THETA_MAX_STEP_DEG = 6.0
CAR_POSE_SMOOTH_MAX_DIST = 1.2
```

### 6.8 调试输出策略

当前目标是关闭图传，保留IDE串口诊断。

```python
DRAW_DEBUG_OVERLAY = False
DRAW_GRID_OVERLAY = False
DRAW_GRID_POINTS = False
DRAW_CELL_CENTERS = False

PERIODIC_FULL_MAP_DEBUG = False
IDE_EVENT_DEBUG = True
IDE_INITIAL_FULL_MAP = True
IDE_EVENT_FULL_MAP = True
IDE_GOAL_TRACE = True
IDE_FPS_DEBUG = True
IDE_PERF_DEBUG = True
```

行为：

- 启动后输出一次首帧FULL_MAP。
- 后续不周期输出FULL_MAP。
- 快速帧仍持续抓图、分析和维护状态。
- 事件发生时输出事件名和一帧FULL_MAP。
- 每秒输出FPS和分阶段PERF诊断。

“静默”指不再周期发送全图，不代表停止分析，也不代表当前完全没有FPS/PERF文本。

---

## 7. 当前协议实际状态

协议仍保持旧MCU布局，尚未完成总体规划中的协议升级。

包类型：

```python
FULL_MAP   = 0x01
POS_UPDATE = 0x02
MAP_REQUEST = 0x03
HEARTBEAT  = 0x04
```

串口：

```python
UART_BAUDRATE = 115200
UART_CANDIDATES = (5, 12, 11)
HEARTBEAT_PERIOD_MS = 500
POS_UPDATE_PERIOD_MS = 50
```

当前会尝试打开UART5、UART12、UART11，并向已打开端口广播。

### 7.1 FULL_MAP

当前固定为旧格式，函数注释标记59字节。

包含：

- `16×12` 墙体位图。
- 箱子整格坐标。
- 目标整格坐标。
- 最多2个炸弹槽位。
- 车辆毫米坐标和 `0.1度` 角度。
- XOR校验。

### 7.2 POS_UPDATE

当前仍是19字节payload兼容布局：

- `frame_id`
- 车辆毫米坐标和角度
- 最多3个箱子整格坐标
- 炸弹数量固定写0
- XOR校验

重要限制：

- 当前动态对象内部是0.1格坐标，但旧协议输出仍压回整格/毫米兼容格式。
- 当前协议没有稳定对象ID、对象状态和 `map_version` 字段。
- `VISION_MAX_BOMBS=3`，但旧MCU FULL_MAP只有2个炸弹槽。
- `POS_UPDATE` 当前不发送炸弹。
- 总体规划中的新协议仍待后续双方联调。

虽然 `POS_UPDATE_PERIOD_MS=50`，实际发送频率受视觉主循环限制。当前空地图快速帧约 `8～12 FPS`，因此实际不会稳定达到20 Hz。

---

## 8. 性能优化完整记录

### 8.1 性能优化前基线

关闭图传后仍只有：

```text
正常快速帧：约0.7～1.4 FPS
常见值：约0.9 FPS
首帧高质量建图：复杂场景曾约8.9 s
```

分段统计显示正常帧主要耗时在 `region` 动态采样分类。

当前性能统计项：

```text
snapshot
region
components
fallback
tracking
fullmap
loop
```

### 8.2 预计算19200个动态采样点

开关：

```python
USE_PRECOMPUTED_SAMPLE_COORDS = True
```

内容：

- 对 `160×120 = 19200` 个0.1格采样点预计算屏幕X/Y和可见性。
- 使用4个bytearray保存，约占 `76800 B`。
- 继续使用原亮度增益和颜色分类器。
- 关闭开关可回到旧实时几何计算路径。

等价验证：

- 19200点坐标和可见性逐点一致。
- 模拟全图和ROI分类结果逐字节一致。

实机收益：

```text
优化前 region：约815～834 ms
优化后 region：约208～215 ms
FPS：约1.1 -> 3.5～3.8
```

这是当前最大的一项严格等价收益。

### 8.3 车辆颜色分类严格等价短路

优化内容：

- 在调用LAB前先计算原最终阶段已有的RGB硬门。
- 青色和绿色都不可能时跳过LAB。
- 只可能一种身份时只计算对应LAB分数。

验证：

- 对全部65536种RGB565颜色比较新旧输出，完全一致。
- 模拟LAB调用减少约53%。

实机收益很小：

```text
region仍约204～210 ms
```

结论：

- 保留。
- 不继续围绕该方向派生修改。

### 8.4 单遍多类别组件提取

开关：

```python
USE_MULTI_CLASS_COMPONENTS = True
```

旧流程对同一ROI分类图分别遍历5次：

```text
车头
车尾
箱子
红炸弹
暗炸弹
```

新流程一次遍历提取全部类别。

验证：

- 120组随机分类图的新旧组件结果完全一致。
- 组件数量、4邻域、像素数、质心、边界框和排序一致。

实机收益：

```text
components：约25～26 ms -> 8.8～9.3 ms
FPS：约3.3～3.6 -> 3.7～3.9
```

### 8.5 分类与访问缓冲复用

开关：

```python
USE_REUSABLE_REGION_BUFFERS = True
```

实现：

- 复用一个19200字节分类缓冲。
- 复用一个19200字节访问缓冲。
- 使用1～255代际标记避免每帧清零。
- 代际回绕时重建访问缓冲。

验证：

- 连续320次组件提取包含代际回绕，结果一致。
- 连续不同尺寸ROI分类逐字节一致。

实机平均FPS收益不明显：

```text
中央约3.7～4.0 FPS
region仍约207～214 ms
```

结论：

- 保留以减少持续分配和潜在GC压力。
- 不计作主要平均FPS收益。

### 8.6 自适应快速ROI

开关：

```python
USE_ADAPTIVE_FAST_RADIUS = True
```

参数：

```python
CAR_TRACK_RADIUS_CELLS = 2.5
NEAR_OBJECT_PREPARE_RADIUS_CELLS = 4.0
OBJECT_TRACK_RADIUS_CELLS = 2.2
OBJECT_ROI_WAKE_MARGIN_CELLS = 1.0
```

逻辑：

- 尚未建立对象轨迹时仍用4格范围。
- 没有未移除对象在车辆5格内时，车辆使用2.5格ROI。
- 对象进入5格唤醒范围时，恢复4格物体准备ROI。
- 不降低ROI内部采样密度。

空地图收益：

```text
中央 region：约207～214 ms -> 82～84 ms
中央 FPS：约3.7～4.0 -> 约8.1
边缘裁剪区域：约9.3～11 FPS
近期正常帧：常见8～12 FPS
```

该优化的空地图性能收益已经确认。

单箱单目标统一ROI版本实测：

- 事件链路成功触发 `BOX_GOAL_MATCHED`。
- 事件后 `boxes=0 / goals=0`，状态移除正确。
- 推动阶段仍主要靠 `GLOBAL_GOAL_ASSIGN` 兜底，局部候选跟踪不够稳定。
- 车辆接近/推动箱子时FPS约 `3.3～3.9`，`region≈160～204 ms`。

### 8.6.1 车辆ROI与对象ROI首版拆分

当前新增：

```python
USE_SEPARATE_OBJECT_ROIS = True
OBJECT_TRACK_RADIUS_CELLS = 2.2
```

逻辑：

- 车辆识别固定使用 `2.5` 格小ROI。
- 当未移除对象进入车辆 `5.0` 格唤醒范围时，以对象轨迹中心单独扫描箱子/炸弹ROI。
- 对象ROI观测与车辆ROI已有观测去重合并，再交给原跟踪、目标事件和爆炸事务逻辑。
- `update_object_tracks()` 的关联搜索范围扩展到唤醒区，但只有进入原 `1.8` 格激活/退出半径内才会因漏检累加 miss。
- 本次未修改颜色阈值、采样密度、连通规则、目标事件判定或全图回退策略。

尚未确认：

- 单箱存在时5格唤醒是否足够及时。
- 接近箱子时独立对象ROI是否能把推动阶段FPS提高到明显优于 `3.3～3.9`。
- 推动过程是否漏跟踪。
- 是否能更早建立 `GOAL_CANDIDATE`，减少依赖 `GLOBAL_GOAL_ASSIGN` 兜底。

单箱单目标远近距离实测已通过：

- 5格外纯车辆小ROI约 `9.4～9.6 FPS`，`region≈59 ms`。
- 进入唤醒区约 `7.8 FPS`，`region≈87 ms`。
- 推动阶段约 `5.2 FPS`，`region≈146 ms`。
- 能稳定出现 `GOAL_CANDIDATE` 并触发 `BOX_GOAL_MATCHED`。

多箱测试发现：

- 多个对象ROI同时唤醒后，推动阶段约 `3.6～4.5 FPS`，`region≈173～218 ms`。
- 一个错误事件把 `box=2` 分配到了远处旧目标 `(3,1)`，而不是当前消失证据对应的目标。
- 原因是全局目标消失兜底的floor计数跨事件残留，且无候选箱子的兜底分配过宽。

已修正：

```python
GLOBAL_GOAL_ASSIGN_MAX_DIST = 3.0
GLOBAL_GOAL_ASSIGN_FORWARD_LOOKAHEAD = 4.0
GLOBAL_GOAL_ASSIGN_LATERAL_TOLERANCE = 1.25
OBJECT_ROI_MAX_TRACKS = 2
```

- 任意 `commit_box_goal_match()` 后调用 `reset_global_goal_evidence()` 清空全局目标消失计数。
- 无显式 `GOAL_CANDIDATE` 时，`GLOBAL_GOAL_ASSIGN` 必须满足距离或运动方向合理性。
- 若目标消失确认但没有合理箱子可分配，输出 `TRACE GLOBAL_GOAL_NO_ASSIGN` 并清掉该目标计数。
- 每帧最多扫描2个最相关的对象ROI：正在活动/推动的对象优先，否则选择离车最近的对象。
- `GOAL_CANDIDATE` 本身不抢对象ROI名额，避免旧候选在连续推箱时长期占用扫描窗口。
- 历史上曾设为 `OBJECT_ROI_MAX_TRACKS=1` 以压低多对象ROI开销；多箱假动作复测证明偏窄，当前优先恢复连续跟踪灵敏度。

关掉IDE图传后的多箱复测：

- 远离/轻载约 `9.6～10.0 FPS`，`region≈66～70 ms`。
- 推动阶段常见 `5.4～6.1 FPS`，`region≈134～148 ms`。
- 仍有漏检：箱子已被跟踪到目标格，例如 `boxes=1 box0=(3,1)` 且 `goals=1 goal0=(3,1)`，但轨迹仍为 `VALID`，没有进入原 missing 验证分支。

已新增：

- `track_is_on_goal_candidate()`。
- 当箱子轨迹贴近候选目标中心时，即使还不是 missing，也进入原 `GOAL_VERIFY`。
- 强地面证据提交 `BOX_GOAL_MATCHED`；目标仍可见则保留原不匹配判断。
- 车或其他对象遮挡时仍由 `GOAL_VERIFY_BLOCKED` 阻止提交。

进一步复测后发现：

- 有时左上箱已有 `GOAL_CANDIDATE goal=(3,1)`，目标 `(3,1)` 也出现 `evidence=FLOOR`，但该箱仍是 `VALID` 且停在 `(3,2)` 附近。
- 原全局目标消失兜底只处理 missing 箱子，因此不会把这个显式候选箱子拿来提交完成。

已新增：

- `unresolved_goal_completion_boxes()`。
- 全局目标消失兜底候选包含：
  - `missing >= BOX_GOAL_MISSING_FRAMES` 的箱子；
  - 已有显式 `GOAL_CANDIDATE` 的未完成箱子。
- trace中的 `missing` 字段改为 `candidates`。
- 显式候选目标等于消失目标时优先分配；否则仍走距离和运动方向合理性门槛。

继续复测后发现：

- 右侧目标 `(13,3)` 曾被识别为 `FLOOR`，但当时候选池只有别的箱子，因此输出 `GLOBAL_GOAL_NO_ASSIGN` 后清掉计数。
- 后续 `box2` 停在 `(13,5)` 附近但没有显式候选，原候选池不会继续监听该目标，导致残留。

已新增：

- `track_can_complete_any_global_goal()`。
- 对任一剩余目标通过 `global_goal_track_score()` 合理性评分的近目标箱子，也加入全局目标消失候选池。
- 没有显式候选时仍必须通过距离/运动方向门槛，不恢复旧的无约束误配。

打乱推箱顺序后发现：

- 开局未推箱时也会出现 `GLOBAL_GOAL ... candidates=2`。
- 原因是初始箱子与上方目标相距约2格，满足近目标距离门槛。

已收窄：

- 没有显式 `GOAL_CANDIDATE` 的近目标箱子，必须通过 `recent_motion_axis_direction()` 得到明确运动轴和方向，才允许进入全局目标消失候选池。
- 初始静止箱子不应再触发全局目标监听；推动过但未建候选的箱子仍可参与兜底。

进一步优化：

- 新增 `relevant_global_goals_for_candidates()`。
- 全局目标消失验证不再遍历所有 `g_static_goals`。
- 显式 `GOAL_CANDIDATE` 只验证自己的候选目标。
- 无显式候选时，只验证通过 `global_goal_track_score()` 的相关目标。
- 不相关目标的全局floor计数和trace状态会被清零，避免旧证据残留。
- 本步是保质量提帧率：不改颜色阈值、采样密度、ROI半径或事件阈值，只减少无效目标验证。

假动作复测后又发现：

- `GLOBAL_GOAL_ASSIGN` 仍会把“目标点变地面”过早归因给没有对应候选的箱子。
- 典型情况是箱子原本有另一个候选目标，或只是近距离/运动方向评分通过，却被分配到另一个目标。
- 这说明全局目标消失兜底不能继续作为“推测完成”路径，否则假动作越多越容易误提交。

当前已改为：

```python
GLOBAL_GOAL_CONFIRMATION_ONLY = True
GLOBAL_GOAL_EXPLICIT_ASSIGN_RADIUS = 1.35
```

含义：

- 全局目标消失只确认已有显式 `GOAL_CANDIDATE` 的箱子。
- 没有显式候选的箱子不进入全局目标消失候选池。
- 有别的候选目标的箱子不会被分配给另一个目标。
- 显式候选提交需满足 missing、已进入目标格，或距候选目标中心不超过 `1.35` 格。
- 旧的近目标/运动方向评分逻辑暂时保留在代码中，但在 `GLOBAL_GOAL_CONFIRMATION_ONLY=True` 时不负责提交事件。

这一步不是为了提高表面成功率，而是为了先消除假动作误判。后续主线应回到连续对象跟踪和候选建立稳定性，而不是继续增强全局猜测。

随后三箱假动作复测显示：

- 假动作误提交被压住后，真实完成漏检主要表现为 `BOX_MISSING_NO_GOAL`。
- 右侧箱子常需要先横推再竖推到 `(13,3)`，旧轴锁定在这种路径上会延续上一段轴，导致转向后没有建立目标候选。

当前已补强：

```python
OBJECT_ROI_MAX_TRACKS = 2
OBJECT_ROI_SECONDARY_PERIOD = 2
OBJECT_AXIS_SWITCH_FRAMES = 2
```

含义：

- 每帧最多扫描2个相关对象ROI，降低多箱假动作时当前箱子拿不到ROI的概率。
- 最相关对象ROI每帧扫描，第二对象ROI当前隔帧扫描，以回收一部分 `OBJECT_ROI_MAX_TRACKS=2` 带来的计算量。
- 已锁定X轴时，若连续2帧看到强Y向运动，允许切换到Y轴。
- 已锁定Y轴时，若连续2帧看到强X向运动，允许切换到X轴。
- 切轴仍要求原来的运动量和方向比例门槛，不因抖动切换。
- 远离旧候选目标时，`BOX_GOAL_MATCHED` 和 `BOX_GOAL_MISMATCHED` 不再直接提交；会输出 `GOAL_VERIFY_STALE` 并清掉旧候选，等待重新建立正确候选。
- 本步仍需实测确认FPS和候选稳定性。

### 8.7 已撤回：旧位置4格分级恢复

曾尝试：

```text
2.5格ROI丢车
→ 以上一位置做4格局部搜索
→ 再失败才全图
```

实测：

```text
4次局部恢复全部失败
成功率0/4
回退耗时由约500 ms增加到612～624 ms
```

该路线已完整撤回。

不要重新沿旧位置盲目扩大半径，除非有新的证据。

### 8.8 时间速度预测ROI

开关：

```python
USE_PREDICTED_CAR_RECOVERY = True
```

首版问题：

- 使用格/帧速度。
- 第一次漏检后车辆位置保持旧值，却把速度重算为0。
- 之后无法继续预测。

修正后：

```python
CAR_PREDICTION_TIME_GAIN = 1.15
CAR_PREDICTION_MIN_SPEED_PER_MS = 0.0008
CAR_PREDICTION_MAX_ELAPSED_MS = 1500
```

当前行为：

- 速度单位为格/ms。
- 漏检时保留最后可信速度和最后观测时间。
- 按距最后观测的真实毫秒数外推。
- 预测时间乘1.15补偿。
- 超过1500 ms不再继续增加外推距离。
- 速度太低时跳过预测。
- 预测失败仍保留全图兜底。

实测：

```text
约13次预测中成功1次
成功率约8%
```

结论：

- 有过真实召回案例，成本相对低，暂时保留。
- 不再优先调预测倍率。

### 8.9 连续全图回退限频

参数：

```python
CAR_FULL_FALLBACK_RETRY_MS = 400
```

问题来源：

```text
首次漏检
→ 每帧执行约500 ms全图回退
→ 帧率跌到1.6～1.7
→ 相邻图像时间更长、车辆跨帧距离更大
→ 更容易继续漏检
```

当前策略：

1. 每次新的失联事件仍立即允许一次全图扫描。
2. 若失败，继续运行正常ROI和预测ROI。
3. 距上次全图扫描结束不足400 ms时跳过重复全图。
4. 超过400 ms后再允许全图。
5. 车辆一旦找回，冷却立即清零。

当前诊断计数：

```text
prediction_count
prediction_success
full_fallback_count
full_fallback_success
full_fallback_skip
```

最新实测：

```text
首次全图失败
后续窗口 full_fallback_skip=5
再后续窗口 full_fallback_skip=2
第二次允许的全图扫描成功找回车辆

失联最低窗口：
原约1.6～1.7 FPS
提升到约4.6 FPS

恢复后：
立即回到约8～12 FPS
```

结论：

- 连续全图扫描造成的低帧率反馈链已经被切断。
- 该项完成。

---

## 9. 当前性能状态

### 9.1 首帧

空地图近期日志：

```text
首帧总循环约3.6 s
其中 fullmap约3.1 s
region约0.48 s
```

复杂地图和验证格较多时可能更慢，历史上曾约8.9 s。

第一帧允许较慢，暂时不是当前优化重点。

### 9.2 正常空地图快速帧

根据车辆位置和ROI裁剪程度：

```text
常见FPS：8～10
边缘或裁剪较多时：10～12
region：约40～85 ms
components：约3～5 ms
loop：约70～120 ms
```

### 9.3 车辆失联

当前：

- 首次全图回退仍约500 ms。
- 但不会再每帧连续执行。
- 失联窗口实测最低约4.6 FPS。
- 第二次允许的全图回退可以成功找回车辆。

---

## 10. 当前已知风险与未完成验证

按优先级排列：

### 10.1 动态对象ROI尚未在优化后复测

下一项必须验证：

- 单箱在车辆远处时被冻结并保持输出。
- 车辆进入5格唤醒范围时切回4格ROI。
- 进入1.5格激活范围后持续跟踪。
- 推动时不漏位置。
- 离开1.8格后正确冻结。

### 10.2 箱子目标事件需在高帧率版本复测

旧低帧率版本：

- 慢推曾成功触发事件。
- 快推容易丢失。

当前快速帧提高后，应重新验证：

```text
BOX_GOAL_MATCHED
BOX_GOAL_MISMATCHED
```

### 10.3 爆炸事务尚未完整实测

代码框架存在，但必须分别验证：

- 炸弹进入普通墙。
- 部分遮挡。
- 爆炸中心推导。
- 3×3清墙。
- 5×5验证。
- 边界墙不被清除。
- 失败时回滚。

### 10.4 当前协议仍是旧兼容格式

视觉内部已有：

- 稳定对象ID。
- 对象状态。
- 地图版本。
- 0.1格动态坐标。

但UART尚未完整传出这些信息。

完整系统联调前必须按总体规划升级双方协议，或至少确认首条可运行链路需要哪些字段。

### 10.5 全图回退本身仍昂贵

当前限频解决了反馈链，但单次全图动态扫描仍约500 ms。

后续可评估：

- 只为找车执行车辆类别全图搜索，而不是五类全部分类。
- 底层 `find_blobs()` 作为候选搜索。
- 分块或交错全图搜索。

任何替代方案必须保留最终召回能力。

### 10.6 `find_blobs()` 尚未评估

固定性能路线中的后续项：

```text
评估底层 find_blobs() 候选搜索
```

不要直接替换现有分类器。应先做旁路候选召回率测试。

---

## 11. 下一项测试

测试场景：

```text
空地图
+ 3个箱子
+ 3个目标点
+ 目标点与箱子打乱对应关系
```

操作顺序：

1. 保持IDE图传关闭，只保留串口/FPS/PERF输出。
2. 先做与上一轮相同的打乱配对加假动作。
3. 可以把某个箱子推到目标点旁边后再推走。
4. 再把箱子推向其他目标点。
5. 保留从首帧到最后一次事件或明显漏检后的完整IDE输出。

观察重点：

### 11.1 全局兜底是否还误提交

预期：

```text
没有对应 GOAL_CANDIDATE 的箱子不应触发 GLOBAL_GOAL_ASSIGN
有其他候选目标的箱子不应被分配到另一个目标
```

如果出现：

```text
TRACE GLOBAL_GOAL_NO_ASSIGN
```

这通常表示保护生效，不一定是坏事。需要看是否因此留下一个真实已完成但没有候选的箱子。

### 11.2 候选是否建立

重点看每次真实推向目标时是否出现：

```text
TRACE GOAL_CANDIDATE box=... goal=(...)
```

如果真实完成没有事件，并且日志里也没有对应 `GOAL_CANDIDATE`，下一步应优化连续对象跟踪和候选建立，而不是放宽全局兜底。

如果出现：

```text
TRACE GOAL_VERIFY_STALE ...
```

表示旧候选已经离当前箱子太远，程序会主动清掉该候选。重点观察之后是否能重新建立正确候选。

### 11.3 帧率与耗时

检查：

- 轻载/远离对象时FPS是否仍能到约 `8～10`。
- 推动阶段是否从上一轮的约 `3～4 FPS` 回升，目标仍是尽量接近 `5～6 FPS`。
- `tracking` 是否不再因为无关目标验证长期升高。
- `region` 是否从上一轮常见 `190～215 ms` 降下来。

### 11.4 最终判断

理想结果：

```text
假动作不误提交
真实推入目标且有 GOAL_CANDIDATE 时仍提交 BOX_GOAL_MATCHED
如果漏检，能够明确看到是候选没建立或连续跟踪丢失
```

当前宁可暴露“候选没建立”的问题，也不要靠全局目标消失继续猜测完成。


---

## 12. 后续性能优化固定路线

不要因单次日志偏离以下主路线：

1. 已完成：分段性能统计。
2. 已完成：连续全图回退限频。
3. 已完成：预计算动态采样坐标。
4. 已完成：单遍多类别组件提取。
5. 已完成：分类和访问缓冲复用。
6. 进行中：拆分车辆与动态对象独立ROI。
7. 待评估：底层 `find_blobs()` 候选搜索。
8. 最后才考虑：降低采样密度或简化分类。

第6项当前已经完成两个子步骤：

```text
1. 根据是否有动态对象靠近选择2.5格或4.0格统一ROI
2. 车辆2.5格ROI + 对象轨迹中心2.2格ROI的首版拆分
```

首版拆分已经写入代码，尚未实机验证收益和跟踪质量。

---

## 13. 建议的恢复检查清单

下次开始时按以下顺序：

1. 确认当前运行文件是 `vision_staging\boot.py`。
2. 核对 `boot.py` 与 `main.py` 哈希一致。
3. 不要误用 `openart` 或 `G:\` 的旧副本覆盖 staging。
4. 检查性能优化进度文档最后一项。
5. 先执行单箱单目标测试，不直接上复杂地图。
6. 将完整FPS/PERF和事件输出保存为附件。
7. 只根据本轮目标修改一项主行为。
8. 修改后同步 boot/main，并做语法检查。
9. 记录实测收益或负收益。
10. 确认无回归后再进入下一项。

---

## 14. 常用验证命令

语法检查：

```powershell
python -m py_compile `
  ".\vision_staging\boot.py" `
  ".\vision_staging\main.py"
```

核对 staging 双入口：

```powershell
Get-FileHash -Algorithm SHA256 -LiteralPath `
  ".\vision_staging\boot.py", `
  ".\vision_staging\main.py"
```

当前预期哈希：

```text
8E5B25151B6F963CAC626B251BA2183170866EF2B4F9FA30974141D961005121
```

同步到设备前核对所有入口：

```powershell
$paths = @(
  ".\vision_staging\boot.py",
  ".\vision_staging\main.py",
  "C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\boot.py",
  "C:\Users\L\Desktop\智能视觉完赛 (2) - 副本\智能视觉完赛\重构代码\openart\main.py",
  "G:\boot.py",
  "G:\main.py",
  "G:\sd\main.py"
)
Get-FileHash -Algorithm SHA256 -LiteralPath $paths
```

---

## 15. 当前关键开关汇总

```python
# 标定
GEOMETRY_PROFILE = GEOMETRY_PROFILE_INNER
CALIBRATION_PROFILE = CALIBRATION_PROFILE_NEW

# 图传与IDE
DRAW_DEBUG_OVERLAY = False
DRAW_GRID_OVERLAY = False
DRAW_GRID_POINTS = False
DRAW_CELL_CENTERS = False
PERIODIC_FULL_MAP_DEBUG = False
IDE_EVENT_DEBUG = True
IDE_INITIAL_FULL_MAP = True
IDE_EVENT_FULL_MAP = True
IDE_GOAL_TRACE = True
IDE_FPS_DEBUG = True
IDE_PERF_DEBUG = True

# 性能
USE_PRECOMPUTED_SAMPLE_COORDS = True
USE_MULTI_CLASS_COMPONENTS = True
USE_REUSABLE_REGION_BUFFERS = True
USE_ADAPTIVE_FAST_RADIUS = True
USE_SEPARATE_OBJECT_ROIS = True
USE_PREDICTED_CAR_RECOVERY = True

# 分层状态
STATIC_MAP_LOCK = True
STATIC_WALL_POST_APPLY = False
LAYERED_STATE_ENABLED = True
NEAR_OBJECT_TRACKING = True
EXPLOSION_EVENTS_ENABLED = True

# ROI与回退
CAR_TRACK_RADIUS_CELLS = 2.5
NEAR_OBJECT_PREPARE_RADIUS_CELLS = 4.0
OBJECT_TRACK_RADIUS_CELLS = 2.2
OBJECT_ROI_MAX_TRACKS = 2
OBJECT_ROI_SECONDARY_PERIOD = 2
OBJECT_ROI_WAKE_MARGIN_CELLS = 1.0
OBJECT_AXIS_SWITCH_FRAMES = 2
CAR_PREDICTION_TIME_GAIN = 1.15
CAR_PREDICTION_MIN_SPEED_PER_MS = 0.0008
CAR_PREDICTION_MAX_ELAPSED_MS = 1500
CAR_FULL_FALLBACK_RETRY_MS = 400
```

---

## 16. 不应重新引入的旧路线

- 不要按固定地图坐标补墙。
- 不要重新启用弱墙结构补偿作为正式结果。
- 不要用车头朝向判断推动方向。
- 不要把动态物体不可见直接当作完成或删除。
- 不要让动态对象反向污染静态地图。
- 不要每帧重新做高质量静态全图建图。
- 不要在连续失联时每帧执行约500 ms全图回退。
- 不要继续使用旧位置4格局部恢复；实测成功率为0。
- 不要一次修改多项性能策略后只看总FPS。
- 不要在未验证召回率前降低采样密度。

---

## 17. 最终接手原则

```text
静态层保存地图事实。
动态层保存对象身份和运动状态。
车辆层保持最高更新频率。
输出时合成，内部状态不互相覆盖。

首帧允许慢，但必须准确。
快速帧必须持续分析，而不是静默停机。
高成本全图扫描是兜底，不是常规循环。

准确度、灵敏度、连续性和延迟同等属于质量。
先做严格等价优化，再做ROI优化，最后才考虑牺牲质量。

每次只改一项。
每次都记录数据。
负收益路线及时撤回。
所有规则必须适用于未知比赛地图。
```
