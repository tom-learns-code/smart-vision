---
name: csolver-integration-plan
description: c_solver集成到RT1064项目的完整方案、优化历程、经验总结、代码改造记录
metadata: 
  node_type: memory
  type: project
  updated: 2026-06-28
  originSessionId: fc6b8d4d-8aed-4fba-b0c9-9c841dcbc1cc
---

# c_solver 集成 RT1064 项目 — 完整记忆

## 一、两套代码库位置

| 项目 | 路径 | 说明 |
|------|------|------|
| c_solver (Python) | `project_test_5/c_solver/` | track.c 逐行翻译，3030行，联合A*搜索 |
| RT1064 整车项目 | `比赛代码/1.1 上赛道test - 副本/project/code/` | solver.c/h + path_follower.c/h + vision_parser.c/h + main.c |

## 二、c_solver 核心特点

- **算法**: 联合A*（非8步管线），在一个搜索中同时处理推弹+推箱+炸墙
- **入口**: `run_phase1(raw_map, p1_res, mode, skip_flag)` + `run_phase2(box_ids, target_ids, p2_res)`
- **阶段1输出**: 炸弹坐标、P1动作码序列、终点位置、updated_map
- **阶段2输出**: P2动作码序列、终点（基地）位置
- **动作码**: 0-3=移动, 4=爆炸, 5-8=观察(上下左右)
- **TRACK_NUM**: 控制"看几个就停"的阈值，4=简单模式

**Why**: 这个算法来自比赛对手的C代码，能解Python 8步管线解不了的地图。[[python-pipeline-failure-analysis]] 记录了对比。

## 三、Python版优化历程（已完成的7轮）

| 轮次 | 内容 | 结果 | 总计加速 |
|------|------|------|---------|
| P2 | BFS位掩码(vis_rows替代reach_vis) | ✅ PASS | 490.9→449.7s (-41.2s) |
| A | P2 4D展平(p2_vis 1D数组) | ✅ PASS | 449.7→441.0s (-8.7s) |
| C | 状态复用池(避免list()分配) | ✅ PASS | 441.0→437.1s (-3.9s) |
| S1 | f值提前终止(循环内break) | ✅ PASS | 437.1→434.8s (-2.3s) |
| S2 | pool按关键性排序 | ✅ PASS | 434.8→432.3s (-2.5s) |
| P1 | 死锁检测 | ❌ 回退(map1(4)宽松FAIL) | — |
| P3 | 排列启发式 | ❌ 回退(瓶颈-4.9s) | — |
| B | 入口剪枝 | ❌ 回退(瓶颈-12.5s) | — |
| S3 | P2代价传入启发式 | ❌ 回退(瓶颈-4.3s) | — |

**评估标准**: 以瓶颈图map2(4)为锚点，减速=回退。不以总时间为唯一标准。
**瓶颈图**: map2(4) Python 189.1s→165.1s (-12.7%)，对应C版 7.39s→预期~6.2s。

## 四、比赛规则关键点

- 地图16×12，`#=墙 @=车 *=炸弹 $=推箱 .=目标`
- 炸弹炸3×3内部墙，外围墙不可炸
- 10类卡通图片→对应数字0-9，箱子图片和目标任务数字必须匹配
- **三种游戏模式**（非阶段）: (1)纯推箱 (2)按图分类(目标排一排) (3)升级版(目标随机)
- 转向只在识别箱子/目标时需要，推物全向不需要转向
- 未完成箱子罚30秒，必须完成≥50%

## 五、双摄像头架构

| 摄像头 | 串口 | 职责 | 协议 |
|--------|------|------|------|
| OpenART1 | UART5 | 地图识别+位置跟踪 | 0x01(全图)/0x02(位置)/0x03(MCU请求) |
| OpenART2 | 新串口(UART6?) | 箱子图片分类+目标数字识别 | 0x10(识箱子)/0x11(识目标)/0x12(回复) |

识别请求MCU主动发，OpenART2跑对应模型后回复。槽位号放在请求中，回复时回显。

## 六、集成方案架构

```
状态机: WAIT_MAP → EXEC_P1 → WAIT_RECOG → EXEC_P2 → DONE

阶段1: solve_phase1() → P1动作(push_bomb+observe+free_move) → path_follower执行
  过渡: 发0x03请求新全图 → 等0x01 → 提取最新箱子/目标位置 + box_ids/target_ids匹配
阶段2: solve_phase2() → P2动作(push_box+free_move) → path_follower执行 → DONE
```

### 改动文件清单

| 文件 | 操作 | 内容 |
|------|------|------|
| solver.h | 修改 | phase1/2_output_t, observe_meta_t, ACTION_OBSERVE |
| solver_adapter.c/h | 新建 | 桥接层(P1/P2输出→action_t[])，**可合并入c_solver省掉** |
| path_follower.c/h | 修改 | observe状态(APPROACH→FACE→RECOGNIZE), push_bomb复用PUSHING |
| main.c | 修改 | 5状态机 + box_ids/target_ids管理 |
| vision_parser | 不改 | 接口够用 |
| motion_control | 不改 | 通用 |

### observe执行流程

```
APPROACH: P控制走到观察站位(car_target)
  → 距离<ARRIVE_MM
FACE: IMU转向面对look_dir（唯一需要转向的地方）
  → |angle_error|<3°
RECOGNIZE: 发0x10/0x11请求→等0x12回复
  → 结果存入box_ids[slot]或target_ids[slot]→advance
```

### push_bomb执行流程

复用现有PUSHING状态(APPROACH+PUSH_EXEC)，到达目标墙格自动完成。下一action为WAIT 0.5s(爆炸倒计时)。

## 七、经验教训

1. **死锁检测不能过度**: 静态死锁检查没考虑"推弹炸墙后死锁解除"的动态过程，导致误杀可行组合
2. **入口剪枝的额外计算开销可能超过收益**: 每次调用的计算开销×调用次数 >> 偶尔剪掉组合省的时间
3. **排列启发式重新排序对原本就好的顺序是纯开销**: 计算距离的成本>省下的搜索时间
4. **Agent工作流不能跑Python**: 子agent无法执行Bash，测试跑图必须用主线程Bash后台
5. **import静态变量的引用陷阱**: `from module import var` 在 `reset_globals()` 重绑定后，引用指向旧对象
6. **增量评估优于总和评估**: P3在总和中被掩盖(+41.2-1.3=+39.9仍正)，但单看P3是-1.3s减速——必须逐轮对比上一轮
7. **瓶颈图锚定**: 总计加速但瓶颈图减速→要回退（B: 总计-28s但瓶颈-12.5s）
8. **转换层可省**: c_solver内部模拟回放已经知道每步做了什么，直接在那里输出结构化信息，不必事后解析

## 八、2026-06-28 代码改造记录（本次会话）

### 改造目标
1. c_solver 接口改造：raw action codes → 结构化 action（供中间层 PathFollower 消费）
2. 模拟器 ID 识别系统：地图加载随机分配 ID，observe 时"读取"，显示 "?" vs 数字
3. 两阶段衔接：P1 末尾 end_of_phase1 → 收集已发现 ID → 调 P2 → 追加 actions

### 改动的 11 个文件

**c_solver 侧（4个）:**
- `c_solver/types.py` — PhaseResult 新增 `structured_actions`, `observed_box_indices`, `observed_target_indices`
- `c_solver/action_converter.py` — **新建**。P1: 遍历 p1_acts+p1_states，检测状态变化区分 walk/push/observe/explode，累积 walk steps 输出 free_move(含waypoints)，push 输出 push_box/push_bomb，observe 输出 observe(含方向+物体类型+索引)，末尾追加 end_of_phase1。P2: 前向模拟转换（无需预存状态）
- `c_solver/track_port.py` — 新增 `g_best_p1_states`/`g_best_p2_states` 全局；在"best found"处同步保存状态序列；run_phase1/run_phase2 末尾调用 converter
- `c_solver/__init__.py` — 导出 convert_p1_actions, convert_p2_actions, TRACK_NUM

**模拟器侧（7个）:**
- `core/config.py` — 新增 `OBSERVE_DURATION = 0.3`
- `core/game_object.py` — 新增 `label_id: int`, `discovered_id: bool`；`draw()` 显示 ?/数字
- `core/target.py` — `draw()` 显示 ID 标签
- `core/map_analysis.py` — 新增 `assign_random_ids(n_boxes, n_goals)`：从 0-9 随机抽 n 个不重复→共享池→箱/目标各自 shuffle
- `core/path_follower.py` — 新增 OBSERVE 状态、`_init_observe`/`_tick_observe`、end_of_phase1 停在 IDLE 等待外部处理、`resume_after_phase1()` 跳过标记进入P2、`current_action_type()` 查询方法
- `run_solver.py` — 新增 `solve_with_c_solver()`、`solve_phase2_with_c_solver()`、`_build_raw_map()`；旧 8-step pipeline 保留不删
- `run_sim.py` — load_and_solve 集成 ID 分配+c_solver；create_objects_from_map 应用 label_ids；observe 完成后标记 discovered_id（根据方向查找目标格）；end_of_phase1 检测(IDLE状态) → 收集 known IDs → solve_phase2 → 扩展 actions → pf.resume_after_phase1()；draw_status 显示识别进度

### 关键技术决策
1. **action_converter 放在 c_solver 内部**：利用已有的 p1_states/p1_acts 序列，在 run_phase1 末尾一次性转换
2. **end_of_phase1 处理**：不自动 advance（避免 DONE），而是停在 IDLE 让主循环检测→收集ID→调P2→resume_after_phase1()
3. **bomb_id 偏移**：push_bomb 的 bomb_id = len(boxes) + bomb_index，与现有约定一致
4. **push target**：车推完后停在物体原位置（(nc, nr) = bomb/box 旧坐标），不是车原来的位置
5. **observe 'both' 处理**：当箱子在目标点上时，observe_meta.object_type='both'，主循环按方向查找被观察格上的目标点
6. **ID 分配**：箱子和目标点共享同一个从 0-9 抽出的数字池，各自独立 shuffle，保证 1-to-1 配对可能

### 新增经验
9. **end_of_phase1 的 DONE 陷阱**: 如果 _enter_action 里直接 _advance()，pf 会因为 actions 列表耗尽而进入 DONE 状态，导致外部无法追加 P2 actions。正确做法是停在 IDLE，由外部检测处理后 resume
10. **push action 的 target 语义**: target = 车推完后的位置（物体原位置），不是车推之前的位置。path_follower 用 target 计算 car_end
11. **bomb_id 偏移约定**: 整个代码库约定 bomb_id = box数量 + 炸弹索引，action_converter 必须遵守此约定
12. **observe 方向映射**: c_solver 内部 p_dir=[0,2,3,1] 映射到观察码 5-8 → 实际方向 UP/LEFT/RIGHT/DOWN，转换时必须正确映射

### 当前状态
- 所有代码改动已完成并编译验证通过
- 端到端流程已跑通（map1(4), map11 测试通过）
- **尚未在 pygame 可视化界面中实际运行测试**
- 下次上线建议：先跑 `python run_sim.py maps_import/map8.txt` 验证可视化效果，观察 ID 显示、observe 流程、P1→P2 衔接

## 九、2026-06-28/30 斜线+Dijkstra+ID修复（本次会话）

### 改造目标
1. 从 project_test_4.5.1 移植规避节点图（斜线）+ Dijkstra 微任务调度器（顺路推）
2. 修复 ID 消除法匹配（未观察箱/目标自动配对）
3. 修复爆炸时序——墙按实际爆炸顺序逐步清除
4. 各种 bug 修复

### 移植的文件（3个）
- `core/avoidance_graph.py` — 从 4.5.1 直接拷贝（547行）。格线代价改为 1.5
- `algorithms/box_planner.py` — 从 4.5.1 直接拷贝（608行）
- `algorithms/micro_scheduler.py` — 从 4.5.1 直接拷贝（290行）

### 关键修改

**`core/avoidance_graph.py`**：
- `composite_dijkstra` 格线代价 1.0→1.5
- `get_waypoints` 格线代价统一 1.5（取消软区/开放区区分代价，保留软区禁斜线逻辑）

**`run_solver.py`**：
- `_smooth_actions` 完全重写：用 AvoidanceGraph.get_waypoints（含软区/规避节点/安全连线/临时起点），按爆炸时序逐步更新墙
- `_resolve_ids` 新增：消除法推理填满未观察 ID（-1），避免 run_phase2 中 -1==-1 错误配对
- `solve_phase2_dijkstra` 混合策略：先试 Dijkstra 顺路推，push_box_bfs 失败则回退 c_solver P2 + 斜线平滑
- P1 平滑：用初始墙（非爆炸后），爆炸时序由 _smooth_actions 内部 wait action 逐步处理

**`c_solver/action_converter.py`**：
- `last_exploded_bomb` 追踪：push_bomb 爆炸时记录 raw 索引，传给 wait action 的 bomb_id
- 推箱/推弹 theta=None（全向推送）

**`run_sim.py`**：
- 启动不自动求解（load_map_only），按 R 求解、T 运行、Z 软重置
- Z 重置：pf.load_actions(result['actions']) 同一引用，保证 P2 追加后 pf 可见
- g_p1_res = None 修掉（在 do_solve 之后误清）
- 目标点显示改为空心正方形

**`core/path_follower.py`**：
- free_move 角度等待：位置到达后若 theta 非 None，原地转向到误差<5°再 advance
- 推箱超时：_push_elapsed > 2s + 物体在 1.5 格内 → 强制 advance

### 效果（map0 基准）
| action | 之前 | 之后 |
|--------|------|------|
| P1[6] (2,5)→(10,8) | 14步绕左边 | 4步 3斜线 |
| P1[0] (1,1)→(2,8) | 10步含穿墙 | 7步格线（尊重墙） |
| P1 全部 free_move | 步数多 | 28步 5斜线 |

### 新增经验
13. **规避节点起点/终点临时加入**：AvoidanceGraph.get_waypoints 把 start 临时加为节点用完即删，裸调 composite_dijkstra 没做这一步导致无斜线
14. **软区禁斜线**：箱子/炸弹周围 8 格为软障碍区，两端同在软区时捷径被禁用——这是正确的安全行为
15. **爆炸墙必须按时序**：_smooth_actions 一次性移除所有爆炸墙会导致前面的 free_move 穿过未炸的墙。wait action 的 bomb_id 用于定位爆炸位置、按实际时序逐步清除
16. **bomb_id 检测时机**：c_solver 中 push 和爆炸在同一步发生（mask 在 push 步就变了），code 4 时 mask 已变检测不到。必须在 push_bomb 处记录 last_exploded_bomb
17. **Z 重置引用一致性**：result['actions'] 和 pf.actions 必须是同一列表对象，否则 P2 追加后 pf 看不到
18. **g_p1_res 清空时机**：do_solve 通过 nonlocal 设值，后面不应再 g_p1_res = None

### 当前状态
- 所有改动完成并编译验证通过
- ID 匹配、爆炸时序、斜线均正确
- P2 箱子位置动态追踪生效

## 十、2026-07-08/09 批量测试+连锁推修复

### 新增文件
- `batch_test.py` — 批量测试脚本：每张图跑N次随机ID，输出双轨（raw+structured）路径、ID分配、连锁推检测

### 连锁推修复

**根因**：`micro_scheduler.py` `schedule_micro_tasks()` 第144-151行，`car_obs` 已收集所有未完成箱子位置作为障碍物，但只给 `car_bfs`（车能否到达站位）用，推箱路径完全不检查。

**修复**（只改 `micro_scheduler.py`）：
```python
other_boxes = {所有其他未完成箱子的当前位置}
if task.box_end in other_boxes: continue          # 推达位置被占
if 推箱过程中任何一步 nxt in other_boxes: continue  # 路径碰撞
```
加了两处约束检测，不改原有逻辑。Dijkstra 遇到碰撞就尝试其他顺序。

**效果**：4张连锁推地图全部归零（20次运行 0连锁推）。

### 批量测试结果（48次）
- ID匹配：0错误（`_resolve_ids` 消除法推理正确）
- 连锁推：修复前7次（map2(4)/map3(1)/map4(1)/map5(1)），修复后0次
- P1观察模式一致：每张图始终观察相同索引
- ~~map7 P2全部失败（独立bug，未修复）~~ **已订正**：是P1无解，非P2问题，见第十五节
- Dijkstra P2对简单地图成功，复杂图回退 c_solver P2

### 关键经验
19. **连锁推根因极简**：`car_obs = walls | all_box_positions` 已经算好了全部障碍物，car_bfs 用了一遍，推箱路径却没复用——漏一行检测导致7次碰撞
20. **Dijkstra调度器缺少碰撞检测**：旧pipeline的 `scheduler.py` 第223-248行有 `obj_path_ok` 逐格验证，micro_scheduler移植时遗漏了
21. **随机hash导致不可复现**：Python `hash()` 跨进程随机，批量测试用 `seed = hash(mapname) ^ (run*prime)` 不同进程跑出不同ID

### 当前状态
- 连锁推：已修复
- ID匹配：算法正确，模拟器显示待确认
- 斜线/爆炸时序：正确
- 待修：map7 P2失败、P1 BFS路点优化（替换为复合Dijkstra）
- 待实际 pygame 跑测验证

## 十一、2026-07-09 推箱/ID/路径多项修复

### 1. 推箱结束判定修复（path_follower.py）
**问题**：欧氏距离同时看X和Y，上次推残留的垂直偏差导致下次水平推永远达不到ARRIVE(5px)
**修复**：改为单轴判定——推DOWN只看Y，推RIGHT只看X

### 2. ID匹配修复（run_sim.py + run_solver.py）
**根因**：c_solver `dot_pos` 行优先扫描 vs 模拟器 `sorted(goals)` 列优先，同一索引指向不同目标
**修复**：目标排序统一为 `sorted(goals, key=lambda p: (p[1], p[0]))`（行优先）
**额外**：run_sim.py 箱到目标时加 `box.label_id == target.label_id` 比对，不匹配不消失

### 3. Walk BFS修复（track_port.py）
**问题**：路径重建BFS把 walk_r/walk_c（推箱站位）当障碍物，车被夹在箱和弹之间只能绕行
**修复**：`if nr == walk_r and nc == walk_c: block = False`——允许终点被物体占据

### 4. 推箱平滑（path_follower.py）
- APPROACH→PUSH不跳变：记录进入PUSH时的车速，0.3秒内线性过渡到PUSH_V=150
- 推完停顿：物体到达后车停0.15秒再进下一步

### 5. 多格推拆分（micro_scheduler.py）
**问题**：Dijkstra合并多步推→一格一顿vs连续多格，前者稳但慢、后者快但偏
**修复**：n_steps>1时拆为前N-1步合并+最后1步单推

### 6. 碰撞管线修改（已回退）
曾尝试去切向摩擦+子步轴钳制，效果不佳已恢复备份

### 7. map1(1)/map2(1) P1路径分析（此结论2026-07-10已证伪，见第十二节）
- ~~map1(1)：目标在墙下方一行，必须DOWN→RIGHT，A*路径正确~~ **错**：目标在(13,5)同行，非墙下方
- ~~map2(1)：目标就是墙本身，A*直接RIGHT推，路径正确~~ **错**：目标在(2,4)(2,6)空地
- Walk BFS修复对map0效果：之前绕左边13步→现在直走右边10步

### 备份文件
- `collision_pipeline.py.bak` — 碰撞管线原始版本
- `run_sim.py.bak2`, `run_solver.py.bak2` — ID修复前版本
- `track_port.py.bak3`, `run_solver.py.bak3`, `micro_scheduler.py.bak3` — 本次修复前版本

### 新增经验
22. **目标排序一致性**：c_solver内部行优先、外部sorted()列优先，索引对不上导致ID错配——跨模块的数据契约必须显式对齐
23. **推箱到达判定**：欧氏距离有跨轴污染，单轴判定更鲁棒
24. **Walk BFS终点阻塞**：BFS不应把"推箱站位"当障碍物——那是起点/终点，不是中间障碍

## 十二、2026-07-10 炸弹直推修复（本次会话）

### 用实跑坐标追踪推翻旧结论
用逐步模拟车/炸弹位置的脚本(不靠读代码猜)得到 ground truth：
- map1(1)：车@(3,5)、箱$(2,5)、炸弹*(4,5)(7,5)(10,5)、目标.(13,5) **全在行5**
- map2(1)：车@(2,5)、炸弹*同上、目标.(2,4)(2,6)、箱$(12,4)(12,6)
- 关键教训：**记忆里的结论可能是错的，涉及路径必须实跑追踪，不能靠读地图/代码猜**

### 问题1：map2(1) 推完炸弹绕开炸点（run_solver.py 中间层bug）
**现象**：炸弹推进墙爆炸后(5,5)已是空地，车去推下一个炸弹时却绕第4行 `[(4,5),(4,4),(5,4),(6,4),(6,5)]`，不走空地。
**根因**：`_smooth_actions` 的 wait 分支(第308-325行)清了 `walls` 却没清 `obj_set`——爆炸炸弹残留为**软障碍**，AvoidanceGraph 对软障碍周围禁斜线，导致绕行。
**修复**：wait 清墙时加 `obj_set.discard((bc, br))`（第324行后）。
**验证**：map2(1) 三段 free_move 全部从绕行变直走（`[(4,5),(5,5),(6,5)]` 等）。
**注意**：raw 动作序列本就是直走的，绕行是中间层平滑器引入的，不是 c_solver 算法问题。

### 问题2：map1(1) 无法直推炸弹进墙（track_port.py 算法层）
**现象**：炸弹本可从(4,5)直推进(5,5)墙，算法却"下推一格(4,5)→(4,6)+右推进(5,6)"绕行，P1共34步。
**根因**：引爆墙候选池 pool（第2568-2577行）用凸包相交过滤。map1(1) 所有关键点共线→凸包退化成**零面积线段**→`is_wall_intersect_hull` 的3×3角点取整 `int(2r±1.5)` 只覆盖到行6，炸弹同行墙(5,5)被判凸包外排除。pool 只剩 (6,5)(6,8)(6,11)，是 P1 三重循环枚举炸弹目标的唯一来源，(5,5)不进池就永远选不到直推。
**修复（方案A）**：pool 构建后额外补"每个炸弹四邻的可推入墙格"（邻格是墙+反向格车可站）并去重（第2576行后新增，标记 `[FIX2]`）。
**效果**：map1(1) 炸点(6,5)→(5,5)，P1 **34步→16步**，纯直推零绕行。
**零回归证明**（关键）：实测17张图 pool 增量，**仅 map1(1) +3，其余16张全 +0**（组合数不变）——非退化图的炸弹相邻墙本就在池里，补墙被去重滤掉。map0 的113s慢是其固有(35墙3炸弹6545组合)，与改动无关。

### 备份文件
- `run_solver.py.bak4` — 问题1修复前
- `c_solver/track_port.py.bak4` — 问题2修复前

### 新增经验
25. **记忆结论会过时/出错**：涉及具体路径/坐标的结论，用实跑追踪脚本验证，别信旧记忆也别靠读代码猜（旧记忆说map1目标在墙下方，实际在同行）
26. **绕开空地=软障碍残留**：爆炸/移除物体后必须从 obj_set(软障碍集)移除，只清 walls 不够——规避图对软障碍周围禁斜线会绕行
27. **凸包退化陷阱**：关键点共线时凸包变零面积线段，几何相交判定会漏掉线上的格子。补丁思路是"过滤后额外补必要候选+去重"，比改几何底座(影响全图)更安全
28. **只增不减+去重=安全扩候选**：候选池只增不减且去重，对不需要的图天然零扰动（重复被滤），正确性单调（搜索取最优，候选变多只会不变或更优）

## 十三、2026-07-10/11 P2 车撞非目标箱子修复

### 问题：map1(4) action18 终点(2,9)有箱子，车路径穿过它撞开——没把非推动物体当障碍
**ID分配**：箱(3,3):0, (2,8):6, (2,9):9；目标(13,3):0, (12,4):6, (13,9):9。

### 根因分析（双层）
1. **decompose 层**：`decompose_boxes` 调用 `push_box_bfs` 时 `obstacles` 只有墙(不含其他箱子)，导致每个箱子的推箱路径规划时认为其他箱子位置为空，算出的 `car_target`(推箱站位)可能就是另一箱子的初始位置。
2. **schedule 层**：`schedule_micro_tasks` 的 Dijkstra 虽然检查了推箱路径碰撞(`box_end`/逐格)，但**没检查 car_target 是否被其他箱子占据**。

### 修复（双保险，micro_scheduler.py）

**修复1(根本性，decompose层)**：`decompose_boxes` 第58-60行，循环内为每个箱子单独构建障碍集：
```python
box_obstacles = obstacles | {boxes[j] for j in range(len(boxes)) if j != i}
```

**修复2(防御性，schedule层)**：`schedule_micro_tasks` 第150-156行，`car_bfs` 前加检查：
```python
# 推箱站位检查：car_target 不能被其他未完成箱子占据
other_boxes = {...}  # 挪到 car_bfs 之前
if task.car_target in other_boxes:
    continue
```

### 验证结果
- map1(4) 固定ID：P1+P2共37 actions, 0 穿箱错误 ✓
- map0/map1(4)/map2(4)/map8 批量测试：全部 P2 OK, 0 errors ✓

### 备份文件
- `algorithms/micro_scheduler.py.bak4` — 修复前
- `run_solver.py.bak5` — 修复前（本次还涉及问题1的 obj_set 修复）

### 新增经验
29. **decompose时obstacles必须包含其他箱子**：否则算出的 car_target 可能穿箱。`push_box_bfs` 的 obstacles 参数要包含"除当前箱外所有箱子的初始位置"
30. **调度器需双重防线**：即使 decompose 正确，schedule 里也要检查 car_target——因为调度顺序可能让箱子移动到未预料的位置
31. **修复顺序**：先加防御性检查(改动小、风险低)验证问题可复现，再加根本性修复(改动大、可能影响路径长度)，两步独立验证避免混淆

## 十四、2026-07-11 P2回退路径逐格推合并

### 问题：有时推物一格一顿，有时连续n-1快推+1慢推
**根因**：P2 有两条路径产生不同粒度：
- **Dijkstra 成功**（micro_scheduler）：`_merge_push_segments`合并同向连续推，再拆"n-1快+1慢" → 流畅
- **c_solver P2 回退**（action_converter `convert_p2_actions`）：每推一格生成一个独立 push_box，**从不合并** → 一格一顿

实测发现 map6/9/10/11 **几乎都走 c_solver P2 回退**（Dijkstra 成功率不高），所以一格一顿很常见。

### 修复（run_solver.py）
新增 `_merge_consecutive_pushes(actions)`：扫描 actions，把连续同 box_id、同 push_dir、中间无打断的 push_box 合并为"前 n-1 步快推 + 最后 1 格慢推"。在 c_solver P2 回退路径（`_smooth_actions` 前）调用。**不改 action_converter.py 和 micro_scheduler.py**。

### 验证
| 地图 | 原始最大连续推 | 合并后 |
|------|------|------|
| map6 | [4] | [2] |
| map9 | [6,5,6,7] | 全≤2 |
| map10 | [7,7,7] | 全≤2 |
| map11 | [12,12,12] | 全≤2（12连推→2） |
- Dijkstra 成功路径（map0）不受影响，0 穿箱回归。

### 备份
- `run_solver.py.bak6` — 修复前

### 新增经验
32. **两条P2路径粒度不同**：Dijkstra走micro_scheduler(合并)，回退走action_converter(逐格)。一格一顿=走了回退。后处理合并统一粒度，比改两个转换器省事
33. **Dijkstra P2成功率不高**：多数复杂图（map6/9/10/11）都回退c_solver P2——回退路径的质量很重要

## 十五、2026-07-11/12 map7 P1无解根因调查（未修复，用户要求先记录）

### 结论：map7 有解，但 P1 A* 搜不出来（搜索能力边界，非bug）

**正确引爆点**（用户提供，实测验证）：
- 炸弹1 (列3行6)=内部(6,3)：炸弹(7,3)向上推1格进墙
- 炸弹2 (列9行3)=内部(3,9)：炸弹(9,13)**推着走多步**撞进(3,9)墙
- 炸完后推箱连通矩阵**全Y**：所有箱子可达所有目标，dot0(1,9)那个被墙半包围的角落被(3,9)爆炸打通

**逐层排查（实测坐实）**：
1. (6,3)和(3,9)**都在候选池pool**里 ✓
2. 组合通过`fast_forward_bfs_check`预检（ret=False）✓
3. 组合进入`solve_with_bombs`✓
4. **但 P1 A* 返回 metric=-1（无解）** ← 卡在这里

**为什么搜不出**：搜索深度太大——
- 炸弹1 (7,3) 被箱子 (8,3) 堵住推动站位，需先腾挪箱子
- 炸弹2 (9,13) 要推着走多步才能到 (3,9)（不是相邻墙一步推入）
- 加箱子分配，联合序列节点数指数爆炸

实测节点上限 5000/40000/150000 都失败（150000 跑2分钟超时）。单纯加节点不可行。

### fast_forward_bfs_check 的潜在隐患（本次未触发但需注意）
第977-1010行用**箱子和目标同索引配对**（box[i]→dot[i]）判可行性，不考虑交叉分配。若正确解需要交叉分配（box0→dot2），可能误剪。map7正确组合恰好同索引也能过，未触发，但这是个脆弱点。

### 出路（未实施）
1. **改进P1启发式/剪枝**（治本，动算法核心，风险高，需专门开一轮）
2. 保守兜底方案救不了map7——因为P1一步都没解出，没有"P1解完"的前提

### 新增经验
34. **map7坐标教训**：多次(列,行)vs(内部r,c)换算出错，务必用脚本验证坐标，别手工换算
35. **"P2失败"可能是P1无解的假象**：solve_with_c_solver P1失败时返回success=False，P2根本没机会跑。要区分"P1无解"和"P2无解"
36. **fast_forward_bfs_check同索引配对**：预检假设box[i]→dot[i]，交叉分配的解可能被误剪（map7未触发，但是隐患）
37. **搜索能力边界≠bug**：组合正确、能进搜索、但A*节点耗尽搜不出，是启发式/剪枝不够强，不是逻辑错误

---

## TODO：map7 P1 无解（优先级：低，有空再大修，实在不行按特例抛弃）

**快速摘要**：P1 枚举 522 组，15 组过初筛但全部失败。正确引爆组合 (3,6)+(9,3)（列,行）组合正确、过所有过滤、炸后连通全Y，**但 P1 A* 在节点上限内搜不出**——炸弹1被箱子堵站位、炸弹2需推着走多步到远处墙，联合序列深度太大。

---

### 以下是完整排查记录（保留备查）

### 问题摘要
map7 的 P1 A* 在正确引爆组合 (6,3)+(3,9)（内部r,c）上搜不出解。炸弹1(7,3)被箱子(8,3)堵住推动站位，炸弹2(9,13)需推着走多步到(3,9)撞墙。组合正确、通过所有过滤器、但 A* 在默认节点上限(1000/3000)及提高上限(5000/40000/150000)后均失败或超时。

### 地图结构（内部坐标 r=行, c=列, 12行×16列）
```
r 0 ################
r 1 #@-#---#-.#----#   car@(1,1)  dot0.(1,9)
r 2 #---#--#-####--#   墙密布
r 3 ##-###-#-#-----#   bomb2引爆点(3,9)=#
r 4 #-.#-#-#-#--#--#   dot1.(4,2)
r 5 #-##-#---------#
r 6 #--#-------##--#
r 7 #--*--------#--#   bomb1*(7,3)
r 8 #--$-#-----$---#   box0$(8,3) box1$(8,11)
r 9 #-$--########*-#   box2$(9,2) bomb2*(9,13)
r10 #----#.--------#   dot2.(10,6)
r11 ################
```
- 车(1,1) 箱[(8,3)(8,11)(9,2)] 炸弹[(7,3)(9,13)] 目标[(1,9)(4,2)(10,6)]

### 正确引爆组合
| 炸弹 | 初始(内r,c) | 引爆墙(内r,c) | (列,行) | 难度 |
|------|-----------|-------------|---------|------|
| bomb1 | (7,3) | (6,3) | (3,6) | 箱子(8,3)堵站位，需先腾挪 |
| bomb2 | (9,13) | (3,9) | (9,3) | **非相邻墙**，需推着走多步 |

### 已逐层确认的事实
1. ✓ (6,3)和(3,9)都在候选池 pool 里（凸包相交纳入）
2. ✓ 组合通过 `fast_forward_bfs_check`（ret=False，未被剪）
3. ✓ 组合进入 `solve_with_bombs`
4. ✗ P1 A* 返回 metric=-1（节点上限不足）
5. ✓ 炸后推箱连通矩阵**全Y**（所有box可达所有dot）

### 搜索难点分析
- **炸弹1(7,3)站位被堵**：推炸弹向上进(6,3)，车需站(8,3)，但(8,3)有箱子$。P1 必须先推开箱子腾出站位
- **炸弹2(9,13)远距离推行**：从(9,13)推到(3,9)不是一步入墙，需"先推到某个空地→再推→再撞墙"的多段推，搜索空间大
- **联合序列**：两炸弹+三箱子+迷宫墙，A* 状态空间 (车位置 × 掩码 × 所有物位置) 在深层组合时指数爆炸
- **启发式不够强**：`get_h_p1` 对"先推箱让路再推炸弹"这种长链缺乏有效引导

### 潜在改进方向（按风险从低到高）
1. **针对性放宽 map7 的节点上限**：判断"组合数很少但搜索失败"时自动提高 MAX_NODES（如在 1000→15000 中间加一档），只对特定图增加开销
2. **改进 fast_forward_bfs_check**：解除同索引配对假设（box[i]→dot[i]），改为判断"是否存在某种分配使全部可达"——这个修复难度中等，且能让 P1～P2 的连通预检更准确，间接减少被误剪的组合
3. **改进 get_h_p1 启发式**：对"炸弹被障碍物堵住"这种模式加额外启发成本，引导 A* 优先解除站位阻塞
4. **拆解 P1 为子问题**：先解"清障（推箱让位）→推炸弹→炸墙"的分阶段搜索，降低每段复杂度

### 与保守兜底方案的关系
用户提的"P1解完→重求全图→单独算P2"兜底方案**救不了map7**——因为 P1 一步都没解出，不存在"P1解完"的前提。此方案对另一类情况适用：P1成功但初始P2因ID全未观察/炸墙残余阻挡而失败时，用炸过墙的新全图重算P2。

### 涉及的关键代码位置
- `c_solver/track_port.py`: `solve_with_bombs`(1228行) P1 A*主搜索, `fast_forward_bfs_check`(960行) 第977行同索引配对隐患, `get_h_p1`(467行) 启发式, pool构建(2593-2598行), `is_wall_intersect_hull`(2422行), `global_can_be_first`(2620行)
- `c_solver/constants.py`: `MAX_NODES_1=1000`(13行), `MAX_NODES=3000`(14行)
- `maps_import/map7.txt`: 地图文件

### 涉及的其他潜在问题（本次标记，未深入）
- `fast_forward_bfs_check` 同索引配对可能误剪交叉分配的解（map7未触发但是隐患）
- `solve_with_bombs` 内 goal_idx==-1 时（1600行）无任何原因输出，调试时需手动插桩才能知道是启发式剪枝(1402行)还是堆空(1599行)导致失败
