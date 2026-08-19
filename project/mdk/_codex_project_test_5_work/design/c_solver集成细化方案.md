# c_solver 集成 RT1064 项目 — 细化方案

> 基于"中间层完整规划.md" + 现有项目代码 + c_solver Python 翻译

---

## 一、数据流全景

```
全图帧(0x01)
    │
    ▼
solve_phase1(map_input_t*) → phase1_output_t
    │ .bomb_r/c[3]  炸弹坐标
    │ .p1_path[]     阶段1动作码序列 (0-3移动,4爆炸,5-8观察)
    │ .end_px/y      阶段1结束位置
    │
    ▼
p1_path_to_actions() → action_t[]  (push_bomb + observe_box + observe_target + free_move)
    │
    ▼
path_follower 执行P1动作序列
    │ 推炸弹→引爆→清墙→走位→观察箱子→观察目标
    │ 观察结果存入 g_box_ids[] / g_target_ids[]
    ▼
P1_DONE → 请求新全图(0x03 reason=0x02) → 等待0x01到达
    │
    ▼
从全图提取箱子($)/目标(.)坐标 + g_box_ids/g_target_ids 匹配
    │
    ▼
solve_phase2(g_box_ids, g_target_ids, p1_end_pos) → phase2_output_t
    │ .p2_path[]  阶段2动作码序列 (0-3移动 + 推箱)
    │ .end_px/y   终点(基地)
    │
    ▼
p2_path_to_actions() → action_t[]  (push_box + free_move)
    │
    ▼
path_follower 执行P2动作序列 → 推箱到目标 → 前往基地 → DONE
```

---

## 二、solver.h 修改

### 2.1 新增数据类型

```c
// ===== 观察动作元数据 =====
typedef struct {
    uint8  slot_idx;      // 结果存入 box_ids/target_ids 的槽位
    uint8  look_dir;      // 观察方向 0=UP 1=DOWN 2=LEFT 3=RIGHT
    // car_target 和 target 继承 action_t 的公共字段
} observe_meta_t;

// ===== 阶段1输出 =====
typedef struct {
    uint8  success;
    uint8  bomb_r[3];          // 3个炸弹行坐标
    uint8  bomb_c[3];          // 3个炸弹列坐标
    uint8  path_len;            // P1动作序列长度
    int8   path[MAX_PATH];     // P1动作码 (0-3移动, 4爆炸, 5-8观察)
    uint8  end_px, end_py;     // P1结束后车位置
    int8   obs_slots[MAX_PATH]; // 每个观察动作对应的槽位(-1=非观察, 0..=槽位)
    int8   obs_types[MAX_PATH]; // 0=观察箱子, 1=观察目标 (仅当obs_slots>=0时有效)
} phase1_output_t;

// ===== 阶段2输出 =====
typedef struct {
    uint8  success;
    uint8  path_len;
    int8   path[MAX_PATH];     // P2动作码
    uint8  end_px, end_py;     // 终点
} phase2_output_t;
```

### 2.2 action_t 扩展

在现有 action_t 中新增 observe_meta 字段：

```c
typedef struct {
    uint8  type;                // 新增: ACTION_OBSERVE=4
    uint8  target_x, target_y;
    float  theta;               // observe时: 目标朝向(rad)
    uint8  wp_count;
    waypoint_t waypoints[MAX_WP];
    push_meta_t push_meta;      // push_box / push_bomb 用
    observe_meta_t observe_meta;// observe 用
    float  wait_duration;
    uint8  narrow_passage;
} action_t;

#define ACTION_FREE_MOVE   0
#define ACTION_PUSH_BOX    1
#define ACTION_PUSH_BOMB   2
#define ACTION_WAIT        3
#define ACTION_OBSERVE     4    // 新增
```

### 2.3 新增常量

```c
// 观察子类型
#define OBSERVE_BOX    0   // 观察箱子，识别图片类别
#define OBSERVE_TARGET 1   // 观察目标，识别数字

// 阶段1输入（和map_input_t一致，复用）
// 阶段2输入额外需要box_ids和target_ids
typedef struct {
    map_input_t map;
    int8  box_ids[4];        // 箱子图片类别(-1=未识别)
    int8  target_ids[4];     // 目标数字(-1=未识别)
    uint8 car_end_x;          // P1结束后车位置
    uint8 car_end_y;
} phase2_input_t;
```

---

## 三、solver_adapter.c/h（新建）

这是 c_solver 和 action_t 之间的桥接层。

### 3.1 接口

```c
// solver_adapter.h
#include "solver.h"

// 阶段1: 全图 → P1输出(包含观察槽位分配)
phase1_output_t solve_phase1(map_input_t *map);

// 阶段2: 箱子/目标ID + P1结束位置 → P2输出
phase2_output_t solve_phase2(int8 box_ids[4], int8 target_ids[4],
                              uint8 car_x, uint8 car_y);

// P1路径 → action_t[] 序列
uint8 p1_path_to_actions(phase1_output_t *p1, action_t *actions);

// P2路径 → action_t[] 序列
uint8 p2_path_to_actions(phase2_output_t *p2, uint8 car_start_x, uint8 car_start_y,
                          action_t *actions);

// 将 raw_map[12][16] char 转为 map_input_t
void char_map_to_input(const char raw_map[12][16], map_input_t *out);
```

### 3.2 p1_path_to_actions 实现逻辑

c_solver P1动作码含义：
```
0=上移, 1=下移, 2=左移, 3=右移
4=爆炸 (炸弹在上一步推入墙格后触发)
5=朝上观察, 6=朝下观察, 7=朝左观察, 8=朝右观察
```

转换规则：
```
遍历 p1->path[0..path_len-1]:

  act ∈ {0,1,2,3}:  → free_move
    car 走到 (car_r+dr[act], car_c+dc[act])
    如果该步推动了物体(前后位置有箱子/炸弹):
      act ∈ {0,1,2,3} 且 next_cell 有箱子 → push_box
      act ∈ {0,1,2,3} 且 next_cell 有炸弹箱 → push_bomb

  act == 4: → wait 0.5s (爆炸倒计时)

  act ∈ {5,6,7,8}: → observe
    look_dir = act - 5
    查 obs_slots[step] 和 obs_types[step]:
      obs_type=0 → ACTION_OBSERVE + observe_meta.sub_type=OBSERVE_BOX
      obs_type=1 → ACTION_OBSERVE + observe_meta.sub_type=OBSERVE_TARGET
    car_target = 当前车位置
    target = 被观察的格子坐标 = (car_r+dr[look_dir], car_c+dc[look_dir])
    theta = look_dir 对应的弧度 (UP=π/2, DOWN=-π/2, LEFT=π, RIGHT=0)
```

### 3.3 路径模拟 — 推物检测

c_solver 的 P1 路径中，移动动作(0-3)可能隐含推物。需要通过**初始地图+动作序列模拟**来判断每步是否推了东西：

```c
// 模拟执行P1路径，同时生成action_t序列
static uint8 simulate_p1_and_generate(
    const char raw_map[12][16],
    phase1_output_t *p1,
    action_t *actions)
{
    // 1. 加载初始地图状态
    //    车位置、墙体、箱子($)、炸弹箱(*)、目标(.)
    // 2. 逐动作模拟:
    //    - 移动: 更新车位置，检测是否推了物体
    //    - 爆炸: 从墙体集合移除3x3范围
    //    - 观察: 记录观察槽位和类型
    // 3. push_bomb的car_target = 炸弹后方站位 = (bomb_r - dr, bomb_c - dc)
    //    push_bomb的target/wall_target = 炸弹被推入的墙格坐标
}
```

---

## 四、path_follower.c 修改

### 4.1 新增状态

```c
#define FOLLOWER_OBSERVING  6   // 新增: 观察状态
```

### 4.2 enter_current_action 扩展

```c
case ACTION_PUSH_BOMB:
    // 和 PUSH_BOX 逻辑一致，走 PUSHING 状态
    g_state = FOLLOWER_PUSHING;
    g_sub_state = PUSH_APPROACH;
    g_push_dist_total_mm = (float)cur->push_meta.n_steps * PF_GRID_MM;
    break;

case ACTION_OBSERVE:
    g_state = FOLLOWER_OBSERVING;
    g_sub_state = PUSH_APPROACH;  // 复用: 先走到观察站位
    break;
```

### 4.3 update_observing 实现

```c
static void update_observing(void)
{
    action_t *cur = &g_output.actions[g_action_idx];
    observe_meta_t *om = &cur->observe_meta;

    if(g_sub_state == PUSH_APPROACH) {
        // 走到观察站位 (和MOVING逻辑相同)
        float tgt_x = grid_to_mm(cur->target_x);
        float tgt_y = grid_to_mm(cur->target_y);
        float pos_x, pos_y, pos_theta;
        vision_get_fused_pose(&pos_x, &pos_y, &pos_theta);

        float dx = tgt_x - pos_x;
        float dy = tgt_y - pos_y;
        float dist = sqrtf(dx*dx + dy*dy);

        if(dist < PF_ARRIVE_MM) {
            g_sub_state = 1;  // FACE阶段
            return;
        }

        float vx = pf_clamp(PF_KP_POS * dx, -PF_MAX_V_MM_S, PF_MAX_V_MM_S);
        float vy = pf_clamp(PF_KP_POS * dy, -PF_MAX_V_MM_S, PF_MAX_V_MM_S);
        motion_set_world_velocity(vx, vy, heading_hold());
        return;
    }

    // FACE: 转向面对观察方向
    if(g_sub_state == 1) {
        float target_theta = om->look_dir * (PI / 2.0f); // 0→0, 1→π/2, 2→π, 3→3π/2
        float pos_theta;
        // 读当前朝向
        float px, py;
        vision_get_fused_pose(&px, &py, &pos_theta);

        float err = target_theta - pos_theta;
        // 角度归一化到[-π, π]
        while(err > PI) err -= 2*PI;
        while(err < -PI) err += 2*PI;

        if(fabsf(err) < 0.05f) {  // ~3°
            // 转向完成 → 触发识别
            g_sub_state = 2;  // RECOGNIZE阶段
            return;
        }

        float omega = pf_clamp(6.0f * err, -360.0f, 360.0f);
        motion_set_world_velocity(0, 0, omega);
        return;
    }

    // RECOGNIZE: 等视觉识别结果
    if(g_sub_state == 2) {
        // OpenART通过全图帧识别指定格子的图案
        // 识别结果由 vision_parser 填入 g_vision_state
        // 这里从全图快照中读取对应格子的识别结果

        vision_state_t snapshot;
        if(vision_take_full_map_snapshot(&snapshot) && snapshot.full_map_valid) {
            // 查 target 格子的识别结果
            int8 gx = (int8)cur->target_x;
            int8 gy = (int8)cur->target_y;
            
            // TODO: 和OpenART对齐具体识别数据格式
            // 目前假设全图帧中包含格子的图片类别/数字信息
            
            advance_action();
            return;
        }

        // 超时保护: 2s未识别到 → STUCK
        if(g_action_ticks > 400) {  // 400 ticks = 2s
            g_state = FOLLOWER_STUCK;
            stop_output();
        }
        return;
    }
}
```

### 4.4 push_bomb 完成判定

在 `update_pushing` 的 PUSH_EXEC 阶段，push_bomb 和 push_box 的区别：

```c
// push_bomb 完成判定:
if(cur->type == ACTION_PUSH_BOMB) {
    // 推炸弹: 到达目标墙格即完成（炸弹自动引爆）
    if(g_push_dist_done_mm >= g_push_dist_total_mm) {
        // 插入爆炸等待
        // 简化为: 直接advance (下一action是WAIT 0.5s, 在p1_path_to_actions中已插入)
        advance_action();
        return;
    }
}
// push_box 完成判定: 保持现有逻辑（视觉闭环 + 开环回退）
```

---

## 五、main.c 修改

### 5.1 新状态枚举

```c
typedef enum {
    MISSION_WAIT_MAP = 0,
    MISSION_EXEC_P1,      // 执行阶段1
    MISSION_WAIT_RECOG,   // P1完成, 请求新全图, 等待
    MISSION_EXEC_P2,      // 执行阶段2
    MISSION_DONE,
    MISSION_ERROR
} mission_state_t;
```

### 5.2 新增全局变量

```c
static phase1_output_t g_p1_output;
static phase2_output_t g_p2_output;
static int8  g_box_ids[4];       // 识别的箱子类别
static int8  g_target_ids[4];    // 识别的目标数字
static uint8 g_p1_done;          // P1路径是否执行完毕
```

### 5.3 新增函数

```c
// 阶段1启动: 从全图计算P1
static void mission_start_phase1(void)
{
    if(!mission_copy_vision_map(&g_mission_map)) {
        mission_enter_error(1);
        return;
    }

    g_p1_output = solve_phase1(&g_mission_map);
    if(!g_p1_output.success) {
        mission_enter_error(2);  // P1无解
        return;
    }

    // P1路径 → action_t[]
    action_t actions[MAX_ACTIONS];
    uint8 act_count = p1_path_to_actions(&g_p1_output, actions);

    // 包装成 solver_output_t 以复用 path_follower_load
    solver_output_t out;
    memset(&out, 0, sizeof(out));
    out.success = 1;
    out.action_count = act_count;
    memcpy(out.actions, actions, act_count * sizeof(action_t));

    path_follower_load(&out);
    if(path_follower_state() == FOLLOWER_STUCK) {
        mission_enter_error(3);
        return;
    }

    odometry_reset();
    device_init_flag = 0;
    g_mission_state = MISSION_EXEC_P1;
    g_p1_done = 0;
}

// 阶段2启动
static void mission_start_phase2(void)
{
    // 从最新全图帧获取箱子/目标坐标
    vision_state_t snapshot;
    if(!vision_take_full_map_snapshot(&snapshot) || !snapshot.full_map_valid) {
        mission_enter_error(4);
        return;
    }

    // 使用P1阶段观察得到的 box_ids / target_ids
    g_p2_output = solve_phase2(g_box_ids, g_target_ids,
                                g_p1_output.end_px, g_p1_output.end_py);
    if(!g_p2_output.success) {
        mission_enter_error(5);  // P2无解
        return;
    }

    action_t actions[MAX_ACTIONS];
    uint8 act_count = p2_path_to_actions(&g_p2_output,
                                          g_p1_output.end_px, g_p1_output.end_py,
                                          actions);

    solver_output_t out;
    memset(&out, 0, sizeof(out));
    out.success = 1;
    out.action_count = act_count;
    memcpy(out.actions, actions, act_count * sizeof(action_t));

    path_follower_load(&out);
    g_mission_state = MISSION_EXEC_P2;
}
```

### 5.4 mission_update 改造

```c
static void mission_update(void)
{
    switch(g_mission_state) {
        case MISSION_WAIT_MAP:
            if(vision_full_map_ready())
                mission_start_phase1();
            break;

        case MISSION_EXEC_P1:
            if(path_follower_state() == FOLLOWER_STUCK) {
                mission_stop_and_wait(VP_REQ_REPLAN);
            } else if(path_follower_state() == FOLLOWER_DONE && !g_p1_done) {
                g_p1_done = 1;
                // P1完成后请求新全图(用于P2前的矫正)
                vision_request_full_map(VP_REQ_REPLAN);
                g_mission_state = MISSION_WAIT_RECOG;
                debug_send("#P1_DONE waiting new map for P2");
            }
            break;

        case MISSION_WAIT_RECOG:
            if(vision_full_map_ready())
                mission_start_phase2();
            break;

        case MISSION_EXEC_P2:
            if(path_follower_state() == FOLLOWER_STUCK) {
                mission_stop_and_wait(VP_REQ_REPLAN);
            } else if(path_follower_state() == FOLLOWER_DONE) {
                device_init_flag = 1;
                motion_emergency_stop();
                g_mission_state = MISSION_DONE;
                debug_send("#DONE mission complete");
            }
            break;

        case MISSION_DONE:
        case MISSION_ERROR:
        default:
            device_init_flag = 1;
            break;
    }
}
```

---

## 六、c_solver 算法入口适配

c_solver Python版入口是 `run_phase1(raw_map, p1_res, mode, skip_flag)` 和 `run_phase2(box_ids, target_ids, p2_res)`。C移植时需要：

### 6.1 solve_phase1 内部流程

```c
phase1_output_t solve_phase1(map_input_t *map)
{
    phase1_output_t out;
    memset(&out, 0, sizeof(out));

    // 1. map_input_t → raw_map[12][16] char格式
    char raw_map[12][16];
    map_input_to_chars(map, raw_map);

    // 2. 调用c_solver阶段1核心
    //    = run_phase1移植到C
    //    内部包含: 凸包→炸弹枚举→A*搜索→路径回放→TSP/贪心观察
    PhaseResult p1_res;
    int success = run_phase1(raw_map, &p1_res, 4/*mode=TRACK_NUM*/, 0);

    if(!success) return out;

    // 3. 填充输出
    out.success = 1;
    out.path_len = p1_res.path_length;
    memcpy(out.path, p1_res.path, p1_res.path_length);
    out.end_px = p1_res.end_px;
    out.end_py = p1_res.end_py;

    // 4. 提取炸弹坐标和观察槽位分配
    //    炸弹坐标可以从 g_best_bombs 获取
    //    观察槽位: 遍历path, 对每个观察动作(5-8)分配槽位
    //    根据obs_b/obs_t的bit变化判断是观察箱子还是目标

    out.bomb_r[0] = g_best_bombs[0][0];
    // ... etc

    // 填充 obs_slots[] 和 obs_types[]
    int box_slot = 0, target_slot = 0;
    uint16 obs_b_before = 0, obs_t_before = 0;
    for(int i = 0; i < p1_res.path_length; i++) {
        if(p1_res.path[i] >= 5 && p1_res.path[i] <= 8) {
            // 这是一个观察动作
            // 判断是观察箱子还是目标（需要c_solver内部暴露obs_b/obs_t变化）
            if(/* 观察到新箱子 */) {
                out.obs_slots[i] = box_slot;
                out.obs_types[i] = OBSERVE_BOX;
                box_slot++;
            } else {
                out.obs_slots[i] = target_slot;
                out.obs_types[i] = OBSERVE_TARGET;
                target_slot++;
            }
        } else {
            out.obs_slots[i] = -1;
        }
    }

    return out;
}
```

### 6.2 solve_phase2 内部流程

```c
phase2_output_t solve_phase2(int8 box_ids[4], int8 target_ids[4],
                              uint8 car_x, uint8 car_y)
{
    phase2_output_t out;
    memset(&out, 0, sizeof(out));

    // 调用c_solver阶段2核心
    PhaseResult p2_res;
    int success = run_phase2(box_ids, target_ids, &p2_res);

    if(!success) return out;

    out.success = 1;
    out.path_len = p2_res.path_length;
    memcpy(out.path, p2_res.path, p2_res.path_length);
    out.end_px = p2_res.end_px;
    out.end_py = p2_res.end_py;

    return out;
}
```

---

## 七、c_solver 需要导出的新增接口

为了让 adapter 能正确解析观察动作的槽位分配，c_solver 需要额外暴露：

```c
// track_port.c中新增导出
extern int8 g_obs_box_slots[MAX_STEPS];   // 每个观察动作对应的箱子槽位
extern int8 g_obs_target_slots[MAX_STEPS];// 每个观察动作对应的目标槽位
extern int8 g_obs_slot_count;             // 总观察槽位数
```

这样 adapter 可以直接读取这些信息来填充 `obs_slots[]` 和 `obs_types[]`。

---

## 八、文件改动总览

| 文件 | 操作 | 主要内容 |
|------|------|---------|
| `solver.h` | **修改** | 新增 phase1/phase2_output_t、observe_meta_t、ACTION_OBSERVE |
| `solver_adapter.h` | **新建** | solve_phase1/2接口、路径转换接口 |
| `solver_adapter.c` | **新建** | p1/p2路径→action_t[]转换、路径模拟、观察槽位解析 |
| `c_solver/` (Python) | **移植到C** | track_port.py → solver_core.c (run_phase1/run_phase2) |
| `path_follower.h` | **修改** | 新增 FOLLOWER_OBSERVING、ACTION_PUSH_BOMB处理 |
| `path_follower.c` | **修改** | update_observing()、push_bomb完成判定、enter_current_action扩展 |
| `main.c` | **修改** | 新状态机、mission_start_phase1/2、box_ids/target_ids管理 |
| `vision_parser.h` | **不改** | 接口已够用 |

## 九、OpenART 识别数据对齐（待确定）

观察动作需要从全图帧中获取指定格子的识别结果。当前全图协议(0x01)只含墙体位图+坐标，不含图片类别/数字信息。需要和视觉组对齐：

**方案A**（推荐）：全图帧扩展识别结果字段
- 在 0x01 包中增加 "每个箱子的图片类别(1B)" 和 "每个目标的数字(1B)"
- MCU 发 observe 动作时，OpenART 在下一帧全图中携带这些信息

**方案B**：新增专用识别请求/响应协议
- MCU 发 "请识别格子(x,y)" → OpenART 回发 "类别=N"

**方案C**：MCU 自行从全图帧中提取
- 全图帧包含所有视觉信息（俯视图），MCU 侧跑轻量分类模型不可行

**需要和视觉组确认**：OpenART 是否能实时输出每个箱子/目标的识别结果，数据格式是什么。

---

## 十、实施顺序

| 批次 | 内容 | 预估 |
|------|------|------|
| 0 | c_solver Python→C 移植 (run_phase1/run_phase2) | 核心工作 |
| 1 | solver.h 修改 + solver_adapter.c 新建 | 1-2h |
| 2 | path_follower.c 修改 (observe + push_bomb) | 1-2h |
| 3 | main.c 状态机改造 | 0.5h |
| 4 | 和视觉组对齐 OpenART 识别数据格式 | 沟通 |
| 5 | 全链路编译+上机调试 | 2-3h |
