# code_review_issues.txt 问题真实性复核

生成时间：2026-06-14 21:27:14 +08:00

复核范围：
- `project/code`
- `project/user/src`
- `project/user/inc`
- `project/user/user_config.h`
- `project/mdk/rt1064.uvprojx`

本文件只记录检查结论，未修改代码。

## 总体结论

`code_review_issues.txt` 中的大多数问题在当前代码中确实存在，但严重程度有几项被放大。

当前最需要优先处理的是：

1. `device_init_flag` 未声明为 `volatile`
2. `vision_parser.c` 环形缓冲读写索引跨 ISR 访问
3. `blue.c` 的 `BT_SPEED_SUB` 缺少下限保护
4. `main.c` 关全局中断期间复制 `g_vision_state`
5. `avoidance_graph.c` 仍为空实现，当前依赖 BFS 回退

## 逐条复核

| 编号 | 报告问题 | 真实性 | 复核严重度 | 结论 |
|---|---|---:|---:|---|
| 1 | `device_init_flag` 未声明为 `volatile` | 真实 | 严重 | 它在主循环/蓝牙命令中写入，在 `motor_control_5ms()` 5ms 中断中读取。优化编译下存在状态不可见风险，可能影响启动/停车保护。 |
| 2 | `back_forth_mode/back_forth_speed` 未声明为 `volatile` | 部分真实 | 低到中 | 变量确实不是 `volatile`，但报告称 `motion_control.c:711` 为 ISR 读取不准确；当前该位置是 `motion_set_velocity()` 中写 `back_forth_mode = 0`，不是 ISR 读取。 |
| 3 | `vision_parser.c` 环形缓冲存在 ISR 抢占风险 | 真实 | 中等偏低 | LPUART4 ISR 写入，PIT ISR 消费。`rx_write_idx` 为 `volatile`，`rx_read_idx` 不是。16 位读写本身通常是原子的，但跨 ISR 可见性和边界行为仍建议修正。 |
| 4 | `main.c` 关全局中断期间执行较大 `memcpy` | 真实 | 中等偏低 | `mission_copy_vision_map()` 在关全局中断期间复制 `vision_state_t`，当前结构约 234 字节。RT1064 速度足够快，但会短暂阻塞 PIT/UART。 |
| 5 | `g_mission_error` 写入后未有效使用 | 真实 | 低 | 影响错误诊断和后续恢复策略，不直接破坏当前停车保护。 |
| 6 | `avoidance_graph.c` 三个函数为空实现 | 真实 | 中等 | `avoidance_graph_get_waypoints()` 永远返回 0，`micro_scheduler.c` 会回退到 `car_bfs_path()`。基础路径规划不至于完全失效，但复杂避障和路径优化能力缺失。 |
| 7 | `BT_SPEED_SUB` 无下限保护 | 真实 | 中等偏低 | `target_linear_speed -= STEP_SPEED` 后没有 clamp。当前未看到 `bt_cmd_handler()` 被 LPUART ISR 直接调用；若后续重新启用蓝牙命令，这条风险会上升。 |
| 8 | 主循环 catch-up 逻辑意图不清 | 真实 | 低 | `while(now - last_tick >= 5U)` 循环体只递增 `last_tick`，主循环实际仍全速运行。属于可读性/意图问题。 |
| 9 | 使用 `NAN` 作为 `theta` 哨兵 | 真实 | 很低 | `solver.c` 和 `micro_scheduler.c` 中有 `theta = NAN`。当前 `path_follower` 未使用 action 的 `theta`，暂时没有直接运行风险。 |
| 10 | `VP_DMA_BUF_SIZE` 命名误导 | 真实 | 很低 | 实际是软件环形缓冲，未使用 DMA。 |
| 11 | `mm_to_grid()` 中 `g < 0` 检查冗余 | 真实 | 很低 | 前面已经判断 `mm < 0`，除法后 `g < 0` 不会成立。属于防御式冗余。 |
| 12 | `isr.h` 包含不必要的 `motion_control.h` | 真实 | 低 | `isr.h` 当前没有使用 `motion_control.h` 中的声明，会增加不必要依赖。 |
| 13 | `user_config.h` 未实际参与源码 include | 真实 | 低 | 文件在 Keil 工程中列出，但源码中没有 `#include "user_config.h"`；文件头也说明是旧版参考。 |
| 14 | `motor_pwm_init()` 初始化顺序与 `wheel_hw` 顺序不一致 | 真实 | 低 | PWM 初始化顺序为 C6、C10、C8；`wheel_hw` 顺序为 C6、C8、C10。功能上独立初始化，不直接影响运行，但维护上容易误解。 |

## 重点证据位置

- `project/code/blue.c`
  - `device_init_flag` 定义：`int device_init_flag = 1;`
  - `BT_SPEED_SUB`：`target_linear_speed -= STEP_SPEED;`
- `project/code/blue.h`
  - `extern int device_init_flag;`
- `project/user/inc/main.h`
  - `extern int device_init_flag;`
- `project/code/motion_control.c`
  - `motor_control_5ms()` 中读取 `device_init_flag`
  - `motion_set_velocity()` 中写 `back_forth_mode = 0`
  - `motor_pwm_init()` 初始化顺序与 `wheel_hw` 不一致
- `project/code/debug_modes.c/.h`
  - `back_forth_mode/back_forth_speed` 为普通全局变量
- `project/user/src/isr.c`
  - `PIT_IRQHandler()` 调用 `vision_parser_update()`、`path_follower_update()`、`motor_control_5ms()`
  - `LPUART4_IRQHandler()` 调用 `vision_parser_rx_handler()`
  - `LPUART8_IRQHandler()` 调用 `ai_tuner_uart8_rx_handler()`
- `project/code/vision_parser.c`
  - `rx_write_idx` 为 `volatile`
  - `rx_read_idx` 为普通 `uint16`
- `project/user/src/main.c`
  - `mission_copy_vision_map()` 关全局中断复制 `g_vision_state`
  - `g_mission_error` 写入后仅 `(void)g_mission_error`
- `project/code/avoidance_graph.c`
  - `find_avoidance_nodes()` 仅 `memset`
  - `check_safe_connection()` 返回 0
  - `avoidance_graph_get_waypoints()` 返回 0

## 建议处理优先级

1. 将 `device_init_flag` 的定义和所有 extern 声明改为 `volatile int`
2. 将 `rx_read_idx` 也改为 `volatile`，并评估是否需要限制 LPUART4/PIT 抢占关系
3. 给 `BT_SPEED_SUB` 后的 `target_linear_speed` 增加下限 clamp
4. 缩短 `mission_copy_vision_map()` 关全局中断区域，或改为双缓冲/只复制必要字段
5. 明确 `avoidance_graph.c` 是 TODO 还是要实现；若暂不实现，应保留 BFS 回退并写清楚
6. 低优先级清理：`g_mission_error` 显示/上报、`VP_DMA_BUF_SIZE` 重命名、`isr.h` 依赖清理、PWM 初始化顺序整理
