# c_solver → RT1064 C 移植指南

## 目标平台

NXP i.MX RT1064, Cortex-M7 @ 600MHz
- 32KB I-Cache + 32KB D-Cache (L1)
- 512KB FlexRAM (可配置为TCM) + 512KB OCRAM = 1MB SRAM
- 4MB QSPI Flash (XiP)
- TCM 延迟 ~20ns (单周期，直连CPU)

## 内存布局建议

### DTCM (低延迟，~20ns, 最多512KB, 优先放热点)

| 变量 | 大小 | 原因 |
|------|------|------|
| `q_f[MAX_NODES]` | 6 KB | A*堆比较热路径，每节点必读 |
| `q_g[MAX_NODES]` | 6 KB | 同上 |
| `hash_keys[HASH_SIZE]` | 32 KB | 哈希表主体，每节点encode+insert |
| `hash_gen[HASH_SIZE]` | 16 KB | 代标记，和hash_keys成对访问 |
| `heap[MAX_NODES]` | 6 KB | 手写二叉堆，每节点push+pop |
| `fast_wall[8][12][16]` | 1.5 KB (bool) | A*扩展每方向必查，8种mask |
| `current_hash_gen` | 2 B | 代清除计数器 |
| `heap_size, node_cnt` | 4 B | 堆操作高频读写 |
| `bfs_dist_x[3][12][16]` | 1.1 KB | get_h_p1启发式查表 |
| `bfs_dist_dot[3][12][16]` | 1.1 KB | P2启发式查表 |
| `global_dist[12][16]` | 384 B | BFS距离暂存 |
| **DTCM 小计** | **~70 KB** | |

### OCRAM (中等延迟，512KB, 放次热点)

| 变量 | 大小 | 原因 |
|------|------|------|
| `q[MAX_NODES]` (struct Node) | ~90 KB | State(30B)×3000，A*节点存储 |
| `p2_vis[12][16][12][16]` | 72 KB | P2阶段4D标记，只在P2搜索时大量访问 |
| `rq_p2[MAX_P2_Q]` (struct QN) | ~160 KB | P2队列 (10000×16B)，只在P2用 |
| `reach_vis[12][16]` | 384 B | BFS访问标记，每节点BFS必用 |
| `deadlock_map[12][16]` | 144 B | 死锁预计算表 |
| `global_base_wall[12][16]` | 144 B | 基础墙体缓存 |
| `global_exp_cov[12][16]` | 192 B | 爆炸覆盖掩码 |
| `p1_dist_vis[12][16]` | 384 B | 路径回放BFS标记 |
| **OCRAM 小计** | **~325 KB** | |

### Flash/QSPI (只读，4MB)

| 变量 | 大小 | 原因 |
|------|------|------|
| `original_map[12][17]` | 204 B | 原始地图，只读 |
| `perms[6][3]` | 36 B | 排列常量表 |
| `dr[4], dc[4], p_dir[4]` | 24 B | 方向常量 |
| `pos_hash_table[256]` | 1 KB | Zobrist随机数表(延迟初始化) |
| `BOX_TARGET_MAP[3]` | 6 B | 分配常量 |
| 代码 (.text) | ~50-80 KB | 求解器代码 |
| **Flash 小计** | **~60-90 KB** | |

### 总内存预算

| 区域 | 已用 | 总量 | 余量 |
|------|------|------|------|
| DTCM | ~70 KB | ≤512 KB | 充裕 |
| OCRAM | ~325 KB | 512 KB | 187 KB |
| Flash | ~90 KB | 4 MB | 充裕 |

## P1-P3 优化对应的C改动

### P2: BFS位掩码 (已实施)

CTrack_port.py中A*主循环的BFS用`vis_rows[12]` (uint16_t行掩码)替代了原来的
reach_vis+vis_id方案。C移植时：

```c
// 每行一个uint16_t位掩码，12行 = 192位，覆盖12x16网格
uint16_t vis_rows[12] = {0};

// 标记访问: vis_rows[r] |= (1 << c);
// 检查访问: (vis_rows[r] & (1 << c)) == 0

// 位掩码优势:
// 1. 避免memset (vis_id代清除) — 直接用局部变量，BFS后自然释放
// 2. 每行16位，D-cache友好 (单行掩码只需2字节)
// 3. 位操作在Cortex-M7上单周期执行
```

### P3: P2排列启发式 (已实施)

P2阶段枚举6种推箱排列时，按"箱到目标距离和"升序排列，近的排列先试。
这利用了best_score剪枝更快收敛。

```c
// 排列排序key: Σ曼哈顿距离(箱[k], 目标[k])
// 升序排列per_order[0..5]，近的先试
// 不影响最终最优解选择，仅改变尝试顺序
```

## 编译器建议

```
arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
  -O3 -fno-common -funroll-loops \
  -ffunction-sections -fdata-sections \
  -Wl,--gc-sections
```

关键flags:
- `-O3`: 最高优化 + 自动循环展开
- `-mcpu=cortex-m7`: 针对M7指令集优化
- `-mfpu=fpv5-sp-d16 -mfloat-abi=hard`: 硬件FPU加速浮点评分
- `-funroll-loops`: A*内循环展开 (hashtable probe, 4-direction checks)
- `-ffunction-sections -fdata-sections + --gc-sections`: 去掉未用代码

### 链接脚本要点

```
DTCM  (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
OCRAM (rwx) : ORIGIN = 0x20200000, LENGTH = 512K
```

## 对比：优化前→后预期提速

| 指标 | 优化前(C基线) | 优化后预期 |
|------|--------------|-----------|
| 超时限制 | 10.5s (截断) | 移除 |
| P1 A* 耗时 | 基准 | -5~15% (BFS位掩码) |
| P2 A* 耗时 | 基准 | -5~10% (排列启发式) |
| 内存占用量 | ~同等 | ~同等 (位掩码更小) |
| 解的质量 | 同上 | 同上 (逻辑不变) |
