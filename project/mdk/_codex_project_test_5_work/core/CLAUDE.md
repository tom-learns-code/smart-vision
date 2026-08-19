# core/ — 物理引擎 + 地图分析 + 工具模块

## 文件分工

### 地图分析 & 工具
| 文件 | 职责 |
|------|------|
| `map_analysis.py` | 地图扫描(scan_map)、文件加载(load_map_from_file)、连通分量BFS(analyze_connectivity)、3x3范围炸墙(get_walls_in_3x3)、矩形范围(get_walls_in_rect)、边界墙判定(is_boundary_wall)、窄道检测(compute_narrow_cells) |
| `bfs.py` | 统一 BFS 最短路径(bfs_shortest_path)、可达区域(bfs_reachable_region) |
| `distance.py` | 曼哈顿/欧氏/切比雪夫距离、到集合最近距离、距离矩阵 |
| `direction.py` | 方向常量和邻居计算 |
| `map_preprocess.py` | 地图预处理 |
| `avoidance_graph.py` | 规避节点+安全连线图（阶段二核心）。find_avoidance_nodes → _build_avoidance_adj → composite_dijkstra。类 AvoidanceGraph 管理生命周期 |

### 物理引擎
| 文件 | 职责 |
|------|------|
| `physics.py` | SAT碰撞检测(sat_collision)、连续碰撞(continuous_collision)、Vector2、速度钳制 |
| `collision_pipeline.py` | 多级碰撞管道(检测→响应→修正)、process_single_collision |
| `adaptive_physics.py` | 自适应子步数、空间哈希优化(optimized_collision_detection) |
| `car_move.py` | 车辆移动逻辑 |

### 游戏对象
| 文件 | 职责 |
|------|------|
| `car.py` | 车辆类（位置/速度/角度/顶点） |
| `bomb.py` | 炸弹类（爆炸状态/入墙计时） |
| `wall.py` | 墙体类（可破坏/不可破坏） |
| `target.py` | 目标点类 |
| `game_object.py` | 基础游戏对象 |
| `game_factory.py` | 对象工厂（创建车/箱/弹/目标/墙） |
| `game_logic.py` | 炸弹爆炸检测、目标收集检测、世界状态导出 |

### 其他
| `config.py` | 全局配置（屏幕/物理/速度/颜色/炸弹参数） |
| `font_manager.py` | 字体管理 |
| `editor.py` | 地图编辑器 |

## AvoidanceGraph 生命周期

```
开局: AvoidanceGraph(walls, w, h) → 预计算墙体规避节点+安全连线(永久缓存)
设物体: ag.set_objects(box_positions) → 增量更新物体规避节点+连线
free_move: ag.get_waypoints(start, target, obstacles) → 复合图 Dijkstra
推物完成: ag.move_object(old, new) → 增量更新
到目标: ag.remove_object(pos) → 清除节点+连线
```

## 调用规范
- `map_analysis.analyze_connectivity` 内部用 1D 数组索引 `idx = y*w + x`，保持C移植友好
- `map_analysis.scan_map` 标准输出字段：walls, boxes, goals, bombs, car, walkable, counts, width, height
- `physics.sat_collision` 返回 (collided, normal, overlap)，法线指向 poly1
- 所有网格坐标均为 (x, y) 整数元组
