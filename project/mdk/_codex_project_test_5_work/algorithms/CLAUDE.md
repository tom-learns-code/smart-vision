# algorithms/ — 8 步求解管线

## 管线调用链（数据流向）

```
scan_map (地图数据)
  → compare_connectivity (理想 vs 当前连通分量)
  → analyze_channels (管道/窄口/死锁)
  → preplan_blasts (炸点预规划 P0/P1 + 连锁救援链)
  → solve_assignment (箱→目标分配，全排列+炸弹穷举)
  → refine_blast_points (炸点精评，4方案跨对收益)
  → generate_pair_plans (MicroTask 分解，炸前/炸后分割)
  → schedule_pair_plans (Dijkstra 全局调度微任务交织)
  → generate_actions (输出 v1.0 规范 Action 列表)
```

## 文件分工

| 文件 | 步骤 | 职责 |
|------|------|------|
| `path_analysis.py` | 2b+3 | 连通分量对比、阻塞物定位、格子分类(channel/corner/open)、6类死锁检测、车可达性 |
| `blast_select.py` | 3.5 | 区域桥接墙评分(P0a/P0b/P0c/P1)、连锁救援链 DFS 搜索(max_depth=3) |
| `assignment.py` | 4 | 全排列+炸弹mask穷举、匈牙利替代、合并墙态可行性回验、多炸弹接力DFS |
| `blast_refine.py` | 5 | 精评推弹+推箱总代价、4方案跨对收益(转向墙/通道/合力/近邻清障) |
| `path_search.py` | 6 | Dijkstra推箱/推弹BFS、段合并(_merge_push_segments)、MicroTask/PairPlan 数据结构 |
| `scheduler.py` | 7 | Dijkstra状态空间搜索、动态墙态推导、转向代价、任意交叉不同pair的微任务 |
| `action_generator.py` | 8 | 将调度序列转Action列表、规避节点图waypoint、窄道标记、chain逐任务炸墙 |
| `replay.py` | — | 回放已生成的 Action 序列 |

## 关键数据结构

- `PairPlan`: pair_id + box/goal + bomb_tasks + pre/post_box_tasks + is_chain
- `MicroTask`: task_type(push_box/push_bomb) + obj_start/end + push_dir + n_steps + car_target/end
- `Action`: type(free_move/push_box/push_bomb/wait) + target + push_meta + waypoints + narrow_passage
- `blast_candidate`: wall + benefit向量(P0a/P0b/P0c/P1a/P1b/P2) + score

## 调用规范
- 所有 BFS 统一使用 `core.bfs.bfs_shortest_path`
- 距离计算统一使用 `core.distance.manhattan_distance`
- 墙体 3x3 炸毁统一使用 `core.map_analysis.get_walls_in_3x3`
