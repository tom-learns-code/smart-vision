"""
Action 生成器 (步骤 8)

将调度序列转为算法层输出接口规范 v1.0 格式的 Action 列表。
参考: project_test_4.5.1/design/算法层输出接口规范.txt
"""

from typing import List, Tuple, Optional, Set, Dict

from algorithms.path_search import PairPlan, MicroTask, car_bfs
from core.avoidance_graph import AvoidanceGraph
from core.map_analysis import compute_narrow_cells


# =============================================================================
# 路径拐点提取
# =============================================================================

def _extract_waypoints(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """从BFS路径中提取拐点: 仅保留方向变化的点"""
    if len(path) <= 2:
        return list(path)
    wps = [path[0]]
    for i in range(1, len(path) - 1):
        prev_d = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
        next_d = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
        if prev_d != next_d:
            wps.append(path[i])
    wps.append(path[-1])
    return wps


# =============================================================================
# Action 生成
# =============================================================================

def generate_actions(
    schedule: List[Tuple[int, int, int]],
    plans: List[PairPlan],
    car_start: Tuple[int, int],
    walls: Set[Tuple[int, int]],
    width: int,
    height: int,
    boxes: List[Tuple[int, int]],
    bombs: List[Tuple[int, int]],
    avoidance_graph: Optional[AvoidanceGraph] = None,
) -> Optional[List[Dict]]:
    """
    将调度序列转为 Action 列表（规范 v1.0 格式）。

    Action 类型: free_move | push_box | push_bomb | wait
    """
    n = len(plans)
    all_tasks = []
    blast_after = []
    blast_clears_list = []
    total_per_pair = []
    per_task_clears = []  # chain: {task_idx: walls_set}

    for plan in plans:
        if plan.is_direct:
            tasks = plan.box_tasks
            ba = -1
            ptc = {}
        else:
            tasks = plan.pre_box_tasks + plan.bomb_tasks + plan.post_box_tasks
            ba = len(plan.pre_box_tasks) + len(plan.bomb_tasks) - 1
            ptc = {}
            if plan.is_chain and plan.chain:
                bomb_start = len(plan.pre_box_tasks)
                from core.map_analysis import get_walls_in_3x3
                sim_w = set(walls)
                for bi, (cbomb, cwall) in enumerate(plan.chain):
                    blast = get_walls_in_3x3(cwall, sim_w)
                    sim_w -= blast
                    ptc[bomb_start + bi] = set(blast)
        all_tasks.append(tasks)
        total_per_pair.append(len(tasks))
        blast_after.append(ba)
        blast_clears_list.append(set(plan.blast_clears) if not plan.is_direct else set())
        per_task_clears.append(ptc)

    actions = []
    car_pos = car_start
    progress = [0] * n
    cur_walls = set(walls)
    narrow_set = compute_narrow_cells(cur_walls, width, height)

    def _check_waypoints_narrow(wps):
        """检查路径点是否经过窄道"""
        for wp in wps:
            if (int(wp[0]), int(wp[1])) in narrow_set:
                return True
        return False

    def _check_push_narrow(obj_start, obj_end, push_dir_str):
        """检查推物过程中车的位置是否经过窄道"""
        dx, dy = {'UP': (0,-1), 'DOWN': (0,1), 'LEFT': (-1,0), 'RIGHT': (1,0)}[push_dir_str]
        sx, sy = obj_start
        ex, ey = obj_end
        steps = abs(ex - sx) + abs(ey - sy)
        cur = (sx, sy)
        for _ in range(steps):
            car_p = (cur[0] - dx, cur[1] - dy)
            if car_p in narrow_set:
                return True
            cur = (cur[0] + dx, cur[1] + dy)
        return False

    box_positions = list(boxes)
    bomb_positions = list(bombs)
    bomb_alive = [True] * len(bombs)
    bomb_map = {b: i for i, b in enumerate(bombs)}

    # 链式对: 预计算 task_idx -> chain_bomb 的映射 + 修正per_task_clears
    task_chain_bomb = {}  # (pair_id, task_idx) -> (chain_bomb, chain_wall)
    for pi, plan in enumerate(plans):
        if plan.is_chain and plan.chain:
            bomb_start = len(plan.pre_box_tasks)
            # 找每个chain bomb的最后一段task索引
            chain_last_task = {}  # chain_idx -> last_task_idx
            cur_ci = 0
            for ti in range(bomb_start, bomb_start + len(plan.bomb_tasks)):
                if ti < len(all_tasks[pi]):
                    tsk = all_tasks[pi][ti]
                    if tsk.seg_idx == 0 and ti > bomb_start:
                        cur_ci += 1  # seg_idx重置→下一个chain bomb
                    if cur_ci < len(plan.chain):
                        task_chain_bomb[(pi, ti)] = plan.chain[cur_ci]
                    chain_last_task[cur_ci] = ti

            # 重建per_task_clears: 炸墙关联到每个chain bomb的最后一段
            from core.map_analysis import get_walls_in_3x3
            sim_w = set(walls)
            ptc_new = {}
            for ci in range(len(plan.chain)):
                cbomb, cwall = plan.chain[ci]
                blast = get_walls_in_3x3(cwall, sim_w)
                sim_w -= blast
                ptc_new[chain_last_task.get(ci, bomb_start + ci)] = set(blast)
            per_task_clears[pi] = ptc_new

    for pair_id, task_idx, _ in schedule:
        task = all_tasks[pair_id][task_idx]
        last_bomb_idx = -1  # 本步若推了炸弹则记录其真实索引

        # 自由移动时: 所有物体都是障碍物
        # 规则: 非pushing状态 = 全部物体阻碍; pushing状态 = 被推对象豁免(由物理引擎处理)
        obs = set(cur_walls)
        for bi, bpos in enumerate(bomb_positions):
            if bomb_alive[bi]:
                obs.add(bpos)
        for bi, bpos in enumerate(box_positions):
            obs.add(bpos)

        # free_move
        if car_pos != task.car_target:
            wps = None
            if avoidance_graph is not None:
                # 物体世界坐标(近似: 格心)
                obj_world = {}
                for i2 in range(n):
                    p2 = progress[i2]
                    if p2 >= len(all_tasks[i2]):
                        continue
                    if p2 == 0:
                        gpos = box_positions[i2] if i2 < len(box_positions) else plans[i2].box
                    else:
                        gpos = all_tasks[i2][p2 - 1].obj_end
                    obj_world[gpos] = (gpos[0] * 50 + 25, gpos[1] * 50 + 25)
                for bi2, bp2 in enumerate(bomb_positions):
                    if bomb_alive[bi2]:
                        obj_world[bp2] = (bp2[0] * 50 + 25, bp2[1] * 50 + 25)

                start_grid = car_pos  # car_pos已是网格坐标
                wps = avoidance_graph.get_waypoints(
                    start_grid, task.car_target, obs,
                    object_world_positions=obj_world)

            if wps is None:
                cbfs = car_bfs(car_pos, task.car_target, obs, width, height)
                if cbfs is None:
                    print(f'  [ERROR] car_bfs failed: {car_pos} -> {task.car_target}')
                    return None
                _, cpath = cbfs
                wps = _extract_waypoints(cpath)

            actions.append({
                "type": "free_move",
                "target": task.car_target,
                "theta": None,
                "waypoints": [(int(p[0]), int(p[1])) for p in wps],
                "narrow_passage": _check_waypoints_narrow(wps),
            })

        # push 动作
        if task.task_type == 'push_box':
            actions.append({
                "type": "push_box",
                "target": task.car_end,
                "theta": None,
                "push_meta": {
                    "box_id": pair_id,
                    "box_start": task.obj_start,
                    "box_target": task.obj_end,
                    "push_dir": task.push_dir,
                },
                "narrow_passage": _check_push_narrow(
                    task.obj_start, task.obj_end, task.push_dir),
            })
            box_positions[pair_id] = task.obj_end
        else:
            # 链中的每个bomb_task对应不同的炸弹
            chain_info = task_chain_bomb.get((pair_id, task_idx))
            if chain_info:
                cbomb, cwall = chain_info
                bomb_idx = bomb_map.get(cbomb, -1)
                # 链中每个bomb有自己的目标墙
                wall_for_this_bomb = cwall
            else:
                bomb_idx = bomb_map.get(task.obj_start, -1)
                if bomb_idx < 0:
                    bomb_idx = bomb_map.get(plans[pair_id].bomb, -1)
                wall_for_this_bomb = task.obj_end
            actions.append({
                "type": "push_bomb",
                "target": task.car_end,
                "theta": None,
                "push_meta": {
                    "bomb_id": len(boxes) + bomb_idx,  # 偏移避免与box_id冲突
                    "bomb_start": task.obj_start,
                    "bomb_target": task.obj_end,
                    "wall_target": wall_for_this_bomb,
                    "push_dir": task.push_dir,
                },
                "narrow_passage": _check_push_narrow(
                    task.obj_start, task.obj_end, task.push_dir),
            })
            bomb_positions[bomb_idx] = task.obj_end
            last_bomb_idx = bomb_idx  # 记录刚推的炸弹真实索引(供整体炸墙 wait 用)

        progress[pair_id] = task_idx + 1

        # chain逐任务炸墙: 每完成一个bomb任务立即清除对应墙 + wait
        ptc = per_task_clears[pair_id]
        if task_idx in ptc:
            cur_walls -= ptc[task_idx]
            narrow_set = compute_narrow_cells(cur_walls, width, height)
            if avoidance_graph is not None:
                avoidance_graph = AvoidanceGraph(cur_walls, width, height)
            # 标记对应炸弹已爆炸
            # 优先用刚推完的炸弹真实索引(与 push_bomb action 的 bomb_id 一致);
            # 多段推弹后 task.obj_start 是中间位置, 查 bomb_map 会失配返回 -1
            cbi = last_bomb_idx
            if cbi < 0:
                cbi = bomb_map.get(task.obj_start, -1)
            if cbi >= 0:
                bomb_alive[cbi] = False
            actions.append({
                "type": "wait",
                "target": None,
                "theta": None,
                "duration": 0.5,
                "reason": "bomb_explosion",
                # 引爆元数据: verify 据此独立算 3x3 清墙(支持空格引爆)
                "bomb_id": (len(boxes) + cbi) if cbi >= 0 else -1,
                "blast_center": task.obj_end,  # 炸弹停位=引爆中心(可能是空格)
            })

        # 炸墙(整体): 插入 wait 动作 (非chain的最后bomb)
        if blast_after[pair_id] >= 0 and task_idx == blast_after[pair_id] and task_idx not in ptc:
            clears = blast_clears_list[pair_id]
            cur_walls -= clears
            narrow_set = compute_narrow_cells(cur_walls, width, height)
            # 用刚推完的炸弹真实索引(与 push_bomb action 的 bomb_id 一致),
            # 多段推弹后 plans[pair_id].bomb 查 bomb_map 会失配返回 -1
            bomb_idx = last_bomb_idx
            if bomb_idx < 0:
                bomb_idx = bomb_map.get(plans[pair_id].bomb, -1)
            if bomb_idx >= 0:
                bomb_alive[bomb_idx] = False
            actions.append({
                "type": "wait",
                "target": None,
                "theta": None,
                "duration": 0.5,
                "reason": "bomb_explosion",
                # 引爆元数据: verify 据此独立算 3x3 清墙(支持空格引爆)
                "bomb_id": (len(boxes) + bomb_idx) if bomb_idx >= 0 else -1,
                "blast_center": task.obj_end,  # 炸弹停位=引爆中心(可能是空格)
            })
            # 重建规避节点图 (墙变了)
            if avoidance_graph is not None:
                avoidance_graph = AvoidanceGraph(cur_walls, width, height)

        car_pos = task.car_end

    return actions


def compute_action_stats(actions: List[Dict]) -> Dict:
    """统计 Action 序列"""
    free_moves = 0; free_dist = 0
    push_boxes = 0; box_dist = 0
    push_bombs = 0; bomb_dist = 0
    waits = 0

    for a in actions:
        t = a['type']
        if t == 'free_move':
            free_moves += 1
            wps = a.get('waypoints', [])
            if wps and len(wps) >= 2:
                for i in range(len(wps) - 1):
                    free_dist += abs(wps[i+1][0] - wps[i][0]) + abs(wps[i+1][1] - wps[i][1])
            else:
                free_dist += 1
        elif t == 'push_box':
            push_boxes += 1
            m = a['push_meta']
            box_dist += abs(m['box_target'][0] - m['box_start'][0]) + abs(m['box_target'][1] - m['box_start'][1])
        elif t == 'push_bomb':
            push_bombs += 1
            m = a['push_meta']
            bomb_dist += abs(m['bomb_target'][0] - m['bomb_start'][0]) + abs(m['bomb_target'][1] - m['bomb_start'][1])
        elif t == 'wait':
            waits += 1

    return {
        'free_moves': free_moves, 'free_dist': free_dist,
        'push_boxes': push_boxes, 'box_dist': box_dist,
        'push_bombs': push_bombs, 'bomb_dist': bomb_dist,
        'waits': waits,
        'total_actions': len(actions),
        'total_dist': free_dist + box_dist + bomb_dist,
    }
