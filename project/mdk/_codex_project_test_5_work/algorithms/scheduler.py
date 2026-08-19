"""
全局调度器 (步骤 7)

Dijkstra 搜索最优 MicroTask 交织序列。
"顺路推" 自然产生——调度器可任意交叉不同对的微任务。
"""

import heapq
from typing import List, Tuple, Optional, Set, Dict
from dataclasses import dataclass

from algorithms.path_search import PairPlan, MicroTask, car_bfs
from core.map_analysis import get_walls_in_3x3

_DIR_VEC = {'UP': (0, -1), 'DOWN': (0, 1), 'LEFT': (-1, 0), 'RIGHT': (1, 0)}
_DIR_CODE = {'UP': 0, 'DOWN': 1, 'LEFT': 2, 'RIGHT': 3, None: 4}


def schedule_pair_plans(
    plans: List[PairPlan],
    car_start: Tuple[int, int],
    walls: Set[Tuple[int, int]],
    width: int,
    height: int,
    boxes: List[Tuple[int, int]],
    bombs: List[Tuple[int, int]],
    turn_penalty: int = 2,
    avoidance_graph=None,
) -> Optional[List[Tuple[int, int, int]]]:
    """
    Dijkstra 搜索最优微任务交织序列。

    状态: (car_x, car_y, p0, p1, p2, last_dir_code)
    其中 p_i = pair i 已完成的 MicroTask 数。

    已炸墙从进度推导:
      p_i > blast_after → pair i 的墙已炸

    Returns:
        [(pair_id, task_idx, car_bfs_distance), ...] 或 None
    """
    n = len(plans)

    # 每个 pair 的平铺任务列表 + 炸墙时机
    all_tasks = []      # [plan.all_tasks]
    blast_after = []    # 完成哪个task后炸墙 (-1=不炸)
    blast_clears = []   # 炸掉哪些墙
    total_per_pair = [] # 每对总任务数
    per_task_clears = []  # chain专用: {task_idx: cleared_walls_set} 逐任务炸墙

    for plan in plans:
        if plan.is_direct:
            tasks = plan.box_tasks
            ba = -1
            ptc = {}
        else:
            tasks = plan.pre_box_tasks + plan.bomb_tasks + plan.post_box_tasks
            ba = len(plan.pre_box_tasks) + len(plan.bomb_tasks) - 1
            # chain: 每个bomb任务最后一段完成后立即清除对应墙
            ptc = {}
            if plan.is_chain and plan.chain:
                bomb_start = len(plan.pre_box_tasks)
                # 找每个chain bomb的最后一段task索引
                chain_last = {}  # chain_idx -> last_task_idx
                cur_ci = 0
                for ti in range(bomb_start, bomb_start + len(plan.bomb_tasks)):
                    tsk = plan.bomb_tasks[ti - bomb_start]
                    if tsk.seg_idx == 0 and ti > bomb_start:
                        cur_ci += 1
                    chain_last[cur_ci] = ti
                # 炸墙关联到最后一段
                from core.map_analysis import get_walls_in_3x3
                sim_w = set(walls)
                for ci in range(len(plan.chain)):
                    cbomb, cwall = plan.chain[ci]
                    blast = get_walls_in_3x3(cwall, sim_w)
                    sim_w -= blast
                    ptc[chain_last.get(ci, bomb_start + ci)] = set(blast)
        all_tasks.append(tasks)
        total_per_pair.append(len(tasks))
        blast_after.append(ba)
        blast_clears.append(set(plan.blast_clears) if not plan.is_direct else set())
        per_task_clears.append(ptc)

    def active_walls(progress):
        """根据进度计算当前墙集(减去已炸墙)"""
        w = set(walls)
        for i in range(n):
            # chain逐任务清除
            for tidx, bset in per_task_clears[i].items():
                if progress[i] > tidx:
                    w -= bset
            # 整体清除 (所有bomb完成后)
            if blast_after[i] >= 0 and progress[i] > blast_after[i]:
                w -= blast_clears[i]
        return w

    def current_obj_positions(progress):
        """返回当前所有未完成物体的位置 {obj_pos}"""
        obs = set()
        for i in range(n):
            tasks = all_tasks[i]
            p = progress[i]
            if p >= len(tasks):
                continue  # 已完成, 箱/弹已消失
            # 物体当前位置
            if p == 0:
                # 还未推任何任务
                if plans[i].is_chain and plans[i].chain:
                    for cbomb, cwall in plans[i].chain:
                        obs.add(cbomb)  # 链中所有炸弹初始位置
                elif plans[i].bomb_tasks:
                    obs.add(plans[i].bomb)
                else:
                    obs.add(plans[i].box)
            else:
                obs.add(tasks[p - 1].obj_end)
        return obs

    def box_only_positions(progress):
        """返回未完成箱子的位置 (车不能穿, 用于car_bfs)"""
        obs = set()
        for i in range(n):
            tasks = all_tasks[i]
            p = progress[i]
            if p >= len(tasks):
                continue
            if plans[i].is_direct:
                if p == 0:
                    obs.add(plans[i].box)
                else:
                    obs.add(tasks[p - 1].obj_end)
            else:
                # 炸弹对: 炸弹还在且箱子还在
                if p < len(plans[i].bomb_tasks):
                    pass  # 还在推弹阶段, 箱子原地未动
                # 箱子位置
                if p == 0:
                    obs.add(plans[i].box)
                elif p <= len(plans[i].bomb_tasks):
                    obs.add(plans[i].box)  # 弹推完了但箱未动
                else:
                    obs.add(tasks[p - 1].obj_end)
        return obs

    def all_obj_positions(progress):
        """返回所有未消失物体的位置 (箱+弹, 车不能站)"""
        obs = set()
        for i in range(n):
            tasks = all_tasks[i]
            p = progress[i]
            if p >= len(tasks):
                continue
            plan = plans[i]
            if plan.is_direct:
                obs.add(plan.box if p == 0 else tasks[p - 1].obj_end)
                continue

            pre_len = len(plan.pre_box_tasks)
            bomb_len = len(plan.bomb_tasks)
            ba = blast_after[i]  # pre_len + bomb_len - 1

            # 炸弹位置
            if plan.is_chain and plan.chain:
                # 链式: 每个炸弹单独追踪(已推+已炸的不算)
                bomb_start = pre_len
                for ci, (cbomb, cwall) in enumerate(plan.chain):
                    if p <= bomb_start + ci:
                        obs.add(cbomb)  # 还未推
                    # p > start+ci: 已推+已炸 → 不在obs
            elif ba >= 0 and p <= ba:  # 炸弹还没炸
                if p <= pre_len:
                    obs.add(plan.bomb)  # 还没开始推弹
                else:
                    obs.add(tasks[p - 1].obj_end)  # 正在推弹

            # 箱子位置
            if p == 0:
                obs.add(plan.box)
            elif p <= pre_len:
                obs.add(tasks[p - 1].obj_end)  # pre_box 移动中
            elif p <= ba + 1:
                obs.add(plan.pre_box_tasks[-1].obj_end if plan.pre_box_tasks else plan.box)
            else:
                obs.add(tasks[p - 1].obj_end)  # post_box 移动中
        return obs

    # 初始状态
    start_progress = tuple(0 for _ in range(n))
    start_state = (car_start[0], car_start[1]) + start_progress + (_DIR_CODE[None],)

    # heap: (cost, car_x, car_y, p0, p1, p2, last_code, path)
    heap = [(0,) + start_state + ((),)]
    best: Dict[Tuple, int] = {start_state: 0}

    while heap:
        item = heapq.heappop(heap)
        cost = item[0]
        cx, cy = item[1], item[2]
        progress = item[3:3 + n]
        last_code = item[3 + n]
        path = item[4 + n]

        state_key = (cx, cy) + progress + (last_code,)
        if best.get(state_key, float('inf')) < cost:
            continue

        # 全部完成?
        if all(progress[i] == total_per_pair[i] for i in range(n)):
            return list(path)

        cur_walls = active_walls(progress)
        cur_all_obj = all_obj_positions(progress)

        # 尝试推进每个未完成的 pair
        for i in range(n):
            p = progress[i]
            if p >= total_per_pair[i]:
                continue

            task = all_tasks[i][p]

            # 验证: 物体路径上每格在当前障碍物下是否可通
            obj_path_ok = True
            dx, dy = _DIR_VEC[task.push_dir]
            cur_obj = task.obj_start
            # 物体经过的位置会被清空, 车可以站上去
            cleared = {task.obj_start}
            for k in range(task.n_steps):
                nxt = (cur_obj[0] + dx, cur_obj[1] + dy)
                car_stand = (cur_obj[0] - dx, cur_obj[1] - dy)
                # 车必须能站 (物体移开后该位置清空)
                if car_stand in cur_walls or (car_stand in cur_all_obj and car_stand not in cleared):
                    obj_path_ok = False; break
                # 物体下一格
                is_last_step = (k == task.n_steps - 1)
                if task.task_type == 'push_bomb' and is_last_step:
                    if not (0 <= nxt[0] < width and 0 <= nxt[1] < height):
                        obj_path_ok = False; break
                else:
                    if nxt in cur_walls:
                        obj_path_ok = False; break
                    if nxt in cur_all_obj and nxt not in cleared:
                        obj_path_ok = False; break
                cur_obj = nxt
                cleared.add(nxt)
            if not obj_path_ok:
                continue

            # 车 BFS 到推物站位 (车不能穿墙/箱/弹)
            # 注: 调度器状态空间大、障碍物动态变化，car_bfs 在网格上
            # 效率高于每次重建复合图；斜线捷径在 Action 生成阶段由
            # AvoidanceGraph.get_waypoints() 处理
            car_obs = cur_walls | cur_all_obj
            cbfs = car_bfs((cx, cy), task.car_target, car_obs, width, height)
            if cbfs is None:
                continue
            car_dist = float(cbfs[0])

            # 转向代价
            prev_dir = None
            if last_code != _DIR_CODE[None]:
                for dname, dc in _DIR_CODE.items():
                    if dc == last_code:
                        prev_dir = dname
                        break
            t_cost = turn_penalty if (prev_dir and prev_dir != task.push_dir) else 0

            new_cost = cost + car_dist + task.n_steps + t_cost
            new_progress = list(progress)
            new_progress[i] = p + 1
            new_progress = tuple(new_progress)

            # 新方向: 推完后车的位置 + 推的方向
            new_last = _DIR_CODE[task.push_dir]

            new_state = (task.car_end[0], task.car_end[1]) + new_progress + (new_last,)
            if new_cost < best.get(new_state, float('inf')):
                best[new_state] = new_cost
                new_path = path + ((i, p, car_dist),)
                heapq.heappush(heap, (new_cost,) + new_state + (new_path,))

    return None
