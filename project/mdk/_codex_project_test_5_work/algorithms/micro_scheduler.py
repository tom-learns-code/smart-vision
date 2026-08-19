"""
微任务调度器

将每个箱子的推箱路径分解为不可分割的微任务（推箱段），
用 Dijkstra 在全局状态空间中搜索最优跨箱交织序列。

状态 = (车位置, 各箱完成段数, 上次推箱方向)

代价 = 车自由移动距离 + 推箱步数 + 转向惩罚
"""

import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict
from collections import defaultdict

from core.direction import DIRECTION_VECTORS
from core.map_preprocess import MapPreprocess
from algorithms.box_planner import (
    push_box_bfs, car_bfs, _merge_push_segments,
    generate_actions_for_pair, generate_actions_with_retry,
    compute_total_distance,
)

# 方向编码（用于状态哈希）
_DIR_CODE = {'UP': 0, 'DOWN': 1, 'LEFT': 2, 'RIGHT': 3, None: 4}


@dataclass
class MicroTask:
    """一个不可分割的推箱段"""
    box_id: int
    task_idx: int           # 该箱子内的第几段 (0-based)
    box_start: Tuple[int, int]
    box_end: Tuple[int, int]
    push_dir: str
    n_steps: int
    car_target: Tuple[int, int]  # 推之前车应站的位置
    car_end: Tuple[int, int]     # 推完后车的位置


def decompose_boxes(
    boxes: List[Tuple[int, int]],
    goals: List[Tuple[int, int]],
    walls: Set[Tuple[int, int]],
    width: int,
    height: int,
    obstacles: Set[Tuple[int, int]],
    preprocess: MapPreprocess = None,
) -> Optional[List[List[MicroTask]]]:
    """
    将每个箱子的推箱路径分解为微任务列表。

    Returns:
        micro_tasks[box_id] = [MicroTask, ...] 或 None（某箱不可解）
    """
    all_tasks = []
    for i, (box, goal) in enumerate(zip(boxes, goals)):
        # 为当前箱子构建障碍:墙 + 其他箱子的初始位置(不包括当前箱子自己)
        box_obstacles = obstacles | {boxes[j] for j in range(len(boxes)) if j != i}
        pr = push_box_bfs(box, goal, walls, width, height, box_obstacles,
                          preprocess=preprocess)
        if pr is None:
            return None
        _, _, path = pr
        segs = _merge_push_segments(path, box)

        tasks = []
        for j, seg in enumerate(segs):
            tasks.append(MicroTask(
                box_id=i, task_idx=j,
                box_start=seg['box_start'],
                box_end=seg['box_end'],
                push_dir=seg['push_dir'],
                n_steps=seg['n_steps'],
                car_target=seg['car_target'],
                car_end=seg['car_end'],
            ))
        all_tasks.append(tasks)
    return all_tasks


def schedule_micro_tasks(
    micro_tasks: List[List[MicroTask]],
    car_start: Tuple[int, int],
    walls: Set[Tuple[int, int]],
    width: int,
    height: int,
    box_start_positions: List[Tuple[int, int]],
    preprocess: MapPreprocess = None,
    turn_penalty: int = 2,
) -> Optional[List[Tuple[int, int, int]]]:
    """
    Dijkstra 搜索最优微任务交织序列。

    State: (car_x, car_y, p0, p1, p2, last_dir_code)
    其中 p_i = box i 已完成多少段微任务。

    Returns:
        [(box_id, task_idx, car_bfs_distance), ...] 或 None
    """
    n_boxes = len(micro_tasks)
    total_tasks = [len(tasks) for tasks in micro_tasks]

    # 初始状态
    start_progress = tuple(0 for _ in range(n_boxes))
    start_state = (car_start[0], car_start[1]) + start_progress + (_DIR_CODE[None],)

    # heap: (cost, car_x, car_y, p0, p1, p2, last_dir_code, path)
    heap = [(0,) + start_state + ((),)]
    best: Dict[Tuple, int] = {start_state: 0}

    while heap:
        item = heapq.heappop(heap)
        cost = item[0]
        cx, cy = item[1], item[2]
        progress = item[3:3+n_boxes]
        last_dir_code = item[3+n_boxes]
        path = item[4+n_boxes]

        state_key = (cx, cy) + progress + (last_dir_code,)
        if best.get(state_key, float('inf')) < cost:
            continue

        # 检查是否全部完成
        if all(progress[i] == total_tasks[i] for i in range(n_boxes)):
            return list(path)

        # 计算各箱当前位置
        def box_pos(bid):
            p = progress[bid]
            if p == 0:
                return box_start_positions[bid]
            return micro_tasks[bid][p - 1].box_end

        box_positions = [box_pos(bid) for bid in range(n_boxes)]

        # 尝试推进每个未完成的箱子
        for bid in range(n_boxes):
            p = progress[bid]
            if p >= total_tasks[bid]:
                continue

            task = micro_tasks[bid][p]

            # 构建障碍物：墙 + 未完成箱子的当前位置（已完成箱子视为消失）
            car_obs = set(walls)
            for j in range(n_boxes):
                if progress[j] < total_tasks[j]:  # 未完成才计入
                    car_obs.add(box_positions[j])

            # 连锁推检测：推箱目标/路径不能被其他箱子占据
            other_boxes = {box_positions[j] for j in range(n_boxes)
                          if j != bid and progress[j] < total_tasks[j]}

            # 推箱站位检查：car_target 不能被其他未完成箱子占据
            if task.car_target in other_boxes:
                continue

            # 车能否到达推箱站位？
            cbfs = car_bfs((cx, cy), task.car_target, car_obs, width, height)
            if cbfs is None:
                continue
            car_dist = cbfs[0]
            # 目标位置检查
            if task.box_end in other_boxes:
                continue
            # 逐格路径检查
            dx, dy = DIRECTION_VECTORS[task.push_dir]
            cur = task.box_start
            blocked = False
            for _ in range(task.n_steps):
                nxt = (cur[0] + dx, cur[1] + dy)
                if nxt in other_boxes:
                    blocked = True; break
                if nxt in walls:
                    blocked = True; break
                cur = nxt
            if blocked:
                continue

            # 转向代价
            prev_dir = None
            if last_dir_code != _DIR_CODE[None]:
                for dname, dc in _DIR_CODE.items():
                    if dc == last_dir_code:
                        prev_dir = dname
                        break
            turn_cost = 0
            if prev_dir is not None and prev_dir != task.push_dir:
                turn_cost = turn_penalty

            # 总代价
            new_cost = cost + car_dist + task.n_steps + turn_cost

            # 新状态
            new_progress = list(progress)
            new_progress[bid] = p + 1
            new_progress = tuple(new_progress)
            new_last = _DIR_CODE[task.push_dir]
            new_state = (task.car_end[0], task.car_end[1]) + new_progress + (new_last,)

            if new_cost < best.get(new_state, float('inf')):
                best[new_state] = new_cost
                new_path = path + ((bid, p, car_dist),)
                heapq.heappush(heap, (new_cost,) + new_state + (new_path,))

    return None


def generate_actions_from_schedule(
    schedule: List[Tuple[int, int, int]],
    micro_tasks: List[List[MicroTask]],
    car_start: Tuple[int, int],
    walls: Set[Tuple[int, int]],
    width: int,
    height: int,
    obstacles: Set[Tuple[int, int]],
    preprocess: MapPreprocess = None,
    max_retries: int = 3,
    avoidance_graph=None,
) -> Optional[List[Dict]]:
    """
    将微任务调度序列转换为完整的 Action 列表。

    对每个微任务，生成 free_move + push_box。
    遇到 car_bfs 失败时尝试修复（扩展/绕行）。

    avoidance_graph: optional AvoidanceGraph for composite Dijkstra shortcuts.
    """
    all_actions = []
    car_pos = car_start
    box_positions = [tasks[0].box_start for tasks in micro_tasks]
    box_done = [False] * len(micro_tasks)  # 箱子是否已全部完成

    for bid, tidx, _ in schedule:
        task = micro_tasks[bid][tidx]

        # 当前障碍物：墙 + 所有未完成的箱子（包括当前箱子——车不能穿过它）
        obs = set(walls)
        for j, bp in enumerate(box_positions):
            if not box_done[j]:
                obs.add(bp)
        # 对于非第一段的任务，当前箱位可能已经变了
        # 构建当前段的 seg
        seg = {
            'box_start': task.box_start,
            'box_end': task.box_end,
            'push_dir': task.push_dir,
            'n_steps': task.n_steps,
            'car_target': task.car_target,
            'car_end': task.car_end,
        }

        # free_move
        if car_pos != task.car_target:
            wps = None

            # 阶段二：优先使用规避节点图（复合 Dijkstra）
            if avoidance_graph is not None:
                wps = avoidance_graph.get_waypoints(car_pos, task.car_target, obs)

            if wps is None:
                # 回退：car_bfs + LOS 平滑
                cr = car_bfs(car_pos, task.car_target,
                             obs | {task.box_start}, width, height)
                if cr is None:
                    # 尝试用 generate_actions_with_retry 从当前位置推到目标
                    sub_result = generate_actions_with_retry(
                        box_positions[bid],
                        micro_tasks[bid][-1].box_end,
                        walls, width, height, obs,
                        car_pos, bid, preprocess=preprocess, max_retries=max_retries,
                        avoidance_graph=avoidance_graph,
                    )
                    if sub_result is None:
                        return None
                    sub_actions, _ = sub_result
                    all_actions.extend(sub_actions)
                    return all_actions

                _, cpath = cr
                from algorithms.box_planner import _los_smooth
                # 阶段二：LOS 路径平滑（截弯取直）
                # 非目标物体 = 未完成箱子中不是当前箱子的那些（扩展8邻域）
                non_target = {box_positions[j] for j in range(len(box_positions))
                             if not box_done[j] and j != bid}
                # 当前箱子作为硬障碍（仅自身格，不扩展——车可以相邻但不能穿过）
                current_box = {box_positions[bid]} if not box_done[bid] else set()
                los_walls = walls | current_box
                wps = _los_smooth(cpath, los_walls, non_target, width, height)
            all_actions.append({
                "type": "free_move", "target": task.car_target,
                "theta": None, "waypoints": wps, "narrow_passage": False,
            })

        # push_box 前验证目标位置未被其他箱子占据（第二道防线）
        for j, bp in enumerate(box_positions):
            if j != bid and not box_done[j] and bp == task.box_end:
                return None

        # 多格推拆分：前 N-1 步合并快速推，最后 1 步单格精准推
        dx, dy = DIRECTION_VECTORS[task.push_dir]
        if task.n_steps > 1:
            # 前 N-1 步
            n_fast = task.n_steps - 1
            mid_box = (task.box_start[0] + dx * n_fast,
                       task.box_start[1] + dy * n_fast)
            mid_car = (task.box_start[0] + dx * (n_fast - 1),
                       task.box_start[1] + dy * (n_fast - 1))
            all_actions.append({
                "type": "push_box", "target": mid_car,
                "theta": None,
                "push_meta": {
                    "box_id": bid,
                    "box_start": task.box_start,
                    "box_target": mid_box,
                    "push_dir": task.push_dir,
                },
                "narrow_passage": False,
            })
            # 最后 1 步
            all_actions.append({
                "type": "push_box", "target": task.car_end,
                "theta": None,
                "push_meta": {
                    "box_id": bid,
                    "box_start": mid_box,
                    "box_target": task.box_end,
                    "push_dir": task.push_dir,
                },
                "narrow_passage": False,
            })
        else:
            all_actions.append({
                "type": "push_box", "target": task.car_end,
                "theta": None,
                "push_meta": {
                    "box_id": bid,
                    "box_start": task.box_start,
                    "box_target": task.box_end,
                    "push_dir": task.push_dir,
                },
                "narrow_passage": False,
            })

        car_pos = task.car_end
        box_positions[bid] = task.box_end
        # 检查此箱是否全部完成
        if tidx == len(micro_tasks[bid]) - 1:
            box_done[bid] = True

    return all_actions
