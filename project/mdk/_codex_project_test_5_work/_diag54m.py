# -*- coding: utf-8 -*-
"""深度trace: 3对一起调度时, 从每个可达状态尝试推进各对的失败原因.
   复刻 scheduler 内层校验逻辑, 打印每个 (progress, pair) 的 obj_path/car 校验结果。"""
import sys, os, heapq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map, get_walls_in_3x3
import algorithms.assignment as A
from algorithms.path_search import push_box_bfs
from core.distance import manhattan_distance

def _p(box, goal, walls, w, h, obstacles, same_region, car_start=None):
    manh = manhattan_distance(box, goal)
    result = push_box_bfs(box, goal, walls, w, h, obstacles, initial_car_pos=car_start)
    if result is None:
        return {'reachable': False, 'path_steps': 0, 'turns': 0, 'manhattan': manh, 'score': -1}
    steps, turns = result[0], result[1]
    return {'reachable': True, 'path_steps': steps, 'turns': turns, 'manhattan': manh, 'wall_penalty': 0, 'score': 1000-steps*10}
A._compute_pair_score = _p

from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment
from algorithms.blast_refine import refine_blast_points
from algorithms.path_search import generate_pair_plans
from algorithms.scheduler import schedule_pair_plans, _DIR_VEC
from core.bfs import bfs_shortest_path
from core.avoidance_graph import AvoidanceGraph

md = load_map_from_file('maps_import/map5(4).txt')
info = scan_map(md)
conn = compare_connectivity(info)
ch = analyze_channels(info, conn['ideal']['regions'])
pp = preplan_blasts(info, conn, ch)
alloc = solve_assignment(info, conn, ch, pp)
refined = refine_blast_points(info, alloc, conn, ch, pp)
plans = generate_pair_plans(info, refined['pairs'])
walls = info['walls']; w, h = info['width'], info['height']

# 复刻 all_tasks / blast_after / blast_clears
all_tasks = []; blast_after = []; blast_clears = []
for plan in plans:
    if plan.is_direct:
        all_tasks.append(plan.box_tasks); blast_after.append(-1); blast_clears.append(set())
    else:
        all_tasks.append(plan.pre_box_tasks + plan.bomb_tasks + plan.post_box_tasks)
        blast_after.append(len(plan.pre_box_tasks)+len(plan.bomb_tasks)-1)
        blast_clears.append(set(plan.blast_clears))
n = len(plans)
total = [len(t) for t in all_tasks]
print("total_per_pair:", total)
print("blast_after:", blast_after, " blast_clears:", [sorted(b) for b in blast_clears])

def active_walls(progress):
    ww = set(walls)
    for i in range(n):
        if blast_after[i] >= 0 and progress[i] > blast_after[i]:
            ww -= blast_clears[i]
    return ww

def all_obj(progress):
    obs = set()
    for i in range(n):
        p = progress[i]; tasks = all_tasks[i]
        if p >= len(tasks): continue
        plan = plans[i]
        if plan.is_direct:
            obs.add(plan.box if p==0 else tasks[p-1].obj_end)
        else:
            pre_len=len(plan.pre_box_tasks); ba=blast_after[i]
            if ba>=0 and p<=ba:
                if p<=pre_len: obs.add(plan.bomb)
                else: obs.add(tasks[p-1].obj_end)
            if p==0: obs.add(plan.box)
            elif p<=pre_len: obs.add(tasks[p-1].obj_end)
            elif p<=ba+1: obs.add(plan.pre_box_tasks[-1].obj_end if plan.pre_box_tasks else plan.box)
            else: obs.add(tasks[p-1].obj_end)
    return obs

# 手动检查关键状态: 想让 pair2 先炸墙(推进到 blast_after+1)
# pair2 total=5, blast_after=3 (pre2+bomb1-1=2? 重算)
print("\npair2 detail: pre=%d bomb=%d post=%d" % (
    len(plans[2].pre_box_tasks), len(plans[2].bomb_tasks), len(plans[2].post_box_tasks)))
print("pair2 blast_after idx =", blast_after[2], "-> 推进到 progress", blast_after[2]+1, "后炸墙")

# 模拟: 单独把pair2推到底, 看每步car校验
def try_advance(progress, car, i):
    """尝试推进pair i 一步, 返回 (ok, reason, new_car)"""
    p = progress[i]
    if p >= total[i]: return False, "done", car
    task = all_tasks[i][p]
    cur_walls = active_walls(progress)
    cur_obj = all_obj(progress)
    dx, dy = _DIR_VEC[task.push_dir]
    o = task.obj_start; cleared={o}
    for k in range(task.n_steps):
        nxt=(o[0]+dx,o[1]+dy); cs=(o[0]-dx,o[1]-dy)
        if cs in cur_walls or (cs in cur_obj and cs not in cleared):
            return False, f"car_stand {cs} blocked(wall={cs in cur_walls},obj={cs in cur_obj})", car
        is_last=(k==task.n_steps-1)
        if not (task.task_type=='push_bomb' and is_last):
            if nxt in cur_walls: return False, f"obj_next {nxt} is WALL", car
            if nxt in cur_obj and nxt not in cleared: return False, f"obj_next {nxt} is OBJ", car
        o=nxt; cleared.add(nxt)
    cbfs = bfs_shortest_path(car, task.car_target, cur_walls|cur_obj, w, h)
    if cbfs is None:
        return False, f"car cannot reach car_target {task.car_target} (car at {car})", car
    return True, "OK", task.car_end

# 从初始 (0,0,0) car=(3,5) 尝试各对
import itertools
prog=[0,0,0]; car=info['car']
print(f"\n=== 从初始 progress={tuple(prog)} car={car} 各对推进校验 ===")
for i in range(n):
    ok, reason, nc = try_advance(tuple(prog), car, i)
    print(f"  pair{i} task0 ({all_tasks[i][0].task_type} {all_tasks[i][0].obj_start}->{all_tasks[i][0].obj_end}): {ok} | {reason}")

# 继续: BFS式探索所有可达状态, 打印每个状态能推进哪些对
print("\n=== 全状态可达性探索 (BFS over progress) ===")
from collections import deque
# 状态: (progress_tuple, car) -- car简化只追踪关键
start = (tuple(prog), info['car'])
seen = {start}
dq = deque([start])
reachable_progs = set()
reachable_progs.add(tuple(prog))
dead_ends = []
while dq:
    (pg, cr) = dq.popleft()
    advanced_any = False
    for i in range(n):
        ok, reason, nc = try_advance(pg, cr, i)
        if ok:
            advanced_any = True
            npg = list(pg); npg[i]+=1; npg=tuple(npg)
            ns = (npg, nc)
            reachable_progs.add(npg)
            if ns not in seen:
                seen.add(ns); dq.append(ns)
    if not advanced_any:
        dead_ends.append((pg, cr))
print("可达 progress 状态数:", len(reachable_progs))
print("最深状态:")
for pg in sorted(reachable_progs, key=lambda x:-sum(x))[:8]:
    done = all(pg[i]==total[i] for i in range(n))
    print(f"   {pg} {'<-- 全完成!' if done else ''}")
print(f"\n是否存在全完成状态 {tuple(total)}:", tuple(total) in reachable_progs)
# 打印若干死端及其阻塞原因
print("\n死端状态(无法推进)样本:")
shown=set()
for pg, cr in dead_ends:
    key=pg
    if key in shown: continue
    shown.add(key)
    if len(shown)>6: break
    print(f"  progress={pg} car={cr}:")
    for i in range(n):
        if pg[i]<total[i]:
            ok,reason,_=try_advance(pg,cr,i)
            print(f"    pair{i} next({all_tasks[i][pg[i]].obj_start}->{all_tasks[i][pg[i]].obj_end}): {reason}")
