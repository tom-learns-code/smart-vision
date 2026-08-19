# -*- coding: utf-8 -*-
"""instrument scheduler 找 map5(4) patched 下能到达的最深 progress 状态"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map
import algorithms.assignment as A
import algorithms.scheduler as SCH
from algorithms.path_search import push_box_bfs
from core.distance import manhattan_distance

def _p(box, goal, walls, w, h, obstacles, same_region, car_start=None):
    manh = manhattan_distance(box, goal)
    result = push_box_bfs(box, goal, walls, w, h, obstacles, initial_car_pos=car_start)
    if result is None:
        return {'reachable': False, 'path_steps': 0, 'turns': 0, 'manhattan': manh, 'score': -1}
    steps, turns = result[0], result[1]
    wp = A._calculate_wall_penalty(box, goal, walls, w, h)
    score = 1000 + (200 if same_region else 0) - steps*10 - turns*30 - manh*1 - wp*8
    return {'reachable': True, 'path_steps': steps, 'turns': turns, 'manhattan': manh, 'wall_penalty': wp, 'score': score}
A._compute_pair_score = _p

# wrap schedule_pair_plans 内部 heap 探索: 改用打补丁记录 max progress
import heapq
_orig = SCH.schedule_pair_plans
# 直接重写一个带探针的版本太复杂; 改为在 best 上做后处理:
# 简单办法: monkeypatch heapq.heappush 在 scheduler 模块, 记录所有 progress

max_prog = [None]
visited_progs = set()
_orig_push = heapq.heappush
def _traced_push(heap, item):
    # item = (cost, cx, cy, p0,p1,p2, last_code, path)  for n=3
    if len(item) >= 7:
        prog = item[3:6]
        visited_progs.add(prog)
        s = sum(prog)
        if max_prog[0] is None or s > sum(max_prog[0]):
            max_prog[0] = prog
    return _orig_push(heap, item)

from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment
from algorithms.blast_refine import refine_blast_points
from algorithms.path_search import generate_pair_plans
from core.avoidance_graph import AvoidanceGraph

md = load_map_from_file('maps_import/map5(4).txt')
info = scan_map(md)
conn = compare_connectivity(info)
ch = analyze_channels(info, conn['ideal']['regions'])
pp = preplan_blasts(info, conn, ch)
alloc = solve_assignment(info, conn, ch, pp)
refined = refine_blast_points(info, alloc, conn, ch, pp)
plans = generate_pair_plans(info, refined['pairs'])
total = [len(pl.box_tasks)+len(pl.pre_box_tasks)+len(pl.post_box_tasks)+len(pl.bomb_tasks) for pl in plans]
# 实际 total_per_pair: scheduler 内部 all_tasks = pre+bomb+post (direct=box_tasks)
print("plans total micro per pair:", total)
print("pair order:", [(pl.box, pl.goal) for pl in plans])

SCH.heapq.heappush = _traced_push
graph = AvoidanceGraph(info['walls'], info['width'], info['height'])
sched = SCH.schedule_pair_plans(plans, info['car'], info['walls'], info['width'],
                                info['height'], info['boxes'], info['bombs'],
                                avoidance_graph=graph)
SCH.heapq.heappush = _orig_push
print(f"schedule: {'OK' if sched else 'None'}")
print(f"max progress reached: {max_prog[0]}  (total targets: see above)")
print(f"distinct progress states visited: {len(visited_progs)}")
# 打印所有访问到的"接近完成"状态
print("deepest states:")
for pr in sorted(visited_progs, key=lambda x: -sum(x))[:15]:
    print("  ", pr)
