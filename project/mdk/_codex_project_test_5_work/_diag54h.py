# -*- coding: utf-8 -*-
"""修正版: 正确读 box_tasks(直达对存这字段), 并深查 schedule=None 原因"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map
import algorithms.assignment as A
from algorithms.path_search import push_box_bfs
from core.distance import manhattan_distance

PATCH = (len(sys.argv) > 1 and sys.argv[1] == 'patch')
if PATCH:
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
    print("=== PATCHED ===")
else:
    print("=== BASELINE ===")

from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment
from algorithms.blast_refine import refine_blast_points
from algorithms.path_search import generate_pair_plans
from algorithms.scheduler import schedule_pair_plans
from core.avoidance_graph import AvoidanceGraph

md = load_map_from_file('maps_import/map5(4).txt')
info = scan_map(md)
conn = compare_connectivity(info)
ch = analyze_channels(info, conn['ideal']['regions'])
pp = preplan_blasts(info, conn, ch)
alloc = solve_assignment(info, conn, ch, pp)
refined = refine_blast_points(info, alloc, conn, ch, pp)
plans = generate_pair_plans(info, refined['pairs'])

print("\n--- plans (正确读 box_tasks/pre/post/bomb) ---")
for pl in plans:
    bt = len(pl.box_tasks)
    pre = len(pl.pre_box_tasks); post = len(pl.post_box_tasks)
    nb = len(pl.bomb_tasks)
    total_micro = bt + pre + post + nb
    print(f"  box{pl.box}->goal{pl.goal} is_direct={pl.is_direct} bomb={pl.bomb} wall={pl.wall}")
    print(f"      box_tasks={bt} pre={pre} post={post} bomb_tasks={nb} TOTAL_micro={total_micro}")
    for t in pl.box_tasks:
        print(f"        BOX {t.obj_start}->{t.obj_end} dir={t.push_dir} car_target={t.car_target}")
    for t in pl.bomb_tasks:
        print(f"        BOMB {t.obj_start}->{t.obj_end} dir={t.push_dir} car_target={t.car_target}")
    for t in pl.post_box_tasks:
        print(f"        POST {t.obj_start}->{t.obj_end} dir={t.push_dir} car_target={t.car_target}")

graph = AvoidanceGraph(info['walls'], info['width'], info['height'])
sched = schedule_pair_plans(plans, info['car'], info['walls'], info['width'],
                            info['height'], info['boxes'], info['bombs'],
                            avoidance_graph=graph)
print(f"\n--- schedule: {'OK len='+str(len(sched)) if sched else 'None (FAIL)'} ---")
