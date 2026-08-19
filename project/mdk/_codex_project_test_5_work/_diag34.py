# -*- coding: utf-8 -*-
"""诊断 map3(4) 合入解锁型炸点后的 SOLVEFAIL 劣化:
看 step4 alloc 给三对配了什么 bomb_plan, step6 plans 哪对 tasks=0, step7 schedule 是否 None.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.map_analysis import load_map_from_file, scan_map
from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment
from algorithms.blast_refine import refine_blast_points
from algorithms.path_search import generate_pair_plans
from algorithms.scheduler import schedule_pair_plans
from core.avoidance_graph import AvoidanceGraph

mf = "maps_import/map3(4).txt"
md = load_map_from_file(mf)
info = scan_map(md)
print("boxes:", info['boxes'])
print("goals:", info['goals'])
print("bombs:", info['bombs'])
print("car:", info['car'])

conn = compare_connectivity(info)
ch = analyze_channels(info, conn['ideal']['regions'])
pp = preplan_blasts(info, conn, ch)
print("\n=== preplan ===")
print("blast_candidates count:", len(pp['blast_candidates']))
ubp = pp.get('unlock_by_pair', {})
print("unlock_by_pair keys:", list(ubp.keys()))
for k, v in ubp.items():
    print("  pair", k, "-> centers:", [(c['wall'], 'bomb=', c.get('bomb')) for c in v])

alloc = solve_assignment(info, conn, ch, pp)
print("\n=== alloc pairs ===")
for p in alloc['pairs']:
    bp = p.get('bomb_plan')
    print("  box", p['box'], "-> goal", p['goal'],
          "| uses_bomb=", p.get('uses_bomb'),
          "| bomb_plan=", None if bp is None else {'bomb': bp.get('bomb'), 'wall': bp.get('wall'), 'chain': bp.get('chain')})

refined = refine_blast_points(info, alloc, conn, ch, pp)
print("\n=== refined pairs ===")
for p in refined['pairs']:
    bp = p.get('bomb_plan')
    print("  box", p['box'], "-> goal", p['goal'],
          "| bomb_plan=", None if bp is None else {'bomb': bp.get('bomb'), 'wall': bp.get('wall'), 'chain': bp.get('chain')})

plans = generate_pair_plans(info, refined['pairs'])
print("\n=== plans (step6) ===")
for i, pl in enumerate(plans):
    print("  plan", i, "box", pl.box, "-> goal", pl.goal,
          "| is_direct=", pl.is_direct, "| is_chain=", pl.is_chain,
          "| bomb=", pl.bomb, "wall=", pl.wall,
          "| bomb_tasks=", len(pl.bomb_tasks),
          "| pre=", len(pl.pre_box_tasks), "post=", len(pl.post_box_tasks),
          "| box_tasks=", len(pl.box_tasks))

graph = AvoidanceGraph(info['walls'], info['width'], info['height'])
sched = schedule_pair_plans(plans, info['car'], info['walls'],
                            info['width'], info['height'],
                            info['boxes'], info['bombs'],
                            avoidance_graph=graph)
print("\n=== schedule (step7) ===")
print("schedule is None:", sched is None)
if sched is not None:
    print("schedule len:", len(sched) if hasattr(sched, '__len__') else '?')
