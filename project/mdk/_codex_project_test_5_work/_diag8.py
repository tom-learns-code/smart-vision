# -*- coding: utf-8 -*-
"""map8 完整管线诊断: 看当前分配/plans/schedule 卡在哪 (ASCII标签避GBK)"""
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

mp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps_export', 'map8.txt')
md = load_map_from_file(mp)
info = scan_map(md)
print('car=%s' % (info['car'],))
print('boxes=%s' % (info['boxes'],))
print('goals=%s' % (sorted(info['goals']),))
print('bombs=%s' % (info['bombs'],))

conn = compare_connectivity(info)
ch = analyze_channels(info, conn['ideal']['regions'])
pp = preplan_blasts(info, conn, ch)
alloc = solve_assignment(info, conn, ch, pp)
print('\n=== alloc summary:', alloc['summary'])
for p in alloc['pairs']:
    print('  box=%s goal=%s uses_bomb=%s needs_chain=%s bomb_plan=%s merged=%s' % (
        p['box'], p['goal'], p.get('uses_bomb'), p.get('needs_chain'),
        p.get('bomb_plan'), '_merged_walls' in p))

refined = refine_blast_points(info, alloc, conn, ch, pp)
print('\n=== plans (step6):')
plans = generate_pair_plans(info, refined['pairs'])
for pl in plans:
    print('  pair%d box=%s goal=%s is_direct=%s is_chain=%s box_tasks=%d bomb_tasks=%d pre=%d post=%d' % (
        pl.pair_id, pl.box, pl.goal, pl.is_direct, pl.is_chain,
        len(pl.box_tasks or []), len(pl.bomb_tasks or []),
        len(pl.pre_box_tasks or []), len(pl.post_box_tasks or [])))
    for t in (pl.pre_box_tasks or []):
        print('      PRE  %s->%s dir=%s n=%d' % (t.obj_start, t.obj_end, t.push_dir, t.n_steps))
    for t in (pl.bomb_tasks or []):
        print('      BOMB %s->%s dir=%s n=%d' % (t.obj_start, t.obj_end, t.push_dir, t.n_steps))
    for t in (pl.post_box_tasks or []):
        print('      POST %s->%s dir=%s n=%d' % (t.obj_start, t.obj_end, t.push_dir, t.n_steps))
    if pl.is_direct:
        for t in (pl.box_tasks or []):
            print('      BOX  %s->%s dir=%s n=%d' % (t.obj_start, t.obj_end, t.push_dir, t.n_steps))

graph = AvoidanceGraph(info['walls'], info['width'], info['height'])
sched = schedule_pair_plans(plans, info['car'], info['walls'], info['width'],
                            info['height'], info['boxes'], info['bombs'],
                            avoidance_graph=graph)
print('\n=== step7 schedule:', 'OK len=%d' % len(sched) if sched else 'None')
if sched:
    for (pid, tidx, cd) in sched:
        print('  pair%d task%d cardist=%s' % (pid, tidx, cd))
