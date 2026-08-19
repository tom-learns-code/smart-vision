"""诊断 map5(4): pairs + 各箱在合并墙态下的推箱路径(看是否走廊重叠)。"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map
from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment
from algorithms.blast_refine import refine_blast_points
from algorithms.path_search import generate_pair_plans, push_box_bfs

mp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps_import', 'map5(4).txt')
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
print('\n=== refined pairs:')
for p in refined['pairs']:
    print('  box=%s goal=%s uses_bomb=%s bomb_plan=%s merged=%s' % (
        p['box'], p['goal'], p.get('uses_bomb'),
        p.get('bomb_plan'), '_merged_walls' in p))

print('\n=== plans (step6):')
plans = generate_pair_plans(info, refined['pairs'])
for pl in plans:
    print('  pair%d box=%s goal=%s is_direct=%s is_chain=%s box_tasks=%d bomb_tasks=%d' % (
        pl.pair_id, pl.box, pl.goal, pl.is_direct, pl.is_chain,
        len(pl.box_tasks or []), len(pl.bomb_tasks or [])))
    for t in (pl.box_tasks or []):
        print('      BOX %s->%s dir=%s n=%d car_target=%s' % (
            t.obj_start, t.obj_end, t.push_dir, t.n_steps, t.car_target))
