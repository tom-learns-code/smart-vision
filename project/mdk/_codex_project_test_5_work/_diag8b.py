# -*- coding: utf-8 -*-
"""map8 pair2 炸弹引爆路径 + car可达性深查 (ASCII标签)"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map, get_walls_in_3x3
from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment
from algorithms.blast_refine import refine_blast_points
from algorithms.path_search import generate_pair_plans, push_bomb_bfs, push_box_bfs
from core.bfs import bfs_shortest_path as car_bfs

mp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps_export', 'map8.txt')
md = load_map_from_file(mp)
info = scan_map(md)
walls = info['walls']; w, h = info['width'], info['height']
boxes = info['boxes']; bombs = info['bombs']; car = info['car']
all_obs = walls | set(boxes) | set(bombs)

conn = compare_connectivity(info)
ch = analyze_channels(info, conn['ideal']['regions'])
pp = preplan_blasts(info, conn, ch)
alloc = solve_assignment(info, conn, ch, pp)
refined = refine_blast_points(info, alloc, conn, ch, pp)

print('=== refined pairs bomb_plan:')
for p in refined['pairs']:
    print('  box=%s goal=%s uses_bomb=%s bomb_plan=%s' % (
        p['box'], p['goal'], p.get('uses_bomb'), p.get('bomb_plan')))

# pair2: box(2,9) bomb(2,2)
print('\n=== pair2 bomb X(2,2) push paths to various walls:')
bomb = (2, 2)
obs_b = (all_obs - walls) - {bomb}
for wall in [(2, 5), (1, 3)]:
    r = push_bomb_bfs(bomb, wall, walls, w, h, obs_b, car_start=car)
    if r is None:
        print('  wall=%s: None' % (wall,))
        continue
    steps, turns, path = r
    print('  wall=%s: steps=%d path=%s' % (wall, steps, path))
    # 手动验证car可达性: 逐段car站位
    cur_car = car
    prev = bomb
    ok = True
    for (pos, dname) in path:
        # 推 pos 方向 dname, 车站在 prev 的反方向
        dvec = {'UP': (0,-1),'DOWN':(0,1),'LEFT':(-1,0),'RIGHT':(1,0)}[dname]
        car_stand = (prev[0] - dvec[0], prev[1] - dvec[1])
        cb = car_bfs(cur_car, car_stand, all_obs, w, h)
        print('      push %s->%s %s: car needs %s from %s -> %s' % (
            prev, pos, dname, car_stand, cur_car,
            'OK d=%d' % cb[0] if cb else 'UNREACHABLE'))
        if cb is None:
            ok = False
        cur_car = prev  # 车推完跟进到物体原位
        prev = pos
    print('      => car-reachable all steps: %s' % ok)
    blast = get_walls_in_3x3(wall, walls)
    print('      blast_clears(%s)=%s' % (wall, sorted(blast)))
    # 炸后 box(2,9)->goal(2,4) 推箱
    tw = walls - blast
    pobs = (all_obs - walls) - {(2,9)} - {bomb}
    br = push_box_bfs((2,9),(2,4), tw, w, h, pobs)
    print('      box(2,9)->(2,4) after blast: %s' % (
        'OK steps=%d' % br[0] if br else 'None'))
