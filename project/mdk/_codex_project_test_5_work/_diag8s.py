# -*- coding: utf-8 -*-
"""map8 scheduler 卡点诊断: 单独/组合调度 pair, 看卡在哪个 progress (ASCII)"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map
from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment
from algorithms.blast_refine import refine_blast_points
from algorithms.path_search import generate_pair_plans
from algorithms import scheduler as SCH
from core.avoidance_graph import AvoidanceGraph

mp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps_export', 'map8.txt')
md = load_map_from_file(mp)
info = scan_map(md)
conn = compare_connectivity(info)
ch = analyze_channels(info, conn['ideal']['regions'])
pp = preplan_blasts(info, conn, ch)
alloc = solve_assignment(info, conn, ch, pp)
refined = refine_blast_points(info, alloc, conn, ch, pp)
plans = generate_pair_plans(info, refined['pairs'])
graph = AvoidanceGraph(info['walls'], info['width'], info['height'])

# monkeypatch heappush 记录达到的最深 progress
import heapq
_orig_push = heapq.heappush
_max_prog = {}
def _trace_push(h, item):
    # item = (cost, tiebreak, car, progress_tuple, last_dir, path)
    try:
        prog = item[3]
        key = tuple(prog)
        _max_prog[key] = _max_prog.get(key, 0) + 1
    except Exception:
        pass
    return _orig_push(h, item)

def run(sel_plans, label):
    _max_prog.clear()
    heapq.heappush = _trace_push
    try:
        res = SCH.schedule_pair_plans(sel_plans, info['car'], info['walls'],
                                      info['width'], info['height'],
                                      info['boxes'], info['bombs'],
                                      avoidance_graph=graph)
    finally:
        heapq.heappush = _orig_push
    print('--- %s: %s' % (label, ('OK len=%d' % len(res)) if res else 'None'))
    tot = [len((p.pre_box_tasks or []))+len((p.bomb_tasks or []))+len((p.post_box_tasks or []))
           if not p.is_direct else len(p.box_tasks or []) for p in sel_plans]
    print('    total_per_pair=%s' % tot)
    deepest = sorted(_max_prog.keys(), key=lambda k: sum(k))[-8:]
    print('    deepest progress states reached: %s' % deepest)

# 各种组合
import copy
def clone(pl):
    return pl
run(plans, 'ALL three pairs')
run([plans[1]], 'pair1 ONLY')
run([plans[1], plans[2]], 'pair1 + pair2')
run([plans[2]], 'pair2 ONLY')
