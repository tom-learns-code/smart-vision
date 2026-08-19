# -*- coding: utf-8 -*-
"""解析录制 Action 序列, 提取箱子分配/炸点/距离统计, 与算法解对照."""
import json, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def analyze(fname):
    with open(fname, encoding='utf-8') as fp:
        acts = json.load(fp)
    print('='*70)
    print('FILE:', fname, '| frames:', len(acts))
    print('='*70)

    # 统计
    free_dist = box_dist = bomb_dist = 0
    n_free = n_pushbox = n_pushbomb = n_wait = 0
    box_pushes = {}   # box_id -> [(start, end, dir), ...]
    bomb_pushes = {}  # 追踪推弹

    for i, a in enumerate(acts):
        t = a['type']
        if t == 'free_move':
            n_free += 1
            wps = a.get('waypoints', [])
            for j in range(1, len(wps)):
                free_dist += manhattan(tuple(wps[j-1]), tuple(wps[j]))
        elif t == 'push_box':
            n_pushbox += 1
            m = a['push_meta']
            bs, bt = tuple(m['box_start']), tuple(m['box_target'])
            box_dist += manhattan(bs, bt)
            bid = m['box_id']
            box_pushes.setdefault(bid, []).append((bs, bt, m['push_dir']))
        elif t == 'push_bomb':
            n_pushbomb += 1
            m = a.get('push_meta', {})
            bs = tuple(m.get('box_start', m.get('bomb_start', (0,0))))
            bt = tuple(m.get('box_target', m.get('bomb_target', (0,0))))
            bomb_dist += manhattan(bs, bt)
            bomb_pushes.setdefault(i, []).append((bs, bt, m))
        elif t == 'wait':
            n_wait += 1

    print('--- 动作统计 ---')
    print(f'  free_move={n_free} free_dist={free_dist}')
    print(f'  push_box ={n_pushbox} box_dist={box_dist}')
    print(f'  push_bomb={n_pushbomb} bomb_dist={bomb_dist}')
    print(f'  wait={n_wait}')
    print(f'  TOTAL_DIST = {free_dist + box_dist + bomb_dist}')

    print('--- 箱子推动轨迹 (box_id: start -> ... -> end) ---')
    for bid in sorted(box_pushes):
        segs = box_pushes[bid]
        start = segs[0][0]
        end = segs[-1][1]
        print(f'  box_id={bid}: {start} -> {end}  ({len(segs)} segs)')
        for s, e, d in segs:
            print(f'      {s} -> {e} [{d}]')

    print('--- 炸弹推动 (帧idx: meta) ---')
    for idx in sorted(bomb_pushes):
        for bs, bt, m in bomb_pushes[idx]:
            print(f'  frame{idx}: {bs} -> {bt}  meta={m}')
    print()

for f in ['recorded_001.json', 'recorded_003.json']:
    analyze(f)
