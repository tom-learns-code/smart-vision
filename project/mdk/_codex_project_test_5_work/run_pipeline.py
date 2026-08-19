"""
算法管线验证脚本
运行步骤 1-3.5，打印所有输出数据
"""

import sys, os

sys.stdout.reconfigure(encoding='utf-8')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.map_analysis import load_map_from_file, scan_map
from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment


def print_sep(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def print_step1(info):
    print_sep("步骤 1: 地图扫描")
    c = info['counts']
    print(f"  尺寸: {info['width']} x {info['height']}")
    print(f"  墙体: {c['walls']}  箱子: {c['boxes']}  目标: {c['goals']}  炸弹: {c['bombs']}")
    print(f"  车: {info['car']}")
    print(f"  箱子位置: {info['boxes']}")
    print(f"  目标位置: {sorted(info['goals'])}")
    print(f"  炸弹位置: {info['bombs']}")
    print(f"  可走格: {len(info['walkable'])}")


def print_step2b(conn):
    print_sep("步骤 2b: 连通分量对比")
    print(f"  理想连通: {conn['ideal']['component_count']} 分量, 全通={conn['ideal']['is_single']}")
    for i, reg in enumerate(conn['ideal']['regions']):
        print(f"    分量{i}: {len(reg)} 格")
    print(f"  当前连通: {conn['current']['component_count']} 分量, 全通={conn['current']['is_single']}")
    for i, reg in enumerate(conn['current']['regions']):
        print(f"    分量{i}: {len(reg)} 格")
    print(f"  element_regions: car={conn['element_regions']['car']}")
    print(f"    boxes: {conn['element_regions']['boxes']}")
    print(f"    goals: {conn['element_regions']['goals']}")
    print(f"    bombs: {conn['element_regions']['bombs']}")
    print(f"  被阻塞: {conn['is_blocked']}")
    if conn['blockers']:
        print(f"  阻塞物 ({len(conn['blockers'])}):")
        for bl in conn['blockers']:
            print(f"    {bl['type']} @ {bl['pos']} → 释放区域 {bl['blocks_regions']}")
    print(f"  区域失衡:")
    for imb in conn['region_imbalance']:
        flag = " ⚠" if imb['boxes_count'] != imb['goals_count'] else ""
        print(f"    分量{imb['region_id']}: {imb['boxes_count']}箱 vs {imb['goals_count']}目标{flag}")


def print_step3(ch):
    print_sep("步骤 3: 管道/窄口分析")
    from collections import Counter
    tc = Counter(ch['grid_types'].values())
    print(f"  格子分类: open={tc.get('open',0)} channel={tc.get('channel',0)} "
          f"corner={tc.get('corner',0)} dead_end={tc.get('dead_end',0)} dead={tc.get('dead',0)}")
    print(f"  管道数: {len(ch['channels'])}")
    for i, c in enumerate(ch['channels']):
        ex = [f"{e['state']}" for e in c['exit_states']]
        print(f"    管{i}: len={c['length']:2d} {c['orientation']} "
              f"passable={str(c['passable']):5s} exits={ex}")
    print(f"  子区域数: {len(ch['sub_regions'])}")
    for i, sr in enumerate(ch['sub_regions']):
        print(f"    子区域{i}: {len(sr['tiles'])} 格, parent={sr['parent_region']}")
    print(f"  可通子区域对: {len(ch['passable_pairs'])}")
    for a, b, ok in ch['passable_pairs']:
        print(f"    ({a},{b}) passable={ok}")
    print(f"  死锁箱子 ({len(ch['deadlocked_boxes'])}):")
    for idx, pos, types in ch['deadlocked_boxes']:
        print(f"    boxes[{idx}]={pos} → {types}")
    print(f"  死锁炸弹 ({len(ch['deadlocked_bombs'])}):")
    for idx, pos, types in ch['deadlocked_bombs']:
        print(f"    bombs[{idx}]={pos} → {types}")
    print(f"  死锁目标 ({len(ch.get('deadlocked_goals', []))}):")
    for idx, pos, types in ch.get('deadlocked_goals', []):
        print(f"    goals[{idx}]={pos} → {types}")


def print_step35(pp):
    print_sep("步骤 3.5: 炸点预规划")
    print(f"  主分量: {pp['main_region']}")
    print(f"  候选炸点 ({len(pp['blast_candidates'])}):")
    for i, c in enumerate(pp['blast_candidates']):
        b = c['benefit']
        extra = ""
        if c.get('frees_element'):
            extra = f" frees={c['frees_element']}"
        print(f"    {i:2d}. wall={c['wall']} "
              f"p0a={b['p0a']} p0b={b['p0b']} p0c={b['p0c']} "
              f"p1a={b['p1a']} p1b={b['p1b']} p2={b['p2']} "
              f"depth={c['chain_depth']} unlocks_bomb={c['unlocks_bomb']} "
              f"score={c['score']:.1f}{extra}")
    print(f"\n  连锁救援 ({len(pp['rescue_chains'])}):")
    for rc in pp['rescue_chains']:
        print(f"    depth={rc['chain_depth']} target={rc['terminal_target']}")
        for j, (b, w) in enumerate(rc['sequence']):
            print(f"      {j}: bomb{b} → wall{w}")
    print(f"\n  分量相邻对 ({len(pp['region_adjacent_pairs'])}):")
    for ap in pp['region_adjacent_pairs']:
        print(f"    {ap['regions'][0]}↔{ap['regions'][1]} via wall{ap['wall']}")


def print_step4(alloc):
    print_sep("步骤 4: 箱子-目标分配")
    pairs = alloc.get('pairs', [])
    if not pairs:
        print("  无有效配对！")
        return
    print(f"  配对结果 ({len(pairs)} 对):")
    for i, p in enumerate(pairs):
        bp = p.get('bomb_plan')
        nc = p.get('needs_chain', False)
        if bp:
            chain = bp.get('chain')
            chain_str = ""
            if chain:
                chain_str = " CHAIN:" + "->".join([f"b{b}->w{w}" for b,w in chain])
            print(f"    {i}. box{p['box']} -> goal{p['goal']}  [炸弹] "
                  f"bomb{bp['bomb']}->wall{bp['wall']} "
                  f"score={p['score']:.0f} steps={p['path_steps']} depth={bp.get('chain_depth',1)}{chain_str}")
        elif nc:
            print(f"    {i}. box{p['box']} -> goal{p['goal']}  [需连锁救援] "
                  f"score={p['score']:.0f}")
        else:
            print(f"    {i}. box{p['box']} -> goal{p['goal']}  [直达] "
                  f"score={p['score']:.0f} steps={p['path_steps']} same_region={p['same_region']}")
    s = alloc['summary']
    chain_msg = ", 需连锁救援!" if s.get('needs_chain_rescue') else ""
    print(f"\n  总结: 可解={s['is_solvable']}{chain_msg}, 用弹={s.get('total_bombs_used','?')}, "
          f"总分={s.get('total_score','?')}")
    if alloc.get('unused_bombs'):
        print(f"  未用炸弹: {alloc['unused_bombs']}")


def run_map(filepath, label):
    print(f"\n{'#'*60}")
    print(f"#  {label}: {filepath}")
    print(f"{'#'*60}")

    md = load_map_from_file(filepath)
    info = scan_map(md)

    print_step1(info)

    conn = compare_connectivity(info)
    print_step2b(conn)

    ch = analyze_channels(info, conn['ideal']['regions'])
    print_step3(ch)

    pp = preplan_blasts(info, conn, ch)
    print_step35(pp)

    alloc = solve_assignment(info, conn, ch, pp)
    print_step4(alloc)


if __name__ == '__main__':
    maps = [
        ('maps_export/map0.txt', 'map0'),
        ('maps_export/map5.txt', 'map5 (互锁场景)'),
    ]
    for path, label in maps:
        full = os.path.join(os.path.dirname(__file__), path)
        run_map(full, label)
        print()
