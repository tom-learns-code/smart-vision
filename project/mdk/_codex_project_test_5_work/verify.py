"""
算法验证脚本 — 直观可视化管线输出
用法: python verify.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.map_analysis import load_map_from_file, scan_map
from algorithms.path_analysis import compare_connectivity, analyze_channels
from algorithms.blast_select import preplan_blasts
from algorithms.assignment import solve_assignment


def show_map(md, alloc):
    """打印地图，标注盒子配对ID(0/1/2)和炸弹(B/D/chain)"""
    w, h = md['width'], md['height']
    walls = md['walls']
    boxes = md['boxes_start']
    goals = md['goals']
    bombs_list = md['bombs']
    car = md['car_start']

    # 箱ID: pos -> (idx, 'B'|'D'|'C')
    box_id = {}
    for i, p in enumerate(alloc.get('pairs', [])):
        bp = p.get('bomb_plan')
        tag = 'C' if (bp and bp.get('chain')) else ('B' if p.get('uses_bomb') else 'D')
        box_id[p['box']] = (i, tag)

    # 弹分配: pos -> pair_idx
    bomb_owner = {}
    for i, p in enumerate(alloc.get('pairs', [])):
        bp = p.get('bomb_plan')
        if bp and bp.get('bomb'):
            bomb_owner[bp['bomb']] = i

    print('   ', end='')
    for x in range(w):
        print(f'{x:2d}', end=' ')
    print()

    for y in range(h):
        print(f'{y:2d} ', end='')
        for x in range(w):
            p = (x, y)
            if p == car:
                c = '@'
            elif p in box_id:
                idx, tag = box_id[p]
                c = f'{idx}{tag}'
            elif p in goals:
                c = '.'
            elif p in bomb_owner:
                c = f'{bomb_owner[p]}*'
            elif p in bombs_list:
                c = '*'
            elif p in walls:
                c = '#'
            else:
                c = ' '
            print(f' {c} ', end='')
        print()
    print()


def show_allocation(alloc):
    """打印分配结果"""
    for i, p in enumerate(alloc.get('pairs', [])):
        bp = p.get('bomb_plan')
        box = p['box']
        goal = p['goal']

        if bp:
            chain = bp.get('chain')
            if chain:
                steps = ' -> '.join([f'b{c[0]}>>w{c[1]}' for c in chain])
                print(f'  [{i}]  箱{box} -> 目{goal}')
                print(f'       链救援({bp.get("chain_depth",1)}层): {steps}')
            else:
                bomb = bp.get('bomb', '?')
                wall = bp.get('wall', '?')
                print(f'  [{i}]  箱{box} -> 目{goal}')
                print(f'       炸弹: 弹{bomb} >> 墙{wall}')
        elif p.get('needs_chain'):
            print(f'  [{i}]  箱{box} -> 目{goal}  [需链,未解决]')
        else:
            print(f'  [{i}]  箱{box} -> 目{goal}  直达 {p["path_steps"]}步')


# ============================================================
BASE = os.path.dirname(os.path.abspath(__file__))
MAPS = [
    ('map0.txt', 'map0 (2解锁+1捷径)'),
    ('map5.txt', 'map5 (1解锁+2层链救援)'),
    ('map7.txt', 'map7 (2解锁无捷径)'),
]

for filename, desc in MAPS:
    filepath = os.path.join(BASE, 'maps_export', filename)
    md = load_map_from_file(filepath)
    info = scan_map(md)
    conn = compare_connectivity(info)
    ch = analyze_channels(info, conn['ideal']['regions'])
    pp = preplan_blasts(info, conn, ch)
    alloc = solve_assignment(info, conn, ch, pp)

    print()
    print('=' * 62)
    print(f'  {desc}')
    print(f'  {filepath}')
    print('=' * 62)

    # ---- 步骤1 ----
    c = info['counts']
    print(f'\n  [步骤1] 箱={c["boxes"]} 目={c["goals"]} 弹={c["bombs"]}')
    print(f'  箱: {info["boxes"]}  目: {sorted(info["goals"])}  弹: {info["bombs"]}')

    # ---- 步骤2b ----
    print(f'\n  [步骤2b] 理想分量={conn["ideal"]["component_count"]}  当前分量={conn["current"]["component_count"]}')
    blk = conn.get('blockers', [])
    if blk:
        print(f'  阻塞物: {[(b["type"],b["pos"]) for b in blk]}')

    # ---- 步骤3 ----
    dl_g = ch.get('deadlocked_goals', [])
    dl_bx = ch.get('deadlocked_boxes', [])
    dl_bm = ch.get('deadlocked_bombs', [])
    if dl_g:
        print(f'\n  [步骤3] 死锁目标: {[(p,t[0]) for _,p,t in dl_g]}')
    if dl_bx:
        print(f'          死锁箱子: {[(p,t[0]) for _,p,t in dl_bx]}')
    if dl_bm:
        print(f'          死锁炸弹: {[(p,t[0]) for _,p,t in dl_bm]}')
    print(f'          管道={len(ch["channels"])} 子区域={len(ch["sub_regions"])}')

    # ---- 步骤3.5 ----
    print(f'\n  [步骤3.5] 候选炸点={len(pp["blast_candidates"])}  链={len(pp.get("rescue_chains",[]))}')

    # ---- 步骤4 ----
    s = alloc['summary']
    print(f'\n  [步骤4] 分配结果:')
    show_allocation(alloc)
    print(f'\n  可解={s["is_solvable"]}  用弹={s["total_bombs_used"]}/{len(info["bombs"])}  总分={s["total_score"]:.0f}')
    unused = alloc.get('unused_bombs', [])
    if unused:
        print(f'  未用弹: {unused}')

    # ---- 地图 ----
    print(f'\n  [地图] (D=直达 B=炸弹 C=链 x*=配对x的炸弹)')
    show_map(md, alloc)

print()
