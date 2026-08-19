"""map5(4) 可行性矩阵: 各 box×goal 在 (无炸弹 / 各单炸弹3x3墙态) 下 push_box_bfs 是否可达。"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map, get_walls_in_3x3, is_boundary_wall
from algorithms.path_search import push_box_bfs

mp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps_import', 'map5(4).txt')
md = load_map_from_file(mp)
info = scan_map(md)
walls = set(info['walls']); w, h = info['width'], info['height']
boxes = info['boxes']; goals = sorted(info['goals']); bombs = info['bombs']
car = info['car']
print('car=%s boxes=%s goals=%s bombs=%s' % (car, boxes, goals, bombs))

# 枚举可破坏墙(非边界)作为引爆中心 -> 合并墙态; 也含"无炸弹"
def gen_wall_states():
    states = [('none', frozenset(walls))]
    # 每个非边界墙格 c 作为引爆中心(炸弹推到 c 或推到空格使 c 在3x3内)
    cells = set()
    for x in range(w):
        for y in range(h):
            cells.add((x, y))
    for c in cells:
        blast = get_walls_in_3x3(c, walls)
        if blast:  # 该引爆点能炸到墙
            ns = frozenset(walls - blast)
            states.append(('blast@%s' % (c,), ns))
    # 去重墙态
    seen = {}
    for name, st in states:
        if st not in seen:
            seen[st] = name
    return [(n, st) for st, n in seen.items()]

states = gen_wall_states()
print('wall states(去重):', len(states))

base_obs = (walls | set(boxes) | set(bombs)) - walls  # boxes+bombs
for bi, box in enumerate(boxes):
    for gi, goal in enumerate(goals):
        ok_states = []
        for name, st in states:
            obs = base_obs - {box} - (set(boxes) - {box})  # 豁免其他箱
            obs = obs - set(bombs)  # 炸弹会被推走/引爆
            r = push_box_bfs(box, goal, set(st), w, h, obs, initial_car_pos=car)
            if r is not None:
                ok_states.append(name)
        tag = 'REACH' if ok_states else 'NONE'
        print('  box%s->goal%s : %s  (%d states ok) %s' % (
            box, goal, tag, len(ok_states), ok_states[:4]))
