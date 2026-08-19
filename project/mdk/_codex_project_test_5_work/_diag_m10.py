# -*- coding: utf-8 -*-
"""map10 三解对照: 验证箱子分配 + 炸点选择 + 总代价."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map, get_walls_in_3x3
from algorithms.path_search import push_box_bfs, push_bomb_bfs

md = load_map_from_file('maps_export/map10.txt')
info = scan_map(md)
walls = set(info['walls']); w,h = info['width'], info['height']
car = info['car']
boxes = [(2,9),(3,8),(8,8)]
bombs = [(4,7),(10,7),(12,8)]

def box_cost(box, goal, blast_centers, free_bombs, other_boxes):
    """炸掉blast_centers的3x3后, box->goal的push_box_bfs代价."""
    tw = set(walls)
    for c in blast_centers:
        tw -= get_walls_in_3x3(c, walls)
    obs = (set(other_boxes)) | (set(bombs) - set(free_bombs))
    r = push_box_bfs(box, goal, tw, w, h, obs, initial_car_pos=car)
    return r, sorted(set(walls)-tw)

print('='*60)
print('三个解的箱子分配 (来自录制/算法解析):')
print('  算法 : a(2,9)->g1(4,1)[?]  b(3,8)->g2(6,10)直达  c(8,8)->g3(11,1)炸(11,6)')
print('  001  : a(2,9)->g1(4,1)      b(3,8)->g3(11,1)      c(8,8)->g2(6,10)')
print('  003  : a(2,9)->g1(4,1)      b(3,8)->g2(6,10)      c(8,8)->g3(11,1)炸(13,4)')
print('='*60)

# g(4,1)需要炸什么? a(2,9)->g1(4,1). 录制都炸(4,7)区? 看001/003 frame5-7炸弹X(4,7)推到(4,4)
print()
print('--- a(2,9)->g1(4,1): 炸弹X(4,7)上推引爆 ---')
for c in [(4,4),(4,5),(4,3)]:
    r,cw = box_cost((2,9),(4,1),[c],[(4,7)],[(3,8),(8,8)])
    print(f'  炸(4,7)->引爆@{c} 清{cw}: ' + ('None' if r is None else f'box steps={r[0]} turns={r[1]}'))

print()
print('--- g3(11,1): 哪个box+哪个炸点最优 ---')
for box in [(3,8),(8,8)]:
    for c in [(11,6),(12,7),(13,4),(13,7)]:
        # 用各炸弹试
        for bomb in bombs:
            r,cw = box_cost(box,(11,1),[c],[bomb],[bb for bb in boxes if bb!=box])
            if r is not None:
                print(f'  box{box}->g(11,1) 炸@{c}(清{cw}) 用弹{bomb}: box steps={r[0]} turns={r[1]}')

print()
print('--- g2(6,10): 各box直达代价(无需炸墙) ---')
for box in boxes:
    r,_ = box_cost(box,(6,10),[],[],[bb for bb in boxes if bb!=box])
    print(f'  box{box}->g(6,10): ' + ('None' if r is None else f'steps={r[0]} turns={r[1]}'))
