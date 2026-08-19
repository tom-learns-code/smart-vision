"""录制分配 + merged walls 下, trace scheduler 到达的最深 progress + 死端原因。"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map, get_walls_in_3x3
from algorithms.path_search import generate_pair_plans
import algorithms.scheduler as SCH
from core.avoidance_graph import AvoidanceGraph
import heapq

md = load_map_from_file('maps_import/map5(4).txt')
info = scan_map(md)
walls = info['walls']
blast = get_walls_in_3x3((11, 7), walls)
merged = walls - blast

pairs = [
    {'box':(2,3),'goal':(13,7),'uses_bomb':False,'bomb_plan':None,'needs_chain':False,
     'same_region':False,'score':0,'path_steps':0,'_merged_walls':merged},
    {'box':(2,5),'goal':(13,5),'uses_bomb':False,'bomb_plan':None,'needs_chain':False,
     'same_region':False,'score':0,'path_steps':0,'_merged_walls':merged},
    {'box':(2,9),'goal':(12,6),'uses_bomb':True,
     'bomb_plan':{'bomb':(11,8),'wall':(11,7),'bomb_steps':1,'bomb_turns':0,'chain_depth':1},
     'needs_chain':False,'same_region':False,'score':0,'path_steps':0},
]
plans = generate_pair_plans(info, pairs)
total = [len(pl.box_tasks if pl.is_direct else pl.pre_box_tasks+pl.bomb_tasks+pl.post_box_tasks) for pl in plans]
print("total_per_pair:", total)

# trace heappush
max_prog = [None]
visited = set()
_orig_push = heapq.heappush
def _traced(heap, item):
    if len(item) >= 7:
        prog = item[3:6]
        visited.add(prog)
        if max_prog[0] is None or sum(prog) > sum(max_prog[0]):
            max_prog[0] = prog
    return _orig_push(heap, item)

graph = AvoidanceGraph(walls, info['width'], info['height'])
SCH.heapq.heappush = _traced
sched = SCH.schedule_pair_plans(plans, info['car'], walls, info['width'], info['height'],
                                info['boxes'], info['bombs'], avoidance_graph=graph)
SCH.heapq.heappush = _orig_push
print(f"schedule: {'OK' if sched else 'None'}")
print(f"max progress: {max_prog[0]}")
print(f"states visited: {len(visited)}")
print("deepest states:")
for pr in sorted(visited, key=lambda x: -sum(x))[:12]:
    print("  ", pr)
EOF_MARKER_REMOVE = None
