"""诊断: 测试录制分配下步骤6/7的表现。
录制分配: pair0(2,3)->(13,7), pair1(2,5)->(13,5), pair2(2,9)->(12,6)+bomb=(11,8) wall=(11,7)
目的: 确认录制分配是否能在当前代码下产出正确 plans 并 schedule 成功。
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map, get_walls_in_3x3
from algorithms.path_search import generate_pair_plans, push_box_bfs
from algorithms.scheduler import schedule_pair_plans
from core.avoidance_graph import AvoidanceGraph

md = load_map_from_file('maps_import/map5(4).txt')
info = scan_map(md)
walls = info['walls']

# --- 构造录制分配（目标对调：(2,3)<->(2,9)）---
# 炸弹(11,8)上推到(11,7)引爆，炸开(11,6)(12,7)
recorded_pairs = [
    {'box': (2, 3), 'goal': (13, 7), 'uses_bomb': False, 'bomb_plan': None,
     'needs_chain': False, 'same_region': False, 'score': 0, 'path_steps': 0},
    {'box': (2, 5), 'goal': (13, 5), 'uses_bomb': False, 'bomb_plan': None,
     'needs_chain': False, 'same_region': False, 'score': 0, 'path_steps': 0},
    {'box': (2, 9), 'goal': (12, 6), 'uses_bomb': True,
     'bomb_plan': {'bomb': (11, 8), 'wall': (11, 7), 'bomb_steps': 1, 'bomb_turns': 0, 'chain_depth': 1},
     'needs_chain': False, 'same_region': False, 'score': 0, 'path_steps': 0},
]

# pair0 依赖 pair2 炸开(12,7): 用 merged_walls
blast_clears_pair2 = get_walls_in_3x3((11, 7), walls)
print(f"pair2 blast_clears (炸(11,7)): {sorted(blast_clears_pair2)}")
merged = walls - blast_clears_pair2
recorded_pairs[0]['_merged_walls'] = merged

print("\n=== 步骤6: generate_pair_plans ===")
plans = generate_pair_plans(info, recorded_pairs)
for pl in plans:
    print(f"  pair{pl.pair_id} box={pl.box} goal={pl.goal} is_direct={pl.is_direct} "
          f"box_tasks={len(pl.box_tasks or [])} bomb_tasks={len(pl.bomb_tasks or [])} "
          f"pre={len(pl.pre_box_tasks)} post={len(pl.post_box_tasks)}")
    for t in (pl.pre_box_tasks + pl.bomb_tasks + pl.post_box_tasks) if not pl.is_direct else (pl.box_tasks or []):
        print(f"      {t.task_type} {t.obj_start}->{t.obj_end} dir={t.push_dir} n={t.n_steps}")

# 空plan检查
if any(len((pl.box_tasks or []))==0 and len((pl.bomb_tasks or []))==0 and len(pl.pre_box_tasks)==0 for pl in plans):
    print("\n[WARN] 有空plan! 检查具体原因...")
    # 直接测push_box_bfs看哪个对失败
    for i, p in enumerate(recorded_pairs):
        if p.get('uses_bomb'): continue
        eff_walls = p.get('_merged_walls', walls)
        obs = set(info['bombs']) - {(11,8)}  # unused_bombs
        r = push_box_bfs(p['box'], p['goal'], eff_walls, info['width'], info['height'], obs)
        print(f"  pair{i} push_box_bfs({'merged' if '_merged_walls' in p else 'orig'} walls): {'OK steps='+str(r[0]) if r else 'None'}")

print("\n=== 步骤7: schedule ===")
graph = AvoidanceGraph(walls, info['width'], info['height'])
sched = schedule_pair_plans(plans, info['car'], walls, info['width'], info['height'],
                            info['boxes'], info['bombs'], avoidance_graph=graph)
print(f"schedule: {'OK len='+str(len(sched)) if sched else 'None'}")
if sched:
    for pair_id, task_idx, car_dist in sched:
        pl = plans[pair_id]
        tasks = (pl.box_tasks if pl.is_direct else pl.pre_box_tasks+pl.bomb_tasks+pl.post_box_tasks)
        t = tasks[task_idx]
        print(f"  pair{pair_id} task{task_idx} {t.task_type} {t.obj_start}->{t.obj_end} car_dist={car_dist:.1f}")
