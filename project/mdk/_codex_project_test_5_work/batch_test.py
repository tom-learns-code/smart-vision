# -*- coding: utf-8 -*-
"""批量测试：每张图跑N次随机ID，输出raw+structured双轨路径"""
import sys, os, json, time, random, traceback
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.map_analysis import load_map_from_file, scan_map, assign_random_ids
from run_solver import solve_with_c_solver, solve_phase2_dijkstra, _resolve_ids

MAPS_DIR = 'maps_import'
MAPS = [
    'map0.txt',
    'map1(1).txt', 'map1(4).txt',
    'map2(1).txt', 'map2(4).txt',
    'map3(1).txt', 'map3(4).txt',
    'map4(1).txt', 'map4(4).txt',
    'map5(1).txt', 'map5(4).txt',
    'map6.txt', 'map7.txt', 'map8.txt',
    'map9.txt', 'map10.txt', 'map11.txt',
]
RUNS_PER_MAP = 3
OUT_BASE = 'batch_results'

ACT_NAMES = {0: '上', 1: '下', 2: '左', 3: '右', 4: 'BOOM', 5: '看上', 6: '看下', 7: '看左', 8: '看右'}


def detect_chain_push(actions):
    """检测连锁推：push_box[b]的target上有另一个箱子"""
    issues = []
    box_pos = {}
    for i, a in enumerate(actions):
        if a['type'] == 'push_box':
            m = a['push_meta']
            bid = m['box_id']
            if bid not in box_pos:
                box_pos[bid] = tuple(m['box_start'])
            target = tuple(m['box_target'])
            for oid, opos in box_pos.items():
                if oid != bid and opos == target:
                    issues.append({
                        'action_idx': i,
                        'pusher_box': bid,
                        'victim_box': oid,
                        'at_position': list(target),
                    })
            box_pos[bid] = target
    return issues


def summarize_structured(actions):
    """结构化action摘要"""
    out = []
    for i, a in enumerate(actions):
        t = a['type']
        if t == 'free_move':
            wps = a.get('waypoints', [])
            diags = sum(1 for j in range(1, len(wps))
                        if wps[j][0] != wps[j-1][0] and wps[j][1] != wps[j-1][1])
            out.append({
                'idx': i, 'type': 'free_move',
                'steps': len(wps) - 1,
                'diagonals': diags,
                'start': list(wps[0]) if wps else None,
                'end': list(wps[-1]) if wps else None,
                'theta': a.get('theta'),
            })
        elif t in ('push_box', 'push_bomb'):
            m = a['push_meta']
            out.append({
                'idx': i, 'type': t,
                'obj_id': m.get('box_id', m.get('bomb_id')),
                'push_dir': m['push_dir'],
                'start': list(m.get('box_start', m.get('bomb_start'))),
                'end': list(m.get('box_target', m.get('bomb_target'))),
            })
        elif t == 'observe':
            m = a['observe_meta']
            out.append({
                'idx': i, 'type': 'observe',
                'direction': m['direction'],
                'obj_type': m['object_type'],
                'obj_index': m['object_index'],
                'car_pos': list(a['target']),
            })
        elif t == 'wait':
            out.append({'idx': i, 'type': 'wait', 'reason': a.get('reason', '?')})
        elif t == 'end_of_phase1':
            out.append({'idx': i, 'type': 'end_of_phase1'})
        else:
            out.append({'idx': i, 'type': t})
    return out


def run_one_map(map_file, run_idx, seed):
    """跑一张图一次"""
    random.seed(seed)

    raw = load_map_from_file(os.path.join(MAPS_DIR, map_file))
    info = scan_map(raw)
    n_boxes = info['counts']['boxes']
    n_goals = info['counts']['goals']

    # 随机 ID
    box_ids, goal_ids = assign_random_ids(n_boxes, n_goals)
    raw['box_label_ids'] = box_ids
    raw['goal_label_ids'] = goal_ids

    result = {
        'map': map_file, 'run': run_idx, 'seed': seed,
        'counts': {'boxes': n_boxes, 'goals': n_goals, 'bombs': info['counts']['bombs']},
        'god_box_ids': {str(k): v for k, v in sorted(box_ids.items())},
        'god_goal_ids': {str(k): v for k, v in sorted(goal_ids.items())},
    }

    # P1
    t0 = time.perf_counter()
    p1_result = solve_with_c_solver(raw, verbose=False)
    p1_time = time.perf_counter() - t0

    if not p1_result['success']:
        result['error'] = f"P1 failed: {p1_result.get('error', '?')}"
        return result

    p1_res = p1_result['p1_res']
    result['p1_ok'] = True
    result['p1_time_s'] = round(p1_time, 1)

    # === RAW 路径 ===
    result['raw_path'] = [
        {'step': i, 'code': int(p1_res.path[i]),
         'name': ACT_NAMES.get(int(p1_res.path[i]), f'?({int(p1_res.path[i])})')}
        for i in range(p1_res.path_length)
    ]

    # === STRUCTURED 摘要 ===
    result['structured'] = summarize_structured(p1_res.structured_actions)

    # === P1 观察到什么 ===
    result['p1_observed_box_indices'] = sorted(p1_res.observed_box_indices)
    result['p1_observed_target_indices'] = sorted(p1_res.observed_target_indices)

    # 模拟车知道的 ID（未观察到=-1）
    sim_known_box = [-1] * n_boxes
    sim_known_target = [-1] * n_goals
    for bi in p1_res.observed_box_indices:
        if 0 <= bi < n_boxes:
            sim_known_box[bi] = box_ids.get(bi, -1)
    for ti in p1_res.observed_target_indices:
        if 0 <= ti < n_goals:
            sim_known_target[ti] = goal_ids.get(ti, -1)

    result['car_known_box_ids'] = sim_known_box
    result['car_known_target_ids'] = sim_known_target

    # _resolve_ids 推理
    resolved_box, resolved_target = _resolve_ids(sim_known_box, sim_known_target)
    result['resolved_box_ids'] = resolved_box
    result['resolved_target_ids'] = resolved_target

    # P2
    t0 = time.perf_counter()
    p2_result = solve_phase2_dijkstra(p1_res, sim_known_box, sim_known_target, raw)
    p2_time = time.perf_counter() - t0

    result['p2_ok'] = p2_result['success']
    result['p2_time_s'] = round(p2_time, 3)
    result['p2_actions_count'] = len(p2_result.get('actions', []))
    result['p2_error'] = p2_result.get('error', '')

    if p2_result['success']:
        result['p2_structured'] = summarize_structured(p2_result['actions'])
        chain = detect_chain_push(p2_result['actions'])
        result['chain_push_count'] = len(chain)
        result['chain_push_issues'] = chain

    return result


def main():
    for mp in MAPS:
        fpath = os.path.join(MAPS_DIR, mp)
        if not os.path.exists(fpath):
            continue

        safe = mp.replace('.txt', '').replace('(', '_').replace(')', '')
        out_dir = os.path.join(OUT_BASE, safe)
        os.makedirs(out_dir, exist_ok=True)

        print(f'\n=== {mp} ===')
        for run_i in range(RUNS_PER_MAP):
            seed = abs(hash(mp)) ^ (run_i * 10007)
            print(f'  第{run_i+1}次...', end=' ', flush=True)
            try:
                r = run_one_map(mp, run_i, seed)
                out_file = os.path.join(out_dir, f'run{run_i+1}.json')
                with open(out_file, 'w', encoding='utf-8') as f:
                    json.dump(r, f, ensure_ascii=False, indent=2, default=str)
                p1s = 'P1OK' if r.get('p1_ok') else 'P1FAIL'
                p2s = 'P2OK' if r.get('p2_ok') else 'P2FAIL'
                chain_n = r.get('chain_push_count', 0)
                cw = f' 连锁推:{chain_n}' if chain_n > 0 else ''
                raw_n = len(r.get('raw_path', []))
                struct_n = len(r.get('structured', []))
                print(f'{p1s} {p2s} raw:{raw_n}步 struct:{struct_n}条{cw}')
            except Exception as e:
                print(f'ERROR: {e}')
                traceback.print_exc()


if __name__ == '__main__':
    main()
