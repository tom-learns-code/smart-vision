# -*- coding: utf-8 -*-
"""按顺序跑所有测试图，每张图的结果单独保存"""
import sys, json, time, os, traceback

sys.path.insert(0, '.')
from c_solver import read_map, reset_globals, run_phase1, PhaseResult
import c_solver.track_port as tp  # 通过模块访问变量，避免reset_globals后引用失效

MAPS = [
    'maps_import/map1(4).txt',
    'maps_import/map2(4).txt',
    'maps_import/map3(4).txt',
    'maps_import/map4(4).txt',
    'maps_import/map5(4).txt',
    'maps_import/map8.txt',
    'maps_import/map11.txt',
]

OUT_DIR = 'results/round_S2sort'
os.makedirs(OUT_DIR, exist_ok=True)

for mp in MAPS:
    name = os.path.basename(mp).replace('.txt', '')
    out_path = os.path.join(OUT_DIR, name + '.json')
    print(f'\n{"="*50}')
    print(f'>>> {name} ...', flush=True)
    try:
        raw_map = read_map(mp)
        reset_globals()
        p1_res = PhaseResult()
        t0 = time.perf_counter()
        success = run_phase1(raw_map, p1_res, mode=4, skip_flag=0)
        elapsed = time.perf_counter() - t0

        result = {
            'map': name,
            'p1_ok': success,
            'p1_steps': p1_res.path_length if success else 0,
            'p1_path': [int(x) for x in p1_res.path[:p1_res.path_length]] if success else [],
            'bombs_r': [int(tp.g_best_bombs[i][0]) for i in range(3)] if success else [],
            'bombs_c': [int(tp.g_best_bombs[i][1]) for i in range(3)] if success else [],
            'end_px': p1_res.end_px if success else 0,
            'end_py': p1_res.end_py if success else 0,
            'elapsed_s': round(elapsed, 2),
            'p1_exec_cnt': tp.p1_exec_cnt,
        }

        with open(out_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False)
        print(f'<<< {name}: OK={success} steps={result["p1_steps"]} time={elapsed:.1f}s', flush=True)
    except Exception as e:
        print(f'<<< {name}: ERROR - {e}', flush=True)
        traceback.print_exc()

print('\nAll done.')
