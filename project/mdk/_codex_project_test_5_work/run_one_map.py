# -*- coding: utf-8 -*-
"""单张地图求解脚本，输出JSON结果文件"""
import sys, json, time, os

map_path = sys.argv[1]
out_path = sys.argv[2]

sys.path.insert(0, '.')
from c_solver import read_map, reset_globals, run_phase1, PhaseResult
from c_solver.track_port import g_best_bombs, p1_exec_cnt

raw_map = read_map(map_path)
reset_globals()
p1_res = PhaseResult()
t0 = time.perf_counter()
success = run_phase1(raw_map, p1_res, mode=4, skip_flag=0)
elapsed = time.perf_counter() - t0

map_name = os.path.basename(map_path).replace('.txt', '')

result = {
    'map': map_name,
    'p1_ok': success,
    'p1_steps': p1_res.path_length if success else 0,
    'p1_path': [int(x) for x in p1_res.path[:p1_res.path_length]] if success else [],
    'bombs_r': [int(g_best_bombs[i][0]) for i in range(3)] if success else [],
    'bombs_c': [int(g_best_bombs[i][1]) for i in range(3)] if success else [],
    'end_px': p1_res.end_px if success else 0,
    'end_py': p1_res.end_py if success else 0,
    'elapsed_s': round(elapsed, 2),
    'p1_exec_cnt': p1_exec_cnt,
}

with open(out_path, 'w', encoding='utf-8') as f:
    json.dump(result, f, ensure_ascii=False)
print(f'{map_name}: OK={success} steps={result["p1_steps"]} time={elapsed:.1f}s → {out_path}')
