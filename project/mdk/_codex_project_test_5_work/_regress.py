"""批量回归: 跑全部地图, 输出 verify 状态 + step4/总耗时。"""
import sys, os, glob, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.map_analysis import load_map_from_file, scan_map
from run_solver import solve, verify_actions

base = os.path.dirname(os.path.abspath(__file__))
maps = sorted(glob.glob(os.path.join(base, 'maps_export', '*.txt')))
maps += sorted(glob.glob(os.path.join(base, 'maps_import', '*.txt')))

ok_cnt = fail_cnt = 0
slow = []
rows = []
for mp in maps:
    name = os.path.relpath(mp, base).replace('\\', '/')
    try:
        md = load_map_from_file(mp)
        info = scan_map(md)
        res = solve(md, verbose=False)
        if not res['success']:
            rows.append((name, 'SOLVEFAIL', res.get('error', ''), 0, 0))
            fail_cnt += 1
            continue
        ok, msg = verify_actions(res['actions'], info)
        t = res['timing']
        status = 'OK' if ok else 'VERIFAIL'
        if ok:
            ok_cnt += 1
        else:
            fail_cnt += 1
        if t['step4'] > 500 or t['total'] > 500:
            slow.append(name)
        rows.append((name, status, msg, t['step4'], t['total']))
    except Exception as e:
        rows.append((name, 'EXCEPTION', repr(e), 0, 0))
        fail_cnt += 1

print('%-22s %-10s %-8s %-8s %s' % ('MAP', 'STATUS', 'STEP4ms', 'TOTms', 'MSG'))
print('-' * 90)
for name, status, msg, s4, tot in rows:
    print('%-22s %-10s %-8.0f %-8.0f %s' % (name, status, s4, tot, msg))
print('-' * 90)
print('OK=%d FAIL=%d  (total %d maps)' % (ok_cnt, fail_cnt, len(maps)))
if slow:
    print('SLOW(>500ms): ' + ', '.join(slow))
