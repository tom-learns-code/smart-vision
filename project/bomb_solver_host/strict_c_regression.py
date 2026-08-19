from __future__ import annotations

import argparse
import itertools
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace


HOST_DIR = Path(__file__).resolve().parent
DEFAULT_BASELINE = Path(r"C:\Users\L\Desktop\算法\project_test_5")


def _mask_indices(mask: int) -> list[int]:
    return [index for index in range(3) if mask & (1 << index)]


def _parse_path(line: str) -> list[int]:
    fields = line.split()
    return [int(value) for value in fields[1:]]


def run_c(executable: Path, map_path: Path,
          box_ids: list[int] | None = None,
          goal_ids: list[int] | None = None) -> SimpleNamespace:
    args = [str(executable), str(map_path)]
    if box_ids is not None and goal_ids is not None:
        args.extend(str(value) for value in box_ids)
        args.extend(str(value) for value in goal_ids)
    completed = subprocess.run(args, text=True, encoding="utf-8",
                               errors="replace", capture_output=True)
    output = completed.stdout
    if "BOMB_RESULT P1_FAIL" in output:
        raise RuntimeError("C P1 failed")
    if completed.returncode != 0 or "BOMB_RESULT P1_OK" not in output:
        raise RuntimeError(f"C solver failed ({completed.returncode}):\n"
                           f"{output[-1200:]}\n{completed.stderr[-600:]}")

    result_line = next(line for line in output.splitlines()
                       if line.startswith("BOMB_RESULT P1_OK"))
    values = dict(re.findall(r"(bombs|boxes|goals|obs_box|obs_goal|end)=([^ ]+)",
                             result_line))
    blast_line = next(line for line in output.splitlines()
                      if line.startswith("P1_BLAST"))
    blasts = [tuple(map(int, token.split(',')))
              for token in blast_line.split()[1:]]
    p1_path = _parse_path(next(line for line in output.splitlines()
                               if line.startswith("P1_PATH")))
    states = []
    for line in output.splitlines():
        if not line.startswith("P1_STATE "):
            continue
        fields = [int(value) for value in line.split()[1:]]
        states.append(SimpleNamespace(
            r=fields[1], c=fields[2], mask=fields[3],
            b=fields[4:7], bx=fields[7:10]))
    if len(states) != len(p1_path) + 1:
        raise RuntimeError(f"state/path mismatch {len(states)}/{len(p1_path)}")

    p2_path = None
    if "BOMB_RESULT P2_OK" in output:
        p2_path = _parse_path(next(line for line in output.splitlines()
                                   if line.startswith("P2_PATH")))
    elif box_ids is not None:
        raise RuntimeError("C P2 failed")

    end_x, end_y = map(int, values['end'].split(','))
    return SimpleNamespace(
        path=p1_path, path_length=len(p1_path), states=states,
        bomb_count=int(values['bombs']), box_count=int(values['boxes']),
        goal_count=int(values['goals']),
        observed_box_mask=int(values['obs_box']),
        observed_goal_mask=int(values['obs_goal']),
        end_px=end_x, end_py=end_y, blasts=blasts,
        p2_path=p2_path, output=output,
    )


def build_p1_result(c_result, map_data, tp, convert_p1_actions):
    goals = sorted(map_data['goals'], key=lambda p: (p[1], p[0]))
    dot_pos = [(y << 4) | x for x, y in goals] + [255] * (3 - len(goals))
    actions, boxes, targets = convert_p1_actions(
        c_result.path, c_result.states, c_result.path_length,
        c_result.bomb_count, c_result.box_count, dot_pos,
        tp.fast_wall, tp.p1_dist_vis, tp.global_dist,
        tp.global_parent, tp.global_action,
    )
    result = SimpleNamespace(
        path=c_result.path, path_length=c_result.path_length,
        end_px=c_result.end_px, end_py=c_result.end_py,
        structured_actions=actions,
        observed_box_indices=boxes,
        observed_target_indices=targets,
    )
    if set(boxes) != set(_mask_indices(c_result.observed_box_mask)):
        raise RuntimeError("C/result box observation mask mismatch")
    if set(targets) != set(_mask_indices(c_result.observed_goal_mask)):
        raise RuntimeError("C/result goal observation mask mismatch")
    tp.g_best_bombs = [[y, x] for x, y in c_result.blasts] + [[0, 0]] * (
        3 - len(c_result.blasts))
    final = c_result.states[-1]
    tp.g_final_p1_state = tp.SimState(
        r=final.r, c=final.c, mask=final.mask,
        b=list(final.b), bx=list(final.bx))
    return result


def run_map(executable: Path, baseline: Path, map_name: str,
            permutations_limit: int = 0) -> dict:
    from core.map_analysis import load_map_from_file
    import c_solver.track_port as tp
    from c_solver.action_converter import convert_p1_actions, convert_p2_actions
    from run_solver import _resolve_ids
    from strict_regression import (
        _map_parts, replay_p1_raw, replay_p1_structured,
        replay_p2_structured, resolved_assignment,
    )

    map_path = baseline / "maps_import" / map_name
    map_data = load_map_from_file(str(map_path))
    _, _, _, boxes, _, goals, _ = _map_parts(map_data)
    started = time.perf_counter()

    first = run_c(executable, map_path)
    p1_result = build_p1_result(first, map_data, tp, convert_p1_actions)
    p1_action_count = len(p1_result.structured_actions)
    max_p2_action_count = 0
    p1_replay = replay_p1_raw(map_data, p1_result)
    replay_p1_structured(map_data, p1_result, p1_replay)

    permutations = list(itertools.permutations(range(len(goals))))
    if permutations_limit:
        permutations = permutations[:permutations_limit]
    for permutation in permutations:
        actual_box_ids = [10 + index for index in range(len(boxes))]
        actual_goal_ids = [0] * len(goals)
        for box_index, goal_index in enumerate(permutation):
            actual_goal_ids[goal_index] = actual_box_ids[box_index]
        known_boxes, known_goals, assignment = resolved_assignment(
            actual_box_ids, actual_goal_ids, p1_replay)
        resolved_boxes, resolved_goals = _resolve_ids(known_boxes, known_goals)
        c_run = run_c(executable, map_path, resolved_boxes, resolved_goals)
        if c_run.blasts != first.blasts or c_run.path != first.path:
            raise RuntimeError("C P1 changed between ID permutations")

        assigned_positions = [255] * 3
        sorted_goals = sorted(map_data['goals'], key=lambda p: (p[1], p[0]))
        for box_index, goal_index in enumerate(assignment):
            gx, gy = sorted_goals[goal_index]
            assigned_positions[box_index] = (gy << 4) | gx
        initial = first.states[-1]
        p2_initial = tp.SimState(
            r=initial.r, c=initial.c, mask=initial.mask,
            b=list(initial.b), bx=list(initial.bx))
        actions = convert_p2_actions(
            c_run.p2_path, len(c_run.p2_path), p2_initial,
            len(boxes), assigned_positions)
        max_p2_action_count = max(max_p2_action_count, len(actions))
        replay_p2_structured(map_data, p1_replay, actions, assignment)

    return {
        'map': map_name,
        'p1_steps': first.path_length,
        'p2_cases': len(permutations),
        'p1_actions': p1_action_count,
        'max_p2_actions': max_p2_action_count,
        'seconds': round(time.perf_counter() - started, 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, default=DEFAULT_BASELINE)
    parser.add_argument('--exe', type=Path, default=HOST_DIR / 'bomb_cli.exe')
    parser.add_argument('--map', action='append', dest='maps')
    parser.add_argument('--permutations', type=int, default=0)
    args = parser.parse_args()

    baseline = args.baseline.resolve()
    sys.path.insert(0, str(baseline))
    from strict_regression import MAPS

    maps = args.maps or [name for name in MAPS if name != 'map7.txt']
    failures = 0
    for map_name in maps:
        print(f"START {map_name}", flush=True)
        try:
            result = run_map(args.exe.resolve(), baseline, map_name,
                             args.permutations)
            print(f"PASS  {map_name} p1={result['p1_steps']} "
                  f"actions={result['p1_actions']}/{result['max_p2_actions']} "
                  f"cases={result['p2_cases']} total={result['seconds']}s",
                  flush=True)
        except Exception as exc:
            failures += 1
            print(f"FAIL  {map_name}: {type(exc).__name__}: {exc}", flush=True)
    print(f"SUMMARY pass={len(maps)-failures} fail={failures}", flush=True)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
