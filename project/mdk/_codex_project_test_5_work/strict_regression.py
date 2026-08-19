# -*- coding: utf-8 -*-
"""Independent rule replay and full-map regression for the contest solver.

The solver's own success flag is intentionally not trusted here.  P1 raw
codes and both phases' structured actions are replayed against a separate,
small rules engine.  map7 is an explicit expected exception.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import itertools
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from core.map_analysis import load_map_from_file
from run_solver import solve_with_c_solver, solve_phase2_dijkstra, _resolve_ids


Pos = Tuple[int, int]
DIRS: Tuple[Pos, ...] = ((0, -1), (0, 1), (-1, 0), (1, 0))
DIR_NAMES = ('UP', 'DOWN', 'LEFT', 'RIGHT')
DIR_THETA = (-90.0, 90.0, 180.0, 0.0)
BASES: Tuple[Pos, Pos] = ((1, 6), (14, 6))
MAPS = (
    'map0.txt',
    'map1(1).txt', 'map1(4).txt',
    'map2(1).txt', 'map2(4).txt',
    'map3(1).txt', 'map3(4).txt',
    'map4(1).txt', 'map4(4).txt',
    'map5(1).txt', 'map5(4).txt',
    'map6.txt', 'map7.txt', 'map8.txt',
    'map9.txt', 'map10.txt', 'map11.txt',
)


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def add(a: Pos, b: Pos) -> Pos:
    return a[0] + b[0], a[1] + b[1]


def sub(a: Pos, b: Pos) -> Pos:
    return a[0] - b[0], a[1] - b[1]


def in_bounds(pos: Pos, width: int, height: int) -> bool:
    return 0 <= pos[0] < width and 0 <= pos[1] < height


def angle_error(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def observation_quota(count: int) -> int:
    return max(0, count - 1)


def explode_walls(walls: Set[Pos], center: Pos,
                  width: int, height: int) -> None:
    cx, cy = center
    for y in range(cy - 1, cy + 2):
        for x in range(cx - 1, cx + 2):
            if 0 < x < width - 1 and 0 < y < height - 1:
                walls.discard((x, y))


@dataclass
class P1Replay:
    car: Pos
    boxes: List[Optional[Pos]]
    walls: Set[Pos]
    observed_boxes: Set[int]
    observed_targets: Set[int]
    blast_centers: List[Pos]


def _map_parts(map_data: dict):
    width = int(map_data['width'])
    height = int(map_data['height'])
    walls = set(map_data['walls'])
    boxes = list(map_data.get('boxes_start', map_data.get('boxes', [])))
    bombs = list(map_data.get('bombs', []))
    goals = sorted(map_data['goals'], key=lambda p: (p[1], p[0]))
    car = tuple(map_data.get('car_start', map_data.get('car')))
    return width, height, walls, boxes, bombs, goals, car


def replay_p1_raw(map_data: dict, p1_res) -> P1Replay:
    """Replay P1 without calling any solver transition helper."""
    import c_solver.track_port as tp

    width, height, walls, boxes0, bombs0, goals, car = _map_parts(map_data)
    original_walls = set(walls)
    boxes: List[Optional[Pos]] = list(boxes0)
    bombs: List[Optional[Pos]] = list(bombs0)
    selected = [
        (int(tp.g_best_bombs[i][1]), int(tp.g_best_bombs[i][0]))
        for i in range(len(bombs0))
    ]
    require(len(set(selected)) == len(selected),
            f'P1 duplicate blast centers: {selected}')
    for center in selected:
        require(center in original_walls,
                f'P1 blast center is not an original wall: {center}')
        require(0 < center[0] < width - 1 and 0 < center[1] < height - 1,
                f'P1 blast center is on the immutable boundary: {center}')

    observed_boxes: Set[int] = set()
    observed_targets: Set[int] = set()
    used_centers: List[Pos] = []
    pending_blast: Optional[Tuple[int, Pos]] = None

    for step in range(p1_res.path_length):
        action = int(p1_res.path[step])
        if action == 4:
            require(pending_blast is not None,
                    f'P1 step {step}: BOOM without a bomb entering a wall')
            pending_blast = None
            continue

        require(pending_blast is None,
                f'P1 step {step}: missing BOOM immediately after wall detonation')

        if 0 <= action <= 3:
            dv = DIRS[action]
            nxt = add(car, dv)
            require(in_bounds(nxt, width, height),
                    f'P1 step {step}: car leaves map at {nxt}')

            box_idx = next((i for i, pos in enumerate(boxes) if pos == nxt), None)
            bomb_idx = next((i for i, pos in enumerate(bombs) if pos == nxt), None)
            require(not (box_idx is not None and bomb_idx is not None),
                    f'P1 step {step}: overlapping box and bomb at {nxt}')

            if box_idx is not None:
                target = add(nxt, dv)
                require(in_bounds(target, width, height),
                        f'P1 step {step}: box {box_idx} leaves map')
                occupied = {p for p in boxes if p is not None} | {
                    p for p in bombs if p is not None
                }
                occupied.discard(nxt)
                require(target not in walls and target not in occupied,
                        f'P1 step {step}: illegal box push {nxt}->{target}')
                boxes[box_idx] = target
            elif bomb_idx is not None:
                target = add(nxt, dv)
                require(in_bounds(target, width, height),
                        f'P1 step {step}: bomb {bomb_idx} leaves map')
                occupied = {p for p in boxes if p is not None} | {
                    p for p in bombs if p is not None
                }
                occupied.discard(nxt)
                require(target not in occupied,
                        f'P1 step {step}: bomb collision at {target}')
                if target in walls:
                    require(target in selected,
                            f'P1 step {step}: bomb enters non-selected wall {target}')
                    require(target not in used_centers,
                            f'P1 step {step}: bomb air-detonates at destroyed wall {target}')
                    bombs[bomb_idx] = None
                    used_centers.append(target)
                    explode_walls(walls, target, width, height)
                    pending_blast = bomb_idx, target
                else:
                    bombs[bomb_idx] = target
            else:
                require(nxt not in walls,
                        f'P1 step {step}: car walks into wall {nxt}')
            car = nxt
            continue

        if 5 <= action <= 8:
            direction = action - 5
            looked = add(car, DIRS[direction])
            found = False
            for i, pos in enumerate(boxes):
                if pos == looked:
                    observed_boxes.add(i)
                    found = True
            for i, pos in enumerate(goals):
                if pos == looked:
                    observed_targets.add(i)
                    found = True
            require(found,
                    f'P1 step {step}: observe {DIR_NAMES[direction]} sees no box/goal')
            continue

        raise ValidationError(f'P1 step {step}: unknown raw action {action}')

    require(pending_blast is None, 'P1 ends before BOOM acknowledgement')
    require(all(pos is None for pos in bombs),
            f'P1 leaves unexploded bombs: {bombs}')
    require(set(used_centers) == set(selected),
            f'P1 used blast centers {used_centers}, selected {selected}')

    need_boxes = observation_quota(len(boxes0))
    need_targets = observation_quota(len(goals))
    require(len(observed_boxes) == need_boxes,
            f'P1 observed {len(observed_boxes)} boxes, N-2 requires {need_boxes}')
    require(len(observed_targets) == need_targets,
            f'P1 observed {len(observed_targets)} targets, N-2 requires {need_targets}')
    require(observed_boxes == set(p1_res.observed_box_indices),
            f'P1 raw/declared box observations differ: '
            f'{sorted(observed_boxes)} vs {sorted(p1_res.observed_box_indices)}')
    require(observed_targets == set(p1_res.observed_target_indices),
            f'P1 raw/declared target observations differ: '
            f'{sorted(observed_targets)} vs {sorted(p1_res.observed_target_indices)}')

    final_boxes = [None if value == 255 else (value & 15, value >> 4)
                   for value in tp.g_final_p1_state.bx[:len(boxes0)]]
    require(car == (tp.g_final_p1_state.c, tp.g_final_p1_state.r),
            f'P1 final car mismatch: replay={car}, solver='
            f'{(tp.g_final_p1_state.c, tp.g_final_p1_state.r)}')
    require(boxes == final_boxes,
            f'P1 final boxes mismatch: replay={boxes}, solver={final_boxes}')
    require((p1_res.end_px, 11 - p1_res.end_py) == car,
            f'P1 result endpoint mismatch: {(p1_res.end_px, 11-p1_res.end_py)} != {car}')

    return P1Replay(car, boxes, walls, observed_boxes,
                    observed_targets, used_centers)


def _cell_for_point(x: float, y: float) -> Pos:
    return math.floor(x + 0.5 + 1e-9), math.floor(y + 0.5 + 1e-9)


def validate_route(car: Pos, action: dict, blocked: Set[Pos],
                   width: int, height: int, label: str) -> Pos:
    raw_wps = action.get('waypoints') or []
    target_raw = action.get('target')
    require(target_raw is not None, f'{label}: free_move has no target')
    target = tuple(target_raw)
    points: List[Tuple[float, float]] = [(float(car[0]), float(car[1]))]
    points.extend((float(p[0]), float(p[1])) for p in raw_wps)
    if not raw_wps or tuple(raw_wps[-1]) != target:
        points.append((float(target[0]), float(target[1])))

    for seg_idx, (start, end) in enumerate(zip(points, points[1:])):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = max(1, int(math.ceil(distance * 40.0)))
        for sample in range(1, samples + 1):
            ratio = sample / samples
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            cell = _cell_for_point(x, y)
            require(in_bounds(cell, width, height),
                    f'{label}: segment {seg_idx} leaves map near {(x, y)}')
            require(cell not in blocked,
                    f'{label}: segment {seg_idx} crosses obstacle {cell}')
    return target


def _direction_from_name(name: str) -> Tuple[int, Pos]:
    require(name in DIR_NAMES, f'unknown direction {name!r}')
    idx = DIR_NAMES.index(name)
    return idx, DIRS[idx]


def _validate_push(car: Pos, action: dict, positions: List[Optional[Pos]],
                   other_positions: Iterable[Optional[Pos]], walls: Set[Pos],
                   width: int, height: int, object_kind: str) -> Tuple[Pos, int, Pos]:
    meta = action['push_meta']
    key = 'box_id' if object_kind == 'box' else 'bomb_id'
    start_key = f'{object_kind}_start'
    target_key = f'{object_kind}_target'
    obj_id = int(meta[key])
    start = tuple(meta[start_key])
    target = tuple(meta[target_key])
    _, dv = _direction_from_name(meta['push_dir'])

    require(0 <= obj_id < len(positions),
            f'{object_kind} push has invalid id {obj_id}')
    require(positions[obj_id] == start,
            f'{object_kind} {obj_id} start mismatch: {positions[obj_id]} != {start}')
    delta = sub(target, start)
    steps = delta[0] * dv[0] + delta[1] * dv[1]
    require(steps > 0 and (delta[0], delta[1]) == (dv[0] * steps, dv[1] * steps),
            f'{object_kind} {obj_id} non-linear push {start}->{target}')
    require(car == sub(start, dv),
            f'{object_kind} {obj_id} car is not behind object: car={car}, start={start}')

    occupied = {p for i, p in enumerate(positions)
                if p is not None and i != obj_id}
    occupied.update(p for p in other_positions if p is not None)
    current = start
    car_after = car
    for _ in range(steps):
        nxt = add(current, dv)
        require(in_bounds(nxt, width, height),
                f'{object_kind} {obj_id} leaves map at {nxt}')
        require(nxt not in occupied,
                f'{object_kind} {obj_id} collides at {nxt}')
        if object_kind == 'box':
            require(nxt not in walls,
                    f'box {obj_id} pushed into wall {nxt}')
        elif nxt in walls and nxt != target:
            raise ValidationError(f'bomb {obj_id} crosses wall before target {nxt}')
        car_after = current
        current = nxt

    require(tuple(action['target']) == car_after,
            f'{object_kind} {obj_id} action car target mismatch: '
            f'{tuple(action["target"])} != {car_after}')
    return car_after, obj_id, target


def replay_p1_structured(map_data: dict, p1_res, raw_result: P1Replay) -> None:
    width, height, walls, boxes0, bombs0, goals, car = _map_parts(map_data)
    boxes: List[Optional[Pos]] = list(boxes0)
    bombs: List[Optional[Pos]] = list(bombs0)
    observed_boxes: Set[int] = set()
    observed_targets: Set[int] = set()
    pending: Optional[Tuple[int, Pos]] = None

    for index, action in enumerate(p1_res.structured_actions):
        kind = action['type']
        label = f'P1 structured action {index} ({kind})'
        if pending is not None and kind != 'wait':
            raise ValidationError(f'{label}: explosion wait is missing')

        if kind == 'free_move':
            blocked = set(walls)
            blocked.update(p for p in boxes if p is not None)
            blocked.update(p for p in bombs if p is not None)
            blocked.discard(car)
            car = validate_route(car, action, blocked, width, height, label)
        elif kind == 'push_box':
            car, obj_id, target = _validate_push(
                car, action, boxes, bombs, walls, width, height, 'box')
            boxes[obj_id] = target
        elif kind == 'push_bomb':
            meta = action['push_meta']
            raw_bomb_id = int(meta['bomb_id']) - len(boxes)
            normalized = dict(action)
            normalized['push_meta'] = dict(meta)
            normalized['push_meta']['bomb_id'] = raw_bomb_id
            car, obj_id, target = _validate_push(
                car, normalized, bombs, boxes, walls, width, height, 'bomb')
            if target in walls:
                require(target in raw_result.blast_centers,
                        f'{label}: bomb target is not a selected wall {target}')
                pending = obj_id, target
            else:
                bombs[obj_id] = target
        elif kind == 'wait':
            require(action.get('reason') == 'bomb_explosion' and pending is not None,
                    f'{label}: wait is not tied to a wall detonation')
            obj_id, center = pending
            require(bombs[obj_id] is not None,
                    f'{label}: bomb {obj_id} already disappeared')
            bombs[obj_id] = None
            explode_walls(walls, center, width, height)
            pending = None
        elif kind == 'observe':
            require(tuple(action['target']) == car,
                    f'{label}: observation target does not equal car position')
            meta = action['observe_meta']
            direction, dv = _direction_from_name(meta['direction'])
            require(angle_error(float(action['theta']), DIR_THETA[direction]) < 1e-6,
                    f'{label}: theta does not face observed cell')
            looked = add(car, dv)
            box_hits = {i for i, pos in enumerate(boxes) if pos == looked}
            target_hits = {i for i, pos in enumerate(goals) if pos == looked}
            require(box_hits or target_hits,
                    f'{label}: no object in declared direction')
            obj_type = meta['object_type']
            obj_index = int(meta['object_index'])
            if obj_type in ('box', 'both'):
                require(obj_index in box_hits,
                        f'{label}: declared box {obj_index} is not at {looked}')
                observed_boxes.update(box_hits)
            if obj_type in ('goal', 'both'):
                observed_targets.update(target_hits)
        elif kind == 'end_of_phase1':
            require(index == len(p1_res.structured_actions) - 1,
                    'end_of_phase1 is not the final P1 action')
        else:
            raise ValidationError(f'{label}: unsupported action')

    require(pending is None, 'P1 structured path ends before explosion wait')
    require(car == raw_result.car, f'P1 structured/raw car mismatch: {car}/{raw_result.car}')
    require(boxes == raw_result.boxes,
            f'P1 structured/raw boxes mismatch: {boxes}/{raw_result.boxes}')
    require(walls == raw_result.walls, 'P1 structured/raw wall state mismatch')
    require(observed_boxes == raw_result.observed_boxes,
            f'P1 structured/raw box observations mismatch')
    require(observed_targets == raw_result.observed_targets,
            f'P1 structured/raw target observations mismatch')


def resolved_assignment(actual_box_ids: Sequence[int],
                        actual_target_ids: Sequence[int],
                        p1: P1Replay) -> Tuple[List[int], List[int], List[int]]:
    known_boxes = [-1] * len(actual_box_ids)
    known_targets = [-1] * len(actual_target_ids)
    for index in p1.observed_boxes:
        known_boxes[index] = actual_box_ids[index]
    for index in p1.observed_targets:
        known_targets[index] = actual_target_ids[index]
    box_ids, target_ids = _resolve_ids(known_boxes, known_targets)

    assignment: List[int] = []
    for box_index, value in enumerate(box_ids):
        matches = [i for i, target_value in enumerate(target_ids)
                   if target_value == value]
        require(len(matches) == 1,
                f'box {box_index} resolves to {len(matches)} targets')
        assignment.append(matches[0])

    for box_index, value in enumerate(actual_box_ids):
        expected = actual_target_ids.index(value)
        require(assignment[box_index] == expected,
                f'N-2 inferred box {box_index}->goal {assignment[box_index]}, '
                f'actual goal is {expected}')
    return known_boxes, known_targets, assignment


def replay_p2_structured(map_data: dict, p1: P1Replay, actions: Sequence[dict],
                         assignment: Sequence[int]) -> Pos:
    width, height, _, _, _, goals, _ = _map_parts(map_data)
    walls = set(p1.walls)
    boxes = list(p1.boxes)
    active = {i for i, pos in enumerate(boxes) if pos is not None}
    car = p1.car

    for index, action in enumerate(actions):
        kind = action['type']
        label = f'P2 action {index} ({kind})'
        if kind == 'free_move':
            blocked = set(walls)
            blocked.update(boxes[i] for i in active)
            blocked.discard(car)
            car = validate_route(car, action, blocked, width, height, label)
            continue
        if kind != 'push_box':
            raise ValidationError(f'{label}: unsupported P2 action')

        box_id = int(action['push_meta']['box_id'])
        require(box_id in active,
                f'{label}: pushes completed/nonexistent box {box_id}')
        other_boxes = [boxes[i] for i in active if i != box_id]
        car, _, target = _validate_push(
            car, action, boxes, other_boxes, walls, width, height, 'box')
        boxes[box_id] = target
        if target == goals[assignment[box_id]]:
            active.remove(box_id)
            boxes[box_id] = None

    require(not active, f'P2 ends with unfinished boxes: {sorted(active)}')
    require(car in BASES, f'P2 does not return to a base: final car={car}')
    return car


def _path_follower_observe_test() -> None:
    from core.path_follower import PathFollower, FollowerState

    follower = PathFollower()
    follower.load_actions([{
        'type': 'observe', 'target': (0, 0), 'theta': 90.0,
        'observe_meta': {'direction': 'DOWN', 'object_type': 'box',
                         'object_index': 0},
    }])
    cmd = follower.update(0.0, 0.0, 270.0, {}, 1.0)
    require(cmd.omega != 0.0 and follower.state == FollowerState.OBSERVE,
            'PathFollower completed observation while facing away')
    for _ in range(4):
        follower.update(0.0, 0.0, 90.0, {}, 0.1)
    require(follower.is_done(),
            'PathFollower did not finish after heading aligned and dwell elapsed')


def run_map(map_name: str, permutations: int = 0,
            solver_output: bool = False) -> dict:
    map_path = os.path.join('maps_import', map_name)
    map_data = load_map_from_file(map_path)
    _, _, _, boxes, _, goals, _ = _map_parts(map_data)
    started = time.perf_counter()

    stream = None if solver_output else io.StringIO()
    with contextlib.nullcontext() if stream is None else contextlib.redirect_stdout(stream):
        p1_result = solve_with_c_solver(map_data, verbose=False)
    require(p1_result['success'], f'{map_name}: P1 failed: {p1_result.get("error")}')
    p1_res = p1_result['p1_res']
    p1 = replay_p1_raw(map_data, p1_res)
    replay_p1_structured(map_data, p1_res, p1)

    all_perms = list(itertools.permutations(range(len(goals))))
    if permutations > 0:
        all_perms = all_perms[:permutations]
    p2_times = []
    for permutation in all_perms:
        actual_box_ids = [10 + i for i in range(len(boxes))]
        actual_target_ids = [0] * len(goals)
        for box_index, goal_index in enumerate(permutation):
            actual_target_ids[goal_index] = actual_box_ids[box_index]
        known_boxes, known_targets, assignment = resolved_assignment(
            actual_box_ids, actual_target_ids, p1)
        p2_started = time.perf_counter()
        with contextlib.nullcontext() if stream is None else contextlib.redirect_stdout(stream):
            p2 = solve_phase2_dijkstra(
                p1_res, known_boxes, known_targets, map_data, verbose=False)
        require(p2['success'],
                f'{map_name} assignment {permutation}: P2 failed: {p2.get("error")}')
        try:
            replay_p2_structured(map_data, p1, p2['actions'], assignment)
        except ValidationError as exc:
            raise ValidationError(
                f'{map_name} assignment {permutation}: {exc}') from exc
        p2_times.append(time.perf_counter() - p2_started)

    return {
        'map': map_name,
        'status': 'PASS',
        'p1_seconds': round(p1_result['timing']['total'] / 1000.0, 3),
        'p1_raw_steps': p1_res.path_length,
        'p1_structured_actions': len(p1_res.structured_actions),
        'observed_boxes': sorted(p1.observed_boxes),
        'observed_targets': sorted(p1.observed_targets),
        'assignments_tested': len(all_perms),
        'max_p2_seconds': round(max(p2_times, default=0.0), 3),
        'total_seconds': round(time.perf_counter() - started, 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', dest='maps', action='append',
                        help='map filename; may be repeated')
    parser.add_argument('--include-map7', action='store_true')
    parser.add_argument('--permutations', type=int, default=0,
                        help='0 tests every box/target ID permutation')
    parser.add_argument('--solver-output', action='store_true')
    parser.add_argument('--report', default='strict_regression_report.json')
    args = parser.parse_args()

    _path_follower_observe_test()
    selected = args.maps or list(MAPS)
    if not args.include_map7:
        selected = [name for name in selected if name != 'map7.txt']

    report = {'started_at': time.strftime('%Y-%m-%d %H:%M:%S'), 'results': []}
    failed = False
    for map_name in selected:
        print(f'START {map_name}', flush=True)
        try:
            result = run_map(map_name, args.permutations, args.solver_output)
            report['results'].append(result)
            print(f"PASS  {map_name} p1={result['p1_seconds']}s "
                  f"assignments={result['assignments_tested']} "
                  f"total={result['total_seconds']}s", flush=True)
        except Exception as exc:
            failed = True
            result = {'map': map_name, 'status': 'FAIL',
                      'error': f'{type(exc).__name__}: {exc}'}
            report['results'].append(result)
            print(f'FAIL  {map_name}: {result["error"]}', flush=True)

    report['finished_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
    report['passed'] = sum(r['status'] == 'PASS' for r in report['results'])
    report['failed'] = sum(r['status'] == 'FAIL' for r in report['results'])
    with open(args.report, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    print(f"SUMMARY pass={report['passed']} fail={report['failed']} "
          f"report={args.report}", flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
