from __future__ import annotations

import pathlib
import subprocess
import sys

DIRS = ((0, -1), (0, 1), (-1, 0), (1, 0))


def load_map(path: pathlib.Path, override: tuple[int, int] | None = None):
    rows = [line.rstrip("\r\n") for line in path.read_text().splitlines() if line]
    walls: set[tuple[int, int]] = set()
    boxes: list[tuple[int, int]] = []
    goals: list[tuple[int, int]] = []
    car = (0, 0)
    for y, row in enumerate(rows):
        for x, cell in enumerate(row):
            if cell == "#":
                walls.add((x, y))
            elif cell == "$":
                boxes.append((x, y))
            elif cell == ".":
                goals.append((x, y))
            elif cell == "@":
                car = (x, y)
    return rows, walls, boxes, goals, override or car


def run_one(exe: pathlib.Path, path: pathlib.Path,
            override: tuple[int, int] | None = None) -> str:
    args = [str(exe), path.name]
    if override:
        args += [str(override[0]), str(override[1])]
    proc = subprocess.run(
        args, cwd=path.parent, text=True, capture_output=True, check=False
    )
    if proc.returncode != 0:
        raise AssertionError(f"{path.name}: C solver failed\n{proc.stdout}{proc.stderr}")
    return proc.stdout


def validate(path: pathlib.Path, output: str,
             override: tuple[int, int] | None = None) -> tuple[int, int]:
    rows, walls, box_list, goal_list, car = load_map(path, override)
    boxes = {i: pos for i, pos in enumerate(box_list)}
    active_goals = set(goal_list)
    for bid, pos in list(boxes.items()):
        if pos in active_goals:
            del boxes[bid]
            active_goals.remove(pos)

    lines = output.splitlines()
    result = lines[0]
    assert "success=1" in result and "check=OK" in result, (path.name, result)
    free_steps = 0
    push_steps = 0
    for line in lines[2:]:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "FREE":
            target = (int(parts[1]), int(parts[2]))
            count = int(parts[3])
            points = [tuple(map(int, item.split(","))) for item in parts[4:]]
            assert len(points) == count and points[0] == car, (path.name, line, car)
            for point in points[1:]:
                assert abs(point[0] - car[0]) + abs(point[1] - car[1]) == 1, (path.name, line)
                assert point not in walls and point not in boxes.values(), (path.name, line)
                car = point
                free_steps += 1
            assert car == target, (path.name, line, car)
        elif parts[0] == "PUSH":
            bid = int(parts[1])
            start = tuple(map(int, parts[2].split(",")))
            target = tuple(map(int, parts[3].split(",")))
            direction = int(parts[4])
            count = int(parts[5])
            car_target = tuple(map(int, parts[6].split(",")))
            assert boxes.get(bid) == start and car == car_target, (path.name, line, boxes, car)
            dx, dy = DIRS[direction]
            box = start
            for step in range(count):
                nxt = (box[0] + dx, box[1] + dy)
                assert nxt not in walls, (path.name, line, "wall", nxt)
                assert nxt not in [p for other, p in boxes.items() if other != bid], (path.name, line, "box", nxt)
                car = box
                box = nxt
                boxes[bid] = box
                push_steps += 1
                if box in active_goals:
                    assert step + 1 == count, (path.name, line, "early goal")
                    active_goals.remove(box)
                    del boxes[bid]
            assert box == target, (path.name, line, box)
        else:
            raise AssertionError((path.name, line))
    assert not boxes and not active_goals, (path.name, boxes, active_goals)
    return free_steps, push_steps


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: run_regression.py SOLVER_EXE MAP_DIR", file=sys.stderr)
        return 2
    exe = pathlib.Path(sys.argv[1]).resolve()
    map_dir = pathlib.Path(sys.argv[2]).resolve()
    paths = sorted(map_dir.glob("*.txt"))
    cases = [(path, None) for path in paths]
    cases.append((map_dir / "map2.txt", (2, 7)))
    totals = []
    for path, override in cases:
        output = run_one(exe, path, override)
        free_steps, push_steps = validate(path, output, override)
        label = path.name if override is None else f"{path.name}@2,7"
        result = output.splitlines()[0]
        fallback = "fallback=1" in result
        totals.append((label, free_steps, push_steps, fallback))
        print(f"PASS {label:14s} free={free_steps:3d} push={push_steps:2d} fallback={int(fallback)}")
    print(f"SUMMARY pass={len(totals)}/{len(cases)} grid4=1 semantic_replay=1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
