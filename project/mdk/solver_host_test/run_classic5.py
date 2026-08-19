from __future__ import annotations

import pathlib
import sys

from run_regression import run_one, validate


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: run_classic5.py SOLVER_EXE MAP_DIR", file=sys.stderr)
        return 2

    exe = pathlib.Path(sys.argv[1]).resolve()
    map_dir = pathlib.Path(sys.argv[2]).resolve()
    paths = sorted(map_dir.glob("*.txt"))
    for path in paths:
        output = run_one(exe, path)
        free_steps, push_steps = validate(path, output)
        first = output.splitlines()[0]
        print(
            f"PASS {path.name:18s} free={free_steps:3d} "
            f"push={push_steps:2d} {first}"
        )
    print(f"SUMMARY pass={len(paths)}/{len(paths)} classic_capacity=5")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
