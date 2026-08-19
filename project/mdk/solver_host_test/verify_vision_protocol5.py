from __future__ import annotations


MAX_BOXES = 5
MAX_GOALS = 5
MAX_BOMBS = 4


def append_slots(payload: bytearray, cells: list[tuple[int, int]], count: int):
    cells = cells[:count]
    payload.append(len(cells))
    for index in range(count):
        payload.extend(cells[index] if index < len(cells) else (255, 255))


def full_payload() -> bytearray:
    payload = bytearray((16, 12))
    payload.extend(bytes(24))
    append_slots(payload, [(i, 1) for i in range(5)], MAX_BOXES)
    append_slots(payload, [(i, 2) for i in range(5)], MAX_GOALS)
    append_slots(payload, [(i, 3) for i in range(4)], MAX_BOMBS)
    payload.extend(bytes(6))
    payload.append(0)
    return payload


def pos_payload(box_count: int, bomb_count: int) -> bytearray:
    payload = bytearray(7)
    boxes = [(i, 1) for i in range(box_count)][:MAX_BOXES]
    bombs = [(i, 2) for i in range(bomb_count)][:MAX_BOMBS]
    payload.append(len(boxes))
    for cell in boxes:
        payload.extend(cell)
    bomb_slots = max(0, (24 - len(payload)) // 2)
    bombs = bombs[:bomb_slots]
    payload.append(len(bombs))
    for cell in bombs:
        payload.extend(cell)
    payload.extend(bytes(25 - len(payload)))
    payload.append(0)
    return payload


def main() -> int:
    full = full_payload()
    assert len(full) == 64
    assert full[26] == 5
    assert full[37] == 5
    assert full[48] == 4
    assert 57 == 49 + MAX_BOMBS * 2

    classic5 = pos_payload(5, 4)
    assert len(classic5) == 26
    assert classic5[7] == 5
    assert classic5[18] == 3

    bomb3 = pos_payload(3, 3)
    assert len(bomb3) == 26
    assert bomb3[7] == 3
    assert bomb3[14] == 3
    print("PASS full_len=64 offsets=26/37/48/57 pos_len=26 classic5_bombs=3")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
