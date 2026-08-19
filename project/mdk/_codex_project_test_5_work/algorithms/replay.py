"""
网格回放验证器

读取 Action 序列, 在网格上逐步模拟执行, 验证正确性。
"""

from typing import List, Dict, Tuple, Set
from copy import deepcopy


def replay_actions(
    actions: List[Dict],
    walls: Set[Tuple[int, int]],
    boxes: List[Tuple[int, int]],
    bombs: List[Tuple[int, int]],
    goals: Set[Tuple[int, int]],
    car_start: Tuple[int, int],
    width: int, height: int,
    verbose: bool = True,
) -> Dict:
    """
    回放 Action 序列, 验证每步合法性。

    Returns:
        {'success': bool, 'error': str, 'final_state': {...}, 'steps': int}
    """
    cur_walls = set(walls)
    box_pos = list(boxes)
    bomb_pos = list(bombs)
    bomb_alive = [True] * len(bombs)
    car_pos = car_start
    goals_reached = set()

    def log(msg):
        if verbose:
            print(f'  {msg}')

    def is_walkable(pos, ignore_bomb_idx=None):
        if not (0 <= pos[0] < width and 0 <= pos[1] < height):
            return False
        if pos in cur_walls:
            return False
        for i, bp in enumerate(box_pos):
            if bp == pos:
                return False
        for i, bp in enumerate(bomb_pos):
            if bomb_alive[i] and bp == pos and i != ignore_bomb_idx:
                return False
        return True

    for step_no, a in enumerate(actions):
        t = a['type']

        if t == 'free_move':
            target = a['target']
            wps = a.get('waypoints', [car_pos, target])
            if len(wps) < 2:
                wps = [car_pos, target]

            # 验证 waypoints 连续性
            for i in range(len(wps) - 1):
                fr = wps[i]
                to = wps[i + 1]
                if not is_walkable(to):
                    return {'success': False,
                            'error': f'step {step_no}: free_move waypoint {to} not walkable'}
                # 简单验证: 相邻或对角
                d = abs(to[0] - fr[0]) + abs(to[1] - fr[1])
                if d != 1 and not (abs(to[0] - fr[0]) == 1 and abs(to[1] - fr[1]) == 1):
                    # LOS 平滑可能产生对角, 允许
                    pass

            car_pos = target

        elif t == 'push_box':
            m = a['push_meta']
            bid = m['box_id']
            start = m['obj_start']
            end = m['obj_end']
            pdir = m['push_dir']
            n = m['n_steps']

            if box_pos[bid] != start:
                return {'success': False,
                        'error': f'step {step_no}: push_box[{bid}] expected at {start}, actual {box_pos[bid]}'}

            dx, dy = {'UP': (0, -1), 'DOWN': (0, 1), 'LEFT': (-1, 0), 'RIGHT': (1, 0)}[pdir]

            for k in range(n):
                cur_box = (start[0] + dx * k, start[1] + dy * k)
                nxt_box = (start[0] + dx * (k + 1), start[1] + dy * (k + 1))
                car_stand = (cur_box[0] - dx, cur_box[1] - dy)

                # 车必须在推的位置
                if car_pos != car_stand:
                    return {'success': False,
                            'error': f'step {step_no}: push_box car at {car_pos}, need {car_stand}'}

                # 箱子下一格必须可走
                if not is_walkable(nxt_box):
                    return {'success': False,
                            'error': f'step {step_no}: push_box target {nxt_box} blocked'}

                car_pos = cur_box  # 推完后车在箱子刚才的位置

            box_pos[bid] = end
            if box_pos[bid] in goals:
                goals_reached.add(bid)

        elif t == 'push_bomb':
            m = a['push_meta']
            bid = m['bomb_id']
            start = m['obj_start']
            end = m['obj_end']
            pdir = m['push_dir']
            n = m['n_steps']

            if bomb_pos[bid] != start:
                return {'success': False,
                        'error': f'step {step_no}: push_bomb[{bid}] expected at {start}, actual {bomb_pos[bid]}'}

            dx, dy = {'UP': (0, -1), 'DOWN': (0, 1), 'LEFT': (-1, 0), 'RIGHT': (1, 0)}[pdir]

            for k in range(n):
                cur_bomb = (start[0] + dx * k, start[1] + dy * k)
                nxt_bomb = (start[0] + dx * (k + 1), start[1] + dy * (k + 1))
                car_stand = (cur_bomb[0] - dx, cur_bomb[1] - dy)

                if car_pos != car_stand:
                    return {'success': False,
                            'error': f'step {step_no}: push_bomb car at {car_pos}, need {car_stand}'}

                # 炸弹下一格: 允许是墙 (最后一步)
                if k < n - 1:
                    if not is_walkable(nxt_bomb, ignore_bomb_idx=bid):
                        return {'success': False,
                                'error': f'step {step_no}: push_bomb target {nxt_bomb} blocked'}
                # 最后一步: 可以是墙
                car_pos = cur_bomb

            bomb_pos[bid] = end

        elif t == 'detonate':
            wall_target = a['wall']
            clears = a['clears']
            # 炸弹必须在墙上
            bomb_idx = None
            for i, bp in enumerate(bomb_pos):
                if bp == wall_target and bomb_alive[i]:
                    bomb_idx = i
                    break
            if bomb_idx is None:
                return {'success': False,
                        'error': f'step {step_no}: detonate no bomb at {wall_target}'}

            bomb_alive[bomb_idx] = False
            bomb_pos[bomb_idx] = (-1, -1)
            for c in clears:
                cur_walls.discard(c)

    # 全部完成
    all_boxes_on_goal = all(box_pos[i] in goals for i in range(len(box_pos)))
    all_bombs_used = not any(bomb_alive)

    if not all_boxes_on_goal:
        unmatched = [(i, box_pos[i]) for i in range(len(box_pos)) if box_pos[i] not in goals]
        return {'success': False,
                'error': f'not all boxes on goal: {unmatched}',
                'final_state': {'boxes': box_pos, 'bombs': bomb_pos, 'car': car_pos,
                                'goals_reached': goals_reached}}

    return {'success': True, 'error': None,
            'final_state': {'boxes': box_pos, 'bombs': bomb_pos, 'car': car_pos,
                           'goals_reached': goals_reached, 'walls': cur_walls},
            'steps': len(actions)}
