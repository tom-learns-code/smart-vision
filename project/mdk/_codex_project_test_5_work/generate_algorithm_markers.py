"""
生成算法可视化标记

运行 8 步求解管线，提取箱-目标配对和炸点信息，
输出到 algorithm_markers.json 供可视化渲染器使用。
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.map_analysis import load_map_from_file, scan_map
from run_solver import solve
from config_shared import MAP_FILE_PATH


def generate_markers(map_file_path=None, verbose=True):
    """
    使用新管线生成算法可视化标记。

    Args:
        map_file_path: 地图文件路径，默认使用共享配置
        verbose: 是否打印详细信息

    Returns:
        标记数据字典
    """
    total_start_time = time.time()

    if map_file_path is None:
        map_file_path = MAP_FILE_PATH

    if verbose:
        print("=" * 80)
        print("生成算法可视化标记（新管线）")
        print("=" * 80)

    # 加载地图
    if not os.path.exists(map_file_path):
        print(f"[错误] 地图文件不存在: {map_file_path}")
        return None

    md = load_map_from_file(map_file_path)
    info = scan_map(md)

    if verbose:
        c = info['counts']
        print(f"地图: {info['width']}×{info['height']}, "
              f"墙{c['walls']} 箱{c['boxes']} 目标{c['goals']} 炸弹{c['bombs']}")

    # 运行 8 步管线
    if verbose:
        print("\n[求解] 调用 8 步求解管线...")

    result = solve(md, verbose=verbose)

    if not result['success']:
        print(f"[失败] 求解失败: {result.get('error', '未知错误')}")
        return {'box_goal_pairs': [], 'blast_points': [], 'bomb_assignments': []}

    pairs = result.get('pairs', [])

    # 1. 构建箱子-目标点对信息
    box_goal_pairs_info = []
    for idx, p in enumerate(pairs):
        reachable = p.get('path_steps', 0) > 0 or p.get('same_region', False)
        box_goal_pairs_info.append({
            'box': list(p['box']),
            'goal': list(p['goal']),
            'reachable': reachable,
            'label': f'P{idx + 1}'
        })
        if verbose:
            status = "可达" if reachable else "需炸弹"
            print(f"  P{idx + 1}: 箱子{p['box']} -> 目标{p['goal']}, {status}")

    # 2. 构建炸点信息
    blast_points = []
    blast_label_counter = 1

    for idx, p in enumerate(pairs):
        bp = p.get('bomb_plan')
        if not bp:
            continue
        wall = bp.get('wall')
        bomb = bp.get('bomb')
        if wall is None:
            continue
        # 连锁救援可能有多个墙
        chain = bp.get('chain')
        if chain:
            for cbomb, cwall in chain:
                blast_points.append({
                    'bomb': list(cbomb),
                    'wall': list(cwall),
                    'for_pair': idx,
                    'label': f'B{blast_label_counter}',
                    'score': bp.get('chain_depth', 1)
                })
                blast_label_counter += 1
        else:
            blast_points.append({
                'bomb': list(bomb) if bomb else None,
                'wall': list(wall),
                'for_pair': idx,
                'label': f'B{blast_label_counter}',
                'score': bp.get('bomb_steps', 0) + bp.get('box_steps', 0)
            })
            blast_label_counter += 1

    # 3. 炸弹分配箭头
    bomb_assignments = []
    bomb_to_pair_and_walls = {}  # bomb -> (pair_index, [walls])

    for idx, p in enumerate(pairs):
        bp = p.get('bomb_plan')
        if not bp:
            continue
        bomb = bp.get('bomb')
        wall = bp.get('wall')
        chain = bp.get('chain')
        if chain:
            for cbomb, cwall in chain:
                if cbomb not in bomb_to_pair_and_walls:
                    bomb_to_pair_and_walls[cbomb] = (idx, [])
                bomb_to_pair_and_walls[cbomb][1].append(cwall)
        elif bomb and wall:
            if bomb not in bomb_to_pair_and_walls:
                bomb_to_pair_and_walls[bomb] = (idx, [])
            bomb_to_pair_and_walls[bomb][1].append(wall)

    for bomb, (pair_idx, walls) in bomb_to_pair_and_walls.items():
        bomb_assignments.append({
            'bomb': list(bomb),
            'pair_index': pair_idx,
            'walls': [list(w) for w in walls]
        })

    total_time = time.time() - total_start_time
    if verbose:
        print("\n" + "=" * 80)
        print("生成完成")
        print("=" * 80)
        print(f"箱子-目标点配对: {len(box_goal_pairs_info)} 个")
        print(f"炸点数量: {len(blast_points)} 个")
        print(f"炸弹分配箭头: {len(bomb_assignments)} 个")
        print(f"总计用时: {total_time:.3f}秒")
        print("=" * 80)

    return {
        'box_goal_pairs': box_goal_pairs_info,
        'blast_points': blast_points,
        'bomb_assignments': bomb_assignments
    }


def main():
    """主函数"""
    # 确保使用脚本所在目录作为工作目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # 生成标记
    markers = generate_markers(verbose=True)
    if markers is None:
        return

    # 输出到文件
    output_file = 'algorithm_markers.json'
    output_path = os.path.join(script_dir, output_file)

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(markers, f, indent=2, ensure_ascii=False)

    # 短暂延迟，确保文件写入完成
    time.sleep(0.3)

    # 自动启动主程序
    print("\n[启动] 正在启动主程序...")
    print("[提示] 按 V 键切换可视化显示")
    main_script = os.path.join(script_dir, 'main.py')
    os.environ['ENABLE_VISUALIZATION'] = '1'
    os.system(f'python "{main_script}"')


if __name__ == '__main__':
    main()
