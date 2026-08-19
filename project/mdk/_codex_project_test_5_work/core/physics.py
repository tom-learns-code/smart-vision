"""
物理引擎核心模块
包含碰撞检测、物理响应和数学工具
"""
import pygame
import math


class Vector2(pygame.Vector2):
    """扩展的向量类，支持更多数学运算"""
    pass


def project_poly(vertices, axis):
    """多边形在轴上的投影"""
    dots = [v.dot(axis) for v in vertices]
    return min(dots), max(dots)


def polygons_overlap(proj1, proj2):
    """检测投影重叠"""
    return proj1[0] <= proj2[1] and proj2[0] <= proj1[1]


def get_axes(vertices):
    """获取多边形的所有分离轴"""
    axes = []
    n = len(vertices)
    for i in range(n):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % n]
        edge = p2 - p1
        normal = Vector2(-edge.y, edge.x).normalize()
        axes.append(normal)
    return axes


def sat_collision(poly1_vertices, poly2_vertices, center1=None, center2=None):
    """
    分离轴定理实现旋转矩形碰撞检测
    返回: (是否碰撞, 法线向量, 穿透深度)

    参数:
        poly1_vertices: 第一个多边形的顶点列表
        poly2_vertices: 第二个多边形的顶点列表
        center1: 第一个多边形的物理中心(可选)
        center2: 第二个多边形的物理中心(可选)
    """
    # 获取所有分离轴
    axes1 = get_axes(poly1_vertices)
    axes2 = get_axes(poly2_vertices)
    axes = axes1 + axes2

    min_overlap = float('inf')
    collision_normal = None

    for axis in axes:
        proj1 = project_poly(poly1_vertices, axis)
        proj2 = project_poly(poly2_vertices, axis)

        if not polygons_overlap(proj1, proj2):
            return False, None, 0

        overlap = min(proj1[1], proj2[1]) - max(proj1[0], proj2[0])
        if overlap < min_overlap:
            min_overlap = overlap
            collision_normal = axis

    # 确保法线指向poly1
    # 如果没有提供物理中心,则计算顶点的几何中心作为后备
    if center1 is None:
        center1 = sum(poly1_vertices, Vector2(0, 0)) / len(poly1_vertices)
    if center2 is None:
        center2 = sum(poly2_vertices, Vector2(0, 0)) / len(poly2_vertices)

    direction = center1 - center2
    if collision_normal.dot(direction) < 0:
        collision_normal = -collision_normal

    return True, collision_normal, min_overlap


def continuous_collision(obj, prev_pos, target, steps=20):
    """
    连续碰撞检测（优化版 - 支持旋转）
    返回: (是否碰撞, 碰撞时间t(0-1), 法线)
    """
    direction = obj.pos - prev_pos
    if direction.length() == 0:
        return False, 0, None

    # 获取初始和最终角度（如果有旋转）
    prev_angle = getattr(obj, 'prev_angle', getattr(obj, 'angle', 0))
    curr_angle = getattr(obj, 'angle', 0)
    angle_diff = curr_angle - prev_angle

    for i in range(1, steps + 1):
        t = i / steps
        test_pos = prev_pos + direction * t

        # 获取测试顶点（支持旋转和未旋转对象）
        if hasattr(obj, 'get_rotated_vertices'):
            # 插值角度
            test_angle = prev_angle + angle_diff * t
            test_vertices = obj.get_rotated_vertices(test_pos, test_angle)
        else:
            # 对于不旋转的对象，临时创建顶点
            half_size = obj.size / 2
            test_vertices = [
                Vector2(test_pos.x - half_size, test_pos.y - half_size),
                Vector2(test_pos.x + half_size, test_pos.y - half_size),
                Vector2(test_pos.x + half_size, test_pos.y + half_size),
                Vector2(test_pos.x - half_size, test_pos.y + half_size)
            ]

        target_vertices = target.get_vertices()

        # 获取物理中心,确保SAT法线方向判断正确
        center1 = obj.center if hasattr(obj, 'center') else obj.pos
        center2 = target.center if hasattr(target, 'center') else target.pos
        collided, normal, _ = sat_collision(test_vertices, target_vertices, center1, center2)

        if collided:
            # 二分法精确碰撞点
            t_start = (i - 1) / steps
            t_end = t
            for _ in range(5):  # 5次二分迭代
                t_mid = (t_start + t_end) / 2
                test_pos_mid = prev_pos + direction * t_mid
                if hasattr(obj, 'get_rotated_vertices'):
                    test_angle_mid = prev_angle + angle_diff * t_mid
                    test_vertices_mid = obj.get_rotated_vertices(test_pos_mid, test_angle_mid)
                else:
                    half_size = obj.size / 2
                    test_vertices_mid = [
                        Vector2(test_pos_mid.x - half_size, test_pos_mid.y - half_size),
                        Vector2(test_pos_mid.x + half_size, test_pos_mid.y - half_size),
                        Vector2(test_pos_mid.x + half_size, test_pos_mid.y + half_size),
                        Vector2(test_pos_mid.x - half_size, test_pos_mid.y + half_size)
                    ]
                # 获取物理中心,确保SAT法线方向判断正确
                center1 = obj.center if hasattr(obj, 'center') else obj.pos
                center2 = target.center if hasattr(target, 'center') else target.pos
                collided_mid, _, _ = sat_collision(test_vertices_mid, target_vertices, center1, center2)
                if collided_mid:
                    t_end = t_mid
                else:
                    t_start = t_mid
            return True, t_end, normal

    return False, 0, None


def calculate_penetration(player, box, normal):
    """计算穿透深度"""
    player_proj = project_poly(player.get_vertices(), normal)
    box_proj = project_poly(box.get_vertices(), normal)
    overlap1 = player_proj[1] - box_proj[0]
    overlap2 = box_proj[1] - player_proj[0]
    return min(abs(overlap1), abs(overlap2))


def prevent_penetration(player, box, normal, penetration):
    """位置修正防止穿模"""
    correction = normal * penetration * 0.5
    player.pos -= correction
    box.pos += correction


def find_contact_point(player, box, normal):
    """找到碰撞接触点"""
    # 简化的接触点计算：找到最接近的顶点对
    player_verts = player.get_vertices()
    box_verts = box.get_vertices()

    min_dist = float('inf')
    contact_point = Vector2(0, 0)

    for pv in player_verts:
        for bv in box_verts:
            dist = (pv - bv).length()
            if dist < min_dist:
                min_dist = dist
                contact_point = (pv + bv) / 2

    return contact_point


def clamp_velocity(obj, max_speed):
    """速度限制"""
    if obj.velocity.length() > max_speed:
        obj.velocity.scale_to_length(max_speed)
