"""
物理管线 - 碰撞检测和响应的完整流程
"""
import pygame
import math
from .physics import Vector2, sat_collision, continuous_collision, project_poly, polygons_overlap, get_axes


class CollisionPipeline:
    """物理管线：处理所有碰撞检测和响应"""

    def __init__(self, friction=0.5, torque_factor=0.2, wall_torque_factor=0.1):
        self.friction = friction
        self.torque_factor = torque_factor
        self.wall_torque_factor = wall_torque_factor
        self.last_collision_info = None

    def build_collision_tree(self, car, box, walls):
        """构建碰撞检测树（按优先级排序）"""
        collision_pairs = [
            (box, walls),   # 箱子-墙（最高优先级）
            (car, walls),   # 车-墙
            (car, box)      # 车-箱
        ]
        return collision_pairs

    def build_collision_tree_for_boxes(self, car, boxes, walls):
        """构建多个箱子的碰撞检测树"""
        collision_pairs = []
        # 每个箱子的碰撞
        for box in boxes:
            collision_pairs.extend([
                (box, walls),   # 箱子-墙
                (car, box)      # 车-箱
            ])
        # 箱子-箱子碰撞
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                collision_pairs.append((boxes[i], boxes[j]))
        # 车-墙
        collision_pairs.append((car, walls))
        return collision_pairs

    def process_frame(self, car, box, walls, dt, substeps):
        """
        处理一帧的物理更新
        返回: (collision_active, chain_collision)
        """
        collision_active = False
        chain_collision = False

        for _ in range(substeps):
            sub_dt = dt / substeps

            # 1. 预测位移
            car_pred_pos, car_pred_angle = car.predict_movement(sub_dt)
            box_pred_pos = box.predict_movement(sub_dt)

            # 2. 构建碰撞树
            collision_pairs = self.build_collision_tree(car, box, walls)

            # 3. 按优先级处理碰撞
            frame_collided = False

            for obj, target in collision_pairs:
                if isinstance(target, list):
                    # 目标是墙列表
                    for wall in target:
                        collided = self.process_single_collision(
                            obj, wall, sub_dt
                        )
                        if collided:
                            frame_collided = True
                            collision_active = True
                else:
                    # 目标是单个物体
                    collided = self.process_single_collision(
                        obj, target, sub_dt
                    )
                    if collided:
                        frame_collided = True
                        collision_active = True

            # 4. 更新位置
            car.update(sub_dt)
            box.update(sub_dt)

            # 5. 检测链式碰撞
            chain_collision = self.check_chain_collision(car, box, walls)

            # 6. 最终位置对齐
            self.final_position_align(car, box, walls)

        return collision_active, chain_collision

    def process_single_collision(self, obj, target, dt, no_resistance=False):
        """处理单个碰撞对（优化版 - 支持炸弹穿墙）"""
        # 快速预检测：AABB检查
        if not obj.rect.colliderect(target.rect):
            return False

        # 炸弹与墙的碰撞：不可破坏的墙阻挡，可破坏的墙允许穿入
        if obj.type == "bomb" and target.type == "wall":
            # 如果是不可破坏的墙，则进行碰撞检测（阻挡炸弹）
            if hasattr(target, 'is_indestructible') and target.is_indestructible:
                pass  # 继续碰撞检测
            else:
                # 可破坏的墙，允许炸弹穿入
                return False
        if target.type == "bomb" and obj.type == "wall":
            return False

        # 连续碰撞检测（根据速度动态调整步数）
        velocity_mag = obj.velocity.length() if hasattr(obj, 'velocity') else 0
        ccd_steps = max(5, min(15, int(velocity_mag / 30)))  # 速度越快步数越多，但上限15

        collided, collision_t, normal = continuous_collision(
            obj, obj.prev_pos, target, steps=ccd_steps
        )

        if collided:
            # 确保法线指向obj（从target指向obj）
            if target.type == "wall":
                wall_normal = self.calculate_wall_normal(obj, target)
                if wall_normal.dot(normal) < 0:
                    normal = wall_normal

            # 计算碰撞点
            contact_point = self.find_contact_point(obj, target, normal)

            # 根据目标类型处理响应
            if target.type == "wall":
                self.resolve_wall_collision(obj, target, normal, contact_point)
            elif target.type == "box":
                # 区分是车推箱子还是箱子碰箱子
                if obj.type == "car":
                    self.resolve_push_collision(obj, target, normal, contact_point, no_resistance=no_resistance)
                else:  # 箱子碰箱子
                    self.resolve_box_box_collision(obj, target, normal, contact_point)
            elif target.type == "bomb":
                # 处理涉及炸弹的碰撞
                if obj.type == "car":
                    self.resolve_push_collision(obj, target, normal, contact_point, no_resistance=no_resistance)
                elif obj.type == "box":
                    self.resolve_box_box_collision(obj, target, normal, contact_point)
                elif obj.type == "bomb":
                    self.resolve_bomb_bomb_collision(obj, target, normal, contact_point)

            # 穿透修正
            penetration = self.calculate_sat_penetration(obj, target)
            self.penetration_correction(obj, target, normal, penetration)

            return True

        return False

    def resolve_wall_collision(self, obj, wall, normal, contact_point):
        """墙体碰撞：法向速度归零，墙体完全静止，墙为光滑表面"""
        # 1. 法向速度处理（完全清零，消除抖动）
        v_normal_mag = obj.velocity.dot(normal)
        v_normal = normal * v_normal_mag
        obj.velocity -= v_normal

        # 2. 切向速度保留（墙为光滑表面，不施加摩擦力）

        # 3. 角速度处理（车和箱子）- 禁用扭矩，避免墙体碰撞导致旋转
        # 注释：墙体碰撞不产生扭矩，避免角度变化

        # 4. 确保墙体速度永远为零（双重保险）
        wall.velocity = Vector2(0, 0)

        # 记录碰撞信息
        self.last_collision_info = {
            'type': 'wall',
            'obj': obj,
            'wall': wall,
            'normal': normal,
            'contact_point': contact_point,
            'force': abs(v_normal_mag)
        }

    def resolve_push_collision(self, car, box, normal, contact_point, no_resistance=False):
        """车推箱子物理（法线轴对齐）"""
        # 轴对齐法线到水平或垂直方向
        if abs(normal.x) > abs(normal.y):
            aligned_normal = Vector2(1, 0) if normal.x > 0 else Vector2(-1, 0)
        else:
            aligned_normal = Vector2(0, 1) if normal.y > 0 else Vector2(0, -1)

        # 检查倾斜碰撞
        car_dir = car.velocity.normalize() if car.velocity.length() > 0.1 else Vector2(0, 0)
        alignment = abs(aligned_normal.dot(car_dir)) if car_dir.length() > 0 else 1.0

        # 法向动量传递
        total_mass = car.mass + box.mass
        v_normal_mag = car.velocity.dot(aligned_normal)
        new_v_normal = (car.mass * v_normal_mag) / total_mass

        # 4. 切向摩擦力传递（倾斜碰撞时减少摩擦，避免抽搐）
        tangent = Vector2(-aligned_normal.y, aligned_normal.x)
        v_tangent_mag = car.velocity.dot(tangent)

        # 根据对齐度调整摩擦系数：越倾斜摩擦越小
        friction_adjusted = self.friction * (0.2 + 0.8 * alignment)

        # 保存碰撞前的速度，用于平滑过渡
        car_velocity_before = car.velocity.copy()
        box_velocity_before = box.velocity.copy()

        # 计算新速度
        new_box_velocity = aligned_normal * new_v_normal + tangent * v_tangent_mag * friction_adjusted

        # 车速度调整
        if no_resistance:
            car.velocity = car.velocity
            if hasattr(box, 'is_being_pushed'):
                box.is_being_pushed = True
        else:
            # 正常模式：车速度根据碰撞调整
            new_car_velocity = car.velocity - aligned_normal * v_normal_mag * (car.mass / total_mass)
            new_car_velocity -= tangent * v_tangent_mag * friction_adjusted
            # 速度平滑混合（70%新速度，30%旧速度）- 替换式更新
            car.velocity = new_car_velocity * 0.7 + car_velocity_before * 0.3

        # 箱子速度更新
        box.velocity = new_box_velocity

        # 倾斜碰撞时增加阻尼，减少震荡
        if alignment < 0.8:
            car.angular_velocity *= 0.6
            # 速度也轻微阻尼
            car.velocity *= 0.9
            box.velocity *= 0.9

        # 7. 关键修复：如果箱子几乎无法移动（被墙阻挡），则车也不能向前移动
        # 检查箱子是否接近静止（法向速度很小）
        box_v_normal = box.velocity.dot(aligned_normal)
        if abs(box_v_normal) < 0.1 and abs(v_normal_mag) > 0.1:
            # 箱子被阻挡，车的法向速度必须完全清零
            v_normal = aligned_normal * car.velocity.dot(aligned_normal)
            car.velocity -= v_normal

        # 记录碰撞信息
        self.last_collision_info = {
            'type': 'push',
            'car': car,
            'box': box,
            'normal': aligned_normal,
            'contact_point': contact_point,
            'force': car.velocity.length()
        }

    def resolve_box_box_collision(self, box1, box2, normal, contact_point):
        """箱子与箱子的碰撞——禁止连锁推, 两者速度均清零"""
        box1.velocity = Vector2(0, 0)
        box2.velocity = Vector2(0, 0)
        self.last_collision_info = {
            'type': 'box_box',
            'box1': box1, 'box2': box2,
            'normal': normal, 'contact_point': contact_point,
        }

    def resolve_bomb_bomb_collision(self, bomb1, bomb2, normal, contact_point):
        """炸弹与炸弹的碰撞——禁止连锁推, 两者速度均清零"""
        bomb1.velocity = Vector2(0, 0)
        bomb2.velocity = Vector2(0, 0)
        self.last_collision_info = {
            'type': 'bomb_bomb',
            'bomb1': bomb1, 'bomb2': bomb2,
            'normal': normal, 'contact_point': contact_point,
        }

    def penetration_correction(self, obj, target, normal, penetration):
        """SAT穿透深度修正"""
        if penetration <= 0:
            return

        # 精确修正系数：墙用100%，车-箱子碰撞也用100%（因为箱子可能被墙阻挡）
        if target.is_static:
            correction_factor = 1.0
            offset = 0.0
        elif obj.type == "car" and target.type == "box":
            correction_factor = 1.0
            offset = 0.001
        else:
            correction_factor = 0.85
            offset = 0.0

        # 质量比修正
        if target.is_static:
            correction = normal * penetration
            obj.pos += correction
            target.velocity = Vector2(0, 0)
        else:
            ratio = obj.mass / (obj.mass + target.mass)
            obj.pos += normal * (penetration * ratio * correction_factor + offset)
            target.pos -= normal * penetration * (1 - ratio) * correction_factor

    def calculate_sat_penetration(self, obj1, obj2):
        """计算SAT穿透深度（优化版 - 支持旋转物体）"""
        verts1 = obj1.vertices if hasattr(obj1, 'vertices') else obj1.get_vertices()
        verts2 = obj2.vertices if hasattr(obj2, 'vertices') else obj2.get_vertices()

        # 获取所有分离轴
        from .physics import get_axes
        axes1 = get_axes(verts1)
        axes2 = get_axes(verts2)
        axes = axes1 + axes2

        min_overlap = float('inf')

        for axis in axes:
            proj1 = project_poly(verts1, axis)
            proj2 = project_poly(verts2, axis)

            if not polygons_overlap(proj1, proj2):
                return 0  # 不重叠，无穿透

            overlap = min(proj1[1], proj2[1]) - max(proj1[0], proj2[0])
            if overlap < min_overlap:
                min_overlap = overlap

        return max(0, min_overlap)

    def final_position_align(self, car, box, walls):
        """最终位置对齐（三重保障 - 使用SAT检测）"""
        # 车-箱对齐
        car_verts = car.get_vertices() if hasattr(car, 'get_vertices') else car.get_rotated_vertices()
        box_verts = box.get_vertices()
        car_box_collided, normal, penetration = sat_collision(car_verts, box_verts, car.center, box.center)

        if car_box_collided and normal is not None and penetration > 0:
            # 使用SAT返回的精确法线和穿透深度
            car.pos -= normal * penetration * 0.51
            box.pos += normal * penetration * 0.49

        # 物体-墙对齐 - 墙体完全静止
        for obj in [car, box]:
            obj_verts = obj.get_vertices() if hasattr(obj, 'get_vertices') else obj.get_rotated_vertices()
            for wall in walls:
                center1 = obj.center if hasattr(obj, 'center') else obj.pos
                center2 = wall.center if hasattr(wall, 'center') else wall.pos
                collided, normal, penetration = sat_collision(obj_verts, wall.get_vertices(), center1, center2)
                if collided and normal is not None and penetration > 0:
                    # 对于墙体，使用墙法线更准确
                    wall_normal = self.calculate_wall_normal(obj, wall)
                    obj.pos += wall_normal * penetration * 1.01
                    # 确保墙体静止
                    wall.velocity = Vector2(0, 0)

    def final_position_align_boxes(self, car, boxes, walls):
        """最终位置对齐（支持多箱子 - 使用SAT检测）"""
        # 车-每个箱子对齐
        car_verts = car.get_vertices() if hasattr(car, 'get_vertices') else car.get_rotated_vertices()
        for box in boxes:
            box_verts = box.get_vertices()
            collided, normal, penetration = sat_collision(car_verts, box_verts, car.center, box.center)
            if collided and normal is not None and penetration > 0:
                # 使用SAT返回的精确法线和穿透深度
                car.pos -= normal * penetration * 0.51
                box.pos += normal * penetration * 0.49

        # 箱子-箱子对齐
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                box1, box2 = boxes[i], boxes[j]
                collided, normal, penetration = sat_collision(box1.get_vertices(), box2.get_vertices(), box1.center, box2.center)
                if collided and normal is not None and penetration > 0:
                    # 使用SAT返回的精确法线和穿透深度
                    ratio = box1.mass / (box1.mass + box2.mass)
                    box1.pos -= normal * penetration * ratio
                    box2.pos += normal * penetration * (1 - ratio)

        # 物体-墙对齐 - 墙体完全静止
        for obj in [car] + boxes:
            obj_verts = obj.get_vertices() if hasattr(obj, 'get_vertices') else obj.get_rotated_vertices()
            for wall in walls:
                center1 = obj.center if hasattr(obj, 'center') else obj.pos
                center2 = wall.center if hasattr(wall, 'center') else wall.pos
                collided, normal, penetration = sat_collision(obj_verts, wall.get_vertices(), center1, center2)
                if collided and normal is not None and penetration > 0:
                    # 对于墙体，使用墙法线更准确
                    wall_normal = self.calculate_wall_normal(obj, wall)
                    obj.pos += wall_normal * penetration * 1.01
                    # 确保墙体静止
                    wall.velocity = Vector2(0, 0)

    def check_chain_collision(self, car, box, walls):
        """检查链式碰撞（使用SAT精确检测）"""
        # 车-箱碰撞
        car_box_collided, _, _ = sat_collision(
            car.get_vertices() if hasattr(car, 'get_vertices') else car.get_rotated_vertices(),
            box.get_vertices()
        )

        # 箱-墙碰撞
        box_wall_collided = any(
            sat_collision(box.get_vertices(), wall.get_vertices())[0]
            for wall in walls
        )

        return car_box_collided and box_wall_collided

    def check_any_chain_collision(self, car, boxes, walls):
        """检查任意箱子的链式碰撞"""
        for box in boxes:
            if self.check_chain_collision(car, box, walls):
                return True
        return False

    def calculate_wall_normal(self, obj, wall):
        """计算墙体法线"""
        dx = (obj.pos.x - wall.pos.x) / (wall.width / 2 + obj.size / 2)
        dy = (obj.pos.y - wall.pos.y) / (wall.height / 2 + obj.size / 2)

        if abs(dx) > abs(dy):
            return Vector2(1 if dx > 0 else -1, 0)
        else:
            return Vector2(0, 1 if dy > 0 else -1)

    def find_contact_point(self, obj1, obj2, normal):
        """找到碰撞接触点"""
        verts1 = obj1.vertices if hasattr(obj1, 'vertices') else obj1.get_vertices()
        verts2 = obj2.vertices if hasattr(obj2, 'vertices') else obj2.get_vertices()

        min_dist = float('inf')
        contact_point = obj1.center

        for v1 in verts1:
            for v2 in verts2:
                dist = (v1 - v2).length()
                if dist < min_dist:
                    min_dist = dist
                    contact_point = (v1 + v2) / 2

        return contact_point


def project_poly(vertices, axis):
    """多边形在轴上的投影"""
    dots = [v.dot(axis) for v in vertices]
    return min(dots), max(dots)
