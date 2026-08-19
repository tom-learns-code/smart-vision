"""
游戏逻辑模块 - 处理游戏特定逻辑（爆炸、目标判定等）
"""
from .physics import Vector2
from .config import EDITOR_GRID_SIZE, BOMB_EXPLOSION_DELAY, TARGET_DISTANCE_THRESHOLD, SCREEN_WIDTH, SCREEN_HEIGHT, OBJECT_SIZE


class BombExplosion:
    """炸弹爆炸逻辑"""
    
    @staticmethod
    def explode(bomb, walls, grid_size=EDITOR_GRID_SIZE):
        """
        炸弹爆炸 - 找到离炸弹中心最近的墙体，炸毁以该墙体为中心的3x3区域内的墙体
        
        Args:
            bomb: 炸弹对象
            walls: 墙体列表
            grid_size: 网格大小
            
        Returns:
            炸毁的墙体数量
        """
        if bomb.is_exploded:
            return 0
        
        bomb.is_exploded = True
        
        # 找到离炸弹中心最近的墙体（跳过不可破坏的墙）
        closest_wall = None
        closest_distance = float('inf')

        for wall in walls:
            if wall.type == "wall" and not wall.is_indestructible:
                distance = (bomb.pos - wall.pos).length()
                if distance < closest_distance:
                    closest_distance = distance
                    closest_wall = wall

        if closest_wall is None:
            return 0
        
        # 计算以最近墙体为中心的3x3区域的网格坐标
        center_grid_x = int(closest_wall.pos.x / grid_size)
        center_grid_y = int(closest_wall.pos.y / grid_size)
        
        # 3x3区域：左上、上、右上、左、中、右、左下、下、右下
        walls_to_remove = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                target_grid_x = center_grid_x + dx
                target_grid_y = center_grid_y + dy

                for wall in walls:
                    # 跳过不可破坏的墙
                    if wall.is_indestructible:
                        continue
                    wall_grid_x = int(wall.pos.x / grid_size)
                    wall_grid_y = int(wall.pos.y / grid_size)

                    if wall_grid_x == target_grid_x and wall_grid_y == target_grid_y:
                        walls_to_remove.append(wall)
                        break
        
        # 移除墙体
        for wall in walls_to_remove:
            if wall in walls:
                walls.remove(wall)
        
        return len(walls_to_remove)


class BombDetection:
    """炸弹检测逻辑"""
    
    @staticmethod
    def check_in_wall(bomb, walls):
        """
        检查炸弹是否完全进入墙体内
        
        Args:
            bomb: 炸弹对象
            walls: 墙体列表
            
        Returns:
            是否在墙体内
        """
        # 获取炸弹的四个角点
        bomb_vertices = [
            Vector2(bomb.pos.x - bomb.size / 2, bomb.pos.y - bomb.size / 2),
            Vector2(bomb.pos.x + bomb.size / 2, bomb.pos.y - bomb.size / 2),
            Vector2(bomb.pos.x + bomb.size / 2, bomb.pos.y + bomb.size / 2),
            Vector2(bomb.pos.x - bomb.size / 2, bomb.pos.y + bomb.size / 2),
        ]
        
        # 检查炸弹的所有角点是否都在某个墙体内
        covered_count = 0
        for vertex in bomb_vertices:
            for wall in walls:
                if wall.type == "wall":
                    dx = abs(vertex.x - wall.pos.x)
                    dy = abs(vertex.y - wall.pos.y)
                    
                    if dx <= wall.width / 2 and dy <= wall.height / 2:
                        covered_count += 1
                        break
        
        return covered_count == 4


class TargetDetection:
    """目标点检测逻辑"""
    
    @staticmethod
    def check_box_target_collision(boxes, targets):
        """
        检查箱子是否到达目标点
        
        Args:
            boxes: 箱子列表
            targets: 目标点列表
            
        Returns:
            (是否触发, 触发的箱子, 触发的目标点)
        """
        for box in boxes:
            for target in targets:
                if not target.is_active:
                    continue
                
                # 箱子中心与目标点中心距离判定
                distance = (box.pos - target.pos).length()
                threshold = box.size * TARGET_DISTANCE_THRESHOLD
                
                if distance <= threshold:
                    target.is_active = False
                    boxes.remove(box)
                    return True, box, target
        
        return False, None, None


def get_world_state(car, boxes, walls, bombs, targets):
    """
    获取世界状态供算法使用

    Args:
        car: 车辆对象
        boxes: 箱子列表
        walls: 墙体列表
        bombs: 炸弹列表
        targets: 目标点列表

    Returns:
        dict: 世界状态字典
    """
    from .config import SCREEN_WIDTH, SCREEN_HEIGHT, OBJECT_SIZE

    return {
        "car": car.get_state(),
        "boxes": [b.get_state() for b in boxes],
        "walls": [w.get_state() for w in walls],
        "bombs": [b.get_state() for b in bombs],
        "targets": [t.get_state() for t in targets],
        "config": {
            "screen_width": SCREEN_WIDTH,
            "screen_height": SCREEN_HEIGHT,
            "object_size": OBJECT_SIZE
        }
    }
