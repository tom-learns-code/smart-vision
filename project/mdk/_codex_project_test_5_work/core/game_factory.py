"""
游戏对象管理器 - 管理游戏对象的创建和初始化
"""
from .car import Car
from .game_object import GameObject
from .wall import Wall
from .bomb import Bomb
from .target import Target
from .config import OBJECT_SIZE, COLOR_CAR, COLOR_BOX, COLOR_BOMB, COLOR_TARGET, WALL_SIZE, COLOR_WALL, SCREEN_WIDTH, SCREEN_HEIGHT


class GameFactory:
    """游戏对象工厂 - 创建和管理游戏对象"""
    
    @staticmethod
    def create_car(x=None, y=None):
        """创建车辆"""
        if x is None:
            x = 200
        if y is None:
            y = SCREEN_HEIGHT / 2
        return Car(x, y, OBJECT_SIZE, COLOR_CAR)
    
    @staticmethod
    def create_box(x=None, y=None):
        """创建箱子"""
        if x is None:
            x = SCREEN_WIDTH / 2 - 100
        if y is None:
            y = SCREEN_HEIGHT / 2
        return GameObject(x, y, OBJECT_SIZE, COLOR_BOX)
    
    @staticmethod
    def create_bomb(x=None, y=None):
        """创建炸弹"""
        if x is None:
            x = SCREEN_WIDTH / 2 - 100
        if y is None:
            y = SCREEN_HEIGHT / 2 + 100
        return Bomb(x, y, OBJECT_SIZE, COLOR_BOMB)
    
    @staticmethod
    def create_target(x=None, y=None):
        """创建目标点"""
        if x is None:
            x = SCREEN_WIDTH * 0.85
        if y is None:
            y = SCREEN_HEIGHT / 2
        return Target(x, y, OBJECT_SIZE, COLOR_TARGET)
    
    @staticmethod
    def create_wall(x, y, width=None, height=None, is_boundary=False):
        """创建墙体"""
        if width is None:
            width = WALL_SIZE
        if height is None:
            height = WALL_SIZE
        return Wall(x, y, width, height, COLOR_WALL, is_boundary)
    
    @staticmethod
    def create_boundary_walls():
        """创建边界墙体"""
        wall_thickness = 20
        boundary_walls = [
            Wall(SCREEN_WIDTH / 2, wall_thickness / 2,
                  SCREEN_WIDTH, wall_thickness, COLOR_WALL, is_boundary=True),  # 上墙
            Wall(SCREEN_WIDTH / 2, SCREEN_HEIGHT - wall_thickness / 2,
                  SCREEN_WIDTH, wall_thickness, COLOR_WALL, is_boundary=True),  # 下墙
            Wall(wall_thickness / 2, SCREEN_HEIGHT / 2,
                  wall_thickness, SCREEN_HEIGHT, COLOR_WALL, is_boundary=True),  # 左墙
            Wall(SCREEN_WIDTH - wall_thickness / 2, SCREEN_HEIGHT / 2,
                  wall_thickness, SCREEN_HEIGHT, COLOR_WALL, is_boundary=True),  # 右墙
        ]
        return boundary_walls
    
    @staticmethod
    def create_default_walls():
        """创建默认墙体"""
        walls = []
        # 默认添加一个中间墙体
        walls.append(Wall(SCREEN_WIDTH * 0.7, SCREEN_HEIGHT / 2,
                         WALL_SIZE, WALL_SIZE, COLOR_WALL))
        return walls
