"""
路径预计算执行器 - 预计算路径并执行
"""


class PathExecutor:
    """路径执行器 - 预计算路径后执行"""

    def __init__(self):
        """初始化路径执行器"""
        self.algorithm_module = None
        self.is_running = False
        self.current_index = 0
        self.path = []  # 预计算的路径
        self.max_path_length = 1000  # 最大路径长度

        # 速度检测
        self.stuck_timer = 0.0  # 卡住计时器（秒）
        self.stuck_threshold = 1.0  # 卡住阈值（秒）
        self.min_speed_threshold = 5.0  # 最小速度阈值（像素/秒）

    def load_algorithm(self, module_path):
        """
        加载外部算法模块

        Args:
            module_path: 算法模块的文件路径

        Returns:
            bool: 是否加载成功
        """
        import importlib.util
        import sys
        from pathlib import Path

        try:
            module_name = Path(module_path).stem
            spec = importlib.util.spec_from_file_location(module_name, module_path)
            self.algorithm_module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = self.algorithm_module
            spec.loader.exec_module(self.algorithm_module)
            return True
        except Exception as e:
            print(f"算法加载失败: {e}")
            return False

    def compute_path(self, world_state):
        """
        计算完整路径

        Args:
            world_state: 世界状态字典

        Returns:
            list: 路径数据列表
        """
        if not self.algorithm_module or not hasattr(self.algorithm_module, 'generate_path'):
            print("未加载有效算法模块")
            return []

        try:
            return self.algorithm_module.generate_path(world_state)
        except Exception as e:
            print(f"路径计算失败: {e}")
            return []

    def load_path(self, path):
        """
        加载预计算路径

        Args:
            path: 路径数据列表
        """
        self.path = path
        self.current_index = 0
        self.is_running = False
        self.stuck_timer = 0.0  # 重置卡住计时器

    def get_current_target(self):
        """
        获取当前目标点

        Returns:
            dict or None: 当前目标点
        """
        if not self.path or self.current_index >= len(self.path):
            return None
        return self.path[self.current_index]

    def check_stuck(self, car_speed, dt):
        """
        检查车辆是否卡住

        Args:
            car_speed: 车辆速度（像素/秒）
            dt: 时间步长（秒）

        Returns:
            bool: 是否卡住
        """
        if car_speed < self.min_speed_threshold:
            self.stuck_timer += dt
            if self.stuck_timer >= self.stuck_threshold:
                return True
        else:
            self.stuck_timer = max(0, self.stuck_timer - dt)
        return False

    def step(self):
        """
        执行一步，前进到下一个目标点

        Returns:
            bool: 是否还有更多目标点
        """
        if not self.path or self.current_index >= len(self.path):
            self.is_running = False
            return False

        self.current_index += 1

        if self.current_index >= len(self.path):
            self.is_running = False
            return False

        return True

    def reset(self):
        """重置执行器"""
        self.is_running = False
        self.current_index = 0
        self.path = []
        self.stuck_timer = 0.0
