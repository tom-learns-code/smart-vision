"""
字体管理器 - 支持中文显示
"""
import pygame
import os
import sys


class FontManager:
    """字体管理器：统一处理中文字体加载"""

    # 字体文件路径（按优先级）
    FONT_PATHS = [
        # Windows系统字体
        "C:/Windows/Fonts/simhei.ttf",      # 黑体
        "C:/Windows/Fonts/msyh.ttf",         # 微软雅黑
        "C:/Windows/Fonts/simkai.ttf",      # 楷体
        # macOS系统字体
        "/System/Library/Fonts/PingFang.ttc",
        "/Library/Fonts/Arial Unicode.ttf",
        # Linux系统字体
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]

    # 系统字体名称（按优先级）
    SYS_FONT_NAMES = [
        "Microsoft YaHei",  # 微软雅黑
        "SimHei",          # 黑体
        "SimKai",          # 楷体
        "PingFang SC",     # 苹方（macOS）
        "Arial Unicode MS", # Arial Unicode
        "WenQuanYi Zen Hei", # 文泉驿正黑（Linux）
        "sans-serif",      # 通用无衬线
    ]

    _fonts = {}

    @classmethod
    def get_font(cls, size=24, bold=False):
        """
        获取字体实例

        Args:
            size: 字体大小
            bold: 是否加粗

        Returns:
            pygame.font.Font对象
        """
        key = (size, bold)

        if key in cls._fonts:
            return cls._fonts[key]

        # 尝试从文件加载
        font = cls._load_font_file(size)

        # 如果文件加载失败，尝试系统字体
        if font is None:
            font = cls._load_system_font(size)

        # 如果都失败，使用默认字体
        if font is None:
            print("警告：无法加载中文字体，使用默认字体（中文将显示为方框）")
            font = pygame.font.Font(None, size)

        cls._fonts[key] = font
        return font

    @classmethod
    def _load_font_file(cls, size):
        """从文件加载字体"""
        for font_path in cls.FONT_PATHS:
            if os.path.exists(font_path):
                try:
                    return pygame.font.Font(font_path, size)
                except Exception as e:
                    print(f"字体文件加载失败 {font_path}: {e}")
                    continue
        return None

    @classmethod
    def _load_system_font(cls, size):
        """加载系统字体"""
        for font_name in cls.SYS_FONT_NAMES:
            try:
                font = pygame.font.SysFont(font_name, size)
                # 测试字体是否支持中文
                test_surface = font.render("中", True, (255, 255, 255))
                if test_surface:
                    return font
            except Exception as e:
                print(f"系统字体加载失败 {font_name}: {e}")
                continue
        return None

    @classmethod
    def clear_cache(cls):
        """清除字体缓存"""
        cls._fonts.clear()
