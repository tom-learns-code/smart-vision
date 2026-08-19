# 车推箱子算法求解器

## 目录
- `algorithms/` — 8 步求解管线（连通分析→炸点规划→分配→路径→调度→Action）
- `core/` — 物理引擎 + 地图分析 + BFS + 规避节点图 + 碰撞管线
- `design/` — 架构图 + 算法输出接口规范 v1.0 + 阶段二修正方案
- `vis/` — Pygame 可视化渲染（renderer / window）
- `maps_export/` — 标准地图文件（.txt 格式）
- `maps_import/` — 导入地图 + 历史存档

## 入口
- 完整求解：`python run_solver.py [map_file]`
- 管线验证（步骤 1-4）：`python run_pipeline.py`
- 可视化验证（3 张地图）：`python verify.py`
- 游戏运行：`python main.py`

## 构建 & 测试
- 依赖：`pip install -r requirements.txt`（仅 pygame）
- 无 CI / 无 pytest

## 禁区
- `maps_import/存放/` 历史存档，只读不写
- `__pycache__/` 编译缓存，不手动修改
- `recorded_*.json` 录播数据，只读
