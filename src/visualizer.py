import matplotlib.pyplot as plt
import numpy as np
import math
from typing import Tuple, List, Optional

from src.config import VehicleConfig
from src.interfaces import PlannerResult, BaseMap
from src.grid_map import GridMap
from src.heuristic import HolonomicHeuristic

class Visualizer:
    """
    科研专用可视化器。
    负责将规划结果绘制成高质量的图表（用于论文插图或调试）。
    """
    def __init__(self, vehicle_config: VehicleConfig):
        self.vehicle_config = vehicle_config
        # 论文常用的配色方案
        self.colors = {
            'obstacle': '.k',
            'start': '#00CC00',   # Green
            'goal': '#FF0000',    # Red
            'path': 'red',
            'tree': 'green',
            'vehicle': 'black'
        }

    def visualize_planning_result(self, 
                                  map_env: GridMap, 
                                  start: Tuple[float, float, float], 
                                  goal: Tuple[float, float, float], 
                                  result: PlannerResult,
                                  show_search_tree: bool = True,
                                  title: str = "Hybrid A* Planning"):
        """
        绘制静态结果图：包含地图、搜索树、最终路径和起终点车辆。
        这是论文中最常用的展示形式。
        """
        plt.figure(figsize=(10, 10))
        
        # 1. 绘制地图障碍物
        if hasattr(map_env, 'plot_map'):
            map_env.plot_map() # 调用 GridMap 自带的绘图
        
        # 2. [科研] 绘制搜索树 (Search Tree)
        # 用透明度高的点表示探索过的区域，展示算法的搜索范围
        if show_search_tree and result.debug_data:
            history = result.debug_data.expansion_history
            if history:
                x_list = [p[0] for p in history]
                y_list = [p[1] for p in history]
                plt.plot(x_list, y_list, '.', color=self.colors['tree'], 
                         markersize=2, alpha=0.2, label="Search Tree")

        # 3. 绘制最终路径
        if result.success:
            plt.plot(result.path_x, result.path_y, '-', color=self.colors['path'], 
                     linewidth=2.0, label="Final Path")
            
            # (可选) 在路径上每隔一段画一个车身轮廓，展示姿态变化
            # skip = max(1, len(result.path_x) // 5)
            # for i in range(0, len(result.path_x), skip):
            #     self._plot_car_body(result.path_x[i], result.path_y[i], result.path_yaw[i], color='gray', alpha=0.3)

        # 4. 绘制起点和终点车辆
        self._plot_car_body(start[0], start[1], start[2], color='blue', label="Start")
        self._plot_car_body(goal[0], goal[1], goal[2], color='red', label="Goal")

        plt.title(title)
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        # 不调用 plt.show()，允许外部继续添加元素或保存图片
        
    def visualize_heuristic_heatmap(self, heuristic: HolonomicHeuristic, map_env: GridMap):
        """
        [论文神器] 绘制启发式代价热力图。
        展示 Dijkstra 势场是如何引导搜索避开死胡同的。
        """
        if heuristic.heuristic_map is None:
            print("Heuristic map is empty. Cannot plot heatmap.")
            return

        plt.figure(figsize=(10, 8))
        
        # 处理无穷大值以便绘图 (将 inf 设为最大值的 1.2 倍)
        h_map = heuristic.heuristic_map.copy()
        valid_mask = h_map != float('inf')
        max_val = np.max(h_map[valid_mask]) if np.any(valid_mask) else 100.0
        h_map[~valid_mask] = max_val * 1.2

        # 绘制热力图 (注意转置)
        plt.imshow(h_map.T, origin='lower', cmap='jet_r', 
                   extent=[0, map_env.width_m, 0, map_env.height_m],
                   interpolation='nearest')
        
        plt.colorbar(label='Heuristic Cost to Goal')
        plt.title("Holonomic Heuristic Potential Field")
        
        # 叠加障碍物点
        obs_x, obs_y = [], []
        for x in range(map_env.width_idx):
            for y in range(map_env.height_idx):
                if map_env.obstacle_map[x][y]:
                    obs_x.append(x * map_env.config.xy_resolution)
                    obs_y.append(y * map_env.config.xy_resolution)
        plt.plot(obs_x, obs_y, '.k', markersize=1, alpha=0.5)

    def animate_path(self, result: PlannerResult, dt: float = 0.05):
        """简单的路径回放动画"""
        if not result.success:
            return
            
        print("Playing animation... (Close window to exit)")
        for i in range(len(result.path_x)):
            plt.cla()
            plt.axis("equal")
            plt.grid(True)
            
            # 画当前车
            self._plot_car_body(result.path_x[i], result.path_y[i], result.path_yaw[i])
            # 画轨迹
            plt.plot(result.path_x, result.path_y, "-r", alpha=0.3)
            
            plt.pause(dt)

    def _plot_car_body(self, x, y, yaw, color='black', alpha=1.0, label=None):
        """内部辅助函数：绘制车辆轮廓"""
        outline = self.vehicle_config.vehicle_outline
        
        # 旋转矩阵 (2x2)
        rot = np.array([
            [math.cos(yaw), math.sin(yaw)],
            [-math.sin(yaw), math.cos(yaw)]
        ])
        
        # 变换: Outline(2xN) -> Transpose -> Dot -> Transpose -> Translate
        # (N, 2) dot (2, 2)
        rotated_outline = (outline.T.dot(rot)).T
        rotated_outline[0, :] += x
        rotated_outline[1, :] += y
        
        plt.plot(rotated_outline[0, :], rotated_outline[1, :], 
                 color=color, alpha=alpha, linewidth=1.5, label=label)
        
        # 画个箭头表示车头朝向
        arrow_len = self.vehicle_config.wheelbase * 0.5
        plt.arrow(x, y, arrow_len * math.cos(yaw), arrow_len * math.sin(yaw),
                  head_width=0.3, fc=color, ec=color, alpha=alpha)