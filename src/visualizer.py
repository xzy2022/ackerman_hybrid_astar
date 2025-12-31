import matplotlib.pyplot as plt
import numpy as np
import math
from typing import Tuple, List, Optional

from src.config import VehicleConfig
from src.interfaces import PlannerResult
from src.grid_map import GridMap
from src.heuristic import HolonomicHeuristic

class Visualizer:
    """
    科研专用可视化器。
    负责将规划结果绘制成高质量的图表（用于论文插图或调试），并提供动画演示功能。
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
                                  animate: bool = False, # <--- 新增参数：是否开启动画
                                  title: str = "Hybrid A* Planning Result"):
        """
        绘制规划结果。
        如果 animate=False (默认)，绘制静态结果图（论文用）。
        如果 animate=True，则在绘制背景后开始动态演示小车运动轨迹。
        """
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111)
        
        # --- 1. 绘制静态背景元素 ---
        
        # 绘制地图障碍物
        if hasattr(map_env, 'plot_map'):
            map_env.plot_map() # 调用 GridMap 自带的绘图
        
        # [科研] 绘制搜索树 (Search Tree)
        # 用透明度高的点表示探索过的区域，展示算法的搜索范围
        if show_search_tree and result.debug_data and result.debug_data.expansion_history:
            history = result.debug_data.expansion_history
            x_list = [p[0] for p in history]
            y_list = [p[1] for p in history]
            ax.plot(x_list, y_list, '.', color=self.colors['tree'],
                     markersize=2, alpha=0.8, label="Closed List (Explored)")

            # [新增] 绘制 Open List (候选前沿/搜索边界)
            # 用蓝色点表示"可能的未来"，与绿色的"过去"形成对比
            if result.debug_data.open_list_points:
                open_list = result.debug_data.open_list_points
                ox = [p[0] for p in open_list]
                oy = [p[1] for p in open_list]
                ax.plot(ox, oy, '.', color='blue',
                        markersize=2, alpha=0.2, label="Open List (Frontier)")

        # 绘制最终完整路径线 (作为背景)
        if result.success and result.path_x:
            ax.plot(result.path_x, result.path_y, '-', color=self.colors['path'], 
                     linewidth=2.0, alpha=0.6, label="Final Path")

        # 绘制起点和终点车辆姿态
        self._plot_car_body(ax, start[0], start[1], start[2], color=self.colors['start'], label="Start")
        self._plot_car_body(ax, goal[0], goal[1], goal[2], color=self.colors['goal'], label="Goal")

        # 绘制严格碰撞检测发现的碰撞点（黄色 X 标记）
        if result.collided_obstacles_coords:
            collision_x = [coord[0] for coord in result.collided_obstacles_coords]
            collision_y = [coord[1] for coord in result.collided_obstacles_coords]
            ax.plot(collision_x, collision_y, 'yx', markersize=10, markeredgewidth=2, 
                    label="Strict Collision Detected", zorder=15)

        # 设置图表属性
        ax.set_title(title)
        ax.legend(loc='upper right')
        ax.axis("equal")
        ax.grid(True)

        # --- 2. 动画演示 (可选) ---
        if animate and result.success:
            print("Starting animation... (Press 'q' on the plot window to quit usually works)")
            self._run_animation(ax, result)
        else:
            # 如果不是动画模式，直接显示静态图
            plt.show()


        
    def visualize_heuristic_heatmap(self, heuristic: HolonomicHeuristic, map_env: GridMap):
        """
        [论文神器] 绘制启发式代价热力图。
        展示 Dijkstra 势场是如何引导搜索避开死胡同的。
        """
        if heuristic.heuristic_map is None:
            print("Heuristic map is empty. Cannot plot heatmap.")
            return

        plt.figure(figsize=(10, 8))
        
        # 处理无穷大值以便绘图 (将 inf 设为最大值的 1.2 倍，显示为深红色区域)
        h_map = heuristic.heuristic_map.copy()
        valid_mask = h_map != float('inf')
        max_val = np.max(h_map[valid_mask]) if np.any(valid_mask) else 100.0
        h_map[~valid_mask] = max_val * 1.2

        # 绘制热力图 (注意需要转置 .T 才能与地图坐标系对齐)
        # origin='lower' 确保 (0,0) 在左下角
        plt.imshow(h_map.T, origin='lower', cmap='jet_r', 
                   extent=[0, map_env.width_m, 0, map_env.height_m],
                   interpolation='nearest')
        
        plt.colorbar(label='Heuristic Cost to Goal (Dijkstra distance)')
        plt.title("Holonomic Heuristic Potential Field (Obstacle Aware)")
        
        # 叠加半透明的障碍物点，方便对比
        obs_x, obs_y = [], []
        for x in range(map_env.width_idx):
            for y in range(map_env.height_idx):
                if map_env.obstacle_map[x][y]:
                    obs_x.append(x * map_env.config.xy_resolution)
                    obs_y.append(y * map_env.config.xy_resolution)
        plt.plot(obs_x, obs_y, '.k', markersize=2, alpha=0.4, label="Obstacles")
        plt.legend()
        plt.axis("equal")
        # 热力图通常单独展示，直接显示
        plt.show()

    def _run_animation(self, ax, result: PlannerResult):
        """执行路径跟随动画的内部循环"""
        path_len = len(result.path_x)
        # 根据路径长度自动调整跳帧数，避免长路径动画过慢
        skip = max(1, path_len // 100) if path_len > 100 else 1
        dt = 0.015 # 每帧暂停时间

        current_car_artists = []

        for i in range(0, path_len, skip):
            # 1. 清除上一帧的车辆 (如果有)
            for artist in current_car_artists:
                artist.remove()
            current_car_artists.clear()
            
            # 2. 绘制当前帧车辆，并保存返回的绘图对象
            # 使用黑色实线绘制移动中的车辆
            artists = self._plot_car_body(ax, result.path_x[i], result.path_y[i], result.path_yaw[i], 
                                          color=self.colors['vehicle'])
            current_car_artists.extend(artists)

            # 3. 暂停以更新画面
            plt.pause(dt)

        # 动画结束，绘制最终位置（红色强调）并保留
        for artist in current_car_artists:
            artist.remove()
        self._plot_car_body(ax, result.path_x[-1], result.path_y[-1], result.path_yaw[-1], color='red')
        plt.draw()
        print("Animation finished.")
        # 保持窗口打开直到用户关闭
        plt.show()


    def _plot_car_body(self, ax, x, y, yaw, color='black', alpha=1.0, label=None):
        """
        内部辅助函数：绘制车辆轮廓和方向箭头，以及【调试用碰撞检测圆】。
        Returns:
            list: 包含绘制的 Line2D, FancyArrow, Patch 对象的列表，用于后续清除动画帧。
        """
        artists = [] # 存储本帧所有绘图对象
        outline = self.vehicle_config.vehicle_outline
        
        # 旋转矩阵 (2x2)
        rot = np.array([
            [math.cos(yaw), math.sin(yaw)],
            [-math.sin(yaw), math.cos(yaw)]
        ])
        
        # 变换: Outline(2xN) -> Transpose -> Dot -> Transpose -> Translate
        rotated_outline = (outline.T.dot(rot)).T
        rotated_outline[0, :] += x
        rotated_outline[1, :] += y
        
        # 1. 绘制车身轮廓线
        line, = ax.plot(rotated_outline[0, :], rotated_outline[1, :], 
                        color=color, alpha=alpha, linewidth=1.5, label=label)
        artists.append(line)
        
        # 2. 绘制车头箭头
        arrow_len = self.vehicle_config.wheelbase * 0.5
        arrow = ax.arrow(x, y, arrow_len * math.cos(yaw), arrow_len * math.sin(yaw),
                         head_width=0.3, fc=color, ec=color, alpha=alpha, zorder=10)
        artists.append(arrow)

        # 3. [新增] 绘制碰撞检测圆 (Visual Debugging)
        # 只有在配置文件中定义了相关属性才绘制
        if True:
            if hasattr(self.vehicle_config, 'collision_offsets') and \
            hasattr(self.vehicle_config, 'collision_radius'):
                
                for offset in self.vehicle_config.collision_offsets:
                    # 计算圆心的世界坐标
                    # 沿车身航向轴 (Heading) 前后偏移
                    cx = x + offset * math.cos(yaw)
                    cy = y + offset * math.sin(yaw)
                    r = self.vehicle_config.collision_radius
                    
                    # 创建圆对象 (Cyan青色，虚线)
                    circle = plt.Circle((cx, cy), r, color='cyan', fill=False, 
                                    linestyle='--', linewidth=1, alpha=0.8, zorder=11)
                    ax.add_patch(circle)
                    artists.append(circle) # 加入列表以便动画清除
        
        return artists