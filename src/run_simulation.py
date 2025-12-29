import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

# 确保能导入 src 包 (如果脚本在根目录下，通常不需要这行，但为了稳健加上)
sys.path.append(os.getcwd())

from src.config import HybridAStarConfig, VehicleConfig
from src.grid_map import GridMap
from src.planner import HybridAStarPlanner

def plot_vehicle(x, y, yaw, vehicle_config, color='black'):
    """辅助函数：绘制车辆轮廓"""
    outline = vehicle_config.vehicle_outline
    rot_mat = np.array([
        [math.cos(yaw), math.sin(yaw)],
        [-math.sin(yaw), math.cos(yaw)]
    ])
    
    # 旋转 + 平移
    rotated_outline = (outline.T.dot(rot_mat)).T
    rotated_outline[0, :] += x
    rotated_outline[1, :] += y
    
    plt.plot(rotated_outline[0, :], rotated_outline[1, :], color=color, linewidth=1.5)
    
    # 画箭头指示方向
    arrow_len = 1.0
    plt.arrow(x, y, arrow_len * math.cos(yaw), arrow_len * math.sin(yaw), 
              head_width=0.3, head_length=0.4, fc=color, ec=color)

def main():
    print("=== Hybrid A* Research Simulation Start ===")
    
    # 1. 初始化配置 (Configuration)
    # 你可以在这里修改参数进行对比实验 (Ablation Study)
    h_config = HybridAStarConfig()
    h_config.move_step_grid = 2.0  # 调整步长
    h_config.heuristic_weight = 5.0 # 调整启发式权重
    
    v_config = VehicleConfig()
    # v_config.wheelbase = 2.8 # 修改车辆参数
    
    # 2. 初始化地图 (Environment)
    grid_map = GridMap(h_config, v_config)
    # 生成随机地图：50x50米，80个障碍物
    grid_map.generate_random_map(width_m=50.0, height_m=50.0, obstacle_num=80)
    
    # 3. 设定任务 (Mission)
    # 起点 (x, y, yaw_rad)
    start = (10.0, 10.0, math.radians(0.0)) 
    # 终点
    goal = (40.0, 40.0, math.radians(90.0))
    
    # 确保起点终点不被障碍物覆盖 (简单清理一下周边)
    # 实际项目中应该有更严谨的 check
    s_idx = (round(start[0]/h_config.xy_resolution), round(start[1]/h_config.xy_resolution))
    g_idx = (round(goal[0]/h_config.xy_resolution), round(goal[1]/h_config.xy_resolution))
    grid_map.obstacle_map[s_idx[0]-2:s_idx[0]+3, s_idx[1]-2:s_idx[1]+3] = False
    grid_map.obstacle_map[g_idx[0]-2:g_idx[0]+3, g_idx[1]-2:g_idx[1]+3] = False

    # 4. 核心规划 (Planning)
    planner = HybridAStarPlanner(h_config, v_config)
    print("Planning...")
    
    result = planner.plan(start, goal, grid_map)
    
    # 5. 结果分析与可视化 (Analysis & Visualization)
    if result.success:
        print(f"Path Found! Cost: {result.cost:.2f}")
        print(f"Nodes Expanded: {result.debug_data.nodes_expanded}")
        print(f"Time Elapsed: {result.debug_data.execution_time_ms:.2f} ms")
        
        # --- 绘图 ---
        plt.figure(figsize=(10, 10))
        
        # A. 画静态地图
        grid_map.plot_map()
        
        # B. [科研核心] 可视化搜索树 (Search Tree Visualization)
        # 这就是我们增加 SearchDebugData 的目的
        print("Plotting Search Tree (Nodes Expanded)...")
        # 将扩展历史解包
        explored_x = [p[0] for p in result.debug_data.expansion_history]
        explored_y = [p[1] for p in result.debug_data.expansion_history]
        # 用绿色小点表示搜索过的区域，透明度设低一点
        plt.plot(explored_x, explored_y, ".g", markersize=2, alpha=0.3, label="Explored Nodes")
        
        # C. 画最终路径
        plt.plot(result.path_x, result.path_y, "-r", linewidth=2.0, label="Final Path")
        
        # D. 画起点终点车辆
        plot_vehicle(start[0], start[1], start[2], v_config, color='blue')
        plot_vehicle(goal[0], goal[1], goal[2], v_config, color='red')
        
        plt.title(f"Hybrid A* Result (Cost: {result.cost:.2f})")
        plt.legend()
        plt.axis("equal")
        plt.show()
        
        # (可选) 动画展示：如果你想看动态过程，可以遍历 path_x 并动态 plot_vehicle
    else:
        print("Planning Failed.")
        # 即使失败，也可以画出 explored nodes 看看是哪里堵死了
        plt.figure(figsize=(10, 10))
        grid_map.plot_map()
        explored_x = [p[0] for p in result.debug_data.expansion_history]
        explored_y = [p[1] for p in result.debug_data.expansion_history]
        plt.plot(explored_x, explored_y, ".g", markersize=2, alpha=0.3)
        plt.title("Planning Failed - Search Tree")
        plt.show()

if __name__ == "__main__":
    main()