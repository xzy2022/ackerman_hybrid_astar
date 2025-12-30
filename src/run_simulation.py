import sys
import os
import math
import matplotlib.pyplot as plt


from src.config import HybridAStarConfig, VehicleConfig
from src.grid_map import GridMap
from src.planner import HybridAStarPlanner
from src.visualizer import Visualizer  

def main():
    print("=== Hybrid A* Research Simulation ===")
    
    # 1. 初始化配置 (Configuration)
    # 你可以在这里修改参数进行对比实验 (Ablation Study)
    h_config = HybridAStarConfig()
    h_config.move_step_grid = 2.0  # 调整步长
    h_config.heuristic_weight = 5.0 # 调整启发式权重
    
    v_config = VehicleConfig()
    
    # 2. 初始化地图 (Environment)
    grid_map = GridMap(h_config, v_config)
    # 生成随机地图：50x50米，80个障碍物
    grid_map.generate_random_map(width_m=50.0, height_m=50.0, obstacle_num=150)
    
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
    
    if result.success:
        print(f"Success! Cost: {result.cost:.2f}, Nodes: {result.debug_data.nodes_expanded}")
        
        # 5. 严格碰撞检测（Ground Truth Check）
        # 使用多边形精确检测验证规划结果的安全性
        # 这可以作为消融实验的一部分，对比圆形近似检测与多边形精确检测的差异
        print("Running strict collision detection...")
        collided_obstacles = grid_map.check_strict_path_collision(
            result.path_x, result.path_y, result.path_yaw
        )
        result.collided_obstacles_coords = collided_obstacles
        
        if collided_obstacles:
            print(f"WARNING: Strict collision detected! {len(collided_obstacles)} obstacle(s) collided.")
            print("This indicates the circle approximation model used during planning")
            print("may have missed some collisions that polygon detection catches.")
        else:
            print("Strict collision check passed! No collisions detected.")
    else:
        print("Planning Failed.")

    # 5. [Visualization] 结果展示 (GUI Phase)
    # 只有需要看结果时才实例化 Visualizer
    viz = Visualizer(v_config)
    
    # 图1: 规划结果总览
    viz.visualize_planning_result(grid_map, start, goal, result, 
                                  show_search_tree=True, 
                                  animate = True,  # 开启动画演示
                                  title="Scenario 1: Random Obstacles")
    
    # 图2 (可选): 启发式热力图分析
    # 为了画这个图，我们需要单独访问一下 heuristic 对象（或者让 planner 返回它）
    # 这里演示重新实例化一个 heuristic 来画图
    from src.heuristic import HolonomicHeuristic
    heuristic_debug = HolonomicHeuristic(h_config, grid_map)
    heuristic_debug.calculate(start, goal) # 触发一次计算以生成 map
    viz.visualize_heuristic_heatmap(heuristic_debug, grid_map)
    
    plt.show()

if __name__ == "__main__":
    main()