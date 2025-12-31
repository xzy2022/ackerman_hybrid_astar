import sys
import os
import math
import random
import argparse
import numpy as np
import matplotlib.pyplot as plt

# 添加项目根目录到 Python 路径
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(current_dir)
sys.path.append(root_dir)

from src.config import HybridAStarConfig, VehicleConfig
from src.grid_map import GridMap
from src.planner import HybridAStarPlanner
from src.visualizer import Visualizer

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Hybrid A* Simulation with Performance Control')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility (e.g., 5, 42, etc.)')
    parser.add_argument('--no-visualization', action='store_true',
                        help='Run without visualization (for testing)')

    # === [新增] 性能控制参数 ===
    parser.add_argument('--log-tree', action='store_true',
                        help='Record and visualize the search tree (expansion history)')
    parser.add_argument('--log-closed', action='store_true',
                        help='Record the closed set costs (for analysis)')
    parser.add_argument('--sample-rate', type=int, default=10,
                        help='Sampling rate for debug data (default: 10, i.e., record 1 in 10 nodes)')

    args = parser.parse_args()

    print("=== Hybrid A* Research Simulation ===")

    # 设置随机种子（如果提供）
    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)
        print(f"Random seed set to: {args.seed}")
    else:
        print("Random seed: not set (using random initialization)")

    # 1. 初始化配置
    h_config = HybridAStarConfig()
    h_config.move_step_grid = 2.0
    h_config.heuristic_weight = 5.0

    # === [新增] 将命令行参数应用到配置 ===
    # 注意：--log-tree 是开关参数，如果指定则为 True，否则保持配置默认值
    if args.log_tree:
        h_config.record_expansion_history = True
    if args.log_closed:
        h_config.record_visited_costs = True
    h_config.debug_sample_rate = args.sample_rate

    # 打印当前的调试配置状态
    print(f"Debug Config: Tree={h_config.record_expansion_history}, "
          f"Closed={h_config.record_visited_costs}, "
          f"SampleRate={h_config.debug_sample_rate}")

    v_config = VehicleConfig()

    # 2. 初始化地图
    grid_map = GridMap(h_config, v_config)
    grid_map.generate_random_map(width_m=50.0, height_m=50.0, obstacle_num=150)

    # 3. 设定任务
    start = (10.0, 10.0, math.radians(0.0))
    goal = (40.0, 40.0, math.radians(90.0))

    # 清理起终点周边（智能计算范围）
    s_idx = grid_map.get_index_from_pos(start[0], start[1])
    g_idx = grid_map.get_index_from_pos(goal[0], goal[1])

    # [智能计算] 根据 VehicleConfig 动态计算清理半径
    # 确保覆盖整个车身（前悬 3.3m，后悬 1.0m）+ 安全余量
    safe_margin_m = max(v_config.front_hang, v_config.rear_hang) + 1.0
    margin = int(math.ceil(safe_margin_m / h_config.xy_resolution))

    print(f"Clearing {margin*2+1}x{margin*2+1} grid around Start/Goal "
          f"(safe_radius={safe_margin_m:.1f}m, margin={margin} grids)")

    # 清理起点和终点周边的障碍物
    for dx in range(-margin, margin+1):
        for dy in range(-margin, margin+1):
            if 0 <= s_idx[0]+dx < grid_map.width_idx and 0 <= s_idx[1]+dy < grid_map.height_idx:
                grid_map.obstacle_map[s_idx[0]+dx][s_idx[1]+dy] = False
            if 0 <= g_idx[0]+dx < grid_map.width_idx and 0 <= g_idx[1]+dy < grid_map.height_idx:
                grid_map.obstacle_map[g_idx[0]+dx][g_idx[1]+dy] = False

    # 4. 核心规划
    planner = HybridAStarPlanner(h_config, v_config)
    print("Planning...")

    result = planner.plan(start, goal, grid_map)

    # === [改进点 1]：增强的失败诊断信息 ===
    if result.success:
        print(f"Success! Cost: {result.cost:.2f}, Nodes: {result.debug_data.nodes_expanded}")
        print("Running strict collision detection...")
        collided_obstacles = grid_map.check_strict_path_collision(
            result.path_x, result.path_y, result.path_yaw
        )
        result.collided_obstacles_coords = collided_obstacles

        if collided_obstacles:
            print(f"WARNING: Strict collision detected! {len(collided_obstacles)} obstacle(s) collided.")
        else:
            print("Strict collision check passed! No collisions detected.")
    else:
        print("\n" + "="*40)
        print("  PLANNING FAILED - DIAGNOSTICS")
        print("="*40)
        print(f"  - Nodes Expanded: {result.debug_data.nodes_expanded} (If this is 1, Start is blocked)")
        print(f"  - Execution Time: {result.debug_data.execution_time_ms:.2f} ms")

        # 诊断 A: 起点/终点碰撞检测
        is_start_hit = grid_map.check_collision(*start)
        is_goal_hit = grid_map.check_collision(*goal)
        print(f"  - Start Pose Collision: {is_start_hit} {'[CRITICAL]' if is_start_hit else '[OK]'}")
        print(f"  - Goal Pose Collision:  {is_goal_hit} {'[CRITICAL]' if is_goal_hit else '[OK]'}")

        # 诊断 B: 启发式连通性 (Dijkstra)
        from src.heuristic import HolonomicHeuristic
        print("  - Checking Holonomic Heuristic (Dijkstra)...")
        h_debug = HolonomicHeuristic(h_config, grid_map)
        h_val = h_debug.calculate(start, goal)
        print(f"    Heuristic value from Start: {h_val}")
        if h_val == float('inf'):
            print("    [!] START IS DISCONNECTED from GOAL (Heuristic is INF).")
            print("    Possible causes: Enclosed by obstacles, or map disconnected.")
        else:
            print("    [OK] Path exists in 2D Grid (vehicle treated as point mass).")
            print("    Note: Holonomic Heuristic ignores vehicle geometry (width/length).")
            print("    If planning failed, likely cause: Vehicle body collision")
            print("              (car's front/rear hits obstacles despite clear path for center).")
        print("="*40 + "\n")

    # 5. [Visualization] 结果展示
    if not args.no_visualization:
        viz = Visualizer(v_config)

        # === [改进点 2]：动态标题显示状态 ===
        title_prefix = f"Seed={args.seed}" if args.seed is not None else "Random"
        status_str = "SUCCESS" if result.success else "FAILED"
        title = f"{title_prefix}: Hybrid A* Result [{status_str}]"

        # 显示规划结果（即使失败也会显示搜索树和地图）
        viz.visualize_planning_result(grid_map, start, goal, result,
                                      show_search_tree=True,
                                      animate=True if result.success else False, # 失败时不播放动画
                                      title=title)

        # === [改进点 3]：失败时强制显示热力图 ===
        # 这能让你直观看到为什么从起点走不通
        if not result.success:
            print("Displaying Heuristic Heatmap for failure analysis...")
            # 如果之前没算过（诊断块里算过一次，但这里需要对象），重新实例化
            from src.heuristic import HolonomicHeuristic
            heuristic_debug = HolonomicHeuristic(h_config, grid_map)
            heuristic_debug.calculate(start, goal)
            viz.visualize_heuristic_heatmap(heuristic_debug, grid_map)

        plt.show()
    else:
        print("Visualization skipped (--no-visualization flag set)")

if __name__ == "__main__":
    main()
