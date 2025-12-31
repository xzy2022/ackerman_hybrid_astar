"""
批量测试脚本 - Hybrid A* 算法性能评估
========================================

自动化执行多次规划实验，记录统计数据并生成报告。

作者：Auto-generated
日期：2025-12-31
"""

import sys
import os
import random
import math
import time
import csv
import numpy as np
from datetime import datetime

# --- 1. 环境路径设置 ---
# 将项目根目录添加到 python path，确保能导入 src 模块
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(current_dir)
sys.path.append(root_dir)

from src.config import HybridAStarConfig, VehicleConfig, CollisionMethod
from src.grid_map import GridMap
from src.planner import HybridAStarPlanner


def ensure_dir(directory):
    """确保目录存在，不存在则创建"""
    if not os.path.exists(directory):
        os.makedirs(directory)


def calculate_path_length_meters(path_x, path_y):
    """
    计算路径的几何物理长度 (米)

    Args:
        path_x: 路径 x 坐标列表
        path_y: 路径 y 坐标列表

    Returns:
        float: 路径总长度（米）
    """
    if len(path_x) < 2:
        return 0.0

    length = 0.0
    for i in range(len(path_x) - 1):
        dx = path_x[i+1] - path_x[i]
        dy = path_y[i+1] - path_y[i]
        length += math.hypot(dx, dy)

    return length


def run_single_experiment(exp_id, seed, start_pose, goal_pose, h_config, v_config):
    """
    运行单次实验

    Args:
        exp_id: 实验编号
        seed: 随机种子
        start_pose: 起始位姿 (x, y, yaw)
        goal_pose: 目标位姿 (x, y, yaw)
        h_config: HybridAStarConfig 配置
        v_config: VehicleConfig 配置

    Returns:
        dict: 实验结果字典
    """
    # [关键] 设置随机种子，确保地图生成可复现
    random.seed(seed)
    np.random.seed(seed)

    # 初始化地图（与 src/run_simulation.py 保持一致）
    grid_map = GridMap(h_config, v_config)
    grid_map.generate_random_map(width_m=50.0, height_m=50.0, obstacle_num=150)

    # 清理起终点附近的障碍物（保证起终点合法）
    s_idx = grid_map.get_index_from_pos(start_pose[0], start_pose[1])
    g_idx = grid_map.get_index_from_pos(goal_pose[0], goal_pose[1])

    # [智能计算] 根据 VehicleConfig 动态计算清理半径 (适配 FOOTPRINT/POLYGON 严格检测)
    # 逻辑：取前后悬的最大值，加上 1.0m 的安全余量，再换算成栅格数
    # 这样确保车身旋转时不会立刻扫到障碍物
    safe_margin_m = max(v_config.front_hang, v_config.rear_hang) + 1.0
    margin = int(math.ceil(safe_margin_m / h_config.xy_resolution))

    # 仅在调试时打印，避免批量测试刷屏
    # print(f"Clearing radius: {margin} grids ({safe_margin_m:.1f}m)")

    for dx in range(-margin, margin + 1):
        for dy in range(-margin, margin + 1):
            # 清理起点附近
            nx, ny = s_idx[0] + dx, s_idx[1] + dy
            if 0 <= nx < grid_map.width_idx and 0 <= ny < grid_map.height_idx:
                grid_map.obstacle_map[nx][ny] = False

            # 清理终点附近
            nx, ny = g_idx[0] + dx, g_idx[1] + dy
            if 0 <= nx < grid_map.width_idx and 0 <= ny < grid_map.height_idx:
                grid_map.obstacle_map[nx][ny] = False

    # 初始化规划器
    planner = HybridAStarPlanner(h_config, v_config)

    # 执行规划并记录时间（使用 perf_counter 获得高精度计时）
    start_time = time.perf_counter()

    try:
        result = planner.plan(start_pose, goal_pose, grid_map)

        # 计算执行时间（毫秒）
        end_time = time.perf_counter()
        execution_time_ms = (end_time - start_time) * 1000.0

        # 计算物理路径长度
        path_length_m = 0.0
        if result.success:
            path_length_m = calculate_path_length_meters(result.path_x, result.path_y)

        # 严格碰撞检测 (Ground Truth Check)
        if result.success:
            collided = grid_map.check_strict_path_collision(
                result.path_x, result.path_y, result.path_yaw
            )
            collision_count = len(collided)
        else:
            collision_count = -1  # 规划失败，标记为 -1

        return {
            "exp_id": exp_id,
            "seed": seed,
            "success": result.success,
            "execution_time_ms": execution_time_ms,
            "nodes_expanded": result.debug_data.nodes_expanded,
            "path_cost": result.cost if result.success else 0.0,  # 混合代价
            "path_length_m": path_length_m,  # 物理长度（米）
            "steps_count": len(result.path_x) if result.success else 0,  # 路径点数
            "collision_count": collision_count
        }

    except Exception as e:
        # 捕获异常，防止某次崩溃中断整个测试
        end_time = time.perf_counter()
        execution_time_ms = (end_time - start_time) * 1000.0
        print(f"[Exp {exp_id:03d}] ERROR: {str(e)}")

        return {
            "exp_id": exp_id,
            "seed": seed,
            "success": False,
            "execution_time_ms": execution_time_ms,
            "nodes_expanded": 0,
            "path_cost": 0.0,
            "path_length_m": 0.0,
            "steps_count": 0,
            "collision_count": -2  # 异常标记
        }


def generate_summary(data, filepath, config, total_runs, start_pose, goal_pose):
    """
    生成统计汇总报告

    Args:
        data: 实验数据列表
        filepath: 输出文件路径
        config: HybridAStarConfig 配置
        total_runs: 总实验次数
        start_pose: 起始位姿
        goal_pose: 目标位姿
    """
    success_cases = [d for d in data if d['success']]
    success_rate = (len(success_cases) / total_runs) * 100

    # 仅统计成功案例的平均值
    if success_cases:
        avg_time = sum(d['execution_time_ms'] for d in success_cases) / len(success_cases)
        max_time = max(d['execution_time_ms'] for d in success_cases)
        min_time = min(d['execution_time_ms'] for d in success_cases)
        avg_nodes = sum(d['nodes_expanded'] for d in success_cases) / len(success_cases)
        avg_cost = sum(d['path_cost'] for d in success_cases) / len(success_cases)
        avg_length_m = sum(d['path_length_m'] for d in success_cases) / len(success_cases)
        avg_steps = sum(d['steps_count'] for d in success_cases) / len(success_cases)

        # 统计虽然规划成功但存在几何碰撞的案例（反映圆形近似模型的精度不足）
        unsafe_cases = [d for d in success_cases if d['collision_count'] > 0]
        safe_cases = [d for d in success_cases if d['collision_count'] == 0]
        unsafe_rate = (len(unsafe_cases) / len(success_cases)) * 100

        # 碰撞点统计
        if unsafe_cases:
            avg_collisions = sum(d['collision_count'] for d in unsafe_cases) / len(unsafe_cases)
            max_collisions = max(d['collision_count'] for d in unsafe_cases)
        else:
            avg_collisions = 0
            max_collisions = 0
    else:
        avg_time = 0
        max_time = 0
        min_time = 0
        avg_nodes = 0
        avg_cost = 0
        avg_length_m = 0
        avg_steps = 0
        unsafe_rate = 0
        safe_cases = []
        avg_collisions = 0
        max_collisions = 0

    # 写入报告
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write("Hybrid A* Algorithm Performance Evaluation Report\n")
        f.write("=" * 60 + "\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Total Experiments: {total_runs}\n\n")

        f.write("--- Experiment Setup ---\n")
        f.write(f"Start Pose: ({start_pose[0]:.1f}, {start_pose[1]:.1f}, {math.degrees(start_pose[2]):.1f}°)\n")
        f.write(f"Goal Pose:  ({goal_pose[0]:.1f}, {goal_pose[1]:.1f}, {math.degrees(goal_pose[2]):.1f}°)\n")
        f.write(f"Map Size: 50m x 50m\n")
        f.write(f"Obstacle Count: 150 (randomly distributed)\n\n")

        f.write("--- Algorithm Parameters ---\n")
        f.write(f"XY Resolution: {config.xy_resolution} m\n")
        f.write(f"Move Step Grid: {config.move_step_grid}\n")
        f.write(f"Heuristic Weight: {config.heuristic_weight}\n")
        f.write(f"Yaw Resolution: {config.yaw_resolution_deg}°\n")
        f.write(f"Penalty Reverse: {config.penalty_reverse}\n")
        f.write(f"Penalty Steer Change: {config.penalty_steer_change}\n")
        f.write(f"Penalty Gear Switch: {config.penalty_gear_switch}\n\n")

        f.write("--- Statistics ---\n")
        f.write(f"Success Rate: {success_rate:.2f}% ({len(success_cases)}/{total_runs})\n\n")

        if success_cases:
            f.write("Performance Metrics (Success Cases Only):\n")
            f.write(f"  Planning Time:\n")
            f.write(f"    Average: {avg_time:.2f} ms\n")
            f.write(f"    Min:     {min_time:.2f} ms\n")
            f.write(f"    Max:     {max_time:.2f} ms\n")
            f.write(f"  Nodes Expanded (Average): {avg_nodes:.1f}\n")
            f.write(f"  Path Cost (Mixed): {avg_cost:.2f}\n")
            f.write(f"    (g + h + penalties)\n")
            f.write(f"  Path Length (Physical): {avg_length_m:.2f} m\n")
            f.write(f"  Path Steps (Average): {avg_steps:.1f}\n\n")

            f.write("Safety Analysis:\n")
            f.write(f"  Safe Plans: {len(safe_cases)} ({100-unsafe_rate:.1f}%)\n")
            f.write(f"  Unsafe Plans: {len(unsafe_cases)} ({unsafe_rate:.1f}%)\n")
            f.write(f"    (Plans that passed circle check but failed strict polygon check)\n")
            if unsafe_cases:
                f.write(f"  Collisions in Unsafe Plans:\n")
                f.write(f"    Average: {avg_collisions:.1f} points\n")
                f.write(f"    Max:     {max_collisions} points\n")

        # 异常统计
        error_cases = [d for d in data if d['collision_count'] == -2]
        if error_cases:
            f.write(f"\nErrors/Exceptions: {len(error_cases)} cases\n")

        f.write("\n" + "=" * 60 + "\n")
        f.write("End of Report\n")
        f.write("=" * 60 + "\n")


def run_experiment():
    """主实验函数"""
    # --- 2. 实验参数配置 ---
    NUM_EXPERIMENTS = 100  # 批量测试：100次实验
    START_SEED = 1

    # 固定的起终点
    START_POSE = (10.0, 10.0, math.radians(0.0))
    GOAL_POSE = (40.0, 40.0, math.radians(90.0))

    # 输出路径（带时间戳）
    RESULTS_DIR = os.path.join(current_dir, 'results')
    ensure_dir(RESULTS_DIR)

    # 生成时间戳
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    CSV_FILE = os.path.join(RESULTS_DIR, f'batch_test_log_{timestamp}.csv')
    SUMMARY_FILE = os.path.join(RESULTS_DIR, f'summary_report_{timestamp}.txt')

    # 初始化配置（保持所有实验一致）
    h_config = HybridAStarConfig()
    h_config.move_step_grid = 2.0
    h_config.heuristic_weight = 5.0

    # [显式指定] 碰撞检测方法
    # 即使 config.py 默认值是 FOOTPRINT，显式设置可以确保实验的可复现性和明确性
    h_config.collision_method = CollisionMethod.FOOTPRINT

    v_config = VehicleConfig()

    # 准备 CSV 表头
    headers = [
        "Experiment_ID",
        "Seed",
        "Success",
        "Planning_Time_ms",
        "Nodes_Expanded",
        "Path_Cost",          # 混合代价（g + h + 惩罚）
        "Path_Length_m",       # 物理长度（米）
        "Steps_Count",         # 路径点数
        "Strict_Collision_Count"
    ]

    print("=" * 70)
    print("Hybrid A* Batch Experiment")
    print("=" * 70)
    print(f"Total Experiments: {NUM_EXPERIMENTS}")
    print(f"Start Pose: ({START_POSE[0]}, {START_POSE[1]}, {math.degrees(START_POSE[2]):.1f}°)")
    print(f"Goal Pose:  ({GOAL_POSE[0]}, {GOAL_POSE[1]}, {math.degrees(GOAL_POSE[2]):.1f}°)")
    print(f"Results Directory: {RESULTS_DIR}")
    print("=" * 70)
    print()

    experiment_data = []

    # --- 3. 循环执行实验 ---
    with open(CSV_FILE, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(headers)

        for i in range(NUM_EXPERIMENTS):
            seed = START_SEED + i
            exp_id = i + 1

            # 执行单次实验
            result = run_single_experiment(
                exp_id, seed, START_POSE, GOAL_POSE, h_config, v_config
            )

            # 准备 CSV 行数据
            row = [
                result['exp_id'],
                result['seed'],
                result['success'],
                f"{result['execution_time_ms']:.2f}",
                result['nodes_expanded'],
                f"{result['path_cost']:.2f}",
                f"{result['path_length_m']:.2f}",
                result['steps_count'],
                result['collision_count']
            ]

            # 实时写入 CSV（防止程序崩溃导致数据丢失）
            writer.writerow(row)
            f.flush()

            # 保存到内存
            experiment_data.append(result)

            # 实时输出进度
            status = "SUCCESS" if result['success'] else "FAILED"
            collision_str = f" | Collisions: {result['collision_count']}" if result['success'] else ""
            print(f"[Exp {exp_id:03d}/{NUM_EXPERIMENTS} | Seed {seed}] "
                  f"{status} | Time: {result['execution_time_ms']:.1f}ms | "
                  f"Nodes: {result['nodes_expanded']}{collision_str}")

    # --- 4. 生成统计报告 ---
    print()
    print("=" * 70)
    print("Generating summary report...")
    generate_summary(experiment_data, SUMMARY_FILE, h_config, NUM_EXPERIMENTS,
                     START_POSE, GOAL_POSE)
    print("=" * 70)
    print()
    print(f"Experiment completed!")
    print(f"Results saved to: {RESULTS_DIR}")
    print(f"  - Raw data: {os.path.basename(CSV_FILE)}")
    print(f"  - Summary:  {os.path.basename(SUMMARY_FILE)}")
    print()


if __name__ == "__main__":
    run_experiment()
