"""
点模型功能测试脚本
验证 GridMap 和 Visualizer 对点模型的支持
"""

import sys
sys.path.insert(0, 'src')

import math
import numpy as np
from src.config import PointConfig, VehicleConfig, HybridAStarConfig
from src.grid_map import GridMap
from src.visualizer import Visualizer

def test_point_collision_detection():
    """测试点模型碰撞检测"""
    print("=" * 60)
    print("测试1: 点模型碰撞检测")
    print("=" * 60)

    # 创建点模型配置
    point_config = PointConfig(radius=0.5)
    print(f"[OK] 点模型配置: radius={point_config.radius}m")

    # 创建地图配置
    map_config = HybridAStarConfig()
    map_config.xy_resolution = 0.5

    # 创建地图
    grid_map = GridMap(map_config, point_config)

    # 生成简单测试地图（20x20米）
    grid_map.generate_random_map(width_m=20.0, height_m=20.0, obstacle_num=10)
    print(f"[OK] 地图生成: {grid_map.width_idx}x{grid_map.height_idx} 栅格")

    # 测试碰撞检测
    # 测试点1：原点（假设没有障碍物，因为我们没有清理）
    collision1 = grid_map.check_collision(10.0, 10.0, 0.0)  # yaw参数被忽略
    print(f"[OK] 碰撞检测 (10.0, 10.0): {collision1}")

    # 测试点2：不同yaw（结果应该相同）
    collision2 = grid_map.check_collision(10.0, 10.0, math.pi/2)
    collision3 = grid_map.check_collision(10.0, 10.0, math.pi)
    print(f"[OK] 碰撞检测一致性 (yaw=0, 90, 180): {collision1} == {collision2} == {collision3}")

    assert collision2 == collision1, "点模型不应该受yaw影响"
    assert collision3 == collision1, "点模型不应该受yaw影响"

    print("[OK] 测试通过！\n")

def test_ackermann_vs_point():
    """对比阿克曼模型和点模型"""
    print("=" * 60)
    print("测试2: 阿克曼模型 vs 点模型对比")
    print("=" * 60)

    # 创建配置
    point_config = PointConfig(radius=0.5)
    ackermann_config = VehicleConfig()  # 默认阿克曼车辆

    map_config = HybridAStarConfig()
    map_config.xy_resolution = 0.5

    # 创建两个地图（相同障碍物布局）
    point_map = GridMap(map_config, point_config)
    ackermann_map = GridMap(map_config, ackermann_config)

    # 生成相同地图
    np.random.seed(42)
    point_map.generate_random_map(width_m=20.0, height_m=20.0, obstacle_num=5)

    np.random.seed(42)
    ackermann_map.generate_random_map(width_m=20.0, height_m=20.0, obstacle_num=5)

    # 测试同一个位置
    test_x, test_y, test_yaw = 10.0, 10.0, 0.0

    point_collision = point_map.check_collision(test_x, test_y, test_yaw)
    ackermann_collision = ackermann_map.check_collision(test_x, test_y, test_yaw)

    print(f"[OK] 点模型碰撞: {point_collision}")
    print(f"[OK] 阿克曼模型碰撞: {ackermann_collision}")
    print(f"[INFO] 点模型半径: {point_config.radius}m")
    print(f"[INFO] 阿克曼模型宽度: {ackermann_config.width}m, 碰撞半径: {ackermann_config.collision_radius:.2f}m")

    # 注意：结果可能不同，因为阿克曼模型更大
    print("[OK] 对比测试完成！\n")

def test_visualizer_point_model():
    """测试 Visualizer 点模型绘制"""
    print("=" * 60)
    print("测试3: Visualizer 点模型绘制")
    print("=" * 60)

    # 创建配置
    point_config = PointConfig(radius=0.8)

    # 创建 Visualizer
    viz = Visualizer(point_config)
    print(f"[OK] Visualizer 创建成功，模型类型: {viz.robot_config.model_type}")

    # 测试绘制方法（不实际显示，只检查方法存在）
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))

    # 调用 _plot_car_body 绘制点模型
    artists = viz._plot_car_body(ax, x=10.0, y=10.0, yaw=math.radians(45), color='blue', alpha=0.8)

    print(f"[OK] 绘制对象数量: {len(artists)}")
    print(f"[INFO] 绘制对象类型: {[type(a).__name__ for a in artists]}")

    # 点模型应该返回2个对象：Circle + Arrow
    assert len(artists) == 2, "点模型应该绘制圆和箭头"

    plt.close(fig)
    print("[OK] 测试通过！\n")

def test_visualizer_ackermann_model():
    """测试 Visualizer 阿克曼模型绘制（对比）"""
    print("=" * 60)
    print("测试4: Visualizer 阿克曼模型绘制（对比）")
    print("=" * 60)

    # 创建阿克曼配置
    ackermann_config = VehicleConfig()

    # 创建 Visualizer
    viz = Visualizer(ackermann_config)
    print(f"[OK] Visualizer 创建成功，模型类型: {viz.robot_config.model_type}")

    # 测试绘制方法
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))

    # 调用 _plot_car_body 绘制阿克曼模型
    artists = viz._plot_car_body(ax, x=10.0, y=10.0, yaw=math.radians(45), color='red', alpha=0.8)

    print(f"[OK] 绘制对象数量: {len(artists)}")
    print(f"[INFO] 绘制对象类型: {[type(a).__name__ for a in artists]}")

    # 阿克曼模型应该返回更多对象：outline + arrow + 多个collision circles
    assert len(artists) > 2, "阿克曼模型应该绘制轮廓、箭头和碰撞圆"

    plt.close(fig)
    print("[OK] 测试通过！\n")

def test_performance_comparison():
    """性能对比：点模型 vs 阿克曼模型"""
    print("=" * 60)
    print("测试5: 性能对比（碰撞检测速度）")
    print("=" * 60)

    import time

    # 创建配置
    point_config = PointConfig(radius=0.5)
    ackermann_config = VehicleConfig()

    map_config = HybridAStarConfig()
    map_config.collision_method = "circle"  # 使用圆形检测以便对比
    map_config.xy_resolution = 0.5

    # 创建地图
    point_map = GridMap(map_config, point_config)
    ackermann_map = GridMap(map_config, ackermann_config)

    np.random.seed(42)
    point_map.generate_random_map(width_m=50.0, height_m=50.0, obstacle_num=100)

    np.random.seed(42)
    ackermann_map.generate_random_map(width_m=50.0, height_m=50.0, obstacle_num=100)

    # 生成随机测试点
    num_tests = 1000
    test_points = [(np.random.uniform(5, 45), np.random.uniform(5, 45), 0.0) for _ in range(num_tests)]

    # 测试点模型性能
    start_time = time.time()
    for x, y, yaw in test_points:
        point_map.check_collision(x, y, yaw)
    point_time = time.time() - start_time

    # 测试阿克曼模型性能
    start_time = time.time()
    for x, y, yaw in test_points:
        ackermann_map.check_collision(x, y, yaw)
    ackermann_time = time.time() - start_time

    print(f"[INFO] 点模型平均检测时间: {point_time/num_tests*1000:.4f} ms")
    print(f"[INFO] 阿克曼模型平均检测时间: {ackermann_time/num_tests*1000:.4f} ms")
    print(f"[INFO] 性能提升: {ackermann_time/point_time:.2f}x")

    print("[OK] 性能测试完成！\n")

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("点模型功能测试套件")
    print("=" * 60 + "\n")

    try:
        test_point_collision_detection()
        test_ackermann_vs_point()
        test_visualizer_point_model()
        test_visualizer_ackermann_model()
        test_performance_comparison()

        print("=" * 60)
        print("[SUCCESS] 所有测试通过！点模型功能正常！")
        print("=" * 60)

    except Exception as e:
        print(f"\n[FAILED] 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
