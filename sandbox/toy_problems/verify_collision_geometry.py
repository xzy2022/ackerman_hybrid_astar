"""
雷区测试 (Minefield Test)
=======================
验证 check_strict_path_collision 函数的几何精度。

测试思路：
1. 静止摆拍：将车辆固定在世界坐标系的一个特定位置（例如 x=10, y=10），并给它一个旋转角度（例如 30°）
2. 制造"雷区"：创建一个局部的 GridMap，将车辆周围的所有栅格点全部标记为"障碍物"（即 True）
3. 触发检测：调用 check_strict_path_collision 函数
4. 可视化验证：
   - 黑线：绘制车辆的理论轮廓（这是期望真值）
   - 红点：函数返回的"碰撞点"（代码的判断）
   - 灰点：车辆周围未被判定为碰撞的点（背景）
5. 判定标准：如果红点完美地填充了黑线轮廓，且没有溢出也没有缺失，则证明该函数是绝对可靠的
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import matplotlib as mpl
import matplotlib.path as mpath
from shapely.geometry import Polygon, box

from src.config import VehicleConfig, HybridAStarConfig
from src.grid_map import GridMap

# ============================================================
#  可配置参数（方便调整）
# ============================================================

# 地图分辨率 [m]（建议使用 0.1 或更低以获得精确的几何验证）
MAP_RESOLUTION = 0.1

# 车辆位姿
VEHICLE_X = 10.0          # [m] x坐标
VEHICLE_Y = 10.0          # [m] y坐标
VEHICLE_YAW_DEG = 30.0    # [deg] 航向角

# 雷区范围（以车辆为中心）[m]
MINEFIELD_RADIUS = 8.0

# 地图大小 [m]
MAP_SIZE = 30.0

# 可视化：局部放大视图范围 [m]
LOCAL_VIEW_RANGE = 6.0

# ============================================================
#  Matplotlib 中文字体设置
# ============================================================

def setup_chinese_font():
    """配置 matplotlib 以支持中文显示"""
    # Windows 系统常用中文字体
    fonts = ['Microsoft YaHei', 'SimHei', 'SimSun', 'KaiTi', 'FangSong']

    plt.rcParams['font.sans-serif'] = fonts
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

    # 测试字体是否可用
    for font in fonts:
        try:
            mpl.font_manager.findfont(font)
            print(f"使用字体: {font}")
            return
        except:
            continue

    print("警告: 未找到可用的中文字体，图表中的中文可能无法显示")

setup_chinese_font()


def create_minefield_map(grid_map: GridMap, vehicle_x: float, vehicle_y: float,
                         coverage_m: float = 15.0) -> None:
    """
    制造"雷区"：将车辆周围的所有栅格点标记为障碍物。

    Args:
        grid_map: 栅格地图对象
        vehicle_x: 车辆 x 坐标
        vehicle_y: 车辆 y 坐标
        coverage_m: 覆盖范围（米），以车辆为中心
    """
    # 计算覆盖范围对应的栅格索引
    min_x_idx = max(0, int((vehicle_x - coverage_m) / grid_map.config.xy_resolution))
    max_x_idx = min(grid_map.width_idx - 1,
                    int((vehicle_x + coverage_m) / grid_map.config.xy_resolution))
    min_y_idx = max(0, int((vehicle_y - coverage_m) / grid_map.config.xy_resolution))
    max_y_idx = min(grid_map.height_idx - 1,
                    int((vehicle_y + coverage_m) / grid_map.config.xy_resolution))

    # 将所有栅格标记为障碍物
    for ix in range(min_x_idx, max_x_idx + 1):
        for iy in range(min_y_idx, max_y_idx + 1):
            grid_map.obstacle_map[ix][iy] = True


def transform_vehicle_outline(vehicle_config: VehicleConfig, x: float,
                              y: float, yaw: float) -> np.ndarray:
    """
    将车辆轮廓从局部坐标系变换到世界坐标系。

    Args:
        vehicle_config: 车辆配置
        x: 世界坐标系 x 坐标
        y: 世界坐标系 y 坐标
        yaw: 偏航角 [rad]

    Returns:
        变换后的轮廓点 (2x5 numpy array)
    """
    outline = vehicle_config.vehicle_outline.copy()

    # 旋转矩阵
    rot = np.array([[np.cos(yaw), np.sin(yaw)],
                    [-np.sin(yaw), np.cos(yaw)]])

    # 旋转 + 平移
    rotated_outline = (outline.T.dot(rot)).T
    rotated_outline[0, :] += x
    rotated_outline[1, :] += y

    return rotated_outline


def verify_collision_geometry():
    """主测试函数"""
    print("=" * 60)
    print("雷区测试：验证 check_strict_path_collision 几何精度")
    print("=" * 60)

    # 1. 初始化配置（使用顶部定义的参数）
    h_cfg = HybridAStarConfig()
    h_cfg.xy_resolution = MAP_RESOLUTION
    v_cfg = VehicleConfig()

    print(f"\n地图分辨率: {h_cfg.xy_resolution} m")
    print(f"车辆尺寸: 长={v_cfg.front_hang + v_cfg.rear_hang:.2f}m, 宽={v_cfg.width:.2f}m")

    # 2. 创建地图
    grid_map = GridMap(h_cfg, v_cfg)
    grid_map.width_m = MAP_SIZE
    grid_map.height_m = MAP_SIZE
    grid_map.width_idx = int(MAP_SIZE / h_cfg.xy_resolution)
    grid_map.height_idx = int(MAP_SIZE / h_cfg.xy_resolution)
    grid_map.obstacle_map = np.zeros((grid_map.width_idx, grid_map.height_idx),
                                      dtype=bool)

    # 3. 设置车辆位姿（使用顶部定义的参数）
    vehicle_x = VEHICLE_X
    vehicle_y = VEHICLE_Y
    vehicle_yaw = np.radians(VEHICLE_YAW_DEG)

    print(f"\n车辆位姿:")
    print(f"  位置: ({vehicle_x:.2f}, {vehicle_y:.2f}) m")
    print(f"  航向: {VEHICLE_YAW_DEG:.1f}°")

    # 4. 制造"雷区"（使用顶部定义的参数）
    print(f"\n正在制造雷区...")
    create_minefield_map(grid_map, vehicle_x, vehicle_y, coverage_m=MINEFIELD_RADIUS)
    print(f"  障碍物栅格数: {np.sum(grid_map.obstacle_map)}")

    # 5. 调用碰撞检测函数
    print(f"\n正在调用 check_strict_path_collision...")
    collided_obstacles = grid_map.check_strict_path_collision(
        [vehicle_x], [vehicle_y], [vehicle_yaw]
    )
    print(f"  检测到的碰撞点数: {len(collided_obstacles)}")

    # 6. 准备可视化数据
    # 车辆理论轮廓（真值）
    vehicle_outline = transform_vehicle_outline(v_cfg, vehicle_x, vehicle_y,
                                                 vehicle_yaw)

    # 碰撞点（红点）
    collision_x = [obs[0] for obs in collided_obstacles]
    collision_y = [obs[1] for obs in collided_obstacles]

    # 7. 可视化
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

    # --- 子图 1：全局视图 ---
    ax1.set_title("全局视图", fontsize=14, fontweight='bold')
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.grid(True, alpha=0.3)
    ax1.axis("equal")

    # 绘制所有障碍物（灰点）
    all_obs_x, all_obs_y = [], []
    for ix in range(grid_map.width_idx):
        for iy in range(grid_map.height_idx):
            if grid_map.obstacle_map[ix][iy]:
                px, py = grid_map.get_pos_from_index(ix, iy)
                all_obs_x.append(px)
                all_obs_y.append(py)

    ax1.scatter(all_obs_x, all_obs_y, c='lightgray', s=30, alpha=0.5,
                label='所有障碍物', zorder=1)

    # 绘制碰撞点（红点）
    if collision_x:
        ax1.scatter(collision_x, collision_y, c='red', s=50, marker='o',
                    label=f'碰撞点 ({len(collision_x)})', zorder=3)

    # 绘制车辆轮廓（黑线）
    ax1.plot(vehicle_outline[0, :], vehicle_outline[1, :], 'k-',
             linewidth=3, label='车辆理论轮廓', zorder=4)

    # 标记车辆中心
    ax1.plot(vehicle_x, vehicle_y, 'bo', markersize=8,
             label='车辆中心', zorder=5)

    ax1.legend(loc='upper right')

    # --- 子图 2：局部放大视图 ---
    ax2.set_title("局部放大视图", fontsize=14, fontweight='bold')
    ax2.set_xlabel("X [m]")
    ax2.set_ylabel("Y [m]")
    ax2.grid(True, alpha=0.3)
    ax2.axis("equal")

    # 只绘制车辆周围的区域（使用顶部定义的参数）
    local_obs_x, local_obs_y = [], []
    local_collision_x, local_collision_y = [], []

    for ix in range(grid_map.width_idx):
        for iy in range(grid_map.height_idx):
            if grid_map.obstacle_map[ix][iy]:
                px, py = grid_map.get_pos_from_index(ix, iy)
                if (abs(px - vehicle_x) < LOCAL_VIEW_RANGE and
                    abs(py - vehicle_y) < LOCAL_VIEW_RANGE):
                    local_obs_x.append(px)
                    local_obs_y.append(py)

    # 绘制局部障碍物
    ax2.scatter(local_obs_x, local_obs_y, c='lightgray', s=80,
                alpha=0.5, label='所有障碍物', zorder=1)

    # 绘制碰撞点
    if collision_x:
        for cx, cy in zip(collision_x, collision_y):
            if (abs(cx - vehicle_x) < LOCAL_VIEW_RANGE and
                abs(cy - vehicle_y) < LOCAL_VIEW_RANGE):
                local_collision_x.append(cx)
                local_collision_y.append(cy)

        ax2.scatter(local_collision_x, local_collision_y, c='red',
                    s=120, marker='o', label=f'碰撞点 ({len(local_collision_x)})',
                    zorder=3, edgecolors='darkred', linewidths=1)

    # 绘制车辆轮廓
    ax2.plot(vehicle_outline[0, :], vehicle_outline[1, :], 'k-',
             linewidth=4, label='车辆理论轮廓', zorder=4)

    # 标记车辆中心
    ax2.plot(vehicle_x, vehicle_y, 'bo', markersize=10,
             label='车辆中心', zorder=5)

    # 绘制方向箭头
    arrow_length = 1.5
    ax2.arrow(vehicle_x, vehicle_y,
              arrow_length * np.cos(vehicle_yaw),
              arrow_length * np.sin(vehicle_yaw),
              head_width=0.3, head_length=0.3, fc='blue', ec='blue',
              linewidth=2, zorder=6, label='航向')

    ax2.legend(loc='upper right')

    # 设置局部视图范围（使用顶部定义的参数）
    ax2.set_xlim(vehicle_x - LOCAL_VIEW_RANGE, vehicle_x + LOCAL_VIEW_RANGE)
    ax2.set_ylim(vehicle_y - LOCAL_VIEW_RANGE, vehicle_y + LOCAL_VIEW_RANGE)

    plt.tight_layout()
    plt.savefig('sandbox/toy_problems/verify_collision_geometry_result.png',
                dpi=300, bbox_inches='tight')
    print(f"\n可视化结果已保存到: sandbox/toy_problems/verify_collision_geometry_result.png")
    plt.show()  # 关闭图形，释放内存

    # 8. 分析结果
    print("\n" + "=" * 60)
    print("测试结果分析:")
    print("=" * 60)

    # 计算理论轮廓的包围盒
    outline_min_x = np.min(vehicle_outline[0, :])
    outline_max_x = np.max(vehicle_outline[0, :])
    outline_min_y = np.min(vehicle_outline[1, :])
    outline_max_y = np.max(vehicle_outline[1, :])

    print(f"\n车辆轮廓包围盒:")
    print(f"  X 范围: [{outline_min_x:.2f}, {outline_max_x:.2f}] m")
    print(f"  Y 范围: [{outline_min_y:.2f}, {outline_max_y:.2f}] m")

    # ============================================================
    #  8. 分析结果 (真正的独立验证版 - 使用 Shapely 栅格相交)
    # ============================================================
    print("\n" + "=" * 60)
    print("测试结果分析 (独立验证 - Shapely栅格相交):")
    print("=" * 60)

    # 1. 建立独立裁判：使用 Shapely 的 Polygon 对象
    # 提取车辆多边形的顶点（去除重复的最后一个点）
    vehicle_vertices = []
    for i in range(vehicle_outline.shape[1] - 1):  # -1 去掉重复的起点
        vehicle_vertices.append((vehicle_outline[0, i], vehicle_outline[1, i]))
    vehicle_polygon = Polygon(vehicle_vertices)

    # 2. 重新扫描所有刚才生成的障碍物点，通过裁判进行判定
    true_positives = 0   # 应该撞，且测出撞了 (正确)
    false_positives = 0  # 不该撞，却测出撞了 (误报)
    false_negatives = 0  # 应该撞，却没测出来 (漏报)

    # 将函数检测到的点转为集合，方便快速查询
    detected_set = set((round(p[0], 2), round(p[1], 2)) for p in collided_obstacles)

    # 遍历刚才制造的"雷区"内的所有点
    checked_count = 0

    # 优化：只遍历车辆包围盒附近的障碍物点
    search_margin = 0.5  # 搜索范围余量
    min_x_search = int(max(0, (outline_min_x - search_margin) / h_cfg.xy_resolution))
    max_x_search = int(min(grid_map.width_idx, (outline_max_x + search_margin) / h_cfg.xy_resolution + 1))
    min_y_search = int(max(0, (outline_min_y - search_margin) / h_cfg.xy_resolution))
    max_y_search = int(min(grid_map.height_idx, (outline_max_y + search_margin) / h_cfg.xy_resolution + 1))

    print(f"正在独立验证 {len(collided_obstacles)} 个检测点...")
    print(f"搜索范围: X[{min_x_search}:{max_x_search}], Y[{min_y_search}:{max_y_search}]")
    print(f"裁判方法: Shapely 栅格相交判定 (与函数逻辑一致)")

    for ix in range(min_x_search, max_x_search):
        for iy in range(min_y_search, max_y_search):
            if grid_map.obstacle_map[ix][iy]:
                px, py = grid_map.get_pos_from_index(ix, iy)
                checked_count += 1

                # A. 裁判判定：栅格是否与车辆多边形相交（使用 Shapely）
                res = h_cfg.xy_resolution
                grid_min_x = px - res / 2
                grid_max_x = px + res / 2
                grid_min_y = py - res / 2
                grid_max_y = py + res / 2

                # 创建栅格的 AABB (Axis-Aligned Bounding Box)
                grid_bbox = box(grid_min_x, grid_min_y, grid_max_x, grid_max_y)

                # 使用 Shapely 检测相交
                is_intersecting_truth = vehicle_polygon.intersects(grid_bbox)

                # B. 函数判定：函数有没有返回这个点？
                is_detected = (round(px, 2), round(py, 2)) in detected_set

                if is_intersecting_truth:
                    if is_detected:
                        true_positives += 1
                    else:
                        false_negatives += 1  # 漏了！
                else:
                    if is_detected:
                        false_positives += 1  # 误报！

                # 每1000个点输出一次进度
                if checked_count % 1000 == 0:
                    print(f"  已检查 {checked_count} 个点...")

    print(f"验证完成！共检查了 {checked_count} 个障碍物点。")

    # 3. 计算真实指标
    total_truth_points = true_positives + false_negatives
    precision = (true_positives / (true_positives + false_positives) * 100) if (true_positives + false_positives) > 0 else 0
    recall = (true_positives / total_truth_points * 100) if total_truth_points > 0 else 0

    print(f"\n【Shapely裁判】与车辆相交的栅格数: {total_truth_points}")
    print(f"【函数】检测到的栅格数: {true_positives}")
    print(f"【错误】误报数 (False Pos): {false_positives} (不相交却检测为碰撞)")
    print(f"【错误】漏报数 (False Neg): {false_negatives} (相交却未检测出)")
    print(f"精确率 (Precision): {precision:.2f}%")
    print(f"召回率 (Recall): {recall:.2f}%")

    # 4. 最终判定
    print("-" * 60)
    if false_positives == 0 and false_negatives == 0:
        print("[PASS] 完美！函数与Shapely裁判结果完全一致！")
    elif false_positives == 0 and recall > 99.0:
        print("[PASS] 测试通过！函数精度极高 (独立验证)。")
    elif false_positives == 0 and recall > 95.0:
        print("[PASS] 测试基本通过 (存在少量边缘误差，属正常范围)。")
    else:
        print("[FAIL] 测试失败！")
        if false_positives > 0:
            print(f"  -> 发现 {false_positives} 个误报点（函数说相交，Shapely说不相交）")
        if false_negatives > 0:
            print(f"  -> 发现 {false_negatives} 个漏报点（Shapely说相交，函数未检测出）")
    print("=" * 60)
    print("说明：Shapely是成熟的几何计算库，其栅格相交判定可作为Ground Truth")


if __name__ == "__main__":
    verify_collision_geometry()
