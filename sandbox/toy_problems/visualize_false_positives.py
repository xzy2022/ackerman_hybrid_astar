"""
可视化误报点：展示为什么栅格相交判定更严格
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import matplotlib.path as mpath
from src.config import VehicleConfig, HybridAStarConfig
from src.grid_map import GridMap

# 配置中文
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# 初始化
h_cfg = HybridAStarConfig()
h_cfg.xy_resolution = 0.1
v_cfg = VehicleConfig()

# 创建地图
grid_map = GridMap(h_cfg, v_cfg)
grid_map.width_m = 30.0
grid_map.height_m = 30.0
grid_map.width_idx = int(30.0 / h_cfg.xy_resolution)
grid_map.height_idx = int(30.0 / h_cfg.xy_resolution)
grid_map.obstacle_map = np.zeros((grid_map.width_idx, grid_map.height_idx), dtype=bool)

# 设置车辆位姿
vehicle_x, vehicle_y, vehicle_yaw = 10.0, 10.0, np.radians(30)

# 制造雷区
def create_minefield_map(gm, vx, vy, coverage_m=8.0):
    min_x_idx = max(0, int((vx - coverage_m) / gm.config.xy_resolution))
    max_x_idx = min(gm.width_idx - 1, int((vx + coverage_m) / gm.config.xy_resolution))
    min_y_idx = max(0, int((vy - coverage_m) / gm.config.xy_resolution))
    max_y_idx = min(gm.height_idx - 1, int((vy + coverage_m) / gm.config.xy_resolution))

    for ix in range(min_x_idx, max_x_idx + 1):
        for iy in range(min_y_idx, max_y_idx + 1):
            gm.obstacle_map[ix][iy] = True

create_minefield_map(grid_map, vehicle_x, vehicle_y, coverage_m=8.0)

# 调用碰撞检测
collided_obstacles = grid_map.check_strict_path_collision([vehicle_x], [vehicle_y], [vehicle_yaw])

# 构建车辆多边形
outline = v_cfg.vehicle_outline.copy()
rot = np.array([
    [np.cos(vehicle_yaw), np.sin(vehicle_yaw)],
    [-np.sin(vehicle_yaw), np.cos(vehicle_yaw)]
])
rotated_outline = (outline.T.dot(rot)).T
rotated_outline[0, :] += vehicle_x
rotated_outline[1, :] += vehicle_y

# 建立独立裁判
ground_truth_polygon = mpath.Path(rotated_outline.T)

# 找出误报点
detected_set = set((round(p[0], 2), round(p[1], 2)) for p in collided_obstacles)
false_positives = []
true_positives = []

# 只检查车辆包围盒附近的点
outline_min_x = np.min(rotated_outline[0, :])
outline_max_x = np.max(rotated_outline[0, :])
outline_min_y = np.min(rotated_outline[1, :])
outline_max_y = np.max(rotated_outline[1, :])

search_margin = 0.5
min_x_search = int(max(0, (outline_min_x - search_margin) / h_cfg.xy_resolution))
max_x_search = int(min(grid_map.width_idx, (outline_max_x + search_margin) / h_cfg.xy_resolution + 1))
min_y_search = int(max(0, (outline_min_y - search_margin) / h_cfg.xy_resolution))
max_y_search = int(min(grid_map.height_idx, (outline_max_y + search_margin) / h_cfg.xy_resolution + 1))

for ix in range(min_x_search, max_x_search):
    for iy in range(min_y_search, max_y_search):
        if grid_map.obstacle_map[ix][iy]:
            px, py = grid_map.get_pos_from_index(ix, iy)
            is_inside_truth = ground_truth_polygon.contains_point((px, py))
            is_detected = (round(px, 2), round(py, 2)) in detected_set

            if is_detected and not is_inside_truth:
                false_positives.append((px, py))
            elif is_detected and is_inside_truth:
                true_positives.append((px, py))

print(f"找到 {len(false_positives)} 个误报点")

# 可视化
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_title("误报点可视化：栅格相交判定 vs 栅格中心判定", fontsize=14, fontweight='bold')
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True, alpha=0.3)
ax.axis("equal")

# 绘制车辆轮廓
ax.plot(rotated_outline[0, :], rotated_outline[1, :], 'k-', linewidth=3, label='车辆轮廓')

# 绘制真值内部点（绿色）
if true_positives:
    tx, ty = zip(*true_positives)
    ax.scatter(tx, ty, c='lime', s=20, alpha=0.5, label=f'真值内部点 ({len(true_positives)})', zorder=2)

# 绘制误报点（红色，强调）
if false_positives:
    fx, fy = zip(*false_positives)
    ax.scatter(fx, fy, c='red', s=80, marker='X', label=f'误报点 ({len(false_positives)}) - 栅格中心在车外但栅格与车身相交', zorder=4, edgecolors='darkred', linewidths=1)

# 标记车辆中心
ax.plot(vehicle_x, vehicle_y, 'bo', markersize=10, label='车辆中心', zorder=5)

# 局部放大视图
view_range = 3.0
ax.set_xlim(vehicle_x - view_range, vehicle_x + view_range)
ax.set_ylim(vehicle_y - view_range, vehicle_y + view_range)

ax.legend(loc='upper right')
plt.tight_layout()
# plt.savefig('sandbox/toy_problems/false_positives_visualization.png', dpi=300, bbox_inches='tight')
# print(f"可视化已保存到: sandbox/toy_problems/false_positives_visualization.png")
plt.show()
