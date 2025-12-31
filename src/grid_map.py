import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mpath
from typing import Tuple, Optional, List
from shapely.geometry import Polygon, box

# 导入接口与配置
from src.interfaces import BaseMap
from src.config import HybridAStarConfig, VehicleConfig

class GridMap(BaseMap):
    """
    基于栅格的地图实现 (Grid Map Implementation).
    
    修正版 (SSOT): 统一了物理坐标与栅格索引的转换逻辑。
    - 索引 (i, j) 对应的物理范围是 [i*res, (i+1)*res)
    - 物理中心位于 (i+0.5)*res
    """

    def __init__(self, config: HybridAStarConfig, vehicle_config: VehicleConfig):
        """
        初始化栅格地图。
        
        Args:
            config: 包含分辨率等地图参数
            vehicle_config: 包含车辆尺寸，用于内部的碰撞检测计算
        """
        self.config = config
        self.vehicle_config = vehicle_config
        
        # 地图尺寸 (索引单位)
        self.width_idx = 0
        self.height_idx = 0
        
        # 地图物理边界 (米)
        self.width_m = 0.0
        self.height_m = 0.0
        
        # 障碍物数据 (W x H)
        # 初始化为空，需要调用 generate_random_map 或 load_from_image 填充
        self.obstacle_map: Optional[np.ndarray] = None

    # =========================================================
    #  核心转换逻辑 (Single Source of Truth)
    #  所有涉及坐标转换的地方都必须调用这两个方法
    # =========================================================
    
    def get_index_from_pos(self, x: float, y: float) -> Tuple[int, int]:
        """
        世界坐标 -> 栅格索引 (向下取整)
        Ex: res=1.0, pos=0.9 -> index=0; pos=1.1 -> index=1
        """
        # 使用 floor 确保正数区间内的逻辑一致性
        x_idx = int(math.floor(x / self.config.xy_resolution))
        y_idx = int(math.floor(y / self.config.xy_resolution))
        return x_idx, y_idx

    def get_pos_from_index(self, x_idx: int, y_idx: int) -> Tuple[float, float]:
        """
        栅格索引 -> 世界坐标中心
        Ex: res=1.0, index=0 -> pos=0.5
        用于：可视化画点、A* 节点中心坐标计算、碰撞检测精确距离计算
        """
        x = (x_idx + 0.5) * self.config.xy_resolution
        y = (y_idx + 0.5) * self.config.xy_resolution
        return x, y

    # =========================================================
    #  业务逻辑
    # =========================================================

    def check_collision(self, x: float, y: float, yaw: float) -> bool:
        """
        [实现 BaseMap 接口]
        检查车辆在特定位姿下是否与障碍物碰撞。

        采用 "多圆覆盖模型" (Circle Approximation) 进行快速检测。
        改进版：使用动态搜索半径，覆盖整个物理圆范围。
        """
        if self.obstacle_map is None:
            return True # 地图未初始化视为不可通行

        # 遍历车身上的每一个碰撞检测圆 (由 VehicleConfig 定义)
        for offset in self.vehicle_config.collision_offsets:
            # 1. 计算圆心在世界坐标系的位置
            cx = x + offset * math.cos(yaw)
            cy = y + offset * math.sin(yaw)

            # 2. 转换为栅格索引 (使用统一接口)
            cx_idx, cy_idx = self.get_index_from_pos(cx, cy)

            # 3. 越界检查 (Out of bounds check)
            if (cx_idx < 0 or cx_idx >= self.width_idx or
                cy_idx < 0 or cy_idx >= self.height_idx):
                return True

            # 4. [核心改进] 动态计算搜索半径
            # 确保搜索范围覆盖整个物理圆
            search_radius_idx = math.ceil(self.vehicle_config.collision_radius / self.config.xy_resolution)

            # 5. 遍历覆盖圆的所有潜在栅格
            for i in range(-search_radius_idx, search_radius_idx + 1):
                for j in range(-search_radius_idx, search_radius_idx + 1):
                    nx, ny = cx_idx + i, cy_idx + j

                    # 边界检查
                    if 0 <= nx < self.width_idx and 0 <= ny < self.height_idx:
                        if self.obstacle_map[nx][ny]:
                            # 障碍物中心坐标
                            obs_x, obs_y = self.get_pos_from_index(nx, ny)

                            # 计算距离 (圆心到障碍物中心)
                            dist = math.hypot(cx - obs_x, cy - obs_y)

                            # 判定碰撞
                            # 考虑到栅格是对角线覆盖，用 radius + resolution/2 稍微保守一点
                            # 但这里用纯半径判断已经足够精确（因为我们遍历了所有覆盖的栅格）
                            if dist <= self.vehicle_config.collision_radius:
                                return True

        return False # 通过所有检查，无碰撞

    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        [实现 BaseMap 接口] 获取物理边界
        Returns: (min_x, max_x, min_y, max_y)
        """
        return (0.0, self.width_m, 0.0, self.height_m)

    def generate_random_map(self, width_m: float, height_m: float, obstacle_num: int):
        """生成测试用的随机障碍物地图"""
        self.width_m = width_m
        self.height_m = height_m
        
        # 计算数组大小 (使用 round 确定整体尺寸是合理的)
        self.width_idx = round(width_m / self.config.xy_resolution)
        self.height_idx = round(height_m / self.config.xy_resolution)
        
        self.obstacle_map = np.zeros((self.width_idx, self.height_idx), dtype=bool)
        
        # 生成四周墙壁
        for i in range(self.width_idx):
            self.obstacle_map[i][0] = True
            self.obstacle_map[i][self.height_idx - 1] = True
        for i in range(self.height_idx):
            self.obstacle_map[0][i] = True
            self.obstacle_map[self.width_idx - 1][i] = True
            
        # 随机障碍物
        # 注意：这里我们避开最外圈墙壁，所以范围缩进
        for _ in range(obstacle_num):
            x = random.randint(1, self.width_idx - 2)
            y = random.randint(1, self.height_idx - 2)
            self.obstacle_map[x][y] = True

    def plot_map(self):
        """可视化地图 (Matplotlib)"""
        if self.obstacle_map is None:
            return
            
        obs_x, obs_y = [], []
        for x in range(self.width_idx):
            for y in range(self.height_idx):
                if self.obstacle_map[x][y]:
                    # 调用 get_pos_from_index 获取物理中心
                    # 这样画出来的点，物理上就是感知认为的障碍物中心
                    # 并且会与 imshow 的热力图方块中心对齐
                    px, py = self.get_pos_from_index(x, y)
                    obs_x.append(px)
                    obs_y.append(py)
        
        plt.plot(obs_x, obs_y, ".k")
        plt.axis("equal")
        plt.grid(True)

    def check_strict_path_collision(self, path_x: List[float], path_y: List[float], path_yaw: List[float]) -> List[Tuple[float, float]]:
        """
        严格碰撞检测（多边形精确检测）。

       使用车辆多边形进行精确碰撞检测，对比规划时的圆形近似模型。
        这可以作为 Ground Truth Check 和消融实验分析。

        实现：使用 Shapely 几何库进行高效的栅格-多边形相交检测。

        Args:
            path_x: 路径 x 坐标列表 [m]
            path_y: 路径 y 坐标列表 [m]
            path_yaw: 路径航向角列表 [rad]

        Returns:
            List[Tuple[float, float]]: 被碰撞的障碍物坐标列表 [(x, y), ...]
        """
        if self.obstacle_map is None:
            return []

        collided_obstacles = []
        outline = self.vehicle_config.vehicle_outline

        # 遍历路径上的每一个位姿
        for i in range(len(path_x)):
            x, y, yaw = path_x[i], path_y[i], path_yaw[i]

            # 1. 构建车辆多边形（Shapely Polygon）
            # 旋转矩阵 (2x2)
            rot = np.array([
                [math.cos(yaw), math.sin(yaw)],
                [-math.sin(yaw), math.cos(yaw)]
            ])

            # 变换: Outline(2xN) -> Transpose -> Dot -> Transpose -> Translate
            rotated_outline = (outline.T.dot(rot)).T
            rotated_outline[0, :] += x
            rotated_outline[1, :] += y

            # 提取多边形顶点（去除重复的最后一个点）
            vehicle_vertices = []
            for j in range(rotated_outline.shape[1] - 1):  # -1 去掉重复的起点
                vehicle_vertices.append((rotated_outline[0, j], rotated_outline[1, j]))

            # 创建 Shapely Polygon 对象
            vehicle_polygon = Polygon(vehicle_vertices)

            # 2. 计算车辆 Bounding Box 以优化检测范围
            min_x_idx = max(0, int(math.floor(np.min(rotated_outline[0, :]) / self.config.xy_resolution)))
            max_x_idx = min(self.width_idx - 1, int(math.ceil(np.max(rotated_outline[0, :]) / self.config.xy_resolution)))
            min_y_idx = max(0, int(math.floor(np.min(rotated_outline[1, :]) / self.config.xy_resolution)))
            max_y_idx = min(self.height_idx - 1, int(math.ceil(np.max(rotated_outline[1, :]) / self.config.xy_resolution)))

            # 3. 仅检查 Bounding Box 内的障碍物栅格
            for ix in range(min_x_idx, max_x_idx + 1):
                for iy in range(min_y_idx, max_x_idx + 1):
                    if self.obstacle_map[ix][iy]:
                        # 获取障碍物栅格中心的物理坐标
                        obs_center_x, obs_center_y = self.get_pos_from_index(ix, iy)

                        # 计算栅格的边界（用于 Shapely 相交检测）
                        res = self.config.xy_resolution
                        grid_min_x = obs_center_x - res / 2
                        grid_max_x = obs_center_x + res / 2
                        grid_min_y = obs_center_y - res / 2
                        grid_max_y = obs_center_y + res / 2

                        # 创建栅格的 AABB (Axis-Aligned Bounding Box)
                        grid_bbox = box(grid_min_x, grid_min_y, grid_max_x, grid_max_y)

                        # 使用 Shapely 检测相交（高效且精确）
                        if vehicle_polygon.intersects(grid_bbox):
                            collided_obstacles.append((obs_center_x, obs_center_y))

        return collided_obstacles

# --- 单元测试 ---
if __name__ == "__main__":
    print("Testing GridMap with Corrected Coordinates...")
    
    h_cfg = HybridAStarConfig()
    # 假设分辨率是 1.0
    h_cfg.xy_resolution = 1.0 
    v_cfg = VehicleConfig()
    
    # 2. 实例化地图
    gm = GridMap(h_cfg, v_cfg)
    gm.generate_random_map(20, 20, 30)
    
    # 验证 SSOT
    # 1. 测试坐标转换
    test_pos = (5.9, 5.1)
    idx = gm.get_index_from_pos(*test_pos)
    print(f"Pos {test_pos} -> Index {idx} (Expected: (5, 5))")
    
    center_pos = gm.get_pos_from_index(*idx)
    print(f"Index {idx} -> Center Pos {center_pos} (Expected: (5.5, 5.5))")

    # 2. 绘图验证
    plt.figure(figsize=(6, 6))
    gm.plot_map()
    
    # 画一个红色的框代表 Grid(5,5) 的物理范围
    rect = plt.Rectangle((5.0, 5.0), 1.0, 1.0, linewidth=2, edgecolor='r', facecolor='none', label='Grid(5,5) Bounds')
    plt.gca().add_patch(rect)
    
    # 画转换后的中心点
    plt.plot(center_pos[0], center_pos[1], "xr", markersize=12, label='Grid Center')
    
    plt.legend()
    plt.title("GridMap Coordinate System Check")
    plt.show()