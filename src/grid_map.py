import math
import random
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional

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
        """
        if self.obstacle_map is None:
            return True # 地图未初始化视为不可通行
            
        # 遍历车身上的每一个碰撞检测圆 (由 VehicleConfig 定义)
        for offset in self.vehicle_config.collision_offsets:
            # 1. 计算圆心在世界坐标系的位置
            #公式: cx = x + offset * cos(yaw)
            cx = x + offset * math.cos(yaw)
            cy = y + offset * math.sin(yaw)
            
            # 2. 转换为栅格索引 (使用统一接口)
            cx_idx, cy_idx = self.get_index_from_pos(cx, cy)
            
            # 3. 越界检查 (Out of bounds check)
            if (cx_idx < 0 or cx_idx >= self.width_idx or
                cy_idx < 0 or cy_idx >= self.height_idx):
                return True
            
            # 4. 栅格占用检查
            # 简单的点检查：如果圆心所在的格子是障碍物，则碰撞
            if self.obstacle_map[cx_idx][cy_idx]:
                return True
            
            # 5. 邻域精确检查 (Circle Approximation refinement)
            # 因为我们把圆心离散化到了 cx_idx，但这不代表圆只覆盖这一个格子。
            # 如果圆半径很大，或者圆心刚好在格子边缘，可能碰撞到隔壁格子。
            # 这里我们检查圆心所在的 3x3 邻域。
            for i in range(-1, 2):
                for j in range(-1, 2):
                    nx, ny = cx_idx + i, cy_idx + j
                    
                    # 边界检查
                    if 0 <= nx < self.width_idx and 0 <= ny < self.height_idx:
                        if self.obstacle_map[nx][ny]:
                            # 如果邻居是障碍物，计算【圆心】到【障碍物格子中心】的物理距离
                            obs_x_center, obs_y_center = self.get_pos_from_index(nx, ny)
                            
                            dist = math.hypot(cx - obs_x_center, cy - obs_y_center)
                            
                            # 考虑到障碍物其实是方格，这里用圆半径判断是保守估计
                            # 更严谨的做法是 Circle-AABB 碰撞，但在规划中这样通常够用了
                            # 这里的判定逻辑是：如果障碍物中心在圆内，则碰撞 (近似)
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