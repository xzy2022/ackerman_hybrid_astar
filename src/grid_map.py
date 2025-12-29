import math
import random
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional



# 导入接口与配置
from src.interfaces import BaseMap
from src.config import HybridAStarConfig, VehicleConfig

class GridMap(BaseMap):
    """
    基于栅格的地图实现 (Grid Map Implementation).
    
    继承自 BaseMap，实现了具体的碰撞检测逻辑。
    内部维护一个 numpy 二维数组作为占用栅格：
    - False (0): 自由区域
    - True  (1): 障碍物
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
            
            # 2. 转换为栅格索引
            cx_idx = round(cx / self.config.xy_resolution)
            cy_idx = round(cy / self.config.xy_resolution)
            
            # 3. 越界检查 (Out of bounds check)
            if (cx_idx < 0 or cx_idx >= self.width_idx or
                cy_idx < 0 or cy_idx >= self.height_idx):
                return True
            
            # 4. 栅格占用检查
            # 简单的点检查：如果圆心所在的格子是障碍物，则碰撞
            if self.obstacle_map[cx_idx][cy_idx]:
                return True
            
            # 5. (可选) 邻域安全检查
            # 如果圆的半径比栅格分辨率大，单纯检查圆心是不够的。
            # 这里做一个简单的近似：检查圆心周围一圈的格子
            # 这种方法比完全的几何求交快得多，对于自动驾驶路径规划通常足够精确
            search_radius_grid = int(math.ceil(self.vehicle_config.collision_radius / self.config.xy_resolution))
            
            # 优化：只在靠近障碍物时才进行细致检查 (可以根据具体需求开启)
            # 这里为了健壮性，我们检查圆心周围的小方块区域
            # 注意：这还是近似，如果需要绝对精确，应用 KD-Tree 查询圆内所有障碍点
            for i in range(-1, 2): # 检查 3x3 邻域
                for j in range(-1, 2):
                    nx, ny = cx_idx + i, cy_idx + j
                    if 0 <= nx < self.width_idx and 0 <= ny < self.height_idx:
                        if self.obstacle_map[nx][ny]:
                            # 如果邻域有障碍物，计算精确距离
                            dist = math.hypot((nx - cx_idx)*self.config.xy_resolution, 
                                            (ny - cy_idx)*self.config.xy_resolution)
                            if dist <= self.vehicle_config.collision_radius:
                                return True

        return False # 通过所有检查，无碰撞

    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        [实现 BaseMap 接口] 获取物理边界
        Returns: (min_x, max_x, min_y, max_y)
        """
        return (0.0, self.width_m, 0.0, self.height_m)

    # --- 以下是 GridMap 特有的辅助方法 (非 BaseMap 接口强制) ---

    def generate_random_map(self, width_m: float, height_m: float, obstacle_num: int):
        """生成测试用的随机障碍物地图"""
        self.width_m = width_m
        self.height_m = height_m
        
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
                    obs_x.append(x * self.config.xy_resolution)
                    obs_y.append(y * self.config.xy_resolution)
        
        plt.plot(obs_x, obs_y, ".k")
        plt.axis("equal")
        plt.grid(True)

# --- 单元测试 ---
if __name__ == "__main__":
    # 简单的自我测试，确保类可以被实例化和调用
    print("Testing GridMap...")
    
    # 1. 实例化配置
    h_cfg = HybridAStarConfig()
    v_cfg = VehicleConfig()
    
    # 2. 实例化地图
    gm = GridMap(h_cfg, v_cfg)
    gm.generate_random_map(50, 50, 100)
    
    # 3. 测试碰撞检测
    # 假设在地图中心放一辆车
    is_hit = gm.check_collision(25.0, 25.0, 0.0)
    print(f"Collision check at (25, 25): {'HIT' if is_hit else 'FREE'}")
    
    # 4. 绘图
    plt.figure(figsize=(8, 8))
    gm.plot_map()
    # 画一个代表车的点
    plt.plot(25.0, 25.0, "or", markersize=10, label="Test Car")
    plt.legend()
    plt.title("Refactored GridMap Test")
    plt.show()