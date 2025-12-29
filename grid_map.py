import math
import random
import numpy as np
import matplotlib.pyplot as plt
from config import Config

class GridMap:
    def __init__(self, config):
        self.config = config
        self.xy_res = config.XY_RES
        self.width_idx = 0
        self.height_idx = 0
        self.min_x = 0
        self.min_y = 0
        # 障碍物地图: 2D numpy数组, False(0)=自由, True(1)=障碍
        self.obstacle_map = None 

    def generate_map(self, width_m, height_m, obstacle_num, start_m=None, goal_m=None):
        """
        生成随机栅格地图
        :param width_m: 地图物理宽度 (米)
        :param height_m: 地图物理高度 (米)
        :param obstacle_num: 障碍物数量 (散点)
        :param start_m: (x, y) 起点物理坐标，用于避开生成障碍物
        :param goal_m: (x, y) 终点物理坐标，用于避开生成障碍物
        """
        # 1. 计算栅格尺寸
        self.width_idx = round(width_m / self.xy_res)
        self.height_idx = round(height_m / self.xy_res)
        
        # 2. 初始化地图 (默认为False/自由)
        self.obstacle_map = np.zeros((self.width_idx, self.height_idx), dtype=bool)

        # 3. 生成边界墙 (四周设为障碍)
        for i in range(self.width_idx):
            self.obstacle_map[i][0] = True
            self.obstacle_map[i][self.height_idx - 1] = True
        for i in range(self.height_idx):
            self.obstacle_map[0][i] = True
            self.obstacle_map[self.width_idx - 1][i] = True

        # 4. 随机生成障碍物
        for _ in range(obstacle_num):
            x = random.randint(1, self.width_idx - 2)
            y = random.randint(1, self.height_idx - 2)

            # 检查是否覆盖了起点或终点 (如果有的话)
            if start_m and self.check_collision_with_point(x, y, start_m):
                continue
            if goal_m and self.check_collision_with_point(x, y, goal_m):
                continue

            self.obstacle_map[x][y] = True

    def check_collision_with_point(self, x_idx, y_idx, point_m):
        """简单的距离检查，防止障碍物生成在关键点上"""
        px_idx = round(point_m[0] / self.xy_res)
        py_idx = round(point_m[1] / self.xy_res)
        # 如果在关键点周围 2米 范围内，则视为冲突
        safe_dist_idx = 2.0 / self.xy_res 
        if math.hypot(x_idx - px_idx, y_idx - py_idx) <= safe_dist_idx:
            return True
        return False

    def calc_xy_index_from_position(self, pos, lower_bound=0):
        """将物理坐标(米)转换为栅格索引"""
        return round((pos - lower_bound) / self.xy_res)

    def calc_position_from_index(self, index, lower_bound=0):
        """将栅格索引转换为物理坐标(米)"""
        return index * self.xy_res + lower_bound

    def plot_map(self, start_m=None, goal_m=None):
        """使用Matplotlib可视化地图"""
        # 获取所有障碍物的x, y索引
        obs_x, obs_y = [], []
        for x in range(self.width_idx):
            for y in range(self.height_idx):
                if self.obstacle_map[x][y]:
                    # 转回物理坐标用于绘图
                    ox = self.calc_position_from_index(x)
                    oy = self.calc_position_from_index(y)
                    obs_x.append(ox)
                    obs_y.append(oy)

        plt.figure(figsize=(10, 10))
        # 画障碍物 (黑色方块)
        plt.plot(obs_x, obs_y, ".k")
        
        # 画起点 (绿色)
        if start_m:
            plt.plot(start_m[0], start_m[1], "og", markersize=10, label="Start")
        
        # 画终点 (红色)
        if goal_m:
            plt.plot(goal_m[0], goal_m[1], "xr", markersize=10, label="Goal")
        
        plt.grid(True)
        plt.axis("equal")
        plt.title("Grid Map Generator Test")
        plt.legend()


# --- 单元测试代码 ---
if __name__ == "__main__":
    print("Testing Grid Map Generation...")
    
    # 实例化配置
    cfg = Config()
    
    # 实例化地图
    gm = GridMap(cfg)
    
    # 设定测试用的起点和终点
    sx, sy = 10.0, 10.0
    gx, gy = 40.0, 40.0
    
    # 生成 50x50米的地图，包含50个随机障碍物
    gm.generate_map(50.0, 50.0, 50, start_m=[sx, sy], goal_m=[gx, gy])
    
    print(f"Map Size: {gm.width_idx} x {gm.height_idx}")
    
    # 绘图确认
    gm.plot_map(start_m=[sx, sy], goal_m=[gx, gy])
    plt.show()