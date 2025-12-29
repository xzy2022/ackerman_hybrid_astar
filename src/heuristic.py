import heapq
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional, List

# 导入接口与配置
from src.interfaces import BaseHeuristic
from src.config import HybridAStarConfig
from src.grid_map import GridMap 

class HolonomicHeuristic(BaseHeuristic):
    """
    全向启发式算法 (Holonomic Heuristic).
    
    核心原理：
    忽略车辆的非完整约束（即假设车可以像人一样向任意方向移动），
    从目标点出发，在栅格地图上运行反向 Dijkstra 算法，计算所有栅格到目标点的最短路径距离。
    
    这个距离构成了 Hybrid A* 的 h(n) 值，能有效引导搜索避开死胡同（U形陷阱）。
    """

    def __init__(self, config: HybridAStarConfig, grid_map: GridMap):
        self.config = config
        self.grid_map = grid_map
        
        # 缓存数据
        # heuristic_map[x][y] 存储从 (x,y) 到目标的距离代价
        self.heuristic_map: Optional[np.ndarray] = None
        
        # 记录上一次计算的目标点索引，用于判断是否需要重算
        self.last_goal_index: Optional[Tuple[int, int]] = None

    def calculate(self, current_pose: Tuple[float, float, float], 
                  goal_pose: Tuple[float, float, float]) -> float:
        """
        [实现 BaseHeuristic 接口]
        获取当前状态的启发式代价。如果目标点变了，会自动重新计算势场。
        """
        # 1. 解析目标点索引
        gx_idx = round(goal_pose[0] / self.config.xy_resolution)
        gy_idx = round(goal_pose[1] / self.config.xy_resolution)
        current_goal_idx = (gx_idx, gy_idx)

        # 2. 检查是否需要重新运行 Dijkstra (Lazy Update)
        if self.heuristic_map is None or current_goal_idx != self.last_goal_index:
            self._update_dijkstra_field(current_goal_idx)
            self.last_goal_index = current_goal_idx

        # 3. 解析当前点索引
        cx_idx = round(current_pose[0] / self.config.xy_resolution)
        cy_idx = round(current_pose[1] / self.config.xy_resolution)

        # 4. 查表获取 h_cost
        h_cost = float('inf')
        
        # 边界检查
        if (0 <= cx_idx < self.grid_map.width_idx and 
            0 <= cy_idx < self.grid_map.height_idx):
            h_cost = self.heuristic_map[cx_idx][cy_idx]

        # 5. 降级处理 (Fallback)
        # 如果查表结果是 inf (说明该点在障碍物内或无法到达的死区)，
        # 为了不让 A* 报错或卡死，回退使用欧几里得距离作为保底估计。
        if h_cost == float('inf'):
            return math.hypot(current_pose[0] - goal_pose[0], 
                              current_pose[1] - goal_pose[1])
        
        return h_cost

    def _update_dijkstra_field(self, goal_index: Tuple[int, int]):
        """
        核心逻辑：运行反向 Dijkstra 生成全图势场
        """
        # print(f"Heuristic: Re-calculating field for goal {goal_index}...")
        
        gx, gy = goal_index
        w = self.grid_map.width_idx
        h = self.grid_map.height_idx
        
        # 初始化地图为无穷大
        self.heuristic_map = np.full((w, h), float('inf'))
        
        # 目标点在地图外，直接返回（此时所有查询都会走 Fallback）
        if not (0 <= gx < w and 0 <= gy < h):
            print("Warning: Goal is outside grid map!")
            return

        # 初始化搜索
        self.heuristic_map[gx][gy] = 0.0
        
        # 优先队列: (cost, x_ind, y_ind)
        pq = [(0.0, gx, gy)]
        
        # 8邻域移动动作 (dx, dy, cost)
        # 直线代价 1.0，对角线代价 1.414
        motions = [
            (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]
        
        # 为了加速，提取 obstacle_map 的引用
        # 注意：这里我们假设 GridMap 内部维护了 obstacle_map 属性
        obs_map = self.grid_map.obstacle_map

        while pq:
            cost, cx, cy = heapq.heappop(pq)
            
            # 如果当前路径比已记录的更长，跳过
            if cost > self.heuristic_map[cx][cy]:
                continue
            
            # 扩展邻居
            for dx, dy, move_cost in motions:
                nx, ny = cx + dx, cy + dy
                
                # 越界检查
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                
                # 障碍物检查 (如果该格子是障碍物，则跳过)
                if obs_map[nx][ny]:
                    continue
                
                new_cost = cost + move_cost
                if new_cost < self.heuristic_map[nx][ny]:
                    self.heuristic_map[nx][ny] = new_cost
                    heapq.heappush(pq, (new_cost, nx, ny))
        
        # print("Heuristic: Field updated.")

    def visualize_cost_map(self):
        """
        科研功能：可视化启发式代价热力图。
        这对于论文中展示“势场引导效果”非常有用。
        """
        if self.heuristic_map is None:
            print("Heuristic map is empty.")
            return

        # 转置以匹配 (x, y) 坐标系显示习惯
        # 使用 'jet' 颜色映射，无穷大值通常显示为深红色或特定颜色
        # 为了显示效果，可以将 inf 替换为一个大数值
        disp_map = self.heuristic_map.copy()
        max_val = np.nanmax(disp_map[disp_map != float('inf')])
        # 将 inf 设置为稍微大一点的值，以便显示
        disp_map[disp_map == float('inf')] = max_val * 1.2

        # 2. 计算物理范围 (extent)
        # extent = [xmin, xmax, ymin, ymax]
        # 注意：这里假设地图原点是 (0,0)
        map_width_m = self.grid_map.width_idx * self.config.xy_resolution
        map_height_m = self.grid_map.height_idx * self.config.xy_resolution
        extent = [0, map_width_m, 0, map_height_m]

        # 3. 绘制 imshow 并指定 extent
        # alpha=0.6 设置透明度，这样能透过热力图看到底下的障碍物点
        plt.imshow(disp_map.T, 
                   origin='lower', 
                   cmap='jet_r', 
                   interpolation='nearest',
                   extent=extent,  # 把索引拉伸成物理尺寸
                   alpha=0.6)
        
        plt.colorbar(label='Heuristic Cost (Distance to Goal)')
        plt.title("Holonomic Heuristic Field")
        
        if self.last_goal_index:
            # 目标点也要转回米制单位才能对齐
            gx_m = self.last_goal_index[0] * self.config.xy_resolution
            gy_m = self.last_goal_index[1] * self.config.xy_resolution
            plt.plot(gx_m, gy_m, "*w", markersize=15, label="Goal")
            plt.legend()

# --- 单元测试 ---
if __name__ == "__main__":
    from src.config import VehicleConfig
    
    print("Testing HolonomicHeuristic...")
    
    # 1. 准备环境
    h_cfg = HybridAStarConfig()
    v_cfg = VehicleConfig()
    gm = GridMap(h_cfg, v_cfg)
    gm.generate_random_map(50, 50, 80) # 生成一些障碍物
    
    # 2. 初始化启发式
    heuristic = HolonomicHeuristic(h_cfg, gm)
    
    # 3. 设定测试点
    start_pos = (10.0, 10.0, 0.0)
    goal_pos = (40.0, 40.0, 0.0)
    
    # 4. 计算代价 (第一次调用会触发 Dijkstra 计算)
    cost = heuristic.calculate(start_pos, goal_pos)
    print(f"Heuristic cost from {start_pos} to {goal_pos}: {cost:.2f}")
    
    # 5. 可视化
    gm.plot_map() # 画底层障碍物
    heuristic.visualize_cost_map() # 画上层热力图
    plt.show()