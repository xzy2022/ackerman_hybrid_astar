import numpy as np
import math
import heapq
from config import Config

class Heuristic:
    def __init__(self, config, grid_map):
        self.config = config
        self.grid_map = grid_map
        # 初始化启发式地图 (存每个格子到终点的代价)
        # 初始值为无穷大
        self.heuristic_map = np.full((grid_map.width_idx, grid_map.height_idx), float('inf'))

    def calc_heuristic(self, node, goal_node):
        """
        对外接口：获取某个节点的启发式代价 h(n)
        :param node: 当前搜索节点 (包含 x_ind, y_ind)
        :param goal_node: 目标节点
        :return: h value
        """
        # 1. 基础启发式：查表 (Holonomic Heuristic)
        # 检查索引是否越界
        if (0 <= node.x_index < self.grid_map.width_idx and 
            0 <= node.y_index < self.grid_map.height_idx):
            
            h_cost = self.heuristic_map[node.x_index][node.y_index]
            
            # 如果查不到(比如是死区)，回退到欧几里得距离，防止报错
            if h_cost == float('inf'):
                return math.hypot(node.x_list[-1] - goal_node.x_list[-1],
                                  node.y_list[-1] - goal_node.y_list[-1])
            return h_cost
            
        else:
            # 越界情况
            return float('inf')

    def calc_holonomic_heuristic_with_obstacle(self, goal_node):
        """
        核心逻辑：从终点反向运行 Dijkstra，生成全图的势场
        这只需要在路径规划开始前运行一次！
        """
        print("Calculating Holonomic Heuristic Map...")
        
        # 目标点的栅格坐标
        gx, gy = goal_node.x_index, goal_node.y_index
        
        # 1. 初始化
        self.heuristic_map[gx][gy] = 0.0
        
        # 优先队列: (cost, x_ind, y_ind)
        pq = [(0.0, gx, gy)]
        
        # 定义8邻域移动 (可以走斜线)
        # (dx, dy, cost)
        motions = [
            (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0), # 上下左右
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414) # 对角线
        ]
        
        # 2. Dijkstra 主循环
        while True:
            if not pq:
                break
                
            cost, cx, cy = heapq.heappop(pq)
            
            # 如果当前取代价已经大于记录值，跳过
            if cost > self.heuristic_map[cx][cy]:
                continue
                
            # 搜索邻居
            for dx, dy, move_cost in motions:
                nx, ny = cx + dx, cy + dy
                
                # 越界检查
                if (nx < 0 or nx >= self.grid_map.width_idx or 
                    ny < 0 or ny >= self.grid_map.height_idx):
                    continue
                    
                # 障碍物检查 (如果该格子是障碍物，则无法通行，cost保持为inf)
                if self.grid_map.obstacle_map[nx][ny]:
                    continue
                
                # 更新代价
                new_cost = cost + move_cost
                if new_cost < self.heuristic_map[nx][ny]:
                    self.heuristic_map[nx][ny] = new_cost
                    heapq.heappush(pq, (new_cost, nx, ny))
        
        print("Heuristic Map Calculated.")

# --- 单元测试 ---
if __name__ == "__main__":
    from grid_map import GridMap
    import matplotlib.pyplot as plt
    
    cfg = Config()
    gm = GridMap(cfg)
    gm.generate_map(50, 50, 50) # 随机障碍物
    
    # 假设终点在 (40, 40)
    # 我们构造一个假的 Goal Node，只用到它的 index
    class MockNode:
        def __init__(self, x, y):
            self.x_index = x
            self.y_index = y
            self.x_list = [x * cfg.XY_RES] # 备用
            self.y_list = [y * cfg.XY_RES]
            
    goal_n = MockNode(round(40/cfg.XY_RES), round(40/cfg.XY_RES))
    
    # 计算启发式
    h_gen = Heuristic(cfg, gm)
    h_gen.calc_holonomic_heuristic_with_obstacle(goal_n)
    
    # 可视化热力图 (Heatmap)
    # 颜色越深代表离终点越近，黑色区域代表不可达或障碍物
    plt.figure(figsize=(10, 8))
    # 旋转矩阵以便与地图方向一致
    plt.imshow(h_gen.heuristic_map.T, origin='lower', cmap='jet', interpolation='nearest')
    plt.colorbar(label='Heuristic Cost (Distance to Goal)')
    plt.title("Holonomic Heuristic Heatmap (Dijkstra Field)")
    
    # 把障碍物点一下，看看是否对上了
    obs_x, obs_y = [], []
    for x in range(gm.width_idx):
        for y in range(gm.height_idx):
            if gm.obstacle_map[x][y]:
                obs_x.append(x)
                obs_y.append(y)
    plt.plot(obs_x, obs_y, ".k", label="Obstacles")
    
    plt.plot(goal_n.x_index, goal_n.y_index, "xw", markersize=15, label="Goal")
    plt.legend()
    plt.show()