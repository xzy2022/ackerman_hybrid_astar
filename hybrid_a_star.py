import heapq
import math
import matplotlib.pyplot as plt
import numpy as np
from config import Config
from grid_map import GridMap
from heuristic import Heuristic

class Node:
    def __init__(self, x_ind, y_ind, yaw_ind, direction, x_list, y_list, yaw_list,
                 steer, parent=None, cost=0.0, yaw=0.0):
        """
        Hybrid A* 的节点
        :param x_ind: 栅格地图 X 索引 (int)
        :param y_ind: 栅格地图 Y 索引 (int)
        :param yaw_ind: 航向角索引 (int)
        :param direction: 移动方向 (1: 前进, -1: 后退)
        :param x_list: 轨迹上的 X 坐标列表 (float)
        :param y_list: 轨迹上的 Y 坐标列表 (float)
        :param yaw_list: 轨迹上的 Yaw 角度列表 (float)
        :param steer: 到达该节点时的转向角 (float)
        :param parent: 直接存储父节点对象引用
        :param cost: 路径代价值 (g + h) (float)
        :param yaw: 实际的物理航向角 (float)
        """
        self.x_index = x_ind
        self.y_index = y_ind
        self.yaw_index = yaw_ind
        self.direction = direction
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.steering = steer
        self.parent = parent  # 指向父节点对象
        self.cost = cost
        self.g_cost = 0.0     # g: 从起点到当前的实际代价
        self.yaw = yaw

    # 定义小于运算符，用于堆排序 (heapq 会根据 cost 自动排序)
    def __lt__(self, other):
        return self.cost < other.cost

# --- 辅助函数 ---
def normalize_angle(angle):
    """
    将角度标准化到 [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def move_bicycle_model(x, y, yaw, distance, steer, L):
    """
    离散化的阿克曼车辆运动学模型 (Bicycle Model)
    :param distance: 移动距离 (speed * dt)
    :param steer: 转向角
    :param L: 轴距
    """
    x += distance * math.cos(yaw)
    y += distance * math.sin(yaw)
    yaw += distance * math.tan(steer) / L
    yaw = normalize_angle(yaw)
    return x, y, yaw

def get_final_path(closed_node):
    """从 Goal 节点回溯到 Start"""
    path_x, path_y, path_yaw = [], [], []
    curr = closed_node
    
    while curr is not None:
        # x_list 是一小段轨迹 (Start -> End)，回溯时需要倒过来
        path_x.extend(curr.x_list[::-1])
        path_y.extend(curr.y_list[::-1])
        path_yaw.extend(curr.yaw_list[::-1])
        curr = curr.parent
        
    # 此时得到的是 Goal -> Start，需要再次反转
    return path_x[::-1], path_y[::-1], path_yaw[::-1]

def check_collision(node, grid_map_obj, config):
    """
    检测节点路径是否碰撞
    :param node: 当前节点
    :param grid_map_obj: GridMap 对象 (包含 obstacle_map)
    :param config: 配置对象
    :return: True=碰撞/越界, False=安全
    """
    # 遍历轨迹上的每一个采样点
    for x, y, yaw in zip(node.x_list, node.y_list, node.yaw_list):
        
        # 遍历车身上的每一个碰撞检测圆 (3圆模型)
        for offset in config.collision_circle_offsets:
            # 计算圆心世界坐标
            cx = x + offset * math.cos(yaw)
            cy = y + offset * math.sin(yaw)
            
            # 转换为栅格索引
            cx_idx = round(cx / config.XY_RES)
            cy_idx = round(cy / config.XY_RES)
            
            # 1. 检查越界
            if (cx_idx < 0 or cx_idx >= grid_map_obj.width_idx or
                cy_idx < 0 or cy_idx >= grid_map_obj.height_idx):
                return True # 越界
            
            # 2. 检查障碍物 (圆心所在栅格)
            if grid_map_obj.obstacle_map[cx_idx][cy_idx]:
                return True
            
            # (可选) 3. 简单的邻域检查，模拟圆的半径覆盖
            # 如果圆半径比栅格大很多，只查中心点是不够的，这里做一个简单的十字扩散检查
            # 实际项目中通常使用 KD-Tree 或距离变换图(DT)加速
            steps = 1 # 简单检查周围一圈
            for i in range(-steps, steps + 1):
                for j in range(-steps, steps + 1):
                    nx, ny = cx_idx + i, cy_idx + j
                    if (0 <= nx < grid_map_obj.width_idx and 
                        0 <= ny < grid_map_obj.height_idx):
                        if grid_map_obj.obstacle_map[nx][ny]:
                            # 如果障碍物在圆的半径内
                            dist = math.hypot((nx-cx_idx)*config.XY_RES, (ny-cy_idx)*config.XY_RES)
                            if dist <= config.collision_circle_radius:
                                return True
    
    return False # 通过所有检查


def calc_next_states(current_node, config):
    """生成子节点，并计算 G-Cost"""
    next_nodes = []
    step_length = config.XY_RES * 2.0
    
    # 增加转向采样的密度，提高搜索灵活性
    n_steer = 3 
    steer_inputs = [0.0]
    for i in range(n_steer):
        s = config.MAX_STEER * (i + 1) / n_steer
        steer_inputs.append(s)
        steer_inputs.append(-s)
        
    directions = [1, -1] 

    for direction in directions:
        for steer in steer_inputs:
            # --- 1. 轨迹推演 ---
            x_list = list(current_node.x_list)
            y_list = list(current_node.y_list)
            yaw_list = list(current_node.yaw_list)

            # 获取当前物理状态 (取上一段轨迹的终点)
            node_x = x_list[-1]
            node_y = y_list[-1]
            node_yaw = current_node.yaw # 使用精确物理角度

            # 模拟积分
            dist = direction * step_length
            num_sub_steps = 5
            d_sub = dist / num_sub_steps
            
            curr_x, curr_y, curr_yaw = node_x, node_y, node_yaw
            
            # 注意：新节点不应包含父节点的整个历史，只包含这一步的轨迹
            # 这样节省内存，回溯时再拼接
            step_x_list = []
            step_y_list = []
            step_yaw_list = []

            for _ in range(num_sub_steps):
                curr_x, curr_y, curr_yaw = move_bicycle_model(
                    curr_x, curr_y, curr_yaw, d_sub, steer, config.L
                )
                step_x_list.append(curr_x)
                step_y_list.append(curr_y)
                step_yaw_list.append(curr_yaw)

            # --- 2. 计算栅格索引 (用于 A* CloseSet 判重) ---
            x_ind = round(curr_x / config.XY_RES)
            y_ind = round(curr_y / config.XY_RES)
            yaw_ind = round(curr_yaw / config.YAW_RES)

            # --- 3. 代价计算 (G Cost) ---
            # 基础代价
            cost_inc = step_length
            
            # 惩罚项
            if direction == -1:
                cost_inc += step_length * 0.5  # 倒车惩罚
            
            if direction != current_node.direction:
                cost_inc += 2.0  # 换挡惩罚 (Gear Switch)
            
            cost_inc += abs(steer) * 0.5  # 转向惩罚
            
            # 累加父节点的 g_cost
            new_g_cost = current_node.g_cost + cost_inc

            # --- 4. 创建节点 ---
            new_node = Node(x_ind, y_ind, yaw_ind, direction, 
                            step_x_list, step_y_list, step_yaw_list,
                            steer, parent=current_node, cost=0.0, yaw=curr_yaw)
            new_node.g_cost = new_g_cost
            
            next_nodes.append(new_node)
            
    return next_nodes

# --- 3. 核心主循环 ---
def hybrid_a_star_planning(start_m, goal_m, grid_map, config):
    """
    Hybrid A* 主入口
    :param start_m: [x, y, yaw] (米, 米, 弧度)
    :param goal_m: [x, y, yaw] (米, 米, 弧度)
    :return: (x, y, yaw) path lists
    """
    
    # 1. 初始化 Start 和 Goal
    sx, sy, syaw = start_m
    gx, gy, gyaw = goal_m
    
    start_node = Node(round(sx/config.XY_RES), round(sy/config.XY_RES), round(syaw/config.YAW_RES),
                      1, [sx], [sy], [syaw], 0, parent=None, cost=0.0, yaw=syaw)
    
    goal_node = Node(round(gx/config.XY_RES), round(gy/config.XY_RES), round(gyaw/config.YAW_RES),
                     1, [gx], [gy], [gyaw], 0, parent=None, cost=0.0, yaw=gyaw)

    # 2. 准备 OpenList 和 ClosedList
    open_list = [] 
    heapq.heappush(open_list, start_node)
    
    # ClosedList 使用字典: key=(x_ind, y_ind, yaw_ind) -> Node
    closed_list = {}
    closed_list[(start_node.x_index, start_node.y_index, start_node.yaw_index)] = start_node

    # 3. 计算启发式地图 (从终点反向 Dijkstra)
    heuristic_calc = Heuristic(config, grid_map)
    heuristic_calc.calc_holonomic_heuristic_with_obstacle(goal_node)

    print("Start Hybrid A* Search...")
    
    while True:
        # --- 3.1 检查 OpenList ---
        if not open_list:
            print("Error: Cannot find path, OpenList is empty.")
            return None, None, None

        # 取出 f 值最小的节点
        current = heapq.heappop(open_list)
        
        # --- 3.2 检查是否到达目标 (Goal Check) ---
        # 判据：距离小于 1.0米 且 角度差小于 20度
        dist_to_goal = math.hypot(current.x_list[-1] - gx, current.y_list[-1] - gy)
        angle_to_goal = abs(normalize_angle(current.yaw - gyaw))
        
        if dist_to_goal < 1.0 and angle_to_goal < math.radians(20.0):
            print(f"Goal Reached! Final Cost: {current.cost:.2f}")
            return get_final_path(current)

        # --- 3.3 扩展子节点 ---
        next_nodes = calc_next_states(current, config)
        
        for neighbor in next_nodes:
            # 1. 碰撞检测
            if check_collision(neighbor, grid_map, config):
                continue
            
            # 2. 计算启发式 h(n)
            # 使用 Holonomic Heuristic (无运动学约束的 2D A*)
            h = heuristic_calc.calc_heuristic(neighbor, goal_node)
            
            # 3. 计算总代价 f(n) = g(n) + h(n)
            neighbor.cost = neighbor.g_cost + h
            
            # 4. 查重与更新
            state_key = (neighbor.x_index, neighbor.y_index, neighbor.yaw_index)
            
            if state_key in closed_list:
                # 如果这个状态已经访问过，并且之前的路径代价更小，则跳过
                if closed_list[state_key].cost <= neighbor.cost:
                    continue
            
            # 加入 OpenList 和 ClosedList
            heapq.heappush(open_list, neighbor)
            closed_list[state_key] = neighbor

# --- 4. 运行测试 ---
if __name__ == "__main__":
    from grid_map import GridMap
    
    cfg = Config()
    
    # 1. 生成带障碍物的地图
    gm = GridMap(cfg)
    gm.generate_map(50, 50, 150) # 尺寸50x50米，生成150个随机障碍点
    
    # 2. 定义起点和终点 [x, y, yaw]
    # 确保起点终点不在障碍物上
    start_pos = [10.0, 10.0, np.deg2rad(0.0)]
    goal_pos = [40.0, 40.0, np.deg2rad(90.0)]
    
    # 清理起点终点附近的障碍物
    def clear_area(gm, x, y):
        idx_x = round(x / cfg.XY_RES)
        idx_y = round(y / cfg.XY_RES)
        for i in range(-2, 3):
            for j in range(-2, 3):
                gm.obstacle_map[idx_x+i][idx_y+j] = False
                
    clear_area(gm, start_pos[0], start_pos[1])
    clear_area(gm, goal_pos[0], goal_pos[1])
    
    # 3. 运行规划
    path_x, path_y, path_yaw = hybrid_a_star_planning(start_pos, goal_pos, gm, cfg)
    
    # 4. 绘图
    
    # 画地图 (该方法不会自动弹窗plot)
    gm.plot_map()
    
    # 画起点终点
    plt.plot(start_pos[0], start_pos[1], "og", label="Start", markersize=10)
    plt.plot(goal_pos[0], goal_pos[1], "xr", label="Goal", markersize=10)
    plt.arrow(start_pos[0], start_pos[1], math.cos(start_pos[2])*2, math.sin(start_pos[2])*2, head_width=0.5, color='g')
    plt.arrow(goal_pos[0], goal_pos[1], math.cos(goal_pos[2])*2, math.sin(goal_pos[2])*2, head_width=0.5, color='r')

    if path_x:
        plt.plot(path_x, path_y, "-r", linewidth=2, label="Hybrid A* Path")
        plt.title("Hybrid A* Path Planning")
    else:
        plt.title("Failed to find path")
        
    plt.legend()
    plt.show()