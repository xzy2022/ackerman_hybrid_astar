import math
import matplotlib.pyplot as plt
import numpy as np
from config import Config
from grid_map import GridMap

class Node:
    def __init__(self, x_ind, y_ind, yaw_ind, direction, x_list, y_list, yaw_list,
                 steer, pin_index, cost, yaw):
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
        :param pin_index: 父节点索引 (int)
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
        self.parent_index = pin_index
        self.cost = cost
        self.yaw = yaw

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

def calc_next_states(current_node, config):
    """
    生成当前节点的所有可能的子节点 (Motion Primitives)
    """
    next_nodes = []
    
    # 运动采样参数
    step_length = config.XY_RES * 2.0  # 每次扩展的弧长，通常设为栅格大小的1.5-2倍
    
    # 转向角离散化 (生成: -MAX, -MAX*0.6, 0, MAX*0.6, MAX 等档位)
    # 这里我们生成 N_STEER 个档位
    n_steer = 2  # 左右各分多少份
    steer_inputs = [0.0] # 包含回正
    for i in range(n_steer):
        # 简单等分，也可以设计非线性的
        s = config.MAX_STEER * (i + 1) / n_steer
        steer_inputs.append(s)   # 左转
        steer_inputs.append(-s)  # 右转
        
    # 方向离散化 (1: 前进, -1: 后退)
    directions = [1, -1]

    for direction in directions:
        for steer in steer_inputs:
            # 1. 准备初始状态
            # 使用列表推导式复制一份新的轨迹列表，避免修改原列表
            x_list = list(current_node.x_list)
            y_list = list(current_node.y_list)
            yaw_list = list(current_node.yaw_list)

            # 获取当前物理状态 (取上一段轨迹的终点)
            node_x = x_list[-1]
            node_y = y_list[-1]
            node_yaw = yaw_list[-1]

            # 2. 模拟积分
            dist = direction * step_length
            num_sub_steps = 5
            d_sub = dist / num_sub_steps
            
            curr_x, curr_y, curr_yaw = node_x, node_y, node_yaw
            
            for _ in range(num_sub_steps):
                curr_x, curr_y, curr_yaw = move_bicycle_model(
                    curr_x, curr_y, curr_yaw, d_sub, steer, config.L
                )
                x_list.append(curr_x)
                y_list.append(curr_y)
                yaw_list.append(curr_yaw)

            # 3. 计算栅格索引 (用于 A* CloseSet 判重)
            x_ind = round(curr_x / config.XY_RES)
            y_ind = round(curr_y / config.XY_RES)
            yaw_ind = round(curr_yaw / config.YAW_RES)

            # 4. 创建新节点
            # 注意：传入 updated yaw_list
            new_node = Node(x_ind, y_ind, yaw_ind, direction, 
                            x_list, y_list, yaw_list, 
                            steer, current_node.parent_index, 0.0, curr_yaw)
            
            next_nodes.append(new_node)
            
    return next_nodes

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

# --- 单元测试与可视化 ---
if __name__ == "__main__":
    print("Testing Collision Check...")
    cfg = Config()
    
    # 1. 生成带障碍物的地图 (50x50m)
    gm = GridMap(cfg)
    gm.generate_map(20, 20, 50) # 尺寸20x20米，生成50个随机障碍点
    
    # 2. 设置起点 (在地图中心附近，假设它是空的)
    # 为了防止起点正好生成在障碍物上，我们可以强制清除起点附近的障碍
    start_x_m, start_y_m = 5.0, 5.0
    sx_idx = round(start_x_m / cfg.XY_RES)
    sy_idx = round(start_y_m / cfg.XY_RES)
    # 清除起点周围的障碍物
    for i in range(-2, 3):
        for j in range(-2, 3):
            gm.obstacle_map[sx_idx+i][sy_idx+j] = False

    start_node = Node(sx_idx, sy_idx, 0, 1, 
                      [start_x_m], [start_y_m], [0.0], 
                      0.0, -1, 0.0, 0.0)

    # 3. 计算运动基元 (Motion Primitives)
    next_nodes = calc_next_states(start_node, cfg)
    
    # 4. 碰撞检测过滤
    safe_nodes = []
    collision_nodes = []
    
    for node in next_nodes:
        if not check_collision(node, gm, cfg):
            safe_nodes.append(node)
        else:
            collision_nodes.append(node)
            
    print(f"Total paths: {len(next_nodes)}")
    print(f"Safe paths: {len(safe_nodes)} (Green)")
    print(f"Collision paths: {len(collision_nodes)} (Red)")

    # 5. 绘图验证
    gm.plot_map() # 画黑色障碍物
    
    # 画碰撞的路径 (红色)
    for node in collision_nodes:
        plt.plot(node.x_list, node.y_list, "-r", alpha=0.5, linewidth=1)
        
    # 画安全的路径 (绿色)
    for node in safe_nodes:
        plt.plot(node.x_list, node.y_list, "-g", linewidth=2)
        # 画终点箭头
        plt.arrow(node.x_list[-1], node.y_list[-1], 
                  math.cos(node.yaw), math.sin(node.yaw),
                  head_width=0.3, color='g')
    
    # 画起点
    plt.plot(start_x_m, start_y_m, "ob", markersize=8, label="Start")
    
    plt.title("Collision Check Verification\nRed: Collided, Green: Safe")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.show()