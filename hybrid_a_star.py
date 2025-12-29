import math
import matplotlib.pyplot as plt
import numpy as np
from config import Config

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
            node_yaw = yaw_list[-1]  # 从历史列表中取最后一个角度，更加一致

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

# --- 单元测试与可视化 ---
if __name__ == "__main__":
    print("Testing Motion Primitives with updated Node class...")
    cfg = Config()
    
    # 初始化起点：
    # 注意：yaw_list 初始化为 [0.0]
    start_node = Node(x_ind=0, y_ind=0, yaw_ind=0, direction=1, 
                      x_list=[0.0], y_list=[0.0], yaw_list=[0.0], 
                      steer=0.0, pin_index=-1, cost=0.0, yaw=0.0)
    
    # 计算下一步所有可能的轨迹
    next_nodes = calc_next_states(start_node, cfg)
    
    # 可视化
    plt.figure(figsize=(8, 8))
    
    for node in next_nodes:
        # 画轨迹
        plt.plot(node.x_list, node.y_list, "-g" if node.direction == 1 else "-r")
        
        # 画终点箭头 (使用 node.yaw 绘制正确的朝向)
        plt.arrow(node.x_list[-1], node.y_list[-1], 
                  math.cos(node.yaw) * 0.5, math.sin(node.yaw) * 0.5,
                  head_width=0.1, color='k')
                  
    plt.grid(True)
    plt.axis("equal")
    plt.title(f"Motion Primitives with Yaw Tracking\nGenerated {len(next_nodes)} branches")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.show()