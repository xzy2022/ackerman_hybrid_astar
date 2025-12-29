import math
import matplotlib.pyplot as plt
import numpy as np
from config import Config

class Node:
    def __init__(self, x_ind, y_ind, yaw_ind, direction, x_list, y_list,
                 steer, pin_index, cost):
        """
        Hybrid A* 的节点
        :param x_ind: 栅格地图 X 索引
        :param y_ind: 栅格地图 Y 索引
        :param yaw_ind: 航向角索引
        :param direction: 移动方向 (1: 前进, -1: 后退)
        :param x_list: 轨迹上的 X 坐标列表 (用于碰撞检测和画图)
        :param y_list: 轨迹上的 Y 坐标列表
        :param steer: 到达该节点时的转向角
        :param pin_index: 父节点索引 (用于回溯路径)
        :param cost: 路径代价值 (g + h)
        """
        self.x_index = x_ind
        self.y_index = y_ind
        self.yaw_index = yaw_ind
        self.direction = direction
        self.x_list = x_list
        self.y_list = y_list
        self.steering = steer
        self.parent_index = pin_index
        self.cost = cost

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
            # 1. 预测新状态
            # 我们需要保存中间轨迹点用于碰撞检测
            x_list = [current_node.x_list[-1]]
            y_list = [current_node.y_list[-1]]
            yaw_list = [current_node.yaw_index] # 这里存的是角度值还是索引暂存疑，先存最后的物理角度更合适

            # 获取当前物理状态 (取轨迹列表的最后一个点)
            node_x = current_node.x_list[-1]
            node_y = current_node.y_list[-1]
            # 注意：Node类里通常只存了离散yaw_ind，我们需要还原回物理yaw
            # 这里为了简单，假设Node在生成时会携带真实的yaw，或者我们从yaw_ind计算
            # *为了简化Demo，建议在Node里直接存一个 self.yaw 物理值*
            node_yaw = getattr(current_node, 'yaw', 0.0) 

            # 模拟积分 (将 step_length 切分成小段 sub_step)
            # 这样画出来的线是圆弧，而不是直线
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

            # 2. 计算栅格索引 (用于去重和A*记录)
            x_ind = round(curr_x / config.XY_RES)
            y_ind = round(curr_y / config.XY_RES)
            yaw_ind = round(curr_yaw / config.YAW_RES)

            # 3. 创建新节点 (Cost暂时设为0，后面统一算)
            new_node = Node(x_ind, y_ind, yaw_ind, direction, x_list, y_list,
                            steer, current_node.parent_index, 0.0)
            new_node.yaw = curr_yaw # 补充物理角度属性
            
            next_nodes.append(new_node)
            
    return next_nodes

# --- 单元测试与可视化 ---
if __name__ == "__main__":
    print("Testing Motion Primitives...")
    cfg = Config()
    
    # 创建一个位于 (0,0), 朝向 0 的初始节点
    start_node = Node(0, 0, 0, 1, [0], [0], 0, -1, 0)
    start_node.yaw = 0.0 # 初始物理角度
    
    # 计算下一步所有可能的轨迹
    next_nodes = calc_next_states(start_node, cfg)
    
    # 可视化
    plt.figure(figsize=(8, 8))
    
    for node in next_nodes:
        # 画轨迹
        plt.plot(node.x_list, node.y_list, "-g" if node.direction == 1 else "-r")
        # 画终点箭头
        plt.arrow(node.x_list[-1], node.y_list[-1], 
                  math.cos(node.yaw) * 0.5, math.sin(node.yaw) * 0.5,
                  head_width=0.2, color='k')
                  
    plt.grid(True)
    plt.axis("equal")
    plt.title(f"Motion Primitives (Green: Fwd, Red: Rev)\nGenerated {len(next_nodes)} branches")
    plt.show()