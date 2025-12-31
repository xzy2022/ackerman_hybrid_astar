import heapq
import math
import time
import numpy as np
from typing import List, Tuple, Dict, Optional, Any

# 导入配置与接口
from src.config import HybridAStarConfig, VehicleConfig
from src.interfaces import BasePlanner, BaseMap, PlannerResult, SearchDebugData
from src.heuristic import HolonomicHeuristic
from src.grid_map import GridMap

class Node:
    """
    Hybrid A* 搜索树节点。
    存储状态、路径片段、父节点引用以及代价值。
    """
    def __init__(self, 
                 x_ind: int, y_ind: int, yaw_ind: int, 
                 direction: int,
                 x_list: List[float], y_list: List[float], yaw_list: List[float],
                 steer: float,
                 parent: 'Node' = None,
                 g_cost: float = 0.0, f_cost: float = 0.0):
        
        # 离散状态索引 (用于 ClosedList 判重)
        self.x_index = x_ind
        self.y_index = y_ind
        self.yaw_index = yaw_ind
        
        # 动作与轨迹
        self.direction = direction # 1: Forward, -1: Backward
        self.x_list = x_list       # 这一步的轨迹片段
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.steering = steer      # 到达此节点所用的转向角
        
        # 树结构
        self.parent = parent
        
        # 代价
        self.g_cost = g_cost       # 起点到当前的实际代价
        self.f_cost = f_cost       # g + h

    # 堆排序依据 (f_cost 越小越优先)
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class HybridAStarPlanner(BasePlanner):
    """
    混合 A* 路径规划器实现。
    """
    
    def plan(self, start: Tuple[float, float, float], 
             goal: Tuple[float, float, float], 
             map_env: BaseMap) -> PlannerResult:
        """
        [实现 BasePlanner 接口] 主规划循环
        """
        start_time = time.time()
        
        # 1. 确保地图类型正确 (因为我们需要用 GridMap 初始化启发式)
        # 如果传入的是通用 BaseMap，可能需要适配，这里假设是 GridMap
        if not isinstance(map_env, GridMap):
            raise TypeError("HybridAStarPlanner requires a GridMap instance.")
            
        # 2. 初始化启发式算法 (Strategy Injection)
        # 这里我们在 plan 内部实例化，确保使用最新的地图
        heuristic_algo = HolonomicHeuristic(self.config, map_env)
        
        # 3. 初始化起点和终点节点
        sx, sy, syaw = start
        gx, gy, gyaw = goal
        
        start_node = self._create_node(sx, sy, syaw, 1, 0.0, None, 0.0)
        # 计算初始 h 值
        h_start = heuristic_algo.calculate((sx, sy, syaw), goal)
        start_node.f_cost = start_node.g_cost + h_start * self.config.heuristic_weight
        
        # 4. 初始化 OpenList 和 ClosedList
        open_list = []
        heapq.heappush(open_list, start_node)
        
        closed_list = {} # Key: (x_ind, y_ind, yaw_ind) -> Node
        closed_list[(start_node.x_index, start_node.y_index, start_node.yaw_index)] = start_node
        
        # 5. 初始化调试数据
        debug_data = SearchDebugData()
        
        print(f"Planning start: {start} -> {goal}")
        
        # --- 主循环 ---
        while True:
            # A. 检查 OpenList 是否为空
            if not open_list:
                print("Fail: Open set is empty.")
                return PlannerResult(success=False, debug_data=debug_data)
            
            # B. 弹出优先级最高的节点
            current = heapq.heappop(open_list)
            debug_data.nodes_expanded += 1
            
            # [Debug] 记录扩展历史 (取轨迹片段的最后一个点)
            debug_data.expansion_history.append((current.x_list[-1], current.y_list[-1], current.yaw_list[-1]))
            
            # C. 判断是否到达终点 (Goal Check)
            if self._is_goal_reached(current, goal):
                print(f"Goal Reached! Cost: {current.f_cost:.2f}")
                final_path = self._trace_path(current)
                
                # 填充统计数据
                debug_data.execution_time_ms = (time.time() - start_time) * 1000
                debug_data.visited_nodes_cost = {k: v.f_cost for k, v in closed_list.items()}
                
                final_path.debug_data = debug_data
                return final_path
            
            # D. 扩展子节点 (Expand)
            next_nodes = self._expand_node(current)
            
            for neighbor in next_nodes:
                # D.1 碰撞检测 (Map Check)
                # 取轨迹片段的每一点进行检测，确保整段轨迹安全
                is_collision = False

                # 基于物理距离的采样检测（防止穿墙效应）
                check_interval = self.config.collision_check_interval  # [m] 检测间隔
                total_points = len(neighbor.x_list)

                # 计算每个点代表的物理距离
                dist_per_point = self.config.step_size / total_points

                # 计算需要跳过多少个点才能满足检测间隔
                step_gap = max(1, int(round(check_interval / dist_per_point)))

                for i in range(0, total_points, step_gap):
                    if map_env.check_collision(neighbor.x_list[i], neighbor.y_list[i], neighbor.yaw_list[i]):
                        is_collision = True
                        break

                # 确保终点一定被检测（可能因 step_gap 被跳过）
                if not is_collision:
                    if map_env.check_collision(neighbor.x_list[-1], neighbor.y_list[-1], neighbor.yaw_list[-1]):
                        is_collision = True

                if is_collision:
                    continue
                
                # D.2 计算启发式代价 h(n)
                # 取节点末端状态计算 h
                neighbor_pose = (neighbor.x_list[-1], neighbor.y_list[-1], neighbor.yaw_list[-1])
                h = heuristic_algo.calculate(neighbor_pose, goal)
                
                # D.3 计算总代价 f(n) = g(n) + w * h(n)
                neighbor.f_cost = neighbor.g_cost + self.config.heuristic_weight * h
                
                # D.4 CloseList 查重与更新
                state_key = (neighbor.x_index, neighbor.y_index, neighbor.yaw_index)
                
                if state_key in closed_list:
                    # 如果已存在且旧代价更低，跳过
                    if closed_list[state_key].f_cost <= neighbor.f_cost:
                        continue
                
                # 加入队列
                heapq.heappush(open_list, neighbor)
                closed_list[state_key] = neighbor

    def _create_node(self, x, y, yaw, direction, steer, parent, g_cost, traj_x=None, traj_y=None, traj_yaw=None):
        """辅助函数：封装节点创建与索引计算"""
        x_ind = round(x / self.config.xy_resolution)
        y_ind = round(y / self.config.xy_resolution)
        yaw_ind = round(yaw / self.config.yaw_resolution)
        
        # 如果没有提供轨迹片段，默认为单点
        if traj_x is None:
            traj_x, traj_y, traj_yaw = [x], [y], [yaw]
            
        return Node(x_ind, y_ind, yaw_ind, direction, 
                    traj_x, traj_y, traj_yaw, steer, 
                    parent, g_cost, 0.0)

    def _is_goal_reached(self, node: Node, goal: Tuple[float, float, float]) -> bool:
        """判断是否满足终止条件"""
        gx, gy, gyaw = goal
        dx = node.x_list[-1] - gx
        dy = node.y_list[-1] - gy
        dist = math.hypot(dx, dy)

        # 角度差需标准化到 [-pi, pi]
        dyaw = node.yaw_list[-1] - gyaw
        while dyaw >= math.pi: dyaw -= 2*math.pi
        while dyaw < -math.pi: dyaw += 2*math.pi

        # 从配置中读取阈值
        return dist <= self.config.goal_dist_threshold and abs(dyaw) <= self.config.goal_yaw_threshold

    def _expand_node(self, current: Node) -> List[Node]:
        """
        [核心逻辑] 生成子节点
        包含：动作采样 -> 运动学积分 -> 代价计算
        """
        next_nodes = []
        
        # 1. 采样动作空间 (Steering)
        # 动态生成：左转、直线、右转
        steer_inputs = [0.0]
        max_steer = self.vehicle_config.max_steer
        # 采样数量，例如左右各采样 2 个角度
        num_steer_sample = 2
        for i in range(num_steer_sample):
            s = max_steer * (i + 1) / num_steer_sample
            steer_inputs.append(s)
            steer_inputs.append(-s)
            
        # 2. 采样方向 (Direction: Forward/Backward)
        directions = [1, -1] # 1: Forward, -1: Backward
        
        # 获取当前物理状态 (取上一段轨迹的终点)
        c_x = current.x_list[-1]
        c_y = current.y_list[-1]
        c_yaw = current.yaw_list[-1]
        
        # 3. 遍历所有动作组合
        for d in directions:
            for steer in steer_inputs:
                # 3.1 运动学推演 (Kinematic Simulation)
                # 生成一段微小的轨迹片段
                final_x, final_y, final_yaw, traj_x, traj_y, traj_yaw, move_dist = \
                    self._simulate_motion(c_x, c_y, c_yaw, d, steer)
                
                # 3.2 计算 G-Cost (Cost Function)
                added_cost = self._calculate_cost(current, d, steer, move_dist)
                new_g_cost = current.g_cost + added_cost
                
                # 3.3 创建新节点
                new_node = self._create_node(
                    final_x, final_y, final_yaw, d, steer,
                    current, new_g_cost, traj_x, traj_y, traj_yaw
                )
                
                next_nodes.append(new_node)
                
        return next_nodes

    def _simulate_motion(self, x, y, yaw, direction, steer):
        """
        [物理引擎] 自行车模型积分
        使用固定积分分辨率保证轨迹精度的一致性
        """
        step_len = self.config.step_size # 配置中的扩展步长

        # 基于固定分辨率计算积分步数（而非硬编码 sub_steps=5）
        step_interp = self.config.step_interpolation  # [m] 每次积分的微元长度
        sub_steps = math.ceil(step_len / step_interp)  # 向上取整确保覆盖全长

        d_sub = (step_len * direction) / sub_steps
        L = self.vehicle_config.wheelbase

        traj_x, traj_y, traj_yaw = [], [], []
        curr_x, curr_y, curr_yaw = x, y, yaw

        for _ in range(sub_steps):
            # Bicycle Model
            curr_x += d_sub * math.cos(curr_yaw)
            curr_y += d_sub * math.sin(curr_yaw)
            curr_yaw += d_sub * math.tan(steer) / L
            
            # Normalize Yaw
            while curr_yaw >= math.pi: curr_yaw -= 2*math.pi
            while curr_yaw < -math.pi: curr_yaw += 2*math.pi
            
            traj_x.append(curr_x)
            traj_y.append(curr_y)
            traj_yaw.append(curr_yaw)
            
        return curr_x, curr_y, curr_yaw, traj_x, traj_y, traj_yaw, step_len

    def _calculate_cost(self, current_node: Node, direction: int, steer: float, length: float) -> float:
        """
        [代价函数] Cost Function Design
        这里是论文创新的高发区。
        """
        cost = length
        
        # 1. 倒车惩罚 (Penalty for backward driving)
        if direction == -1:
            cost += length * self.config.penalty_reverse
            
        # 2. 换挡惩罚 (Penalty for gear switch)
        if direction != current_node.direction:
            cost += self.config.penalty_gear_switch
            
        # 3. 转向惩罚 (Penalty for steering)
        cost += abs(steer) * 0.1 # 轻微惩罚大转向，鼓励走直线
        
        # 4. 频繁打舵惩罚 (Penalty for steering change)
        # 鼓励平滑的转向，避免蛇形走位
        cost += abs(steer - current_node.steering) * self.config.penalty_steer_change
        
        return cost

    def _trace_path(self, end_node: Node) -> PlannerResult:
        """从终点回溯生成完整路径"""
        path_x, path_y, path_yaw, path_dir = [], [], [], []
        curr = end_node
        
        while curr.parent is not None:
            # 注意：x_list 是从父节点到子节点的轨迹
            # 回溯时需要反转这一小段，然后添加到总路径中
            path_x.extend(curr.x_list[::-1])
            path_y.extend(curr.y_list[::-1])
            path_yaw.extend(curr.yaw_list[::-1])
            # 记录方向 (bool: True为前进)
            d_bool = True if curr.direction == 1 else False
            path_dir.extend([d_bool] * len(curr.x_list))
            
            curr = curr.parent
            
        # 此时得到的路径是从 Goal -> Start 的，需要再次完全反转
        result = PlannerResult(
            path_x=path_x[::-1],
            path_y=path_y[::-1],
            path_yaw=path_yaw[::-1],
            path_direction=path_dir[::-1],
            success=True,
            cost=end_node.f_cost
        )
        return result