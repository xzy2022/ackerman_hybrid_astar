from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Any, Dict
import numpy as np
from src.config import HybridAStarConfig, VehicleConfig

@dataclass
class SearchDebugData:
    """
    研究专用：调试与可观测性数据容器。
    
    用于在规划结束后回溯算法的内部状态，服务于：
    1. 论文可视化（绘制搜索树扩展范围、热力图）
    2. 算法调试（分析为何陷入局部最优）
    3. 性能分析（统计扩展节点数）
    """
    # 记录节点的被访问/扩展顺序：[(x, y, yaw), ...]
    # 用于制作"算法蔓延"的动画 (Closed List)
    expansion_history: List[Tuple[float, float, float]] = field(default_factory=list)

    # 记录 Open List 中剩余的候选节点
    # 用于可视化"搜索边界"或"候选前沿"
    open_list_points: List[Tuple[float, float]] = field(default_factory=list)

    # 记录闭集（ClosedList）中所有节点的最终代价信息
    # key: 状态唯一标识 (如 grid_index_tuple), value: g_cost + h_cost
    # 用于绘制代价分布热力图
    visited_nodes_cost: Dict[Any, float] = field(default_factory=dict)

    # 记录关键的统计指标
    nodes_expanded: int = 0
    execution_time_ms: float = 0.0

@dataclass
class PlannerResult:
    """
    规划器的标准化输出结果。
    """
    # 路径坐标列表 (物理单位)
    path_x: List[float] = field(default_factory=list)  # [m]
    path_y: List[float] = field(default_factory=list)  # [m]
    path_yaw: List[float] = field(default_factory=list)# [rad]
    
    # 路径上的方向指示 (True: 前进, False: 后退)
    path_direction: List[bool] = field(default_factory=list)
    
    # 规划状态
    success: bool = False
    
    # 路径总代价 (Cost)
    cost: float = float('inf')
    
    # 调试/科研数据包 (如果不开启 debug 模式，可能为空)
    debug_data: SearchDebugData = field(default_factory=SearchDebugData)
    
    # 严格碰撞检测结果：被碰撞的障碍物坐标列表 [(x, y), ...]
    # 用于 Ground Truth Check 和消融实验分析
    collided_obstacles_coords: List[Tuple[float, float]] = field(default_factory=list)

class BaseMap(ABC):
    """
    地图环境抽象基类。
    
    任何具体的地图实现（栅格地图、点云地图、矢量地图）都必须实现此接口。
    规划器只通过此接口与环境交互。
    """
    
    @abstractmethod
    def check_collision(self, x: float, y: float, yaw: float) -> bool:
        """
        检查给定车辆位姿(Pose)是否存在碰撞。
        
        具体的实现需要结合 VehicleConfig (车身尺寸) 和地图障碍物信息进行判断。
        
        Args:
            x (float): 车辆后轴中心 x 坐标 [m]
            y (float): 车辆后轴中心 y 坐标 [m]
            yaw (float): 车辆航向角 [rad]
            
        Returns:
            bool: True 表示发生碰撞或越界(不可通行)，False 表示安全。
        """
        pass
    
    @abstractmethod
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        获取地图的物理边界，用于判断越界。
        
        Returns:
            Tuple: (min_x, max_x, min_y, max_y) 单位 [m]
        """
        pass

class BaseHeuristic(ABC):
    """
    启发式算法接口 (Strategy Pattern)。
    
    允许通过配置轻松切换不同的启发式策略（如：欧几里得距离、Holonomic-Dijkstra、Reed-Shepp 等）。
    """
    
    @abstractmethod
    def calculate(self, current_pose: Tuple[float, float, float], 
                  goal_pose: Tuple[float, float, float]) -> float:
        """
        计算从当前状态到目标状态的估计代价 h(n)。
        
        Args:
            current_pose: 当前车辆位姿 (x, y, yaw) [m, m, rad]
            goal_pose: 目标车辆位姿 (x, y, yaw) [m, m, rad]
            
        Returns:
            float: 启发式代价值 (非负数)
        """
        pass

class BasePlanner(ABC):
    """
    路径规划器基类。
    """
    def __init__(self, config: HybridAStarConfig, vehicle_config: VehicleConfig):
        self.config = config
        self.vehicle_config = vehicle_config

    @abstractmethod
    def plan(self, start: Tuple[float, float, float], 
             goal: Tuple[float, float, float], 
             map_env: BaseMap) -> PlannerResult:
        """
        执行路径规划任务。
        
        Args:
            start: 起点位姿 (x, y, yaw) [m, m, rad]
            goal: 终点位姿 (x, y, yaw) [m, m, rad]
            map_env: 实现了 BaseMap 接口的地图环境对象
            
        Returns:
            PlannerResult: 包含路径、成功标志及调试数据的对象
        """
        pass