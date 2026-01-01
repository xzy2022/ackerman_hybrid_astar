from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Any, Dict
import numpy as np
from src.config import HybridAStarConfig, VehicleConfig, RobotConfig

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
    # 用于制作"算法蔓延"的动画 (Closed List - 绿色点)
    expansion_history: List[Tuple[float, float, float]] = field(default_factory=list)

    # [Open Set] 记录被生成但尚未扩展的节点
    # 用于可视化搜索的前沿(Frontier) - 蓝色点
    open_list_points: List[Tuple[float, float, float]] = field(default_factory=list)

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

    注意：
    - path_yaw 和 path_direction 为"弱约束"字段：
      * 阿克曼模型规划：必须提供有效的航向和方向信息
      * 点模型规划：path_yaw 可为空列表或全为0，path_direction 可为空或全为True
    - 可视化器需要根据规划器类型处理这些字段为空的情况
    """
    # 路径坐标列表 (物理单位)
    path_x: List[float] = field(default_factory=list)  # [m]
    path_y: List[float] = field(default_factory=list)  # [m]
    path_yaw: List[float] = field(default_factory=list)# [rad] (弱约束：点模型可为空)

    # 路径上的方向指示 (True: 前进, False: 后退)
    # (弱约束：点模型或全向移动可为空或全为True)
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

        具体的实现需要结合 RobotConfig (机器人尺寸) 和地图障碍物信息进行判断。

        注意：yaw 参数保持接口一致性，但具体实现可能根据模型类型选择性使用：
        - 阿克曼模型：必须考虑 yaw 进行车辆轮廓碰撞检测
        - 点模型：可直接忽略 yaw 参数，仅使用 (x, y) 进行点/圆碰撞检测

        Args:
            x (float): 机器人中心 x 坐标 [m]
            y (float): 机器人中心 y 坐标 [m]
            yaw (float): 机器人航向角 [rad] (点模型实现中可忽略)

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

    支持多种机器人模型（阿克曼、点模型等）和规划算法。
    """
    def __init__(self, config: HybridAStarConfig, robot_config: RobotConfig):
        """
        初始化规划器。

        Args:
            config: 规划算法配置（HybridAStarConfig 或其子类）
            robot_config: 机器人配置（VehicleConfig, PointConfig 等）
        """
        self.config = config
        self.robot_config = robot_config

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