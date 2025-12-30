import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass
class VehicleConfig:
    """
    车辆物理参数配置
    用于描述阿克曼转向车辆的几何尺寸和运动学限制。
    """
    # --- 基础几何参数 ---
    wheelbase: float = 2.5       # [m] 轴距 (L/WB)
    width: float = 2.0           # [m] 车宽 (W)
    front_hang: float = 3.3      # [m] 前悬 (LF): 后轴中心到车头距离
    rear_hang: float = 1.0       # [m] 后悬 (LB): 后轴中心到车尾距离
    
    # --- 运动学限制 ---
    max_steer_deg: float = 35.0  # [deg] 最大前轮转向角 (输入用角度，方便阅读)
    
    # --- 派生属性 (由 __post_init__ 自动计算，不要手动赋值) ---
    max_steer: float = field(init=False)  # [rad]
    vehicle_outline: np.ndarray = field(init=False) # 用于绘图的轮廓点
    collision_radius: float = field(init=False)     # [m] 碰撞检测圆半径
    collision_offsets: List[float] = field(init=False) # [m] 碰撞检测圆圆心偏移量(相对于后轴)

    def __post_init__(self):
        """在初始化后自动计算派生参数"""
        # 1. 角度转弧度
        self.max_steer = math.radians(self.max_steer_deg)

        # 2. 生成车辆轮廓 (用于绘图)
        # 坐标系原点：后轴中心
        # 顺序：右前 -> 左前 -> 左后 -> 右后 -> 右前
        self.vehicle_outline = np.array([
            [self.front_hang, self.front_hang, -self.rear_hang, -self.rear_hang, self.front_hang],
            [self.width / 2, -self.width / 2, -self.width / 2, self.width / 2, self.width / 2]
        ])

        # 3. 计算碰撞检测圆参数 (3圆覆盖模型)
        # 半径 = 车宽一半 + 安全余量(0.1m)
        self.collision_radius = self.width / 2.0 + 0.2
        
        # 圆心位置：后轴(0)，轴距中点(L/2)，前轴(L)
        # 如果车辆更长，可以在此修改逻辑增加圆的数量
        self.collision_offsets = [0.0, self.wheelbase / 2.0, self.wheelbase]
        self.collision_offsets = [-self.rear_hang / 2, 0.0, self.wheelbase / 2.0, self.wheelbase, 0.9 * self.front_hang] 

@dataclass
class HybridAStarConfig:
    """
    混合A* 算法参数配置
    用于控制搜索行为、分辨率和代价函数权重。
    """
    # --- 分辨率参数 ---
    xy_resolution: float = 0.5   # [m] 栅格地图分辨率
    yaw_resolution_deg: float = 15.0 # [deg] 航向角离散化分辨率
    
    # --- 搜索步长 ---
    # 每次扩展的步长倍率 (step_length = xy_resolution * move_step_grid)
    move_step_grid: float = 2.0  
    
    # --- 代价权重 (Cost Weights) ---
    # 将原来的魔法数值提取为可配置项，方便做参数敏感性分析
    heuristic_weight: float = 5.0      # 启发式代价(H值)的权重
    penalty_reverse: float = 50.0       # 倒车惩罚
    penalty_steer_change: float = 5.0  # 频繁打方向盘的惩罚
    penalty_gear_switch: float = 5.0   # 换挡(前进变后退)的惩罚
    
    # --- 其他 ---
    extend_area: float = 0.0     # [m] 碰撞检测额外延展距离
    
    # --- 派生属性 ---
    yaw_resolution: float = field(init=False) # [rad]
    step_size: float = field(init=False)      # [m] 实际物理步长

    def __post_init__(self):
        self.yaw_resolution = math.radians(self.yaw_resolution_deg)
        self.step_size = self.xy_resolution * self.move_step_grid