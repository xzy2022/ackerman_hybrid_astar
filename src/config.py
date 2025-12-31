import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple

# --- 碰撞检测方法常量 ---
class CollisionMethod:
    """碰撞检测方法枚举"""
    CIRCLE = "circle"       # 圆形近似 (速度快，精度低)
    POLYGON = "polygon"     # 多边形几何 (速度慢，精度极高)
    FOOTPRINT = "footprint" # 栅格查表 (速度极快，精度中高，待实现)

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

        # 3. 计算碰撞检测圆参数（修正版）
        # 优化策略：使用更多的小圆，而非少而大的圆

        # [修正点 1]：增加安全余量 (Safety Margin)
        # 1.0 (半宽) + 0.15 (覆盖角的对角线增量 + 少量安全缓冲)
        # 建议值为 1.1 到 1.2 之间
        self.collision_radius = self.width / 2.0 + 0.38  # 1.15m（对宽度2.0m的车）

        # [修正点 2]：调整圆心分布
        # 增加圆的数量是好事，但要注意首尾圆的位置
        num_circles = 7

        # 让首尾圆心距离车头/车尾大约 radius * 0.5 的位置
        # 这样可以避免圆头突出车身太长（变得像个毛毛虫）
        # 依靠增大的半径去覆盖车头车尾
        self.collision_offsets = np.linspace(
            -self.rear_hang + self.collision_radius * 0.5,
            self.front_hang - self.collision_radius * 0.5,
            num_circles
        )

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

    # --- 运动学积分与碰撞检测 ---
    step_interpolation: float = 0.1    # [m] 运动学积分的微元长度（越小轨迹越精确，但计算量越大）
    collision_check_interval: float = 0.1  # [m] 碰撞检测间隔（应小于障碍物最小尺寸，防止穿墙）

    # 碰撞检测方法选择
    # 可选值: CollisionMethod.CIRCLE, CollisionMethod.POLYGON, CollisionMethod.FOOTPRINT
    collision_method: str = CollisionMethod.FOOTPRINT

    # --- 查表法专用参数 ---
    # 查找表的角度分辨率 [deg] (越小越精确，但预计算稍慢，内存稍大。推荐 1.0 或 2.0 度)
    footprint_yaw_res_deg: float = 2.0

    # 查表法膨胀系数 [m]
    # 用于补偿 "车辆实际中心" 与 "栅格中心" 的对齐误差 (最大误差 ≈ resolution * 0.7)
    # 建议设为 resolution * 0.5 左右
    footprint_padding: float = 0.5 * xy_resolution
    
    # --- 代价权重 (Cost Weights) ---
    # 将原来的魔法数值提取为可配置项，方便做参数敏感性分析
    heuristic_weight: float = 1.05      # 启发式代价(H值)的权重
    penalty_reverse: float = 50.0       # 倒车惩罚
    penalty_steer_change: float = 5.0  # 频繁打方向盘的惩罚
    penalty_gear_switch: float = 5.0   # 换挡(前进变后退)的惩罚
    
    # --- 其他 ---
    extend_area: float = 0.0     # [m] 碰撞检测额外延展距离

    # --- [新增] 调试与可视化性能控制 ---
    # 是否记录扩展历史 (OpenList 中弹出的节点，用于绘制搜索树)
    # 默认关闭以提升性能，需要可视化时通过 --log-tree 开启
    record_expansion_history: bool = False

    # 是否记录闭集代价 (ClosedList，用于绘制热力图或分析)
    # 默认关闭以节省内存，需要热力图时通过 --log-closed 开启
    record_visited_costs: bool = False

    # 采样频率：每隔 N 个节点记录一次 (1 表示全部记录，10 表示记录 1/10)
    # 增大此值可显著提升大规模搜索时的规划速度和绘图速度
    debug_sample_rate: int = 10

    # --- 终止条件阈值 ---
    goal_dist_threshold: float = 1.0       # [m] 终点距离阈值
    goal_yaw_threshold_deg: float = 10.0   # [deg] 终点航向角阈值

    # --- 派生属性 ---
    yaw_resolution: float = field(init=False) # [rad]
    step_size: float = field(init=False)      # [m] 实际物理步长。在运动学中被切分为多个微元（step_interpolation）来实现
    goal_yaw_threshold: float = field(init=False) # [rad] 终点航向角阈值(弧度)

    def __post_init__(self):
        self.yaw_resolution = math.radians(self.yaw_resolution_deg)
        self.step_size = self.xy_resolution * self.move_step_grid
        self.goal_yaw_threshold = math.radians(self.goal_yaw_threshold_deg)