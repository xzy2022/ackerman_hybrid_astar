import math
import numpy as np

class Config:
    def __init__(self):
        # --- 车辆物理参数 ---
        # 轴距 (Wheelbase): 前轴和后轴之间的距离
        self.L = 2.5  # [m]
        self.WB = self.L  # 别名，方便公式对应
        
        # 车辆宽度
        self.W = 2.0  # [m]
        
        # 后轴中心到车头的距离 (Front Hang)
        self.LF = 3.3  # [m] 
        # 后轴中心到车尾的距离 (Rear Hang)
        self.LB = 1.0  # [m]
        
        # 最大前轮转向角 (转换为弧度)
        self.MAX_STEER = math.radians(35.0)  # [rad]
        
        # --- 地图与搜索参数 ---
        # 栅格地图分辨率
        self.XY_RES = 0.5  # [m]

        # 车辆移动步长 (每次扩展的距离，以分辨率为单位)
        # 如果设为 2.0，代表每次走 2个格子
        self.move_step_size = 1.6
        
        # 角度离散化分辨率 (用于Hybrid A*的三维搜索)
        self.YAW_RES = math.radians(15.0)  # [rad]
        
        # 碰撞检测时的延展安全距离 (可选)
        self.EXTEND_AREA = 0.0 

        # --- 导出车辆几何轮廓 (用于绘图) ---
        # 这是一个简单的矩形轮廓定义 [x, y, x, y...]
        # 这里的坐标系原点是车辆的【后轴中心】
        self.vehicle_outline = np.array([
            [self.LF, self.LF, -self.LB, -self.LB, self.LF],
            [self.W / 2, -self.W / 2, -self.W / 2, self.W / 2, self.W / 2]
        ])

        # --- 碰撞检测参数 (3圆覆盖法) ---
        # 使用多个圆来近似矩形车身，可以极大简化碰撞检测计算（将多边形求交简化为点到圆心的距离判断）
        
        # 1. 计算圆的半径
        # 半径通常取车宽的一半，加上一点余量(0.1m)以容纳车角
        # 这样即使车身旋转，只要圆不碰到障碍物，车身大部分区域就是安全的
        self.collision_circle_radius = self.W / 2.0 + 0.1 

        # 2. 计算圆心的位置 (相对于后轴中心)
        # 这里采用简化的 3 圆模型：
        # - 第1个圆: 在后轴 (x=0)
        # - 第2个圆: 在轴距中间 (x=L/2)
        # - 第3个圆: 在前轴 (x=L)
        # 注：对于特别长的车头(LF)或车尾(LB)，可能需要调整这些位置或增加圆的数量
        self.collision_circle_offsets = [0.0, self.L / 2.0, self.L]