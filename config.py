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
        
        # 角度离散化分辨率 (用于Hybrid A*的三维搜索)
        self.YAW_RES = math.radians(15.0)  # [rad]
        
        # 碰撞检测时的延展安全距离 (可选)
        self.EXTEND_AREA = 0.0 

        # --- 导出车辆几何轮廓 (用于绘图) ---
        # 这是一个简单的矩形轮廓定义 [x, y, x, y...]
        self.vehicle_outline = np.array([
            [self.LF, self.LF, -self.LB, -self.LB, self.LF],
            [self.W / 2, -self.W / 2, -self.W / 2, self.W / 2, self.W / 2]
        ])