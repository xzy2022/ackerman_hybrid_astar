import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mpath
from typing import Tuple, Optional, List, Dict
from shapely.geometry import Polygon, box
from shapely.affinity import rotate

# 导入接口与配置
from src.interfaces import BaseMap
from src.config import HybridAStarConfig, VehicleConfig, RobotConfig, CollisionMethod

class GridMap(BaseMap):
    """
    基于栅格的地图实现 (Grid Map Implementation).
    
    修正版 (SSOT): 统一了物理坐标与栅格索引的转换逻辑。
    - 索引 (i, j) 对应的物理范围是 [i*res, (i+1)*res)
    - 物理中心位于 (i+0.5)*res
    """

    def __init__(self, config: HybridAStarConfig, robot_config: RobotConfig):
        """
        初始化栅格地图。

        Args:
            config: 包含分辨率等地图参数
            robot_config: 包含机器人尺寸，用于内部的碰撞检测计算
        """
        self.config = config
        self.robot_config = robot_config

        # 地图尺寸 (索引单位)
        self.width_idx = 0
        self.height_idx = 0

        # 地图物理边界 (米)
        self.width_m = 0.0
        self.height_m = 0.0

        # 障碍物数据 (W x H)
        # 初始化为空，需要调用 generate_random_map 或 load_from_image 填充
        self.obstacle_map: Optional[np.ndarray] = None

        # [新增] 查找表 (Footprint Table)
        # Dict[int_deg, List[Tuple[int, int]]]
        # key: 离散角度 (度数)，value: 车辆占用的栅格相对偏移量列表
        self.footprint_table: Dict[int, List[Tuple[int, int]]] = {}
        self.footprint_yaw_res = config.footprint_yaw_res_deg  # 记录分辨率

        # 如果配置选择了 FOOTPRINT 且不是点模型，则执行预计算
        # 点模型不需要 footprint table（直接查表）
        if (self.config.collision_method == CollisionMethod.FOOTPRINT and
            self.robot_config.model_type.value != "point"):
            self._init_collision_lookup_table()

    # =========================================================
    #  核心转换逻辑 (Single Source of Truth)
    #  所有涉及坐标转换的地方都必须调用这两个方法
    # =========================================================
    
    def get_index_from_pos(self, x: float, y: float) -> Tuple[int, int]:
        """
        世界坐标 -> 栅格索引 (向下取整)
        Ex: res=1.0, pos=0.9 -> index=0; pos=1.1 -> index=1
        """
        # 使用 floor 确保正数区间内的逻辑一致性
        x_idx = int(math.floor(x / self.config.xy_resolution))
        y_idx = int(math.floor(y / self.config.xy_resolution))
        return x_idx, y_idx

    def get_pos_from_index(self, x_idx: int, y_idx: int) -> Tuple[float, float]:
        """
        栅格索引 -> 世界坐标中心
        Ex: res=1.0, index=0 -> pos=0.5
        用于：可视化画点、A* 节点中心坐标计算、碰撞检测精确距离计算
        """
        x = (x_idx + 0.5) * self.config.xy_resolution
        y = (y_idx + 0.5) * self.config.xy_resolution
        return x, y

    # =========================================================
    #  业务逻辑：碰撞检测入口
    # =========================================================

    def check_collision(self, x: float, y: float, yaw: float) -> bool:
        """
        [实现 BaseMap 接口]
        检查车辆在特定位姿下是否与障碍物碰撞。

        根据 robot_config.model_type 分发到具体实现：
        - POINT: 直接查表，忽略 yaw (O(1) 复杂度)
        - ACKERMANN: 根据 config.collision_method 分发
        """
        if self.obstacle_map is None:
            return True # 地图未初始化视为不可通行

        # [新增] 点模型快速通道
        if self.robot_config.model_type.value == "point":
            return self._check_collision_point(x, y)

        # 阿克曼模型：根据 collision_method 分发
        method = self.config.collision_method

        if method == CollisionMethod.CIRCLE:
            return self._check_collision_circle(x, y, yaw)
        elif method == CollisionMethod.POLYGON:
            return self._check_collision_polygon(x, y, yaw)
        elif method == CollisionMethod.FOOTPRINT:
            return self._check_collision_footprint(x, y, yaw)
        else:
            # 默认回退到圆形检测
            return self._check_collision_circle(x, y, yaw)

    # =========================================================
    #  具体检测策略实现
    # =========================================================

    def _check_collision_point(self, x: float, y: float) -> bool:
        """
        [点模型专用碰撞检测]
        极速 O(1) 查表，忽略 yaw 参数。

        逻辑：
        1. 将点 (x, y) 视为一个半径为 robot_config.radius 的圆
        2. 检查该圆覆盖的所有栅格是否为障碍物
        3. 不考虑航向角，点模型全向移动

        性能优化：
        - 直接查表，无角度计算
        - 仅检查圆覆盖的栅格范围
        """
        radius = self.robot_config.radius
        x_idx, y_idx = self.get_index_from_pos(x, y)

        # 计算需要检查的栅格半径
        radius_idx = math.ceil(radius / self.config.xy_resolution)

        # 遍历圆覆盖的所有栅格
        for dx in range(-radius_idx, radius_idx + 1):
            for dy in range(-radius_idx, radius_idx + 1):
                # 检查是否在圆内
                if dx*dx + dy*dy > radius_idx*radius_idx:
                    continue

                check_x = x_idx + dx
                check_y = y_idx + dy

                # 越界检查
                if (check_x < 0 or check_x >= self.width_idx or
                    check_y < 0 or check_y >= self.height_idx):
                    return True

                # 查表检查障碍物
                if self.obstacle_map[check_x][check_y]:
                    return True

        return False

    def _check_collision_circle(self, x: float, y: float, yaw: float) -> bool:
        """
        策略1: 圆形近似检测 (原始逻辑)
        优点: 极快
        缺点: 存在覆盖漏洞或过度保守
        详细说明：直观的做法是判断距离D是否小于半径R+分辨率/根号2，但这会导致过于保守。
        实际上只比较距离D和半径R，因为工程上D会考虑安全裕量，这样可以减少大量的碰撞检测计算。
        """
        # 遍历车身上的每一个碰撞检测圆 (由 VehicleConfig 定义)
        for offset in self.robot_config.collision_offsets:
            # 1. 计算圆心在世界坐标系的位置
            cx = x + offset * math.cos(yaw)
            cy = y + offset * math.sin(yaw)

            # 2. 转换为栅格索引 (使用统一接口)
            cx_idx, cy_idx = self.get_index_from_pos(cx, cy)

            # 3. 越界检查 (Out of bounds check)
            if (cx_idx < 0 or cx_idx >= self.width_idx or
                cy_idx < 0 or cy_idx >= self.height_idx):
                return True

            # 4. [核心改进] 动态计算搜索半径
            # 确保搜索范围覆盖整个物理圆
            search_radius_idx = math.ceil(self.robot_config.collision_radius / self.config.xy_resolution)

            # 5. 遍历覆盖圆的所有潜在栅格
            for i in range(-search_radius_idx, search_radius_idx + 1):
                for j in range(-search_radius_idx, search_radius_idx + 1):
                    nx, ny = cx_idx + i, cy_idx + j

                    # 边界检查
                    if 0 <= nx < self.width_idx and 0 <= ny < self.height_idx:
                        if self.obstacle_map[nx][ny]:
                            # 障碍物中心坐标
                            obs_x, obs_y = self.get_pos_from_index(nx, ny)

                            # 计算距离 (圆心到障碍物中心)
                            dist = math.hypot(cx - obs_x, cy - obs_y)

                            # 判定碰撞
                            # 考虑到栅格是对角线覆盖，用 radius + resolution/2 稍微保守一点
                            # 但这里用纯半径判断已经足够精确（因为我们遍历了所有覆盖的栅格）
                            if dist <= self.robot_config.collision_radius:
                                return True

        return False # 通过所有检查，无碰撞

    def _check_collision_polygon(self, x: float, y: float, yaw: float) -> bool:
        """
        策略2: 多边形精确检测 (Shapely)
        优点: 100% 几何精确，Ground Truth
        缺点: 每次调用都要进行多边形相交运算，比圆检测慢 10-50 倍
        """
        # 1. 构建车辆 Shapely 多边形
        outline = self.robot_config.vehicle_outline
        rot = np.array([
            [math.cos(yaw), math.sin(yaw)],
            [-math.sin(yaw), math.cos(yaw)]
        ])
        rotated_outline = (outline.T.dot(rot)).T
        rotated_outline[0, :] += x
        rotated_outline[1, :] += y

        # 转换为顶点列表（去除重复的最后一个点）
        vehicle_vertices = []
        for j in range(rotated_outline.shape[1] - 1):  # -1 去掉重复的起点
            vehicle_vertices.append((rotated_outline[0, j], rotated_outline[1, j]))

        vehicle_polygon = Polygon(vehicle_vertices)

        # 2. 计算包围盒 (AABB)，快速筛选潜在的障碍物栅格
        min_x_idx = max(0, int(math.floor(np.min(rotated_outline[0, :]) / self.config.xy_resolution)))
        max_x_idx = min(self.width_idx - 1, int(math.ceil(np.max(rotated_outline[0, :]) / self.config.xy_resolution)))
        min_y_idx = max(0, int(math.floor(np.min(rotated_outline[1, :]) / self.config.xy_resolution)))
        max_y_idx = min(self.height_idx - 1, int(math.ceil(np.max(rotated_outline[1, :]) / self.config.xy_resolution)))

        # 3. 遍历包围盒内的所有格子
        res = self.config.xy_resolution
        half_res = res / 2.0

        for ix in range(min_x_idx, max_x_idx + 1):
            for iy in range(min_y_idx, max_y_idx + 1):
                # 只有当该格子是障碍物时，才进行昂贵的几何相交检查
                if self.obstacle_map[ix][iy]:
                    # 获取该障碍物格子的物理中心
                    obs_center_x, obs_center_y = self.get_pos_from_index(ix, iy)

                    # 构建格子的几何对象 (box)
                    grid_box = box(obs_center_x - half_res, obs_center_y - half_res,
                                   obs_center_x + half_res, obs_center_y + half_res)

                    # 4. 精确检测
                    if vehicle_polygon.intersects(grid_box):
                        return True

        return False

    def _init_collision_lookup_table(self):
        """
        预计算车辆在不同角度下的栅格占用掩码 (Footprint Mask)。
        结果存储在 self.footprint_table 中。
        """
        print(f"[GridMap] Pre-computing footprint table (Res: {self.footprint_yaw_res} deg)...")

        # 1. 准备基础多边形 (以车辆后轴中心为原点 (0,0))
        # 使用 shapely 构建
        outline = self.robot_config.vehicle_outline
        base_coords = list(zip(outline[0, :], outline[1, :]))
        # 去重
        if base_coords[0] == base_coords[-1]:
            base_coords.pop()
        vehicle_poly = Polygon(base_coords)

        # [关键技巧]：预先膨胀多边形
        # 补偿"车辆中心"与"栅格中心"不重合带来的量化误差
        # 同时也可以作为安全缓冲
        padding = getattr(self.config, 'footprint_padding', 0.1)
        if padding > 0:
            vehicle_poly = vehicle_poly.buffer(padding, join_style=2)  # join_style=2 (mitre) 保持棱角

        # 2. 遍历所有离散角度 (0, 360)
        num_angles = int(360 / self.footprint_yaw_res)
        res = self.config.xy_resolution
        half_res = res / 2.0

        for i in range(num_angles):
            deg = i * self.footprint_yaw_res
            yaw = math.radians(deg)

            # 3. 旋转多边形 (绕原点 (0,0))
            # shapely 的 rotate 默认是逆时针
            rotated_poly = rotate(vehicle_poly, deg, origin=(0, 0), use_radians=False)

            # 4. 栅格化 (Rasterization)
            # 找出这个旋转后的形状覆盖了哪些相对栅格 (dx, dy)
            # 方法：获取 Bounding Box -> 遍历 -> 严格相交检测

            min_x, min_y, max_x, max_y = rotated_poly.bounds

            # 转换为相对索引范围
            min_ix = int(math.floor(min_x / res))
            max_ix = int(math.ceil(max_x / res))
            min_iy = int(math.floor(min_y / res))
            max_iy = int(math.ceil(max_y / res))

            offsets = []

            for dx in range(min_ix, max_ix + 1):
                for dy in range(min_iy, max_iy + 1):
                    # 构建该栅格的几何框
                    # 我们要判断的是，当车中心在 (0,0) 时，它是否覆盖了位于 (dx, dy) 的格子
                    grid_poly = box(dx * res, dy * res, (dx + 1) * res, (dy + 1) * res)

                    if rotated_poly.intersects(grid_poly):
                        offsets.append((dx, dy))

            self.footprint_table[int(deg)] = offsets

        avg_cells = np.mean([len(v) for v in self.footprint_table.values()]) if self.footprint_table else 0
        print(f"[GridMap] Footprint table ready. Avg cells per angle: {avg_cells:.1f}")

    def _check_collision_footprint(self, x: float, y: float, yaw: float) -> bool:
        """
        策略3: 查表法 (Footprint Table)
        优点: 极快 (整数加法)，精度高 (预计算了几何形状)
        """
        # 1. 计算车辆中心所在的栅格索引
        cx_idx, cy_idx = self.get_index_from_pos(x, y)

        # 2. 计算角度索引
        # 归一化到 [0, 360) 并找到最近的离散角度
        deg = math.degrees(yaw) % 360
        if deg < 0:
            deg += 360

        # 找到最近的 key
        # 比如 res=2, deg=3.1 -> index=2 (即 4度) 或者直接 int(deg/res)*res
        target_deg = int(round(deg / self.footprint_yaw_res)) * int(self.footprint_yaw_res)
        target_deg = target_deg % 360  # 防止 360 溢出变成 0

        # 获取偏移量列表
        # 如果没有预计算 (比如中途切换了配置)，尝试实时 fallback 或报错
        if not self.footprint_table:
            # 懒加载 fallback
            self._init_collision_lookup_table()

        offsets = self.footprint_table.get(target_deg, [])

        # 3. 遍历偏移量进行检测
        for dx, dy in offsets:
            nx = cx_idx + dx
            ny = cy_idx + dy

            # 越界检查
            if nx < 0 or nx >= self.width_idx or ny < 0 or ny >= self.height_idx:
                return True  # 车身部分在地图外，视为碰撞

            # 查表
            if self.obstacle_map[nx][ny]:
                return True

        return False

    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        [实现 BaseMap 接口] 获取物理边界
        Returns: (min_x, max_x, min_y, max_y)
        """
        return (0.0, self.width_m, 0.0, self.height_m)

    # =========================================================
    #  [新增] 推土机地图生成辅助方法
    # =========================================================

    def _simple_motion_step(self, x: float, y: float, yaw: float,
                           steer: float, distance: float) -> Tuple[float, float, float]:
        """
        简易运动学模拟步骤（自行车模型）。

        独立于 planner.py 实现，避免循环依赖。
        用于推土机地图生成时的轨迹模拟。

        Args:
            x, y, yaw: 当前位姿
            steer: 转向角 [rad]
            distance: 移动距离 [m]

        Returns:
            (new_x, new_y, new_yaw): 更新后的位姿
        """
        wheelbase = self.robot_config.wheelbase

        # 自行车模型积分
        new_x = x + distance * math.cos(yaw)
        new_y = y + distance * math.sin(yaw)
        new_yaw = yaw + distance * math.tan(steer) / wheelbase

        return new_x, new_y, new_yaw

    def _clear_obstacles_for_pose(self, x: float, y: float, yaw: float, inflation: float = 1.0):
        """
        清除特定位姿下车辆轮廓覆盖的所有障碍物栅格。

        用于推土机地图生成：在地图上"挖掘"出一条可行路径。

        Args:
            x, y, yaw: 车辆位姿
            inflation: 尺寸放大系数（推土机比真车大 N 倍）
        """
        if self.obstacle_map is None:
            return

        res = self.config.xy_resolution

        # 1. 获取车辆轮廓并放大
        outline = self.robot_config.vehicle_outline.copy()
        outline[0, :] *= inflation  # X 方向（车长）放大
        outline[1, :] *= inflation  # Y 方向（车宽）放大

        # 2. 旋转并平移到目标位姿
        rot = np.array([
            [math.cos(yaw), math.sin(yaw)],
            [-math.sin(yaw), math.cos(yaw)]
        ])
        transformed_outline = (outline.T.dot(rot)).T
        transformed_outline[0, :] += x
        transformed_outline[1, :] += y

        # 3. 构建 Shapely 多边形
        vertices = []
        for j in range(transformed_outline.shape[1] - 1):  # 去掉重复的起点
            vertices.append((transformed_outline[0, j], transformed_outline[1, j]))
        vehicle_poly = Polygon(vertices)

        # 4. 计算包围盒并遍历相关栅格
        min_x_idx = max(0, int(math.floor(np.min(transformed_outline[0, :]) / res)))
        max_x_idx = min(self.width_idx - 1, int(math.ceil(np.max(transformed_outline[0, :]) / res)))
        min_y_idx = max(0, int(math.floor(np.min(transformed_outline[1, :]) / res)))
        max_y_idx = min(self.height_idx - 1, int(math.ceil(np.max(transformed_outline[1, :]) / res)))

        # 5. 清除多边形覆盖的所有障碍物
        half_res = res / 2.0
        for ix in range(min_x_idx, max_x_idx + 1):
            for iy in range(min_y_idx, max_y_idx + 1):
                # 检查栅格中心是否在多边形内
                center_x, center_y = self.get_pos_from_index(ix, iy)
                grid_box = box(center_x - half_res, center_y - half_res,
                              center_x + half_res, center_y + half_res)

                if vehicle_poly.intersects(grid_box):
                    self.obstacle_map[ix][iy] = False

    def generate_random_map(self, width_m: float, height_m: float, obstacle_num: int):
        """生成测试用的随机障碍物地图"""
        self.width_m = width_m
        self.height_m = height_m
        
        # 计算数组大小 (使用 round 确定整体尺寸是合理的)
        self.width_idx = round(width_m / self.config.xy_resolution)
        self.height_idx = round(height_m / self.config.xy_resolution)
        
        self.obstacle_map = np.zeros((self.width_idx, self.height_idx), dtype=bool)
        
        # 生成四周墙壁
        for i in range(self.width_idx):
            self.obstacle_map[i][0] = True
            self.obstacle_map[i][self.height_idx - 1] = True
        for i in range(self.height_idx):
            self.obstacle_map[0][i] = True
            self.obstacle_map[self.width_idx - 1][i] = True
            
        # 随机障碍物
        # 注意：这里我们避开最外圈墙壁，所以范围缩进
        for _ in range(obstacle_num):
            x = random.randint(1, self.width_idx - 2)
            y = random.randint(1, self.height_idx - 2)
            self.obstacle_map[x][y] = True

    def generate_guaranteed_map(self, start: Tuple[float, float, float],
                               goal: Tuple[float, float, float],
                               width_m: float, height_m: float,
                               obstacle_num: Optional[int] = None):
        """
        推土机地图生成（逆向可行性地图生成）。

        核心思想：
        1. 先生成大量随机障碍物（制造困难环境）
        2. 模拟一个"虚拟推土机"从起点开到终点
        3. 推土机沿途清除障碍物，留下一条保证可行的路径

        Args:
            start: 起点位姿 (x, y, yaw) [m, m, rad]
            goal: 终点位姿 (x, y, yaw) [m, m, rad]
            width_m, height_m: 地图物理尺寸 [m]
            obstacle_num: 初始障碍物数量（None 则自动计算）
        """
        print(f"[GridMap] Generating guaranteed feasible map (Bulldozer Method)...")

        # 1. 初始化底图（先生成大量随机障碍物）
        if obstacle_num is None:
            # 自动计算：每平方米 2-3 个障碍物
            obstacle_num = int(width_m * height_m * 0.25)

        self.generate_random_map(width_m, height_m, obstacle_num)

        # 2. 准备推土机参数
        curr_x, curr_y, curr_yaw = start
        goal_x, goal_y, goal_yaw = goal

        step_size = self.config.bulldozer_step_size
        wheelbase = self.robot_config.wheelbase
        max_steer = self.robot_config.max_steer
        inflation = self.config.bulldozer_inflation
        noise_deg = self.config.bulldozer_steer_noise_deg

        # 3. 开始挖掘循环
        max_iter = 2000  # 防止无限循环
        iteration = 0
        goal_threshold = 2.0  # [m] 距离终点 2m 时停止

        print(f"  - Start: ({curr_x:.1f}, {curr_y:.1f}, {math.degrees(curr_yaw):.1f}°)")
        print(f"  - Goal:  ({goal_x:.1f}, {goal_y:.1f}, {math.degrees(goal_yaw):.1f}°)")
        print(f"  - Inflation: {inflation}x, Noise: {noise_deg}°, Step: {step_size}m")

        while iteration < max_iter:
            # 3.1 挖掉当前位置
            self._clear_obstacles_for_pose(curr_x, curr_y, curr_yaw, inflation)

            # 3.2 检查是否到达终点附近
            dist_to_goal = math.hypot(goal_x - curr_x, goal_y - curr_y)
            if dist_to_goal < goal_threshold:
                print(f"  - Reached goal vicinity after {iteration} iterations (dist={dist_to_goal:.2f}m)")
                break

            # 3.3 计算指向终点的理想航向
            target_yaw = math.atan2(goal_y - curr_y, goal_x - curr_x)

            # 3.4 计算需要的转向角（简单的 P 控制器）
            # 角度误差归一化到 [-π, π]
            yaw_error = (target_yaw - curr_yaw + math.pi) % (2 * math.pi) - math.pi
            base_steer = np.clip(yaw_error, -max_steer, max_steer)

            # 3.5 加入随机噪声（制造蜿蜒路径，避免直线）
            noise_rad = math.radians(noise_deg)
            noise = np.random.uniform(-noise_rad, noise_rad)
            final_steer = np.clip(base_steer + noise, -max_steer, max_steer)

            # 3.6 运动学更新
            curr_x, curr_y, curr_yaw = self._simple_motion_step(
                curr_x, curr_y, curr_yaw, final_steer, step_size
            )

            # 3.7 边界保护（防止跑出地图）
            curr_x = np.clip(curr_x, 0.0, width_m)
            curr_y = np.clip(curr_y, 0.0, height_m)

            iteration += 1

        # 4. 确保终点也被清理
        self._clear_obstacles_for_pose(goal_x, goal_y, goal_yaw, inflation)

        # 5. 在终点附近进行多次转向清理（确保车辆可以从不同角度接近）
        for yaw_offset in [-math.radians(15), 0, math.radians(15)]:
            self._clear_obstacles_for_pose(goal_x, goal_y, goal_yaw + yaw_offset, inflation)

        print(f"[GridMap] Guaranteed map generation complete. Cleared {iteration} poses.")

    def plot_map(self):
        """可视化地图 (Matplotlib)"""
        if self.obstacle_map is None:
            return
            
        obs_x, obs_y = [], []
        for x in range(self.width_idx):
            for y in range(self.height_idx):
                if self.obstacle_map[x][y]:
                    # 调用 get_pos_from_index 获取物理中心
                    # 这样画出来的点，物理上就是感知认为的障碍物中心
                    # 并且会与 imshow 的热力图方块中心对齐
                    px, py = self.get_pos_from_index(x, y)
                    obs_x.append(px)
                    obs_y.append(py)
        
        plt.plot(obs_x, obs_y, ".k")
        plt.axis("equal")
        plt.grid(True)

    def check_strict_path_collision(self, path_x: List[float], path_y: List[float], path_yaw: List[float]) -> List[Tuple[float, float]]:
        """
        严格碰撞检测（多边形精确检测）。

       使用车辆多边形进行精确碰撞检测，对比规划时的圆形近似模型。
        这可以作为 Ground Truth Check 和消融实验分析。

        实现：使用 Shapely 几何库进行高效的栅格-多边形相交检测。

        Args:
            path_x: 路径 x 坐标列表 [m]
            path_y: 路径 y 坐标列表 [m]
            path_yaw: 路径航向角列表 [rad]

        Returns:
            List[Tuple[float, float]]: 被碰撞的障碍物坐标列表 [(x, y), ...]
        """
        if self.obstacle_map is None:
            return []

        collided_obstacles = []
        outline = self.robot_config.vehicle_outline

        # 遍历路径上的每一个位姿
        for i in range(len(path_x)):
            x, y, yaw = path_x[i], path_y[i], path_yaw[i]

            # 1. 构建车辆多边形（Shapely Polygon）
            # 旋转矩阵 (2x2)
            rot = np.array([
                [math.cos(yaw), math.sin(yaw)],
                [-math.sin(yaw), math.cos(yaw)]
            ])

            # 变换: Outline(2xN) -> Transpose -> Dot -> Transpose -> Translate
            rotated_outline = (outline.T.dot(rot)).T
            rotated_outline[0, :] += x
            rotated_outline[1, :] += y

            # 提取多边形顶点（去除重复的最后一个点）
            vehicle_vertices = []
            for j in range(rotated_outline.shape[1] - 1):  # -1 去掉重复的起点
                vehicle_vertices.append((rotated_outline[0, j], rotated_outline[1, j]))

            # 创建 Shapely Polygon 对象
            vehicle_polygon = Polygon(vehicle_vertices)

            # 2. 计算车辆 Bounding Box 以优化检测范围
            min_x_idx = max(0, int(math.floor(np.min(rotated_outline[0, :]) / self.config.xy_resolution)))
            max_x_idx = min(self.width_idx - 1, int(math.ceil(np.max(rotated_outline[0, :]) / self.config.xy_resolution)))
            min_y_idx = max(0, int(math.floor(np.min(rotated_outline[1, :]) / self.config.xy_resolution)))
            max_y_idx = min(self.height_idx - 1, int(math.ceil(np.max(rotated_outline[1, :]) / self.config.xy_resolution)))

            # 3. 仅检查 Bounding Box 内的障碍物栅格
            for ix in range(min_x_idx, max_x_idx + 1):
                for iy in range(min_y_idx, max_x_idx + 1):
                    if self.obstacle_map[ix][iy]:
                        # 获取障碍物栅格中心的物理坐标
                        obs_center_x, obs_center_y = self.get_pos_from_index(ix, iy)

                        # 计算栅格的边界（用于 Shapely 相交检测）
                        res = self.config.xy_resolution
                        grid_min_x = obs_center_x - res / 2
                        grid_max_x = obs_center_x + res / 2
                        grid_min_y = obs_center_y - res / 2
                        grid_max_y = obs_center_y + res / 2

                        # 创建栅格的 AABB (Axis-Aligned Bounding Box)
                        grid_bbox = box(grid_min_x, grid_min_y, grid_max_x, grid_max_y)

                        # 使用 Shapely 检测相交（高效且精确）
                        if vehicle_polygon.intersects(grid_bbox):
                            collided_obstacles.append((obs_center_x, obs_center_y))

        return collided_obstacles

# --- 单元测试 ---
if __name__ == "__main__":
    print("Testing GridMap with Corrected Coordinates...")
    
    h_cfg = HybridAStarConfig()
    # 假设分辨率是 1.0
    h_cfg.xy_resolution = 1.0 
    v_cfg = VehicleConfig()
    
    # 2. 实例化地图
    gm = GridMap(h_cfg, v_cfg)
    gm.generate_random_map(20, 20, 30)
    
    # 验证 SSOT
    # 1. 测试坐标转换
    test_pos = (5.9, 5.1)
    idx = gm.get_index_from_pos(*test_pos)
    print(f"Pos {test_pos} -> Index {idx} (Expected: (5, 5))")
    
    center_pos = gm.get_pos_from_index(*idx)
    print(f"Index {idx} -> Center Pos {center_pos} (Expected: (5.5, 5.5))")

    # 2. 绘图验证
    plt.figure(figsize=(6, 6))
    gm.plot_map()
    
    # 画一个红色的框代表 Grid(5,5) 的物理范围
    rect = plt.Rectangle((5.0, 5.0), 1.0, 1.0, linewidth=2, edgecolor='r', facecolor='none', label='Grid(5,5) Bounds')
    plt.gca().add_patch(rect)
    
    # 画转换后的中心点
    plt.plot(center_pos[0], center_pos[1], "xr", markersize=12, label='Grid Center')
    
    plt.legend()
    plt.title("GridMap Coordinate System Check")
    plt.show()