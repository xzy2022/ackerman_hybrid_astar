# test/test_collision_model.py
import unittest
import math
from src.config import VehicleConfig, HybridAStarConfig
from src.grid_map import GridMap

class TestCollisionModel(unittest.TestCase):
    def test_corner_coverage(self):
        """测试车辆四角是否被碰撞圆覆盖"""
        v_cfg = VehicleConfig()
        
        # 以前角为例
        # 前角局部坐标
        corner_x = v_cfg.front_hang
        corner_y = v_cfg.width / 2.0
        
        # 找到最近的圆
        min_dist = float('inf')
        for offset in v_cfg.collision_offsets:
            dist = math.hypot(corner_x - offset, corner_y - 0)
            if dist < min_dist:
                min_dist = dist
        
        print(f"前角距最近圆心的距离: {min_dist:.4f} m")
        print(f"碰撞圆半径: {v_cfg.collision_radius:.4f} m")
        
        # 断言：距离必须小于半径（否则就是暴露了）
        self.assertLess(min_dist, v_cfg.collision_radius, 
                        "前角未被碰撞圆覆盖，可能导致碰撞！")

if __name__ == '__main__':
    unittest.main()