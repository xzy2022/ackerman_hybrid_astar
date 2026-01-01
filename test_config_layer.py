"""
配置层重构测试脚本
验证新的 RobotConfig 体系是否正常工作
"""

import sys
sys.path.insert(0, 'src')

from config import (
    RobotConfig, VehicleConfig, PointConfig, AckermannConfig,
    RobotModel, HybridAStarConfig
)

def test_ackermann_config():
    """测试阿克曼车辆配置"""
    print("=" * 60)
    print("测试1: 阿克曼车辆配置 (VehicleConfig)")
    print("=" * 60)

    # 使用 VehicleConfig（向后兼容）
    vehicle = VehicleConfig(
        wheelbase=2.5,
        width=2.0,
        max_steer_deg=35.0
    )

    print(f"[OK] 模型类型: {vehicle.model_type}")
    print(f"[OK] 轴距: {vehicle.wheelbase}m")
    print(f"[OK] 车宽: {vehicle.width}m")
    print(f"[OK] 最大转向角: {vehicle.max_steer_deg}° = {vehicle.max_steer:.4f} rad")
    print(f"[OK] 碰撞半径: {vehicle.collision_radius:.4f}m")
    print(f"[OK] 碰撞圆数量: {len(vehicle.collision_offsets)}")

    # 验证模型类型
    assert vehicle.model_type == RobotModel.ACKERMANN, "模型类型应该是 ACKERMANN"
    assert hasattr(vehicle, 'wheelbase'), "应该有 wheelbase 属性"
    assert hasattr(vehicle, 'vehicle_outline'), "应该计算 vehicle_outline"

    print("[OK] 测试通过！\n")

def test_point_config():
    """测试点模型配置"""
    print("=" * 60)
    print("测试2: 点模型配置 (PointConfig)")
    print("=" * 60)

    # 创建点模型配置
    point_robot = PointConfig(radius=0.5)

    print(f"[OK] 模型类型: {point_robot.model_type}")
    print(f"[OK] 碰撞半径: {point_robot.radius}m")

    # 验证模型类型
    assert point_robot.model_type == RobotModel.POINT, "模型类型应该是 POINT"
    assert not hasattr(point_robot, 'wheelbase'), "点模型不应该有 wheelbase"
    assert not hasattr(point_robot, 'vehicle_outline'), "点模型不需要 vehicle_outline"

    print("[OK] 测试通过！\n")

def test_backward_compatibility():
    """测试向后兼容性"""
    print("=" * 60)
    print("测试3: 向后兼容性 (AckermannConfig 别名)")
    print("=" * 60)

    # 使用别名创建配置
    ackermann = AckermannConfig(wheelbase=3.0, width=2.2)

    print(f"[OK] 使用 AckermannConfig 别名创建成功")
    print(f"[OK] 模型类型: {ackermann.model_type}")
    print(f"[OK] 轴距: {ackermann.wheelbase}m")

    assert ackermann.model_type == RobotModel.ACKERMANN

    print("[OK] 向后兼容性测试通过！\n")

def test_inheritance():
    """测试继承关系"""
    print("=" * 60)
    print("测试4: 继承关系")
    print("=" * 60)

    vehicle = VehicleConfig()
    point = PointConfig()

    # 验证继承关系
    assert isinstance(vehicle, RobotConfig), "VehicleConfig 应该继承 RobotConfig"
    assert isinstance(point, RobotConfig), "PointConfig 应该继承 RobotConfig"

    print(f"[OK] VehicleConfig 是 RobotConfig 的子类: {isinstance(vehicle, RobotConfig)}")
    print(f"[OK] PointConfig 是 RobotConfig 的子类: {isinstance(point, RobotConfig)}")

    # 验证基类属性
    print(f"[OK] VehicleConfig.radius: {vehicle.radius}m")
    print(f"[OK] PointConfig.radius: {point.radius}m")

    print("[OK] 继承关系测试通过！\n")

def test_planner_config():
    """测试规划器配置（保持不变）"""
    print("=" * 60)
    print("测试5: 规划器配置 (HybridAStarConfig)")
    print("=" * 60)

    planner_cfg = HybridAStarConfig(
        xy_resolution=0.5,
        yaw_resolution_deg=15.0
    )

    print(f"[OK] XY 分辨率: {planner_cfg.xy_resolution}m")
    print(f"[OK] Yaw 分辨率: {planner_cfg.yaw_resolution_deg}° = {planner_cfg.yaw_resolution:.4f} rad")
    print(f"[OK] 步长: {planner_cfg.step_size}m")
    print(f"[OK] 启发式权重: {planner_cfg.heuristic_weight}")

    print("[OK] 测试通过！\n")

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("配置层重构测试套件")
    print("=" * 60 + "\n")

    try:
        test_ackermann_config()
        test_point_config()
        test_backward_compatibility()
        test_inheritance()
        test_planner_config()

        print("=" * 60)
        print("[SUCCESS] 所有测试通过！配置层重构成功！")
        print("=" * 60)

    except Exception as e:
        print(f"\n[FAILED] 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
