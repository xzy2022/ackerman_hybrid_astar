import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


from src.config import VehicleConfig

def plot_dimension_arrow(ax, start, end, text, offset_text=(0, 0.2)):
    """辅助函数：绘制标注尺寸的双向箭头"""
    ax.annotate(
        '', xy=start, xytext=end,
        arrowprops=dict(arrowstyle='<->', lw=1.5, color='blue')
    )
    # 计算文字位置（中点）
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    ax.text(mid_x + offset_text[0], mid_y + offset_text[1], text, 
            color='blue', ha='center', va='center', fontweight='bold')

def visualize_vehicle_configuration():
    # 1. 初始化配置
    config = VehicleConfig()
    
    # 创建画布
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # --- A. 绘制车身轮廓 (Vehicle Outline) ---
    outline = config.vehicle_outline
    # 闭合轮廓以便绘图
    outline_x = np.append(outline[0, :], outline[0, 0])
    outline_y = np.append(outline[1, :], outline[1, 0])
    
    ax.plot(outline_x, outline_y, 'k-', linewidth=2, label='Vehicle Outline')
    ax.fill(outline_x, outline_y, 'gray', alpha=0.1) # 填充灰色底色

    # --- B. 绘制关键坐标点 ---
    # 原点 (0,0) - 后轴中心
    ax.plot(0, 0, 'ro', markersize=8, label='Rear Axle (Origin) (0,0)', zorder=10)
    
    # 前轴中心 (Wheelbase, 0)
    ax.plot(config.wheelbase, 0, 'go', markersize=8, label=f'Front Axle ({config.wheelbase}, 0)', zorder=10)

    # --- C. 绘制碰撞检测圆 (Collision Circles) ---
    # 这是最关键的验证：圆是否包住了车身？
    first_circle = True
    for offset in config.collision_offsets:
        circle = plt.Circle((offset, 0), config.collision_radius, 
                            color='cyan', alpha=0.4, linestyle='--')
        ax.add_patch(circle)
        # 标记圆心
        ax.plot(offset, 0, 'bx', markersize=5)
        
        # 仅为第一个圆添加图例，避免重复
        if first_circle:
            circle.set_label('Collision Circle')
            first_circle = False

    # --- D. 标注物理尺寸 (Dimensions) ---
    # 1. 轴距 (Wheelbase)
    plot_dimension_arrow(ax, (0, 0), (config.wheelbase, 0), 
                         f"WB={config.wheelbase}m", offset_text=(0, -0.5))
    
    # 2. 前悬 (Front Hang - 后轴到车头)
    # 注意：代码中 front_hang 定义的是后轴到车头的距离，而非前轴到车头
    plot_dimension_arrow(ax, (0, 1.5), (config.front_hang, 1.5), 
                         f"Front Hang (LF)={config.front_hang}m")
    
    # 3. 后悬 (Rear Hang - 后轴到车尾)
    plot_dimension_arrow(ax, (0, 1.5), (-config.rear_hang, 1.5), 
                         f"Rear Hang (LB)={config.rear_hang}m")
    
    # 4. 车宽 (Width)
    plot_dimension_arrow(ax, (config.wheelbase + 1.0, -config.width/2), 
                             (config.wheelbase + 1.0, config.width/2), 
                             f"W={config.width}m", offset_text=(0.4, 0))

    # --- 设置图表属性 ---
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.set_title("Vehicle Configuration & Collision Model Visualization", fontsize=14)
    ax.set_xlabel("X [m] (Longitudinal)")
    ax.set_ylabel("Y [m] (Lateral)")
    
    # 设置显示范围，留出一点边距
    margin = 1.0
    ax.set_xlim(-config.rear_hang - margin, config.front_hang + margin)
    ax.set_ylim(-config.width - margin, config.width + margin)
    
    ax.legend(loc='upper right')
    
    print("可视化窗口已打开。请检查：")
    print("1. 红色原点是否位于后轴中心。")
    print("2. 青色圆圈是否完全覆盖了车身矩形（尤其是四个角）。")
    print("3. 如果角露在外面，说明 collision_radius 或 offsets 需要调整。")
    
    plt.show()

if __name__ == "__main__":
    visualize_vehicle_configuration()