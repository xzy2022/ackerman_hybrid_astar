import matplotlib.pyplot as plt
import math
import numpy as np
from config import Config
from grid_map import GridMap
from hybrid_a_star import hybrid_a_star_planning

def main():
    print("Hybrid A* Algorithm Start")
    
    # 1. 配置参数
    config = Config()
    
    # 2. 初始化地图
    # 50x50米，60个障碍物
    grid_map = GridMap(config)
    sx, sy, syaw = 10.0, 10.0, math.radians(0.0)
    gx, gy, gyaw = 40.0, 40.0, math.radians(90.0)
    
    grid_map.generate_map(50, 50, 60, start_m=[sx, sy], goal_m=[gx, gy])

    # 3. 运行规划
    path_x, path_y, path_yaw = hybrid_a_star_planning(
        [sx, sy, syaw], 
        [gx, gy, gyaw], 
        grid_map, 
        config
    )

    # 4. 可视化与动画
    if path_x:
        print("Visualization...")
        
        # --- A. 绘制静态背景 ---
        # 调用 grid_map 的绘图方法绘制 障碍物、起点、终点
        grid_map.plot_map(start_m=[sx, sy], goal_m=[gx, gy])
        
        # 绘制规划出的完整红色路径线
        plt.plot(path_x, path_y, "-r", label="Hybrid A* Path", linewidth=1.5)
        
        # 添加图例和标题
        plt.title("Hybrid A* Path Planning Simulation")
        plt.legend()
        
        # --- B. 车辆运动动画 ---
        # 我们使用 Python 的列表切片 path_x[::N] 来跳帧
        # 否则点太密，动画会很慢。这里每隔 2 个点画一帧。
        skip_step = 2 
        
        print("Animating vehicle movement...")
        for i in range(0, len(path_x), skip_step):
            # 1. 绘制当前帧的车辆
            # 我们需要获取 plot 返回的对象，以便后面删除它
            car_lines = plot_car(path_x[i], path_y[i], path_yaw[i], config)
            
            # 2. 暂停一小段时间，让画面渲染出来
            plt.pause(0.01) 
            
            # 3. 如果不是最后一帧，就移除当前车辆
            # 这样会产生移动的效果，而不是留下一长串影子
            if i < len(path_x) - skip_step:
                for line in car_lines:
                    line.remove()
        
        # 最后一帧保留在画面上
        plot_car(path_x[-1], path_y[-1], path_yaw[-1], config)
        
        print("Goal Reached!")
        plt.show() # 最后阻塞住窗口，让用户查看结果
        
    else:
        print("Failed to find path.")

def plot_car(x, y, yaw, config):
    """
    绘制车辆轮廓
    :return: 返回 matplotlib 的线对象列表，方便后续清除(remove)
    """
    # 1. 直接使用 config 中定义的车辆轮廓 (原点在后轴中心)
    # 形状为 (2, 5) -> [[x1, x2...], [y1, y2...]]
    car_outline = np.copy(config.vehicle_outline)
    
    # 2. 旋转矩阵
    # 这里的写法是为了配合下面的转置点乘：Points(Nx2) dot Rot(2x2)
    # 实际上等于标准旋转矩阵的转置
    rot_mat = np.array([
        [math.cos(yaw), math.sin(yaw)],
        [-math.sin(yaw), math.cos(yaw)]
    ])
    
    # 3. 旋转变换
    # outline: (2, 5), rot: (2, 2)
    # (2, 5).T -> (5, 2) dot (2, 2) -> (5, 2)
    # 再转置回 (2, 5) 以便切片绘图
    car_outline = (car_outline.T.dot(rot_mat)).T
    
    # 4. 平移
    car_outline[0, :] += x
    car_outline[1, :] += y
    
    # 5. 绘图
    # 这里我们用 "-k" (黑色实线) 画车身
    line, = plt.plot(car_outline[0, :], car_outline[1, :], "-k")
    
    # 进阶：为了看清车头，我们可以画一个箭头
    arrow_len = 1.0
    arrow = plt.arrow(x, y, 
                      arrow_len * math.cos(yaw), 
                      arrow_len * math.sin(yaw), 
                      head_width=0.3, color='k')
    
    # 返回绘制的对象，以便动画循环中可以 .remove() 掉它们
    # arrow 其实包含 patch，删除比较麻烦，简单起见动画里可以只返回 line
    # 如果想把箭头也删掉，需要把 arrow 也加进列表
    return [line, arrow]

if __name__ == '__main__':
    main()