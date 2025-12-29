import matplotlib.pyplot as plt
import math
from config import Config
from grid_map import GridMap
from hybrid_a_star import hybrid_a_star_planning
import numpy as np

def main():
    print("Hybrid A* Algorithm Start")
    
    # 1. 配置参数
    config = Config()
    
    # 2. 初始化地图
    # 50x50米，50个障碍物
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

    # 4. 可视化结果
    if path_x:
        print("Visualization...")
        # 画地图
        grid_map.plot_map(start_m=[sx, sy], goal_m=[gx, gy])
        
        # 在原图上画路径
        # 车辆轮廓动画太复杂，这里先画简单的线条轨迹
        plt.plot(path_x, path_y, "-r", label="Hybrid A* Path", linewidth=2)
        
        # 简单画几个车辆姿态 (每隔几个点画一个车)
        for i in range(0, len(path_x), 15):
            plot_car(path_x[i], path_y[i], path_yaw[i], config)
            
        plt.legend()
        plt.show()
    else:
        print("Failed to find path.")

def plot_car(x, y, yaw, config):
    # 简单的车辆轮廓变换和绘制
    car_outline = np.array([
        [-config.LB, (config.L + config.LF), (config.L + config.LF), -config.LB, -config.LB],
        [config.W / 2, config.W / 2, -config.W / 2, -config.W / 2, config.W / 2]
    ])
    
    # 旋转矩阵
    rot_mat = np.array([
        [math.cos(yaw), math.sin(yaw)],
        [-math.sin(yaw), math.cos(yaw)]
    ])
    
    # 旋转并平移
    car_outline = (car_outline.T.dot(rot_mat)).T
    car_outline[0, :] += x
    car_outline[1, :] += y
    
    plt.plot(car_outline[0, :], car_outline[1, :], "-k")



if __name__ == '__main__':
    main()