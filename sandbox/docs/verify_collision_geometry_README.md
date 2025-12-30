# 雷区测试参数说明

## 可调整参数（位于 `verify_collision_geometry.py` 顶部）

### 地图相关
```python
MAP_RESOLUTION = 0.5    # 地图分辨率 [m]
MAP_SIZE = 30.0         # 地图大小 [m] (30m x 30m)
```

- **MAP_RESOLUTION**: 栅格地图的分辨率，值越小越精确，但计算量越大
- **MAP_SIZE**: 测试地图的物理尺寸

### 车辆位姿
```python
VEHICLE_X = 10.0          # [m] x坐标
VEHICLE_Y = 10.0          # [m] y坐标
VEHICLE_YAW_DEG = 30.0    # [deg] 航向角
```

- **VEHICLE_X/Y**: 车辆在世界坐标系中的位置
- **VEHICLE_YAW_DEG**: 车辆朝向角度（度），0° 表示朝东（X轴正方向）

### 雷区设置
```python
MINEFIELD_RADIUS = 8.0    # 雷区范围（以车辆为中心）[m]
```

- **MINEFIELD_RADIUS**: 障碍物生成的半径范围，以车辆中心为圆心
- 该值决定了"雷区"的密集程度

### 可视化设置
```python
LOCAL_VIEW_RANGE = 6.0    # 局部放大视图范围 [m]
```

- **LOCAL_VIEW_RANGE**: 右侧子图（局部放大视图）的显示范围
- 该值控制放大视图显示多少米范围内的内容

## 使用建议

### 测试场景1：标准精度测试
```python
MAP_RESOLUTION = 0.5      # 标准分辨率
VEHICLE_YAW_DEG = 30.0    # 30度斜角
MINEFIELD_RADIUS = 8.0    # 标准雷区
```

### 测试场景2：高精度测试
```python
MAP_RESOLUTION = 0.2      # 更高分辨率
VEHICLE_YAW_DEG = 45.0    # 45度斜角（更极端）
MINEFIELD_RADIUS = 10.0   # 更大雷区
```

### 测试场景3：不同角度测试
可以修改 `VEHICLE_YAW_DEG` 测试不同角度：
- `0.0` - 车辆水平（朝东）
- `90.0` - 车辆垂直（朝北）
- `30.0` / `45.0` / `60.0` - 斜角测试

### 测试场景4：不同位置测试
可以修改 `VEHICLE_X` 和 `VEHICLE_Y` 将车辆放在地图的不同位置：
```python
VEHICLE_X = 15.0          # 地图中心
VEHICLE_Y = 15.0
```

## 注意事项

1. **分辨率vs性能**: `MAP_RESOLUTION` 越小，测试越精确，但计算时间和内存占用会增加
2. **雷区范围**: `MINEFIELD_RADIUS` 应该足够大，确保完全覆盖车辆轮廓
3. **视图范围**: `LOCAL_VIEW_RANGE` 建议设置为 2-3 倍车长，以便清晰观察细节

## 输出文件

运行后会生成：
- `verify_collision_geometry_result.png` - 包含全局视图和局部放大视图的可视化结果

## 快速修改示例

测试不同分辨率的影响：
```python
# 只需修改这一行
MAP_RESOLUTION = 0.3  # 从 0.5 改为 0.3
```

测试不同角度：
```python
# 只需修改这一行
VEHICLE_YAW_DEG = 45.0  # 从 30.0 改为 45.0
```
