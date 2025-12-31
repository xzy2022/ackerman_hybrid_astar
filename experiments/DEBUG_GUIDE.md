# 调试批量测试中的特定实验

## 快速开始

假设你运行了批量测试，发现某次实验结果异常，想要可视化调试：

### 步骤1：查看CSV找到异常实验

打开 `experiments/results/batch_test_log_YYYYMMDD_HHMMSS.csv`，找到你想调试的实验。

例如，发现种子为5的实验有很多碰撞：

```
Experiment_ID,Seed,Success,Planning_Time_ms,Nodes_Expanded,Path_Cost,Strict_Collision_Count
...
5,5,True,245.3,1245,150.06,127
...
```

### 步骤2：使用种子复现实验

```bash
# 使用相同的种子可视化该实验
python src/run_simulation.py --seed 5
```

这将：
- 使用相同的随机种子（5）生成相同的地图
- 使用与批量测试完全相同的参数
- 显示规划结果和碰撞检测的可视化

### 步骤3：分析可视化结果

可视化窗口会显示：
- **图1**：规划结果总览
  - 黑色点：障碍物
  - 绿色线：规划路径
  - 蓝色星：搜索树
  - 红叉：严格碰撞检测发现的碰撞点（如果有）

- **图2**：启发式热力图

## 常用调试场景

### 场景1：分析高碰撞次数

```bash
# 查看有127个碰撞点的实验（种子=5）
python src/run_simulation.py --seed 5
```

观察可视化中的红叉位置，分析：
- 碰撞点集中在路径的哪些区域？
- 是否都在转弯处（圆形模型精度不足）？
- 是否在直线上（可能是其他问题）？

### 场景2：分析规划失败

```bash
# 查看规划失败的实验（种子=3）
python src/run_simulation.py --seed 3
```

观察：
- 起终点是否被障碍物包围？
- 是否存在不可逾越的障碍物墙？
- 路径是否过于复杂导致算法超时？

### 场景3：对比不同参数

```bash
# 运行种子10的实验（默认参数）
python src/run_simulation.py --seed 10

# 然后修改 src/run_simulation.py 中的参数（例如启发式权重）
# h_config.heuristic_weight = 10.0
# 再次运行相同种子，观察差异
python src/run_simulation.py --seed 10
```

### 场景4：快速测试（无可视化）

当只需要验证结果，不需要查看可视化时：

```bash
# 快速验证，不弹出窗口
python src/run_simulation.py --seed 5 --no-visualization
```

输出示例：
```
=== Hybrid A* Research Simulation ===
Random seed set to: 5
Planning...
Goal Reached! Cost: 150.06
Success! Cost: 150.06, Nodes: 8144
Running strict collision detection...
WARNING: Strict collision detected! 127 obstacle(s) collided.
Visualization skipped (--no-visualization flag set)
```

## 参数说明

### --seed SEED
- **类型**：整数
- **默认值**：None（随机）
- **作用**：设置随机种子，确保可复现
- **示例**：
  - `--seed 1`
  - `--seed 42`
  - `--seed 100`

### --no-visualization
- **类型**：标志（无需参数）
- **默认值**：False（显示可视化）
- **作用**：跳过可视化窗口，仅输出文本结果
- **用途**：快速测试、批量运行脚本

## 与批量测试的对应关系

批量测试（`experiments/run_batch_test.py`）和单次测试（`src/run_simulation.py`）使用**完全相同的**：
- 地图尺寸（50m × 50m）
- 障碍物数量（150个）
- 起终点位置
- 算法参数（启发式权重、步长等）

因此，使用相同的种子会生成**完全相同的实验场景**。

## 实用技巧

### 技巧1：批量筛选异常实验

使用 Python 快速找出异常实验：

```python
import pandas as pd

# 读取最新的实验数据
df = pd.read_csv('experiments/results/batch_test_log_20251231_101326.csv')

# 找出碰撞点最多的前5个实验
top_collisions = df.nlargest(5, 'Strict_Collision_Count')
print(top_collisions[['Experiment_ID', 'Seed', 'Strict_Collision_Count']])
```

输出：
```
   Experiment_ID  Seed  Strict_Collision_Count
4              5     5                    127
12            15    15                      89
...
```

然后可视化这些实验：
```bash
python src/run_simulation.py --seed 5
python src/run_simulation.py --seed 15
```

### 技巧2：创建调试脚本

创建 `debug_exp.sh`（Linux/Mac）或 `debug_exp.bat`（Windows）：

**Windows (debug_exp.bat)**:
```batch
@echo off
echo Debugging experiments with high collision count...
python src/run_simulation.py --seed 5
echo.
echo Next experiment...
pause
python src/run_simulation.py --seed 15
pause
```

**Linux/Mac (debug_exp.sh)**:
```bash
#!/bin/bash
echo "Debugging experiments with high collision count..."
python src/run_simulation.py --seed 5
echo ""
echo "Next experiment..."
python src/run_simulation.py --seed 15
```

### 技巧3：记录调试会话

使用 `tee`（Linux）或重定向（Windows）保存输出：

**Windows**:
```bash
python src/run_simulation.py --seed 5 > debug_seed5.txt 2>&1
```

**Linux/Mac**:
```bash
python src/run_simulation.py --seed 5 | tee debug_seed5.txt
```

## 故障排除

### 问题1：种子不生效
**症状**：使用相同种子但结果不同

**原因**：
- 参数配置不一致（检查 `run_batch_test.py` 和 `run_simulation.py` 的参数）
- Python/NumPy 随机种子未同时设置

**解决**：确保两个文件中的参数一致，都已设置 `random.seed()` 和 `np.random.seed()`

### 问题2：可视化窗口卡死
**症状**：窗口弹出但无响应

**解决**：使用 `--no-visualization` 跳过可视化，或在后台运行

### 问题3：找不到异常实验
**症状**：CSV中看到的种子，运行后结果不同

**原因**：可能看错文件，查看的是旧的实验结果

**解决**：使用 `ls -lt`（Linux）或文件管理器确认查看的是最新的CSV文件

## 总结

通过 `--seed` 参数，你可以：
1. ✓ 精确复现批量测试中的任何实验
2. ✓ 可视化分析异常情况
3. ✓ 对比不同参数的效果
4. ✓ 调试和优化算法

这使得批量测试和单次调试形成闭环，极大提升实验效率！
