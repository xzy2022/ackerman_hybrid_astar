# 批量测试脚本改进说明

## 改进日期
2025-12-31

## 改进内容

基于专业的代码审查意见，对 `experiments/run_batch_test.py` 进行了关键改进，提升了实验数据的科学性和可信度。

---

## 1. 计时精度改进 ✓

### 修改前
```python
start_time = time.time()  # 系统挂钟时间
execution_time_ms = (time.time() - start_time) * 1000.0
```

### 修改后
```python
start_time = time.perf_counter()  # 高精度单调时钟
end_time = time.perf_counter()
execution_time_ms = (end_time - start_time) * 1000.0
```

### 改进效果
- ✓ 使用专门为性能测量设计的 `perf_counter()`
- ✓ 不受系统时间调整（NTP同步）影响
- ✓ 跨平台一致性更好
- ✓ 分辨率更高（微秒级）

---

## 2. 指标分离 - 物理长度 vs 混合代价 ✓

### 问题诊断
原代码将"混合代价"和"路径长度"混淆：

```python
# 原来的问题
"path_cost": result.cost,  # 混合值（g + h + 惩罚）
"path_length_steps": len(result.path_x)  # 只是点数，不是物理长度
```

**问题**：
- `result.cost` 包含了物理距离 + 控制惩罚（倒车、转向等）
- 无法直观判断"车到底走了多远"
- 调整惩罚参数后，不知道代价增加是因为路径变长还是惩罚变大

### 改进方案
添加了 **三个独立的指标**：

#### ① Path_Cost（混合代价）
```python
"path_cost": result.cost,  # g + h + penalties
```
**用途**：评估算法优化目标的达成情况

#### ② Path_Length_m（物理长度）
```python
def calculate_path_length_meters(path_x, path_y):
    """计算路径的几何物理长度"""
    length = 0.0
    for i in range(len(path_x) - 1):
        dx = path_x[i+1] - path_x[i]
        dy = path_y[i+1] - path_y[i]
        length += math.hypot(dx, dy)
    return length

path_length_m = calculate_path_length_meters(result.path_x, result.path_y)
```
**用途**：评估路径的物理长度（米），直观反映"车走了多远"

#### ③ Steps_Count（路径点数）
```python
"steps_count": len(result.path_x)  # 路径离散点数量
```
**用途**：评估路径的离散化程度

### 改进效果
现在 CSV 输出包含完整的三个维度：

| Experiment_ID | ... | Path_Cost | Path_Length_m | Steps_Count |
|:---|:---|:---|:---|:---|
| 1 | ... | 150.06 | 90.80 | 455 |

**数据分析示例**：

```python
import pandas as pd

df = pd.read_csv('batch_test_log_YYYYMMDD_HHMMSS.csv')

# 评估路径长度（物理）
print(f"平均路径长度: {df['Path_Length_m'].mean():.1f} 米")

# 评估混合代价（优化目标）
print(f"平均混合代价: {df['Path_Cost'].mean():.1f}")

# 评估效率（代价/长度比）
efficiency = df['Path_Cost'] / df['Path_Length_m']
print(f"平均效率: {efficiency.mean():.2f} 代价/米")
```

---

## 3. CSV 表头更新 ✓

### 修改前
```csv
Experiment_ID,Seed,Success,Planning_Time_ms,Nodes_Expanded,Path_Cost,Strict_Collision_Count,Path_Length_Steps
```

### 修改后
```csv
Experiment_ID,Seed,Success,Planning_Time_ms,Nodes_Expanded,Path_Cost,Path_Length_m,Steps_Count,Strict_Collision_Count
```

### 字段说明
| 字段 | 类型 | 说明 |
|:---|:---|:---|
| `Experiment_ID` | int | 实验编号 (1-100) |
| `Seed` | int | 随机种子 |
| `Success` | bool | 是否规划成功 |
| `Planning_Time_ms` | float | 规划耗时（毫秒，高精度） |
| `Nodes_Expanded` | int | 扩展节点数 |
| `Path_Cost` | float | **混合代价** (g + h + penalties) |
| `Path_Length_m` | float | **物理长度**（米） |
| `Steps_Count` | int | **路径点数** |
| `Strict_Collision_Count` | int | 严格碰撞检测发现的碰撞点数 |

---

## 4. 报告输出优化 ✓

### 统计报告新增指标

```
Performance Metrics (Success Cases Only):
  Planning Time:
    Average: 11714.77 ms
    Min:     11714.77 ms
    Max:     11714.77 ms
  Nodes Expanded (Average): 8144.0
  Path Cost (Mixed): 150.06
    (g + h + penalties)
  Path Length (Physical): 90.80 m      ← 新增！
  Path Steps (Average): 455.0           ← 新增！
```

---

## 5. 数据分析示例

### 示例1：评估路径绕行程度
```python
import pandas as pd

df = pd.read_csv('batch_test_log_YYYYMMDD_HHMMSS.csv')
df_success = df[df['Success']]

# 计算直线距离（起点到终点）
straight_distance = 42.43  # sqrt((40-10)^2 + (40-10)^2)

# 计算绕行率
detour_rate = (df_success['Path_Length_m'] / straight_distance - 1) * 100
print(f"平均绕行率: {detour_rate.mean():.1f}%")
```

### 示例2：分析控制惩罚影响
```python
# 分离物理距离和控制惩罚
df_success['control_penalty'] = df_success['Path_Cost'] - df_success['Path_Length_m']

print(f"平均控制惩罚: {df_success['control_penalty'].mean():.1f}")
print(f"惩罚占比: {(df_success['control_penalty'] / df_success['Path_Cost']).mean():.1%}")
```

### 示例3：效率分析
```python
# 计算单位长度代价
df_success['cost_per_meter'] = df_success['Path_Cost'] / df_success['Path_Length_m']

high_efficiency = df_success[df_success['cost_per_meter'] < 2.0]
low_efficiency = df_success[df_success['cost_per_meter'] > 3.0]

print(f"高效率路径: {len(high_efficiency)} 条")
print(f"低效率路径: {len(low_efficiency)} 条")
```

---

## 6. 向后兼容性说明

### 注意
- **旧的 CSV 文件** (`batch_test_log.csv`，无时间戳) 使用旧的表头
- **新的 CSV 文件** (`batch_test_log_YYYYMMDD_HHMMSS.csv`) 使用新的表头
- 分析脚本需要根据表头名称进行调整

### 兼容性处理
如需兼容旧数据，使用 `pandas` 读取时检测列名：

```python
import pandas as pd

df = pd.read_csv('batch_test_log_YYYYMMDD_HHMMSS.csv')

# 检查是否有新列
if 'Path_Length_m' in df.columns:
    # 新格式
    path_lengths = df['Path_Length_m']
    steps_count = df['Steps_Count']
else:
    # 旧格式
    path_lengths = None
    steps_count = df['Path_Length_Steps']
```

---

## 7. 总结

### 改进带来的价值
1. **更精确的计时**：使用 `perf_counter()` 确保时间测量可靠
2. **更清晰的指标分离**：
   - Path_Cost = 优化目标（混合）
   - Path_Length_m = 物理指标（直观）
   - Steps_Count = 离散化指标（算法相关）
3. **更科学的分析**：可以独立分析路径长度和代价
4. **更灵活的对比**：调整参数时能区分是路径变了还是惩罚变了

### 适用的研究场景
- **参数敏感性分析**：改变惩罚参数，观察 Path_Cost 变化但 Path_Length_m 不变
- **路径质量评估**：比较不同算法在相同起终点下的 Path_Length_m
- **效率分析**：计算 Cost/Meter 比率，评估算法优化效率

---

**改进者**: AI Code Reviewer
**批准者**: User
**版本**: v2.0
**状态**: ✅ 已测试并部署
