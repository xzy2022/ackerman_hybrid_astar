# 实验系统使用指南

本目录包含 Hybrid A* 算法的自动化批量测试实验系统。

## 目录结构

```
experiments/
├── results/                  # 实验结果输出目录
│   ├── batch_test_log.csv    # 原始实验数据
│   └── summary_report.txt    # 统计汇总报告
├── docs/                     # 实验文档
│   └── experiment_setup.md   # 详细实验说明
├── run_batch_test.py         # 批量测试脚本（核心）
└── README.md                 # 本文件
```

## 快速开始

### 1. 运行批量测试

```bash
# 在项目根目录下执行
python experiments/run_batch_test.py
```

### 2. 查看结果

实验完成后，查看 `experiments/results/` 目录：

- **batch_test_log_YYYYMMDD_HHMMSS.csv** - 原始数据
- **summary_report_YYYYMMDD_HHMMSS.txt** - 汇总报告

### 3. 预期输出示例

```
[Exp 001/100 | Seed 1] SUCCESS | Time: 245.3ms | Nodes: 1245 | Collisions: 0
[Exp 002/100 | Seed 2] SUCCESS | Time: 189.7ms | Nodes: 987 | Collisions: 0
[Exp 003/100 | Seed 3] FAILED  | Time: 500.0ms | Nodes: 5000
...
```

## 实验配置

### 默认参数

在 `run_batch_test.py` 中修改以下参数：

```python
NUM_EXPERIMENTS = 100          # 实验次数
START_SEED = 1                 # 起始种子
START_POSE = (10.0, 10.0, 0.0) # 起始位姿
GOAL_POSE = (40.0, 40.0, 90.0)  # 目标位姿
```

### 算法参数

修改 HybridAStarConfig 的参数：

```python
h_config = HybridAStarConfig()
h_config.heuristic_weight = 5.0  # 启发式权重
h_config.move_step_grid = 2.0    # 步长倍率
```

## 关键指标说明

### 1. 成功率 (Success Rate)
- 规划器找到可行路径的比例
- 预期：80-95%

### 2. 规划时间 (Planning Time)
- 单次规划的平均耗时
- 预期：100-500ms

### 3. 安全违规率 (Safety Violation Rate)
- **重要指标**：圆形近似模型的精度评估
- 规划成功但严格碰撞检测发现碰撞的比例
- 预期：0-10%

## 数据分析

### 使用 Python 分析

```python
import pandas as pd
import glob

# 读取最新的实验数据
csv_files = glob.glob('experiments/results/batch_test_log_*.csv')
latest_csv = max(csv_files)  # 获取最新的文件

df = pd.read_csv(latest_csv)

# 成功率
success_rate = df['Success'].mean() * 100
print(f"成功率: {success_rate:.2f}%")

# 平均规划时间
avg_time = df[df['Success']]['Planning_Time_ms'].mean()
print(f"平均时间: {avg_time:.2f}ms")

# 安全违规率
unsafe = df[(df['Success']) & (df['Strict_Collision_Count'] > 0)]
unsafe_rate = len(unsafe) / len(df[df['Success']]) * 100
print(f"违规率: {unsafe_rate:.2f}%")
```

### 使用 Excel 分析

1. 打开 `experiments/results/` 目录
2. 找到最新的 `batch_test_log_YYYYMMDD_HHMMSS.csv`
3. 双击打开，使用数据透视表或公式统计
4. 绘制图表（成功率、时间分布等）

## 常见问题

### Q1: 实验中断了怎么办？
A: 实验数据已实时保存到 CSV（文件名带时间戳，不会覆盖之前的实验）。可查看已完成的部分结果。从中断处继续，修改起始种子即可。

### Q2: 如何找到最新的实验结果？
A: 在 `experiments/results/` 目录中，文件按时间排序，最后的就是最新的。或者使用文件名中的时间戳来识别。

### Q3: 旧的结果会被覆盖吗？
A: **不会**。每个实验的结果文件都有唯一的时间戳，可以保存多次实验的结果用于对比。

### Q4: 如何调试单次实验？
A: 使用 `src/run_simulation.py` 并指定相同的种子来复现批量测试中的某次实验：

```bash
# 复现种子为5的实验，并可视化
python src/run_simulation.py --seed 5

# 复现但不显示可视化窗口（快速测试）
python src/run_simulation.py --seed 5 --no-visualization

# 不指定种子（随机初始化）
python src/run_simulation.py
```

**支持的参数**：
- `--seed N`：指定随机种子（N为整数，如1、5、42等）
- `--no-visualization`：不显示可视化窗口（用于快速测试）

这样可以：
- 复现批量测试中特定某次实验的场景
- 可视化观察该次实验的规划过程
- 调试和分析异常情况

### Q5: 为什么有些实验失败？
A: 可能原因：
- 障碍物密度过高，无可行路径
- 起终点被障碍物包围（虽然已清理，但极端情况仍可能发生）
- 算法参数过于保守

### Q6: 如何测试不同场景？
A: 修改 `START_POSE` 和 `GOAL_POSE`，或调整 `obstacle_num` 参数。

## 扩展实验

### 参数敏感性分析

创建多个配置文件，对比不同参数的性能：

```python
configs = [
    {"heuristic_weight": 3.0, "name": "low_h"},
    {"heuristic_weight": 5.0, "name": "medium_h"},
    {"heuristic_weight": 10.0, "name": "high_h"},
]

for cfg in configs:
    run_experiment(cfg)
```

### 场景测试

测试特定场景（如狭窄通道、平行泊车）：
- 修改地图生成逻辑
- 设置特定的起终点和障碍物布局

## 性能优化建议

如果实验运行较慢：

1. **减少实验次数**: `NUM_EXPERIMENTS = 50`
2. **降低障碍物密度**: `obstacle_num = 100`
3. **增大栅格分辨率**: `xy_resolution = 1.0`
4. **使用更快的硬件**: 确保使用 SSD，增加内存

## 联系方式

如有问题或建议，请查看项目主 README 或提交 Issue。

---

**版本**: v1.0
**最后更新**: 2025-12-31
