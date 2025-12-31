# 批量测试实验系统 - 快速开始

## 🚀 快速运行

```bash
# 在项目根目录下
cd D:\Workspace\00_MyRepo\ackerman_hybrid_astar
python experiments/run_batch_test.py
```

## 📊 预期输出

运行过程中会实时显示进度：
```
[Exp 001/100 | Seed 1] SUCCESS | Time: 245.3ms | Nodes: 1245 | Collisions: 0
[Exp 002/100 | Seed 2] SUCCESS | Time: 189.7ms | Nodes: 987 | Collisions: 5
...
```

## 📁 结果文件

实验完成后，查看 `experiments/results/` 目录：

### 1. batch_test_log.csv（原始数据）
可用 Excel 或 Python 分析，包含每条实验的详细数据。

### 2. summary_report.txt（汇总报告）
自动生成的统计报告，关键指标包括：

- **Success Rate**: 规划成功率
- **Average Planning Time**: 平均规划时间
- **Safety Violation Rate**: ⚠️ **关键指标** - 圆形近似模型的精度评估
  - 规划成功但严格检测发现碰撞的比例
  - 反映圆形近似模型是否足够保守

## ⚠️ 重要发现

从测试实验中发现：
```
Unsafe Plans: 1 (100.0%)
Collisions: 63 points
```

这说明**圆形近似模型存在精度不足**！

虽然规划器认为路径安全，但严格的多边形检测发现了63个碰撞点。

## 📝 如何分析结果

### 方法1: 使用 Python
```python
import pandas as pd

df = pd.read_csv('experiments/results/batch_test_log.csv')

# 关键指标
print(f"成功率: {df['Success'].mean() * 100:.1f}%")
print(f"平均时间: {df['Planning_Time_ms'].mean():.1f}ms")

# 安全违规分析
unsafe = df[(df['Success']) & (df['Strict_Collision_Count'] > 0)]
print(f"违规率: {len(unsafe) / len(df[df['Success']]) * 100:.1f}%")
```

### 方法2: 使用 Excel
1. 打开 `experiments/results/batch_test_log.csv`
2. 使用数据透视表统计
3. 绘制图表

## 🔧 参数调整

### 修改实验次数
编辑 `run_batch_test.py`:
```python
NUM_EXPERIMENTS = 50  # 从100改为50
```

### 修改算法参数
```python
h_config = HybridAStarConfig()
h_config.heuristic_weight = 10.0  # 修改启发式权重
```

### 修改起终点
```python
START_POSE = (5.0, 5.0, math.radians(45.0))
GOAL_POSE = (45.0, 45.0, math.radians(135.0))
```

## 📖 详细文档

查看 `experiments/docs/experiment_setup.md` 了解完整的实验设计和参数说明。

## 🐛 调试单次实验

如果某次实验结果异常，可单独运行：

```bash
# 使用相同的种子
python src/run_simulation.py
```

或修改 `run_batch_test.py` 中的 `NUM_EXPERIMENTS = 1`，指定特定的种子。

## 💡 下一步

基于实验结果，你可以：

1. **调整圆形碰撞模型参数**
   - 增加碰撞圆的数量
   - 增大碰撞圆的半径
   - 优化圆的分布策略

2. **参数优化**
   - 尝试不同的启发式权重
   - 调整分辨率
   - 修改代价函数权重

3. **对比实验**
   - 对比不同参数的性能
   - 测试不同场景（狭窄通道、泊车等）

4. **论文撰写**
   - 使用统计数据生成图表
   - 分析安全违规率的根本原因
   - 提出改进方案

## ⏱️ 预计运行时间

- 单次实验: 约 10-30 秒（取决于地图复杂度）
- 100次实验: 约 20-50 分钟
- 可随时中断（Ctrl+C），已完成的数据会自动保存

---

**Happy Experimenting! 🎯**
