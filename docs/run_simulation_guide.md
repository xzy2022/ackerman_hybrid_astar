# Hybrid A* Simulation - 命令行参数说明文档

本文档详细说明 `src/run_simulation.py` 的所有命令行参数及其使用场景。

---

## 📋 参数列表

### 基础参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--seed` | int | `None` | 随机种子，用于复现实验结果 |
| `--no-visualization` | flag | `False` | 禁用可视化（用于批量测试） |

### 性能控制参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--log-tree` | flag | `False` | 记录并可视化搜索树（绿色=Closed Set，蓝色=Open Set） |
| `--log-closed` | flag | `False` | 记录闭集代价（用于生成热力图和动画） |
| `--sample-rate` | int | `10` | 采样率，每 N 个节点记录 1 个（1=全量记录，10=记录1/10） |
| `--animate-search` | flag | `False` | 播放搜索过程动画（需要 `--log-closed`） |

---

## 🎯 使用场景示例

### 场景 1：快速测试（性能优先）

**适用**：调试代码、快速验证功能、批量实验

```bash
# 基础运行（无搜索树，无热力图）
python src/run_simulation.py --seed 42

# 无可视化（纯性能测试）
python src/run_simulation.py --seed 42 --no-visualization

# 批量测试多个种子
for seed in 1 2 3 4 5; do
    python src/run_simulation.py --seed $seed --no-visualization
done
```

**特点**：
- ✅ 最快速度
- ❌ 无搜索树可视化
- ❌ 无热力图分析
- 采样率：10（记录 1/10 节点）

---

### 场景 2：查看搜索过程（调试用）

**适用**：分析算法行为、理解搜索策略、查找失败原因

```bash
# 显示搜索树（绿色=已探索，蓝色=候选前沿）
python src/run_simulation.py --seed 42 --log-tree

# 高密度搜索树（全量记录）
python src/run_simulation.py --seed 42 --log-tree --sample-rate 1

# 稀疏搜索树（快速预览）
python src/run_simulation.py --seed 42 --log-tree --sample-rate 50
```

**输出**：
- 🟢 **绿色点云**：Closed Set（已探索节点）
- 🔵 **蓝色点云**：Open Set（候选前沿）
- 🔴 **红色线条**：最终路径

---

### 场景 3：静态代价对比分析（论文插图）

**适用**：生成论文配图、分析启发式有效性、对比实际搜索与理想指引

```bash
# 左图：启发式场 | 右图：实际搜索代价
python src/run_simulation.py --seed 42 --log-closed --log-tree

# 高质量对比图（全量记录）
python src/run_simulation.py --seed 42 --log-closed --log-tree --sample-rate 1

# 平衡模式（记录 1/5 节点）
python src/run_simulation.py --seed 42 --log-closed --log-tree --sample-rate 5
```

**输出**：
- **左图**：启发式场 (h) - 理想的距离指引
- **右图**：实际搜索代价 (f = g + h) - 真实的搜索波前

---

### 场景 4：动态搜索过程动画（演示用）

**适用**：教学演示、视频制作、直观展示算法扩展过程

```bash
# 播放搜索动画（左图静态参考，右图动态扩展）
python src/run_simulation.py --seed 42 --log-closed --animate-search

# 完整记录 + 动画（高质量）
python src/run_simulation.py --seed 42 --log-closed --log-tree --sample-rate 1 --animate-search

# 快速动画（采样率 10，减少节点数）
python src/run_simulation.py --seed 42 --log-closed --animate-search --sample-rate 10
```

**动画说明**：
- **左图**：静态启发式场（参考）
- **右图**：动态 Closed List 扩展过程
  - 初始：全白（未探索）
  - 过程：彩色区域从起点向外扩散
  - 结束：显示完整搜索波前

**性能调优**：
- 修改 `src/run_simulation.py` 第 170 行的 `batch_size`：
  - `batch_size=10`：更平滑，较慢
  - `batch_size=20`：平衡（默认）
  - `batch_size=50`：更快，可能跳帧

---

### 场景 5：完整调试模式（开发用）

**适用**：深度调试、详细分析、问题定位

```bash
# 记录所有数据 + 搜索树 + 热力图 + 动画
python src/run_simulation.py --seed 42 --log-closed --log-tree --sample-rate 1 --animate-search

# 包含失败诊断和严格碰撞检测
python src/run_simulation.py --seed 5 --log-closed --log-tree --sample-rate 1
```

**输出**：
1. 终端诊断信息（失败时）
2. 搜索树可视化（绿点 + 蓝点）
3. 搜索过程动画（1x2 对比图）
4. 严格碰撞检测结果

---

### 场景 6：失败案例分析（问题诊断）

**适用**：分析规划失败原因、验证起终点设置、诊断启发式问题

```bash
# 失败案例（seed=5 通常会失败）
python src/run_simulation.py --seed 5 --log-tree

# 失败时自动显示：
# - 节点扩展数
# - 起点/终点碰撞检测
# - 启发式连通性检查
# - Heuristic 热力图（分析死胡同）
```

**诊断输出示例**：
```
  PLANNING FAILED - DIAGNOSTICS
========================================
  - Nodes Expanded: 1234
  - Start Pose Collision: False [OK]
  - Goal Pose Collision: False [OK]
  - Checking Holonomic Heuristic (Dijkstra)...
    Heuristic value from Start: 84.84
    [OK] Path exists in 2D Grid (vehicle treated as point mass).
    If planning failed, likely cause: Vehicle body collision
              (car's front/rear hits obstacles despite clear path for center).
========================================
```

---

## 📊 参数组合对照表

| 场景 | `--seed` | `--log-tree` | `--log-closed` | `--sample-rate` | `--animate-search` | 内存占用 | 速度 |
|------|---------|--------------|----------------|-----------------|-------------------|---------|------|
| 快速测试 | ✅ | ❌ | ❌ | 10 | ❌ | 低 | ⚡⚡⚡ |
| 搜索树预览 | ✅ | ✅ | ❌ | 10-50 | ❌ | 中 | ⚡⚡ |
| 高质量搜索树 | ✅ | ✅ | ❌ | 1 | ❌ | 高 | ⚡ |
| 静态对比图 | ✅ | ✅ | ✅ | 5-10 | ❌ | 高 | ⚡ |
| 搜索动画 | ✅ | ❌ | ✅ | 10 | ✅ | 高 | ⚡ |
| 完整调试 | ✅ | ✅ | ✅ | 1 | ✅ | 极高 | 慢 |

---

## 🎨 可视化效果说明

### 1. 搜索树可视化 (`--log-tree`)

**图例**：
- 🟢 **绿色点**：Closed Set（已扩展节点）
- 🔵 **蓝色点**：Open Set（候选前沿节点）
- 🔴 **红色线**：最终路径
- 🟢 **绿色车**：起点
- 🔴 **红色车**：终点

**用途**：观察算法如何从起点向终点探索，蓝色"触须"显示搜索方向。

### 2. 静态代价对比图 (`--log-closed`)

**左图 - 启发式场 (h)**：
- 显示理想的欧几里得距离（考虑障碍物）
- 蓝色 = 远离目标，红色 = 接近目标
- 作为"完美导航指引"的参考

**右图 - 实际搜索代价 (f = g + h)**：
- 显示实际访问节点的总代价
- 白色 = 未访问区域
- 彩色 = 已探索区域（红色 = 低代价）

**用途**：对比启发式的理论指引与实际搜索的差异，验证启发式质量。

### 3. 搜索过程动画 (`--animate-search`)

**动画特点**：
- 左图：静态启发式场（不变）
- 右图：Closed List 动态扩展
- 从白色逐渐填充彩色
- 标题显示当前扩展节点数：`Search Process: 123 / 5000 Nodes`

**用途**：直观展示搜索波前如何扩散，适合教学和演示。

---

## ⚖️ 性能与质量权衡

### 采样率 (`--sample-rate`) 的影响

| 采样率 | 记录节点数 | 内存占用 | 绘图速度 | 适用场景 |
|--------|-----------|---------|---------|---------|
| `1` | 100% | 极高 | 慢 | 论文插图、高精度分析 |
| `5` | 20% | 高 | 中 | 详细调试、高质量可视化 |
| `10` | 10% | 中 | 快 | 日常开发、平衡模式（默认） |
| `50` | 2% | 低 | 极快 | 快速预览、大规模搜索 |
| `100` | 1% | 极低 | 极快 | 批量测试、性能优先 |

**计算公式**：
```
实际记录节点数 = 总扩展节点数 / sample_rate
```

**示例**：
- 扩展 5000 节点，sample-rate=10 → 记录 500 个节点
- 扩展 5000 节点，sample-rate=50 → 记录 100 个节点

---

## 🔧 常见问题

### Q1: 为什么加了 `--log-tree` 还是看不到搜索树？

**A**: 检查是否同时设置了采样率过高。例如：
```bash
# ❌ 错误：采样率太高，点太少
python src/run_simulation.py --seed 42 --log-tree --sample-rate 100

# ✅ 正确：降低采样率
python src/run_simulation.py --seed 42 --log-tree --sample-rate 10
```

### Q2: 动画播放太慢/太快怎么办？

**A**: 修改 `src/run_simulation.py` 第 170 行的 `batch_size`：
```python
# 第 170 行
batch_size = 20  # 改为 10（更慢更平滑）或 50（更快）
```

### Q3: 失败时如何查看热力图？

**A**: 失败时会自动显示 Heuristic 热力图，无需额外参数。但如果想看 Closed List，需要：
```bash
# 失败时也记录 Closed List
python src/run_simulation.py --seed 5 --log-closed
```

### Q4: 如何生成论文级别的配图？

**A**: 使用全量记录：
```bash
python src/run_simulation.py --seed 42 --log-closed --log-tree --sample-rate 1
```
然后：
1. 等待搜索树图弹出，关闭
2. 等待代价对比图弹出，截图保存
3. 如果加了 `--animate-search`，等待动画播放完成

### Q5: 批量实验时如何加速？

**A**: 使用高采样率 + 无可视化：
```bash
python src/run_simulation.py --seed 42 --sample-rate 100 --no-visualization
```

---

## 📝 最佳实践

### 开发阶段
```bash
# 快速迭代
python src/run_simulation.py --seed 42
```

### 调试阶段
```bash
# 查看搜索树 + 失败诊断
python src/run_simulation.py --seed 42 --log-tree
```

### 论文配图
```bash
# 高质量静态图
python src/run_simulation.py --seed 42 --log-closed --log-tree --sample-rate 1
```

### 演示视频
```bash
# 录制搜索动画（使用录屏软件）
python src/run_simulation.py --seed 42 --log-closed --animate-search
```

### 批量测试
```bash
# 自动化测试
for seed in {1..100}; do
    python src/run_simulation.py --seed $seed --no-visualization --sample-rate 100
done
```

---

## 🔗 相关文件

- **实现文件**: `src/run_simulation.py`
- **规划器**: `src/planner.py`
- **可视化器**: `src/visualizer.py`
- **配置文件**: `src/config.py`

---

**版本**: v1.0
**最后更新**: 2025-12-31
**维护者**: Hybrid A* Research Team
