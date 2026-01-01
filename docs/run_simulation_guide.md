# Hybrid A* Simulation - 命令行参数说明文档

本文档详细说明 `src/run_simulation.py` 的所有命令行参数及使用示例。

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

```bash
# 基础运行（无搜索树，无热力图）
python src/run_simulation.py --seed 42

# 无可视化（纯性能测试）
python src/run_simulation.py --seed 42 --no-visualization

# 显示搜索树（绿色=已探索，蓝色=候选前沿）
python src/run_simulation.py --seed 42 --log-tree

# 高密度搜索树（全量记录）
python src/run_simulation.py --seed 42 --log-tree --sample-rate 1

# 稀疏搜索树（快速预览）
python src/run_simulation.py --seed 42 --log-tree --sample-rate 50

# 播放搜索动画（左图静态参考，右图动态扩展）
# --animate-search 需要 --log-closed 并受 --sample-rate 影响
python src/run_simulation.py --seed 42 --log-closed --animate-search

```

## 🔗 相关文件

- **实现文件**: `src/run_simulation.py`
- **规划器**: `src/planner.py`
- **可视化器**: `src/visualizer.py`
- **配置文件**: `src/config.py`

---

**版本**: v1.0
**最后更新**: 2026-01-01
**维护者**: Hybrid A* Research Team
