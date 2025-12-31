
常用命令
```bash
python src/run_simulation.py --seed 4 --log-tree --log-closed  --sample-rate 1 --animate-search
python d:/Workspace/00_MyRepo/ackerman_hybrid_astar/experiments/run_batch_test.py
```
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
