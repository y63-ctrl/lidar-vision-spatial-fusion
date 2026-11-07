# M5 管线集成 — 完成验证

> 完成日期：2026-11-07
> 对应任务：T5.1 Pipeline/CLI、T5.2 timing/summary、T5.3 错误处理/退出码、T5.4 端到端测试

---

## 1. 结论

M5 已完成。当前项目有一个统一入口：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_pipeline.ps1
```

一条命令完成：

```text
KITTI 读取
  → 图像/点云退化评估
  → 自动选择融合策略
  → 投影 + 取色 / 深度着色
  → ROI + 高度过滤 + 体素降采样
  → 2D BEV 占据栅格
  → fused.pcd / overlay.png / fused_view.png
  → degradation.json / occupancy.pgm / occupancy.yaml / occupancy.png
  → timing.json / summary.csv
```

`--all` 可连续处理多帧；`--fail-on-degraded` 在退化时返回退出码 3。

---

## 2. 新增代码

| 文件 | 内容 |
|---|---|
| `include/pipeline/Pipeline.h` / `src/pipeline/Pipeline.cpp` | 单帧端到端编排 |
| `include/core/StageTimings.h` | 各阶段耗时结构 |
| `include/io/TimingJsonWriter.h` / `.cpp` | `timing.json` |
| `include/utils/Log.h` / `src/utils/Log.cpp` | INFO/WARN/ERROR |
| `app/main.cpp` | `lidar_fusion` CLI |
| `scripts/run_pipeline.ps1` | 自动注入运行时 PATH 的 CLI 封装 |
| `tests/test_pipeline.cpp` | 端到端输出/时序/开关测试 |

---

## 3. 测试结果

```text
1/17 smoke ............ Passed
2/17 calibration ...... Passed
3/17 kitti_source ..... Passed
4/17 data_integration . Passed
5/17 projection ....... Passed
6/17 fusion ........... Passed
7/17 overlay .......... Passed
8/17 pcd_roundtrip .... Passed
9/17 image_degradation  Passed
10/17 lidar_degradation Passed
11/17 degradation_report Passed
12/17 preprocess ...... Passed
13/17 bev_grid ........ Passed
14/17 map_io .......... Passed
15/17 grid_render ..... Passed
16/17 pipeline ........ Passed
17/17 deps_check ...... Passed

100% tests passed, 0 tests failed out of 17
```

另执行了 `--clean-first` 全量重编译，再跑 CTest，仍为 17/17 通过。

---

## 4. 单帧运行

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_pipeline.ps1
```

输出：

```text
[INFO] frame 000000 | FULL_FUSION | points 20000/16764 | grid 1060/7108/51832 | 41 ms
[INFO] processed 1 frame(s); outputs in results
```

`results/`：

```text
000000_degradation.json
000000_fused.pcd
000000_fused_view.png
000000_occupancy.pgm
000000_occupancy.png
000000_occupancy.yaml
000000_overlay.png
000000_timing.json
summary.csv
```

### 4.1 `summary.csv`

```csv
frame,input_points,projected_points,colored_points,action,overall,image_confidence,lidar_confidence,map_trusted,occupied,free,unknown,read_ms,degradation_ms,fusion_ms,mapping_ms,write_ms,compute_ms,total_ms
000000,20000,16764,16764,FULL_FUSION,0.6846,0.5756,0.7937,1,1060,7108,51832,3.9456,6.3657,3.3014,0.7549,26.5126,10.4220,40.8811
```

### 4.2 `timing.json`

```json
{
  "frame": "000000",
  "read_ms": 3.9456,
  "degradation_ms": 6.3657,
  "fusion_ms": 3.3014,
  "mapping_ms": 0.7549,
  "write_ms": 26.5126,
  "compute_ms": 10.4220,
  "total_ms": 40.8811
}
```

`compute_ms` 不含写盘；`total_ms` 含读取、计算、写出。当前写盘占大头（PCD + 多张 PNG），M6 性能优化时优先处理。

---

## 5. 多帧运行

测试时把样例复制为 `000000/000001/000002` 三帧，运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_pipeline.ps1 `
  -DataRoot <seq_root> -All -Out .\results_seq -NoViz -NoPcd
```

输出：

```text
[INFO] frame 000000 | FULL_FUSION | points 20000/16764 | grid 1060/7108/51832 | 20 ms
[INFO] frame 000001 | FULL_FUSION | points 20000/16764 | grid 1060/7108/51832 | 12 ms
[INFO] frame 000002 | FULL_FUSION | points 20000/16764 | grid 1060/7108/51832 | 11 ms
[INFO] processed 3 frame(s); outputs in results_seq
```

`summary.csv` 有 1 行 header + 3 行数据；`--no-viz --no-pcd` 时每帧只写 degradation/occupancy/timing。

---

## 6. CLI 与退出码

```text
lidar_fusion --data-root <dir> --frame 000000 --out results
lidar_fusion --data-root <dir> --all --out results --no-viz --no-pcd
lidar_fusion ... --mode pinhole|depth|auto
lidar_fusion ... --fail-on-degraded
```

| 退出码 | 含义 | 验证 |
|---:|---|---|
| 0 | 成功 | 样例单帧/三帧 |
| 1 | 参数错误 | `--unknown-option` |
| 2 | 数据/处理/IO 错误 | 缺失 data root |
| 3 | `--fail-on-degraded` 且退化 | 全黑图像样例 |

退化阻断测试：把样例图像替换为全黑图，运行 `--fail-on-degraded`，输出：

```text
[INFO] frame 000000 | GEOMETRY_ONLY | points 20000/16764 | grid 1060/7108/51832 | 28 ms
[WARN] degradation blocked by --fail-on-degraded
```

退出码为 3，同时仍写出 degradation/occupancy/timing，便于事后分析。

---

## 7. Pipeline 契约

`Pipeline::run(frame, readMs)` 返回 `FrameRunResult`：

- `degradation`：完整报告；
- `timing`：各阶段耗时；
- `input/projected/colored` 点数；
- `occupied/free/unknown` 栅格统计；
- `outputs`：实际写出的文件路径。

阶段顺序固定：

```text
degradation → fusion → mapping → write
```

`--no-viz` 仅跳过 overlay/fused_view/occupancy.png；`--no-pcd` 仅跳过 PCD；栅格 PGM/YAML、degradation.json、timing.json 始终写出。

---

## 8. 验收清单

| 验收项 | 状态 | 证据 |
|---|---|---|
| Pipeline 编排 | 通过 | `pipeline` 测试 |
| CLI 单帧 | 通过 | `run_pipeline.ps1` |
| CLI 多帧 | 通过 | 3 帧 `--all` |
| 输出目录/全部文件 | 通过 | 9 个文件 |
| `timing.json` | 通过 | 单帧 JSON |
| `summary.csv` | 通过 | 1 header + 3 rows |
| `--no-viz` / `--no-pcd` | 通过 | 单元测试 + CLI |
| 退出码 0/1/2/3 | 通过 | 四类场景 |
| 日志/错误上下文 | 通过 | INFO/WARN/ERROR |
| 端到端测试 | 通过 | `pipeline` CTest |

---

## 9. 已知限制

1. **`summary.csv` 每次运行覆盖**，不做多运行聚合；M6 可加 mean/median/p95 的报告。
2. **写盘占比高**（PCD + 多张 PNG），当前 40.9ms 总耗时里 write 约 26.5ms；M6 可优化。
3. **首帧冷启动更慢**，后续帧约 11–20ms（取决于是否写可视化）。
4. **没有 compare.png 三联图**；M6 生成。
5. **没有真实 KITTI 回归**；样例仍为合成数据。
6. **`--fail-on-degraded` 仍会写结果**，只是返回 3；这是为了保留诊断材料。

---

## 10. 下一步（M6）

M6 验证交付：

```text
T6.1 性能/内存验证：compute/end-to-end、线程扩展、ASan
T6.2 README 三联图、演示材料
T6.3 repo polish / 可选 CI
T6.4 最终验收 + v0.1.0 tag
```
