# M3 退化评估 — 完成验证

> 完成日期：2026-11-07
> 对应任务：T3.1 图像退化指标、T3.2 点云退化指标、T3.3 报告聚合与 JSON

---

## 1. 结论

M3 已完成。当前项目可以：

- 计算图像均值、标准差、直方图熵、暗像素比例、过曝比例；
- 计算点云近场点数、方位角覆盖、FOV 点数、密度分数；
- 输出图像/点云置信度、稳定 reason code；
- 聚合 `FULL_FUSION / GEOMETRY_ONLY / IMAGE_ONLY / BOTH_DEGRADED` 四种动作；
- 输出 `degradation.json`，包含 `overall`、`action`、`map_trusted` 和各项 metrics；
- 用退化动作驱动 `FusionStrategyFactory`，`fuse_frame` 默认进入 `auto` 模式。

---

## 2. 新增代码

| 文件 | 内容 |
|---|---|
| `include/degradation/ImageDegradationCheck.h` / `.cpp` | 图像指标与置信度 |
| `include/degradation/LidarDegradationCheck.h` / `.cpp` | 点云密度/角覆盖/FOV |
| `include/degradation/DegradationEvaluator.h` / `.cpp` | 双模态聚合、action、map_trusted |
| `include/io/DegradationJsonWriter.h` / `.cpp` | 稳定 JSON 输出 |
| `tools/degradation_bench.cpp` | 热循环性能基准 |
| `tests/test_image_degradation.cpp` | 正常/暗光/过曝/低对比/空图 |
| `tests/test_lidar_degradation.cpp` | 稠密/稀疏/低覆盖/空点云 |
| `tests/test_degradation_report.cpp` | 四种 action、JSON 写出 |

---

## 3. 测试结果

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1
```

```text
1/12 smoke ............ Passed
2/12 calibration ...... Passed
3/12 kitti_source ..... Passed
4/12 data_integration . Passed
5/12 projection ....... Passed
6/12 fusion ........... Passed
7/12 overlay .......... Passed
8/12 pcd_roundtrip .... Passed
9/12 image_degradation  Passed
10/12 lidar_degradation Passed
11/12 degradation_report Passed
12/12 deps_check ...... Passed

100% tests passed, 0 tests failed out of 12
```

另执行了 `cmake --build build --config Release --clean-first` 全量重编译，再跑 CTest，仍为 12/12 通过。

### 3.1 图像测试覆盖

| 场景 | 期望 |
|---|---|
| 正常纹理 | 不退化，confidence > 0.35 |
| 全黑 | degraded，`DARK_IMAGE` + `LOW_CONTRAST` |
| 全白 | degraded，`OVEREXPOSED` |
| 低对比度 120/121 | degraded，`LOW_CONTRAST` |
| 空图像 | 抛 `std::invalid_argument` |

### 3.2 点云测试覆盖

| 场景 | 期望 |
|---|---|
| 20,000 点全环 | 不退化，coverage > 0.95，FOV > 500 |
| 100 点 | degraded，`SPARSE_POINTS` |
| 1,000 点集中在 0.05 rad | degraded，`LOW_COVERAGE` |
| 空点云 | degraded，`EMPTY_CLOUD`，confidence=0 |

### 3.3 报告测试覆盖

| 输入 | action | map_trusted |
|---|---|---|
| 正常图像 + 稠密点云 | `FULL_FUSION` | true |
| 暗光图像 + 稠密点云 | `GEOMETRY_ONLY` | true |
| 正常图像 + 稀疏点云 | `IMAGE_ONLY` | false |
| 暗光图像 + 稀疏点云 | `BOTH_DEGRADED` | false |

JSON 写出后检查了 `frame`、`action`、`map_trusted`、`image`、`lidar`、`metrics`、`reasons` 字段，也检查了退化样例中的 `DARK_IMAGE` reason；实际文件用 `python -m json.tool` 验证语法。

---

## 4. 样例运行

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\fuse_frame.ps1
```

输出：

```text
source: KittiDataSource
frame: 000000
degradation: FULL_FUSION (overall=0.684623, 6.30 ms)
strategy: auto(FULL_FUSION)
input points: 20000
projected points: 16764
colored points: 16764
elapsed: 30.61 ms
```

> 6.30 ms 是 `fuse_frame` 进程内首次调用的冷启动时间，包含 OpenCV/OpenMP 运行时初始化；热循环中 evaluator 中位数见第 5 节。

### 4.1 `degradation.json`

```json
{
  "frame": "000000",
  "image": {
    "confidence": 0.5756,
    "degraded": false,
    "reasons": [],
    "metrics": {
      "bright_ratio": 0.0047,
      "contrast_score": 0.4108,
      "dark_ratio": 0.0000,
      "entropy": 0.5704,
      "exposure_score": 0.9953,
      "mean": 111.6713,
      "stddev": 26.2894
    }
  },
  "lidar": {
    "confidence": 0.7937,
    "degraded": false,
    "reasons": [],
    "metrics": {
      "azimuth_coverage": 0.8889,
      "density_score": 0.7302,
      "fov_points": 16764.0000,
      "near_points": 14604.0000,
      "total_points": 20000.0000
    }
  },
  "overall": 0.6846,
  "action": "FULL_FUSION",
  "map_trusted": true
}
```

实际输出在 `results/000000_degradation.json`；`python -m json.tool` 解析通过。

---

## 5. 性能

`tools/degradation_bench.cpp` 对同一帧做 3 次 warmup，再跑 20 次取中位数：

```text
frame points: 20000
image median: 0.45270 ms
lidar median: 0.42365 ms
evaluator median: 0.86715 ms
```

| 检查 | M3 目标 | 实测（热循环） |
|---|---:|---:|
| T3.1 图像评估 | < 2 ms | **0.45 ms** |
| T3.2 点云评估 | < 2 ms | **0.42 ms** |
| T3.3 双模态聚合 | — | **0.87 ms** |

冷启动一次性运行约 6–8 ms，主要是 OpenCV/OpenMP 初始化；M5 处理序列时使用热循环，不受单次启动影响。

---

## 6. 置信度与动作规则

### 图像

```text
contrast = clamp(stddev / 64, 0, 1)
entropy  = clamp(entropy_bits / 8, 0, 1)
exposure = 1 - max(dark_ratio, bright_ratio)
confidence = 0.5*contrast + 0.3*entropy + 0.2*exposure
```

退化条件：`confidence < 0.35` 或任一 reason 触发。

reason code：

```text
LOW_CONTRAST
DARK_IMAGE
OVEREXPOSED
LOW_ENTROPY
```

### 点云

```text
density  = min(1, near_points / expected_near_points)    # default expected=20000
coverage = non_empty_azimuth_bins / 36
confidence = 0.6*density + 0.4*coverage
```

退化条件：`confidence < 0.4`、近场点 < 2000、coverage < 0.4，或 FOV 点 < 500。

reason code：

```text
EMPTY_CLOUD
SPARSE_POINTS
LOW_COVERAGE
FEW_FOV_POINTS
```

### 聚合

```text
overall = 0.5*image_confidence + 0.5*lidar_confidence
```

| image | lidar | action | map_trusted |
|---|---|---|---|
| OK | OK | `FULL_FUSION` | true |
| degraded | OK | `GEOMETRY_ONLY` | true |
| OK | degraded | `IMAGE_ONLY` | false |
| degraded | degraded | `BOTH_DEGRADED` | false |

`map_trusted = !lidar.degraded`：占据栅格来自 LiDAR 几何，图像退化不影响地图可信度。

---

## 7. 验收清单

| 验收项 | 状态 | 证据 |
|---|---|---|
| 图像直方图/对比度/曝光 | 通过 | `image_degradation` 测试 |
| 暗光/过曝/低对比 reason | 通过 | `DARK_IMAGE` / `OVEREXPOSED` / `LOW_CONTRAST` |
| 点云密度/方位覆盖/FOV | 通过 | `lidar_degradation` 测试 |
| 稀疏/低覆盖/空点云 | 通过 | `SPARSE_POINTS` / `LOW_COVERAGE` / `EMPTY_CLOUD` |
| 双模态聚合 | 通过 | `DegradationEvaluator` |
| 四种 action | 通过 | `degradation_report` 测试 |
| JSON 输出 | 通过 | `results/000000_degradation.json` |
| action 驱动策略 | 通过 | `fuse_frame --mode auto` |
| 性能目标 | 通过 | 热循环 0.45 / 0.42 / 0.87 ms |

---

## 8. 已知限制

1. **阈值基于合成样例调参**；真实 KITTI 帧需要重新检查 `expected_near_points`、`min_azimuth_coverage` 和图像阈值。
2. **FOV 点统计依赖图像尺寸和标定**；如果图像缺失，FOV 为 0 且不触发 `FEW_FOV_POINTS`，但当前数据源保证图像存在。
3. **图像评估使用单通道灰度**，不区分颜色通道退化（例如单通道过曝）；M6 可扩展。
4. **JSON 为手写 writer**，没有引入 JSON 库；当前字段固定、已通过 `json.tool` 校验。
5. **冷启动 6–8 ms** 来自 OpenCV/OpenMP 初始化；序列处理使用热循环，evaluator 中位数 0.87 ms。
6. **`map_trusted` 只由 LiDAR 决定**；图像退化时仍可能输出几何地图，但颜色质量下降。

---

## 9. 下一步（M4）

M4 空间表征：

```text
T4.1 ROI + 高度过滤 + PCL 体素降采样
T4.2 2D BEV 占据栅格 + 射线标记 free
T4.3 PGM + YAML 导出
T4.4 栅格可视化与质量统计
```

M4 会使用**原始点云**生成栅格，不使用 M2 的 `fused.pcd`（后者只含有效投影点）。
