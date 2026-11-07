# 性能与内存验证（M6）

> 日期：2026-11-07
> 目的：记录单帧 compute/end-to-end、10 帧序列、线程扩展、峰值内存，判断是否满足 M2/M6 的性能目标。

---

## 1. 测试环境

| 项 | 值 |
|---|---|
| CPU | AMD Ryzen 9 8945HX，16 核 / 32 线程 |
| RAM | 31.8 GB |
| OS | Windows 11 家庭版 中文版 |
| 编译器 | MSVC 19.44（VS 2022 Build Tools） |
| 构建 | Release / Visual Studio 17 2022 / x64 |
| CMake | 3.31.6 |
| OpenCV | 5.1.0（预编译） |
| PCL | 1.15.1（AllInOne MSVC 2022 x64） |
| Eigen | 3.4.0 |
| OpenMP | 2.0，`omp_get_max_threads()=32` |

> 10 帧测试把同一合成帧复制为 `000000`–`000009`，因此 I/O cache 友好；这是“工程链路”的参考数据，不是真实 KITTI 数据集评测。

---

## 2. 单帧耗时

10 帧全输出（PCD + 3 张 PNG + PGM/YAML + JSON + summary）：

| 阶段 | mean | median | p95 |
|---|---:|---:|---:|
| read | 10.842 | 13.574 | 14.288 |
| degradation | 1.897 | 1.325 | 7.137 |
| fusion | 0.749 | 0.421 | 3.728 |
| mapping | 0.773 | 0.760 | 0.839 |
| write | 27.492 | 27.573 | 29.579 |
| **compute** | **3.419** | **2.499** | **11.677** |
| **end-to-end** | **41.754** | **43.981** | **45.844** |

单位：ms。

结论：

- compute 中位数 **2.50 ms**，远低于 25 ms 目标；
- end-to-end 中位数 **43.98 ms**、p95 **45.84 ms**，低于 50 ms 目标；
- 写盘（PCD + PNG）占大头，约 27.5 ms。

首帧冷启动/文件系统抖动会让 p95 偏高；M5 的逐帧 `timing.json` 会记录每帧真实值。

---

## 3. 不写可视化的 compute 路径

`--no-viz --no-pcd --threads 8`，10 帧：

| 阶段 | mean | median | p95 |
|---|---:|---:|---:|
| read | 3.791 | 3.755 | 4.034 |
| degradation | 1.554 | 1.028 | 6.417 |
| fusion | 0.404 | 0.364 | 0.786 |
| mapping | 0.552 | 0.547 | 0.592 |
| write | 4.470 | 4.464 | 6.160 |
| **compute** | **2.509** | **1.940** | **7.760** |
| **end-to-end** | **10.772** | **10.103** | **17.690** |

单位：ms。适合作为“序列处理/边缘设备”的参考路径；完整演示仍建议保留可视化。

---

## 4. 线程扩展（fusion）

10 帧、`--no-viz --no-pcd`，改变 `--threads`，取 `fusion_ms` 中位数：

| threads | fusion median (ms) | speedup |
|---:|---:|---:|
| 1 | 0.641 | 1.00× |
| 2 | 0.488 | 1.31× |
| 4 | 0.465 | 1.38× |
| 8 | 0.364 | 1.76× |

fusion 只有 0.4–0.6 ms，线程创建/调度开销占比高，因此加速比远低于线性；真实 12 万点 KITTI 帧的收益会更明显。

---

## 5. 峰值内存

10 帧全输出，使用 `Start-Process` + 轮询 `WorkingSet64`（10 ms 间隔）：

```text
PeakWorkingSetMB ≈ 33.1 MB
```

说明：

- 这是 Windows working set，不是 Java/托管堆式的“总分配”；
- PCL/OpenCV 的共享 DLL 页可能部分计入/不计入 working set；
- 10 帧处理没有观察到持续增长。

### AddressSanitizer（MSVC）

使用单独的 ASan 构建：

```powershell
cmake -S . -B build_asan -G "Visual Studio 17 2022" -A x64 `
  "-DCMAKE_CXX_FLAGS=/EHsc /fsanitize=address /Zi" ...
cmake --build build_asan --config Release
```

结果：

- `lvf_test_pipeline.exe` 在 ASan 下通过；
- `lidar_fusion --all --no-viz --no-pcd` 处理 10 帧，退出码 0，无 ASan 报错；
- Windows 版 MSVC ASan **不支持 `detect_leaks`**（运行时报 `detect_leaks is not supported on this platform`）；
- 本机没有 Valgrind，因此没有做 Valgrind 泄漏报告；建议 Linux CI 用 `-fsanitize=address,undefined` 补泄漏/UB 检查。

---

## 6. 结论

| 目标 | 结果 |
|---|---|
| compute 中位数 ≤ 25 ms | 2.50 ms（全输出）/ 1.94 ms（纯 compute） |
| end-to-end 中位数 ≤ 50 ms | 43.98 ms |
| end-to-end p95 ≤ 50 ms | 45.84 ms |
| 线程可配置 | 1/2/4/8 均可运行 |
| 10 帧无崩溃 | 通过 |
| 峰值内存 | 约 33 MB |

性能目标达到。下一步若要进一步压缩端到端耗时，优先优化写盘（二进制 PCD 已用，PNG 可降采样/异步写），而不是投影本身。
