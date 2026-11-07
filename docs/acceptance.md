# 最终验收（v0.1.0）

> 日期：2026-11-07
> 项目：LiDAR-Vision Spatial Fusion Pipeline
> 范围：M0–M6 单帧 MVP + 小序列

---

## 1. 结论

**通过。** 单帧 Pipeline、退化评估、BEV 栅格、PGM/YAML、PCD、可视化、timing、summary、多帧运行、退出码、性能和 ASan 内存错误检查均已验证。

未完成的仅是 **Windows 平台 ASan 泄漏检测**：MSVC ASan 不支持 `detect_leaks`，且本机没有 Valgrind。ASan 内存错误检查本身已通过（pipeline 测试 + 10 帧 CLI）。

---

## 2. 功能验收

- [x] `lidar_fusion` 单帧命令成功，退出码 0。
- [x] 一条命令输出 8 类帧结果 + `summary.csv`。
- [x] `fused.pcd` 带 RGB，字段 `x y z rgb`，可被 PCL 读回。
- [x] `overlay.png` 用深度着色显示投影点，`fused_view.png` 用 RGB。
- [x] `degradation.json` 包含 image/lidar/overall/action/map_trusted。
- [x] 四种 action 均通过测试：`FULL_FUSION`、`GEOMETRY_ONLY`、`IMAGE_ONLY`、`BOTH_DEGRADED`。
- [x] `occupancy.pgm` / `.yaml` 输出，尺寸、原点、分辨率、阈值正确。
- [x] 栅格区分 occupied/free/unknown；障碍物后方保持 unknown。
- [x] `timing.json` 含 read/degradation/fusion/mapping/write/compute/total。
- [x] `summary.csv` 每帧一行；`--all` 三帧产生 3 行数据。
- [x] `--no-viz` / `--no-pcd` 可关闭对应输出。
- [x] `--fail-on-degraded` 在暗光样例上返回 3。
- [x] 参数错误返回 1，数据/处理错误返回 2。
- [x] `--version` / `--help` 可用。
- [x] 缺失 data root、损坏文件、非法帧号均有明确错误。

---

## 3. 工程验收

- [x] C++17，CMake ≥ 3.20，Visual Studio 2022 x64。
- [x] 干净目录可配置、可编译。
- [x] `--clean-first` 全量重编译通过。
- [x] CTest 17/17 通过。
- [x] 标定组合、投影边界、栅格行列、PGM/YAML、PCD 均有单元测试。
- [x] 无裸 `new/delete` 管理点云；Frame 使用 `shared_ptr`/PCL `Ptr`。
- [x] OpenMP 线程数可配置，1/2/4/8 均能运行。
- [x] 代码中无硬编码绝对依赖路径（路径由 CMake/CLI/脚本传入）。
- [x] 非 ASCII Windows 路径下 PNG/PCD/JSON/PGM 均可写出。
- [x] `.gitignore` 排除 build/results/大依赖。
- [x] README、接口、M0–M6 验证文档完整。
- [x] MIT 代码许可证，KITTI 数据许可单独说明。
- [x] 可选 Ubuntu CI 工作流已添加。

---

## 4. 性能验收

测试环境：AMD Ryzen 9 8945HX（16C/32T）、31.8 GB RAM、Windows 11、MSVC Release。

| 指标 | 目标 | 实测 |
|---|---:|---:|
| compute 中位数（全输出） | ≤ 25 ms | **2.50 ms** |
| end-to-end 中位数（全输出） | ≤ 50 ms | **43.98 ms** |
| end-to-end p95（全输出） | ≤ 50 ms | **45.84 ms** |
| compute 中位数（no-viz/no-pcd） | — | **1.94 ms** |
| 峰值 working set（10 帧） | — | **约 33 MB** |
| 1/2/4/8 线程 | 可运行 | 通过；fusion 中位数 0.641/0.488/0.465/0.364 ms |

详见 [performance.md](performance.md)。

- [x] compute 目标达成。
- [x] end-to-end 目标达成。
- [x] 10 帧序列无崩溃。
- [x] 峰值内存有记录。
- [x] MSVC ASan 内存错误检查：pipeline 测试 + 10 帧 CLI 通过。
- [ ] ASan 泄漏检测 / Valgrind：Windows ASan 不支持 `detect_leaks`，本机无 Valgrind；建议 Linux CI 补跑。

---

## 5. 交付物

- [x] 源码：`app/`、`include/`、`src/`、`tools/`、`tests/`
- [x] 构建：`CMakeLists.txt`、`CMakePresets.json`
- [x] 脚本：`scripts/`
- [x] 样例：`data/sample/`
- [x] 结果：`results/`（PCD/PNG/PGM/YAML/JSON/CSV）
- [x] 文档：README、TASKS、build、interfaces、performance、M0–M6 验证、acceptance
- [x] 三联图：`docs/images/compare.png` 与 `results/000000_compare.png`
- [x] 许可证：MIT（代码）
- [x] 可选 CI：`.github/workflows/ci.yml`
- [x] Git tag：`v0.1.0`

---

## 6. 已知限制

1. **ASan 泄漏检测受限**：MSVC ASan 在 Windows 不支持 `detect_leaks`，本机没有 Valgrind；ASan 内存错误检查本身已通过。建议 Linux CI 增加 `-fsanitize=address,undefined` 做泄漏/UB 检查。
2. **样例是合成数据**，不是真实 KITTI；真实帧需要替换 `data/sample` 后再做一次目视验收。
3. **没有 ROS 运行时验证**；PGM/YAML 按 map_server 语义输出，但未在本机加载。
4. **高度过滤是近似地面去除**，没有 RANSAC；坡道/低矮障碍需调参。
5. **Raycast 是 2D 近似**，不建模光束高度/入射角。
6. **没有多帧栅格累积**，当前逐帧独立地图。
7. **没有地图精度指标**（IoU/Chamfer），也没有真实 KITTI 标注评测。
8. **`summary.csv` 每次覆盖**，没有跨运行聚合报告；performance.md 中的统计由脚本离线计算。

---

## 7. 一句话

**MVP 已可交付：一条命令从 KITTI 单帧走到彩色点云、退化 JSON、BEV PGM/YAML、三联图、timing 和 summary；17 项测试全绿，性能目标达成。**
