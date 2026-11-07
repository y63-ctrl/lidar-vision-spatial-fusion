# M0 工程基线 — 完成验证

> 完成日期：2026-11-07
> 对应任务：T0.1 环境与依赖、T0.2 仓库骨架与现代 CMake、T0.3 样例数据与校验脚本

---

## 1. 结论

M0 已完成。项目当前具备：

- 可配置、可编译的 C++17 / CMake 工程；
- 真实链接 OpenCV、Eigen、PCL、OpenMP 的依赖检查程序；
- 可运行的应用入口与 CTest 冒烟测试；
- 单帧 KITTI 风格样例数据与 PowerShell/Bash 校验脚本；
- 构建、依赖、运行、排错文档。

---

## 2. 环境

| 项 | 实际值 |
|---|---|
| OS | Windows 11 x64 |
| 编译器 | MSVC 19.44.35228.0（VS 2022 Build Tools） |
| CMake | 3.31.6-msvc6 |
| 生成器 | Visual Studio 17 2022 / x64 |
| OpenCV | 5.1.0（`opencv_world510`） |
| Eigen | 3.4.0 |
| PCL | 1.15.1（AllInOne MSVC 2022 x64） |
| OpenMP | 2.0，`omp_get_max_threads() = 32` |

依赖目录（本机）：

```text
C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\
├── pcl\                 # PCL 1.15.1 + Boost/FLANN/VTK/Qhull/Eigen
├── opencv\opencv\build\ # OpenCV 5.1.0
└── openni2\             # 从 PCL 自带 MSI 提取的 OpenNI2.dll
```

---

## 3. 配置与构建

```powershell
$cmake = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$pcl   = "C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\pcl"
$cv    = "C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\opencv\opencv\build"
$eigen = "C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\pcl\3rdParty\Eigen3\share\eigen3\cmake"

& $cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  "-DCMAKE_PREFIX_PATH=$pcl;$cv" `
  "-DEigen3_DIR=$eigen"

& $cmake --build build --config Release --parallel 8
```

关键配置输出：

```text
-- Found OpenCV: .../_deps/opencv/opencv/build (found version "5.1.0")
-- Found OpenMP_CXX: -openmp (found version "2.0")
-- OpenCV : 5.1.0
-- Eigen  : 3.4.0
-- PCL    : 1.15.1
-- OpenMP : 2.0 (-openmp)
-- Configuring done
-- Generating done
```

构建产物：

```text
build/Release/lvf_core.lib
build/Release/lidar_fusion.exe
build/Release/lvf_deps_check.exe
build/Release/lvf_smoke_test.exe
```

另用全新的 `build_verify/` 目录做过一次干净配置 + 编译 + `RUN_TESTS`，同样 2/2 通过；临时目录已删除，避免和正式 `build/` 混淆。

`CMakePresets.json` 也已验证：

```powershell
& $cmake --preset vs2022 "-DCMAKE_PREFIX_PATH=$pcl;$cv" "-DEigen3_DIR=$eigen"
& $cmake --build --preset vs2022-release
```

---

## 4. 依赖检查

命令：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_deps.ps1
```

输出：

```text
LiDAR-Vision Spatial Fusion Pipeline dependency check
Version: 0.1.0
Summary: OpenCV 5.1.0 | Eigen 3.4.0 | PCL 1.15.1

[OK] OpenCV 5.1.0 (core, imgcodecs)
[OK] Eigen 3.4.0
[OK] PCL 1.15.1 (common, io, filters)
[OK] OpenMP 200203 (max threads: 32)

All required dependencies are available.
```

说明：该检查不仅编译包含头文件，还实际执行了：

- `cv::Mat` 分配与 PNG 编码；
- Eigen 矩阵乘法；
- PCL `VoxelGrid` 滤波；
- PCD 二进制写入/读回；
- OpenMP 线程数查询。

---

## 5. 自动测试

命令：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1
```

输出：

```text
Test project .../build
    Start 1: smoke
1/2 Test #1: smoke ............................   Passed
    Start 2: deps_check
2/2 Test #2: deps_check .......................   Passed

100% tests passed, 0 tests failed out of 2
```

---

## 6. 应用入口

```powershell
.\build\Release\lidar_fusion.exe
.\build\Release\lidar_fusion.exe --version
```

输出：

```text
LiDAR-Vision Spatial Fusion Pipeline v0.1.0
M0 scaffold: build, dependencies and data checks are ready.
Dependencies: OpenCV 5.1.0 | Eigen 3.4.0 | PCL 1.15.1
OpenMP: enabled
Next milestone: KITTI data access + calibration parsing (M1).
```

`--version` 输出：

```text
0.1.0
```

---

## 7. 样例数据

`data/sample/` 是**合成样例**，不是真实 KITTI 数据，用于 M0 的数据校验和后续 M1/M2 的最小输入。

```text
data/sample/
├── image_2/000000.png       # 1242x375, 18,372 bytes
├── velodyne/000000.bin      # 7,500 points, 120,000 bytes
└── calib/000000.txt         # P2 / R0_rect / Tr_velo_to_cam 等
```

校验命令：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_data.ps1
```

输出：

```text
[OK]   image: ... (18372 bytes)
[OK]   velodyne: ... (120000 bytes)
[OK]   calib: ... (827 bytes)
[OK]   velodyne layout: 7500 points (4 float32 per point)
[OK]   calib key present: P2
[OK]   calib key present: R0_rect
[OK]   calib key present: Tr_velo_to_cam
[OK]   image dimensions: 1242x375

Data check passed: ... (frame 000000)
```

Bash 版本也已验证通过：

```bash
bash scripts/check_data.sh
```

---

## 8. M0 验收清单

| 验收项 | 状态 | 证据 |
|---|---|---|
| CMake 从干净目录可配置 | 通过 | 第 3 节配置输出 |
| 现代 target-based CMake | 通过 | `CMakeLists.txt` |
| Core/App/Tools/Tests 四个 target | 通过 | 第 3 节构建产物 |
| OpenCV 链接 | 通过 | `deps_check` 的 PNG 编码 |
| Eigen 链接 | 通过 | `deps_check` 的矩阵乘法 |
| PCL 链接 | 通过 | `VoxelGrid` + PCD round-trip |
| OpenMP 链接 | 通过 | `deps_check` 输出线程数 |
| CTest 通过 | 通过 | 2/2 tests passed |
| 样例数据校验（PowerShell） | 通过 | 第 7 节 |
| 样例数据校验（Bash） | 通过 | 第 7 节 |
| `.gitignore` 排除大文件/依赖 | 通过 | `.gitignore` |
| 构建文档 | 通过 | `docs/build.md` |

---

## 9. 已知限制与说明

1. **OpenCV 版本为 5.1.0**，不是 4.x。本项目 M0 只使用 `core` / `imgproc` / `imgcodecs` 的稳定 API，暂未发现兼容性问题；如果后续要求锁定 4.x，需要替换 OpenCV 包并重新验证。
2. **OpenNI2 只提供运行 DLL**，没有配置头文件/库，所以 PCL 的 OpenNI2 IO 功能在 CMake 中是 disabled。M0 不使用深度相机，不影响 PCD/VoxelGrid。
3. **PCL 运行时 DLL 较多**（PCL + VTK + FLANN + Qhull + OpenCV）。M0 通过 `scripts/check_deps.ps1` 和 CTest 的 `ENVIRONMENT_MODIFICATION` 处理，尚未做安装包/运行时打包，留到 M5/M6。
4. **样例是合成数据**，真实 KITTI 数据需要用户下载后替换 `data/sample`，见 `data/README.md`。
5. **没有提交 Git 仓库/tag**。M0 只要求工程基线；`v0.1.0` tag 在 M6 最终验收时打。

---

## 10. 下一步（M1）✅ 已完成

> M1 已完成，证据见 [m1-verification.md](m1-verification.md)。

M1 数据接入：

- `T1.1` 核心类型与接口契约；
- `T1.2` KITTI 标定解析与矩阵组合；
- `T1.3` KittiDataSource：RGB + `.bin` + calib；
- `T1.4` 数据接入测试与样例统计。

建议先用合成样例把 `Frame` / `Calibration` / 投影矩阵链路跑通，再替换成真实 KITTI 帧。
