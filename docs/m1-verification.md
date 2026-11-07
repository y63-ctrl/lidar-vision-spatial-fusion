# M1 数据接入 — 完成验证

> 完成日期：2026-11-07
> 对应任务：T1.1 核心类型与接口、T1.2 标定解析、T1.3 KittiDataSource、T1.4 数据接入测试与样例统计

---

## 1. 结论

M1 已完成。当前项目可以：

- 读取 KITTI 风格单帧目录：`image_2/*.png`、`velodyne/*.bin`、`calib/*.txt`；
- 解析 `P2`、`R0_rect`、`Tr_velo_to_cam`，补齐齐次矩阵并组合出 `T_img`；
- 用共享指针持有图像/点云，不产生整帧深拷贝；
- 自动发现帧号、补零、拒绝路径分隔符；
- 支持 `calibOverride`，为外参漂移演示预留入口；
- 对缺失文件、截断 `.bin`、错误标定、非有限值给出明确异常；
- 通过 5 个 CTest（冒烟 + 标定 + 数据源 + 数据集成 + 依赖检查）。

---

## 2. 新增代码

| 文件 | 内容 |
|---|---|
| `include/core/Types.h` / `src/core/Types.cpp` | `Calibration`、`Frame`、`FusionResult`、`GridMap`、`PipelineConfig` 等 |
| `include/data/IDataSource.h` | 数据源抽象基类 |
| `include/data/KittiCalibration.h` / `src/data/KittiCalibration.cpp` | KITTI 标定解析与矩阵组合 |
| `include/data/KittiDataSource.h` / `src/data/KittiDataSource.cpp` | 图像、点云、标定读取 |
| `include/fusion/IFusionStrategy.h` | 融合策略接口（M2 实现） |
| `tools/inspect_frame.cpp` | 单帧统计工具 |
| `tests/test_calibration.cpp` | 标定/投影数学单测 |
| `tests/test_kitti_source.cpp` | 数据源、补零、错误路径、calib override |
| `tests/test_data_integration.cpp` | 单帧端到端读取与统计 |
| `tests/TestHarness.h` | 轻量测试断言 |

---

## 3. 构建

```powershell
$cmake = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$pcl   = "C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\pcl"
$cv    = "C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\opencv\opencv\build"
$eigen = "C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\pcl\3rdParty\Eigen3\share\eigen3\cmake"

& $cmake --preset vs2022 "-DCMAKE_PREFIX_PATH=$pcl;$cv" "-DEigen3_DIR=$eigen"
& $cmake --build --preset vs2022-release
```

构建新增产物：

```text
build/Release/lvf_inspect_frame.exe
build/Release/lvf_test_calibration.exe
build/Release/lvf_test_kitti_source.exe
build/Release/lvf_test_data_integration.exe
```

---

## 4. 自动测试

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1
```

结果：

```text
Start 1: smoke
1/5 Test #1: smoke ............................   Passed
Start 2: calibration
2/5 Test #2: calibration ......................   Passed
Start 3: kitti_source
3/5 Test #3: kitti_source .....................   Passed
Start 4: data_integration
4/5 Test #4: data_integration .................   Passed
Start 5: deps_check
5/5 Test #5: deps_check .......................   Passed

100% tests passed, 0 tests failed out of 5
```

随后执行了 `cmake --build build --config Release --clean-first` 全量重编译，再跑一次 CTest，仍为 5/5 通过。

### 4.1 标定测试覆盖

`tests/test_calibration.cpp` 覆盖：

- `P2` 多行解析；
- `R0_rect` 3×3 → 4×4；
- `Tr_velo_to_cam` 3×4 → 4×4；
- Velodyne → Camera 轴变换；
- `T_img = P2 * R0_rect * T_velo_cam`；
- 已知点 `(10, 2, 1)` 投影到 `u=30, v=30, depth=10`；
- 缺少 `P2` / `Tr_velo_to_cam` 抛异常；
- `P2` 数值个数错误抛异常；
- `P2` 第三行错误抛异常；
- KITTI 常见的 `P2(2,3) ≈ 0.0027` 被接受；
- 从文件加载标定。

### 4.2 数据源测试覆盖

`tests/test_kitti_source.cpp` 覆盖：

- 自动发现 `000000`；
- 读取 1242×375 图像、20000 点点云；
- `next()` 到末尾返回 `false`；
- `"0"`、`"000000.png"` 归一化；
- 带路径分隔符的帧号被拒绝；
- 缺失数据根目录抛异常；
- 截断 `.bin`（15 字节）抛异常；
- `calibOverride` 生效（通过修改外参平移 0.5m 验证）。

### 4.3 数据集成测试覆盖

`tests/test_data_integration.cpp` 输出：

```text
image: 1242x375
cloud: 20000 points
range: [5.042, 46.436] m
intensity: [9.53912e-05, 0.999949]
[PASS] data_integration
```

并检查：

- 图像尺寸/通道；
- 点云指针非空、点数正确；
- 所有点坐标有限；
- 强度范围在 `[0,1]`；
- `T_img` 有限；
- 标定有效。

---

## 5. 单帧统计工具

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\inspect_frame.ps1
```

> 直接运行 `build\Release\lvf_inspect_frame.exe` 需要手动把 PCL/OpenCV/VTK 的 `bin` 加入 `PATH`；脚本封装了这一步。

输出：

```text
source: KittiDataSource
frame: 000000
image: 1242x375 channels=3
cloud: 20000 points
range: [5.042, 46.436] m
intensity: [0.000, 1.000]
calibration.valid: true
T_img = P2 * R0_rect * Tr_velo_to_cam:
  [   621.0000,   -500.0000,      0.0000,      0.0000 ]
  [   187.5000,      0.0000,   -500.0000,      0.0000 ]
  [     1.0000,      0.0000,      0.0000,      0.0000 ]
```

该矩阵来自合成样例的简化标定；真实 KITTI 的 `P2` 和 `Tr_velo_to_cam` 会得到不同的数值，但组合顺序一致。

---

## 6. 关键约定

- 点云始终保存在 **Velodyne 坐标系**；
- 投影链严格为 `P2 * R0_rect * T_velo_cam`；
- `P2` 已包含相机内参，不重复乘 `K`；
- 帧号统一为 6 位数字；
- 图像用 `std::ifstream` + `cv::imdecode` 读取，避免 Windows 非 ASCII 路径下 `cv::imread` 失败；
- 数据源异常包含路径/帧号；
- 数据层不打印日志，由 CLI 统一捕获。

详见 [interfaces.md](interfaces.md)。

---

## 7. 验收清单

| 验收项 | 状态 | 证据 |
|---|---|---|
| 冻结 `Calibration` / `Frame` / `IDataSource` / `IFusionStrategy` | 通过 | `include/core/Types.h`、`include/data/`、`include/fusion/` |
| 解析 `P2` / `R0_rect` / `Tr_velo_to_cam` | 通过 | `calibration` 测试 |
| `T_img` 组合顺序正确 | 通过 | 已知点投影 `(30,30,10)` |
| 读取 RGB | 通过 | 1242×375、3 通道 |
| 读取 `.bin` | 通过 | 20000 点、16 字节步长校验 |
| 缺文件/截断/坏标定报错 | 通过 | `kitti_source` 测试 |
| 自动发现帧与补零 | 通过 | `discoverFrames`、`normalizeFrameId` |
| 单帧统计工具 | 通过 | `lvf_inspect_frame` |
| 文档 | 通过 | `docs/interfaces.md`、本文件 |
| CTest | 通过 | 5/5 |

---

## 8. 已知限制

1. **样例仍是合成数据**，不是真实 KITTI。真实帧需要按 `data/README.md` 替换；M2 的投影叠加图应在真实 KITTI 上再做一次目视验收。
2. **尚未实现投影/融合**；M1 只完成数据读取与标定组合，`IFusionStrategy` 还是接口。
3. **没有真实 KITTI 的标定文件回归样本**；当前测试使用合成标定 + 标准 KITTI 格式。后续拿到真实帧后补一个 `000000.txt` 的解析测试。
4. **图像解码走内存 buffer**，比 `cv::imread` 多一次读取，但解决了非 ASCII 路径问题；M2 可做性能测量。
5. **OpenCV 为 5.1.0**，与 M0 记录一致。

---

## 9. 下一步（M2）✅ 已完成

> M2 已完成，证据见 [m2-verification.md](m2-verification.md)。

M2 投影融合：

```text
T2.1 投影核心：3D -> 2D，边界/深度过滤
T2.2 OpenMP 并行取色 + 生成 RGB 点云
T2.3 策略模式：正常融合 / 仅深度
T2.4 投影叠加图 + 融合点云视图
T2.5 RGB 点云 PCD 落盘
```

M2 的第一张 `overlay.png` 是关键验收点：点云边缘必须贴合图像物体边缘。
