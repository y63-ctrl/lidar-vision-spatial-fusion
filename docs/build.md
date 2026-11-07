# 构建与依赖说明（M0）

本文档记录 **M0 工程基线** 的实际构建方式。目标：在干净环境中完成配置、编译，并验证 OpenCV / Eigen / PCL / OpenMP 可链接。

---

## 1. 已验证环境

| 项 | 版本 |
|---|---|
| OS | Windows 11 x64 |
| 编译器 | MSVC 19.44（Visual Studio 2022 Build Tools） |
| CMake | 3.31.6（随 VS Build Tools 提供） |
| 生成器 | Visual Studio 17 2022 / x64 |
| OpenCV | 5.1.0（Windows 预编译包，`opencv_world510`） |
| Eigen | 3.4.0（PCL AllInOne 自带） |
| PCL | 1.15.1（AllInOne MSVC 2022 x64） |
| OpenMP | MSVC `/openmp`，v2.0，32 线程 |

> 这不是唯一支持的组合。Ubuntu 22.04 下使用 `libopencv-dev`、`libeigen3-dev`、`libpcl-dev` 也可以，后续 CI 会补 Linux 构建。

### 本机实测依赖路径

为避免把 500MB+ 的第三方库放进项目目录，本机把依赖解压在 DSH 工作区的 `_deps` 下：

| 依赖 | 路径 |
|---|---|
| PCL | `C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\pcl` |
| OpenCV build | `C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\opencv\opencv\build` |
| Eigen3 config | `C:\Users\ydyyy\Documents\deepseek-harness\default-workspace\_deps\pcl\3rdParty\Eigen3\share\eigen3\cmake` |
| OpenNI2 runtime | `...\_deps\openni2\OpenNI2\Redist\OpenNI2.dll`（复制到了 PCL `bin`） |

这些路径只用于本机复现；换机器时按第 2 节重新安装并替换路径即可。

---

## 2. 依赖安装（Windows）

M0 使用预编译包，避免从源码编译 PCL/VTK：

1. **PCL 1.15.1 AllInOne（MSVC 2022 x64）**
   - 下载：`PCL-1.15.1-AllInOne-msvc2022-win64.exe`
   - 安装或解压到本地目录，例如：
     ```text
     C:\deps\PCL 1.15.1
     ```
   - 该包同时提供 PCL、Boost、FLANN、VTK、Qhull、OpenNI2 等依赖。

2. **OpenCV 5.1.0 Windows**
   - 下载：`opencv-5.1.0-windows.exe`
   - 解压到：
     ```text
     C:\deps\opencv
     ```
   - CMake 配置目录为 `C:\deps\opencv\build`。

3. **Eigen**
   - PCL AllInOne 通常自带 Eigen；如果 `find_package(Eigen3)` 找不到，可额外下载 Eigen 3.4.x 并设置 `Eigen3_DIR`。
   - 本项目只使用 Eigen 的头文件接口。

4. **CMake / Ninja**
   - 本项目使用 VS Build Tools 自带的 CMake：
     ```text
     C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe
     ```
   - 也可以使用独立 CMake ≥ 3.20。

> 本地依赖目录不要提交到 Git；`.gitignore` 已忽略 `_deps/`、`third_party/`。

---

## 3. 配置与构建

在项目根目录执行（路径替换成你的实际安装目录）：

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

如果使用 `CMakePresets.json`：

```powershell
& $cmake --preset vs2022 `
  "-DCMAKE_PREFIX_PATH=$pcl;$cv" `
  "-DEigen3_DIR=$eigen"

& $cmake --build --preset vs2022-release
```

---

## 4. 验证

### 4.1 依赖检查

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_deps.ps1
```

脚本会从 `build/CMakeCache.txt` 读取 PCL/OpenCV/VTK 路径，临时拼好运行时 `PATH`，再执行 `lvf_deps_check.exe`。

期望输出包含：

```text
[OK] OpenCV ...
[OK] Eigen ...
[OK] PCL ... (common, io, filters)
[OK] OpenMP ...
All required dependencies are available.
```

### 4.2 应用与测试

```powershell
.\build\Release\lidar_fusion.exe
.\build\Release\lidar_fusion.exe --version

# 如果 ctest 不在 PATH，用脚本自动定位 VS 自带的 ctest
powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1
```

### 4.3 样例数据检查

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_data.ps1
```

或：

```bash
bash scripts/check_data.sh
```

---

## 5. 常见问题

### `find_package(PCL)` 找不到

- 确认 `CMAKE_PREFIX_PATH` 指向 PCL 安装根目录（包含 `PCLConfig.cmake` 的目录）；
- 确认安装的是 **MSVC 2022 x64** 版本，不要混用 MinGW 版 PCL；
- 清理 `build/` 后重新配置，CMake 会缓存旧的 `PCL_DIR`。

### OpenCV 与 PCL 的 Eigen 版本冲突

- PCL AllInOne 可能带自己的 Eigen；如果 CMake 找到的 Eigen 与 PCL 不一致，优先使用 PCL 包内的 Eigen；
- 在 `CMakeCache.txt` 中检查 `Eigen3_DIR`、`PCL_DIR` 是否指向同一套依赖。

### 运行时找不到 DLL

- `scripts/check_deps.ps1` 会自动把 OpenCV、PCL、VTK、FLANN、Qhull 的 `bin` 加入当前进程的 `PATH`；
- CTest 通过 `ENVIRONMENT_MODIFICATION` 为 `deps_check` 注入同样的路径，所以 `scripts/test.ps1` 不需要手动改系统 `PATH`；
- PCL 的 `pcl_io.dll` 还依赖 `OpenNI2.dll`。本机从 PCL 自带的 `OpenNI-Windows-x64-2.2.msi` 中提取后，复制到了 PCL 的 `bin` 目录；
- 如果直接双击 exe，仍然可能缺 DLL；M5/M6 会补 `install`/打包步骤，把运行时依赖收拢到输出目录。

---

## 6. M0 完成标准

- [x] CMake 从干净目录可配置；
- [x] `lvf_core` / `lidar_fusion` / `lvf_deps_check` / `lvf_smoke_test` 可编译；
- [x] OpenCV / Eigen / PCL / OpenMP 链接成功；
- [x] `ctest` 通过；
- [x] 样例数据检查脚本可用；
- [x] `.gitignore` 排除大文件与本地依赖。
