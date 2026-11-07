# 接口与数据契约（M1/M2）

> 对应任务：T1.1 核心类型与接口契约、T2.1–T2.3 投影/融合策略
> 代码位置：`include/core/Types.h`、`include/data/`、`include/fusion/`

---

## 1. 坐标系

| 坐标系 | 轴向 | 用途 |
|---|---|---|
| Velodyne | x 向前，y 向左，z 向上 | 点云存储、后续 BEV 栅格 |
| Camera | x 向右，y 向下，z 向前 | 投影计算 |
| Image | u 向右，v 向下 | 像素采样 |

**不变量：**

- `Frame::cloud` 始终保存在 **Velodyne 坐标系**。
- 只有投影/取色阶段转到 Camera 坐标系；BEV 栅格使用 Velodyne 的 x-y 平面。
- 所有长度单位是米；角度在内部使用弧度；时间/耗时单位在输出层使用毫秒。
- 图像约定为 BGR8（OpenCV 默认），不是 RGB。

---

## 2. `Calibration`

```cpp
struct Calibration {
    Eigen::Matrix<float, 3, 4> P2;
    Eigen::Matrix4f R0_rect;
    Eigen::Matrix4f T_velo_cam;

    Eigen::Matrix<float, 3, 4> veloToImage() const;
    Eigen::Vector3f veloToCamera(const Eigen::Vector3f&) const;
    bool isValid() const;
};
```

| 字段 | 形状 | 含义 |
|---|---|---|
| `P2` | 3×4 | KITTI 相机 2（左彩色）投影矩阵，已含内参 |
| `R0_rect` | 4×4 | 整流矩阵；文件中的 3×3 补成齐次形式 |
| `T_velo_cam` | 4×4 | Velodyne → Camera；文件中的 3×4 补成齐次形式 |

投影链：

```text
T_img = P2 * R0_rect * T_velo_cam
uvw   = T_img * [x_v, y_v, z_v, 1]^T
u     = uvw.x / uvw.z
v     = uvw.y / uvw.z
```

有效投影条件由 M2 实现：

```text
uvw.z > min_depth
0 <= u < image.cols
0 <= v < image.rows
```

`P2` 已包含内参，**不要**再乘一次 `K`。如果界面需要 `K`，可临时提取 `P2.leftCols<3>()`，但投影仍用 `P2`。

`Calibration::isValid()` 检查：

- 所有矩阵元素有限；
- `P2` 第三行近似 `[0, 0, 1, *]`；
- 齐次矩阵最后一行为 `[0, 0, 0, 1]`。

更严格的 KITTI 文件格式校验在 `KittiCalibration` 中完成。

---

## 3. `Frame`

```cpp
struct Frame {
    std::string id;
    cv::Mat image;          // BGR8
    CloudXYZI::Ptr cloud;   // Velodyne coordinates
    Calibration calib;
};
```

所有权：

- `cv::Mat` 是引用计数头；复制 `Frame` 不会复制像素；
- `CloudXYZI::Ptr` 是 PCL 点云共享指针；复制 `Frame` 不会复制点云；
- 读取完成后，管线把 `Frame` 当作**只读**数据；
- 不要在 `Frame` 中保存指向局部变量的指针/引用。

`CloudXYZI` 与 `CloudXYZRGB` 是 `pcl::PointCloud<...>` 的别名：

```cpp
using CloudXYZI   = pcl::PointCloud<pcl::PointXYZI>;
using CloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
```

---

## 4. `IDataSource`

```cpp
class IDataSource {
public:
    virtual ~IDataSource() = default;
    virtual bool next(Frame& frame) = 0;
    virtual std::string name() const = 0;
};
```

契约：

1. `next(frame)` 返回 `false` 表示没有更多帧；
2. 成功时，`frame` 被完整填充，替换旧内容；
3. I/O、文件格式、标定解析失败时抛出 `std::runtime_error`，消息包含路径；
4. 数据源实例**不是线程安全**的；需要并行读取时，每线程一个实例；
5. 数据源只负责读取和校验，不做投影/融合。

---

## 5. `KittiDataSource`

```cpp
KittiDataSource(
    std::filesystem::path dataRoot,
    std::vector<std::string> frameIds = {},
    std::filesystem::path calibOverride = {});
```

### 5.1 目录布局

```text
<root>/
├── image_2/<frame>.png       # BGR8
├── velodyne/<frame>.bin      # float32 x,y,z,intensity
└── calib/<frame>.txt         # P2, R0_rect, Tr_velo_to_cam
```

### 5.2 帧号

- 构造时传入空 `frameIds`：自动扫描 `image_2/*.png`，按字典序排序；
- 传入 `{"0"}`、`{"000000"}`、`{"000000.png"}` 都归一化为 `000000`；
- 帧号不能包含 `/`、`\`，避免路径穿越；
- 超过 6 位的纯数字帧号会被拒绝（KITTI object 使用 6 位）。

### 5.3 标定覆盖

`calibOverride` 非空时，所有帧使用该标定文件，忽略 `<root>/calib/<frame>.txt`。这是为“无人机震动/外参漂移”演示预留的入口。

### 5.4 读取规则

- 图像：用 C++ 二进制读入，再 `cv::imdecode` 解码，避免 Windows 非 ASCII 路径下 `cv::imread` 失败；
- 点云：文件大小必须是 16 的倍数；按 `<x,y,z,intensity>` 四个 float32 读取；
- 空文件、截断文件、非有限值、缺失标定均抛异常；
- 解码后的图像必须是 3 通道（OpenCV 会按 `IMREAD_COLOR` 返回 BGR8）；
- 点云写入 `width = N`、`height = 1`、`is_dense = true`。

---

## 6. `IFusionStrategy`

```cpp
class IFusionStrategy {
public:
    virtual ~IFusionStrategy() = default;
    virtual FusionResult fuse(const Frame& frame) const = 0;
};
```

M1 冻结接口；M2 已实现：

- `PinholeFusionStrategy`：正常图像双线性/最近邻取色；
- `DepthOnlyFusionStrategy`：图像退化时按深度伪彩色；
- `FusionStrategyFactory`：按名称或退化动作创建策略；
- `FusionOptions`：`min_depth` / `max_depth` / `threads` / `bilinear`。

M2 的输出约定：

- `FusionResult::cloud` 仍保存在 Velodyne 坐标系，只包含有效投影点；
- `FusionResult::projected_points` 与 `colored_points` 当前相等；
- `FusionResult::overlay` 暂不由融合策略填充；叠加图/融合视图由 `OverlayRenderer` 生成；
- PCD 只写有效投影点；M4 栅格化应使用原始点云，避免丢点。

`FusionResult`：

```cpp
struct FusionResult {
    CloudXYZRGB::Ptr cloud;
    cv::Mat overlay;
    std::size_t input_points;
    std::size_t projected_points;
    std::size_t colored_points;
};
```

---

## 6.1 退化报告与动作

```cpp
struct ModalityStatus {
    float confidence;
    bool degraded;
    std::vector<std::string> reasons;
    std::map<std::string, double> metrics;
};

struct DegradationReport {
    ModalityStatus image;
    ModalityStatus lidar;
    float overall;
    std::string action;
    bool map_trusted;
};
```

`DegradationEvaluator` 组合 `ImageDegradationCheck` 和 `LidarDegradationCheck`：

```text
overall = image_weight * image.confidence + (1 - image_weight) * lidar.confidence
```

默认 `image_weight = 0.5`。

### 动作映射

| image | lidar | action | map_trusted |
|---|---|---|---|
| OK | OK | `FULL_FUSION` | true |
| degraded | OK | `GEOMETRY_ONLY` | true |
| OK | degraded | `IMAGE_ONLY` | false |
| degraded | degraded | `BOTH_DEGRADED` | false |

`FusionStrategyFactory::createForAction()`：

- `FULL_FUSION` → `PinholeFusionStrategy`
- `GEOMETRY_ONLY` / `IMAGE_ONLY` / `BOTH_DEGRADED` → `DepthOnlyFusionStrategy`

`map_trusted` 只由 LiDAR 退化决定：占据栅格来自 LiDAR 几何，图像退化影响颜色/语义，不直接让地图失效。

---

## 7. `PipelineConfig` 默认值

| 字段 | 默认值 | 说明 |
|---|---|---|
| `data_root` | `data/sample` | KITTI 风格数据根目录 |
| `output_dir` | `results` | 输出目录 |
| `frame_id` | `000000` | 单帧 ID |
| `min_depth` | `0.5 m` | 投影最小深度 |
| `max_depth` | `80 m` | 投影最大深度 |
| `resolution` | `0.2 m` | BEV 栅格分辨率 |
| `voxel_size` | `0.2 m` | 体素降采样尺寸 |
| `x_min/x_max` | `-10 / 50 m` | BEV ROI |
| `y_min/y_max` | `-20 / 20 m` | BEV ROI |
| `z_min/z_max` | `-1.5 / 2.5 m` | 高度过滤 |
| `threads` | `0` | 0 = 使用 OpenMP 默认线程数 |
| `make_visualization` | `true` | 是否生成 overlay/compare |
| `raycast` | `true` | 是否用射线标记 free space |

这些字段目前由 `PipelineConfig` 定义，CLI 参数映射在 M5 接入。

---

## 7.1 栅格地图约定

`GridMap`：

```cpp
struct GridMap {
    int width;
    int height;
    float resolution;
    Eigen::Vector2f origin;
    std::vector<std::int8_t> cells;  // 100 occupied, 0 free, -1 unknown
};
```

### 坐标

```cpp
col = floor((x - x_min) / resolution);
row = floor((y_max - y) / resolution);   // 第 0 行 = y_max
```

`origin = [x_min, y_min]` 是左下角；图像/PNG/PGM 第 0 行是顶部，对应 `y_max`。这个方向和 ROS map_server 的 `origin` 约定一致。

### 地图语义

| 值 | 含义 | PGM 像素 |
|---:|---|---:|
| 100 | occupied | 0 |
| 0 | free | 254 |
| -1 | unknown | 205 |

`occupied` 来自预处理后的点云；`free` 来自从传感器原点到 occupied cell 的 2D Bresenham 射线；被障碍物挡住的区域保持 unknown。

### 文件

- `<frame>_occupancy.pgm`：P5 二值 PGM，尺寸 = grid width × height；
- `<frame>_occupancy.yaml`：`image/resolution/origin/negate/occupied_thresh/free_thresh`，并附带 `frame_id: velodyne`、`height_min/max`、`raycast`、`inflation_cells`；
- `<frame>_occupancy.png`：仅供预览，白=free、黑=occupied、灰=unknown。

### 注意

- M4 栅格使用**原始点云**，不是 M2 的 `fused.pcd`；后者只含有效投影点；
- 当前 raycast 是 2D 近似，不建模光束高度；
- 高度过滤是近似地面去除，不是 RANSAC。

---

## 7.2 Pipeline 契约

```cpp
FrameRunResult Pipeline::run(const Frame& frame, double readMs = 0.0) const;
```

阶段顺序固定：

```text
degradation → fusion → mapping → write
```

`FrameRunResult`：

| 字段 | 含义 |
|---|---|
| `degradation` | 完整退化报告 |
| `timing` | read/degradation/fusion/mapping/write/compute/total |
| `input/projected/colored_points` | 融合统计 |
| `occupied/free/unknown_cells` | 栅格统计 |
| `outputs` | 实际写出的路径 |

输出：

- `<frame>_degradation.json`
- `<frame>_fused.pcd`（可关）
- `<frame>_overlay.png` / `<frame>_fused_view.png`（可关）
- `<frame>_occupancy.pgm` / `.yaml`（始终写）
- `<frame>_occupancy.png`（可关）
- `<frame>_timing.json`（始终写）

CLI 退出码：

| 退出码 | 含义 |
|---:|---|
| 0 | 成功 |
| 1 | 参数错误 |
| 2 | 数据/处理/IO 错误 |
| 3 | `--fail-on-degraded` 且检测到退化 |

`--no-viz` 跳过 overlay/fused_view/occupancy.png；`--no-pcd` 跳过 PCD；geometry JSON/PGM/YAML/timing 始终写出。

---

## 8. 错误与日志约定

- 所有数据层异常使用 `std::runtime_error`，消息前缀包含组件名（如 `KittiCalibration(...)`、`KittiDataSource(...)`）；
- 异常消息必须包含出问题的路径/帧号；
- 数据层不打印日志，由 CLI/Pipeline 捕获后统一输出；
- 测试使用 `tests/TestHarness.h` 的 `checkThrows` 验证错误路径。

---

## 9. 线程安全

| 对象 | 线程安全 |
|---|---|
| `Calibration` | 只读安全 |
| `Frame` | 加载完成后只读安全；共享 `cv::Mat`/点云指针，不要并发修改 |
| `IDataSource` | 否；每线程一个实例 |
| `IFusionStrategy` | 实现应为无状态或只读配置；M2 保证可并发调用 |
| `PipelineConfig` | 配置完成后只读 |
