# 样例数据

`data/sample/` 是 M0 使用的**合成单帧测试数据**，不是真实 KITTI 数据。它的作用是：

- 验证 `image_2 / velodyne / calib` 三件套布局；
- 验证 `.bin` 的 float32 四元组格式与 calib 必需字段；
- 为 M1/M2 的数据读取、投影和叠加图提供一个确定性的最小输入。

## 目录

```text
data/sample/
├── image_2/
│   └── 000000.png       # 1242x375 合成图
├── velodyne/
│   └── 000000.bin       # float32 x,y,z,intensity
└── calib/
    └── 000000.txt       # P0-P3, R0_rect, Tr_velo_to_cam
```

## 重新生成

需要 Python 3、NumPy、OpenCV：

```bash
python scripts/make_sample_fixture.py
```

## 替换成真实 KITTI

官方 3D Object Detection benchmark：<https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d>

下载后从训练集取一帧（仅复制单帧，不要提交完整数据集）：

```text
data_object_image_2/training/image_2/000000.png
data_object_velodyne/training/velodyne/000000.bin
data_object_calib/training/calib/000000.txt
```

保持相同目录结构：

```text
data/sample/
├── image_2/000000.png
├── velodyne/000000.bin
└── calib/000000.txt
```

注意：

- 完整 KITTI 数据集不应提交到 Git（`.gitignore` 已忽略 `data/kitti/`）；
- KITTI 数据受其自身许可约束（非商业使用，具体以官网为准），README 中需注明来源；
- 合成数据的标定是简化模型，真实 KITTI 请使用其 `P2`、`R0_rect`、`Tr_velo_to_cam` 原值；
- 当前 `data/sample` 仍是合成样例，只用于 M0 校验；真实 KITTI 帧在 M1/M2 做投影验收时替换。
