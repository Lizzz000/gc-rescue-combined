# 安全区测试（test/field）

只依赖 OpenCV 的纯核心测试，直接驱动 `FieldLocator::update`，不需要 ROS/TensorRT。

| 测试 | 内容 | 退出码 |
| --- | --- | --- |
| `test_field_matrix` | 单帧矩阵：干扰/视角/成像/物块网格/负例；每帧按中线可见性标为可判/边界/不可判 | 0 通过，1 有失败 |
| `test_field_sequence` | 序列：静止抖动与翻转、单/双帧注入后恢复、自动曝光爬坡与跳变；不测保持（属 bridge） | 0/1 |
| `test_field_replay` | 实拍回放：`data/<clip>/labels.yaml`，格式见 `data/README.md` | 无数据返回 77（跳过） |

## 运行

```bash
./run.sh                    # 全部；或 ./run.sh matrix --report [--filter 物块]
```

Windows：`.\run.ps1 matrix --report`。CMake 独立构建：`cmake -S . -B build && cmake --build build && ctest --test-dir build`。
板端 colcon：`colcon test --packages-select vision_pkg`（`BUILD_TESTING` 下由 vision_pkg 登记）。

## 口径

- 算法参数（N、滤波系数、`field_max_fps`）由 `field_config.py` 从 rescue.yaml/rescue_advanced.yaml 生成，不写死；
  测试常量与阈值全部在 `scene_params.hpp`。
- 中线可判性：有/无中线两次渲染差分得到看得见的中线像素，与只有相机几何和围边的干净场景比较，
  按米均匀采样计可见比例（整体与远/近三分之一段）。不可判的帧必须输出无效。
- 外框准确度：算法四条边在画内的采样点到真值内沿线（地面高度内边线）的平均距离；画外角点不计。
- 计时只做本机相对比较：设 `FIELD_TIMING_REFERENCE_MS=<同机基线中位数>` 时检查不超过 1.2 倍；绝对值需板端。
- 生成器 `../host/field_scene.hpp` 的角点真值由 Jetson 端维护；本目录的内沿真值在 `visibility.hpp` 独立计算。
