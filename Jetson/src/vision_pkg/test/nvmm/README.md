# NVMM 像素入口回归

只需 C++17 编译器和 CMake，Windows/Linux 均可运行，不需要 ROS、OpenCV 或 Jetson SDK。

```sh
cmake -S Jetson/src/vision_pkg/test/nvmm -B /path/to/build/nvmm
cmake --build /path/to/build/nvmm
ctest --test-dir /path/to/build/nvmm --output-on-failure
```

测试直接调用采集路径使用的 `camera/gstreamer/nvmm_pixels.hpp`。桩仅提供 SDK
结构和 map/sync/unmap 调用结果，不重写生产校验。覆盖默认 I420、全范围 I420、BGRx，
自定义管线 BGR 约束、拒绝 NV12/RGBA、损坏平面和尺寸，以及失败/异常路径的解除映射。
不能代替板端 GStreamer、实际像素色转或 CUDA 集成测试。
