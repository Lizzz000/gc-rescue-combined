# CPU 分割后处理回归

只需 C++17 与 OpenCV core/imgproc，不需要 ROS、TensorRT、CUDA、模型或相机。

```sh
cmake -S src/vision_pkg/test/postprocess -B build-postprocess -DCMAKE_BUILD_TYPE=Release
cmake --build build-postprocess
ctest --test-dir build-postprocess --output-on-failure
```

板端正常构建在 `BUILD_TESTING` 下登记同一个目标。

测试直接调用生产 `SegPostprocessor::run`。独立参考实现保留原始稳定排序和同类别 greedy NMS，并用完整原型矩阵计算 logits，对照生产的局部行带 GEMM。逐项比较检测顺序、置信度、类别、裁剪后的框、标签图及每个实例的二值掩膜。

覆盖 end-to-end/legacy 两种布局、同分抑制和输出预算、异常模型数值、类别饱和、画外框、奇数尺寸与 1×1、掩膜开关和预算、跨帧容量增长及输出所有权。随机测试使用固定种子；阈值、双线性插值、legacy 去 padding/最近邻降采样顺序保持原约定。

这些测试验证 CPU 算法和接口去耦，不替代板端 TensorRT 引擎、CUDA kernel、NVMM 映射和 ROS ABI 集成验证。
