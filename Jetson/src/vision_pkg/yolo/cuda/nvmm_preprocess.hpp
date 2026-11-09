// Copyright 2026 li
// SPDX-License-Identifier: AGPL-3.0
// H13：NVMM 帧直接在 GPU 上做（I420 全范围色转）+ letterbox + /255 + NCHW，写入 TRT 输入（float）。
// 推理前处理的唯一入口；只在 TensorRT 构建里编译。
#pragma once

#include <cstdint>

#include <cuda_runtime_api.h>

namespace vision {

// 把 NvBufSurface（单帧 I420 或 BGRx，pitch-linear）经 EGLImage 注册成 CUDA 可读指针；析构时解除注册。
class NvmmView {
public:
  explicit NvmmView(const void * nvbufsurface);
  ~NvmmView();
  NvmmView(const NvmmView &) = delete;
  NvmmView & operator=(const NvmmView &) = delete;

  bool bgrx{false};  // false：I420（MJPEG，全范围）；true：BGRx（raw 管线，VIC 已转色）
  const std::uint8_t * plane[3]{};
  int pitch[3]{};
  int width{0};
  int height{0};

private:
  void * surface_{nullptr};
  void * resource_{nullptr};
};

// 与 CPU 帧逐值一致：I420 用 camera.cpp 的整数全范围 I420→BGR；BGRx 直接取 VIC 的转换结果（与
// CPU 帧丢 x 通道相同）。随后像素中心双线性、114 填充、/255，输出 RGB 平面。
void nvmmLetterbox(
  const NvmmView & source, float * destination, int dst_width, int dst_height,
  cudaStream_t stream);

}  // namespace vision
