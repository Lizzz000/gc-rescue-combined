// Copyright 2026 li
// SPDX-License-Identifier: AGPL-3.0
#include "yolo/cuda/nvmm_preprocess.hpp"

#include <cuda.h>
#include <cudaEGL.h>
#include <nvbufsurface.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace vision {
namespace {

void check(CUresult result, const char * what)
{
  if (result != CUDA_SUCCESS) {
    throw std::runtime_error(std::string(what) + " failed: " + std::to_string(static_cast<int>(result)));
  }
}

// I420：与 camera.cpp vision_camera_detail::fused_i420 相同的常量和舍入。
constexpr int kY = 1220542, kUB = 2116026, kUG = -409993, kVG = -852492, kVR = 1673527;
constexpr int kRound = 1 << 19, kShift = 20;

struct Planes {
  const std::uint8_t * y;  // BGRx 时为唯一平面
  const std::uint8_t * u;
  const std::uint8_t * v;
  int pitch_y, pitch_u, pitch_v;
};

__device__ __forceinline__ float saturate(int value)
{
  return static_cast<float>(min(max(value, 0), 255));
}

// CUDA 对有符号 int 的 >> 是算术移位，等于 CPU 路径的显式 floor 除法。
template<bool Bgrx>
__device__ __forceinline__ void bgrAt(const Planes & p, int x, int y, float bgr[3])
{
  if (Bgrx) {
    const std::uint8_t * pixel = p.y + y * p.pitch_y + x * 4;
    bgr[0] = pixel[0];
    bgr[1] = pixel[1];
    bgr[2] = pixel[2];
    return;
  }
  const int luma = (219 * p.y[y * p.pitch_y + x] + 127) / 255;
  const int u = (224 * p.u[(y >> 1) * p.pitch_u + (x >> 1)] + 4095) / 255 - 128;
  const int v = (224 * p.v[(y >> 1) * p.pitch_v + (x >> 1)] + 4095) / 255 - 128;
  const int scaled = luma * kY;
  bgr[0] = saturate((scaled + kRound + kUB * u) >> kShift);
  bgr[1] = saturate((scaled + kRound + kVG * v + kUG * u) >> kShift);
  bgr[2] = saturate((scaled + kRound + kVR * v) >> kShift);
}

template<bool Bgrx>
__global__ void letterboxKernel(
  Planes planes, float * __restrict__ dst, int srcW, int srcH, int dstW, int dstH,
  int newW, int newH, int padLeft, int padTop)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= dstW || y >= dstH) {return;}
  const int planeSize = dstH * dstW;
  const int idx = y * dstW + x;
  const int srcX = x - padLeft;
  const int srcY = y - padTop;
  if (srcX < 0 || srcX >= newW || srcY < 0 || srcY >= newH) {
    constexpr float pad = 114.0f / 255.0f;
    dst[idx] = pad;
    dst[planeSize + idx] = pad;
    dst[2 * planeSize + idx] = pad;
    return;
  }
  // 像素中心采样（与原上游 kernel 打补丁后的公式逐字一致，对齐 OpenCV INTER_LINEAR）。
  const float fx = fmaxf((static_cast<float>(srcX) + 0.5f) * srcW / static_cast<float>(newW) - 0.5f, 0.0f);
  const float fy = fmaxf((static_cast<float>(srcY) + 0.5f) * srcH / static_cast<float>(newH) - 0.5f, 0.0f);
  const int x0 = static_cast<int>(fx);
  const int y0 = static_cast<int>(fy);
  const int x1 = min(x0 + 1, srcW - 1);
  const int y1 = min(y0 + 1, srcH - 1);
  const float ax = fx - x0;
  const float ay = fy - y0;
  float p00[3], p01[3], p10[3], p11[3];
  bgrAt<Bgrx>(planes, x0, y0, p00);
  bgrAt<Bgrx>(planes, x1, y0, p01);
  bgrAt<Bgrx>(planes, x0, y1, p10);
  bgrAt<Bgrx>(planes, x1, y1, p11);
  constexpr float normScale = 1.0f / 255.0f;
#pragma unroll
  for (int c = 0; c < 3; ++c) {
    const int srcC = 2 - c;
    const float val = (1.0f - ax) * (1.0f - ay) * p00[srcC]
      + ax * (1.0f - ay) * p01[srcC]
      + (1.0f - ax) * ay * p10[srcC]
      + ax * ay * p11[srcC];
    dst[c * planeSize + idx] = val * normScale;
  }
}

}  // namespace

NvmmView::NvmmView(const void * nvbufsurface)
: surface_(const_cast<void *>(nvbufsurface))
{
  if (surface_ == nullptr) {
    throw std::invalid_argument(
      "segmentation requires an NVMM frame; file, V4L2 and pipelines that did not negotiate NVMM cannot segment");
  }
  // Driver API 不会给新线程绑定主上下文。Runtime 的 cudaSetDevice 会。采集线程上的 NVDEC 绑过的上下文带不过来。
  const cudaError_t device_error = cudaSetDevice(0);
  if (device_error != cudaSuccess) {
    throw std::runtime_error(std::string("cudaSetDevice(0) failed: ") + cudaGetErrorString(device_error));
  }
  auto * surface = static_cast<NvBufSurface *>(surface_);
  const auto format = surface->surfaceList[0].colorFormat;
  // YUV420 与 YUV420_ER 都是分开的 Y/U/V 平面。JPEG 全范围系数由核自己套，不接受 NV12 这类交错色度。
  const bool i420 = format == NVBUF_COLOR_FORMAT_YUV420 || format == NVBUF_COLOR_FORMAT_YUV420_ER;
  const bool bgrx_format = format == NVBUF_COLOR_FORMAT_BGRx;
  if (!i420 && !bgrx_format) {
    throw std::invalid_argument("NVMM segmentation only accepts planar I420 or BGRx");
  }
  const int surface_width = static_cast<int>(surface->surfaceList[0].width);
  const int surface_height = static_cast<int>(surface->surfaceList[0].height);
  if (surface_width < 2 || surface_height < 2 ||
    (i420 && (surface_width % 2 != 0 || surface_height % 2 != 0)))
  {
    throw std::invalid_argument("NVMM frame dimensions are not usable for letterbox");
  }
  if (NvBufSurfaceMapEglImage(surface, 0) != 0) {
    throw std::runtime_error("NvBufSurfaceMapEglImage failed");
  }
  CUgraphicsResource resource = nullptr;
  CUeglFrame frame{};
  CUresult result = cuGraphicsEGLRegisterImage(
    &resource, surface->surfaceList[0].mappedAddr.eglImage, CU_GRAPHICS_REGISTER_FLAGS_READ_ONLY);
  if (result == CUDA_SUCCESS) {
    result = cuGraphicsResourceGetMappedEglFrame(&frame, resource, 0, 0);
    if (result != CUDA_SUCCESS) {cuGraphicsUnregisterResource(resource);}
  }
  if (result == CUDA_SUCCESS && frame.frameType != CU_EGL_FRAME_TYPE_PITCH) {
    cuGraphicsUnregisterResource(resource);
    result = CUDA_ERROR_INVALID_VALUE;
  }
  if (result != CUDA_SUCCESS) {
    NvBufSurfaceUnMapEglImage(surface, 0);
    check(result, "CUDA EGL mapping of NVMM frame");
  }
  resource_ = resource;
  bgrx = bgrx_format;
  const auto & params = surface->surfaceList[0].planeParams;
  const int plane_count = bgrx ? 1 : 3;
  for (int p = 0; p < plane_count; ++p) {
    plane[p] = static_cast<const std::uint8_t *>(frame.frame.pPitch[p]);
    pitch[p] = static_cast<int>(params.pitch[p]);
    if (plane[p] == nullptr || pitch[p] <= 0) {
      cuGraphicsUnregisterResource(resource);
      NvBufSurfaceUnMapEglImage(surface, 0);
      resource_ = nullptr;
      throw std::runtime_error("NVMM plane pointer or pitch is unusable");
    }
  }
  width = static_cast<int>(frame.width);
  height = static_cast<int>(frame.height);
  if (width != surface_width || height != surface_height) {
    cuGraphicsUnregisterResource(resource);
    NvBufSurfaceUnMapEglImage(surface, 0);
    resource_ = nullptr;
    throw std::runtime_error("NVMM EGL frame size does not match the surface");
  }
}

NvmmView::~NvmmView()
{
  cuGraphicsUnregisterResource(static_cast<CUgraphicsResource>(resource_));
  NvBufSurfaceUnMapEglImage(static_cast<NvBufSurface *>(surface_), 0);
}

void nvmmLetterbox(
  const NvmmView & source, float * destination, int dst_width, int dst_height,
  cudaStream_t stream)
{
  const int srcW = source.width;
  const int srcH = source.height;
  if (srcW < 2 || srcH < 2 || dst_width < 2 || dst_height < 2 || destination == nullptr) {
    throw std::invalid_argument("letterbox source and network size must be at least 2 pixels");
  }
  // 几何与上游 letterboxPreprocess / computeScalePad 相同。
  const float scale = fminf(static_cast<float>(dst_height) / srcH, static_cast<float>(dst_width) / srcW);
  const int newW = static_cast<int>(roundf(srcW * scale));
  const int newH = static_cast<int>(roundf(srcH * scale));
  if (newW < 1 || newH < 1) {
    throw std::invalid_argument("letterbox scale collapsed a source side to zero");
  }
  const int padLeft = static_cast<int>(roundf((dst_width - newW) / 2.0f - 0.1f));
  const int padTop = static_cast<int>(roundf((dst_height - newH) / 2.0f - 0.1f));
  const Planes planes{source.plane[0], source.plane[1], source.plane[2],
    source.pitch[0], source.pitch[1], source.pitch[2]};
  const dim3 block(32, 8);
  const dim3 grid((dst_width + block.x - 1) / block.x, (dst_height + block.y - 1) / block.y);
  if (source.bgrx) {
    letterboxKernel<true><<<grid, block, 0, stream>>>(
      planes, destination, srcW, srcH, dst_width, dst_height, newW, newH, padLeft, padTop);
  } else {
    letterboxKernel<false><<<grid, block, 0, stream>>>(
      planes, destination, srcW, srcH, dst_width, dst_height, newW, newH, padLeft, padTop);
  }
}

}  // namespace vision
