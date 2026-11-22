#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <nvbufsurface.h>

namespace vision_camera {

// 文件回放的 CPU BGR 帧上传到与生产推理兼容的 BGRx 表面。调用方持有源像素直到返回。
inline std::shared_ptr<NvBufSurface> uploadReplayBgr(
  const std::uint8_t * data, int width, int height, std::size_t stride, std::string & error)
{
  if (!data || width < 2 || height < 2 ||
    static_cast<std::uint64_t>(width) * 4 > std::numeric_limits<std::size_t>::max())
  {
    error = "NVMM replay requires a usable BGR image";
    return {};
  }
  const auto source_bytes = static_cast<std::size_t>(width) * 3;
  const auto destination_bytes = static_cast<std::size_t>(width) * 4;
  const auto last_row = static_cast<std::size_t>(height - 1);
  if (stride < source_bytes ||
    stride > (std::numeric_limits<std::size_t>::max() - source_bytes) / last_row)
  {
    error = "NVMM replay source stride cannot contain its visible pixels";
    return {};
  }

  NvBufSurfaceCreateParams configuration{};
  configuration.width = static_cast<std::uint32_t>(width);
  configuration.height = static_cast<std::uint32_t>(height);
  configuration.colorFormat = NVBUF_COLOR_FORMAT_BGRx;
  configuration.layout = NVBUF_LAYOUT_PITCH;
  configuration.memType = NVBUF_MEM_SURFACE_ARRAY;
  NvBufSurface * allocation = nullptr;
  if (NvBufSurfaceCreate(&allocation, 1, &configuration) != 0) {
    error = "NvBufSurfaceCreate failed for NVMM replay";
    return {};
  }
  auto surface = std::shared_ptr<NvBufSurface>(allocation, NvBufSurfaceDestroy);
  const auto & plane = surface->surfaceList[0];
  const auto & layout = plane.planeParams;
  const auto pitch = layout.pitch[0];
  const std::uint64_t end = static_cast<std::uint64_t>(last_row) * pitch + destination_bytes;
  if (plane.width != configuration.width || plane.height != configuration.height ||
    plane.colorFormat != configuration.colorFormat || layout.num_planes != 1 ||
    layout.width[0] < configuration.width || layout.height[0] < configuration.height ||
    layout.bytesPerPix[0] != 4 || pitch < destination_bytes || layout.psize[0] < end)
  {
    error = "NVMM replay surface cannot contain its visible BGRx pixels";
    return {};
  }
  if (NvBufSurfaceMap(surface.get(), 0, 0, NVBUF_MAP_WRITE) != 0) {
    error = "NvBufSurfaceMap failed for NVMM replay";
    return {};
  }

  auto * destination = static_cast<std::uint8_t *>(plane.mappedAddr.addr[0]);
  bool synced = false;
  if (destination) {
    for (int y = 0; y < height; ++y) {
      const auto * source = data + static_cast<std::size_t>(y) * stride;
      auto * row = destination + static_cast<std::size_t>(y) * pitch;
      for (int x = 0; x < width; ++x) {
        row[0] = source[0];
        row[1] = source[1];
        row[2] = source[2];
        row[3] = 0;
        row += 4;
        source += 3;
      }
    }
    synced = NvBufSurfaceSyncForDevice(surface.get(), 0, 0) == 0;
  }
  const bool unmapped = NvBufSurfaceUnMap(surface.get(), 0, 0) == 0;
  if (!destination) {
    error = "NVMM replay plane has no CPU mapping";
    return {};
  }
  if (!synced) {
    error = "NvBufSurfaceSyncForDevice failed for NVMM replay";
    return {};
  }
  if (!unmapped) {
    error = "NvBufSurfaceUnMap failed for NVMM replay";
    return {};
  }
  surface->numFilled = 1;
  error.clear();
  return surface;
}

}  // namespace vision_camera
