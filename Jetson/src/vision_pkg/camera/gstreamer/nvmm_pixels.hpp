#pragma once

#include <nvbufsurface.h>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace vision_camera::detail {
// The callback consumes CPU-mapped pixels synchronously. It must not retain plane pointers.
template<class Copy>
bool withNvmmCpuPixels(NvBufSurface * surface, bool require_bgr, std::string & error, Copy && copy)
{
  // The automatic pipeline supports I420/BGRx; a custom pipeline must deliver CPU BGR.
  // Keep this gate in the NVMM entry, so its early dispatch cannot bypass the BGR contract.
  if (require_bgr) {
    error = "custom camera pipeline must deliver BGR at appsink; NVMM I420/BGRx is not BGR";
    return false;
  }
  if (!surface || !surface->surfaceList || surface->batchSize < 1 || surface->numFilled != 1) {
    error = "NVMM capture requires one populated surface";
    return false;
  }
  const auto & params = surface->surfaceList[0];
  const bool i420 = params.colorFormat == NVBUF_COLOR_FORMAT_YUV420 ||
    params.colorFormat == NVBUF_COLOR_FORMAT_YUV420_ER;
  const bool bgrx = params.colorFormat == NVBUF_COLOR_FORMAT_BGRx;
  if (!i420 && !bgrx) {
    error = "NVMM capture only supports planar I420 or BGRx";
    return false;
  }
  constexpr auto max_int = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  if (params.width == 0 || params.height == 0 || params.width > max_int || params.height > max_int ||
    (i420 && (params.width % 2 || params.height % 2 || params.height > max_int / 3 * 2)))
  {
    error = "NVMM capture dimensions are invalid or exceed the image representation";
    return false;
  }
  const unsigned planes = bgrx ? 1 : 3;
  if (params.planeParams.num_planes != planes) {
    error = "NVMM plane count does not match its pixel format";
    return false;
  }
  for (unsigned plane = 0; plane < planes; ++plane) {
    const std::uint64_t columns = plane == 0 ? params.width : params.width / 2;
    const std::uint64_t rows = plane == 0 ? params.height : params.height / 2;
    const std::uint64_t pixel_bytes = bgrx ? 4 : 1;
    const auto & layout = params.planeParams;
    const std::uint64_t row_bytes = columns * pixel_bytes;
    const std::uint64_t last_row_end = (rows - 1) * layout.pitch[plane] + row_bytes;
    if (layout.width[plane] < columns || layout.height[plane] < rows ||
      layout.bytesPerPix[plane] != pixel_bytes || layout.pitch[plane] < row_bytes ||
      layout.psize[plane] < last_row_end)
    {
      error = "NVMM plane dimensions, stride or allocation cannot contain its visible pixels";
      return false;
    }
  }
  if (NvBufSurfaceMap(surface, 0, -1, NVBUF_MAP_READ) != 0) {
    error = "NvBufSurfaceMap failed";
    return false;
  }
  struct Unmap {
    NvBufSurface * surface;
    ~Unmap() {NvBufSurfaceUnMap(surface, 0, -1);}
  } unmap{surface};
  if (NvBufSurfaceSyncForCpu(surface, 0, -1) != 0) {
    error = "NvBufSurfaceSyncForCpu failed";
    return false;
  }
  for (unsigned plane = 0; plane < planes; ++plane) {
    if (!params.mappedAddr.addr[plane]) {
      error = "NVMM plane has no CPU mapping";
      return false;
    }
  }
  std::forward<Copy>(copy)(params);
  return true;
}
}  // namespace vision_camera::detail
