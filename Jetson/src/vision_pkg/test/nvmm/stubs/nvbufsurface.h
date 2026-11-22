#pragma once

#include <cstdint>

// SDK boundary only: the test calls the production validation/map/copy lifetime helper.
enum NvBufSurfaceColorFormat {
  NVBUF_COLOR_FORMAT_YUV420, NVBUF_COLOR_FORMAT_YUV420_ER,
  NVBUF_COLOR_FORMAT_BGRx, NVBUF_COLOR_FORMAT_NV12, NVBUF_COLOR_FORMAT_RGBA
};
enum NvBufSurfaceMemMapFlags {NVBUF_MAP_READ, NVBUF_MAP_WRITE};
enum NvBufSurfaceLayout {NVBUF_LAYOUT_PITCH};
enum NvBufSurfaceMemType {NVBUF_MEM_SURFACE_ARRAY};
struct NvBufSurfaceCreateParams {
  std::uint32_t width{}, height{};
  NvBufSurfaceColorFormat colorFormat{};
  NvBufSurfaceLayout layout{};
  NvBufSurfaceMemType memType{};
};
struct NvBufSurfacePlaneParams {
  std::uint32_t num_planes{};
  std::uint32_t width[4]{}, height[4]{}, pitch[4]{}, offset[4]{}, psize[4]{}, bytesPerPix[4]{};
};
struct NvBufSurfaceMappedAddr {void * addr[4]{};};
struct NvBufSurfaceParams {
  std::uint32_t width{}, height{};
  NvBufSurfaceColorFormat colorFormat{};
  NvBufSurfacePlaneParams planeParams{};
  NvBufSurfaceMappedAddr mappedAddr{};
};
struct NvBufSurface {
  std::uint32_t batchSize{}, numFilled{};
  NvBufSurfaceParams * surfaceList{};
};
int NvBufSurfaceMap(NvBufSurface *, int, int, NvBufSurfaceMemMapFlags);
int NvBufSurfaceSyncForCpu(NvBufSurface *, int, int);
int NvBufSurfaceSyncForDevice(NvBufSurface *, int, int);
int NvBufSurfaceUnMap(NvBufSurface *, int, int);
int NvBufSurfaceCreate(NvBufSurface **, std::uint32_t, NvBufSurfaceCreateParams *);
int NvBufSurfaceDestroy(NvBufSurface *);
