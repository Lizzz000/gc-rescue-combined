#include "camera/gstreamer/nvmm_pixels.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
int mapped = 0, synced = 0, unmapped = 0;
int map_result = 0, sync_result = 0;
int passed = 0, failed = 0;

void check(bool value, const char * label)
{
  if (value) {++passed;} else {++failed; std::cerr << "FAIL: " << label << '\n';}
}

struct Frame {
  unsigned char storage[3][256]{};
  NvBufSurfaceParams params{};
  NvBufSurface surface{1, 1, &params};
  explicit Frame(NvBufSurfaceColorFormat format = NVBUF_COLOR_FORMAT_YUV420)
  {
    params.width = 8;
    params.height = 4;
    params.colorFormat = format;
    const bool packed = format == NVBUF_COLOR_FORMAT_BGRx;
    params.planeParams.num_planes = packed ? 1 : 3;
    for (int plane = 0; plane < 3; ++plane) {
      const auto width = plane == 0 ? 8u : 4u;
      const auto height = plane == 0 ? 4u : 2u;
      const auto bytes = packed ? 4u : 1u;
      params.planeParams.width[plane] = width;
      params.planeParams.height[plane] = height;
      params.planeParams.pitch[plane] = width * bytes + 8;
      params.planeParams.psize[plane] = params.planeParams.pitch[plane] * height;
      params.planeParams.bytesPerPix[plane] = bytes;
      params.mappedAddr.addr[plane] = storage[plane];
    }
  }
};

void reset()
{
  mapped = synced = unmapped = 0;
  map_result = sync_result = 0;
}

bool consume(Frame & frame, bool require_bgr, int & copies, std::string & error)
{
  return vision_camera::detail::withNvmmCpuPixels(&frame.surface, require_bgr, error,
    [&](const NvBufSurfaceParams &) {++copies;});
}

void rejected(Frame & frame, const char * label, bool require_bgr = false)
{
  reset();
  int copies = 0;
  std::string error;
  const bool result = consume(frame, require_bgr, copies, error);
  check(!result && copies == 0 && !error.empty(), label);
  check(mapped == unmapped, "rejected frame releases every successful CPU map");
}
}  // namespace

int NvBufSurfaceMap(NvBufSurface *, int, int, NvBufSurfaceMemMapFlags)
{
  if (map_result == 0) {++mapped;}
  return map_result;
}
int NvBufSurfaceSyncForCpu(NvBufSurface *, int, int) {++synced; return sync_result;}
int NvBufSurfaceUnMap(NvBufSurface *, int, int) {++unmapped; return 0;}

int main()
{
  {
    reset();
    std::string error;
    int copies = 0;
    check(!vision_camera::detail::withNvmmCpuPixels(nullptr, false, error,
      [&](const NvBufSurfaceParams &) {++copies;}) && copies == 0 && mapped == 0,
      "missing surface is rejected before the SDK map call");
  }
  {
    Frame frame;
    frame.surface.surfaceList = nullptr;
    rejected(frame, "missing surface list");
  }
  {
    Frame frame;
    frame.surface.batchSize = 0;
    rejected(frame, "empty surface batch");
  }
  for (const auto filled : {0u, 2u}) {
    Frame frame;
    frame.surface.numFilled = filled;
    rejected(frame, "capture requires exactly one populated image");
  }
  for (const auto format : {NVBUF_COLOR_FORMAT_YUV420, NVBUF_COLOR_FORMAT_YUV420_ER,
      NVBUF_COLOR_FORMAT_BGRx})
  {
    reset();
    Frame frame(format);
    int copies = 0;
    std::string error;
    check(consume(frame, false, copies, error) && copies == 1,
      "default planar I420/full-range I420/BGRx remains supported with padded rows");
    check(mapped == 1 && synced == 1 && unmapped == 1, "successful copy maps/syncs/unmaps once");
    rejected(frame, "custom BGR contract rejects NVMM I420/BGRx", true);
  }
  for (const auto format : {NVBUF_COLOR_FORMAT_NV12, NVBUF_COLOR_FORMAT_RGBA}) {
    Frame frame(format);
    rejected(frame, "NV12/RGBA cannot reach a three-plane copy");
  }
  {
    Frame frame;
    frame.params.planeParams.num_planes = 2;
    rejected(frame, "I420 requires three planes");
  }
  {
    Frame frame(NVBUF_COLOR_FORMAT_BGRx);
    frame.params.planeParams.num_planes = 2;
    rejected(frame, "BGRx requires one plane");
  }
  for (int plane = 0; plane < 3; ++plane) {
    Frame frame;
    frame.params.mappedAddr.addr[plane] = nullptr;
    rejected(frame, "missing Y/U/V mapping is rejected before copying");
  }
  {
    Frame frame;
    frame.params.planeParams.pitch[1] = 3;
    rejected(frame, "chroma row shorter than visible width");
  }
  {
    Frame frame(NVBUF_COLOR_FORMAT_BGRx);
    frame.params.planeParams.pitch[0] = 31;
    rejected(frame, "packed row needs four bytes per pixel");
  }
  {
    Frame frame;
    frame.params.planeParams.psize[2] = 2;
    rejected(frame, "plane allocation shorter than final visible row");
  }
  {
    Frame frame;
    frame.params.planeParams.height[1] = 1;
    rejected(frame, "chroma plane has insufficient rows");
  }
  {
    Frame frame;
    frame.params.planeParams.width[2] = 3;
    rejected(frame, "chroma plane has insufficient columns");
  }
  {
    Frame frame;
    frame.params.planeParams.bytesPerPix[1] = 2;
    rejected(frame, "planar chroma must have one byte per pixel");
  }
  for (const auto width : {0u, 7u, std::numeric_limits<unsigned>::max()}) {
    Frame frame;
    frame.params.width = width;
    rejected(frame, "I420 dimensions must be positive, even, and fit OpenCV int");
  }
  {
    Frame frame;
    frame.params.height = 1431655766u;
    rejected(frame, "I420 combined Mat height cannot overflow int");
  }
  {
    Frame frame;
    reset();
    map_result = -1;
    int copies = 0;
    std::string error;
    check(!consume(frame, false, copies, error) && copies == 0 && unmapped == 0,
      "failed map is not copied or unmapped");
  }
  {
    Frame frame;
    reset();
    sync_result = -1;
    int copies = 0;
    std::string error;
    check(!consume(frame, false, copies, error) && copies == 0 && unmapped == 1,
      "failed CPU synchronization releases map without reading pixels");
  }
  {
    Frame frame;
    reset();
    std::string error;
    try {
      vision_camera::detail::withNvmmCpuPixels(&frame.surface, false, error,
        [](const NvBufSurfaceParams &) {throw std::runtime_error("copy/allocation failed");});
      check(false, "copy exception propagates");
    } catch (const std::runtime_error &) {
      check(mapped == 1 && unmapped == 1, "copy/OpenCV exception still releases CPU mapping");
    }
  }
  std::cout << "NVMM pixels: " << passed << " passed, " << failed << " failed\n";
  return failed == 0 ? 0 : 1;
}
