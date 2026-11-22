#include "camera/gstreamer/nvmm_replay.hpp"

#include <algorithm>
#include <array>
#include <initializer_list>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

enum class Failure {None, Create, Pitch, Map, NullMapping, Sync, Unmap};
Failure failure = Failure::None;
NvBufSurfaceCreateParams requested{};
NvBufSurfaceMemMapFlags map_flags = NVBUF_MAP_READ;
std::uint32_t requested_batch = 0;
NvBufSurfaceParams surface_params{};
NvBufSurface sdk_surface{1, 0, &surface_params};

// 输入、输出都带行填充，奇数宽度覆盖普通 BGR 与 BGRx 的不同像素步长。
constexpr int width = 3;
constexpr int height = 2;
constexpr std::size_t source_pitch = 12, destination_pitch = 20;
constexpr std::uint8_t sentinel = 0xcd;
const std::array<std::uint8_t, source_pitch * height> source{
  1, 2, 3, 4, 5, 6, 7, 8, 9, sentinel, sentinel, sentinel,
  10, 11, 12, 13, 14, 15, 16, 17, 18, sentinel, sentinel, sentinel};
std::array<std::uint8_t, destination_pitch * height> destination{};
std::vector<std::string_view> calls;
int passed = 0, failed = 0;

void check(bool value, const char * label)
{
  if (value) {++passed;} else {++failed; std::cerr << "FAIL: " << label << '\n';}
}

bool called(std::initializer_list<std::string_view> expected)
{
  return std::equal(expected.begin(), expected.end(), calls.begin(), calls.end());
}

void expectFailure(
  Failure injected, std::string_view diagnostic, std::initializer_list<std::string_view> expected)
{
  failure = injected;
  calls.clear();
  std::string error;
  const auto result = vision_camera::uploadReplayBgr(source.data(), width, height, source_pitch, error);
  check(!result && error.find(diagnostic) != std::string::npos, "失败回放不给出已上传的帧");
  check(called(expected), "失败后按 SDK 资源获取顺序清理");
}
}  // namespace

int NvBufSurfaceCreate(NvBufSurface ** surface, std::uint32_t batch, NvBufSurfaceCreateParams * config)
{
  calls.push_back("create");
  requested = *config;
  requested_batch = batch;
  if (failure == Failure::Create) {return -1;}
  surface_params = {};
  surface_params.width = config->width;
  surface_params.height = config->height;
  surface_params.colorFormat = config->colorFormat;
  surface_params.planeParams.num_planes = 1;
  surface_params.planeParams.width[0] = config->width;
  surface_params.planeParams.height[0] = config->height;
  surface_params.planeParams.pitch[0] = failure == Failure::Pitch ? 11 : destination_pitch;
  surface_params.planeParams.psize[0] = destination.size();
  surface_params.planeParams.bytesPerPix[0] = 4;
  sdk_surface.numFilled = 0;
  destination.fill(sentinel);
  *surface = &sdk_surface;
  return 0;
}

int NvBufSurfaceMap(NvBufSurface * surface, int index, int plane, NvBufSurfaceMemMapFlags flags)
{
  check(surface == &sdk_surface && index == 0 && plane == 0, "只映射回放表面的第一个平面");
  calls.push_back("map");
  map_flags = flags;
  if (failure == Failure::Map) {return -1;}
  surface_params.mappedAddr.addr[0] =
    failure == Failure::NullMapping ? nullptr : destination.data();
  return 0;
}

int NvBufSurfaceSyncForDevice(NvBufSurface * surface, int index, int plane)
{
  check(surface == &sdk_surface && index == 0 && plane == 0, "只同步已上传的第一个平面");
  check(surface->numFilled == 1, "同步前驱动要求表面已记录填充帧数");
  calls.push_back("sync");
  return failure == Failure::Sync ? -1 : 0;
}

int NvBufSurfaceUnMap(NvBufSurface * surface, int index, int plane)
{
  check(surface == &sdk_surface && index == 0 && plane == 0, "释放同一 CPU 映射");
  calls.push_back("unmap");
  surface_params.mappedAddr.addr[0] = nullptr;
  return failure == Failure::Unmap ? -1 : 0;
}

int NvBufSurfaceDestroy(NvBufSurface * surface)
{
  check(surface == &sdk_surface, "释放已分配的 SDK 表面");
  calls.push_back("destroy");
  return 0;
}

int main()
{
  std::string error = "previous error";
  auto result = vision_camera::uploadReplayBgr(source.data(), width, height, source_pitch, error);
  check(result && result->numFilled == 1 && error.empty(), "交付一个填充完整的 NVMM 表面");
  check(requested.width == width && requested.height == height && requested_batch == 1 &&
    requested.colorFormat == NVBUF_COLOR_FORMAT_BGRx &&
    requested.layout == NVBUF_LAYOUT_PITCH && requested.memType == NVBUF_MEM_SURFACE_ARRAY,
    "为 CUDA EGL 分配单个线性 BGRx Jetson 表面");
  check(map_flags == NVBUF_MAP_WRITE && called({"create", "map", "sync", "unmap"}),
    "CPU 写入后先同步到设备再解映射");
  auto expected = destination;
  expected.fill(sentinel);
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      std::copy_n(source.data() + y * source_pitch + x * 3, 3,
        expected.data() + y * destination_pitch + x * 4);
      expected[y * destination_pitch + x * 4 + 3] = 0;
    }
  }
  check(destination == expected, "保持 BGR 顺序、X 字节和行填充");
  auto second_owner = result;
  result.reset();
  check(second_owner.get() == &sdk_surface && called({"create", "map", "sync", "unmap"}),
    "仍有消费者时保留同一 SDK 表面");
  second_owner.reset();
  check(called({"create", "map", "sync", "unmap", "destroy"}), "最后一个引用销毁表面一次");

  expectFailure(Failure::Create, "NvBufSurfaceCreate", {"create"});
  expectFailure(Failure::Pitch, "visible BGRx pixels", {"create", "destroy"});
  expectFailure(Failure::Map, "NvBufSurfaceMap", {"create", "map", "destroy"});
  expectFailure(Failure::NullMapping, "no CPU mapping", {"create", "map", "unmap", "destroy"});
  expectFailure(Failure::Sync, "NvBufSurfaceSyncForDevice", {"create", "map", "sync", "unmap", "destroy"});
  expectFailure(Failure::Unmap, "NvBufSurfaceUnMap", {"create", "map", "sync", "unmap", "destroy"});

  calls.clear();
  result = vision_camera::uploadReplayBgr(source.data(), width, height, 8, error);
  check(!result && calls.empty() && !error.empty(), "短源行在调用 SDK 前被拒绝");
  result = vision_camera::uploadReplayBgr(source.data(), 1, height, source_pitch, error);
  check(!result && calls.empty() && !error.empty(), "CUDA 前处理不接受的尺寸在分配前被拒绝");
  std::cout << "NVMM replay: " << passed << " passed, " << failed << " failed\n";
  return failed != 0;
}
