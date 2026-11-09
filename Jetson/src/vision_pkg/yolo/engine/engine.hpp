#pragma once
#include <atomic>
#include <chrono>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace vision
{
constexpr std::size_t kBytesPerMiB = 1024U * 1024U;
constexpr std::size_t kMaxWorkspaceMiB =
  std::numeric_limits<std::size_t>::max() / kBytesPerMiB;

// 只接受完整十进制数字，拒绝负号、空白、尾随字符及整数/单位转换溢出。
std::size_t parseBuildInteger(
  std::string_view text, std::size_t minimum, std::size_t maximum, const char * name);

bool writeBinaryPayload(std::ostream & stream, const void * data, std::size_t size);

bool writeBinaryComplete(
  const std::filesystem::path & path, const void * data, std::size_t size);

// 校验临时文件后才 rename；写入、关闭、内容校验或取消失败都保留正式目标。
// 不承诺断电持久性；调用方的 verify 必须校验其真实文件格式。
template<class Verify, class Cancelled>
bool commitVerifiedBinary(
  const std::filesystem::path & destination, const void * data, std::size_t size,
  Verify && verify, Cancelled && cancelled, std::string & error)
{
  if (cancelled()) {error = "engine write cancelled"; return false;}
  // 独占创建同目录下的临时目录；并行进程和重入调用都不能写入/清理彼此的文件。
  // 时间戳仅生成候选名，所有权由 create_directory 的原子成功决定。
  static std::atomic<std::uint64_t> sequence{0};
  std::filesystem::path staging;
  bool owned = false;
  for (int attempt = 0; attempt < 64; ++attempt) {
    staging = destination.string() + ".partial-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
      std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
    std::error_code create_error;
    if (std::filesystem::create_directory(staging, create_error)) {owned = true; break;}
    if (create_error && create_error != std::errc::file_exists) {
      error = "cannot create engine staging directory: " + create_error.message();
      return false;
    }
  }
  if (!owned) {error = "cannot reserve a unique engine staging directory"; return false;}
  struct Cleanup
  {
    std::filesystem::path directory;
    ~Cleanup()
    {
      std::error_code ec;
      std::filesystem::remove(directory / "payload", ec);
      std::filesystem::remove(directory, ec);
    }
  } cleanup{staging};
  const std::filesystem::path temporary = staging / "payload";
  if (!writeBinaryComplete(temporary, data, size)) {
    error = "cannot completely write and close " + temporary.string();
    return false;
  }
  std::error_code ec;
  const auto bytes = std::filesystem::file_size(temporary, ec);
  if (ec || bytes != size) {
    error = "incomplete engine file: " + temporary.string();
    return false;
  }
  if (cancelled()) {error = "engine write cancelled"; return false;}
  if (!verify(temporary, error)) {return false;}
  if (cancelled()) {error = "engine write cancelled"; return false;}
  std::filesystem::rename(temporary, destination, ec);
  if (ec) {error = "cannot move the engine into place: " + ec.message(); return false;}
  error.clear();
  return true;
}
}  // namespace vision

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 不依赖 TensorRT 的输入合同：离线精度/校准身份、分割张量布局、协作取消及引擎缓存。
// 生产加载复用分割张量校验；构建、精度回退和缓存迁移只服务显式离线入口。
// 纯值检查可在无 GPU 环境运行，但不能证明真实引擎可执行。

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// ---- 离线构建的精度尝试与校准数据身份 ----
/// 已规范化的精度名（fp32 / fp16 / int8）。未知或空串返回空列表。
/// int8 的第二项固定是 fp16：部分 TensorRT 10.x 无法以 INT8 构建 YOLO26 end2end 图，
/// 回退是预定路径。板端是否仍会走这条回退要实测，顺序本身不依赖板端。
std::vector<std::string> precisionAttempts(const std::string & requested);

// 给标定器和 engine 缓存同一份确定排序的图片清单和内容指纹，不解码图片或访问 GPU。
// 清单语义变化须递增 kBuilderRevision，使旧标定集的引擎缓存失效。

// 仅扫描目录直接包含的常见图片文件，排序保证批次与缓存指纹使用相同顺序。
std::vector<std::string> calibrationImages(const std::string & dir);

// 分块计算完整文件的 FNV-1a 内容指纹；读取失败使用明确标记，不能冒充空文件。
std::string calibrationImageHash(const std::string & path);

// 将目录、文件名与内容纳入缓存键；长度前缀避免名称拼接产生歧义。
std::string calibrationDatasetFingerprint(const std::string & dir);

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// 在上游分配缓冲、预热前检查分割引擎合同；纯值校验不依赖 TensorRT 或 CUDA。

// 描述引擎枚举顺序中的真实张量；本校验不分配 GPU 缓冲，也不会重新排序输出以猜测布局。
// ---- 生产引擎的输入与分割输出布局 ----
struct SegEngineTensor
{
  std::string name;
  bool input{false};
  bool float32{false};
  bool linear_device{false};
  std::vector<std::int64_t> shape;
};

enum class SegOutputLayout {EndToEnd, Legacy};

// 类别表约束 legacy 的通道数；显式布局只能来自可信的导出元数据或调用方合同。
// 两种解释仍同时成立时拒绝猜测，避免将通道排列静默当成逐检测行读取。
bool resolveSegOutputLayout(
  const std::vector<std::int64_t> & detections,
  const std::vector<std::int64_t> & prototypes, std::size_t class_count,
  SegOutputLayout & layout, std::string & error,
  std::optional<SegOutputLayout> declared = std::nullopt);

// 构建前和加载后共享输入合同；此处不要求网络阶段尚未确定的存储格式。
bool validateSegInputTensors(
  const std::vector<SegEngineTensor> & inputs, std::string & error);

bool validateSegEngineTensors(
  const std::vector<SegEngineTensor> & tensors, std::string & error,
  SegOutputLayout * resolved_layout = nullptr, std::size_t class_count = 0);

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0

// 只读取调用方提供的线程安全停止谓词；一旦取消便锁存，不因后续返回值变化重新开始工作。
// 谓词异常也视为取消，避免异常逃出 TensorRT 的 noexcept 回调；无谓词时保持原行为。
// ---- 协作取消：只在调用方检查点响应 ----
class BuildCancellation {
public:
  using Predicate = std::function<bool()>;
  explicit BuildCancellation(Predicate predicate = {}) : predicate_(std::move(predicate)) {}

  bool requested() const noexcept;

private:
  Predicate predicate_;
  mutable std::atomic<bool> cancelled_{false};
};

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// 缓存身份、精确旧名候选和验证后的文件迁移；不依赖 TensorRT、CUDA 或 ROS。

// ---- 离线引擎缓存：模型内容、运行环境与构建条件共同确定身份 ----
namespace EngineCache
{
// modified 仅复现历史键；稳定键依赖完整内容身份及环境/构建条件，不因同步改动 mtime 而重建。
struct Identity
{
  std::string filename, content_hash, precision, gpu, tensor_rt, builder_revision, calibration;
  std::uintmax_t bytes{0};
  std::uint64_t modified{0};
  int dla_core{-1}, optimization_level{5};
  std::size_t workspace_mb{1024};
};

// 保留历史缓存名使用的 FNV 初值；完整文件内容指纹仍由 calibrationImageHash 计算。
std::string hash(const std::string & value);

namespace detail
{
}  // namespace detail

bool readEmbeddedOnnx(
  const std::filesystem::path & model, std::uintmax_t bytes,
  std::string & content_hash, std::string & error);

bool readModel(const std::filesystem::path & model, Identity & identity, std::string & error);

// legacy=true 精确复现 dual-v4 的字段顺序和 mtime 转换；稳定身份只移除 mtime。
std::string key(const Identity & identity, bool legacy = false);

struct Paths
{
  std::filesystem::path engine, failed, timing, calibration;
};

Paths paths(const std::filesystem::path & directory, const Identity & identity, bool legacy);

std::filesystem::path normalized(const std::filesystem::path & value);

std::filesystem::path directory(
  const std::filesystem::path & model, const std::filesystem::path & explicit_directory = {});

// 同时兼容符号链接的逻辑名字/父目录和真实名字/父目录；只产生精确身份对应的文件名。
std::vector<Paths> legacyCandidates(
  const std::filesystem::path & logical_model, const std::filesystem::path & real_model,
  const std::filesystem::path & destination, const Identity & logical, const Identity & real);

enum class Promotion {Missing, Rejected, Reused, Copied, Cancelled, Failed};
struct PromotionResult
{
  Promotion status;
  std::string error;
};
using Verifier = std::function<bool(const std::filesystem::path &, std::string &)>;

// 先验证旧文件，再复制到新目录临时文件并复核，最后改名；保留旧缓存，失败绝不触发重新构建。
PromotionResult promote(
  const std::filesystem::path & source, const std::filesystem::path & destination,
  const Verifier & verify, const std::function<bool()> & cancelled = {});
}  // namespace EngineCache


// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// 从 ONNX 构建本机 TensorRT 引擎，管理缓存键、反序列化检查和 INT8→FP16 回退。
// 引擎缓存绑定运行时 TensorRT/GPU 与构建条件；计时缓存仅供构建器复用，不代表模型验证通过。
// 构建/缓存迁移仅用于独立离线开发入口，生产 DetectNode 不调用 resolveEngine。
// 生产只复用明确的既有引擎；本模块不执行逐帧推理，也不替运行节点选择降级后端。

#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>

// 构建输入和缓存条件；增加影响引擎内容的选项时必须同步缓存键。
struct BuildOptions
{
  /// "fp32"、"fp16" 或 "int8"。仅在确实需要构建 engine 时才会用到。
  std::string precision{"fp16"};

  /// calibration 图像所在目录，使用 "int8" 时必需。
  std::string int8_calib_dir;

  /// engine 与计时缓存的写出位置；为空时使用 weakly_canonical(ONNX) 的真实父目录。
  std::string cache_dir;

  /// TensorRT tactic workspace 池大小，单位为 MiB。它不是 engine 的内存占用。
  std::size_t workspace_mb{1024};

  /// TensorRT 10.x 构建优化级别为 0..5；更高等级通常增加构建时间，实际推理性能须测量。
  int builder_optimization_level{5};

  /// -1 使用 GPU；非负值选择 DLA 核心，是否可用由当前设备能力检查决定。
  int dla_core{-1};

  /// 持久化并复用 tactic timing cache。
  bool use_timing_cache{true};

  /// 可选线程安全停止谓词；true 或异常请求协作取消，不参与缓存身份。
  /// 只能在构建检查点/优化步骤边界响应，不能保证当前步骤或 CUDA 同步的退出时间。
  std::function<bool()> cancel_requested;
};

// 解析/构建结果；失败时路径为空，实际精度与回退标志供上层日志明确说明。
struct BuildResult
{
  std::filesystem::path engine_path;
  /// 回退被触发时，可能与 BuildOptions::precision 不同。
  std::string precision_used;
  bool fallback_applied{false};
  /// 取消与构建失败分开：取消不触发 FP16 回退，也不写 INT8 失败标记。
  bool cancelled{false};
  /// 成功时为空，否则说明 engine 为何不可用。
  std::string error;
};

/// 对 ".trt" / ".engine" 返回 true（不区分大小写）。
bool isEnginePath(const std::string & path);

/// 运行时加载的 TensorRT 库版本（来自 getInferLibVersion()），例如 "10.x.y"。
std::string runtimeTensorRTVersion();

/// 编译期 TensorRT 头文件版本（NV_TENSORRT_MAJOR/MINOR/PATCH）。
std::string compiledTensorRTVersion();

/// 当前 CUDA 设备的签名："<GPU 型号>/sm<major><minor>"；取不到时返回 "unknown-gpu"。
std::string deviceSignature();

/// 检查运行时 TensorRT 库与编译期头文件是否兼容：主版本不同返回 false 并填写 error；
/// 次版本/补丁号不同只打印警告（engine 是否可用由 verifyEngineFile 的试反序列化决定）。
bool checkTensorRTRuntime(std::string & error);

/// 用当前 TensorRT 反序列化并验收 FLOAT、LINEAR 设备存储、静态 NCHW 和输出布局。
/// 不支持的合同在上游分配缓冲和预热前拒绝；error 包含上下文与检查原因，反序列化失败另保留 TensorRT 报错。
bool verifyEngineFile(const std::filesystem::path & engine_path, std::string & error);

/// 生产加载额外使用真实类别表数量消除布局歧义；无法唯一确定时在分配/预热前拒绝。
bool verifyEngineFileForClasses(
  const std::filesystem::path & engine_path, std::string & error, std::size_t class_count);

/// 缓存键包含 GPU/SM、运行时 TensorRT、精度/DLA/构建选项、ONNX 文件元信息和标定集指纹。
/// 只接受内嵌权重的二进制 ONNX（IR 1..10）；外置数据在查缓存前拒绝。
/// 身份包含真实文件名、大小与完整内容指纹；mtime 不参与稳定键，仅用于精确查找旧缓存。
std::string cacheKey(const std::string & onnx_path, const BuildOptions & options);

/// 离线开发接口：把 `model_path` 解析为一个可加载的 engine；可写缓存，不能接入生产逐帧/启动路径。
///
/// ".trt"/".engine" 路径先经 verifyEngineFile 校验，通过才原样返回。".onnx" 路径会
/// 先验收稳定缓存，再精确验收旧身份缓存并保留原文件迁移；无有效匹配缓存时才
/// 构建；若 INT8 构建失败，则 *再以 FP16 构建一次*，并写下 "<stem>.<key>.int8-failed"
/// 标记：输入不变时下次启动直接走 FP16，不再重复耗时的 INT8 构建（删掉标记即可重试）。
BuildResult resolveEngine(const std::string & model_path, const BuildOptions & options);
