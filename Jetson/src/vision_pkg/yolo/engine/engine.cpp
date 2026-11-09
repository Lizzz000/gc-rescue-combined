// 阅读：TensorRT 引擎的构建、文件验收和精度回退。
// 缓存身份和内嵌 ONNX 读取在 engine_cache.cpp。
// 逐帧推理不经过这里。生产启动只加载已经建好的引擎。
#include "yolo/engine/engine.hpp"
#include "yolo/tensorrt/tensorrt.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <utility>

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <NvOnnxParser.h>

constexpr const char * kBuilderRevision = "dual-v4";

std::vector<char> readBinary(const std::filesystem::path & path);

class BuildLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stdout, "[engine][TRT] %s\n", msg ? msg : "(null)");
    }
  }
};

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

// 新精度名要同时进这个函数、buildOnce() 的 flag、precisionAttempts() 和 cacheKey()。漏一处就会把不同精度写成同一个缓存。
/// 把用户传入的精度字符串规范化；无法识别时返回 ""。
std::string normalisePrecision(const std::string & raw)
{
  const std::string p = lower(raw);
  if (p == "fp16" || p == "half") {return "fp16";}
  if (p == "int8" || p == "int8_entropy") {return "int8";}
  if (p == "fp32" || p == "float32" || p == "none" || p.empty()) {return "fp32";}
  return "";
}

/// 当前设备的签名，例如 "Orin/sm87"（型号字符串以驱动报告为准）。取不到时返回 "unknown-gpu"。
std::string deviceSignature()
{
  int device = 0;
  if (cudaGetDevice(&device) != cudaSuccess) {return "unknown-gpu";}
  cudaDeviceProp prop{};
  if (cudaGetDeviceProperties(&prop, device) != cudaSuccess) {return "unknown-gpu";}
  return std::string(prop.name) + "/sm" + std::to_string(prop.major) + std::to_string(prop.minor);
}

#include <sstream>

// ---- 原 engine.hpp 的 inline 实现（v3 拆分移入，声明仍在 engine.hpp） ----
namespace vision {
std::size_t parseBuildInteger(
  std::string_view text, std::size_t minimum, std::size_t maximum, const char * name)
{
  if (text.empty()) {throw std::invalid_argument(std::string(name) + " must not be empty");}
  std::size_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value, 10);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
    value < minimum || value > maximum)
  {
    throw std::invalid_argument(std::string(name) + " must be a decimal integer in " +
      std::to_string(minimum) + ".." + std::to_string(maximum));
  }
  return value;
}

bool writeBinaryPayload(std::ostream & stream, const void * data, std::size_t size)
{
  if (!data || size == 0 ||
    size > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()))
  {return false;}
  stream.write(static_cast<const char *>(data), static_cast<std::streamsize>(size));
  stream.flush();
  return static_cast<bool>(stream);
}

bool writeBinaryComplete(
  const std::filesystem::path & path, const void * data, std::size_t size)
{
  if (!data || size == 0 ||
    size > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()))
  {return false;}
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {return false;}
  const bool written = writeBinaryPayload(stream, data, size);
  stream.close();
  return written && !stream.fail();
}
}  // namespace vision

std::vector<std::string> precisionAttempts(const std::string & requested)
{
  if (requested != "fp32" && requested != "fp16" && requested != "int8") {
    return {};
  }
  std::vector<std::string> attempts{requested};
  if (requested == "int8") {attempts.push_back("fp16");}
  return attempts;
}

std::vector<std::string> calibrationImages(const std::string & dir)
{
  std::vector<std::string> images;
  std::error_code error;
  std::filesystem::directory_iterator entry(std::filesystem::path(dir), error);
  const std::filesystem::directory_iterator end;
  while (!error && entry != end) {
    std::error_code file_error;
    if (entry->is_regular_file(file_error)) {
      std::string extension = entry->path().extension().string();
      std::transform(extension.begin(), extension.end(), extension.begin(),
        [](unsigned char c) {return static_cast<char>(std::tolower(c));});
      if (extension == ".jpg" || extension == ".jpeg" || extension == ".png" ||
        extension == ".bmp" || extension == ".webp")
      {
        images.push_back(entry->path().string());
      }
    }
    entry.increment(error);
  }
  std::sort(images.begin(), images.end());
  return images;
}

std::string calibrationImageHash(const std::string & path)
{
  std::ifstream image(path, std::ios::binary);
  if (!image) {return "unreadable";}
  std::uint64_t hash = 14695981039346656037ULL;
  std::array<char, 65536> buffer;
  while (image) {
    image.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    for (std::streamsize i = 0; i < image.gcount(); ++i) {
      hash ^= static_cast<unsigned char>(buffer[static_cast<std::size_t>(i)]);
      hash *= 1099511628211ULL;
    }
  }
  if (!image.eof()) {return "unreadable";}
  std::ostringstream digest;
  digest << std::hex << hash;
  return digest.str();
}

std::string calibrationDatasetFingerprint(const std::string & dir)
{
  if (dir.empty()) {return "none";}
  const auto images = calibrationImages(dir);
  std::ostringstream fingerprint;
  fingerprint << dir.size() << ':' << dir << '#' << images.size();
  for (const auto & image : images) {
    const std::string name = std::filesystem::path(image).filename().string();
    fingerprint << '|' << name.size() << ':' << name << ':' << calibrationImageHash(image);
  }
  return fingerprint.str();
}

bool resolveSegOutputLayout(
  const std::vector<std::int64_t> & detections,
  const std::vector<std::int64_t> & prototypes, std::size_t class_count,
  SegOutputLayout & layout, std::string & error,
  std::optional<SegOutputLayout> declared)
{
  error.clear();
  const auto fail = [&](const char * reason) {error = reason; return false;};
  if (detections.size() != 3 || prototypes.size() != 4 ||
    detections[0] != 1 || prototypes[0] != 1 ||
    class_count > static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {return fail("segmentation layout requires batch-one shapes and an addressable class count");}
  for (const auto * shape : {&detections, &prototypes}) {
    for (const auto axis : *shape) {
      if (axis <= 0 || axis > std::numeric_limits<int>::max()) {
        return fail("segmentation layout requires positive static dimensions");
      }
    }
  }
  const bool end_to_end = detections[2] == 6 + prototypes[1];
  const bool legacy = prototypes[1] == 32 && detections[1] > 4 + prototypes[1] &&
    (class_count == 0 || static_cast<std::size_t>(detections[1] - 4 - prototypes[1]) == class_count);
  if (declared) {
    if ((*declared != SegOutputLayout::EndToEnd && *declared != SegOutputLayout::Legacy) ||
      (*declared == SegOutputLayout::EndToEnd && !end_to_end) ||
      (*declared == SegOutputLayout::Legacy && !legacy))
    {return fail("declared segmentation layout conflicts with tensor shapes or class count");}
    layout = *declared;
    return true;
  }
  if (end_to_end && legacy) {
    return fail("ambiguous segmentation layout; supply trustworthy class/export metadata");
  }
  if (!end_to_end && !legacy) {
    return fail("segmentation layout conflicts with tensor shapes or class count");
  }
  layout = end_to_end ? SegOutputLayout::EndToEnd : SegOutputLayout::Legacy;
  return true;
}

bool validateSegInputTensors(
  const std::vector<SegEngineTensor> & inputs, std::string & error)
{
  error.clear();
  const auto fail = [&](const char * reason) {error = reason; return false;};
  if (inputs.size() != 1 || !inputs.front().input) {
    return fail("segmentation requires exactly one input tensor");
  }
  const auto & input = inputs.front();
  if (!input.float32) {return fail("segmentation input must use FLOAT");}
  if (input.shape.size() != 4 || input.shape[0] != 1 || input.shape[1] != 3) {
    return fail("segmentation input must be static [1,3,H,W] NCHW");
  }
  std::size_t elements = 1;
  for (const auto axis : input.shape) {
    if (axis <= 0 || axis > std::numeric_limits<int>::max() ||
      elements > std::numeric_limits<std::size_t>::max() / sizeof(float) /
      static_cast<std::size_t>(axis))
    {
      return fail("segmentation input requires positive static dimensions without overflow");
    }
    elements *= static_cast<std::size_t>(axis);
  }
  // CUDA 及 CPU 标定的 plane/通道索引均使用 int，先验收再窄化维度或分配缓冲。
  if (input.shape[2] > std::numeric_limits<int>::max() / 3 / input.shape[3]) {
    return fail("segmentation input exceeds the preprocessor's signed pixel index range");
  }
  return true;
}

bool validateSegEngineTensors(
  const std::vector<SegEngineTensor> & tensors, std::string & error,
  SegOutputLayout * resolved_layout, std::size_t class_count)
{
  error.clear();
  const auto fail = [&](const std::string & reason) {
    error = reason;
    return false;
  };
  if (tensors.size() != 3) {
    return fail("segmentation requires one input and two outputs");
  }
  const SegEngineTensor * input = nullptr;
  std::vector<const SegEngineTensor *> outputs;
  for (const auto & tensor : tensors) {
    if (!tensor.float32 || !tensor.linear_device) {
      return fail("tensor '" + tensor.name + "' must use FLOAT and LINEAR device storage; "
        "the GPU preprocessing and decoder access float buffers");
    }
    if (tensor.shape.empty()) {return fail("tensor '" + tensor.name + "' has no dimensions");}
    std::size_t elements = 1;
    for (const auto axis : tensor.shape) {
      if (axis <= 0 || axis > std::numeric_limits<int>::max() ||
        elements > std::numeric_limits<std::size_t>::max() / sizeof(float) /
        static_cast<std::size_t>(axis))
      {
        return fail("tensor '" + tensor.name +
          "' requires positive static dimensions without overflow");
      }
      elements *= static_cast<std::size_t>(axis);
    }
    if (tensor.input) {
      if (input != nullptr) {return fail("segmentation supports only one input");}
      input = &tensor;
    } else {
      outputs.push_back(&tensor);
    }
  }
  if (!input || outputs.size() != 2) {
    return fail("segmentation requires one input and two outputs");
  }
  if (!validateSegInputTensors({*input}, error)) {return false;}
  // 上游按引擎枚举顺序收集输出，本地也按首个框张量、第二个原型张量读取，不能静默重排。
  const auto & detection_shape = outputs[0]->shape;
  const auto & prototype_shape = outputs[1]->shape;
  if (prototype_shape.size() != 4 || prototype_shape[0] != 1 ||
    prototype_shape[2] * prototype_shape[3] > std::numeric_limits<int>::max())
  {
    return fail("second output must be [1,P,H,W] prototypes with an addressable plane");
  }
  if (detection_shape.size() != 3 || detection_shape[0] != 1) {
    return fail("first output must be a batch-one detection tensor");
  }
  const bool end_to_end = detection_shape[2] == 6 + prototype_shape[1];
  const bool legacy = prototype_shape[1] == 32 && detection_shape[1] > 4 + prototype_shape[1];
  if (!end_to_end && !legacy) {
    return fail("unsupported segmentation layout: expected [1,N,6+P], or legacy "
      "[1,4+C+32,N] with 32 prototypes");
  }
  // 不解码的离线文件验收只检查存储/结构；生产另带类别身份取得唯一布局。
  if (resolved_layout && !resolveSegOutputLayout(
      detection_shape, prototype_shape, class_count, *resolved_layout, error)) {return false;}
  return true;
}

// 缓存身份、内嵌 ONNX 读取和文件迁移在 engine_cache.cpp。

bool BuildCancellation::requested() const noexcept
{
  if (cancelled_.load()) {return true;}
  if (!predicate_) {return false;}
  try {
    if (!predicate_()) {return cancelled_.load();}
  } catch (...) {
    // 回调失败后不再继续构建或把失败写成 INT8 不可用标记。
  }
  cancelled_.store(true);
  return true;
}


namespace
{
struct TrtVersion
{
  int major{0};
  int minor{0};
  int patch{0};
};

/// 解码 getInferLibVersion() 的整数编码。TensorRT 10 头文件（NvInferRuntimeBase.h）写明格式为
/// (MAJOR * 100 + MINOR) * 100 + PATCH；本集成在 CMake 里已限定 TensorRT 10.x。
TrtVersion decodeTrtVersion(std::int64_t value)
{
  TrtVersion v;
  v.major = static_cast<int>(value / 10000);
  v.minor = static_cast<int>((value / 100) % 100);
  v.patch = static_cast<int>(value % 100);
  return v;
}

std::string toString(const TrtVersion & v)
{
  return std::to_string(v.major) + "." + std::to_string(v.minor) + "." + std::to_string(v.patch);
}

TrtVersion runtimeVersion() {return decodeTrtVersion(getInferLibVersion());}

TrtVersion compiledVersion()
{
  return TrtVersion{NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH};
}

/// 反序列化时收集 TensorRT 的错误文本，用于拼进最终报错；警告照常打印。
class CapturingLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    // TensorRT 可从内部线程回调；错误路径使用有界存储，不在 noexcept 中分配内存。
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      if (severity <= Severity::kERROR) {
        if (used_ != 0) {append(" | ");}
        append(msg ? msg : "(null)");
      } else if (severity == Severity::kWARNING) {
        std::fprintf(stdout, "[engine][TRT] %s\n", msg ? msg : "(null)");
      }
    } catch (...) {
      // 连锁日志失败不能越过 TensorRT 的 noexcept 边界或终止错误处理。
    }
  }
  std::string messages() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::string(messages_.data(), used_);
  }

private:
  void append(const char * text) noexcept
  {
    while (*text && used_ < messages_.size()) {messages_[used_++] = *text++;}
  }
  mutable std::mutex mutex_;
  std::array<char, 4096> messages_{};
  std::size_t used_{0};
};
}  // namespace

std::string runtimeTensorRTVersion() {return toString(runtimeVersion());}

std::string compiledTensorRTVersion() {return toString(compiledVersion());}

bool checkTensorRTRuntime(std::string & error)
{
  const TrtVersion runtime = runtimeVersion();
  const TrtVersion compiled = compiledVersion();
  if (runtime.major != compiled.major) {
    error = "TensorRT runtime library " + toString(runtime) + " does not match the headers this "
      "node was compiled against (" + toString(compiled) + "). Rebuild vision_pkg inside the "
      "environment that runs it (same host or same container image).";
    return false;
  }
  if (runtime.minor != compiled.minor || runtime.patch != compiled.patch) {
    std::cout << "[engine] warning: TensorRT runtime " << toString(runtime)
              << " differs from the compile-time headers " << toString(compiled)
              << "; engines are keyed and verified against the runtime version" << std::endl;
  }
  return true;
}

static bool verifyEngineFileImpl(
  const std::filesystem::path & engine_path, std::string & error,
  const std::size_t * class_count)
{
  const std::string context = " (runtime TensorRT " + runtimeTensorRTVersion() + ", GPU " +
    deviceSignature() + ")";
  std::error_code ec;
  if (!std::filesystem::is_regular_file(engine_path, ec)) {
    error = "engine file not found: " + engine_path.string() + context;
    return false;
  }
  const std::vector<char> blob = readBinary(engine_path);
  if (blob.empty()) {
    error = "engine file is empty or unreadable: " + engine_path.string() + context;
    return false;
  }
  CapturingLogger logger;
  std::unique_ptr<nvinfer1::IRuntime> runtime(nvinfer1::createInferRuntime(logger));
  if (!runtime) {
    error = "failed to create a TensorRT runtime" + context;
    return false;
  }
  // 声明顺序保证 engine 先于 runtime 析构。
  std::unique_ptr<nvinfer1::ICudaEngine> engine(
    runtime->deserializeCudaEngine(blob.data(), blob.size()));
  if (!engine) {
    // 本函数只返回失败，不构建或回退；生产调用方仍要求预构建引擎，离线构建由独立入口处理。
    error = "cannot deserialize TensorRT engine " + engine_path.string() + context +
      ". A serialized engine only loads with the TensorRT version and GPU it was built for. "
      "Select an existing compatible .trt/.engine built for this runtime environment. "
      "Production detect_node accepts prebuilt engines only and never builds from ONNX; "
      "any required engine build belongs to a separately requested offline workflow.";
    const auto messages = logger.messages();
    if (!messages.empty()) {error += " TensorRT said: " + messages;}
    return false;
  }
  // 固定上游按实际 dtype 分配，却用 float 指针做 GPU 预处理和输出读取。
  // 这里先验收布局，避免 HALF/动态/额外输入在构造期间进入预热造成越界。
  std::vector<SegEngineTensor> tensors;
  for (int i = 0; i < engine->getNbIOTensors(); ++i) {
    const char * name = engine->getIOTensorName(i);
    if (name == nullptr || engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kNONE) {
      error = "invalid engine I/O tensor" + context;
      return false;
    }
    SegEngineTensor tensor;
    tensor.name = name;
    tensor.input = engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
    tensor.float32 = engine->getTensorDataType(name) == nvinfer1::DataType::kFLOAT;
    tensor.linear_device = engine->getTensorFormat(name) == nvinfer1::TensorFormat::kLINEAR &&
      engine->getTensorLocation(name) == nvinfer1::TensorLocation::kDEVICE;
    const auto dims = engine->getTensorShape(name);
    for (int axis = 0; axis < dims.nbDims; ++axis) {
      tensor.shape.push_back(dims.d[axis]);
    }
    tensors.push_back(std::move(tensor));
  }
  std::string contract_error;
  SegOutputLayout layout;
  if (!validateSegEngineTensors(tensors, contract_error,
      class_count ? &layout : nullptr, class_count ? *class_count : 0)) {
    error = "unsupported segmentation engine " + engine_path.string() + ": " +
      contract_error + context;
    return false;
  }
  error.clear();
  return true;
}

bool verifyEngineFile(const std::filesystem::path & engine_path, std::string & error)
{
  return verifyEngineFileImpl(engine_path, error, nullptr);
}

bool verifyEngineFileForClasses(
  const std::filesystem::path & engine_path, std::string & error, std::size_t class_count)
{
  return verifyEngineFileImpl(engine_path, error, &class_count);
}

/// 标定集身份包含目录、稳定排序的候选图片名与完整内容哈希；候选范围与标定器共用。
/// 等字节数替换图片也会换键；无法解码的候选保守参与指纹，实际取批次时跳过。
// 清单和内容哈希只改 calibrationDatasetFingerprint()。采样语义变了就递增 kBuilderRevision。
std::string calibrationFingerprint(const std::string & dir)
{
  return calibrationDatasetFingerprint(dir);
}

std::filesystem::path timingCachePath(
  const BuildOptions & options,
  const std::filesystem::path & engine_path)
{
  if (!options.use_timing_cache) {return std::filesystem::path();}
  std::filesystem::path dir = options.cache_dir.empty() ? engine_path.parent_path() :
    std::filesystem::path(options.cache_dir);
  if (dir.empty()) {dir = std::filesystem::current_path();}
  return dir / (engine_path.stem().string() + ".timing.cache");
}

std::vector<char> readBinary(const std::filesystem::path & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {return std::vector<char>();}
  return std::vector<char>(std::istreambuf_iterator<char>(f),
                           std::istreambuf_iterator<char>());
}

bool writeBinary(const std::filesystem::path & path, const void * data, std::size_t size)
{
  return vision::writeBinaryComplete(path, data, size);
}

/// 按单一精度做一次构建尝试。这里没有任何回退逻辑。
bool buildOnce(
  const std::filesystem::path & onnx, const std::filesystem::path & out,
  const BuildOptions & options,
  const std::string & precision, std::string & error_out,
  BuildCancellation & cancellation)
{
  auto cancelled = [&] {
      if (!cancellation.requested()) {return false;}
      error_out = "TensorRT engine build cancelled";
      return true;
    };
  if (cancelled()) {return false;}
  // 声明在 builder 之前，保证 TensorRT 对象析构时 monitor 及其停止状态仍然有效。
  BuildProgressMonitor monitor(cancellation);
  BuildLogger logger;

  std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger));
  if (!builder) {
    error_out = "failed to create the TensorRT builder";
    return false;
  }

  const int dla_cores = builder->getNbDLACores();
  if (options.dla_core >= 0 && dla_cores <= 0) {
    std::cout << "[engine] dla_core=" << options.dla_core
              << " requested but this module reports no DLA cores "
      "(Jetson Orin Nano has none) - building for the GPU"
              << std::endl;
  }

  std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(0));
  if (!network) {
    error_out = "failed to create the TensorRT network";
    return false;
  }

  std::unique_ptr<nvonnxparser::IParser> parser(
    nvonnxparser::createParser(*network, logger));
  if (!parser) {
    error_out = "failed to create the ONNX parser";
    return false;
  }

  if (cancelled()) {return false;}
  if (!parser->parseFromFile(onnx.string().c_str(),
                             static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
  {
    std::ostringstream ss;
    ss << "failed to parse " << onnx;
    for (int i = 0; i < parser->getNbErrors(); ++i) {
      ss << "; " << parser->getError(i)->desc();
    }
    error_out = ss.str();
    return false;
  }

  // 在配置、标定缓冲分配及 buildSerializedNetwork 前验收；最终引擎验收仍保留。
  // 所有精度共用生产输入合同，INT8 失败回退也不能绕过布局拒绝。
  std::vector<SegEngineTensor> inputs;
  for (int i = 0; i < network->getNbInputs(); ++i) {
    const auto * input = network->getInput(i);
    if (!input) {error_out = "parsed network contains a missing input tensor"; return false;}
    SegEngineTensor tensor;
    tensor.name = input->getName() ? input->getName() : "";
    tensor.input = true;
    tensor.float32 = input->getType() == nvinfer1::DataType::kFLOAT;
    const auto dimensions = input->getDimensions();
    for (int axis = 0; axis < dimensions.nbDims; ++axis) {
      tensor.shape.push_back(dimensions.d[axis]);
    }
    inputs.push_back(std::move(tensor));
  }
  if (!validateSegInputTensors(inputs, error_out)) {return false;}

  if (cancelled()) {return false;}
  std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
  if (!config) {
    error_out = "failed to create the TensorRT builder config";
    return false;
  }

  if (options.cancel_requested) {config->setProgressMonitor(&monitor);}
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                             options.workspace_mb * vision::kBytesPerMiB);
  config->setBuilderOptimizationLevel(options.builder_optimization_level);

  if (options.dla_core >= 0 && dla_cores > 0) {
    config->setFlag(nvinfer1::BuilderFlag::kGPU_FALLBACK);
    config->setDefaultDeviceType(nvinfer1::DeviceType::kDLA);
    config->setDLACore(options.dla_core);
  }

  // 复用并持久化 tactic timing cache；实际构建时间由目标环境记录，不能从是否命中直接推断。
  std::unique_ptr<nvinfer1::ITimingCache> timing_cache;
  const std::filesystem::path timing_path = timingCachePath(options, out);
  if (options.use_timing_cache && !timing_path.empty()) {
    const std::vector<char> blob = readBinary(timing_path);
    // TensorRT 10.x 由 IBuilderConfig 创建计时缓存，再绑定到同一份构建配置。
    timing_cache.reset(config->createTimingCache(blob.data(), blob.size()));
    if (timing_cache) {
      if (!blob.empty()) {
        std::cout << "[engine] timing cache hit: " << timing_path << std::endl;
      }
      config->setTimingCache(*timing_cache, false);
    }
  }

  // 注意：较新的 TensorRT 10.x 头文件已把 kFP16/kINT8 标志和隐式量化标定器标为 deprecated
  // （推荐强类型网络 + 显式量化），目前仍可用；编译时会出现 -Wdeprecated-declarations 警告。
  std::unique_ptr<ImageFolderCalibrator> calibrator;
  if (precision == "int8") {
    if (options.int8_calib_dir.empty()) {
      error_out = "precision is int8 but int8_calib_dir is empty";
      return false;
    }
    const auto & input_shape = inputs.front().shape;
    const cv::Size input_size(static_cast<int>(input_shape[3]), static_cast<int>(input_shape[2]));

    calibrator = std::make_unique<ImageFolderCalibrator>(
      options.int8_calib_dir, input_size, out.string() + ".calib",
      network->getInput(0)->getName(), &cancellation);
    if (cancelled()) {return false;}
    if (calibrator->imageCount() == 0) {
      error_out = "no calibration images in " + options.int8_calib_dir;
      return false;
    }
    std::cout << "[engine] INT8 calibration over " << calibrator->imageCount()
              << " images at " << input_size.width << "x" << input_size.height << std::endl;

    config->setFlag(nvinfer1::BuilderFlag::kINT8);
    config->setInt8Calibrator(calibrator.get());
  } else if (precision == "fp16") {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  if (cancelled()) {return false;}
  std::cout << "[engine] buildSerializedNetwork " << precision << " begin" << std::endl;
  std::unique_ptr<nvinfer1::IHostMemory> plan(
    builder->buildSerializedNetwork(*network, *config));
  // 取消可能已由 monitor 观察到；即使 TensorRT 返回计划，也不把它提交为新缓存。
  if (cancelled()) {return false;}
  // getBatch(false) 既可能表示数据耗尽也可能是资源失败；失败不能提交为成功 INT8 引擎。
  if (calibrator && !calibrator->lastError().empty()) {
    error_out = "INT8 calibration failed: " + calibrator->lastError();
    return false;
  }
  if (!plan) {
    error_out = "TensorRT engine build failed (see the [engine][TRT] lines above)";
    return false;
  }

  if (options.use_timing_cache && timing_cache && !timing_path.empty()) {
    std::unique_ptr<nvinfer1::IHostMemory> serialized(timing_cache->serialize());
    if (cancelled()) {return false;}
    if (serialized) {
      writeBinary(timing_path, serialized->data(), serialized->size());
    }
  }

  // 写入和关闭均成功，并通过真实 TensorRT 文件验收后才提交正式路径。
  return vision::commitVerifiedBinary(out, plan->data(), plan->size(), verifyEngineFile,
    [&] {return cancellation.requested();}, error_out);
}


bool isEnginePath(const std::string & path)
{
  const std::string ext = lower(std::filesystem::path(path).extension().string());
  return ext == ".trt" || ext == ".engine";
}

// 稳定键不受同步工具改动 mtime 影响；legacy 键仅用于精确查找已完成的旧构建。
bool cacheIdentity(
  const std::filesystem::path & model, const BuildOptions & options,
  EngineCache::Identity & identity, std::string & error)
{
  if (!EngineCache::readModel(model, identity, error)) {return false;}
  identity.precision = normalisePrecision(options.precision);
  identity.dla_core = options.dla_core;
  identity.gpu = deviceSignature();
  identity.tensor_rt = runtimeTensorRTVersion();
  identity.builder_revision = kBuilderRevision;
  identity.workspace_mb = options.workspace_mb;
  identity.optimization_level = options.builder_optimization_level;
  // 标定目录字符串沿用旧算法，不能先 canonicalize 后使已有 INT8 身份失配。
  identity.calibration = calibrationFingerprint(options.int8_calib_dir);
  return true;
}

std::string cacheKey(const std::string & onnx_path, const BuildOptions & options)
{
  EngineCache::Identity identity;
  std::string error;
  if (!cacheIdentity(EngineCache::normalized(onnx_path), options, identity, error)) {
    throw std::runtime_error("cannot create engine cache identity: " + error);
  }
  return EngineCache::key(identity);
}


// 显式引擎路径只验收后原样返回；仅显式 ONNX 开发请求才进入身份匹配、迁移和必要构建。
BuildResult resolveEngine(const std::string & model_path, const BuildOptions & options)
{
  BuildResult result;
  BuildCancellation cancellation(options.cancel_requested);
  auto cancelled = [&] {
      if (!cancellation.requested()) {return false;}
      result.cancelled = true;
      result.error = "TensorRT engine initialization cancelled";
      return true;
    };
  if (cancelled()) {return result;}
  if (options.workspace_mb == 0 || options.workspace_mb > vision::kMaxWorkspaceMiB ||
    options.builder_optimization_level < 0 || options.builder_optimization_level > 5)
  {
    result.error = "invalid builder options: optimization_level must be 0..5 and "
      "workspace_mb must be positive without overflowing its byte conversion";
    return result;
  }
  if (model_path.empty()) {result.error = "model_path is empty"; return result;}
  std::string runtime_error;
  if (!checkTensorRTRuntime(runtime_error)) {result.error = runtime_error; return result;}
  std::cout << "[engine] TensorRT runtime " << runtimeTensorRTVersion() << " (headers "
            << compiledTensorRTVersion() << "), GPU " << deviceSignature() << std::endl;
  if (isEnginePath(model_path)) {
    std::string error;
    const bool verified = verifyEngineFile(model_path, error);
    if (cancelled()) {return result;}
    if (!verified) {result.error = error; return result;}
    result.engine_path = model_path;
    result.precision_used = "prebuilt";
    return result;
  }
  if (lower(std::filesystem::path(model_path).extension().string()) != ".onnx") {
    result.error = "model_path must be a .onnx, .trt or .engine file, got " + model_path;
    return result;
  }
  std::error_code ec;
  if (!std::filesystem::is_regular_file(model_path, ec)) {
    result.error = "model not found: " + model_path;
    return result;
  }
  const std::filesystem::path logical_model(model_path);
  const auto real_model = std::filesystem::weakly_canonical(logical_model, ec);
  if (ec) {result.error = "cannot resolve ONNX path: " + ec.message(); return result;}
  // 未指定目录时使用 ONNX 解析后的真实父目录，符号链接不应把引擎写到临时安装目录。
  const auto directory = EngineCache::directory(real_model, options.cache_dir);
  std::filesystem::create_directories(directory, ec);
  if (ec) {result.error = "cannot create engine cache directory: " + ec.message(); return result;}
  const auto requested = normalisePrecision(options.precision);
  if (requested.empty()) {
    result.error = "unknown precision '" + options.precision + "' (use fp32, fp16 or int8)";
    return result;
  }
  const auto keep_file = [](const std::filesystem::path &, std::string & error) {
      error.clear(); return true;
    };
  const auto stop = [&] {return cancellation.requested();};
  std::string last_error;
  for (const auto & precision : precisionAttempts(requested)) {
    if (cancelled()) {return result;}
    BuildOptions attempt = options;
    attempt.precision = precision;
    EngineCache::Identity identity;
    if (!cacheIdentity(real_model, attempt, identity, result.error)) {return result;}
    if (cancelled()) {return result;}
    auto logical_identity = identity;
    logical_identity.filename = logical_model.filename().string();
    const auto current = EngineCache::paths(directory, identity, false);
    const auto legacy = EngineCache::legacyCandidates(
      logical_model, real_model, directory, logical_identity, identity);

    // 附属缓存仅从同模型/运行时/精度/标定身份的旧名迁移，不扫描任意 engine 或失败标记。
    const auto migrate_sidecars = [&](bool engine_available) {
        for (const auto & old : legacy) {
          std::vector<std::pair<std::filesystem::path, std::filesystem::path>> files{
            {old.calibration, current.calibration}};
          if (attempt.use_timing_cache) {files.emplace_back(old.timing, current.timing);}
          for (const auto & file : files) {
            const auto copied = EngineCache::promote(file.first, file.second, keep_file, stop);
            if (copied.status == EngineCache::Promotion::Cancelled) {return false;}
            if (copied.status == EngineCache::Promotion::Failed) {
              if (!engine_available) {result.error = copied.error; return false;}
              std::cerr << "[engine] optional cache migration failed: " << copied.error << std::endl;
            }
          }
        }
        return true;
      };
    const auto accept_engine = [&](const char * source) {
        std::cout << "[engine] " << source << ": " << current.engine << std::endl;
        result.engine_path = current.engine;
        result.precision_used = precision;
        result.fallback_applied = precision != requested;
        if (result.fallback_applied) {
          std::cout << "[engine] using FP16 because INT8 is unavailable: " << last_error << std::endl;
        }
      };

    if (std::filesystem::is_regular_file(current.engine, ec)) {
      std::string error;
      const bool verified = verifyEngineFile(current.engine, error);
      if (cancelled()) {return result;}
      if (verified) {
        if (!migrate_sidecars(true)) {cancelled(); return result;}
        if (cancelled()) {return result;}
        accept_engine("cache hit");
        return result;
      }
      const auto rejected = current.engine.string() + ".rejected";
      std::filesystem::rename(current.engine, rejected, ec);
      if (ec) {
        result.error = "cannot quarantine invalid cached engine: " + ec.message();
        return result;
      }
      std::cerr << "[engine] cached engine rejected: " << error << std::endl;
    }

    // 成功的旧 INT8 engine 必须先于失败标记处理，不能被早先一次失败覆盖。
    for (const auto & old : legacy) {
      const auto copied = EngineCache::promote(old.engine, current.engine, verifyEngineFile, stop);
      if (copied.status == EngineCache::Promotion::Cancelled) {cancelled(); return result;}
      if (copied.status == EngineCache::Promotion::Failed) {
        result.error = copied.error;
        return result;  // 存储错误不能变成一次耗时的新构建。
      }
      if (copied.status == EngineCache::Promotion::Rejected) {
        std::cerr << "[engine] legacy cached engine rejected: " << copied.error << std::endl;
        continue;
      }
      if (copied.status == EngineCache::Promotion::Copied ||
        copied.status == EngineCache::Promotion::Reused)
      {
        if (cancelled()) {return result;}
        if (!migrate_sidecars(true)) {cancelled(); return result;}
        if (cancelled()) {return result;}
        std::cout << "[engine] verified legacy cache: " << old.engine << std::endl;
        accept_engine("cache migrated");
        return result;
      }
    }

    if (precision == "int8") {
      auto failed = current.failed;
      bool known_failure = std::filesystem::is_regular_file(failed, ec);
      if (!known_failure) {
        for (const auto & old : legacy) {
          if (!std::filesystem::is_regular_file(old.failed, ec)) {continue;}
          const auto copied = EngineCache::promote(old.failed, current.failed, keep_file, stop);
          if (copied.status == EngineCache::Promotion::Cancelled) {cancelled(); return result;}
          // 复制失败仍保留旧标记的“不要重试 INT8”含义，读取旧文件即可。
          if (copied.status == EngineCache::Promotion::Failed) {
            std::cerr << "[engine] failure marker migration failed: " << copied.error << std::endl;
          }
          failed = old.failed;
          known_failure = true;
          break;
        }
      }
      if (known_failure) {
        if (!migrate_sidecars(true)) {cancelled(); return result;}
        if (cancelled()) {return result;}
        const auto reason = readBinary(failed);
        last_error = "an earlier INT8 build with the same inputs failed (" +
          std::string(reason.begin(), reason.end()) + "); remove matching failure markers to retry INT8";
        std::cerr << "[engine] skipping INT8: " << last_error << std::endl;
        continue;
      }
    }
    if (!migrate_sidecars(false)) {cancelled(); return result;}
    if (cancelled()) {return result;}
    std::cout << "[engine] preparing " << precision << " engine from " << real_model
              << " (offline build; this can take several minutes on a Jetson"
              << "; production startup only reuses existing engines)"
              << std::endl;
    std::string error;
    const bool built = buildOnce(real_model, current.engine, attempt, precision, error, cancellation);
    if (cancelled()) {return result;}
    if (built) {accept_engine("wrote"); return result;}
    last_error = error;
    std::cerr << "[engine] " << precision << " build failed: " << error << std::endl;
    if (cancelled()) {return result;}
    if (precision == "int8") {
      const auto reason = error.empty() ? std::string("unknown error") : error;
      writeBinary(current.failed, reason.data(), reason.size());
    }
  }
  result.error = last_error.empty() ? "engine build failed" : last_error;
  return result;
}
