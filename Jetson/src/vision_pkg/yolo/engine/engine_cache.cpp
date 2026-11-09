// 缓存身份、内嵌 ONNX 读取和验证后的文件迁移。不构建引擎。
// TensorRT 构建、精度回退和反序列化验收留在 engine.cpp。

#include "yolo/engine/engine.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace EngineCache {
std::string hash(const std::string & value)
{
  std::uint64_t digest = 1469598103934665603ULL;
  for (unsigned char byte : value) {
    digest ^= byte;
    digest *= 1099511628211ULL;
  }
  std::ostringstream text;
  text << std::hex << digest;
  return text.str();
}
}  // namespace EngineCache

namespace EngineCache {
namespace detail {
// nvonnxparser 不公开 ModelProto；本项目不引入第二份 protobuf/ONNX 运行时。
// 按 ONNX v1.17.0 的 onnx.proto（IR <= 10）遍历所有可包含 TensorProto 的路径。
// https://github.com/onnx/onnx/blob/v1.17.0/onnx/onnx.proto
// 这里只判定权重是否完全内嵌并防护 wire 边界，不替代 TensorRT 的语义解析。
// 未知字段/版本保守拒绝；raw_data、字符串和无张量类型元信息不按 protobuf 猜测。
class EmbeddedOnnxReader
{
  enum class Kind {None, Model, Graph, Node, Attribute, Tensor, Sparse, Training, Function};
  struct Field {unsigned wires{0}; Kind child{Kind::None};};
  static constexpr unsigned varint = 1U, fixed64 = 2U, bytes = 4U, fixed32 = 32U;

  static Field field(Kind kind, std::uint32_t id)
  {
    switch (kind) {
      case Kind::Model:
        switch (id) {
          case 1: case 5: return {varint};
          case 2: case 3: case 4: case 6: case 8: case 14: return {bytes};
          case 7: return {bytes, Kind::Graph};
          case 20: return {bytes, Kind::Training};
          case 25: return {bytes, Kind::Function};
        } break;
      case Kind::Graph:
        switch (id) {
          case 1: return {bytes, Kind::Node};
          case 5: return {bytes, Kind::Tensor};
          case 15: return {bytes, Kind::Sparse};
          case 2: case 10: case 11: case 12: case 13: case 14: case 16: return {bytes};
        } break;
      case Kind::Node:
        if (id == 5) {return {bytes, Kind::Attribute};}
        if (id >= 1 && id <= 9) {return {bytes};}
        break;
      case Kind::Attribute:
        switch (id) {
          case 5: case 10: return {bytes, Kind::Tensor};
          case 6: case 11: return {bytes, Kind::Graph};
          case 22: case 23: return {bytes, Kind::Sparse};
          case 1: case 4: case 9: case 13: case 14: case 15: case 21: return {bytes};
          case 2: return {fixed32};
          case 3: case 20: return {varint};
          case 7: return {bytes | fixed32};
          case 8: return {bytes | varint};
        } break;
      case Kind::Tensor:
        switch (id) {
          case 1: case 5: case 7: case 11: return {varint | bytes};
          case 2: case 14: return {varint};
          case 3: case 6: case 8: case 9: case 12: case 13: case 16: return {bytes};
          case 4: return {bytes | fixed32};
          case 10: return {bytes | fixed64};
        } break;
      case Kind::Sparse:
        if (id == 1 || id == 2) {return {bytes, Kind::Tensor};}
        if (id == 3) {return {varint | bytes};}
        break;
      case Kind::Training:
        if (id == 1 || id == 2) {return {bytes, Kind::Graph};}
        if (id == 3 || id == 4) {return {bytes};}
        break;
      case Kind::Function:
        if (id == 7) {return {bytes, Kind::Node};}
        if (id == 11) {return {bytes, Kind::Attribute};}
        if (id == 2 || id == 3) {return {varint};}  // IR < 8 的已废弃字段。
        if (id == 1 || (id >= 4 && id <= 14)) {return {bytes};}
        break;
      case Kind::None: break;
    }
    return {};
  }

public:
  EmbeddedOnnxReader(std::istream & stream, std::uintmax_t size, std::string & error)
  : stream_(stream), size_(size), error_(error) {}

  bool read(std::string & digest)
  {
    if (!message(Kind::Model, size_, 0)) {return false;}
    if (!has_version_ || !has_graph_ || stream_.peek() != std::char_traits<char>::eof() ||
      stream_.bad())
    {
      return fail("ONNX model is incomplete or changed while reading");
    }
    std::ostringstream text;
    text << std::hex << hash_;
    digest = text.str();
    error_.clear();
    return true;
  }

private:
  bool fail(const char * reason) {error_ = reason; return false;}
  void hashByte(unsigned char byte)
  {hash_ ^= byte; hash_ *= 1099511628211ULL;}

  bool integer(std::uintmax_t end, std::uint64_t & value)
  {
    value = 0;
    for (unsigned index = 0; index < 10; ++index) {
      if (position_ >= end) {return fail("truncated ONNX varint");}
      const int next = stream_.get();
      if (next == std::char_traits<char>::eof()) {return fail("truncated ONNX file");}
      ++position_;
      const auto byte = static_cast<unsigned char>(next);
      hashByte(byte);
      if (index == 9 && byte > 1) {return fail("overflowing ONNX varint");}
      value |= static_cast<std::uint64_t>(byte & 127U) << (index * 7);
      if ((byte & 128U) == 0) {return true;}
    }
    return fail("overflowing ONNX varint");
  }

  bool consume(std::uintmax_t count, std::uintmax_t end)
  {
    if (count > end - position_) {return fail("truncated ONNX field");}
    std::array<char, 65536> buffer;
    while (count != 0) {
      const auto chunk = static_cast<std::streamsize>(std::min<std::uintmax_t>(count, buffer.size()));
      stream_.read(buffer.data(), chunk);
      if (stream_.gcount() != chunk) {return fail("truncated ONNX file");}
      for (std::streamsize i = 0; i < chunk; ++i) {hashByte(static_cast<unsigned char>(buffer[i]));}
      position_ += static_cast<std::uintmax_t>(chunk);
      count -= static_cast<std::uintmax_t>(chunk);
    }
    return true;
  }

  bool message(Kind kind, std::uintmax_t end, unsigned depth)
  {
    if (depth > 64) {return fail("ONNX nesting exceeds the supported limit");}
    while (position_ < end) {
      if (++fields_ > 1000000) {return fail("ONNX field count exceeds the supported limit");}
      std::uint64_t tag = 0, value = 0;
      if (!integer(end, tag)) {return false;}
      if (tag > std::numeric_limits<std::uint32_t>::max() || (tag >> 3) == 0) {
        return fail("invalid ONNX field tag");
      }
      const auto id = static_cast<std::uint32_t>(tag >> 3);
      const auto wire = static_cast<unsigned>(tag & 7U);
      const auto descriptor = field(kind, id);
      if ((descriptor.wires & (1U << wire)) == 0) {
        return fail("unsupported ONNX field or wire type; export a binary ONNX with IR <= 10");
      }
      if (kind == Kind::Tensor && id == 13) {
        return fail("ONNX external_data is unsupported; embed all weights before using the engine cache");
      }
      if (wire == 0) {
        if (!integer(end, value)) {return false;}
        if (kind == Kind::Tensor && id == 14 && value != 0) {
          return fail("ONNX external data_location is unsupported; embed all weights");
        }
        if (kind == Kind::Model && id == 1) {
          if (value == 0 || value > 10 || has_version_) {
            return fail("unsupported or repeated ONNX IR version; expected 1..10");
          }
          has_version_ = true;
        }
      } else if (wire == 2) {
        if (!integer(end, value)) {return false;}
        if (value > end - position_) {return fail("truncated ONNX message or data");}
        if (kind == Kind::Model && id == 7) {has_graph_ = true;}
        if (descriptor.child != Kind::None) {
          if (!message(descriptor.child, position_ + value, depth + 1)) {return false;}
        } else if (!consume(value, end)) {return false;}
      } else if (!consume(wire == 1 ? 8 : 4, end)) {return false;}
    }
    return true;
  }

  std::istream & stream_;
  const std::uintmax_t size_;
  std::string & error_;
  std::uintmax_t position_{0};
  std::uint64_t hash_{14695981039346656037ULL};
  std::size_t fields_{0};
  bool has_version_{false}, has_graph_{false};
};
}  // namespace detail
}  // namespace EngineCache

namespace EngineCache {
bool readEmbeddedOnnx(
  const std::filesystem::path & model, std::uintmax_t bytes,
  std::string & content_hash, std::string & error)
{
  std::ifstream stream(model, std::ios::binary);
  if (!stream) {error = "cannot open ONNX model"; return false;}
  detail::EmbeddedOnnxReader reader(stream, bytes, error);
  return reader.read(content_hash);
}

bool readModel(const std::filesystem::path & model, Identity & identity, std::string & error)
{
  std::error_code code;
  identity.filename = model.filename().string();
  identity.bytes = std::filesystem::file_size(model, code);
  if (code) {error = "cannot read model size: " + code.message(); return false;}
  // 必须先拒绝外置权重，再生成任何可命中现有引擎的身份；内嵌模型沿用原键。
  if (!readEmbeddedOnnx(model, identity.bytes, identity.content_hash, error)) {return false;}
  const auto modified = std::filesystem::last_write_time(model, code);
  if (code) {error = "cannot read model timestamp: " + code.message(); return false;}
  identity.modified = static_cast<std::uint64_t>(modified.time_since_epoch().count());
  error.clear();
  return true;
}

std::string key(const Identity & identity, bool legacy)
{
  std::ostringstream text;
  text << identity.filename << '|' << identity.bytes << '|';
  if (legacy) {text << identity.modified << '|';}
  text << identity.content_hash << '|' << identity.precision << '|' << identity.dla_core << '|'
       << identity.gpu << '|' << identity.tensor_rt << '|' << identity.builder_revision << '|'
       << identity.workspace_mb << '|' << identity.optimization_level << '|'
       << identity.calibration << '|' << hash(identity.calibration);
  return hash(text.str());
}

Paths paths(const std::filesystem::path & directory, const Identity & identity, bool legacy)
{
  const auto stem = std::filesystem::path(identity.filename).stem().string() +
    "." + key(identity, legacy);
  return {directory / (stem + ".trt"), directory / (stem + ".int8-failed"),
    directory / (stem + ".timing.cache"), directory / (stem + ".trt.calib")};
}

std::filesystem::path normalized(const std::filesystem::path & value)
{
  std::error_code error;
  const auto path = std::filesystem::weakly_canonical(value, error);
  return error ? std::filesystem::absolute(value).lexically_normal() : path;
}

std::filesystem::path directory(
  const std::filesystem::path & model, const std::filesystem::path & explicit_directory)
{
  return explicit_directory.empty() ? normalized(model).parent_path() : explicit_directory;
}

std::vector<Paths> legacyCandidates(
  const std::filesystem::path & logical_model, const std::filesystem::path & real_model,
  const std::filesystem::path & destination, const Identity & logical, const Identity & real)
{
  std::vector<Paths> candidates;
  for (const auto & directory : {destination, logical_model.parent_path(), real_model.parent_path()}) {
    const auto parent = directory.empty() ? std::filesystem::current_path() : directory;
    for (const auto & identity : {logical, real}) {
      const auto candidate = paths(parent, identity, true);
      const auto name = normalized(candidate.engine);
      const bool duplicate = std::any_of(candidates.begin(), candidates.end(), [&](const Paths & item) {
          return normalized(item.engine) == name;
        });
      if (!duplicate) {candidates.push_back(candidate);}
    }
  }
  return candidates;
}

PromotionResult promote(
  const std::filesystem::path & source, const std::filesystem::path & destination,
  const Verifier & verify, const std::function<bool()> & cancelled)
{
  const auto stopped = [&] {return cancelled && cancelled();};
  if (stopped()) {return {Promotion::Cancelled, {}};}
  std::error_code error;
  if (!std::filesystem::is_regular_file(source, error)) {return {Promotion::Missing, {}};}
  std::string reason;
  if (!verify(source, reason)) {return {Promotion::Rejected, reason};}
  if (stopped()) {return {Promotion::Cancelled, {}};}
  if (normalized(source) == normalized(destination)) {return {Promotion::Reused, {}};}
  if (std::filesystem::is_regular_file(destination, error) && verify(destination, reason)) {
    return {Promotion::Reused, {}};
  }
  if (stopped()) {return {Promotion::Cancelled, {}};}
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const std::filesystem::path temporary = destination.string() + ".migrate-" + suffix + ".partial";
  std::filesystem::copy_file(source, temporary, std::filesystem::copy_options::none, error);
  if (error) {
    const auto reason = "cannot copy cached file: " + error.message();
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    return {Promotion::Failed, reason};
  }
  const auto remove_temporary = [&] {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
    };
  if (stopped()) {remove_temporary(); return {Promotion::Cancelled, {}};}
  if (!verify(temporary, reason)) {
    remove_temporary();
    return {Promotion::Failed, "copied cache verification failed: " + reason};
  }
  if (stopped()) {remove_temporary(); return {Promotion::Cancelled, {}};}
  std::filesystem::rename(temporary, destination, error);
  if (error) {
    remove_temporary();
    return {Promotion::Failed, "cannot commit cached file: " + error.message()};
  }
  return {Promotion::Copied, {}};
}
}  // namespace EngineCache
