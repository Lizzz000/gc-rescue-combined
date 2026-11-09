// ---- MP4 封装声明 ----
// 职责：把预览帧写成 MP4，不负责采集。
#pragma once
#include "stream_protocol.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace record {

// 仅表示 JPEG 输入无效；调用方可统计并跳过，其他异常仍须终止会话。
class BadJpeg : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// 独占单路 MJPEG 编码资源，调用方串行使用；原子创建文件，拒绝覆盖。
// Windows 的 UTF-8 路径字符串应通过 std::filesystem::u8path() 传入。
class Mp4Writer {
 public:
  // H.264 固定 libx264；encoder 参数仅为兼容保留，任何取值都用 libx264。
  Mp4Writer(const std::filesystem::path& path, int width, int height, int fps,
            const std::string& encoder = "x264");
  ~Mp4Writer() noexcept;

  Mp4Writer(const Mp4Writer&) = delete;
  Mp4Writer& operator=(const Mp4Writer&) = delete;
  Mp4Writer(Mp4Writer&&) = delete;
  Mp4Writer& operator=(Mp4Writer&&) = delete;

  // PTS 为采集时间减首帧后向下取整的微秒数，不按标称帧率重排。
  // BadJpeg 不推进帧数或时间；来源、时间和规格错误不归入 BadJpeg。
  // 编码、封装或 I/O 失败后停止接收，finish 再次报错。
  void write(const Frame& frame);

  // 成功后幂等；排空、尾部或关闭失败均上抛，重复调用保留错误。
  // 析构尽力完成并释放资源但不抛异常；调用方须显式 finish 才能获知失败。
  void finish();

  // 已接受帧数（含待封装包）；崩溃仍可能丢失未完成片段或系统缓冲数据。
  std::uint64_t frames() const;
  // 返回实际 FFmpeg 编码器名（libx264）。
  const std::string& encoder_name() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // 命名空间 record
