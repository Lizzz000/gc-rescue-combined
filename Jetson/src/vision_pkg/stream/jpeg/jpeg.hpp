#pragma once
// ---- JPEG 编码 ----
// 全包唯一的 JPEG 编码模块（无 ROS 运行时依赖）：按「帧键＋缩放＋质量」缓存最近一帧的字节，
// 帧键、缩放与质量都不变时直接复用，供 sensor_msgs/CompressedImage 与 MJPEG 图传共用同一次编码。
// 编码器本身不带锁；仅当两路要求的缩放和质量相同，调用方才可共享编码结果。

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

enum class JpegBackend {OpenCv, NvJpeg};
class NvJpegEncoder;
class JpegEncoderTestAccess;

class JpegEncoder {
public:
  JpegEncoder();
  ~JpegEncoder();
  JpegEncoder(const JpegEncoder &) = delete;
  JpegEncoder & operator=(const JpegEncoder &) = delete;

  /// 启动装配时选择后端；首次成功编码后不能更换。插件或硬件故障不自动转为软件执行。
  /// NvJpeg 对非偶数或不在16～32768范围的合法输出使用 OpenCV 尺寸兼容路径，尺寸/质量不变。
  void setBackend(JpegBackend backend);
  JpegBackend backend() const;
  /// 最近一次成功真实编码的后端；尚未编码时为 OpenCv，缓存命中不改变它。
  JpegBackend lastEncodeBackend() const;
  std::uint64_t hardwareEncodeCount() const;
  std::uint64_t sizeCompatibilityEncodeCount() const;

  /// 编码一帧并返回字节；帧键、缩放、质量与上一次完全相同则直接返回缓存，不重新编码。
  /// 返回值共享缓存本体；HTTP可直接共享，ROS CompressedImage仍须将字节复制到消息data，不重新编码。
  /// 输出边长按 scale 取整、最小为 1；取整后尺寸不变时共享原像素。
  /// 空图返回 nullptr 并保留缓存；编码返回失败时清缓存，抛出的异常保留旧缓存并交给调用方。
  /// frame_key 由调用方保证对应同一份像素内容；本类不对图像求哈希，同键换图会命中旧缓存。
  std::shared_ptr<const std::vector<unsigned char>> encode(
    const cv::Mat & bgr, std::uint64_t frame_key, double scale, int quality);

  /// 真实编码次数（缓存命中不计），供测试与诊断使用；reset() 不清零。
  std::uint64_t encodeCount() const;

  /// 最近一次编码的字节数；当前没有可用缓存时为 0。
  std::size_t size() const;

  /// 丢弃缓存：画面来源变化（例如阶段切换）后不允许再复用旧字节。
  /// 已返回给消费者的共享字节仍然有效；本类没有内部锁，跨线程调用需由外层串行化。
  void reset();

private:
  friend class JpegEncoderTestAccess;
  // 窄测试接缝：真实NV管线初始化完成后、下次push前注入一次错误，不替换整个编码实现。
  void failNextHardwareEncodeForTest();
  std::unique_ptr<NvJpegEncoder> hardware_;
  JpegBackend backend_{JpegBackend::OpenCv};
  JpegBackend last_backend_{JpegBackend::OpenCv};
  std::uint64_t hardware_encodes_{0};
  std::uint64_t size_compatibility_encodes_{0};
  std::shared_ptr<const std::vector<unsigned char>> bytes_;
  // 缓存键：帧键＋缩放＋质量，三者任一不同就重新编码。
  std::uint64_t frame_key_{0};
  double scale_{0.0};
  int quality_{0};
  bool valid_{false};
  std::uint64_t encodes_{0};
};
