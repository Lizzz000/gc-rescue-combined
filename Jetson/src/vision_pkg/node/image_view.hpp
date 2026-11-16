#pragma once
// ROS Image 的八位像素适配。共享消息保住像素；只用本包选择的 OpenCV，不跨版本桥接。

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace vision_ros
{
struct ImageView
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image::ConstSharedPtr owner;
  cv::Mat image;  // 共享的输入像素只读；绘制前由调用者 clone。
};
using ImageViewConstPtr = std::shared_ptr<const ImageView>;

namespace image_detail
{
inline cv::Mat checkedView8(const sensor_msgs::msg::Image::ConstSharedPtr & message, int channels)
{
  if (!message || message->width == 0 || message->height == 0 ||
    message->width > std::uint32_t(std::numeric_limits<int>::max()) ||
    message->height > std::uint32_t(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("image dimensions must fit non-empty OpenCV rows/columns");
  }
  const std::uint64_t row_bytes = std::uint64_t(message->width) * channels;
  const std::uint64_t total_bytes = std::uint64_t(message->height) * message->step;
  if (message->step < row_bytes || total_bytes != message->data.size()) {
    throw std::invalid_argument("image step or payload length is inconsistent with dimensions");
  }
  if (message->is_bigendian > 1) {throw std::invalid_argument("invalid image endian flag");}
  // 每个通道恰好一个字节，0/1 两种端序标记都不改变像素或通道顺序。
  return cv::Mat(int(message->height), int(message->width), CV_MAKETYPE(CV_8U, channels),
    const_cast<std::uint8_t *>(message->data.data()), message->step);
}
}  // namespace image_detail

inline ImageViewConstPtr bgrImageView(const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  if (!message) {throw std::invalid_argument("null image message");}
  namespace enc = sensor_msgs::image_encodings;
  int channels = 0, conversion = -1;
  if (message->encoding == enc::BGR8) {channels = 3;}
  else if (message->encoding == enc::RGB8) {channels = 3; conversion = cv::COLOR_RGB2BGR;}
  else if (message->encoding == enc::BGRA8) {channels = 4; conversion = cv::COLOR_BGRA2BGR;}
  else if (message->encoding == enc::RGBA8) {channels = 4; conversion = cv::COLOR_RGBA2BGR;}
  else if (message->encoding == enc::MONO8) {channels = 1; conversion = cv::COLOR_GRAY2BGR;}
  else {throw std::invalid_argument("unsupported stream image encoding: " + message->encoding);}
  const cv::Mat input = image_detail::checkedView8(message, channels);
  auto view = std::make_shared<ImageView>();
  view->header = message->header;
  view->owner = message;
  if (conversion < 0) {view->image = input;}
  else {cv::cvtColor(input, view->image, conversion);}
  return view;
}

inline ImageViewConstPtr labelMaskView(const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  if (!message || (message->encoding != sensor_msgs::image_encodings::MONO8 &&
    message->encoding != sensor_msgs::image_encodings::TYPE_8UC1)) {
    throw std::invalid_argument("label mask requires mono8 or 8UC1 class IDs");
  }
  auto view = std::make_shared<ImageView>();
  view->header = message->header;
  view->owner = message;
  // mono8/8UC1 都表示原始类别字节，禁止灰度转换、缩放或重映射标签。
  view->image = image_detail::checkedView8(message, 1);
  return view;
}
}  // namespace vision_ros
