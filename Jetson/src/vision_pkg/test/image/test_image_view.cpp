#include "check.hpp"
#include "node/image_view.hpp"

#include <initializer_list>
#include <limits>

namespace {
using Image = sensor_msgs::msg::Image;

Image::SharedPtr message(const std::string & encoding, std::uint32_t width,
  std::uint32_t height, std::uint32_t step, std::initializer_list<std::uint8_t> bytes)
{
  auto image = std::make_shared<Image>();
  image->encoding = encoding; image->width = width; image->height = height; image->step = step;
  image->data = bytes;
  image->header.frame_id = "camera_optical";
  image->header.stamp.sec = 42;
  return image;
}

template<class Function> bool rejects(Function function)
{
  try {function();} catch (const std::invalid_argument &) {return true;}
  return false;
}

void sharing()
{
  Group group("shared image ownership and padded stride");
  auto source = message("bgr8", 1, 2, 5, {10, 20, 30, 99, 98, 40, 50, 60, 97, 96});
  const auto * data = source->data.data();
  std::weak_ptr<const Image> owner = source;
  auto view = vision_ros::bgrImageView(source);
  CHECK(view->image.data == data);
  CHECK(view->image.step == 5 && view->image.rows == 2 && view->image.cols == 1);
  CHECK(view->image.at<cv::Vec3b>(1, 0) == cv::Vec3b(40, 50, 60));
  CHECK(view->header.frame_id == "camera_optical" && view->header.stamp.sec == 42);
  source.reset();
  CHECK(!owner.expired());
  CHECK(view->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(10, 20, 30));
  auto canvas = view->image.clone();
  canvas.setTo(cv::Scalar::all(0));
  CHECK(view->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(10, 20, 30));
  view.reset();
  CHECK(owner.expired());
}

void encodings()
{
  Group group("explicit eight-bit encodings and class IDs");
  auto rgb = message("rgb8", 1, 1, 4, {10, 20, 30, 99});
  const auto bgr = vision_ros::bgrImageView(rgb);
  CHECK(bgr->image.data != rgb->data.data());
  CHECK(bgr->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(30, 20, 10));
  CHECK(rgb->data[0] == 10 && rgb->data[2] == 30 && rgb->data[3] == 99);
  CHECK(vision_ros::bgrImageView(message("bgra8", 1, 1, 4, {1, 2, 3, 255}))->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(1, 2, 3));
  CHECK(vision_ros::bgrImageView(message("rgba8", 1, 1, 4, {1, 2, 3, 255}))->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(3, 2, 1));
  CHECK(vision_ros::bgrImageView(message("mono8", 1, 1, 1, {127}))->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(127, 127, 127));
  for (const char * encoding : {"mono8", "8UC1"}) {
    auto mask = message(encoding, 2, 2, 3, {0, 1, 99, 128, 255, 99});
    mask->is_bigendian = 1;  // Bytes have no endian conversion.
    const auto view = vision_ros::labelMaskView(mask);
    CHECK(view->image.data == mask->data.data() && view->image.step == 3);
    CHECK(view->image.at<std::uint8_t>(0, 0) == 0 && view->image.at<std::uint8_t>(0, 1) == 1);
    CHECK(view->image.at<std::uint8_t>(1, 0) == 128 && view->image.at<std::uint8_t>(1, 1) == 255);
  }
  auto big = message("bgr8", 1, 1, 3, {1, 2, 3}); big->is_bigendian = 1;
  CHECK(vision_ros::bgrImageView(big)->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(1, 2, 3));
}

void malformed()
{
  Group group("invalid image metadata and unsupported encodings");
  CHECK(rejects([] {vision_ros::bgrImageView(nullptr);}));
  CHECK(rejects([] {vision_ros::labelMaskView(nullptr);}));
  CHECK(rejects([] {vision_ros::bgrImageView(message("bgr8", 0, 1, 0, {}));}));
  CHECK(rejects([] {vision_ros::bgrImageView(message("bgr8", 1, 0, 3, {}));}));
  CHECK(rejects([] {vision_ros::bgrImageView(message("bgr8", 1, 1, 2, {1, 2}));}));
  CHECK(rejects([] {vision_ros::bgrImageView(message("bgr8", 1, 2, 3, {1, 2, 3}));}));
  CHECK(rejects([] {vision_ros::bgrImageView(message("bgr8", 1, 1, 3, {1, 2, 3, 4}));}));
  CHECK(rejects([] {vision_ros::bgrImageView(message("bgr8", std::numeric_limits<std::uint32_t>::max(), 1, 3, {1, 2, 3}));}));
  CHECK(rejects([] {vision_ros::bgrImageView(message("rgb16", 1, 1, 6, {0, 1, 0, 2, 0, 3}));}));
  CHECK(rejects([] {vision_ros::labelMaskView(message("mono16", 1, 1, 2, {0, 1}));}));
  CHECK(rejects([] {vision_ros::labelMaskView(message("rgb8", 1, 1, 3, {1, 2, 3}));}));
  CHECK(rejects([] {auto image = message("bgr8", 1, 1, 3, {1, 2, 3}); image->is_bigendian = 2; vision_ros::bgrImageView(image);}));
}
}  // namespace

int main()
{
  sharing(); encodings(); malformed();
  return finishTests("ROS image views");
}
