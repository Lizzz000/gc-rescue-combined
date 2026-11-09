#include "yolo/tensorrt/tensorrt.hpp"
#include "yolo/yolo.hpp"

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
#include <cuda_runtime.h>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// 归一化尺度、填充值和通道顺序必须与网络真实输入以及 CUDA letterbox kernel 对齐。
constexpr float kNormScale = 1.0f / 255.0f;
constexpr float kPadValue = 114.0f / 255.0f;

ImageFolderCalibrator::ImageFolderCalibrator(
  const std::string & dir,
  const cv::Size & input_size,
  std::string cache_file,
  std::string input_name, const BuildCancellation * cancellation)
: cancellation_(cancellation), input_size_(input_size), cache_file_(std::move(cache_file)),
  input_name_(std::move(input_name))
{
  if (stopped()) {return;}
  std::error_code ec;
  const std::filesystem::path root(dir);
  if (!std::filesystem::is_directory(root, ec)) {
    error_ = "calibration directory does not exist: " + dir;
    return;
  }

  if (stopped()) {return;}
  files_ = calibrationImages(dir);
  // 排序保证顺序确定，这样 calibration 缓存才可复现。
  std::sort(files_.begin(), files_.end());

  if (stopped()) {return;}
  if (input_size_.width > 0 && input_size_.height > 0) {
    host_blob_.assign(
      static_cast<std::size_t>(3) * input_size_.width * input_size_.height, kPadValue);
  }
}

ImageFolderCalibrator::~ImageFolderCalibrator()
{
  if (device_blob_ != nullptr) {
    cudaFree(device_blob_);
  }
}

int ImageFolderCalibrator::getBatchSize() const noexcept
{
  // 每个 batch 只放一张图；getBatch() 自行分配并复用设备缓冲，
  // 将自有缓冲的指针写入 TensorRT 提供的 bindings 输出数组。
  return 1;
}

bool ImageFolderCalibrator::loadAndPreprocess(const std::string & path)
{
  if (stopped()) {return false;}
  const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (stopped()) {return false;}
  if (image.empty()) {
    error_ = "cannot read calibration image: " + path;
    return false;
  }
  if (input_size_.width <= 0 || input_size_.height <= 0) {
    error_ = "engine input size is unknown; export the ONNX at a fixed imgsz";
    return false;
  }

  const Letterbox lb = letterboxFor(image.size(), input_size_);
  if (lb.new_w <= 0 || lb.new_h <= 0) {
    error_ = "degenerate letterbox for " + path;
    return false;
  }

  std::fill(host_blob_.begin(), host_blob_.end(), kPadValue);

  cv::Mat resized;
  if (stopped()) {return false;}
  cv::resize(image, resized, cv::Size(lb.new_w, lb.new_h), 0, 0, cv::INTER_LINEAR);
  if (stopped()) {return false;}

  const int out_w = input_size_.width;
  const int out_h = input_size_.height;
  const int plane = out_w * out_h;

  float * plane_r = host_blob_.data();
  float * plane_g = plane_r + plane;
  float * plane_b = plane_g + plane;

  for (int y = 0; y < lb.new_h; ++y) {
    const int dst_y = y + lb.pad_top;
    if (dst_y < 0 || dst_y >= out_h) {continue;}
    const unsigned char * row = resized.ptr<unsigned char>(y);
    for (int x = 0; x < lb.new_w; ++x) {
      const int dst_x = x + lb.pad_left;
      if (dst_x < 0 || dst_x >= out_w) {continue;}
      const int di = dst_y * out_w + dst_x;
      const int si = x * 3;
      // OpenCV 读进来是 BGR，而网络要的是分成三个平面的 RGB。
      plane_b[di] = row[si + 0] * kNormScale;
      plane_g[di] = row[si + 1] * kNormScale;
      plane_r[di] = row[si + 2] * kNormScale;
    }
  }
  return !stopped();
}

// 读图失败在 loadAndPreprocess() 和这个函数里跳过该张，不中断整批标定。
bool ImageFolderCalibrator::getBatch(
  void * bindings[], char const * names[],
  int32_t nb_bindings) noexcept
{
  try {
    if (bindings != nullptr && nb_bindings > 0) {bindings[0] = nullptr;}
    // 资源/绑定失败锁存至构建结束，TensorRT 再次询问批次也不能清掉失败证据。
    if (!error_.empty()) {return false;}
    if (nb_bindings != 1 || bindings == nullptr || names == nullptr ||
      names[0] == nullptr || input_name_ != names[0])
    {
      error_ = "calibrator requires one input binding named " + input_name_;
      return false;
    }
    if (stopped()) {return false;}
    if (host_blob_.empty()) {
      error_ = "engine input size is unknown; export the ONNX at a fixed imgsz";
      return false;
    }
    if (files_.empty()) {
      error_ = "no calibration images found in the directory";
      return false;
    }
    // 坏图可能连续出现；循环跳过，不随坏图数量增长调用栈。
    while (next_ < files_.size()) {
      if (stopped()) {return false;}
      if (loadAndPreprocess(files_[next_])) {break;}
      // 取消不是坏图；不得清掉停止状态并继续读取下一张。
      if (stopped()) {return false;}
      std::cerr << "[engine][calib] " << error_ << " (skipped)" << std::endl;
      error_.clear();
      ++next_;
    }
    if (next_ >= files_.size()) {
      // 没有剩余可解码图片，按批次结束返回；绑定仍为空，不残留上一张图。
      return false;
    }

    if (stopped()) {return false;}
    const std::size_t bytes = host_blob_.size() * sizeof(float);
    if (device_blob_ == nullptr) {
      const cudaError_t alloc_error = cudaMalloc(&device_blob_, bytes);
      if (alloc_error != cudaSuccess) {
        error_ = std::string("cudaMalloc for calibration failed: ") +
          cudaGetErrorString(alloc_error);
        std::cerr << "[engine][calib] " << error_ << std::endl;
        return false;
      }
    }
    if (stopped()) {return false;}
    const cudaError_t err = cudaMemcpy(
      device_blob_, host_blob_.data(), bytes, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
      error_ = std::string("cudaMemcpy for calibration failed: ") + cudaGetErrorString(err);
      std::cerr << "[engine][calib] " << error_ << std::endl;
      return false;
    }

    // 上传已在途时不能强行中断；完成后仍复核停止，不交付已取消的绑定。
    if (stopped()) {return false;}
    bindings[0] = device_blob_;
    ++next_;
    return true;
  } catch (const std::exception & e) {
    error_ = std::string("calibrator exception: ") + e.what();
    std::cerr << "[engine][calib] " << error_ << std::endl;
    return false;
  }
}

void const * ImageFolderCalibrator::readCalibrationCache(std::size_t & length) noexcept
{
  length = 0;
  if (stopped()) {return nullptr;}
  cache_.clear();
  if (cache_file_.empty()) {return nullptr;}

  std::ifstream f(cache_file_, std::ios::binary);
  if (!f) {return nullptr;}
  cache_.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  if (stopped() || cache_.empty()) {return nullptr;}

  std::cout << "[engine] calibration cache hit: " << cache_file_ << std::endl;
  length = cache_.size();
  return cache_.data();
}

void ImageFolderCalibrator::writeCalibrationCache(
  void const * ptr,
  std::size_t length) noexcept
{
  // 检查点已观察到停止时不发起新 IO；检查之后已经开始的文件写入不由此回调原子中断。
  if (stopped() || cache_file_.empty() || ptr == nullptr || length == 0) {return;}
  try {
    std::string error;
    // 缓存内容是 TensorRT 的不透明字节；完整写入、关闭及长度由公共提交器核对。
    // 写失败保留旧缓存，不把截断内容交给下一次标定，也不虚报成功。
    const bool written = vision::commitVerifiedBinary(cache_file_, ptr, length,
      [](const auto &, std::string &) {return true;}, [&] {return stopped();}, error);
    if (!written) {
      std::cerr << "[engine] cannot commit calibration cache " << cache_file_
                << ": " << error << std::endl;
      return;
    }
    std::cout << "[engine] wrote calibration cache: " << cache_file_ << std::endl;
  } catch (const std::exception & error) {
    // TensorRT 回调是 noexcept；缓存是可重建的加速数据，失败只报告，不终止进程。
    std::cerr << "[engine] calibration cache write failed: " << error.what() << std::endl;
  } catch (...) {
    std::cerr << "[engine] calibration cache write failed: unknown exception" << std::endl;
  }
}
