#pragma once

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <variant>

#include "camera/camera.hpp"
#include "node/detector.hpp"
#include "node/diagnostics.hpp"
#include "node/latest_slot.hpp"
#include "node/stage_gate.hpp"
#include "node/worker_group.hpp"
#include "stream/stream.hpp"

// 两条发布路径共享只读 BGR；NVMM 只由推理线程取用，所有权与源帧一起交接。
struct DetectFrame
{
  cv::Mat image;
  mutable std::shared_ptr<void> nvmm;
  std::int64_t stamp_ns{0};
  std::uint64_t sequence{0};
  std::uint64_t generation{0};
  std::uint64_t camera_session{0};
};

class DetectRuntime
{
public:
  enum class Log
  {
    Info, Reopening, OpenFailed, ReadFailed, InferenceFailed, ResultPublishFailed, RawPublishFailed
  };
  struct Ports
  {
    std::function<std::int64_t()> source_now;
    std::function<bool()> wants_raw, wants_mask;
    std::function<bool(const DetectFrame &, const SegRun &)> publish_seg;
    std::function<bool(const DetectFrame &, const FieldResult &)> publish_field;
    std::function<void(const DetectFrame &)> publish_raw;
    std::function<void(int, std::int64_t, std::uint64_t)> publish_task;
    std::function<void(Log, const std::string &)> log;
  };
  struct Status
  {
    double capture_fps, seg_fps, field_fps, raw_fps;
    double seg_inference_ms, field_inference_ms, seg_gpu_ms, seg_post_ms, field_post_ms;
    std::uint64_t inference_replaced, raw_replaced, result_replaced, camera_reopens;
    std::uint64_t seg_skipped, field_skipped;
    bool seg_active, field_active;
  };

  explicit DetectRuntime(Ports ports);
  ~DetectRuntime();
  void setCamera(const CameraConfig & config, double raw_max_fps);
  void setDetector(std::unique_ptr<Detector> detector);
  void setRates(double seg, double field);
  std::pair<double, double> rates() const;
  void setTask(int mask);
  void refreshTask();
  bool supportsTask(int mask) const;
  bool hasSeg() const;
  bool hasField() const;
  const FieldConfig & fieldConfig() const;
  Status sampleStatus();

  // 采集和各发布线程在停止时先唤醒并回收，再释放相机。
  void start(bool raw_enabled);
  void stop();
  void rethrowIfFailed() const {workers_.rethrowIfFailed();}

  // 消息在进入本边界前组装；同帧消息只有通过最后一次复核才会成组发布。
  template<class Publish>
  bool publishCurrent(const DetectFrame & frame, int task_bit, Publish publish)
  {
    auto gate = stage_.lockPublication();
    if (!frameIsCurrent(frame, task_bit)) {return false;}
    publish();
    if (task_bit == 0) {raw_published_.fetch_add(1);}
    return true;
  }

private:
  void captureLoop();
  void inferenceLoop();
  void rawLoop();
  void resultLoop();
  void wakeWaiters();
  void clearPendingFrames();
  bool frameIsCurrent(const DetectFrame & frame, int task_bit) const;
  bool segActive() const;
  bool fieldActive() const;
  void logLetterboxScale();

  Ports ports_;
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<UsbCamera> camera_;
  double raw_max_fps_{0.0};
  LatestSlot<const DetectFrame> inference_slot_, raw_slot_;
  // 邮箱只整体替换已完成的结果及其实际输入帧。
  struct CompletedFrame
  {
    std::shared_ptr<const DetectFrame> frame;
    std::variant<SegRun, FieldResult> result;
  };
  LatestSlot<const CompletedFrame> result_slot_;
  std::uint64_t sequence_{0};  // 只由采集线程递增。

  std::atomic<std::uint64_t> captured_frames_{0}, raw_published_{0}, camera_reopens_{0};
  RateMeter seg_meter_, field_meter_;
  WindowCounter capture_window_, seg_window_, field_window_, raw_window_;
  std::atomic<double> seg_gpu_ms_{0.0}, seg_post_ms_{0.0}, field_post_ms_{0.0};
  std::atomic<double> seg_max_fps_{0.0}, field_max_fps_{0.0};
  std::atomic<std::uint64_t> seg_skipped_{0}, field_skipped_{0};

  StageGate stage_;
  WorkerGroup workers_;
  FrameRateLimiter raw_limiter_;
};
