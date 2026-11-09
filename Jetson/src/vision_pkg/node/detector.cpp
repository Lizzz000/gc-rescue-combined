// Detector 的创建、分割和场地转发。话题和线程在 detect_node.cpp、detect_workers.cpp。

#include "node/detector.hpp"

#include "yolo/engine/engine.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

Detector::Detector() = default;
Detector::~Detector() = default;
using ms_d = std::chrono::duration<double, std::milli>;
const char * toString(DetectTask task)
{
  switch (task) {
    case DetectTask::Segment: return "seg";
    case DetectTask::Field: return "field";
    case DetectTask::Both: return "both";
  }
  return "?";
}
bool parseTask(const std::string & text, DetectTask & out)
{
  if (text == "seg") {out = DetectTask::Segment; return true;}
  if (text == "field") {out = DetectTask::Field; return true;}
  if (text == "both") {out = DetectTask::Both; return true;}
  return false;
}
// 已处理的后端拒绝、加载失败或取消返回 nullptr；cancelled_out 区分停止请求与真实失败。
// 场地范围已由节点入口的 readFieldParameters 检查；这里只装配后端，不执行引擎构建。
std::unique_ptr<Detector> Detector::create(
  const DetectorConfig & config,
  std::string & error, std::vector<std::string> & notes, bool * cancelled_out)
{
  error.clear(); notes.clear();
  if (cancelled_out) {*cancelled_out = false;}
  BuildCancellation cancellation(config.cancel_requested);
  auto cancelled = [&] {
      if (!cancellation.requested()) {return false;}
      if (cancelled_out) {*cancelled_out = true;}
      error = "detector initialization cancelled";
      return true;
    };
  if (cancelled()) {return nullptr;}
  const bool wants_seg = config.task != DetectTask::Field;
  // task 决定本次会话加载哪些资源；逐帧 active_task 切换在节点完成，不会在 segment()/field() 中重建后端。
  const bool wants_field = config.task != DetectTask::Segment;
  auto detector = std::unique_ptr<Detector>(new Detector());
  detector->config_ = config;
  if (wants_field) {
    detector->field_locator_ = FieldLocator(config.field);
    if (cancelled()) {return nullptr;}
    detector->field_available_ = true;
    notes.push_back(std::string("field ready: OpenCV purple boundary / ") +
      (config.field.use_blue_interior ? "blue" : "red") + " field / black divider");
  }
#if VISION_ENABLE_TENSORRT
  // 分割只使用 TensorRT GPU；未启用时仅提供场地算法。
  if (wants_seg && config.seg_model.empty()) {
    error = "seg_model is required for task=seg or both";
    return nullptr;
  }
  if (wants_seg) {
    // 生产分割只加载显式预构建引擎，缺失/合同不符立即失败，绝不触发冷构建或精度回退。
    if (!isEnginePath(config.seg_model)) {
      error = "production segmentation requires a prebuilt .trt/.engine file";
      return nullptr;
    }
    // 与实际适配器共用固定上游的类别读取规则，避免预检与解码各自猜测布局。
    const auto class_count = yolos::utils::getClassNames(config.seg_labels).size();
    if (!checkTensorRTRuntime(error) ||
      !verifyEngineFileForClasses(config.seg_model, error, class_count)) {
      return nullptr;
    }
    if (cancelled()) {return nullptr;}
    try {
      detector->segmentor_ = std::make_unique<FastSegmentor>(
        config.seg_model, config.seg_labels, config.dla_core);
    } catch (const std::exception & e) {
      error = std::string("failed to load the segmentation engine: ") + e.what();
      return nullptr;
    }
    if (cancelled()) {return nullptr;}
    // 此处只读形状用于诊断；verifyEngineFileForClasses 已在上游分配/预热前结合类别数验收张量布局。
    const cv::Size input = detector->segmentor_->getInputShape();
    notes.push_back("seg ready: " + std::filesystem::path(config.seg_model).filename().string() +
                    " (prebuilt), input " + std::to_string(input.width) +
                    "x" + std::to_string(input.height) + ", " +
      (detector->segmentor_->classCount() > 0 ?
      std::to_string(detector->segmentor_->classCount()) :
      std::string("unknown")) + " classes, " +
                    std::to_string(detector->segmentor_->prototypeCount()) +
                    " prototypes, layout " +
      (detector->segmentor_->isEndToEnd() ?
      std::string("yolo26 end-to-end + bounded class-aware NMS") :
      std::string("legacy (NMS)")));
    if (detector->segmentor_->prototypeCount() != 32) {
      // 当前部署模型通常为32个原型；合法的end-to-end布局按引擎实际P解码，legacy合同才固定32。
      notes.push_back("segmentation engine reports " +
                      std::to_string(detector->segmentor_->prototypeCount()) +
                      " prototypes; the validated end-to-end layout uses this engine-reported count, "
                      "while the deployed model normally reports 32");
    }
    if (!detector->segmentor_->isEndToEnd()) {
      notes.push_back("the segmentation engine is not YOLO26 layout, so iou " +
                      std::to_string(config.iou) +
                      " applies; Ultralytics validates with 0.7");
    }
  }


  detector->seg_available_ = wants_seg;
#else
  if (wants_seg) {
    error = "seg requires a TensorRT build; CPU builds support task=field active_task=field";
    return nullptr;
  }
#endif
  if (cancelled()) {return nullptr;}
  return detector;
}
cv::Size Detector::segInput() const
{
#if VISION_ENABLE_TENSORRT
  return segmentor_ ? segmentor_->getInputShape() : cv::Size();
#else
  return {};
#endif
}
// 分割不沿用上一帧检测结果：每次新建 SegRun、透传选项并回填耗时；未加载分割时返回空结果。
SegRun Detector::segment(const cv::Mat & image, bool want_label_mask, const void * nvmm) const
{
  SegRun run;
#if VISION_ENABLE_TENSORRT
  if (!segmentor_) {return run;}

  SegOptions options;
  options.conf_threshold = config_.conf_seg;
  options.iou_threshold = config_.iou;
  options.max_detections = config_.max_detections;
  options.mask_scale = config_.mask_scale;
  options.mask_label_offset = config_.mask_label_offset;
  options.want_label_mask = want_label_mask;
  // ROS 生产链只按订阅需求合成类别标签图，不为每个实例额外保存整张二值图。
  options.want_instance_masks = false;
  options.max_masks = config_.max_masks;

  SegTiming timing;
  const auto t0 = std::chrono::steady_clock::now();
  run.detections = segmentor_->segmentFast(image, options, run.label_mask, &timing, nvmm);
  const auto t1 = std::chrono::steady_clock::now();
  run.inference_ms = ms_d(t1 - t0).count();
  run.gpu_ms = timing.gpu_ms;
  run.post_ms = timing.post_ms;
#else
  (void)image;
  (void)want_label_mask;
  (void)nvmm;
#endif
  return run;
}


// 有状态场地定位：直接转发 FieldLocator::update()，必须按帧序调用，换相机/重连后 resetField()。
FieldResult Detector::field(const cv::Mat & image)
{
  return field_available_ ? field_locator_.update(image) : FieldResult();
}
void Detector::resetField() {field_locator_.reset();}
