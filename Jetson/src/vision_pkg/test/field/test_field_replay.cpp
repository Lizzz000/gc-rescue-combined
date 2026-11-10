// 安全区实拍回放：读 data/<clip>/labels.yaml 与逐帧图片，按标注逐帧检查。
// 没有任何带 labels.yaml 的片段时返回 77（跳过）。目录可用环境变量 FIELD_REPLAY_DIR 覆盖。
// labels.yaml 中 reference_only: 1 的片段（如手机拍摄）只报告不判。格式见 data/README.md。
// 用法：test_field_replay [--report]
#include "check.hpp"
#include "harness.hpp"
#include "metrics.hpp"
#include "visibility.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#ifndef FIELD_DATA_DIR
#define FIELD_DATA_DIR "data"
#endif

namespace {

namespace fs = std::filesystem;
using namespace field_test;

struct LabeledFrame
{
  std::string file;
  bool present = false;
  Judge judge = Judge::BORDERLINE;
  std::vector<float> divider;   // x_far, y_far, x_near, y_near（原图像素），可空
  std::vector<float> corners;   // 内沿 TL,TR,BR,BL 共 8 个数（原图像素），可空
};

struct Clip
{
  fs::path dir;
  bool reference_only = false;
  int image_width = 0;
  std::vector<LabeledFrame> frames;
};

Judge parseJudge(const std::string & s)
{
  if (s == "judgeable") {return Judge::JUDGEABLE;}
  if (s == "unjudgeable") {return Judge::UNJUDGEABLE;}
  return Judge::BORDERLINE;
}

Clip loadClip(const fs::path & dir)
{
  Clip clip;
  clip.dir = dir;
  cv::FileStorage fsr((dir / "labels.yaml").string(), cv::FileStorage::READ);
  clip.reference_only = int(fsr["reference_only"]) != 0;
  clip.image_width = int(fsr["image_width"]);
  for (const auto & node : fsr["frames"]) {
    LabeledFrame f;
    f.file = std::string(node["file"]);
    f.present = int(node["present"]) != 0;
    f.judge = parseJudge(std::string(node["judge"]));
    if (!node["divider"].empty()) {node["divider"] >> f.divider;}
    if (!node["corners"].empty()) {node["corners"] >> f.corners;}
    clip.frames.push_back(f);
  }
  return clip;
}

std::vector<Clip> findClips()
{
  const char * env = std::getenv(kReplayDirEnv);
  const fs::path root = env && *env ? env : FIELD_DATA_DIR;
  std::vector<Clip> clips;
  if (!fs::is_directory(root)) {return clips;}
  std::vector<fs::path> dirs;
  for (const auto & e : fs::directory_iterator(root)) {
    if (e.is_directory() && fs::exists(e.path() / "labels.yaml")) {dirs.push_back(e.path());}
  }
  std::sort(dirs.begin(), dirs.end());
  for (const auto & d : dirs) {
    Clip c = loadClip(d);
    if (!c.frames.empty()) {clips.push_back(std::move(c));}
  }
  return clips;
}

struct ClipStats
{
  int frames = 0, judgeable = 0, judgeable_valid = 0, unjudgeable_valid = 0, absent_valid = 0, wrong = 0, predicted = 0;
  std::vector<double> split_err, edge_dist;
  std::vector<bool> valid;
};

cv::Point2f scaled(const std::vector<float> & v, size_t i, cv::Point2d s)
{
  return {float(v[i] * s.x), float(v[i + 1] * s.y)};
}

void scoreFrame(ClipStats & st, const LabeledFrame & f, const FieldResult & r, cv::Point2d scale, cv::Size size)
{
  ++st.frames;
  const bool valid = splitValid(r);
  st.valid.push_back(valid);
  st.predicted += predictedAsMeasured(r);
  if (!f.present) {st.absent_valid += committedOutline(r) || valid; return;}
  st.judgeable += f.judge == Judge::JUDGEABLE;
  st.judgeable_valid += f.judge == Judge::JUDGEABLE && valid;
  st.unjudgeable_valid += f.judge == Judge::UNJUDGEABLE && valid;
  if (committedOutline(r) && f.corners.size() == 8) {
    std::array<cv::Point2f, 4> truth;
    for (int k = 0; k < 4; ++k) {truth[k] = scaled(f.corners, 2 * k, scale);}
    if (auto d = edgeLineDistance(r.corners, truth, size, kEdgeSamples)) {st.edge_dist.push_back(*d);}
  }
  if (valid && f.divider.size() == 4) {
    const double e = std::abs(splitLateralError(r, scaled(f.divider, 0, scale), scaled(f.divider, 2, scale)));
    st.split_err.push_back(e);
    st.wrong += e > kSplitLabelTolPx;
  }
}

double mean(const std::vector<double> & v)
{
  double s = 0;
  for (double x : v) {s += x;}
  return v.empty() ? 0.0 : s / v.size();
}

ClipStats runClip(const Clip & clip, const FieldConfig & config)
{
  const cv::Size size(config.proc_width, config.proc_height);
  FieldLocator locator(config);
  ClipStats st;
  for (const auto & f : clip.frames) {
    const cv::Mat image = cv::imread((clip.dir / f.file).string(), cv::IMREAD_COLOR);
    if (image.empty()) {std::fprintf(stderr, "读不到 %s\n", (clip.dir / f.file).c_str()); continue;}
    if (clip.image_width > 0 && clip.image_width != image.cols) {
      std::fprintf(stderr, "%s 宽 %d 与 labels.yaml 的 image_width %d 不一致\n", f.file.c_str(), image.cols, clip.image_width);
    }
    // 标注是原图像素；算法角点在处理分辨率
    const cv::Point2d scale(double(size.width) / image.cols, double(size.height) / image.rows);
    scoreFrame(st, f, locator.update(image), scale, size);
  }
  return st;
}

void expect(bool gate, bool ok, const std::string & clip, const char * what)
{
  if (!gate) {
    if (!ok) {std::printf("  [仅参考] %s：%s\n", clip.c_str(), what);}
    return;
  }
  if (ok) {++g_passed; return;}
  ++g_failed;
  std::fprintf(stderr, "FAIL [回放] %s：%s\n", clip.c_str(), what);
}

void judgeClip(const Clip & clip, const ClipStats & st)
{
  const std::string name = clip.dir.filename().string();
  const bool gate = !clip.reference_only;
  std::printf("[回放%s] %s 帧 %d，可判有效 %d/%d，不可判仍有效 %d，无场地误检 %d，错标 %d，预测冒充 %d，翻转 %d，"
    "中线误差 %.2f px，边线距离 %.2f px\n", gate ? "" : "·仅参考", name.c_str(), st.frames, st.judgeable_valid,
    st.judgeable, st.unjudgeable_valid, st.absent_valid, st.wrong, st.predicted, countFlips(st.valid),
    mean(st.split_err), mean(st.edge_dist));
  expect(gate, st.judgeable == 0 || ratio(st.judgeable_valid, st.judgeable) >= kMinValidRate, name, "可判帧有效率不足");
  expect(gate, st.unjudgeable_valid == 0, name, "不可判帧仍输出分区");
  expect(gate, st.absent_valid == 0, name, "无场地帧误检");
  expect(gate, st.wrong == 0, name, "分区标签错误");
  expect(gate, st.predicted <= kMaxPredictedAsMeasured, name, "预测冒充测量");
  expect(gate, st.split_err.empty() || mean(st.split_err) <= kSplitErrMaxPx, name, "中线误差超限");
  expect(gate, st.edge_dist.empty() || mean(st.edge_dist) <= kEdgeLineTolPx, name, "边到内沿线距离超限");
}

}  // namespace

int main()
{
  const auto clips = findClips();
  if (clips.empty()) {
    std::printf("field replay: 没有带 labels.yaml 的片段，跳过（返回 %d）\n", kExitSkipped);
    return kExitSkipped;
  }
  const FieldConfig config = productionConfig();
  for (const auto & clip : clips) {judgeClip(clip, runClip(clip, config));}
  return finishTests("field replay");
}
