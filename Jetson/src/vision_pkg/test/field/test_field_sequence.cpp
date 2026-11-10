// 安全区序列测试：只检查分区层逐帧如实输出——静止抖动、有效↔无效翻转、注入后恢复、
// 不拿预测冒充测量。保持（hold）属于 bridge 层，由 bridge 的 test_safety_hold 负责，这里不测。
// 用法：test_field_sequence [--report]
#include "check.hpp"
#include "field_scene.hpp"
#include "harness.hpp"
#include "metrics.hpp"
#include "visibility.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iterator>
#include <string>
#include <vector>

namespace {

using field_scene::Scene;
using namespace field_test;

// 每帧的场景由调度函数给出：返回该帧的场景与“是否注入帧”。
struct FrameSpec
{
  Scene scene;
  bool injected = false;
};
using Schedule = std::function<FrameSpec(int)>;

struct FrameLog
{
  bool valid = false, injected = false, predicted = false;
  double exposure = 1.0;
  Judge judge = Judge::UNJUDGEABLE;
  double split_err = 0.0;
  std::vector<cv::Point2f> corners;
  double ms = 0.0;
};

std::vector<FrameLog> runSequence(const Schedule & schedule, int frames, const FieldConfig & config)
{
  const cv::Size size(config.proc_width, config.proc_height);
  FieldLocator locator(config);
  std::vector<FrameLog> log;
  for (int i = 0; i < frames; ++i) {
    FrameSpec spec = schedule(i);
    spec.scene.seed = 1000003u + unsigned(i);
    field_scene::Truth truth;
    const cv::Mat image = field_scene::render(spec.scene, size, truth);
    const auto t0 = std::chrono::steady_clock::now();
    const FieldResult r = locator.update(image);
    FrameLog f;
    f.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    f.injected = spec.injected;
    f.exposure = spec.scene.exposure;
    f.valid = splitValid(r);
    f.predicted = predictedAsMeasured(r);
    f.judge = dividerVisibility(spec.scene, size).judge;
    if (f.valid) {f.split_err = splitLateralError(r, truth.split_far, truth.split_near);}
    if (committedOutline(r)) {f.corners = r.corners;}
    log.push_back(f);
  }
  return log;
}

int warmupFrames(const FieldConfig & config)
{
  return config.locate_hits_needed + kWarmupMargin;
}

std::vector<FrameLog> scored(const std::vector<FrameLog> & log, const FieldConfig & config)
{
  return {log.begin() + std::min<size_t>(log.size(), warmupFrames(config)), log.end()};
}

std::vector<bool> validSeries(const std::vector<FrameLog> & log)
{
  std::vector<bool> v;
  for (const auto & f : log) {v.push_back(f.valid);}
  return v;
}

int countPredicted(const std::vector<FrameLog> & log)
{
  int n = 0;
  for (const auto & f : log) {n += f.predicted;}
  return n;
}

// 静止时四角 x/y 标准差的均值（只用全部帧都提交的角点序列）。
double cornerJitter(const std::vector<FrameLog> & log)
{
  double sum = 0;
  int n = 0;
  for (int k = 0; k < 4; ++k) {
    std::vector<double> xs, ys;
    for (const auto & f : log) {
      if (f.corners.size() != 4) {continue;}
      xs.push_back(f.corners[k].x);
      ys.push_back(f.corners[k].y);
    }
    if (xs.size() < 2) {continue;}
    sum += stddev(xs) + stddev(ys);
    n += 2;
  }
  return n ? sum / n : 0.0;
}

double flipsPerMinute(int flips, size_t frames, double fps)
{
  return frames && fps > 0 ? flips * 60.0 * fps / double(frames) : 0.0;
}

void expect(bool ok, const char * dimension, const std::string & what)
{
  if (ok) {++g_passed; return;}
  ++g_failed;
  std::fprintf(stderr, "FAIL [%s] %s\n", dimension, what.c_str());
}

std::string fmt(const char * f, double a, double b = 0, double c = 0)
{
  char buf[200];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

// ---- 1. 静止：抖动、翻转、预测冒充 ----
void testStatic(const FieldConfig & config, const DetectValues & detect, std::vector<double> & ms)
{
  Group group("field sequence static");
  const Scene base;
  const auto log = scored(runSequence([&](int) {return FrameSpec{base, false};}, kSequenceFrames, config), config);
  std::vector<double> split;
  for (const auto & f : log) {if (f.valid) {split.push_back(f.split_err);} ms.push_back(f.ms);}
  const int flips = countFlips(validSeries(log));
  const double sj = stddev(split), cj = cornerJitter(log);
  std::printf("[静止] 帧 %zu，有效 %zu，中线抖动 %.3f px，角点抖动 %.3f px，翻转 %d（%.1f 次/分钟 @%.0f fps），预测冒充 %d\n",
    log.size(), split.size(), sj, cj, flips, flipsPerMinute(flips, log.size(), detect.field_max_fps),
    detect.field_max_fps, countPredicted(log));
  expect(split.size() == log.size(), "稳定", fmt("静止可判帧有效 %.0f/%.0f", split.size(), log.size()));
  expect(sj <= kMaxSplitJitterPx, "稳定", fmt("中线抖动 %.3f px > %.1f", sj, kMaxSplitJitterPx));
  expect(cj <= kMaxCornerJitterPx, "稳定", fmt("角点抖动 %.3f px > %.1f", cj, kMaxCornerJitterPx));
  expect(flips <= kMaxFlipsStatic, "稳定", fmt("静止翻转 %.0f 次 > %.0f", flips, kMaxFlipsStatic));
  expect(countPredicted(log) <= kMaxPredictedAsMeasured, "稳定", fmt("预测冒充测量 %.0f 帧", countPredicted(log)));
}

// ---- 2. 注入单帧/两帧失明：注入帧必须如实无效，结束后立即恢复 ----
Schedule injectionSchedule()
{
  return [](int i) {
      FrameSpec spec;
      const int phase = i % kInjectEvery, burst = i / kInjectEvery;
      const int length = kInjectLengths[burst % (sizeof(kInjectLengths) / sizeof(kInjectLengths[0]))];
      spec.injected = i >= kInjectEvery && phase < length;
      if (spec.injected) {spec.scene.exposure = kInjectExposure;}
      return spec;
    };
}

void testInjection(const FieldConfig & config, const DetectValues & detect)
{
  Group group("field sequence injection");
  const auto log = scored(runSequence(injectionSchedule(), kSequenceFrames, config), config);
  int bursts = 0, honest_fail = 0, recover_fail = 0, unjudgeable_injected = 0;
  for (size_t i = 0; i < log.size(); ++i) {
    const auto & f = log[i];
    if (f.injected) {
      unjudgeable_injected += f.judge == Judge::UNJUDGEABLE;
      honest_fail += f.valid && f.judge == Judge::UNJUDGEABLE;
      continue;
    }
    if (i == 0 || !log[i - 1].injected) {continue;}
    // 注入刚结束：kMaxRecoverFrames 帧内必须恢复有效
    ++bursts;
    bool recovered = false;
    for (size_t k = i; k < std::min(log.size(), i + kMaxRecoverFrames); ++k) {recovered |= log[k].valid;}
    recover_fail += !recovered;
  }
  const int flips = countFlips(validSeries(log));
  std::printf("[注入] 注入 %d 次（不可判注入帧 %d），注入帧仍有效 %d，未在 %d 帧内恢复 %d，翻转 %d（%.1f 次/分钟），预测冒充 %d\n",
    bursts, unjudgeable_injected, honest_fail, kMaxRecoverFrames, recover_fail, flips,
    flipsPerMinute(flips, log.size(), detect.field_max_fps), countPredicted(log));
  expect(bursts > 0 && unjudgeable_injected > 0, "稳定", "注入未生效（注入帧真值不是不可判）");
  expect(honest_fail == 0, "稳定", fmt("不可判注入帧仍输出分区 %.0f 帧", honest_fail));
  expect(recover_fail == 0, "稳定", fmt("注入结束后未及时恢复 %.0f 次", recover_fail));
  expect(flips <= 2 * bursts, "稳定", fmt("翻转 %.0f 次 > 注入次数×2 = %.0f", flips, 2 * bursts));
  expect(countPredicted(log) <= kMaxPredictedAsMeasured, "稳定", fmt("预测冒充测量 %.0f 帧", countPredicted(log)));
}

// ---- 3. 自动曝光爬坡与跳变 ----
Schedule autoExposureSchedule()
{
  return [](int i) {
      FrameSpec spec;
      const int ramp = 2 * kAeRampFrames;
      if (i < ramp) {
        const double t = std::abs(i - kAeRampFrames) / double(kAeRampFrames);   // 1 → 0 → 1
        spec.scene.exposure = kAeRampLow + (1.0 - kAeRampLow) * t;
      } else if (i < ramp + kAeJumpFrames) {
        spec.scene.exposure = kAeJumpGain;
      }
      return spec;
    };
}

void testAutoExposure(const FieldConfig & config, const DetectValues & detect)
{
  Group group("field sequence auto exposure");
  const int frames = 2 * kAeRampFrames + 2 * kAeJumpFrames;
  const auto log = scored(runSequence(autoExposureSchedule(), frames, config), config);
  int judgeable = 0, judgeable_valid = 0, unjudgeable_valid = 0, wrong = 0;
  for (const auto & f : log) {
    judgeable += f.judge == Judge::JUDGEABLE;
    judgeable_valid += f.judge == Judge::JUDGEABLE && f.valid;
    unjudgeable_valid += f.judge == Judge::UNJUDGEABLE && f.valid;
    wrong += f.valid && std::abs(f.split_err) > kSplitLabelTolPx;
  }
  // 按增益分档统计可判帧有效率，定位失效区间
  const auto & bins = kAeReportBins;
  for (size_t b = 0; b + 1 < std::size(bins); ++b) {
    int n = 0, ok = 0;
    for (const auto & f : log) {
      if (f.judge != Judge::JUDGEABLE || f.exposure < bins[b] || f.exposure >= bins[b + 1]) {continue;}
      ++n; ok += f.valid;
    }
    if (n) {std::printf("[自动曝光] 增益 [%.2f, %.2f) 可判有效 %d/%d\n", bins[b], bins[b + 1], ok, n);}
  }
  const int flips = countFlips(validSeries(log));
  std::printf("[自动曝光] 可判有效 %d/%d，不可判仍有效 %d，错标 %d，翻转 %d（%.1f 次/分钟），预测冒充 %d\n", judgeable_valid,
    judgeable, unjudgeable_valid, wrong, flips, flipsPerMinute(flips, log.size(), detect.field_max_fps), countPredicted(log));
  expect(ratio(judgeable_valid, judgeable) >= kMinValidRateAe, "抗干扰",
    fmt("爬坡/跳变可判有效率 %.0f/%.0f < %.2f", judgeable_valid, judgeable, kMinValidRateAe));
  expect(unjudgeable_valid == 0, "抗干扰", fmt("不可判帧仍输出分区 %.0f", unjudgeable_valid));
  expect(wrong == 0, "抗干扰", fmt("错标 %.0f 帧", wrong));
  expect(countPredicted(log) <= kMaxPredictedAsMeasured, "稳定", fmt("预测冒充测量 %.0f 帧", countPredicted(log)));
}

}  // namespace

int main()
{
  const FieldConfig config = productionConfig();
  const DetectValues detect = detectValues();
  std::vector<double> ms;
  testStatic(config, detect, ms);
  testInjection(config, detect);
  testAutoExposure(config, detect);
  reportTiming("静止序列", median(ms)) ? (void)++g_passed : (void)++g_failed;
  return finishTests("field sequence");
}
