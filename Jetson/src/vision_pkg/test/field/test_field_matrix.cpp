// 安全区单帧矩阵：每例生成一小段静止序列跑完整 FieldLocator，按中线可见性把每帧标为
// 可判/边界/不可判，再按“抗干扰 / 准确 / 计时”三类判据检查。阈值见 scene_params.hpp。
// 用法：test_field_matrix [--report] [--filter 子串]
#include "check.hpp"
#include "field_scene.hpp"
#include "harness.hpp"
#include "metrics.hpp"
#include "visibility.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace {

using field_scene::Block;
using field_scene::Floor;
using field_scene::Scene;
using namespace field_test;

// 例的期望类别
enum class Kind
{
  SINGLE,          // 单因素正例：可判帧有效率 ≥ kMinValidRate
  COMBO,           // 组合正例：可判帧有效率 ≥ kMinValidRateCombo
  NEGATIVE_FIELD,  // 无完整场地：不得提交外框，不得输出分区
  NEGATIVE_SPLIT,  // 有外框无中线证据：不得输出分区
  STRESS,          // 超出现场条件的压力档：只报告
};

struct Case
{
  std::string group, name;
  Scene scene;
  Kind kind;
  double edge_tol;
  int seeds;
};

// ---------------- 矩阵定义 ----------------

class MatrixBuilder
{
public:
  MatrixBuilder & add(const std::string & group, const std::string & name, const Scene & s, Kind kind = Kind::SINGLE,
    double edge_tol = kEdgeLineTolPx, int seeds = kMatrixSeeds)
  {
    cases_.push_back({group, name, s, kind, edge_tol, seeds});
    return *this;
  }
  std::vector<Case> take() {return std::move(cases_);}

private:
  std::vector<Case> cases_;
};

void addIllumination(MatrixBuilder & m, const Scene & base)
{
  for (double e : {0.45, 0.7, 1.35, 1.6}) {
    Scene s = base; s.exposure = e;
    char name[32]; std::snprintf(name, sizeof(name), "增益%.2f", e);
    m.add("曝光", name, s);
  }
  const std::pair<const char *, cv::Vec3d> tints[] = {{"暖光", kTintWarm}, {"冷光", kTintCold}, {"荧光偏绿", kTintGreen}};
  for (const auto & [name, tint] : tints) {
    Scene s = base; s.tint = tint; s.awb_compensation = kAwbResidual;
    m.add("白平衡残差", name, s);
  }
  {Scene s = base; s.tint = kTintWarm; s.awb_compensation = kAwbResidual; s.exposure = 0.6; m.add("白平衡残差", "暖光+暗", s);}
  {Scene s = base; s.tint = kTintWarm; m.add("白平衡残差", "暖光(AWB锁定)", s, Kind::STRESS);}
}

void addReflectionShadow(MatrixBuilder & m, const Scene & base)
{
  {Scene s = base; s.glare = 2; m.add("反光", "高光斑", s);}
  {Scene s = base; s.glare = 1; s.glare_on_divider = true; m.add("反光", "高光压中线", s);}
  {Scene s = base; s.shadows = 1; s.shadow_strength = kShadowLight; m.add("阴影", "30%单块", s);}
  {Scene s = base; s.shadows = 1; s.shadow_strength = kShadowDark; m.add("阴影", "55%单块", s);}
  {Scene s = base; s.shadows = 2; s.shadow_strength = kShadowDark; s.gradient = 0.35; m.add("阴影", "55%双块+梯度", s);}
}

void addSurroundings(MatrixBuilder & m, const Scene & base)
{
  const std::pair<const char *, Floor> floors[] = {{"灰", Floor::GRAY}, {"木纹", Floor::WOOD}, {"绿地胶", Floor::RUBBER},
    {"强纹理", Floor::TEXTURE}, {"近色红", Floor::NEAR_RED}};
  for (const auto & [name, floor] : floors) {Scene s = base; s.floor = floor; m.add("地面", name, s);}
  {Scene s = base; s.clutter = 4; m.add("背景", "红紫杂物", s);}
  {
    Scene s = base;
    s.backdrop = {{-0.45, 0.15, 0.12, 0.10, kRed}, {0.0, 0.50, 0.30, 0.15, kPurple}, {0.45, 0.25, 0.12, 0.08, kRed}};
    m.add("背景", "近色背景", s);
  }
  {Scene s = base; s.divider_bgr = kLightGrayDivider; m.add("中线", "浅灰中线", s);}
  {Scene s = base; s.divider_bgr = kLightGrayDivider; s.exposure = 0.7; m.add("中线", "浅灰中线+暗", s);}
}

void addViewpoint(MatrixBuilder & m, const Scene & base)
{
  {Scene s = base; s.cam_d = 1.1; s.cam_h = 0.3; m.add("视角", "远", s);}
  {Scene s = base; s.cam_d = 1.5; s.cam_h = 0.35; m.add("视角", "更远", s);}
  {Scene s = base; s.cam_d = 0.3; s.cam_h = 0.18; m.add("视角", "近", s, Kind::SINGLE, kEdgeLineTolNearPx);}
  {Scene s = base; s.yaw_deg = 12; m.add("视角", "偏航12°", s);}
  {Scene s = base; s.cam_h = 0.12; m.add("视角", "低机位", s);}
  {Scene s = base; s.cam_x = 0.25; s.yaw_deg = 22; m.add("视角", "部分出画", s);}
}

void addImaging(MatrixBuilder & m, const Scene & base)
{
  for (int k : {7, 11, 21}) {Scene s = base; s.motion_blur = k; m.add("成像", "运动模糊" + std::to_string(k), s);}
  {Scene s = base; s.contrast = 0.55; m.add("成像", "低对比", s);}
  const std::pair<int, double> jpegs[] = {{30, 6.0}, {50, 4.0}, {70, 3.0}};
  for (const auto & [q, noise] : jpegs) {
    Scene s = base; s.jpeg = q; s.noise = noise;
    m.add("成像", "JPEG" + std::to_string(q) + "+噪声", s);
  }
}

void addCombinations(MatrixBuilder & m, const Scene & base)
{
  {Scene s = base; s.tint = kTintWarm; s.awb_compensation = kAwbResidual; s.glare = 1; s.shadows = 1;
   s.floor = Floor::WOOD; s.blocks = {{0.0, 0.18, kBlockSize, 0.05, kBlack}};
   m.add("组合", "暖光残差+反光+阴影+木纹+黑块压线", s, Kind::COMBO);}
  {Scene s = base; s.exposure = 0.6; s.contrast = 0.7; s.jpeg = 40; s.motion_blur = 7; s.clutter = 3;
   s.blocks = {{0.08, 0.22, kBlockSize, 0.05, kWhite}}; m.add("组合", "暗+低对比+模糊+杂物+白块", s, Kind::COMBO);}
  {Scene s = base; s.divider_bgr = kLightGrayDivider; s.shadows = 1; s.shadow_strength = kShadowLight; s.jpeg = 50;
   m.add("组合", "浅灰中线+30%阴影+JPEG50", s, Kind::COMBO);}
  {Scene s = base; s.floor = Floor::NEAR_RED; s.tint = kTintWarm; s.awb_compensation = kAwbResidual;
   s.blocks = {{0.12, 0.12, kBlockSize, 0.03, kRed}}; m.add("组合", "近色地面+暖光残差+红块", s, Kind::COMBO);}
}

const char * colourName(int i)
{
  static const char * names[] = {"黑", "白", "蓝", "红"};
  return names[i];
}

void addBlockGrid(MatrixBuilder & m, const Scene & base)
{
  static const char * rows[] = {"近", "中", "远"};
  static const char * offsets[] = {"压线", "贴线", "离线"};
  const cv::Scalar colours[] = {kBlack, kWhite, kBlue, kRed};
  for (int c = 0; c < 4; ++c) {
    for (double h : kBlockHeights) {
      for (int r = 0; r < 3; ++r) {
        for (int o = 0; o < 3; ++o) {
          Scene s = base;
          s.blocks = {{kBlockOffsets[o], kBlockRows[r], kBlockSize, h, colours[c]}};
          char name[64];
          std::snprintf(name, sizeof(name), "%s%dcm/%s/%s", colourName(c), int(h * 100 + 0.5), rows[r], offsets[o]);
          m.add("物块", name, s, Kind::SINGLE, kEdgeLineTolPx, kGridSeeds);
        }
      }
    }
  }
}

void addNegatives(MatrixBuilder & m, const Scene & base)
{
  {Scene s = base; s.tray = false; s.clutter = 6; m.add("负例", "无场地+红紫杂物", s, Kind::NEGATIVE_FIELD);}
  {Scene s = base; s.tray = false; s.floor = Floor::NEAR_RED;
   s.backdrop = {{-0.2, 0.2, 0.15, 0.10, kRed}, {0.2, 0.2, 0.15, 0.10, kPurple}};
   m.add("负例", "无场地+近色地面背景", s, Kind::NEGATIVE_FIELD);}
  {Scene s = base; s.interior = false; m.add("负例", "只有紫框", s, Kind::NEGATIVE_FIELD);}
  {Scene s = base; s.purple_frame = false; m.add("负例", "只有红板", s, Kind::NEGATIVE_FIELD);}
  {Scene s = base; s.divider = false; m.add("负例", "无中线", s, Kind::NEGATIVE_SPLIT);}
  {Scene s = base; s.divider = false;
   for (int r = 0; r < 3; ++r) {s.blocks.push_back({0.0, kBlockRows[r], kBlockSize, kBlockHeights[r + 1], kBlack});}
   m.add("负例", "无中线+黑块列", s, Kind::NEGATIVE_SPLIT);}
}

std::vector<Case> buildMatrix()
{
  MatrixBuilder m;
  const Scene base;
  m.add("基线", "白地面", base);
  addIllumination(m, base);
  addReflectionShadow(m, base);
  addSurroundings(m, base);
  addViewpoint(m, base);
  addImaging(m, base);
  addCombinations(m, base);
  addBlockGrid(m, base);
  addNegatives(m, base);
  return m.take();
}

// ---------------- 运行与统计 ----------------

struct Stats
{
  int frames = 0, committed = 0, valid = 0;
  int judgeable = 0, judgeable_valid = 0, unjudgeable = 0, unjudgeable_valid = 0, borderline = 0;
  int wrong_label = 0, predicted_as_measured = 0, edge_predicted = 0;
  std::vector<double> split_err, edge_dist, corner_err, center_err, ms;
  double min_visibility = 1.0, min_ends = 1.0, geometric = 0.0;
};

// 把一帧结果并入统计。
void scoreFrame(Stats & st, const FieldResult & r, const field_scene::Truth & truth, const InnerTruth & inner,
  const Visibility & vis, cv::Size size)
{
  ++st.frames;
  const bool valid = splitValid(r);
  st.committed += committedOutline(r);
  st.valid += valid;
  st.predicted_as_measured += predictedAsMeasured(r);
  st.edge_predicted += hasPredictedEdge(r);
  st.min_visibility = std::min(st.min_visibility, vis.total);
  st.min_ends = std::min({st.min_ends, vis.thirds[0], vis.thirds[2]});
  st.geometric = vis.geometric;
  switch (vis.judge) {
    case Judge::JUDGEABLE: ++st.judgeable; st.judgeable_valid += valid; break;
    case Judge::UNJUDGEABLE: ++st.unjudgeable; st.unjudgeable_valid += valid; break;
    case Judge::BORDERLINE: ++st.borderline; break;
  }
  if (committedOutline(r) && truth.present) {
    if (auto d = edgeLineDistance(r.corners, inner.corners, size, kEdgeSamples)) {st.edge_dist.push_back(*d);}
    if (auto e = inFrameCornerError(r.corners, inner.corners, size)) {st.corner_err.push_back(*e);}
  }
  if (!valid || !truth.present) {return;}
  const double err = splitLateralError(r, truth.split_far, truth.split_near);
  st.split_err.push_back(std::abs(err));
  st.wrong_label += std::abs(err) > kSplitLabelTolPx;
  const auto centers = halfCenters(r);
  if (!centers) {++st.wrong_label; return;}
  st.center_err.push_back(0.5 * (cv::norm((*centers)[0] - inner.left_center) + cv::norm((*centers)[1] - inner.right_center)));
}

// 第 k 个种子：独立噪声，相机位置按 0 为中心对称微扰。
Scene seededScene(const Case & c, int k, unsigned salt)
{
  Scene s = c.scene;
  const int shift = k - c.seeds / 2;
  s.seed = unsigned(k + 1) * 7919u + salt;
  s.cam_x += kSeedCamShiftX * shift;
  s.cam_d += kSeedCamShiftD * shift;
  return s;
}

void runCase(const Case & c, unsigned salt, const FieldConfig & config, Stats & st)
{
  const cv::Size size(config.proc_width, config.proc_height);
  for (int k = 0; k < c.seeds; ++k) {
    Scene scene = seededScene(c, k, salt);
    FieldLocator locator(config);
    const unsigned base_seed = scene.seed;
    for (int i = 0; i < kMatrixFrames; ++i) {
      scene.seed = base_seed * 31u + unsigned(i);
      field_scene::Truth truth;
      const cv::Mat image = field_scene::render(scene, size, truth);
      const auto t0 = std::chrono::steady_clock::now();
      const FieldResult r = locator.update(image);
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      if (i < kMatrixFrames - kMatrixScored) {continue;}
      st.ms.push_back(ms);
      scoreFrame(st, r, truth, innerTruth(scene, size), dividerVisibility(scene, size), size);
    }
  }
}

double mean(const std::vector<double> & v)
{
  double s = 0;
  for (double x : v) {s += x;}
  return v.empty() ? 0.0 : s / v.size();
}

// ---------------- 判据 ----------------

struct Verdict
{
  int robust = 0, accuracy = 0;   // 各类失败条数
};

void fail(int & counter, const char * dimension, const Case & c, const char * what)
{
  ++counter;
  ++g_failed;
  std::fprintf(stderr, "FAIL [%s] %s/%s：%s\n", dimension, c.group.c_str(), c.name.c_str(), what);
}

void pass() {++g_passed;}

void judgeRobustness(const Case & c, const Stats & st, Verdict & v)
{
  char msg[160];
  if (c.kind == Kind::STRESS) {return;}
  if (c.kind == Kind::NEGATIVE_FIELD) {
    std::snprintf(msg, sizeof(msg), "负例外框提交 %d 帧、分区 %d 帧", st.committed, st.valid);
    (st.committed + st.valid > kMaxFalseOnNegative) ? fail(v.robust, "抗干扰", c, msg) : pass();
    return;
  }
  if (c.kind == Kind::NEGATIVE_SPLIT) {
    std::snprintf(msg, sizeof(msg), "无中线却输出分区 %d 帧", st.valid);
    (st.valid > kMaxFalseOnNegative) ? fail(v.robust, "抗干扰", c, msg) : pass();
    return;
  }
  const double need = c.kind == Kind::COMBO ? kMinValidRateCombo : kMinValidRate;
  if (st.judgeable > 0) {
    std::snprintf(msg, sizeof(msg), "可判帧有效率 %d/%d < %.2f", st.judgeable_valid, st.judgeable, need);
    (ratio(st.judgeable_valid, st.judgeable) < need) ? fail(v.robust, "抗干扰", c, msg) : pass();
  }
  std::snprintf(msg, sizeof(msg), "不可判帧仍输出分区 %d/%d", st.unjudgeable_valid, st.unjudgeable);
  (st.unjudgeable_valid > kMaxFalseOnUnjudgeable) ? fail(v.robust, "抗干扰", c, msg) : pass();
  std::snprintf(msg, sizeof(msg), "分区标签错误 %d 帧（误差 > %.0f px）", st.wrong_label, kSplitLabelTolPx);
  (st.wrong_label > 0) ? fail(v.robust, "抗干扰", c, msg) : pass();
  std::snprintf(msg, sizeof(msg), "预测冒充测量 %d 帧", st.predicted_as_measured);
  (st.predicted_as_measured > kMaxPredictedAsMeasured) ? fail(v.robust, "抗干扰", c, msg) : pass();
}

void judgeAccuracy(const Case & c, const Stats & st, Verdict & v)
{
  if (c.kind != Kind::SINGLE && c.kind != Kind::COMBO) {return;}
  char msg[160];
  if (!st.split_err.empty()) {
    std::snprintf(msg, sizeof(msg), "中线横向误差均值 %.2f px > %.1f", mean(st.split_err), kSplitErrMaxPx);
    (mean(st.split_err) > kSplitErrMaxPx) ? fail(v.accuracy, "准确", c, msg) : pass();
  }
  if (!st.edge_dist.empty()) {
    std::snprintf(msg, sizeof(msg), "边到内沿线距离均值 %.2f px > %.1f", mean(st.edge_dist), c.edge_tol);
    (mean(st.edge_dist) > c.edge_tol) ? fail(v.accuracy, "准确", c, msg) : pass();
  }
  if (kGateCenterError && !st.center_err.empty()) {
    std::snprintf(msg, sizeof(msg), "半区中心误差均值 %.2f px > %.1f", mean(st.center_err), kCenterTolPx);
    (mean(st.center_err) > kCenterTolPx) ? fail(v.accuracy, "准确", c, msg) : pass();
  }
}

const char * kindName(Kind k)
{
  switch (k) {
    case Kind::SINGLE: return "单因素";
    case Kind::COMBO: return "组合";
    case Kind::NEGATIVE_FIELD: return "负例";
    case Kind::NEGATIVE_SPLIT: return "负例中线";
    case Kind::STRESS: return "压力";
  }
  return "?";
}

std::string fmtOpt(const std::vector<double> & v)
{
  if (v.empty()) {return "-";}
  char b[16]; std::snprintf(b, sizeof(b), "%.2f", mean(v));
  return b;
}

void printRow(const Case & c, const Stats & st)
{
  std::printf("%-40s %-6s %3d/%-3d %3d/%-3d 判%2d/%-2d 不%2d/%-2d 边%-2d 错%-2d 预%-2d 补边%-2d %6s %6s %6s %6s %4.2f/%4.2f/%4.2f\n",
    (c.group + "/" + c.name).c_str(), kindName(c.kind), st.committed, st.frames, st.valid, st.frames,
    st.judgeable_valid, st.judgeable, st.unjudgeable_valid, st.unjudgeable, st.borderline, st.wrong_label,
    st.predicted_as_measured, st.edge_predicted, fmtOpt(st.split_err).c_str(), fmtOpt(st.edge_dist).c_str(),
    fmtOpt(st.corner_err).c_str(), fmtOpt(st.center_err).c_str(), st.geometric, st.min_visibility, st.min_ends);
}

void printHeader()
{
  std::printf("%-40s %-6s %-7s %-7s %-8s %-8s %-4s %-4s %-4s %-5s %6s %6s %6s %6s %s\n", "场景", "类别", "外框", "分区",
    "可判有效", "不可判有效", "边界", "错标", "预测", "补边", "中线px", "边线px", "角点px", "中心px",
    "几何可见/最低可见/最低端段");
}

}  // namespace

int main(int argc, char ** argv)
{
  bool report = false;
  std::string filter;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--report") == 0) {report = true;}
    if (std::strcmp(argv[i], "--filter") == 0 && i + 1 < argc) {filter = argv[++i];}
  }
  const FieldConfig config = productionConfig();
  const auto cases = buildMatrix();
  if (report) {printHeader();}
  Verdict verdict;
  std::vector<double> all_ms;
  std::map<std::string, std::pair<int, int>> judge_totals;   // 组 → (可判有效, 可判)
  for (size_t i = 0; i < cases.size(); ++i) {
    const Case & c = cases[i];
    if (!filter.empty() && (c.group + "/" + c.name).find(filter) == std::string::npos) {continue;}
    Stats st;
    runCase(c, unsigned(i), config, st);
    all_ms.insert(all_ms.end(), st.ms.begin(), st.ms.end());
    auto & jt = judge_totals[c.group];
    jt.first += st.judgeable_valid; jt.second += st.judgeable;
    if (report) {printRow(c, st);}
    judgeRobustness(c, st, verdict);
    judgeAccuracy(c, st, verdict);
  }
  std::printf("[汇总] 可判帧有效率（按组）：");
  for (const auto & [g, t] : judge_totals) {std::printf(" %s %d/%d", g.c_str(), t.first, t.second);}
  std::printf("\n[汇总] 抗干扰失败 %d 条，准确失败 %d 条\n", verdict.robust, verdict.accuracy);
  reportTiming("矩阵", median(all_ms)) ? pass() : (void)++g_failed;
  return finishTests("field matrix");
}
