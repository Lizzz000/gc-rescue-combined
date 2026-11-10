// 三个安全区测试共用的装配：读 yaml 生成的生产参数、相对计时比较。
#pragma once

#include "scene_params.hpp"
#include "field/field.hpp"

#include <cstdio>
#include <cstdlib>
#include <optional>

namespace field_test {

inline FieldConfig productionConfig()
{
  FieldConfig c;
#define SET(name, value) c.name = value;
#define DETECT(name, value)
#include "field_config_values.inc"
#undef DETECT
#undef SET
  validateFieldConfig(c);
  return c;
}

struct DetectValues
{
  double field_max_fps = 0.0;
};

inline DetectValues detectValues()
{
  DetectValues d;
#define SET(name, value)
#define DETECT(name, value) d.name = value;
#include "field_config_values.inc"
#undef DETECT
#undef SET
  return d;
}

// 计时只做本机相对比较：参考值由环境变量给出（通常是同机跑基线提交的中位数）。
inline std::optional<double> timingReferenceMs()
{
  const char * text = std::getenv(kTimingReferenceEnv);
  if (text == nullptr || *text == '\0') {return std::nullopt;}
  return std::atof(text);
}

// 返回 true 表示通过或无参考值；打印中位数与参考值。
inline bool reportTiming(const char * label, double median_ms)
{
  const auto reference = timingReferenceMs();
  if (!reference) {
    std::printf("[计时] %s 中位 %.2f ms/帧（无参考值，只报告；绝对值需板端）\n", label, median_ms);
    return true;
  }
  const bool ok = median_ms <= *reference * kTimingRegressionRatio;
  std::printf("[计时] %s 中位 %.2f ms/帧，参考 %.2f ms，上限 ×%.2f：%s\n", label, median_ms, *reference,
    kTimingRegressionRatio, ok ? "通过" : "退化");
  return ok;
}

}  // namespace field_test
