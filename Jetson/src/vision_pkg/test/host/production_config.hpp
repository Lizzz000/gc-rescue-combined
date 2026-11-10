#pragma once
// 与 rescue_advanced.yaml 的 vision.field 一致：run.sh 先用 field_config.py 展开生产参数。
#include "field/field.hpp"

inline FieldConfig productionConfig()
{
  FieldConfig c;
#define SET(name, value) c.name = value;
#include "field_config_values.inc"
#undef SET
  validateFieldConfig(c);
  return c;
}
