#pragma once
// 检测节点读取场地参数。纯几何仍在 field/，不在这里。

#include "field/field.hpp"

// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// ROS 适配只维护字段名和类型；默认值来自统一配置，纯场地核心不读取配置文件。
#include <interface_pkg/app/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <stdexcept>
#include <string>

// 字段清单在 field/field.hpp 的 VISION_FIELD_PARAMETERS；这里只定义声明和读取。

inline void declareFieldParameters(rclcpp::Node & node)
{
#define DECLARE_FIELD(kind, name) rescue::declareRequired<kind>(node, "field." #name);
  VISION_FIELD_PARAMETERS(DECLARE_FIELD)
#undef DECLARE_FIELD
}

// 先检查 ROS int64 的表示范围，再收窄到纯值核心的 int。
inline int fieldInteger(const rclcpp::Node & node, const char * key)
{
  const auto value = node.get_parameter(key).as_int();
  if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
    throw rescue::ConfigurationError(std::string(key) + " exceeds the field integer range");
  }
  return static_cast<int>(value);
}
inline int fieldValue(const rclcpp::Node & node, const char * key, int *)
{return fieldInteger(node, key);}
inline double fieldValue(const rclcpp::Node & node, const char * key, double *)
{return node.get_parameter(key).as_double();}
inline bool fieldValue(const rclcpp::Node & node, const char * key, bool *)
{return node.get_parameter(key).as_bool();}

// 先完成所有有类型的赋值，再统一校验跨字段关系；错误转换为配置异常，阻止资源启动。
inline FieldConfig readFieldParameters(const rclcpp::Node & node)
{
  FieldConfig config;
#define READ_FIELD(kind, name) config.name = fieldValue(node, "field." #name, static_cast<kind *>(nullptr));
  VISION_FIELD_PARAMETERS(READ_FIELD)
#undef READ_FIELD
  try {
    validateFieldConfig(config);
  } catch (const std::invalid_argument & error) {
    throw rescue::ConfigurationError(std::string("invalid field parameters: ") + error.what());
  }
  return config;
}
