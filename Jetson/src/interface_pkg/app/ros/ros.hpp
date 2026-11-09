#pragma once
#include <interface_pkg/app/core/core.hpp>
// ---- 识别 ROS 边界 ----
// 职责：把 ROS 消息转成纯值，不代替业务准入。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <cstdint>
#include <string>
#include <builtin_interfaces/msg/time.hpp>
#include <interface_pkg/msg/mission_status.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/object_hypothesis.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// 接收端共用的 ROS 无状态校验；框与类别委托纯值层，阶段截止和超时历史由各节点持有。
struct RecognitionValidation
{
  // 五个枚举均从 0 连续编号；只验证范围，不判断时间、水位或业务就绪。
  // 接收者必须在推进自己的状态水位前调用，避免非法新状态挡住合法旧状态。
  static bool missionStatusEnums(const interface_pkg::msg::MissionStatus & value) noexcept
  {
    using MissionStatus = interface_pkg::msg::MissionStatus;
    return value.destination <= MissionStatus::DEST_RIGHT &&
           value.operation <= MissionStatus::OP_SWITCHING &&
           value.outcome <= MissionStatus::RESULT_ROI_INVALID &&
           value.transport_phase <= MissionStatus::TRANSPORT_LATER &&
           value.cycle_phase <= MissionStatus::CYCLE_FAULT;
  }

  static bool sourceTime(
    const builtin_interfaces::msg::Time & value, std::int64_t now_ns, std::int64_t & ns)
  {
    // ns 是源端 ROS 时间的纳秒值；这里只拒绝零、未来和畸形时间，不判断节点自己的超时。
    // 仅在返回 true 时使用 ns；now_ns 必须来自同一 ROS 时钟，不能传 steady_clock 的绝对时间。
    if (value.sec < 0 || value.nanosec >= 1000000000u) {return false;}
    ns = static_cast<std::int64_t>(value.sec) * 1000000000LL + value.nanosec;
    return ns > 0 && ns <= now_ns;
  }

  static bool axisAlignedBox(const vision_msgs::msg::BoundingBox2D & box)
  {
    return RecognitionValueValidation::axisAlignedBox(
      {box.center.position.x, box.center.position.y, box.size_x, box.size_y, box.center.theta});
  }
};

// 将 ROS 识别输入逐项转换为纯值；保留输入顺序，转换不代替业务准入。
struct RecognitionRosConversion
{
  // 只提取核心需要的字段，保留缺失假设和输入位置，由具体算法判断有效性。
  static RecognitionDetection detection(
    const vision_msgs::msg::Detection2D & input, std::size_t index = 0)
  {
    RecognitionDetection result;
    result.id = input.id;
    result.input_index = index;
    const auto & box = input.bbox;
    result.box = {box.center.position.x, box.center.position.y,
      box.size_x, box.size_y, box.center.theta};
    // 核心只使用第一条假设；完整 results 和 pose 仍留在原 ROS 消息，选择转发时不会丢失。
    if (!input.results.empty()) {
      const auto & first = input.results.front().hypothesis;
      result.hypothesis = RecognitionHypothesis{first.class_id, first.score};
    }
    return result;
  }

  // 不校验整帧或去重 ID；每项保留原始索引，包括非法框，由 ROI、跟踪、选择核心处理。
  static RecognitionDetections detections(const vision_msgs::msg::Detection2DArray & input)
  {
    RecognitionDetections result;
    result.reserve(input.detections.size());
    for (std::size_t i = 0; i < input.detections.size(); ++i) {
      result.push_back(detection(input.detections[i], i));
    }
    return result;
  }
};

// ---- 运行参数 ----
// 职责：缺参即配置错误，不在节点里另存默认值。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace rescue
{
// 配置错误与设备/运行故障分开报告，入口在创建业务资源前拒绝不完整参数。
class ConfigurationError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

// 生产值只能由统一配置提供；不在节点声明中维护另一套数字默认值。
template<class T>
T declareRequired(
  rclcpp::Node & node, const char * name,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor =
    rcl_interfaces::msg::ParameterDescriptor{})
{
  // 类型由模板固定，值取 ROS override；未提供值或类型不符均转为可定位的配置错误。
  // 这里只声明单项参数，不代替统一装配层的范围、路径与跨节点连接校验。
  try {
    return node.declare_parameter<T>(name, descriptor);
  } catch (const std::exception & error) {
    throw ConfigurationError(std::string("required parameter '") + name + "': " + error.what());
  }
}
}  // namespace rescue

// ---- 运行入口 ----
// 职责：按角色启动单个节点，不创建业务线程。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <csignal>
#include <exception>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/contexts/default_context.hpp>

namespace rescue
{
// 角色表只保存名称和真实入口函数；分发本身不创建 ROS 节点或业务线程。
struct RuntimeRole
{
  const char * name;
  int (*run)(int, char **);
};

// SIGINT 可在节点已加入 ROS 图、但仍创建服务时关闭 context。
// context 已停止时接受 NOT_INIT，其他 ROS 故障仍失败。
inline bool isShutdownContextError(const rclcpp::exceptions::RCLError & error)
{
  if (rclcpp::ok()) {return false;}
  return error.ret == RCL_RET_NOT_INIT;
}

// 独立角色沿用一个单线程执行器；节点先析构，再最终关闭ROS上下文。
template<class Node>
int runNodeRole(int argc, char ** argv)
{
  // 退出码：0 正常/认可的信号停止，2 配置错误，1 其他运行异常。
  int result = 0;
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();
    if (rclcpp::ok()) {rclcpp::spin(node);}
  } catch (const ConfigurationError & error) {
    std::cerr << "configuration: " << error.what() << '\n';
    result = 2;
  } catch (const rclcpp::exceptions::RCLError & error) {
    if (!isShutdownContextError(error)) {
      std::cerr << "runtime ROS failure: " << error.what() << '\n';
      result = 1;
    }
  } catch (...) {
    std::cerr << "runtime failure\n";
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

// --role属于程序，--ros-args之后完整交给ROS；不提供旧可执行名称或隐式角色。
inline int dispatchRuntime(int argc, char ** argv, std::initializer_list<RuntimeRole> roles)
{
  std::string selected;
  // 借用 argv 指针，仅删除程序自己的 --role 参数；调用真实入口期间原 argv 仍有效。
  std::vector<char *> ros_arguments;
  if (argc > 0) {ros_arguments.push_back(argv[0]);}
  bool ros_section = false;
  bool seen_role = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "--ros-args") {ros_section = true;}
    if (!ros_section && argument == "--role") {
      if (seen_role || i + 1 >= argc) {
        std::cerr << "configuration: --role must occur once with a value\n";
        return 2;
      }
      seen_role = true;
      selected = argv[++i];
    } else if (!ros_section && argument == "--help") {
      std::cout << "Required: --role <";
      const char * separator = "";
      for (const auto & role : roles) {
        std::cout << separator << role.name;
        separator = "|";
      }
      std::cout << "> [--ros-args ...]\n";
      return 0;
    } else if (!ros_section) {
      std::cerr << "configuration: unknown runtime argument '" << argument << "'\n";
      return 2;
    } else {
      ros_arguments.push_back(argv[i]);
    }
  }
  if (selected.empty()) {
    std::cerr << "configuration: --role is required\n";
    return 2;
  }
  for (const auto & role : roles) {
    if (selected == role.name) {
      // nohup等父级可能忽略SIGINT；进入ROS前恢复，随后由ROS登记自己的停止处理。
      std::signal(SIGINT, SIG_DFL);
      std::signal(SIGTERM, SIG_DFL);
      // 保留 argv[argc] 的空指针约定；角色拒绝发生在这一初始化边界之前。
      ros_arguments.push_back(nullptr);
      return role.run(static_cast<int>(ros_arguments.size() - 1), ros_arguments.data());
    }
  }
  std::cerr << "configuration: unsupported --role '" << selected << "'\n";
  return 2;
}
}  // namespace rescue
