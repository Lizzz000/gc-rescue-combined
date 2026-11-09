// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// vision_runtime 进程入口：--role vision（同进程组合 DetectNode + StreamNode）/ detect / stream。
// 阅读：vision 同时跑检测和图传；detect、stream 只跑其中一个。
// 检测发布分割和场地结果，图传订阅这些结果。这里不选择目标，也不发 CAN。
// 下一份：node/detect_node.hpp。分割在 yolo/yolo.hpp，场地在 field/field.hpp。
// 图传接着看 node/stream_node.hpp，HTTP 在 stream/mjpeg/mjpeg.cpp。
#include <atomic>
#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>

#include <interface_pkg/app/core.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include "node/detect_node.hpp"
#include "node/stream_node.hpp"

namespace
{
rclcpp::NodeOptions nodeOptions()
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  return options;
}

/// 单线程执行器驱动一个节点：与单独进程里的默认执行器等价，收到 Ctrl-C 后 spin_once 返回并退出循环。
/// 图像进程内订阅建立时仅打印一次自检，优先报告默认的处理帧路径，仍兼容显式原图来源。
/// 两个节点以裸指针传入：线程参数不能绑定 shared_ptr 转换产生的临时量，节点由 main 持有到 join 之后。
/// 异常只保存到本线程专属的 error；先通知另一执行器退出，再由 main 在 join 后重新抛出并报告失败。
void runExecutor(
  rclcpp::Node * node, const char * name, DetectNode * detect_node,
  std::atomic<bool> & spinning, std::exception_ptr & error)
{
  try {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    bool reported_intra_process = false;
    while (spinning.load() && rclcpp::ok()) {
      if (detect_node && !reported_intra_process) {
        std::size_t intra = detect_node->processedIntraProcessSubscribers();
        const char * topic = "~/image_processed";
        if (intra == 0) {
          intra = detect_node->rawIntraProcessSubscribers();
          topic = "~/image_raw";
        }
        if (intra > 0) {
          reported_intra_process = true;
          RCLCPP_INFO(detect_node->get_logger(),
                      "intra-process check: %zu intra-process subscriber(s) on %s",
                      intra, topic);
        }
      }
      executor.spin_once(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node->get_logger(), "%s stopped spinning", name);
  } catch (...) {
    error = std::current_exception();
    spinning.store(false);
  }
}
}  // namespace

int runVisionComposition(int argc, char ** argv)
{
  int result = 0;
  try {
    // 初始化也属于入口异常边界；无效 ROS 参数不能逃出 main 后触发未捕获异常。
    rclcpp::init(argc, argv);
    // 组合入口持有节点，必须存活到两个执行器和各自后台线程回收完成。
    auto detect_node = std::make_shared<DetectNode>(nodeOptions());
    // 已有引擎加载或节点初始化期间收到停止后，不再构造第二节点或开启 HTTP 服务。
    if (!rclcpp::ok()) {
      detect_node.reset();
      rclcpp::shutdown();
      return 0;
    }
    auto stream_node = std::make_shared<StreamNode>(nodeOptions());
    if (!rclcpp::ok()) {
      detect_node.reset();
      stream_node.reset();
      rclcpp::shutdown();
      return 0;
    }

    // 两个节点各自在线程里被自己的单线程执行器驱动；异常和 SIGINT 都先 join，再销毁节点。
    // stream_node 构造时已经登记订阅；执行器开始驱动回调后，检测端自检与图传接收都能运行。
    std::atomic<bool> spinning{true};
    std::exception_ptr stream_error, detect_error;
    std::thread stream_thread, detect_thread;
    try {
      stream_thread = std::thread(
        runExecutor, stream_node.get(), "stream_node", nullptr,
        std::ref(spinning), std::ref(stream_error));
      detect_thread = std::thread(
        runExecutor, detect_node.get(), "detect_node", detect_node.get(),
        std::ref(spinning), std::ref(detect_error));
      detect_thread.join();
      stream_thread.join();
    } catch (...) {
      // 第二个线程创建失败时，第一个线程仍可能在运行；必须先停并 join，避免 std::thread 析构触发 terminate。
      spinning.store(false);
      if (detect_thread.joinable()) {detect_thread.join();}
      if (stream_thread.joinable()) {stream_thread.join();}
      throw;
    }
    if (stream_error) {std::rethrow_exception(stream_error);}
    if (detect_error) {std::rethrow_exception(detect_error);}

    // 销毁节点会停止各工作线程与图传服务；SIGINT 已经关闭 ROS 上下文时也必须完成这一步。
    // 正常退出先检测后图传：先停止源帧产生，再回收图传队列与 HTTP 服务。
    // 上方异常路径同样先 join，再由 RAII 按构造逆序回收图传与检测节点。
    detect_node.reset();
    stream_node.reset();
  } catch (const rescue::ConfigurationError & error) {
    std::cerr << "configuration: " << error.what() << '\n';
    result = 2;
  } catch (const rclcpp::exceptions::RCLError & error) {
    // 与独立入口共用窄判断，只正常结束信号停止造成的context失效。
    if (!rescue::isShutdownContextError(error)) {
      RCLCPP_FATAL(rclcpp::get_logger("vision_runtime"), "%s", error.what());
      result = 1;
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_runtime"), "%s", error.what());
    result = 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_runtime"), "unknown executor failure");
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

// 独立 detect/stream 角色复用相同节点实现；参数仍须由统一装配明确提供，不在此补生产默认值。
int runDetectRole(int argc, char ** argv)
{return rescue::runNodeRole<DetectNode>(argc, argv);}
int runStreamRole(int argc, char ** argv)
{return rescue::runNodeRole<StreamNode>(argc, argv);}

// role 只选择装配，节点名、ROS 接口和组合进程内的执行器结构保持原样。
int main(int argc, char ** argv)
{
  return rescue::dispatchRuntime(argc, argv, {
    {"vision", runVisionComposition}, {"detect", runDetectRole}, {"stream", runStreamRole}});
}
