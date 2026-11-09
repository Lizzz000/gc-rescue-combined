// SelectNode：按任务阶段和运输资格保留一个目标。
// 轨迹从 TrackNode 来，选中的目标交给 mission 的 target 与 bridge。

#include "node/select_node.hpp"
#include "node/track_node.hpp"

#include <exception>
#include <interface_pkg/app/core.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <interface_pkg/app/core.hpp>
#include <limits>

// ---- SelectNode：构造与参数 ----

// 目标选择使用实际会话、运输资格与独立帧历史，输出沿用被选轨迹的源消息。

/**
 * @brief 在选择器和帧闸门入口检查配置后创建目标选择接口。
 * @param[in] options ROS节点选项，须携带完整统一参数。
 * @throws rescue::ConfigurationError 非法配置时拒绝创建业务接口。
 */
SelectNode::SelectNode(const rclcpp::NodeOptions & options)
: Node("select_node", options), settings_(loadParameters())
{
  try {
    frame_gate_.setTimeout(settings_.reset_timeout_s);
    selector_ = std::make_unique<TargetSelector>(
      settings_.image_width, settings_.image_height,
      settings_.blue_distance_ratio, settings_.candidate_classes, settings_.blue_class);
  } catch (const std::invalid_argument & error) {
    // 只转译纯选择器构造对启动参数的拒绝，不覆盖随后 ROS 装配或运行期回调。
    throw rescue::ConfigurationError(error.what());
  }
  selector_->setFirstClass(std::to_string(settings_.first_class));
  selector_->setLaterTransports(false);
  createRosInterfaces();
  RCLCPP_INFO(get_logger(), "Target selection: %s -> ~/target", settings_.tracks_topic.c_str());
}

/**
 * @brief 读取选择参数并检查首趟资格；几何和超时交给所属构造入口。
 * @return 参数集合；缺失、类型或首趟资格错误抛ConfigurationError。
 */
SelectNode::Settings SelectNode::loadParameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto tracks_topic = rescue::declareRequired<std::string>(
    *this, "tracks_topic", descriptor);
  const auto mode_topic = rescue::declareRequired<std::string>(
    *this, "active_task_topic", descriptor);
  const auto image_width = rescue::declareRequired<std::int64_t>(
    *this, "image_width", descriptor);
  const auto image_height = rescue::declareRequired<std::int64_t>(
    *this, "image_height", descriptor);
  const auto ratio = rescue::declareRequired<double>(
    *this, "blue_min_distance_ratio", descriptor);
  const auto classes = rescue::declareRequired<std::vector<std::int64_t>>(
    *this, "candidate_class_ids", descriptor);
  const auto blue = rescue::declareRequired<std::int64_t>(
    *this, "blue_class_id", descriptor);
  const auto first_class = rescue::declareRequired<std::int64_t>(
    *this, "first_class_id", descriptor);
  const auto mission_topic = rescue::declareRequired<std::string>(
    *this, "mission_status_topic", descriptor);
  const auto reset_timeout_s = rescue::declareRequired<double>(
    *this, "reset_timeout_s", descriptor);
  if (first_class < 0 || first_class == blue ||
    std::find(classes.begin(), classes.end(), first_class) == classes.end() ||
    mission_topic.empty() || mission_topic.find_first_of(" \t\n\r\f\v") != std::string::npos)
  {
    throw rescue::ConfigurationError("first class must be a non-blue candidate");
  }
  return {tracks_topic, mode_topic, mission_topic, image_width, image_height, ratio,
    classes, blue, first_class, reset_timeout_s};
}

// ---- SelectNode：ROS 接口 ----

// 默认互斥回调组串行维护选择历史和源帧截止。
void SelectNode::createRosInterfaces()
{
  target_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "~/target", rclcpp::SensorDataQoS().keep_last(1));
  mode_sub_ = create_subscription<interface_pkg::msg::TaskState>(
    settings_.active_task_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::TaskState::ConstSharedPtr msg) {modeCallback(msg);});
  mission_sub_ = create_subscription<interface_pkg::msg::MissionStatus>(
    settings_.mission_status_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::MissionStatus::ConstSharedPtr msg) {missionCallback(msg);});
  tracks_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    settings_.tracks_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {tracksCallback(msg);});
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this] {checkTimeout();});
}

// ---- SelectNode：会话、运输资格与轨迹事件 ----

// 实际 seg 阶段才允许选择；同模式新 epoch 与模式切换一样清空历史。
void SelectNode::modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg)
{
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), stamp)) {
    return;
  }
  recognition::TaskSession next{msg->task, msg->epoch, stamp};
  if (!recognition::isNewSession(session_, next)) {return;}
  session_ = std::move(next);
  enabled_ = session_->task == interface_pkg::msg::TaskState::TASK_SEG;
  // 保留较新的超时或任务截止时间。
  frame_gate_.clear(stamp);
  clearTarget(msg->header.stamp);
}

// 运输阶段只调整首趟类别资格；同资格的状态心跳不会重置当前目标。
void SelectNode::missionCallback(
  const interface_pkg::msg::MissionStatus::ConstSharedPtr & msg)
{
  using interface_pkg::msg::MissionStatus;
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), stamp) ||
    !RecognitionValidation::missionStatusEnums(*msg)) {return;}
  // 水位只记录合法状态；拒绝比最近合法状态更旧的消息，同时间的资格变化仍可处理。
  if (stamp < mission_stamp_ns_) {return;}
  mission_stamp_ns_ = stamp;
  const bool later = msg->transport_phase == MissionStatus::TRANSPORT_LATER;
  if (later == later_transports_) {return;}
  later_transports_ = later;
  selector_->setLaterTransports(later);
  // 保留较新的超时或任务截止时间。
  frame_gate_.clear(stamp);
  clearTarget(msg->header.stamp);
}

/**
 * @brief 在帧入口检查时间与身份，再一次筛选检测并发布原始选中消息。
 * @param[in] msg 非空ROS轨迹数组；时间为同一ROS时钟，坐标为原图像素。
 * @note 发布后才登记帧历史。
 */
void SelectNode::tracksCallback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (!enabled_) {return;}
  const auto current = now().nanoseconds();
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  if (!frame_gate_.accepts(stamp, current)) {return;}
  const auto arrival = FrameGate::Clock::now();
  // 不依赖看门狗先执行；即使源时间只推进少量，实际断流超时后旧 ID 也不能继续享有余量。
  if (frame_gate_.historyChanged(stamp, msg->header.frame_id) || frame_gate_.expired(arrival)) {
    selector_->reset();
  }
  // 转换保留每条记录在原 ROS 数组中的位置；过滤坏记录后不能把压缩后的下标回传。
  const auto detections = RecognitionRosConversion::detections(*msg);
  const auto selected = selector_->select(detections);
  auto output = std::make_unique<vision_msgs::msg::Detection2DArray>(
    TrackRosConversion::selected(*msg, selected));
  target_pub_->publish(std::move(output));
  last_header_ = msg->header;
  // 空轨迹数组也记录，及时发布空目标并保持断流计时与真实帧到达一致。
  frame_gate_.record(stamp, msg->header.frame_id, arrival);
}

// ---- SelectNode：输出撤销与超时清理 ----

// 撤掉目标并清切换余量历史；截止只前移，由阶段或看门狗调用方传入源边界。
void SelectNode::clearTarget(const builtin_interfaces::msg::Time & stamp)
{
  selector_->reset();
  vision_msgs::msg::Detection2DArray empty;
  empty.header = last_header_;
  empty.header.stamp = stamp;
  target_pub_->publish(empty);
  frame_gate_.clear();
}

// 不依赖 ROS 源时钟前进；断流后前移截止，使迟到轨迹不能复活旧目标。
void SelectNode::checkTimeout()
{
  if (frame_gate_.expired(FrameGate::Clock::now())) {
    const auto cutoff = now();
    clearTarget(static_cast<builtin_interfaces::msg::Time>(cutoff));
    frame_gate_.clear(cutoff.nanoseconds());
  }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(SelectNode)
