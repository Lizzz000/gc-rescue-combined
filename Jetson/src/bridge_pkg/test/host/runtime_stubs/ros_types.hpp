#pragma once
// Minimal host substitutes for ROS transport/types only. BridgeRuntime itself is production code.
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <interface_pkg/app/core/recognition.hpp>

namespace rclcpp {
class Time {
public:
  Time(std::int64_t value = 0) : value_(value) {}
  std::int64_t nanoseconds() const {return value_;}
private:
  std::int64_t value_;
};
struct Logger {};
struct NodeOptions {};
struct Clock {using SharedPtr = std::shared_ptr<Clock>;};
struct Node {
  explicit Node(const NodeOptions & = {}) {}
  virtual ~Node() = default;
  std::int64_t time_ns{10000000000LL};
  Time now() const {return Time(time_ns);}
  Logger get_logger() const {return {};}
  Clock::SharedPtr get_clock() {return std::make_shared<Clock>();}
};
template<class T> struct Subscription {using SharedPtr = std::shared_ptr<Subscription>;};
struct TimerBase {using SharedPtr = std::shared_ptr<TimerBase>;};
template<class T> struct Publisher {
  using SharedPtr = std::shared_ptr<Publisher>;
  void publish(const T &) {}
};
template<class Service> struct Client {
  using SharedPtr = std::shared_ptr<Client>;
  using SharedFuture = std::shared_future<std::shared_ptr<typename Service::Response>>;
  struct Pending {std::int64_t request_id;};
  bool ready{true};
  std::vector<std::shared_ptr<typename Service::Request>> requests;
  std::vector<std::function<void(SharedFuture)>> callbacks;
  std::vector<std::int64_t> removed;
  bool service_is_ready() const {return ready;}
  template<class Callback>
  Pending async_send_request(std::shared_ptr<typename Service::Request> request, Callback callback) {
    requests.push_back(std::move(request));
    callbacks.emplace_back(std::move(callback));
    return {static_cast<std::int64_t>(requests.size())};
  }
  void remove_pending_request(std::int64_t id) {removed.push_back(id);}
  // Deliberately permit a queued callback after cancellation to exercise production generation checks.
  void reply(std::size_t index, typename Service::Response response) {
    std::promise<std::shared_ptr<typename Service::Response>> promise;
    promise.set_value(std::make_shared<typename Service::Response>(std::move(response)));
    callbacks.at(index)(promise.get_future().share());
  }
};
}
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
struct HostHeader {rclcpp::Time stamp; std::string frame_id;};
namespace interface_pkg::msg {
struct MissionStatus {
  using ConstSharedPtr = std::shared_ptr<const MissionStatus>;
  static constexpr std::uint8_t DEST_NONE=0, DEST_LEFT=1, DEST_RIGHT=2;
  static constexpr std::uint8_t OP_IDLE=0, OP_SWITCHING=2, RESULT_SUCCESS=2, RESULT_ROI_INVALID=6;
  static constexpr std::uint8_t TRANSPORT_FIRST=0, TRANSPORT_LATER=1, CYCLE_WAITING_TURN=2, CYCLE_FAULT=8,
    CYCLE_LOCALIZING_START=9;
  HostHeader header;
  std::uint8_t destination{0}, operation{0}, outcome{0}, transport_phase{0}, cycle_phase{0};
  std::uint64_t roi_request_id{0}, cycle_generation{0};
};
struct TaskState {
  using ConstSharedPtr = std::shared_ptr<const TaskState>;
  HostHeader header;
  std::string task;
  std::uint64_t epoch{0};
};
struct SafetyTarget {
  using ConstSharedPtr = std::shared_ptr<const SafetyTarget>;
  HostHeader header;
  bool valid{false};
  std::uint8_t side{0};
  double x{0}, y{0};
};
struct RoiCheck {
  bool valid{false}, passed{false};
  std::uint32_t orange_count{0};
  std::uint64_t request_id{0};
};
struct BridgeStatus {
  HostHeader header;
  std::string can_interface, last_error;
  bool socket_open{false}, interface_up{false};
  std::uint64_t tx_frames{0}, rx_frames{0}, bus_errors{0}, dropped_frames{0};
};
}
namespace interface_pkg::srv {
struct CheckRoi {
  struct Request {};
  struct Response {msg::RoiCheck result;};
};
struct MissionCommand {
  struct Request {
    static constexpr std::uint8_t TURN_COMPLETED=1, SAFETY_ARRIVED=2, START_SEARCH=3, DELIVERY_COMPLETED=4;
    std::uint8_t command{0};
    std::uint64_t roi_request_id{0};
  };
  struct Response {bool accepted{false};};
};
}
namespace vision_msgs::msg {
struct Detection2DArray {
  using ConstSharedPtr = std::shared_ptr<const Detection2DArray>;
  HostHeader header;
  std::vector<RecognitionDetection> detections;
};
}
struct RecognitionValidation {
  static bool sourceTime(rclcpp::Time source, std::int64_t current, std::int64_t & stamp) {
    stamp = source.nanoseconds();
    return stamp > 0 && stamp <= current;
  }
};
struct RecognitionRosConversion {
  static RecognitionDetection detection(const RecognitionDetection & input) {return input;}
};
