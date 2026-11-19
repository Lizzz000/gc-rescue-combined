#pragma once

// 夹爪请求身份与终态缓存。不依赖 ROS 或串口，运行层和主机回归共用。
#include <array>
#include <cstdint>
#include <optional>

enum class ArrivalPointDecision { Reject, Resend, Accept };

struct GripperResult
{
  std::uint8_t kind;
  bool success;
  std::uint32_t request_id;
};

class ArrivalPointSession
{
public:
  // 构造时的启动张开已在途；kind 0 只能查询它，不能再次开爪。
  ArrivalPointDecision onRequest(std::uint8_t kind, std::uint32_t request_id, bool authorized = true)
  {
    if (kind > 3 || request_id == 0) {return ArrivalPointDecision::Reject;}
    auto & record = records_[kind];
    if (record.request_id == request_id) {return ArrivalPointDecision::Resend;}
    // STM32 编号递增、溢出跳过 0。已替换的旧编号不能重新触发开合。
    if (record.request_id != 0 && !newer(request_id, record.request_id)) {
      return ArrivalPointDecision::Reject;
    }
    if (kind == 0) {
      record = {request_id, startup_result_};
      return ArrivalPointDecision::Resend;
    }
    // 已知编号仍可查询终态，新动作必须属于当前阶段。
    if (!authorized || active_ || (kind == 3 && !reject_open_authorized_)) {
      return ArrivalPointDecision::Reject;
    }
    record = {request_id, std::nullopt};
    active_ = kind;
    if (kind == 3) {reject_open_authorized_ = false;}
    return ArrivalPointDecision::Accept;
  }

  std::optional<GripperResult> result(std::uint8_t kind, std::uint32_t request_id) const
  {
    if (kind > 3) {return std::nullopt;}
    const auto & record = records_[kind];
    if (record.request_id != request_id || request_id == 0 || !record.success.has_value()) {
      return std::nullopt;
    }
    return GripperResult{kind, *record.success, request_id};
  }

  // 必须先保存终态，再 acknowledge 底层舵机。断线或队列过期不清此缓存。
  std::optional<GripperResult> complete(bool success)
  {
    if (!active_) {return std::nullopt;}
    const auto kind = *active_;
    if (kind == 0) {startup_result_ = success;}
    records_[kind].success = success;
    active_.reset();
    return result(kind, records_[kind].request_id);
  }

  void authorizeRejectOpen() {reject_open_authorized_ = true;}
  // 明确进入新搜索时撤旧拒绝授权；保留编号水位、结果和在途动作，防止跨轮重放。
  void beginSearch() {reject_open_authorized_ = false;}
  bool busy() const {return active_.has_value();}

private:
  struct Record
  {
    std::uint32_t request_id{0};
    std::optional<bool> success;
  };
  static bool newer(std::uint32_t next, std::uint32_t previous)
  {
    const auto distance = next - previous;
    return distance != 0 && distance < 0x80000000u;
  }
  std::array<Record, 4> records_{};
  std::optional<std::uint8_t> active_{0};
  std::optional<bool> startup_result_;
  bool reject_open_authorized_{false};
};
