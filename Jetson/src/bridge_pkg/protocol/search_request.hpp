#pragma once

// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include <cstdint>
#include <optional>

enum class SearchRequestDecision {Reject, Pending, Replay, Start};

// 类型 19 的事务身份跨串口重连和 Mission 重启保留，不用接收时间推测重发。
// 一次只执行一个搜索；终态只缓存最近事务，更旧编号始终拒绝。
class SearchRequestSession
{
public:
  SearchRequestDecision onRequest(std::uint32_t request_id)
  {
    if (request_id == 0) {return SearchRequestDecision::Reject;}
    if (request_id == request_id_) {
      return result_ ? SearchRequestDecision::Replay : SearchRequestDecision::Pending;
    }
    const auto distance = static_cast<std::uint32_t>(request_id - request_id_);
    if (busy() || (request_id_ != 0 && distance >= 0x80000000U)) {
      return SearchRequestDecision::Reject;
    }
    request_id_ = request_id;
    result_.reset();
    return SearchRequestDecision::Start;
  }

  void complete(std::uint32_t request_id, bool success)
  {
    if (busy() && request_id == request_id_) {result_ = success;}
  }

  // Mission 重启作废旧成功和在途请求，但保留编号水位，旧编号不能借重启再次切换。
  void invalidate() {if (request_id_ != 0) {result_ = false;}}
  bool busy() const {return request_id_ != 0 && !result_;}
  std::uint32_t requestId() const {return request_id_;}
  std::optional<bool> result() const {return result_;}

private:
  std::uint32_t request_id_{0};
  std::optional<bool> result_;
};
