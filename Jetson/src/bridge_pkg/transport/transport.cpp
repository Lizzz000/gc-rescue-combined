#include "transport/transport.hpp"

#include <stdexcept>

// 保存与替换 Bridge 应答帧，不持有发送资格、时钟或队列。
// 发送、过期或断线撤销的是队列中的发送机会；旧快照保留但不能自行重发，新回复先替换快照再入队。

// 回复 ID 与固定槽的映射集中在此，周期 ID 不得误用回复存储。
std::optional<std::size_t> BridgeReplySlots::index(canid_t id)
{
  if (id == CanProtocol::ARRIVAL_ACK_ID) {return 0;}
  if (id == CanProtocol::ROI_RESULT_ID) {return 1;}
  if (id == CanProtocol::COMMAND_RESULT_ID) {return 2;}
  if (id == CanProtocol::TURN_REQUEST_ID) {return 3;}
  return std::nullopt;
}

void BridgeReplySlots::store(canid_t id, const can_frame & frame)
{
  const auto slot = index(id);
  // 原样复制编码后的整帧；本组件不改写 DLC／字节，也不负责入队或刷新回复 TTL。
  frames_[*slot] = frame;
}

// 读取不消费快照；运行模块只为仍有队列发送机会的 ID 调用它。
const can_frame & BridgeReplySlots::frame(canid_t id) const
{
  const auto slot = index(id);
  return frames_[*slot];
}
