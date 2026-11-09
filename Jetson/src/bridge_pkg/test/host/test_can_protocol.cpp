#include "check.hpp"
#include "protocol/can_protocol.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

namespace {

bool bytesEqual(const can_frame & frame, canid_t id, std::uint8_t len, const std::uint8_t * expect)
{
  if (frame.can_id != id || frame.len != len) {return false;}
  return std::memcmp(frame.data, expect, len) == 0;
}

can_frame commandFrame(canid_t id, std::uint8_t len, std::uint8_t request, std::uint32_t token)
{
  can_frame frame{};
  frame.can_id = id;
  frame.len = len;
  frame.data[0] = request;
  for (unsigned byte = 0; byte < 4; ++byte) {
    frame.data[1 + byte] = static_cast<std::uint8_t>(token >> (8 * byte));
  }
  return frame;
}

bool threwInvalid(void (*call)())
{
  try {
    call();
  } catch (const std::invalid_argument &) {
    return true;
  }
  return false;
}

}  // namespace

void testCanProtocol()
{
  Group group("CanProtocol");

  // 0x180：valid、vx、vy、角速度，int16 小端。负数走补码，不看主机字节序。
  const std::uint8_t velocity_bytes[] = {1, 0xfe, 0xff, 0x00, 0x00, 0xe8, 0x03};
  CHECK(bytesEqual(CanProtocol::velocity(true, -2, 0, 1000), CanProtocol::VELOCITY_ID, 7, velocity_bytes));
  const std::uint8_t velocity_invalid[] = {0, 0, 0, 0, 0, 0, 0};
  CHECK(bytesEqual(CanProtocol::velocity(false, -2, 0, 1000), CanProtocol::VELOCITY_ID, 7, velocity_invalid));

  // 坐标帧携带 24 位源帧编号；无效帧仍保留 ID 和 DLC，载荷清零。
  const std::uint8_t center_bytes[] = {1, 0x88, 0x13, 0xff, 0xff, 0x56, 0x34, 0x12};
  CHECK(bytesEqual(CanProtocol::center(CanProtocol::TARGET_ID, true, 5000, -1, 0x123456), CanProtocol::TARGET_ID, 8, center_bytes));
  CHECK(bytesEqual(CanProtocol::center(CanProtocol::SAFETY_ID, true, 5000, -1, 0x123456), CanProtocol::SAFETY_ID, 8, center_bytes));
  const std::uint8_t center_invalid[] = {0, 0, 0, 0, 0, 0, 0, 0};
  CHECK(bytesEqual(CanProtocol::center(CanProtocol::TARGET_ID, false, 5000, -1), CanProtocol::TARGET_ID, 8, center_invalid));

  // std::round 在半整数处远离零。越界失败并把输出写成 0，不饱和。
  // 半整数和边界用整数乘 1，避免十进制小数再乘 10000 时不是精确的 .5。
  std::int16_t scaled = 99;
  CHECK(CanProtocol::fixedPoint(0.5, 10000.0, scaled) && scaled == 5000);
  CHECK(CanProtocol::fixedPoint(-1.5, 1000.0, scaled) && scaled == -1500);
  CHECK(CanProtocol::fixedPoint(0.5, 1.0, scaled) && scaled == 1);
  CHECK(CanProtocol::fixedPoint(-0.5, 1.0, scaled) && scaled == -1);
  CHECK(CanProtocol::fixedPoint(32767.0, 1.0, scaled) && scaled == 32767);
  CHECK(CanProtocol::fixedPoint(-32768.0, 1.0, scaled) && scaled == -32768);
  scaled = 99;
  CHECK(!CanProtocol::fixedPoint(32768.0, 1.0, scaled) && scaled == 0);
  scaled = 99;
  CHECK(!CanProtocol::fixedPoint(-32769.0, 1.0, scaled) && scaled == 0);
  scaled = 99;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  CHECK(!CanProtocol::fixedPoint(nan, 1000.0, scaled) && !CanProtocol::fixedPoint(1.0, 0.0, scaled) &&
    !CanProtocol::fixedPoint(1.0, -1.0, scaled) && scaled == 0);

  // 0x190 到达应答只有受理位。
  const std::uint8_t ack_yes[] = {1};
  const std::uint8_t ack_no[] = {0};
  CHECK(bytesEqual(CanProtocol::arrivalAck(true), CanProtocol::ARRIVAL_ACK_ID, 1, ack_yes));
  CHECK(bytesEqual(CanProtocol::arrivalAck(false), CanProtocol::ARRIVAL_ACK_ID, 1, ack_no));

  // 0x191：通过必须带 1 或 2。失败和无效都把后两字节清零。
  const std::uint8_t roi_left[] = {1, 1, 1};
  const std::uint8_t roi_right[] = {1, 1, 2};
  const std::uint8_t roi_fail[] = {1, 0, 0};
  const std::uint8_t roi_invalid[] = {0, 0, 0};
  CHECK(bytesEqual(CanProtocol::roiResult(true, true, 1), CanProtocol::ROI_RESULT_ID, 3, roi_left));
  CHECK(bytesEqual(CanProtocol::roiResult(true, true, 2), CanProtocol::ROI_RESULT_ID, 3, roi_right));
  CHECK(bytesEqual(CanProtocol::roiResult(true, false, 2), CanProtocol::ROI_RESULT_ID, 3, roi_fail));
  CHECK(bytesEqual(CanProtocol::roiResult(false, true, 1), CanProtocol::ROI_RESULT_ID, 3, roi_invalid));
  CHECK(threwInvalid([] { CanProtocol::roiResult(true, true, 0); }));

  // 0x192 token 小端。命令 3 没有身份，调用方带进来的 token 也不写出。
  const std::uint8_t command_turn[] = {1, CanProtocol::TURN_COMPLETED, 0x04, 0x03, 0x02, 0x01};
  const std::uint8_t command_search[] = {1, CanProtocol::START_SEARCH, 0, 0, 0, 0};
  const std::uint8_t command_reject[] = {0, CanProtocol::DELIVERY_COMPLETED, 0xd4, 0xc3, 0xb2, 0xa1};
  CHECK(bytesEqual(
    CanProtocol::commandResult(true, CanProtocol::TURN_COMPLETED, 0x01020304u),
    CanProtocol::COMMAND_RESULT_ID, 6, command_turn));
  CHECK(bytesEqual(
    CanProtocol::commandResult(true, CanProtocol::START_SEARCH, 0xffffffffu),
    CanProtocol::COMMAND_RESULT_ID, 6, command_search));
  CHECK(bytesEqual(
    CanProtocol::commandResult(false, CanProtocol::DELIVERY_COMPLETED, 0xa1b2c3d4u),
    CanProtocol::COMMAND_RESULT_ID, 6, command_reject));
  CHECK(threwInvalid([] { CanProtocol::commandResult(true, 0, 1); }));
  CHECK(threwInvalid([] { CanProtocol::commandResult(true, 9, 1); }));

  // 0x193 同样小端，目的区只能是 1 或 2。
  const std::uint8_t turn_bytes[] = {1, 2, 0xd4, 0xc3, 0xb2, 0xa1};
  CHECK(bytesEqual(CanProtocol::turnRequest(2, 0xa1b2c3d4u), CanProtocol::TURN_REQUEST_ID, 6, turn_bytes));
  CHECK(threwInvalid([] { CanProtocol::turnRequest(0, 1); }));

  // 五条入站指令：长度或请求字节不对是 MALFORMED，不读 token。未知 ID 和标志位是 NONE。
  const auto arrival = CanProtocol::parseCommand(commandFrame(CanProtocol::ARRIVAL_ID, 1, 1, 0));
  CHECK(arrival.kind == CanProtocol::CommandKind::ARRIVAL && arrival.token == 0);
  const auto arrival_len = CanProtocol::parseCommand(commandFrame(CanProtocol::ARRIVAL_ID, 2, 1, 0));
  CHECK(arrival_len.kind == CanProtocol::CommandKind::MALFORMED && arrival_len.token == 0);
  const auto arrival_byte = CanProtocol::parseCommand(commandFrame(CanProtocol::ARRIVAL_ID, 1, 0, 0));
  CHECK(arrival_byte.kind == CanProtocol::CommandKind::MALFORMED);

  const auto turned = CanProtocol::parseCommand(commandFrame(CanProtocol::TURN_COMPLETED_ID, 5, 1, 0x01020304u));
  CHECK(turned.kind == CanProtocol::CommandKind::MISSION && turned.command == CanProtocol::TURN_COMPLETED &&
    turned.token == 0x01020304u);
  const auto turned_short = CanProtocol::parseCommand(commandFrame(CanProtocol::TURN_COMPLETED_ID, 4, 1, 0x01020304u));
  CHECK(turned_short.kind == CanProtocol::CommandKind::MALFORMED && turned_short.token == 0 && turned_short.command == CanProtocol::TURN_COMPLETED);

  const auto safety = CanProtocol::parseCommand(commandFrame(CanProtocol::SAFETY_ARRIVED_ID, 5, 1, 0x0000007bu));
  CHECK(safety.kind == CanProtocol::CommandKind::MISSION && safety.command == CanProtocol::SAFETY_ARRIVED && safety.token == 0x7bu);
  const auto delivery = CanProtocol::parseCommand(commandFrame(CanProtocol::DELIVERY_COMPLETED_ID, 5, 1, 0xa1b2c3d4u));
  CHECK(delivery.kind == CanProtocol::CommandKind::MISSION && delivery.command == CanProtocol::DELIVERY_COMPLETED &&
    delivery.token == 0xa1b2c3d4u);
  const auto search = CanProtocol::parseCommand(commandFrame(CanProtocol::START_SEARCH_ID, 1, 1, 0xffffffffu));
  CHECK(search.kind == CanProtocol::CommandKind::MISSION && search.command == CanProtocol::START_SEARCH && search.token == 0);
  const auto search_len = CanProtocol::parseCommand(commandFrame(CanProtocol::START_SEARCH_ID, 5, 1, 1));
  CHECK(search_len.kind == CanProtocol::CommandKind::MALFORMED && search_len.command == CanProtocol::START_SEARCH && search_len.token == 0);

  const auto old_id = CanProtocol::parseCommand(commandFrame(0x201, 1, 1, 0));
  const auto other_id = CanProtocol::parseCommand(commandFrame(0x202, 1, 1, 0));
  CHECK(old_id.kind == CanProtocol::CommandKind::NONE && other_id.kind == CanProtocol::CommandKind::NONE);
  can_frame flagged = commandFrame(CanProtocol::ARRIVAL_ID, 1, 1, 0);
  flagged.can_id |= CAN_EFF_FLAG;
  const auto extended = CanProtocol::parseCommand(flagged);
  flagged.can_id = CanProtocol::ARRIVAL_ID | CAN_RTR_FLAG;
  const auto remote = CanProtocol::parseCommand(flagged);
  flagged.can_id = CanProtocol::ARRIVAL_ID | CAN_ERR_FLAG;
  const auto error_frame = CanProtocol::parseCommand(flagged);
  CHECK(extended.kind == CanProtocol::CommandKind::NONE && remote.kind == CanProtocol::CommandKind::NONE &&
    error_frame.kind == CanProtocol::CommandKind::NONE);

  // 12 个 ID：三条周期、四条事件、五条指令。旧 0x201 不在任何一类里。
  CHECK(CanProtocol::streamId(CanProtocol::VELOCITY_ID) && CanProtocol::streamId(CanProtocol::TARGET_ID) &&
    CanProtocol::streamId(CanProtocol::SAFETY_ID) && !CanProtocol::streamId(CanProtocol::ARRIVAL_ACK_ID));
  CHECK(CanProtocol::replyId(CanProtocol::ARRIVAL_ACK_ID) && CanProtocol::replyId(CanProtocol::ROI_RESULT_ID) &&
    CanProtocol::replyId(CanProtocol::COMMAND_RESULT_ID) && CanProtocol::replyId(CanProtocol::TURN_REQUEST_ID) &&
    !CanProtocol::replyId(CanProtocol::VELOCITY_ID));
  CHECK(CanProtocol::transmitId(CanProtocol::VELOCITY_ID) && CanProtocol::transmitId(CanProtocol::TURN_REQUEST_ID) &&
    !CanProtocol::transmitId(CanProtocol::ARRIVAL_ID) && !CanProtocol::transmitId(0x201));
  const auto filters = CanProtocol::commandFilters();
  CHECK(filters.size() == 5 && filters[0].can_id == CanProtocol::ARRIVAL_ID &&
    filters[1].can_id == CanProtocol::TURN_COMPLETED_ID && filters[2].can_id == CanProtocol::SAFETY_ARRIVED_ID &&
    filters[3].can_id == CanProtocol::START_SEARCH_ID && filters[4].can_id == CanProtocol::DELIVERY_COMPLETED_ID &&
    filters[0].can_mask == (CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG));
}
