#include "check.hpp"
#include "control/controller.hpp"

#include <stdexcept>
#include <string>

namespace {

RoiClassIds classIds()
{
  return {0, 1, 2, 3};
}

RoiValidator validator()
{
  // 右下象限。边界恰好落在 0.5 上的框算相交，完全在左上的框不算。
  return RoiValidator(100, 100, 0.5, 0.5, 0.5, 0.5, classIds());
}

RecognitionDetection boxAt(
  const char * id, const char * class_id, double center_x, double center_y, double width, double height,
  double score = 0.9)
{
  RecognitionDetection detection;
  detection.id = id;
  detection.box = {center_x, center_y, width, height, 0};
  detection.hypothesis = RecognitionHypothesis{class_id, score};
  return detection;
}

RoiRuleResult passedGreen(std::uint32_t green, std::uint32_t black = 0, std::uint32_t orange = 0)
{
  RoiRuleResult result;
  result.valid = true;
  result.passed = true;
  result.green_count = green;
  result.black_count = black;
  result.orange_count = orange;
  result.reason = "ROI accepted";
  return result;
}

}  // namespace

void testRoiAndTransport()
{
  Group group("RoiValidator.applyTransportRules");
  const auto rules = validator();

  const auto empty = rules.evaluate({});
  CHECK(empty.valid && !empty.passed && empty.reason == "empty ROI");

  const auto green = rules.evaluate({boxAt("a", "0", 75, 75, 10, 10)});
  CHECK(green.valid && green.passed && green.green_count == 1 && green.other_count == 0);

  // 蓝色先于未知类别。两件都进了 ROI，拒绝原因只取第一条。
  const auto blue = rules.evaluate({
    boxAt("b", "1", 75, 75, 10, 10),
    boxAt("c", "9", 80, 80, 10, 10)});
  CHECK(blue.valid && !blue.passed && blue.reason == "blue in ROI" && blue.blue_count == 1 && blue.other_count == 1);

  const auto orange = rules.evaluate({boxAt("d", "3", 75, 75, 10, 10)});
  CHECK(orange.valid && orange.passed && orange.orange_count == 1);

  const auto mixed = rules.evaluate({
    boxAt("e", "3", 70, 70, 10, 10),
    boxAt("f", "0", 80, 80, 10, 10)});
  CHECK(mixed.valid && !mixed.passed && mixed.reason == "orange must be exactly one and alone");

  const auto two = rules.evaluate({
    boxAt("same", "0", 70, 70, 10, 10),
    boxAt("same", "0", 80, 80, 10, 10)});
  CHECK(two.valid && two.passed && two.green_count == 2);

  const auto outside = rules.evaluate({boxAt("g", "0", 10, 10, 10, 10)});
  CHECK(outside.valid && !outside.passed && outside.green_count == 0);

  // 右边缘恰好等于 ROI 左边缘，算触边，计入。
  const auto touching = rules.evaluate({boxAt("h", "0", 45, 75, 10, 10)});
  CHECK(touching.valid && touching.passed && touching.green_count == 1);

  const auto broken = rules.evaluate({
    boxAt("i", "0", 75, 75, 10, 10),
    boxAt("j", "0", 75, 75, 0, 10)});
  CHECK(!broken.valid && !broken.passed && broken.green_count == 1 && broken.reason == "malformed detection");

  const auto bad_score = rules.evaluate({boxAt("k", "0", 75, 75, 10, 10, 1.5)});
  CHECK(!bad_score.valid && bad_score.reason == "malformed detection hypothesis" && bad_score.green_count == 0);

  const auto leading_zero = rules.evaluate({boxAt("m", "000", 75, 75, 10, 10)});
  CHECK(leading_zero.valid && leading_zero.passed && leading_zero.green_count == 1);

  bool rejected_size = false;
  try {
    RoiValidator(0, 100, 0, 0, 1, 1, classIds());
  } catch (const std::invalid_argument &) {
    rejected_size = true;
  }
  bool rejected_class = false;
  try {
    RoiValidator(100, 100, 0, 0, 1, 1, RoiClassIds{1, 1, 2, 3});
  } catch (const std::invalid_argument &) {
    rejected_class = true;
  }
  CHECK(rejected_size && rejected_class);

  const TransportPolicy production{1, 3};
  const auto first_one = applyTransportRules(passedGreen(1), MissionState::Transport::First, production);
  CHECK(first_one.accepted && first_one.destination == MissionState::Destination::Left && first_one.result.passed);

  const auto first_two = applyTransportRules(passedGreen(2), MissionState::Transport::First, production);
  CHECK(!first_two.accepted && !first_two.result.passed);

  const auto first_orange = applyTransportRules(passedGreen(0, 0, 1), MissionState::Transport::First, production);
  CHECK(!first_orange.accepted && !first_orange.result.passed);

  const auto later_orange = applyTransportRules(passedGreen(0, 0, 1), MissionState::Transport::Later, production);
  CHECK(later_orange.accepted && later_orange.destination == MissionState::Destination::Right);

  const auto later_supply = applyTransportRules(passedGreen(2, 1), MissionState::Transport::Later, production);
  CHECK(later_supply.accepted && later_supply.destination == MissionState::Destination::Left);

  const auto over_supply = applyTransportRules(passedGreen(3, 1), MissionState::Transport::Later, production);
  CHECK(!over_supply.accepted && !over_supply.result.passed);

  // 件数规则不再复查颜色组合。passed 已为真时，橙加绿仍按橙色件数去右区。
  const auto lied = applyTransportRules(passedGreen(1, 0, 1), MissionState::Transport::Later, TransportPolicy{0, 0});
  CHECK(lied.accepted && lied.destination == MissionState::Destination::Right && lied.result.passed);

  RoiRuleResult invalid;
  invalid.reason = "malformed detection";
  const auto ignored = applyTransportRules(invalid, MissionState::Transport::Later, production);
  CHECK(!ignored.accepted && !ignored.result.passed);

  const auto unlimited = applyTransportRules(passedGreen(2), MissionState::Transport::First, TransportPolicy{0, 0});
  CHECK(unlimited.accepted && unlimited.destination == MissionState::Destination::Left);
}
