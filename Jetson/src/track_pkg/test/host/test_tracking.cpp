#include <gtest/gtest.h>
#include "tracking/tracking.hpp"
#include <set>

namespace {
RecognitionDetection detection(double x, const std::string & category = "0002")
{
  RecognitionDetection result;
  result.id = "source";
  result.box = {x, 50, 40, 40, 0};
  result.hypothesis = RecognitionHypothesis{category, 0.9};
  return result;
}

SegTracker tracker()
{
  return SegTracker(TrackerParameters::validated(.3, 30, 1, .3, 3, .2, true));
}
}

TEST(TrackingAdapter, IntegerIdentitySurvivesAllGeometryPrecisionBoundaries)
{
  const auto saved = KalmanBoxTracker::count.load();
  for (const std::uint64_t base : {16777214ULL, 2147483645ULL, 9007199254740991ULL}) {
    KalmanBoxTracker::count.store(base);
    auto core = tracker();
    const auto output = core.update({detection(40), detection(140), detection(240)});
    std::set<std::string> ids;
    for (const auto & item : output) {
      ids.insert(item.id);
      ASSERT_TRUE(item.hypothesis);
      EXPECT_EQ(item.hypothesis->class_id, "2");
      EXPECT_DOUBLE_EQ(item.box.width, 40);
      EXPECT_DOUBLE_EQ(item.box.height, 40);
      EXPECT_FLOAT_EQ(item.hypothesis->score, .9f);
    }
    EXPECT_EQ(ids, (std::set<std::string>{std::to_string(base + 1), std::to_string(base + 2),
      std::to_string(base + 3)}));
  }
  KalmanBoxTracker::count.store(saved);
}

TEST(TrackingAdapter, ClassesAndSessionResetRetainIndependentIdentities)
{
  auto core = tracker();
  const auto first = core.update({detection(40, "0"), detection(140, "2"), detection(240, "3")});
  ASSERT_EQ(first.size(), 3u);
  std::set<std::string> ids;
  for (const auto & item : first) ids.insert(item.id);
  EXPECT_EQ(ids.size(), 3u);
  const auto next = core.update({detection(241, "3"), detection(141, "2"), detection(41, "0")});
  ASSERT_EQ(next.size(), 3u);
  for (std::size_t i = 0; i < first.size(); ++i) EXPECT_EQ(next[i].id, first[i].id);
  core.reset();
  const auto reset = core.update({detection(40, "0")});
  ASSERT_EQ(reset.size(), 1u);
  EXPECT_EQ(ids.count(reset[0].id), 0u);
}

TEST(TrackingAdapter, EmptyFramesAndMalformedInputDoNotPublishPredictions)
{
  auto core = tracker();
  const auto first = core.update({detection(40)});
  ASSERT_EQ(first.size(), 1u);
  EXPECT_TRUE(core.update({}).empty());
  auto malformed = detection(40);
  malformed.box.width = 0;
  EXPECT_TRUE(core.update({malformed}).empty());
  const auto next = core.update({detection(40)});
  ASSERT_EQ(next.size(), 1u);
  EXPECT_EQ(next[0].id, first[0].id);
}

TEST(TargetSelection, LargeNumericIdentitiesRetainTieAndSwitchSemantics)
{
  TargetSelector selector(1000, 1000, .05, {0, 2, 3}, 1);
  auto larger = detection(500), smaller = detection(500);
  larger.id = "9007199254740993";
  smaller.id = "9007199254740992";
  smaller.input_index = 1;
  EXPECT_EQ(selector.select({larger, smaller}), 1u);
  // Forty pixels is below the configured .05 normalized switching margin.
  larger.box.center_y += 40;
  EXPECT_EQ(selector.select({larger, smaller}), 1u);
  larger.box.center_y += 20;
  EXPECT_EQ(selector.select({larger, smaller}), 0u);
  EXPECT_FALSE(selector.select({}));
  larger.box.center_y = smaller.box.center_y;
  EXPECT_EQ(selector.select({larger, smaller}), 1u);
}

TEST(TargetSelection, BlueObstacleOverridesPreviousIdentityAndTripEligibility)
{
  TargetSelector selector(1000, 1000, .05, {0, 2, 3}, 1);
  auto first = detection(500, "0"), later = detection(700, "2");
  first.id = "16777217";
  later.id = "16777218";
  later.input_index = 1;
  selector.setLaterTransports(false);
  EXPECT_EQ(selector.select({first, later}), 0u);
  auto blue = first;
  blue.id = "blue";
  blue.input_index = 2;
  blue.hypothesis->class_id = "1";
  EXPECT_FALSE(selector.select({first, later, blue}));
  selector.setLaterTransports(true);
  EXPECT_EQ(selector.select({first, later, blue}), 1u);
}
