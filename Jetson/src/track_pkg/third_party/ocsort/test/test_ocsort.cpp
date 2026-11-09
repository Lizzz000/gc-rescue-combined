#include <gtest/gtest.h>
#include "OCSort.hpp"

static Eigen::MatrixXf detection(float x = 10.0f) {
  Eigen::MatrixXf input(1, 6);
  input << x, 10, x + 40, 50, .9f, 0;
  return input;
}

TEST(OCSortCore, EmptyFramesAgeAndExpireTracks) {
  OCSort tracker(.3f, 2, 1);
  Eigen::MatrixXf empty(0, 6);
  EXPECT_TRUE(tracker.update(empty).empty());
  tracker.update(detection());
  for (int i = 0; i < 3; ++i) EXPECT_TRUE(tracker.update(empty).empty());
  EXPECT_TRUE(tracker.trackers.empty());
}

TEST(OCSortCore, MovingTargetRetainsIdentity) {
  OCSort tracker(.3f, 30, 1);
  auto first = tracker.update(detection());
  ASSERT_EQ(first.size(), 1u);
  for (int i = 1; i < 20; ++i) {
    auto result = tracker.update(detection(10 + i));
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0](4), first[0](4));
    EXPECT_TRUE(result[0].allFinite());
  }
}

TEST(OCSortCore, ReappearanceRetainsIdentityWithinMaxAge) {
  OCSort tracker(.3f, 30, 1);
  auto first = tracker.update(detection());
  tracker.update(detection(11));
  tracker.update(detection(12));
  tracker.update(Eigen::MatrixXf(0, 6));
  tracker.update(Eigen::MatrixXf(0, 6));
  auto result = tracker.update(detection(15));
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0](4), first[0](4));
}

TEST(OCSortCore, IndependentInstancesNeverResetIds) {
  OCSort first(.3f, 30, 1);
  const auto a = first.update(detection());
  OCSort second(.3f, 30, 1);
  const auto b = second.update(detection());
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_NE(a[0](4), b[0](4));
}

TEST(OCSortCore, RejectsMalformedInput) {
  OCSort tracker(.3f);
  EXPECT_THROW(tracker.update(Eigen::MatrixXf(0, 0)), std::invalid_argument);
}

TEST(OCSortCore, IncludesConfidenceThresholdAndRecoversWeakDetection) {
  OCSort tracker(.3f, 30, 1, .3f, 3, "iou", .2f, true);
  auto input = detection();
  input(0, 4) = .3f;
  const auto first = tracker.update(input);
  ASSERT_EQ(first.size(), 1u);
  input(0, 4) = .2f;
  const auto weak = tracker.update(input);
  ASSERT_EQ(weak.size(), 1u);
  EXPECT_EQ(weak[0](4), first[0](4));
  EXPECT_FLOAT_EQ(weak[0](6), .2f);
}

TEST(OCSortCore, DiscardsCollapsedPredictionWithoutLosingOtherTracks) {
  OCSort tracker(.3f, 30, 1);
  Eigen::MatrixXf input(3, 6);
  input.row(0) = detection(10);
  input.row(1) = detection(100);
  input.row(2) = detection(200);
  ASSERT_EQ(tracker.update(input).size(), 3u);
  const int first_id = tracker.trackers[0].id;
  const int last_id = tracker.trackers[2].id;
  tracker.trackers[1].kf->x(2) = 0;  // Zero predicted area.
  tracker.trackers[1].kf->x(6) = 0;
  EXPECT_TRUE(tracker.update(Eigen::MatrixXf(0, 6)).empty());
  ASSERT_EQ(tracker.trackers.size(), 2u);
  EXPECT_EQ(tracker.trackers[0].id, first_id);
  EXPECT_EQ(tracker.trackers[1].id, last_id);
}

TEST(OCSortCore, MotionDirectionResolvesAmbiguousOverlap) {
  Eigen::MatrixXf detections(2, 6);
  detections.row(0) = detection(5);
  detections.row(1) = detection(15);
  Eigen::MatrixXf tracks(2, 5);
  tracks.row(0) = detection().leftCols(5);
  tracks.row(1) = tracks.row(0);
  Eigen::MatrixXf velocities(2, 2);
  velocities << 0, -1, 0, 1;
  const auto result = associate(detections, tracks, .3f, velocities, tracks, .2f);
  ASSERT_EQ(std::get<0>(result).size(), 2u);
  EXPECT_TRUE(std::get<1>(result).empty());
  EXPECT_TRUE(std::get<2>(result).empty());
  for (const auto& match : std::get<0>(result)) EXPECT_EQ(match(0), match(1));
}

TEST(OCSortCore, MultipleTargetsKeepIdsAcrossInputReordering) {
  OCSort tracker(.3f, 30, 1);
  Eigen::MatrixXf input(2, 6);
  input.row(0) = detection(10);
  input.row(1) = detection(200);
  const auto first = tracker.update(input);
  ASSERT_EQ(first.size(), 2u);
  input.row(0) = detection(201);
  input.row(1) = detection(11);
  const auto next = tracker.update(input);
  ASSERT_EQ(next.size(), 2u);
  for (const auto& track : next) {
    const auto& previous = track(0) > 100 ? first[0] : first[1];
    EXPECT_EQ(track(4), previous(4));
  }
  EXPECT_EQ(tracker.update(detection(12)).size(), 1u);
}
