// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane_width.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(LaneWidth, EqualityOperator) {
  const LaneWidth kLaneWidth{1.1 /* sOffset */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  LaneWidth lane_width = kLaneWidth;

  EXPECT_EQ(kLaneWidth, lane_width);
  lane_width.s_0 = 5.;
  EXPECT_NE(kLaneWidth, lane_width);
  lane_width.s_0 = 1.1;
  lane_width.a = 5.;
  EXPECT_NE(kLaneWidth, lane_width);
  lane_width.a = 2.2;
  lane_width.b = 10.;
  EXPECT_NE(kLaneWidth, lane_width);
  lane_width.b = 3.3;
  lane_width.c = 10.;
  EXPECT_NE(kLaneWidth, lane_width);
  lane_width.c = 4.4;
  lane_width.d = 10.;
  EXPECT_NE(kLaneWidth, lane_width);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
