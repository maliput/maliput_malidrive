// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lanes.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Lanes, EqualityOperator) {
  const LaneOffset kLaneOffset{2.1, 2.2};
  const LaneSection kLaneSection{
      2.1 /* s_0 */,
      true /* single_side */,
      {{Lane::Id("test_id1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{1.1, 2.2}}}} /* left_lanes */,
      {Lane::Id("test_id2"), Lane::Type::kShoulder, false, {}, {}} /* center_lane */,
      {{Lane::Id("test_id3"), Lane::Type::kRestricted, false, {}, std::vector<LaneWidth>{{5.5, 6.6}}}} /* right_lanes */
  };
  const Lanes kLanes{{{kLaneOffset}}, {{kLaneSection}}};

  Lanes lanes = kLanes;
  EXPECT_EQ(kLanes, lanes);
  lanes.lanes_offset[0].s_0 = 5.;
  EXPECT_NE(kLanes, lanes);
  lanes.lanes_offset[0].s_0 = 2.1;
  lanes.lanes_section[0].left_lanes[0].width_description[0].s_0 = 5.;
  EXPECT_NE(kLanes, lanes);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
