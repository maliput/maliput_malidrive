// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/lane_link.h"

#include <gtest/gtest.h>

#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(LaneLinkTest, EqualityOperator) {
  const LaneLink::LinkAttributes kPredecessor{LaneLink::LinkAttributes::Id{"50"} /* Id */};
  const LaneLink::LinkAttributes kSuccessor{LaneLink::LinkAttributes::Id{"100"} /* Id */};
  const LaneLink kLaneLink{kPredecessor, kSuccessor};
  LaneLink dut = kLaneLink;

  EXPECT_EQ(kLaneLink, dut);
  dut.predecessor->id = LaneLink::LinkAttributes::Id("25");
  EXPECT_NE(kLaneLink, dut);
  dut.predecessor->id = LaneLink::LinkAttributes::Id("50");
  dut.successor->id = LaneLink::LinkAttributes::Id("25");
  EXPECT_NE(kLaneLink, dut);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
