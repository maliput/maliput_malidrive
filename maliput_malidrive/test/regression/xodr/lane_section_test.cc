// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput_malidrive/xodr/lane_section.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(LaneSection, EqualityOperator) {
  const LaneSection kLaneSection{
      2.1 /* s_0 */,
      true /* single_side */,
      {{Lane::Id("test_id1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{1.1, 2.2}}}} /* left_lanes */,
      {Lane::Id("test_id2"), Lane::Type::kShoulder, false, {}, {}} /* center_lane */,
      {{Lane::Id("test_id3"), Lane::Type::kRestricted, false, {}, std::vector<LaneWidth>{{5.5, 6.6}}}} /* right_lanes */
  };
  LaneSection lane_section = kLaneSection;
  EXPECT_EQ(kLaneSection, lane_section);
  lane_section.s_0 = 993.;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.s_0 = 2.1;
  lane_section.single_side = false;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.single_side = true;
  lane_section.left_lanes[0].type = Lane::Type::kEntry;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.left_lanes[0].type = Lane::Type::kDriving;
  lane_section.center_lane.type = Lane::Type::kSidewalk;
  EXPECT_NE(kLaneSection, lane_section);
  lane_section.center_lane.type = Lane::Type::kShoulder;
  lane_section.right_lanes[0].type = Lane::Type::kRoadWorks;
  EXPECT_NE(kLaneSection, lane_section);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
