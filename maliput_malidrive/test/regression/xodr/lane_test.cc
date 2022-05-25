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
#include "maliput_malidrive/xodr/lane.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(LaneSpeed, EqualityOperator) {
  const Lane::Speed kSpeed{0.1 /* s_offset */, 45. /* max */, Unit::kMph /* unit */};
  Lane::Speed speed = kSpeed;

  EXPECT_EQ(kSpeed, speed);
  speed.s_offset = 5.;
  EXPECT_NE(kSpeed, speed);
  speed.s_offset = 0.1;
  speed.max = 150.;
  EXPECT_NE(kSpeed, speed);
  speed.max = 45.;
  speed.unit = Unit::kMs;
  EXPECT_NE(kSpeed, speed);
  speed.unit = Unit::kMph;
  EXPECT_EQ(kSpeed, speed);
};

GTEST_TEST(Lane, TypeToStr) {
  const std::string kExpected1{"driving"};
  const std::string kExpected2{"shoulder"};
  EXPECT_EQ(kExpected1, Lane::type_to_str(Lane::Type::kDriving));
  EXPECT_EQ(kExpected2, Lane::type_to_str(Lane::Type::kShoulder));
}

GTEST_TEST(Lane, StrToType) {
  const std::string kDriving{"driving"};
  const std::string kShoulder{"shoulder"};
  EXPECT_EQ(Lane::Type::kDriving, Lane::str_to_type(kDriving));
  EXPECT_EQ(Lane::Type::kShoulder, Lane::str_to_type(kShoulder));
  const std::string kWrongValue{"WrongValue"};
  EXPECT_THROW(Lane::str_to_type(kWrongValue), maliput::common::assertion_error);
}

GTEST_TEST(Lane, EqualityOperator) {
  const LaneLink lane_link{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id("35")},
                           LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id("70")}};
  const Lane kLane{
      Lane::Id("test_id") /* id */,
      Lane::Type::kDriving /* type */,
      false /* level */,
      lane_link /* lane_link */,
      {{1.1 /* sOffset */, 2.2 /* a */}, {6.6 /* sOffset */, 7.7 /* a */}} /* widths */,
      {{0. /* sOffset */, 15. /* max */, Unit::kMph /* unit */}} /* speed */,
      std::nullopt /* userData */
  };
  Lane lane = kLane;

  EXPECT_EQ(kLane, lane);
  lane.id = Lane::Id("different_id");
  EXPECT_NE(kLane, lane);
  lane.id = Lane::Id("test_id");
  lane.type = Lane::Type::kShoulder;
  EXPECT_NE(kLane, lane);
  lane.type = Lane::Type::kDriving;
  lane.level = true;
  EXPECT_NE(kLane, lane);
  lane.level = false;
  lane.lane_link.predecessor->id = LaneLink::LinkAttributes::Id("150");
  EXPECT_NE(kLane, lane);
  lane.lane_link.predecessor->id = LaneLink::LinkAttributes::Id("35");
  lane.width_description[0].s_0 = 5.;
  EXPECT_NE(kLane, lane);
  lane.width_description[0].s_0 = 1.1;
  lane.speed[0].max = 35.;
  EXPECT_NE(kLane, lane);
  lane.speed[0].max = 15.;
  lane.user_data = std::make_optional<std::string>("<root></root>");
  EXPECT_NE(kLane, lane);
  lane.user_data = std::nullopt;
  EXPECT_EQ(kLane, lane);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
