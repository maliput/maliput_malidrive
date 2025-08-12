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
#include <maliput/common/error.h>

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

// Holds the Lane::Type and its string representation.
struct LaneTypeAndStrType {
  Lane::Type type{};
  std::string str_type{};
};

// Provides a list with all the Lane::Type and its string representations.
std::vector<LaneTypeAndStrType> GetTestingLaneTypeAndStringTypes() {
  return std::vector<LaneTypeAndStrType>{
      {Lane::Type::kNone, "none"},
      {Lane::Type::kDriving, "driving"},
      {Lane::Type::kStop, "stop"},
      {Lane::Type::kShoulder, "shoulder"},
      {Lane::Type::kBiking, "biking"},
      {Lane::Type::kSidewalk, "sidewalk"},
      {Lane::Type::kBorder, "border"},
      {Lane::Type::kRestricted, "restricted"},
      {Lane::Type::kParking, "parking"},
      {Lane::Type::kCurb, "curb"},
      {Lane::Type::kBidirectional, "bidirectional"},
      {Lane::Type::kMedian, "median"},
      {Lane::Type::kSpecial1, "special1"},
      {Lane::Type::kSpecial2, "special2"},
      {Lane::Type::kSpecial3, "special3"},
      {Lane::Type::kRoadWorks, "roadworks"},
      {Lane::Type::kTram, "tram"},
      {Lane::Type::kRail, "rail"},
      {Lane::Type::kEntry, "entry"},
      {Lane::Type::kExit, "exit"},
      {Lane::Type::kOffRamp, "offRamp"},
      {Lane::Type::kOnRamp, "onRamp"},
      {Lane::Type::kConnectingRamp, "connectingRamp"},
      {Lane::Type::kBus, "bus"},
      {Lane::Type::kTaxi, "taxi"},
      {Lane::Type::kHOV, "hov"},
      {Lane::Type::kMwyEntry, "mwyEntry"},
      {Lane::Type::kMwyExit, "mwyExit"},
  };
}

// Tests Lane::Type to and from string conversion.
class TypeStringConversionTest : public ::testing::TestWithParam<LaneTypeAndStrType> {
 public:
  const Lane::Type type_{GetParam().type};
  const std::string str_type_{GetParam().str_type};
};

TEST_P(TypeStringConversionTest, TypeToStr) { ASSERT_EQ(str_type_, Lane::type_to_str(type_)); }

TEST_P(TypeStringConversionTest, StrToType) { ASSERT_EQ(type_, Lane::str_to_type(str_type_)); }

GTEST_TEST(LaneTypeTest, StrToTypeThrowsWithUnknownToken) {
  const std::string kWrongValue{"WrongValue"};
  EXPECT_THROW(Lane::str_to_type(kWrongValue), maliput::common::assertion_error);
}

INSTANTIATE_TEST_CASE_P(GroupTypeStringConversionTest, TypeStringConversionTest,
                        ::testing::ValuesIn(GetTestingLaneTypeAndStringTypes()));

// Holds the Lane::Advisory and its string representation.
struct LaneAdvisoryAndStrAdvisory {
  Lane::Advisory advisory{};
  std::string str_advisory{};
};

std::vector<LaneAdvisoryAndStrAdvisory> GetTestingLaneAdvisoryAndStringAdvisories() {
  return std::vector<LaneAdvisoryAndStrAdvisory>{
      {Lane::Advisory::kNone, "none"},
      {Lane::Advisory::kInner, "inner"},
      {Lane::Advisory::kOuter, "outer"},
      {Lane::Advisory::kBoth, "both"},
  };
}

// Tests Lane::Advisory to and from string conversion.
class AdvisoryStringConversionTest : public ::testing::TestWithParam<LaneAdvisoryAndStrAdvisory> {
 public:
  const Lane::Advisory advisory_{GetParam().advisory};
  const std::string str_advisory_{GetParam().str_advisory};
};

TEST_P(AdvisoryStringConversionTest, AdvisoryToStr) { ASSERT_EQ(str_advisory_, Lane::advisory_to_str(advisory_)); }

TEST_P(AdvisoryStringConversionTest, StrToAdvisory) { ASSERT_EQ(advisory_, Lane::str_to_advisory(str_advisory_)); }

GTEST_TEST(LaneAdvisoryTest, StrToAdvisoryThrowsWithUnknownToken) {
  const std::string kWrongValue{"WrongValue"};
  EXPECT_THROW(Lane::str_to_advisory(kWrongValue), maliput::common::assertion_error);
}

INSTANTIATE_TEST_CASE_P(GroupAdvisoryStringConversionTest, AdvisoryStringConversionTest,
                        ::testing::ValuesIn(GetTestingLaneAdvisoryAndStringAdvisories()));

// Holds the Lane::Direction and its string representation.
struct LaneDirectionAndStrDirection {
  Lane::Direction direction{};
  std::string str_direction{};
};

std::vector<LaneDirectionAndStrDirection> GetTestingLaneDirectionAndStringDirections() {
  return std::vector<LaneDirectionAndStrDirection>{
      {Lane::Direction::kStandard, "standard"},
      {Lane::Direction::kReversed, "reversed"},
      {Lane::Direction::kBoth, "both"},
  };
}

// Tests Lane::Direction to and from string conversion.
class DirectionStringConversionTest : public ::testing::TestWithParam<LaneDirectionAndStrDirection> {
 public:
  const Lane::Direction direction_{GetParam().direction};
  const std::string str_direction_{GetParam().str_direction};
};

TEST_P(DirectionStringConversionTest, DirectionToStr) { ASSERT_EQ(str_direction_, Lane::direction_to_str(direction_)); }

TEST_P(DirectionStringConversionTest, StrToDirection) { ASSERT_EQ(direction_, Lane::str_to_direction(str_direction_)); }

GTEST_TEST(LaneDirectionTest, StrToDirectionThrowsWithUnknownToken) {
  const std::string kWrongValue{"WrongValue"};
  EXPECT_THROW(Lane::str_to_direction(kWrongValue), maliput::common::assertion_error);
}

INSTANTIATE_TEST_CASE_P(GroupDirectionStringConversionTest, DirectionStringConversionTest,
                        ::testing::ValuesIn(GetTestingLaneDirectionAndStringDirections()));

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
      std::nullopt /* userData */,
      Lane::Advisory::kOuter /* advisory */,
      Lane::Direction::kStandard /* direction */,
      false /* dynamic_lane_direction */,
      false /* dynamic_lane_type */,
      true /* road_works */
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
  lane.advisory = Lane::Advisory::kInner;
  EXPECT_NE(kLane, lane);
  lane.advisory = Lane::Advisory::kOuter;
  EXPECT_EQ(kLane, lane);
  lane.direction = Lane::Direction::kReversed;
  EXPECT_NE(kLane, lane);
  lane.direction = Lane::Direction::kStandard;
  EXPECT_EQ(kLane, lane);
  lane.dynamic_lane_direction = std::make_optional<bool>(true);
  EXPECT_NE(kLane, lane);
  lane.dynamic_lane_direction = std::make_optional<bool>(false);
  EXPECT_EQ(kLane, lane);
  lane.dynamic_lane_type = std::make_optional<bool>(true);
  EXPECT_NE(kLane, lane);
  lane.dynamic_lane_type = std::make_optional<bool>(false);
  EXPECT_EQ(kLane, lane);
  lane.road_works = std::make_optional<bool>(false);
  EXPECT_NE(kLane, lane);
  lane.road_works = std::make_optional<bool>(true);
  EXPECT_EQ(kLane, lane);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
