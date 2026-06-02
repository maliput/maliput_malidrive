// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include "maliput_malidrive/builder/road_marking_type_mapper.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace builder {
namespace test {
namespace {

using T = maliput::api::objects::RoadMarkingType;

using RoadMarkingTypeMapperTest = ::testing::Test;

TEST_F(RoadMarkingTypeMapperTest, DirectMappings) {
  EXPECT_EQ(T::kStop, MapRoadMarkingTypeString("stop"));
  EXPECT_EQ(T::kStopLine, MapRoadMarkingTypeString("stop_line"));
  EXPECT_EQ(T::kCrosswalk, MapRoadMarkingTypeString("crosswalk"));
  EXPECT_EQ(T::kCrosswalk, MapRoadMarkingTypeString("zebra_crossing"));
  EXPECT_EQ(T::kParkingSpace, MapRoadMarkingTypeString("parking_space"));
  EXPECT_EQ(T::kEmergencyLane, MapRoadMarkingTypeString("emergency_lane"));
  EXPECT_EQ(T::kSpeedLimit, MapRoadMarkingTypeString("speed_limit"));
  EXPECT_EQ(T::kDoNotStop, MapRoadMarkingTypeString("no_stopping"));
  EXPECT_EQ(T::kRailRoad, MapRoadMarkingTypeString("rail_road_crossing"));
  EXPECT_EQ(T::kGiveWay, MapRoadMarkingTypeString("give_way"));
}

TEST_F(RoadMarkingTypeMapperTest, ArrowMappings) {
  EXPECT_EQ(T::kArrowTurnRight, MapRoadMarkingTypeString("arrow_turn_right"));
  EXPECT_EQ(T::kArrowTurnLeft, MapRoadMarkingTypeString("arrow_turn_left"));
  EXPECT_EQ(T::kArrowForwardTurnRight, MapRoadMarkingTypeString("arrow_forward_turn_right"));
  EXPECT_EQ(T::kArrowForwardTurnLeft, MapRoadMarkingTypeString("arrow_forward_turn_left"));
  EXPECT_EQ(T::kArrowForward, MapRoadMarkingTypeString("arrow_forward"));
  EXPECT_EQ(T::kArrowForwardTurnRightTurnLeft, MapRoadMarkingTypeString("arrow_forward_turn_right_turn_left"));
  EXPECT_EQ(T::kArrowTurnRightTurnLeft, MapRoadMarkingTypeString("arrow_turn_right_turn_left"));
  EXPECT_EQ(T::kArrowUTurnRight, MapRoadMarkingTypeString("arrow_u_turn_right"));
  EXPECT_EQ(T::kArrowUTurnLeft, MapRoadMarkingTypeString("arrow_u_turn_left"));
}

TEST_F(RoadMarkingTypeMapperTest, CaseInsensitive) {
  EXPECT_EQ(T::kStopLine, MapRoadMarkingTypeString("Stop_Line"));
  EXPECT_EQ(T::kStopLine, MapRoadMarkingTypeString("STOP_LINE"));
  EXPECT_EQ(T::kCrosswalk, MapRoadMarkingTypeString("CROSSWALK"));
  EXPECT_EQ(T::kArrowForward, MapRoadMarkingTypeString("Arrow_Forward"));
  EXPECT_EQ(T::kGiveWay, MapRoadMarkingTypeString("Give_Way"));
}

TEST_F(RoadMarkingTypeMapperTest, UnknownMappings) {
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("other"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("none"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString(""));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("unknown_value"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("traffic_light"));
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
