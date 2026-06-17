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
  EXPECT_EQ(T::kStop, MapRoadMarkingTypeString("Stop"));
  EXPECT_EQ(T::kStopLine, MapRoadMarkingTypeString("StopLine"));
  EXPECT_EQ(T::kCrosswalk, MapRoadMarkingTypeString("Crosswalk"));
  EXPECT_EQ(T::kCrosswalk, MapRoadMarkingTypeString("ZebraCrossing"));
  EXPECT_EQ(T::kCarParking, MapRoadMarkingTypeString("ParkingSpace"));
  EXPECT_EQ(T::kEmergencyLane, MapRoadMarkingTypeString("EmergencyLane"));
  EXPECT_EQ(T::kSpeedLimit, MapRoadMarkingTypeString("SpeedLimit"));
  EXPECT_EQ(T::kNoStopping, MapRoadMarkingTypeString("NoStopping"));
  EXPECT_EQ(T::kRailroadCrossing, MapRoadMarkingTypeString("RailRoadCrossing"));
  EXPECT_EQ(T::kGiveWay, MapRoadMarkingTypeString("GiveWay"));
}

TEST_F(RoadMarkingTypeMapperTest, ArrowMappings) {
  EXPECT_EQ(T::kPrescribedRightTurn, MapRoadMarkingTypeString("ArrowTurnRight"));
  EXPECT_EQ(T::kPrescribedLeftTurn, MapRoadMarkingTypeString("ArrowTurnLeft"));
  EXPECT_EQ(T::kPrescribedRightTurnAndStraight, MapRoadMarkingTypeString("ArrowForwardTurnRight"));
  EXPECT_EQ(T::kPrescribedLeftTurnAndStraight, MapRoadMarkingTypeString("ArrowForwardTurnLeft"));
  EXPECT_EQ(T::kPrescribedStraight, MapRoadMarkingTypeString("ArrowForward"));
  EXPECT_EQ(T::kPrescribedLeftTurnRightTurnAndStraight, MapRoadMarkingTypeString("ArrowForwardTurnRightTurnLeft"));
  EXPECT_EQ(T::kPrescribedLeftTurnAndRightTurn, MapRoadMarkingTypeString("ArrowTurnRightTurnLeft"));
  EXPECT_EQ(T::kPrescribedUTurnRight, MapRoadMarkingTypeString("ArrowUTurnRight"));
  EXPECT_EQ(T::kPrescribedUTurnLeft, MapRoadMarkingTypeString("ArrowUTurnLeft"));
}

TEST_F(RoadMarkingTypeMapperTest, StrictPascalCase) {
  EXPECT_EQ(T::kStopLine, MapRoadMarkingTypeString("StopLine"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("Stop_Line"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("STOP_LINE"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("CROSSWALK"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("Arrow_Forward"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("Give_Way"));
}

TEST_F(RoadMarkingTypeMapperTest, UnknownMappings) {
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("Other"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("other"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("none"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString(""));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("unknown_value"));
  EXPECT_EQ(T::kUnknown, MapRoadMarkingTypeString("TrafficLight"));
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
