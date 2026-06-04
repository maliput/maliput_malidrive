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
#include "maliput_malidrive/builder/traffic_sign_type_mapper.h"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/rules/traffic_sign.h>

namespace malidrive {
namespace builder {
namespace test {
namespace {

using maliput::api::rules::TrafficSignType;

struct MapSignTypeStringTestCase {
  std::string input;
  TrafficSignType expected;
};

class MapSignTypeStringTest : public ::testing::TestWithParam<MapSignTypeStringTestCase> {};

TEST_P(MapSignTypeStringTest, MapsCorrectly) {
  const auto& test_case = GetParam();
  EXPECT_EQ(test_case.expected, MapSignTypeString(test_case.input));
}

INSTANTIATE_TEST_CASE_P(AllKnownTypes, MapSignTypeStringTest,
                        ::testing::Values(
                            // Known mappings.
                            MapSignTypeStringTestCase{"Stop", TrafficSignType::kStop},
                            MapSignTypeStringTestCase{"Yield", TrafficSignType::kYield},
                            MapSignTypeStringTestCase{"SpeedLimit", TrafficSignType::kSpeedLimit},
                            MapSignTypeStringTestCase{"NoEntry", TrafficSignType::kNoEntry},
                            MapSignTypeStringTestCase{"OneWay", TrafficSignType::kOneWay},
                            MapSignTypeStringTestCase{"PedestrianCrossing", TrafficSignType::kPedestrianCrossing},
                            MapSignTypeStringTestCase{"NoLeftTurn", TrafficSignType::kNoLeftTurn},
                            MapSignTypeStringTestCase{"NoRightTurn", TrafficSignType::kNoRightTurn},
                            MapSignTypeStringTestCase{"NoUTurn", TrafficSignType::kNoUTurn},
                            MapSignTypeStringTestCase{"SchoolZone", TrafficSignType::kSchoolZone},
                            MapSignTypeStringTestCase{"Construction", TrafficSignType::kConstruction},
                            MapSignTypeStringTestCase{"RailroadCrossing", TrafficSignType::kRailroadCrossing},
                            MapSignTypeStringTestCase{"NoOvertaking", TrafficSignType::kNoOvertaking},
                            MapSignTypeStringTestCase{"GiveWay", TrafficSignType::kYield},
                            MapSignTypeStringTestCase{"SpeedLimitBegin", TrafficSignType::kSpeedLimit},
                            MapSignTypeStringTestCase{"Crosswalk", TrafficSignType::kPedestrianCrossing}));

INSTANTIATE_TEST_CASE_P(UnknownTypes, MapSignTypeStringTest,
                        ::testing::Values(
                            // Unrecognized strings map to kUnknown.
                            MapSignTypeStringTestCase{"TrafficLight", TrafficSignType::kUnknown},
                            MapSignTypeStringTestCase{"traffic_light", TrafficSignType::kUnknown},
                            MapSignTypeStringTestCase{"", TrafficSignType::kUnknown},
                            MapSignTypeStringTestCase{"unknown_value", TrafficSignType::kUnknown}));

INSTANTIATE_TEST_CASE_P(StrictPascalCase, MapSignTypeStringTest,
                        ::testing::Values(
                            // Non-PascalCase variants are rejected.
                            MapSignTypeStringTestCase{"Stop", TrafficSignType::kStop},
                            MapSignTypeStringTestCase{"STOP", TrafficSignType::kUnknown},
                            MapSignTypeStringTestCase{"Yield", TrafficSignType::kYield},
                            MapSignTypeStringTestCase{"speed_limit", TrafficSignType::kUnknown},
                            MapSignTypeStringTestCase{"No_Overtaking", TrafficSignType::kUnknown}));

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
