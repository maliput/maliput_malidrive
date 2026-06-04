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
#include "maliput_malidrive/traffic_control_device/device_type.h"

#include <string>
#include <utility>

#include <gtest/gtest.h>

namespace malidrive {
namespace traffic_control_device {
namespace test {
namespace {

// ---------------------------------------------------------------------------
// StringToTrafficControlDeviceType — known values
// ---------------------------------------------------------------------------

struct StringToTypeTestCase {
  std::string input;
  TrafficControlDeviceType expected;
};

class StringToTrafficControlDeviceTypeTest : public ::testing::TestWithParam<StringToTypeTestCase> {};

TEST_P(StringToTrafficControlDeviceTypeTest, MapsCorrectly) {
  EXPECT_EQ(GetParam().expected, StringToTrafficControlDeviceType(GetParam().input));
}

INSTANTIATE_TEST_CASE_P(KnownValues, StringToTrafficControlDeviceTypeTest,
                        ::testing::Values(
                            // Canonical CamelCase forms.
                            StringToTypeTestCase{"TrafficLight", TrafficControlDeviceType::kTrafficLight},
                            StringToTypeTestCase{"TrafficSign", TrafficControlDeviceType::kTrafficSign},
                            StringToTypeTestCase{"RoadMarking", TrafficControlDeviceType::kRoadMarking},
                            StringToTypeTestCase{"RoadObject", TrafficControlDeviceType::kRoadObject}));

INSTANTIATE_TEST_CASE_P(StrictCamelCase, StringToTrafficControlDeviceTypeTest,
                        ::testing::Values(
                            // Non-CamelCase variants are rejected.
                            StringToTypeTestCase{"TRAFFIC_LIGHT", TrafficControlDeviceType::kUnknown},
                            StringToTypeTestCase{"traffic_light", TrafficControlDeviceType::kUnknown},
                            StringToTypeTestCase{"Traffic_Light", TrafficControlDeviceType::kUnknown},
                            StringToTypeTestCase{"Road_Marking", TrafficControlDeviceType::kUnknown}));

INSTANTIATE_TEST_CASE_P(UnknownValues, StringToTrafficControlDeviceTypeTest,
                        ::testing::Values(
                            // Unrecognized strings map to kUnknown.
                            StringToTypeTestCase{"", TrafficControlDeviceType::kUnknown},
                            StringToTypeTestCase{"Other", TrafficControlDeviceType::kUnknown},
                            StringToTypeTestCase{"unknown_value", TrafficControlDeviceType::kUnknown}));

// ---------------------------------------------------------------------------
// TrafficControlDeviceTypeToString — all enum values
// ---------------------------------------------------------------------------

struct TypeToStringTestCase {
  TrafficControlDeviceType input;
  std::string expected;
};

class TrafficControlDeviceTypeToStringTest : public ::testing::TestWithParam<TypeToStringTestCase> {};

TEST_P(TrafficControlDeviceTypeToStringTest, MapsCorrectly) {
  EXPECT_EQ(GetParam().expected, std::string(TrafficControlDeviceTypeToString(GetParam().input)));
}

INSTANTIATE_TEST_CASE_P(AllValues, TrafficControlDeviceTypeToStringTest,
                        ::testing::Values(TypeToStringTestCase{TrafficControlDeviceType::kTrafficLight, "TrafficLight"},
                                          TypeToStringTestCase{TrafficControlDeviceType::kTrafficSign, "TrafficSign"},
                                          TypeToStringTestCase{TrafficControlDeviceType::kRoadMarking, "RoadMarking"},
                                          TypeToStringTestCase{TrafficControlDeviceType::kRoadObject, "RoadObject"},
                                          TypeToStringTestCase{TrafficControlDeviceType::kUnknown, "Unknown"}));

// ---------------------------------------------------------------------------
// Round-trip: StringToTrafficControlDeviceType ∘ TrafficControlDeviceTypeToString
// ---------------------------------------------------------------------------

TEST(TrafficControlDeviceTypeRoundTripTest, TrafficLight) {
  const auto type = TrafficControlDeviceType::kTrafficLight;
  EXPECT_EQ(type, StringToTrafficControlDeviceType(TrafficControlDeviceTypeToString(type)));
}

TEST(TrafficControlDeviceTypeRoundTripTest, TrafficSign) {
  const auto type = TrafficControlDeviceType::kTrafficSign;
  EXPECT_EQ(type, StringToTrafficControlDeviceType(TrafficControlDeviceTypeToString(type)));
}

TEST(TrafficControlDeviceTypeRoundTripTest, RoadMarking) {
  const auto type = TrafficControlDeviceType::kRoadMarking;
  EXPECT_EQ(type, StringToTrafficControlDeviceType(TrafficControlDeviceTypeToString(type)));
}

TEST(TrafficControlDeviceTypeRoundTripTest, RoadObject) {
  const auto type = TrafficControlDeviceType::kRoadObject;
  EXPECT_EQ(type, StringToTrafficControlDeviceType(TrafficControlDeviceTypeToString(type)));
}

}  // namespace
}  // namespace test
}  // namespace traffic_control_device
}  // namespace malidrive
