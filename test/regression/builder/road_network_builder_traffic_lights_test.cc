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
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/traffic_lights.h>
#include <maliput/base/traffic_light_book.h>

#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

class RoadNetworkBuilderTrafficLightsTest : public ::testing::Test {
 protected:
  const std::string xodr_file_path_{
      utility::FindResourceInPath("figure8_trafficlights/figure8_trafficlights.xodr", kMalidriveResourceFolder)};
  const std::string traffic_signal_db_path_{
      utility::FindResourceInPath("traffic_signal_db/traffic_signal_db_example.yaml", kMalidriveResourceFolder)};
  const std::string traffic_light_book_path_{
      utility::FindResourceInPath("figure8_trafficlights/figure8_trafficlights.yaml", kMalidriveResourceFolder)};
};

// Builds a RoadNetwork with the traffic_signal_db parameter and verifies that
// traffic lights are created from XODR signals + YAML database.
// The figure8_trafficlights XODR has 4 signals (ids: 109, 110, 111, 112),
// all of type "1000001" which is defined in the YAML database.
TEST_F(RoadNetworkBuilderTrafficLightsTest, BuildWithTrafficLightsFromYaml) {
  const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
      {params::kOpendriveFile, xodr_file_path_},
      {params::kTrafficSignalDb, traffic_signal_db_path_},
      {params::kOmitNonDrivableLanes, "false"},
  })};
  std::unique_ptr<maliput::api::RoadNetwork> dut;
  dut = RoadNetworkBuilder(rn_config.ToStringMap())();
  ASSERT_NE(dut, nullptr);

  const auto* traffic_light_book = dut->traffic_light_book();
  ASSERT_NE(traffic_light_book, nullptr);

  // All 4 signals should have been created as traffic lights.
  const auto traffic_lights = traffic_light_book->TrafficLights();
  EXPECT_EQ(4u, traffic_lights.size());

  // Verify each signal ID is present. The traffic light IDs equal the signal IDs.
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("109")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("110")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("111")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("112")), nullptr);

  // Verify each traffic light has the expected structure (3 bulbs per group).
  for (const auto* tl : traffic_lights) {
    ASSERT_NE(tl, nullptr);
    const auto bulb_groups = tl->bulb_groups();
    ASSERT_EQ(1u, bulb_groups.size());
    const auto bulbs = bulb_groups[0]->bulbs();
    EXPECT_EQ(3u, bulbs.size());
  }
}

// Builds a RoadNetwork without the traffic_signal_db parameter. The traffic
// light book should be empty because only the YAML database path is omitted.
TEST_F(RoadNetworkBuilderTrafficLightsTest, BuildWithoutTrafficLightsFromYaml) {
  const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
      {params::kOpendriveFile, xodr_file_path_},
      {params::kOmitNonDrivableLanes, "false"},
  })};
  std::unique_ptr<maliput::api::RoadNetwork> dut;
  ASSERT_NO_THROW(dut = RoadNetworkBuilder(rn_config.ToStringMap())());
  ASSERT_NE(dut, nullptr);

  const auto* traffic_light_book = dut->traffic_light_book();
  ASSERT_NE(traffic_light_book, nullptr);

  // No traffic_light_book path and no traffic_signal_db → empty book.
  const auto traffic_lights = traffic_light_book->TrafficLights();
  EXPECT_TRUE(traffic_lights.empty());
}

// Builds a RoadNetwork with BOTH the traffic_light_book file (maliput YAML
// format) and the traffic_signal_db. Both sources should contribute traffic
// lights to the book.
TEST_F(RoadNetworkBuilderTrafficLightsTest, BuildWithTrafficLightsFromBookAndYaml) {
  const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
      {params::kOpendriveFile, xodr_file_path_},
      {params::kTrafficLightBook, traffic_light_book_path_},
      {params::kTrafficSignalDb, traffic_signal_db_path_},
      {params::kOmitNonDrivableLanes, "false"},
  })};
  std::unique_ptr<maliput::api::RoadNetwork> dut;
  ASSERT_NO_THROW(dut = RoadNetworkBuilder(rn_config.ToStringMap())());
  ASSERT_NE(dut, nullptr);

  const auto* traffic_light_book = dut->traffic_light_book();
  ASSERT_NE(traffic_light_book, nullptr);

  const auto traffic_lights = traffic_light_book->TrafficLights();
  // The traffic_light_book file for figure8 defines 4 traffic lights
  // (EastFacing, NorthFacing, SouthFacing, WestFacing).
  // The traffic_signal_db also creates 4 traffic lights from XODR signals
  // (109, 110, 111, 112).
  // Total should be 8.
  EXPECT_EQ(8u, traffic_lights.size());

  // Verify the file-based traffic lights are present.
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("EastFacing")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("NorthFacing")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("SouthFacing")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("WestFacing")), nullptr);

  // Verify the YAML-database-based traffic lights are also present.
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("109")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("110")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("111")), nullptr);
  EXPECT_NE(traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("112")), nullptr);
}

// Builds a RoadNetwork with all XODR signals matched by the YAML database.
// All 4 signals in figure8_trafficlights.xodr are type "1000001", which is
// defined in the database.
TEST_F(RoadNetworkBuilderTrafficLightsTest, BuildWithMultipleSignals) {
  const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
      {params::kOpendriveFile, xodr_file_path_},
      {params::kTrafficSignalDb, traffic_signal_db_path_},
      {params::kOmitNonDrivableLanes, "false"},
  })};
  std::unique_ptr<maliput::api::RoadNetwork> dut;
  ASSERT_NO_THROW(dut = RoadNetworkBuilder(rn_config.ToStringMap())());
  ASSERT_NE(dut, nullptr);

  const auto* traffic_light_book = dut->traffic_light_book();
  ASSERT_NE(traffic_light_book, nullptr);

  // All 4 signals are type "1000001" -> all should be matched.
  const auto traffic_lights = traffic_light_book->TrafficLights();
  EXPECT_EQ(4u, traffic_lights.size());

  // Verify each one has the expected 3-bulb structure.
  for (const auto* tl : traffic_lights) {
    ASSERT_NE(tl, nullptr);
    const auto bulb_groups = tl->bulb_groups();
    ASSERT_EQ(1u, bulb_groups.size());
    const auto* red = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("RedBulb"));
    const auto* yellow = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("YellowBulb"));
    const auto* green = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("GreenBulb"));
    EXPECT_NE(red, nullptr);
    EXPECT_NE(yellow, nullptr);
    EXPECT_NE(green, nullptr);
  }
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
