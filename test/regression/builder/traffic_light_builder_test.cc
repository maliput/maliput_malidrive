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
#include "maliput_malidrive/builder/traffic_light_builder.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/traffic_lights.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/signal/signal.h"
#include "maliput_malidrive/xodr/signal/signal_reference.h"
#include "maliput_malidrive/xodr/validity.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

// Test fixture for TrafficLightBuilder.
// Builds a RoadNetwork from the figure8_trafficlights XODR (which contains 4
// signals, all of type "1000001") and prepares a TrafficSignalDatabaseLoader backed by
// the example YAML database.
class TrafficLightBuilderFigure8Test : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("figure8_trafficlights/figure8_trafficlights.xodr", kMalidriveResourceFolder);
    traffic_signal_db_path_ =
        utility::FindResourceInPath("traffic_signal_db/traffic_signal_db_example.yaml", kMalidriveResourceFolder);

    const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
        {params::kOpendriveFile, xodr_file_path},
        {params::kTrafficSignalDb, traffic_signal_db_path_},
        {params::kOmitNonDrivableLanes, "false"},
    })};
    road_network_ = RoadNetworkBuilder(rn_config.ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_geometry_ = dynamic_cast<const malidrive::RoadGeometry*>(road_network_->road_geometry());
    ASSERT_NE(road_geometry_, nullptr);
    const auto manager = road_geometry_->get_manager();
    ASSERT_NE(manager, nullptr);
    // Road 44 contains signal id="111", type="1000001", subtype="-1", country="OpenDRIVE".
    const auto road_headers = manager->GetRoadHeaders();
    ASSERT_FALSE(road_headers.empty());
    const auto road_header = road_headers.find(xodr::RoadHeader::Id("44"));
    ASSERT_FALSE(road_header == road_headers.end());
    ASSERT_TRUE(road_header->second.signals.has_value());
    ASSERT_FALSE(road_header->second.signals->signals.empty());
    signal_ = road_header->second.signals->signals[0];
    loader_ = std::make_unique<traffic_signal::TrafficSignalDatabaseLoader>(traffic_signal_db_path_);
  }

  std::string traffic_signal_db_path_;
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const malidrive::RoadGeometry* road_geometry_{};
  xodr::signal::Signal signal_{0.0, 0.0, xodr::signal::Signal::Id("none")};
  std::unique_ptr<traffic_signal::TrafficSignalDatabaseLoader> loader_;
};

TEST_F(TrafficLightBuilderFigure8Test, Constructor) {
  EXPECT_NO_THROW(TrafficLightBuilder(signal_, xodr::RoadHeader::Id("44"), *loader_, road_geometry_));
}

TEST_F(TrafficLightBuilderFigure8Test, ConstructorThrows) {
  EXPECT_THROW(TrafficLightBuilder(signal_, xodr::RoadHeader::Id("44"), *loader_, nullptr), std::invalid_argument);
}

// Verifies that a TrafficLight is successfully created from a XODR signal that
// has a matching definition in the YAML database (type "1000001").
TEST_F(TrafficLightBuilderFigure8Test, BuildTrafficLight) {
  TrafficLightBuilder builder(signal_, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_light = builder();
  ASSERT_NE(traffic_light, nullptr);

  // The TrafficLight ID should be the signal ID.
  EXPECT_EQ(traffic_light->id(), maliput::api::rules::TrafficLight::Id(signal_.id.string()));

  // There should be one BulbGroup.
  const auto bulb_groups = traffic_light->bulb_groups();
  ASSERT_EQ(1u, bulb_groups.size());
  EXPECT_NE(bulb_groups[0], nullptr);

  // The BulbGroup should contain 3 bulbs (Red, Yellow, Green) as defined in the
  // YAML database for type "1000001".
  const auto bulbs = bulb_groups[0]->bulbs();
  ASSERT_EQ(3u, bulbs.size());

  // Verify bulb IDs and colors.
  const auto* red_bulb = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("RedBulb"));
  ASSERT_NE(red_bulb, nullptr);
  EXPECT_EQ(maliput::api::rules::BulbColor::kRed, red_bulb->color());
  EXPECT_EQ(maliput::api::rules::BulbType::kRound, red_bulb->type());

  const auto* yellow_bulb = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("YellowBulb"));
  ASSERT_NE(yellow_bulb, nullptr);
  EXPECT_EQ(maliput::api::rules::BulbColor::kYellow, yellow_bulb->color());
  EXPECT_EQ(maliput::api::rules::BulbType::kRound, yellow_bulb->type());

  const auto* green_bulb = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("GreenBulb"));
  ASSERT_NE(green_bulb, nullptr);
  EXPECT_EQ(maliput::api::rules::BulbColor::kGreen, green_bulb->color());
  EXPECT_EQ(maliput::api::rules::BulbType::kRound, green_bulb->type());

  // Verify bulb states.
  // RedBulb: ["Off", "On", "Blinking"]
  const auto& red_states = red_bulb->states();
  EXPECT_EQ(3u, red_states.size());
  // GreenBulb: ["Off", "On"]
  const auto& green_states = green_bulb->states();
  EXPECT_EQ(2u, green_states.size());
}

// Verifies that the builder returns nullptr when no matching definition is found
// in the YAML database for the signal's type/subtype fingerprint.
TEST_F(TrafficLightBuilderFigure8Test, BuildTrafficLightNoMatchingDefinition) {
  // Create a signal with a type that doesn't exist in the YAML database.
  xodr::signal::Signal unknown_signal = signal_;
  unknown_signal.type = "UnknownType";

  TrafficLightBuilder builder(unknown_signal, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_light = builder();
  EXPECT_EQ(traffic_light, nullptr);
}

// Verifies that the builder returns nullptr when the matching definition in the
// YAML database has a sign_type other than "traffic_light" (e.g. "stop").
TEST_F(TrafficLightBuilderFigure8Test, BuildTrafficLightNonTrafficLightSignType) {
  // Type "206" exists in the YAML database with sign_type: "stop".
  xodr::signal::Signal stop_signal = signal_;
  stop_signal.type = "206";

  TrafficLightBuilder builder(stop_signal, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_light = builder();
  EXPECT_EQ(traffic_light, nullptr);
}

// Verifies the position of the traffic light.
TEST_F(TrafficLightBuilderFigure8Test, PositionOfTrafficLight) {
  TrafficLightBuilder builder(signal_, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_light = builder();
  ASSERT_NE(traffic_light, nullptr);

  const auto& position = traffic_light->position_road_network();
  EXPECT_NEAR(position.x(), 6.0667, 1e-3);
  EXPECT_NEAR(position.y(), -2.8325, 1e-3);
}

// ---------------------------------------------------------------------------
// Tests using TwoRoadsWithTrafficLights.xodr / traffic_signal_db/traffic_signal_db_example.yaml.
//
// This lightweight map defines two straight roads (Road 1 and Road 2), each
// with a single lane section (lanes {1, 0, -1}).
//
//   Road 1 (100 m, hdg=0, x=[0,100]):
//     - Signal S1: type="1000001", s=50, t=2, orientation="+",
//       validity={fromLane=-1, toLane=-1}, signalReference to S2.
//
//   Road 2 (80 m, hdg=0, x=[110,190]):
//     - Signal S2: type="1000001", s=40, t=-2, orientation="-",
//       no explicit validity (all lanes), signalReference to S1.
//
// The companion YAML database (traffic_signal_db_example.yaml) defines type
// "1000001" as a three-bulb vertical traffic light (RedBulb, YellowBulb,
// GreenBulb).
//
// Expected maliput lane IDs:
//   Road 1: "1_0_1" (left), "1_0_-1" (right)
//   Road 2: "2_0_1" (left), "2_0_-1" (right)
// ---------------------------------------------------------------------------

class TrafficLightBuilderTwoRoadsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file =
        utility::FindResourceInPath("TwoRoadsWithTrafficLights.xodr", kMalidriveResourceFolder);
    const std::string yaml_db =
        utility::FindResourceInPath("traffic_signal_db/traffic_signal_db_example.yaml", kMalidriveResourceFolder);

    road_network_ = RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                                             {params::kOpendriveFile, xodr_file},
                                                                             {params::kTrafficSignalDb, yaml_db},
                                                                             {params::kOmitNonDrivableLanes, "false"},
                                                                         })
                                           .ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    traffic_light_book_ = road_network_->traffic_light_book();
    ASSERT_NE(traffic_light_book_, nullptr);
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const maliput::api::rules::TrafficLightBook* traffic_light_book_{};
};

// Verifies that both signals produce a TrafficLight and no extra lights exist.
TEST_F(TrafficLightBuilderTwoRoadsTest, TwoTrafficLightsCreated) {
  const auto traffic_lights = traffic_light_book_->TrafficLights();
  EXPECT_EQ(2u, traffic_lights.size());
  EXPECT_NE(traffic_light_book_->GetTrafficLight(maliput::api::rules::TrafficLight::Id("S1")), nullptr);
  EXPECT_NE(traffic_light_book_->GetTrafficLight(maliput::api::rules::TrafficLight::Id("S2")), nullptr);
}

// Verifies all fields of the TrafficLight created from signal S1.
TEST_F(TrafficLightBuilderTwoRoadsTest, TrafficLightS1Fields) {
  const auto* tl = traffic_light_book_->GetTrafficLight(maliput::api::rules::TrafficLight::Id("S1"));
  ASSERT_NE(tl, nullptr);

  // ID.
  EXPECT_EQ(maliput::api::rules::TrafficLight::Id("S1"), tl->id());

  // Position: Road 1 is straight at y=0, hdg=0. Signal at s=50, t=2 → (50, 2, z_offset=6).
  const auto& pos = tl->position_road_network();
  EXPECT_NEAR(50., pos.x(), 1e-1);
  EXPECT_NEAR(2., pos.y(), 1e-1);
  EXPECT_NEAR(6., pos.z(), 1e-1);

  // Orientation: Road heading = 0, h_offset = 0, orientation = "+" → yaw ~ 0.
  const auto& rot = tl->orientation_road_network();
  EXPECT_NEAR(0., rot.roll(), 1e-3);
  EXPECT_NEAR(0., rot.pitch(), 1e-3);
  EXPECT_NEAR(0., rot.yaw(), 1e-1);

  // BulbGroups: one group named "TrafficLight_S1_Bulbs".
  const auto bulb_groups = tl->bulb_groups();
  ASSERT_EQ(1u, bulb_groups.size());
  EXPECT_EQ(maliput::api::rules::BulbGroup::Id("TrafficLight_S1_Bulbs"), bulb_groups[0]->id());

  // Bulbs: RedBulb, YellowBulb and GreenBulb (from traffic_signal_db_example.yaml, type "1000001").
  const auto bulbs = bulb_groups[0]->bulbs();
  ASSERT_EQ(3u, bulbs.size());

  const auto* red = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("RedBulb"));
  ASSERT_NE(red, nullptr);
  EXPECT_EQ(maliput::api::rules::BulbColor::kRed, red->color());
  EXPECT_EQ(maliput::api::rules::BulbType::kRound, red->type());
  EXPECT_EQ(3u, red->states().size());  // Off, On, Blinking
  // Position relative to traffic light frame.
  EXPECT_NEAR(0., red->position_bulb_group().x(), 1e-6);
  EXPECT_NEAR(0., red->position_bulb_group().y(), 1e-6);
  EXPECT_NEAR(0.4, red->position_bulb_group().z(), 1e-6);
  // Bounding box (explicitly set in the YAML).
  const auto& bb = red->bounding_box();
  EXPECT_NEAR(-0.0889, bb.p_BMin.x(), 1e-6);
  EXPECT_NEAR(-0.1778, bb.p_BMin.y(), 1e-6);
  EXPECT_NEAR(-0.1778, bb.p_BMin.z(), 1e-6);
  EXPECT_NEAR(0.0889, bb.p_BMax.x(), 1e-6);
  EXPECT_NEAR(0.1778, bb.p_BMax.y(), 1e-6);
  EXPECT_NEAR(0.1778, bb.p_BMax.z(), 1e-6);

  const auto* yellow = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("YellowBulb"));
  ASSERT_NE(yellow, nullptr);
  EXPECT_EQ(maliput::api::rules::BulbColor::kYellow, yellow->color());
  EXPECT_EQ(maliput::api::rules::BulbType::kRound, yellow->type());
  EXPECT_EQ(3u, yellow->states().size());  // Off, On, Blinking
  EXPECT_NEAR(0., yellow->position_bulb_group().x(), 1e-6);
  EXPECT_NEAR(0., yellow->position_bulb_group().y(), 1e-6);
  EXPECT_NEAR(0., yellow->position_bulb_group().z(), 1e-6);

  const auto* green = bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("GreenBulb"));
  ASSERT_NE(green, nullptr);
  EXPECT_EQ(maliput::api::rules::BulbColor::kGreen, green->color());
  EXPECT_EQ(maliput::api::rules::BulbType::kRound, green->type());
  EXPECT_EQ(2u, green->states().size());  // Off, On
  EXPECT_NEAR(0., green->position_bulb_group().x(), 1e-6);
  EXPECT_NEAR(0., green->position_bulb_group().y(), 1e-6);
  EXPECT_NEAR(-0.4, green->position_bulb_group().z(), 1e-6);

  // Related lanes for S1:
  //   - From signal's own validity (fromLane=-1, toLane=-1) on road 1 → {"1_0_-1"}
  //   - From signalReference to S2 on road 2 (no validity) → all lanes of road 2 at s=50 → {"2_0_1", "2_0_-1"}
  // Total (sorted): {"1_0_-1", "2_0_-1", "2_0_1"}
  const auto& related = tl->related_lanes();
  ASSERT_EQ(3u, related.size());
  std::vector<std::string> lane_ids;
  for (const auto& lid : related) {
    lane_ids.push_back(lid.string());
  }
  std::sort(lane_ids.begin(), lane_ids.end());
  EXPECT_EQ("1_0_-1", lane_ids[0]);
  EXPECT_EQ("2_0_-1", lane_ids[1]);
  EXPECT_EQ("2_0_1", lane_ids[2]);
}

// Verifies all fields of the TrafficLight created from signal S2.
TEST_F(TrafficLightBuilderTwoRoadsTest, TrafficLightS2Fields) {
  const auto* tl = traffic_light_book_->GetTrafficLight(maliput::api::rules::TrafficLight::Id("S2"));
  ASSERT_NE(tl, nullptr);

  // ID.
  EXPECT_EQ(maliput::api::rules::TrafficLight::Id("S2"), tl->id());

  // Position: Road 2 starts at x=110, hdg=0. Signal at s=40, t=-2 → (150, -2, z_offset=6).
  const auto& pos = tl->position_road_network();
  EXPECT_NEAR(150., pos.x(), 1e-1);
  EXPECT_NEAR(-2., pos.y(), 1e-1);
  EXPECT_NEAR(6., pos.z(), 1e-1);

  // Orientation: Road heading = 0, h_offset = 0, orientation = "-" (against S) → yaw ~ π.
  const auto& rot = tl->orientation_road_network();
  EXPECT_NEAR(0., rot.roll(), 1e-3);
  EXPECT_NEAR(0., rot.pitch(), 1e-3);
  EXPECT_NEAR(M_PI, std::abs(rot.yaw()), 1e-1);

  // BulbGroups: one group.
  const auto bulb_groups = tl->bulb_groups();
  ASSERT_EQ(1u, bulb_groups.size());
  EXPECT_EQ(maliput::api::rules::BulbGroup::Id("TrafficLight_S2_Bulbs"), bulb_groups[0]->id());

  // Bulbs: RedBulb, YellowBulb and GreenBulb (from traffic_signal_db_example.yaml, type "1000001").
  const auto bulbs = bulb_groups[0]->bulbs();
  ASSERT_EQ(3u, bulbs.size());
  EXPECT_NE(bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("RedBulb")), nullptr);
  EXPECT_NE(bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("YellowBulb")), nullptr);
  EXPECT_NE(bulb_groups[0]->GetBulb(maliput::api::rules::Bulb::Id("GreenBulb")), nullptr);

  // Related lanes for S2:
  //   - From signal's own validity (empty → all lanes) on road 2 at s=40 → {"2_0_1", "2_0_-1"}
  //   - From signalReference to S1 on road 1 (no validity) → all lanes of road 1 at s=40 → {"1_0_1", "1_0_-1"}
  // Total (sorted): {"1_0_-1", "1_0_1", "2_0_-1", "2_0_1"}
  const auto& related = tl->related_lanes();
  ASSERT_EQ(4u, related.size());
  std::vector<std::string> lane_ids;
  for (const auto& lid : related) {
    lane_ids.push_back(lid.string());
  }
  std::sort(lane_ids.begin(), lane_ids.end());
  EXPECT_EQ("1_0_-1", lane_ids[0]);
  EXPECT_EQ("1_0_1", lane_ids[1]);
  EXPECT_EQ("2_0_-1", lane_ids[2]);
  EXPECT_EQ("2_0_1", lane_ids[3]);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
