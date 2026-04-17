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
#include "maliput_malidrive/builder/traffic_sign_builder.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/traffic_sign.h>
#include <maliput/api/rules/traffic_sign_book.h>

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

// Test fixture for TrafficSignBuilder.
// Builds a RoadNetwork from the figure8_trafficlights XODR (which contains
// signals of type "1000001") and prepares a TrafficSignalDatabaseLoader backed
// by the example YAML database. Individual tests that require a static traffic
// sign create a copy of the signal and change its type to "206" (stop sign).
class TrafficSignBuilderFigure8Test : public ::testing::Test {
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

TEST_F(TrafficSignBuilderFigure8Test, Constructor) {
  EXPECT_NO_THROW(TrafficSignBuilder(signal_, xodr::RoadHeader::Id("44"), *loader_, road_geometry_));
}

TEST_F(TrafficSignBuilderFigure8Test, ConstructorThrows) {
  EXPECT_THROW(TrafficSignBuilder(signal_, xodr::RoadHeader::Id("44"), *loader_, nullptr), std::invalid_argument);
}

// Verifies that the builder creates a TrafficSign with TrafficSignType::kUnknown
// when the signal's type maps to an unrecognized sign_type in the YAML database.
TEST_F(TrafficSignBuilderFigure8Test, BuildTrafficSignReturnsUnknownForUnrecognizedSignType) {
  // The figure8 signal has type "1000001" which resolves to sign_type "traffic_light".
  // "traffic_light" is not mapped to any TrafficSignType enum, so the builder
  // should create a TrafficSign with TrafficSignType::kUnknown.
  TrafficSignBuilder builder(signal_, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_sign = builder();
  ASSERT_NE(traffic_sign, nullptr);
  EXPECT_EQ(maliput::api::rules::TrafficSignType::kUnknown, traffic_sign->type());
}

// Verifies that a TrafficSign is successfully created from a XODR signal whose
// type maps to sign_type "stop" in the YAML database (type "206").
TEST_F(TrafficSignBuilderFigure8Test, BuildTrafficSign) {
  xodr::signal::Signal stop_signal = signal_;
  stop_signal.type = "206";

  TrafficSignBuilder builder(stop_signal, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_sign = builder();
  ASSERT_NE(traffic_sign, nullptr);

  // The TrafficSign ID should equal the signal ID.
  EXPECT_EQ(traffic_sign->id(), maliput::api::rules::TrafficSign::Id(stop_signal.id.string()));

  // The type must be kStop as defined in the YAML database for type "206".
  EXPECT_EQ(maliput::api::rules::TrafficSignType::kStop, traffic_sign->type());
}

// Verifies that the builder returns nullptr when no matching definition is
// found in the YAML database for the signal's type/subtype fingerprint.
TEST_F(TrafficSignBuilderFigure8Test, BuildTrafficSignNoMatchingDefinition) {
  xodr::signal::Signal unknown_signal = signal_;
  unknown_signal.type = "UnknownType";

  TrafficSignBuilder builder(unknown_signal, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_sign = builder();
  EXPECT_EQ(traffic_sign, nullptr);
}

// Verifies the position of the TrafficSign created from a stop-sign signal
// placed at the same s/t coordinates as the figure8 road-44 signal.
TEST_F(TrafficSignBuilderFigure8Test, PositionOfTrafficSign) {
  xodr::signal::Signal stop_signal = signal_;
  stop_signal.type = "206";

  TrafficSignBuilder builder(stop_signal, xodr::RoadHeader::Id("44"), *loader_, road_geometry_);
  auto traffic_sign = builder();
  ASSERT_NE(traffic_sign, nullptr);

  const auto& position = traffic_sign->position_road_network();
  EXPECT_NEAR(position.x(), 6.0667, 1e-3);
  EXPECT_NEAR(position.y(), -2.8325, 1e-3);
}

// ---------------------------------------------------------------------------
// Tests using TwoRoadsWithTrafficSigns.xodr / traffic_signal_db/traffic_signal_db_example.yaml.
//
// This lightweight map defines two straight roads (Road 1 and Road 2), each
// with a single lane section (lanes {1, 0, -1}).
//
//   Road 1 (100 m, hdg=0, x=[0,100]):
//     - Signal SS1: type="206", s=50, t=2, orientation="+", zOffset=1.0,
//       validity={fromLane=-1, toLane=-1}, signalReference to SS2.
//
//   Road 2 (80 m, hdg=0, x=[110,190]):
//     - Signal SS2: type="206", s=40, t=-2, orientation="-", zOffset=1.0,
//       no explicit validity (all lanes), signalReference to SS1.
//
// The companion YAML database (traffic_signal_db_example.yaml) defines type
// "206" as a static stop sign (sign_type: "stop").
//
// Expected maliput lane IDs:
//   Road 1: "1_0_1" (left), "1_0_-1" (right)
//   Road 2: "2_0_1" (left), "2_0_-1" (right)
// ---------------------------------------------------------------------------

class TrafficSignBuilderTwoRoadsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file =
        utility::FindResourceInPath("TwoRoadsWithTrafficSigns.xodr", kMalidriveResourceFolder);
    const std::string yaml_db =
        utility::FindResourceInPath("traffic_signal_db/traffic_signal_db_example.yaml", kMalidriveResourceFolder);

    road_network_ = RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                                             {params::kOpendriveFile, xodr_file},
                                                                             {params::kTrafficSignalDb, yaml_db},
                                                                             {params::kOmitNonDrivableLanes, "false"},
                                                                         })
                                           .ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    traffic_sign_book_ = road_network_->traffic_sign_book();
    ASSERT_NE(traffic_sign_book_, nullptr);
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const maliput::api::rules::TrafficSignBook* traffic_sign_book_{};
};

// Verifies that both signals produce a TrafficSign and no extra signs exist.
TEST_F(TrafficSignBuilderTwoRoadsTest, TwoTrafficSignsCreated) {
  const auto traffic_signs = traffic_sign_book_->TrafficSigns();
  EXPECT_EQ(2u, traffic_signs.size());
  EXPECT_NE(traffic_sign_book_->GetTrafficSign(maliput::api::rules::TrafficSign::Id("SS1")), nullptr);
  EXPECT_NE(traffic_sign_book_->GetTrafficSign(maliput::api::rules::TrafficSign::Id("SS2")), nullptr);
}

// Verifies all fields of the TrafficSign created from signal SS1.
TEST_F(TrafficSignBuilderTwoRoadsTest, TrafficSignSS1Fields) {
  const auto* ts = traffic_sign_book_->GetTrafficSign(maliput::api::rules::TrafficSign::Id("SS1"));
  ASSERT_NE(ts, nullptr);

  // ID.
  EXPECT_EQ(maliput::api::rules::TrafficSign::Id("SS1"), ts->id());

  // Type: "206" maps to sign_type "stop" → kStop.
  EXPECT_EQ(maliput::api::rules::TrafficSignType::kStop, ts->type());

  // Position: Road 1 is straight at y=0, hdg=0. Signal at s=50, t=2, zOffset=1 → (50, 2, 1).
  const auto& pos = ts->position_road_network();
  EXPECT_NEAR(50., pos.x(), 1e-1);
  EXPECT_NEAR(2., pos.y(), 1e-1);
  EXPECT_NEAR(1., pos.z(), 1e-1);

  // Orientation: Road heading = 0, h_offset = 0, orientation = "+" → yaw ~ 0.
  const auto& rot = ts->orientation_road_network();
  EXPECT_NEAR(0., rot.roll(), 1e-3);
  EXPECT_NEAR(0., rot.pitch(), 1e-3);
  EXPECT_NEAR(0., rot.yaw(), 1e-1);

  // Related lanes for SS1:
  //   - From signal's own validity (fromLane=-1, toLane=-1) on road 1 → {"1_0_-1"}
  //   - From signalReference to SS2 on road 2 (no validity) → all lanes of road 2 → {"2_0_1", "2_0_-1"}
  // Total (sorted): {"1_0_-1", "2_0_-1", "2_0_1"}
  const auto& related = ts->related_lanes();
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

// Verifies all fields of the TrafficSign created from signal SS2.
TEST_F(TrafficSignBuilderTwoRoadsTest, TrafficSignSS2Fields) {
  const auto* ts = traffic_sign_book_->GetTrafficSign(maliput::api::rules::TrafficSign::Id("SS2"));
  ASSERT_NE(ts, nullptr);

  // ID.
  EXPECT_EQ(maliput::api::rules::TrafficSign::Id("SS2"), ts->id());

  // Type: "206" maps to sign_type "stop" → kStop.
  EXPECT_EQ(maliput::api::rules::TrafficSignType::kStop, ts->type());

  // Position: Road 2 starts at x=110, hdg=0. Signal at s=40, t=-2, zOffset=1 → (150, -2, 1).
  const auto& pos = ts->position_road_network();
  EXPECT_NEAR(150., pos.x(), 1e-1);
  EXPECT_NEAR(-2., pos.y(), 1e-1);
  EXPECT_NEAR(1., pos.z(), 1e-1);

  // Orientation: Road heading = 0, h_offset = 0, orientation = "-" (against S) → yaw ~ π.
  const auto& rot = ts->orientation_road_network();
  EXPECT_NEAR(0., rot.roll(), 1e-3);
  EXPECT_NEAR(0., rot.pitch(), 1e-3);
  EXPECT_NEAR(M_PI, std::abs(rot.yaw()), 1e-1);

  // Related lanes for SS2:
  //   - From signal's own validity (empty → all lanes) on road 2 → {"2_0_1", "2_0_-1"}
  //   - From signalReference to SS1 on road 1 (no validity) → all lanes of road 1 → {"1_0_1", "1_0_-1"}
  // Total (sorted): {"1_0_-1", "1_0_1", "2_0_-1", "2_0_1"}
  const auto& related = ts->related_lanes();
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

// Verifies that FindByType correctly retrieves both stop signs.
TEST_F(TrafficSignBuilderTwoRoadsTest, FindByTypeStop) {
  const auto stop_signs = traffic_sign_book_->FindByType(maliput::api::rules::TrafficSignType::kStop);
  EXPECT_EQ(2u, stop_signs.size());
}

// Verifies that the traffic light book remains empty when the XODR contains
// only static signs (no traffic lights).
TEST_F(TrafficSignBuilderTwoRoadsTest, TrafficLightBookIsEmpty) {
  const auto* tlb = road_network_->traffic_light_book();
  ASSERT_NE(tlb, nullptr);
  EXPECT_TRUE(tlb->TrafficLights().empty());
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
