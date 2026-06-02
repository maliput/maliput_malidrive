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
#include "maliput_malidrive/builder/traffic_control_device_books_builder.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/objects/road_marking.h>
#include <maliput/api/objects/road_marking_book.h>
#include <maliput/api/objects/road_object.h>
#include <maliput/api/objects/road_object_book.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/traffic_lights.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/api/rules/traffic_sign.h>
#include <maliput/api/rules/traffic_sign_book.h>
#include <maliput/common/maliput_throw.h>

#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_geometry_builder.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

// ---------------------------------------------------------------------------
// TrafficControlDeviceBooksBuilder tests.
// Uses the RoadWithAllDeviceTypes.xodr resource and all_device_types_test_db.yaml.
// ---------------------------------------------------------------------------

class TrafficControlDeviceBooksBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    xodr_file_path_ = utility::FindResourceInPath("RoadWithAllDeviceTypes.xodr", kMalidriveResourceFolder);
    tcd_db_path_ = utility::FindResourceInPath("traffic_control_device_db/all_device_types_test_db.yaml",
                                               kMalidriveResourceFolder);

    const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
        {params::kOpendriveFile, xodr_file_path_},
        {params::kTrafficControlDeviceDb, tcd_db_path_},
        {params::kOmitNonDrivableLanes, "false"},
    })};

    road_network_ = RoadNetworkBuilder(rn_config.ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
  }

  std::string xodr_file_path_;
  std::string tcd_db_path_;
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
};

// Verifies that the builder rejects a nullptr RoadGeometry.
TEST_F(TrafficControlDeviceBooksBuilderTest, ConstructorThrowsOnNullptrRoadGeometry) {
  EXPECT_THROW(TrafficControlDeviceBooksBuilder(nullptr, std::nullopt, std::nullopt),
               maliput::common::assertion_error);
}

// Verifies that all four books are non-null.
TEST_F(TrafficControlDeviceBooksBuilderTest, AllBooksAreCreated) {
  ASSERT_NE(road_network_->traffic_light_book(), nullptr);
  ASSERT_NE(road_network_->traffic_sign_book(), nullptr);
  ASSERT_NE(road_network_->road_object_book(), nullptr);
  ASSERT_NE(road_network_->road_marking_book(), nullptr);
}

// Verifies that the TrafficLightBook contains exactly one traffic light with the expected ID.
TEST_F(TrafficControlDeviceBooksBuilderTest, TrafficLightBookPopulated) {
  const auto* tlb = road_network_->traffic_light_book();
  const auto tls = tlb->TrafficLights();
  ASSERT_EQ(1u, tls.size());
  EXPECT_EQ(maliput::api::rules::TrafficLight::Id("TL1"), tls[0]->id());
}

// Verifies that the traffic light has the expected bulb groups and bulbs from the YAML database.
TEST_F(TrafficControlDeviceBooksBuilderTest, TrafficLightBulbStructure) {
  const auto* tl = road_network_->traffic_light_book()->GetTrafficLight(
      maliput::api::rules::TrafficLight::Id("TL1"));
  ASSERT_NE(tl, nullptr);

  const auto& bulb_groups = tl->bulb_groups();
  ASSERT_EQ(1u, bulb_groups.size());

  const auto* bg = bulb_groups[0];
  ASSERT_NE(bg, nullptr);
  // The YAML defines 3 bulbs: RedBulb, YellowBulb, GreenBulb.
  EXPECT_EQ(3u, bg->bulbs().size());
}

// Verifies that the TrafficSignBook contains exactly one stop sign with the expected type.
TEST_F(TrafficControlDeviceBooksBuilderTest, TrafficSignBookPopulated) {
  const auto* tsb = road_network_->traffic_sign_book();
  const auto signs = tsb->TrafficSigns();
  ASSERT_EQ(1u, signs.size());
  EXPECT_EQ(maliput::api::rules::TrafficSign::Id("TS1"), signs[0]->id());
}

// Verifies the traffic sign type is kStop.
TEST_F(TrafficControlDeviceBooksBuilderTest, TrafficSignTypeIsStop) {
  const auto* ts = road_network_->traffic_sign_book()->GetTrafficSign(
      maliput::api::rules::TrafficSign::Id("TS1"));
  ASSERT_NE(ts, nullptr);
  EXPECT_EQ(maliput::api::rules::TrafficSignType::kStop, ts->type());
}

// Verifies that the RoadMarkingBook contains exactly one crosswalk road marking.
TEST_F(TrafficControlDeviceBooksBuilderTest, RoadMarkingBookPopulated) {
  const auto* rmb = road_network_->road_marking_book();
  const auto markings = rmb->RoadMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_EQ(maliput::api::objects::RoadMarking::Id("RM1"), markings[0]->id());
}

// Verifies the road marking type is kCrosswalk.
TEST_F(TrafficControlDeviceBooksBuilderTest, RoadMarkingTypeIsCrosswalk) {
  const auto* rm = road_network_->road_marking_book()->GetRoadMarking(
      maliput::api::objects::RoadMarking::Id("RM1"));
  ASSERT_NE(rm, nullptr);
  EXPECT_EQ(maliput::api::objects::RoadMarkingType::kCrosswalk, rm->type());
}

// Verifies that FindByType works for the road marking book.
TEST_F(TrafficControlDeviceBooksBuilderTest, RoadMarkingFindByType) {
  const auto* rmb = road_network_->road_marking_book();
  const auto crosswalks = rmb->FindByType(maliput::api::objects::RoadMarkingType::kCrosswalk);
  EXPECT_EQ(1u, crosswalks.size());
  const auto stop_lines = rmb->FindByType(maliput::api::objects::RoadMarkingType::kStopLine);
  EXPECT_TRUE(stop_lines.empty());
}

// Verifies that the RoadObjectBook contains exactly one barrier road object.
TEST_F(TrafficControlDeviceBooksBuilderTest, RoadObjectBookPopulated) {
  const auto* rob = road_network_->road_object_book();
  const auto objects = rob->RoadObjects();
  ASSERT_EQ(1u, objects.size());
  EXPECT_EQ(maliput::api::objects::RoadObject::Id("RO1"), objects[0]->id());
}

// Verifies the road object type is kBarrier.
TEST_F(TrafficControlDeviceBooksBuilderTest, RoadObjectTypeIsBarrier) {
  const auto* ro = road_network_->road_object_book()->GetRoadObject(
      maliput::api::objects::RoadObject::Id("RO1"));
  ASSERT_NE(ro, nullptr);
  EXPECT_EQ(maliput::api::objects::RoadObjectType::kBarrier, ro->type());
}

// Verifies that FindByType works for the road object book.
TEST_F(TrafficControlDeviceBooksBuilderTest, RoadObjectFindByType) {
  const auto* rob = road_network_->road_object_book();
  const auto barriers = rob->FindByType(maliput::api::objects::RoadObjectType::kBarrier);
  EXPECT_EQ(1u, barriers.size());
  const auto buildings = rob->FindByType(maliput::api::objects::RoadObjectType::kBuilding);
  EXPECT_TRUE(buildings.empty());
}

// Verifies that the unmatched object of type="tree" is NOT placed in any book.
TEST_F(TrafficControlDeviceBooksBuilderTest, UnmatchedObjectIsSkipped) {
  const auto* rob = road_network_->road_object_book();
  EXPECT_EQ(nullptr, rob->GetRoadObject(maliput::api::objects::RoadObject::Id("UNMATCHED1")));

  const auto* rmb = road_network_->road_marking_book();
  EXPECT_EQ(nullptr, rmb->GetRoadMarking(maliput::api::objects::RoadMarking::Id("UNMATCHED1")));
}

// ---------------------------------------------------------------------------
// Tests behavior when no TCD database is provided.
// Without a database, no devices are created — all books are empty.
// ---------------------------------------------------------------------------

class TrafficControlDeviceBooksBuilderNoDbTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("RoadWithAllDeviceTypes.xodr", kMalidriveResourceFolder);

    // No TCD database specified — all devices are skipped.
    const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
        {params::kOpendriveFile, xodr_file_path},
        {params::kOmitNonDrivableLanes, "false"},
    })};

    road_network_ = RoadNetworkBuilder(rn_config.ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
};

// Without a TCD database, all signal and object books are empty.
TEST_F(TrafficControlDeviceBooksBuilderNoDbTest, AllBooksAreEmpty) {
  EXPECT_TRUE(road_network_->traffic_light_book()->TrafficLights().empty());
  EXPECT_TRUE(road_network_->traffic_sign_book()->TrafficSigns().empty());
  EXPECT_TRUE(road_network_->road_object_book()->RoadObjects().empty());
  EXPECT_TRUE(road_network_->road_marking_book()->RoadMarkings().empty());
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
