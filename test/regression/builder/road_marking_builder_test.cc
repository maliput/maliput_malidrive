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
#include "maliput_malidrive/builder/road_marking_builder.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/objects/road_marking.h>
#include <maliput/api/road_network.h>
#include <maliput/common/maliput_throw.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/traffic_control_device/traffic_control_device_database_loader.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/object/object.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

// ---------------------------------------------------------------------------
// RoadMarkingBuilder tests.
// Uses the RoadWithRoadMarkings.xodr resource and road_marking_test_db.yaml.
// ---------------------------------------------------------------------------

class RoadMarkingBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("RoadWithRoadMarkings.xodr", kMalidriveResourceFolder);
    tcd_db_path_ =
        utility::FindResourceInPath("traffic_control_device_db/road_marking_test_db.yaml", kMalidriveResourceFolder);

    const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
        {params::kOpendriveFile, xodr_file_path},
        {params::kTrafficControlDeviceDb, tcd_db_path_},
        {params::kOmitNonDrivableLanes, "false"},
    })};
    road_network_ = RoadNetworkBuilder(rn_config.ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_geometry_ = dynamic_cast<const malidrive::RoadGeometry*>(road_network_->road_geometry());
    ASSERT_NE(road_geometry_, nullptr);

    const auto* manager = road_geometry_->get_manager();
    ASSERT_NE(manager, nullptr);
    const auto road_headers = manager->GetRoadHeaders();
    const auto road_header_it = road_headers.find(xodr::RoadHeader::Id("1"));
    ASSERT_NE(road_header_it, road_headers.end());
    ASSERT_TRUE(road_header_it->second.objects.has_value());
    objects_ = road_header_it->second.objects->objects;
    ASSERT_EQ(4u, objects_.size());

    loader_ = std::make_unique<traffic_control_device::TrafficControlDeviceDatabaseLoader>(tcd_db_path_);
  }

  const xodr::object::Object& FindObject(const std::string& id) const {
    for (const auto& obj : objects_) {
      if (obj.id.string() == id) return obj;
    }
    MALIPUT_THROW_MESSAGE("Object " + id + " not found");
  }

  std::string tcd_db_path_;
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const malidrive::RoadGeometry* road_geometry_{};
  std::vector<xodr::object::Object> objects_;
  std::unique_ptr<traffic_control_device::TrafficControlDeviceDatabaseLoader> loader_;
  const xodr::RoadHeader::Id road_id_{"1"};
};

TEST_F(RoadMarkingBuilderTest, Constructor) {
  const auto& object = FindObject("rm_stop_line");
  EXPECT_NO_THROW(RoadMarkingBuilder(object, road_id_, *loader_, road_geometry_));
}

TEST_F(RoadMarkingBuilderTest, ConstructorThrowsOnNullptrRoadGeometry) {
  const auto& object = FindObject("rm_stop_line");
  EXPECT_THROW(RoadMarkingBuilder(object, road_id_, *loader_, nullptr), std::invalid_argument);
}

TEST_F(RoadMarkingBuilderTest, BuildStopLine) {
  const auto& object = FindObject("rm_stop_line");
  RoadMarkingBuilder builder(object, road_id_, *loader_, road_geometry_);
  auto road_marking = builder();
  ASSERT_NE(road_marking, nullptr);

  EXPECT_EQ(road_marking->id(), maliput::api::objects::RoadMarking::Id("rm_stop_line"));
  EXPECT_EQ(road_marking->type(), maliput::api::objects::RoadMarkingType::kStopLine);
  EXPECT_EQ(road_marking->name(), "StopLine");

  // Position: straight road at hdg=0, s=30, t=0 → inertial x≈30, y≈0.
  const auto& pos = road_marking->position().inertial_position();
  EXPECT_NEAR(pos.x(), 30.0, 0.5);
  EXPECT_NEAR(pos.y(), 0.0, 0.5);

  // Bounding box: length=0.3, width=7.0, height=0.0.
  const auto& bb = road_marking->bounding_box();
  EXPECT_NEAR(bb.box_size().x(), 0.3, 1e-3);
  EXPECT_NEAR(bb.box_size().y(), 7.0, 1e-3);
  EXPECT_NEAR(bb.box_size().z(), 0.0, 1e-3);

  // Related lanes: both lanes.
  EXPECT_GE(road_marking->related_lanes().size(), 2u);
}

TEST_F(RoadMarkingBuilderTest, BuildCrosswalk) {
  const auto& object = FindObject("rm_crosswalk");
  RoadMarkingBuilder builder(object, road_id_, *loader_, road_geometry_);
  auto road_marking = builder();
  ASSERT_NE(road_marking, nullptr);

  EXPECT_EQ(road_marking->id(), maliput::api::objects::RoadMarking::Id("rm_crosswalk"));
  EXPECT_EQ(road_marking->type(), maliput::api::objects::RoadMarkingType::kCrosswalk);

  const auto& pos = road_marking->position().inertial_position();
  EXPECT_NEAR(pos.x(), 50.0, 0.5);
  EXPECT_NEAR(pos.y(), 0.0, 0.5);
}

TEST_F(RoadMarkingBuilderTest, BuildArrowForward) {
  const auto& object = FindObject("rm_arrow");
  RoadMarkingBuilder builder(object, road_id_, *loader_, road_geometry_);
  auto road_marking = builder();
  ASSERT_NE(road_marking, nullptr);

  EXPECT_EQ(road_marking->id(), maliput::api::objects::RoadMarking::Id("rm_arrow"));
  EXPECT_EQ(road_marking->type(), maliput::api::objects::RoadMarkingType::kArrowForward);

  const auto& pos = road_marking->position().inertial_position();
  EXPECT_NEAR(pos.x(), 70.0, 0.5);
  EXPECT_NEAR(pos.y(), -1.75, 0.5);

  // Only lane -1 via validity.
  EXPECT_EQ(1u, road_marking->related_lanes().size());
}

TEST_F(RoadMarkingBuilderTest, ReturnsNullptrForNonRoadMarkingDeviceType) {
  // The pole object maps to device_type: road_object, so should return nullptr.
  const auto& object = FindObject("rm_pole");
  RoadMarkingBuilder builder(object, road_id_, *loader_, road_geometry_);
  auto road_marking = builder();
  EXPECT_EQ(road_marking, nullptr);
}

TEST_F(RoadMarkingBuilderTest, ReturnsNullptrForNoMatchingDefinition) {
  // Create a fake object with a type that's not in the database.
  xodr::object::Object fake_object;
  fake_object.id = xodr::object::Object::Id("fake_obj");
  fake_object.s = 10.0;
  fake_object.t = 0.0;
  fake_object.z_offset = 0.0;
  fake_object.type = xodr::object::Object::str_to_object_type("building");

  RoadMarkingBuilder builder(fake_object, road_id_, *loader_, road_geometry_);
  auto road_marking = builder();
  EXPECT_EQ(road_marking, nullptr);
}

class RoadMarkingBuilderElevatedRoadTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("SingleRoadElevatedAllDeviceTypes.xodr", kMalidriveResourceFolder);
    tcd_db_path_ = utility::FindResourceInPath("traffic_control_device_db/all_device_types_test_db.yaml",
                                               kMalidriveResourceFolder);

    const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
        {params::kOpendriveFile, xodr_file_path},
        {params::kTrafficControlDeviceDb, tcd_db_path_},
        {params::kOmitNonDrivableLanes, "false"},
    })};
    road_network_ = RoadNetworkBuilder(rn_config.ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_geometry_ = dynamic_cast<const malidrive::RoadGeometry*>(road_network_->road_geometry());
    ASSERT_NE(road_geometry_, nullptr);

    const auto* manager = road_geometry_->get_manager();
    ASSERT_NE(manager, nullptr);
    const auto road_headers = manager->GetRoadHeaders();
    const auto road_header_it = road_headers.find(xodr::RoadHeader::Id("1"));
    ASSERT_NE(road_header_it, road_headers.end());
    ASSERT_TRUE(road_header_it->second.objects.has_value());

    bool found_object = false;
    for (const auto& object : road_header_it->second.objects->objects) {
      if (object.id.string() == "OBJ_CROSSWALK") {
        object_ = object;
        found_object = true;
        break;
      }
    }
    ASSERT_TRUE(found_object);

    loader_ = std::make_unique<traffic_control_device::TrafficControlDeviceDatabaseLoader>(tcd_db_path_);
  }

  std::string tcd_db_path_;
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const malidrive::RoadGeometry* road_geometry_{};
  xodr::object::Object object_;
  std::unique_ptr<traffic_control_device::TrafficControlDeviceDatabaseLoader> loader_;
  const xodr::RoadHeader::Id road_id_{"1"};
};

TEST_F(RoadMarkingBuilderElevatedRoadTest, PositionZIsRelativeToRoadSurface) {
  RoadMarkingBuilder builder(object_, road_id_, *loader_, road_geometry_);
  const auto road_marking = builder();
  ASSERT_NE(road_marking, nullptr);

  const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{1, object_.s, object_.t};
  const maliput::api::RoadPosition rp =
      road_geometry_->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
  const double road_surface_z = rp.ToInertialPosition().z();
  const double road_marking_z = road_marking->position().inertial_position().z();

  EXPECT_NEAR(road_surface_z + object_.z_offset, road_marking_z, 1e-6);
  EXPECT_GT(std::abs(road_marking_z - object_.z_offset), 1.0);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
