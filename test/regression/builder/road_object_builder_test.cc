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
#include "maliput_malidrive/builder/road_object_builder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/objects/road_object.h>
#include <maliput/api/objects/road_object_book.h>
#include <maliput/api/road_network.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/builder/road_object_type_mapper.h"
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
const char kSignalRoadObjectDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
      subtype: "-1"
      country: "OpenDRIVE"
      name: "*"
    properties:
      device_type: RoadObject
      device_semantics: Barrier
      is_position_dynamic: true
      default_bounding_box:
        length: 1.0
        width: 0.5
        height: 1.2
)";

// ---------------------------------------------------------------------------
// Type mapper tests.
// ---------------------------------------------------------------------------

using RoadObjectTypeMapperTest = ::testing::Test;

TEST_F(RoadObjectTypeMapperTest, DirectMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  EXPECT_EQ(MaliputType::kBarrier, MapXodrObjectType(XodrType::kBarrier));
  EXPECT_EQ(MaliputType::kBuilding, MapXodrObjectType(XodrType::kBuilding));
  EXPECT_EQ(MaliputType::kGantry, MapXodrObjectType(XodrType::kGantry));
  EXPECT_EQ(MaliputType::kObstacle, MapXodrObjectType(XodrType::kObstacle));
  EXPECT_EQ(MaliputType::kPole, MapXodrObjectType(XodrType::kPole));
  EXPECT_EQ(MaliputType::kTrafficIsland, MapXodrObjectType(XodrType::kTrafficIsland));
  EXPECT_EQ(MaliputType::kTree, MapXodrObjectType(XodrType::kTree));
  EXPECT_EQ(MaliputType::kVegetation, MapXodrObjectType(XodrType::kVegetation));
}

TEST_F(RoadObjectTypeMapperTest, SpecificTypeMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  // Types with specific maliput mappings instead of generic types
  EXPECT_EQ(MaliputType::kStreetLamp, MapXodrObjectType(XodrType::kStreetLamp));
  EXPECT_EQ(MaliputType::kWind, MapXodrObjectType(XodrType::kWind));
  EXPECT_EQ(MaliputType::kRailing, MapXodrObjectType(XodrType::kRailing));
  EXPECT_EQ(MaliputType::kSoundBarrier, MapXodrObjectType(XodrType::kSoundBarrier));
  EXPECT_EQ(MaliputType::kPatch, MapXodrObjectType(XodrType::kPatch));
}

TEST_F(RoadObjectTypeMapperTest, UnknownMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kCrosswalk));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kParkingSpace));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kRoadMark));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kNone));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(std::nullopt));
}

TEST_F(RoadObjectTypeMapperTest, StaticVehicleTypeMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  // Vehicle types now map to specific static vehicle types
  EXPECT_EQ(MaliputType::kBikeStatic, MapXodrObjectType(XodrType::kBike));
  EXPECT_EQ(MaliputType::kBusStatic, MapXodrObjectType(XodrType::kBus));
  EXPECT_EQ(MaliputType::kCarStatic, MapXodrObjectType(XodrType::kCar));
  EXPECT_EQ(MaliputType::kMotorbikeStatic, MapXodrObjectType(XodrType::kMotorbike));
  EXPECT_EQ(MaliputType::kPedestrianStatic, MapXodrObjectType(XodrType::kPedestrian));
  EXPECT_EQ(MaliputType::kTrailerStatic, MapXodrObjectType(XodrType::kTrailer));
  EXPECT_EQ(MaliputType::kTrainStatic, MapXodrObjectType(XodrType::kTrain));
  EXPECT_EQ(MaliputType::kTramStatic, MapXodrObjectType(XodrType::kTram));
  EXPECT_EQ(MaliputType::kVanStatic, MapXodrObjectType(XodrType::kVan));
}

TEST_F(RoadObjectTypeMapperTest, SubtypeMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  // Barrier subtype mappings
  EXPECT_EQ(MaliputType::kGuardRail, MapXodrObjectType(XodrType::kBarrier, "guardRail"));
  EXPECT_EQ(MaliputType::kGuardWall, MapXodrObjectType(XodrType::kBarrier, "wall"));
  EXPECT_EQ(MaliputType::kGuardWall, MapXodrObjectType(XodrType::kBarrier, "jerseyBarrier"));
  EXPECT_EQ(MaliputType::kSoundBarrier, MapXodrObjectType(XodrType::kBarrier, "noiseProtections"));
  EXPECT_EQ(MaliputType::kRailing, MapXodrObjectType(XodrType::kBarrier, "railing"));
  EXPECT_EQ(MaliputType::kBarrier, MapXodrObjectType(XodrType::kBarrier, "otherSubtype"));
  EXPECT_EQ(MaliputType::kBarrier, MapXodrObjectType(XodrType::kBarrier, std::nullopt));

  // Obstacle subtype mappings
  EXPECT_EQ(MaliputType::kPylon, MapXodrObjectType(XodrType::kObstacle, "roadBlockage"));
  EXPECT_EQ(MaliputType::kObstacle, MapXodrObjectType(XodrType::kObstacle, "otherSubtype"));
  EXPECT_EQ(MaliputType::kObstacle, MapXodrObjectType(XodrType::kObstacle, std::nullopt));

  // Pole subtype mappings
  EXPECT_EQ(MaliputType::kStreetLamp, MapXodrObjectType(XodrType::kPole, "streetLamp"));
  EXPECT_EQ(MaliputType::kWind, MapXodrObjectType(XodrType::kPole, "wind"));
  EXPECT_EQ(MaliputType::kDelineator, MapXodrObjectType(XodrType::kPole, "permanentDelineator"));
  EXPECT_EQ(MaliputType::kPole, MapXodrObjectType(XodrType::kPole, "otherSubtype"));
  EXPECT_EQ(MaliputType::kPole, MapXodrObjectType(XodrType::kPole, std::nullopt));

  // RoadSurface subtype mappings
  EXPECT_EQ(MaliputType::kPatch, MapXodrObjectType(XodrType::kRoadSurface, "patch"));
  EXPECT_EQ(MaliputType::kRoadSurface, MapXodrObjectType(XodrType::kRoadSurface, "otherSubtype"));
  EXPECT_EQ(MaliputType::kRoadSurface, MapXodrObjectType(XodrType::kRoadSurface, std::nullopt));

  // Crosswalk subtype mappings
  EXPECT_EQ(MaliputType::kPedestrianStatic, MapXodrObjectType(XodrType::kCrosswalk, "pedestrian"));
  EXPECT_EQ(MaliputType::kBikeStatic, MapXodrObjectType(XodrType::kCrosswalk, "bicycle"));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kCrosswalk, "otherSubtype"));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kCrosswalk, std::nullopt));
}

// ---------------------------------------------------------------------------
// RoadObjectBuilder tests.
// Uses the TwoRoadsWithRoadObjects.xodr resource.
// ---------------------------------------------------------------------------

class RoadObjectBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("TwoRoadsWithRoadObjects.xodr", kMalidriveResourceFolder);
    const std::string tcd_db_path =
        utility::FindResourceInPath("traffic_control_device_db/road_object_test_db.yaml", kMalidriveResourceFolder);

    road_network_ = RoadNetworkBuilder(
        RoadNetworkConfiguration::FromMap({
                                              {params::kOpendriveFile, xodr_file_path},
                                              {params::kTrafficControlDeviceDb, tcd_db_path},
                                              {params::kOmitNonDrivableLanes, "false"},
                                              {params::kLinearTolerance, std::to_string(kLinearTolerance)},
                                          })
            .ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_object_book_ = road_network_->road_object_book();
    ASSERT_NE(road_object_book_, nullptr);
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const maliput::api::objects::RoadObjectBook* road_object_book_{};
  constexpr static double kLinearTolerance = 1e-5;
};

TEST_F(RoadObjectBuilderTest, RoadObjectBookIsPopulated) {
  const auto all_objects = road_object_book_->RoadObjects();
  EXPECT_EQ(4u, all_objects.size());
}

TEST_F(RoadObjectBuilderTest, GetRoadObjectByIdReturnsNullForUnknownId) {
  EXPECT_EQ(nullptr, road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("nonexistent")));
}

TEST_F(RoadObjectBuilderTest, BarrierObjectMapsToGuardRailDueToSubtype) {
  // obj_barrier: guard rail at s=20, t=3
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_barrier"));
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("obj_barrier"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kGuardRail);
  EXPECT_EQ(road_object->name(), "GuardRail");
  EXPECT_EQ(road_object->subtype(), "guardRail");
  EXPECT_FALSE(road_object->is_dynamic());
  EXPECT_TRUE(road_object->is_movable());

  // Position: straight road at hdg=0, s=20, t=3 → inertial x≈20, y≈3.
  const auto& pos = road_object->position().inertial_position();
  EXPECT_NEAR(pos.x(), 20.0, 0.5);
  EXPECT_NEAR(pos.y(), 3.0, 0.5);
  EXPECT_NEAR(pos.z(), 0.5, 0.1);  // zOffset = 0.5

  // Bounding box: length=10, width=0.3, height=1.
  const auto& bb = road_object->bounding_box();
  EXPECT_NEAR(bb.box_size().x(), 10.0, 1e-3);
  EXPECT_NEAR(bb.box_size().y(), 0.3, 1e-3);
  EXPECT_NEAR(bb.box_size().z(), 1.0, 1e-3);

  // Related lanes: only lane -1 via validity.
  EXPECT_FALSE(road_object->related_lanes().empty());

  // Properties: material = "metal".
  const auto& props = road_object->properties();
  auto it = props.find("material");
  ASSERT_NE(it, props.end());
  EXPECT_EQ(it->second, "metal");
}

TEST_F(RoadObjectBuilderTest, FindByTypeGuardRail) {
  const auto guard_rails = road_object_book_->FindByType(maliput::api::objects::RoadObjectType::kGuardRail);
  ASSERT_EQ(1u, guard_rails.size());
  EXPECT_EQ(guard_rails[0]->id(), maliput::api::objects::RoadObject::Id("obj_barrier"));
}

TEST_F(RoadObjectBuilderTest, BuildingObjectWithRadius) {
  // obj_building: building at s=50, t=-5, radius=4
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_building"));
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("obj_building"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kBuilding);
  EXPECT_EQ(road_object->name(), "Warehouse");
  EXPECT_FALSE(road_object->is_movable());

  // Bounding box from radius: length=width=2*4=8.
  const auto& bb = road_object->bounding_box();
  EXPECT_NEAR(bb.box_size().x(), 8.0, 1e-3);
  EXPECT_NEAR(bb.box_size().y(), 8.0, 1e-3);
  EXPECT_NEAR(bb.box_size().z(), 8.0, 1e-3);

  // No outlines defined.
  EXPECT_EQ(0, road_object->num_outlines());
}

TEST_F(RoadObjectBuilderTest, ObstacleObjectWithOutline) {
  // obj_obstacle: obstacle at s=100, t=0, with outlines.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_obstacle"));
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("obj_obstacle"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kObstacle);
  EXPECT_TRUE(road_object->is_movable());

  // The obstacle has one outline with 4 corners.
  ASSERT_EQ(1, road_object->num_outlines());
  const auto* outline = road_object->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(4, outline->num_corners());
  EXPECT_TRUE(outline->is_closed());

  // Verify outline corner positions are approximately correct.
  // On a straight road at hdg=0: s maps to x and t maps to y.
  const auto& corners = outline->corners();
  EXPECT_NEAR(corners[0].position().x(), 96.0, kLinearTolerance);
  EXPECT_NEAR(corners[0].position().y(), -3.5, kLinearTolerance);
  EXPECT_NEAR(corners[1].position().x(), 96.0, kLinearTolerance);
  EXPECT_NEAR(corners[1].position().y(), 3.5, kLinearTolerance);
  EXPECT_NEAR(corners[2].position().x(), 100.0, kLinearTolerance);
  EXPECT_NEAR(corners[2].position().y(), 3.5, kLinearTolerance);
  EXPECT_NEAR(corners[3].position().x(), 100.0, kLinearTolerance);
  EXPECT_NEAR(corners[3].position().y(), -3.5, kLinearTolerance);

  // Validity spans both lanes.
  EXPECT_GE(road_object->related_lanes().size(), 2u);
}

TEST_F(RoadObjectBuilderTest, VegetationObjectWithCornerLocalOutline) {
  // obj_vegetation: vegetation at s=0, t=4, hdg=pi/3, star-shaped cornerLocal outline.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_vegetation"));
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("obj_vegetation"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kVegetation);
  EXPECT_EQ(road_object->name(), "StarBush");
  EXPECT_FALSE(road_object->is_dynamic());
  EXPECT_FALSE(road_object->is_movable());

  // The vegetation has one outline with 6 corners (star shape).
  ASSERT_EQ(1, road_object->num_outlines());
  const auto* outline = road_object->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(6, outline->num_corners());
  EXPECT_TRUE(outline->is_closed());

  // Verify outline corner positions in the inertial frame.
  // On this straight road at hdg=0, inertial x = s and y = t.
  //
  // cornerLocal (u, v) → inertial (x, y) via object pose (s=0, t=4, hdg=π/3):
  //   x = s + u·cos(hdg) - v·sin(hdg)
  //   y = t + u·sin(hdg) + v·cos(hdg)
  //
  // Expected values were computed with the following Python script:
  //
  //   from math import pi, cos, sin
  //   def uv2xy(x0, y0, u, v, hdg):
  //       x = x0 + cos(hdg) * u - sin(hdg) * v
  //       y = y0 + sin(hdg) * u + cos(hdg) * v
  //       return x, y
  //
  //   v_coords = [0, 0.866, 2.598, 0, -2.598, -0.866]
  //   u_coords = [3, 0.5, -1.5, -1, -1.5, 0.5]
  //   hdg = pi / 3
  //   x0 = 0
  //   y0 = 4.0
  //   for i in range(len(u_coords)):
  //       print(uv2xy(x0, y0, u_coords[i], v_coords[i], hdg))
  //
  // Output:
  //   (1.5000000000000004,  6.598076211353316)
  //   (-0.49997799967732376, 4.8660127018922195)
  //   (-2.9999339990319713,  3.9999618943233424)
  //   (-0.5000000000000001,  3.1339745962155616)
  //   (1.499933999031971,    1.4019618943233418)
  //   (0.9999779996773239,   4.00001270189222)
  const auto& corners = outline->corners();
  // Corner 0: (u=3.0,   v= 0.0  ) → (1.50000,   6.59807)
  EXPECT_NEAR(corners[0].position().x(), 1.50000, kLinearTolerance);
  EXPECT_NEAR(corners[0].position().y(), 6.59807, kLinearTolerance);
  // Corner 1: (u=0.5,   v= 0.866) → (-0.49997,   4.866)
  EXPECT_NEAR(corners[1].position().x(), -0.49997, kLinearTolerance);
  EXPECT_NEAR(corners[1].position().y(), 4.86601, kLinearTolerance);
  // Corner 2: (u=-1.5,  v= 2.598) → (-2.99993,   3.99996)
  EXPECT_NEAR(corners[2].position().x(), -2.99993, kLinearTolerance);
  EXPECT_NEAR(corners[2].position().y(), 3.99996, kLinearTolerance);
  // Corner 3: (u=-1.0,  v= 0.0  ) → (-0.50000,   3.13397)
  EXPECT_NEAR(corners[3].position().x(), -0.50000, kLinearTolerance);
  EXPECT_NEAR(corners[3].position().y(), 3.13397, kLinearTolerance);
  // Corner 4: (u=-1.5,  v=-2.598) → (1.49993,   1.40196)
  EXPECT_NEAR(corners[4].position().x(), 1.49993, kLinearTolerance);
  EXPECT_NEAR(corners[4].position().y(), 1.40196, kLinearTolerance);
  // Corner 5: (u=0.5,   v=-0.866) → (0.99997,   4.00001)
  EXPECT_NEAR(corners[5].position().x(), 0.99997, kLinearTolerance);
  EXPECT_NEAR(corners[5].position().y(), 4.00001, kLinearTolerance);
}

// ---------------------------------------------------------------------------
// Tests for RoadObject methods not covered above.
// ---------------------------------------------------------------------------

TEST_F(RoadObjectBuilderTest, BarrierOrientation) {
  // obj_barrier: hdg=0 on a straight road at hdg=0 → yaw ≈ 0.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_barrier"));
  ASSERT_NE(road_object, nullptr);

  const auto& orientation = road_object->orientation();
  EXPECT_NEAR(orientation.roll(), 0.0, kLinearTolerance);
  EXPECT_NEAR(orientation.pitch(), 0.0, kLinearTolerance);
  EXPECT_NEAR(orientation.yaw(), 0.0, kLinearTolerance);
}

TEST_F(RoadObjectBuilderTest, VegetationOrientation) {
  // obj_vegetation: hdg=pi/3 on a straight road at hdg=0 → yaw ≈ pi/3.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_vegetation"));
  ASSERT_NE(road_object, nullptr);

  const auto& orientation = road_object->orientation();
  EXPECT_NEAR(orientation.yaw(), M_PI / 3.0, kLinearTolerance);
}

TEST_F(RoadObjectBuilderTest, OutlinesVectorAccessor) {
  // obj_obstacle has 1 outline; obj_building has 0.
  const auto* obstacle = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_obstacle"));
  ASSERT_NE(obstacle, nullptr);
  const auto& obstacle_outlines = obstacle->outlines();
  ASSERT_EQ(1u, obstacle_outlines.size());
  EXPECT_NE(obstacle_outlines[0], nullptr);

  const auto* building = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_building"));
  ASSERT_NE(building, nullptr);
  EXPECT_TRUE(building->outlines().empty());
}

TEST_F(RoadObjectBuilderTest, RoadObjectPositionHasLanePosition) {
  // The builder always constructs RoadObjectPosition with lane-relative info.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_barrier"));
  ASSERT_NE(road_object, nullptr);

  const auto& pos = road_object->position();
  EXPECT_TRUE(pos.has_lane_position());
  ASSERT_TRUE(pos.lane_id().has_value());
  ASSERT_TRUE(pos.lane_position().has_value());

  // obj_barrier at s=20, t=3 is closest to lane 1_0_1 (the left lane, width 3.5).
  // The lane_position s should be approximately 20.
  EXPECT_NEAR(pos.lane_position()->s(), 20.0, kLinearTolerance);
}

TEST_F(RoadObjectBuilderTest, OutlineCornerHeight) {
  // obj_obstacle outline corners have height = 0.05.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_obstacle"));
  ASSERT_NE(road_object, nullptr);
  ASSERT_EQ(1, road_object->num_outlines());

  const auto* outline = road_object->outline(0);
  ASSERT_NE(outline, nullptr);
  for (const auto& corner : outline->corners()) {
    ASSERT_TRUE(corner.height().has_value());
    EXPECT_NEAR(corner.height().value(), 0.05, kLinearTolerance);
  }
}

TEST_F(RoadObjectBuilderTest, VegetationOutlineCornerHeight) {
  // obj_vegetation outline corners have height = 2.0.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_vegetation"));
  ASSERT_NE(road_object, nullptr);
  ASSERT_EQ(1, road_object->num_outlines());

  const auto* outline = road_object->outline(0);
  ASSERT_NE(outline, nullptr);
  for (const auto& corner : outline->corners()) {
    ASSERT_TRUE(corner.height().has_value());
    EXPECT_NEAR(corner.height().value(), 2.0, kLinearTolerance);
  }
}

TEST_F(RoadObjectBuilderTest, OutlineId) {
  // obj_obstacle outline has id "obstacle_outline" from the XODR.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_obstacle"));
  ASSERT_NE(road_object, nullptr);
  ASSERT_EQ(1, road_object->num_outlines());

  const auto* outline = road_object->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(outline->id(), maliput::api::objects::Outline::Id("obstacle_outline"));
}

TEST_F(RoadObjectBuilderTest, VegetationOutlineId) {
  // obj_vegetation outline has id "vegetation_star_outline" from the XODR.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_vegetation"));
  ASSERT_NE(road_object, nullptr);
  ASSERT_EQ(1, road_object->num_outlines());

  const auto* outline = road_object->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(outline->id(), maliput::api::objects::Outline::Id("vegetation_star_outline"));
}

class RoadObjectBuilderElevatedRoadTest : public ::testing::Test {
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

    const auto manager = road_geometry_->get_manager();
    ASSERT_NE(manager, nullptr);
    const auto road_headers = manager->GetRoadHeaders();
    const auto road_header_it = road_headers.find(xodr::RoadHeader::Id("1"));
    ASSERT_NE(road_header_it, road_headers.end());
    ASSERT_TRUE(road_header_it->second.objects.has_value());

    bool found_object = false;
    for (const auto& object : road_header_it->second.objects->objects) {
      if (object.id.string() == "OBJ_BARRIER") {
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
};

TEST_F(RoadObjectBuilderElevatedRoadTest, PositionZIsRelativeToRoadSurface) {
  RoadObjectBuilder builder(RoadObjectBuilder::SourceType::kObject, object_, xodr::RoadHeader::Id("1"), *loader_,
                            road_geometry_);
  const auto road_object = builder();
  ASSERT_NE(road_object, nullptr);

  const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{1, object_.s, object_.t};
  const maliput::api::RoadPosition rp =
      road_geometry_->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
  const double road_surface_z = rp.ToInertialPosition().z();
  const double road_object_z = road_object->position().inertial_position().z();

  EXPECT_NEAR(road_surface_z + object_.z_offset, road_object_z, 1e-6);
  EXPECT_GT(std::abs(road_object_z - object_.z_offset), 1.0);
}

TEST_F(RoadObjectBuilderElevatedRoadTest, ObjectOrientationDoesNotAffectYaw) {
  xodr::object::Object object_with_positive_orientation = object_;
  object_with_positive_orientation.orientation = xodr::Orientation::kWithS;
  object_with_positive_orientation.hdg = 0.;

  xodr::object::Object object_with_negative_orientation = object_;
  object_with_negative_orientation.orientation = xodr::Orientation::kAgainstS;
  object_with_negative_orientation.hdg = 0.;

  const auto positive_orientation_road_object =
      RoadObjectBuilder(RoadObjectBuilder::SourceType::kObject, object_with_positive_orientation,
                        xodr::RoadHeader::Id("1"), *loader_, road_geometry_)();
  ASSERT_NE(positive_orientation_road_object, nullptr);

  const auto negative_orientation_road_object =
      RoadObjectBuilder(RoadObjectBuilder::SourceType::kObject, object_with_negative_orientation,
                        xodr::RoadHeader::Id("1"), *loader_, road_geometry_)();
  ASSERT_NE(negative_orientation_road_object, nullptr);

  EXPECT_NEAR(positive_orientation_road_object->orientation().yaw(), 0., 1e-5);
  EXPECT_NEAR(negative_orientation_road_object->orientation().yaw(), 0., 1e-5);
}

class RoadObjectBuilderSignalTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("RoadWithAllDeviceTypes.xodr", kMalidriveResourceFolder);
    const RoadNetworkConfiguration rn_config{RoadNetworkConfiguration::FromMap({
        {params::kOpendriveFile, xodr_file_path},
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
    ASSERT_TRUE(road_header_it->second.signals.has_value());
    signals_ = road_header_it->second.signals->signals;
    loader_ = std::make_unique<traffic_control_device::TrafficControlDeviceDatabaseLoader>(kSignalRoadObjectDb);
  }

  const xodr::signal::Signal& FindSignal(const std::string& id) const {
    for (const auto& signal : signals_) {
      if (signal.id.string() == id) return signal;
    }
    MALIPUT_THROW_MESSAGE("Signal " + id + " not found");
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const malidrive::RoadGeometry* road_geometry_{};
  std::vector<xodr::signal::Signal> signals_;
  std::unique_ptr<traffic_control_device::TrafficControlDeviceDatabaseLoader> loader_;
};

TEST_F(RoadObjectBuilderSignalTest, BuildRoadObjectFromSignal) {
  const auto& signal = FindSignal("TL1");
  RoadObjectBuilder builder(RoadObjectBuilder::SourceType::kSignal, signal, xodr::RoadHeader::Id("1"), *loader_,
                            road_geometry_);
  auto road_object = builder();
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("TL1"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kBarrier);
  EXPECT_TRUE(road_object->is_dynamic());
  EXPECT_TRUE(road_object->is_movable());
  EXPECT_EQ(road_object->num_outlines(), 0);
  EXPECT_EQ(road_object->related_lanes().size(), 1u);
  EXPECT_NEAR(road_object->bounding_box().box_size().x(), 1.0, 1e-6);
  EXPECT_NEAR(road_object->bounding_box().box_size().y(), 0.5, 1e-6);
}

TEST_F(RoadObjectBuilderSignalTest, SignalOrientationAppliesYawOffset) {
  xodr::signal::Signal signal_with_positive_orientation = FindSignal("TL1");
  signal_with_positive_orientation.orientation = xodr::Orientation::kWithS;
  signal_with_positive_orientation.h_offset = 0.;

  xodr::signal::Signal signal_with_negative_orientation = FindSignal("TL1");
  signal_with_negative_orientation.orientation = xodr::Orientation::kAgainstS;
  signal_with_negative_orientation.h_offset = 0.;

  const auto positive_orientation_road_object =
      RoadObjectBuilder(RoadObjectBuilder::SourceType::kSignal, signal_with_positive_orientation,
                        xodr::RoadHeader::Id("1"), *loader_, road_geometry_)();
  ASSERT_NE(positive_orientation_road_object, nullptr);

  const auto negative_orientation_road_object =
      RoadObjectBuilder(RoadObjectBuilder::SourceType::kSignal, signal_with_negative_orientation,
                        xodr::RoadHeader::Id("1"), *loader_, road_geometry_)();
  ASSERT_NE(negative_orientation_road_object, nullptr);

  EXPECT_NEAR(positive_orientation_road_object->orientation().yaw(), M_PI, 1e-5);
  EXPECT_NEAR(negative_orientation_road_object->orientation().yaw(), 0., 1e-5);
}

TEST_F(RoadObjectBuilderTest, FindByLane) {
  // obj_barrier has validity on lane -1 only → related to lane "1_0_-1".
  const auto objects_lane_minus1 = road_object_book_->FindByLane(maliput::api::LaneId("1_0_-1"));
  // At least the barrier should be found.
  bool found_barrier = false;
  for (const auto* obj : objects_lane_minus1) {
    if (obj->id() == maliput::api::objects::RoadObject::Id("obj_barrier")) {
      found_barrier = true;
    }
  }
  EXPECT_TRUE(found_barrier);

  // obj_obstacle spans both lanes (fromLane=-1 toLane=1).
  const auto objects_lane_1 = road_object_book_->FindByLane(maliput::api::LaneId("1_0_1"));
  bool found_obstacle = false;
  for (const auto* obj : objects_lane_1) {
    if (obj->id() == maliput::api::objects::RoadObject::Id("obj_obstacle")) {
      found_obstacle = true;
    }
  }
  EXPECT_TRUE(found_obstacle);

  // No objects on road 2's lanes.
  const auto objects_road2 = road_object_book_->FindByLane(maliput::api::LaneId("2_0_1"));
  EXPECT_TRUE(objects_road2.empty());
}

TEST_F(RoadObjectBuilderTest, FindInRadius) {
  // obj_barrier is at approximately (20, 3, 0.5).
  // Search near it with a small radius.
  const auto nearby = road_object_book_->FindInRadius(maliput::api::InertialPosition(20.0, 3.0, 0.0), 2.0);
  bool found_barrier = false;
  for (const auto* obj : nearby) {
    if (obj->id() == maliput::api::objects::RoadObject::Id("obj_barrier")) {
      found_barrier = true;
    }
  }
  EXPECT_TRUE(found_barrier);

  // Search far from any object should return empty.
  const auto far_away = road_object_book_->FindInRadius(maliput::api::InertialPosition(500.0, 500.0, 0.0), 1.0);
  EXPECT_TRUE(far_away.empty());
}

// ---------------------------------------------------------------------------

class RoadObjectBuilderDirectionFilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file =
        utility::FindResourceInPath("SingleRoadBidirectionalLanes.xodr", kMalidriveResourceFolder);

    road_network_ = RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                                             {params::kOpendriveFile, xodr_file},
                                                                             {params::kOmitNonDrivableLanes, "false"},
                                                                         })
                                           .ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_geometry_ = dynamic_cast<const malidrive::RoadGeometry*>(road_network_->road_geometry());
    ASSERT_NE(road_geometry_, nullptr);

    const auto* manager = road_geometry_->get_manager();
    ASSERT_NE(manager, nullptr);
    const auto road_headers = manager->GetRoadHeaders();
    const auto road_header_it = road_headers.find(road_id_);
    ASSERT_NE(road_header_it, road_headers.end());
    ASSERT_TRUE(road_header_it->second.signals.has_value());
    signals_ = road_header_it->second.signals->signals;
    loader_ = std::make_unique<traffic_control_device::TrafficControlDeviceDatabaseLoader>(kSignalRoadObjectDb);
  }

  const xodr::signal::Signal& FindSignal(const std::string& id) const {
    for (const auto& signal : signals_) {
      if (signal.id.string() == id) return signal;
    }
    MALIPUT_THROW_MESSAGE("Signal " + id + " not found");
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const malidrive::RoadGeometry* road_geometry_{};
  std::vector<xodr::signal::Signal> signals_;
  std::unique_ptr<traffic_control_device::TrafficControlDeviceDatabaseLoader> loader_;
  const xodr::RoadHeader::Id road_id_{"1"};
};

// Verifies that road objects created from signals with orientation="+" (kWithS) filter related_lanes correctly.
TEST_F(RoadObjectBuilderDirectionFilterTest, SignalOrientationWithSFilters) {
  const auto& signal = FindSignal("S_WithS");
  const auto ro =
      RoadObjectBuilder(RoadObjectBuilder::SourceType::kSignal, signal, road_id_, *loader_, road_geometry_)();
  ASSERT_NE(ro, nullptr);
  // Only lane -1 (WithS) should be included
  EXPECT_EQ(ro->related_lanes().size(), 1);
  EXPECT_EQ(ro->related_lanes()[0].string(), "1_0_-1");
}

// Verifies that road objects created from signals with orientation="-" (kAgainstS) filter related_lanes correctly.
TEST_F(RoadObjectBuilderDirectionFilterTest, SignalOrientationAgainstSFilters) {
  const auto& signal = FindSignal("S_AgainstS");
  const auto ro =
      RoadObjectBuilder(RoadObjectBuilder::SourceType::kSignal, signal, road_id_, *loader_, road_geometry_)();
  ASSERT_NE(ro, nullptr);
  // Only lane +1 (AgainstS) should be included
  EXPECT_EQ(ro->related_lanes().size(), 1);
  EXPECT_EQ(ro->related_lanes()[0].string(), "1_0_1");
}

// Verifies that road objects created from signals with orientation="none" (kBidirectional) don't filter related_lanes.
TEST_F(RoadObjectBuilderDirectionFilterTest, SignalOrientationBidirectionalNoFilter) {
  const auto& signal = FindSignal("S_Bidirectional");
  const auto ro =
      RoadObjectBuilder(RoadObjectBuilder::SourceType::kSignal, signal, road_id_, *loader_, road_geometry_)();
  ASSERT_NE(ro, nullptr);
  // Both lanes should be included (no filter)
  EXPECT_EQ(ro->related_lanes().size(), 2);
  const auto lane_ids = ro->related_lanes();
  EXPECT_TRUE(std::any_of(lane_ids.begin(), lane_ids.end(), [](const auto& id) { return id.string() == "1_0_-1"; }));
  EXPECT_TRUE(std::any_of(lane_ids.begin(), lane_ids.end(), [](const auto& id) { return id.string() == "1_0_1"; }));
}

// Expected inertial position of one ContinuousObject sample.
struct ExpectedSample {
  double x{};
  double y{};
};

// Checks that `samples`' point_sample()s match `expected`, in order, within `tolerance`.
// z is always 0 for this map: the arc road has no elevation profile and every repeat
// has zOffsetStart = zOffsetEnd = 0.
void ExpectSamplePoints(const std::vector<maliput::api::objects::ContinuousObject>& samples,
                        const std::vector<ExpectedSample>& expected, double tolerance) {
  ASSERT_EQ(expected.size(), samples.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("sample index " + std::to_string(i));
    EXPECT_NEAR(samples[i].point_sample().x(), expected[i].x, tolerance);
    EXPECT_NEAR(samples[i].point_sample().y(), expected[i].y, tolerance);
    EXPECT_NEAR(samples[i].point_sample().z(), 0., tolerance);
  }
}

// ArcLaneOffsetWithGuardRail.xodr: road "1" is one arc, curvature = 0.025 -> R = 40 m,
// starting at (0, 0) with hdg = 0, curving left about C = (0, 40). No elevation profile
// and no superelevation, so z = 0 everywhere. The XODR `t` coordinate is measured from
// the reference line (the `laneOffset a=2.0` shifts lanes, not `t`), therefore:
//
//   from math import sin, cos
//   R = 40.0
//   def pt(s, t):
//       th = s / R
//       return ((R - t) * sin(th), R - (R - t) * cos(th))
//   def lerp(a, b, r): return a + (b - a) * r
//
// Every sample of an attached (non-detached) repeat must also satisfy |P - C| = R - t.
class ContinuousObjectRepeatSamplingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("ArcLaneOffsetWithGuardRail.xodr", kMalidriveResourceFolder);
    // Explicitly pin the sampling distance to 5.0 m (equivalent to the old default of
    // 10 samples over this fixture's 50 m repeats) so the hardcoded expectations below
    // remain valid regardless of the builder's default continuous_object_sampling_distance.
    road_network_ =
        RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                                 {params::kOpendriveFile, xodr_file_path},
                                                                 {params::kOmitNonDrivableLanes, "false"},
                                                                 {params::kContinuousObjectSamplingDistance, "5.0"},
                                                             })
                               .ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_object_book_ = road_network_->road_object_book();
    ASSERT_NE(road_object_book_, nullptr);
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const maliput::api::objects::RoadObjectBook* road_object_book_{};
  constexpr static double kLinearTolerance = 1e-2;
};

TEST_F(ContinuousObjectRepeatSamplingTest, GuardRailRepeatProducesSamples) {
  const auto* ro = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_right_boundary"));
  ASSERT_NE(ro, nullptr);
  const auto& samples = ro->continuous_properties();
  ASSERT_EQ(11u, samples.size());
  for (const auto& sample : samples) {
    EXPECT_NEAR(sample.width(), 0.3, kLinearTolerance);
    EXPECT_NEAR(sample.height(), 1.0, kLinearTolerance);
  }

  // guardrail_right_boundary: repeat s=[0, 50], t = -0.5 constant.
  const std::vector<ExpectedSample> kExpected{
      {0.00000, -0.50000},   // s =  0.0
      {5.04933, -0.18401},   // s =  5.0
      {10.01986, 0.75905},   // s = 10.0
      {14.83404, 2.31444},   // s = 15.0
      {19.41673, 4.45791},   // s = 20.0
      {23.69644, 7.15599},   // s = 25.0
      {27.60637, 10.36660},  // s = 30.0
      {31.08551, 14.03963},  // s = 35.0
      {34.07957, 18.11776},  // s = 40.0
      {36.54184, 22.53735},  // s = 45.0
      {38.43388, 27.22944},  // s = 50.0
  };
  ExpectSamplePoints(samples, kExpected, kLinearTolerance);

  // On the arc, not a chord: |P - C| = R - t = 40.5 for every sample.
  const maliput::api::InertialPosition kCenter{0., 40., 0.};
  for (const auto& sample : samples) {
    const auto& p = sample.point_sample();
    EXPECT_NEAR(std::hypot(p.x() - kCenter.x(), p.y() - kCenter.y()), 40.5, kLinearTolerance);
  }

  // Uniform arc-length spacing between consecutive samples: 2*40.5*sin(2.5/40) ~= 5.05920.
  for (size_t i = 1; i < samples.size(); ++i) {
    const auto& prev = samples[i - 1].point_sample();
    const auto& curr = samples[i].point_sample();
    EXPECT_NEAR(std::hypot(curr.x() - prev.x(), curr.y() - prev.y()), 5.05920, kLinearTolerance);
  }
}

TEST_F(ContinuousObjectRepeatSamplingTest, LeftGuardRailInterpolatesLateralOffset) {
  const auto* ro = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_left_boundary"));
  ASSERT_NE(ro, nullptr);
  const auto& samples = ro->continuous_properties();
  ASSERT_EQ(11u, samples.size());

  // guardrail_left_boundary: repeat s=[50, 100], t: 4.5 -> 5.0 (interpolated).
  const std::vector<ExpectedSample> kExpected{
      {33.68895, 28.80606},  // s =  50.0, t = 4.50
      {34.77266, 33.10328},  // s =  55.0, t = 4.55
      {35.31132, 37.49590},  // s =  60.0, t = 4.60
      {35.29808, 41.91516},  // s =  65.0, t = 4.65
      {34.73470, 46.29209},  // s =  70.0, t = 4.70
      {33.63152, 50.55856},  // s =  75.0, t = 4.75
      {32.00727, 54.64837},  // s =  80.0, t = 4.80
      {29.88874, 58.49826},  // s =  85.0, t = 4.85
      {27.31037, 62.04889},  // s =  90.0, t = 4.90
      {24.31366, 65.24576},  // s =  95.0, t = 4.95
      {20.94653, 68.04003},  // s = 100.0, t = 5.00
  };
  ExpectSamplePoints(samples, kExpected, kLinearTolerance);

  // |P - C| = R - t = 35.5 - 0.05*i for every sample, proving `t` is interpolated
  // across the repeat rather than pinned to tStart.
  const maliput::api::InertialPosition kCenter{0., 40., 0.};
  for (size_t i = 0; i < samples.size(); ++i) {
    SCOPED_TRACE("sample index " + std::to_string(i));
    const auto& p = samples[i].point_sample();
    const double expected_radius = 35.5 - 0.05 * static_cast<double>(i);
    EXPECT_NEAR(std::hypot(p.x() - kCenter.x(), p.y() - kCenter.y()), expected_radius, kLinearTolerance);
  }

  // heightStart == heightEnd == 1.0, so height must stay constant.
  for (const auto& sample : samples) {
    EXPECT_NEAR(sample.height(), 1.0, kLinearTolerance);
  }
}

TEST_F(ContinuousObjectRepeatSamplingTest, LeftGuardRailWidthInterpolatesAcrossRepeat) {
  constexpr double kWidthTolerance{1e-9};

  const auto* left = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_left_boundary"));
  ASSERT_NE(left, nullptr);
  const auto& left_samples = left->continuous_properties();
  ASSERT_EQ(11u, left_samples.size());

  // guardrail_left_boundary: widthStart = 0.3 -> widthEnd = 1.3, linear over the repeat.
  const std::vector<double> kExpectedWidths{0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3};
  ASSERT_EQ(kExpectedWidths.size(), left_samples.size());
  for (size_t i = 0; i < kExpectedWidths.size(); ++i) {
    SCOPED_TRACE("sample index " + std::to_string(i));
    EXPECT_NEAR(left_samples[i].width(), kExpectedWidths[i], kWidthTolerance);
  }

  // Endpoints match the XODR attributes exactly.
  EXPECT_NEAR(left_samples.front().width(), 0.3, kWidthTolerance);
  EXPECT_NEAR(left_samples.back().width(), 1.3, kWidthTolerance);

  // Strictly increasing, with a uniform 0.1 step: linear interpolation, not a
  // step/nearest-neighbour scheme.
  for (size_t i = 1; i < left_samples.size(); ++i) {
    SCOPED_TRACE("gap " + std::to_string(i - 1) + "->" + std::to_string(i));
    EXPECT_GT(left_samples[i].width(), left_samples[i - 1].width());
    EXPECT_NEAR(left_samples[i].width() - left_samples[i - 1].width(), 0.1, kWidthTolerance);
  }

  // Height is untouched by the width interpolation.
  for (const auto& sample : left_samples) {
    EXPECT_NEAR(sample.height(), 1.0, kWidthTolerance);
  }

  // Contrast: guardrail_right_boundary has widthStart == widthEnd == 0.3, so its
  // width must stay constant. This pins down that the ramp above comes from the
  // repeat's widthStart/widthEnd, not from `ratio` leaking into every object.
  const auto* right =
      road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_right_boundary"));
  ASSERT_NE(right, nullptr);
  for (const auto& sample : right->continuous_properties()) {
    EXPECT_NEAR(sample.width(), 0.3, kWidthTolerance);
  }
}

TEST_F(ContinuousObjectRepeatSamplingTest, SamplingSpansRepeatRangeNotRoadRange) {
  const auto* right =
      road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_right_boundary"));
  ASSERT_NE(right, nullptr);
  const auto& right_samples = right->continuous_properties();
  ASSERT_EQ(11u, right_samples.size());
  EXPECT_NEAR(right_samples.front().point_sample().x(), 0.00000, kLinearTolerance);
  EXPECT_NEAR(right_samples.front().point_sample().y(), -0.50000, kLinearTolerance);
  EXPECT_NEAR(right_samples.back().point_sample().x(), 38.43388, kLinearTolerance);
  EXPECT_NEAR(right_samples.back().point_sample().y(), 27.22944, kLinearTolerance);

  const auto* left = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_left_boundary"));
  ASSERT_NE(left, nullptr);
  const auto& left_samples = left->continuous_properties();
  ASSERT_EQ(11u, left_samples.size());
  EXPECT_NEAR(left_samples.front().point_sample().x(), 33.68895, kLinearTolerance);
  EXPECT_NEAR(left_samples.front().point_sample().y(), 28.80606, kLinearTolerance);
  EXPECT_NEAR(left_samples.back().point_sample().x(), 20.94653, kLinearTolerance);
  EXPECT_NEAR(left_samples.back().point_sample().y(), 68.04003, kLinearTolerance);

  // guardrail_right_boundary's repeat only covers s=[0, 50], half of the 100 m road.
  // If sampling were driven by the road length instead of repeat.length, its last
  // sample would land near the road end (s=100) instead of s=50.
  constexpr double kRoadEndX = 20.69857;
  constexpr double kRoadEndY = 67.87251;
  const auto& right_last = right_samples.back().point_sample();
  EXPECT_GT(std::hypot(right_last.x() - kRoadEndX, right_last.y() - kRoadEndY), 1.0);
}

TEST_F(ContinuousObjectRepeatSamplingTest, SamplingDensityIsConfigurable) {
  const std::string xodr_file_path =
      utility::FindResourceInPath("ArcLaneOffsetWithGuardRail.xodr", kMalidriveResourceFolder);

  const auto coarse_network =
      RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                               {params::kOpendriveFile, xodr_file_path},
                                                               {params::kOmitNonDrivableLanes, "false"},
                                                               {params::kContinuousObjectSamplingDistance, "12.5"},
                                                           })
                             .ToStringMap())();
  ASSERT_NE(coarse_network, nullptr);
  const auto* coarse_ro = coarse_network->road_object_book()->GetRoadObject(
      maliput::api::objects::RoadObject::Id("guardrail_right_boundary"));
  ASSERT_NE(coarse_ro, nullptr);
  const auto& coarse_samples = coarse_ro->continuous_properties();

  // sampling_distance = 12.5 -> 5 samples at s = {0, 12.5, 25, 37.5, 50}.
  const std::vector<ExpectedSample> kExpectedCoarse{
      {0.00000, -0.50000},   // s =  0.0
      {12.45126, 1.46150},   // s = 12.5
      {23.69644, 7.15599},   // s = 25.0
      {32.64628, 16.03189},  // s = 37.5
      {38.43388, 27.22944},  // s = 50.0
  };
  ExpectSamplePoints(coarse_samples, kExpectedCoarse, kLinearTolerance);

  const auto fine_network =
      RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                               {params::kOpendriveFile, xodr_file_path},
                                                               {params::kOmitNonDrivableLanes, "false"},
                                                               {params::kContinuousObjectSamplingDistance, "2.5"},
                                                           })
                             .ToStringMap())();
  ASSERT_NE(fine_network, nullptr);
  const auto* fine_ro = fine_network->road_object_book()->GetRoadObject(
      maliput::api::objects::RoadObject::Id("guardrail_right_boundary"));
  ASSERT_NE(fine_ro, nullptr);
  const auto& fine_samples = fine_ro->continuous_properties();
  ASSERT_EQ(21u, fine_samples.size());

  // Endpoints are always sampled regardless of density.
  EXPECT_NEAR(fine_samples.front().point_sample().x(), 0.00000, kLinearTolerance);
  EXPECT_NEAR(fine_samples.front().point_sample().y(), -0.50000, kLinearTolerance);
  EXPECT_NEAR(fine_samples.back().point_sample().x(), 38.43388, kLinearTolerance);
  EXPECT_NEAR(fine_samples.back().point_sample().y(), 27.22944, kLinearTolerance);
}

class RepeatDetachFromReferenceLineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("ArcLaneOffsetWithGuardRail.xodr", kMalidriveResourceFolder);
    // Explicitly pin the sampling distance to 5.0 m (equivalent to the old default of
    // 10 samples over this fixture's 50 m repeats) so the hardcoded expectations below
    // remain valid regardless of the builder's default continuous_object_sampling_distance.
    road_network_ =
        RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                                 {params::kOpendriveFile, xodr_file_path},
                                                                 {params::kOmitNonDrivableLanes, "false"},
                                                                 {params::kContinuousObjectSamplingDistance, "5.0"},
                                                             })
                               .ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_object_book_ = road_network_->road_object_book();
    ASSERT_NE(road_object_book_, nullptr);
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const maliput::api::objects::RoadObjectBook* road_object_book_{};
};

TEST_F(RepeatDetachFromReferenceLineTest, DetachedRepeatSamplesAChord) {
  constexpr double kLinearTolerance{1e-2};

  const auto* detached =
      road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_detached_boundary"));
  ASSERT_NE(detached, nullptr);
  const auto& detached_samples = detached->continuous_properties();
  ASSERT_EQ(11u, detached_samples.size());

  const auto& detached_start = detached_samples.front().point_sample();
  const auto& detached_mid = detached_samples[detached_samples.size() / 2].point_sample();
  const auto& detached_end = detached_samples.back().point_sample();
  const auto expected_mid_x = (detached_start.x() + detached_end.x()) / 2.;
  const auto expected_mid_y = (detached_start.y() + detached_end.y()) / 2.;
  const auto expected_mid_z = (detached_start.z() + detached_end.z()) / 2.;
  EXPECT_NEAR(detached_mid.x(), expected_mid_x, kLinearTolerance);
  EXPECT_NEAR(detached_mid.y(), expected_mid_y, kLinearTolerance);
  EXPECT_NEAR(detached_mid.z(), expected_mid_z, kLinearTolerance);

  const auto* attached =
      road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("guardrail_right_boundary"));
  ASSERT_NE(attached, nullptr);
  const auto& attached_samples = attached->continuous_properties();
  ASSERT_EQ(11u, attached_samples.size());
  const auto& attached_mid = attached_samples[attached_samples.size() / 2].point_sample();

  const double midpoint_delta = std::hypot(attached_mid.x() - expected_mid_x, attached_mid.y() - expected_mid_y);
  EXPECT_GT(midpoint_delta, 0.1);

  // guardrail_detached_boundary is the same repeat as guardrail_right_boundary
  // (t=-0.5, s=[0,50]) plus detachFromReferenceLine="true", so its samples are a
  // straight Lerp between the arc's start and end points, not points on the arc.
  const std::vector<ExpectedSample> kExpected{
      {0.00000, -0.50000},   // ratio = 0.0
      {3.84339, 2.27294},    // ratio = 0.1
      {7.68678, 5.04589},    // ratio = 0.2
      {11.53016, 7.81883},   // ratio = 0.3
      {15.37355, 10.59178},  // ratio = 0.4
      {19.21694, 13.36472},  // ratio = 0.5
      {23.06033, 16.13767},  // ratio = 0.6
      {26.90371, 18.91061},  // ratio = 0.7
      {30.74710, 21.68356},  // ratio = 0.8
      {34.59049, 24.45650},  // ratio = 0.9
      {38.43388, 27.22944},  // ratio = 1.0
  };
  ExpectSamplePoints(detached_samples, kExpected, kLinearTolerance);

  // Uniform spacing along the chord: 47.392879 / 10 ~= 4.739288 between consecutive
  // samples.
  for (size_t i = 1; i < detached_samples.size(); ++i) {
    const auto& prev = detached_samples[i - 1].point_sample();
    const auto& curr = detached_samples[i].point_sample();
    EXPECT_NEAR(std::hypot(curr.x() - prev.x(), curr.y() - prev.y()), 4.739288, kLinearTolerance);
  }

  // The detached branch must not disturb the interpolated width/height dimensions.
  for (const auto& sample : detached_samples) {
    EXPECT_NEAR(sample.width(), 0.3, kLinearTolerance);
    EXPECT_NEAR(sample.height(), 1.0, kLinearTolerance);
  }
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
