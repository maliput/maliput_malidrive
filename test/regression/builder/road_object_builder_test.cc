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

#include <cmath>
#include <memory>
#include <string>

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
// Type mapper tests.
// ---------------------------------------------------------------------------

using RoadObjectTypeMapperTest = ::testing::Test;

TEST_F(RoadObjectTypeMapperTest, DirectMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  EXPECT_EQ(MaliputType::kBarrier, MapXodrObjectType(XodrType::kBarrier));
  EXPECT_EQ(MaliputType::kBuilding, MapXodrObjectType(XodrType::kBuilding));
  EXPECT_EQ(MaliputType::kCrosswalk, MapXodrObjectType(XodrType::kCrosswalk));
  EXPECT_EQ(MaliputType::kGantry, MapXodrObjectType(XodrType::kGantry));
  EXPECT_EQ(MaliputType::kObstacle, MapXodrObjectType(XodrType::kObstacle));
  EXPECT_EQ(MaliputType::kParkingSpace, MapXodrObjectType(XodrType::kParkingSpace));
  EXPECT_EQ(MaliputType::kPole, MapXodrObjectType(XodrType::kPole));
  EXPECT_EQ(MaliputType::kRoadMark, MapXodrObjectType(XodrType::kRoadMark));
  EXPECT_EQ(MaliputType::kRoadSurface, MapXodrObjectType(XodrType::kRoadSurface));
  EXPECT_EQ(MaliputType::kTrafficIsland, MapXodrObjectType(XodrType::kTrafficIsland));
  EXPECT_EQ(MaliputType::kTree, MapXodrObjectType(XodrType::kTree));
  EXPECT_EQ(MaliputType::kVegetation, MapXodrObjectType(XodrType::kVegetation));
}

TEST_F(RoadObjectTypeMapperTest, CloseMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  EXPECT_EQ(MaliputType::kPole, MapXodrObjectType(XodrType::kStreetLamp));
  EXPECT_EQ(MaliputType::kPole, MapXodrObjectType(XodrType::kWind));
  EXPECT_EQ(MaliputType::kBarrier, MapXodrObjectType(XodrType::kRailing));
  EXPECT_EQ(MaliputType::kBarrier, MapXodrObjectType(XodrType::kSoundBarrier));
  EXPECT_EQ(MaliputType::kRoadSurface, MapXodrObjectType(XodrType::kPatch));
}

TEST_F(RoadObjectTypeMapperTest, UnknownMappings) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kBike));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kCar));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kVan));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kTrain));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(XodrType::kNone));
  EXPECT_EQ(MaliputType::kUnknown, MapXodrObjectType(std::nullopt));
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

    road_network_ = RoadNetworkBuilder(
        RoadNetworkConfiguration::FromMap({
                                              {params::kOpendriveFile, xodr_file_path},
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

TEST_F(RoadObjectBuilderTest, BarrierObject) {
  // obj_barrier: barrier at s=20, t=3
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_barrier"));
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("obj_barrier"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kBarrier);
  EXPECT_EQ(road_object->name(), "GuardRail");
  EXPECT_EQ(road_object->subtype(), "guardRail");
  EXPECT_FALSE(road_object->is_dynamic());

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

TEST_F(RoadObjectBuilderTest, FindByTypeBarrier) {
  const auto barriers = road_object_book_->FindByType(maliput::api::objects::RoadObjectType::kBarrier);
  ASSERT_EQ(1u, barriers.size());
  EXPECT_EQ(barriers[0]->id(), maliput::api::objects::RoadObject::Id("obj_barrier"));
}

TEST_F(RoadObjectBuilderTest, BuildingObjectWithRadius) {
  // obj_building: building at s=50, t=-5, radius=4
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_building"));
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("obj_building"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kBuilding);
  EXPECT_EQ(road_object->name(), "Warehouse");

  // Bounding box from radius: length=width=2*4=8.
  const auto& bb = road_object->bounding_box();
  EXPECT_NEAR(bb.box_size().x(), 8.0, 1e-3);
  EXPECT_NEAR(bb.box_size().y(), 8.0, 1e-3);
  EXPECT_NEAR(bb.box_size().z(), 8.0, 1e-3);

  // No outlines defined.
  EXPECT_EQ(0, road_object->num_outlines());
}

TEST_F(RoadObjectBuilderTest, CrosswalkObjectWithOutline) {
  // obj_crosswalk: crosswalk at s=100, t=0, with outlines.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_crosswalk"));
  ASSERT_NE(road_object, nullptr);

  EXPECT_EQ(road_object->id(), maliput::api::objects::RoadObject::Id("obj_crosswalk"));
  EXPECT_EQ(road_object->type(), maliput::api::objects::RoadObjectType::kCrosswalk);

  // The crosswalk has one outline with 4 corners.
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
  // obj_crosswalk has 1 outline; obj_building has 0.
  const auto* crosswalk = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_crosswalk"));
  ASSERT_NE(crosswalk, nullptr);
  const auto& crosswalk_outlines = crosswalk->outlines();
  ASSERT_EQ(1u, crosswalk_outlines.size());
  EXPECT_NE(crosswalk_outlines[0], nullptr);

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
  // obj_crosswalk outline corners have height = 0.05.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_crosswalk"));
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
  // obj_crosswalk outline has id "crosswalk_outline" from the XODR.
  const auto* road_object = road_object_book_->GetRoadObject(maliput::api::objects::RoadObject::Id("obj_crosswalk"));
  ASSERT_NE(road_object, nullptr);
  ASSERT_EQ(1, road_object->num_outlines());

  const auto* outline = road_object->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(outline->id(), maliput::api::objects::Outline::Id("crosswalk_outline"));
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

  // obj_crosswalk spans both lanes (fromLane=-1 toLane=1).
  const auto objects_lane_1 = road_object_book_->FindByLane(maliput::api::LaneId("1_0_1"));
  bool found_crosswalk = false;
  for (const auto* obj : objects_lane_1) {
    if (obj->id() == maliput::api::objects::RoadObject::Id("obj_crosswalk")) {
      found_crosswalk = true;
    }
  }
  EXPECT_TRUE(found_crosswalk);

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

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
