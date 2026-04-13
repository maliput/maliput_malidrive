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
// Uses the StraightRoadWithMultipleRoadObjects.xodr resource.
// ---------------------------------------------------------------------------

class RoadObjectBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xodr_file_path =
        utility::FindResourceInPath("StraightRoadWithMultipleRoadObjects.xodr", kMalidriveResourceFolder);

    road_network_ = RoadNetworkBuilder(RoadNetworkConfiguration::FromMap({
                                                                             {params::kOpendriveFile, xodr_file_path},
                                                                             {params::kOmitNonDrivableLanes, "false"},
                                                                         })
                                           .ToStringMap())();
    ASSERT_NE(road_network_, nullptr);
    road_geometry_ = dynamic_cast<const malidrive::RoadGeometry*>(road_network_->road_geometry());
    ASSERT_NE(road_geometry_, nullptr);

    const auto& road_headers = road_geometry_->get_manager()->GetRoadHeaders();
    const auto it = road_headers.find(xodr::RoadHeader::Id("1"));
    ASSERT_NE(it, road_headers.end());
    ASSERT_TRUE(it->second.objects.has_value());
    objects_ = &it->second.objects->objects;
    ASSERT_EQ(4u, objects_->size());
  }

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const malidrive::RoadGeometry* road_geometry_{};
  const std::vector<xodr::object::Object>* objects_{};
  const xodr::RoadHeader::Id road_id_{"1"};
};

TEST_F(RoadObjectBuilderTest, ConstructorThrowsOnNullptrRoadGeometry) {
  EXPECT_THROW(RoadObjectBuilder(objects_->at(0), road_id_, nullptr), std::invalid_argument);
}

TEST_F(RoadObjectBuilderTest, BarrierObject) {
  // obj_barrier: barrier at s=20, t=3
  RoadObjectBuilder builder(objects_->at(0), road_id_, road_geometry_);
  auto road_object = builder();
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

TEST_F(RoadObjectBuilderTest, BuildingObjectWithRadius) {
  // obj_building: building at s=50, t=-5, radius=4
  RoadObjectBuilder builder(objects_->at(1), road_id_, road_geometry_);
  auto road_object = builder();
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
  // obj_crosswalk: crosswalk at s=80, t=0, with outlines.
  RoadObjectBuilder builder(objects_->at(2), road_id_, road_geometry_);
  auto road_object = builder();
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
  EXPECT_NEAR(corners[0].position().x(), 78.0, 0.5);
  EXPECT_NEAR(corners[0].position().y(), -3.5, 0.5);
  EXPECT_NEAR(corners[1].position().x(), 78.0, 0.5);
  EXPECT_NEAR(corners[1].position().y(), 3.5, 0.5);

  // Validity spans both lanes.
  EXPECT_GE(road_object->related_lanes().size(), 2u);
}

TEST_F(RoadObjectBuilderTest, VegetationObjectWithCornerLocalOutline) {
  // obj_vegetation: vegetation at s=10, t=4, hdg=pi/3, star-shaped cornerLocal outline.
  RoadObjectBuilder builder(objects_->at(3), road_id_, road_geometry_);
  auto road_object = builder();
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

  // Verify outline corner positions in the s-t frame (on this straight road
  // at hdg=0, inertial x≈s and y≈t).
  // cornerLocal (u, v) → s-t via object pose (s=10, t=4, hdg=π/3):
  //   x = 10 + u·cos(π/3) - v·sin(π/3)
  //   y =  4 + u·sin(π/3) + v·cos(π/3)
  // This case is trivial because of the geometry of the road. S and T match to the X and Y inertial coordinates
  // respectively.
  const auto& corners = outline->corners();
  constexpr double tolerance = 1e-3;
  // Corner 0: (u=3.0,   v= 0.0  ) → (11.5,   6.598)
  EXPECT_NEAR(corners[0].position().x(), 11.5, tolerance);
  EXPECT_NEAR(corners[0].position().y(), 6.598, tolerance);
  // Corner 1: (u=0.5,   v= 0.866) → ( 9.5,   4.866)
  EXPECT_NEAR(corners[1].position().x(), 9.5, tolerance);
  EXPECT_NEAR(corners[1].position().y(), 4.866, tolerance);
  // Corner 2: (u=-1.5,  v= 2.598) → ( 7.0,   4.0)
  EXPECT_NEAR(corners[2].position().x(), 7.0, tolerance);
  EXPECT_NEAR(corners[2].position().y(), 4.0, tolerance);
  // Corner 3: (u=-1.0,  v= 0.0  ) → ( 9.5,   3.134)
  EXPECT_NEAR(corners[3].position().x(), 9.5, tolerance);
  EXPECT_NEAR(corners[3].position().y(), 3.134, tolerance);
  // Corner 4: (u=-1.5,  v=-2.598) → (11.5,   1.402)
  EXPECT_NEAR(corners[4].position().x(), 11.5, tolerance);
  EXPECT_NEAR(corners[4].position().y(), 1.402, tolerance);
  // Corner 5: (u=0.5,   v=-0.866) → (11.0,   4.0)
  EXPECT_NEAR(corners[5].position().x(), 11.0, tolerance);
  EXPECT_NEAR(corners[5].position().y(), 4.0, tolerance);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
