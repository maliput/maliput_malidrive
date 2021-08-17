// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/base/road_geometry.h"

#include <memory>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/loader/loader.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/parser_configuration.h"
#include "utility/resources.h"

using maliput::api::RoadGeometryId;

namespace malidrive {
namespace tests {
namespace {

std::unique_ptr<road_curve::Function> MakeZeroCubicPolynomial(double p0, double p1, double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 0., p0, p1, linear_tolerance);
}

class RoadGeometryTest : public ::testing::Test {
 protected:
  const double kLinearTolerance{constants::kStrictLinearTolerance};    // [m]
  const double kScaleLength{constants::kScaleLength};                  // [m]
  const double kAngularTolerance{constants::kStrictAngularTolerance};  // [rad]
  const maliput::math::Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  const std::optional<double> kParserSTolerance{std::nullopt};  // Disables the check because it is not needed.
  const xodr::ParserConfiguration kParserConfiguration{kParserSTolerance};
  const xodr::RoadHeader::Id kRoadId{"0"};
  const double kP0{0.};
  const double kP1{100.};
  const maliput::math::Vector2 kXy0{10., 10.};
  const maliput::math::Vector2 kDXy{(kP1 - kP0) * std::sqrt(2.) / 2., (kP1 - kP0) * std::sqrt(2.) / 2.};
  const bool kAssertContiguity{true};
  std::unique_ptr<road_curve::RoadCurve> road_curve = std::make_unique<road_curve::RoadCurve>(
      kLinearTolerance, kScaleLength,
      std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
      MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
      kAssertContiguity);
  std::unique_ptr<road_curve::Function> reference_line_offset = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
  std::unique_ptr<xodr::DBManager> manager =
      xodr::LoadDataBaseFromFile(utility::FindResource("odr/SingleLane.xodr"), kParserConfiguration);
};

// Tests getters and the constructor of an empty RoadGeometry.
TEST_F(RoadGeometryTest, EmptyRoadGeometry) {
  const xodr::DBManager* manager_ptr = manager.get();
  const RoadGeometry dut(RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance, kAngularTolerance,
                         kScaleLength, kInertialToBackendFrameTranslation);
  EXPECT_EQ(dut.num_junctions(), 0.);
  EXPECT_EQ(dut.num_branch_points(), 0.);
  EXPECT_DOUBLE_EQ(dut.linear_tolerance(), kLinearTolerance);
  EXPECT_DOUBLE_EQ(dut.angular_tolerance(), kAngularTolerance);
  EXPECT_DOUBLE_EQ(dut.scale_length(), kScaleLength);
  EXPECT_EQ(dut.id(), RoadGeometryId("sample_rg"));
  EXPECT_EQ(dut.get_manager(), manager_ptr);
}

TEST_F(RoadGeometryTest, CorrectRoadCharacteristics) {
  auto rg = std::make_unique<RoadGeometry>(RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                           kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
  const auto expected_road_curve_ptr = road_curve.get();
  const auto expected_reference_line_offset_ptr = reference_line_offset.get();
  rg->AddRoadCharacteristics(kRoadId, std::move(road_curve), std::move(reference_line_offset));
  EXPECT_EQ(expected_road_curve_ptr, rg->GetRoadCurve(kRoadId));
  EXPECT_EQ(expected_reference_line_offset_ptr, rg->GetReferenceLineOffset(kRoadId));
}

TEST_F(RoadGeometryTest, InvalidRoadCurve) {
  auto rg = std::make_unique<RoadGeometry>(RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                           kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
  EXPECT_THROW(rg->AddRoadCharacteristics(kRoadId, nullptr, std::move(reference_line_offset)),
               maliput::common::assertion_error);
}

TEST_F(RoadGeometryTest, InvalidReferenceLineOffset) {
  auto rg = std::make_unique<RoadGeometry>(RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                           kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
  EXPECT_THROW(rg->AddRoadCharacteristics(kRoadId, std::move(road_curve), nullptr), maliput::common::assertion_error);
}

TEST_F(RoadGeometryTest, NonExistentRoadId) {
  auto rg = std::make_unique<RoadGeometry>(RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                           kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
  EXPECT_THROW(rg->GetRoadCurve(kRoadId), maliput::common::assertion_error);
  EXPECT_THROW(rg->GetReferenceLineOffset(kRoadId), maliput::common::assertion_error);
}

TEST_F(RoadGeometryTest, DuplicatedRoadId) {
  auto rg = std::make_unique<RoadGeometry>(RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                           kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
  auto road_curve_b = std::make_unique<road_curve::RoadCurve>(
      kLinearTolerance, kScaleLength,
      std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
      MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
      kAssertContiguity);
  auto reference_line_offset_b = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);

  rg->AddRoadCharacteristics(kRoadId, std::move(road_curve), std::move(reference_line_offset));
  EXPECT_THROW(rg->AddRoadCharacteristics(kRoadId, std::move(road_curve_b), std::move(reference_line_offset_b)),
               maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryFigure8Trafficlights, RoundTripPosition) {
  builder::RoadGeometryConfiguration road_geometry_configuration{};
  road_geometry_configuration.id = maliput::api::RoadGeometryId("figure8_trafficlights");
  road_geometry_configuration.opendrive_file =
      utility::FindResource("odr/figure8_trafficlights/figure8_trafficlights.xodr");

  auto road_network =
      ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration.ToStringMap());

  const maliput::api::LanePosition position(0., 0., 0.);
  const maliput::api::LaneId lane_id("1_0_-1");
  auto lane = road_network->road_geometry()->ById().GetLane(lane_id);
  auto inertial_position = lane->ToInertialPosition(position);

  auto result = road_network->road_geometry()->ToRoadPosition(inertial_position);
  EXPECT_EQ(lane_id, result.road_position.lane->id());
  EXPECT_TRUE(maliput::api::test::IsLanePositionClose(position, result.road_position.pos, constants::kLinearTolerance));
}

GTEST_TEST(RoadGeometryFigure8Trafficlights, RoundTripPositionWithInertialToBackendFrameTranslation) {
  builder::RoadGeometryConfiguration road_geometry_configuration{};
  road_geometry_configuration.id = maliput::api::RoadGeometryId("figure8_trafficlights");
  road_geometry_configuration.opendrive_file =
      utility::FindResource("odr/figure8_trafficlights/figure8_trafficlights.xodr");
  road_geometry_configuration.inertial_to_backend_frame_translation = maliput::math::Vector3{1., 2., 3.};

  auto road_network =
      ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration.ToStringMap());

  const maliput::api::LanePosition position(0., 0., 0.);
  const maliput::api::LaneId lane_id("1_0_-1");
  auto lane = road_network->road_geometry()->ById().GetLane(lane_id);
  auto inertial_position = lane->ToInertialPosition(position);

  auto result = road_network->road_geometry()->ToRoadPosition(inertial_position);
  EXPECT_EQ(lane_id, result.road_position.lane->id());
  EXPECT_TRUE(maliput::api::test::IsLanePositionClose(position, result.road_position.pos, constants::kLinearTolerance));
}

// TODO(francocipollone): Adds tests for ToRoadPosition and FindRoadPosition methods
//                        when MalidriveLoader, MalidriveBuilder and MalidriveLane classes are implemented.

}  // namespace
}  // namespace tests
}  // namespace malidrive
