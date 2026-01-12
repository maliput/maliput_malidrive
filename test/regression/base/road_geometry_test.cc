// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/base/road_geometry.h"

#include <memory>

#include <gtest/gtest.h>
#include <maliput/api/compare.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/error.h>
#include <maliput/geometry_base/kd_tree_strategy.h>
#include <maliput/math/compare.h>

#include "assert_compare.h"
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

using malidrive::test::AssertCompare;
using maliput::api::IsLanePositionClose;
using maliput::api::RoadGeometryId;

namespace malidrive {
namespace tests {
namespace {

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

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
  std::unique_ptr<xodr::DBManager> manager = xodr::LoadDataBaseFromFile(
      utility::FindResourceInPath("SingleLane.xodr", kMalidriveResourceFolder), kParserConfiguration);
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

class RoadGeometryFigure8Trafficlights : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("figure8_trafficlights");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("figure8_trafficlights/figure8_trafficlights.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryFigure8Trafficlights, RoundTripPositionAtTheStart) {
  const maliput::api::LanePosition position(0., 0., 0.);
  const maliput::api::LaneId lane_id("1_0_-1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  auto inertial_position = lane->ToInertialPosition(position);

  auto result = road_network_->road_geometry()->ToRoadPosition(inertial_position);
  EXPECT_EQ(lane_id, result.road_position.lane->id());
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(position, result.road_position.pos, constants::kLinearTolerance)));
}

TEST_F(RoadGeometryFigure8Trafficlights, RoundTripPositionWithInertialToBackendFrameTranslation) {
  road_geometry_configuration_.inertial_to_backend_frame_translation = maliput::math::Vector3{1., 2., 3.};
  road_network_ =
      ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());

  const maliput::api::LanePosition position(0., 0., 0.);
  const maliput::api::LaneId lane_id("1_0_-1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  auto inertial_position = lane->ToInertialPosition(position);

  auto result = road_network_->road_geometry()->ToRoadPosition(inertial_position);
  EXPECT_EQ(lane_id, result.road_position.lane->id());
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(position, result.road_position.pos, constants::kLinearTolerance)));
}

TEST_F(RoadGeometryFigure8Trafficlights, RoundTripPositionInBetween) {
  const maliput::api::LanePosition position(80., 0., 0.);
  const maliput::api::LaneId lane_id("1_0_-1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  auto inertial_position = lane->ToInertialPosition(position);

  auto result = road_network_->road_geometry()->ToRoadPosition(inertial_position);
  EXPECT_EQ(lane_id, result.road_position.lane->id());
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(position, result.road_position.pos, constants::kLinearTolerance)));
}

TEST_F(RoadGeometryFigure8Trafficlights, RoundTripPositionAtTheEnd) {
  const maliput::api::LaneId lane_id("1_0_-1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  const maliput::api::LanePosition position(lane->length(), 0., 0.);
  auto inertial_position = lane->ToInertialPosition(position);

  auto result = road_network_->road_geometry()->ToRoadPosition(inertial_position);
  EXPECT_EQ(lane_id, result.road_position.lane->id());
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(position, result.road_position.pos, constants::kLinearTolerance)));
}

class RoadGeometryFindRoadPositions : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("TShapeRoad");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("TShapeRoad.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryFindRoadPositions, FindSingleRoadPosition) {
  const maliput::api::LaneId lane_id("0_0_1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  const maliput::api::LanePosition position(lane->length() / 2, 0., 0.);
  auto inertial_position = lane->ToInertialPosition(position);
  double radius = 0.1;
  maliput::geometry_base::KDTreeStrategy strategy(road_network_->road_geometry(), .05);
  std::vector<maliput::api::RoadPositionResult> results = strategy.FindRoadPositions(inertial_position, radius);
  EXPECT_EQ(static_cast<int>(results.size()), 1);
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(position, results[0].road_position.pos, constants::kLinearTolerance)));
  EXPECT_EQ(results[0].road_position.lane->id(), lane_id);
}

TEST_F(RoadGeometryFindRoadPositions, FindPositionsBetween3BranchingLanes) {
  const maliput::api::LaneId lane_id("0_0_1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  const maliput::api::LanePosition position(lane->length(), 0., 0.);
  auto inertial_position = lane->ToInertialPosition(position);
  double radius = 0.1;
  maliput::geometry_base::KDTreeStrategy strategy(road_network_->road_geometry(), .05);
  std::vector<maliput::api::RoadPositionResult> results = strategy.FindRoadPositions(inertial_position, radius);
  EXPECT_EQ(static_cast<int>(results.size()), 3);
}

TEST_F(RoadGeometryFindRoadPositions, FindPositionsBetween2ParallelLanes) {
  const maliput::api::LaneId lane_id("0_0_1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  const maliput::api::LanePosition position(lane->length() / 2, lane->lane_bounds(lane->length() / 2).min(), 0.);
  auto inertial_position = lane->ToInertialPosition(position);
  double radius = 0.1;
  maliput::geometry_base::KDTreeStrategy strategy(road_network_->road_geometry(), .05);
  std::vector<maliput::api::RoadPositionResult> results = strategy.FindRoadPositions(inertial_position, radius);
  EXPECT_EQ(static_cast<int>(results.size()), 2);
  const maliput::api::LanePosition parallel_lane_position(lane->length() / 2,
                                                          lane->to_right()->lane_bounds(lane->length() / 2).max(), 0.);
  std::vector<std::pair<maliput::api::LaneId, maliput::api::LanePosition>> lanes;
  const maliput::api::LaneId parallel_lane_id("0_0_-1");
  lanes.push_back({parallel_lane_id, parallel_lane_position});
  lanes.push_back({lane_id, position});
  EXPECT_EQ(results.size(), lanes.size());
  for (const auto& expected_lane : lanes) {
    const auto it = std::find_if(results.begin(), results.end(), [&](const auto& result) {
      return result.road_position.lane->id() == expected_lane.first;
    });
    EXPECT_NE(it, results.end());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane.second, it->road_position.pos, constants::kLinearTolerance)));
  }
}

TEST_F(RoadGeometryFindRoadPositions, FindAllRoadPosition) {
  const maliput::api::LaneId lane_id("0_0_1");
  auto lane = road_network_->road_geometry()->ById().GetLane(lane_id);
  const maliput::api::LanePosition position(lane->length() / 2, 0., 0.);
  auto inertial_position = lane->ToInertialPosition(position);
  double radius = 1000.0;
  maliput::geometry_base::KDTreeStrategy strategy(road_network_->road_geometry(), .05);
  std::vector<maliput::api::RoadPositionResult> results = strategy.FindRoadPositions(inertial_position, radius);
  EXPECT_EQ(static_cast<int>(results.size()), road_network_->road_geometry()->ById().GetLanes().size());
}

struct OpenScenarioLanePositionMaliputLane {
  std::string xodr_name;
  RoadGeometry::OpenScenarioLanePosition xodr_lane_position;
  maliput::api::LaneId expected_lane_id{"None"};

  // Implement operator<< for printing in tests.
  friend std::ostream& operator<<(std::ostream& os, const OpenScenarioLanePositionMaliputLane& param) {
    os << "OpenScenarioLanePositionMaliputLane{"
       << "xodr_name: " << param.xodr_name << ", xodr_lane_position: (" << param.xodr_lane_position.road_id << ", "
       << param.xodr_lane_position.s << ", " << param.xodr_lane_position.lane_id << ", "
       << param.xodr_lane_position.offset << ")"
       << ", expected_lane_id: " << param.expected_lane_id.string() << "}";
    return os;
  }
};

class RoadGeometryGetMaliputLaneFromOpenScenarioLanePosition
    : public ::testing::TestWithParam<OpenScenarioLanePositionMaliputLane> {
 protected:
  void SetUp() override {
    param_ = GetParam();
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath(param_.xodr_name, kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  OpenScenarioLanePositionMaliputLane param_{GetParam()};
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_P(RoadGeometryGetMaliputLaneFromOpenScenarioLanePosition, GetMaliputLaneFromOpenScenarioLanePosition) {
  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::Lane* mali_lane = rg->GetMaliputLaneFromOpenScenarioLanePosition(param_.xodr_lane_position);
  EXPECT_NE(mali_lane, nullptr);
  EXPECT_EQ(mali_lane->id(), param_.expected_lane_id);
}

INSTANTIATE_TEST_CASE_P(RoadGeometryGetMaliputLaneFromOpenScenarioLanePositionTests,
                        RoadGeometryGetMaliputLaneFromOpenScenarioLanePosition,
                        ::testing::ValuesIn(std::vector<OpenScenarioLanePositionMaliputLane>{
                            {"LineMultipleSections.xodr", {1, 0., -1, 0.}, maliput::api::LaneId{"1_0_-1"}},
                            {"LineMultipleSections.xodr", {1, 33.3, -1, 0.}, maliput::api::LaneId{"1_1_-1"}},
                            {"LineMultipleSections.xodr", {1, 50., -1, 0.}, maliput::api::LaneId{"1_1_-1"}},
                            {"LineMultipleSections.xodr", {1, 66.6, -1, 0.}, maliput::api::LaneId{"1_2_-1"}},
                            {"LineMultipleSections.xodr", {1, 100., -1, 0.}, maliput::api::LaneId{"1_2_-1"}},
                        }));

class RoadGeometryOpenScenarioConversionsArcLane : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("ArcLane");
    road_geometry_configuration_.opendrive_file = utility::FindResourceInPath("ArcLane.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryOpenScenarioConversionsArcLane, RoundTripOpenScenarioLanePositionToMaliputRoadPositionAtCenterline) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 50., -1, 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const maliput::api::LanePosition expected_lane_position(51.25, 0., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLane, RoundTripOpenScenarioLanePositionToMaliputRoadPositionWithOffset) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 50., -1, 0.5};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const maliput::api::LanePosition expected_lane_position(51.25, 0.5, 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLane,
       RoundTripOpenScenarioLanePositionToMaliputRoadPositionWithOffsetOffLane) {
  // OpenScenario/OpenDrive parameters.
  const double offset = 2.;  // 1 meter off the lane.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 50., -1, offset};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const maliput::api::LanePosition expected_lane_position(51.25, 2., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLane, RoundTripOpenScenarioRoadPositionToMaliputRoadPositionAtCenterline) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const maliput::api::LanePosition expected_lane_position(51.25, 1., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
      rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
  EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
  EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLane, RoundTripOpenScenarioRoadPositionToMaliputRoadPositionWithOffset) {
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., -1.};
    // Maliput expected results.
    const maliput::api::LaneId lane_id("1_0_-1");
    const maliput::api::LanePosition expected_lane_position(51.25, 0., 0.);

    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
    EXPECT_EQ(lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
    const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
        rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
    EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
    EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
    EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
  }
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 1.};
    // Maliput expected results.
    const maliput::api::LaneId lane_id("1_0_1");
    const maliput::api::LanePosition expected_lane_position(48.75, 0., 0.);

    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
    EXPECT_EQ(lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
    const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
        rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
    EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
    EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
    EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
  }
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLane, OpenScenarioRelativeRoadPositionToMaliputRoadPosition) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 0., 1.};
  const double ds = 50.;
  const double dt = 1.;
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_1");
  const maliput::api::LanePosition expected_lane_position(48.750, 1., 0.);
  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
}

class RoadGeometryOpenScenarioConversionsArcLaneRolled : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("ArcLaneRolled");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("ArcLaneRolled.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryOpenScenarioConversionsArcLaneRolled,
       RoundTripOpenScenarioLanePositionToMaliputRoadPositionAtCenterline) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 50., -1, 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const double expected_s = road_network_->road_geometry()->ById().GetLane(lane_id)->length() / 2.;
  const maliput::api::LanePosition expected_lane_position(expected_s, 0., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLaneRolled,
       RoundTripOpenScenarioLanePositionToMaliputRoadPositionWithOffset) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 50., -1, 0.5};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const double expected_s = road_network_->road_geometry()->ById().GetLane(lane_id)->length() / 2.;
  const double expected_r = input_xodr_lane_position.offset / (std::sqrt(2.) / 2.);  // The road is rolled 45 degrees
  const maliput::api::LanePosition expected_lane_position(expected_s, expected_r, 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLaneRolled,
       RoundTripOpenScenarioLanePositionToMaliputRoadPositionWithOffsetOffLane) {
  // OpenScenario/OpenDrive parameters.
  const double offset = -5.;  // 4 meters off the lane.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 50., -1, offset};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const double expected_s = road_network_->road_geometry()->ById().GetLane(lane_id)->length() / 2.;
  const double expected_r = input_xodr_lane_position.offset / (std::sqrt(2.) / 2.);  // The road is rolled 45 degrees
  const maliput::api::LanePosition expected_lane_position(expected_s, expected_r, 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLaneRolled,
       RoundTripOpenScenarioRoadPositionToMaliputRoadPositionAtCenterline) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const double expected_s = road_network_->road_geometry()->ById().GetLane(lane_id)->length() / 2.;
  const maliput::api::LanePosition expected_lane_position(expected_s, 1., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
      rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
  EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
  EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLaneRolled,
       RoundTripOpenScenarioRoadPositionToMaliputRoadPositionWithOffset) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., -0.5};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const maliput::api::Lane* lane{road_network_->road_geometry()->ById().GetLane(lane_id)};
  const double expected_s = lane->length() / 2.;
  const double expected_r =
      lane->lane_bounds(expected_s).max() -
      std::abs(input_xodr_road_position.t / (std::sqrt(2.) / 2.));  // The road is rolled 45 degrees
  const maliput::api::LanePosition expected_lane_position(expected_s, expected_r, 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
      rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
  EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
  EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
}

// This map differs with ArcLaneRolled with respect to a lane offset entry of 2m
class RoadGeometryOpenScenarioConversionsArcLaneRolledAndOffset : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("ArcLaneRolledAndOffset");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("ArcLaneRolledAndOffset.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryOpenScenarioConversionsArcLaneRolledAndOffset,
       RoundTripOpenScenarioRoadPositionToMaliputRoadPositionAtRoadsReferenceLine) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const double expected_s = road_network_->road_geometry()->ById().GetLane(lane_id)->length() / 2.;
  const maliput::api::LanePosition expected_lane_position(expected_s, -1., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
      rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
  EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
  EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsArcLaneRolledAndOffset,
       RoundTripOpenScenarioRoadPositionToMaliputRoadPositionWithOffset) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 1.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const maliput::api::Lane* lane{road_network_->road_geometry()->ById().GetLane(lane_id)};
  const double expected_s = lane->length() / 2.;
  const double expected_r =
      lane->lane_bounds(expected_s).min() +
      std::abs(input_xodr_road_position.t / (std::sqrt(2.) / 2.));  // The road is rolled 45 degrees
  const maliput::api::LanePosition expected_lane_position(expected_s, expected_r, 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
      rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
  EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
  EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
}

class RoadGeometryOpenScenarioConversionsLineMultipleSections : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("LineMultipleSections");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("LineMultipleSections.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryOpenScenarioConversionsLineMultipleSections,
       RoundTripOpenScenarioLanePositionToMaliputRoadPosition) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 50., -1, 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_1_-1");
  const maliput::api::LanePosition expected_lane_position(16.7, 0., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsLineMultipleSections,
       RoundTripOpenScenarioLanePositionToMaliputRoadPositionAtLaneStart) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 0., -1, 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_0_-1");
  const maliput::api::LanePosition expected_lane_position(0., 0., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsLineMultipleSections,
       RoundTripOpenScenarioLanePositionToMaliputRoadPositionAtLaneEnd) {
  // OpenScenario/OpenDrive parameters.
  const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 33.3, -1, 0.};
  // Maliput expected results.
  const maliput::api::LaneId lane_id("1_1_-1");
  const maliput::api::LanePosition expected_lane_position(0, 0., 0.);

  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::RoadPosition mali_road_pos =
      rg->OpenScenarioLanePositionToMaliputRoadPosition(input_xodr_lane_position);
  EXPECT_EQ(lane_id, mali_road_pos.lane->id());
  EXPECT_TRUE(
      AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition(mali_road_pos);
  EXPECT_EQ(input_xodr_lane_position.road_id, xodr_lane_pos.road_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.s - xodr_lane_pos.s) < constants::kLinearTolerance);
  EXPECT_EQ(input_xodr_lane_position.lane_id, xodr_lane_pos.lane_id);
  EXPECT_TRUE(std::abs(input_xodr_lane_position.offset - xodr_lane_pos.offset) < constants::kLinearTolerance);
}

TEST_F(RoadGeometryOpenScenarioConversionsLineMultipleSections, OpenScenarioRelativeRoadPositionToMaliputRoadPosition) {
  ///////////////////////////////////////////////////////
  // RelativeRoadPosition falling within the same Road //
  ///////////////////////////////////////////////////////
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 1.};
    const double ds = 20.;
    const double dt = -2.5;
    // Maliput expected results.
    const maliput::api::LaneId lane_id("1_2_-1");
    const maliput::api::LanePosition expected_lane_position(3.4, -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 1.};
    const double ds = -20.;
    const double dt = -2.5;
    // Maliput expected results.
    const maliput::api::LaneId lane_id("1_0_-1");
    const maliput::api::LanePosition expected_lane_position(30., -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
}

class RoadGeometryOpenScenarioConversionsLineVariableOffset : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("LineVariableOffset");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("LineVariableOffset.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryOpenScenarioConversionsLineVariableOffset, GetRoadOrientationAtOpenScenarioRoadPosition) {
  const RoadGeometry::OpenScenarioRoadPosition xodr_road_position{1, 33., 0.};
  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::math::RollPitchYaw expected_roll_pitch_roll{0., 0., 0.};
  const maliput::math::RollPitchYaw roll_pitch_roll =
      rg->GetRoadOrientationAtOpenScenarioRoadPosition(xodr_road_position);
  EXPECT_TRUE(AssertCompare(CompareVectors(expected_roll_pitch_roll.vector(), roll_pitch_roll.vector(),
                                           constants::kAngularTolerance, maliput::math::CompareType::kAbsolute)));
}

// Test at the begining of the lane 1_0_3 and in the middle of the lane 1_0_-3.
// It should be the same xodr_t due to the effect of the lane offset in this map
TEST_F(RoadGeometryOpenScenarioConversionsLineVariableOffset, RoundTripOpenScenarioRoadPositionToMaliputRoadPosition) {
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 0., 5.};
    // Maliput expected results.
    const maliput::api::LaneId lane_id("1_0_3");
    const maliput::api::LanePosition expected_lane_position(0., 0., 0.);

    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
    EXPECT_EQ(lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
    const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
        rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
    EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
    EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
    EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
  }
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 50., 5.};
    // Maliput expected results.
    const maliput::api::LaneId lane_id("1_0_-3");
    const double expected_s = road_network_->road_geometry()->ById().GetLane(lane_id)->length() / 2.;
    const maliput::api::LanePosition expected_lane_position(expected_s, 0., 0.);

    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRoadPositionToMaliputRoadPosition(input_xodr_road_position);
    EXPECT_EQ(lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
    const RoadGeometry::OpenScenarioRoadPosition xodr_road_pos =
        rg->MaliputRoadPositionToOpenScenarioRoadPosition(mali_road_pos);
    EXPECT_EQ(input_xodr_road_position.road_id, xodr_road_pos.road_id);
    EXPECT_TRUE(std::abs(input_xodr_road_position.s - xodr_road_pos.s) < constants::kLinearTolerance);
    EXPECT_TRUE(std::abs(input_xodr_road_position.t - xodr_road_pos.t) < constants::kLinearTolerance);
  }
}

TEST_F(RoadGeometryOpenScenarioConversionsLineVariableOffset,
       OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition) {
  ///////////////////////////////////////////////////////
  // RelativeLanePosition falling within the same Road //
  ///////////////////////////////////////////////////////
  // Final pos in same lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 20., 1,
                                                                          0.};  // Lane 1_0_1's srh(20.3756,0,0)
    const int d_lane = 0;
    const double xodr_ds = 2.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_1");
    const maliput::api::LanePosition expected_lane_position(22.4592, 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos = rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
        input_xodr_lane_position, d_lane, xodr_ds, offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 20., 1,
                                                                          0.};  // Lane 1_0_1's srh(20.3756,0,0)
    const int d_lane = -2;
    const double xodr_ds = 30.;
    const double offset = -0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_-2");
    const maliput::api::LanePosition expected_lane_position(51.1802, -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos = rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
        input_xodr_lane_position, d_lane, xodr_ds, offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
}

TEST_F(RoadGeometryOpenScenarioConversionsLineVariableOffset,
       OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition) {
  ///////////////////////////////////////////////////////
  // RelativeLanePosition falling within the same Road //
  ///////////////////////////////////////////////////////
  // Final pos in same lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 20., 1,
                                                                          0.};  // Lane 1_0_1's srh(20.3756,0,0)
    const int d_lane = 0;
    const double ds_lane = 2.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_1");
    const maliput::api::LanePosition expected_lane_position(22.3756, 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane, ds_lane,
                                                                            offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{1, 20., 1,
                                                                          0.};  // Lane 1_0_1's srh(20.3756,0,0)
    const int d_lane = -2;
    const double ds_lane = 30.;
    const double offset = -0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_-2");
    const maliput::api::LanePosition expected_lane_position(50.3756, -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane, ds_lane,
                                                                            offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
}

class RoadGeometryOpenScenarioConversionsSingleRoadSDirectionChange : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("SingleRoadSDirectionChange");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("SingleRoadSDirectionChange.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryOpenScenarioConversionsSingleRoadSDirectionChange,
       OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition) {
  ///////////////////////////////////////////////////////
  // RelativeLanePosition falling within the same Road //
  ///////////////////////////////////////////////////////
  // Final pos in same lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5,0,0)
    const int d_lane = 0;
    const double ds_lane = 2.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("2_0_1");
    const maliput::api::LanePosition expected_lane_position(7., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane, ds_lane,
                                                                            offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5,0,0)
    const int d_lane = -1;
    const double ds_lane = 2.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("2_0_-1");
    const maliput::api::LanePosition expected_lane_position(7., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane, ds_lane,
                                                                            offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos out of bounds
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5,0,0)
    const int d_lane = 1;
    const double ds_lane = 2.;
    const double offset = 0.5;
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    EXPECT_THROW(rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane,
                                                                                     ds_lane, offset),
                 maliput::common::assertion_error);
  }
  // d_lane goes off road and throws
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5,0,0)
    const int d_lane = 10;
    const double ds_lane = 2.;
    const double offset = 0.5;
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    EXPECT_THROW(rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane,
                                                                                     ds_lane, offset),
                 maliput::common::assertion_error);
  }
  ///////////////////////////////////////////////////////////
  // RelativeLanePosition not falling within the same Road //
  ///////////////////////////////////////////////////////////
  // Final pos in same lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5, 0, 0)
    const int d_lane = 0;
    const double ds_lane = 10.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_-1");
    const maliput::api::LanePosition expected_lane_position(5., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane, ds_lane,
                                                                            offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 7., -1, 0.};  // Lane 2_0_-1's srh(7, 0, 0)
    const int d_lane = 1;
    const double ds_lane = 5.;
    const double offset = -0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_-1");
    const maliput::api::LanePosition expected_lane_position(8., -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane, ds_lane,
                                                                            offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 7., 1, 0.};  // Lane 2_0_1's srh(7, 0, 0)
    const int d_lane = -1;
    const double ds_lane = -10.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("3_0_1");
    const maliput::api::LanePosition expected_lane_position(3., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(input_xodr_lane_position, d_lane, ds_lane,
                                                                            offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
}

TEST_F(RoadGeometryOpenScenarioConversionsSingleRoadSDirectionChange,
       OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition) {
  ///////////////////////////////////////////////////////
  // RelativeLanePosition falling within the same Road //
  ///////////////////////////////////////////////////////
  // Final pos in same lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5,0,0)
    const int d_lane = 0;
    const double xodr_ds = 2.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("2_0_1");
    const maliput::api::LanePosition expected_lane_position(7., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos = rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
        input_xodr_lane_position, d_lane, xodr_ds, offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5,0,0)
    const int d_lane = -1;
    const double xodr_ds = 2.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("2_0_-1");
    const maliput::api::LanePosition expected_lane_position(7., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos = rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
        input_xodr_lane_position, d_lane, xodr_ds, offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos out of bounds
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5,0,0)
    const int d_lane = 1;
    const double xodr_ds = 2.;
    const double offset = 0.5;
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    EXPECT_THROW(rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(input_xodr_lane_position, d_lane,
                                                                                 xodr_ds, offset),
                 maliput::common::assertion_error);
  }
  ///////////////////////////////////////////////////////////
  // RelativeLanePosition not falling within the same Road //
  ///////////////////////////////////////////////////////////
  // Final pos in same lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 5., 1, 0.};  // Lane 2_0_1's srh(5, 0, 0)
    const int d_lane = 0;
    const double xodr_ds = 10.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_-1");
    const maliput::api::LanePosition expected_lane_position(5., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos = rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
        input_xodr_lane_position, d_lane, xodr_ds, offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 7., -1, 0.};  // Lane 2_0_-1's srh(7, 0, 0)
    const int d_lane = 1;
    const double xodr_ds = 5.;
    const double offset = -0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_-1");
    const maliput::api::LanePosition expected_lane_position(8., -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos = rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
        input_xodr_lane_position, d_lane, xodr_ds, offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in other lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioLanePosition input_xodr_lane_position{2, 7., 1, 0.};  // Lane 2_0_1's srh(7, 0, 0)
    const int d_lane = -1;
    const double xodr_ds = -10.;
    const double offset = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("3_0_1");
    const maliput::api::LanePosition expected_lane_position(3., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos = rg->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
        input_xodr_lane_position, d_lane, xodr_ds, offset);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
}

TEST_F(RoadGeometryOpenScenarioConversionsSingleRoadSDirectionChange,
       OpenScenarioRelativeRoadPositionToMaliputRoadPosition) {
  ///////////////////////////////////////////////////////////
  // RelativeRoadPosition not falling within the same Road //
  ///////////////////////////////////////////////////////////
  // Final pos in same lane
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{2, 5., 1.};  // Lane 2_0_1's srh(5,0,0)
    const double ds = 2.;
    const double dt = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("2_0_1");
    const maliput::api::LanePosition expected_lane_position(7., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in next lane: 1_0_-1 (Change in s direction)
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{2, 5., 1.};  // Lane 2_0_1's srh(5,0,0)
    const double ds = 7.;
    const double dt = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_-1");
    const maliput::api::LanePosition expected_lane_position(8., -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in next lane: 1_0_1 (Change in s direction)
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{2, 5., -1.};  // Lane 2_0_-1's srh(5,0,0)
    const double ds = 7.;
    const double dt = -0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_1");
    const maliput::api::LanePosition expected_lane_position(8., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in previous lane: 3_0_-1 (Change in s direction)
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{2, 5., 1.};  // Lane 2_0_1's srh(5,0,0)
    const double ds = -7.;
    const double dt = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("3_0_-1");
    const maliput::api::LanePosition expected_lane_position(2., -0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in previous lane: 3_0_1 (Change in s direction)
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{2, 5., -1.};  // Lane 2_0_-1's srh(5,0,0)
    const double ds = -7.;
    const double dt = -0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("3_0_1");
    const maliput::api::LanePosition expected_lane_position(2., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in two next lanes: 3_0_1 (Change in s direction in the middle)
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{1, 5., 1.};  // Lane 1_0_1's srh(5,0,0)
    const double ds = 17.;
    const double dt = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("3_0_1");
    const maliput::api::LanePosition expected_lane_position(2., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
  // Final pos in two previous lanes: 1_0_1 (Change in s direction in the middle)
  {
    // OpenScenario/OpenDrive parameters.
    const RoadGeometry::OpenScenarioRoadPosition input_xodr_road_position{3, 5., 1.};  // Lane 3_0_1's srh(5,0,0)
    const double ds = -17.;
    const double dt = 0.5;
    // Maliput expected results.
    const maliput::api::LaneId expected_lane_id("1_0_1");
    const maliput::api::LanePosition expected_lane_position(8., 0.5, 0.);
    auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
    const maliput::api::RoadPosition mali_road_pos =
        rg->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(input_xodr_road_position, ds, dt);
    EXPECT_EQ(expected_lane_id, mali_road_pos.lane->id());
    EXPECT_TRUE(
        AssertCompare(IsLanePositionClose(expected_lane_position, mali_road_pos.pos, constants::kLinearTolerance)));
  }
}

class RoadGeometryOpenScenarioConversionsSShapeSuperelevatedRoad : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("SShapeSuperelevatedRoad");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("SShapeSuperelevatedRoad.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(RoadGeometryOpenScenarioConversionsSShapeSuperelevatedRoad, GetRoadOrientationAtOpenScenarioRoadPosition) {
  const RoadGeometry::OpenScenarioRoadPosition xodr_road_position{1, 60., 0.};
  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::math::RollPitchYaw expected_roll_pitch_roll{-0.745567, 0., 1.5};
  const maliput::math::RollPitchYaw roll_pitch_roll =
      rg->GetRoadOrientationAtOpenScenarioRoadPosition(xodr_road_position);
  EXPECT_TRUE(AssertCompare(CompareVectors(expected_roll_pitch_roll.vector(), roll_pitch_roll.vector(),
                                           constants::kAngularTolerance, maliput::math::CompareType::kAbsolute)));
}

// Holds the input and expected output for the DoBackendCustomCommand test.
struct CommandsInputOutputs {
  std::string command;
  std::string expected_output;
};

std::vector<CommandsInputOutputs> InstanciateCommandsInputOutputsParameters() {
  return {
      {{"OpenScenarioLanePositionToMaliputRoadPosition,1,50,-1,0."}, {"1_0_-1,51.250000,0.000000,0.000000"}},
      {{"OpenScenarioLanePositionToMaliputRoadPosition,1,50,-1,0.5"}, {"1_0_-1,51.250000,0.500000,0.000000"}},
      {{"OpenScenarioRoadPositionToMaliputRoadPosition,1,50,0."}, {"1_0_-1,51.250000,1.000000,0.000000"}},
      {{"OpenScenarioRoadPositionToMaliputRoadPosition,1,50,-1."}, {"1_0_-1,51.250000,0.000000,0.000000"}},
      {{"OpenScenarioRoadPositionToMaliputRoadPosition,1,50,1."}, {"1_0_1,48.750000,0.000000,0.000000"}},
      {{"MaliputRoadPositionToOpenScenarioLanePosition,1_0_-1,51.25,0.,0."}, {"1,50.000000,-1,0.000000"}},
      {{"MaliputRoadPositionToOpenScenarioLanePosition,1_0_-1,51.25,0.5,0."}, {"1,50.000000,-1,0.500000"}},
      {{"MaliputRoadPositionToOpenScenarioRoadPosition,1_0_-1,51.25,1.,0."}, {"1,50.000000,0.000000"}},
      {{"MaliputRoadPositionToOpenScenarioRoadPosition,1_0_-1,51.25,0.,0."}, {"1,50.000000,-1.000000"}},
      {{"MaliputRoadPositionToOpenScenarioRoadPosition,1_0_1,48.75,0.,0."}, {"1,50.000000,1.000000"}},
      {{"OpenScenarioRelativeRoadPositionToMaliputRoadPosition,1,0.,1.,50.,1."}, {"1_0_1,48.750000,1.000000,0.000000"}},
      {{"OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition,1,1,0.,-1,50.,-0.8"},
       {"1_0_-1,48.750000,-0.800000,0.000000"}},
      {{"OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition,1,-1,0.,1,50.,0.8"},
       {"1_0_1,50.000000,0.800000,0.000000"}},
      {{"GetRoadOrientationAtOpenScenarioRoadPosition,1,50.,0."}, {"0.000000,-0.000000,1.250000"}},
  };
}

// Test the DoBackendCustomCommand method.
class RoadGeometryDoBackendCustomCommand : public ::testing::TestWithParam<CommandsInputOutputs> {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("ArcLane");
    road_geometry_configuration_.opendrive_file = utility::FindResourceInPath("ArcLane.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_P(RoadGeometryDoBackendCustomCommand, DoBackendCustomCommand) {
  const auto input_output = GetParam();
  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const std::string result = rg->BackendCustomCommand(input_output.command);
  EXPECT_EQ(input_output.expected_output, result);
}

INSTANTIATE_TEST_CASE_P(RoadGeometryDoBackendCustomCommandGroup, RoadGeometryDoBackendCustomCommand,
                        ::testing::ValuesIn(InstanciateCommandsInputOutputsParameters()));

// Holds the input and expected output for the DoBackendCustomCommand test.
struct GeoReferenceInfoParams {
  std::string xodr_file;
  std::string expected_output;
};

std::vector<GeoReferenceInfoParams> InstanciateGeoReferenceParameters() {
  return {{"Town01.xodr", "+lat_0=4.9000000000000000e+1 +lon_0=8.0000000000000000e+0"}, {"ArcLane.xodr", ""}};
}

class RoadGeometryGeoReferenceInfo : public ::testing::TestWithParam<GeoReferenceInfoParams> {
 protected:
  void SetUp() override { road_geometry_configuration_.id = maliput::api::RoadGeometryId("GeoReferenceInfo"); }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_P(RoadGeometryGeoReferenceInfo, DoGeoReferenceInfo) {
  const auto param = GetParam();
  road_geometry_configuration_.opendrive_file = utility::FindResourceInPath(param.xodr_file, kMalidriveResourceFolder);
  road_network_ =
      ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const std::string result = rg->GeoReferenceInfo();
  EXPECT_EQ(param.expected_output, result);
}

INSTANTIATE_TEST_CASE_P(RoadGeometryGeoReferenceInfoGroup, RoadGeometryGeoReferenceInfo,
                        ::testing::ValuesIn(InstanciateGeoReferenceParameters()));

class ShortStraightLanesMaliputRoadPositionToOpenScenarioLanePosition : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("ShortStraightLanes");
    road_geometry_configuration_.opendrive_file =
        utility::FindResourceInPath("ShortStraightLanes.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }
  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

TEST_F(ShortStraightLanesMaliputRoadPositionToOpenScenarioLanePosition, AtSEnd) {
  auto rg = dynamic_cast<const RoadGeometry*>(road_network_->road_geometry());
  const maliput::api::LaneId lane_id("49_1_4");
  const auto lane = rg->ById().GetLane(lane_id);
  ASSERT_NE(lane, nullptr);
  const double s_end = lane->length();
  const RoadGeometry::OpenScenarioLanePosition xodr_lane_pos =
      rg->MaliputRoadPositionToOpenScenarioLanePosition({lane, {s_end, 0., 0.}});
  EXPECT_EQ(xodr_lane_pos.road_id, 49);
  EXPECT_EQ(xodr_lane_pos.lane_id, 4);
  EXPECT_NEAR(xodr_lane_pos.s, 1.0758588807064431,
              rg->linear_tolerance());  // Value obtained from xodr: road 49, s=0 of third lane section
  EXPECT_EQ(xodr_lane_pos.offset, 0.);
}

// Tests GetCurvature method on an arc lane from ArcLane.xodr.
// The ArcLane.xodr file defines an arc with:
//   - Reference line curvature: κ₀ = 0.025 (radius = 40m, turning left)
//   - Lane width: 2.0m
//   - Three lanes: id=1 (left), id=0 (center), id=-1 (right)
//
// For a parallel curve, the curvature at lateral offset r from the reference line is:
//   κ(r) = κ₀ / (1 + r * κ₀)
//
// In this road, lane 1_0_1 (left of reference) and lane 1_0_-1 (right of reference)
// have their centerlines at different distances from the center of curvature.
class ArcLaneGetCurvatureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    road_geometry_configuration_.id = maliput::api::RoadGeometryId("ArcLane");
    road_geometry_configuration_.opendrive_file = utility::FindResourceInPath("ArcLane.xodr", kMalidriveResourceFolder);
    road_network_ =
        ::malidrive::loader::Load<::malidrive::builder::RoadNetworkBuilder>(road_geometry_configuration_.ToStringMap());
  }

  // Reference line curvature from the XODR file.
  static constexpr double kReferenceCurvature{0.025};
  // Lane width from the XODR file.
  static constexpr double kLaneWidth{2.0};

  builder::RoadGeometryConfiguration road_geometry_configuration_{};
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{nullptr};
};

// Tests that GetCurvature returns the expected curvature at the lane centerline.
// The curvature follows the parallel curve formula: κ(r) = κ₀ / (1 + r * κ₀)
// Based on the observed behavior:
//   - Left lane (1_0_1) centerline has higher curvature ≈ 0.0256 (r_total ≈ -1 from reference)
//   - Right lane (1_0_-1) centerline has lower curvature ≈ 0.0244 (r_total ≈ +1 from reference)
TEST_F(ArcLaneGetCurvatureTest, CurvatureAtCenterline) {
  const auto* rg = road_network_->road_geometry();
  const double tolerance = rg->linear_tolerance();

  // Test left lane (1_0_1)
  // The left lane centerline is at r_total = -1 from the reference line.
  // Expected curvature: κ = 0.025 / (1 + (-1) * 0.025) = 0.025 / 0.975 ≈ 0.0256
  {
    const auto* lane = rg->ById().GetLane(maliput::api::LaneId("1_0_1"));
    ASSERT_NE(lane, nullptr);

    const double r_total = -1.0;  // Left lane centerline offset from reference
    const double expected_curvature = kReferenceCurvature / (1.0 + r_total * kReferenceCurvature);

    // Test at various s positions along the lane.
    const double s_mid = lane->length() / 2.0;
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({0.0, 0.0, 0.0}), tolerance);
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({s_mid, 0.0, 0.0}), tolerance);
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({lane->length(), 0.0, 0.0}), tolerance);
  }

  // Test right lane (1_0_-1)
  // The right lane centerline is at r_total = +1 from the reference line.
  // Expected curvature: κ = 0.025 / (1 + 1 * 0.025) = 0.025 / 1.025 ≈ 0.0244
  {
    const auto* lane = rg->ById().GetLane(maliput::api::LaneId("1_0_-1"));
    ASSERT_NE(lane, nullptr);

    const double r_total = 1.0;  // Right lane centerline offset from reference
    const double expected_curvature = kReferenceCurvature / (1.0 + r_total * kReferenceCurvature);

    const double s_mid = lane->length() / 2.0;
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({0.0, 0.0, 0.0}), tolerance);
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({s_mid, 0.0, 0.0}), tolerance);
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({lane->length(), 0.0, 0.0}), tolerance);
  }
}

// Tests that GetCurvature correctly accounts for the r-coordinate (lateral offset within the lane).
// For a parallel curve at lateral offset r_total from the reference line:
//   κ(r_total) = κ₀ / (1 + r_total * κ₀)
TEST_F(ArcLaneGetCurvatureTest, CurvatureWithLateralOffset) {
  const auto* rg = road_network_->road_geometry();
  const double tolerance = rg->linear_tolerance();

  const auto* lane = rg->ById().GetLane(maliput::api::LaneId("1_0_-1"));
  ASSERT_NE(lane, nullptr);

  const double s_mid = lane->length() / 2.0;
  // Right lane centerline is at r_total = +1 from reference line
  const double lane_centerline_offset = 1.0;

  // Test at r = 0 (lane centerline)
  {
    const double r = 0.0;
    const double r_total = lane_centerline_offset + r;
    const double expected_curvature = kReferenceCurvature / (1.0 + r_total * kReferenceCurvature);
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({s_mid, r, 0.0}), tolerance);
  }

  // Test at r = +0.5 (toward the left in lane frame)
  {
    const double r = 0.5;
    const double r_total = lane_centerline_offset + r;
    const double expected_curvature = kReferenceCurvature / (1.0 + r_total * kReferenceCurvature);
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({s_mid, r, 0.0}), tolerance);
  }

  // Test at r = -0.5 (toward the right in lane frame)
  {
    const double r = -0.5;
    const double r_total = lane_centerline_offset + r;
    const double expected_curvature = kReferenceCurvature / (1.0 + r_total * kReferenceCurvature);
    EXPECT_NEAR(expected_curvature, lane->GetCurvature({s_mid, r, 0.0}), tolerance);
  }
}

// Tests that curvature varies correctly with lateral position.
// For positive reference curvature κ₀ = 0.025 (turning left), the center of curvature is to the left.
// The parallel curve formula κ(r) = κ₀ / (1 + r * κ₀) shows that:
//   - Points farther from center of curvature (higher r) have lower curvature
//   - Points closer to center of curvature (lower r) have higher curvature
TEST_F(ArcLaneGetCurvatureTest, CurvatureVariesWithLateralPosition) {
  const auto* rg = road_network_->road_geometry();

  const auto* left_lane = rg->ById().GetLane(maliput::api::LaneId("1_0_1"));
  const auto* right_lane = rg->ById().GetLane(maliput::api::LaneId("1_0_-1"));
  ASSERT_NE(left_lane, nullptr);
  ASSERT_NE(right_lane, nullptr);

  const double s_mid_left = left_lane->length() / 2.0;
  const double s_mid_right = right_lane->length() / 2.0;

  const double curvature_left = left_lane->GetCurvature({s_mid_left, 0.0, 0.0});
  const double curvature_right = right_lane->GetCurvature({s_mid_right, 0.0, 0.0});

  // The left lane (closer to center of curvature) should have higher curvature than the right lane.
  // This is because its r_total is more negative (closer to the center of curvature).
  EXPECT_GT(curvature_left, curvature_right);
}

}  // namespace
}  // namespace tests
}  // namespace malidrive
