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
#include "maliput_malidrive/base/lane.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/assertion_error.h>
#include <maliput/geometry_base/junction.h>
#include <maliput/geometry_base/road_geometry.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/road_curve/arc_ground_curve.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/ground_curve.h"
#include "maliput_malidrive/road_curve/lane_offset.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/piecewise_ground_curve.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace test {
namespace {

using maliput::api::InertialPosition;
using maliput::api::LanePosition;
using maliput::api::LanePositionResult;
using maliput::api::Rotation;
using maliput::api::test::IsHBoundsClose;
using maliput::api::test::IsInertialPositionClose;
using maliput::api::test::IsLanePositionClose;
using maliput::api::test::IsRBoundsClose;
using maliput::api::test::IsRotationClose;
using maliput::geometry_base::Junction;
using maliput::math::Quaternion;
using maliput::math::Vector2;
using maliput::math::Vector3;

#define IsLanePositionResultClose(lpr_a, lpr_b, tolerance)                                           \
  do {                                                                                               \
    EXPECT_TRUE(IsLanePositionClose(lpr_a.lane_position, lpr_b.lane_position, tolerance));           \
    EXPECT_TRUE(IsInertialPositionClose(lpr_a.nearest_position, lpr_b.nearest_position, tolerance)); \
    EXPECT_NEAR(lpr_a.distance, lpr_b.distance, tolerance);                                          \
  } while (0);

std::unique_ptr<road_curve::Function> MakeCubicPolynomial(double a, double b, double c, double d, double p0, double p1,
                                                          double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(a, b, c, d, p0, p1, linear_tolerance);
}

std::unique_ptr<road_curve::Function> MakeConstantCubicPolynomial(double d, double p0, double p1,
                                                                  double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., d, p0, p1, linear_tolerance);
}

std::unique_ptr<road_curve::Function> MakeZeroCubicPolynomial(double p0, double p1, double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 0., p0, p1, linear_tolerance);
}

// Approximates a path length lower bound for the given @p road_curve
// from @p p0 to @p p1, for constant @p r and @p h offsets, by
// computing the same integral for a 2^@p k_order linear approximation.
//
// @param road_curve The RoadCurve to compute path length for.
// @param p0 The lower integration bound for the p coordinate.
// @param p1 The upper integration bound for the p coordinate.
// @param r The r coordinate offset.
// @param h The h coordinate offset.
// @param k_order Order k of the linear approximation, i.e. 2^k segments
//                are used in the approximation.
// @pre Given lower integration bound @p p0 doesn't meet the range `[road_curve.p0() ; road_curve.p1()]`
// @pre Given upper integration bound @p p1 doesn't meet the range `[road_curve.p0() ; road_curve.p1()]`
// @pre Given lower integration bound @p p0 is greater than or equal to 0.
// @pre Given upper integration bound @p p1 is greater than or equal to
//      the given lower integration bound @p p0.
// @pre Given @p k_order for the linear approximation is a non-negative number.
// @throws maliput::common::assertion_error if preconditions are not met.
double BruteForcePathLengthIntegral(const road_curve::RoadCurve& road_curve, double p0, double p1, double r, double h,
                                    int k_order) {
  MALIDRIVE_IS_IN_RANGE(p0, road_curve.p0(), road_curve.p1());
  MALIDRIVE_IS_IN_RANGE(p1, road_curve.p0(), road_curve.p1());
  MALIPUT_THROW_UNLESS(p1 >= 0);
  MALIPUT_THROW_UNLESS(p1 >= p0);
  MALIPUT_THROW_UNLESS(k_order >= 0);
  double length = 0.0;
  const double d_p = (p1 - p0);
  const int iterations = std::pow(2, k_order);
  // Splits the [p0, p1] interval in 2^k intervals, computes the positions
  // in the global frame for each interval boundary and sums up the path lengths
  // of the segments in the global frame that correspond to each one of those
  // intervals.
  Vector3 inertial_position_at_prev_p = road_curve.W(Vector3(p0, r, h));
  for (int i = 1; i <= iterations; ++i) {
    const double p = p0 + d_p * static_cast<double>(i) / iterations;
    const Vector3 inertial_position_at_p = road_curve.W(Vector3(p, r, h));
    const double ith_step_length = (inertial_position_at_p - inertial_position_at_prev_p).norm();
    length += ith_step_length;
    inertial_position_at_prev_p = inertial_position_at_p;
  }
  return length;
}

class LaneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        kAssertContiguity);
  }

  const maliput::api::LaneId kId{"dut"};
  const int kXordTrack{1};
  const int kXordTrackInvalid{-1};
  const int kXodrLaneId{5};
  const maliput::api::HBounds kElevationBounds{0., 5.};
  const double kP0{0.};
  const double kP1{100.};
  const double kLinearTolerance{1e-11};
  const double kScaleLength{1.};
  const Vector2 kXy0{10., 12.};
  const Vector2 kDXy{(kP1 - kP0) * std::sqrt(2.) / 2., (kP1 - kP0) * std::sqrt(2.) / 2.};
  const double kWidth{5.};
  const double kLaneOffset{10.};
  const std::optional<double> kParserSTolerance{std::nullopt};  // Disables the check because it is not needed.
  const xodr::ParserConfiguration kParserConfiguration{kParserSTolerance};
  const bool kAssertContiguity{true};
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
};

TEST_F(LaneTest, Constructor) {
  EXPECT_NO_THROW(Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1));
}

TEST_F(LaneTest, ConstructorAssertions) {
  const double kP0Off{1.};
  const double kP1Off{99};

  // Invalid XODR Track ID.
  EXPECT_THROW(Lane(kId, kXordTrackInvalid, kXodrLaneId, kElevationBounds, road_curve_.get(),
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kP0, kP1),
               maliput::common::assertion_error);
  // Invalid lane_width.
  EXPECT_THROW(Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(), nullptr,
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kP0, kP1),
               maliput::common::assertion_error);
  // Invalid lane_offset.
  EXPECT_THROW(Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), nullptr, kP0, kP1),
               maliput::common::assertion_error);
  // Invalid road_curve.
  EXPECT_THROW(
      Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, nullptr, MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
           MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kP0, kP1),
      maliput::common::assertion_error);
  // Out of range lane_width.
  EXPECT_THROW(Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                    MakeZeroCubicPolynomial(kP0Off, kP1, kLinearTolerance),
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kP0, kP1),
               maliput::common::assertion_error);
  EXPECT_THROW(Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                    MakeZeroCubicPolynomial(kP0, kP1Off, kLinearTolerance),
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kP0, kP1),
               maliput::common::assertion_error);
  // Out of range lane_offset.
  EXPECT_THROW(Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
                    MakeZeroCubicPolynomial(kP0Off, kP1, kLinearTolerance), kP0, kP1),
               maliput::common::assertion_error);
  EXPECT_THROW(Lane(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                    MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
                    MakeZeroCubicPolynomial(kP0, kP1Off, kLinearTolerance), kP0, kP1),
               maliput::common::assertion_error);
}

TEST_F(LaneTest, BasicLaneAccessors) {
  const Lane dut(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                 MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
                 MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kP0, kP1);

  EXPECT_EQ(kXordTrack, dut.get_track());
  EXPECT_EQ(kXodrLaneId, dut.get_lane_id());
  EXPECT_NEAR(kP0, dut.get_track_s_start(), kLinearTolerance);
  EXPECT_NEAR(kP1, dut.get_track_s_end(), kLinearTolerance);
  EXPECT_NEAR((kP1 - kP0) / 2., dut.TrackSFromLaneS(kDXy.norm() / 2.), kLinearTolerance);
  EXPECT_NEAR(kDXy.norm() / 2., dut.LaneSFromTrackS((kP1 - kP0) / 2.), kLinearTolerance);
}

// Template of a XODR description that contains only the xodr header.
constexpr const char* kXODRHeaderTemplate = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor="1" revMinor="1" name="Test" version="1.21" date="Mon Jun 1 12:00:00 2020"
    north="0" south="0" east="0" west="O" vendor="Toyota Research Institute" >
  </header>
  <road name="Road 0" length="1.4005927435591335e+2" id="0" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="1.3" length="2.">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s='0.'>
        <left>
          <lane id='1' type='driving' level= '0'>
            <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
          </lane>
        </left>
        <center>
          <lane id='0' type='driving' level= '0'>
          </lane>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
)R";

// Initializes a single flat line Lane in a single Junction - Segment environment.
class MalidriveFlatLineLaneFullyInitializedTest : public LaneTest {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));
  }

  const double kAngularTolerance{1e-6};
  const double kSStart{0.};
  const double kSHalf{50.};
  const double kSEnd{100.};
  const double kRCenterline{0.};
  const double kRLeft{1.};
  const double kRRight{-2.};
  const double kH{0};
  const Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  std::unique_ptr<RoadGeometry> road_geometry_;
  const Lane* dut_{};
};

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, Length) {
  EXPECT_NEAR(dut_->length(), kDXy.norm(), kLinearTolerance);
}

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, TrackSFromLaneS) {
  EXPECT_NEAR(kP0, dut_->TrackSFromLaneS(kSStart), kLinearTolerance);
  EXPECT_NEAR((kP1 + kP0) / 2., dut_->TrackSFromLaneS(kSHalf), kLinearTolerance);
  EXPECT_NEAR(kP1, dut_->TrackSFromLaneS(kSEnd), kLinearTolerance);
}

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, LaneSFromTrackS) {
  EXPECT_NEAR(kSStart, dut_->LaneSFromTrackS(kP0), kLinearTolerance);
  EXPECT_NEAR(kSHalf, dut_->LaneSFromTrackS((kP1 + kP0) / 2.), kLinearTolerance);
  EXPECT_NEAR(kSEnd, dut_->LaneSFromTrackS(kP1), kLinearTolerance);
}

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, Bounds) {
  // At the beginning of the lane.
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->lane_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->segment_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(0., 0.), kLinearTolerance));
  // At the end of the lane.
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->lane_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->segment_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(dut_->length(), 0.), kLinearTolerance));
}

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, ToInertialPosition) {
  // At centerline.
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(2.9289321881345254, 19.071067811865476, 0.),
                                      dut_->ToInertialPosition({kSStart, kRCenterline, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(38.2842712474619, 54.426406871192846, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRCenterline, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(73.63961030678928, 89.78174593052022, 0.),
                                      dut_->ToInertialPosition({kSEnd, kRCenterline, kH}), kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(2.2218254069479784, 19.77817459305202, 0.),
                                      dut_->ToInertialPosition({kSStart, kRLeft, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(37.577164466275356, 55.13351365237939, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRLeft, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(72.93250352560273, 90.48885271170676, 0.),
                                      dut_->ToInertialPosition({kSEnd, kRLeft, kH}), kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(4.34314575050762, 17.65685424949238, 0.),
                                      dut_->ToInertialPosition({kSStart, kRRight, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(39.698484809834994, 53.01219330881975, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRRight, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(75.05382386916237, 88.36753236814712, 0.),
                                      dut_->ToInertialPosition({kSEnd, kRRight, kH}), kLinearTolerance));
  //@}
}

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, ToLanePosition) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{2.9289321881345254, 19.071067811865476, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{38.2842712474619, 54.426406871192846, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{73.63961030678928, 89.78174593052022, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the left
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{2.2218254069479784, 19.77817459305202, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{37.577164466275356, 55.13351365237939, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{72.93250352560273, 90.48885271170676, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the right
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRRight, kH};
  expected_result.nearest_position = InertialPosition{4.34314575050762, 17.65685424949238, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRRight, kH};
  expected_result.nearest_position = InertialPosition{39.698484809834994, 53.01219330881975, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRRight, kH};
  expected_result.nearest_position = InertialPosition{75.05382386916237, 88.36753236814712, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, GetOrientation) {
  const Rotation kExpectedRotation = Rotation::FromRpy(/* roll */ 0., /* pitch */ 0., M_PI / 4.);

  // At centerline.
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSStart, kRCenterline, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSHalf, kRCenterline, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSEnd, kRCenterline, kH}), kAngularTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSStart, kRLeft, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSHalf, kRLeft, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSEnd, kRLeft, kH}), kAngularTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSStart, kRRight, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSHalf, kRRight, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotation, dut_->GetOrientation({kSEnd, kRRight, kH}), kAngularTolerance));
  //@}
}

TEST_F(MalidriveFlatLineLaneFullyInitializedTest, EvalMotionDerivatives) {
  const maliput::api::IsoLaneVelocity kVelocity{3., 2., 1.};
  const LanePosition kExpectedResult{3., 2., 1.};

  // At centerline.
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSStart, kRCenterline, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSHalf, kRCenterline, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSEnd, kRCenterline, kH}, kVelocity),
                                  kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSStart, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSHalf, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSEnd, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSStart, kRRight, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSHalf, kRRight, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult, dut_->EvalMotionDerivatives({kSEnd, kRRight, kH}, kVelocity),
                                  kLinearTolerance));
  //@}
}

// Initializes a single flat line Lane in a single Junction - Segment environment.
// Provides a non-zero Inertial to Backend Frame translation.
class MalidriveFlatLineLaneFullyInitializedWithInertialToBackendFrameTranslationTest : public LaneTest {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));
  }

  const double kAngularTolerance{1e-6};
  const double kSStart{0.};
  const double kSHalf{50.};
  const double kSEnd{100.};
  const double kRCenterline{0.};
  const double kRLeft{1.};
  const double kRRight{-2.};
  const double kH{0};
  const Vector3 kInertialToBackendFrameTranslation{1., 2., .5};
  std::unique_ptr<RoadGeometry> road_geometry_;
  const Lane* dut_{};
};

TEST_F(MalidriveFlatLineLaneFullyInitializedWithInertialToBackendFrameTranslationTest, ToInertialPosition) {
  // At centerline.
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(3.9289321881345254, 21.071067811865476, 0.5),
                                      dut_->ToInertialPosition({kSStart, kRCenterline, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(74.63961030678928, 91.78174593052022, 0.5),
                                      dut_->ToInertialPosition({kSEnd, kRCenterline, kH}), kLinearTolerance));
  //@}
}

TEST_F(MalidriveFlatLineLaneFullyInitializedWithInertialToBackendFrameTranslationTest, ToLanePosition) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{3.9289321881345254, 21.071067811865476, 0.5};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{74.63961030678928, 91.78174593052022, 0.5};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

// Initializes a single flat arc Lane in a single Junction - Segment environment.
class MalidriveFlatArcLaneFullyInitializedTest : public LaneTest {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::ArcGroundCurve>(kLinearTolerance, kXy0, kStartHeading, kCurvature, kArcLength, kP0,
                                                     kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));
  }

  const double kAngularTolerance{1e-6};
  const double kStartHeading{M_PI / 3.};
  const double kCurvature{-0.025};  // Equivalent radius = 40m.
  const double kArcLength{100.};

  const double kSStart{0.};
  const double kSHalf{62.5};
  const double kSEnd{125.};
  const double kRCenterline{0.};
  const double kRLeft{1.};
  const double kRRight{-2.};
  const double kH{0};
  const Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  std::unique_ptr<RoadGeometry> road_geometry_;
  const Lane* dut_{};
};

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, Length) { EXPECT_NEAR(dut_->length(), kSEnd, kLinearTolerance); }

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, TrackSFromLaneS) {
  EXPECT_NEAR(kP0, dut_->TrackSFromLaneS(kSStart), kLinearTolerance);
  EXPECT_NEAR((kP1 + kP0) / 2., dut_->TrackSFromLaneS(kSHalf), kLinearTolerance);
  EXPECT_NEAR(kP1, dut_->TrackSFromLaneS(kSEnd), kLinearTolerance);
}

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, LaneSFromTrackS) {
  EXPECT_NEAR(kSStart, dut_->LaneSFromTrackS(kP0), kLinearTolerance);
  EXPECT_NEAR(kSHalf, dut_->LaneSFromTrackS((kP1 + kP0) / 2.), kLinearTolerance);
  EXPECT_NEAR(kSEnd, dut_->LaneSFromTrackS(kP1), kLinearTolerance);
}

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, Bounds) {
  // At the beginning of the lane.
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->lane_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->segment_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(0., 0.), kLinearTolerance));
  // At the end of the lane.
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->lane_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->segment_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(dut_->length(), 0.), kLinearTolerance));
}

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, ToInertialPosition) {
  // At centerline.
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(1.3397459621556127, 17.0, 0.),
                                      dut_->ToInertialPosition({kSStart, kRCenterline, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(54.711772824485934, 40.97529846801386, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRCenterline, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(94.29335591114437, -2.113986376104993, 0.),
                                      dut_->ToInertialPosition({kSEnd, kRCenterline, kH}), kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(0.47372055837117344, 17.5, 0.),
                                      dut_->ToInertialPosition({kSStart, kRLeft, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(54.913187957948104, 41.95480443737414, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRLeft, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(95.28640270633971, -1.9962661036270921, 0.),
                                      dut_->ToInertialPosition({kSEnd, kRLeft, kH}), kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(3.0717967697244903, 16.0, 0.),
                                      dut_->ToInertialPosition({kSStart, kRRight, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(54.3089425575616, 39.01628652929331, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRRight, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(92.3072623207537, -2.349426921060793, 0.),
                                      dut_->ToInertialPosition({kSEnd, kRRight, kH}), kLinearTolerance));
  //@}
}

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, ToLanePosition) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{1.3397459621556127, 17.0, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{54.711772824485934, 40.97529846801386, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{94.29335591114437, -2.113986376104993, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the left
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{0.47372055837117344, 17.5, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{54.913187957948104, 41.95480443737414, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{95.28640270633971, -1.9962661036270921, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the right
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRRight, kH};
  expected_result.nearest_position = InertialPosition{3.0717967697244903, 16.0, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRRight, kH};
  expected_result.nearest_position = InertialPosition{54.3089425575616, 39.01628652929331, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRRight, kH};
  expected_result.nearest_position = InertialPosition{92.3072623207537, -2.349426921060793, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, GetOrientation) {
  const Rotation kExpectedRotationSStart = Rotation::FromRpy(/* roll */ 0., /* pitch */ 0., kStartHeading);
  const Rotation kExpectedRotationSHalf =
      Rotation::FromRpy(/* roll */ 0., /* pitch */ 0., kStartHeading + kArcLength * kCurvature * 0.5);
  const Rotation kExpectedRotationSEnd =
      Rotation::FromRpy(/* roll */ 0., /* pitch */ 0., kStartHeading + kArcLength * kCurvature);

  // At centerline.
  //@{
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSStart, dut_->GetOrientation({kSStart, kRCenterline, kH}), kAngularTolerance));
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSHalf, dut_->GetOrientation({kSHalf, kRCenterline, kH}), kAngularTolerance));
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSEnd, dut_->GetOrientation({kSEnd, kRCenterline, kH}), kAngularTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSStart, dut_->GetOrientation({kSStart, kRLeft, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSHalf, dut_->GetOrientation({kSHalf, kRLeft, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSEnd, dut_->GetOrientation({kSEnd, kRLeft, kH}), kAngularTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSStart, dut_->GetOrientation({kSStart, kRRight, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSHalf, dut_->GetOrientation({kSHalf, kRRight, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSEnd, dut_->GetOrientation({kSEnd, kRRight, kH}), kAngularTolerance));
  //@}
}

TEST_F(MalidriveFlatArcLaneFullyInitializedTest, EvalMotionDerivatives) {
  const maliput::api::IsoLaneVelocity kVelocity{3., 2., 1.};
  const LanePosition kExpectedResultAtCenterline{3., 2., 1.};
  const LanePosition kExpectedResultAtLeft{2.9411764705882355, 2., 1.};
  const LanePosition kExpectedResultAtRight{3.125, 2., 1.};

  // At centerline.
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtCenterline,
                                  dut_->EvalMotionDerivatives({kSStart, kRCenterline, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtCenterline,
                                  dut_->EvalMotionDerivatives({kSHalf, kRCenterline, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtCenterline,
                                  dut_->EvalMotionDerivatives({kSEnd, kRCenterline, kH}, kVelocity), kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtLeft, dut_->EvalMotionDerivatives({kSStart, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtLeft, dut_->EvalMotionDerivatives({kSHalf, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtLeft, dut_->EvalMotionDerivatives({kSEnd, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtRight,
                                  dut_->EvalMotionDerivatives({kSStart, kRRight, kH}, kVelocity), kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtRight, dut_->EvalMotionDerivatives({kSHalf, kRRight, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtRight, dut_->EvalMotionDerivatives({kSEnd, kRRight, kH}, kVelocity),
                                  kLinearTolerance));
  //@}
}

// Initializes a Lane describing a flat 'S' shape curve in a single Xodr Road / Segment environment.
// The Lane's RoadCurve is composed by a PiecewiseGroundCurve consisting of three different GroundCurves.
class MalidriveFlatSLaneFullyInitializedTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    std::vector<std::unique_ptr<road_curve::GroundCurve>> ground_curves;
    ground_curves.push_back(std::make_unique<road_curve::ArcGroundCurve>(kLinearTolerance, kXY0A, kAStartHeading,
                                                                         kACurvature, kALength90DegLeft, kP0A, kP1A));
    ground_curves.push_back(std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXY0B, kDxyB, kP0B, kP1B));
    ground_curves.push_back(std::make_unique<road_curve::ArcGroundCurve>(kLinearTolerance, kXY0C, kCStartHeading,
                                                                         kCCurvature, kCLength90DegRight, kP0C, kP1C));
    road_curve_ =
        std::make_unique<road_curve::RoadCurve>(kLinearTolerance, kScaleLength,
                                                std::make_unique<road_curve::PiecewiseGroundCurve>(
                                                    std::move(ground_curves), kLinearTolerance, kAngularTolerance),
                                                MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
                                                MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));
  }
  const bool kAssertContiguity{true};
  // Road geometry configuration.
  // TODO(#460): Error when increasing linear tolerance.
  const double kLinearTolerance{1e-6};
  const double kAngularTolerance{1e-6};
  const double kScaleLength{1.};
  const Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  // Lane.
  const maliput::api::LaneId kId{"dut"};
  const int kXordTrack{1};
  const int kXordTrackInvalid{-1};
  const int kXodrLaneId{5};
  const maliput::api::HBounds kElevationBounds{0., 5.};
  const std::optional<double> kParserSTolerance{kLinearTolerance};
  const xodr::ParserConfiguration kParserConfiguration{kParserSTolerance};

  const double kRCenterline{0.};
  const double kRLeft{1.};
  const double kRRight{-2.};
  const double kH{0};
  const double kWidth{5.};
  const double kLaneOffset{10.};
  // First geometry.
  const double kP0A{20.};
  const double kP1A{120.};
  const Vector2 kXY0A{5., -100.};
  const double kAStartHeading = 0.;
  const double kACurvature = 1. / 50.;
  const double kALength90DegLeft = M_PI / std::abs(kACurvature);
  const double kSAStart{0.};
  const double kSAEnd{kALength90DegLeft * (1 - kLaneOffset * kACurvature)};
  // Second geometry.
  const double kP0B{10.};
  const double kP1B{20.};
  const Vector2 kXY0B{5., 0.};
  const Vector2 kDxyB{-10. * Vector2::UnitX()};
  const double kSBStart{kSAEnd};
  const double kSBEnd{kSBStart + kDxyB.norm()};
  // Third geometry.
  const double kP0C{40.};
  const double kP1C{140.};
  const Vector2 kXY0C{-5., 0.};
  const double kCStartHeading = M_PI;
  const double kCCurvature = -1. / 50.;
  const double kCLength90DegRight = M_PI / std::abs(kCCurvature);
  const double kSCStart{kSBEnd};
  const double kSCEnd{kSCStart + kCLength90DegRight * (1 - kLaneOffset * kCCurvature)};
  // Piecewise geometry.
  const double kP0{0.};
  const double kP1{kP1A + kP1B + kP1C - kP0A - kP0B - kP0C};
  const double kSStart{kSAStart};
  const double kSEnd{kSCEnd};
  const double kSHalf{(kSEnd - kSStart) / 2};
  // SHalf doesn't match with the middle `p` value of the RoadCurve.
  const double kPForSHalf{124.014084283};

  const Lane* dut_{};
  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
};

TEST_F(MalidriveFlatSLaneFullyInitializedTest, Length) { EXPECT_NEAR(dut_->length(), kSEnd, kLinearTolerance); }

TEST_F(MalidriveFlatSLaneFullyInitializedTest, TrackSFromLaneS) {
  EXPECT_NEAR(kP0, dut_->TrackSFromLaneS(kSStart), kLinearTolerance);
  EXPECT_NEAR(kPForSHalf, dut_->TrackSFromLaneS(kSHalf), kLinearTolerance);
  EXPECT_NEAR(kP1, dut_->TrackSFromLaneS(kSEnd), kLinearTolerance);
}

TEST_F(MalidriveFlatSLaneFullyInitializedTest, LaneSFromTrackS) {
  EXPECT_NEAR(kSStart, dut_->LaneSFromTrackS(kP0), kLinearTolerance);
  EXPECT_NEAR(kSHalf, dut_->LaneSFromTrackS(kPForSHalf), kLinearTolerance);
  EXPECT_NEAR(kSEnd, dut_->LaneSFromTrackS(kP1), kLinearTolerance);
}

TEST_F(MalidriveFlatSLaneFullyInitializedTest, Bounds) {
  // At the beginning of the lane.
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->lane_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->segment_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(0., 0.), kLinearTolerance));
  // At the end of the lane.
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->lane_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose({-kWidth / 2., kWidth / 2.}, dut_->segment_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(dut_->length(), 0.), kLinearTolerance));
}

TEST_F(MalidriveFlatSLaneFullyInitializedTest, ToInertialPosition) {
  // At centerline.
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(5., -90., 0.),
                                      dut_->ToInertialPosition({kSStart, kRCenterline, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(-30.5707765633, -4.27831414064, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRCenterline, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(-5., 110., 0.),
                                      dut_->ToInertialPosition({kSEnd, kRCenterline, kH}), kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(5., -89., 0.), dut_->ToInertialPosition({kSStart, kRLeft, kH}),
                                      kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(-30.9969561727, -5.18295270965, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRLeft, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(-5., 111., 0.), dut_->ToInertialPosition({kSEnd, kRLeft, kH}),
                                      kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(5., -92., 0.), dut_->ToInertialPosition({kSStart, kRRight, kH}),
                                      kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(-29.7184173445, -2.46903700262, 0.),
                                      dut_->ToInertialPosition({kSHalf, kRRight, kH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(-5., 108., 0.), dut_->ToInertialPosition({kSEnd, kRRight, kH}),
                                      kLinearTolerance));
  //@}
}

TEST_F(MalidriveFlatSLaneFullyInitializedTest, ToLanePosition) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{5., -90., 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{-30.5707765633, -4.27831414064, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRCenterline, kH};
  expected_result.nearest_position = InertialPosition{-5., 110., 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the left
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{5., -89., 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{-30.9969561727, -5.18295270965, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRLeft, kH};
  expected_result.nearest_position = InertialPosition{-5., 111., 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the right
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRRight, kH};
  expected_result.nearest_position = InertialPosition{5., -92., 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRRight, kH};
  expected_result.nearest_position = InertialPosition{-29.7184173445, -2.46903700262, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRRight, kH};
  expected_result.nearest_position = InertialPosition{-5., 108., 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

TEST_F(MalidriveFlatSLaneFullyInitializedTest, GetOrientation) {
  const auto kArcGroundCurveC = std::make_unique<road_curve::ArcGroundCurve>(
      kLinearTolerance, kXY0C, kCStartHeading, kCCurvature, kCLength90DegRight, kP0C, kP1C);
  const double kHeadingAtSHalf = kArcGroundCurveC->Heading(kPForSHalf + kP0C - (kP1B - kP0B) - (kP1A - kP0A));
  const Rotation kExpectedRotationSStart = Rotation::FromRpy(/* roll */ 0., /* pitch */ 0., kAStartHeading);
  const Rotation kExpectedRotationSHalf = Rotation::FromRpy(/* roll */ 0., /* pitch */ 0., kHeadingAtSHalf);
  const Rotation kExpectedRotationSEnd =
      Rotation::FromRpy(/* roll */ 0., /* pitch */ 0.,
                        kAStartHeading + kALength90DegLeft * kACurvature + kCLength90DegRight * kCCurvature);

  // At centerline.
  //@{
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSStart, dut_->GetOrientation({kSStart, kRCenterline, kH}), kAngularTolerance));
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSHalf, dut_->GetOrientation({kSHalf, kRCenterline, kH}), kAngularTolerance));
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSEnd, dut_->GetOrientation({kSEnd, kRCenterline, kH}), kAngularTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSStart, dut_->GetOrientation({kSStart, kRLeft, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSHalf, dut_->GetOrientation({kSHalf, kRLeft, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSEnd, dut_->GetOrientation({kSEnd, kRLeft, kH}), kAngularTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(
      IsRotationClose(kExpectedRotationSStart, dut_->GetOrientation({kSStart, kRRight, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSHalf, dut_->GetOrientation({kSHalf, kRRight, kH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationSEnd, dut_->GetOrientation({kSEnd, kRRight, kH}), kAngularTolerance));
  //@}
}

TEST_F(MalidriveFlatSLaneFullyInitializedTest, EvalMotionDerivatives) {
  const maliput::api::IsoLaneVelocity kVelocity{3., 2., 1.};
  const LanePosition kExpectedResultAtCenterline{3., 2., 1.};
  const LanePosition kAExpectedResultAtRight{2.857143, 2., 1.};
  const LanePosition kAExpectedResultAtLeft{3.076923, 2., 1.};
  const LanePosition kCExpectedResultAtRight{3.103448, 2., 1.};
  const LanePosition kCExpectedResultAtLeft{2.950820, 2., 1.};

  // At centerline.
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtCenterline,
                                  dut_->EvalMotionDerivatives({kSStart, kRCenterline, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtCenterline,
                                  dut_->EvalMotionDerivatives({kSHalf, kRCenterline, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResultAtCenterline,
                                  dut_->EvalMotionDerivatives({kSEnd, kRCenterline, kH}, kVelocity), kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsLanePositionClose(kAExpectedResultAtLeft, dut_->EvalMotionDerivatives({kSStart, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kCExpectedResultAtLeft, dut_->EvalMotionDerivatives({kSHalf, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kCExpectedResultAtLeft, dut_->EvalMotionDerivatives({kSEnd, kRLeft, kH}, kVelocity),
                                  kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsLanePositionClose(kAExpectedResultAtRight,
                                  dut_->EvalMotionDerivatives({kSStart, kRRight, kH}, kVelocity), kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kCExpectedResultAtRight,
                                  dut_->EvalMotionDerivatives({kSHalf, kRRight, kH}, kVelocity), kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kCExpectedResultAtRight, dut_->EvalMotionDerivatives({kSEnd, kRRight, kH}, kVelocity),
                                  kLinearTolerance));
  //@}
}

// Hold elevation values and expected values.
struct ElevationValues {
  // Elevation Cubic polynomial. @f$ F(p) = a p^3 + b p^2 + c p + d @f$.
  // @{
  double a{0.};
  double b{0.};
  double c{0.};
  double d{0.};
  // @}
  double expected_z_start{0.};
  double expected_z_half{0.};
  double expected_z_end{0.};
  double expected_s_start{0.};
  double expected_s_half{0.};
  double expected_s_end{0.};
  double expected_pitch_start{0.};
  double expected_pitch_half{0.};
  double expected_pitch_end{0.};
};

// Returns a vector containing expected values for different elevation.
std::vector<ElevationValues> InstantiateElevationParameters() {
  return {
      // Single line Lane with a constant elevation.
      {0. /* a  */, 0. /* b  */, 0. /* c  */, 2.5 /* d  */, 2.5 /* expected_z_start  */, 2.5 /* expected_z_half  */,
       2.5 /* expected_z_end  */, 0. /* expected_s_start  */, 50. /* expected_s_half  */, 100. /* expected_s_end  */,
       0. /* expected_pitch_start  */, 0. /* expected_pitch_half  */, 0. /* expected_pitch_end  */},

      // Single line Lane with linear elevation.
      {0. /* a  */, 0. /* b  */, 0.5 /* c  */, 0. /* d  */, 0. /* expected_z_start  */, 25. /* expected_z_half  */,
       50. /* expected_z_end  */, 0. /* expected_s_start  */, 55.90169943749474241 /* expected_s_half  */,
       111.80339887498948482 /* expected_s_end  */, -0.4636476090008061 /* expected_pitch_start  */,
       -0.4636476090008061 /* expected_pitch_half  */, -0.4636476090008061 /* expected_pitch_end  */},

      // Single line Lane with quadratic elevation.
      {0. /* a  */, 0.01 /* b  */, 1. /* c  */, 0. /* d  */, 0. /* expected_z_start  */, 75. /* expected_z_half  */,
       200. /* expected_z_end  */, 0. /* expected_s_start  */, 90.504607019643771082 /* expected_s_half  */,
       225.24230725861417568 /* expected_s_end  */, -M_PI / 4. /* expected_pitch_start  */,
       -1.10714871779409 /* expected_pitch_half  */, -1.24904577239825 /* expected_pitch_end  */},

      // Single line Lane with cubic elevation.
      {0.01 /* a  */, 0.01 /* b  */, 1. /* c  */, 0. /* d  */, 0. /* expected_z_start  */, 1325. /* expected_z_half  */,
       10200. /* expected_z_end  */, 0. /* expected_s_start  */, 1328.7513197580697639 /* expected_s_half  */,
       10203.915076186799822 /* expected_s_end  */, -M_PI / 4. /* expected_pitch_start  */,
       -1.55781004387472 /* expected_pitch_half  */, -1.56749600874441 /* expected_pitch_end  */},
  };
}

// Initializes a single line Lane with a cubic polynomial elevation in a single Junction -
// Segment environment.
class MalidriveLineLaneWithElevationFullyInitializedTest : public ::testing::TestWithParam<ElevationValues> {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeCubicPolynomial(params_.a, params_.b, params_.c, params_.d, kP0, kP1, kLinearTolerance),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));
  }

  const maliput::api::LaneId kId{"dut"};
  const int kXordTrack{1};
  const int kXordTrackInvalid{-1};
  const int kXodrLaneId{5};
  const maliput::api::HBounds kElevationBounds{0., 5.};
  const double kP0{0.};
  const double kP1{100.};
  const double kScaleLength{1.};
  const Vector2 kXy0{10., 12.};
  const Vector2 kDXy{(kP1 - kP0) * std::sqrt(2.) / 2., (kP1 - kP0) * std::sqrt(2.) / 2.};
  const double kWidth{5.};
  const double kLaneOffset{10.};
  const std::optional<double> kUnarmedSToleranceParserTest{std::nullopt};
  const xodr::ParserConfiguration kParserConfiguration{kUnarmedSToleranceParserTest};

  const double kLinearTolerance{1e-6};
  const double kAngularTolerance{1e-6};
  const double kRCenterline{0.};
  const double kRLeft{1.};
  const double kRRight{-2.};
  const double kZeroH{0};
  const Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  const Lane* dut_{};
  const ElevationValues params_ = GetParam();

  const bool kAssertContiguity{true};
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
  std::unique_ptr<RoadGeometry> road_geometry_;
};

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, Length) {
  // The following constant has been empirically derived.
  // For the proposed set of tests, it is the smallest exponent that allows a polyline integration to match
  // within kLinearTolerance the results of drake's integrators.
  const int kKOrder{13};
  EXPECT_NEAR(dut_->length(), BruteForcePathLengthIntegral(*road_curve_, kP0, kP1, kLaneOffset, kZeroH, kKOrder),
              kLinearTolerance);
}

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, PFromS) {
  EXPECT_NEAR(kP0, dut_->TrackSFromLaneS(params_.expected_s_start), kLinearTolerance);
  EXPECT_NEAR((kP1 + kP0) / 2., dut_->TrackSFromLaneS(params_.expected_s_half), kLinearTolerance);
  EXPECT_NEAR(kP1, dut_->TrackSFromLaneS(params_.expected_s_end), kLinearTolerance);
}

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, SFromP) {
  EXPECT_NEAR(params_.expected_s_start, dut_->LaneSFromTrackS(kP0), kLinearTolerance);
  EXPECT_NEAR(params_.expected_s_half, dut_->LaneSFromTrackS((kP1 + kP0) / 2.), kLinearTolerance);
  EXPECT_NEAR(params_.expected_s_end, dut_->LaneSFromTrackS(kP1), kLinearTolerance);
}

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, Bounds) {
  const maliput::api::RBounds kExpectedRBounds{-kWidth / 2., kWidth / 2.};
  // At the beginning of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->lane_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->segment_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(0., 0.), kLinearTolerance));
  // At the end of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->lane_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->segment_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(dut_->length(), 0.), kLinearTolerance));
}

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, ToInertialPosition) {
  // At centerline.
  //@{
  EXPECT_TRUE(IsInertialPositionClose(
      InertialPosition(2.9289321881345254, 19.071067811865476, params_.expected_z_start),
      dut_->ToInertialPosition({params_.expected_s_start, kRCenterline, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(38.2842712474619, 54.426406871192846, params_.expected_z_half),
                                      dut_->ToInertialPosition({params_.expected_s_half, kRCenterline, kZeroH}),
                                      kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(73.63961030678928, 89.78174593052022, params_.expected_z_end),
                                      dut_->ToInertialPosition({params_.expected_s_end, kRCenterline, kZeroH}),
                                      kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(2.2218254069479784, 19.77817459305202, params_.expected_z_start),
                                      dut_->ToInertialPosition({params_.expected_s_start, kRLeft, kZeroH}),
                                      kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(37.577164466275356, 55.13351365237939, params_.expected_z_half),
                                      dut_->ToInertialPosition({params_.expected_s_half, kRLeft, kZeroH}),
                                      kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(72.93250352560273, 90.48885271170676, params_.expected_z_end),
                                      dut_->ToInertialPosition({params_.expected_s_end, kRLeft, kZeroH}),
                                      kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(4.34314575050762, 17.65685424949238, params_.expected_z_start),
                                      dut_->ToInertialPosition({params_.expected_s_start, kRRight, kZeroH}),
                                      kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(39.698484809834994, 53.01219330881975, params_.expected_z_half),
                                      dut_->ToInertialPosition({params_.expected_s_half, kRRight, kZeroH}),
                                      kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(75.05382386916237, 88.36753236814712, params_.expected_z_end),
                                      dut_->ToInertialPosition({params_.expected_s_end, kRRight, kZeroH}),
                                      kLinearTolerance));
  //@}
}

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, ToLanePosition) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{params_.expected_s_start, kRCenterline, kZeroH};
  expected_result.nearest_position = InertialPosition{2.9289321881345254, 19.071067811865476, params_.expected_z_start};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{params_.expected_s_half, kRCenterline, kZeroH};
  expected_result.nearest_position = InertialPosition{38.2842712474619, 54.426406871192846, params_.expected_z_half};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{params_.expected_s_end, kRCenterline, kZeroH};
  expected_result.nearest_position = InertialPosition{73.63961030678928, 89.78174593052022, params_.expected_z_end};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the left
  //@{
  expected_result.lane_position = LanePosition{params_.expected_s_start, kRLeft, kZeroH};
  expected_result.nearest_position = InertialPosition{2.2218254069479784, 19.77817459305202, params_.expected_z_start};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{params_.expected_s_half, kRLeft, kZeroH};
  expected_result.nearest_position = InertialPosition{37.577164466275356, 55.13351365237939, params_.expected_z_half};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{params_.expected_s_end, kRLeft, kZeroH};
  expected_result.nearest_position = InertialPosition{72.93250352560273, 90.48885271170676, params_.expected_z_end};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the right
  //@{
  expected_result.lane_position = LanePosition{params_.expected_s_start, kRRight, kZeroH};
  expected_result.nearest_position = InertialPosition{4.34314575050762, 17.65685424949238, params_.expected_z_start};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{params_.expected_s_half, kRRight, kZeroH};
  expected_result.nearest_position = InertialPosition{39.698484809834994, 53.01219330881975, params_.expected_z_half};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{params_.expected_s_end, kRRight, kZeroH};
  expected_result.nearest_position = InertialPosition{75.05382386916237, 88.36753236814712, params_.expected_z_end};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, GetOrientation) {
  const Rotation kExpectedRotationStart =
      Rotation::FromRpy(/* roll */ 0., /* pitch */ params_.expected_pitch_start, M_PI / 4.);
  const Rotation kExpectedRotationHalf =
      Rotation::FromRpy(/* roll */ 0., /* pitch */ params_.expected_pitch_half, M_PI / 4.);
  const Rotation kExpectedRotationEnd =
      Rotation::FromRpy(/* roll */ 0., /* pitch */ params_.expected_pitch_end, M_PI / 4.);

  // At centerline.
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotationStart,
                              dut_->GetOrientation({params_.expected_s_start, kRCenterline, kZeroH}),
                              kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(
      kExpectedRotationHalf, dut_->GetOrientation({params_.expected_s_half, kRCenterline, kZeroH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationEnd,
                              dut_->GetOrientation({params_.expected_s_end, kRCenterline, kZeroH}), kAngularTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotationStart, dut_->GetOrientation({params_.expected_s_start, kRLeft, kZeroH}),
                              kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationHalf, dut_->GetOrientation({params_.expected_s_half, kRLeft, kZeroH}),
                              kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationEnd, dut_->GetOrientation({params_.expected_s_end, kRLeft, kZeroH}),
                              kAngularTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsRotationClose(kExpectedRotationStart, dut_->GetOrientation({params_.expected_s_start, kRRight, kZeroH}),
                              kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationHalf, dut_->GetOrientation({params_.expected_s_half, kRRight, kZeroH}),
                              kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(kExpectedRotationEnd, dut_->GetOrientation({params_.expected_s_end, kRRight, kZeroH}),
                              kAngularTolerance));
  //@}
}

TEST_P(MalidriveLineLaneWithElevationFullyInitializedTest, EvalMotionDerivatives) {
  const maliput::api::IsoLaneVelocity kVelocity{3., 2., 1.};
  const LanePosition kExpectedResult{3., 2., 1.};

  // At centerline.
  //@{
  EXPECT_TRUE(IsLanePositionClose(
      kExpectedResult, dut_->EvalMotionDerivatives({params_.expected_s_start, kRCenterline, kZeroH}, kVelocity),
      kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(
      kExpectedResult, dut_->EvalMotionDerivatives({params_.expected_s_half, kRCenterline, kZeroH}, kVelocity),
      kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(
      kExpectedResult, dut_->EvalMotionDerivatives({params_.expected_s_end, kRCenterline, kZeroH}, kVelocity),
      kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult,
                                  dut_->EvalMotionDerivatives({params_.expected_s_start, kRLeft, kZeroH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult,
                                  dut_->EvalMotionDerivatives({params_.expected_s_half, kRLeft, kZeroH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult,
                                  dut_->EvalMotionDerivatives({params_.expected_s_end, kRLeft, kZeroH}, kVelocity),
                                  kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult,
                                  dut_->EvalMotionDerivatives({params_.expected_s_start, kRRight, kZeroH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult,
                                  dut_->EvalMotionDerivatives({params_.expected_s_half, kRRight, kZeroH}, kVelocity),
                                  kLinearTolerance));
  EXPECT_TRUE(IsLanePositionClose(kExpectedResult,
                                  dut_->EvalMotionDerivatives({params_.expected_s_end, kRRight, kZeroH}, kVelocity),
                                  kLinearTolerance));
  //@}
}

INSTANTIATE_TEST_CASE_P(MaldriveLaneSingleLaneWithElevationGroup, MalidriveLineLaneWithElevationFullyInitializedTest,
                        ::testing::ValuesIn(InstantiateElevationParameters()));

struct ArcElevationValues {
  // Elevation Cubic polynomial. @f$ F(p) = a p^3 + b p^2 + c p + d @f$.
  // @{
  double a{0.};
  double b{0.};
  double c{0.};
  double d{0.};
  // @}
  double expected_z_start{0.};
  double expected_z_half{0.};
  double expected_z_end{0.};
  double expected_pitch_start{0.};
  double expected_pitch_half{0.};
  double expected_pitch_end{0.};
};

// Returns a vector containing expected values for different elevation.
std::vector<ArcElevationValues> InstantiateElevationParametersForArc() {
  return {
      // Single ArcLane with a constant elevation.
      {0. /* a  */, 0. /* b  */, 0. /* c  */, 2.5 /* d  */, 2.5 /* expected_z_start  */, 2.5 /* expected_z_half  */,
       2.5 /* expected_z_end  */, 0. /* expected_pitch_start  */, 0. /* expected_pitch_half  */,
       0. /* expected_pitch_end  */},

      // Single ArcLane with linear elevation.
      {0. /* a  */, 0. /* b  */, 0.5 /* c  */, 0. /* d  */, 0. /* expected_z_start  */, 25. /* expected_z_half
                                                                                             */
       ,
       50. /* expected_z_end  */, -0.4636476090008061 /* expected_pitch_start  */,
       -0.4636476090008061 /* expected_pitch_half  */, -0.4636476090008061 /* expected_pitch_end  */},

      // Single ArcLane with quadratic elevation.
      {0. /* a  */, 0.01 /* b  */, 1. /* c  */, 0. /* d  */, 0. /* expected_z_start  */, 75. /* expected_z_half
                                                                                              */
       ,
       200. /* expected_z_end  */, -M_PI / 4. /* expected_pitch_start  */, -1.10714871779409 /* expected_pitch_half  */,
       -1.24904577239825 /* expected_pitch_end  */},

      // Single ArcLane with cubic elevation.
      {0.01 /* a  */, 0.01 /* b  */, 1. /* c  */, 0. /* d  */, 0. /* expected_z_start  */, 1325. /* expected_z_half  */,
       10200. /* expected_z_end  */, -M_PI / 4. /* expected_pitch_start  */,
       -1.55781004387472 /* expected_pitch_half  */, -1.56749600874441 /* expected_pitch_end  */},
  };
}

// Initializes a single line Lane with a cubic polynomial elevation in a single Junction -
// Segment environment.
class MalidriveArcLaneWithElevationFullyInitializedTest : public ::testing::TestWithParam<ArcElevationValues> {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::ArcGroundCurve>(kLinearTolerance, kXy0, kStartHeading, kCurvature, kArcLength, kP0,
                                                     kP1),
        MakeCubicPolynomial(params_.a, params_.b, params_.c, params_.d, kP0, kP1, kLinearTolerance),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));

    kSStart = BruteForcePathLengthIntegral(*road_curve_, kP0, kP0, kLaneOffset, kZeroH, kKOrder);
    kSHalf = BruteForcePathLengthIntegral(*road_curve_, kP0, (kP0 + kP1) / 2, kLaneOffset, kZeroH, kKOrder);
    kSEnd = BruteForcePathLengthIntegral(*road_curve_, kP0, kP1, kLaneOffset, kZeroH, kKOrder);
  }
  const maliput::api::LaneId kId{"dut"};
  const int kXordTrack{1};
  const int kXordTrackInvalid{-1};
  const int kXodrLaneId{5};
  const maliput::api::HBounds kElevationBounds{0., 5.};
  const double kStartHeading{M_PI / 3.};
  const double kCurvature{-0.025};  // Equivalent radius = 40m.
  const double kArcLength{100.};
  const double kP0{0.};
  const double kP1{100.};
  const double kScaleLength{1.};
  const Vector2 kXy0{10., 12.};
  const double kWidth{5.};
  const double kLaneOffset{10.};
  const std::optional<double> kUnarmedSToleranceParserTest{std::nullopt};
  const xodr::ParserConfiguration kParserConfiguration{kUnarmedSToleranceParserTest};

  const double kLinearTolerance{1e-6};
  const double kAngularTolerance{1e-6};
  const double kRCenterline{0.};
  const double kRLeft{1.};
  const double kRRight{-2.};
  const double kZeroH{0};
  const Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};

  const bool kAssertContiguity{true};
  const ArcElevationValues params_ = GetParam();
  // The following constant has been empirically derived.
  // For the proposed set of tests, it is the smallest exponent that allows a polyline integration to match
  // within kLinearTolerance the results of drake's integrators.
  const int kKOrder{13};

  double kSStart{};
  double kSHalf{};
  double kSEnd{};

  const Lane* dut_{};
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
  std::unique_ptr<RoadGeometry> road_geometry_;
};

TEST_P(MalidriveArcLaneWithElevationFullyInitializedTest, Length) {
  EXPECT_NEAR(dut_->length(), kSEnd - kSStart, kLinearTolerance);
}

TEST_P(MalidriveArcLaneWithElevationFullyInitializedTest, PFromS) {
  EXPECT_NEAR(kP0, dut_->TrackSFromLaneS(kSStart), kLinearTolerance);
  EXPECT_NEAR((kP1 + kP0) / 2., dut_->TrackSFromLaneS(kSHalf), kLinearTolerance);
  EXPECT_NEAR(kP1, dut_->TrackSFromLaneS(kSEnd), kLinearTolerance);
}

TEST_P(MalidriveArcLaneWithElevationFullyInitializedTest, SFromP) {
  EXPECT_NEAR(kSStart, dut_->LaneSFromTrackS(kP0), kLinearTolerance);
  EXPECT_NEAR(kSHalf, dut_->LaneSFromTrackS((kP1 + kP0) / 2.), kLinearTolerance);
  EXPECT_NEAR(kSEnd, dut_->LaneSFromTrackS(kP1), kLinearTolerance);
}

TEST_P(MalidriveArcLaneWithElevationFullyInitializedTest, Bounds) {
  const maliput::api::RBounds kExpectedRBounds{-kWidth / 2., kWidth / 2.};
  // At the beginning of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->lane_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->segment_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(0., 0.), kLinearTolerance));
  // At the end of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->lane_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->segment_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(dut_->length(), 0.), kLinearTolerance));
}

TEST_P(MalidriveArcLaneWithElevationFullyInitializedTest, ToInertialPosition) {
  // At centerline.
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(1.3397459621556127, 17.0, params_.expected_z_start),
                                      dut_->ToInertialPosition({kSStart, kRCenterline, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(54.711772824485934, 40.97529846801386, params_.expected_z_half),
                                      dut_->ToInertialPosition({kSHalf, kRCenterline, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(94.29335591114437, -2.113986376104993, params_.expected_z_end),
                                      dut_->ToInertialPosition({kSEnd, kRCenterline, kZeroH}), kLinearTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(0.47372055837117344, 17.5, params_.expected_z_start),
                                      dut_->ToInertialPosition({kSStart, kRLeft, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(54.913187957948104, 41.95480443737414, params_.expected_z_half),
                                      dut_->ToInertialPosition({kSHalf, kRLeft, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(95.28640270633971, -1.9962661036270921, params_.expected_z_end),
                                      dut_->ToInertialPosition({kSEnd, kRLeft, kZeroH}), kLinearTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(3.0717967697244903, 16.0, params_.expected_z_start),
                                      dut_->ToInertialPosition({kSStart, kRRight, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(54.3089425575616, 39.01628652929331, params_.expected_z_half),
                                      dut_->ToInertialPosition({kSHalf, kRRight, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(InertialPosition(92.3072623207537, -2.349426921060793, params_.expected_z_end),
                                      dut_->ToInertialPosition({kSEnd, kRRight, kZeroH}), kLinearTolerance));
  //@}
}

TEST_P(MalidriveArcLaneWithElevationFullyInitializedTest, ToLanePosition) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRCenterline, kZeroH};
  expected_result.nearest_position = InertialPosition{1.3397459621556127, 17.0, params_.expected_z_start};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRCenterline, kZeroH};
  expected_result.nearest_position = InertialPosition{54.711772824485934, 40.97529846801386, params_.expected_z_half};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRCenterline, kZeroH};
  expected_result.nearest_position = InertialPosition{94.29335591114437, -2.113986376104993, params_.expected_z_end};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the left
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRLeft, kZeroH};
  expected_result.nearest_position = InertialPosition{0.47372055837117344, 17.5, params_.expected_z_start};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRLeft, kZeroH};
  expected_result.nearest_position = InertialPosition{54.913187957948104, 41.95480443737414, params_.expected_z_half};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRLeft, kZeroH};
  expected_result.nearest_position = InertialPosition{95.28640270633971, -1.9962661036270921, params_.expected_z_end};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the right
  //@{
  expected_result.lane_position = LanePosition{kSStart, kRRight, kZeroH};
  expected_result.nearest_position = InertialPosition{3.0717967697244903, 16.0, params_.expected_z_start};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSHalf, kRRight, kZeroH};
  expected_result.nearest_position = InertialPosition{54.3089425575616, 39.01628652929331, params_.expected_z_half};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{kSEnd, kRRight, kZeroH};
  expected_result.nearest_position = InertialPosition{92.3072623207537, -2.349426921060793, params_.expected_z_end};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

// TODO(francocipollone): Add tests for GetOrientation method.
// TODO(francocipollone): Add tests for EvalMotionDerivatives method.

INSTANTIATE_TEST_CASE_P(MaldriveLaneSingleArcLaneWithElevationGroup, MalidriveArcLaneWithElevationFullyInitializedTest,
                        ::testing::ValuesIn(InstantiateElevationParametersForArc()));

// Hold superelevation values and expected values.
struct SuperelevatedLaneTestParams {
  // Superelevation Cubic polynomial.
  // @f$ F(p) = pol_coeff[0] p^3 + pol_coeff[1] p^2 + pol_coeff[2] p + pol_coeff[3] @f$.
  // @{
  std::array<double, 4> pol_coeff;
  // @}

  // Geopositions at start(0), half(1) and end(1) of a lane located in the center(0), left(1) and the right(2).
  // For example: inertial_positions[2][0] will access to the InertialPosition in the start of right lane.
  std::array<std::array<InertialPosition, 3>, 3> inertial_positions;
};

// Returns a vector containing expected values for different superelevation.
std::vector<SuperelevatedLaneTestParams> InstantiateSuperelevationParametersForLine() {
  return {
      // To get the InertialPosition given (p,r,h) for a straight superelevated lane the following python code could be
      // useful.
      // @code{python}
      //
      // import sympy as s
      // p,r,h,phi,theta,z0,y0,x0 = s.symbols('p,r,h,phi,theta,z0,y0,x0')
      // # Calculate xyz given the effect of p r and h on a road align with the x-axis.
      // x_prime = p
      // y_prime = r * s.cos(theta) + h * s.sin(theta)
      // z_prime = -r * s.sin(theta) + h * s.cos(theta)
      // # Being `theta` the value of the superelevation in that p.
      // # Now we rotate around z axis to match heading.
      // # `phi` is the angle formed by the x-axis and the heading of the road.
      // x = (x_prime) * s.cos(phi) - (y_prime) * s.sin(phi) + x0
      // y = (x_prime) * s.sin(phi) + (y_prime) * s.cos(phi) + y0
      // z = z_prime + z0
      //
      // @endcode
      // Single line Lane with a constant superelevation.
      {
          {0., 0., 0., -1.},
          {{{{{6.1794857562991025638, 15.820514243700897436,
               -8.4147098480789650665} /* InertialPositionAtCenterStart  */,
              {41.534824815626478784, 51.175853303028273656,
               -8.4147098480789650665} /* InertialPositionAtCenterHalf  */,
              {76.890163874953855004, 86.531192362355649876,
               -8.4147098480789650665}}} /* InertialPositionAtCenterEnd  */,
            {{{5.7974343319290128202, 16.202565668070987180, -9.2561808328868615732} /* InertialPositionAtLeftStart  */,
              {41.152773391256389040, 51.557904727398363400, -9.2561808328868615732} /* InertialPositionAtLeftHalf  */,
              {76.508112450583765260, 86.913243786725739620, -9.2561808328868615732}}} /* InertialPositionAtLeftEnd  */,
            {{{6.9435886050392820511, 15.056411394960717949,
               -6.7317678784631720532} /* InertialPositionAtRightStart  */,
              {42.298927664366658271, 50.411750454288094169, -6.7317678784631720532} /* InertialPositionAtRightHalf  */,
              {77.654266723694034491, 85.767089513615470389,
               -6.7317678784631720532}}}}} /* InertialPositionAtRightEnd  */,
      },
      // Single line Lane with a cubic superelevation.
      {
          {-6.04716270616596e-6, 0.000569931657984447, 0., -0.5},
          {{{{{3.7945541943625438930, 18.205445805637456107,
               -4.7942553860420300538} /* InertialPositionAtCenterStart  */,
              {38.384930812645887124, 54.325747306008865316, 1.6813142922099080234} /* InertialPositionAtCenterHalf  */,
              {76.032461232327927565, 87.388895004981577315,
               -7.4985714325445416151}}} /* InertialPositionAtCenterEnd  */,
            {{{3.1740096137987982823, 18.825990386201201718, -5.2736809246462330591} /* InertialPositionAtLeftStart  */,
              {37.687889987977732185, 55.022788130677020255, 1.8494457214308988036} /* InertialPositionAtLeftHalf  */,
              {75.564639543695244073, 87.856716693614260808, -8.2484285757989965759}}} /* InertialPositionAtLeftEnd  */,
            {{{5.0356433554900351144, 16.964356644509964886,
               -3.8354043088336240430} /* InertialPositionAtRightStart  */,
              {39.779012461982181928, 52.931665656672570512, 1.3450514337679264631} /* InertialPositionAtRightHalf  */,
              {76.968104609593294550, 86.453251627716210331,
               -5.9988571460356334697}}}}} /* InertialPositionAtRightEnd  */,
      },
  };
}

// Initializes a single line Lane with a cubic polynomial superelevation in a single Junction -
// Segment environment.
class MalidriveLineLaneWithSuperelevationFullyInitializedTest
    : public ::testing::TestWithParam<SuperelevatedLaneTestParams> {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    const auto superelevation_coefficients{GetParam().pol_coeff};
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        MakeCubicPolynomial(superelevation_coefficients[0], superelevation_coefficients[1],
                            superelevation_coefficients[2], superelevation_coefficients[3], kP0, kP1, kLinearTolerance),
        kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance),
                                       MakeConstantCubicPolynomial(kLaneOffset, kP0, kP1, kLinearTolerance), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));
    superelevation_ =
        MakeCubicPolynomial(superelevation_coefficients[0], superelevation_coefficients[1],
                            superelevation_coefficients[2], superelevation_coefficients[3], kP0, kP1, kLinearTolerance);

    s_start = BruteForcePathLengthIntegral(*road_curve_, kP0, kP0, kLaneOffset, kZeroH, kKOrder);
    s_half = BruteForcePathLengthIntegral(*road_curve_, kP0, (kP0 + kP1) / 2, kLaneOffset, kZeroH, kKOrder);
    s_end = BruteForcePathLengthIntegral(*road_curve_, kP0, kP1, kLaneOffset, kZeroH, kKOrder);
  }

  const maliput::api::LaneId kId{"dut"};
  const int kXordTrack{1};
  const int kXordTrackInvalid{-1};
  const int kXodrLaneId{5};
  const maliput::api::HBounds kElevationBounds{0., 5.};
  const double kP0{0.};
  const double kPHalf{50.};
  const double kP1{100.};
  const double kScaleLength{1.};
  const Vector2 kXy0{10., 12.};
  const Vector2 kDXy{(kP1 - kP0) * std::sqrt(2.) / 2., (kP1 - kP0) * std::sqrt(2.) / 2.};
  const double kWidth{5.};
  const double kLaneOffset{10.};
  const std::optional<double> kUnarmedSToleranceParserTest{std::nullopt};
  const xodr::ParserConfiguration kParserConfiguration{kUnarmedSToleranceParserTest};

  const double kLinearTolerance{1e-6};
  const double kAngularTolerance{1e-6};
  const double kRCenterline{0.};
  const double kRLeft{1.};
  const double kRRight{-2.};
  const double kZeroH{0};
  const Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  const Lane* dut_{};
  const std::array<std::array<InertialPosition, 3>, 3> kExpectedInertialPositions = GetParam().inertial_positions;
  const int AtCenterLine = 0;
  const int AtLeft = 1;
  const int AtRight = 2;
  const int AtStart = 0;
  const int AtHalf = 1;
  const int AtEnd = 2;
  // The following constant has been empirically derived.
  // For the proposed set of tests, it is the smallest exponent that allows a polyline integration to match
  // within kLinearTolerance the results of drake's integrators.
  const int kKOrder{13};

  const bool kAssertContiguity{true};
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
  std::unique_ptr<road_curve::RoadCurveOffset> road_curve_offset_;
  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<road_curve::Function> superelevation_;

  double s_start{};
  double s_half{};
  double s_end{};
};

TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, Length) {
  EXPECT_NEAR(dut_->length(), s_end - s_start, kLinearTolerance);
}

TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, PFromS) {
  EXPECT_NEAR(kP0, dut_->TrackSFromLaneS(s_start), kLinearTolerance);
  EXPECT_NEAR((kP1 + kP0) / 2., dut_->TrackSFromLaneS(s_half), kLinearTolerance);
  EXPECT_NEAR(kP1, dut_->TrackSFromLaneS(s_end), kLinearTolerance);
}

TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, SFromP) {
  EXPECT_NEAR(s_start, dut_->LaneSFromTrackS(kP0), kLinearTolerance);
  EXPECT_NEAR(s_half, dut_->LaneSFromTrackS((kP1 + kP0) / 2.), kLinearTolerance);
  EXPECT_NEAR(s_end, dut_->LaneSFromTrackS(kP1), kLinearTolerance);
}

TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, Bounds) {
  const maliput::api::RBounds kExpectedRBounds{-kWidth / 2., kWidth / 2.};
  // At the beginning of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->lane_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->segment_bounds(0.), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(0., 0.), kLinearTolerance));
  // At the end of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->lane_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBounds, dut_->segment_bounds(dut_->length()), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(dut_->length(), 0.), kLinearTolerance));
}

TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, ToInertialPosition) {
  // At centerline.
  // @{
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtCenterLine][AtStart],
                                      dut_->ToInertialPosition({s_start, kRCenterline, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtCenterLine][AtHalf],
                                      dut_->ToInertialPosition({s_half, kRCenterline, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtCenterLine][AtEnd],
                                      dut_->ToInertialPosition({s_end, kRCenterline, kZeroH}), kLinearTolerance));
  // @}

  // To the left
  // @{
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtLeft][AtStart],
                                      dut_->ToInertialPosition({s_start, kRLeft, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtLeft][AtHalf],
                                      dut_->ToInertialPosition({s_half, kRLeft, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtLeft][AtEnd],
                                      dut_->ToInertialPosition({s_end, kRLeft, kZeroH}), kLinearTolerance));
  // @}

  // To the right
  // @{
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtRight][AtStart],
                                      dut_->ToInertialPosition({s_start, kRRight, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtRight][AtHalf],
                                      dut_->ToInertialPosition({s_half, kRRight, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose(kExpectedInertialPositions[AtRight][AtEnd],
                                      dut_->ToInertialPosition({s_end, kRRight, kZeroH}), kLinearTolerance));
  // @}
}

TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, ToLanePosition) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{s_start, kRCenterline, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtCenterLine][AtStart];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_half, kRCenterline, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtCenterLine][AtHalf];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_end, kRCenterline, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtCenterLine][AtEnd];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the left
  //@{
  expected_result.lane_position = LanePosition{s_start, kRLeft, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtLeft][AtStart];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_half, kRLeft, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtLeft][AtHalf];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_end, kRLeft, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtLeft][AtEnd];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the right
  //@{
  expected_result.lane_position = LanePosition{s_start, kRRight, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtRight][AtStart];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_half, kRRight, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtRight][AtHalf];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_end, kRRight, kZeroH};
  expected_result.nearest_position = kExpectedInertialPositions[AtRight][AtEnd];
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

// TODO(malidrive#536): Improve these tests by computing with another method the orientation.
TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, GetOrientation) {
  // Functor for getting {Roll, Pitch, Yaw} values of a RoadCurve in a particular {p,r,h} coordinate.
  // The following conditions were considered:
  //  - Elevation function is zero.
  //  - GroundCurve is a straight line with a heading of PI/4.
  auto get_rpy = [](const Vector3& prh, const road_curve::Function* superelevation) {
    const maliput::math::RollPitchYaw r_rpy(superelevation->f(prh.x()), 0, M_PI / 4);
    const maliput::math::Matrix3 r_rpy_dot = r_rpy.CalcRotationMatrixDt({superelevation->f_dot(prh.x()), 0, 0});
    const Vector3 s_hat =
        (r_rpy_dot * Vector3(0, prh.y(), prh.z()) + Vector3(sqrt(2) / 2, sqrt(2) / 2, 0.)).normalized();
    const Vector3 z_hat = Vector3::UnitZ();
    const Vector3 h_hat_0 = (z_hat - z_hat.dot(s_hat) * s_hat).normalized();
    const Vector3 h_hat = Quaternion(superelevation->f(prh.x()), s_hat) * h_hat_0;
    const Vector3 r_hat = h_hat.cross(s_hat);

    const double gamma = std::atan2(s_hat.y(), s_hat.x());
    const double beta = std::atan2(-s_hat.z(), maliput::math::Vector2(s_hat.x(), s_hat.y()).norm());
    const double cb = std::cos(beta);
    const double alpha = std::atan2(r_hat.z() / cb, ((r_hat.y() * s_hat.x()) - (r_hat.x() * s_hat.y())) / cb);
    return Vector3{alpha, beta, gamma};
  };

  // At centerline.
  //@{
  EXPECT_TRUE(
      IsRotationClose(Rotation::FromRpy(get_rpy({kP0, kRCenterline + kLaneOffset, kZeroH}, superelevation_.get())),
                      dut_->GetOrientation({s_start, kRCenterline, kZeroH}), kAngularTolerance));
  EXPECT_TRUE(
      IsRotationClose(Rotation::FromRpy(get_rpy({kPHalf, kRCenterline + kLaneOffset, kZeroH}, superelevation_.get())),
                      dut_->GetOrientation({s_half, kRCenterline, kZeroH}), kAngularTolerance));
  EXPECT_TRUE(
      IsRotationClose(Rotation::FromRpy(get_rpy({kP1, kRCenterline + kLaneOffset, kZeroH}, superelevation_.get())),
                      dut_->GetOrientation({s_end, kRCenterline, kZeroH}), kAngularTolerance));
  //@}

  // To the left
  //@{
  EXPECT_TRUE(IsRotationClose(Rotation::FromRpy(get_rpy({kP0, kRLeft + kLaneOffset, kZeroH}, superelevation_.get())),
                              dut_->GetOrientation({s_start, kRLeft, kZeroH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(Rotation::FromRpy(get_rpy({kPHalf, kRLeft + kLaneOffset, kZeroH}, superelevation_.get())),
                              dut_->GetOrientation({s_half, kRLeft, kZeroH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(Rotation::FromRpy(get_rpy({kP1, kRLeft + kLaneOffset, kZeroH}, superelevation_.get())),
                              dut_->GetOrientation({s_end, kRLeft, kZeroH}), kAngularTolerance));
  //@}

  // To the right
  //@{
  EXPECT_TRUE(IsRotationClose(Rotation::FromRpy(get_rpy({kP0, kRRight + kLaneOffset, kZeroH}, superelevation_.get())),
                              dut_->GetOrientation({s_start, kRRight, kZeroH}), kAngularTolerance));
  EXPECT_TRUE(
      IsRotationClose(Rotation::FromRpy(get_rpy({kPHalf, kRRight + kLaneOffset, kZeroH}, superelevation_.get())),
                      dut_->GetOrientation({s_half, kRRight, kZeroH}), kAngularTolerance));
  EXPECT_TRUE(IsRotationClose(Rotation::FromRpy(get_rpy({kP1, kRRight + kLaneOffset, kZeroH}, superelevation_.get())),
                              dut_->GetOrientation({s_end, kRRight, kZeroH}), kAngularTolerance));
  //@}
}

// TODO(malidrive#536): Improve these tests by computing with another method
//                      the tangent ratios that scale the velocity.
TEST_P(MalidriveLineLaneWithSuperelevationFullyInitializedTest, EvalMotionDerivatives) {
  const maliput::api::IsoLaneVelocity kVelocity{3., 2., 1.};

  // Get the scale factor with which `s` component is scaled when you move away from the center line. (assuming h=0).
  auto get_scale_factor = [road_curve = road_curve_.get(), lane_offset = kLaneOffset](const Vector3& prh) {
    return road_curve->WDot({prh.x(), lane_offset, prh.z()}).norm() /
           road_curve->WDot({prh.x(), prh.y(), prh.z()}).norm();
  };

  // At centerline.
  //@{
  double scale_factor = get_scale_factor(Vector3{kP0, kRCenterline + kLaneOffset, kZeroH});
  LanePosition expected_result{3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(
      expected_result, dut_->EvalMotionDerivatives({s_start, kRCenterline, kZeroH}, kVelocity), kLinearTolerance));
  scale_factor = get_scale_factor(Vector3{kPHalf, kRCenterline + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(
      expected_result, dut_->EvalMotionDerivatives({s_half, kRCenterline, kZeroH}, kVelocity), kLinearTolerance));
  scale_factor = get_scale_factor(Vector3{kP1, kRCenterline + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(
      expected_result, dut_->EvalMotionDerivatives({s_end, kRCenterline, kZeroH}, kVelocity), kLinearTolerance));
  //@}

  // To the left
  //@{
  scale_factor = get_scale_factor(Vector3{kP0, kRLeft + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(expected_result, dut_->EvalMotionDerivatives({s_start, kRLeft, kZeroH}, kVelocity),
                                  kLinearTolerance));
  scale_factor = get_scale_factor(Vector3{kPHalf, kRLeft + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(expected_result, dut_->EvalMotionDerivatives({s_half, kRLeft, kZeroH}, kVelocity),
                                  kLinearTolerance));
  scale_factor = get_scale_factor(Vector3{kP1, kRLeft + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(expected_result, dut_->EvalMotionDerivatives({s_end, kRLeft, kZeroH}, kVelocity),
                                  kLinearTolerance));
  //@}

  // To the right
  //@{
  scale_factor = get_scale_factor(Vector3{kP0, kRRight + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(expected_result, dut_->EvalMotionDerivatives({s_start, kRRight, kZeroH}, kVelocity),
                                  kLinearTolerance));
  scale_factor = get_scale_factor(Vector3{kPHalf, kRRight + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(expected_result, dut_->EvalMotionDerivatives({s_half, kRRight, kZeroH}, kVelocity),
                                  kLinearTolerance));
  scale_factor = get_scale_factor(Vector3{kP1, kRRight + kLaneOffset, kZeroH});
  expected_result = {3. * scale_factor, 2., 1.};
  EXPECT_TRUE(IsLanePositionClose(expected_result, dut_->EvalMotionDerivatives({s_end, kRRight, kZeroH}, kVelocity),
                                  kLinearTolerance));
  //@}
}

INSTANTIATE_TEST_CASE_P(MaldriveLaneSingleLaneWithSuperelevationGroup,
                        MalidriveLineLaneWithSuperelevationFullyInitializedTest,
                        ::testing::ValuesIn(InstantiateSuperelevationParametersForLine()));

// Initializes a single flat line variable width Lane in a single Junction - Segment environment.
class MalidriveFlatLineVariableWidthLaneFullyInitializedTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto manager = xodr::LoadDataBaseFromStr(kXODRHeaderTemplate, kParserConfiguration);
    road_geometry_ =
        std::make_unique<RoadGeometry>(maliput::api::RoadGeometryId("sample_rg"), std::move(manager), kLinearTolerance,
                                       kAngularTolerance, kScaleLength, kInertialToBackendFrameTranslation);
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        kAssertContiguity);

    auto lane_width = MakeCubicPolynomial(kLaneWidthCoeff[0], kLaneWidthCoeff[1], kLaneWidthCoeff[2],
                                          kLaneWidthCoeff[3], kP0, kP1, kLinearTolerance);
    lane_width_ = lane_width.get();
    adjacent_lane_width_ = MakeConstantCubicPolynomial(kWidth, kP0, kP1, kLinearTolerance);
    adjacent_lane_offset_ = MakeConstantCubicPolynomial(kWidth / 2., kP0, kP1, kLinearTolerance);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    const road_curve::RoadCurve* road_curve_ptr = road_curve_.get();
    const road_curve::Function* reference_line_offset_ptr = reference_line_offset_.get();
    auto junction = std::make_unique<Junction>(maliput::api::JunctionId{"dut"});
    auto segment =
        std::make_unique<Segment>(maliput::api::SegmentId{"dut"}, road_curve_ptr, reference_line_offset_ptr, kP0, kP1);
    auto lane_offset = std::make_unique<road_curve::LaneOffset>(
        std::make_optional<>(
            road_curve::LaneOffset::AdjacentLaneFunctions{adjacent_lane_offset_.get(), adjacent_lane_width_.get()}),
        lane_width.get(), reference_line_offset_ptr, road_curve::LaneOffset::kAtLeftFromCenterLane, kP0, kP1,
        kLinearTolerance);
    auto lane = std::make_unique<Lane>(kId, kXordTrack, kXodrLaneId, kElevationBounds, road_curve_ptr,
                                       std::move(lane_width), std::move(lane_offset), kP0, kP1);
    constexpr bool kNotHideLane{false};
    dut_ = segment->AddLane(std::move(lane), kNotHideLane);
    junction->AddSegment(std::move(segment));
    road_geometry_->AddJunction(std::move(junction));
    s_half = dut_->length() / 2.;
    s_end = dut_->length();
  }
  const double kLaneWidthCoeff[4]{4.0e-6, -6.0e-4, 0., 2.};
  const double kAngularTolerance{1e-6};
  const double kScaleLength{1.};
  const double kLinearTolerance{1e-9};
  const std::optional<double> kUnarmedSToleranceParserTest{std::nullopt};
  const xodr::ParserConfiguration kParserConfiguration{kUnarmedSToleranceParserTest};

  const maliput::api::LaneId kId{"dut"};
  const int kXordTrack{1};
  const int kXordTrackInvalid{-1};
  const int kXodrLaneId{5};
  const maliput::api::HBounds kElevationBounds{0., 5.};
  const double kP0{0.};
  const double kPHalf{50.};
  const double kP1{100.};
  const Vector2 kXy0{1., 1.};
  const Vector2 kDXy{(kP1 - kP0) * std::sqrt(2.) / 2., (kP1 - kP0) * std::sqrt(2.) / 2.};
  const double kWidth{2.};
  const double kRCenterline{0.};
  const double kRLeft{0.5};
  const double kRRight{-0.5};
  const double kZeroH{0};
  const Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};
  const Lane* dut_{};
  const road_curve::Function* lane_width_{};

  const bool kAssertContiguity{true};
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<road_curve::Function> adjacent_lane_offset_;
  std::unique_ptr<road_curve::Function> adjacent_lane_width_;

  double s_start{};
  double s_half{};
  double s_end{};
};

// The values obtained in Length, PFromS and SFromP methods are not well calculated yet.
// However the relation between the length and PFromS or SFromP is correct.
// @{
TEST_F(MalidriveFlatLineVariableWidthLaneFullyInitializedTest, Length) {
  EXPECT_NEAR(dut_->length(), s_end - s_start, kLinearTolerance);
}

TEST_F(MalidriveFlatLineVariableWidthLaneFullyInitializedTest, PFromS) {
  EXPECT_NEAR(kP0, dut_->TrackSFromLaneS(s_start), kLinearTolerance);
  EXPECT_NEAR((kP1 + kP0) / 2., dut_->TrackSFromLaneS(s_half), kLinearTolerance);
  EXPECT_NEAR(kP1, dut_->TrackSFromLaneS(s_end), kLinearTolerance);
}

TEST_F(MalidriveFlatLineVariableWidthLaneFullyInitializedTest, SFromP) {
  EXPECT_NEAR(s_start, dut_->LaneSFromTrackS(kP0), kLinearTolerance);
  EXPECT_NEAR(s_half, dut_->LaneSFromTrackS((kP1 + kP0) / 2.), kLinearTolerance);
  EXPECT_NEAR(s_end, dut_->LaneSFromTrackS(kP1), kLinearTolerance);
}
// @}

TEST_F(MalidriveFlatLineVariableWidthLaneFullyInitializedTest, Bounds) {
  const maliput::api::RBounds kExpectedRBoundsAtStart{-lane_width_->f(kP0) / 2, lane_width_->f(kP0) / 2};
  const maliput::api::RBounds kExpectedRBoundsAtHalf{-lane_width_->f((kP0 + kP1) / 2) / 2,
                                                     lane_width_->f((kP0 + kP1) / 2) / 2};
  const maliput::api::RBounds kExpectedRBoundsAtEnd{0., 0.};

  // At the beginning of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBoundsAtStart, dut_->lane_bounds(s_start), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBoundsAtStart, dut_->segment_bounds(s_start), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(s_start, 0.), kLinearTolerance));
  // At the half of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBoundsAtHalf, dut_->lane_bounds(s_half), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBoundsAtHalf, dut_->segment_bounds(s_half), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(s_half, 0.), kLinearTolerance));
  // At the end of the lane.
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBoundsAtEnd, dut_->lane_bounds(s_end), kLinearTolerance));
  EXPECT_TRUE(IsRBoundsClose(kExpectedRBoundsAtEnd, dut_->segment_bounds(s_end), kLinearTolerance));
  EXPECT_TRUE(IsHBoundsClose(kElevationBounds, dut_->elevation_bounds(s_end, 0.), kLinearTolerance));
}

TEST_F(MalidriveFlatLineVariableWidthLaneFullyInitializedTest, ToInertialPosition) {
  // At centerline.
  // @{
  EXPECT_TRUE(IsInertialPositionClose({-1.1213203435592888901, 3.1213203435599963242, 0.},
                                      dut_->ToInertialPosition({s_start, kRCenterline, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose({34.58757210636101, 38.123106012293746, 0.},
                                      dut_->ToInertialPosition({s_half, kRCenterline, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose({70.296464556281, 73.124891681027, 0.},
                                      dut_->ToInertialPosition({s_end, kRCenterline, kZeroH}), kLinearTolerance));
  // @}

  // At Left.
  // @{
  EXPECT_TRUE(IsInertialPositionClose({-1.474873734152916, 3.474873734152916, 0.},
                                      dut_->ToInertialPosition({s_start, kRLeft, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose({34.23401871576773, 38.47665940288702, 0.},
                                      dut_->ToInertialPosition({s_half, kRLeft, kZeroH}), kLinearTolerance));

  // At Right.
  // @{
  EXPECT_TRUE(IsInertialPositionClose({-0.7677669529663687, 2.767766952966369, 0.},
                                      dut_->ToInertialPosition({s_start, kRRight, kZeroH}), kLinearTolerance));
  EXPECT_TRUE(IsInertialPositionClose({34.941125496954285, 37.76955262170047, 0.},
                                      dut_->ToInertialPosition({s_half, kRRight, kZeroH}), kLinearTolerance));
  // @}
}

// TODO(#458): Enable test once RoadCurve::RHat is fixed.
TEST_F(MalidriveFlatLineVariableWidthLaneFullyInitializedTest, ToLanePositionCenter) {
  LanePositionResult expected_result;

  // At centerline.
  //@{
  expected_result.lane_position = LanePosition{s_start, kRCenterline, kZeroH};
  expected_result.nearest_position = {-1.1213203435592888901, 3.1213203435599963242, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_half, kRCenterline, kZeroH};
  expected_result.nearest_position = {34.58757210636101, 38.123106012293746, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_end, kRCenterline, kZeroH};
  expected_result.nearest_position = {70.296464556281, 73.124891681027, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

// TODO(#458): Enable test once RoadCurve::RHat is fixed.
TEST_F(MalidriveFlatLineVariableWidthLaneFullyInitializedTest, DISABLED_ToLanePositionSides) {
  LanePositionResult expected_result;
  // To the left
  //@{
  expected_result.lane_position = LanePosition{s_start, kRLeft, kZeroH};
  expected_result.nearest_position = {-1.474873734152916, 3.474873734152916, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_half, kRLeft, kZeroH};
  expected_result.nearest_position = {35.08751585741569, 38.13060573104691, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}

  // To the right
  //@{
  expected_result.lane_position = LanePosition{s_start, kRRight, kZeroH};
  expected_result.nearest_position = {-0.7677669529663687, 2.767766952966369, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);

  expected_result.lane_position = LanePosition{s_half, kRRight, kZeroH};
  expected_result.nearest_position = {34.08762835530633, 38.11560629354058, 0.};
  expected_result.distance = 0.;
  IsLanePositionResultClose(expected_result, dut_->ToLanePosition(expected_result.nearest_position), kLinearTolerance);
  //@}
}

// TODO(francocipollone): Add tests for GetOrientation method.
// TODO(francocipollone): Add tests for EvalMotionDerivatives method.

// @{ The following tests shows the behavior of a Lane when its
//    length is below linear tolerance. Note that in this test case, only a
//    fraction of the road_curve::RoadCurve is considered (5m out of
//    ~7.85m).
class BelowLinearToleranceLaneTest : public ::testing::Test {
 protected:
  const double kArcLength{M_PI / 0.4};
  const double kP0{0.};
  const double kP1GroundCurve{kP0 + kArcLength};
  const double kP1Lane{kP0 + 5.};
  const double kLinearTolerance{10.};
  const bool kAssertContiguity{true};

  void SetUp() override {
    auto lane_width =
        std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 4., kP0, kP1GroundCurve, kLinearTolerance);
    auto lane_offset =
        std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 1., kP0, kP1GroundCurve, kLinearTolerance);
    auto elevation =
        std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 0., kP0, kP1GroundCurve, kLinearTolerance);
    auto superelevation =
        std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 0., kP0, kP1GroundCurve, kLinearTolerance);
    auto ground_curve = std::make_unique<road_curve::ArcGroundCurve>(
        kLinearTolerance, /*xy0*/ maliput::math::Vector2{0., 0.}, /*start_heading*/ 0.,
        /*curvature*/ 0.4, kArcLength, kP0, kP1GroundCurve);
    road_curve_ =
        std::make_unique<road_curve::RoadCurve>(kLinearTolerance, /*scale_length*/ 1., std::move(ground_curve),
                                                std::move(elevation), std::move(superelevation), kAssertContiguity);
    dut_ = std::make_unique<Lane>(maliput::api::LaneId{"dut"}, /*xodr_track*/ 1, /*xodr_lane_id*/ 1,
                                  maliput::api::HBounds(0., 5.), road_curve_.get(), std::move(lane_width),
                                  std::move(lane_offset), kP0, kP1Lane);
  }

  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<Lane> dut_;
};

TEST_F(BelowLinearToleranceLaneTest, LengthTest) {
  EXPECT_NEAR(/*kP1 - kP0*/ 5.0, dut_->length(), /*test tolerance*/ 1e-12);
}
// @}

}  // namespace
}  // namespace test
}  // namespace malidrive
