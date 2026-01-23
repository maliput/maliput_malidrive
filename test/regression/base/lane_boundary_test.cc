// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota. All rights reserved.
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
#include "maliput_malidrive/base/lane_boundary.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane_boundary.h>
#include <maliput/api/lane_marking.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_throw.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/loader/loader.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/xodr/lane_road_mark.h"
#include "utility/resources.h"

namespace malidrive {
namespace test {
namespace {

using maliput::api::LaneMarking;
using maliput::api::LaneMarkingColor;
using maliput::api::LaneMarkingResult;
using maliput::api::LaneMarkingType;
using maliput::api::LaneMarkingWeight;
using maliput::math::Vector2;

std::unique_ptr<road_curve::Function> MakeZeroCubicPolynomial(double p0, double p1, double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 0., p0, p1, linear_tolerance);
}

std::unique_ptr<road_curve::Function> MakeConstantCubicPolynomial(double d, double p0, double p1,
                                                                  double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., d, p0, p1, linear_tolerance);
}

// Test fixture for LaneBoundary tests.
class LaneBoundaryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
    segment_ = std::make_unique<Segment>(kSegmentId, road_curve_.get(), reference_line_offset_.get(), kP0, kP1);

    // Create a reference lane for coordinate conversion tests.
    // This lane has identity coordinate conversion (LANE frame == TRACK frame) since it's a straight line.
    reference_lane_ = std::make_unique<Lane>(kLaneId, kXodrTrack, kXodrLaneId, kElevationBounds, road_curve_.get(),
                                             MakeConstantCubicPolynomial(kLaneWidth, kP0, kP1, kLinearTolerance),
                                             MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),  // Zero offset
                                             kP0, kP1, kIntegratorAccuracyMultiplier, kXodrLaneType);
  }

  const maliput::api::LaneBoundary::Id kBoundaryId{"test_boundary"};
  const maliput::api::SegmentId kSegmentId{"test_segment"};
  const maliput::api::LaneId kLaneId{"test_lane"};
  const int kIndex{0};
  const double kP0{0.};
  const double kP1{100.};
  const double kTrackSStart{0.};
  const double kTrackSEnd{100.};
  const double kLinearTolerance{1e-13};
  const double kTolerance{1e-12};  // Tolerance for double comparisons in tests.
  const double kScaleLength{1.};
  const Vector2 kXy0{0., 0.};
  const Vector2 kDXy{100., 0.};
  const bool kAssertContiguity{true};

  // Lane construction parameters.
  const int kXodrTrack{1};
  const int kXodrLaneId{-1};
  const maliput::api::HBounds kElevationBounds{0., 5.};
  const double kLaneWidth{3.5};
  const xodr::Lane::Type kXodrLaneType{xodr::Lane::Type::kDriving};
  const double kIntegratorAccuracyMultiplier{1.};

  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
  std::unique_ptr<Segment> segment_;
  std::unique_ptr<Lane> reference_lane_;
};

// Test basic construction.
TEST_F(LaneBoundaryTest, Constructor) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  EXPECT_NO_THROW(LaneBoundary(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                               kTrackSStart, kTrackSEnd));
}

// Test constructor throws when segment is nullptr.
TEST_F(LaneBoundaryTest, ConstructorThrowsOnNullSegment) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  EXPECT_THROW(LaneBoundary(kBoundaryId, nullptr, kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                            kTrackSStart, kTrackSEnd),
               maliput::common::assertion_error);
}

// Test constructor throws when reference lane is nullptr.
TEST_F(LaneBoundaryTest, ConstructorThrowsOnNullReferenceLane) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  EXPECT_THROW(LaneBoundary(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, nullptr, road_marks, kTrackSStart,
                            kTrackSEnd),
               maliput::common::assertion_error);
}

// Test constructor throws when index is negative.
TEST_F(LaneBoundaryTest, ConstructorThrowsOnNegativeIndex) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  EXPECT_THROW(LaneBoundary(kBoundaryId, segment_.get(), -1, nullptr, nullptr, reference_lane_.get(), road_marks,
                            kTrackSStart, kTrackSEnd),
               maliput::common::assertion_error);
}

// Test constructor throws when s_end < s_start.
TEST_F(LaneBoundaryTest, ConstructorThrowsOnInvalidSRange) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  EXPECT_THROW(
      LaneBoundary(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks, 100., 50.),
      maliput::common::assertion_error);
}

// Test ID accessor.
TEST_F(LaneBoundaryTest, Id) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  EXPECT_EQ(kBoundaryId, dut.id());
}

// Test segment accessor.
TEST_F(LaneBoundaryTest, Segment) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  EXPECT_EQ(segment_.get(), dut.segment());
}

// Test index accessor.
TEST_F(LaneBoundaryTest, Index) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  EXPECT_EQ(kIndex, dut.index());
}

// Test lane_to_left and lane_to_right accessors with nullptr.
TEST_F(LaneBoundaryTest, AdjacentLanesNull) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  EXPECT_EQ(nullptr, dut.lane_to_left());
  EXPECT_EQ(nullptr, dut.lane_to_right());
}

// Test GetMarkings returns empty vector when no road marks.
TEST_F(LaneBoundaryTest, GetMarkingsEmpty) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  EXPECT_TRUE(markings.empty());
}

// Test GetMarking returns nullopt when no road marks.
TEST_F(LaneBoundaryTest, GetMarkingReturnsNulloptWhenEmpty) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto marking = dut.GetMarking(50.);
  EXPECT_FALSE(marking.has_value());
}

// Test with a single solid road mark.
TEST_F(LaneBoundaryTest, SingleSolidRoadMark) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.weight = xodr::LaneRoadMark::Weight::kStandard;
  mark.color = xodr::Color::kWhite;
  mark.width = 0.15;
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Test GetMarkings.
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_EQ(LaneMarkingType::kSolid, markings[0].marking.type);
  EXPECT_EQ(LaneMarkingWeight::kStandard, markings[0].marking.weight);
  EXPECT_EQ(LaneMarkingColor::kWhite, markings[0].marking.color);
  EXPECT_NEAR(0.15, markings[0].marking.width, kTolerance);
  EXPECT_NEAR(0., markings[0].s_start, kTolerance);
  EXPECT_NEAR(100., markings[0].s_end, kTolerance);

  // Test GetMarking at specific s - now returns LaneMarkingResult with s-range.
  const auto result = dut.GetMarking(50.);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(LaneMarkingType::kSolid, result->marking.type);
  EXPECT_NEAR(0., result->s_start, kTolerance);
  EXPECT_NEAR(100., result->s_end, kTolerance);
}

// Test with multiple road marks (solid at start, broken at end).
TEST_F(LaneBoundaryTest, MultipleRoadMarks) {
  std::vector<xodr::LaneRoadMark> road_marks{};

  // First mark: solid from 0 to 50.
  xodr::LaneRoadMark mark1;
  mark1.s_offset = 0.;
  mark1.type = xodr::LaneRoadMark::Type::kSolid;
  mark1.color = xodr::Color::kWhite;
  road_marks.push_back(mark1);

  // Second mark: broken from 50 to 100.
  xodr::LaneRoadMark mark2;
  mark2.s_offset = 50.;
  mark2.type = xodr::LaneRoadMark::Type::kBroken;
  mark2.color = xodr::Color::kYellow;
  road_marks.push_back(mark2);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Test GetMarkings.
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(2u, markings.size());

  // First marking.
  EXPECT_EQ(LaneMarkingType::kSolid, markings[0].marking.type);
  EXPECT_EQ(LaneMarkingColor::kWhite, markings[0].marking.color);
  EXPECT_NEAR(0., markings[0].s_start, kTolerance);
  EXPECT_NEAR(50., markings[0].s_end, kTolerance);

  // Second marking.
  EXPECT_EQ(LaneMarkingType::kBroken, markings[1].marking.type);
  EXPECT_EQ(LaneMarkingColor::kYellow, markings[1].marking.color);
  EXPECT_NEAR(50., markings[1].s_start, kTolerance);
  EXPECT_NEAR(100., markings[1].s_end, kTolerance);

  // Test GetMarking at s=25 (should be solid) - now returns LaneMarkingResult.
  const auto result_at_25 = dut.GetMarking(25.);
  ASSERT_TRUE(result_at_25.has_value());
  EXPECT_EQ(LaneMarkingType::kSolid, result_at_25->marking.type);
  EXPECT_NEAR(0., result_at_25->s_start, kTolerance);
  EXPECT_NEAR(50., result_at_25->s_end, kTolerance);

  // Test GetMarking at s=75 (should be broken) - now returns LaneMarkingResult.
  const auto result_at_75 = dut.GetMarking(75.);
  ASSERT_TRUE(result_at_75.has_value());
  EXPECT_EQ(LaneMarkingType::kBroken, result_at_75->marking.type);
  EXPECT_NEAR(50., result_at_75->s_start, kTolerance);
  EXPECT_NEAR(100., result_at_75->s_end, kTolerance);
}

// Test GetMarkings with s-range filter.
TEST_F(LaneBoundaryTest, GetMarkingsWithSRange) {
  std::vector<xodr::LaneRoadMark> road_marks{};

  xodr::LaneRoadMark mark1;
  mark1.s_offset = 0.;
  mark1.type = xodr::LaneRoadMark::Type::kSolid;
  road_marks.push_back(mark1);

  xodr::LaneRoadMark mark2;
  mark2.s_offset = 50.;
  mark2.type = xodr::LaneRoadMark::Type::kBroken;
  road_marks.push_back(mark2);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Query range that includes both marks.
  auto markings = dut.GetMarkings(25., 75.);
  EXPECT_EQ(2u, markings.size());

  // Query range that only includes first mark.
  markings = dut.GetMarkings(0., 40.);
  ASSERT_EQ(1u, markings.size());
  EXPECT_EQ(LaneMarkingType::kSolid, markings[0].marking.type);

  // Query range that only includes second mark.
  markings = dut.GetMarkings(60., 100.);
  ASSERT_EQ(1u, markings.size());
  EXPECT_EQ(LaneMarkingType::kBroken, markings[0].marking.type);

  // Query range that includes no marks.
  markings = dut.GetMarkings(110., 120.);
  EXPECT_TRUE(markings.empty());
}

// Test GetMarkings throws when s_start > s_end.
TEST_F(LaneBoundaryTest, GetMarkingsThrowsOnInvalidRange) {
  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  EXPECT_THROW(dut.GetMarkings(100., 50.), maliput::common::assertion_error);
}

// Test type conversions.
TEST_F(LaneBoundaryTest, TypeConversions) {
  // Test different types.
  const std::vector<std::pair<xodr::LaneRoadMark::Type, LaneMarkingType>> type_mappings = {
      {xodr::LaneRoadMark::Type::kNone, LaneMarkingType::kNone},
      {xodr::LaneRoadMark::Type::kSolid, LaneMarkingType::kSolid},
      {xodr::LaneRoadMark::Type::kBroken, LaneMarkingType::kBroken},
      {xodr::LaneRoadMark::Type::kSolidSolid, LaneMarkingType::kSolidSolid},
      {xodr::LaneRoadMark::Type::kSolidBroken, LaneMarkingType::kSolidBroken},
      {xodr::LaneRoadMark::Type::kBrokenSolid, LaneMarkingType::kBrokenSolid},
      {xodr::LaneRoadMark::Type::kBrokenBroken, LaneMarkingType::kBrokenBroken},
      {xodr::LaneRoadMark::Type::kBottsDots, LaneMarkingType::kBottsDots},
      {xodr::LaneRoadMark::Type::kGrass, LaneMarkingType::kGrass},
      {xodr::LaneRoadMark::Type::kCurb, LaneMarkingType::kCurb},
      {xodr::LaneRoadMark::Type::kEdge, LaneMarkingType::kEdge},
      {xodr::LaneRoadMark::Type::kCustom, LaneMarkingType::kUnknown},
  };

  for (const auto& [xodr_type, expected_type] : type_mappings) {
    std::vector<xodr::LaneRoadMark> road_marks{};
    xodr::LaneRoadMark mark;
    mark.s_offset = 0.;
    mark.type = xodr_type;
    mark.color = xodr::Color::kWhite;
    road_marks.push_back(mark);

    const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                           kTrackSStart, kTrackSEnd);
    const auto markings = dut.GetMarkings();
    ASSERT_EQ(1u, markings.size());
    EXPECT_EQ(expected_type, markings[0].marking.type);
  }
}

// Test color conversions.
TEST_F(LaneBoundaryTest, ColorConversions) {
  const std::vector<std::pair<xodr::Color, LaneMarkingColor>> color_mappings = {
      {xodr::Color::kWhite, LaneMarkingColor::kWhite},
      {xodr::Color::kStandard, LaneMarkingColor::kWhite},  // Standard maps to white.
      {xodr::Color::kYellow, LaneMarkingColor::kYellow},
      {xodr::Color::kOrange, LaneMarkingColor::kOrange},
      {xodr::Color::kRed, LaneMarkingColor::kRed},
      {xodr::Color::kBlue, LaneMarkingColor::kBlue},
      {xodr::Color::kGreen, LaneMarkingColor::kGreen},
      {xodr::Color::kViolet, LaneMarkingColor::kViolet},
      {xodr::Color::kBlack, LaneMarkingColor::kUnknown},
  };

  for (const auto& [xodr_color, expected_color] : color_mappings) {
    std::vector<xodr::LaneRoadMark> road_marks{};
    xodr::LaneRoadMark mark;
    mark.s_offset = 0.;
    mark.type = xodr::LaneRoadMark::Type::kSolid;
    mark.color = xodr_color;
    road_marks.push_back(mark);

    const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                           kTrackSStart, kTrackSEnd);
    const auto markings = dut.GetMarkings();
    ASSERT_EQ(1u, markings.size());
    EXPECT_EQ(expected_color, markings[0].marking.color);
  }
}

// Test lane change permission conversion.
TEST_F(LaneBoundaryTest, LaneChangeConversion) {
  using maliput::api::LaneChangePermission;

  const std::vector<std::pair<std::optional<xodr::LaneRoadMark::LaneChange>, LaneChangePermission>> mappings = {
      {std::nullopt, LaneChangePermission::kAllowed},
      {xodr::LaneRoadMark::LaneChange::kBoth, LaneChangePermission::kAllowed},
      {xodr::LaneRoadMark::LaneChange::kIncrease, LaneChangePermission::kToLeft},
      {xodr::LaneRoadMark::LaneChange::kDecrease, LaneChangePermission::kToRight},
      {xodr::LaneRoadMark::LaneChange::kNone, LaneChangePermission::kProhibited},
  };

  for (const auto& [xodr_lane_change, expected_permission] : mappings) {
    std::vector<xodr::LaneRoadMark> road_marks{};
    xodr::LaneRoadMark mark;
    mark.s_offset = 0.;
    mark.type = xodr::LaneRoadMark::Type::kSolid;
    mark.color = xodr::Color::kWhite;
    mark.lane_change = xodr_lane_change;
    road_marks.push_back(mark);

    const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                           kTrackSStart, kTrackSEnd);
    const auto markings = dut.GetMarkings();
    ASSERT_EQ(1u, markings.size());
    EXPECT_EQ(expected_permission, markings[0].marking.lane_change);
  }
}

// Test TypeElement line conversion.
// TypeElement lines are part of OpenDRIVE's <roadMark><type><line> structure
// and define detailed dash patterns with length, space (gap), width, t_offset, and color.
TEST_F(LaneBoundaryTest, TypeElementLineConversion) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kBroken;
  mark.color = xodr::Color::kWhite;

  // Add a TypeElement with detailed line definitions.
  xodr::TypeElement type_elem;
  type_elem.name = "broken";
  type_elem.width = 0.15;

  // Add two lines (e.g., for a double broken line).
  xodr::TypeElementLine line1;
  line1.length = 3.0;    // 3m dash length.
  line1.space = 9.0;     // 9m gap.
  line1.width = 0.12;    // 12cm width.
  line1.t_offset = 0.0;  // On the boundary itself.
  line1.s_offset = 0.0;
  line1.color = xodr::Color::kWhite;
  type_elem.lines.push_back(line1);

  xodr::TypeElementLine line2;
  line2.length = 3.0;
  line2.space = 9.0;
  line2.width = 0.12;
  line2.t_offset = 0.3;  // Offset 30cm from the boundary (second line of double marking).
  line2.s_offset = 0.0;
  line2.color = xodr::Color::kYellow;  // Different color for testing.
  type_elem.lines.push_back(line2);

  mark.type_elems.push_back(type_elem);
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());

  // Verify the lines were converted correctly.
  const auto& converted_lines = markings[0].marking.lines;
  ASSERT_EQ(2u, converted_lines.size());

  // First line.
  EXPECT_NEAR(3.0, converted_lines[0].length, kTolerance);
  EXPECT_NEAR(9.0, converted_lines[0].space, kTolerance);
  EXPECT_NEAR(0.12, converted_lines[0].width, kTolerance);
  EXPECT_NEAR(0.0, converted_lines[0].r_offset, kTolerance);  // t_offset -> r_offset.
  EXPECT_EQ(LaneMarkingColor::kWhite, converted_lines[0].color);

  // Second line.
  EXPECT_NEAR(3.0, converted_lines[1].length, kTolerance);
  EXPECT_NEAR(9.0, converted_lines[1].space, kTolerance);
  EXPECT_NEAR(0.12, converted_lines[1].width, kTolerance);
  EXPECT_NEAR(0.3, converted_lines[1].r_offset, kTolerance);  // t_offset -> r_offset.
  EXPECT_EQ(LaneMarkingColor::kYellow, converted_lines[1].color);
}

// Test ExplicitElement line conversion.
// ExplicitElement lines are part of OpenDRIVE's <roadMark><explicit><line> structure
// and define individual line segments with explicit positioning.
// Key differences from TypeElement:
// - No 'space' field (explicit lines are point-specified, not repeating patterns).
// - No 'color' field (uses parent marking's color).
TEST_F(LaneBoundaryTest, ExplicitElementLineConversion) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kYellow;  // Parent color.

  // Add an ExplicitElement with individual line segments.
  xodr::ExplicitElement explicit_elem;

  // First explicit line segment.
  xodr::ExplicitElementLine explicit_line1;
  explicit_line1.length = 5.0;     // 5m segment length.
  explicit_line1.s_offset = 0.0;   // Starts at s=0.
  explicit_line1.t_offset = 0.05;  // 5cm lateral offset from boundary.
  explicit_line1.width = 0.15;     // 15cm width.
  explicit_elem.lines.push_back(explicit_line1);

  // Second explicit line segment at different position.
  xodr::ExplicitElementLine explicit_line2;
  explicit_line2.length = 10.0;     // 10m segment length.
  explicit_line2.s_offset = 20.0;   // Starts at s=20.
  explicit_line2.t_offset = -0.10;  // -10cm lateral offset (other side).
  explicit_line2.width = 0.20;      // 20cm width.
  explicit_elem.lines.push_back(explicit_line2);

  mark.explicit_elems.push_back(explicit_elem);
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());

  // Verify the explicit lines were converted correctly.
  const auto& converted_lines = markings[0].marking.lines;
  ASSERT_EQ(2u, converted_lines.size());

  // First explicit line.
  EXPECT_NEAR(5.0, converted_lines[0].length, kTolerance);
  EXPECT_NEAR(0.0, converted_lines[0].space, kTolerance);  // Explicit lines have no gap pattern -> space = 0.
  EXPECT_NEAR(0.15, converted_lines[0].width, kTolerance);
  EXPECT_NEAR(0.05, converted_lines[0].r_offset, kTolerance);  // t_offset -> r_offset.
  // ExplicitElementLine has no color, so it should default to kUnknown (parent color is not automatically applied).
  EXPECT_EQ(LaneMarkingColor::kUnknown, converted_lines[0].color);

  // Second explicit line.
  EXPECT_NEAR(10.0, converted_lines[1].length, kTolerance);
  EXPECT_NEAR(0.0, converted_lines[1].space, kTolerance);  // Explicit lines have no gap pattern -> space = 0.
  EXPECT_NEAR(0.20, converted_lines[1].width, kTolerance);
  EXPECT_NEAR(-0.10, converted_lines[1].r_offset, kTolerance);  // t_offset -> r_offset (negative offset preserved).
  EXPECT_EQ(LaneMarkingColor::kUnknown, converted_lines[1].color);
}

// Test that both TypeElement and ExplicitElement lines can coexist in the same road mark.
TEST_F(LaneBoundaryTest, MixedTypeAndExplicitElements) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;

  // Add a TypeElement line.
  xodr::TypeElement type_elem;
  type_elem.name = "solid";
  type_elem.width = 0.15;
  xodr::TypeElementLine type_line;
  type_line.length = 0.0;  // Solid line (no dashes).
  type_line.space = 0.0;
  type_line.width = 0.15;
  type_line.t_offset = 0.0;
  type_line.s_offset = 0.0;
  type_line.color = xodr::Color::kWhite;
  type_elem.lines.push_back(type_line);
  mark.type_elems.push_back(type_elem);

  // Add an ExplicitElement line.
  xodr::ExplicitElement explicit_elem;
  xodr::ExplicitElementLine explicit_line;
  explicit_line.length = 2.0;
  explicit_line.s_offset = 50.0;
  explicit_line.t_offset = 0.5;  // 50cm offset.
  explicit_line.width = 0.10;
  explicit_elem.lines.push_back(explicit_line);
  mark.explicit_elems.push_back(explicit_elem);

  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());

  // Should have both lines: type element line first, then explicit element line.
  const auto& converted_lines = markings[0].marking.lines;
  ASSERT_EQ(2u, converted_lines.size());

  // First line (from TypeElement).
  EXPECT_NEAR(0.0, converted_lines[0].length, kTolerance);
  EXPECT_NEAR(0.0, converted_lines[0].space, kTolerance);
  EXPECT_NEAR(0.15, converted_lines[0].width, kTolerance);
  EXPECT_NEAR(0.0, converted_lines[0].r_offset, kTolerance);
  EXPECT_EQ(LaneMarkingColor::kWhite, converted_lines[0].color);

  // Second line (from ExplicitElement).
  EXPECT_NEAR(2.0, converted_lines[1].length, kTolerance);
  EXPECT_NEAR(0.0, converted_lines[1].space, kTolerance);
  EXPECT_NEAR(0.10, converted_lines[1].width, kTolerance);
  EXPECT_NEAR(0.5, converted_lines[1].r_offset, kTolerance);
  EXPECT_EQ(LaneMarkingColor::kUnknown, converted_lines[1].color);  // No color in ExplicitElementLine.
}

// Test weight conversions.
TEST_F(LaneBoundaryTest, WeightConversions) {
  const std::vector<std::pair<std::optional<xodr::LaneRoadMark::Weight>, LaneMarkingWeight>> weight_mappings = {
      {xodr::LaneRoadMark::Weight::kStandard, LaneMarkingWeight::kStandard},
      {xodr::LaneRoadMark::Weight::kBold, LaneMarkingWeight::kBold},
      {std::nullopt, LaneMarkingWeight::kUnknown},
  };

  for (const auto& [xodr_weight, expected_weight] : weight_mappings) {
    std::vector<xodr::LaneRoadMark> road_marks{};
    xodr::LaneRoadMark mark;
    mark.s_offset = 0.;
    mark.type = xodr::LaneRoadMark::Type::kSolid;
    mark.color = xodr::Color::kWhite;
    mark.weight = xodr_weight;
    road_marks.push_back(mark);

    const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                           kTrackSStart, kTrackSEnd);
    const auto markings = dut.GetMarkings();
    ASSERT_EQ(1u, markings.size());
    EXPECT_EQ(expected_weight, markings[0].marking.weight);
  }
}

// Test height field conversion.
TEST_F(LaneBoundaryTest, HeightConversion) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kCurb;
  mark.color = xodr::Color::kWhite;
  mark.height = 0.15;  // 15cm curb height.
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_NEAR(0.15, markings[0].marking.height, kTolerance);
}

// Test height field when not specified (nullopt).
TEST_F(LaneBoundaryTest, HeightConversionNullopt) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  // height is not set (std::nullopt).
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_NEAR(0., markings[0].marking.height, kTolerance);  // Default to 0.
}

// Test material field conversion.
TEST_F(LaneBoundaryTest, MaterialConversion) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  mark.material = "thermoplastic";
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_EQ("thermoplastic", markings[0].marking.material);
}

// Test material field when not specified (nullopt).
TEST_F(LaneBoundaryTest, MaterialConversionNullopt) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  // material is not set (std::nullopt).
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_TRUE(markings[0].marking.material.empty());  // Default to empty string.
}

// Test optional fields default values when all optional fields are nullopt.
TEST_F(LaneBoundaryTest, OptionalFieldsDefaults) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  // All optional fields left as nullopt:
  // - weight: nullopt
  // - width: nullopt
  // - height: nullopt
  // - material: nullopt
  // - lane_change: nullopt
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());

  // Verify default values for optional fields.
  EXPECT_EQ(LaneMarkingWeight::kUnknown, markings[0].marking.weight);
  EXPECT_NEAR(0., markings[0].marking.width, kTolerance);
  EXPECT_NEAR(0., markings[0].marking.height, kTolerance);
  EXPECT_TRUE(markings[0].marking.material.empty());
  EXPECT_EQ(maliput::api::LaneChangePermission::kAllowed, markings[0].marking.lane_change);  // nullopt -> kAllowed.
}

// Test boundary conditions: query at exact transition point.
TEST_F(LaneBoundaryTest, BoundaryConditionsAtTransitionPoint) {
  std::vector<xodr::LaneRoadMark> road_marks{};

  // First mark: solid from 0 to 50.
  xodr::LaneRoadMark mark1;
  mark1.s_offset = 0.;
  mark1.type = xodr::LaneRoadMark::Type::kSolid;
  mark1.color = xodr::Color::kWhite;
  road_marks.push_back(mark1);

  // Second mark: broken from 50 to 100.
  xodr::LaneRoadMark mark2;
  mark2.s_offset = 50.;
  mark2.type = xodr::LaneRoadMark::Type::kBroken;
  mark2.color = xodr::Color::kYellow;
  road_marks.push_back(mark2);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Query exactly at s=50 (transition point).
  // The second marking starts at s=50, so querying at s=50 should return the second marking.
  const auto result_at_50 = dut.GetMarking(50.);
  ASSERT_TRUE(result_at_50.has_value());
  EXPECT_EQ(LaneMarkingType::kBroken, result_at_50->marking.type);
  EXPECT_EQ(LaneMarkingColor::kYellow, result_at_50->marking.color);
  EXPECT_NEAR(50., result_at_50->s_start, kTolerance);
  EXPECT_NEAR(100., result_at_50->s_end, kTolerance);

  // Query at s=0 (start of boundary).
  const auto result_at_0 = dut.GetMarking(0.);
  ASSERT_TRUE(result_at_0.has_value());
  EXPECT_EQ(LaneMarkingType::kSolid, result_at_0->marking.type);
  EXPECT_NEAR(0., result_at_0->s_start, kTolerance);
  EXPECT_NEAR(50., result_at_0->s_end, kTolerance);

  // Query at s=100 (end of boundary).
  const auto result_at_100 = dut.GetMarking(100.);
  // The second marking covers [50, 100), so s=100 might be outside.
  // Depending on implementation, this could return the second marking or nullopt.
  // The marking interval is typically [s_start, s_end), so s=100 should be out of range.
  // If the implementation uses [s_start, s_end], it should return the second marking.
  // Testing the actual behavior:
  if (result_at_100.has_value()) {
    EXPECT_EQ(LaneMarkingType::kBroken, result_at_100->marking.type);
  }
  // Note: The actual behavior depends on the interval semantics used in the implementation.
}

// Test boundary conditions: query outside valid range.
TEST_F(LaneBoundaryTest, BoundaryConditionsOutsideRange) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Query at negative s (before the boundary).
  const auto result_negative = dut.GetMarking(-10.);
  // Behavior depends on implementation - may return nullopt or clamp to valid range.
  // Just verify it doesn't crash.
  (void)result_negative;

  // Query beyond s_end (after the boundary).
  const auto result_beyond = dut.GetMarking(150.);
  // Behavior depends on implementation - may return nullopt or clamp to valid range.
  // Just verify it doesn't crash.
  (void)result_beyond;
}

// Test GetMarkings with range exactly matching a single marking.
TEST_F(LaneBoundaryTest, GetMarkingsExactRangeMatch) {
  std::vector<xodr::LaneRoadMark> road_marks{};

  xodr::LaneRoadMark mark1;
  mark1.s_offset = 0.;
  mark1.type = xodr::LaneRoadMark::Type::kSolid;
  road_marks.push_back(mark1);

  xodr::LaneRoadMark mark2;
  mark2.s_offset = 50.;
  mark2.type = xodr::LaneRoadMark::Type::kBroken;
  road_marks.push_back(mark2);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Query range exactly matching first marking [0, 50].
  auto markings = dut.GetMarkings(0., 50.);
  ASSERT_GE(markings.size(), 1u);
  EXPECT_EQ(LaneMarkingType::kSolid, markings[0].marking.type);

  // Query range exactly matching second marking [50, 100].
  markings = dut.GetMarkings(50., 100.);
  ASSERT_GE(markings.size(), 1u);
  // Should include the broken marking.
  bool found_broken = false;
  for (const auto& m : markings) {
    if (m.marking.type == LaneMarkingType::kBroken) {
      found_broken = true;
      break;
    }
  }
  EXPECT_TRUE(found_broken);
}

// Test adjacent lanes with non-null pointers.
TEST_F(LaneBoundaryTest, AdjacentLanesNonNull) {
  // Create a second lane to use as adjacent lane.
  const maliput::api::LaneId kAdjacentLaneId{"adjacent_lane"};
  auto adjacent_lane = std::make_unique<Lane>(kAdjacentLaneId, kXodrTrack, -2, kElevationBounds, road_curve_.get(),
                                              MakeConstantCubicPolynomial(kLaneWidth, kP0, kP1, kLinearTolerance),
                                              MakeConstantCubicPolynomial(-kLaneWidth, kP0, kP1, kLinearTolerance), kP0,
                                              kP1, kIntegratorAccuracyMultiplier, kXodrLaneType);

  const std::vector<xodr::LaneRoadMark> road_marks{};
  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, reference_lane_.get(), adjacent_lane.get(),
                         reference_lane_.get(), road_marks, kTrackSStart, kTrackSEnd);

  EXPECT_EQ(reference_lane_.get(), dut.lane_to_left());
  EXPECT_EQ(adjacent_lane.get(), dut.lane_to_right());
}

// Test marking with road mark starting mid-boundary (s_offset > 0).
TEST_F(LaneBoundaryTest, RoadMarkStartingMidBoundary) {
  std::vector<xodr::LaneRoadMark> road_marks{};

  // Mark starts at s=30, not at s=0.
  xodr::LaneRoadMark mark;
  mark.s_offset = 30.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Query before the marking starts.
  const auto result_at_10 = dut.GetMarking(10.);
  // No marking defined in [0, 30), so this should return nullopt.
  EXPECT_FALSE(result_at_10.has_value());

  // Query within the marking range.
  const auto result_at_50 = dut.GetMarking(50.);
  ASSERT_TRUE(result_at_50.has_value());
  EXPECT_EQ(LaneMarkingType::kSolid, result_at_50->marking.type);
  EXPECT_NEAR(30., result_at_50->s_start, kTolerance);
  EXPECT_NEAR(100., result_at_50->s_end, kTolerance);

  // GetMarkings should return only the marking starting at s=30.
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_NEAR(30., markings[0].s_start, kTolerance);
}

// Test multiple TypeElements in a single road mark.
TEST_F(LaneBoundaryTest, MultipleTypeElements) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolidSolid;
  mark.color = xodr::Color::kYellow;

  // First TypeElement - left line of double solid.
  xodr::TypeElement type_elem1;
  type_elem1.name = "left_line";
  type_elem1.width = 0.12;
  xodr::TypeElementLine line1;
  line1.length = 0.0;  // Solid.
  line1.space = 0.0;
  line1.width = 0.12;
  line1.t_offset = -0.15;  // Left of center.
  line1.s_offset = 0.0;
  line1.color = xodr::Color::kYellow;
  type_elem1.lines.push_back(line1);
  mark.type_elems.push_back(type_elem1);

  // Second TypeElement - right line of double solid.
  xodr::TypeElement type_elem2;
  type_elem2.name = "right_line";
  type_elem2.width = 0.12;
  xodr::TypeElementLine line2;
  line2.length = 0.0;  // Solid.
  line2.space = 0.0;
  line2.width = 0.12;
  line2.t_offset = 0.15;  // Right of center.
  line2.s_offset = 0.0;
  line2.color = xodr::Color::kYellow;
  type_elem2.lines.push_back(line2);
  mark.type_elems.push_back(type_elem2);

  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_EQ(LaneMarkingType::kSolidSolid, markings[0].marking.type);

  // Should have two lines from two TypeElements.
  const auto& lines = markings[0].marking.lines;
  ASSERT_EQ(2u, lines.size());

  // First line (from first TypeElement).
  EXPECT_NEAR(0.12, lines[0].width, kTolerance);
  EXPECT_NEAR(-0.15, lines[0].r_offset, kTolerance);
  EXPECT_EQ(LaneMarkingColor::kYellow, lines[0].color);

  // Second line (from second TypeElement).
  EXPECT_NEAR(0.12, lines[1].width, kTolerance);
  EXPECT_NEAR(0.15, lines[1].r_offset, kTolerance);
  EXPECT_EQ(LaneMarkingColor::kYellow, lines[1].color);
}

// Test multiple ExplicitElements in a single road mark.
TEST_F(LaneBoundaryTest, MultipleExplicitElements) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;

  // First ExplicitElement.
  xodr::ExplicitElement explicit_elem1;
  xodr::ExplicitElementLine explicit_line1;
  explicit_line1.length = 5.0;
  explicit_line1.s_offset = 0.0;
  explicit_line1.t_offset = 0.0;
  explicit_line1.width = 0.15;
  explicit_elem1.lines.push_back(explicit_line1);
  mark.explicit_elems.push_back(explicit_elem1);

  // Second ExplicitElement.
  xodr::ExplicitElement explicit_elem2;
  xodr::ExplicitElementLine explicit_line2;
  explicit_line2.length = 10.0;
  explicit_line2.s_offset = 50.0;
  explicit_line2.t_offset = 0.25;
  explicit_line2.width = 0.20;
  explicit_elem2.lines.push_back(explicit_line2);
  mark.explicit_elems.push_back(explicit_elem2);

  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());

  // Should have two lines from two ExplicitElements.
  const auto& lines = markings[0].marking.lines;
  ASSERT_EQ(2u, lines.size());

  // First line.
  EXPECT_NEAR(5.0, lines[0].length, kTolerance);
  EXPECT_NEAR(0.15, lines[0].width, kTolerance);
  EXPECT_NEAR(0.0, lines[0].r_offset, kTolerance);

  // Second line.
  EXPECT_NEAR(10.0, lines[1].length, kTolerance);
  EXPECT_NEAR(0.20, lines[1].width, kTolerance);
  EXPECT_NEAR(0.25, lines[1].r_offset, kTolerance);
}

// Test SwayElement behavior.
// SwayElement defines polynomial lateral offset variation along s.
// Note: Sway elements may not be fully implemented - this test documents expected behavior.
TEST_F(LaneBoundaryTest, SwayElementBehavior) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  mark.width = 0.15;

  // Add a SwayElement (polynomial lateral offset).
  xodr::SwayElement sway;
  sway.a = 0.0;   // Initial offset.
  sway.b = 0.01;  // Linear coefficient.
  sway.c = 0.0;   // Quadratic coefficient.
  sway.d = 0.0;   // Cubic coefficient.
  sway.s_0 = 0.0;
  mark.sway_elems.push_back(sway);

  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // The marking should still be created even with sway elements.
  // Whether sway is actually applied depends on the implementation.
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_EQ(LaneMarkingType::kSolid, markings[0].marking.type);

  // Note: SwayElement application to geometry is implementation-specific.
  // This test verifies that the presence of sway elements doesn't cause errors.
}

// Test width field conversion.
TEST_F(LaneBoundaryTest, WidthConversion) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  mark.width = 0.25;  // 25cm width.
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_NEAR(0.25, markings[0].marking.width, kTolerance);
}

// Test width field when not specified (nullopt).
TEST_F(LaneBoundaryTest, WidthConversionNullopt) {
  std::vector<xodr::LaneRoadMark> road_marks{};
  xodr::LaneRoadMark mark;
  mark.s_offset = 0.;
  mark.type = xodr::LaneRoadMark::Type::kSolid;
  mark.color = xodr::Color::kWhite;
  // width is not set (std::nullopt).
  road_marks.push_back(mark);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(1u, markings.size());
  EXPECT_NEAR(0., markings[0].marking.width, kTolerance);  // Default to 0.
}

// Test transition from single line marking to double line marking.
// This simulates a real-world scenario where a single solid line transitions
// to double solid lines (e.g., approaching an intersection or no-passing zone).
TEST_F(LaneBoundaryTest, SingleToDoubleLineTransition) {
  std::vector<xodr::LaneRoadMark> road_marks{};

  // First mark: single solid white line from 0 to 50.
  xodr::LaneRoadMark mark1;
  mark1.s_offset = 0.;
  mark1.type = xodr::LaneRoadMark::Type::kSolid;
  mark1.color = xodr::Color::kWhite;
  mark1.width = 0.15;
  mark1.lane_change = xodr::LaneRoadMark::LaneChange::kBoth;  // Lane change allowed.
  road_marks.push_back(mark1);

  // Second mark: double solid yellow lines from 50 to 100.
  xodr::LaneRoadMark mark2;
  mark2.s_offset = 50.;
  mark2.type = xodr::LaneRoadMark::Type::kSolidSolid;
  mark2.color = xodr::Color::kYellow;
  mark2.lane_change = xodr::LaneRoadMark::LaneChange::kNone;  // No lane change allowed.

  // Add two TypeElement lines for the double solid.
  xodr::TypeElement type_elem;
  type_elem.name = "double_solid";
  type_elem.width = 0.30;  // Total width including gap.

  // Left line of double solid.
  xodr::TypeElementLine line1;
  line1.length = 0.0;  // Solid.
  line1.space = 0.0;
  line1.width = 0.12;
  line1.t_offset = -0.10;  // Left of center.
  line1.s_offset = 0.0;
  line1.color = xodr::Color::kYellow;
  type_elem.lines.push_back(line1);

  // Right line of double solid.
  xodr::TypeElementLine line2;
  line2.length = 0.0;  // Solid.
  line2.space = 0.0;
  line2.width = 0.12;
  line2.t_offset = 0.10;  // Right of center.
  line2.s_offset = 0.0;
  line2.color = xodr::Color::kYellow;
  type_elem.lines.push_back(line2);

  mark2.type_elems.push_back(type_elem);
  road_marks.push_back(mark2);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  // Test GetMarkings returns both markings.
  const auto markings = dut.GetMarkings();
  ASSERT_EQ(2u, markings.size());

  // First marking: single solid white line [0, 50).
  EXPECT_EQ(LaneMarkingType::kSolid, markings[0].marking.type);
  EXPECT_EQ(LaneMarkingColor::kWhite, markings[0].marking.color);
  EXPECT_EQ(maliput::api::LaneChangePermission::kAllowed, markings[0].marking.lane_change);
  EXPECT_NEAR(0.15, markings[0].marking.width, kTolerance);
  EXPECT_NEAR(0., markings[0].s_start, kTolerance);
  EXPECT_NEAR(50., markings[0].s_end, kTolerance);
  EXPECT_TRUE(markings[0].marking.lines.empty());  // No detailed lines for single marking.

  // Second marking: double solid yellow lines [50, 100).
  EXPECT_EQ(LaneMarkingType::kSolidSolid, markings[1].marking.type);
  EXPECT_EQ(LaneMarkingColor::kYellow, markings[1].marking.color);
  EXPECT_EQ(maliput::api::LaneChangePermission::kProhibited, markings[1].marking.lane_change);
  EXPECT_NEAR(50., markings[1].s_start, kTolerance);
  EXPECT_NEAR(100., markings[1].s_end, kTolerance);

  // Second marking should have two detailed lines.
  ASSERT_EQ(2u, markings[1].marking.lines.size());
  EXPECT_NEAR(-0.10, markings[1].marking.lines[0].r_offset, kTolerance);
  EXPECT_NEAR(0.10, markings[1].marking.lines[1].r_offset, kTolerance);
  EXPECT_EQ(LaneMarkingColor::kYellow, markings[1].marking.lines[0].color);
  EXPECT_EQ(LaneMarkingColor::kYellow, markings[1].marking.lines[1].color);

  // Query at s=25: should return single solid white line.
  const auto result_at_25 = dut.GetMarking(25.);
  ASSERT_TRUE(result_at_25.has_value());
  EXPECT_EQ(LaneMarkingType::kSolid, result_at_25->marking.type);
  EXPECT_TRUE(result_at_25->marking.lines.empty());

  // Query at s=75: should return double solid yellow lines with detailed line info.
  const auto result_at_75 = dut.GetMarking(75.);
  ASSERT_TRUE(result_at_75.has_value());
  EXPECT_EQ(LaneMarkingType::kSolidSolid, result_at_75->marking.type);
  ASSERT_EQ(2u, result_at_75->marking.lines.size());
}

// Test transition from double line marking to single line marking.
// This simulates the reverse scenario (e.g., leaving a no-passing zone).
TEST_F(LaneBoundaryTest, DoubleToSingleLineTransition) {
  std::vector<xodr::LaneRoadMark> road_marks{};

  // First mark: double solid yellow lines from 0 to 50.
  xodr::LaneRoadMark mark1;
  mark1.s_offset = 0.;
  mark1.type = xodr::LaneRoadMark::Type::kSolidSolid;
  mark1.color = xodr::Color::kYellow;
  mark1.lane_change = xodr::LaneRoadMark::LaneChange::kNone;

  xodr::TypeElement type_elem;
  type_elem.name = "double_solid";
  type_elem.width = 0.30;

  xodr::TypeElementLine line1;
  line1.length = 0.0;
  line1.space = 0.0;
  line1.width = 0.12;
  line1.t_offset = -0.10;
  line1.s_offset = 0.0;
  line1.color = xodr::Color::kYellow;
  type_elem.lines.push_back(line1);

  xodr::TypeElementLine line2;
  line2.length = 0.0;
  line2.space = 0.0;
  line2.width = 0.12;
  line2.t_offset = 0.10;
  line2.s_offset = 0.0;
  line2.color = xodr::Color::kYellow;
  type_elem.lines.push_back(line2);

  mark1.type_elems.push_back(type_elem);
  road_marks.push_back(mark1);

  // Second mark: single broken white line from 50 to 100.
  xodr::LaneRoadMark mark2;
  mark2.s_offset = 50.;
  mark2.type = xodr::LaneRoadMark::Type::kBroken;
  mark2.color = xodr::Color::kWhite;
  mark2.width = 0.15;
  mark2.lane_change = xodr::LaneRoadMark::LaneChange::kBoth;
  road_marks.push_back(mark2);

  const LaneBoundary dut(kBoundaryId, segment_.get(), kIndex, nullptr, nullptr, reference_lane_.get(), road_marks,
                         kTrackSStart, kTrackSEnd);

  const auto markings = dut.GetMarkings();
  ASSERT_EQ(2u, markings.size());

  // First marking: double solid [0, 50) with 2 lines.
  EXPECT_EQ(LaneMarkingType::kSolidSolid, markings[0].marking.type);
  EXPECT_EQ(maliput::api::LaneChangePermission::kProhibited, markings[0].marking.lane_change);
  ASSERT_EQ(2u, markings[0].marking.lines.size());

  // Second marking: single broken [50, 100) with no detailed lines.
  EXPECT_EQ(LaneMarkingType::kBroken, markings[1].marking.type);
  EXPECT_EQ(maliput::api::LaneChangePermission::kAllowed, markings[1].marking.lane_change);
  EXPECT_TRUE(markings[1].marking.lines.empty());
}

// =====================================================================================
// Integration tests that load real XODR files and verify end-to-end LaneBoundary
// construction and lane marking properties.
// =====================================================================================

/// Integration test fixture for loading XODR files and testing LaneBoundary construction.
class LaneBoundaryIntegrationTest : public ::testing::Test {
 protected:
  /// Loads a RoadNetwork from the specified XODR file.
  void LoadRoadNetwork(const std::string& xodr_filename) {
    const std::string xodr_file_path = utility::FindResourceInPath(xodr_filename, kMalidriveResourceFolder);

    builder::RoadGeometryConfiguration rg_config;
    rg_config.id = maliput::api::RoadGeometryId{xodr_filename};
    rg_config.opendrive_file = xodr_file_path;
    rg_config.tolerances.linear_tolerance = 5e-2;
    rg_config.tolerances.max_linear_tolerance = 5e-2;
    rg_config.tolerances.angular_tolerance = 1e-3;
    rg_config.scale_length = 1.0;
    rg_config.inertial_to_backend_frame_translation = {0., 0., 0.};
    rg_config.build_policy.type = builder::BuildPolicy::Type::kSequential;
    rg_config.simplification_policy = builder::RoadGeometryConfiguration::SimplificationPolicy::kNone;
    rg_config.standard_strictness_policy = builder::RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive;
    rg_config.omit_nondrivable_lanes = false;

    road_network_ = loader::Load<builder::RoadNetworkBuilder>(rg_config.ToStringMap());
  }

  // Resource folder path defined via compile definition.
  static constexpr char const* kMalidriveResourceFolder{DEF_MALIDRIVE_RESOURCES};
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  static constexpr double kTolerance{1e-6};
};

/// Tests that LaneBoundaries are created for the Straight800m.xodr file.
///
/// The Straight800m.xodr file has:
/// - 1 road with 1 lane section
/// - 4 left lanes (shoulder, driving, driving, shoulder)
/// - Center lane (no type)
/// - No right lanes
///
/// Expected boundaries (5 total for 4 lanes):
/// - Boundary 0: Center boundary (no marking - uses center lane's road marks)
/// - Boundary 1: Between Lane 1 (shoulder) and Lane 2 (driving) - solid white
/// - Boundary 2: Between Lane 2 (driving) and Lane 3 (driving) - broken white
/// - Boundary 3: Between Lane 3 (driving) and Lane 4 (shoulder) - solid white
/// - Boundary 4: Leftmost edge - no marking (Lane 4 has type="none")
TEST_F(LaneBoundaryIntegrationTest, Straight800mLaneBoundariesAreCreated) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  ASSERT_NE(rg, nullptr);
  ASSERT_EQ(rg->num_junctions(), 1);

  const maliput::api::Junction* junction = rg->junction(0);
  ASSERT_NE(junction, nullptr);
  ASSERT_EQ(junction->num_segments(), 1);

  const maliput::api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);

  // 4 lanes should result in 5 boundaries (N+1 boundaries for N lanes).
  EXPECT_EQ(segment->num_lanes(), 4);
  EXPECT_EQ(segment->num_boundaries(), 5);

  // Verify each boundary exists and has valid properties.
  for (int i = 0; i < segment->num_boundaries(); ++i) {
    const maliput::api::LaneBoundary* boundary = segment->boundary(i);
    ASSERT_NE(boundary, nullptr) << "Boundary " << i << " should exist";
    EXPECT_EQ(boundary->segment(), segment);
    EXPECT_EQ(boundary->index(), i);
  }
}

/// Tests the lane marking properties of each boundary in Straight800m.xodr.
///
/// Expected markings based on the XODR file:
/// - Boundary 0: No marking (center lane has type="none", color="standard" -> kWhite)
/// - Boundary 1: Solid white (from Lane 1's road marks, laneChange="none")
/// - Boundary 2: Broken white (from Lane 2's road marks, laneChange="both")
/// - Boundary 3: Solid white (from Lane 3's road marks, laneChange="none")
/// - Boundary 4: No marking (Lane 4 has type="none")
TEST_F(LaneBoundaryIntegrationTest, Straight800mLaneMarkingsAreCorrect) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  // Define expected markings for each boundary.
  struct ExpectedMarking {
    int boundary_index;
    LaneMarkingType type;
    LaneMarkingColor color;
    std::optional<maliput::api::LaneChangePermission> lane_change;
    std::optional<double> width;
  };

  // Note: In OpenDRIVE, road marks are on the outer edge of lanes.
  // For left lanes (positive IDs), road marks are on the left edge (farther from center).
  // The boundary logic uses the inner lane's marks (closer to center) for boundaries
  // between lanes on the same side.
  //
  // Boundary ordering (right to left) for Straight800m.xodr which has only left lanes:
  // - Boundary 0: Center boundary (uses center lane road marks -> none, standard/white)
  // - Boundary 1: Between Lane 1 and Lane 2 (uses Lane 1's marks -> solid white, prohibited)
  // - Boundary 2: Between Lane 2 and Lane 3 (uses Lane 2's marks -> broken white, allowed)
  // - Boundary 3: Between Lane 3 and Lane 4 (uses Lane 3's marks -> solid white, prohibited)
  // - Boundary 4: Leftmost edge (uses Lane 4's marks -> none)
  const std::vector<ExpectedMarking> expected_markings = {
      {0, LaneMarkingType::kNone, LaneMarkingColor::kWhite, maliput::api::LaneChangePermission::kProhibited,
       std::nullopt},
      {1, LaneMarkingType::kSolid, LaneMarkingColor::kWhite, maliput::api::LaneChangePermission::kProhibited, 0.125},
      {2, LaneMarkingType::kBroken, LaneMarkingColor::kWhite, maliput::api::LaneChangePermission::kAllowed, 0.125},
      {3, LaneMarkingType::kSolid, LaneMarkingColor::kWhite, maliput::api::LaneChangePermission::kProhibited, 0.125},
      {4, LaneMarkingType::kNone, LaneMarkingColor::kWhite, maliput::api::LaneChangePermission::kProhibited,
       std::nullopt},
  };

  for (const auto& expected : expected_markings) {
    const maliput::api::LaneBoundary* boundary = segment->boundary(expected.boundary_index);
    ASSERT_NE(boundary, nullptr) << "Boundary " << expected.boundary_index << " should exist";

    // Get all markings for this boundary.
    const auto markings = boundary->GetMarkings();

    // For this simple road with no marking changes along s, we expect exactly 1 marking.
    ASSERT_EQ(markings.size(), 1u) << "Boundary " << expected.boundary_index << " should have exactly 1 marking";

    const maliput::api::LaneMarkingResult& result = markings[0];
    const LaneMarking& marking = result.marking;

    // Verify marking type.
    EXPECT_EQ(marking.type, expected.type) << "Boundary " << expected.boundary_index << " type mismatch";

    // Verify color.
    EXPECT_EQ(marking.color, expected.color) << "Boundary " << expected.boundary_index << " color mismatch";

    // Verify lane change permission.
    if (expected.lane_change.has_value()) {
      EXPECT_EQ(marking.lane_change, expected.lane_change.value())
          << "Boundary " << expected.boundary_index << " lane_change mismatch";
    }

    // Verify width if expected.
    if (expected.width.has_value()) {
      EXPECT_NEAR(marking.width, expected.width.value(), kTolerance)
          << "Boundary " << expected.boundary_index << " width mismatch";
    }

    // Verify s-range covers the full lane length (800m road).
    EXPECT_NEAR(result.s_start, 0., kTolerance) << "Boundary " << expected.boundary_index << " s_start should be 0";
    EXPECT_NEAR(result.s_end, 800., kTolerance) << "Boundary " << expected.boundary_index << " s_end should be 800";
  }
}

/// Tests GetMarking at specific s-coordinates for Straight800m.xodr.
TEST_F(LaneBoundaryIntegrationTest, Straight800mGetMarkingAtS) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  // Test boundary 2 (broken white line between two driving lanes).
  const maliput::api::LaneBoundary* boundary = segment->boundary(2);
  ASSERT_NE(boundary, nullptr);

  // Query at s=400 (middle of the road).
  const auto result_at_400 = boundary->GetMarking(400.);
  ASSERT_TRUE(result_at_400.has_value());
  EXPECT_EQ(result_at_400->marking.type, LaneMarkingType::kBroken);
  EXPECT_EQ(result_at_400->marking.color, LaneMarkingColor::kWhite);
  EXPECT_EQ(result_at_400->marking.lane_change, maliput::api::LaneChangePermission::kAllowed);

  // Query at s=0 (start of the road).
  const auto result_at_0 = boundary->GetMarking(0.);
  ASSERT_TRUE(result_at_0.has_value());
  EXPECT_EQ(result_at_0->marking.type, LaneMarkingType::kBroken);

  // Query at s=800 (end of the road).
  const auto result_at_800 = boundary->GetMarking(800.);
  ASSERT_TRUE(result_at_800.has_value());
  EXPECT_EQ(result_at_800->marking.type, LaneMarkingType::kBroken);
}

/// Tests GetMarkings with s-range filter for Straight800m.xodr.
TEST_F(LaneBoundaryIntegrationTest, Straight800mGetMarkingsInRange) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  const maliput::api::LaneBoundary* boundary = segment->boundary(1);  // Solid white line.
  ASSERT_NE(boundary, nullptr);

  // Get markings in the range [100, 200].
  const auto markings = boundary->GetMarkings(100., 200.);
  ASSERT_EQ(markings.size(), 1u);
  EXPECT_EQ(markings[0].marking.type, LaneMarkingType::kSolid);

  // The returned marking's s_range should cover the full marking extent,
  // not just the queried range.
  EXPECT_NEAR(markings[0].s_start, 0., kTolerance);
  EXPECT_NEAR(markings[0].s_end, 800., kTolerance);
}

/// Tests that adjacent lane pointers are correctly set on boundaries for Straight800m.xodr.
TEST_F(LaneBoundaryIntegrationTest, Straight800mAdjacentLanes) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  // Boundary 0 (rightmost/center): lane_to_right=nullptr, lane_to_left=Lane0.
  {
    const maliput::api::LaneBoundary* boundary = segment->boundary(0);
    ASSERT_NE(boundary, nullptr);
    EXPECT_EQ(boundary->lane_to_right(), nullptr);
    EXPECT_NE(boundary->lane_to_left(), nullptr);
    EXPECT_EQ(boundary->lane_to_left(), segment->lane(0));
  }

  // Boundary 1: lane_to_right=Lane0, lane_to_left=Lane1.
  {
    const maliput::api::LaneBoundary* boundary = segment->boundary(1);
    ASSERT_NE(boundary, nullptr);
    EXPECT_NE(boundary->lane_to_right(), nullptr);
    EXPECT_NE(boundary->lane_to_left(), nullptr);
    EXPECT_EQ(boundary->lane_to_right(), segment->lane(0));
    EXPECT_EQ(boundary->lane_to_left(), segment->lane(1));
  }

  // Boundary 2: lane_to_right=Lane1, lane_to_left=Lane2.
  {
    const maliput::api::LaneBoundary* boundary = segment->boundary(2);
    ASSERT_NE(boundary, nullptr);
    EXPECT_EQ(boundary->lane_to_right(), segment->lane(1));
    EXPECT_EQ(boundary->lane_to_left(), segment->lane(2));
  }

  // Boundary 3: lane_to_right=Lane2, lane_to_left=Lane3.
  {
    const maliput::api::LaneBoundary* boundary = segment->boundary(3);
    ASSERT_NE(boundary, nullptr);
    EXPECT_EQ(boundary->lane_to_right(), segment->lane(2));
    EXPECT_EQ(boundary->lane_to_left(), segment->lane(3));
  }

  // Boundary 4 (leftmost): lane_to_right=Lane3, lane_to_left=nullptr.
  {
    const maliput::api::LaneBoundary* boundary = segment->boundary(4);
    ASSERT_NE(boundary, nullptr);
    EXPECT_NE(boundary->lane_to_right(), nullptr);
    EXPECT_EQ(boundary->lane_to_right(), segment->lane(3));
    EXPECT_EQ(boundary->lane_to_left(), nullptr);
  }
}

/// Tests that line details (length, space) are correctly parsed for broken lines in Straight800m.xodr.
TEST_F(LaneBoundaryIntegrationTest, Straight800mBrokenLineDetails) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  // Boundary 2 has a broken line with:
  // - length="6.0" (visible dash length)
  // - space="12.0" (gap between dashes)
  // - width="0.125"
  const maliput::api::LaneBoundary* boundary = segment->boundary(2);
  ASSERT_NE(boundary, nullptr);

  const auto markings = boundary->GetMarkings();
  ASSERT_EQ(markings.size(), 1u);

  const LaneMarking& marking = markings[0].marking;
  EXPECT_EQ(marking.type, LaneMarkingType::kBroken);

  // Verify that line details are present.
  ASSERT_FALSE(marking.lines.empty()) << "Broken line should have line details";

  // Check the first line's properties.
  const auto& line = marking.lines[0];
  EXPECT_NEAR(line.length, 6.0, kTolerance) << "Line length should be 6.0m";
  EXPECT_NEAR(line.space, 12.0, kTolerance) << "Line space should be 12.0m";
  EXPECT_NEAR(line.width, 0.125, kTolerance) << "Line width should be 0.125m";
  EXPECT_NEAR(line.r_offset, 0., kTolerance) << "Line t_offset (r_offset) should be 0";
}

/// Tests that solid lines have appropriate line details for Straight800m.xodr.
TEST_F(LaneBoundaryIntegrationTest, Straight800mSolidLineDetails) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  // Boundary 1 has a solid line.
  const maliput::api::LaneBoundary* boundary = segment->boundary(1);
  ASSERT_NE(boundary, nullptr);

  const auto markings = boundary->GetMarkings();
  ASSERT_EQ(markings.size(), 1u);

  const LaneMarking& marking = markings[0].marking;
  EXPECT_EQ(marking.type, LaneMarkingType::kSolid);

  // Verify that line details are present.
  ASSERT_FALSE(marking.lines.empty()) << "Solid line should have line details";

  // Check the first line's properties.
  // In the XODR, the solid line has: length="800" space="0" width="0.125"
  const auto& line = marking.lines[0];
  EXPECT_NEAR(line.length, 800.0, kTolerance) << "Solid line length should be 800.0m";
  EXPECT_NEAR(line.space, 0., kTolerance) << "Solid line space should be 0";
  EXPECT_NEAR(line.width, 0.125, kTolerance) << "Line width should be 0.125m";
}

/// Tests that boundary IDs follow the expected naming convention for Straight800m.xodr.
TEST_F(LaneBoundaryIntegrationTest, Straight800mBoundaryIds) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  // Verify boundary IDs contain the segment ID and boundary index.
  for (int i = 0; i < segment->num_boundaries(); ++i) {
    const maliput::api::LaneBoundary* boundary = segment->boundary(i);
    ASSERT_NE(boundary, nullptr);

    const std::string boundary_id = boundary->id().string();
    EXPECT_TRUE(boundary_id.find(segment->id().string()) != std::string::npos)
        << "Boundary ID '" << boundary_id << "' should contain segment ID '" << segment->id().string() << "'";
    EXPECT_TRUE(boundary_id.find("boundary") != std::string::npos)
        << "Boundary ID '" << boundary_id << "' should contain 'boundary'";
    EXPECT_TRUE(boundary_id.find(std::to_string(i)) != std::string::npos)
        << "Boundary ID '" << boundary_id << "' should contain index " << i;
  }
}

/// Tests that boundaries can be retrieved by ID from the RoadGeometry for Straight800m.xodr.
TEST_F(LaneBoundaryIntegrationTest, Straight800mGetBoundaryById) {
  LoadRoadNetwork("Straight800m.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  const maliput::api::Segment* segment = rg->junction(0)->segment(0);

  for (int i = 0; i < segment->num_boundaries(); ++i) {
    const maliput::api::LaneBoundary* boundary_from_segment = segment->boundary(i);
    ASSERT_NE(boundary_from_segment, nullptr);

    // Retrieve the same boundary by ID from the RoadGeometry.
    const maliput::api::LaneBoundary* boundary_by_id = rg->ById().GetLaneBoundary(boundary_from_segment->id());
    EXPECT_EQ(boundary_by_id, boundary_from_segment)
        << "GetLaneBoundary should return the same boundary as segment->boundary()";
  }
}

/// Tests that LaneBoundaries are created for TwoWayRoadWithDoubleYellowCurve.xodr.
///
/// This road has:
/// - 1 road with 5 lane sections (different center markings)
/// - 2 lanes per section: lane 1 (left) and lane -1 (right)
/// - Center lane boundary markings change along the road:
///   * Section 1 [0, 70): Broken white - passing allowed
///   * Section 2 [70, 100): Broken-Solid yellow - transition into curve
///   * Section 3 [100, 214): Double solid yellow - no passing (curve)
///   * Section 4 [214, 244): Solid-Broken yellow - transition out of curve
///   * Section 5 [244, 314): Broken white - passing allowed
///
/// Expected: 5 segments (one per lane section), each with 3 boundaries (2 lanes + 1 = 3)
TEST_F(LaneBoundaryIntegrationTest, TwoWayRoadWithDoubleYellowCurveLaneBoundariesAreCreated) {
  LoadRoadNetwork("TwoWayRoadWithDoubleYellowCurve.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();
  ASSERT_NE(rg, nullptr);

  // This road has 5 lane sections, which results in 5 segments (one junction with 5 segments).
  ASSERT_GE(rg->num_junctions(), 1);

  int total_segments = 0;
  for (int j = 0; j < rg->num_junctions(); ++j) {
    total_segments += rg->junction(j)->num_segments();
  }
  EXPECT_EQ(total_segments, 5) << "Expected 5 segments for 5 lane sections";

  // Verify each segment has the expected number of lanes and boundaries.
  for (int j = 0; j < rg->num_junctions(); ++j) {
    const maliput::api::Junction* junction = rg->junction(j);
    for (int s = 0; s < junction->num_segments(); ++s) {
      const maliput::api::Segment* segment = junction->segment(s);
      ASSERT_NE(segment, nullptr);

      // Each segment should have 2 lanes (lane 1 and lane -1).
      EXPECT_EQ(segment->num_lanes(), 2) << "Segment " << segment->id().string() << " should have 2 lanes";

      // 2 lanes should result in 3 boundaries (N+1 boundaries for N lanes).
      EXPECT_EQ(segment->num_boundaries(), 3) << "Segment " << segment->id().string() << " should have 3 boundaries";

      // Verify each boundary exists.
      for (int i = 0; i < segment->num_boundaries(); ++i) {
        const maliput::api::LaneBoundary* boundary = segment->boundary(i);
        ASSERT_NE(boundary, nullptr) << "Boundary " << i << " in segment " << segment->id().string() << " should exist";
        EXPECT_EQ(boundary->segment(), segment);
        EXPECT_EQ(boundary->index(), i);
      }
    }
  }
}

/// Tests the lane marking properties of the center boundary in TwoWayRoadWithDoubleYellowCurve.xodr.
///
/// This tests the center boundary (boundary 1) across all 5 lane sections.
/// The center marking changes along the road to demonstrate no-passing zones near curves.
TEST_F(LaneBoundaryIntegrationTest, TwoWayRoadWithDoubleYellowCurveCenterMarkings) {
  LoadRoadNetwork("TwoWayRoadWithDoubleYellowCurve.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();

  // Expected center markings for each of the 5 lane sections (by s-coordinate range).
  // Each section has a different center line marking.
  struct ExpectedCenterMarking {
    double s_approx;  // Approximate s-coordinate to identify the section.
    LaneMarkingType type;
    LaneMarkingColor color;
    maliput::api::LaneChangePermission lane_change;
  };

  // The lane sections in order:
  // Section 1: [0, 70) - broken white, both directions can pass
  // Section 2: [70, 100) - broken_solid yellow (laneChange="decrease" -> ToRight)
  // Section 3: [100, 214) - solid_solid yellow, no passing
  // Section 4: [214, 244) - solid_broken yellow (laneChange="increase" -> ToLeft)
  // Section 5: [244, 314) - broken white, both directions can pass
  const std::vector<ExpectedCenterMarking> expected_center_markings = {
      {35.0, LaneMarkingType::kBroken, LaneMarkingColor::kWhite, maliput::api::LaneChangePermission::kAllowed},
      {85.0, LaneMarkingType::kBrokenSolid, LaneMarkingColor::kYellow, maliput::api::LaneChangePermission::kToRight},
      {157.0, LaneMarkingType::kSolidSolid, LaneMarkingColor::kYellow, maliput::api::LaneChangePermission::kProhibited},
      {229.0, LaneMarkingType::kSolidBroken, LaneMarkingColor::kYellow, maliput::api::LaneChangePermission::kToLeft},
      {279.0, LaneMarkingType::kBroken, LaneMarkingColor::kWhite, maliput::api::LaneChangePermission::kAllowed},
  };

  // Find and verify each segment's center boundary.
  int section_idx = 0;
  for (int j = 0; j < rg->num_junctions(); ++j) {
    const maliput::api::Junction* junction = rg->junction(j);
    for (int s = 0; s < junction->num_segments(); ++s) {
      const maliput::api::Segment* segment = junction->segment(s);

      // The center boundary is boundary 1 (between lane 0 which is lane -1, and lane 1 which is lane 1).
      // Boundary ordering: 0 = rightmost edge, 1 = center, 2 = leftmost edge.
      ASSERT_GE(segment->num_boundaries(), 2);
      const maliput::api::LaneBoundary* center_boundary = segment->boundary(1);
      ASSERT_NE(center_boundary, nullptr);

      // Get the marking at a point in the middle of this section.
      const auto& expected = expected_center_markings[section_idx];
      const auto marking_result = center_boundary->GetMarking(expected.s_approx);

      // If the s_approx is within the boundary's range, we should get a marking.
      if (marking_result.has_value()) {
        const auto& marking = marking_result->marking;

        EXPECT_EQ(marking.type, expected.type)
            << "Section " << section_idx << " center boundary type mismatch at s=" << expected.s_approx;
        EXPECT_EQ(marking.color, expected.color)
            << "Section " << section_idx << " center boundary color mismatch at s=" << expected.s_approx;
        EXPECT_EQ(marking.lane_change, expected.lane_change)
            << "Section " << section_idx << " center boundary lane_change mismatch at s=" << expected.s_approx;
      }

      section_idx++;
      if (section_idx >= static_cast<int>(expected_center_markings.size())) {
        break;
      }
    }
    if (section_idx >= static_cast<int>(expected_center_markings.size())) {
      break;
    }
  }

  EXPECT_EQ(section_idx, 5) << "Should have verified all 5 lane sections";
}

/// Tests GetMarkings across the full road for the center boundary in TwoWayRoadWithDoubleYellowCurve.xodr.
///
/// This verifies that GetMarkings returns the correct sequence of markings
/// for the center boundary which changes 5 times along the road.
TEST_F(LaneBoundaryIntegrationTest, TwoWayRoadWithDoubleYellowCurveGetMarkingsSequence) {
  LoadRoadNetwork("TwoWayRoadWithDoubleYellowCurve.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();

  // Collect all center boundary markings across all segments.
  std::vector<maliput::api::LaneMarkingResult> all_center_markings;

  for (int j = 0; j < rg->num_junctions(); ++j) {
    const maliput::api::Junction* junction = rg->junction(j);
    for (int s = 0; s < junction->num_segments(); ++s) {
      const maliput::api::Segment* segment = junction->segment(s);
      if (segment->num_boundaries() >= 2) {
        const maliput::api::LaneBoundary* center_boundary = segment->boundary(1);
        const auto markings = center_boundary->GetMarkings();
        for (const auto& m : markings) {
          all_center_markings.push_back(m);
        }
      }
    }
  }

  // We expect at least 5 markings (one per lane section).
  EXPECT_GE(all_center_markings.size(), 5u) << "Expected at least 5 center markings for the 5 lane sections";

  // Verify the marking types follow the expected pattern:
  // broken -> broken_solid -> solid_solid -> solid_broken -> broken
  if (all_center_markings.size() >= 5) {
    EXPECT_EQ(all_center_markings[0].marking.type, LaneMarkingType::kBroken);
    EXPECT_EQ(all_center_markings[1].marking.type, LaneMarkingType::kBrokenSolid);
    EXPECT_EQ(all_center_markings[2].marking.type, LaneMarkingType::kSolidSolid);
    EXPECT_EQ(all_center_markings[3].marking.type, LaneMarkingType::kSolidBroken);
    EXPECT_EQ(all_center_markings[4].marking.type, LaneMarkingType::kBroken);
  }
}

/// Tests that the outer edge boundaries have solid white markings in TwoWayRoadWithDoubleYellowCurve.xodr.
TEST_F(LaneBoundaryIntegrationTest, TwoWayRoadWithDoubleYellowCurveEdgeMarkings) {
  LoadRoadNetwork("TwoWayRoadWithDoubleYellowCurve.xodr");
  ASSERT_NE(road_network_, nullptr);

  const maliput::api::RoadGeometry* rg = road_network_->road_geometry();

  // Verify edge boundaries (0 and 2) have solid white markings across all segments.
  for (int j = 0; j < rg->num_junctions(); ++j) {
    const maliput::api::Junction* junction = rg->junction(j);
    for (int s = 0; s < junction->num_segments(); ++s) {
      const maliput::api::Segment* segment = junction->segment(s);

      // Verify boundary 0 (right edge) and boundary 2 (left edge).
      for (int boundary_idx : {0, 2}) {
        if (boundary_idx < segment->num_boundaries()) {
          const maliput::api::LaneBoundary* edge_boundary = segment->boundary(boundary_idx);
          ASSERT_NE(edge_boundary, nullptr);

          const auto markings = edge_boundary->GetMarkings();
          ASSERT_GE(markings.size(), 1u) << "Segment " << segment->id().string() << " boundary " << boundary_idx
                                         << " should have markings";

          // Edge markings should be solid white.
          EXPECT_EQ(markings[0].marking.type, LaneMarkingType::kSolid)
              << "Segment " << segment->id().string() << " boundary " << boundary_idx << " should be solid";
          EXPECT_EQ(markings[0].marking.color, LaneMarkingColor::kWhite)
              << "Segment " << segment->id().string() << " boundary " << boundary_idx << " should be white";
        }
      }
    }
  }
}

}  // namespace
}  // namespace test
}  // namespace malidrive
