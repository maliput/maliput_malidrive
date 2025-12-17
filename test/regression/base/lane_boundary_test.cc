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
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane_marking.h>
#include <maliput/common/maliput_throw.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/xodr/lane_road_mark.h"

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
                                             kP0, kP1, kIntegratorAccuracyMultiplier);
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

}  // namespace
}  // namespace test
}  // namespace malidrive
