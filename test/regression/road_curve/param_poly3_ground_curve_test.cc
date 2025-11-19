// BSD 3-Clause License
//
// Copyright (c) 2025, Woven Planet. All rights reserved.
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
#include "maliput_malidrive/road_curve/param_poly3_ground_curve.h"

#include <cmath>

#include <gtest/gtest.h>
#include <maliput/common/error.h>
#include <maliput/math/compare.h>
#include <maliput/math/vector.h>

#include "assert_compare.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

using malidrive::test::AssertCompare;
using maliput::math::CompareVectors;
using maliput::math::Vector2;

// Test fixture for constructor validation tests
class ParamPoly3GroundCurveConstructorTest : public ::testing::Test {
 protected:
  const Vector2 kZero{0., 0.};
  const double kLinearTolerance{1.0};
  const double kHeading{0.};
  const double kArcLength{10.};
  const double kP0{0.};
  const double kP1{10.};
};

TEST_F(ParamPoly3GroundCurveConstructorTest, CorrectlyConstructedArcLength) {
  EXPECT_NO_THROW(ParamPoly3GroundCurve(kLinearTolerance, kZero, kHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength,
                                        kP0, kP1, ParamPoly3GroundCurve::PRangeType::kArcLength));
}

TEST_F(ParamPoly3GroundCurveConstructorTest, CorrectlyConstructedNormalized) {
  EXPECT_NO_THROW(ParamPoly3GroundCurve(kLinearTolerance, kZero, kHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength,
                                        kP0, kP1, ParamPoly3GroundCurve::PRangeType::kNormalized));
}

TEST_F(ParamPoly3GroundCurveConstructorTest, InvalidNegativeTolerance) {
  EXPECT_THROW(ParamPoly3GroundCurve(-1., kZero, kHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength, kP0, kP1,
                                     ParamPoly3GroundCurve::PRangeType::kArcLength),
               maliput::common::road_geometry_construction_error);
}

TEST_F(ParamPoly3GroundCurveConstructorTest, InvalidZeroTolerance) {
  EXPECT_THROW(ParamPoly3GroundCurve(0., kZero, kHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength, kP0, kP1,
                                     ParamPoly3GroundCurve::PRangeType::kArcLength),
               maliput::common::road_geometry_construction_error);
}

TEST_F(ParamPoly3GroundCurveConstructorTest, InvalidArcLengthTooSmall) {
  EXPECT_THROW(ParamPoly3GroundCurve(kLinearTolerance, kZero, kHeading, 0., 1., 0., 0., 0., 0., 0., 0.,
                                     GroundCurve::kEpsilon / 2., kP0, kP1,
                                     ParamPoly3GroundCurve::PRangeType::kArcLength),
               maliput::common::road_geometry_construction_error);
}

TEST_F(ParamPoly3GroundCurveConstructorTest, InvalidNegativeP0) {
  EXPECT_THROW(ParamPoly3GroundCurve(kLinearTolerance, kZero, kHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength,
                                     -0.01, kP1, ParamPoly3GroundCurve::PRangeType::kArcLength),
               maliput::common::road_geometry_construction_error);
}

TEST_F(ParamPoly3GroundCurveConstructorTest, InvalidP1NotSufficientlyLargerThanP0) {
  EXPECT_THROW(ParamPoly3GroundCurve(kLinearTolerance, kZero, kHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength,
                                     kP0, kP0 + GroundCurve::kEpsilon / 2.,
                                     ParamPoly3GroundCurve::PRangeType::kArcLength),
               maliput::common::road_geometry_construction_error);
}

// Test fixture for functional tests with arc length range
class ParamPoly3GroundCurveArcLengthTest : public ::testing::Test {
 public:
  const double kTolerance{1.e-10};
  const double kLinearTolerance{0.01};
  const double kP0{0.};
  const double kP1{10.};
  const double kArcLength{10.};

  // Straight line: u(p) = p, v(p) = 0
  // Should behave like a horizontal line
  const Vector2 kStraightXY0{0., 0.};
  const double kStraightHeading{0.};
  std::unique_ptr<ParamPoly3GroundCurve> straight_dut_;

  // Parabolic curve: u(p) = p, v(p) = 0.1*p^2
  // Curve that deviates laterally with a parabolic profile
  const Vector2 kParabolicXY0{0., 0.};
  const double kParabolicHeading{0.};
  std::unique_ptr<ParamPoly3GroundCurve> parabolic_dut_;

  // Curve starting at different position and heading
  const Vector2 kOffsetXY0{5., 3.};
  const double kOffsetHeading{M_PI / 4.};
  std::unique_ptr<ParamPoly3GroundCurve> offset_dut_;

  // S-curve: u(p) = p, v(p) = p^3 - 1.5*p^2
  const Vector2 kSCurveXY0{0., 0.};
  const double kSCurveHeading{0.};
  std::unique_ptr<ParamPoly3GroundCurve> s_curve_dut_;

 protected:
  void SetUp() override {
    // Straight line: aU=0, bU=1, cU=0, dU=0, aV=0, bV=0, cV=0, dV=0
    straight_dut_ = std::make_unique<ParamPoly3GroundCurve>(
        kLinearTolerance, kStraightXY0, kStraightHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength, kP0, kP1,
        ParamPoly3GroundCurve::PRangeType::kArcLength);

    // Parabolic: aU=0, bU=1, cU=0, dU=0, aV=0, bV=0, cV=0.1, dV=0
    parabolic_dut_ = std::make_unique<ParamPoly3GroundCurve>(
        kLinearTolerance, kParabolicXY0, kParabolicHeading, 0., 1., 0., 0., 0., 0., 0.1, 0., kArcLength, kP0, kP1,
        ParamPoly3GroundCurve::PRangeType::kArcLength);

    // Offset curve with same polynomial as straight but different start position and heading
    offset_dut_ = std::make_unique<ParamPoly3GroundCurve>(
        kLinearTolerance, kOffsetXY0, kOffsetHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength, kP0, kP1,
        ParamPoly3GroundCurve::PRangeType::kArcLength);

    // S-curve: aU=0, bU=1, cU=0, dU=0, aV=0, bV=0, cV=-1.5, dV=1.0
    s_curve_dut_ = std::make_unique<ParamPoly3GroundCurve>(
        kLinearTolerance, kSCurveXY0, kSCurveHeading, 0., 1., 0., 0., 0., 0., -1.5, 1.0, kArcLength, kP0, kP1,
        ParamPoly3GroundCurve::PRangeType::kArcLength);
  }
};

TEST_F(ParamPoly3GroundCurveArcLengthTest, linear_tolerance) {
  EXPECT_NEAR(kLinearTolerance, straight_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, parabolic_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, offset_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, s_curve_dut_->linear_tolerance(), kTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, p0) {
  EXPECT_NEAR(kP0, straight_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, parabolic_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, offset_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, s_curve_dut_->p0(), kTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, p1) {
  EXPECT_NEAR(kP1, straight_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, parabolic_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, offset_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, s_curve_dut_->p1(), kTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, ArcLength) {
  EXPECT_NEAR(kArcLength, straight_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(kArcLength, parabolic_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(kArcLength, offset_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(kArcLength, s_curve_dut_->ArcLength(), kTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, IsG1Contiguous) {
  EXPECT_TRUE(straight_dut_->IsG1Contiguous());
  EXPECT_TRUE(parabolic_dut_->IsG1Contiguous());
  EXPECT_TRUE(offset_dut_->IsG1Contiguous());
  EXPECT_TRUE(s_curve_dut_->IsG1Contiguous());
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, StraightLineG) {
  // For a straight line u(p) = p, v(p) = 0, the curve should be a horizontal line
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{0., 0.}, straight_dut_->G(kP0), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{5., 0.}, straight_dut_->G(5.), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{10., 0.}, straight_dut_->G(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, StraightLineGDot) {
  // For u(p) = p, v(p) = 0, du/dp = 1, dv/dp = 0
  // In arc length range: p_norm = p, so du/dp_norm = 1, dv/dp_norm = 0
  // GDot should be constant (1, 0) in local coords, same in global for heading=0
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, straight_dut_->GDot(kP0), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, straight_dut_->GDot(5.), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, straight_dut_->GDot(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, StraightLineHeading) {
  // Heading should be 0 (horizontal) throughout
  EXPECT_NEAR(0., straight_dut_->Heading(kP0), kTolerance);
  EXPECT_NEAR(0., straight_dut_->Heading(5.), kTolerance);
  EXPECT_NEAR(0., straight_dut_->Heading(kP1), kTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, StraightLineHeadingDot) {
  // For a straight line, heading is constant, so HeadingDot should be 0
  EXPECT_NEAR(0., straight_dut_->HeadingDot(kP0), kTolerance);
  EXPECT_NEAR(0., straight_dut_->HeadingDot(5.), kTolerance);
  EXPECT_NEAR(0., straight_dut_->HeadingDot(kP1), kTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, ParabolicCurveG) {
  // u(p) = p, v(p) = 0.1*p^2
  // At p=0: (u,v) = (0, 0) -> (x,y) = (0, 0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{0., 0.}, parabolic_dut_->G(kP0), kTolerance)));
  // At p=5: (u,v) = (5, 2.5) -> (x,y) = (5, 2.5)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{5., 2.5}, parabolic_dut_->G(5.), kTolerance)));
  // At p=10: (u,v) = (10, 10) -> (x,y) = (10, 10)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{10., 10.}, parabolic_dut_->G(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, ParabolicCurveGDot) {
  // u(p) = p, v(p) = 0.1*p^2
  // du/dp = 1, dv/dp = 0.2*p
  // At p=0: GDot in local = (1, 0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, parabolic_dut_->GDot(kP0), kTolerance)));
  // At p=5: GDot in local = (1, 1.0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 1.0}, parabolic_dut_->GDot(5.), kTolerance)));
  // At p=10: GDot in local = (1, 2.0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 2.0}, parabolic_dut_->GDot(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, OffsetCurveG) {
  // Same polynomial as straight line but rotated by 45 degrees and offset
  // At p=0: (u,v) = (0, 0) -> rotated and translated
  const double cos45 = std::cos(M_PI / 4.);
  const double sin45 = std::sin(M_PI / 4.);
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{5., 3.}, offset_dut_->G(kP0), kTolerance)));
  // At p=10: (u,v) = (10, 0) -> rotated by 45deg and translated
  const Vector2 expected_p1{5. + 10. * cos45, 3. + 10. * sin45};
  EXPECT_TRUE(AssertCompare(CompareVectors(expected_p1, offset_dut_->G(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, SCurveG) {
  // u(p) = p, v(p) = p^3 - 1.5*p^2
  // At p=0: (u,v) = (0, 0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{0., 0.}, s_curve_dut_->G(kP0), kTolerance)));
  // At p=5: (u,v) = (5, 125 - 37.5) = (5, 87.5)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{5., 87.5}, s_curve_dut_->G(5.), kTolerance)));
  // At p=10: (u,v) = (10, 1000 - 150) = (10, 850)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{10., 850.}, s_curve_dut_->G(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, GInverseStraightLine) {
  // Test points on the curve
  EXPECT_NEAR(kP0, straight_dut_->GInverse(Vector2{0., 0.}), kLinearTolerance);
  EXPECT_NEAR(5., straight_dut_->GInverse(Vector2{5., 0.}), kLinearTolerance);
  EXPECT_NEAR(kP1, straight_dut_->GInverse(Vector2{10., 0.}), kLinearTolerance);

  // Test points off the curve (should return nearest point projection)
  EXPECT_NEAR(5., straight_dut_->GInverse(Vector2{5., 1.}), kLinearTolerance);
  EXPECT_NEAR(5., straight_dut_->GInverse(Vector2{5., -1.}), kLinearTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, GInverseParabolicCurve) {
  // Test points on the curve
  EXPECT_NEAR(kP0, parabolic_dut_->GInverse(Vector2{0., 0.}), kLinearTolerance);
  EXPECT_NEAR(kP1, parabolic_dut_->GInverse(Vector2{10., 10.}), kLinearTolerance);
}

TEST_F(ParamPoly3GroundCurveArcLengthTest, ThrowsOnInvalidP) {
  // Test that methods throw when p is outside valid range
  EXPECT_THROW(straight_dut_->G(kP0 - 0.02), maliput::common::road_geometry_construction_error);
  EXPECT_THROW(straight_dut_->G(kP1 + 0.02), maliput::common::road_geometry_construction_error);
  EXPECT_THROW(straight_dut_->GDot(kP0 - 0.02), maliput::common::road_geometry_construction_error);
  EXPECT_THROW(straight_dut_->Heading(kP0 - 0.02), maliput::common::road_geometry_construction_error);
  EXPECT_THROW(straight_dut_->HeadingDot(kP0 - 0.02), maliput::common::road_geometry_construction_error);
}

// Test fixture for functional tests with normalized range
class ParamPoly3GroundCurveNormalizedTest : public ::testing::Test {
 public:
  const double kTolerance{1.e-12};
  const double kLinearTolerance{0.01};
  const double kP0{0.};
  const double kP1{1.};
  const double kArcLength{10.};

  // Straight line with normalized range
  const Vector2 kStraightXY0{0., 0.};
  const double kStraightHeading{0.};
  std::unique_ptr<ParamPoly3GroundCurve> straight_normalized_dut_;

  // Parabolic curve with normalized range
  const Vector2 kParabolicXY0{0., 0.};
  const double kParabolicHeading{0.};
  std::unique_ptr<ParamPoly3GroundCurve> parabolic_normalized_dut_;

 protected:
  void SetUp() override {
    // Straight line: aU=0, bU=1, cU=0, dU=0, aV=0, bV=0, cV=0, dV=0
    // With normalized range, p goes from 0 to 1
    straight_normalized_dut_ = std::make_unique<ParamPoly3GroundCurve>(
        kLinearTolerance, kStraightXY0, kStraightHeading, 0., 1., 0., 0., 0., 0., 0., 0., kArcLength, kP0, kP1,
        ParamPoly3GroundCurve::PRangeType::kNormalized);

    // Parabolic: aU=0, bU=1, cU=0, dU=0, aV=0, bV=0, cV=0.1, dV=0
    parabolic_normalized_dut_ = std::make_unique<ParamPoly3GroundCurve>(
        kLinearTolerance, kParabolicXY0, kParabolicHeading, 0., 1., 0., 0., 0., 0., 0.1, 0., kArcLength, kP0, kP1,
        ParamPoly3GroundCurve::PRangeType::kNormalized);
  }
};

TEST_F(ParamPoly3GroundCurveNormalizedTest, StraightLineG) {
  // For normalized range, p=0 to p=1 maps to p_norm=0 to p_norm=1
  // u(p_norm) = p_norm, v(p_norm) = 0
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{0., 0.}, straight_normalized_dut_->G(kP0), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{0.5, 0.}, straight_normalized_dut_->G(0.5), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, straight_normalized_dut_->G(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveNormalizedTest, StraightLineGDot) {
  // For u(p) = p, v(p) = 0 with normalized range
  // du/dp_norm = 1, dv/dp_norm = 0
  // dp_norm/dp = 1/(p1-p0) = 1/1 = 1
  // So GDot = (1, 0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, straight_normalized_dut_->GDot(kP0), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, straight_normalized_dut_->GDot(0.5), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, straight_normalized_dut_->GDot(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveNormalizedTest, ParabolicCurveG) {
  // u(p_norm) = p_norm, v(p_norm) = 0.1*p_norm^2
  // At p=0: p_norm=0: (u,v) = (0, 0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{0., 0.}, parabolic_normalized_dut_->G(kP0), kTolerance)));
  // At p=0.5: p_norm=0.5: (u,v) = (0.5, 0.025)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{0.5, 0.025}, parabolic_normalized_dut_->G(0.5), kTolerance)));
  // At p=1: p_norm=1: (u,v) = (1, 0.1)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.1}, parabolic_normalized_dut_->G(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveNormalizedTest, ParabolicCurveGDot) {
  // u(p_norm) = p_norm, v(p_norm) = 0.1*p_norm^2
  // du/dp_norm = 1, dv/dp_norm = 0.2*p_norm
  // dp_norm/dp = 1/(p1-p0) = 1
  // At p=0: GDot = (1, 0)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.}, parabolic_normalized_dut_->GDot(kP0), kTolerance)));
  // At p=0.5: GDot = (1, 0.1)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.1}, parabolic_normalized_dut_->GDot(0.5), kTolerance)));
  // At p=1: GDot = (1, 0.2)
  EXPECT_TRUE(AssertCompare(CompareVectors(Vector2{1., 0.2}, parabolic_normalized_dut_->GDot(kP1), kTolerance)));
}

TEST_F(ParamPoly3GroundCurveNormalizedTest, ArcLength) {
  EXPECT_NEAR(kArcLength, straight_normalized_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(kArcLength, parabolic_normalized_dut_->ArcLength(), kTolerance);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
