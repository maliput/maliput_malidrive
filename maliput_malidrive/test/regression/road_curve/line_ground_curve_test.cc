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
#include "maliput_malidrive/road_curve/line_ground_curve.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/maliput_math_compare.h>

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

using maliput::math::Vector2;
using maliput::math::test::CompareVectors;

class LineGroundCurveConstructorTest : public ::testing::Test {
 protected:
  const Vector2 kZero{0., 0.};
  const Vector2 kUnitX{1., 0.};
};

TEST_F(LineGroundCurveConstructorTest, CorrectlyConstructed) {
  EXPECT_NO_THROW(LineGroundCurve(1.0, kZero, kUnitX, 0., 1.));
}

TEST_F(LineGroundCurveConstructorTest, InvalidNegativeTolerance) {
  EXPECT_THROW(LineGroundCurve(-1., kZero, kUnitX, 0., 1.), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveConstructorTest, InvalidZeroTolerance) {
  EXPECT_THROW(LineGroundCurve(0., kZero, kUnitX, 0., 1.), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveConstructorTest, InvalidLineLengthTooSmall) {
  EXPECT_THROW(LineGroundCurve(1.0, kZero, (GroundCurve::kEpsilon / 2.) * kUnitX, 0., 1.),
               maliput::common::assertion_error);
}

TEST_F(LineGroundCurveConstructorTest, InvalidNegativeP0) {
  EXPECT_THROW(LineGroundCurve(1.0, kZero, kUnitX, -0.01, 1.), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveConstructorTest, InvalidP1SmallerThanP0) {
  EXPECT_THROW(LineGroundCurve(1.0, kZero, kUnitX, 0., -0.01), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveConstructorTest, InvalidP1NotSufficientlyLargerThanP0) {
  EXPECT_THROW(LineGroundCurve(1.0, kZero, kUnitX, 0., GroundCurve::kEpsilon / 2.), maliput::common::assertion_error);
}

class LineGroundCurveTest : public ::testing::Test {
 public:
  const double kTolerance{1.e-13};
  const double kLinearTolerance{0.01};
  const double kP0{10.};
  const double kP1{20.};
  const Vector2 kTrivialXY_p0{0., 0.};
  const Vector2 kTrivialDXY{1., 0.};
  std::unique_ptr<LineGroundCurve> trivial_dut_;
  const Vector2 kQuadrant1XY_p0{1., 0.};
  const Vector2 kQuadrant1DXY{3., 4.};
  std::unique_ptr<LineGroundCurve> quadrant1_dut_;
  const Vector2 kQuadrant3XY_p0{10., 9.};
  const Vector2 kQuadrant3DXY{-4., -3.};
  std::unique_ptr<LineGroundCurve> quadrant3_dut_;

  // @{  These constants here are referenced in comments below.
  // const double kPMidpoint{0.5 * (kP0 + kP1)};
  //
  // const Vector2 kTrivialXY_p1{kTrivialXY_p0 + kTrivialDXY};
  // const Vector2 kTrivialGDot{kTrivialDXY / (kP1 - kP0)};
  // const double kTrivialArcLength{kTrivialDXY.norm()};
  // const double kTrivialHeading{std::atan2(kTrivialDXY.y(), kTrivialDXY.x())};
  // const double kTrivialHeadingDot{0.};
  //
  // const Vector2 kQuadrant1XY_p1{kQuadrant1XY_p0 + kQuadrant1DXY};
  // const Vector2 kQuadrant1GDot{kQuadrant1DXY / (kP1 - kP0)};
  // const double kQuadrant1ArcLength{kQuadrant1DXY.norm()};
  // const double kQuadrant1Heading{std::atan2(kQuadrant1DXY.y(), kQuadrant1DXY.x())};
  // const double kQuadrant1HeadingDot{0.};
  //
  // const Vector2 kQuadrant3XY_p1{kQuadrant3XY_p0 + kQuadrant3DXY};
  // const Vector2 kQuadrant3GDot{kQuadrant3DXY / (kP1 - kP0)};
  // const double kQuadrant3ArcLength{kQuadrant3DXY.norm()};
  // const double kQuadrant3Heading{std::atan2(kQuadrant3DXY.y(), kQuadrant3DXY.x())};
  // const double kQuadrant3HeadingDot{0.};
  // @}
 protected:
  void SetUp() override {
    trivial_dut_ = std::make_unique<LineGroundCurve>(kLinearTolerance, kTrivialXY_p0, kTrivialDXY, kP0, kP1);
    quadrant1_dut_ = std::make_unique<LineGroundCurve>(kLinearTolerance, kQuadrant1XY_p0, kQuadrant1DXY, kP0, kP1);
    quadrant3_dut_ = std::make_unique<LineGroundCurve>(kLinearTolerance, kQuadrant3XY_p0, kQuadrant3DXY, kP0, kP1);
  }
};

TEST_F(LineGroundCurveTest, linear_tolerance) {
  EXPECT_NEAR(kLinearTolerance, trivial_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, quadrant1_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, quadrant3_dut_->linear_tolerance(), kTolerance);
}

TEST_F(LineGroundCurveTest, p0) {
  EXPECT_NEAR(kP0, trivial_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, quadrant1_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, quadrant3_dut_->p0(), kTolerance);
}

TEST_F(LineGroundCurveTest, p1) {
  EXPECT_NEAR(kP1, trivial_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, quadrant1_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, quadrant3_dut_->p1(), kTolerance);
}

TEST_F(LineGroundCurveTest, ArcLength) {
  EXPECT_NEAR(/* kTrivialArcLength */ 1., trivial_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(/* kQuadrant1ArcLength */ 5., quadrant1_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(/* kQuadrant3ArcLength */ 5., quadrant3_dut_->ArcLength(), kTolerance);
}

TEST_F(LineGroundCurveTest, IsG1Contiguous) {
  EXPECT_TRUE(trivial_dut_->IsG1Contiguous());
  EXPECT_TRUE(quadrant1_dut_->IsG1Contiguous());
  EXPECT_TRUE(quadrant3_dut_->IsG1Contiguous());
}

TEST_F(LineGroundCurveTest, G) {
  // check values at p0, p1, and midpoint
  EXPECT_TRUE(CompareVectors(/* kTrivialXY_p0 */ {0., 0.}, trivial_dut_->G(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kTrivialXY_p1 */ {1., 0.}, trivial_dut_->G(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* 0.5*(kTrivialXY_p0 + kTrivialXY_p1) */ {0.5, 0.}, trivial_dut_->G(/* kPMidpoint */ 15.),
                             kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant1XY_p0 */ {1., 0.}, quadrant1_dut_->G(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant1XY_p1 */ {4., 4.}, quadrant1_dut_->G(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* 0.5*(kQuadrant1XY_p0 + kQuadrant1XY_p1) */ {2.5, 2.0},
                             quadrant1_dut_->G(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant3XY_p0 */ {10., 9.}, quadrant3_dut_->G(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant3XY_p1 */ {6., 6.}, quadrant3_dut_->G(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* 0.5*(kQuadrant3XY_p0 + kQuadrant3XY_p1) */ {8.0, 7.5},
                             quadrant3_dut_->G(/* kPMidpoint */ 15.), kTolerance));

  // Confirm that it throws when given p value outside of [p0, p1] by excess of
  // linear tolerance.
  EXPECT_THROW(trivial_dut_->G(9.98), maliput::common::assertion_error);
  EXPECT_THROW(trivial_dut_->G(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->G(9.98), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->G(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->G(9.98), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->G(20.02), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveTest, GDot) {
  // check values at p0, p1, and midpoint
  EXPECT_TRUE(CompareVectors(/* kTrivialGDot */ {0.1, 0.}, trivial_dut_->GDot(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kTrivialGDot */ {0.1, 0.}, trivial_dut_->GDot(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kTrivialGDot */ {0.1, 0.}, trivial_dut_->GDot(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant1GDot */ {0.3, 0.4}, quadrant1_dut_->GDot(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant1GDot */ {0.3, 0.4}, quadrant1_dut_->GDot(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant1GDot */ {0.3, 0.4}, quadrant1_dut_->GDot(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant3GDot */ {-0.4, -0.3}, quadrant3_dut_->GDot(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kQuadrant3GDot */ {-0.4, -0.3}, quadrant3_dut_->GDot(kP1), kTolerance));
  EXPECT_TRUE(
      CompareVectors(/* kQuadrant3GDot */ {-0.4, -0.3}, quadrant3_dut_->GDot(/* kPMidpoint */ 15.), kTolerance));

  // Confirm that it throws when given p value outside of [p0, p1] by excess of
  // linear tolerance.
  EXPECT_THROW(trivial_dut_->GDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(trivial_dut_->GDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->GDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->GDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->GDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->GDot(20.02), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveTest, Heading) {
  // check values at p0, p1, and midpoint
  EXPECT_NEAR(/* kTrivialHeading */ 0.0, trivial_dut_->Heading(kP0), kTolerance);
  EXPECT_NEAR(/* kTrivialHeading */ 0.0, trivial_dut_->Heading(kP1), kTolerance);
  EXPECT_NEAR(/* kTrivialHeading */ 0.0, trivial_dut_->Heading(/* kPMidpoint */ 15.), kTolerance);
  EXPECT_NEAR(/* kQuadrant1Heading */ 0.9272952180016122, quadrant1_dut_->Heading(kP0), kTolerance);
  EXPECT_NEAR(/* kQuadrant1Heading */ 0.9272952180016122, quadrant1_dut_->Heading(kP1), kTolerance);
  EXPECT_NEAR(/* kQuadrant1Heading */ 0.9272952180016122, quadrant1_dut_->Heading(/* kPMidpoint */ 15.), kTolerance);
  EXPECT_NEAR(/* kQuadrant3Heading */ -2.498091544796509, quadrant3_dut_->Heading(kP0), kTolerance);
  EXPECT_NEAR(/* kQuadrant3Heading */ -2.498091544796509, quadrant3_dut_->Heading(kP1), kTolerance);
  EXPECT_NEAR(/* kQuadrant3Heading */ -2.498091544796509, quadrant3_dut_->Heading(/* kPMidpoint */ 15.), kTolerance);

  // Confirm that it throws when given p value outside of [p0, p1] by excess of
  // linear tolerance.
  EXPECT_THROW(trivial_dut_->Heading(9.98), maliput::common::assertion_error);
  EXPECT_THROW(trivial_dut_->Heading(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->Heading(9.98), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->Heading(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->Heading(9.98), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->Heading(20.02), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveTest, HeadingDot) {
  // check values at p0, p1, and midpoint
  EXPECT_NEAR(/* kTrivialHeadingDot */ 0., trivial_dut_->HeadingDot(kP0), kTolerance);
  EXPECT_NEAR(/* kTrivialHeadingDot */ 0., trivial_dut_->HeadingDot(kP1), kTolerance);
  EXPECT_NEAR(/* kTrivialHeadingDot */ 0., trivial_dut_->HeadingDot(/* kPMidpoint */ 15.), kTolerance);
  EXPECT_NEAR(/* kQuadrant1HeadingDot */ 0., quadrant1_dut_->HeadingDot(kP0), kTolerance);
  EXPECT_NEAR(/* kQuadrant1HeadingDot */ 0., quadrant1_dut_->HeadingDot(kP1), kTolerance);
  EXPECT_NEAR(/* kQuadrant1HeadingDot */ 0., quadrant1_dut_->HeadingDot(/* kPMidpoint */ 15.), kTolerance);
  EXPECT_NEAR(/* kQuadrant3HeadingDot */ 0., quadrant3_dut_->HeadingDot(kP0), kTolerance);
  EXPECT_NEAR(/* kQuadrant3HeadingDot */ 0., quadrant3_dut_->HeadingDot(kP1), kTolerance);
  EXPECT_NEAR(/* kQuadrant3HeadingDot */ 0., quadrant3_dut_->HeadingDot(/* kPMidpoint */ 15.), kTolerance);

  // Confirm that it throws when given p value outside of [p0, p1] by excess of
  // linear tolerance.
  EXPECT_THROW(trivial_dut_->HeadingDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(trivial_dut_->HeadingDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->HeadingDot(9.988), maliput::common::assertion_error);
  EXPECT_THROW(quadrant1_dut_->HeadingDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->HeadingDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(quadrant3_dut_->HeadingDot(20.02), maliput::common::assertion_error);
}

TEST_F(LineGroundCurveTest, GInverse) {
  // check values on the center-line at p0, p1, and midpoint
  EXPECT_NEAR(kP0, trivial_dut_->GInverse(/* kTrivialXY_p0 */ {0.0, 0.}), kTolerance);
  EXPECT_NEAR(kP1, trivial_dut_->GInverse(/* kTrivialXY_p1 */ {1.0, 0.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., trivial_dut_->GInverse(/* 0.5*(kTrivialXY_p0 + kTrivialXY_p1) */ {0.5, 0.}),
              kTolerance);
  EXPECT_NEAR(kP0, quadrant1_dut_->GInverse(/* kQuadrant1XY_p0 */ {1.0, 0.0}), kTolerance);
  EXPECT_NEAR(kP1, quadrant1_dut_->GInverse(/* kQuadrant1XY_p1 */ {4.0, 4.0}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., quadrant1_dut_->GInverse(/* 0.5*(kQuadrant1XY_p0 + kQuadrant1XY_p1) */ {2.5, 2.0}),
              kTolerance);
  EXPECT_NEAR(kP0, quadrant3_dut_->GInverse(/* kQuadrant3XY_p0 */ {10.0, 9.0}), kTolerance);
  EXPECT_NEAR(kP1, quadrant3_dut_->GInverse(/* kQuadrant3XY_p1 */ {6.0, 6.0}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., quadrant3_dut_->GInverse(/* 0.5*(kQuadrant3XY_p0 + kQuadrant1XY_p1) */ {8.0, 7.5}),
              kTolerance);

  // Confirm that it works for points off the center-line
  EXPECT_NEAR(kP0, trivial_dut_->GInverse({/* kTrivialXY_p0.x() */ 0.0, 1.0}), kTolerance);
  EXPECT_NEAR(kP1, trivial_dut_->GInverse({/* kTrivialXY_p1.x() */ 1.0, 1.0}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., trivial_dut_->GInverse({/* 0.5*(kTrivialXY_p0 + kTrivialXY_p1).x() */ 0.5, 1.0}),
              kTolerance);
  EXPECT_NEAR(kP0,
              quadrant1_dut_->GInverse({/* kQuadrant1XY_p0.x() + kQuadrant1DXY.y() */ 5.0,
                                        /* kQuadrant1XY_p0.y() - kQuadrant1DXY.x() */ -3.0}),
              kTolerance);
  EXPECT_NEAR(kP1,
              quadrant1_dut_->GInverse({/* kQuadrant1XY_p1.x() + kQuadrant1DXY.y() */ 8.0,
                                        /* kQuadrant1XY_p1.y() - kQuadrant1DXY.x() */ 1.0}),
              kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15.,
              quadrant1_dut_->GInverse({/* 0.5*(kQuadrant1XY_p0 + kQuadrant1XY_p1).x() + kQuadrant1DXY.y() */ 6.5,
                                        /* 0.5*(kQuadrant1XY_p0 + kQuadrant1XY_p1).y() - kQuadrant1DXY.x() */ -1.0}),
              kTolerance);
  EXPECT_NEAR(kP0,
              quadrant3_dut_->GInverse({/* kQuadrant3XY_p0.x() + kQuadrant3DXY.y() */ 7.0,
                                        /* kQuadrant3XY_p0.y() - kQuadrant3DXY.x() */ 13.0}),
              kTolerance);
  EXPECT_NEAR(kP1,
              quadrant3_dut_->GInverse({/* kQuadrant3XY_p1.x() + kQuadrant3DXY.y() */ 3.0,
                                        /* kQuadrant3XY_p1.y() - kQuadrant3DXY.x() */ 10.0}),
              kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15.,
              quadrant3_dut_->GInverse({/* 0.5*(kQuadrant3XY_p0 + kQuadrant3XY_p1).x() + kQuadrant3DXY.y() */ 5.0,
                                        /* 0.5*(kQuadrant3XY_p0 + kQuadrant3XY_p1).y() - kQuadrant3DXY.x() */ 11.5}),
              kTolerance);

  // Confirm that it returns p0 or p1 for points past the end of the line
  EXPECT_NEAR(kP0, trivial_dut_->GInverse({-1.0, 0.0}), kTolerance);
  EXPECT_NEAR(kP0, trivial_dut_->GInverse({-1.0, 1.0}), kTolerance);
  EXPECT_NEAR(kP1, trivial_dut_->GInverse({2.0, 0.0}), kTolerance);
  EXPECT_NEAR(kP1, trivial_dut_->GInverse({2.0, 1.0}), kTolerance);
  EXPECT_NEAR(kP0, quadrant1_dut_->GInverse({0.0, -1.0}), kTolerance);
  EXPECT_NEAR(kP0, quadrant1_dut_->GInverse({-1.0, -1.0}), kTolerance);
  EXPECT_NEAR(kP1, quadrant1_dut_->GInverse({5.0, 5.0}), kTolerance);
  EXPECT_NEAR(kP1, quadrant1_dut_->GInverse({6.0, 6.0}), kTolerance);
  EXPECT_NEAR(kP0, quadrant3_dut_->GInverse({11.0, 10.0}), kTolerance);
  EXPECT_NEAR(kP0, quadrant3_dut_->GInverse({12.0, 11.0}), kTolerance);
  EXPECT_NEAR(kP1, quadrant3_dut_->GInverse({5.0, 5.0}), kTolerance);
  EXPECT_NEAR(kP1, quadrant3_dut_->GInverse({4.0, 4.0}), kTolerance);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
