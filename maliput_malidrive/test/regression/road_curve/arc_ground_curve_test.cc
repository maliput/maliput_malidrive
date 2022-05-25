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
#include "maliput_malidrive/road_curve/arc_ground_curve.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/maliput_math_compare.h>

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

using maliput::math::Vector2;
using maliput::math::test::CompareVectors;

class ArcGroundCurveConstructorTest : public ::testing::Test {
 protected:
  const Vector2 kZero{0., 0.};
  const Vector2 kUnitX{1., 0.};
};

TEST_F(ArcGroundCurveConstructorTest, CorrectlyConstructed) {
  EXPECT_NO_THROW(ArcGroundCurve(1.0, kZero, 0., 1., 1., 0., 1.));
}

TEST_F(ArcGroundCurveConstructorTest, InvalidNegativeTolerance) {
  EXPECT_THROW(ArcGroundCurve(-1., kZero, 0., 1., 1., 0., 1.), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveConstructorTest, InvalidZeroTolerance) {
  EXPECT_THROW(ArcGroundCurve(0., kZero, 0., 1., 1., 0., 1.), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveConstructorTest, InvalidPositiveCurvatureTooSmall) {
  EXPECT_THROW(ArcGroundCurve(1., kZero, 0., GroundCurve::kEpsilon / 2., 1., 0., 1.), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveConstructorTest, InvalidNegativeCurvatureTooSmall) {
  EXPECT_THROW(ArcGroundCurve(1., kZero, 0., -GroundCurve::kEpsilon / 2., 1., 0., 1.),
               maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveConstructorTest, InvalidArcLengthTooSmall) {
  EXPECT_THROW(ArcGroundCurve(1.0, kZero, 0., 1., GroundCurve::kEpsilon / 2., 0., 1.),
               maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveConstructorTest, InvalidNegativeP0) {
  EXPECT_THROW(ArcGroundCurve(1.0, kZero, 0., 1., 1., -0.01, 1.), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveConstructorTest, InvalidP1SmallerThanP0) {
  EXPECT_THROW(ArcGroundCurve(1.0, kZero, 0., 1., 1., 0., -0.01), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveConstructorTest, InvalidP1NotSufficientlyLargerThanP0) {
  EXPECT_THROW(ArcGroundCurve(1.0, kZero, 0., 1., 1., 0., GroundCurve::kEpsilon / 2.),
               maliput::common::assertion_error);
}

class ArcGroundCurveTest : public ::testing::Test {
 public:
  const double kTolerance{1.e-12};
  const double kLinearTolerance{0.01};
  const double kP0{10.};
  const double kP1{20.};
  const double kArcLength{M_PI * 8.};
  const Vector2 kLeftTurn90DegG_p0{0., 0.};
  const double kLeftTurn90DegHeading_p0{0.};
  const double kLeftTurn90DegCurvature{M_PI / 2. / kArcLength};
  std::unique_ptr<ArcGroundCurve> left_turn_90deg_dut_;
  const Vector2 kRightTurn90DegG_p0{0., 0.};
  const double kRightTurn90DegHeading_p0{0.};
  const double kRightTurn90DegCurvature{-M_PI / 2. / kArcLength};
  std::unique_ptr<ArcGroundCurve> right_turn_90deg_dut_;
  const Vector2 kUTurnQuadrant1G_p0{1., 0.};
  const double kUTurnQuadrant1Heading_p0{M_PI / 3.};
  const double kUTurnQuadrant1Curvature{M_PI / kArcLength};
  std::unique_ptr<ArcGroundCurve> u_turn_quadrant1_dut_;
  const Vector2 kSlightRightTurnQuadrant3G_p0{10., 9.};
  const double kSlightRightTurnQuadrant3Heading_p0{M_PI * 7. / 6.};
  const double kSlightRightTurnQuadrant3Curvature{-M_PI / 8. / kArcLength};
  std::unique_ptr<ArcGroundCurve> slight_right_turn_quadrant3_dut_;

  // @{  These constants here are referenced in comments below.
  // const double kPMidpoint{0.5 * (kP0 + kP1)};
  //
  // const double kLeftTurn90DegHeadingDot{kArcLength * kLeftTurn90DegCurvature / (kP1 - kP0)};
  // const double kLeftTurn90DegHeading_p1{kLeftTurn90DegHeading_p0 + kArcLength * kLeftTurn90DegCurvature};
  // const double kLeftTurn90DegHeading_Midpoint{(kLeftTurn90DegHeading_p0 + kLeftTurn90DegHeading_p1) / 2.};
  // const Vector2 kLeftTurn90DegGDot_p0{kArcLength / (kP1 - kP0) * Vector2{std::cos(kLeftTurn90DegHeading_p0),
  //                                                                        std::sin(kLeftTurn90DegHeading_p0)}};
  // const Vector2 kLeftTurn90DegGDot_p1{kArcLength / (kP1 - kP0) * Vector2{std::cos(kLeftTurn90DegHeading_p1),
  //                                                                        std::sin(kLeftTurn90DegHeading_p1)}};
  // const Vector2 kLeftTurn90DegGDot_Midpoint{
  //     kArcLength / (kP1 - kP0) * Vector2{std::cos(kLeftTurn90DegHeading_Midpoint),
  //                                        std::sin(kLeftTurn90DegHeading_Midpoint)}};
  // const Vector2 kLeftTurn90DegGCenter{
  //    kLeftTurn90DegG_p0 + 1. / kLeftTurn90DegCurvature * Vector2{-std::sin(kLeftTurn90DegHeading_p0),
  //                                                                +std::cos(kLeftTurn90DegHeading_p0)}};
  // const Vector2 kLeftTurn90DegG_p1{
  //    kLeftTurn90DegGCenter + 1. / kLeftTurn90DegCurvature * Vector2{+std::sin(kLeftTurn90DegHeading_p1),
  //                                                                   -std::cos(kLeftTurn90DegHeading_p1)}};
  // const Vector2 kLeftTurn90DegG_Midpoint{
  //    kLeftTurn90DegGCenter + 1. / kLeftTurn90DegCurvature * Vector2{+std::sin(kLeftTurn90DegHeading_Midpoint),
  //                                                                   -std::cos(kLeftTurn90DegHeading_Midpoint)}};
  //
  // const double kRightTurn90DegHeadingDot{kArcLength * kRightTurn90DegCurvature / (kP1 - kP0)};
  // const double kRightTurn90DegHeading_p1{kRightTurn90DegHeading_p0 + kArcLength * kRightTurn90DegCurvature};
  // const double kRightTurn90DegHeading_Midpoint{(kRightTurn90DegHeading_p0 + kRightTurn90DegHeading_p1) / 2.};
  // const Vector2 kRightTurn90DegGDot_p0{kArcLength / (kP1 - kP0) * Vector2{std::cos(kRightTurn90DegHeading_p0),
  //                                                                         std::sin(kRightTurn90DegHeading_p0)}};
  // const Vector2 kRightTurn90DegGDot_p1{kArcLength / (kP1 - kP0) * Vector2{std::cos(kRightTurn90DegHeading_p1),
  //                                                                         std::sin(kRightTurn90DegHeading_p1)}};
  // const Vector2 kRightTurn90DegGDot_Midpoint{
  //     kArcLength / (kP1 - kP0) * Vector2{std::cos(kRightTurn90DegHeading_Midpoint),
  //                                        std::sin(kRightTurn90DegHeading_Midpoint)}};
  // const Vector2 kRightTurn90DegGCenter{
  //    kRightTurn90DegG_p0 + 1. / kRightTurn90DegCurvature * Vector2{-std::sin(kRightTurn90DegHeading_p0),
  //                                                                  +std::cos(kRightTurn90DegHeading_p0)}};
  // const Vector2 kRightTurn90DegG_p1{
  //    kRightTurn90DegGCenter + 1. / kRightTurn90DegCurvature * Vector2{+std::sin(kRightTurn90DegHeading_p1),
  //                                                                     -std::cos(kRightTurn90DegHeading_p1)}};
  // const Vector2 kRightTurn90DegG_Midpoint{
  //    kRightTurn90DegGCenter + 1. / kRightTurn90DegCurvature * Vector2{+std::sin(kRightTurn90DegHeading_Midpoint),
  //                                                                     -std::cos(kRightTurn90DegHeading_Midpoint)}};
  //
  // const double kUTurnQuadrant1HeadingDot{kArcLength * kUTurnQuadrant1Curvature / (kP1 - kP0)};
  // const double kUTurnQuadrant1Heading_p1{kUTurnQuadrant1Heading_p0 + kArcLength * kUTurnQuadrant1Curvature};
  // const double kUTurnQuadrant1Heading_Midpoint{(kUTurnQuadrant1Heading_p0 + kUTurnQuadrant1Heading_p1) / 2.};
  // const Vector2 kUTurnQuadrant1GDot_p0{kArcLength / (kP1 - kP0) * Vector2{std::cos(kUTurnQuadrant1Heading_p0),
  //                                                                         std::sin(kUTurnQuadrant1Heading_p0)}};
  // const Vector2 kUTurnQuadrant1GDot_p1{kArcLength / (kP1 - kP0) * Vector2{std::cos(kUTurnQuadrant1Heading_p1),
  //                                                                         std::sin(kUTurnQuadrant1Heading_p1)}};
  // const Vector2 kUTurnQuadrant1GDot_Midpoint{
  //     kArcLength / (kP1 - kP0) * Vector2{std::cos(kUTurnQuadrant1Heading_Midpoint),
  //                                        std::sin(kUTurnQuadrant1Heading_Midpoint)}};
  // const Vector2 kUTurnQuadrant1GCenter{
  //    kUTurnQuadrant1G_p0 + 1. / kUTurnQuadrant1Curvature * Vector2{-std::sin(kUTurnQuadrant1Heading_p0),
  //                                                                  +std::cos(kUTurnQuadrant1Heading_p0)}};
  // const Vector2 kUTurnQuadrant1G_p1{
  //    kUTurnQuadrant1GCenter + 1. / kUTurnQuadrant1Curvature * Vector2{+std::sin(kUTurn90DegHeading_p1),
  //                                                                     -std::cos(kUTurnQuadrant1Heading_p1)}};
  // const Vector2 kUTurnQuadrant1G_Midpoint{
  //    kUTurnQuadrant1GCenter + 1. / kUTurnQuadrant1Curvature * Vector2{+std::sin(kUTurnQuadrant1Heading_Midpoint),
  //                                                                     -std::cos(kUTurnQuadrant1Heading_Midpoint)}};
  //
  // const double kSlightRightTurnQuadrant3HeadingDot{
  //    kArcLength * kSlightRightTurnQuadrant3Curvature / (kP1 - kP0)};
  // const double kSlightRightTurnQuadrant3Heading_p1{
  //    kSlightRightTurnQuadrant3Heading_p0 + kArcLength * kSlightRightTurnQuadrant3Curvature};
  // const double kSlightRightTurnQuadrant3Heading_Midpoint{
  //    (kSlightRightTurnQuadrant3Heading_p0 + kSlightRightTurnQuadrant3Heading_p1) / 2.};
  // const Vector2 kSlightRightTurnQuadrant3GDot_p0{
  //    kArcLength / (kP1 - kP0) * Vector2{std::cos(kSlightRightTurnQuadrant3Heading_p0),
  //                                       std::sin(kSlightRightTurnQuadrant3Heading_p0)}};
  // const Vector2 kSlightRightTurnQuadrant3GDot_p1{
  //    kArcLength / (kP1 - kP0) * Vector2{std::cos(kSlightRightTurnQuadrant3Heading_p1),
  //                                       std::sin(kSlightRightTurnQuadrant3Heading_p1)}};
  // const Vector2 kSlightRightTurnQuadrant3GDot_Midpoint{
  //    kArcLength / (kP1 - kP0) * Vector2{std::cos(kSlightRightTurnQuadrant3Heading_Midpoint),
  //                                       std::sin(kSlightRightTurnQuadrant3Heading_Midpoint)}};
  // const Vector2 kSlightRightTurnQuadrant3GCenter{
  //    kSlightRightTurnQuadrant3G_p0 + 1. / kSlightRightTurnQuadrant3Curvature *
  //        Vector2{-std::sin(kSlightRightTurnQuadrant3Heading_p0), std::cos(kSlightRightTurnQuadrant3Heading_p0)}};
  // const Vector2 kSlightRightTurnQuadrant3G_p1{
  //    kSlightRightTurnQuadrant3GCenter + 1. / kSlightRightTurnQuadrant3Curvature *
  //        Vector2{+std::sin(kSlightRightTurn90DegHeading_p1), -std::cos(kSlightRightTurnQuadrant3Heading_p1)}};
  // const Vector2 kSlightRightTurnQuadrant3G_Midpoint{
  //    kSlightRightTurnQuadrant3GCenter + 1. / kSlightRightTurnQuadrant3Curvature *
  //        Vector2{+std::sin(kSlightRightTurnQuadrant3Heading_Midpoint),
  //                -std::cos(kSlightRightTurnQuadrant3Heading_Midpoint)}};
  // const Vector2 kQuadrant3G_p1{kQuadrant3G_p0 + kQuadrant3DXY};
  // const Vector2 kQuadrant3GDot{kQuadrant3DXY / (kP1 - kP0)};
  // @}
 protected:
  void SetUp() override {
    left_turn_90deg_dut_ = std::make_unique<ArcGroundCurve>(
        kLinearTolerance, kLeftTurn90DegG_p0, kLeftTurn90DegHeading_p0, kLeftTurn90DegCurvature, kArcLength, kP0, kP1);
    right_turn_90deg_dut_ =
        std::make_unique<ArcGroundCurve>(kLinearTolerance, kRightTurn90DegG_p0, kRightTurn90DegHeading_p0,
                                         kRightTurn90DegCurvature, kArcLength, kP0, kP1);
    u_turn_quadrant1_dut_ =
        std::make_unique<ArcGroundCurve>(kLinearTolerance, kUTurnQuadrant1G_p0, kUTurnQuadrant1Heading_p0,
                                         kUTurnQuadrant1Curvature, kArcLength, kP0, kP1);
    slight_right_turn_quadrant3_dut_ = std::make_unique<ArcGroundCurve>(
        kLinearTolerance, kSlightRightTurnQuadrant3G_p0, kSlightRightTurnQuadrant3Heading_p0,
        kSlightRightTurnQuadrant3Curvature, kArcLength, kP0, kP1);
  }
};

TEST_F(ArcGroundCurveTest, linear_tolerance) {
  EXPECT_NEAR(kLinearTolerance, left_turn_90deg_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, right_turn_90deg_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, u_turn_quadrant1_dut_->linear_tolerance(), kTolerance);
  EXPECT_NEAR(kLinearTolerance, slight_right_turn_quadrant3_dut_->linear_tolerance(), kTolerance);
}

TEST_F(ArcGroundCurveTest, p0) {
  EXPECT_NEAR(kP0, left_turn_90deg_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, right_turn_90deg_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, u_turn_quadrant1_dut_->p0(), kTolerance);
  EXPECT_NEAR(kP0, slight_right_turn_quadrant3_dut_->p0(), kTolerance);
}

TEST_F(ArcGroundCurveTest, p1) {
  EXPECT_NEAR(kP1, left_turn_90deg_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, right_turn_90deg_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, u_turn_quadrant1_dut_->p1(), kTolerance);
  EXPECT_NEAR(kP1, slight_right_turn_quadrant3_dut_->p1(), kTolerance);
}

TEST_F(ArcGroundCurveTest, ArcLength) {
  EXPECT_NEAR(kArcLength, left_turn_90deg_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(kArcLength, right_turn_90deg_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(kArcLength, u_turn_quadrant1_dut_->ArcLength(), kTolerance);
  EXPECT_NEAR(kArcLength, slight_right_turn_quadrant3_dut_->ArcLength(), kTolerance);
}

TEST_F(ArcGroundCurveTest, IsG1Contiguous) {
  EXPECT_TRUE(left_turn_90deg_dut_->IsG1Contiguous());
  EXPECT_TRUE(right_turn_90deg_dut_->IsG1Contiguous());
  EXPECT_TRUE(u_turn_quadrant1_dut_->IsG1Contiguous());
  EXPECT_TRUE(slight_right_turn_quadrant3_dut_->IsG1Contiguous());
}

TEST_F(ArcGroundCurveTest, G) {
  // check values at p0, p1, and midpoint
  EXPECT_TRUE(CompareVectors(/* kLeftTurn90DegG_p0 */ {0., 0.}, left_turn_90deg_dut_->G(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kLeftTurn90DegG_p1 */ {16., 16.}, left_turn_90deg_dut_->G(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kLeftTurn90DegG_Midpoint */ {11.313708498984761, 4.686291501015239},
                             left_turn_90deg_dut_->G(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kRightTurn90DegG_p0 */ {0., 0.}, right_turn_90deg_dut_->G(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kRightTurn90DegG_p1 */ {16., -16.}, right_turn_90deg_dut_->G(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kRightTurn90DegG_Midpoint */ {11.313708498984761, -4.686291501015239},
                             right_turn_90deg_dut_->G(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kUTurnQuadrant1G_p0 */ {1., 0.}, u_turn_quadrant1_dut_->G(kP0), kTolerance));
  EXPECT_TRUE(
      CompareVectors(/* kUTurnQuadrant1G_p1 */ {-12.856406460551018, 8.}, u_turn_quadrant1_dut_->G(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kUTurnQuadrant1G_Midpoint */ {-1.9282032302755061, 10.928203230275509},
                             u_turn_quadrant1_dut_->G(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kSlightRightTurnQuadrant3G_p0 */ {10., 9.}, slight_right_turn_quadrant3_dut_->G(kP0),
                             kTolerance));
  EXPECT_TRUE(CompareVectors(/* kSlightRightTurnQuadrant3G_p1 */ {-13.646323697916673, 0.9731547142802199},
                             slight_right_turn_quadrant3_dut_->G(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kSlightRightTurnQuadrant3G_Midpoint */ {-1.4278742205976478, 3.8220975545173204},
                             slight_right_turn_quadrant3_dut_->G(/* kPMidpoint */ 15.), kTolerance));

  // Confirm that it throws when given p value outside of [p0, p1] by excess of
  // linear tolerance.
  EXPECT_THROW(left_turn_90deg_dut_->G(9.98), maliput::common::assertion_error);
  EXPECT_THROW(left_turn_90deg_dut_->G(20.02), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->G(9.98), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->G(20.02), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->G(9.98), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->G(20.02), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->G(9.98), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->G(20.02), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveTest, GDot) {
  // check values at p0, p1, and midpoint
  EXPECT_TRUE(CompareVectors(/* kLeftTurn90DegGDot_p0 */ {2.5132741228718345, 0.}, left_turn_90deg_dut_->GDot(kP0),
                             kTolerance));
  EXPECT_TRUE(CompareVectors(/* kLeftTurn90DegGDot_p1 */ {0., 2.5132741228718345}, left_turn_90deg_dut_->GDot(kP1),
                             kTolerance));
  EXPECT_TRUE(CompareVectors(/* kLeftTurn90DegGDot_Midpoint */ {1.7771531752633463, 1.7771531752633463},
                             left_turn_90deg_dut_->GDot(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kRightTurn90DegGDot_p0 */ {2.5132741228718345, 0.}, right_turn_90deg_dut_->GDot(kP0),
                             kTolerance));
  EXPECT_TRUE(CompareVectors(/* kRightTurn90DegGDot_p1 */ {0., -2.5132741228718345}, right_turn_90deg_dut_->GDot(kP1),
                             kTolerance));
  EXPECT_TRUE(CompareVectors(/* kRightTurn90DegGDot_Midpoint */ {1.7771531752633463, -1.7771531752633463},
                             right_turn_90deg_dut_->GDot(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kUTurnQuadrant1GDot_p0 */ {1.2566370614359172, 2.176559237081061},
                             u_turn_quadrant1_dut_->GDot(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kUTurnQuadrant1GDot_p1 */ {-1.2566370614359172, -2.176559237081061},
                             u_turn_quadrant1_dut_->GDot(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kUTurnQuadrant1GDot_Midpoint */ {-2.176559237081061, 1.2566370614359172},
                             u_turn_quadrant1_dut_->GDot(/* kPMidpoint */ 15.), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kSlightRightTurnQuadrant3GDot_p0 */ {-2.1765592370810616, -1.2566370614359166},
                             slight_right_turn_quadrant3_dut_->GDot(kP0), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kSlightRightTurnQuadrant3GDot_p1 */ {-2.4917727143450508, -0.3280481012636511},
                             slight_right_turn_quadrant3_dut_->GDot(kP1), kTolerance));
  EXPECT_TRUE(CompareVectors(/* kSlightRightTurnQuadrant3GDot_Midpoint */ {-2.3798949906277245, -0.8078654902161948},
                             slight_right_turn_quadrant3_dut_->GDot(/* kPMidpoint */ 15.), kTolerance));

  // Confirm that it throws when given p value outside of [p0, p1] by excess of
  // linear tolerance.
  EXPECT_THROW(left_turn_90deg_dut_->GDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(left_turn_90deg_dut_->GDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->GDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->GDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->GDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->GDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->GDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->GDot(20.02), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveTest, Heading) {
  // check values at p0, p1, and midpoint
  EXPECT_NEAR(/* kLeftTurn90DegHeading_p0 */ 0., left_turn_90deg_dut_->Heading(kP0), kTolerance);
  EXPECT_NEAR(/* kLeftTurn90DegHeading_p1 */ M_PI / 2., left_turn_90deg_dut_->Heading(kP1), kTolerance);
  EXPECT_NEAR(/* (kLeftTurn90DegHeading_p0 + kLeftTurn90DegHeading_p1) / 2. */ M_PI / 4.,
              left_turn_90deg_dut_->Heading(/* kPMidpoint */ 15.), kTolerance);
  EXPECT_NEAR(/* kRightTurn90DegHeading_p0 */ 0., right_turn_90deg_dut_->Heading(kP0), kTolerance);
  EXPECT_NEAR(/* kRightTurn90DegHeading_p1 */ -M_PI / 2., right_turn_90deg_dut_->Heading(kP1), kTolerance);
  EXPECT_NEAR(/* (kRightTurn90DegHeading_p0 + kRightTurn90DegHeading_p1) / 2. */ -M_PI / 4.,
              right_turn_90deg_dut_->Heading(/* kPMidpoint */ 15.), kTolerance);
  EXPECT_NEAR(/* kUTurnQuadrant1Heading_p0 */ M_PI / 3., u_turn_quadrant1_dut_->Heading(kP0), kTolerance);
  EXPECT_NEAR(/* kUTurnQuadrant1Heading_p1 */ M_PI * 4. / 3., u_turn_quadrant1_dut_->Heading(kP1), kTolerance);
  EXPECT_NEAR(/* (kUTurnQuadrant1Heading_p0 + kUTurnQuadrant1Heading_p1) / 2. */ M_PI * 5. / 6.,
              u_turn_quadrant1_dut_->Heading(/* kPMidpoint */ 15.), kTolerance);
  EXPECT_NEAR(/* kSlightRightTurnQuadrant3Heading_p0 */ M_PI * 7. / 6., slight_right_turn_quadrant3_dut_->Heading(kP0),
              kTolerance);
  EXPECT_NEAR(/* kSlightRightTurnQuadrant3Heading_p1 */ M_PI * 25. / 24.,
              slight_right_turn_quadrant3_dut_->Heading(kP1), kTolerance);
  EXPECT_NEAR(/* (kSlightRightTurnQuadrant3Heading_p0 + kSlightRightTurnQuadrant3Heading_p1) / 2. */ M_PI * 53. / 48.,
              slight_right_turn_quadrant3_dut_->Heading(/* kPMidpoint */ 15.), kTolerance);

  // Confirm that it throws when given p value outside of [p0, p1] by excess of
  // linear tolerance.
  EXPECT_THROW(left_turn_90deg_dut_->Heading(9.98), maliput::common::assertion_error);
  EXPECT_THROW(left_turn_90deg_dut_->Heading(20.02), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->Heading(9.98), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->Heading(20.02), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->Heading(9.98), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->Heading(20.02), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->Heading(9.98), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->Heading(20.02), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveTest, HeadingDot) {
  // check values at p0, p1, and midpoint
  EXPECT_NEAR(/* kLeftTurn90DegHeadingDot */ M_PI / 20., left_turn_90deg_dut_->HeadingDot(kP0), kTolerance);
  EXPECT_NEAR(/* kLeftTurn90DegHeadingDot */ M_PI / 20., left_turn_90deg_dut_->HeadingDot(kP1), kTolerance);
  EXPECT_NEAR(/* kLeftTurn90DegHeadingDot */ M_PI / 20., left_turn_90deg_dut_->HeadingDot(/* kPMidpoint */ 15.),
              kTolerance);
  EXPECT_NEAR(/* kRightTurn90DegHeadingDot */ -M_PI / 20., right_turn_90deg_dut_->HeadingDot(kP0), kTolerance);
  EXPECT_NEAR(/* kRightTurn90DegHeadingDot */ -M_PI / 20., right_turn_90deg_dut_->HeadingDot(kP1), kTolerance);
  EXPECT_NEAR(/* kRightTurn90DegHeadingDot */ -M_PI / 20., right_turn_90deg_dut_->HeadingDot(/* kPMidpoint */ 15.),
              kTolerance);
  EXPECT_NEAR(/* kUTurnQuadrant1HeadingDot */ M_PI / 10., u_turn_quadrant1_dut_->HeadingDot(kP0), kTolerance);
  EXPECT_NEAR(/* kUTurnQuadrant1HeadingDot */ M_PI / 10., u_turn_quadrant1_dut_->HeadingDot(kP1), kTolerance);
  EXPECT_NEAR(/* kUTurnQuadrant1HeadingDot */ M_PI / 10., u_turn_quadrant1_dut_->HeadingDot(/* kPMidpoint */ 15.),
              kTolerance);
  EXPECT_NEAR(/* kSlightRightTurnQuadrant3HeadingDot */ -M_PI / 80., slight_right_turn_quadrant3_dut_->HeadingDot(kP0),
              kTolerance);
  EXPECT_NEAR(/* kSlightRightTurnQuadrant3HeadingDot */ -M_PI / 80., slight_right_turn_quadrant3_dut_->HeadingDot(kP1),
              kTolerance);
  EXPECT_NEAR(/* kSlightRightTurnQuadrant3HeadingDot */ -M_PI / 80.,
              slight_right_turn_quadrant3_dut_->HeadingDot(/* kPMidpoint */ 15.), kTolerance);

  // Confirm that it throws when given p value outside of [p0, p1]
  EXPECT_THROW(left_turn_90deg_dut_->HeadingDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(left_turn_90deg_dut_->HeadingDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->HeadingDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->HeadingDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->HeadingDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->HeadingDot(20.02), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->HeadingDot(9.98), maliput::common::assertion_error);
  EXPECT_THROW(slight_right_turn_quadrant3_dut_->HeadingDot(20.02), maliput::common::assertion_error);
}

TEST_F(ArcGroundCurveTest, GInverse) {
  // check values on the center-line at p0, p1, and midpoint
  EXPECT_NEAR(kP0, left_turn_90deg_dut_->GInverse(/* kLeftTurn90DegG_p0 */ {0., 0.}), kTolerance);
  EXPECT_NEAR(kP1, left_turn_90deg_dut_->GInverse(/* kLeftTurn90DegG_p1 */ {16., 16.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15.,
              left_turn_90deg_dut_->GInverse(/* kLeftTurn90DegG_Midpoint */ {11.313708498984761, 4.686291501015239}),
              kTolerance);
  EXPECT_NEAR(kP0, right_turn_90deg_dut_->GInverse(/* kRightTurn90DegG_p0 */ {0., 0.}), kTolerance);
  EXPECT_NEAR(kP1, right_turn_90deg_dut_->GInverse(/* kRightTurn90DegG_p1 */ {16., -16.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15.,
              right_turn_90deg_dut_->GInverse(/* kRightTurn90DegG_Midpoint */ {11.313708498984761, -4.686291501015239}),
              kTolerance);
  EXPECT_NEAR(kP0, u_turn_quadrant1_dut_->GInverse(/* kUTurnQuadrant1G_p0 */ {1.0, 0.0}), kTolerance);
  EXPECT_NEAR(kP1, u_turn_quadrant1_dut_->GInverse(/* kUTurnQuadrant1G_p1 */ {-12.856406460551018, 8.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15.,
              u_turn_quadrant1_dut_->GInverse(
                  /* kUTurnQuadrant1G_Midpoint */ {-1.9282032302755061, 10.928203230275509}),
              kTolerance);
  EXPECT_NEAR(kP0, slight_right_turn_quadrant3_dut_->GInverse(/* kSlightRightTurnQuadrant3G_p0 */ {10.0, 9.0}),
              kTolerance);
  EXPECT_NEAR(kP1,
              slight_right_turn_quadrant3_dut_->GInverse(
                  /* kSlightRightTurnQuadrant3G_p1 */ {-13.646323697916698, 0.97315471428021283}),
              kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15.,
              slight_right_turn_quadrant3_dut_->GInverse(
                  /* kSlightRightTurnQuadrant3G_Midpoint */ {-1.4278742205976442, 3.8220975545173204}),
              kTolerance);

  // Confirm that it works for points off the center-line
  EXPECT_NEAR(kP0, left_turn_90deg_dut_->GInverse({0., -1.}), kTolerance);
  EXPECT_NEAR(kP0, left_turn_90deg_dut_->GInverse({0., 1.}), kTolerance);
  EXPECT_NEAR(kP1, left_turn_90deg_dut_->GInverse({15., 16.}), kTolerance);
  EXPECT_NEAR(kP1, left_turn_90deg_dut_->GInverse({17., 16.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., left_turn_90deg_dut_->GInverse({11., 5.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., left_turn_90deg_dut_->GInverse({12., 4.}), kTolerance);
  EXPECT_NEAR(kP0, right_turn_90deg_dut_->GInverse({0., -1.}), kTolerance);
  EXPECT_NEAR(kP0, right_turn_90deg_dut_->GInverse({0., 1.}), kTolerance);
  EXPECT_NEAR(kP1, right_turn_90deg_dut_->GInverse({15., -16.}), kTolerance);
  EXPECT_NEAR(kP1, right_turn_90deg_dut_->GInverse({17., -16.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., right_turn_90deg_dut_->GInverse({11., -5.}), kTolerance);
  EXPECT_NEAR(/* kPMidpoint */ 15., right_turn_90deg_dut_->GInverse({12., -4.}), kTolerance);

  // Confirm that it returns p0 or p1 for points past the end of the arc
  EXPECT_NEAR(kP0, left_turn_90deg_dut_->GInverse({-1., -1.}), kTolerance);
  EXPECT_NEAR(kP0, left_turn_90deg_dut_->GInverse({-1., 0.}), kTolerance);
  EXPECT_NEAR(kP0, left_turn_90deg_dut_->GInverse({-1., 1.}), kTolerance);
  EXPECT_NEAR(kP1, left_turn_90deg_dut_->GInverse({15., 17.}), kTolerance);
  EXPECT_NEAR(kP1, left_turn_90deg_dut_->GInverse({16., 17.}), kTolerance);
  EXPECT_NEAR(kP1, left_turn_90deg_dut_->GInverse({17., 17.}), kTolerance);
  EXPECT_NEAR(kP0, right_turn_90deg_dut_->GInverse({-1., -1.}), kTolerance);
  EXPECT_NEAR(kP0, right_turn_90deg_dut_->GInverse({-1., 0.}), kTolerance);
  EXPECT_NEAR(kP0, right_turn_90deg_dut_->GInverse({-1., 1.}), kTolerance);
  EXPECT_NEAR(kP1, right_turn_90deg_dut_->GInverse({15., -17.}), kTolerance);
  EXPECT_NEAR(kP1, right_turn_90deg_dut_->GInverse({16., -17.}), kTolerance);
  EXPECT_NEAR(kP1, right_turn_90deg_dut_->GInverse({17., -17.}), kTolerance);

  // Confirm that it throws for points too close to the center of rotation
  EXPECT_THROW(left_turn_90deg_dut_->GInverse(/* kLeftTurn90DegGCenter */ {0., 16.}), maliput::common::assertion_error);
  EXPECT_THROW(right_turn_90deg_dut_->GInverse(/* kRightTurn90DegGCenter */ {0., -16.}),
               maliput::common::assertion_error);
  EXPECT_THROW(u_turn_quadrant1_dut_->GInverse(/* kUTurnQuadrant1GCenter */ {-5.928203230275509, 4.}),
               maliput::common::assertion_error);
  EXPECT_THROW(
      slight_right_turn_quadrant3_dut_->GInverse(/* kSlightRightTurnQuadrant3GCenter */ {-22., 64.42562584220408}),
      maliput::common::assertion_error);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
