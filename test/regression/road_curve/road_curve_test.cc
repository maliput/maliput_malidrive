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
#include "maliput_malidrive/road_curve/road_curve.h"

#include <array>
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/matrix.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_math_compare.h>

#include "maliput_malidrive/road_curve/arc_ground_curve.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/test_utilities/function_stub.h"
#include "maliput_malidrive/test_utilities/ground_curve_stub.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

using maliput::math::Matrix3;
using maliput::math::RollPitchYaw;
using maliput::math::Vector2;
using maliput::math::Vector3;
using maliput::math::test::CompareMatrices;
using maliput::math::test::CompareVectors;

// TODO(maliput#336)  Move this to maliput and use the right gtest semantics.
#define CompareRollPitchYaw(rpy_a, rpy_b, angular_tolerance)                  \
  do {                                                                        \
    EXPECT_NEAR(rpy_a.roll_angle(), rpy_b.roll_angle(), angular_tolerance);   \
    EXPECT_NEAR(rpy_a.pitch_angle(), rpy_b.pitch_angle(), angular_tolerance); \
    EXPECT_NEAR(rpy_a.yaw_angle(), rpy_b.yaw_angle(), angular_tolerance);     \
  } while (false);

// Constructs a Function stub.
std::unique_ptr<Function> MakeFunctionStub(double f_result, double f_dot_result, double f_dot_dot_result,
                                           double p0_result, double p1_result, bool is_g1_contiguous_result) {
  return std::make_unique<FunctionStub>(f_result, f_dot_result, f_dot_dot_result, p0_result, p1_result,
                                        is_g1_contiguous_result);
}

// Constructs a zeroed stub Function.
std::unique_ptr<Function> MakeZeroedFunctionStub(double p0_result, double p1_result, bool is_g1_contiguous_result) {
  return MakeFunctionStub(0, 0, 0, p0_result, p1_result, is_g1_contiguous_result);
}

// Constructs a cubic polynomial Function @f$ F(p) = a p^3 + b p^2 + c p + d / p ∈ [p0; p1] @f$.
std::unique_ptr<Function> MakeCubicPolynomialFunction(double a, double b, double c, double d, double p0, double p1,
                                                      double linear_tolerance) {
  return std::make_unique<CubicPolynomial>(a, b, c, d, p0, p1, linear_tolerance);
}

// Constructs a GroundCurveStub.
std::unique_ptr<GroundCurve> MakeGroundCurveStub(const Vector2& g_result, const Vector2& g_dot_result,
                                                 double heading_result, double heading_dot_result,
                                                 double d_g_inverse_result, double arc_length_result,
                                                 double linear_tolerance_result, double p0_result, double p1_result,
                                                 bool is_g1_contiguous_result) {
  return std::make_unique<GroundCurveStub>(g_result, g_dot_result, heading_result, heading_dot_result,
                                           d_g_inverse_result, arc_length_result, linear_tolerance_result, p0_result,
                                           p1_result, is_g1_contiguous_result);
}

// Constructs a ArcGroundCurve.
std::unique_ptr<GroundCurve> MakeArcGroundCurve(double linear_tolerance, const Vector2& xy0, double heading,
                                                double curvature, double arc_length, double p0, double p1) {
  return std::make_unique<ArcGroundCurve>(linear_tolerance, xy0, heading, curvature, arc_length, p0, p1);
}

// Constructs a LineGroundCurve.
std::unique_ptr<GroundCurve> MakeLineGroundCurve(double linear_tolerance, const Vector2& xy0, const Vector2& dxy,
                                                 double p0, double p1) {
  return std::make_unique<LineGroundCurve>(linear_tolerance, xy0, dxy, p0, p1);
}

// Constructs a zeroed GroundCurveStub.
std::unique_ptr<GroundCurve> MakeZeroedGroundCurveStub(double p0_result, double p1_result,
                                                       bool is_g1_contiguous_result) {
  const double kLinearTolerance{1e-12};
  const double kZero{0.};
  const Vector2 kZero2d(0., 0.);
  return MakeGroundCurveStub(kZero2d, kZero2d, kZero, kZero, kZero, p1_result - p0_result, kLinearTolerance, p0_result,
                             p1_result, is_g1_contiguous_result);
}

GTEST_TEST(MalidriveRoadCurveConstructorTest, CorrectlyConstructed) {
  const double kLinearTolerance{1e-15};
  const double kScaleLength{1};
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};

  // Correctly constructed RoadCurve.
  EXPECT_NO_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity));
}

GTEST_TEST(MalidriveRoadCurveConstructorTest, NegativeLinearTolerance) {
  const double kLinearTolerance{-1e-12};
  const double kScaleLength{1};
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};

  // Throws because of negative linear tolerance.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
}

GTEST_TEST(MalidriveRoadCurveConstructorTest, NegativeScaleLength) {
  const double kLinearTolerance{1e-15};
  const double kScaleLength{-1};
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};

  // Throws because of negative scale length.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
}

GTEST_TEST(MalidriveRoadCurveConstructorTest, NotG1Functions) {
  const double kLinearTolerance{1e-15};
  const double kScaleLength{1};
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  const bool kIsNotG1Contiguous{false};
  const bool kAssertContiguity{true};
  const bool kDontAssertContiguity{false};

  // Throws because GroundCurve is not G1.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsNotG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  // Throws because Elevation is not G1.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsNotG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  // Throws because Superelevation is not G1.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kP0, kP1, kIsNotG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  // Elevation is not G1 but contiguity is not asserted.
  EXPECT_NO_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                            MakeZeroedFunctionStub(kP0, kP1, kIsNotG1Contiguous),
                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kDontAssertContiguity));
  // Superelevation is not G1 but contiguity is not asserted.
  EXPECT_NO_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                            MakeZeroedFunctionStub(kP0, kP1, kIsNotG1Contiguous), kDontAssertContiguity));
}

GTEST_TEST(MalidriveRoadCurveConstructorTest, WrongRange) {
  const double kLinearTolerance{1e-15};
  const double kScaleLength{1};
  const double kPA{0.};
  const double kPB{10.};
  const double kPC{90.};
  const double kPD{100.};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};

  // GroundCurve has a different range.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kPA, kPC, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kPB, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  // Elevation has a different range.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPC, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPB, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  // Superelevation has a different range.
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPC, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
  EXPECT_THROW(RoadCurve(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPA, kPD, kIsG1Contiguous),
                         MakeZeroedFunctionStub(kPB, kPD, kIsG1Contiguous), kAssertContiguity),
               maliput::common::assertion_error);
}

GTEST_TEST(MalidriveRoadCurveAccessorsTest, Accessors) {
  const double kLinearTolerance{1e-15};
  const double kScaleLength{1};
  const double kP0{0.};
  const double kP1{100.};
  const double kLMax{kP1 - kP0};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};

  // Correctly constructed RoadCurve.
  const RoadCurve dut(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                      MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                      MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);

  EXPECT_EQ(kP0, dut.p0());
  EXPECT_EQ(kP1, dut.p1());
  EXPECT_EQ(kLMax, dut.LMax());
}

GTEST_TEST(MalidriveRoadCurveWTest, ParameterIsNotInRange) {
  const double kLinearTolerance{1e-15};
  const double kScaleLength{1};
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};

  const RoadCurve dut(kLinearTolerance, kScaleLength, MakeZeroedGroundCurveStub(kP0, kP1, kIsG1Contiguous),
                      MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                      MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);

  EXPECT_THROW(dut.W({2 * kP1, 0., 0.}), maliput::common::assertion_error);
}

// Provides constants and different type of roads for API tests.
class MalidriveRoadCurveStubTest : public ::testing::Test {
 public:
  // RoadCurve constants.
  //@{ // Arbitrary low constants that allowed comparisons with Python float resolution.
  const double kAngularTolerance{1e-14};
  const double kLinearTolerance{1e-14};
  //@}
  const double kScaleLength{1.};
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  // GroundCurve constants.
  const Vector2 kG{10., 20.};
  const Vector2 kGDot{1., std::sqrt(3.)};
  const double kHeading{M_PI / 3.};
  const double kHeadingDot{1.2};
  const double kZeroHeading{0.};
  // Elevation constants.
  const double kZ{30.};
  const double kZDot{0.2};
  const double kZDotDot{-0.1};
  // Superelevation constants.
  const double kTheta{M_PI / 5.};
  const double kDTheta{0.3};

  // @{  These constants here are referenced in comments below.
  // const double kCosHeading{std::cos(kHeading)};
  // const double kSinHeading{std::sin(kHeading)};
  // const double kCosTheta{std::cos(kTheta)};
  // const double kSinTheta{std::sin(kTheta)};
  // const double kBeta{-std::atan(kZDot / kGDot.norm())};
  // const double kCosBeta{std::cos(kBeta)};
  // const double kSinBeta{std::sin(kBeta)};
  // const double kDAlpha{kDTheta};
  // const double kDBeta{-kCosBeta * kCosBeta * kZDotDot / kGDot.norm()};
  // const double kDGamma{kHeadingDot};
  // @}

  const double kP{50.};
  const double kR{3.};
  const double kH{1.};
  const double kZero{0.};
  const bool kAssertContiguity{true};

  std::unique_ptr<RoadCurve> flat_dut_;
  std::unique_ptr<RoadCurve> elevated_dut_;
  std::unique_ptr<RoadCurve> superelevated_dut_;
  std::unique_ptr<RoadCurve> pitched_dut_;

 protected:
  void SetUp() override {
    flat_dut_ = std::make_unique<RoadCurve>(kLinearTolerance, kScaleLength,
                                            MakeGroundCurveStub(kG, kGDot, kHeading, kHeadingDot, kP, kP1 - kP0,
                                                                kLinearTolerance, kP0, kP1, kIsG1Contiguous),
                                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    elevated_dut_ = std::make_unique<RoadCurve>(kLinearTolerance, kScaleLength,
                                                MakeGroundCurveStub(kG, kGDot, kHeading, kHeadingDot, kP, kP1 - kP0,
                                                                    kLinearTolerance, kP0, kP1, kIsG1Contiguous),
                                                MakeFunctionStub(kZ, kZero, kZero, kP0, kP1, kIsG1Contiguous),
                                                MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    superelevated_dut_ = std::make_unique<RoadCurve>(
        kLinearTolerance, kScaleLength,
        MakeGroundCurveStub(kG, kGDot, kHeading, kHeadingDot, kP, kP1 - kP0, kLinearTolerance, kP0, kP1,
                            kIsG1Contiguous),
        MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
        MakeFunctionStub(kTheta, kDTheta, kZero, kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    pitched_dut_ = std::make_unique<RoadCurve>(kLinearTolerance, kScaleLength,
                                               MakeGroundCurveStub(kG, kGDot, kHeading, kHeadingDot, kP, kP1 - kP0,
                                                                   kLinearTolerance, kP0, kP1, kIsG1Contiguous),
                                               MakeFunctionStub(kZ, kZDot, kZDotDot, kP0, kP1, kIsG1Contiguous),
                                               MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
  }
};

class MalidriveRoadCurveStubWTest : public MalidriveRoadCurveStubTest {};

TEST_F(MalidriveRoadCurveStubWTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZero}, flat_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZ}, elevated_dut_->W(kPRH), kLinearTolerance));
  //@{ At the centerline, the superelevation and elevation (with pitched roads) make no difference because the
  //   curve is the center of rotation.
  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZero}, superelevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZ}, pitched_dut_->W(kPRH), kLinearTolerance));
  //@}
}

TEST_F(MalidriveRoadCurveStubWTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(
      CompareVectors({/* kG.x() - kR * kSinHeading */ 7.401923788646684, /* kG.y() + kR * kCosHeading */ 21.5, kZero},
                     flat_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors({/* kG.x() - kR * kSinHeading */ 7.401923788646684, /* kG.y() + kR * kCosHeading */ 21.5, kZ},
                     elevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() - kR * kSinHeading * kCosTheta */ 7.89811219233389,
                              /* kG.y() + kR * kCosHeading * kCosTheta */ 21.21352549156242,
                              /* kR * kSinTheta */ 1.763355756877419},
                             superelevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors({/* kG.x() - kR * kSinHeading */ 7.401923788646684, /* kG.y() + kR * kCosHeading */ 21.5, kZ},
                     pitched_dut_->W(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubWTest, WithAVerticalOffset) {
  const maliput::math::Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors(flat_dut_->W(kPRH), {kG.x(), kG.y(), kH}, kLinearTolerance));
  EXPECT_TRUE(CompareVectors(elevated_dut_->W(kPRH), {kG.x(), kG.y(), kZ + kH}, kLinearTolerance));
  EXPECT_TRUE(CompareVectors(superelevated_dut_->W(kPRH),
                             {/* kG.x() + kH * kSinHeading * kSinTheta */ 10.50903696045513,
                              /* kG.y() - kH * kCosHeading * kSinTheta */ 19.70610737385377,
                              /* kH * kCosTheta */ 0.8090169943749475},
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(pitched_dut_->W(kPRH),
                             {/* kG.x() + kH * kCosHeading * kSinBeta */ 9.950248140489501,
                              /* kG.y() + kH * kSinHeading * kSinBeta */ 19.913827251556786,
                              /* kZ + kH * kCosBeta */ 30.995037190209988},
                             kLinearTolerance));
}

class MalidriveRoadCurveStubOrientationOfPTest : public MalidriveRoadCurveStubTest {};

TEST_F(MalidriveRoadCurveStubOrientationOfPTest, OrientatationOfP) {
  CompareRollPitchYaw(RollPitchYaw(kZero, kZero, kHeading), flat_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kZero, kZero, kHeading), elevated_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kZero, /* -atan(kZDot / kGDot.norm()) */ -0.09966865249116204, kHeading),
                      pitched_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kTheta, kZero, kHeading), superelevated_dut_->Orientation(kP), kAngularTolerance);
}

class MalidriveRoadCurveStubWDotTest : public MalidriveRoadCurveStubTest {};

TEST_F(MalidriveRoadCurveStubWDotTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, superelevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZDot}, pitched_dut_->WDot(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubWDotTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors({/*kGDot.x() -kCosHeading * kR * kDGamma*/ -0.8000000000000005,
                              /*kGDot.y() - kSinHeading * kR * kDGamma*/ -1.3856406460551018, 0.},
                             flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/*kGDot.x() -kCosHeading * kR * kDGamma*/ -0.8000000000000005,
                              /*kGDot.y() - kSinHeading * kR * kDGamma*/ -1.3856406460551018, 0.},
                             elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      {/*kGDot.x() - kCosTheta * kCosHeading * kR * kDGamma + kSinTheta * kSinHeading * kR * kDTheta*/
       0.0019026745347084928,
       /*kGDot.y() -kCosTheta * kSinHeading * kR * kDGamma -kSinTheta * kCosHeading * kR * kDTheta*/
       -1.0547179251620677,
       /*kCosTheta * kR * kDTheta*/ 0.7281152949374526},
      superelevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/*kGDot.x() - kCosHeading * kR * kDGamma*/ -0.8000000000000005,
                              /*kGDot.y() - kSinHeading * kR * kDGamma*/ -1.3856406460551018,
                              /*kZDot*/ 0.2},
                             pitched_dut_->WDot(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubWDotTest, WithAVerticalOffset) {
  const Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      {/*kGDot.x()  + kSinTheta * kCosHeading * kH * kDGamma + kCosTheta * kSinHeading * kH * kDAlpha*/
       1.5628599321420948,
       /*kGDot.y() + kSinTheta * kSinHeading * kH * kDGamma - kCosTheta * kCosHeading * kH * kDAlpha*/
       2.2215426109587875,
       /*-kSinTheta * kH * kDAlpha*/ -0.17633557568774194},
      superelevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      {/*kGDot.x() + kCosBeta * kCosHeading * kDBeta * kH - kSinBeta * kSinHeading * kDGamma * kH*/ 1.1280369315528962,
       /*kGDot.y() + kCosBeta * kSinHeading * kDBeta * kH + kSinBeta * kCosHeading * kDGamma * kH*/ 1.7150083526133144,
       /*kZDot - kSinBeta * kDBeta * kH*/ 0.2049259266842079},
      pitched_dut_->WDot(kPRH), kLinearTolerance));
}

class MalidriveRoadCurveStubNormalizedDerivatives : public MalidriveRoadCurveStubTest {
 public:
  const Vector3 kExpectedSHat{0.5000000000000001, 0.8660254037844387, kZero};
};

//@{ Expected values in these tests are the same as in the MalidriveRoadCurveStubWDotTest but normalized.
class MalidriveRoadCurveStubSHatTest : public MalidriveRoadCurveStubNormalizedDerivatives {};

TEST_F(MalidriveRoadCurveStubSHatTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};
  const Vector3 kExpectedPitchedSHat{0.49751859510499463, 0.8617274844321391, 0.09950371902099893};

  EXPECT_TRUE(CompareVectors(kExpectedSHat, flat_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSHat, elevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSHat, superelevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedPitchedSHat, pitched_dut_->SHat(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubSHatTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};
  const Vector3 kExpectedFlatSHat{-0.5000000000000002, -0.8660254037844386, kZero};
  const Vector3 kExpectedSuperelevatedSHat{0.0014845694727125196, -0.8229479111929087, 0.5681148928840971};
  const Vector3 kExpectedPitchedSHat{-0.4961389383568341, -0.8593378488473195, 0.12403473458920845};

  EXPECT_TRUE(CompareVectors(kExpectedFlatSHat, flat_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedFlatSHat, elevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSuperelevatedSHat, superelevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedPitchedSHat, pitched_dut_->SHat(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubSHatTest, WithAVerticalOffset) {
  const Vector3 kPRH{kP, kZero, kH};
  const Vector3 kExpectedSuperelevatedSHat{0.5741744530994537, 0.8161659195754601, -0.06478340166652961};
  const Vector3 kExpectedPitchedSHat{0.5468107194094988, 0.8313424187227605, 0.099336901356226};

  EXPECT_TRUE(CompareVectors(kExpectedSHat, flat_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSHat, elevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSuperelevatedSHat, superelevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedPitchedSHat, pitched_dut_->SHat(kPRH), kLinearTolerance));
}
//@}

class MalidriveRoadCurveStubRHatTest : public MalidriveRoadCurveStubNormalizedDerivatives {
 public:
  // Expected derivative is orthonormal to s_hat at the centerline.
  const Vector3 kExpectedRHat{-kExpectedSHat.y(), kExpectedSHat.x(), kExpectedSHat.z()};
};

// Vectors can be evaluated using numpy and scipy:
//
// @code{python}
//
// from scipy.spatial.transform import Rotation as R
// import numpy as np
//
// np.matmul(R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix(), [0, 1, 0])
//
// @endcode
TEST_F(MalidriveRoadCurveStubRHatTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};
  // Expected derivative is orthonormal to s_hat at the centerline but it is shifted because of superelevation.
  const Vector3 kExpectedSuperelevatedRHat{-0.70062926922203672, 0.40450849718747384, 0.58778525229247314};

  EXPECT_TRUE(CompareVectors(kExpectedRHat, flat_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedRHat, elevated_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSuperelevatedRHat, superelevated_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedRHat, pitched_dut_->RHat(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubRHatTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};
  const Vector3 kExpectedFlatSHat{-0.5000000000000002, -0.8660254037844386, kZero};
  // Expected derivative is orthonormal to s_hat at the lateral offset
  const Vector3 kExpectedFlatRHat{-kExpectedFlatSHat.y(), kExpectedFlatSHat.x(), kExpectedFlatSHat.z()};
  const Vector3 kExpectedSuperelevatedRHat{0.80841328164017134, 0.33538844850188776, 0.48371743268076017};

  EXPECT_TRUE(CompareVectors(kExpectedFlatRHat, flat_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedFlatRHat, elevated_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSuperelevatedRHat, superelevated_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedFlatRHat, pitched_dut_->RHat(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubRHatTest, WithAVerticalOffset) {
  const Vector3 kPRH{kP, kZero, kH};
  const Vector3 kExpectedSuperelevatedRHat{-0.63977220436668092, 0.49663871500361961, 0.58655052065389168};
  const Vector3 kExpectedPitchedRHat{-0.83547479582279804, 0.5495287668037534, 0};

  EXPECT_TRUE(CompareVectors(kExpectedRHat, flat_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedRHat, elevated_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedSuperelevatedRHat, superelevated_dut_->RHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedPitchedRHat, pitched_dut_->RHat(kPRH), kLinearTolerance));
}

class MalidriveRoadCurveStubOrientationOfPRHTest : public MalidriveRoadCurveStubTest {
 protected:
  Matrix3 MakeRotationMatrix(const Vector3& s_hat, const Vector3& r_hat) const {
    const Vector3 h_hat = s_hat.cross(r_hat);
    return Matrix3({s_hat[0], r_hat[0], h_hat[0], s_hat[1], r_hat[1], h_hat[1], s_hat[2], r_hat[2], h_hat[2]});
  }
};

// This test compares the rotation matrix of
// RoadCurve::Orientation(prh) with another constructed from the
// orthonormal vector basis s_hat, r_hat and h_hat.
TEST_F(MalidriveRoadCurveStubOrientationOfPRHTest, Orientation) {
  const Vector3 kAtCenterline{kP, kZero, kZero};
  const Vector3 kWithLateralOffset{kP, kR, kZero};
  // TODO(agalbachicar): Should add and fix the bug when kH is not zero.

  for (const Vector3& prh : {kAtCenterline, kWithLateralOffset}) {
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(flat_dut_->SHat(prh), flat_dut_->RHat(prh)),
                                flat_dut_->Orientation(prh).ToMatrix(), kLinearTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(elevated_dut_->SHat(prh), elevated_dut_->RHat(prh)),
                                elevated_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(superelevated_dut_->SHat(prh), superelevated_dut_->RHat(prh)),
                                superelevated_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(pitched_dut_->SHat(prh), pitched_dut_->RHat(prh)),
                                pitched_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
  }
}

class MalidriveRoadCurveStubWInverseTest : public MalidriveRoadCurveStubTest {};

TEST_F(MalidriveRoadCurveStubWInverseTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors(kPRH, flat_dut_->WInverse({kG.x(), kG.y(), kZero}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, elevated_dut_->WInverse({kG.x(), kG.y(), kZ}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, superelevated_dut_->WInverse({kG.x(), kG.y(), kZero}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, pitched_dut_->WInverse({kG.x(), kG.y(), kZ}), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubWInverseTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors(kPRH, flat_dut_->WInverse({7.401923788646684, 21.5, kZero}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, elevated_dut_->WInverse({7.401923788646684, 21.5, kZ}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      kPRH, superelevated_dut_->WInverse({7.89811219233389, 21.21352549156242, 1.763355756877419}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, pitched_dut_->WInverse({7.401923788646684, 21.5, kZ}), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveStubWInverseTest, WithAVerticalOffset) {
  const maliput::math::Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors(kPRH, flat_dut_->WInverse({kG.x(), kG.y(), kH}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, elevated_dut_->WInverse({kG.x(), kG.y(), kZ + kH}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH,
                             superelevated_dut_->WInverse({10.50903696045513, 19.70610737385377, 0.8090169943749475}),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, pitched_dut_->WInverse({9.950248140489501, 19.913827251556786, 30.995037190209988}),
                             kLinearTolerance));
}

// Use a LineGroundCurve
class MalidriveRoadCurveLineTest : public ::testing::Test {
 public:
  // LineGroundCurve constants.
  //@{ // Arbitrary low constants that allowed comparisons with Python float resolution.
  const double kAngularTolerance{1e-14};
  const double kLinearTolerance{1e-13};
  const double kScaleLength{1};
  //@}
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};
  // GroundCurve constants.
  const Vector2 kG0{10., 20.};
  const Vector2 kDG{10., 10 * std::sqrt(3.)};
  // Elevation constants.
  const double kZ0{30.};
  const double kZDot0{0.2};
  const double kZDotDot{-0.002};
  // No superelevation.
  const double kTheta0{0.};
  const double kDTheta{0.};

  // extra constants
  const double kP{50.};
  const double kR{3.};
  const double kH{1.};
  const double kZero{0.};
  // derived constants
  // kHeading{std::atan2(kDG[1], kDG[0]}
  const double kHeading{M_PI / 3.};
  const double kHeadingDot{0.};
  const Vector2 kG{kG0 + kDG * (kP - kP0) / (kP1 - kP0)};
  const Vector2 kGDot{kDG / (kP1 - kP0)};
  const double kZDot{kZDot0 + kZDotDot * kP};

  // Lane Offset constant.
  const double kLinearOffset{1. / 50.};

  // @{  These constants here are referenced in comments below.
  // const double kCosHeading{std::cos(kHeading)};
  // const double kSinHeading{std::sin(kHeading)};
  // const double kCosTheta{std::cos(kTheta)};
  // const double kSinTheta{std::sin(kTheta)};
  //
  // const Vector3 kFlatW{kG[0], kG[1], 0.};
  // const Vector3 kFlatDW{kDG[0], kDG[1], 0.};
  // const Vector3 kFlatWDot{kFlatDW / (kP1 - kP0)};
  // const double kFlatBeta{0.};
  // const double kFlatDBeta{0.};
  //
  // const Vector3 kElevatedW{kG[0], kG[1], kZ0};
  // const Vector3 kElevatedDW{kDG[0], kDG[1], 0.};
  // const Vector3 kElevatedWDot{kElevatedDW / (kP1 - kP0)};
  // const double kElevatedBeta{0.};
  // const double kElevatedDBeta{0.};
  //
  // const Vector3 kPitchedW_p0{kG0[0], kG0[1], kZ0};
  // const Vector3 kPitchedDW{kDG[0], kDG[1], kZDot0 * (kP1 - kP0)};
  // const Vector3 kPitchedW{kPitchedW_p0 + kPitchedDW * (kP - kP0) / (kP1 - kP0)};
  // const Vector3 kPitchedWDot{kPitchedDW / (kP1 - kP0)};
  // const double kPitchedBeta{-std::atan2(kZDot0, kGDot.norm())};
  // const double kPitchedCosBeta{std::cos(kPitchedBeta)};
  // const double kPitchedSinBeta{std::sin(kPitchedBeta)};
  // const double kPitchedDBeta{0.};
  //
  // const Vector3 kCrestW{kG[0], kG[1], kZ0 + kZDot0 * kP + 0.5 * kZDotDot * kP * kP};
  // const Vector3 kCrestWDot{kGDot[0], kGDot[1], kZDot};
  // const double kCrestBeta{-std::atan2(kZDot, kGDot.norm())};
  // const double kCrestCosBeta{std::cos(kCrestBeta)};
  // const double kCrestSinBeta{std::sin(kCrestBeta)};
  // const double kCrestDBeta{-kZDotDot * std::pow(kCrestCosBeta, 2) / kGDot.norm()};
  //
  // const double kCosBeta{std::cos(kBeta)};
  // const double kSinBeta{std::sin(kBeta)};
  // const double kDAlpha{kDTheta};
  // const double kDBeta{-kCosBeta * kCosBeta * kZDotDot / (kP1 - kP0)};
  // const double kDGamma{kHeadingDot};
  // @}

  std::unique_ptr<Function> lane_offset_;
  std::unique_ptr<RoadCurve> flat_dut_;
  std::unique_ptr<RoadCurve> elevated_dut_;
  std::unique_ptr<RoadCurve> pitched_dut_;
  std::unique_ptr<RoadCurve> crest_dut_;

 protected:
  void SetUp() override {
    lane_offset_ = MakeCubicPolynomialFunction(kZero, kZero, kLinearOffset, kZero, kP0, kP1, kLinearTolerance);
    flat_dut_ = std::make_unique<RoadCurve>(kLinearTolerance, kScaleLength,
                                            MakeLineGroundCurve(kLinearTolerance, kG0, kDG, kP0, kP1),
                                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
                                            MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    elevated_dut_ = std::make_unique<RoadCurve>(kLinearTolerance, kScaleLength,
                                                MakeLineGroundCurve(kLinearTolerance, kG0, kDG, kP0, kP1),
                                                MakeFunctionStub(kZ0, kZero, kZero, kP0, kP1, kIsG1Contiguous),
                                                MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    pitched_dut_ = std::make_unique<RoadCurve>(
        kLinearTolerance, kScaleLength, MakeLineGroundCurve(kLinearTolerance, kG0, kDG, kP0, kP1),
        MakeCubicPolynomialFunction(0., 0., kZDot0, kZ0, kP0, kP1, kLinearTolerance),
        MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    crest_dut_ = std::make_unique<RoadCurve>(
        kLinearTolerance, kScaleLength, MakeLineGroundCurve(kLinearTolerance, kG0, kDG, kP0, kP1),
        MakeCubicPolynomialFunction(0., 0.5 * kZDotDot, kZDot0, kZ0, kP0, kP1, kLinearTolerance),
        MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
  }
};

class MalidriveRoadCurveLineWDotTest : public MalidriveRoadCurveLineTest {};

TEST_F(MalidriveRoadCurveLineWDotTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZDot0}, pitched_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZDot}, crest_dut_->WDot(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWDotTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZDot0}, pitched_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZDot}, crest_dut_->WDot(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWDotTest, WithAVerticalOffset) {
  const Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZero}, elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kGDot.x(), kGDot.y(), kZDot0}, pitched_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() + kH * kCrestDBeta * kCrestCosBeta * kCosHeading */ 0.10357770876399967,
                              /* kGDot.y() + kH * kCrestDBeta * kCrestCosBeta * kSinHeading */ 0.17940185411081958,
                              /* kZDot - kH * kCrestDBeta * kCrestSinBeta */ 0.10357770876399967},
                             crest_dut_->WDot(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWDotTest, WithVariableOffset) {
  const Vector3 kPRH{kP, lane_offset_->f(kP), kZero};
  const Vector3 kExpectedWDot{0.082679491924311, 0.183205080756888, 0.};
  const Vector3 kExpectedSHat{0.411345846661781, 0.911479343942639, 0.};
  EXPECT_TRUE(CompareVectors(kExpectedWDot, flat_dut_->WDot(kPRH, lane_offset_.get()), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedWDot.normalized(), flat_dut_->SHat(kPRH, lane_offset_.get()), kLinearTolerance));
}

class MalidriveRoadCurveLineOrientationOfPTest : public MalidriveRoadCurveLineTest {};

TEST_F(MalidriveRoadCurveLineOrientationOfPTest, OrientatationOfP) {
  CompareRollPitchYaw(RollPitchYaw(kZero, kZero, kHeading), flat_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kZero, kZero, kHeading), elevated_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kZero, /* kPitchedBeta */ -0.7853981633974484, kHeading),
                      pitched_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kZero, /* kCrestBeta */ -0.46364760900080615, kHeading), crest_dut_->Orientation(kP),
                      kAngularTolerance);
}

class MalidriveRoadCurveLineOrientationOfPRHTest : public MalidriveRoadCurveLineTest {
 protected:
  Matrix3 MakeRotationMatrix(const Vector3& s_hat, const Vector3& r_hat) const {
    const Vector3 h_hat = s_hat.cross(r_hat);
    return Matrix3({s_hat[0], r_hat[0], h_hat[0], s_hat[1], r_hat[1], h_hat[1], s_hat[2], r_hat[2], h_hat[2]});
  }
};

// This test compares the rotation matrix of
// RoadCurve::Orientation(prh) with another constructed from the
// orthonormal vector basis s_hat, r_hat and h_hat.
TEST_F(MalidriveRoadCurveLineOrientationOfPRHTest, Orientation) {
  const Vector3 kAtCenterline{kP, kZero, kZero};
  const Vector3 kWithLateralOffset{kP, kR, kZero};
  const Vector3 kWithVerticalOffset{kP, kZero, kH};

  for (const Vector3& prh : {kAtCenterline, kWithLateralOffset, kWithVerticalOffset}) {
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(flat_dut_->SHat(prh), flat_dut_->RHat(prh)),
                                flat_dut_->Orientation(prh).ToMatrix(), kLinearTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(elevated_dut_->SHat(prh), elevated_dut_->RHat(prh)),
                                elevated_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(pitched_dut_->SHat(prh), pitched_dut_->RHat(prh)),
                                pitched_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(crest_dut_->SHat(prh), crest_dut_->RHat(prh)),
                                crest_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
  }
}

TEST_F(MalidriveRoadCurveLineOrientationOfPRHTest, WithVariableOffset) {
  const Vector3 kPRH{kP, lane_offset_->f(kP), kZero};
  const RollPitchYaw kExpectedOrientation(0., 0., 1.1468662036877597);
  const auto orientation{flat_dut_->Orientation(kPRH, lane_offset_.get())};
  EXPECT_NEAR(kExpectedOrientation.roll_angle(), orientation.roll_angle(), kAngularTolerance);
  EXPECT_NEAR(kExpectedOrientation.pitch_angle(), orientation.pitch_angle(), kAngularTolerance);
  EXPECT_NEAR(kExpectedOrientation.yaw_angle(), orientation.yaw_angle(), kAngularTolerance);
}

class MalidriveRoadCurveLineWTest : public MalidriveRoadCurveLineTest {};

TEST_F(MalidriveRoadCurveLineWTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZero}, flat_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZ0}, elevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZ0 + kZDot0 * kP}, pitched_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZ0 + kZDot0 * kP + 0.5 * kZDotDot * kP * kP}, crest_dut_->W(kPRH),
                             kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                              /* kG.y() + kR * kCosHeading */ 30.160254037844386, kZero},
                             flat_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                              /* kG.y() + kR * kCosHeading */ 30.160254037844386, kZ0},
                             elevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                              /* kG.y() + kR * kCosHeading */ 30.160254037844386, kZ0 + kZDot0 * kP},
                             pitched_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                      /* kG.y() + kR * kCosHeading */ 30.160254037844386, kZ0 + kZDot0 * kP + 0.5 * kZDotDot * kP * kP},
                     crest_dut_->W(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWTest, WithAVerticalOffset) {
  const maliput::math::Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kH}, flat_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({kG.x(), kG.y(), kZ0 + kH}, elevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(pitched_dut_->W(kPRH),
                             {/* kG.x() + kH * kCosHeading * kPitchedSinBeta */ 14.646446609406727,
                              /* kG.y() + kH * kSinHeading * kPitchedSinBeta */ 28.047881602148593,
                              /* kZ0 + kZDot0 * kP + kH * kPitchedCosBeta */ 40.707106781186546},
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(crest_dut_->W(kPRH),
                             {/* kG.x() + kH * kCosHeading * kCrestSinBeta */ 14.776393202250022,
                              /* kG.y() + kH * kSinHeading * kCrestSinBeta */ 28.272955703223644,
                              /* kZ0 + kZDot0*kP + 0.5*kZDotDot*kP*kP + kH*kCrestCosBeta */ 38.39442719099991},
                             kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWTest, WithVariableOffset) {
  const Vector3 kPRH{kP, lane_offset_->f(kP), kZero};
  const Vector3 kExpectedW{14.13397459621556, 29.160254037844386, 0.};
  EXPECT_TRUE(CompareVectors(kExpectedW, flat_dut_->W(kPRH), kLinearTolerance));
}

class MalidriveRoadCurveLineWInverseTest : public MalidriveRoadCurveLineTest {};

TEST_F(MalidriveRoadCurveLineWInverseTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors(kPRH, flat_dut_->WInverse({kG.x(), kG.y(), kZero}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, elevated_dut_->WInverse({kG.x(), kG.y(), kZ0}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, pitched_dut_->WInverse({kG.x(), kG.y(), kZ0 + kZDot0 * kP}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, crest_dut_->WInverse({kG.x(), kG.y(), kZ0 + kZDot0 * kP + 0.5 * kZDotDot * kP * kP}),
                             kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWInverseTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors(kPRH,
                             flat_dut_->WInverse({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                                                  /* kG.y() + kR * kCosHeading */ 30.160254037844386, kZero}),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH,
                             elevated_dut_->WInverse({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                                                      /* kG.y() + kR * kCosHeading */ 30.160254037844386, kZ0}),
                             kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors(kPRH,
                     pitched_dut_->WInverse({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                                             /* kG.y() + kR * kCosHeading */ 30.160254037844386, kZ0 + kZDot0 * kP}),
                     kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH,
                             crest_dut_->WInverse({/* kG.x() - kR * kSinHeading */ 12.401923788646684,
                                                   /* kG.y() + kR * kCosHeading */ 30.160254037844386,
                                                   kZ0 + kZDot0 * kP + 0.5 * kZDotDot * kP * kP}),
                             kLinearTolerance));
}

TEST_F(MalidriveRoadCurveLineWInverseTest, WithAVerticalOffset) {
  const maliput::math::Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors(kPRH, flat_dut_->WInverse({kG.x(), kG.y(), kH}), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH, elevated_dut_->WInverse({kG.x(), kG.y(), kZ0 + kH}), kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors(kPRH,
                     pitched_dut_->WInverse({/* kG.x() + kH * kCosHeading * kPitchedSinBeta */ 14.646446609406727,
                                             /* kG.y() + kH * kSinHeading * kPitchedSinBeta */ 28.047881602148593,
                                             /* kZ0 + kZDot0 * kP + kH * kPitchedCosBeta */ 40.707106781186546}),
                     kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      kPRH,
      crest_dut_->WInverse({/* kG.x() + kH * kCosHeading * kCrestSinBeta */ 14.776393202250022,
                            /* kG.y() + kH * kSinHeading * kCrestSinBeta */ 28.272955703223644,
                            /* kZ0 + kZDot0*kP + 0.5*kZDotDot*kP*kP + kH*kCrestCosBeta */ 38.39442719099991}),
      kLinearTolerance));
}

// Use an ArcGroundCurve
class MalidriveRoadCurveArcTest : public ::testing::Test {
 public:
  // ArcGroundCurve constants.
  //@{ // Arbitrary low constants that allowed comparisons with Python float resolution.
  const double kAngularTolerance{1e-14};
  const double kLinearTolerance{1e-13};
  const double kScaleLength{1};
  //@}
  const double kP0{0.};
  const double kP1{100.};
  const bool kIsG1Contiguous{true};
  const bool kAssertContiguity{true};
  // GroundCurve constants corresponding to 180 degree U-turn.
  const Vector2 kG0{10., 20.};
  const double kHeading0{M_PI / 3.};
  const double kArcLength{M_PI * 8.};
  const double kArcAngle{M_PI};
  // Elevation constants.
  const double kZ0{30.};
  const double kZDot0{0.2};
  const double kZDotDot{-0.002};
  // Constant superelevation.
  const double kTheta{-M_PI / 12.};
  const double kDTheta{0.};

  // extra constants
  const double kP{50.};
  const double kR{3.};
  const double kH{1.};
  const double kZero{0.};
  // derived constants
  const double kCurvature{kArcAngle / kArcLength};
  const double kRadius{1. / kCurvature};
  const double kHeading{kHeading0 + kArcAngle * (kP - kP0) / (kP1 - kP0)};
  const double kHeadingDot{kArcAngle / (kP1 - kP0)};
  const double kZDot{kZDot0 + kZDotDot * kP};

  // @{  These constants here are referenced in comments below.
  // const double kCosHeading0{std::cos(kHeading)};
  // const double kSinHeading0{std::sin(kHeading)};
  // const double kCosHeading{std::cos(kHeading)};
  // const double kSinHeading{std::sin(kHeading)};
  // const double kCosTheta{std::cos(kTheta)};
  // const double kSinTheta{std::sin(kTheta)};
  //
  // const Vector2 kCenter{kG0 + kRadius * Vector2{-kSinHeading0, kCosHeading0}};
  // const Vector2 kG{kCenter - kRadius * Vector2{-kSinHeading, kCosHeading}};
  // const Vector2 kGDot{kArcLength / (kP1 - kP0) * Vector2{kCosHeading, kSinHeading}};
  //
  // const double kPitchedBeta{-std::atan2(kZDot0, kGDot.norm())};
  // const double kCosPitchedBeta{std::cos(kPitchedBeta)};
  // const double kSinPitchedBeta{std::sin(kPitchedBeta)};
  // // pitched road curve with lateral offset
  // const double kPitchedRBeta{-std::atan2(kZDot0, (1 - kR / kRadius) * kGDot.norm())};
  // const double kCosPitchedRBeta{std::cos(kPitchedRBeta)};
  // const double kSinPitchedRBeta{std::sin(kPitchedRBeta)};
  // @}

  std::unique_ptr<RoadCurve> flat_dut_;
  std::unique_ptr<RoadCurve> elevated_dut_;
  std::unique_ptr<RoadCurve> pitched_dut_;
  std::unique_ptr<RoadCurve> superelevated_dut_;

 protected:
  void SetUp() override {
    flat_dut_ = std::make_unique<RoadCurve>(
        kLinearTolerance, kScaleLength,
        MakeArcGroundCurve(kLinearTolerance, kG0, kHeading0, kCurvature, kArcLength, kP0, kP1),
        MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
        kAssertContiguity);
    elevated_dut_ = std::make_unique<RoadCurve>(
        kLinearTolerance, kScaleLength,
        MakeArcGroundCurve(kLinearTolerance, kG0, kHeading0, kCurvature, kArcLength, kP0, kP1),
        MakeFunctionStub(kZ0, kZero, kZero, kP0, kP1, kIsG1Contiguous),
        MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    pitched_dut_ = std::make_unique<RoadCurve>(
        kLinearTolerance, kScaleLength,
        MakeArcGroundCurve(kLinearTolerance, kG0, kHeading0, kCurvature, kArcLength, kP0, kP1),
        MakeCubicPolynomialFunction(0., 0., kZDot0, kZ0, kP0, kP1, kLinearTolerance),
        MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous), kAssertContiguity);
    superelevated_dut_ = std::make_unique<RoadCurve>(
        kLinearTolerance, kScaleLength,
        MakeArcGroundCurve(kLinearTolerance, kG0, kHeading0, kCurvature, kArcLength, kP0, kP1),
        MakeZeroedFunctionStub(kP0, kP1, kIsG1Contiguous),
        MakeFunctionStub(kTheta, kZero, kZero, kP0, kP1, kIsG1Contiguous), kAssertContiguity);
  }
};

class MalidriveRoadCurveArcWDotTest : public MalidriveRoadCurveArcTest {};

TEST_F(MalidriveRoadCurveArcWDotTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors({/* kGDot.x() */ -0.2176559237081061, /* kGDot.y() */ 0.12566370614359182, kZero},
                             flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() */ -0.2176559237081061, /* kGDot.y() */ 0.12566370614359182, kZero},
                             elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() */ -0.2176559237081061, /* kGDot.y() */ 0.12566370614359182, kZDot0},
                             pitched_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() */ -0.2176559237081061, /* kGDot.y() */ 0.12566370614359182, kZero},
                             superelevated_dut_->WDot(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcWDotTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors({/* kGDot.x() - kR * kHeadingDot * kCosHeading*/ -0.1360349523175663,
                              /* kGDot.y() - kR * kHeadingDot * kSinHeading*/ 0.07853981633974488, kZero},
                             flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() - kR * kHeadingDot * kCosHeading*/ -0.1360349523175663,
                              /* kGDot.y() - kR * kHeadingDot * kSinHeading*/ 0.07853981633974488, kZero},
                             elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() - kR * kHeadingDot * kCosHeading*/ -0.1360349523175663,
                              /* kGDot.y() - kR * kHeadingDot * kSinHeading*/ 0.07853981633974488, kZDot0},
                             pitched_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() - kR * kHeadingDot * kCosHeading * kCosTheta*/ -0.13881611947518255,
                              /* kGDot.y() - kR * kHeadingDot * kSinHeading * kCosTheta*/ 0.08014552394685596, kZero},
                             superelevated_dut_->WDot(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcWDotTest, WithAVerticalOffset) {
  const Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors({/* kGDot.x() */ -0.2176559237081061, /* kGDot.y() */ 0.12566370614359182, kZero},
                             flat_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() */ -0.2176559237081061, /* kGDot.y() */ 0.12566370614359182, kZero},
                             elevated_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors({/* kGDot.x() + kH * kHeadingDot * kSinHeading * kSinPitchedBeta */ -0.2078749363852686,
                      /* kGDot.y() - kH * kHeadingDot * kCosHeading * kSinPitchedBeta */ 0.14260487313493342, kZDot0},
                     pitched_dut_->WDot(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kGDot.x() + kH * kHeadingDot * kSinHeading * kSinTheta*/ -0.2106142364162262,
                              /* kGDot.y() + kH * kHeadingDot * kCosHeading * kSinTheta*/ 0.12159818609007579, kZero},
                             superelevated_dut_->WDot(kPRH), kLinearTolerance));
}

class MalidriveRoadCurveArcSHatTest : public MalidriveRoadCurveArcTest {};

TEST_F(MalidriveRoadCurveArcSHatTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             flat_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             elevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading * kCosPitchedBeta */ -0.6776466796536645,
                              /* kSinHeading * kCosPitchedBeta */ 0.39123949291349935,
                              /* kSinPitchedBeta */ 0.6226769922994999},
                             pitched_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             superelevated_dut_->SHat(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcSHatTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             flat_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             elevated_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading * kCosPitchedRBeta */ -0.5349160274107262,
                              /* kSinHeading * kCosPitchedRBeta */ 0.30883391241942804,
                              /* kSinPitchedRBeta */ 0.7864391000953832},
                             pitched_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             superelevated_dut_->SHat(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcSHatTest, WithAVerticalOffset) {
  const Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             flat_dut_->SHat(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             elevated_dut_->SHat(kPRH), kLinearTolerance));
  // expected values normalized from WDot test expectations above
  EXPECT_TRUE(CompareVectors({-0.6459977126878251, 0.44316271824404935, 0.6215253497329318}, pitched_dut_->SHat(kPRH),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kCosHeading */ -0.8660254037844386, /* kSinHeading */ 0.5, kZero},
                             superelevated_dut_->SHat(kPRH), kLinearTolerance));
}

class MalidriveRoadCurveArcSHatRHatOrthogonalTest : public MalidriveRoadCurveArcTest {};

TEST_F(MalidriveRoadCurveArcSHatRHatOrthogonalTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_NEAR(kZero, flat_dut_->SHat(kPRH).dot(flat_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, elevated_dut_->SHat(kPRH).dot(elevated_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, pitched_dut_->SHat(kPRH).dot(pitched_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, superelevated_dut_->SHat(kPRH).dot(superelevated_dut_->RHat(kPRH)), kLinearTolerance);
}

TEST_F(MalidriveRoadCurveArcSHatRHatOrthogonalTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_NEAR(kZero, flat_dut_->SHat(kPRH).dot(flat_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, elevated_dut_->SHat(kPRH).dot(elevated_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, pitched_dut_->SHat(kPRH).dot(pitched_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, superelevated_dut_->SHat(kPRH).dot(superelevated_dut_->RHat(kPRH)), kLinearTolerance);
}

TEST_F(MalidriveRoadCurveArcSHatRHatOrthogonalTest, WithAVerticalOffset) {
  const Vector3 kPRH{kP, kZero, kH};

  EXPECT_NEAR(kZero, flat_dut_->SHat(kPRH).dot(flat_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, elevated_dut_->SHat(kPRH).dot(elevated_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, pitched_dut_->SHat(kPRH).dot(pitched_dut_->RHat(kPRH)), kLinearTolerance);
  EXPECT_NEAR(kZero, superelevated_dut_->SHat(kPRH).dot(superelevated_dut_->RHat(kPRH)), kLinearTolerance);
}

class MalidriveRoadCurveArcOrientationOfPTest : public MalidriveRoadCurveArcTest {};

TEST_F(MalidriveRoadCurveArcOrientationOfPTest, OrientatationOfP) {
  CompareRollPitchYaw(RollPitchYaw(kZero, kZero, kHeading), flat_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kZero, kZero, kHeading), elevated_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kZero, /* kPitchedBeta */ -0.6721592337385545, kHeading),
                      pitched_dut_->Orientation(kP), kAngularTolerance);
  CompareRollPitchYaw(RollPitchYaw(kTheta, kZero, kHeading), superelevated_dut_->Orientation(kP), kAngularTolerance);
}

class MalidriveRoadCurveArcOrientationOfPRHTest : public MalidriveRoadCurveArcTest {
 protected:
  Matrix3 MakeRotationMatrix(const Vector3& s_hat, const Vector3& r_hat) const {
    const Vector3 h_hat = s_hat.cross(r_hat);
    return Matrix3({s_hat[0], r_hat[0], h_hat[0], s_hat[1], r_hat[1], h_hat[1], s_hat[2], r_hat[2], h_hat[2]});
  }
};

// This test compares the rotation matrix of
// MalidriveRoadCurve::Orientation(prh) with another constructed from the
// orthonormal vector basis s_hat, r_hat and h_hat.
TEST_F(MalidriveRoadCurveArcOrientationOfPRHTest, Orientation) {
  const Vector3 kAtCenterline{kP, kZero, kZero};
  const Vector3 kWithLateralOffset{kP, kR, kZero};
  const Vector3 kWithVerticalOffset{kP, kZero, kH};

  for (const Vector3& prh : {kAtCenterline, kWithLateralOffset, kWithVerticalOffset}) {
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(flat_dut_->SHat(prh), flat_dut_->RHat(prh)),
                                flat_dut_->Orientation(prh).ToMatrix(), kLinearTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(elevated_dut_->SHat(prh), elevated_dut_->RHat(prh)),
                                elevated_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
    EXPECT_TRUE(CompareMatrices(MakeRotationMatrix(superelevated_dut_->SHat(prh), superelevated_dut_->RHat(prh)),
                                superelevated_dut_->Orientation(prh).ToMatrix(), kAngularTolerance));
  }
}

class MalidriveRoadCurveArcWTest : public MalidriveRoadCurveArcTest {};

TEST_F(MalidriveRoadCurveArcWTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZero},
                             flat_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZ0},
                             elevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZero},
                             superelevated_dut_->W(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcWTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors({/* kG.x() - kR * kSinHeading */ 5.571796769724493,
                              /* kG.y() + kR * kCosHeading */ 28.33012701892219, kZero},
                             flat_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() - kR * kSinHeading */ 5.571796769724493,
                              /* kG.y() + kR * kCosHeading */ 28.33012701892219, kZ0},
                             elevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() - kR * kSinHeading * kCosTheta */ 5.6229080302908905,
                              /* kG.y() + kR * kCosHeading * kCosTheta */ 28.418654319062085,
                              /* kR * kSinTheta */ -0.7764571353075622},
                             superelevated_dut_->W(kPRH), kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcWTest, WithAVerticalOffset) {
  const maliput::math::Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kH}, flat_dut_->W(kPRH),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZ0 + kH},
                             elevated_dut_->W(kPRH), kLinearTolerance));
  EXPECT_TRUE(CompareVectors({/* kG.x() + kH * kSinHeading * kSinTheta */ 6.942387247173233,
                              /* kG.y() - kH * kCosHeading * kSinTheta */ 30.704059362233494,
                              /* kH * kCosTheta */ 0.9659258262890683},
                             superelevated_dut_->W(kPRH), kLinearTolerance));
}

class MalidriveRoadCurveArcWInverseTest : public MalidriveRoadCurveArcTest {};

TEST_F(MalidriveRoadCurveArcWInverseTest, AtTheCenterline) {
  const Vector3 kPRH{kP, kZero, kZero};

  EXPECT_TRUE(CompareVectors(
      kPRH, flat_dut_->WInverse({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZero}),
      kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      kPRH, elevated_dut_->WInverse({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZ0}),
      kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      kPRH, superelevated_dut_->WInverse({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZero}),
      kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcWInverseTest, WithALateralOffset) {
  const Vector3 kPRH{kP, kR, kZero};

  EXPECT_TRUE(CompareVectors(kPRH,
                             flat_dut_->WInverse({/* kG.x() - kR * kSinHeading */ 5.571796769724493,
                                                  /* kG.y() + kR * kCosHeading */ 28.33012701892219, kZero}),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kPRH,
                             elevated_dut_->WInverse({/* kG.x() - kR * kSinHeading */ 5.571796769724493,
                                                      /* kG.y() + kR * kCosHeading */ 28.33012701892219, kZ0}),
                             kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors(kPRH,
                     superelevated_dut_->WInverse({/* kG.x() - kR * kSinHeading * kCosTheta */ 5.6229080302908905,
                                                   /* kG.y() + kR * kCosHeading * kCosTheta */ 28.418654319062085,
                                                   /* kR * kSinTheta */ -0.7764571353075622}),
                     kLinearTolerance));
}

TEST_F(MalidriveRoadCurveArcWInverseTest, WithAVerticalOffset) {
  const maliput::math::Vector3 kPRH{kP, kZero, kH};

  EXPECT_TRUE(CompareVectors(kPRH,
                             flat_dut_->WInverse({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kH}),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      kPRH, elevated_dut_->WInverse({/* kG.x() */ 7.071796769724494, /* kG.y() */ 30.928203230275507, kZ0 + kH}),
      kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors(kPRH,
                     superelevated_dut_->WInverse({/* kG.x() + kH * kSinHeading * kSinTheta */ 6.942387247173233,
                                                   /* kG.y() - kH * kCosHeading * kSinTheta */ 30.704059362233494,
                                                   /* kH * kCosTheta */ 0.9659258262890683}),
                     kLinearTolerance));
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
