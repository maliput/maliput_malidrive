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
#include "maliput_malidrive/builder/road_curve_factory.h"

#include <memory>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_math_compare.h>

#include "maliput_malidrive/road_curve/arc_ground_curve.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/piecewise_ground_curve.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

using maliput::math::Vector2;
using maliput::math::test::CompareVectors;

GTEST_TEST(RoadCurveFactoryConstructorTest, Prerequisites) {
  const double kAngularTolerance{1e-12};
  const double kLinearTolerance{1e-12};
  const double kScaleLength{1.};
  const double kZero{0.};
  // Zeroed arguments.
  EXPECT_THROW(RoadCurveFactory(kZero, kScaleLength, kAngularTolerance), maliput::common::assertion_error);
  EXPECT_THROW(RoadCurveFactory(kLinearTolerance, kZero, kAngularTolerance), maliput::common::assertion_error);
  EXPECT_THROW(RoadCurveFactory(kLinearTolerance, kScaleLength, kZero), maliput::common::assertion_error);
  // Negative arguments.
  EXPECT_THROW(RoadCurveFactory(-kLinearTolerance, kScaleLength, kAngularTolerance), maliput::common::assertion_error);
  EXPECT_THROW(RoadCurveFactory(kLinearTolerance, -kScaleLength, kAngularTolerance), maliput::common::assertion_error);
  EXPECT_THROW(RoadCurveFactory(kLinearTolerance, kScaleLength, -kAngularTolerance), maliput::common::assertion_error);
  // Positive arguments.
  EXPECT_NO_THROW(RoadCurveFactory(kAngularTolerance, kScaleLength, kLinearTolerance));
}

class RoadCurveFactoryTest : public ::testing::Test {
 protected:
  const double kAngularTolerance{1e-10};
  const double kLinearTolerance{1e-10};
  const double kScaleLength{1.};
  const double kA{1.};
  const double kB{2.};
  const double kC{3.};
  const double kD{4.};
  const double kP0{0.};
  const double kP1{1.};
  const double kP2{2.};
  const double kCurvature{-0.002};
  const Vector2 kStartPoint{1., 2.};
  const Vector2 kStartPointB{1. + std::sqrt(2.) / 2., 2. + std::sqrt(2.) / 2.};
  const double kHeading{M_PI / 4.};
  const xodr::Geometry kArcGeometry{
      kP0, kStartPoint, kHeading, kP1 - kP0, xodr::Geometry::Type::kArc, {xodr::Geometry::Arc{kCurvature}}};
  const xodr::Geometry kLineGeometry{
      kP0, kStartPoint, kHeading, kP1 - kP0, xodr::Geometry::Type::kLine, {xodr::Geometry::Line{}}};
  const xodr::Geometry kArcGeometryB{
      kP1, kStartPointB, kHeading, kP2 - kP1, xodr::Geometry::Type::kArc, {xodr::Geometry::Arc{kCurvature}}};
  const RoadCurveFactory dut_{kLinearTolerance, kScaleLength, kAngularTolerance};
};

TEST_F(RoadCurveFactoryTest, CubicPolynomial) {
  auto cubic_polynomial = dut_.MakeCubicPolynomial(kA, kB, kC, kD, kP0, kP1);
  EXPECT_NE(dynamic_cast<road_curve::CubicPolynomial*>(cubic_polynomial.get()), nullptr);
}

TEST_F(RoadCurveFactoryTest, LineGroundCurve) {
  auto line_ground_curve = dut_.MakeLineGroundCurve(kLineGeometry);

  EXPECT_NE(dynamic_cast<road_curve::LineGroundCurve*>(line_ground_curve.get()), nullptr);
  EXPECT_EQ(kLinearTolerance, line_ground_curve->linear_tolerance());
}

TEST_F(RoadCurveFactoryTest, ArcGroundCurve) {
  auto arc_ground_curve = dut_.MakeArcGroundCurve(kArcGeometry);

  EXPECT_NE(dynamic_cast<road_curve::ArcGroundCurve*>(arc_ground_curve.get()), nullptr);
  EXPECT_EQ(kLinearTolerance, arc_ground_curve->linear_tolerance());
}

TEST_F(RoadCurveFactoryTest, PiecewiseGroundCurve) {
  auto piecewise_ground_curve = dut_.MakePiecewiseGroundCurve({kLineGeometry, kArcGeometryB});

  EXPECT_NE(dynamic_cast<road_curve::PiecewiseGroundCurve*>(piecewise_ground_curve.get()), nullptr);
  EXPECT_EQ(kLinearTolerance, piecewise_ground_curve->linear_tolerance());
}

TEST_F(RoadCurveFactoryTest, PiecewiseGroundCurveExpectsFailure) {
  EXPECT_THROW(dut_.MakePiecewiseGroundCurve({}), maliput::common::assertion_error);
}

TEST_F(RoadCurveFactoryTest, RoadCurve) {
  const bool kAssertContiguity{true};
  auto malidrive_road_curve = dut_.MakeMalidriveRoadCurve(
      dut_.MakeLineGroundCurve(kLineGeometry), dut_.MakeCubicPolynomial(kA, kB, kC, kD, kP0, kP1),
      dut_.MakeCubicPolynomial(kA, kB, kC, kD, kP0, kP1), kAssertContiguity);

  EXPECT_NE(malidrive_road_curve.get(), nullptr);
  EXPECT_EQ(kLinearTolerance, malidrive_road_curve->linear_tolerance());
  EXPECT_EQ(kScaleLength, malidrive_road_curve->scale_length());
}

TEST_F(RoadCurveFactoryTest, MakeCubicPolynomialFromY) {
  auto dut = dut_.MakeCubicPolynomial(0 /* p0 */, 10 /* p1 */, 5 /* y */, 1 /* dy */);
  EXPECT_NEAR(0., dut->f(0.), kLinearTolerance);
  EXPECT_NEAR(0., dut->f_dot(0.), kLinearTolerance);
  EXPECT_NEAR(5., dut->f(10.), kLinearTolerance);
  EXPECT_NEAR(1., dut->f_dot(10.), kLinearTolerance);

  dut = dut_.MakeCubicPolynomial(5 /* p0 */, 65 /* p1 */, -3 /* y */, -1 /* dy */);
  EXPECT_NEAR(0., dut->f(5.), kLinearTolerance);
  EXPECT_NEAR(0., dut->f_dot(5.), kLinearTolerance);
  EXPECT_NEAR(-3., dut->f(65.), kLinearTolerance);
  EXPECT_NEAR(-1., dut->f_dot(65.), kLinearTolerance);

  dut = dut_.MakeCubicPolynomial(1.27 /* p0 */, 7.23 /* p1 */, 13.25 /* y */, -6.23 /* dy */);
  EXPECT_NEAR(0., dut->f(1.27), kLinearTolerance);
  EXPECT_NEAR(0., dut->f_dot(1.27), kLinearTolerance);
  EXPECT_NEAR(13.25, dut->f(7.23), kLinearTolerance);
  EXPECT_NEAR(-6.23, dut->f_dot(7.23), kLinearTolerance);
}

// Tests the elevation, superelevation and offset function that comes from a xodr::ElevationProfile,
// xodr::LateralProfile and xodr::LaneOffsets respectively. The result of the MakeElevation, MakeSuperelevation and
// MakeReferenceLineOffset method is evaluated in four known points to guarantee unanimity on the cubic polynomial that
// was generated.
class RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest : public ::testing::Test {
 protected:
  const double kAngularTolerance{1e-10};
  const double kLinearTolerance{1e-10};
  const double kScaleLength{1.};
  const double kP0{2.4};
  const double kP1{62.4};
  const double kP0_33{(kP1 - kP0) / 3};
  const double kP0_66{(kP1 - kP0) * 2 / 3};
  const bool kEnsureContiguity{true};
  const bool kDontEnsureContiguity{false};
  void SetUp() override {
    road_curve_factory_ = std::make_unique<RoadCurveFactory>(kLinearTolerance, kScaleLength, kAngularTolerance);
  }
  std::unique_ptr<RoadCurveFactoryBase> road_curve_factory_;
};

// MakeElevation, MakeSuperElevation and MakeReferenceLineOffset methods share same implementation: MakeCubicFromXodr.
TEST_F(RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest, Throws) {
  const xodr::ElevationProfile kElevationProfile{{{
      {0. /* s_0 */, 0. /* a */, 0. /* b */, 0.06 /* c */, -0.004 /* d */},
      {10. /* s_0 */, 2. /* a */, 0. /* b */, 0.06 /* c */, -0.004 /* d */},
      {20. /* s_0 */, 4. /* a */, 0. /* b */, 0.06 /* c */, -0.004 /* d */},
  }}};

  // Negative p0.
  EXPECT_THROW(road_curve_factory_->MakeElevation(kElevationProfile, -2, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // p1 equal to p0.
  EXPECT_THROW(road_curve_factory_->MakeElevation(kElevationProfile, kP0, kP0, kEnsureContiguity),
               maliput::common::assertion_error);
  // Zero number of polynomials.
  EXPECT_THROW(road_curve_factory_->MakeElevation({}, kP0, kP0, kEnsureContiguity), maliput::common::assertion_error);
  // First width's offset different than zero.
  EXPECT_THROW(road_curve_factory_->MakeElevation({{{1. /* s_0 */, 2., 3., 4, 5.}}}, kP0, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // Share same s_0 value.
  EXPECT_THROW(road_curve_factory_->MakeElevation({{{1. /* s_0 */, 2., 3., 4, 5.}, {1. /* s_0 */, 2., 3., 4, 5.}}}, kP0,
                                                  kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // It doesn't meet C1 continuity between both functions.
  EXPECT_THROW(road_curve_factory_->MakeElevation({{{2.4 /* s_0 */, 2., 3., 4, 5.}, {20. /* s_0 */, 6., 7., 8, 9.}}},
                                                  kP0, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // It doesn't meet C1 continuity between both functions however continuity isn't ensured.
  EXPECT_NO_THROW(road_curve_factory_->MakeElevation({{{2.4 /* s_0 */, 2., 3., 4, 5.}, {20. /* s_0 */, 6., 7., 8, 9.}}},
                                                     kP0, kP1, kDontEnsureContiguity));
}

// MakeElevation, MakeSuperElevation and MakeReferenceLineOffset methods share same implementation: MakeCubicFromXodr.
TEST_F(RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest, LastFunctionLengthZero) {
  // Last function length is zero so is discarded.
  EXPECT_NO_THROW(road_curve_factory_->MakeElevation({{{kP0 /* s_0 */, 2., 3., 4, 5.}, {kP1 /* s_0 */, 2., 3., 4, 5.}}},
                                                     kP0, kP1, kEnsureContiguity));
}

TEST_F(RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest, ZeroFunctions) {
  const xodr::ElevationProfile kElevationProfile{};
  const xodr::LateralProfile kLateralProfile{};
  const double kZAtP0{0.};
  const double kZAtP0_33{0.};
  const double kZAtP0_66{0.};
  const double kZAtP1{0.};
  const double kZDotAtP0{0.};
  const double kZDotAtP0_33{0.};
  const double kZDotAtP0_66{0.};
  const double kZDotAtP1{0.};
  const std::array<std::unique_ptr<road_curve::Function>, 3> duts{
      road_curve_factory_->MakeElevation(kElevationProfile, kP0, kP1, kEnsureContiguity),
      road_curve_factory_->MakeSuperelevation(kLateralProfile, kP0, kP1, kEnsureContiguity),
      road_curve_factory_->MakeReferenceLineOffset({}, kP0, kP1)};

  for (const auto& dut : duts) {
    EXPECT_NEAR(kZAtP0, dut->f(kP0), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_33, dut->f(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_66, dut->f(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZAtP1, dut->f(kP1), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0, dut->f_dot(kP0), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_33, dut->f_dot(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_66, dut->f_dot(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP1, dut->f_dot(kP1), kLinearTolerance);
  }
}

TEST_F(RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest, OneFunction) {
  const xodr::ElevationProfile kElevationProfile{{{2.4 /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */}}};
  const xodr::LateralProfile kLateralProfile{{{2.4 /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */}}};
  const xodr::LaneOffset kLaneOffset{2.4 /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */};
  const double kZAtP0{3.0};
  const double kZAtP0_33{247.59776};
  const double kZAtP0_66{1316.65376};
  const double kZAtP1{4083};
  const double kZDotAtP0{2.};
  const double kZDotAtP0_33{28.8928};
  const double kZDotAtP0_66{82.0128};
  const double kZDotAtP1{170.};
  const std::array<std::unique_ptr<road_curve::Function>, 3> duts{
      road_curve_factory_->MakeElevation(kElevationProfile, kP0, kP1, kEnsureContiguity),
      road_curve_factory_->MakeSuperelevation(kLateralProfile, kP0, kP1, kEnsureContiguity),
      road_curve_factory_->MakeReferenceLineOffset({kLaneOffset}, kP0, kP1)};

  for (const auto& dut : duts) {
    EXPECT_NEAR(kZAtP0, dut->f(kP0), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_33, dut->f(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_66, dut->f(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZAtP1, dut->f(kP1), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0, dut->f_dot(kP0), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_33, dut->f_dot(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_66, dut->f_dot(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP1, dut->f_dot(kP1), kLinearTolerance);
  }
}

TEST_F(RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest, MultipleFunctions) {
  const xodr::ElevationProfile kElevationProfile{
      {{2.4 /* s_0 */, 0. /* a */, 0. /* b */, -0.04682548664554094 /* c */, 0.00130179488167362 /* d */},
       {40. /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */}}};
  const xodr::LateralProfile kLateralProfile{
      {{2.4 /* s_0 */, 0. /* a */, 0. /* b */, -0.04682548664554094 /* c */, 0.00130179488167362 /* d */},
       {40. /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */}}};
  const std::vector<xodr::LaneOffset> kLaneOffsets{
      {2.4 /* s_0 */, 0. /* a */, 0. /* b */, -0.04682548664554094 /* c */, 0.00130179488167362 /* d */},
      {40. /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */}};
  const double kZAtP0{0.};
  const double kZAtP0_33{-7.40756865049167};
  const double kZAtP0_66{3.};
  const double kZAtP1{411.074240000000};
  const double kZDotAtP0{0.};
  const double kZDotAtP0_33{-0.438525182281379};
  const double kZDotAtP0_66{2.};
  const double kZDotAtP1{39.4528000000000};
  const std::array<std::unique_ptr<road_curve::Function>, 3> duts{
      road_curve_factory_->MakeElevation(kElevationProfile, kP0, kP1, kEnsureContiguity),
      road_curve_factory_->MakeSuperelevation(kLateralProfile, kP0, kP1, kEnsureContiguity),
      road_curve_factory_->MakeReferenceLineOffset(kLaneOffsets, kP0, kP1)};

  for (const auto& dut : duts) {
    EXPECT_NEAR(kZAtP0, dut->f(kP0), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_33, dut->f(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_66, dut->f(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZAtP1, dut->f(kP1), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0, dut->f_dot(kP0), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_33, dut->f_dot(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_66, dut->f_dot(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP1, dut->f_dot(kP1), kLinearTolerance);
  }
}

TEST_F(RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest, ElevationSuperelevationOneFunctionAndGap) {
  const xodr::ElevationProfile kElevationProfile{{{40. /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */}}};
  const xodr::LateralProfile kLateralProfile{{{40. /* s_0 */, 3. /* a */, 2. /* b */, 0.5 /* c */, 0.01 /* d */}}};
  const double kZAtP0{0.};
  const double kZAtP0_33{-7.40756865049167};
  const double kZAtP0_66{3.};
  const double kZAtP1{411.074240000000};
  const double kZDotAtP0{0.};
  const double kZDotAtP0_33{-0.438525182281379};
  const double kZDotAtP0_66{2.};
  const double kZDotAtP1{39.4528000000000};
  const std::array<std::unique_ptr<road_curve::Function>, 2> duts{
      road_curve_factory_->MakeElevation(kElevationProfile, kP0, kP1, kEnsureContiguity),
      road_curve_factory_->MakeSuperelevation(kLateralProfile, kP0, kP1, kEnsureContiguity)};

  for (const auto& dut : duts) {
    EXPECT_NEAR(kZAtP0, dut->f(kP0), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_33, dut->f(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZAtP0_66, dut->f(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZAtP1, dut->f(kP1), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0, dut->f_dot(kP0), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_33, dut->f_dot(kP0_33), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP0_66, dut->f_dot(kP0_66), kLinearTolerance);
    EXPECT_NEAR(kZDotAtP1, dut->f_dot(kP1), kLinearTolerance);
  }
}

TEST_F(RoadCurveFactoryMakeElevationSuperelevationReferenceLineOffsetTest, ReferenceLineOffsetOneFunctionAndGap) {
  const xodr::LaneOffset kLaneOffset{40. /* s_0 */, 0. /* a */, 0. /* b */, 0.0375 /* c */, -0.00125 /* d */};
  const double kZAtP0{0.};
  const double kZAtP0_33{0.};
  const double kZAtP0_66{0.};
  const double kZAtP1{4.76672000000000};
  const double kZDotAtP0{0.};
  const double kZDotAtP0_33{0.};
  const double kZDotAtP0_66{0.};
  const double kZDotAtP1{-0.201600000000000};

  const auto dut = road_curve_factory_->MakeReferenceLineOffset({kLaneOffset}, kP0, kP1);
  EXPECT_NEAR(kZAtP0, dut->f(kP0), kLinearTolerance);
  EXPECT_NEAR(kZAtP0_33, dut->f(kP0_33), kLinearTolerance);
  EXPECT_NEAR(kZAtP0_66, dut->f(kP0_66), kLinearTolerance);
  EXPECT_NEAR(kZAtP1, dut->f(kP1), kLinearTolerance);
  EXPECT_NEAR(kZDotAtP0, dut->f_dot(kP0), kLinearTolerance);
  EXPECT_NEAR(kZDotAtP0_33, dut->f_dot(kP0_33), kLinearTolerance);
  EXPECT_NEAR(kZDotAtP0_66, dut->f_dot(kP0_66), kLinearTolerance);
  EXPECT_NEAR(kZDotAtP1, dut->f_dot(kP1), kLinearTolerance);
}

// Tests the lane width function that overcomes from a std::vector<xodr::LaneWidth>.
class RoadCurveFactoryMakeLaneWidthTest : public ::testing::Test {
 protected:
  const double kAngularTolerance{1e-10};
  const double kLinearTolerance{1e-10};
  const double kScaleLength{1.};
  const double kP0{10.};
  const double kP1{40.};
  const double kP0_33{(kP1 + kP0) / 3};
  const double kP0_50{(kP1 + kP0) / 2};
  const double kP0_66{(kP1 + kP0) * 2 / 3};
  const bool kEnsureContiguity{true};
  const bool kDontEnsureContiguity{false};
  const std::vector<xodr::LaneWidth> kLaneWidths{{
      {0. /* offset */, 0. /* a */, 0. /* b */, 0.06 /* c */, -0.004 /* d */},
      {10. /* offset */, 2. /* a */, 0. /* b */, 0.06 /* c */, -0.004 /* d */},
      {20. /* offset */, 4. /* a */, 0. /* b */, 0.06 /* c */, -0.004 /* d */},
  }};
  void SetUp() override {
    road_curve_factory_ = std::make_unique<RoadCurveFactory>(kLinearTolerance, kScaleLength, kAngularTolerance);
  }
  std::unique_ptr<RoadCurveFactoryBase> road_curve_factory_;
};

TEST_F(RoadCurveFactoryMakeLaneWidthTest, Throws) {
  // Negative p0.
  EXPECT_THROW(road_curve_factory_->MakeLaneWidth(kLaneWidths, -2, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // p1 equal to p0.
  EXPECT_THROW(road_curve_factory_->MakeLaneWidth(kLaneWidths, kP0, kP0, kEnsureContiguity),
               maliput::common::assertion_error);
  // Zero number of polynomials.
  EXPECT_THROW(road_curve_factory_->MakeLaneWidth({}, kP0, kP0, kEnsureContiguity), maliput::common::assertion_error);
  // First width's offset different than zero.
  EXPECT_THROW(road_curve_factory_->MakeLaneWidth({{1. /* offset */, 2., 3., 4, 5.}}, kP0, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // Share same offset value.
  EXPECT_THROW(road_curve_factory_->MakeLaneWidth({{1. /* offset */, 2., 3., 4, 5.}, {1. /* offset */, 2., 3., 4, 5.}},
                                                  kP0, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // Function length is zero.
  EXPECT_THROW(road_curve_factory_->MakeLaneWidth({{kP1 /* offset */, 2., 3., 4, 5.}}, kP0, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // It doesn't meet C1 continuity between both functions.
  EXPECT_THROW(road_curve_factory_->MakeLaneWidth(
                   {{2.4 /* offset */, 2., 3., 4, 5.}, {20. /* offset */, 6., 7., 8, 9.}}, kP0, kP1, kEnsureContiguity),
               maliput::common::assertion_error);
  // It doesn't meet C1 continuity between both functions however continuity isn't ensured.
  EXPECT_THROW(
      road_curve_factory_->MakeLaneWidth({{2.4 /* offset */, 2., 3., 4, 5.}, {20. /* offset */, 6., 7., 8, 9.}}, kP0,
                                         kP1, kDontEnsureContiguity),
      maliput::common::assertion_error);
}

TEST_F(RoadCurveFactoryMakeLaneWidthTest, MultipleLaneWidth) {
  const double kWidthAtP0{0.};
  const double kWidthAtP0_33{1.48148148148148};
  const double kWidthAtP0_50{3.};
  const double kWidthAtP0_66{4.51851851851852};
  const double kWidthAtP1{6.};
  const double kWidthDotAtP0{0.};
  const double kWidthDotAtP0_33{0.26666666666666};
  const double kWidthDotAtP0_50{0.3};
  const double kWidthDotAtP0_66{00.26666666666666};
  const double kWidthDotAtP1{0.};
  const std::unique_ptr<road_curve::Function> dut =
      road_curve_factory_->MakeLaneWidth(kLaneWidths, kP0, kP1, kEnsureContiguity);

  EXPECT_NEAR(kWidthAtP0, dut->f(kP0), kLinearTolerance);
  EXPECT_NEAR(kWidthAtP0_33, dut->f(kP0_33), kLinearTolerance);
  EXPECT_NEAR(kWidthAtP0_50, dut->f(kP0_50), kLinearTolerance);
  EXPECT_NEAR(kWidthAtP0_66, dut->f(kP0_66), kLinearTolerance);
  EXPECT_NEAR(kWidthAtP1, dut->f(kP1), kLinearTolerance);
  EXPECT_NEAR(kWidthDotAtP0, dut->f_dot(kP0), kLinearTolerance);
  EXPECT_NEAR(kWidthDotAtP0_33, dut->f_dot(kP0_33), kLinearTolerance);
  EXPECT_NEAR(kWidthDotAtP0_50, dut->f_dot(kP0_50), kLinearTolerance);
  EXPECT_NEAR(kWidthDotAtP0_66, dut->f_dot(kP0_66), kLinearTolerance);
  EXPECT_NEAR(kWidthDotAtP1, dut->f_dot(kP1), kLinearTolerance);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
