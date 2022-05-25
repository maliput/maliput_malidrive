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
#include "maliput_malidrive/road_curve/piecewise_ground_curve.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/maliput_math_compare.h>

#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/road_curve/arc_ground_curve.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

using maliput::math::Vector2;
using maliput::math::test::CompareVectors;

class PiecewiseGroundCurveConstructorTest : public ::testing::Test {
 protected:
  const double kLinearTolerance{1e-9};
  const double kAngularTolerance{1e-12};
  const double kEpsilon{kLinearTolerance};
  // Line x-direction.
  const double kP0LineX{10.};
  const double kP1LineX{110.};
  const double kPHalfLineX{(kP1LineX - kP0LineX) / 2 + kP0LineX};
  const Vector2 kXY0LineX{0., 0.};
  const Vector2 kDxyLineX{100. * Vector2::UnitX()};
  const double kPLineXToArcLeft{kP1LineX - kP0LineX - kEpsilon};
  const double kPLineXToArcRight{kP1LineX - kP0LineX};
  const Vector2 kLineXMiddleCoord{50., 10.};
  const Vector2 kLineXEndCoord{100., -1.};
  // Arc turning left 90 degree.
  const double kP0Arc{20.};
  const double kP1Arc{120.};
  const double kPHalfArc{(kP1Arc - kP0Arc) / 2 + kP0Arc};
  const Vector2 kXY0Arc{kDxyLineX + kXY0LineX};
  const double kStartHeading = 0.;
  const double kCurvature = 1. / 50.;
  const double kArcLength90DegLeft = M_PI * 0.5 / kCurvature;
  const double kPArcToLineYLeft{kP1Arc - kP0Arc - kEpsilon};
  const double kPArcToLineYRight{kP1Arc - kP0Arc};
  const Vector2 kArcMiddleCoord{125., 25.};
  const Vector2 kArcEndCoord{145., 50.};
  // Line y-direction.
  const double kP0LineY{30.};
  const double kP1LineY{130.};
  const double kPHalfLineY{(kP1LineY - kP0LineY) / 2 + kP0LineY};
  const Vector2 kXY0LineY{150., 50.};
  const Vector2 kDxyLineY{100. * Vector2::UnitY()};
  const double kPLineYToEndLeft{kP1LineY - kP0LineY - kEpsilon};
  const double kPLineYToEndRight{kP1LineY - kP0LineY};
  const Vector2 kLineYMiddleCoord{160., 100.};
  const Vector2 kLineYEndCoord{160., 150.};
  // Picewise groundcurve.
  const double kTotalLength{kDxyLineX.norm() + kDxyLineY.norm() + kArcLength90DegLeft};
  const double kP0{0.};
  const double kP1{kPLineXToArcRight + kPArcToLineYRight + kPLineYToEndRight};
};

TEST_F(PiecewiseGroundCurveConstructorTest, CorrectlyConstructed) {
  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX));
  ground_curves.push_back(std::make_unique<ArcGroundCurve>(kLinearTolerance, kXY0Arc, kStartHeading, kCurvature,
                                                           kArcLength90DegLeft, kP0Arc, kP1Arc));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY));
  EXPECT_NO_THROW(PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, kAngularTolerance));
}

TEST_F(PiecewiseGroundCurveConstructorTest, ZeroGroundCurves) {
  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  EXPECT_THROW(PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, kAngularTolerance),
               maliput::common::assertion_error);
}

TEST_F(PiecewiseGroundCurveConstructorTest, NullptrGroundCurve) {
  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX));
  ground_curves.push_back(nullptr);
  EXPECT_THROW(PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, kAngularTolerance),
               maliput::common::assertion_error);
}

TEST_F(PiecewiseGroundCurveConstructorTest, NotG1ContiguousGroundCurvesAtTheEdges) {
  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX * 0.5, kP0LineX, kP1LineX));
  ground_curves.push_back(std::make_unique<ArcGroundCurve>(kLinearTolerance, kXY0Arc, kStartHeading, kCurvature,
                                                           kArcLength90DegLeft, kP0Arc, kP1Arc));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY));
  EXPECT_THROW(PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, kAngularTolerance),
               maliput::common::assertion_error);
}

// LineGroundCurve mock for testing G1 contiguous checking.
class MockNonContiguousLineGroundCurve : public LineGroundCurve {
 public:
  // Constructs a MockNonContiguousLineGroundCurve.
  // For parameter information @see LineGroundCurve.
  MockNonContiguousLineGroundCurve(const double linear_tolerance, const maliput::math::Vector2& xy0,
                                   const maliput::math::Vector2& dxy, double p0, double p1)
      : LineGroundCurve(linear_tolerance, xy0, dxy, p0, p1) {}

 private:
  bool DoIsG1Contiguous() const override { return false; }
};

TEST_F(PiecewiseGroundCurveConstructorTest, NotG1ContiguousGroundCurves) {
  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX));
  ground_curves.push_back(std::make_unique<ArcGroundCurve>(kLinearTolerance, kXY0Arc, kStartHeading, kCurvature,
                                                           kArcLength90DegLeft, kP0Arc, kP1Arc));
  ground_curves.push_back(
      std::make_unique<MockNonContiguousLineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY));
  EXPECT_THROW(PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, kAngularTolerance),
               maliput::common::assertion_error);
}

TEST_F(PiecewiseGroundCurveConstructorTest, BadLinearTolerance) {
  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX));
  ground_curves.push_back(std::make_unique<ArcGroundCurve>(kLinearTolerance, kXY0Arc, kStartHeading, kCurvature,
                                                           kArcLength90DegLeft, kP0Arc, kP1Arc));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY));
  EXPECT_THROW(PiecewiseGroundCurve(std::move(ground_curves), -1 * kLinearTolerance, kAngularTolerance),
               maliput::common::assertion_error);
}

TEST_F(PiecewiseGroundCurveConstructorTest, BadAngularTolerance) {
  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX));
  ground_curves.push_back(std::make_unique<ArcGroundCurve>(kLinearTolerance, kXY0Arc, kStartHeading, kCurvature,
                                                           kArcLength90DegLeft, kP0Arc, kP1Arc));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY));
  EXPECT_THROW(PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, -1 * kAngularTolerance),
               maliput::common::assertion_error);
}

class PiecewiseGroundCurveTest : public PiecewiseGroundCurveConstructorTest {
 public:
  void SetUp() override {
    std::vector<std::unique_ptr<GroundCurve>> ground_curves;
    ground_curves.push_back(
        std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX));
    ground_curves.push_back(std::make_unique<ArcGroundCurve>(kLinearTolerance, kXY0Arc + kDeltaXY0Arc,
                                                             kStartHeading + kDeltaHeading, kCurvature,
                                                             kArcLength90DegLeft, kP0Arc, kP1Arc));
    ground_curves.push_back(
        std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY));
    piecewise_ground_curve_ =
        std::make_unique<PiecewiseGroundCurve>(std::move(ground_curves), kLinearTolerance, kAngularTolerance);
  }

 protected:
  // A slightely gap in the start position of the second GroundCurve.
  const Vector2 kDeltaXY0Arc{Vector2::UnitY() * kLinearTolerance / 2};
  // A slightely gap in the start heading of the second GroundCurve.
  const double kDeltaHeading{kAngularTolerance / 2};
  const std::unique_ptr<GroundCurve> kExpectedLineXDirection =
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX);
  const std::unique_ptr<GroundCurve> kExpectedArc = std::make_unique<ArcGroundCurve>(
      kLinearTolerance, kXY0Arc, kStartHeading, kCurvature, kArcLength90DegLeft, kP0Arc, kP1Arc);
  const std::unique_ptr<GroundCurve> kExpectedLineYDirection =
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY);
  std::unique_ptr<GroundCurve> piecewise_ground_curve_;
};

TEST_F(PiecewiseGroundCurveTest, p0) { EXPECT_NEAR(kP0, piecewise_ground_curve_->p0(), kLinearTolerance); }

TEST_F(PiecewiseGroundCurveTest, p1) { EXPECT_NEAR(kP1, piecewise_ground_curve_->p1(), kLinearTolerance); }

TEST_F(PiecewiseGroundCurveTest, ArcLength) {
  EXPECT_NEAR(kTotalLength, piecewise_ground_curve_->ArcLength(), kLinearTolerance);
}

TEST_F(PiecewiseGroundCurveTest, IsGContiguous) { EXPECT_TRUE(piecewise_ground_curve_->IsG1Contiguous()); }

TEST_F(PiecewiseGroundCurveTest, G) {
  // First geometry.
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kP0LineX), piecewise_ground_curve_->G(kP0), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kPHalfLineX), piecewise_ground_curve_->G(kPLineXToArcRight / 2),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kP1LineX - kEpsilon),
                             piecewise_ground_curve_->G(kPLineXToArcLeft), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kP1LineX), piecewise_ground_curve_->G(kPLineXToArcRight),
                             kLinearTolerance));
  // Second geometry.
  EXPECT_TRUE(CompareVectors(kExpectedArc->G(kP0Arc), piecewise_ground_curve_->G(kPLineXToArcRight), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedArc->G(kPHalfArc),
                             piecewise_ground_curve_->G(kPLineXToArcRight + kPArcToLineYRight / 2), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedArc->G(kP1Arc - kEpsilon),
                             piecewise_ground_curve_->G(kPLineXToArcRight + kPArcToLineYLeft), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedArc->G(kP1Arc), piecewise_ground_curve_->G(kPLineXToArcRight + kPArcToLineYRight),
                             kLinearTolerance));
  // Third geometry.
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->G(kP0LineY),
                             piecewise_ground_curve_->G(kPLineXToArcRight + kPArcToLineYRight), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->G(kPHalfLineY),
                             piecewise_ground_curve_->G(kPLineXToArcRight + kPArcToLineYRight + kPLineYToEndRight / 2),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->G(kP1LineY - kEpsilon),
                             piecewise_ground_curve_->G(kPLineXToArcRight + kPArcToLineYRight + kPLineYToEndLeft),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->G(kP1LineY), piecewise_ground_curve_->G(kP1), kLinearTolerance));
}

TEST_F(PiecewiseGroundCurveTest, GDot) {
  // First geometry.
  EXPECT_TRUE(
      CompareVectors(kExpectedLineXDirection->GDot(kP0LineX), piecewise_ground_curve_->GDot(kP0), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->GDot(kPHalfLineX),
                             piecewise_ground_curve_->GDot((kPLineXToArcRight) / 2), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->GDot(kP1LineX - kEpsilon),
                             piecewise_ground_curve_->GDot(kPLineXToArcLeft), kLinearTolerance));
  // Second geometry.
  EXPECT_TRUE(
      CompareVectors(kExpectedArc->GDot(kP0Arc), piecewise_ground_curve_->GDot(kPLineXToArcRight), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedArc->GDot(kPHalfArc),
                             piecewise_ground_curve_->GDot(kPLineXToArcRight + kPArcToLineYRight / 2),
                             kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedArc->GDot(kP1Arc - kEpsilon),
                             piecewise_ground_curve_->GDot(kPLineXToArcRight + kPArcToLineYLeft), kLinearTolerance));
  // Third geometry.
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->GDot(kP0LineY),
                             piecewise_ground_curve_->GDot(kPLineXToArcRight + kPArcToLineYRight), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      kExpectedLineYDirection->GDot(kPHalfLineY),
      piecewise_ground_curve_->GDot(kPLineXToArcRight + kPArcToLineYRight + kPLineYToEndRight / 2), kLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->GDot(kP1LineY - kEpsilon),
                             piecewise_ground_curve_->GDot(kPLineXToArcRight + kPArcToLineYRight + kPLineYToEndLeft),
                             kLinearTolerance));
  EXPECT_TRUE(
      CompareVectors(kExpectedLineYDirection->GDot(kP1LineY), piecewise_ground_curve_->GDot(kP1), kLinearTolerance));
}

TEST_F(PiecewiseGroundCurveTest, Heading) {
  // First geometry.
  EXPECT_NEAR(kExpectedLineXDirection->Heading(kP0LineX), piecewise_ground_curve_->Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(kExpectedLineXDirection->Heading(kPHalfLineX), piecewise_ground_curve_->Heading(kPLineXToArcRight / 2),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedLineXDirection->Heading(kP1LineX - kEpsilon), piecewise_ground_curve_->Heading(kPLineXToArcLeft),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedLineXDirection->Heading(kP1LineX), piecewise_ground_curve_->Heading(kPLineXToArcRight),
              kLinearTolerance);
  // Second geometry.
  EXPECT_NEAR(kExpectedArc->Heading(kP0Arc), piecewise_ground_curve_->Heading(kPLineXToArcRight), kLinearTolerance);
  EXPECT_NEAR(kExpectedArc->Heading(kPHalfArc),
              piecewise_ground_curve_->Heading(kPLineXToArcRight + kPArcToLineYRight / 2), kLinearTolerance);
  EXPECT_NEAR(kExpectedArc->Heading(kP1Arc - kEpsilon),
              piecewise_ground_curve_->Heading(kPLineXToArcRight + kPArcToLineYLeft), kLinearTolerance);
  EXPECT_NEAR(kExpectedArc->Heading(kP1Arc), piecewise_ground_curve_->Heading(kPLineXToArcRight + kPArcToLineYRight),
              kLinearTolerance);
  // Third geometry.
  EXPECT_NEAR(kExpectedLineYDirection->Heading(kP0LineY),
              piecewise_ground_curve_->Heading(kPLineXToArcRight + kPArcToLineYRight), kLinearTolerance);
  EXPECT_NEAR(kExpectedLineYDirection->Heading(kPHalfLineY),
              piecewise_ground_curve_->Heading(kPLineXToArcRight + kPArcToLineYRight + kPLineYToEndRight / 2),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedLineYDirection->Heading(kP1LineY - kEpsilon),
              piecewise_ground_curve_->Heading(kPLineXToArcRight + kPArcToLineYRight + kPLineYToEndLeft),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedLineYDirection->Heading(kP1LineY), piecewise_ground_curve_->Heading(kP1), kLinearTolerance);
}

TEST_F(PiecewiseGroundCurveTest, HeadingDot) {
  // First geometry.
  EXPECT_NEAR(kExpectedLineXDirection->HeadingDot(kP0LineX), piecewise_ground_curve_->HeadingDot(kP0),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedLineXDirection->HeadingDot(kPHalfLineX),
              piecewise_ground_curve_->HeadingDot(kPLineXToArcRight / 2), kLinearTolerance);
  EXPECT_NEAR(kExpectedLineXDirection->HeadingDot(kP1LineX - kEpsilon),
              piecewise_ground_curve_->HeadingDot(kPLineXToArcLeft), kLinearTolerance);
  // Second geometry.
  EXPECT_NEAR(kExpectedArc->HeadingDot(kP0Arc), piecewise_ground_curve_->HeadingDot(kPLineXToArcRight),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedArc->HeadingDot(kPHalfArc),
              piecewise_ground_curve_->HeadingDot(kPLineXToArcRight + kPArcToLineYRight / 2), kLinearTolerance);
  EXPECT_NEAR(kExpectedArc->HeadingDot(kP1Arc - kEpsilon),
              piecewise_ground_curve_->HeadingDot(kPLineXToArcRight + kPArcToLineYLeft), kLinearTolerance);
  // Third geometry.
  EXPECT_NEAR(kExpectedLineYDirection->HeadingDot(kP0LineY),
              piecewise_ground_curve_->HeadingDot(kPLineXToArcRight + kPArcToLineYRight), kLinearTolerance);
  EXPECT_NEAR(kExpectedLineYDirection->HeadingDot(kPHalfLineY),
              piecewise_ground_curve_->HeadingDot(kPLineXToArcRight + kPArcToLineYLeft + kPLineYToEndRight / 2),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedLineYDirection->HeadingDot(kP1LineY - kEpsilon),
              piecewise_ground_curve_->HeadingDot(kPLineXToArcRight + kPArcToLineYLeft + kPLineYToEndLeft),
              kLinearTolerance);
  EXPECT_NEAR(kExpectedLineYDirection->HeadingDot(kP1LineY), piecewise_ground_curve_->HeadingDot(kP1),
              kLinearTolerance);
}

class PiecewiseGroundCurveGInverseTest : public PiecewiseGroundCurveConstructorTest {
 public:
  void SetUp() override {
    std::vector<std::unique_ptr<GroundCurve>> ground_curves;
    ground_curves.push_back(
        std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX));
    ground_curves.push_back(std::make_unique<ArcGroundCurve>(kLinearTolerance, kXY0Arc, kStartHeading, kCurvature,
                                                             kArcLength90DegLeft, kP0Arc, kP1Arc));
    ground_curves.push_back(
        std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY));
    piecewise_ground_curve_ =
        std::make_unique<PiecewiseGroundCurve>(std::move(ground_curves), kLinearTolerance, kAngularTolerance);
  }

 protected:
  const double kStrictLinearTolerance{1e-10};
  const std::unique_ptr<GroundCurve> kExpectedLineXDirection =
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineX, kDxyLineX, kP0LineX, kP1LineX);
  const std::unique_ptr<GroundCurve> kExpectedArc = std::make_unique<ArcGroundCurve>(
      kLinearTolerance, kXY0Arc, kStartHeading, kCurvature, kArcLength90DegLeft, kP0Arc, kP1Arc);
  const std::unique_ptr<GroundCurve> kExpectedLineYDirection =
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineY, kDxyLineY, kP0LineY, kP1LineY);
  std::unique_ptr<GroundCurve> piecewise_ground_curve_;
};

TEST_F(PiecewiseGroundCurveGInverseTest, GInverse) {
  const Vector2 kEpsilonX{Vector2::UnitX() * kEpsilon};
  const Vector2 kEpsilonY{Vector2::UnitY() * kEpsilon};

  // First geometry.
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kExpectedLineXDirection->GInverse(kLineXMiddleCoord)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kLineXMiddleCoord)),
                             kStrictLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kExpectedLineXDirection->GInverse(kLineXEndCoord - kEpsilonX)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kLineXEndCoord - kEpsilonX)),
                             kStrictLinearTolerance));
  // Second geometry.
  EXPECT_TRUE(CompareVectors(kExpectedArc->G(kExpectedArc->GInverse(kLineXEndCoord)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kLineXEndCoord)),
                             kStrictLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedArc->G(kExpectedArc->GInverse(kArcMiddleCoord)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kArcMiddleCoord)),
                             kStrictLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedArc->G(kExpectedArc->GInverse(kArcEndCoord - kEpsilonY)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kArcEndCoord - kEpsilonY)),
                             kStrictLinearTolerance));
  // Third geometry.
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->G(kExpectedLineYDirection->GInverse(kArcEndCoord)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kArcEndCoord)),
                             kStrictLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->G(kExpectedLineYDirection->GInverse(kLineYMiddleCoord)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kLineYMiddleCoord)),
                             kStrictLinearTolerance));
  EXPECT_TRUE(CompareVectors(kExpectedLineYDirection->G(kExpectedLineYDirection->GInverse(kLineYEndCoord)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kLineYEndCoord)),
                             kStrictLinearTolerance));

  // Equidistant coordinate to startpoint LineX and endpoint LineY.
  const Vector2 kCoordEquidistantLineXLineY{0., 150.};
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kExpectedLineXDirection->GInverse(kCoordEquidistantLineXLineY)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kCoordEquidistantLineXLineY)),
                             kStrictLinearTolerance));
  EXPECT_TRUE(CompareVectors(
      kExpectedLineYDirection->G(kExpectedLineYDirection->GInverse(kCoordEquidistantLineXLineY + kEpsilonX)),
      piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kCoordEquidistantLineXLineY + kEpsilonX)),
      kStrictLinearTolerance));
  // Center of curvature of arc GroundCurve.
  const Vector2 kCoordCenterOfCurvature{100., 50.};
  EXPECT_THROW(kExpectedArc->GInverse(kCoordCenterOfCurvature), maliput::common::assertion_error);
  EXPECT_TRUE(CompareVectors(kExpectedLineXDirection->G(kExpectedLineXDirection->GInverse(kCoordCenterOfCurvature)),
                             piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kCoordCenterOfCurvature)),
                             kStrictLinearTolerance));
  EXPECT_TRUE(
      CompareVectors(kExpectedLineYDirection->G(kExpectedLineYDirection->GInverse(kCoordCenterOfCurvature + kEpsilonY)),
                     piecewise_ground_curve_->G(piecewise_ground_curve_->GInverse(kCoordCenterOfCurvature + kEpsilonY)),
                     kStrictLinearTolerance));
}

// Tests the PFromP method using overlapped geometries.
// The use of large tolerances is due to better exemplify the issue.
class PiecewiseGroundCurveOverlappedGeometries : public ::testing::Test {
 protected:
  const double kLinearTolerance{1};
  const double kAngularTolerance{1};
  // Line A.
  const double kP0LineA{0.};
  const double kP1LineA{10.};
  const double kPHalfLineA{(kP1LineA - kP0LineA) / 2 + kP0LineA};
  const Vector2 kXY0LineA{0., 0.};
  const Vector2 kDxyLineA{10. * Vector2::UnitX()};
  // Line B.
  const double kP0LineB{9.};
  const double kP1LineB{20.};
  const double kPHalfLineB{(kP1LineB - kP0LineB) / 2 + kP0LineB};
  const Vector2 kXY0LineB{9., 0.};
  const Vector2 kDxyLineB{11. * Vector2::UnitX()};
  // Line C.
  const double kP0LineC{19.};
  const double kP1LineC{30.};
  const double kPHalfLineC{(kP1LineC - kP0LineC) / 2 + kP0LineC};
  const Vector2 kXY0LineC{19., 0.};
  const Vector2 kDxyLineC{11. * Vector2::UnitX()};
  // Line D.
  const double kP0LineD{29.};
  const double kP1LineD{39.};
  const double kPHalfLineD{(kP1LineD - kP0LineD) / 2 + kP0LineD};
  const Vector2 kXY0LineD{29., 0.};
  const Vector2 kDxyLineD{10. * Vector2::UnitX()};
};

TEST_F(PiecewiseGroundCurveOverlappedGeometries, PFromP) {
  const double kExpectedPPiecewiseHalfLineA{kPHalfLineA};
  const double kExpectedPPiecewiseHalfLineB{kPHalfLineB + (kP1LineA - kP0LineB)};
  const double kExpectedPPiecewiseHalfLineC{kPHalfLineC + (kP1LineA - kP0LineB) + (kP1LineB - kP0LineC)};
  const double kExpectedPPiecewiseHalfLineD{kPHalfLineD + (kP1LineA - kP0LineB) + (kP1LineB - kP0LineC) +
                                            (kP1LineC - kP0LineD)};

  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineA, kDxyLineA, kP0LineA, kP1LineA));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineB, kDxyLineB, kP0LineB, kP1LineB));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineC, kDxyLineC, kP0LineC, kP1LineC));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineD, kDxyLineD, kP0LineD, kP1LineD));

  const auto dut = PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, kAngularTolerance);
  EXPECT_EQ(kExpectedPPiecewiseHalfLineA, dut.PFromP(kPHalfLineA));
  EXPECT_EQ(kExpectedPPiecewiseHalfLineB, dut.PFromP(kPHalfLineB));
  EXPECT_EQ(kExpectedPPiecewiseHalfLineC, dut.PFromP(kPHalfLineC));
  EXPECT_EQ(kExpectedPPiecewiseHalfLineD, dut.PFromP(kPHalfLineD));
}

// Tests the PFromP method using distanced geometries.
// The use of large tolerances is due to better exemplify the issue.
class PiecewiseGroundCurveDistancedGeometries : public ::testing::Test {
 protected:
  const double kLinearTolerance{1.1};  // Note that all the differences are by 1.
  const double kAngularTolerance{1};
  // Line A.
  const double kP0LineA{0.};
  const double kP1LineA{9.};
  const double kPHalfLineA{(kP1LineA - kP0LineA) / 2 + kP0LineA};
  const Vector2 kXY0LineA{0., 0.};
  const Vector2 kDxyLineA{9. * Vector2::UnitX()};
  // Line B.
  const double kP0LineB{10.};
  const double kP1LineB{19.};
  const double kPHalfLineB{(kP1LineB - kP0LineB) / 2 + kP0LineB};
  const Vector2 kXY0LineB{10., 0.};
  const Vector2 kDxyLineB{9. * Vector2::UnitX()};
  // Line C.
  const double kP0LineC{20.};
  const double kP1LineC{29.};
  const double kPHalfLineC{(kP1LineC - kP0LineC) / 2 + kP0LineC};
  const Vector2 kXY0LineC{20., 0.};
  const Vector2 kDxyLineC{9. * Vector2::UnitX()};
  // Line D.
  const double kP0LineD{30.};
  const double kP1LineD{39.};
  const double kPHalfLineD{(kP1LineD - kP0LineD) / 2 + kP0LineD};
  const Vector2 kXY0LineD{30., 0.};
  const Vector2 kDxyLineD{9. * Vector2::UnitX()};
};

TEST_F(PiecewiseGroundCurveDistancedGeometries, PFromP) {
  const double kExpectedPPiecewiseHalfLineA{kPHalfLineA};
  const double kExpectedPPiecewiseHalfLineB{kPHalfLineB - (kP0LineB - kP1LineA)};
  const double kExpectedPPiecewiseHalfLineC{kPHalfLineC - (kP0LineB - kP1LineA) - (kP0LineC - kP1LineB)};
  const double kExpectedPPiecewiseHalfLineD{kPHalfLineD - (kP0LineB - kP1LineA) - (kP0LineC - kP1LineB) -
                                            (kP0LineD - kP1LineC)};

  std::vector<std::unique_ptr<GroundCurve>> ground_curves;
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineA, kDxyLineA, kP0LineA, kP1LineA));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineB, kDxyLineB, kP0LineB, kP1LineB));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineC, kDxyLineC, kP0LineC, kP1LineC));
  ground_curves.push_back(
      std::make_unique<LineGroundCurve>(kLinearTolerance, kXY0LineD, kDxyLineD, kP0LineD, kP1LineD));

  const auto dut = PiecewiseGroundCurve(std::move(ground_curves), kLinearTolerance, kAngularTolerance);
  EXPECT_EQ(kExpectedPPiecewiseHalfLineA, dut.PFromP(kPHalfLineA));
  EXPECT_EQ(kExpectedPPiecewiseHalfLineB, dut.PFromP(kPHalfLineB));
  EXPECT_EQ(kExpectedPPiecewiseHalfLineC, dut.PFromP(kPHalfLineC));
  EXPECT_EQ(kExpectedPPiecewiseHalfLineD, dut.PFromP(kPHalfLineD));
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
