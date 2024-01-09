// BSD 3-Clause License
//
// Copyright (c) 2024, Woven Planet. All rights reserved.
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
#include "maliput_malidrive/road_curve/spiral_ground_curve.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
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

// Test class to validate the constructor constraints of the SpiralGroundCurve.
class SpiralGroundCurveConstructorTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-9};
  const maliput::math::Vector2 kXy0{1., 0.};
  static constexpr double kStartHeading{1.23};
  static constexpr double kStartCurvature{0.1};
  static constexpr double kEndCurvature{0.01};
  static constexpr double kArcLength{100.};
  static constexpr double kP0{1.};
  static constexpr double kP1{10.};
};

TEST_F(SpiralGroundCurveConstructorTest, CorrectlyConstructedDoesNotThrow) {
  EXPECT_NO_THROW(SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0, kP1));
}

TEST_F(SpiralGroundCurveConstructorTest, NegativeLinearToleranceThrows) {
  static constexpr double kNegativeTolerance{-1.0};
  EXPECT_THROW({ SpiralGroundCurve(kNegativeTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0, kP1); }, maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, SameCurvatureThrows) {
  static constexpr double kSameCurvature{0.01};
  EXPECT_THROW({ SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kSameCurvature, kSameCurvature, kArcLength, kP0, kP1); }, maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, CurvaturesWithDifferentSignsThrows) {
  EXPECT_THROW({ SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, -kEndCurvature, kArcLength, kP0, kP1); }, maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, SmallArcLengthThrows) {
  static constexpr double kSmallArcLength{GroundCurve::kEpsilon / 2.};
  EXPECT_THROW({ SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kSmallArcLength, kP0, kP1); }, maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, NegativeP0Throws) {
  static constexpr double kNegativeP0{-1.};
  EXPECT_THROW({ SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kNegativeP0, kP1); }, maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, NegativeP1Throws) {
  static constexpr double kNegativeP1{-1.};
  EXPECT_THROW({ SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0, kNegativeP1); }, maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, P0AndP1AreAlmostEqualThrows) {
  static constexpr double kAlmostP0P1{kP0 + GroundCurve::kEpsilon / 2.};
  EXPECT_THROW({ SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0, kAlmostP0P1); }, maliput::common::assertion_error);
}

// Test class to validate the correct value retrieval of the accessors.
class SpiralGroundCurveAccessorsTest : public ::testing::Test {
 protected:
  static constexpr double kTolerance{1e-12};
  static constexpr double kLinearTolerance{1e-9};
  const maliput::math::Vector2 kXy0{1., 0.};
  static constexpr double kStartHeading{1.23};
  static constexpr double kStartCurvature{0.1};
  static constexpr double kEndCurvature{0.01};
  static constexpr double kArcLength{100.};
  static constexpr double kP0{1.};
  static constexpr double kP1{10.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0, kP1};
};

TEST_F(SpiralGroundCurveAccessorsTest, LinearTolerance) {
  EXPECT_NEAR(kLinearTolerance, dut_.linear_tolerance(), kTolerance);
}

TEST_F(SpiralGroundCurveAccessorsTest, p0) {
  EXPECT_NEAR(kP0, dut_.p0(), kTolerance);
}

TEST_F(SpiralGroundCurveAccessorsTest, p1) {
  EXPECT_NEAR(kP1, dut_.p1(), kTolerance);
}

TEST_F(SpiralGroundCurveAccessorsTest, ArcLength) {
  EXPECT_NEAR(kArcLength, dut_.ArcLength(), kTolerance);
}

TEST_F(SpiralGroundCurveAccessorsTest, IsG1Contiguous) {
  EXPECT_TRUE(dut_.IsG1Contiguous());
}

class NormalizedSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kTolerance{1e-6};
  static constexpr double kLinearTolerance{1e-9};
  const maliput::math::Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{0.};
  static constexpr double kStartCurvature{0.};
  static constexpr double kEndCurvature{1.};
  static constexpr double kArcLength{1.};
  static constexpr double kP0{0.};
  static constexpr double kMidP{0.5};
  static constexpr double kP1{1.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0, kP1};
};


TEST_F(NormalizedSpiralGroundCurveTest, G) {
  // check values at p0, mid p, and p1.
  EXPECT_TRUE(AssertCompare(CompareVectors({0., 0.}, dut_.G(kP0), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4968840292 , 0.0414810243}, dut_.G(kMidP), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9045242379 , 0.3102683017}, dut_.G(kP1), kTolerance)));
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
