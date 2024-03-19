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

#include <cmath>

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
  const Vector2 kXy0{1., 0.};
  static constexpr double kStartHeading{1.23};
  static constexpr double kStartCurvature{0.1};
  static constexpr double kEndCurvature{0.01};
  static constexpr double kArcLength{100.};
  static constexpr double kP0{1.};
  static constexpr double kP1{10.};
};

TEST_F(SpiralGroundCurveConstructorTest, CorrectlyConstructedDoesNotThrow) {
  EXPECT_NO_THROW(
      SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0, kP1));
}

TEST_F(SpiralGroundCurveConstructorTest, NegativeLinearToleranceThrows) {
  static constexpr double kNegativeTolerance{-1.0};
  EXPECT_THROW(
      {
        SpiralGroundCurve(kNegativeTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0,
                          kP1);
      },
      maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, SameCurvatureThrows) {
  static constexpr double kSameCurvature{0.01};
  EXPECT_THROW(
      {
        SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kSameCurvature, kSameCurvature, kArcLength, kP0, kP1);
      },
      maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, SmallArcLengthThrows) {
  static constexpr double kSmallArcLength{GroundCurve::kEpsilon / 2.};
  EXPECT_THROW(
      {
        SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kSmallArcLength, kP0,
                          kP1);
      },
      maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, NegativeP0Throws) {
  static constexpr double kNegativeP0{-1.};
  EXPECT_THROW(
      {
        SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength,
                          kNegativeP0, kP1);
      },
      maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, NegativeP1Throws) {
  static constexpr double kNegativeP1{-1.};
  EXPECT_THROW(
      {
        SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0,
                          kNegativeP1);
      },
      maliput::common::assertion_error);
}

TEST_F(SpiralGroundCurveConstructorTest, P0AndP1AreAlmostEqualThrows) {
  static constexpr double kAlmostP0P1{kP0 + GroundCurve::kEpsilon / 2.};
  EXPECT_THROW(
      {
        SpiralGroundCurve(kLinearTolerance, kXy0, kStartHeading, kStartCurvature, kEndCurvature, kArcLength, kP0,
                          kAlmostP0P1);
      },
      maliput::common::assertion_error);
}

// Test class to validate the correct value retrieval of the accessors.
class SpiralGroundCurveAccessorsTest : public ::testing::Test {
 protected:
  static constexpr double kTolerance{1e-12};
  static constexpr double kLinearTolerance{1e-9};
  const Vector2 kXy0{1., 0.};
  static constexpr double kStartHeading{1.23};
  static constexpr double kStartCurvature{0.1};
  static constexpr double kEndCurvature{0.01};
  static constexpr double kArcLength{100.};
  static constexpr double kP0{1.};
  static constexpr double kP1{10.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0,       kStartHeading, kStartCurvature,
                               kEndCurvature,    kArcLength, kP0,           kP1};
};

TEST_F(SpiralGroundCurveAccessorsTest, LinearTolerance) {
  EXPECT_NEAR(kLinearTolerance, dut_.linear_tolerance(), kTolerance);
}

TEST_F(SpiralGroundCurveAccessorsTest, p0) { EXPECT_NEAR(kP0, dut_.p0(), kTolerance); }

TEST_F(SpiralGroundCurveAccessorsTest, p1) { EXPECT_NEAR(kP1, dut_.p1(), kTolerance); }

TEST_F(SpiralGroundCurveAccessorsTest, ArcLength) { EXPECT_NEAR(kArcLength, dut_.ArcLength(), kTolerance); }

TEST_F(SpiralGroundCurveAccessorsTest, IsG1Contiguous) { EXPECT_TRUE(dut_.IsG1Contiguous()); }

// Approximates a path length lower bound for the given @p dut
// from @p p0 to @p p1, by computing the same integral for a 2^@p k_order
// linear approximation.
//
// @param dut The SpiralGroundCurve to compute path length for.
// @param p0 The lower integration bound for the @f$ p @f$ coordinate.
// @param p1 The upper integration bound for the @f$ p @f$ coordinate.
// @param k_order Order k of the linear approximation, i.e. 2^k segments
//                are used in the approximation.
// @pre Given lower integration bound @p p0 doesn't meet the range `[road_curve.p0() ; road_curve.p1()]`
// @pre Given upper integration bound @p p1 doesn't meet the range `[road_curve.p0() ; road_curve.p1()]`
// @pre Given lower integration bound @p p0 is greater than or equal to 0.
// @pre Given upper integration bound @p p1 is greater than or equal to the given lower integration bound @p p0.
// @pre Given @p k_order for the linear approximation is a non-negative number.
// @throws maliput::common::assertion_error if preconditions are not met.
double BruteForcePathLengthIntegral(const SpiralGroundCurve& dut, double p0, double p1, int k_order) {
  MALIDRIVE_IS_IN_RANGE(p0, dut.p0(), dut.p1());
  MALIDRIVE_IS_IN_RANGE(p1, dut.p0(), dut.p1());
  MALIPUT_THROW_UNLESS(p1 >= 0.);
  MALIPUT_THROW_UNLESS(p1 >= p0);
  MALIPUT_THROW_UNLESS(k_order >= 0);

  double length = 0.0;
  const int iterations = std::pow(2, k_order);
  const double step_p = (p1 - p0) / static_cast<double>(iterations);
  // Splits the [p0, p1] interval in 2^k intervals, computes the positions
  // in the global frame for each interval boundary and sums up the path lengths
  // of the segments in the global frame that correspond to each one of those
  // intervals.
  Vector2 pos_prev_p = dut.G(p0);
  for (int i = 1; i <= iterations; ++i) {
    const double p = p0 + static_cast<double>(i) * step_p;
    const Vector2 pos_p = dut.G(p);
    const double ith_step_length = (pos_p - pos_prev_p).norm();
    length += ith_step_length;
    pos_prev_p = pos_p;
  }

  return length;
}

// Evaluates the SpiralGroundCurve with a normalized spiral.
// This tests asserts the internal Fresnel power series expansion are correct
// when the normalization factors do not intervene.
// Values here were retrieved against an alternative and simpler implementation in python
// which does not account for normalization factors and evaluates directly the parameters.
// A witness python script is provided below:
// @code{.py}
// import numpy as np
// import math
// import scipy.special as sp
//
// def rot_vec(x, y, angle):
//     ca = math.cos(angle)
//     sa = math.sin(angle)
//     return (x * ca - y * sa, x * sa + y * ca)
//
// def fresnel(t):
//     yx = sp.fresnel(t)
//     return np.array([yx[1], yx[0]])
//
// def fresnel_heading(t, k_dot):
//     return 0.5 * t * t * k_dot
//
// def heading(t, heading0, k_dot, heading0_spiral):
//     return fresnel_heading(t, k_dot) - heading0_spiral + heading0
//
// def heading_dot(t, k_dot):
//     return t * k_dot
//
// def g(t, heading0, norm, k_dot, heading0_spiral, xy0_spiral):
//     xy = np.array(fresnel(t) * norm) - xy0_spiral
//     h = heading(t, heading0, k_dot, heading0_spiral)
//     x, y = rot_vec(xy[0], xy[1], h)
//     return np.array([x, y])
//
// def g_dot(t, heading0, k_dot, heading0_spiral):
//     h = heading(t, heading0, k_dot, heading0_spiral)
//     return np.array([np.cos(h), np.sin(h)])
//
// def t_from_p(p, s0_spiral, norm):
//     return (p - 0 + s0_spiral) / norm
//
// def arc_length(t1, heading0, k_order, norm, k_dot, heading0_spiral, xy0_spiral):
//     iters = 2 ** k_order
//     step_t = t1 / iters
//     xy_prev = g(0, heading0, norm, k_dot, heading0_spiral, xy0_spiral)
//     l = 0.
//     for i in range(1, iters+1):
//         t = step_t * i
//         xy = g(t, heading0, norm, k_dot, heading0_spiral, xy0_spiral)
//         l += np.linalg.norm(xy - xy_prev)
//         xy_prev = xy
//     return l
//
// # Change the following items to reproduce test results {
// k0 = 0.
// k1 = 0.2
// L = 10.
// heading0 = 0.
// # }
//
// k_dot = (k1 - k0) / L
// norm = math.sqrt(math.pi / abs(k_dot))
// s0_spiral = k0 / k_dot
// xy0_spiral = np.array(sp.fresnel(s0_spiral / norm)) * norm
// heading0_spiral = fresnel_heading(s0_spiral / norm, k_dot)
//
// P = np.array([0., 5., 10.0])
//
// T = t_from_p(P, s0_spiral, norm)
//
// print(f"k_dot:{k_dot:.10f}")
// print(f"norm:{norm:.10f}")
// print(f"s0_spiral:{s0_spiral:.10f}")
// print(f"xy0_spiral:{xy0_spiral}")
// print(f"heading0_spiral:{heading0_spiral:.10f}")
// print(f"heading0:{heading0:.10f}")
// print("---")
//
// for i in range(0, len(T)):
//     xy = g(T[i], heading0, norm, k_dot, heading0_spiral, xy0_spiral)
//     xy_dot = g_dot(T[i], heading0, k_dot, heading0_spiral)
//     h = heading(T[i], heading0, k_dot, heading0_spiral)
//     h_dot = heading_dot(T[i], k_dot)
//     print(f"t:{T[i]} , p:{P[i]}")
//     print(f"g({T[i]}): ({xy[0]:.10f} , {xy[1]:.10f})")
//     print(f"g_dot({T[i]}): ({xy_dot[0]:.10f} , {xy_dot[1]:.10f})")
//     print(f"heading({T[i]}): {h}")
//     print(f"heading_dot({T[i]}): {h_dot}")
//     print(f"arc_length({T[i]}): {arc_length(T[i], heading0, 5, norm, k_dot, heading0_spiral, xy0_spiral)}")
//     print("---")
// @code
// </pre>

// Base class containing tolerances and brute force integration parameters.
class BaseSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-10};
  static constexpr double kGInverseTolerance{5e-2};  // TODO: reduce to 1e-6 or smaller.
  static constexpr int kOrder{7};
  static constexpr double kToleranceForBruteForceIntegral{5e-2};  // TODO: reduce to 1e-6 or smaller.
};

// Tests the SpiralGroundCurve with a curvature derivative of one, making it normalized.
class NormalizedSpiralGroundCurveTest : public BaseSpiralGroundCurveTest {
 protected:
  const Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{0.};
  static constexpr double kStartCurvature{0.};
  static constexpr double kEndCurvature{1.};
  static constexpr double kArcLength{1.};
  static constexpr double kP0{0.};
  static constexpr double kMidP{0.5};
  static constexpr double kP1{1.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0,       kStartHeading, kStartCurvature,
                               kEndCurvature,    kArcLength, kP0,           kP1};
};

TEST_F(NormalizedSpiralGroundCurveTest, G) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0., 0.}, dut_.G(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4979964103, 0.0406516876}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9370155385, 0.3162123412}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(NormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9992085327, 0.0397782381}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9873615638, 0.1584838866}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(NormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.03978873577297383, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.15915494309189532, dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(NormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(0.28209479177387814, dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.5641895835477563, dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(NormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4979964103, 0.0406516876}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.9370155385, 0.3162123412}), kGInverseTolerance);
}

TEST_F(NormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
}

// Tests that an offset in the p-parameter of the GroundCurve does not affect the resulting geometry.
class OffsetPNormalizedSpiralGroundCurveTest : public BaseSpiralGroundCurveTest {
 protected:
  const Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{0.};
  static constexpr double kStartCurvature{0.};
  static constexpr double kEndCurvature{1.};
  static constexpr double kArcLength{1.};
  static constexpr double kP0{10.};
  static constexpr double kMidP{10.5};
  static constexpr double kP1{11.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0,       kStartHeading, kStartCurvature,
                               kEndCurvature,    kArcLength, kP0,           kP1};
};

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, G) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0., 0.}, dut_.G(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4979964103, 0.0406516876}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9370155385, 0.3162123412}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9992085327, 0.0397782381}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9873615638, 0.1584838866}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.03978873577297383, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.15915494309189532, dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(0.28209479177387814, dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.5641895835477563, dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4968840292, 0.0414810243}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.9045242379, 0.3102683017}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(OffsetPNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
}

// Tests how the spiral evolves with a positive curvatures but negative curvature derivative.
class InvertedCurvatureNormalizedSpiralGroundCurveTest : public BaseSpiralGroundCurveTest {
 protected:
  const Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{0.};
  static constexpr double kStartCurvature{1.};
  static constexpr double kEndCurvature{0.};
  static constexpr double kArcLength{1.};
  static constexpr double kP0{0.};
  static constexpr double kMidP{0.5};
  static constexpr double kP1{1.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0,       kStartHeading, kStartCurvature,
                               kEndCurvature,    kArcLength, kP0,           kP1};
};

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, G) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0., 0.}, dut_.G(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4896982421, -0.0851954681}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9889076155, -0.0070775745}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9928843092, 0.1190829484}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9873615638, 0.1584838866}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.11936620731892149, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.15915494309189532, dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0.5641895835477563, dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(0.28209479177387814, dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(0., dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4896982421, -0.0851954681}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.9889076155, -0.0070775745}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
}

// Tests how the spiral evolves with a negative curvature derivative.
class InvertedCurvatureGradientNormalizedSpiralGroundCurveTest : public BaseSpiralGroundCurveTest {
 protected:
  const Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{0.};
  static constexpr double kStartCurvature{0.};
  static constexpr double kEndCurvature{-1.};
  static constexpr double kArcLength{1.};
  static constexpr double kP0{0.};
  static constexpr double kMidP{0.5};
  static constexpr double kP1{1.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0,       kStartHeading, kStartCurvature,
                               kEndCurvature,    kArcLength, kP0,           kP1};
};

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, G) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0., 0.}, dut_.G(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4979964103, -0.0406516876}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9370155385, -0.3162123412}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9992085327, -0.0397782381}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9873615638, -0.1584838866}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(-0.03978873577297383, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(-0.15915494309189532, dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(-0.28209479177387814, dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(-0.5641895835477563, dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4979964103, -0.0406516876}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.9370155385, -0.3162123412}), kGInverseTolerance);
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
}

// Test that the start non zero heading rotates the spiral by that angle.
class HeadingOffsetNormalizedSpiralGroundCurveTest : public BaseSpiralGroundCurveTest {
 protected:
  const Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{M_PI * 0.25};
  static constexpr double kStartCurvature{0.};
  static constexpr double kEndCurvature{1.};
  static constexpr double kArcLength{1.};
  static constexpr double kP0{0.};
  static constexpr double kMidP{0.5};
  static constexpr double kP1{1.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0,       kStartHeading, kStartCurvature,
                               kEndCurvature,    kArcLength, kP0,           kP1};
};

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, G) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0., 0.}, dut_.G(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.3233915547, 0.3808817227}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4389741506, 0.8861659321}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0.7071067812, 0.7071067812}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.6784196674, 0.7346745912}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.5861050263, 0.8102350882}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0.7853981633974483, dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.8251868991704221, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.9445531064893435, dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(0.28209479177387814, dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.5641895835477563, dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.3233915547, 0.3808817227}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.4389741506, 0.8861659321}), kGInverseTolerance);
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
}

// Tests the case in which the curvature derivative is not normalized.
class DenormalizedSpiralGroundCurveTest : public BaseSpiralGroundCurveTest {
 protected:
  const Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{0.};
  static constexpr double kStartCurvature{0.};
  static constexpr double kEndCurvature{0.2};
  static constexpr double kArcLength{10.};
  static constexpr double kP0{0.};
  static constexpr double kMidP{5.};
  static constexpr double kP1{10.};
  const SpiralGroundCurve dut_{kLinearTolerance, kXy0,       kStartHeading, kStartCurvature,
                               kEndCurvature,    kArcLength, kP0,           kP1};
};

TEST_F(DenormalizedSpiralGroundCurveTest, G) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0., 0.}, dut_.G(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({4.9681738083, 0.4227178689}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({9.0253069245, 3.1602035564}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(DenormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9999987335, 0.0015915488}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9999797358, 0.0063661547}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(DenormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.0015915494309189536, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.006366197723675814, dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(DenormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(0.007978845608028654, dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(0.015957691216057307, dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(DenormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({4.9681738083, 0.4227178689}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({9.0253069245, 3.1602035564}), kGInverseTolerance);
}

TEST_F(DenormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive