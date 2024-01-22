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
//
// def fresnel_cos(p):
//     x = 0.
//     for i in range(10):
//         x += ((-1.) ** i) * (p ** (4. * i + 1.)) / ( math.factorial(2 * i) * (4. * i + 1.))
//     return x
//
// def fresnel_sin(p):
//     x = 0.
//     for i in range(10):
//         x += ((-1.) ** i) * (p ** (4. * i + 3.)) / ( math.factorial(2 * i + 1) * (4. * i + 3.))
//     return x
//
// def fresnel_cos_prime(p):
//     return np.cos(p * p)
//
// def fresnel_sin_prime(p):
//     return np.sin(p * p)
//
// def g_p(p):
//     fcv = np.vectorize(fresnel_cos)
//     fsv = np.vectorize(fresnel_sin)
//     return fcv(p), fsv(p)
//
// def g_prime(p):
//     fcpv = np.vectorize(fresnel_cos_prime)
//     fspv = np.vectorize(fresnel_sin_prime)
//     return fcpv(p), fspv(p)
//
// def heading(p):
//     fcpv = np.vectorize(fresnel_cos_prime)
//     fspv = np.vectorize(fresnel_sin_prime)
//     return np.arctan2(fspv(p), fcpv(p))
//
// def heading_dot(p):
//     return 2. * p
//
// P = np.array([0., 0.5, 1.0])
// x, y = g_p(P)
// x_dot, y_dot = g_prime(P)
// h = heading(P)
// h_dot = heading_dot(P)
//
// for i in range(0, len(P)):
//     print(f"g({P[i]}): ({x[i]:.10f} , {y[i]:.10f})")
//     print(f"g_dot({P[i]}): ({x_dot[i]:.10f} , {y_dot[i]:.10f})")
//     print(f"heading({P[i]}): {h[i]}")
//     print(f"heading_dot({P[i]}): {h_dot[i]}")
// @code
//
// Yielding:
//
// <pre>
// g(0.0): (0.0000000000 , 0.0000000000)
// g_dot(0.0): (1.0000000000 , 0.0000000000)
// heading(0.0): 0.0
// heading_dot(0.0): 0.0
// g(0.5): (0.4968840292 , 0.0414810243)
// g_dot(0.5): (0.9689124217 , 0.2474039593)
// heading(0.5): 0.25
// heading_dot(0.5): 1.0
// g(1.0): (0.9045242379 , 0.3102683017)
// g_dot(1.0): (0.5403023059 , 0.8414709848)
// heading(1.0): 1.0
// heading_dot(1.0): 2.0
// </pre>
class NormalizedSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-10};
  static constexpr double kGInverseTolerance{1e-6};
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
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4968840292, 0.0414810243}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9045242379, 0.3102683017}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(NormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9689124217, 0.2474039593}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.5403023059, 0.8414709848}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(NormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.25, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(1., dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(NormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(1., dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(2., dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(NormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4968840292, 0.0414810243}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.9045242379, 0.3102683017}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(NormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  constexpr int kOrder{7};
  constexpr double kToleranceForBruteForceIntegral{1e-5};
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
}

// Same test as NormalizedSpiralGroundCurveTest, but with offsets on kP0.
// The purpose of this test is to assure the internal parametrization is robust.
class OffsetPNormalizedSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-10};
  static constexpr double kGInverseTolerance{1e-6};
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
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4968840292, 0.0414810243}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9045242379, 0.3102683017}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9689124217, 0.2474039593}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.5403023059, 0.8414709848}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.25, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(1., dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(1., dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(2., dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(OffsetPNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4968840292, 0.0414810243}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.9045242379, 0.3102683017}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(OffsetPNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  constexpr int kOrder{7};
  constexpr double kToleranceForBruteForceIntegral{1e-5};
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
}

// This test case works with the negative curvature range [-1.0; 0.0] from a normalized spiral
// and it is translated and rotated to the origin.
class InvertedCurvatureNormalizedSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-10};
  static constexpr double kGInverseTolerance{1e-6};
  const Vector2 kXy0{0., 0.};
  static constexpr double kStartHeading{0.};
  static constexpr double kStartCurvature{-1.};
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
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4464256398, -0.1977910221}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.7497983049, -0.5934922224}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.7316888689, -0.6816387600}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.5403023059, -0.8414709848}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(-0.75, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(-1., dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(-2., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(-1., dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(0., dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4464256398, -0.1977910221}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.7497983049, -0.5934922224}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(InvertedCurvatureNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  constexpr int kOrder{7};
  constexpr double kToleranceForBruteForceIntegral{1e-5};
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
}

// This test case works with the negative curvature range [0.0; -1.0] from a normalized spiral.
// The produced curve is equivalent to the one created in NormalizedSpiralGroundCurveTest but
// mirrored w.r.t. the x axis.
class InvertedCurvatureGradientNormalizedSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-10};
  static constexpr double kGInverseTolerance{1e-6};
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
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4968840292, -0.0414810243}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9045242379, -0.3102683017}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9689124217, -0.2474039593}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.5403023059, -0.8414709848}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(-0.25, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(-1., dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(-1., dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(-2., dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.4968840292, -0.0414810243}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.9045242379, -0.3102683017}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(InvertedCurvatureGradientNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  constexpr int kOrder{7};
  constexpr double kToleranceForBruteForceIntegral{1e-5};
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
}

// Test that the start heading makes the normalized spiral to rotate.
class HeadingOffsetNormalizedSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-10};
  static constexpr double kGInverseTolerance{1e-6};
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
  EXPECT_TRUE(AssertCompare(CompareVectors({0.3220185529, 0.3806815800}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.4202024022, 0.8589880425}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({0.7071067811, 0.7071067811}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.5101835264, 0.8600655610}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({-0.2129584151, 0.9770612639}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(M_PI * 0.25, dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(M_PI * 0.25 + 0.25, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(M_PI * 0.25 + 1., dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(1., dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(2., dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({0., 0.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({0.3220185529, 0.3806815800}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({0.4202024022, 0.8589880424}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(HeadingOffsetNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  constexpr int kOrder{7};
  constexpr double kToleranceForBruteForceIntegral{1e-5};
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
}

// Tests that the non-zero spiral position translates the values of SpiraGroundCurve::G().
class TranslatedNormalizedSpiralGroundCurveTest : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{1e-10};
  static constexpr double kGInverseTolerance{1e-6};
  const Vector2 kXy0{-1., 2.};
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

TEST_F(TranslatedNormalizedSpiralGroundCurveTest, G) {
  EXPECT_TRUE(AssertCompare(CompareVectors({-1., 2.}, dut_.G(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({-0.5031159707, 2.0414810243}, dut_.G(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({-0.0954757620, 2.3102683017}, dut_.G(kP1), kLinearTolerance)));
}

TEST_F(TranslatedNormalizedSpiralGroundCurveTest, GDot) {
  EXPECT_TRUE(AssertCompare(CompareVectors({1., 0.}, dut_.GDot(kP0), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.9689124217, 0.2474039593}, dut_.GDot(kMidP), kLinearTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors({0.5403023059, 0.8414709848}, dut_.GDot(kP1), kLinearTolerance)));
}

TEST_F(TranslatedNormalizedSpiralGroundCurveTest, Heading) {
  EXPECT_NEAR(0., dut_.Heading(kP0), kLinearTolerance);
  EXPECT_NEAR(0.25, dut_.Heading(kMidP), kLinearTolerance);
  EXPECT_NEAR(1., dut_.Heading(kP1), kLinearTolerance);
}

TEST_F(TranslatedNormalizedSpiralGroundCurveTest, HeadingDot) {
  EXPECT_NEAR(0., dut_.HeadingDot(kP0), kLinearTolerance);
  EXPECT_NEAR(1., dut_.HeadingDot(kMidP), kLinearTolerance);
  EXPECT_NEAR(2., dut_.HeadingDot(kP1), kLinearTolerance);
}

TEST_F(TranslatedNormalizedSpiralGroundCurveTest, GInverse) {
  EXPECT_NEAR(kP0, dut_.GInverse({-1., 2.}), kGInverseTolerance);
  EXPECT_NEAR(kMidP, dut_.GInverse({-0.5031159707, 2.0414810243}), kGInverseTolerance);
  EXPECT_NEAR(kP1, dut_.GInverse({-0.0954757620, 2.3102683017}), kGInverseTolerance);
}

// Check the internal normalization parameter does not affect the construct arc length.
TEST_F(TranslatedNormalizedSpiralGroundCurveTest, ProperNormalizationYieldsArcLengthToBekP1MinuskP0) {
  constexpr int kOrder{7};
  constexpr double kToleranceForBruteForceIntegral{1e-5};
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kP1, kOrder), dut_.ArcLength(), kToleranceForBruteForceIntegral);
  EXPECT_NEAR(BruteForcePathLengthIntegral(dut_, kP0, kMidP, kOrder), dut_.ArcLength() / 2.,
              kToleranceForBruteForceIntegral);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
