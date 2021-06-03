// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/road_curve/cubic_polynomial.h"

#include <gtest/gtest.h>

#include <maliput/common/assertion_error.h>
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

constexpr double kTolerance{1e-15};
constexpr double kA{1.};
constexpr double kB{2.};
constexpr double kC{3.};
constexpr double kD{4.};
constexpr double kP0{0.25};
constexpr double kP1{50.};

GTEST_TEST(CubicPolynomial, Constructor) {
  // Correct construction.
  EXPECT_NO_THROW(CubicPolynomial(kA, kB, kC, kD, kP0, kP1, kTolerance));
  // Same p0 and p1.
  EXPECT_THROW(CubicPolynomial(kA, kB, kC, kD, kP0, kP0, kTolerance), maliput::common::assertion_error);
  // p1 < p0
  EXPECT_THROW(CubicPolynomial(kA, kB, kC, kD, kP1, kP0, kTolerance), maliput::common::assertion_error);
  // p0 is less than 0.
  EXPECT_THROW(CubicPolynomial(kA, kB, kC, kD, -kP1, kP0, kTolerance), maliput::common::assertion_error);
}

// Evaluates a quadratic polynomial behind the Function interface.
// Computations in this test can be reproduced in python using
// the following snippet:
// @code{
// def f(a, b, c, d, p):
//     return a * p**3 + b * p**2 + c * p + d
//
// def f_dot(a, b, c, p):
//     return 3 * a * p**2 + 2 * b * p + c
//
// def f_dot_dot(a, b, p):
//     return 6 * a * p + 2 * b
//
// a = 1
// b = 2
// c = 3
// d = 4
//
// print(f(a,b,c,d, 0.5))
// print(f_dot(a,b,c, 0.5))
// print(f_dot_dot(a,b, 0.5))
//
//
// print(f(a,b,c,d, 20.5))
// print(f_dot(a,b,c, 20.5))
// print(f_dot_dot(a,b, 20.5))
//
// print(f(a,b,c,d, 50.0))
// print(f_dot(a,b,c, 20.5))
// print(f_dot_dot(a,b, 20.5))
// @endcode
GTEST_TEST(CubicPolynomial, FunctionApi) {
  const CubicPolynomial dut(kA, kB, kC, kD, kP0, kP1, kTolerance);

  EXPECT_NEAR(dut.f(0.5), 6.125, kTolerance);
  EXPECT_NEAR(dut.f_dot(0.5), 5.75, kTolerance);
  EXPECT_NEAR(dut.f_dot_dot(0.5), 7, kTolerance);

  EXPECT_NEAR(dut.f(20.5), 9521.125, kTolerance);
  EXPECT_NEAR(dut.f_dot(20.5), 1345.75, kTolerance);
  EXPECT_NEAR(dut.f_dot_dot(20.5), 127., kTolerance);

  EXPECT_NEAR(dut.f(50.), 130154, kTolerance);
  EXPECT_NEAR(dut.f_dot(50.), 7703., kTolerance);
  EXPECT_NEAR(dut.f_dot_dot(50.), 304., kTolerance);
}

GTEST_TEST(CubicPolynomial, Range) {
  const CubicPolynomial dut(kA, kB, kC, kD, kP0, kP1, kTolerance);

  EXPECT_THROW(dut.f(kP1 + 1.), maliput::common::assertion_error);
  EXPECT_THROW(dut.f_dot(kP1 + 1.), maliput::common::assertion_error);
  EXPECT_THROW(dut.f_dot_dot(kP1 + 1.), maliput::common::assertion_error);
}

GTEST_TEST(CubicPolynomial, IsG1Contiguous) {
  EXPECT_TRUE(CubicPolynomial(kA, kB, kC, kD, kP0, kP1, kTolerance).IsG1Contiguous());
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
