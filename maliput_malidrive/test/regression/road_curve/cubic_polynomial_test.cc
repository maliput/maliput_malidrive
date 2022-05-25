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
