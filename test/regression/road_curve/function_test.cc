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
#include "maliput_malidrive/road_curve/function.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

// Quadratic polynomial implementation behind Function interface.
// In other words, it describes @f$ F(p) = a*p^2 + b*p + c @f$.
class QuadraticPolynomial : public Function {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticPolynomial);

  QuadraticPolynomial(double a, double b, double c, double p0, double p1) : a_(a), b_(b), c_(c), p0_(p0), p1_(p1) {}

 private:
  double do_f(double p) const override {
    MALIDRIVE_IS_IN_RANGE(p, p0_, p1_);
    return a_ * p * p + b_ * p + c_;
  }
  double do_f_dot(double p) const override {
    MALIDRIVE_IS_IN_RANGE(p, p0_, p1_);
    return 2. * a_ * p + b_;
  }
  double do_f_dot_dot(double p) const override {
    MALIDRIVE_IS_IN_RANGE(p, p0_, p1_);
    return 2. * a_;
  }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }

  const double a_{};
  const double b_{};
  const double c_{};
  const double p0_{};
  const double p1_{};
};

constexpr double kTolerance{1e-15};

// Evaluates a quadratic polynomial behind the Function interface.
// Computations in this test can be reproduced in python using
// the following snippet:
// @code{
// def f(a, b, c, p):
//   return a * p * p + b * p + c
//
// def f_dot(a, b, p):
//   return 2. * a *p + b
//
// def f_dot_dot(a):
//   return 2. * a
//
// a = 1.
// b = 2.
// c = 3.
//
// print(f(a, b, c, 5))
// print(f_dot(a, b, 5))
// print(f_dot_dot(a))
// @endcode
GTEST_TEST(QuadraticPolynomial, BasicTest) {
  constexpr double kA{1.};
  constexpr double kB{2.};
  constexpr double kC{3.};
  constexpr double kP0{1.};
  constexpr double kP1{11.};

  const QuadraticPolynomial dut(kA, kB, kC, kP0, kP1);

  EXPECT_NEAR(dut.f(5.), 38.0, kTolerance);
  EXPECT_NEAR(dut.f_dot(5.), 12.0, kTolerance);
  EXPECT_NEAR(dut.f_dot_dot(5.), 2.0, kTolerance);
  EXPECT_EQ(dut.p0(), kP0);
  EXPECT_EQ(dut.p1(), kP1);
  EXPECT_TRUE(dut.IsG1Contiguous());
}

// Absolute linear function implementation behind Function interface.
// In other words, it describes @f$ F(p) = |a*p + b | @f$. This function is G⁰
// but not G¹. At @f$ p = -b/a @f$ (the vertex), left and right derivatives of
// @f$ F(p) @f$ are distinct.
class AbsLinearPolynomial : public Function {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(AbsLinearPolynomial);

  AbsLinearPolynomial(double a, double b, double p0, double p1) : a_(a), b_(b), p0_(p0), p1_(p1), vertex_(-b / a) {}

 private:
  double do_f(double p) const override {
    MALIDRIVE_IS_IN_RANGE(p, p0_, p1_);
    return std::abs(a_ * p + b_);
  }
  double do_f_dot(double p) const override {
    MALIDRIVE_IS_IN_RANGE(p, p0_, p1_);
    MALIDRIVE_THROW_UNLESS(p != vertex_);
    return p > (-b_ / a_) ? a_ : -a_;
  }
  double do_f_dot_dot(double p) const override {
    MALIDRIVE_IS_IN_RANGE(p, p0_, p1_);
    MALIDRIVE_THROW_UNLESS(p != vertex_);
    return 0.;
  }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return vertex_ < p0_ || vertex_ > p1_; }

  const double a_{};
  const double b_{};
  const double p0_{};
  const double p1_{};
  const double vertex_{};
};

// The vertex is not in range --> F(p) is G¹ in the interval.
GTEST_TEST(AbsLinear, VertexIsNotInRange) {
  constexpr double kA{1.};
  constexpr double kB{1.};
  constexpr double kP0{2.};
  constexpr double kP1{7.};

  const AbsLinearPolynomial dut(kA, kB, kP0, kP1);

  EXPECT_NEAR(dut.f(5.), 6., kTolerance);
  EXPECT_NEAR(dut.f_dot(5.), 1., kTolerance);
  EXPECT_NEAR(dut.f_dot_dot(5.), 0., kTolerance);
  EXPECT_EQ(dut.p0(), kP0);
  EXPECT_EQ(dut.p1(), kP1);
  EXPECT_TRUE(dut.IsG1Contiguous());
}

// The vertex is in range. --> F(p) is not G¹ in the interval.
GTEST_TEST(AbsLinear, VertexIsInRange) {
  constexpr double kA{1.};
  constexpr double kB{-1.};
  constexpr double kP0{0.};
  constexpr double kP1{3.};

  const AbsLinearPolynomial dut(kA, kB, kP0, kP1);

  // Shows that the function, first and second derivatives exist for any point.
  EXPECT_NEAR(dut.f(0.5), 0.5, kTolerance);
  EXPECT_NEAR(dut.f_dot(0.5), -1., kTolerance);
  EXPECT_NEAR(dut.f_dot_dot(0.5), 0., kTolerance);
  // The function exists at the vertex, but first and second derivative don't.
  EXPECT_NEAR(dut.f(1), 0., kTolerance);
  EXPECT_THROW(dut.f_dot(1.), maliput::common::assertion_error);
  EXPECT_THROW(dut.f_dot_dot(1.), maliput::common::assertion_error);
  EXPECT_EQ(dut.p0(), kP0);
  EXPECT_EQ(dut.p1(), kP1);
  EXPECT_FALSE(dut.IsG1Contiguous());
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
