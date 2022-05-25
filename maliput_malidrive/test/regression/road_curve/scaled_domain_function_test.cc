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
#include "maliput_malidrive/road_curve/scaled_domain_function.h"

#include <utility>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

constexpr double kTolerance{1e-9};
constexpr double kA{1.};
constexpr double kB{2.};
constexpr double kC{3.};
constexpr double kD{4.};
constexpr double kP0{0.25};
constexpr double kP1{50.};
constexpr double kScaledP0{2.2};
constexpr double kScaledP1{3.3};

GTEST_TEST(ScaledDomainFunction, Identity) {
  auto cubic_polynomial = std::make_unique<CubicPolynomial>(kA, kB, kC, kD, kP0, kP1, kTolerance);
  const CubicPolynomial* cubic_polynomial_ptr = cubic_polynomial.get();

  const ScaledDomainFunction scaled_dut(std::move(cubic_polynomial), kP0, kP1, kTolerance);

  EXPECT_NEAR(cubic_polynomial_ptr->f(kP0), scaled_dut.f(kP0), kTolerance);
  EXPECT_NEAR(cubic_polynomial_ptr->f(kP1), scaled_dut.f(kP1), kTolerance);

  EXPECT_NEAR(cubic_polynomial_ptr->f_dot(kP0), scaled_dut.f_dot(kP0), kTolerance);
  EXPECT_NEAR(cubic_polynomial_ptr->f_dot(kP1), scaled_dut.f_dot(kP1), kTolerance);

  EXPECT_NEAR(cubic_polynomial_ptr->f_dot_dot(kP0), scaled_dut.f_dot_dot(kP0), kTolerance);
  EXPECT_NEAR(cubic_polynomial_ptr->f_dot_dot(kP1), scaled_dut.f_dot_dot(kP1), kTolerance);
}

GTEST_TEST(ScaledDomainFunction, ScaledDomain) {
  auto cubic_polynomial = std::make_unique<CubicPolynomial>(kA, kB, kC, kD, kP0, kP1, kTolerance);
  const CubicPolynomial* cubic_polynomial_ptr = cubic_polynomial.get();

  const ScaledDomainFunction scaled_dut(std::move(cubic_polynomial), kScaledP0, kScaledP1, kTolerance);
  const double kExpectedScale = (kP1 - kP0) / (kScaledP1 - kScaledP0);

  EXPECT_NEAR(cubic_polynomial_ptr->f(kP0), scaled_dut.f(kScaledP0), kTolerance);
  EXPECT_NEAR(cubic_polynomial_ptr->f(kP1), scaled_dut.f(kScaledP1), kTolerance);

  EXPECT_NEAR(cubic_polynomial_ptr->f_dot(kP0), scaled_dut.f_dot(kScaledP0) / kExpectedScale, kTolerance);
  EXPECT_NEAR(cubic_polynomial_ptr->f_dot(kP1), scaled_dut.f_dot(kScaledP1) / kExpectedScale, kTolerance);

  EXPECT_NEAR(cubic_polynomial_ptr->f_dot_dot(kP0), scaled_dut.f_dot_dot(kScaledP0) / (kExpectedScale * kExpectedScale),
              kTolerance);
  EXPECT_NEAR(cubic_polynomial_ptr->f_dot_dot(kP1), scaled_dut.f_dot_dot(kScaledP1) / (kExpectedScale * kExpectedScale),
              kTolerance);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
