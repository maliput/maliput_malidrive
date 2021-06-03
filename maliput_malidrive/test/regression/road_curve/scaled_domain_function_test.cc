// Copyright 2020 Toyota Research Institute
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
