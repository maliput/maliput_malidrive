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

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

#include <maliput/math/saturate.h>

namespace malidrive {
namespace road_curve {
namespace {
// Number of coefficients for the series expansion of Fresnel sin and cosine.
static constexpr size_t kNumCoefficients{10};

// Computes the factorial of @p n at compile time whenever possible.
// @param n A non-negative integer to compute the factorial to.
// @return n!
constexpr long Factorial(const long n) { return n <= 1L ? 1L : n * Factorial(n - 1L); }

// Computes @p base ^ @p exponent at compile time whenever possible.
// @param base The base of the exponential function.
// @param exponent The exponent of the exponential function.
// @return @p base ^ @p exponent.
constexpr double PowerOf(const double base, const long exponent) {
  return exponent == 0L ? 1L : exponent == 1L ? base : base * PowerOf(base, exponent - 1L);
}

// @return The coefficients of the series expansion for a Fresnel cosine.
constexpr std::array<double, kNumCoefficients> GetFresnelCosCoefficients() {
  constexpr std::array<double, kNumCoefficients> coeff{
      PowerOf(-1., 0) / (double{Factorial(2L * 0L)} * (4. * 0. + 1.)),
      PowerOf(-1., 1) / (double{Factorial(2L * 1L)} * (4. * 1. + 1.)),
      PowerOf(-1., 2) / (double{Factorial(2L * 2L)} * (4. * 2. + 1.)),
      PowerOf(-1., 3) / (double{Factorial(2L * 3L)} * (4. * 3. + 1.)),
      PowerOf(-1., 4) / (double{Factorial(2L * 4L)} * (4. * 4. + 1.)),
      PowerOf(-1., 5) / (double{Factorial(2L * 5L)} * (4. * 5. + 1.)),
      PowerOf(-1., 6) / (double{Factorial(2L * 6L)} * (4. * 6. + 1.)),
      PowerOf(-1., 7) / (double{Factorial(2L * 7L)} * (4. * 7. + 1.)),
      PowerOf(-1., 8) / (double{Factorial(2L * 8L)} * (4. * 8. + 1.)),
      PowerOf(-1., 9) / (double{Factorial(2L * 9L)} * (4. * 9. + 1.)),
  };
  return coeff;
}

// @return The coefficients of the series expansion for a Fresnel sine.
constexpr std::array<double, kNumCoefficients> GetFresnelSinCoefficients() {
  constexpr std::array<double, kNumCoefficients> coeff{
      PowerOf(-1., 0) / (double{Factorial(2L * 0L + 1L)} * (4. * 0. + 3.)),
      PowerOf(-1., 1) / (double{Factorial(2L * 1L + 1L)} * (4. * 1. + 3.)),
      PowerOf(-1., 2) / (double{Factorial(2L * 2L + 1L)} * (4. * 2. + 3.)),
      PowerOf(-1., 3) / (double{Factorial(2L * 3L + 1L)} * (4. * 3. + 3.)),
      PowerOf(-1., 4) / (double{Factorial(2L * 4L + 1L)} * (4. * 4. + 3.)),
      PowerOf(-1., 5) / (double{Factorial(2L * 5L + 1L)} * (4. * 5. + 3.)),
      PowerOf(-1., 6) / (double{Factorial(2L * 6L + 1L)} * (4. * 6. + 3.)),
      PowerOf(-1., 7) / (double{Factorial(2L * 7L + 1L)} * (4. * 7. + 3.)),
      PowerOf(-1., 8) / (double{Factorial(2L * 8L + 1L)} * (4. * 8. + 3.)),
      PowerOf(-1., 9) / (double{Factorial(2L * 9L + 1L)} * (4. * 9. + 3.)),
  };
  return coeff;
}

// @return The exponents of series expansion for a Fresnel cosine.
constexpr std::array<double, kNumCoefficients> GetFresnelCosExponents() {
  constexpr std::array<double, kNumCoefficients> exponents{
      4. * 0. + 1., 4. * 1. + 1., 4. * 2. + 1., 4. * 3. + 1., 4. * 4. + 1.,
      4. * 5. + 1., 4. * 6. + 1., 4. * 7. + 1., 4. * 8. + 1., 4. * 9. + 1.,
  };
  return exponents;
}

// @return The exponents of series expansion for a Fresnel sine.
constexpr std::array<double, kNumCoefficients> GetFresnelSinExponents() {
  constexpr std::array<double, kNumCoefficients> exponents{
      4. * 0. + 3., 4. * 1. + 3., 4. * 2. + 3., 4. * 3. + 3., 4. * 4. + 3.,
      4. * 5. + 3., 4. * 6. + 3., 4. * 7. + 3., 4. * 8. + 3., 4. * 9. + 3.,
  };
  return exponents;
}

// Computes the position of a Fresnel spiral at @p t.
// @param t The normalized spiral parameter.
// @return The position of a Fresnel spiral at @p t.
maliput::math::Vector2 FresnelSpiral(double t) {
  static constexpr std::array<double, kNumCoefficients> cos_exponents = GetFresnelCosExponents();
  static constexpr std::array<double, kNumCoefficients> sin_exponents = GetFresnelSinExponents();
  static constexpr std::array<double, kNumCoefficients> cos_coefficients = GetFresnelCosCoefficients();
  static constexpr std::array<double, kNumCoefficients> sin_coefficients = GetFresnelSinCoefficients();

  std::array<double, kNumCoefficients> x_terms{};
  std::array<double, kNumCoefficients> y_terms{};
  std::fill(x_terms.begin(), x_terms.end(), t);
  std::fill(y_terms.begin(), y_terms.end(), t);

  for (size_t i = 0; i < kNumCoefficients; ++i) {
    x_terms[i] = PowerOf(x_terms[i], cos_exponents[i]);
    y_terms[i] = PowerOf(y_terms[i], sin_exponents[i]);
  }

  return maliput::math::Vector2{
      std::inner_product(x_terms.begin(), x_terms.end(), cos_coefficients.begin(), 0.),
      std::inner_product(y_terms.begin(), y_terms.end(), sin_coefficients.begin(), 0.),
  };
}

// @return A vector equivalent of @p v rotated by @p theta.
inline maliput::math::Vector2 Rotate2dVector(const maliput::math::Vector2& v, double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  return maliput::math::Vector2(v.x() * cos_theta - v.y() * sin_theta, v.x() * sin_theta + v.y() * cos_theta);
}

}  // namespace

SpiralGroundCurve::SpiralGroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double heading0,
                                     double curvature0, double curvature1, double arc_length, double p0, double p1)
    : linear_tolerance_(linear_tolerance),
      xy0_(xy0),
      heading0_(heading0),
      curvature0_(curvature0),
      curvature1_(curvature1),
      arc_length_(arc_length),
      p0_(p0),
      p1_(p1),
      validate_p_(maliput::common::RangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_,
                                                                               GroundCurve::kEpsilon)),
      k_dot_{(curvature1 - curvature0) / arc_length},
      norm_{std::sqrt(M_PI) / std::sqrt(std::abs(k_dot_))},
      spiral_p0_{curvature0 * arc_length / (curvature1 - curvature0)},
      spiral_heading0_{std::atan2(std::sin(PowerOf(spiral_p0_ / norm_, 2)), std::cos(PowerOf(spiral_p0_ / norm_, 2)))},
      spiral_xy0_{FresnelSpiral(spiral_p0_ / norm_) * norm_} {
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0.);
  MALIDRIVE_THROW_UNLESS(arc_length_ >= GroundCurve::kEpsilon);
  MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
  MALIDRIVE_THROW_UNLESS(p1_ - p0_ >= GroundCurve::kEpsilon);
  MALIDRIVE_THROW_UNLESS(std::fabs(curvature1_ - curvature0_) >= GroundCurve::kEpsilon);
  spiral_xy0_ = k_dot_ < 0. ? maliput::math::Vector2{spiral_xy0_.x(), -spiral_xy0_.y()} : spiral_xy0_;
}

double SpiralGroundCurve::DoPFromP(double xodr_p) const { return validate_p_(xodr_p); }

double SpiralGroundCurve::NormalizedSpiralCoordinateAt(double p) const { return (p - p0_ + spiral_p0_) / norm_; }

maliput::math::Vector2 SpiralGroundCurve::DoG(double p) const {
  p = validate_p_(p);
  const double t = NormalizedSpiralCoordinateAt(p);
  maliput::math::Vector2 spiral_pos = (FresnelSpiral(t) - spiral_xy0_) * norm_;
  spiral_pos = k_dot_ < 0. ? maliput::math::Vector2{spiral_pos.x(), -spiral_pos.y()} : spiral_pos;
  return xy0_ + Rotate2dVector(spiral_pos, heading0_ - spiral_heading0_);
}

maliput::math::Vector2 SpiralGroundCurve::DoGDot(double p) const {
  p = validate_p_(p);
  const double t = NormalizedSpiralCoordinateAt(p);
  maliput::math::Vector2 g_dot = maliput::math::Vector2{std::cos(PowerOf(t, 2)), std::sin(PowerOf(t, 2))} / norm_;
  g_dot = k_dot_ < 0. ? maliput::math::Vector2{g_dot.x(), -g_dot.y()} : g_dot;
  return Rotate2dVector(g_dot, heading0_ - spiral_heading0_);
}

namespace {

// @brief Find the best p parameter for @p curve that maps a point in the 2D INERTIAL-Frame that minimizes
// the distance to @p point.
// @details The function implements a binary search within a range trying to minimize the distance against
// @p point. The range is split into equal @p partitions. The search can be early terminated if any of the
// extents of the range at each iteration is within @p tolerance.
// @param curve The SpiralGoundCurve to use.
// @param point The point in the 2D INERTIAL-Frame to find a p parameter value for.
// @param p0 The p parameter start range value. It must be non-negative.
// @param p1 The p parameter end range value. It must be greater than @p p0 by GroundCurve::kEpsilon.
// @param partitions The number of partitions of the range [@p p0; @p p1]. It must be greater or equal to 2.
// @param tolerance The tolereance to early stop the search. It must be non-negative.
// @return The p parameter value that minimizes the distance against @p point.
double FindBestPParameter(const SpiralGroundCurve& curve, const maliput::math::Vector2& point, double p0, double p1,
                          size_t partitions, double tolerance) {
  MALIPUT_THROW_UNLESS(p0 >= 0.);
  MALIPUT_THROW_UNLESS(p1 - p0 >= GroundCurve::kEpsilon);
  MALIPUT_THROW_UNLESS(partitions >= 2u);
  MALIPUT_THROW_UNLESS(tolerance >= 0.);

  const size_t kIterations{2u * static_cast<size_t>(std::ceil(std::log(partitions))) + 1u};
  // Initialize the list of p values.
  std::vector<double> p_values(partitions);
  const double step = (p1 - p0) / static_cast<double>(partitions - 1u);
  for (size_t i = 0u; i < partitions - 1u; ++i) {
    p_values[i] = p0 + static_cast<double>(i) * step;
  }
  p_values[partitions - 1u] = p1;

  // Find the p-value that minimizes the distance to the target point using a binary search
  // across the list of p values.
  size_t start_i{0u};
  size_t end_i{partitions - 1u};
  double start_distance{std::numeric_limits<double>::max()};
  double end_distance{std::numeric_limits<double>::max()};
  for (size_t iter = 0u; iter < kIterations; ++iter) {
    const maliput::math::Vector2 start_point = curve.G(p_values[start_i]);
    const maliput::math::Vector2 end_point = curve.G(p_values[end_i]);
    start_distance = (start_point - point).norm();
    end_distance = (end_point - point).norm();
    // Early termination conditions.
    if (start_distance <= tolerance || end_distance <= tolerance) {
      break;
    }
    if ((end_i - start_i) <= 1u) {
      break;
    }
    // Index adjustment for the next iteration.
    if (start_distance <= end_distance) {
      end_i = std::ceil((end_i + start_i) / 2u);
    } else {
      start_i = std::floor((end_i + start_i) / 2u);
    }
  }
  return start_distance <= end_distance ? p_values[start_i] : p_values[end_i];
}

}  // namespace

double SpiralGroundCurve::DoGInverse(const maliput::math::Vector2& point) const {
  static constexpr size_t kMaxIterations{10u};
  static constexpr size_t kPartitionSize{1000u};

  // Initializes the error and the extents of the range to search.
  double start_p = p0_;
  double end_p = p1_;
  double best_p = p0_;
  // Iterates in the search to get a better precision of the p value.
  for (size_t i = 0; i < kMaxIterations; ++i) {
    // Obtains the best p parameter value for the given range.
    best_p = FindBestPParameter(*this, point, start_p, end_p, kPartitionSize, linear_tolerance_);
    // Computes the error, i.e. distance to the point to match. When the error
    // is less or equal to linear_tolerance, there is no need to continue iterating.
    if ((G(best_p) - point).norm() <= linear_tolerance_) {
      return best_p;
    }
    // Creates a new extent whose range is step. Note that FindBestPParameter
    // returns the minimum out of the two possible values in the range.
    const double step = (end_p - start_p) / static_cast<double>(kPartitionSize);
    start_p = std::max(p0_, best_p - step);
    end_p = std::min(p1_, best_p + step);
    // When the range has been constrained beyond the GroundCurve::kEpsilon, we should
    // return to avoid further numerical error even if the parameter is not good enough.
    if ((end_p - start_p) < GroundCurve::kEpsilon) {
      return best_p;
    }
  }

  return best_p;
}

double SpiralGroundCurve::DoHeading(double p) const {
  const maliput::math::Vector2 g_dot = GDot(p);
  return std::atan2(g_dot.y(), g_dot.x());
}

double SpiralGroundCurve::DoHeadingDot(double p) const {
  p = validate_p_(p);
  const double t = NormalizedSpiralCoordinateAt(p);
  return 2. * t * k_dot_ * norm_;
}

}  // namespace road_curve
}  // namespace malidrive
