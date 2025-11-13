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
#include "maliput_malidrive/road_curve/param_poly3_ground_curve.h"

#include <algorithm>
#include <limits>

#include <maliput/common/maliput_throw.h>
#include <maliput/math/saturate.h>

namespace malidrive {
namespace road_curve {

ParamPoly3GroundCurve::ParamPoly3GroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0,
                                             double heading0, double aU, double bU, double cU, double dU, double aV,
                                             double bV, double cV, double dV, double arc_length, double p0, double p1,
                                             PRangeType p_range)
    : linear_tolerance_(linear_tolerance),
      xy0_(xy0),
      heading0_(heading0),
      aU_(aU),
      bU_(bU),
      cU_(cU),
      dU_(dU),
      aV_(aV),
      bV_(bV),
      cV_(cV),
      dV_(dV),
      arc_length_(arc_length),
      p0_(p0),
      p1_(p1),
      p_range_(p_range),
      validate_p_(maliput::common::RangeValidator<maliput::common::road_geometry_construction_error>::
                      GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_, GroundCurve::kEpsilon)) {
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0., maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(arc_length_ >= GroundCurve::kEpsilon, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(p0_ >= 0., maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(p1_ - p0_ >= GroundCurve::kEpsilon, maliput::common::road_geometry_construction_error);
}

double ParamPoly3GroundCurve::NormalizedP(double p) const {
  const double p_offset = p - p0_;
  const double p_range = p1_ - p0_;

  switch (p_range_) {
    case PRangeType::kArcLength:
      // Map [p0, p1] to [0, arc_length]
      return p_offset * arc_length_ / p_range;
    case PRangeType::kNormalized:
      // Map [p0, p1] to [0, 1]
      return p_offset / p_range;
    default:
      MALIDRIVE_THROW_MESSAGE("Invalid PRangeType");
  }
}

double ParamPoly3GroundCurve::U(double p_norm) const {
  return aU_ + bU_ * p_norm + cU_ * p_norm * p_norm + dU_ * p_norm * p_norm * p_norm;
}

double ParamPoly3GroundCurve::V(double p_norm) const {
  return aV_ + bV_ * p_norm + cV_ * p_norm * p_norm + dV_ * p_norm * p_norm * p_norm;
}

double ParamPoly3GroundCurve::UDot(double p_norm) const {
  return bU_ + 2. * cU_ * p_norm + 3. * dU_ * p_norm * p_norm;
}

double ParamPoly3GroundCurve::VDot(double p_norm) const {
  return bV_ + 2. * cV_ * p_norm + 3. * dV_ * p_norm * p_norm;
}

maliput::math::Vector2 ParamPoly3GroundCurve::DoG(double p) const {
  p = validate_p_(p);
  const double p_norm = NormalizedP(p);
  const double u = U(p_norm);
  const double v = V(p_norm);

  // Transform from local (u,v) coordinates to global (x,y) coordinates
  // x = x0 + u*cos(heading0) - v*sin(heading0)
  // y = y0 + u*sin(heading0) + v*cos(heading0)
  return xy0_ + maliput::math::Vector2(u * cos_heading0_ - v * sin_heading0_, u * sin_heading0_ + v * cos_heading0_);
}

maliput::math::Vector2 ParamPoly3GroundCurve::DoGDot(double p) const {
  p = validate_p_(p);
  const double p_norm = NormalizedP(p);
  const double u_dot = UDot(p_norm);
  const double v_dot = VDot(p_norm);

  // Compute the derivative scaling factor based on p_range type
  double dp_norm_dp;
  const double p_range = p1_ - p0_;

  switch (p_range_) {
    case PRangeType::kArcLength:
      dp_norm_dp = arc_length_ / p_range;
      break;
    case PRangeType::kNormalized:
      dp_norm_dp = 1.0 / p_range;
      break;
    default:
      MALIDRIVE_THROW_MESSAGE("Invalid PRangeType");
  }

  // Chain rule: dG/dp = dG/dp_norm * dp_norm/dp
  // where dG/dp_norm in local coords is (u_dot, v_dot)
  const double dx_dp = (u_dot * cos_heading0_ - v_dot * sin_heading0_) * dp_norm_dp;
  const double dy_dp = (u_dot * sin_heading0_ + v_dot * cos_heading0_) * dp_norm_dp;

  return maliput::math::Vector2(dx_dp, dy_dp);
}

double ParamPoly3GroundCurve::DoHeading(double p) const {
  p = validate_p_(p);
  const maliput::math::Vector2 g_dot = DoGDot(p);
  return std::atan2(g_dot.y(), g_dot.x());
}

double ParamPoly3GroundCurve::DoHeadingDot(double p) const {
  // Computes the curvature (change in heading).
  // It uses the second derivatives of the polynomials (u'', v'') and the quotient rule for the derivative of arctangent.
  p = validate_p_(p);
  const double p_norm = NormalizedP(p);
  const double u_dot = UDot(p_norm);
  const double v_dot = VDot(p_norm);

  // Second derivatives of the polynomials
  const double u_ddot = 2. * cU_ + 6. * dU_ * p_norm;
  const double v_ddot = 2. * cV_ + 6. * dV_ * p_norm;

  // Compute the derivative scaling factor
  double dp_norm_dp;
  const double p_range = p1_ - p0_;

  switch (p_range_) {
    case PRangeType::kArcLength:
      dp_norm_dp = arc_length_ / p_range;
      break;
    case PRangeType::kNormalized:
      dp_norm_dp = 1.0 / p_range;
      break;
    default:
      MALIDRIVE_THROW_MESSAGE("Invalid PRangeType");
  }

  // The heading angle is the atan2 of the derivative vector (dy/dp, dx/dp).
  // The derivative of heading with respect to p is given by:
  // d/dp[atan2(y', x')] = (x' * y'' - y' * x'') / (x'^2 + y'^2)

  // Transform to global coordinates and apply chain rule
  // Note the double application of dp_norm_dp due to second derivatives(chain rule applies twice)
  const double dx_dp = (u_dot * cos_heading0_ - v_dot * sin_heading0_) * dp_norm_dp;
  const double dy_dp = (u_dot * sin_heading0_ + v_dot * cos_heading0_) * dp_norm_dp;
  const double dx_dp2 = (u_ddot * cos_heading0_ - v_ddot * sin_heading0_) * dp_norm_dp * dp_norm_dp;
  const double dy_dp2 = (u_ddot * sin_heading0_ + v_ddot * cos_heading0_) * dp_norm_dp * dp_norm_dp;

  const double denominator = dx_dp * dx_dp + dy_dp * dy_dp;
  if (denominator < GroundCurve::kEpsilon) {
    return 0.;
  }
  return (dx_dp * dy_dp2 - dy_dp * dx_dp2) / denominator;
}

double ParamPoly3GroundCurve::DoGInverse(const maliput::math::Vector2& xy) const {
  // For parametric curves, GInverse is typically solved using numerical methods.
  // We'll use a simple grid search followed by refinement using Newton's method.
  //  - grid search: sample points along p to find the closest point.
  //  - Newton's method: refine the closest point to minimize distance.

  // Grid search to find approximate minimum
  constexpr int kNumSamples = 100;
  double min_distance = std::numeric_limits<double>::max();
  double best_p = p0_;

  for (int i = 0; i <= kNumSamples; ++i) {
    const double p_sample = p0_ + (p1_ - p0_) * i / kNumSamples;
    const maliput::math::Vector2 g_sample = DoG(p_sample);
    // TODO: Use squared norm to be more efficient.
    const double distance = (g_sample - xy).norm();

    if (distance < min_distance) {
      min_distance = distance;
      best_p = p_sample;
    }
  }

  // Refine using Newton's method
  // We want to minimize distance^2 = |G(p) - xy|^2
  // Taking derivative: d/dp[distance^2] = 2 * (G(p) - xy) · G'(p)
  // Setting to zero and using Newton iteration
  constexpr int kMaxIterations = 10;
  constexpr double kTolerance = 1e-10;

  double p_current = best_p;
  // Convergence: The loop will break if we are close enough to the minimum or max iterations reached.
  for (int iter = 0; iter < kMaxIterations; ++iter) {
    const maliput::math::Vector2 g = DoG(p_current);
    const maliput::math::Vector2 g_dot = DoGDot(p_current);
    const maliput::math::Vector2 diff = g - xy;

    // Let's use Newton's method to find the root of f(p) = 0
    // https://en.wikipedia.org/wiki/Newton%27s_method
    // Let f(p) = (G(p) - xy) · G'(p)
    // We need f'(p) = d/dp[f(p)] for Newton's update
    // Newton's rule:
    //    p_new = p_current - f(p_current) / f'(p_current)

    const double f = diff.dot(g_dot);  // First derivative of distance^2 / 2
    if (std::abs(f) < kTolerance) {
      // If close enough to zero, we found the minimum.
      break;
    }

    // We need f'(p) which would involve second derivatives of G.
    // Computing G''(p) is complex for parametric curves.
    // Use a simplified approach: assume locally linear behavior
    constexpr double kDelta = 1e-6;
    const double step = kDelta * (p1_ - p0_);
    double p_probe = p_current + step;
    if (p_probe > p1_) {
      p_probe = p_current - step;
    }
    p_probe = maliput::math::saturate(p_probe, p0_, p1_);

    const double actual_step = p_probe - p_current;
    if (std::abs(actual_step) < std::numeric_limits<double>::epsilon()) {
      // If we can't step, keep current p as best estimate.
      break;
    }

    const maliput::math::Vector2 g_probe = DoG(p_probe);
    const maliput::math::Vector2 g_dot_probe = DoGDot(p_probe);
    const maliput::math::Vector2 diff_probe = g_probe - xy;

    const double f_probe = diff_probe.dot(g_dot_probe);
    // Approximate derivative using f_prime = (f(probe) - f(current)) / step.
    const double f_prime = (f_probe - f) / actual_step;

    if (std::abs(f_prime) > kTolerance) {
      double p_next = p_current - f / f_prime;
      p_next = maliput::math::saturate(p_next, p0_, p1_);

      if (std::abs(p_next - p_current) < kTolerance * (p1_ - p0_)) {
        // If the update is small enough, we can stop.
        break;
      }
      p_current = p_next;
    } else {
      // If f' is too small, we cannot proceed with Newton's method.
      // Use the current p as the best estimate.
      break;
    }
  }

  return maliput::math::saturate(p_current, p0_, p1_);
}

}  // namespace road_curve
}  // namespace malidrive
