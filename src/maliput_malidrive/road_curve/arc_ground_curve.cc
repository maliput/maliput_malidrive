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
#include "maliput_malidrive/road_curve/arc_ground_curve.h"

#include <maliput/math/saturate.h>

namespace malidrive {
namespace road_curve {

// This internal namespace has helper functions for angle wrapping copied from maliput_multilane.
namespace {

// Wraps the input angle θ, casting it onto the range [-π, π].
double wrap(double theta) {
  double theta_new = std::fmod(theta + M_PI, 2. * M_PI);
  if (theta_new < 0.) theta_new += 2. * M_PI;
  return theta_new - M_PI;
}

// Implements the saturate function with respect to a range of angles.
// Specifically, given an angle θ ∈ [-π, π] along with lower and upper bounds
// θ_min, θ_max, such that -∞ < θ_min <= θ_max < +∞, this function returns the
// saturated angle sat(θ, θ_min, θ_max) within the range [-π, π].  If θ_max > 2π
// + θ_min, then no saturation occurs.
//
// Note that the interval [θ_min, θ_max] should be mapped into the cyclic
// range [-π, +π] --- and in doing so, it might remain a single closed
// interval [a, b], or it might split into a pair of intervals [-π, a] and [b,
// +π]. In both cases, -π <= a <= b <= +π. If the result is a single
// interval, saturating is the usual. If the result is two intervals, saturating
// involves the extra step picking the 'closest' interval to saturate
// within.
double saturate_on_wrapped_bounds(double theta, double theta_min, double theta_max) {
  MALIDRIVE_THROW_UNLESS(-M_PI <= theta);
  MALIDRIVE_THROW_UNLESS(theta <= M_PI);
  MALIDRIVE_THROW_UNLESS(theta_min <= theta_max);

  if (theta_max >= theta_min + 2. * M_PI) return theta;

  // Wrap on the range [-π, π]. This is not order-preserving.
  const double theta_0 = wrap(theta_min);
  const double theta_1 = wrap(theta_max);

  // Criteria for returning the unsaturated result.
  // First, deal with the case where [θ_min, θ_max] wrapped does not cross the
  // ±π wrap-point (i.e. θ₀ ≤ θ₁).
  if (theta_0 <= theta && theta <= theta_1) return theta;
  // Next, deal with the case where [θ_min, θ_max] wrapped straddles ±π (i.e. θ₁
  // < θ₀).
  if (theta <= theta_1 && theta_1 < theta_0) return theta;
  if (theta_1 < theta_0 && theta_0 <= theta) return theta;

  // Saturate at the appropriate bound.
  const double delta_0 = std::min(std::abs(theta - theta_0), std::abs(theta - 2. * M_PI - theta_0));
  const double delta_1 = std::min(std::abs(theta - theta_1), std::abs(theta + 2. * M_PI - theta_1));
  return (delta_0 <= delta_1) ? theta_0 : theta_1;
}

}  // namespace

double ArcGroundCurve::DoGInverse(const maliput::math::Vector2& xy) const {
  // displacement vector from center of arc to xy
  const maliput::math::Vector2 center_to_xy = xy - center_;

  // throw if test point is to close to the center of curvature
  MALIDRIVE_THROW_UNLESS(center_to_xy.norm() >= linear_tolerance_);

  // compute theta angle
  const double theta = std::atan2(center_to_xy.y(), center_to_xy.x());
  const double theta_min = std::min(theta0_, d_theta_ + theta0_);
  const double theta_max = std::max(theta0_, d_theta_ + theta0_);

  // First, find a saturated theta that is nearest to point q.
  const double theta_nearest = saturate_on_wrapped_bounds(theta, theta_min, theta_max);
  // Find the angle swept from the beginning of the lane (p = p0_) to
  // theta_nearest.
  const double d_theta_nearest = (d_theta_ > 0.) ? theta_nearest - wrap(theta_min) : wrap(theta_max) - theta_nearest;
  // Then, unwrap this angle (if necessary) to deal with possible crossings with
  // the ±π wrap-around point.
  const double d_theta_nearest_unwrapped = (d_theta_nearest < 0.) ? d_theta_nearest + 2. * M_PI : d_theta_nearest;
  // Convert this angular displacement to arc length (s).
  const double scaled_theta_unsaturated = d_theta_nearest_unwrapped / std::abs(d_theta_);

  const double scaled_theta = maliput::math::saturate(scaled_theta_unsaturated, 0., 1.);
  return p0_ + (p1_ - p0_) * scaled_theta;
}

}  // namespace road_curve
}  // namespace malidrive
