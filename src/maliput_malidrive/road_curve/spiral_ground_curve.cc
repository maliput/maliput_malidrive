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
#include <cmath>

#include <maliput/math/fresnel.h>

namespace malidrive {
namespace road_curve {
namespace {

// Rotates @p v by @p angle.
// @param v Input 2D vector to rotate.
// @param angle Input angle to rotate.
// @return The rotated vector.
inline maliput::math::Vector2 RotateVector(const maliput::math::Vector2& v, double angle) {
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);
  return maliput::math::Vector2{ca * v.x() - sa * v.y(), sa * v.x() + ca * v.y()};
}

// Conditionally shifts the y-component of @p v when @p cond is true.
// @param v Input vector.
// @param cond Conditional flag.
// @return When @p cond is true, returns `[v.x(), -v.y()]`, otherwise @p v.
inline maliput::math::Vector2 ConditionalShiftYComponent(const maliput::math::Vector2& v, bool cond) {
  return cond ? maliput::math::Vector2{v.x(), -v.y()} : v;
}

// Finds the best p parameter for @p curve that maps a point in the 2D INERTIAL-Frame that minimizes
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
  MALIDRIVE_THROW_UNLESS(p0 >= 0., maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(p1 - p0 >= GroundCurve::kEpsilon, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(partitions >= 2u, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(tolerance >= 0., maliput::common::road_geometry_construction_error);

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

SpiralGroundCurve::SpiralGroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double heading0,
                                     double curvature0, double curvature1, double arc_length, double p0, double p1)
    : linear_tolerance_(linear_tolerance),
      xy0_(xy0),
      heading0_(heading0),
      curvature0_(curvature0),
      curvature1_(curvature1),
      arc_length_(arc_length),
      k_dot_((curvature1 - curvature0) / arc_length),
      norm_(std::sqrt(M_PI / std::abs(k_dot_))),
      t0_(curvature0 / (k_dot_ * norm_)),
      xy0_spiral_(ConditionalShiftYComponent(maliput::math::ComputeFresnelCosineAndSine(t0_) * norm_, k_dot_ < 0.)),
      heading0_spiral_(maliput::math::FresnelSpiralHeading(t0_ * norm_, k_dot_)),
      p0_(p0),
      p1_(p1),
      validate_p_(maliput::common::RangeValidator<maliput::common::road_geometry_construction_error>::
                      GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_, GroundCurve::kEpsilon)) {
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(arc_length_ >= GroundCurve::kEpsilon, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(p0_ >= 0., maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(p1_ - p0_ >= GroundCurve::kEpsilon, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(std::abs(curvature1_ - curvature0_) >= GroundCurve::kEpsilon,
                         maliput::common::road_geometry_construction_error);
}

maliput::math::Vector2 SpiralGroundCurve::DoG(double p) const {
  p = validate_p_(p);
  const double t = TFromP(p);
  const maliput::math::Vector2 xy =
      ConditionalShiftYComponent(maliput::math::ComputeFresnelCosineAndSine(t) * norm_, k_dot_ < 0.) - xy0_spiral_;
  const double heading = heading0_ - heading0_spiral_;
  return RotateVector(xy, heading) + xy0_;
}

maliput::math::Vector2 SpiralGroundCurve::DoGDot(double p) const {
  const double heading = Heading(p);
  return maliput::math::Vector2{std::cos(heading), std::sin(heading)};
}

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
  p = validate_p_(p);
  const double t = TFromP(p);
  const double heading_fresnel = maliput::math::FresnelSpiralHeading(t * norm_, k_dot_);
  return heading_fresnel - heading0_spiral_ + heading0_;
}

double SpiralGroundCurve::DoHeadingDot(double p) const {
  p = validate_p_(p);
  const double t = TFromP(p);
  return maliput::math::FresnelSpiralHeadingDot(t * norm_, k_dot_);
}

}  // namespace road_curve
}  // namespace malidrive