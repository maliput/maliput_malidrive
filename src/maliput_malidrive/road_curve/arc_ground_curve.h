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
#pragma once

#include <cmath>

#include <maliput/common/range_validator.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace malidrive {
namespace road_curve {

/// GroundCurve specification for a reference curve that describes a constant
/// curvature arc.
///
/// Queries accept p ∈ [p0, p1] with a linear tolerance.
class ArcGroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(ArcGroundCurve);

  ArcGroundCurve() = delete;

  /// Constructs an ArcGroundCurve.
  ///
  /// @param linear_tolerance A non-negative value expected to be the same as
  /// maliput::api::RoadGeometry::linear_tolerance().
  /// @param xy0 A 2D vector that represents the first point of the arc.
  /// @param start_heading The orientation of the tangent vector at @p xy0.
  /// @param curvature Non-zero quantity which indicates the reciprocal of the
  ///        turning radius of the arc. A positive @p curvature makes a
  ///        counterclockwise turn. Its magnitude must be greater than
  ///        GroundCurve::kEpsilon.
  /// @param arc_length The arc's length. It must be greater or equal to
  ///        GroundCurve::kEpsilon.
  /// @param p0 The value of the @f$ p @f$ parameter at the beginning of the
  ///        arc, which must be non negative and smaller than @p p1 by at least
  ///        GroundCurve::kEpsilon.
  /// @param p1 The value of the @f$ p @f$ parameter at the end of the arc,
  ///        which must be greater than @p p0 by at least GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p linear_tolerance is
  ///         non-positive.
  /// @throws maliput::common::assertion_error When @p arc_length is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not sufficiently
  ///         larger than @p p0.
  ArcGroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double start_heading, double curvature,
                 double arc_length, double p0, double p1)
      : linear_tolerance_(linear_tolerance),
        xy0_(xy0),
        arc_length_(arc_length),
        p0_(p0),
        p1_(p1),
        radius_(1. / curvature),
        d_theta_(arc_length * curvature),
        theta0_(start_heading - std::copysign(M_PI / 2., d_theta_)),
        center_(xy0_ - std::abs(radius_) * maliput::math::Vector2{std::cos(theta0_), std::sin(theta0_)}),
        validate_p_(maliput::common::RangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_,
                                                                                 GroundCurve::kEpsilon)) {
    MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0);
    MALIDRIVE_THROW_UNLESS(arc_length_ >= GroundCurve::kEpsilon);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
    MALIDRIVE_THROW_UNLESS(p1_ - p0_ >= GroundCurve::kEpsilon);
    MALIDRIVE_THROW_UNLESS(std::abs(curvature) >= GroundCurve::kEpsilon);
  }

 private:
  double DoTheta(double p) const {
    p = validate_p_(p);
    return theta0_ + (p - p0_) * DoHeadingDot(p);
  }

  double DoPFromP(double xodr_p) const override { return validate_p_(xodr_p); }

  maliput::math::Vector2 DoG(double p) const override {
    p = validate_p_(p);
    const double theta{DoTheta(p)};
    return center_ + std::abs(radius_) * maliput::math::Vector2{std::cos(theta), std::sin(theta)};
  }

  maliput::math::Vector2 DoGDot(double p) const override {
    p = validate_p_(p);
    const double theta{DoTheta(p)};
    return std::copysign(arc_length_ / (p1_ - p0_), d_theta_) *
           maliput::math::Vector2{-std::sin(theta), +std::cos(theta)};
    // for reference, GDot can be computed using the heading angle instead of theta as follows:
    // const double heading{DoHeading(p)};
    // return arc_length_ / (p1_ - p0_) * maliput::math::Vector2{std::cos(heading), std::sin(heading)};
  }

  double DoGInverse(const maliput::math::Vector2&) const override;

  double DoHeading(double p) const override {
    p = validate_p_(p);
    return DoTheta(p) + std::copysign(M_PI / 2., d_theta_);
  }

  double DoHeadingDot(double p) const override {
    validate_p_(p);
    return d_theta_ / (p1_ - p0_);
  }

  double DoArcLength() const override { return arc_length_; }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }

  // The linear tolerance. It is used to distinguish between a line and an arc.
  const double linear_tolerance_{};
  // The first point of the arc in world coordinates.
  const maliput::math::Vector2 xy0_{};
  // The length of the arc.
  const double arc_length_{};
  // The value of the p parameter at the start of the arc.
  const double p0_{};
  // The value of the p parameter at the end of the arc.
  const double p1_{};
  // The length of the radius of the arc.
  const double radius_{};
  // The aperture angle of the arc. Positive values are counter-clockwise.
  const double d_theta_{};
  // The angular position at which the arc starts with respect to `center_`
  // (0 == parallel to x-axis). This angle differs from the heading by ±90 degrees
  // the the sign depending on the sign of the curvature.
  const double theta0_{};
  // Center of rotation in z=0 plane, world coordinates, for the arc reference
  // curve.
  const maliput::math::Vector2 center_{};
  // Validates that p is within [p0, p1] with linear_tolerance.
  const maliput::common::RangeValidator validate_p_;
};

}  // namespace road_curve
}  // namespace malidrive
