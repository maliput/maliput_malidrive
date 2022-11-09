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

/// GroundCurve specification for a reference curve that describes a line.
///
/// Queries accept p ∈ [p0, p1] with a linear tolerance.
class LineGroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(LineGroundCurve);

  LineGroundCurve() = delete;

  /// Constructor. Computes a line from @p xy0 as the initial point of the line
  /// and @p dxy as the difference vector that connects the @p xy0 with the end
  /// point of the reference curve.
  /// @param linear_tolerance A non-negative value expected to be the same as
  /// maliput::api::RoadGeometry::linear_tolerance().
  /// @param xy0 A 2D vector that represents the first point of the line.
  /// @param dxy A 2D difference vector between the last point and @p xy0. The norm of this vector
  /// must be larger than GroundCurve::kEpsilon to avoid having issues when computing the heading.
  /// @param p0 The value of the @f$ p @f$ paramater at the start of the line, which must be non-negative.
  /// @param p1 The value of the @f$ p @f$ paramater at the end of the line, which must be greater than
  /// the value of @p p0_ by at least GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p linear_tolerance is non-positive.
  /// @throws maliput::common::assertion_error When the norm of @p dxy is too small.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not sufficiently larger than @p p0.
  LineGroundCurve(const double linear_tolerance, const maliput::math::Vector2& xy0, const maliput::math::Vector2& dxy,
                  double p0, double p1)
      : linear_tolerance_(linear_tolerance),
        xy0_(xy0),
        dxy_(dxy),
        arc_length_(dxy.norm()),
        heading_(std::atan2(dxy.y(), dxy.x())),
        p0_(p0),
        p1_(p1),
        validate_p_(maliput::common::RangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_,
                                                                                 GroundCurve::kEpsilon)) {
    MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0);
    MALIDRIVE_THROW_UNLESS(arc_length_ >= GroundCurve::kEpsilon);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
    MALIDRIVE_THROW_UNLESS(p1_ - p0_ >= GroundCurve::kEpsilon);
  }

 private:
  double DoPFromP(double xodr_p) const override { return validate_p_(xodr_p); };

  maliput::math::Vector2 DoG(double p) const override;
  maliput::math::Vector2 DoGDot(double p) const override;
  double DoGInverse(const maliput::math::Vector2&) const override;

  double DoHeading(double p) const override {
    validate_p_(p);
    return heading_;
  }

  double DoHeadingDot(double p) const override {
    validate_p_(p);
    return 0.;
  }

  double DoArcLength() const override { return arc_length_; }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }

  // The linear tolerance used to avoid having issues when computing heading_.
  const double linear_tolerance_{};
  // The first point of the line in world coordinates.
  const maliput::math::Vector2 xy0_{};
  // The difference vector that joins the end point of the line with the first one, xy0_.
  const maliput::math::Vector2 dxy_{};
  // The length of the line, computed as the norm of dxy_.
  const double arc_length_{};
  // The constant angle deviation of dxy_ with respect to the x axis of the world frame.
  const double heading_{};
  // The value of the p parameter at the start of the line.
  const double p0_{};
  // The value of the p parameter at the end of the line.
  const double p1_{};
  // Validates that p is within [p0, p1] with linear_tolerance.
  const maliput::common::RangeValidator validate_p_;
};

}  // namespace road_curve
}  // namespace malidrive
