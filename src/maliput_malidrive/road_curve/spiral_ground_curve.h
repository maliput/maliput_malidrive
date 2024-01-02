// BSD 3-Clause License
//
// Copyright (c) 2024, Woven Planet.
// All rights reserved.
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

#include <maliput/common/range_validator.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace malidrive {
namespace road_curve {

/// GroundCurve specification for a reference curve that describes a spiral geometry.
/// The spiral is defined by a start curvature and an end curvature that evolve linearly with its length.
///
/// Look at http://en.wikipedia.org/wiki/Euler_spiral for more details.
///
/// Queries accept p ∈ [p0, p1] with a linear tolerance.
class SpiralGroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(SpiralGroundCurve);

  SpiralGroundCurve() = delete;

  /// Creates a SpiralGroundCurve.
  /// @param linear_tolerance The linear tolerance.
  /// @param xy0 The first point of the spiral in world coordinates.
  /// @param start_heading The heading at the start of the spiral.
  /// @param start_curvature The curvature at the start of the spiral.
  /// @param end_curvature The curvature at the end of the spiral.
  /// @param arc_length The length of the spiral.
  /// @param p0 The value of the p parameter at the start of the spiral.
  /// @param p1 The value of the p parameter at the end of the spiral.
  /// @throw maliput::common::assertion_error When @p linear_tolerance is not positive.
  /// @throw maliput::common::assertion_error When @p arc_length is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throw maliput::common::assertion_error When @p p0 is negative.
  /// @throw maliput::common::assertion_error When @p p1 is less than @p p0 .
  /// @throw maliput::common::assertion_error When @p start_heading is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throw maliput::common::assertion_error When @p start_curvature is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throw maliput::common::assertion_error When @p end_curvature is smaller
  ///         than GroundCurve::kEpsilon.
  SpiralGroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double start_heading,
                    double start_curvature, double end_curvature, double arc_length, double p0, double p1)
      : linear_tolerance_(linear_tolerance),
        xy0_(xy0),
        start_heading_(start_heading),
        start_curvature_(start_curvature),
        end_curvature_(end_curvature),
        arc_length_(arc_length),
        p0_(p0),
        p1_(p1),
        validate_p_(maliput::common::RangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_,
                                                                                 GroundCurve::kEpsilon)) {
    MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0);
    MALIDRIVE_THROW_UNLESS(arc_length_ >= GroundCurve::kEpsilon);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
    MALIDRIVE_THROW_UNLESS(p1_ - p0_ >= GroundCurve::kEpsilon);
    MALIDRIVE_THROW_UNLESS(std::abs(start_heading) >= GroundCurve::kEpsilon);
    MALIDRIVE_THROW_UNLESS(std::abs(start_curvature) >= GroundCurve::kEpsilon);
    MALIDRIVE_THROW_UNLESS(std::abs(end_curvature) >= GroundCurve::kEpsilon);
  }

 private:
  double DoPFromP(double xodr_p) const override;

  maliput::math::Vector2 DoG(double p) const override {
    // Not implemented.
    MALIPUT_THROW_UNLESS(false);
  }

  maliput::math::Vector2 DoGDot(double p) const override {
    // Not implemented.
    MALIPUT_THROW_UNLESS(false);
  }

  double DoGInverse(const maliput::math::Vector2&) const override {
    // Not implemented.
    MALIPUT_THROW_UNLESS(false);
  }

  double DoHeading(double p) const override {
    // Not implemented.
    MALIPUT_THROW_UNLESS(false);
  }

  double DoHeadingDot(double p) const override {
    // Not implemented.
    MALIPUT_THROW_UNLESS(false);
  }

  double DoArcLength() const override { return arc_length_; }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }

  // The linear tolerance.
  const double linear_tolerance_{};
  // The first point of the spiral in world coordinates.
  const maliput::math::Vector2 xy0_{};
  // Start heading.
  const double start_heading_{};
  // Start curvature.
  const double start_curvature_{};
  // End curvature.
  const double end_curvature_{};
  // The length of the spiral.
  const double arc_length_{};
  // The value of the p parameter at the start of the spiral.
  const double p0_{};
  // The value of the p parameter at the end of the spiral.
  const double p1_{};
  // Validates that p is within [p0, p1] with linear_tolerance.
  const maliput::common::RangeValidator validate_p_;
};

}  // namespace road_curve
}  // namespace malidrive
