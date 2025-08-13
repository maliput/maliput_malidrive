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
#pragma once

#include <cmath>

#include <maliput/common/range_validator.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace malidrive {
namespace road_curve {

/// GroundCurve specification for a reference curve that describes a clothoid or
/// Euler spiral.
///
/// Queries accept p ∈ [p0, p1] with a linear tolerance.
class SpiralGroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(SpiralGroundCurve);

  SpiralGroundCurve() = delete;

  /// Constructs an SpiralGroundCurve.
  ///
  /// @param linear_tolerance A non-negative value expected to be the same as
  /// maliput::api::RoadGeometry::linear_tolerance().
  /// @param xy0 A 2D vector that represents the first point of the spiral.
  /// @param heading0 The orientation of the tangent vector at @p xy0.
  /// @param curvature0 The curvature of the spiral at @p p0. The absolute difference
  ///        of the curvatures at @p p1 and @p p0 must be greater than GroundCurve::kEpsilon.
  /// @param curvature1 The curvature of the spiral at @p p1. The absolute difference
  ///        of the curvatures at @p p1 and @p p0 must be greater than GroundCurve::kEpsilon.
  /// @param arc_length The spiral's length. It must be greater or equal to
  ///        GroundCurve::kEpsilon.
  /// @param p0 The value of the @f$ p @f$ parameter at the beginning of the
  ///        spiral, which must be non negative and smaller than @p p1 by at least
  ///        GroundCurve::kEpsilon.
  /// @param p1 The value of the @f$ p @f$ parameter at the end of the spiral,
  ///        which must be greater than @p p0 by at least GroundCurve::kEpsilon.
  /// @throws maliput::common::road_geometry_construction_error When @p linear_tolerance is
  ///         non-positive.
  /// @throws maliput::common::road_geometry_construction_error When @p arc_length is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throws maliput::common::road_geometry_construction_error When @p p0 is negative.
  /// @throws maliput::common::road_geometry_construction_error When @p p1 is not sufficiently
  ///         larger than @p p0.
  // TODO(Santoi): Even so this method should throw a maliput::common::road_geometry_construction_error it
  // actually throws a maliput::common::assertion_error coming from the RangeValidator called in the
  // initialization list. After solving https://github.com/maliput/maliput/issues/666, we can make sure that
  // it throws what we want it to throw.
  SpiralGroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double heading0, double curvature0,
                    double curvature1, double arc_length, double p0, double p1);

 private:
  // @{ NVI implementations.
  double DoPFromP(double xodr_p) const override { return validate_p_(xodr_p); }
  double DoArcLength() const override { return arc_length_; }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }
  maliput::math::Vector2 DoG(double p) const override;
  maliput::math::Vector2 DoGDot(double p) const override;
  double DoGInverse(const maliput::math::Vector2&) const override;
  double DoHeading(double p) const override;
  double DoHeadingDot(double p) const override;
  // @} NVI implementations.

  // Converts the @p p value into a spiral t coordinate.
  // Offsets @p p by `p0_`, scales the difference by `norm_` and then offsets it by `t0_`.
  // @param p The parameter of the GroundCurve.
  // @return The normalized parameter of the spiral.
  inline double TFromP(double p) const { return (p - p0_) / norm_ + t0_; }

  // The linear tolerance.
  const double linear_tolerance_{};
  // The first point of the spiral in the Inertial Frame coordinates.
  const maliput::math::Vector2 xy0_{};
  // The heading at `p0_`.
  const double heading0_{};
  // The curvature of the spiral at `p0_`.
  const double curvature0_{};
  // The curvature of the spiral at `p1_`.
  const double curvature1_{};
  // The length of the spiral.
  const double arc_length_{};
  // The derivative of the spiral.
  const double k_dot_{};
  // The normalization factor that is used to scale p and the spiral parameter.
  const double norm_{};
  // The value that the normalized spiral parameter takes at `p0_`.
  const double t0_{};
  // The position of the spiral at `p0_` before applying any isometry to respect the initial pose (`heading0_` and
  // `xy0_`).
  const maliput::math::Vector2 xy0_spiral_{};
  // The heading of the spiral at `p0_` before applying any isometry to respect the initial pose (`heading0_` and
  // `xy0_`).
  const double heading0_spiral_{};
  // The value of the p parameter at the start of the spiral.
  const double p0_{};
  // The value of the p parameter at the end of the spiral.
  const double p1_{};
  // Validates that p is within [p0, p1] with linear_tolerance.
  const maliput::common::RangeValidator validate_p_;
};

}  // namespace road_curve
}  // namespace malidrive
