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

#include <maliput/common/range_validator.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace malidrive {
namespace road_curve {

/// GroundCurve specification for a reference curve that describes an Euler spiral, i.e. a clothoid.
///
/// Queries accept @f$ p ∈ [p0, p1] @f$ with a linear tolerance.
///
/// Implementation details: it relies on an approximate power series expansion of the Fresnel
/// sine and cosine integrals using 10 terms. The series is known to be sensitive to numerical error
/// when using floating point numbers due to the high range values. Consequently, it is necessary to
/// rely on function parameter normalization (@f$ p @f$ parameter) to reduce the range. It occurs
/// at construction time and it is used throughout the API. Start and end curvatures are both used to define
/// the normalization factor. The @f$ p @f$ parameter is used in the public API of this class and it is
/// converted into the normalized parameter @f$ t @f$ by means of a linear function.
///
/// For mathematical details about the implementation please refer to:
/// - https://en.wikipedia.org/wiki/Euler_spiral
/// - https://en.wikipedia.org/wiki/Fresnel_integral
///
/// Another reference implementation can be found at:
///
/// <a href="https://github.com/pageldev/libOpenDRIVE/tree/master">libOpenDRIVE</a>
/// <a href="https://github.com/pageldev/libOpenDRIVE/blob/master/LICENSE">Apache License 2.0</a>
/// <a
/// href="https://github.com/pageldev/libOpenDRIVE/blob/9a0437f8a18d445d5c43fe2a4c9401d8a4b770f0/src/Geometries/Spiral.cpp">Spiral.cpp</a>
///
/// This implementation does not admit different signs in the start and end curvatures.
// TODO(#265): Evaluate start and end curvature being of different sign.
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
  /// @param curvature0 Quantity which indicates the reciprocal of the
  ///        turning radius of the arc at @p p0. A positive @p curvature0 makes a
  ///        counterclockwise turn. It must be different from @p curvature1 by at least
  ///        GroundCurve::kEpsilon. It must be of the same sign as @p curvature1.
  /// @param curvature1 Quantity which indicates the reciprocal of the
  ///        turning radius of the arc at @p p1. A positive @p curvature1 makes a
  ///        counterclockwise turn. It must be different from @p curvature1 by at least
  ///        GroundCurve::kEpsilon. It must be of the same sign as @p curvature0.
  /// @param arc_length The spiral's length. It must be greate-r or equal to
  ///        GroundCurve::kEpsilon.
  /// @param p0 The value of the @f$ p @f$ parameter at the beginning of the
  ///        spiral, which must be non negative and smaller than @p p1 by at least
  ///        GroundCurve::kEpsilon.
  /// @param p1 The value of the @f$ p @f$ parameter at the end of the spiral,
  ///        which must be greater than @p p0 by at least GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p linear_tolerance is
  ///         non-positive.
  /// @throws maliput::common::assertion_error When @p curvature0 is
  ///         different from @p curvature1 by less than  GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p curvature0 and @p curvature1
  ///         have a different sign.
  /// @throws maliput::common::assertion_error When @p arc_length is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not sufficiently
  ///         larger than @p p0.
  SpiralGroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double heading0, double curvature0,
                    double curvature1, double arc_length, double p0, double p1);

 private:
  // @{ NVI implementations.
  double DoPFromP(double xodr_p) const override;
  maliput::math::Vector2 DoG(double p) const;
  maliput::math::Vector2 DoGDot(double p) const override;
  double DoGInverse(const maliput::math::Vector2& point) const override;
  double DoHeading(double p) const override;
  double DoHeadingDot(double p) const override;
  double DoArcLength() const override { return arc_length_; }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }
  // @} NVI implementations.

  // Computes the normalized spiral function parameter at @p p.
  // @param p The non-normalized parameter of the spiral.
  // @return The normalized spiral parameter @f$ t @f$.
  double NormalizedSpiralCoordinateAt(double p) const;

  // Linear tolerance.
  const double linear_tolerance_{};
  // First point of the spiral in world coordinates.
  const maliput::math::Vector2 xy0_{};
  // Heading at `p0_`.
  const double heading0_{};
  // Curvature at `p0_`.
  const double curvature0_{};
  // Curvature at `p1_`.
  const double curvature1_{};
  // Arc length of the spiral.
  const double arc_length_{};
  // Value of the p parameter at the start of the spiral.
  const double p0_{};
  // Value of the p parameter at the end of the spiral.
  const double p1_{};
  // Validates that p is within [`p0`, `p1`] with `linear_tolerance_`.
  const maliput::common::RangeValidator validate_p_;
  // Curvature derivative at `p0_`.
  const double k_dot_{};
  // Normalization factor.
  const double norm_{};
  // Non-normalized spiral function parameter at `p0_`.
  const double spiral_p0_{};
  // Heading of the normalized spiral at `p0_`.
  const double spiral_heading0_{};
  // Start position of the spiral in the normalized spiral frame.
  const maliput::math::Vector2 spiral_xy0_{};
};

}  // namespace road_curve
}  // namespace malidrive
