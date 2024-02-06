#pragma once

#include <maliput/common/range_validator.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace odr {

class Spiral {
 public:
  Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end);
  maliput::math::Vector2 get_xy(double s) const;
  maliput::math::Vector2 get_grad(double s) const;
 private:
  double s0_ = 0;
  double x0_ = 0;
  double y0_ = 0;
  double hdg0_ = 0;
  double length_ = 0;
  double s0_spiral_ = 0;
  double x0_spiral_ = 0;
  double y0_spiral_ = 0;
  double a0_spiral_ = 0;
  double curv_start_ = 0;
  double curv_end_ = 0;
  double s_start_ = 0;
  double s_end_ = 0;
  double c_dot_ = 0;
    
};

}  // namespace odr

namespace malidrive {
namespace road_curve {

class OdrSpiralGroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(OdrSpiralGroundCurve);

  OdrSpiralGroundCurve() = delete;

  /// Constructs an SpiralGroundCurve.
  ///
  /// @param linear_tolerance A non-negative value expected to be the same as
  /// maliput::api::RoadGeometry::linear_tolerance().
  /// @param xy0 A 2D vector that represents the first point of the spiral.
  /// @param heading0 The orientation of the tangent vector at @p xy0.
  /// @param curvature0 Quantity which indicates the reciprocal of the
  ///        turning radius of the arc at @p p0. A positive @p curvature0 makes a
  ///        counterclockwise turn. It must be different from @p curvature1 by at least
  ///        GroundCurve::kEpsilon.
  /// @param curvature1 Quantity which indicates the reciprocal of the
  ///        turning radius of the arc at @p p1. A positive @p curvature1 makes a
  ///        counterclockwise turn. It must be different from @p curvature1 by at least
  ///        GroundCurve::kEpsilon.
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
  ///         different from @p curvature1 by less than GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p arc_length is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not sufficiently
  ///         larger than @p p0.
  OdrSpiralGroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double heading0, double curvature0,
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
  const odr::Spiral spiral_;

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
  maliput::math::Vector2 spiral_xy0_{};
};

}  // namespace road_curve
}  // namespace malidrive