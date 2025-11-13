#pragma once

#include <cmath>

#include <maliput/common/range_validator.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace malidrive {
namespace road_curve {

/// GroundCurve specification for a reference curve that describes a parametric cubic polynomial
/// in the ground plane following OpenDRIVE's paramPoly3 definition
/// See https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/09_geometries/09_06_param_poly3.html
///
/// The curve is defined by two cubic polynomials in local u,v coordinates:
///   u(p) = aU + bU*p + cU*p² + dU*p³
///   v(p) = aV + bV*p + cV*p² + dV*p³
///
/// These are transformed to global (x,y) coordinates using the initial position (xy0)
/// and heading (heading0).
///
/// Queries accept p ∈ [p0, p1] with a linear tolerance.
class ParamPoly3GroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(ParamPoly3GroundCurve);

  ParamPoly3GroundCurve() = delete;

  /// Specifies the range for the parameter p.
  enum class PRangeType {
    kArcLength,  ///< p is in [0, arc_length]
    kNormalized  ///< p is in [0, 1]
  };

  /// Constructs a ParamPoly3GroundCurve.
  ///
  /// @param linear_tolerance A non-negative value expected to be the same as
  /// maliput::api::RoadGeometry::linear_tolerance().
  /// @param xy0 A 2D vector that represents the first point of the curve (start position).
  /// @param heading0 The orientation of the tangent vector at @p xy0.
  /// @param aU Polynomial parameter a for u coordinate.
  /// @param bU Polynomial parameter b for u coordinate. Should be > 0 for proper alignment.
  /// @param cU Polynomial parameter c for u coordinate.
  /// @param dU Polynomial parameter d for u coordinate.
  /// @param aV Polynomial parameter a for v coordinate. Should be 0 for proper alignment.
  /// @param bV Polynomial parameter b for v coordinate. Should be 0 for proper alignment.
  /// @param cV Polynomial parameter c for v coordinate.
  /// @param dV Polynomial parameter d for v coordinate.
  /// @param arc_length The curve's length. It must be greater or equal to
  ///        GroundCurve::kEpsilon.
  /// @param p0 The value of the @f$ p @f$ parameter at the beginning of the
  ///        curve, which must be non-negative and smaller than @p p1 by at least
  ///        GroundCurve::kEpsilon.
  /// @param p1 The value of the @f$ p @f$ parameter at the end of the curve,
  ///        which must be greater than @p p0 by at least GroundCurve::kEpsilon.
  /// @param p_range Specifies the range type for parameter p.
  /// @throws maliput::common::road_geometry_construction_error When @p linear_tolerance is
  ///         non-positive.
  /// @throws maliput::common::road_geometry_construction_error When @p arc_length is smaller
  ///         than GroundCurve::kEpsilon.
  /// @throws maliput::common::road_geometry_construction_error When @p p0 is negative.
  /// @throws maliput::common::road_geometry_construction_error When @p p1 is not sufficiently
  ///         larger than @p p0.
  ParamPoly3GroundCurve(double linear_tolerance, const maliput::math::Vector2& xy0, double heading0, double aU,
                        double bU, double cU, double dU, double aV, double bV, double cV, double dV,
                        double arc_length, double p0, double p1, PRangeType p_range);

 private:
  // Computes the normalized parameter from the input parameter p.
  // For kArcLength: normalizes p from [p0, p1] to [0, arc_length]
  // For kNormalized: normalizes p from [p0, p1] to [0, 1]
  double NormalizedP(double p) const;

  // Evaluates the u polynomial at the normalized parameter.
  double U(double p_norm) const;

  // Evaluates the v polynomial at the normalized parameter.
  double V(double p_norm) const;

  // Evaluates the derivative of u with respect to the normalized parameter.
  double UDot(double p_norm) const;

  // Evaluates the derivative of v with respect to the normalized parameter.
  double VDot(double p_norm) const;

  // @{ NVI implementations.
  double DoPFromP(double xodr_p) const override { return validate_p_(xodr_p); }
  maliput::math::Vector2 DoG(double p) const override;
  maliput::math::Vector2 DoGDot(double p) const override;
  double DoHeading(double p) const override;
  double DoHeadingDot(double p) const override;
  double DoGInverse(const maliput::math::Vector2& xy) const override;
  double DoArcLength() const override { return arc_length_; }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }
  // @} NVI implementations.

  // The linear tolerance.
  const double linear_tolerance_{};
  // The first point of the curve in world coordinates.
  const maliput::math::Vector2 xy0_{};
  // The heading at the start of the curve.
  const double heading0_{};
  // Polynomial coefficients for u(p).
  const double aU_{};
  const double bU_{};
  const double cU_{};
  const double dU_{};
  // Polynomial coefficients for v(p).
  const double aV_{};
  const double bV_{};
  const double cV_{};
  const double dV_{};
  // The length of the curve.
  const double arc_length_{};
  // The value of the p parameter at the start of the curve.
  const double p0_{};
  // The value of the p parameter at the end of the curve.
  const double p1_{};
  // The parameter range type.
  const PRangeType p_range_{};
  // Precomputed values for coordinate transformation.
  const double cos_heading0_{std::cos(heading0_)};
  const double sin_heading0_{std::sin(heading0_)};
  // Validates that p is within [p0, p1] with linear_tolerance.
  const maliput::common::RangeValidator<maliput::common::road_geometry_construction_error> validate_p_;
};

} // namespace road_curve
}  // namespace malidrive
