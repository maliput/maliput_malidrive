// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/road_curve_factory.h"

#include <cmath>
#include <type_traits>
#include <utility>

#include "maliput_malidrive/road_curve/arc_ground_curve.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/piecewise_function.h"
#include "maliput_malidrive/road_curve/piecewise_ground_curve.h"

#include <maliput/math/vector.h>

namespace malidrive {
namespace builder {
namespace {

// Get coefficients of a translated cubic polynomial:
// @f$ f(p-p0) = a + b (p-p0) + c (p-p0)^2 + d (p-p0)^3 + / p ∈ [`p0`; `p1`] @f$.
//
// @param a The coefficient of independent monomial.
// @param b The coefficient of first degree monomial.
// @param c The coefficient of second degree monomial.
// @param d The coefficient of third degree monomial.
// @param p0 The value to translate the polynomial.
//
// @returns An array with the values {α,β,γ,ε} of @f$ f(p) = α + β p + γ p^2 + ε p^3    / p ∈ [`p0`; `p1`] @f$.
std::array<double, 4> TranslateCubic(double a, double b, double c, double d, double p0) {
  const double alpha = a - b * p0 + p0 * p0 * c - d * p0 * p0 * p0;
  const double beta = b + 3 * d * p0 * p0 - 2 * c * p0;
  const double gamma = c - 3 * d * p0;
  const double epsilon = d;
  return {alpha, beta, gamma, epsilon};
}

// Returns the type of `T` as long as it is one of:
// - xodr::ElevationProfile::Elevation
// - xodr::LateralProfile::Superelevation
// - xodr::LaneOffset
// It throws otherwise.
template <typename T>
constexpr const char* TypeName() {
  if (std::is_same<T, xodr::ElevationProfile::Elevation>::value) {
    return "Elevation";
  } else if (std::is_same<T, xodr::LateralProfile::Superelevation>::value) {
    return "Superelevation";
  } else if (std::is_same<T, xodr::LaneOffset>::value) {
    return "LaneOffset";
  } else {
    // MALIDRIVE_THROW_MESSAGE macro can't be used within a constexpr function.
    throw maliput::common::assertion_error(
        "Only xodr::ElevationProfile::Elevation, xodr::LateralProfile::Superelevation and xodr::LaneOffset types are "
        "allowed by this function. ");
  }
}

}  // namespace

using maliput::math::Vector2;

std::unique_ptr<road_curve::Function> RoadCurveFactory::MakeCubicPolynomial(double a, double b, double c, double d,
                                                                            double p0, double p1) const {
  return std::make_unique<road_curve::CubicPolynomial>(a, b, c, d, p0, p1, linear_tolerance());
}

// System could be solved using the following Python snippet:
//
// @code{python}
//
// import sympy as s
// from sympy.solvers.solveset import linsolve
//
// p,A,B,C,D,p0,p1,y,dy = s.symbols('p, A, B, C, D, p0, p1, y, dy')
// # Declare cubic polynomial f and f_dot.
// f = A*p**3 + B*p**2 + C*p + D
// f_dot = s.diff(f,p)
// # Solve the system of equations.
// print(linsolve([f.subs(p,p0), f.subs(p,p1) - y, f_dot.subs(p, p0), f_dot.subs(p, p1) - dy], (A, B, C, D)))
//
// @endcode
std::unique_ptr<road_curve::Function> RoadCurveFactory::MakeCubicPolynomial(double p0, double p1, double y,
                                                                            double dy) const {
  MALIDRIVE_THROW_UNLESS(p0 >= 0);
  MALIDRIVE_THROW_UNLESS(p1 > p0);
  const double cubic_p0 = p0 * p0 * p0;
  const double quad_p0 = p0 * p0;
  const double cubic_p1 = p1 * p1 * p1;
  const double quad_p1 = p1 * p1;

  const double a = (p0 * dy - p1 * dy + 2 * y) / (cubic_p0 - 3 * quad_p0 * p1 + 3 * p0 * quad_p1 - cubic_p1);
  const double b = (-2 * quad_p0 * dy + p0 * p1 * dy - 3 * p0 * y + quad_p1 * dy - 3 * p1 * y) /
                   (cubic_p0 - 3 * quad_p0 * p1 + 3 * p0 * quad_p1 - cubic_p1);
  const double c = p0 * (quad_p0 * dy + p0 * p1 * dy - 2 * quad_p1 * dy + 6 * p1 * y) /
                   (cubic_p0 - 3 * quad_p0 * p1 + 3 * p0 * quad_p1 - cubic_p1);
  const double d = quad_p0 * (-p0 * p1 * dy + p0 * y + quad_p1 * dy - 3 * p1 * y) /
                   (cubic_p0 - 3 * quad_p0 * p1 + 3 * p0 * quad_p1 - cubic_p1);

  return MakeCubicPolynomial(a, b, c, d, p0, p1);
}

std::unique_ptr<road_curve::GroundCurve> RoadCurveFactory::MakeArcGroundCurve(
    const xodr::Geometry& arc_geometry) const {
  MALIDRIVE_THROW_UNLESS(arc_geometry.type == xodr::Geometry::Type::kArc);

  return std::make_unique<road_curve::ArcGroundCurve>(
      linear_tolerance(), arc_geometry.start_point, arc_geometry.orientation,
      std::get<xodr::Geometry::Arc>(arc_geometry.description).curvature, arc_geometry.length, arc_geometry.s_0,
      arc_geometry.s_0 + arc_geometry.length);
}

std::unique_ptr<road_curve::GroundCurve> RoadCurveFactory::MakeLineGroundCurve(
    const xodr::Geometry& line_geometry) const {
  MALIDRIVE_THROW_UNLESS(line_geometry.type == xodr::Geometry::Type::kLine);

  return std::make_unique<road_curve::LineGroundCurve>(
      linear_tolerance(), line_geometry.start_point,
      line_geometry.length * Vector2(std::cos(line_geometry.orientation), std::sin(line_geometry.orientation)),
      line_geometry.s_0, line_geometry.s_0 + line_geometry.length);
}

std::unique_ptr<road_curve::GroundCurve> RoadCurveFactory::MakePiecewiseGroundCurve(
    const std::vector<xodr::Geometry>& geometries) const {
  MALIDRIVE_THROW_UNLESS(!geometries.empty());

  std::vector<std::unique_ptr<road_curve::GroundCurve>> ground_curves;
  for (const xodr::Geometry& geometry : geometries) {
    switch (geometry.type) {
      case xodr::Geometry::Type::kArc:
        ground_curves.emplace_back(MakeArcGroundCurve(geometry));
        break;
      case xodr::Geometry::Type::kLine:
        ground_curves.emplace_back(MakeLineGroundCurve(geometry));
        break;
      default:
        MALIDRIVE_THROW_MESSAGE("Geometries contain a xodr::Geometry whose type is not in {kLine, kArc}.");
        break;
    }
  }
  return std::make_unique<road_curve::PiecewiseGroundCurve>(std::move(ground_curves), linear_tolerance(),
                                                            angular_tolerance());
}

std::unique_ptr<malidrive::road_curve::Function> RoadCurveFactory::MakeElevation(
    const xodr::ElevationProfile& elevation_profile, double p0, double p1) const {
  MALIDRIVE_THROW_UNLESS(p0 >= 0.);
  MALIDRIVE_THROW_UNLESS(p1 > p0);
  return MakeCubicFromXodr<xodr::ElevationProfile::Elevation>(elevation_profile.elevations, p0, p1,
                                                              FillingGapPolicy::kEnsureContiguity);
}

std::unique_ptr<malidrive::road_curve::Function> RoadCurveFactory::MakeSuperelevation(
    const xodr::LateralProfile& lateral_profile, double p0, double p1) const {
  MALIDRIVE_THROW_UNLESS(p0 >= 0.);
  MALIDRIVE_THROW_UNLESS(p1 > p0);
  return MakeCubicFromXodr<xodr::LateralProfile::Superelevation>(lateral_profile.superelevations, p0, p1,
                                                                 FillingGapPolicy::kEnsureContiguity);
}

std::unique_ptr<malidrive::road_curve::Function> RoadCurveFactory::MakeLaneWidth(
    const std::vector<xodr::LaneWidth>& lane_widths, double p0, double p1) const {
  MALIDRIVE_THROW_UNLESS(p0 >= 0.);
  MALIDRIVE_THROW_UNLESS(p1 > p0);
  const int num_polynomials = static_cast<int>(lane_widths.size());
  MALIDRIVE_THROW_UNLESS(num_polynomials > 0);
  MALIDRIVE_THROW_UNLESS(lane_widths[0].offset == 0);
  std::vector<std::unique_ptr<road_curve::Function>> polynomials;
  for (int i = 0; i < num_polynomials; i++) {
    // Last polynomial's range will be delitimed by p1 instead of next (non-existent) polynomial's offset.
    const bool end{i == num_polynomials - 1};
    const auto coeffs = TranslateCubic(lane_widths[i].a, lane_widths[i].b, lane_widths[i].c, lane_widths[i].d,
                                       lane_widths[i].offset + p0);
    const double p0_i{lane_widths[i].offset + p0};
    const double p1_i{end ? p1 : lane_widths[i + 1].offset + p0};
    if (std::abs(p1_i - p0_i) < road_curve::GroundCurve::kEpsilon) {
      if (!end) {
        MALIDRIVE_THROW_MESSAGE("Functions in a laneWidth description can't share same start point.");
      } else {
        MALIDRIVE_THROW_MESSAGE("Functions that compound laneWidth must be larger than a epsilon: " +
                                std::to_string(road_curve::GroundCurve::kEpsilon));
      }
    }
    polynomials.emplace_back(MakeCubicPolynomial(coeffs[3], coeffs[2], coeffs[1], coeffs[0], p0_i, p1_i));
  }
  return std::make_unique<road_curve::PiecewiseFunction>(std::move(polynomials), linear_tolerance());
}

std::unique_ptr<malidrive::road_curve::Function> RoadCurveFactory::MakeReferenceLineOffset(
    const std::vector<xodr::LaneOffset>& reference_offsets, double p0, double p1) const {
  MALIDRIVE_THROW_UNLESS(p0 >= 0.);
  MALIDRIVE_THROW_UNLESS(p1 > p0);
  return MakeCubicFromXodr<xodr::LaneOffset>(reference_offsets, p0, p1, FillingGapPolicy::kZero);
}

std::unique_ptr<road_curve::RoadCurve> RoadCurveFactory::MakeMalidriveRoadCurve(
    std::unique_ptr<road_curve::GroundCurve> ground_curve, std::unique_ptr<road_curve::Function> elevation,
    std::unique_ptr<road_curve::Function> superelevation) const {
  return std::make_unique<road_curve::RoadCurve>(linear_tolerance(), scale_length(), std::move(ground_curve),
                                                 std::move(elevation), std::move(superelevation));
}

template <class T>
std::unique_ptr<malidrive::road_curve::Function> RoadCurveFactory::MakeCubicFromXodr(const std::vector<T>& xodr_data,
                                                                                     double p0, double p1,
                                                                                     FillingGapPolicy policy) const {
  MALIDRIVE_THROW_UNLESS(p0 >= 0.);
  MALIDRIVE_THROW_UNLESS(p1 > p0);
  const std::string xodr_data_type{TypeName<T>()};
  const int num_polynomials = static_cast<int>(xodr_data.size());
  std::vector<std::unique_ptr<road_curve::Function>> polynomials;
  if (num_polynomials == 0) {
    return MakeCubicPolynomial(0., 0., 0., 0., p0, p1);
  }
  for (int i = 0; i < num_polynomials; i++) {
    // Last polynomial's range will be delitimed by p1 instead of next (non-existent) polynomial's s0.
    const bool end{i == num_polynomials - 1};
    const auto coeffs =
        TranslateCubic(xodr_data[i].a, xodr_data[i].b, xodr_data[i].c, xodr_data[i].d, xodr_data[i].s_0);
    const double p0_i{xodr_data[i].s_0};
    const double p1_i{end ? p1 : xodr_data[i + 1].s_0};
    if (std::abs(p1_i - p0_i) < road_curve::GroundCurve::kEpsilon) {
      if (!end) {
        MALIDRIVE_THROW_MESSAGE("Functions in a " + xodr_data_type + " description can't share same start point.");
      } else {
        MALIDRIVE_THROW_MESSAGE("Functions that compound " + xodr_data_type +
                                " must be larger than a epsilon: " + std::to_string(road_curve::GroundCurve::kEpsilon));
      }
    }
    polynomials.emplace_back(MakeCubicPolynomial(coeffs[3], coeffs[2], coeffs[1], coeffs[0], p0_i, p1_i));
  }
  // Adds an polynomial if there is a gap at the start.
  const auto xodr_cubic = xodr_data.begin();
  if (std::abs(xodr_cubic->s_0 - p0) > linear_tolerance()) {
    switch (policy) {
      case FillingGapPolicy::kZero:
        // It is a zero polynomial.
        polynomials.emplace(polynomials.begin(), MakeCubicPolynomial(0., 0., 0., 0., p0, xodr_cubic->s_0));
        break;
      case FillingGapPolicy::kEnsureContiguity:
        // Ensures C1 continuity with the first polynomial in the list.
        polynomials.emplace(polynomials.begin(),
                            MakeCubicPolynomial(p0, xodr_cubic->s_0, polynomials[0]->f(xodr_cubic->s_0),
                                                polynomials[0]->f_dot(xodr_cubic->s_0)));
        break;
      default:
        MALIDRIVE_THROW_MESSAGE("Unknown FillingGapPolicy value.");
    }
  }
  // If `policy` is kZero then no contiguity is required when creating the PiecewiseFunction.
  return policy == FillingGapPolicy::kZero
             ? std::make_unique<road_curve::PiecewiseFunction>(std::move(polynomials), linear_tolerance(),
                                                               true /* no_contiguity_check */)
             : std::make_unique<road_curve::PiecewiseFunction>(std::move(polynomials), linear_tolerance());
}

}  // namespace builder
}  // namespace malidrive
