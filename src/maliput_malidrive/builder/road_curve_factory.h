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

#include <memory>
#include <vector>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/ground_curve.h"
#include "maliput_malidrive/road_curve/piecewise_function.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/geometry.h"
#include "maliput_malidrive/xodr/lane_offset.h"
#include "maliput_malidrive/xodr/lane_width.h"
#include "maliput_malidrive/xodr/lateral_profile.h"

namespace malidrive {
namespace builder {

/// Interface of a helper class to build road curve related objects by
/// MalidriveRoadGeometryBuilder.
///
/// This class is provided as an interface to facilitate testing of
/// MalidriveRoadGeometry via dependency injection.
class RoadCurveFactoryBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadCurveFactoryBase)
  RoadCurveFactoryBase() = delete;

  /// Constructs RoadCurveFactoryBase.
  ///
  /// @param linear_tolerance RoadGeometry's linear tolerance. It will be used
  ///        to build geometry objects. It must be non negative.
  /// @param scale_length RoadGeometry's scale length. It will be used to build
  ///        geometry objects. It must be positive.
  /// @param angular_tolerance RoadGeometry's angular tolerance. It will be used
  ///        to build geometry objects. It must be positive.
  /// @throws maliput::common::assertion_error When @p linear_tolerance is not
  ///         positive.
  /// @throws maliput::common::assertion_error When @p scale_length is not
  ///         positive.
  /// @throws maliput::common::assertion_error When @p angular_tolerance is not
  ///         positive.
  RoadCurveFactoryBase(double linear_tolerance, double scale_length, double angular_tolerance)
      : linear_tolerance_(linear_tolerance), scale_length_(scale_length), angular_tolerance_(angular_tolerance) {
    MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0.);
    MALIDRIVE_THROW_UNLESS(scale_length_ > 0.);
    MALIDRIVE_THROW_UNLESS(angular_tolerance_ > 0.);
  }

  virtual ~RoadCurveFactoryBase() = default;

  /// Tolerance accessors.
  ///@{
  virtual double linear_tolerance() const { return linear_tolerance_; }
  virtual double scale_length() const { return scale_length_; }
  virtual double angular_tolerance() const { return angular_tolerance_; }
  ///@}

  /// @see road_curve::CubicPolynomial::CubicPolynomial() for argument
  ///      details.
  /// @return A road_curve::CubicPolynomial.
  virtual std::unique_ptr<road_curve::Function> MakeCubicPolynomial(double a, double b, double c, double d, double p0,
                                                                    double p1) const = 0;

  /// Creates a cubic polynomial:
  ///
  /// @f$ f(p) = a p^3 + b p^2 + c p + d / p ∈ [`p0`; `p1`] @f$.
  ///
  /// Meeting the following considerations:
  ///  f(`p0`) = 0      f'(`p0`) = 0
  ///  f(`p1`) = `y`    f'(`p1`) = `dy`
  ///
  /// @param p0 Lower bound extreme of the parameter range. It must not be
  ///        negative and must be less than @p p1.
  /// @param p1 Upper bound extreme of the parameter range. It must be greater
  ///        than @p p0.
  /// @param y Polynomial evaluated at @f$ p = `p1` @f$.
  /// @param dy Derived polynomial evaluated at @f$ p = `p1` @f$.
  /// @return A road_curve::CubicPolynomial.
  ///
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not greater than @p p0.
  virtual std::unique_ptr<road_curve::Function> MakeCubicPolynomial(double p0, double p1, double y,
                                                                    double dy) const = 0;

  /// Makes a road_curve::ArcGroundCurve.
  ///
  /// Its linear tolerance will be the constructor argument.
  ///
  /// @param arc_geometry xodr::Geometry definition to construct a
  ///        road_curve::ArcGroundCurve. Its type must be
  ///        xodr::Geometry::Type::kArc.
  /// @return A road_curve::ArcGroundCurve.
  /// @throws maliput::common::assertion_error When `arc_geometry.type` is not
  ///         xodr::Geometry::Type::kArc.
  virtual std::unique_ptr<road_curve::GroundCurve> MakeArcGroundCurve(const xodr::Geometry& arc_geometry) const = 0;

  /// Makes a road_curve::LineGroundCurve.
  ///
  /// Its linear tolerance will be the constructor argument.
  ///
  /// @param line_geometry xodr::Geometry definition to construct a
  ///        road_curve::LineGroundCurve. Its type must be
  ///        xodr::Geometry::Type::kLine.
  /// @return A road_curve::LineGroundCurve.
  /// @throws maliput::common::assertion_error When `line_geometry.type` is not
  ///         xodr::Geometry::Type::kLine.
  virtual std::unique_ptr<road_curve::GroundCurve> MakeLineGroundCurve(const xodr::Geometry& line_geometry) const = 0;

  /// Makes a road_curve::PiecewiseGroundCurve.
  ///
  /// Its linear tolerance will be the constructor argument.
  ///
  /// @param geometries A vector of xodr::Geometry definitions to construct a
  ///        road_curve::PiecewiseGroundCurve. Item's type must be one of
  ///        {xodr::Geometry::Type::kArc, xodr::Geometry::Type::kLine}. It must
  ///        not be empty. Geometries whose length is less than GroundCurve::kEpsilon are discarded.
  /// @return A road_curve::PiecewiseGroundCurve.
  /// @throws maliput::common::assertion_error When any item of @p geometries
  ///         has other type than {xodr::Geometry::Type::kArc,
  ///         xodr::Geometry::Type::kLine}.
  /// @throws maliput::common::assertion_error When @p geometries is empty.
  virtual std::unique_ptr<road_curve::GroundCurve> MakePiecewiseGroundCurve(
      const std::vector<xodr::Geometry>& geometries) const = 0;

  /// Makes a cubic polynomial that describes the elevation of a Road.
  ///
  /// Constructs a road::curve::Function out of @p elevation_profile.
  /// There could be two scenarios that depends on `elevation_profile`:
  ///  - If it is empty then a zero cubic polynomial is created in the range [`p0`, `p1`]
  ///  - If it contains one or more elevations then a piecewised function is created in the range [`p0`, `p1`]
  ///
  /// @note Handling tolerance: If there is a gap between `p0` and the S0 value of the first elevation
  ///                           then a cubic polynomial is created to fullfill that gap.
  ///                           @see MakeCubicPolynomial To know characteristics of the cubic polynomial that is
  ///                           created.
  /// @param elevation_profile Contains the elevation described in the XODR for the Road.
  /// @param p0 Lower bound extreme of the parameter range. It must not be
  ///        negative and must be less than @p p1.
  /// @param p1 Upper bound extreme of the parameter range. It must be greater
  ///        than @p p0.
  /// @param assert_continuity If true, C1 continuity is assert when function describing the elevation is built.
  ///                          Otherwise only warning messages are printed.
  /// @returns A function that describes the elevation of the Road.
  ///
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not greater enough than @p p0.
  virtual std::unique_ptr<malidrive::road_curve::Function> MakeElevation(
      const xodr::ElevationProfile& elevation_profile, double p0, double p1, bool assert_continuity) const = 0;

  /// Makes a cubic polynomial that describes the superelevation of a Road.
  ///
  /// Constructs a road::curve::Function out of @p lateral_profile.
  /// There could be two scenarios that depends on `lateral_profile`:
  ///  - If it is empty then a zero cubic polynomial is created in the range [`p0`, `p1`]
  ///  - If it contains one or more superelevations then a piecewised function is created in the range [`p0`, `p1`]
  ///
  /// @note Handling tolerance: If there is a gap between `p0` and the S0 value of the first superelevation
  ///                           then a cubic polynomial is created to fullfill that gap.
  ///                           @see MakeCubicPolynomial To know characteristics of the cubic polynomial that is
  ///                           created.
  /// @param lateral_profile Contains the superelevation described in the XODR for the Road.
  /// @param p0 Lower bound extreme of the parameter range. It must not be
  ///        negative and must be less than @p p1.
  /// @param p1 Upper bound extreme of the parameter range. It must be greater
  ///        than @p p0.
  /// @param assert_continuity If true, C1 continuity is assert when function describing the superelevation is built.
  ///                          Otherwise only warning messages are printed.
  /// @returns A function that describes the superelevation of the Road.
  ///
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not greater enough than @p p0.
  virtual std::unique_ptr<malidrive::road_curve::Function> MakeSuperelevation(
      const xodr::LateralProfile& lateral_profile, double p0, double p1, bool assert_continuity) const = 0;

  /// Makes a cubic polynomial that describes the width of a Lane.
  ///
  /// Constructs a road::curve::Function out of @p lane_widths.
  ///
  /// @param lane_widths Contains the width described in the XODR for the Lane.
  /// @param p0 Lower bound extreme of the parameter range. It must not be
  ///        negative and must be less than @p p1.
  /// @param p1 Upper bound extreme of the parameter range. It must be greater
  ///        than @p p0.
  /// @param assert_continuity If true, C1 continuity is assert when function describing the lane width is built.
  ///                          Otherwise only warning messages are printed.
  /// @returns A function that describes the width of the Lane.
  ///
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not greater enough than @p p0.
  /// @throws maliput::common::assertion_error When @p lane_widths 's size is zero.
  /// @throws maliput::common::assertion_error When the first offset value of @p lane_widths is different than zero.
  /// @throws maliput::common::assertion_error When @p lane_widths 's start points are distanced less than linear
  /// tolerance.
  /// @throws maliput::common::assertion_error When @p lane_widths 's functions aren't at least as long as
  /// road::curve::GroundCurve::kEpsilon.
  virtual std::unique_ptr<malidrive::road_curve::Function> MakeLaneWidth(
      const std::vector<xodr::LaneWidth>& lane_widths, double p0, double p1, bool assert_continuity) const = 0;

  /// Makes a cubic polynomial that describes the lateral shift of the road reference line.
  ///
  /// When @p reference_offsets is empty, a zero cubic polynomial is created in the range [`p0`, `p1`].
  /// Otherwise, a PiecewiseFunction is created in the range [`p0`, `p1`].
  ///
  /// @note Handling tolerance: If there is a gap between `p0` and the S0 value of the first offset function
  ///                           then a zero cubic polynomial is created to fullfill that gap.
  ///
  /// @param reference_offsets Contains the offset described in the XODR's `laneOffset` record for the road reference
  /// line.
  /// @param p0 Lower bound extreme of the parameter range. It must not be
  ///        negative and must be less than @p p1.
  /// @param p1 Upper bound extreme of the parameter range. It must be greater
  ///        than @p p0.
  /// @returns A function that describes the lateral shift of the road reference line.
  ///
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is not greater enough than @p p0.
  virtual std::unique_ptr<malidrive::road_curve::Function> MakeReferenceLineOffset(
      const std::vector<xodr::LaneOffset>& reference_offsets, double p0, double p1) const = 0;

  /// Makes a road_curve::MalidriveGroundCurve.
  ///
  /// Its linear tolerance and scale length will be the constructor arguments.
  ///
  /// @see road_curve::RoadCurve::RoadCurve() for argument
  ///      details.
  ///
  /// @return A road_curve::MalidriveGroundCurve.
  virtual std::unique_ptr<road_curve::RoadCurve> MakeMalidriveRoadCurve(
      std::unique_ptr<road_curve::GroundCurve> ground_curve, std::unique_ptr<road_curve::Function> elevation,
      std::unique_ptr<road_curve::Function> superelevation, bool assert_contiguity) const = 0;

 private:
  const double linear_tolerance_{};
  const double scale_length_{};
  const double angular_tolerance_{};
};

/// Concrete RoadCurveFactoryBase implementation.
class RoadCurveFactory final : public RoadCurveFactoryBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadCurveFactory)
  RoadCurveFactory() = delete;

  RoadCurveFactory(double linear_tolerance, double scale_length, double angular_tolerance)
      : RoadCurveFactoryBase(linear_tolerance, scale_length, angular_tolerance) {}

  std::unique_ptr<road_curve::Function> MakeCubicPolynomial(double a, double b, double c, double d, double p0,
                                                            double p1) const override;
  std::unique_ptr<road_curve::Function> MakeCubicPolynomial(double p0, double p1, double y, double dy) const override;

  std::unique_ptr<road_curve::GroundCurve> MakeArcGroundCurve(const xodr::Geometry& arc_geometry) const override;

  std::unique_ptr<road_curve::GroundCurve> MakeLineGroundCurve(const xodr::Geometry& line_geometry) const override;

  std::unique_ptr<road_curve::GroundCurve> MakePiecewiseGroundCurve(
      const std::vector<xodr::Geometry>& geometries) const override;

  std::unique_ptr<malidrive::road_curve::Function> MakeElevation(const xodr::ElevationProfile& elevation_profile,
                                                                 double p0, double p1,
                                                                 bool assert_continuity) const override;
  std::unique_ptr<malidrive::road_curve::Function> MakeSuperelevation(const xodr::LateralProfile& lateral_profile,
                                                                      double p0, double p1,
                                                                      bool assert_continuity) const override;
  std::unique_ptr<malidrive::road_curve::Function> MakeLaneWidth(const std::vector<xodr::LaneWidth>& lane_widths,
                                                                 double p0, double p1,
                                                                 bool assert_continuity) const override;
  std::unique_ptr<malidrive::road_curve::Function> MakeReferenceLineOffset(
      const std::vector<xodr::LaneOffset>& reference_offsets, double p0, double p1) const override;

  std::unique_ptr<road_curve::RoadCurve> MakeMalidriveRoadCurve(std::unique_ptr<road_curve::GroundCurve> ground_curve,
                                                                std::unique_ptr<road_curve::Function> elevation,
                                                                std::unique_ptr<road_curve::Function> superelevation,
                                                                bool assert_contiguity) const override;

 private:
  // Whether MakeCubicFromXodr() should fulfill gaps with a zero polynomial or ensure C1 continuity.
  enum class FillingGapPolicy { kZero = 0, kEnsureContiguity };

  // Constructs a road::curve::Function out of @p xodr_data.
  //
  // @see MakeElevation.
  // @see MakeSuperelevation.
  //
  // @tparam T One of xodr::ElevationProfile::Elevation, xodr::LateralProfile::Superelevation or xodr::LaneOffset.
  //
  // @param xodr_data Contains the cubic polynomials' coefficient.
  // @param p0 Lower bound extreme of the parameter range. It must not be
  //        negative and must be less than @p p1.
  // @param p1 Upper bound extreme of the parameter range. It must be greater
  //        than @p p0.
  // @param policy Determines the behavior of this method to fill geometry gaps.
  // @param continuity_check Determines whether during a picewise-defined function creation C1 continuity should be
  // enforced.
  // @returns A cubic polynomial function.
  //
  // @throws maliput::common::assertion_error When @p p0 is negative.
  // @throws maliput::common::assertion_error When @p p1 is not greater enough than @p p0.
  template <class T>
  std::unique_ptr<malidrive::road_curve::Function> MakeCubicFromXodr(
      const std::vector<T>& xodr_data, double p0, double p1, FillingGapPolicy policy,
      road_curve::PiecewiseFunction::ContinuityCheck continuity_check) const;
};

}  // namespace builder
}  // namespace malidrive
