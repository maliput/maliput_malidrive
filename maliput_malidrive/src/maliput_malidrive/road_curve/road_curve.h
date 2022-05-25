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

#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/ground_curve.h"
#include "maliput_malidrive/road_curve/lane_offset.h"

namespace malidrive {
namespace road_curve {

/// Defines a class for a path in a Segment object surface. The path is
/// defined by an elevation and superelevation Function objects and a
/// GroundCurve reference curve. The latter is a C1 function in the z=0 plane.
/// Its domain is constrained in @f$ [GroundCurve::p0(); GroundCurve::p1()] @f$
/// interval and it should map a ℝ² curve.
/// As per notation, @f$ p @f$ is the parameter of the reference curve, not
/// necessarily arc length @f$ s @f$, and function interpolations and function
/// derivatives as well as headings and heading derivatives are expressed in
/// INERTIAL Frame coordinates.
///
/// The geometry here revolves around an abstract "world function"
///
/// @f$ W: (p,r,h) \mapsto (x,y,z) ∈ ℝ³ @f$
///
/// which maps a `Lane`-frame position to its corresponding representation in
/// world coordinates (with the caveat that instead of the lane's native
/// longitudinal coordinate 's', the reference curve parameter 'p' is used).
///
/// W is derived from three functions which define the lane:
///
///   G: p --> (x,y) = the reference ground curve.
///   Z: p --> z     = the elevation function.
///   Θ: p --> θ     = the superelevation function.
///
/// as:
///
///   (x,y,z) = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0, r, h)
///
/// where:
///
/// - R_αβγ is the roll/pitch/yaw rotation given by angles:
///
///   α = Θ(p)
///   β = -atan2(dZ/dp, sqrt((dG_x/dp)^2 + (dG_y/dp)^2)) at p
///   γ = atan2(dG_y/dp, dG_x/dp) at p
///
/// (R_αβγ is essentially the orientation of the (s,r,h) `Lane`-frame
/// at a location (s,0,0) on the reference-line of the lane.  However, it
/// is *not* necessarily the correct orientation at r != 0 or h != 0.)
///
/// The W(p,r,h) "world function" is defined by the RoadCurve referenced by a
/// Lane's Segment. A Lane is also defined by a r0 lateral offset with respect
/// to the reference curve of the RoadCurve. Thus, a mapping from the local
/// (s,r,h) lane-frame of the Lane becomes:
///
/// (x,y,z) = L(s,r,h) = W(P(s, r0), r + r0, h),
///
/// where P:(s, r0) --> (p) is a (potentially non-linear) function dependent on
/// the RoadCurve's reference-curve, elevation, and superelevation functions.
class RoadCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadCurve);

  /// Constructs a RoadCurve.
  ///
  /// @param linear_tolerance It is expected to be the same as
  ///        maliput::api::RoadGeometry::linear_tolerance(). It must be non
  ///        negative. It is used to integrate an offset RoadCurve's arc length
  ///        and adjust integration parameters accordingly.
  /// @param scale_length It is expected to be the same as
  ///        maliput::api::RoadGeometry::scale_length(). It must be non
  ///        negative. It is used to integrate an offset RoadCurve's arc length
  ///        and adjust integration parameters accordingly.
  /// @param ground_curve The reference GroundCurve of this RoadCurve. It must
  ///        not be nullptr and must be G¹.
  /// @param elevation The elevation Function of this RoadCurve. It must not be
  ///        nullptr.
  /// @param superelevation The superelevation Function of this RoadCurve. It
  ///        must not be nullptr.
  /// @param assert_contiguity True for assert contiguity for `elevation` and `superelevation` functions.
  ///
  /// @throws maliput::common::assertion_error When @p linear_tolerance or
  ///         @p scale_length are negative.
  /// @throws maliput::common::assertion_error When @p ground_curve or
  ///         @p elevation or @p superelevation are nullptr.
  /// @throws maliput::common::assertion_error When @p ground_curve or
  ///         @p elevation or @p superelevation are not G¹.
  /// @throws maliput::common::assertion_error When @p ground_curve is not G¹.
  /// @throws maliput::common::assertion_error When @p ground_curve,
  ///         @p elevation and @p superelevation do not share the same range
  ///         within @p linear_tolerance.
  RoadCurve(double linear_tolerance, double scale_length, std::unique_ptr<GroundCurve> ground_curve,
            std::unique_ptr<Function> elevation, std::unique_ptr<Function> superelevation, bool assert_contiguity);

  ~RoadCurve() = default;

  double p0() const { return ground_curve_->p0(); }
  double p1() const { return ground_curve_->p1(); }
  double LMax() const { return ground_curve_->ArcLength(); }

  /// @return The linear tolerance used to compute all the methods.
  /// @see maliput::api::RoadGeometry::linear_tolerance().
  double linear_tolerance() const { return linear_tolerance_; }

  /// @return The scale length of the tolerance used to compute all the methods.
  /// @see maliput::api::RoadGeometry::scale_length().
  double scale_length() const { return scale_length_; }

  /// Evaluates @f$ W(p, r, h) @f$.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @return A vector in the INERTIAL Frame which is the image of the RoadCurve.
  maliput::math::Vector3 W(const maliput::math::Vector3& prh) const;

  /// Evaluates @f$ W'(p, r, h) @f$ with respect to @f$ p @f$.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @return The derivative of @f$ W @f$ with respect to @f$ p @f$ at @p prh.
  maliput::math::Vector3 WDot(const maliput::math::Vector3& prh) const;

  /// Evaluates @f$ W'(p, r, h) @f$ with respect to @f$ p @f$.
  ///
  /// @note Recall that W is the lane-to-world transform, defined by
  ///   @f$ (x,y,z)  = W(p,r,h) = (G(p), Z(p)) + R_{αβγ}*(0,r(p),h) @f$ \n
  /// where @f$ G @f$ is the reference curve, @f$ Z @f$ is the elevation profile, and @f$ R_{αβγ} @f$ is
  /// a rotation matrix derived from reference curve (heading), elevation,
  /// and superelevation. Thus:
  /// @note  @f$ ∂W/∂p = (∂G(p)/∂p, ∂Z(p)/∂p) + (∂R_{αβγ}/∂p)*(0,r(p),h) + R_{αβγ}*∂(0,r(p),0)/∂p @f$, \n
  /// where: \n
  /// @f$ ∂G(p)/∂p = G'(p) @f$ \n
  /// @f$ ∂Z(p)/∂p = Z'(p) @f$ \n
  /// @f$ ∂R_{αβγ}/∂p = (∂R_{αβγ}/∂α ∂R_{αβγ}/∂β ∂R_{αβγ}/∂γ)*(dα/dp, dβ/dp, dγ/dp) @f$
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @param lane_offset Holds the function, @f$ r(p) @f$, that describes the lateral offset at p.
  ///                    Used to calculate the derivative at @p prh.
  /// @return The derivative of @f$ W @f$ with respect to @f$ p @f$ at @p prh.
  /// @throw maliput::common::assertion_error When @p lane_offset is nullptr.
  /// @throw maliput::common::assertion_error When @p prh .x() is not in range [p0, p1].
  /// @throw maliput::common::assertion_error When @p lane_offset ->p0() is not in range [p0, p1].
  /// @throw maliput::common::assertion_error When @p lane_offset ->p1() is not in range [p0, p1].
  maliput::math::Vector3 WDot(const maliput::math::Vector3& prh, const Function* lane_offset) const;

  /// Evaluates the orientation in the INERTIAL Frame of the RoadCurve at @p p,
  /// i.e. at @f$ (p, 0, 0) @f$.
  ///
  /// @param p The GroundCurve parameter.
  /// @return The orientation in the INERTIAL Frame of the RoadCurve at @p p.
  maliput::math::RollPitchYaw Orientation(double p) const;

  /// Evaluates the orientation in the INERTIAL Frame of the RoadCurve at
  /// @p prh.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @return The orientation in the INERTIAL Frame of the RoadCurve at @p prh.
  maliput::math::RollPitchYaw Orientation(const maliput::math::Vector3& prh) const;

  /// Evaluates the orientation in the INERTIAL Frame of the RoadCurve at @p prh.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @param lane_offset Holds the function, @f$ r(p) @f$, that describes the lateral offset at p.
  ///                    Used to calculate the derivative at @p prh.
  /// @return The orientation in the INERTIAL Frame of the RoadCurve at @p prh.
  /// @throw maliput::common::assertion_error When @p lane_offset is nullptr.
  /// @throw maliput::common::assertion_error When @p prh .x() is not in range [p0, p1].
  maliput::math::RollPitchYaw Orientation(const maliput::math::Vector3& prh, const Function* lane_offset) const;

  /// Evaluates @f$ W⁻¹(x, y, z) @f$.
  ///
  /// @param xyz A point in ℝ³ that would be used to minimize the Euclidean
  ///        distance the image of @f$ W @f$.
  /// @return A vector in RoadCurve's domain whose image through @f$ W @f$ would
  ///         minimize the Euclidean distance to @p xyz.
  maliput::math::Vector3 WInverse(const maliput::math::Vector3& xyz) const;

  /// Evaluates @f$ W'(p, r, h) / |W'(p, r, h)|` with respect to @f$ p @f$.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @return A normalized tangent vector in the direction of increasing
  ///         @f$ p @f$ at @p prh.
  maliput::math::Vector3 SHat(const maliput::math::Vector3& prh) const;

  /// Evaluates @f$ W'(p, r, h) / |W'(p, r, h)|` with respect to @f$ p @f$.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @param lane_offset Holds the function, @f$ r(p) @f$, that describes the lateral offset at p.
  ///                    Used to calculate the derivative at @p prh.
  /// @return A normalized tangent vector in the direction of increasing
  ///         @f$ p @f$ at @p prh.
  /// @throw maliput::common::assertion_error When @p lane_offset is nullptr.
  /// @throw maliput::common::assertion_error When @p prh .x() is not in range [p0, p1].
  maliput::math::Vector3 SHat(const maliput::math::Vector3& prh, const Function* lane_offset) const;

  /// Evaluates the r-axis unit vector at @f$ (p, r, h) @f$.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @return A normalized vector pointing in the direction of increasing
  ///         @f$ r @f$ coordinate at @p prh.
  maliput::math::Vector3 RHat(const maliput::math::Vector3& prh) const;

  /// Evaluates the r-axis unit vector at @f$ (p, r, h) @f$.
  ///
  /// @param prh A vector in the RoadCurve domain.
  /// @param lane_offset Holds the function, @f$ r(p) @f$, that describes the lateral offset at p.
  ///                    Used to calculate the derivative at @p prh.
  /// @return A normalized tangent vector in the direction of increasing
  ///         @f$ r @f$ at @p prh.
  /// @throw maliput::common::assertion_error When @p lane_offset is nullptr.
  /// @throw maliput::common::assertion_error When @p prh .x() is not in range [p0, p1].
  maliput::math::Vector3 RHat(const maliput::math::Vector3& prh, const Function* lane_offset) const;

  /// Evaluates the h-axis unit vector at @f$ (p, r, h) @f$.
  ///
  /// @param p The GroundCurve parameter.
  /// @param s_hat A normalized tangent vector in the direction of increasing @f$ p @f$.
  /// @return A normalized tangent vector in the direction of increasing
  ///         @f$ h @f$ at @p p, i.e. at @f$ (p, 0, 0) @f$.
  ///         It is orthogonal to @p s_hat by construction.
  /// @throw maliput::common::assertion_error When @p lane_offset is nullptr.
  /// @throw maliput::common::assertion_error When @p p is not in range [p0, p1].
  maliput::math::Vector3 HHat(double p, const maliput::math::Vector3& s_hat) const;

  /// Calculates the @f$ p @f$ value that matches with the @f$ p @f$ value in the XODR description.
  /// @param xodr_p The parameter in the XODR description.
  /// @returns The @f$ p @f$ value in the GroundCurve domain.
  double PFromP(double xodr_p) const { return ground_curve_->PFromP(xodr_p); }

 private:
  // Maximum number of iterations to use in DoWInverse.
  static constexpr int kMaxIterations{16};
  const double linear_tolerance_{};
  const double scale_length_{};
  std::unique_ptr<GroundCurve> ground_curve_{};
  std::unique_ptr<Function> elevation_{};
  std::unique_ptr<Function> superelevation_{};
};

}  // namespace road_curve
}  // namespace malidrive
