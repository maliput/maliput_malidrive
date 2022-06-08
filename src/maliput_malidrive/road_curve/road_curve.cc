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
#include "maliput_malidrive/road_curve/road_curve.h"

#include <cmath>
#include <utility>

#include <maliput/math/matrix.h>
#include <maliput/math/saturate.h>

#include "maliput_malidrive/road_curve/cubic_polynomial.h"

namespace malidrive {
namespace road_curve {

RoadCurve::RoadCurve(double linear_tolerance, double scale_length, std::unique_ptr<GroundCurve> ground_curve,
                     std::unique_ptr<Function> elevation, std::unique_ptr<Function> superelevation,
                     bool assert_contiguity)
    : linear_tolerance_(linear_tolerance),
      scale_length_(scale_length),
      ground_curve_(std::move(ground_curve)),
      elevation_(std::move(elevation)),
      superelevation_(std::move(superelevation)) {
  // Non negative quantities check.
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ >= 0.);
  MALIDRIVE_THROW_UNLESS(scale_length_ >= 0.);
  // nullptr check.
  MALIDRIVE_THROW_UNLESS(ground_curve_ != nullptr);
  MALIDRIVE_THROW_UNLESS(elevation_ != nullptr);
  MALIDRIVE_THROW_UNLESS(superelevation_ != nullptr);
  // Contiguity check.
  MALIDRIVE_THROW_UNLESS(ground_curve_->IsG1Contiguous());
  if (assert_contiguity) {
    MALIDRIVE_THROW_UNLESS(elevation_->IsG1Contiguous());
    MALIDRIVE_THROW_UNLESS(superelevation_->IsG1Contiguous());
  }
  // Range checks.
  MALIDRIVE_THROW_UNLESS(std::abs(ground_curve_->p0() - elevation_->p0()) <= linear_tolerance_);
  MALIDRIVE_THROW_UNLESS(std::abs(ground_curve_->p0() - superelevation_->p0()) <= linear_tolerance_);
  MALIDRIVE_THROW_UNLESS(std::abs(elevation_->p0() - superelevation_->p0()) <= linear_tolerance_);
  MALIDRIVE_THROW_UNLESS(std::abs(ground_curve_->p1() - elevation_->p1()) <= linear_tolerance_);
  MALIDRIVE_THROW_UNLESS(std::abs(ground_curve_->p1() - superelevation_->p1()) <= linear_tolerance_);
  MALIDRIVE_THROW_UNLESS(std::abs(elevation_->p1() - superelevation_->p1()) <= linear_tolerance_);
}

maliput::math::Vector3 RoadCurve::W(const maliput::math::Vector3& prh) const {
  MALIDRIVE_IS_IN_RANGE(prh.x(), ground_curve_->p0() - ground_curve_->linear_tolerance(),
                        ground_curve_->p1() + ground_curve_->linear_tolerance());
  const double p = maliput::math::saturate(prh.x(), ground_curve_->p0(), ground_curve_->p1());
  // Calculates z (elevation) of (p,0,0).
  const double z = elevation_->f(p);
  // Calculates x,y of (p,0,0).
  const maliput::math::Vector2 xy = ground_curve_->G(p);
  // Calculates orientation of (p,r,h) basis at (p,0,0).
  const maliput::math::RollPitchYaw rpy = Orientation(p);
  // Rotates (0,r,h) and sums with mapped (p,0,0).
  return rpy.ToMatrix() * maliput::math::Vector3(0., prh.y(), prh.z()) + maliput::math::Vector3(xy.x(), xy.y(), z);
}

maliput::math::Vector3 RoadCurve::WDot(const maliput::math::Vector3& prh) const {
  const road_curve::CubicPolynomial zero_function{0., 0., 0., 0., p0(), p1(), linear_tolerance_};
  return WDot(prh, &zero_function);
}

maliput::math::Vector3 RoadCurve::WDot(const maliput::math::Vector3& prh, const Function* lane_offset) const {
  MALIDRIVE_THROW_UNLESS(lane_offset != nullptr);
  MALIDRIVE_IS_IN_RANGE(prh.x(), ground_curve_->p0() - ground_curve_->linear_tolerance(),
                        ground_curve_->p1() + ground_curve_->linear_tolerance());
  MALIDRIVE_IS_IN_RANGE(lane_offset->p0(), ground_curve_->p0() - ground_curve_->linear_tolerance(),
                        ground_curve_->p1() + ground_curve_->linear_tolerance());
  MALIDRIVE_IS_IN_RANGE(lane_offset->p1(), ground_curve_->p0() - ground_curve_->linear_tolerance(),
                        ground_curve_->p1() + ground_curve_->linear_tolerance());
  const double p = maliput::math::saturate(prh.x(), lane_offset->p0(), lane_offset->p1());
  const double r = prh.y();
  const double h = prh.z();
  const double r_dot = lane_offset->f_dot(p);
  const maliput::math::Vector2 g_prime = ground_curve_->GDot(p);

  const maliput::math::RollPitchYaw rpy_at_centerline = Orientation(p);
  const double beta = rpy_at_centerline.pitch_angle();
  const double cb = std::cos(beta);

  // Evaluate dα/dp, dβ/dp, dγ/dp...
  const double d_alpha = superelevation_->f_dot(p);
  // this definition of d_beta assumes that g_prime.norm() does not vary with p: d{|G'|}/dp = 0
  const double d_beta = -cb * cb * elevation_->f_dot_dot(p) / g_prime.norm();
  const double d_gamma = ground_curve_->HeadingDot(p);

  // compute rotation matrix at centerline (R) and its time derivative (dR_dt)
  const maliput::math::Matrix3 R = rpy_at_centerline.ToMatrix();
  const maliput::math::Matrix3 dR_dt = rpy_at_centerline.CalcRotationMatrixDt({d_alpha, d_beta, d_gamma});

  return maliput::math::Vector3(g_prime.x(), g_prime.y(), elevation_->f_dot(p)) +
         dR_dt * maliput::math::Vector3(0, r, h) + R * maliput::math::Vector3{0., r_dot, 0.};
}

maliput::math::RollPitchYaw RoadCurve::Orientation(double p) const {
  MALIDRIVE_IS_IN_RANGE(p, ground_curve_->p0() - ground_curve_->linear_tolerance(),
                        ground_curve_->p1() + ground_curve_->linear_tolerance());
  p = maliput::math::saturate(p, ground_curve_->p0(), ground_curve_->p1());
  const maliput::math::Vector2 g_prime = ground_curve_->GDot(p);
  return maliput::math::RollPitchYaw(superelevation_->f(p), -std::atan2(elevation_->f_dot(p), g_prime.norm()),
                                     ground_curve_->Heading(p));
}

maliput::math::RollPitchYaw RoadCurve::Orientation(const maliput::math::Vector3& prh) const {
  const road_curve::CubicPolynomial zero_function{0., 0., 0., 0., p0(), p1(), linear_tolerance_};
  return Orientation(prh, &zero_function);
}
maliput::math::RollPitchYaw RoadCurve::Orientation(const maliput::math::Vector3& prh,
                                                   const Function* lane_offset) const {
  MALIDRIVE_IS_IN_RANGE(prh.x(), ground_curve_->p0(), ground_curve_->p1());
  MALIDRIVE_THROW_UNLESS(lane_offset != nullptr);
  const double p = maliput::math::saturate(prh.x(), ground_curve_->p0(), ground_curve_->p1());

  // Calculate s,r basis vectors at (s,r,h).
  const maliput::math::Vector3 s_hat = SHat({p, prh.y(), prh.z()}, lane_offset);
  const maliput::math::Vector3 r_hat = RHat({p, prh.y(), prh.z()}, lane_offset);
  // ...and then derive orientation from those basis vectors.
  //
  // (s_hat  r_hat  h_hat) is an orthonormal basis, obtained by rotating the
  // (x_hat  y_hat  z_hat) basis by some R-P-Y rotation; in this case, we know
  // the value of (s_hat  r_hat  h_hat) (w.r.t. 'xyz' world frame), so we are
  // trying to recover the roll/pitch/yaw.  Since (x_hat  y_hat  z_hat) is an
  // identity matrix (e.g., x_hat = column vector (1, 0, 0), etc), then
  // (s_hat  r_hat  h_hat) equals the R-P-Y matrix itself.
  // If we define a = alpha = roll, b = beta = pitch, g = gamma = yaw,
  // then s_hat is the first column of the rotation, r_hat is the second:
  //   s_hat = (cb * cg, cb * sg, - sb)
  //   r_hat = (- ca * sg + sa * sb * cg, ca * cg + sa * sb * sg, sa * cb)
  // We solve the above for a, b, g.
  const double gamma = std::atan2(s_hat.y(), s_hat.x());
  const double beta = std::atan2(-s_hat.z(), maliput::math::Vector2(s_hat.x(), s_hat.y()).norm());
  const double cb = std::cos(beta);
  const double alpha = std::atan2(r_hat.z() / cb, ((r_hat.y() * s_hat.x()) - (r_hat.x() * s_hat.y())) / cb);
  return {alpha, beta, gamma};
}

maliput::math::Vector3 RoadCurve::WInverse(const maliput::math::Vector3& xyz) const {
  // Gets initial estimate of `p` from the ground curve.
  double p = ground_curve_->GInverse({xyz.x(), xyz.y()});

  // Correction in p computed iteratively.
  double dp{2.0 * linear_tolerance_};

  // Start of iterations.
  for (int i = 0; i < kMaxIterations && std::abs(dp) > linear_tolerance_; ++i) {
    // Gets the position in the INERTIAL Frame at the centerline.
    const maliput::math::Vector3 prh_at_centerline{p, 0, 0};
    const maliput::math::Vector3 w_p = W(prh_at_centerline);
    // Gets the vector difference between w_p and `xyz`.
    const maliput::math::Vector3 w_delta = xyz - w_p;
    // Computes the centerline derivative with respect to p.
    const maliput::math::Vector3 w_dot = WDot(prh_at_centerline);
    // Iterative updates of `p` with Newton's method:
    // Compute correction in p from component of w_delta / w_dot.norm() parallel to centerline:
    //   dp = (w_delta / w_dot.norm()).dot(s_hat);
    // which is equivalent to the following:
    dp = w_delta.dot(w_dot) / w_dot.dot(w_dot);

    p = maliput::math::saturate(p + dp, ground_curve_->p0(), ground_curve_->p1());
  }

  // Recompute with final value of p:
  // Gets the position in the INERTIAL Frame at the centerline.
  const maliput::math::Vector3 prh_at_centerline{p, 0, 0};
  const maliput::math::Vector3 w_p = W(prh_at_centerline);
  // Gets the vector difference between w_p and `xyz`.
  const maliput::math::Vector3 w_delta = xyz - w_p;
  // Computes the orthonormal basis at p, and projects the difference onto r_hat
  // and h_hat to get each component.
  const maliput::math::Vector3 s_hat = SHat(prh_at_centerline);
  const maliput::math::Vector3 h_hat = HHat(p, s_hat);
  const maliput::math::Vector3 r_hat = h_hat.cross(s_hat);
  return maliput::math::Vector3(p, r_hat.dot(w_delta), h_hat.dot(w_delta));
}

maliput::math::Vector3 RoadCurve::SHat(const maliput::math::Vector3& prh) const {
  const road_curve::CubicPolynomial zero_function{0., 0., 0., 0., p0(), p1(), linear_tolerance_};
  return SHat(prh, &zero_function);
}

maliput::math::Vector3 RoadCurve::RHat(const maliput::math::Vector3& prh) const {
  const road_curve::CubicPolynomial zero_function{0., 0., 0., 0., p0(), p1(), linear_tolerance_};
  return RHat(prh, &zero_function);
}

maliput::math::Vector3 RoadCurve::SHat(const maliput::math::Vector3& prh, const Function* lane_offset) const {
  MALIDRIVE_IS_IN_RANGE(prh.x(), ground_curve_->p0(), ground_curve_->p1());
  MALIDRIVE_THROW_UNLESS(lane_offset != nullptr);
  return WDot(prh, lane_offset).normalized();
}

maliput::math::Vector3 RoadCurve::RHat(const maliput::math::Vector3& prh, const Function* lane_offset) const {
  MALIDRIVE_IS_IN_RANGE(prh.x(), ground_curve_->p0(), ground_curve_->p1());
  MALIDRIVE_THROW_UNLESS(lane_offset != nullptr);
  const maliput::math::Vector3 s_hat = SHat(prh, lane_offset);
  const maliput::math::Vector3 h_hat = HHat(prh.x(), s_hat);
  return h_hat.cross(s_hat);
}

maliput::math::Vector3 RoadCurve::HHat(double p, const maliput::math::Vector3& s_hat) const {
  MALIDRIVE_IS_IN_RANGE(p, ground_curve_->p0(), ground_curve_->p1());
  MALIDRIVE_IS_IN_RANGE(s_hat.norm(), 1. - linear_tolerance_, 1 + linear_tolerance_);
  const maliput::math::Vector3 z_hat = maliput::math::Vector3::UnitZ();
  const maliput::math::Vector3 h_hat_0 = (z_hat - z_hat.dot(s_hat) * s_hat).normalized();
  // assuming a flat lateral profile
  const maliput::math::Quaternion s_hat_superelevation(superelevation_->f(p), s_hat);
  return s_hat_superelevation * h_hat_0;
}

}  // namespace road_curve
}  // namespace malidrive
