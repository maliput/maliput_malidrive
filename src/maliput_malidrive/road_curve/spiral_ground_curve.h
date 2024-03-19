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

class SpiralGroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(SpiralGroundCurve);

  SpiralGroundCurve() = delete;

  /// Constructs an SpiralGroundCurve.
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

  // Converts the @p p value into a Fresnel spiral t coordinate.
  // Offsets @p p by p0_ and then the s0_spiral_ (which is the parameter of the spiral for a the
  // given initial curvature) and then scales it by the norm_.
  // @param p The parameter of the GroundCurve.
  // @return The normalized parameter of the Fresnel spiral.
  inline double TFromP(double p) const { return (p - p0_ + s0_spiral_) / norm_; }

  // The linear tolerance. It is used to distinguish between a line and an arc.
  const double linear_tolerance_{};
  // The first point of the arc in world coordinates.
  const maliput::math::Vector2 xy0_{};
  //
  const double heading0_{};

  const double curvature0_{};

  const double curvature1_{};
  // The length of the arc.
  const double arc_length_{};

  const double k_dot_{};

  const double s0_spiral_{};

  const double norm_{};

  const maliput::math::Vector2 xy0_spiral_{};

  const double heading0_spiral_{};
  // The value of the p parameter at the start of the arc.
  const double p0_{};
  // The value of the p parameter at the end of the arc.
  const double p1_{};
  // Validates that p is within [p0, p1] with linear_tolerance.
  const maliput::common::RangeValidator validate_p_;
};

}  // namespace road_curve
}  // namespace malidrive
