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
#include "maliput_malidrive/road_curve/line_ground_curve.h"

#include <maliput/math/saturate.h>

namespace malidrive {
namespace road_curve {

maliput::math::Vector2 LineGroundCurve::DoG(double p) const {
  p = validate_p_(p);
  return xy0_ + (p - p0_) / (p1_ - p0_) * dxy_;
}

maliput::math::Vector2 LineGroundCurve::DoGDot(double p) const {
  validate_p_(p);
  return dxy_ / (p1_ - p0_);
}

double LineGroundCurve::DoGInverse(const maliput::math::Vector2& xy) const {
  // unit vector pointing along line from p0 -> p1
  const maliput::math::Vector2& unit_vector = dxy_ / arc_length_;

  const maliput::math::Vector2& xy0_to_xy = xy - xy0_;

  // Compute the distance along the line from the start of the line at p0 to `xy`
  // scaled by the total arc length.
  const double scaled_distance_unsaturated = xy0_to_xy.dot(unit_vector) / arc_length_;
  const double scaled_distance = maliput::math::saturate(scaled_distance_unsaturated, 0., 1.);
  return p0_ + (p1_ - p0_) * scaled_distance;
}

}  // namespace road_curve
}  // namespace malidrive
