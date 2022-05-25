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
#include "maliput_malidrive/xodr/tools.h"

#include <cmath>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/lateral_profile.h"

namespace malidrive {
namespace xodr {

using maliput::math::Vector2;

double GetDistanceBetweenGeometries(const Geometry& geometry, const Geometry& rhs) {
  switch (geometry.type) {
    case Geometry::Type::kArc:
    case Geometry::Type::kLine: {
      return (ComputeEndpointWithNewLength(geometry, geometry.length) - rhs.start_point).norm();
    }
    default:
      MALIDRIVE_THROW_MESSAGE("Geometry type could not be recognized.");
  }
}

template <class XodrFunction>
double GetDistanceBetweenFunctions(const XodrFunction& lhs, const XodrFunction& rhs) {
  const double s_0 = rhs.s_0;
  const double s_0_2 = s_0 * s_0;
  const double s_0_3 = s_0 * s_0 * s_0;
  const double lhs_result = lhs.a + lhs.b * s_0 + lhs.c * s_0_2 + lhs.d * s_0_3;
  const double rhs_result = rhs.a + rhs.b * s_0 + rhs.c * s_0_2 + rhs.d * s_0_3;
  return std::abs(lhs_result - rhs_result);
}

Vector2 ComputeEndpointWithNewLength(const Geometry& geometry, double length) {
  MALIPUT_THROW_UNLESS(length > 0.);
  switch (geometry.type) {
    case Geometry::Type::kLine: {
      return geometry.start_point +
             Vector2{std::cos(geometry.orientation) * length, std::sin(geometry.orientation) * length};
    }
    case Geometry::Type::kArc: {
      const double curvature{std::get<xodr::Geometry::Arc>(geometry.description).curvature};
      const double radius{std::abs(1. / curvature)};
      const double d_theta{length * curvature};
      const double theta0 = geometry.orientation - std::copysign(M_PI / 2., curvature);
      const Vector2 center = geometry.start_point - radius * Vector2{std::cos(theta0), std::sin(theta0)};
      const double theta1 = theta0 + d_theta;
      return center + radius * Vector2(std::cos(theta1), std::sin(theta1));
    }
    default:
      MALIDRIVE_THROW_MESSAGE("Geometry type could not be recognized.");
  }
}

template double GetDistanceBetweenFunctions<ElevationProfile::Elevation>(const ElevationProfile::Elevation& lhs,
                                                                         const ElevationProfile::Elevation& rhs);
template double GetDistanceBetweenFunctions<LateralProfile::Superelevation>(const LateralProfile::Superelevation& lhs,
                                                                            const LateralProfile::Superelevation& rhs);

}  // namespace xodr
}  // namespace malidrive
