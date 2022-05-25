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

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/lateral_profile.h"
#include "maliput_malidrive/xodr/plan_view.h"

namespace malidrive {
namespace xodr {

/// Holds the geometry description of a XODR road. It gathers the info from XODR nodes:
/// - `planView`.
/// - `elevationProfile`.
/// - `lateralProfile`.
struct ReferenceGeometry {
  /// Equality operator.
  bool operator==(const ReferenceGeometry& other) const {
    return plan_view == other.plan_view && elevation_profile == other.elevation_profile &&
           lateral_profile == other.lateral_profile;
  }
  /// Inequality operator.
  bool operator!=(const ReferenceGeometry& other) const {
    return plan_view != other.plan_view || elevation_profile != other.elevation_profile ||
           lateral_profile != other.lateral_profile;
  }

  /// Contains the geometries which define the layout of the road's
  /// reference line in the x/y-plane.
  PlanView plan_view{};

  /// Contains a series of elevation records which define the characteristics of
  /// the road's elevation along the reference line.
  ElevationProfile elevation_profile{};

  /// Contains a series of superelevation records which define the characteristics of
  /// the road's lateral profile along the reference line.
  LateralProfile lateral_profile{};
};

}  // namespace xodr
}  // namespace malidrive
