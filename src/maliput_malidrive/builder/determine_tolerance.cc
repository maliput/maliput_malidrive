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
#include "maliput_malidrive/builder/determine_tolerance.h"

#include <cmath>

#include <maliput/common/maliput_unused.h>
#include <maliput/math/saturate.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/road_curve/ground_curve.h"
#include "maliput_malidrive/xodr/road_header.h"

namespace malidrive {
namespace builder {

double DetermineRoadGeometryLinearTolerance(const xodr::DBManager* xodr_manager) {
  MALIDRIVE_THROW_UNLESS(xodr_manager);

  const xodr::DBManager::XodrGeometryLengthData& shortest_geometry = xodr_manager->GetShortestGeometry();
  const xodr::DBManager::XodrLaneSectionLengthData& shortest_lane_section = xodr_manager->GetShortestLaneSection();
  const xodr::DBManager::XodrGapBetweenGeometries& largest_geometry_gap = xodr_manager->GetLargestGap();
  const xodr::DBManager::XodrGapBetweenFunctions& largest_elevation_gap = xodr_manager->GetLargestElevationGap();

  const xodr::RoadHeader::Id kRoadHeaderIdNone{"none"};
  // @{ Solves the hard constraint. Linear tolerance must be larger than the larger gap.
  //    When a value is applied, an inflation ratio of 50% is used to avoid numerical
  //    tolerance errors.
  double tolerance = 0;
  if (largest_geometry_gap.road_header_id == kRoadHeaderIdNone &&
      largest_elevation_gap.road_header_id != kRoadHeaderIdNone) {
    tolerance = largest_elevation_gap.distance;
    tolerance = std::max(tolerance * 1.5, kMinLinearTolerance);
  } else if (largest_geometry_gap.road_header_id != kRoadHeaderIdNone &&
             largest_elevation_gap.road_header_id == kRoadHeaderIdNone) {
    tolerance = largest_geometry_gap.distance;
    tolerance = std::max(tolerance * 1.5, kMinLinearTolerance);
  } else if (largest_geometry_gap.road_header_id != kRoadHeaderIdNone &&
             largest_elevation_gap.road_header_id != kRoadHeaderIdNone) {
    tolerance = std::max(largest_geometry_gap.distance, largest_elevation_gap.distance);
    tolerance = std::max(tolerance * 1.5, kMinLinearTolerance);
  }
  // @}

  // @{ When tolerance cannot be defined by using the discontinuities in the xodr,
  //    a soft constraint with a heuristic is used.
  if (tolerance < road_curve::GroundCurve::kEpsilon) {
    const double min_distance = std::min(shortest_geometry.length, shortest_lane_section.length);
    // TODO(agalbachicar): Include elevation and superelevation shortest definitions.
    tolerance = maliput::math::saturate(0.5 * min_distance, kMinLinearTolerance, constants::kLinearTolerance);
  }
  // @}

  return tolerance;
}

double DetermineRoadGeometryAngularTolerance(const xodr::DBManager* xodr_manager) {
  MALIDRIVE_THROW_UNLESS(xodr_manager);
  return constants::kAngularTolerance;
}

double DetermineRoadGeometryScaleLength(const xodr::DBManager* xodr_manager, double linear_tolerance,
                                        double angular_tolerance) {
  MALIDRIVE_THROW_UNLESS(xodr_manager);
  MALIDRIVE_THROW_UNLESS(linear_tolerance > 0.);
  MALIDRIVE_THROW_UNLESS(angular_tolerance > 0.);
  maliput::common::unused(linear_tolerance);
  maliput::common::unused(angular_tolerance);
  return constants::kScaleLength;
}

}  // namespace builder
}  // namespace malidrive
