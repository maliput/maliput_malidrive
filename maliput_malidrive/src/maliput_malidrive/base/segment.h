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
#include <utility>
#include <vector>

#include <maliput/geometry_base/segment.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/road_curve/road_curve_offset.h"

namespace malidrive {

/// Segment implementation that holds a road_curve::RoadCurve.
///
/// Child MalidriveLanes of this segment will be constructed as offsets of the
/// reference curve.
class Segment : public maliput::geometry_base::Segment {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a Segment.
  /// The incidence region of the Segment on the @p road_curve will be
  /// delimited by the range composed by @p p0 and @p p1.
  ///
  /// @param id The id of the Segment.
  /// @param road_curve The reference curve of the Segment shared by all Lanes.
  ///        It must not be nullptr.
  /// @param reference_line_offset The road reference line offset function of the Segment shared by all Lanes.
  ///        It must not be nullptr.
  /// @param p0 The value of the @f$ p @f$ parameter of @p road_curve that matches the start of the Segment.
  /// @param p1 The value of the @f$ p @f$ parameter of @p road_curve that matches the finish of the Segment.
  ///
  /// @throws maliput::common::assertion_error When @p road_curve is nullptr.
  /// @throws maliput::common::assertion_error When @p reference_line_offset is nullptr.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p0 is greater than @p p1.
  Segment(const maliput::api::SegmentId& id, const road_curve::RoadCurve* road_curve,
          const road_curve::Function* reference_line_offset, double p0, double p1)
      : maliput::geometry_base::Segment(id),
        road_curve_(road_curve),
        reference_line_offset_(reference_line_offset),
        p0_(p0),
        p1_(p1) {
    MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr);
    MALIDRIVE_THROW_UNLESS(reference_line_offset_ != nullptr);
    MALIDRIVE_THROW_UNLESS(p1 >= 0.);
    MALIDRIVE_THROW_UNLESS(p1 >= p0);
  }

  /// @returns The lower bound range of @f$ p @f$.
  double p0() const { return p0_; }

  /// @returns The upper bound range of @f$ p @f$.
  double p1() const { return p1_; }

  /// @return The reference curve.
  const road_curve::RoadCurve* road_curve() const { return road_curve_; }

  /// @return The reference line offset function.
  const road_curve::Function* reference_line_offset() const { return reference_line_offset_; }

  /// Adds @p lane to the segment.
  /// @details If `hide_lane` is false, the call is forwarded to maliput::geometry_base::Segment::AddLane.
  /// If `hide_lane` is true, the segment becomes the lane's owner but the lane is hidden.
  /// This behavior is needed to correctly compute the offset of the lanes when
  /// the non-drivable lanes are omitted from the Road Network.
  /// @returns `lane`s raw pointer.
  Lane* AddLane(std::unique_ptr<Lane> lane, bool hide_lane) {
    if (!hide_lane) {
      return maliput::geometry_base::Segment::AddLane(std::move(lane));
    }
    Lane* raw_pointer = lane.get();
    hidden_lanes_.push_back(std::move(lane));
    return raw_pointer;
  }

 private:
  const road_curve::RoadCurve* road_curve_;
  const road_curve::Function* reference_line_offset_;
  const double p0_{};
  const double p1_{};
  // When RoadCurveConfiguration::omit_nondriveable_lanes is true the non-drivable lanes should be omitted
  // but their creation is neccesary to be consistent with other lanes.
  std::vector<std::unique_ptr<Lane>> hidden_lanes_;
};

}  // namespace malidrive
