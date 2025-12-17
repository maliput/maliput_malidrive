// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota. All rights reserved.
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

#include <map>
#include <optional>
#include <vector>

#include <maliput/api/lane_boundary.h>
#include <maliput/api/lane_marking.h>
#include <maliput/geometry_base/lane_boundary.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/lane_road_mark.h"

namespace malidrive {

/// malidrive's implementation of maliput::api::LaneBoundary.
///
/// A LaneBoundary represents the interface between two adjacent lanes or
/// between a lane and the segment edge. It provides lane marking information
/// from OpenDRIVE road marking data.
///
/// This implementation converts XODR LaneRoadMark data to maliput's LaneMarking
/// format, handling the mapping between OpenDRIVE's marking types, colors,
/// weights, and lane change rules to the maliput equivalents.
class LaneBoundary : public maliput::geometry_base::LaneBoundary {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneBoundary)

  /// Constructs a LaneBoundary.
  ///
  /// @param id The unique identifier for this boundary.
  /// @param segment The parent Segment. It must not be nullptr.
  /// @param index The index of this boundary within the parent Segment.
  /// @param lane_to_left The lane on the left side of this boundary (increasing r),
  ///        or nullptr if this is the leftmost boundary.
  /// @param lane_to_right The lane on the right side of this boundary (decreasing r),
  ///        or nullptr if this is the rightmost boundary.
  /// @param reference_lane The lane used for coordinate conversion from TRACK to LANE frame.
  ///        Must not be nullptr.
  /// @param road_marks The OpenDRIVE road marking data for this boundary.
  /// @param track_s_start The start s-coordinate of the boundary in TRACK frame (OpenDRIVE).
  /// @param track_s_end The end s-coordinate of the boundary in TRACK frame (OpenDRIVE).
  ///
  /// @throws maliput::common::assertion_error When @p segment is nullptr.
  /// @throws maliput::common::assertion_error When @p reference_lane is nullptr.
  /// @throws maliput::common::assertion_error When @p index is negative.
  /// @throws maliput::common::assertion_error When @p track_s_end < @p track_s_start.
  LaneBoundary(const maliput::api::LaneBoundary::Id& id, const maliput::api::Segment* segment, int index,
               const maliput::api::Lane* lane_to_left, const maliput::api::Lane* lane_to_right,
               const Lane* reference_lane, const std::vector<xodr::LaneRoadMark>& road_marks, double track_s_start,
               double track_s_end);

  ~LaneBoundary() override = default;

 private:
  /// Holds the s parameter range of a marking interval.
  /// Similar to road_curve::PiecewiseGroundCurve::RoadCurveInterval.
  /// TODO(francocipollone): Create only one struct that works for both marking intervals and road curve intervals.
  struct MarkingInterval {
    /// Creates a MarkingInterval.
    /// @param s_start_in Is the start s-coordinate of the interval.
    /// @param s_end_in Is the end s-coordinate of the interval.
    /// @throws maliput::common::assertion_error When `s_start_in` is greater than `s_end_in`.
    MarkingInterval(double s_start_in, double s_end_in);

    /// Creates a MarkingInterval where the start value is equal to the end value.
    /// Used for point queries in std::map::find().
    /// @param s Is the s-coordinate to query.
    explicit MarkingInterval(double s);

    const double s_start{};
    const double s_end{};

    /// Less than operator for ordering intervals.
    /// Enables std::map lookup where a point query [s, s] will match
    /// the interval [s_start, s_end) that contains s.
    bool operator<(const MarkingInterval& rhs) const;
  };

  /// Builds a map from MarkingInterval to LaneMarking from XODR road mark data.
  /// Performs coordinate conversion from TRACK frame (OpenDRIVE) to LANE frame (maliput).
  ///
  /// @param reference_lane The lane used for coordinate conversion.
  /// @param road_marks The XODR road marking data to convert.
  /// @param track_s_start The start s-coordinate in TRACK frame.
  /// @param track_s_end The end s-coordinate in TRACK frame.
  /// @returns A map from MarkingInterval to LaneMarking for efficient lookup.
  static std::map<MarkingInterval, maliput::api::LaneMarking> BuildIntervalMarkingMap(
      const Lane* reference_lane, const std::vector<xodr::LaneRoadMark>& road_marks, double track_s_start,
      double track_s_end);

  /// Converts a single XODR LaneRoadMark to maliput LaneMarking format.
  ///
  /// @param road_mark The XODR road marking to convert.
  /// @returns The converted LaneMarking.
  static maliput::api::LaneMarking ConvertRoadMark(const xodr::LaneRoadMark& road_mark);

  /// Converts XODR marking type to maliput LaneMarkingType.
  static maliput::api::LaneMarkingType ConvertType(xodr::LaneRoadMark::Type type);

  /// Converts XODR marking weight to maliput LaneMarkingWeight.
  static maliput::api::LaneMarkingWeight ConvertWeight(std::optional<xodr::LaneRoadMark::Weight> weight);

  /// Converts XODR color to maliput LaneMarkingColor.
  static maliput::api::LaneMarkingColor ConvertColor(xodr::Color color);

  /// Converts XODR lane change to maliput LaneChangePermission.
  static maliput::api::LaneChangePermission ConvertLaneChange(
      std::optional<xodr::LaneRoadMark::LaneChange> lane_change);

  // maliput::api::LaneBoundary private virtual method implementations.
  maliput::api::LaneBoundary::Id do_id() const override { return id_; }
  const maliput::api::Segment* do_segment() const override { return segment_; }
  int do_index() const override { return index_; }
  const maliput::api::Lane* do_lane_to_left() const override { return lane_to_left_; }
  const maliput::api::Lane* do_lane_to_right() const override { return lane_to_right_; }
  std::optional<maliput::api::LaneMarkingResult> DoGetMarking(double s) const override;
  std::vector<maliput::api::LaneMarkingResult> DoGetMarkings() const override;
  std::vector<maliput::api::LaneMarkingResult> DoGetMarkings(double s_start, double s_end) const override;

  const maliput::api::LaneBoundary::Id id_;
  const maliput::api::Segment* segment_{};
  const int index_{};
  const maliput::api::Lane* lane_to_left_{};
  const maliput::api::Lane* lane_to_right_{};
  const double s_start_{};
  const double s_end_{};

  /// Map from MarkingInterval to LaneMarking for efficient s-coordinate lookup.
  std::map<MarkingInterval, maliput::api::LaneMarking> interval_marking_map_;
};

}  // namespace malidrive
