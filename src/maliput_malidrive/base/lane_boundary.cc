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
#include "maliput_malidrive/base/lane_boundary.h"

#include <algorithm>

#include <maliput/common/maliput_throw.h>

namespace malidrive {

namespace {

// Helper to validate reference_lane and compute s values.
std::pair<double, double> ComputeSRange(const malidrive::Lane* reference_lane, double track_s_start,
                                        double track_s_end) {
  MALIDRIVE_THROW_UNLESS(reference_lane != nullptr);
  return {reference_lane->LaneSFromTrackS(track_s_start), reference_lane->LaneSFromTrackS(track_s_end)};
}

}  // namespace

LaneBoundary::MarkingInterval::MarkingInterval(double s_start_in, double s_end_in)
    : s_start(s_start_in), s_end(s_end_in) {
  MALIDRIVE_THROW_UNLESS(s_start_in <= s_end_in);
}

LaneBoundary::MarkingInterval::MarkingInterval(double s) : s_start(s), s_end(s) {}

bool LaneBoundary::MarkingInterval::operator<(const MarkingInterval& rhs) const {
  // Compare intervals by their start values.
  // For intervals [a1, b1) and [a2, b2), we use a1 < a2 as the primary comparison.
  // This ensures intervals are sorted by their start position in the map.
  return s_start < rhs.s_start;
}

LaneBoundary::LaneBoundary(const maliput::api::LaneBoundary::Id& id, const maliput::api::Segment* segment, int index,
                           const maliput::api::Lane* lane_to_left, const maliput::api::Lane* lane_to_right,
                           const Lane* reference_lane, const std::vector<xodr::LaneRoadMark>& road_marks,
                           double track_s_start, double track_s_end)
    : id_(id),
      segment_(segment),
      index_(index),
      lane_to_left_(lane_to_left),
      lane_to_right_(lane_to_right),
      s_start_(ComputeSRange(reference_lane, track_s_start, track_s_end).first),
      s_end_(ComputeSRange(reference_lane, track_s_start, track_s_end).second),
      interval_marking_map_(BuildIntervalMarkingMap(reference_lane, road_marks, track_s_start, track_s_end)) {
  MALIDRIVE_THROW_UNLESS(reference_lane != nullptr);
  MALIDRIVE_THROW_UNLESS(segment != nullptr);
  MALIDRIVE_THROW_UNLESS(index >= 0);
  MALIDRIVE_THROW_UNLESS(s_end_ >= s_start_);
}

std::map<LaneBoundary::MarkingInterval, maliput::api::LaneMarking> LaneBoundary::BuildIntervalMarkingMap(
    const Lane* reference_lane, const std::vector<xodr::LaneRoadMark>& road_marks, double track_s_start,
    double track_s_end) {
  MALIDRIVE_THROW_UNLESS(reference_lane != nullptr);
  std::map<MarkingInterval, maliput::api::LaneMarking> result_map;

  if (road_marks.empty()) {
    // No markings available - return empty map.
    return result_map;
  }

  // Sort road marks by s_offset for proper range calculation.
  std::vector<xodr::LaneRoadMark> sorted_marks = road_marks;
  std::sort(sorted_marks.begin(), sorted_marks.end(),
            [](const xodr::LaneRoadMark& a, const xodr::LaneRoadMark& b) { return a.s_offset < b.s_offset; });

  // Convert each road mark to a LaneMarkingResult with appropriate s-ranges.
  // Coordinate conversion: TRACK frame (OpenDRIVE) -> LANE frame (maliput)
  // To ensure consistent s-range boundaries between consecutive markings, we first calculate
  // all the converted s values, then assign them to ensure s_end[i] == s_start[i+1].
  std::vector<double> lane_s_boundaries;
  lane_s_boundaries.reserve(sorted_marks.size() + 1);

  // Calculate all boundary s-values in LANE frame.
  for (size_t i = 0; i < sorted_marks.size(); ++i) {
    const double mark_track_s_start = std::max(track_s_start + sorted_marks[i].s_offset, track_s_start);
    lane_s_boundaries.push_back(reference_lane->LaneSFromTrackS(mark_track_s_start));
  }
  // Add the end boundary.
  lane_s_boundaries.push_back(reference_lane->LaneSFromTrackS(track_s_end));

  // Now create the markings with consistent s-ranges and insert into the map.
  for (size_t i = 0; i < sorted_marks.size(); ++i) {
    const auto& road_mark = sorted_marks[i];

    const double mark_lane_s_start = lane_s_boundaries[i];
    const double mark_lane_s_end = lane_s_boundaries[i + 1];

    // Only add if the range is valid.
    if (mark_lane_s_end > mark_lane_s_start) {
      const MarkingInterval interval(mark_lane_s_start, mark_lane_s_end);
      result_map.emplace(interval, ConvertRoadMark(road_mark));
    }
  }

  return result_map;
}

maliput::api::LaneMarking LaneBoundary::ConvertRoadMark(const xodr::LaneRoadMark& road_mark) {
  maliput::api::LaneMarking marking;

  marking.type = ConvertType(road_mark.type);
  marking.weight = ConvertWeight(road_mark.weight);
  marking.color = ConvertColor(road_mark.color);
  marking.lane_change = ConvertLaneChange(road_mark.lane_change);

  // Set width if available.
  if (road_mark.width.has_value()) {
    marking.width = road_mark.width.value();
  }

  // Set height if available.
  if (road_mark.height.has_value()) {
    marking.height = road_mark.height.value();
  }

  // Set material if available.
  if (road_mark.material.has_value()) {
    marking.material = road_mark.material.value();
  }

  // Convert detailed line definitions from type elements if available.
  // Type elements define line patterns with dash/gap specifications.
  for (const auto& type_elem : road_mark.type_elems) {
    for (const auto& line : type_elem.lines) {
      maliput::api::LaneMarkingLine marking_line;
      marking_line.length = line.length;
      marking_line.space = line.space;
      if (line.width.has_value()) {
        marking_line.width = line.width.value();
      }
      marking_line.r_offset = line.t_offset;  // t_offset in XODR corresponds to r_offset in maliput.
      if (line.color.has_value()) {
        marking_line.color = ConvertColor(line.color.value());
      }
      marking.lines.push_back(marking_line);
    }
  }

  // Convert explicit element line definitions if available.
  // Explicit elements define individual line segments with explicit positioning.
  // Note: ExplicitElementLine has s_offset (start position) and length, but no space (gap) field.
  // These represent explicitly positioned line segments rather than repeating dash patterns.
  for (const auto& explicit_elem : road_mark.explicit_elems) {
    for (const auto& line : explicit_elem.lines) {
      maliput::api::LaneMarkingLine marking_line;
      marking_line.length = line.length;
      marking_line.space = 0.;  // Explicit lines don't have a repeating gap pattern.
      if (line.width.has_value()) {
        marking_line.width = line.width.value();
      }
      marking_line.r_offset = line.t_offset;  // t_offset in XODR corresponds to r_offset in maliput.
      // Note: ExplicitElementLine has no color field, use parent marking color.
      marking.lines.push_back(marking_line);
    }
  }

  // Note: Sway elements (road_mark.sway_elems) define polynomial lateral offset variations.
  // These are not currently supported by the maliput LaneMarking API as they represent
  // OpenDRIVE-specific geometric features (lateral position as a polynomial function of s).

  return marking;
}

maliput::api::LaneMarkingType LaneBoundary::ConvertType(xodr::LaneRoadMark::Type type) {
  switch (type) {
    case xodr::LaneRoadMark::Type::kNone:
      return maliput::api::LaneMarkingType::kNone;
    case xodr::LaneRoadMark::Type::kSolid:
      return maliput::api::LaneMarkingType::kSolid;
    case xodr::LaneRoadMark::Type::kBroken:
      return maliput::api::LaneMarkingType::kBroken;
    case xodr::LaneRoadMark::Type::kSolidSolid:
      return maliput::api::LaneMarkingType::kSolidSolid;
    case xodr::LaneRoadMark::Type::kSolidBroken:
      return maliput::api::LaneMarkingType::kSolidBroken;
    case xodr::LaneRoadMark::Type::kBrokenSolid:
      return maliput::api::LaneMarkingType::kBrokenSolid;
    case xodr::LaneRoadMark::Type::kBrokenBroken:
      return maliput::api::LaneMarkingType::kBrokenBroken;
    case xodr::LaneRoadMark::Type::kBottsDots:
      return maliput::api::LaneMarkingType::kBottsDots;
    case xodr::LaneRoadMark::Type::kGrass:
      return maliput::api::LaneMarkingType::kGrass;
    case xodr::LaneRoadMark::Type::kCurb:
      return maliput::api::LaneMarkingType::kCurb;
    case xodr::LaneRoadMark::Type::kEdge:
      return maliput::api::LaneMarkingType::kEdge;
    case xodr::LaneRoadMark::Type::kCustom:
    default:
      return maliput::api::LaneMarkingType::kUnknown;
  }
}

maliput::api::LaneMarkingWeight LaneBoundary::ConvertWeight(std::optional<xodr::LaneRoadMark::Weight> weight) {
  if (!weight.has_value()) {
    return maliput::api::LaneMarkingWeight::kUnknown;
  }
  switch (weight.value()) {
    case xodr::LaneRoadMark::Weight::kStandard:
      return maliput::api::LaneMarkingWeight::kStandard;
    case xodr::LaneRoadMark::Weight::kBold:
      return maliput::api::LaneMarkingWeight::kBold;
    default:
      return maliput::api::LaneMarkingWeight::kUnknown;
  }
}

maliput::api::LaneMarkingColor LaneBoundary::ConvertColor(xodr::Color color) {
  switch (color) {
    case xodr::Color::kWhite:
    case xodr::Color::kStandard:  // Standard typically means white.
      return maliput::api::LaneMarkingColor::kWhite;
    case xodr::Color::kYellow:
      return maliput::api::LaneMarkingColor::kYellow;
    case xodr::Color::kOrange:
      return maliput::api::LaneMarkingColor::kOrange;
    case xodr::Color::kRed:
      return maliput::api::LaneMarkingColor::kRed;
    case xodr::Color::kBlue:
      return maliput::api::LaneMarkingColor::kBlue;
    case xodr::Color::kGreen:
      return maliput::api::LaneMarkingColor::kGreen;
    case xodr::Color::kViolet:
      return maliput::api::LaneMarkingColor::kViolet;
    case xodr::Color::kBlack:
    default:
      return maliput::api::LaneMarkingColor::kUnknown;
  }
}

maliput::api::LaneChangePermission LaneBoundary::ConvertLaneChange(
    std::optional<xodr::LaneRoadMark::LaneChange> lane_change) {
  if (!lane_change.has_value()) {
    // Default to allowed if not specified (OpenDRIVE default is "both").
    return maliput::api::LaneChangePermission::kAllowed;
  }
  switch (lane_change.value()) {
    case xodr::LaneRoadMark::LaneChange::kBoth:
      return maliput::api::LaneChangePermission::kAllowed;
    case xodr::LaneRoadMark::LaneChange::kIncrease:
      return maliput::api::LaneChangePermission::kToLeft;
    case xodr::LaneRoadMark::LaneChange::kDecrease:
      return maliput::api::LaneChangePermission::kToRight;
    case xodr::LaneRoadMark::LaneChange::kNone:
      return maliput::api::LaneChangePermission::kProhibited;
    default:
      return maliput::api::LaneChangePermission::kUnknown;
  }
}

std::optional<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarking(double s) const {
  // Find the interval containing s using half-open interval semantics [s_start, s_end).
  // At exact boundaries between intervals, prefer the interval that starts at s.
  //
  // Due to floating-point precision issues from coordinate conversions (TRACK -> LANE frame),
  // the boundary between two intervals might not be exactly at the expected value.
  // For example, at track_s=50, lane_s might be 50.0000001.
  //
  // When querying at s=50 (the expected boundary), we should return the interval that
  // semantically starts at that point, not the one that ends just past it.
  //
  // Strategy: Use a small tolerance to determine if we're "at" the start of the next interval.
  // The tolerance should be small enough to not affect normal queries but large enough to
  // handle floating-point conversion errors (typically on the order of 1e-9 to 1e-6).
  constexpr double kBoundaryTolerance = 1e-6;

  if (interval_marking_map_.empty()) {
    return std::nullopt;
  }

  // Use upper_bound: returns iterator to first element with s_start > s.
  auto it = interval_marking_map_.upper_bound(MarkingInterval(s));

  // Check if the previous interval contains s.
  if (it != interval_marking_map_.begin()) {
    auto prev_it = std::prev(it);
    if (prev_it->first.s_start <= s && s < prev_it->first.s_end) {
      // s is in the previous interval. But check if we're at/near the boundary
      // and there's a next interval that starts very close to s.
      // If so, prefer the next interval (half-open interval semantics: boundary goes to next).
      if (it != interval_marking_map_.end() && (it->first.s_start - s) <= kBoundaryTolerance) {
        // s is at or very close to the start of the next interval - return next interval.
        maliput::api::LaneMarkingResult result;
        result.marking = it->second;
        result.s_start = it->first.s_start;
        result.s_end = it->first.s_end;
        return result;
      }
      // Return the previous interval.
      maliput::api::LaneMarkingResult result;
      result.marking = prev_it->second;
      result.s_start = prev_it->first.s_start;
      result.s_end = prev_it->first.s_end;
      return result;
    }
  }

  // Check if s is at/in the first interval (when upper_bound returns begin or first element).
  if (it != interval_marking_map_.end() && it->first.s_start <= s && s < it->first.s_end) {
    maliput::api::LaneMarkingResult result;
    result.marking = it->second;
    result.s_start = it->first.s_start;
    result.s_end = it->first.s_end;
    return result;
  }

  // Handle the last interval's end boundary (inclusive for the very end).
  auto last = std::prev(interval_marking_map_.end());
  if (s >= last->first.s_start && s <= last->first.s_end) {
    maliput::api::LaneMarkingResult result;
    result.marking = last->second;
    result.s_start = last->first.s_start;
    result.s_end = last->first.s_end;
    return result;
  }

  return std::nullopt;
}

std::vector<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarkings() const {
  std::vector<maliput::api::LaneMarkingResult> results;
  results.reserve(interval_marking_map_.size());
  for (const auto& [interval, marking] : interval_marking_map_) {
    maliput::api::LaneMarkingResult result;
    result.marking = marking;
    result.s_start = interval.s_start;
    result.s_end = interval.s_end;
    results.push_back(result);
  }
  return results;
}

std::vector<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarkings(double s_start, double s_end) const {
  MALIPUT_THROW_UNLESS(s_start <= s_end);

  std::vector<maliput::api::LaneMarkingResult> results;
  for (const auto& [interval, marking] : interval_marking_map_) {
    // Check if the marking overlaps with the requested range.
    if (interval.s_end > s_start && interval.s_start < s_end) {
      maliput::api::LaneMarkingResult result;
      result.marking = marking;
      result.s_start = interval.s_start;
      result.s_end = interval.s_end;
      results.push_back(result);
    }
  }
  return results;
}

}  // namespace malidrive
