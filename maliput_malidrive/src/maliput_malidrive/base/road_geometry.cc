// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/base/road_geometry.h"

#include <algorithm>

#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"

namespace {

// Evaluates if `new_road_position_result` provides a closer api::RoadPositionResult than `road_position_result`.
//
// _Closer_ means:
// - When `new_road_position_result.distance` is smaller than
//   `road_position_result.distance` by more than `linear_tolerance`.
// - When distances are equal, if both positions fall within their
//   respective lane's lane bounds or none do, the new r-coordinate
//   is smaller than `road_position_result.road_position.pos.r()`.
// - When the new position falls within `lane`'s lane bounds and
//   `road_position_result.road_position.pos` doesn't.
//
bool IsNewRoadPositionResultCloser(const maliput::api::RoadPositionResult& new_road_position_result,
                                   const maliput::api::RoadPositionResult& road_position_result,
                                   double linear_tolerance) {
  const double delta = new_road_position_result.distance - road_position_result.distance;
  if (delta > linear_tolerance) {
    return false;
  }
  if (delta < -linear_tolerance) {
    return true;
  }

  auto is_within_lane_bounds = [](double r, const maliput::api::RBounds& lane_bounds) {
    return r >= lane_bounds.min() && r < lane_bounds.max();
  };
  // They are almost equal so it is worth checking the `r` coordinate and the
  // lane bounds.
  // When both r-coordinates fall within lane bounds or outside, the position
  // with the minimum absolute r-coordinate prevails.
  // When the new r-coordinate is within lane bounds, and the previous position
  // does not fall within lane bounds, the new result prevails.
  const maliput::api::RBounds new_lane_bounds =
      new_road_position_result.road_position.lane->lane_bounds(new_road_position_result.road_position.pos.s());
  const maliput::api::RBounds current_lane_bounds =
      road_position_result.road_position.lane->lane_bounds(road_position_result.road_position.pos.s());
  const bool is_new_within_lane_bounds =
      is_within_lane_bounds(new_road_position_result.road_position.pos.r(), new_lane_bounds);
  const bool is_current_within_lane_bounds =
      is_within_lane_bounds(road_position_result.road_position.pos.r(), current_lane_bounds);
  if ((is_new_within_lane_bounds && is_current_within_lane_bounds) ||
      (!is_new_within_lane_bounds && !is_current_within_lane_bounds)) {
    if (std::abs(new_road_position_result.road_position.pos.r()) <
        std::abs(road_position_result.road_position.pos.r())) {
      return true;
    }
  } else if (is_new_within_lane_bounds && !is_current_within_lane_bounds) {
    return true;
  }
  return false;
}

}  // namespace

namespace malidrive {

void RoadGeometry::AddRoadCharacteristics(const xodr::RoadHeader::Id& road_id,
                                          std::unique_ptr<road_curve::RoadCurve> road_curve,
                                          std::unique_ptr<road_curve::Function> reference_line_offset) {
  MALIDRIVE_THROW_UNLESS(road_curve != nullptr);
  MALIDRIVE_THROW_UNLESS(reference_line_offset != nullptr);
  if (road_characteristics_.find(road_id) != road_characteristics_.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("Duplicated Road characteristics for RoadID: ") + road_id.string());
  }
  road_characteristics_.emplace(road_id, RoadCharacteristics{std::move(road_curve), std::move(reference_line_offset)});
}

const road_curve::RoadCurve* RoadGeometry::GetRoadCurve(const xodr::RoadHeader::Id& road_id) const {
  if (road_characteristics_.find(road_id) == road_characteristics_.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("There is no RoadCurve for RoadID: ") + road_id.string());
  }
  return road_characteristics_.at(road_id).road_curve.get();
}

const road_curve::Function* RoadGeometry::GetReferenceLineOffset(const xodr::RoadHeader::Id& road_id) const {
  if (road_characteristics_.find(road_id) == road_characteristics_.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("There is no reference line offset function for RoadID: ") + road_id.string());
  }
  return road_characteristics_.at(road_id).reference_line_offset.get();
}

maliput::api::RoadPositionResult RoadGeometry::DoToRoadPosition(
    const maliput::api::InertialPosition& inertial_pos, const std::optional<maliput::api::RoadPosition>& hint) const {
  maliput::api::RoadPositionResult result;
  if (hint.has_value()) {
    MALIDRIVE_THROW_UNLESS(hint->lane != nullptr);
    const maliput::api::LanePositionResult lane_pos = hint->lane->ToLanePosition(inertial_pos);
    result = maliput::api::RoadPositionResult{
        {hint->lane, lane_pos.lane_position}, lane_pos.nearest_position, lane_pos.distance};
  } else {
    const std::vector<maliput::api::RoadPositionResult> road_position_results =
        DoFindRoadPositions(inertial_pos, std::numeric_limits<double>::infinity());
    MALIDRIVE_THROW_UNLESS(road_position_results.size());
    result = road_position_results[0];
    for (const auto& road_position_result : road_position_results) {
      if (IsNewRoadPositionResultCloser(road_position_result, result, linear_tolerance())) {
        result = road_position_result;
      }
    }
  }
  return result;
}

std::vector<maliput::api::RoadPositionResult> RoadGeometry::DoFindRoadPositions(
    const maliput::api::InertialPosition& inertial_position, double radius) const {
  return maliput::geometry_base::BruteForceFindRoadPositionsStrategy(this, inertial_position, radius);
}

}  // namespace malidrive