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
#include "maliput_malidrive/base/road_geometry.h"

#include <algorithm>
#include <string>
#include <vector>

#include <maliput/geometry_base/brute_force_find_road_positions_strategy.h>
#include <maliput/geometry_base/filter_positions.h>
#include <string_view>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/constants.h"

static constexpr double kEpsilon{1e-12};

namespace {

// Evaluates if `new_road_position_result` provides a closer api::RoadPositionResult than `road_position_result`.
//
// _Closer_ means:
// - When `new_road_position_result.distance` is smaller than
//   `road_position_result.distance` by more than malidrive::constants::kStrictLinearTolerance.
// - When distances are equal, if both positions fall within their
//   respective lane's lane bounds or none do, the new r-coordinate
//   is smaller than `road_position_result.road_position.pos.r()`.
// - When the new position falls within `lane`'s lane bounds and
//   `road_position_result.road_position.pos` doesn't.
//
bool IsNewRoadPositionResultCloser(const maliput::api::RoadPositionResult& new_road_position_result,
                                   const maliput::api::RoadPositionResult& road_position_result) {
  const double delta = new_road_position_result.distance - road_position_result.distance;
  const bool different_segment = road_position_result.road_position.lane->segment()->id() !=
                                 new_road_position_result.road_position.lane->segment()->id();

  // When lanes belong to the same segment is expected that the distance value is almost equal so we can't use the
  // distance as main condition. When lanes don't belong to the same segment we can use the distance as main
  // condition.
  if (different_segment) {
    if (delta < -malidrive::constants::kStrictLinearTolerance) {
      return true;
    }
    if (delta >= malidrive::constants::kStrictLinearTolerance) {
      return false;
    }
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

bool is_less_than_or_close(double a, double b) { return a < b || std::abs(a - b) < kEpsilon; }

bool is_greater_than_or_close(double a, double b) { return a > b || std::abs(a - b) < kEpsilon; }

// Splits a string into a vector of strings using a delimiter.
std::vector<std::string_view> split_string(std::string_view s, std::string_view delimiter) {
  std::vector<std::string_view> result;
  size_t pos = 0;
  while ((pos = s.find(delimiter)) != std::string_view::npos) {
    result.emplace_back(s.substr(0, pos));
    s.remove_prefix(pos + delimiter.size());
  }
  result.emplace_back(s);  // Add the last segment
  return result;
}

// A helper class to hold the commands that the RoadGeometry class can execute.
class CommandsHandler {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(CommandsHandler)

  struct Command {
    std::string_view name{};
    std::vector<std::string_view> args{};
  };

  // Constructor.
  CommandsHandler(const malidrive::RoadGeometry* rg) : rg_(rg) {}

  // Parses a string command into a Command object.
  // The string command should be in the format: "<CommandName>,<arg_1_value>,<arg_2_value>,...,<arg_n_value>"
  Command ParseCommand(const std::string& str_command) {
    static constexpr char const* kArgumentDelimiter{","};
    Command command;
    const std::vector<std::string_view> tokens = split_string(str_command, kArgumentDelimiter);
    MALIDRIVE_THROW_UNLESS(tokens.size() > 0);
    command.name = tokens[0];
    for (size_t i = 1; i < tokens.size(); ++i) {
      command.args.emplace_back(tokens[i]);
    }
    return command;
  }

  // Executes a command and returns the output as a string.
  std::string Execute(const Command& command) {
    if (command.name == "OpenScenarioLanePositionToMaliputRoadPosition") {
      if (command.args.size() != 4) {
        MALIDRIVE_THROW_MESSAGE(std::string("OpenScenarioLanePositionToMaliputRoadPosition expects 4 arguments, got ") +
                                std::to_string(command.args.size()));
      }
      const int xodr_road_id = std::stoi(std::string(command.args[0]));
      const double xodr_s = std::stod(std::string(command.args[1]));
      const int xodr_lane_id = std::stoi(std::string(command.args[2]));
      const double offset = std::stod(std::string(command.args[3]));
      const maliput::api::RoadPosition road_position =
          rg_->OpenScenarioLanePositionToMaliputRoadPosition(xodr_road_id, xodr_s, xodr_lane_id, offset);
      return to_output_format(road_position);
    }
    if (command.name == "OpenScenarioRoadPositionToMaliputRoadPosition") {
      if (command.args.size() != 3) {
        MALIDRIVE_THROW_MESSAGE(std::string("OpenScenarioRoadPositionToMaliputRoadPosition expects 3 arguments, got ") +
                                std::to_string(command.args.size()));
      }
      const int xodr_road_id = std::stoi(std::string(command.args[0]));
      const double xodr_s = std::stod(std::string(command.args[1]));
      const double xodr_t = std::stod(std::string(command.args[2]));
      const maliput::api::RoadPosition road_position =
          rg_->OpenScenarioRoadPositionToMaliputRoadPosition(xodr_road_id, xodr_s, xodr_t);
      return to_output_format(road_position);

    } else {
      MALIDRIVE_THROW_MESSAGE(std::string("Unknown command: ") + std::string(command.name));
    }
  }

 private:
  // Converts a RoadPosition to a string using the format: "<lane_id>,<s>,<r>,<h>"
  std::string to_output_format(const maliput::api::RoadPosition& road_position) {
    return road_position.lane->id().string() + "," + std::to_string(road_position.pos.s()) + "," +
           std::to_string(road_position.pos.r()) + "," + std::to_string(road_position.pos.h());
  }

  const malidrive::RoadGeometry* rg_;
};

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

    // Filter the candidates within a linear tolerance of distance.
    const std::vector<maliput::api::RoadPositionResult> near_road_positions_results =
        maliput::geometry_base::FilterRoadPositionResults(
            road_position_results, [tol = linear_tolerance()](const maliput::api::RoadPositionResult& result) {
              return result.distance <= tol;
            });

    // If it is empty then I should use all the road position results.
    const std::vector<maliput::api::RoadPositionResult>& filtered_road_position_results =
        near_road_positions_results.empty() ? road_position_results : near_road_positions_results;
    result = filtered_road_position_results[0];
    for (const auto& road_position_result : filtered_road_position_results) {
      if (IsNewRoadPositionResultCloser(road_position_result, result)) {
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

maliput::api::RoadPosition RoadGeometry::OpenScenarioLanePositionToMaliputRoadPosition(int xodr_road_id, double xodr_s,
                                                                                       int xodr_lane_id,
                                                                                       double offset) const {
  MALIDRIVE_THROW_UNLESS(xodr_road_id >= 0);
  MALIDRIVE_THROW_UNLESS(xodr_s >= 0.);
  MALIDRIVE_THROW_UNLESS(xodr_lane_id != 0);
  const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*> all_lanes = this->ById().GetLanes();
  const Lane* target_lane{nullptr};
  for (const auto& lane : all_lanes) {
    const Lane* mali_lane = dynamic_cast<const Lane*>(lane.second);
    if (mali_lane->get_track() == xodr_road_id && mali_lane->get_lane_id() == xodr_lane_id) {
      if (is_greater_than_or_close(xodr_s, mali_lane->get_track_s_start()) &&
          is_less_than_or_close(xodr_s, mali_lane->get_track_s_end())) {
        if (target_lane != nullptr) {
          target_lane = target_lane->get_track_s_start() > mali_lane->get_track_s_start() ? target_lane : mali_lane;
        } else {
          target_lane = mali_lane;
        }
      }
    }
  }
  if (target_lane == nullptr) {
    MALIDRIVE_THROW_MESSAGE(
        "A maliput lane can't be found for the given OpenSCENARIO lane position: "
        "RoadID: " +
        std::to_string(xodr_road_id) + ", s: " + std::to_string(xodr_s) + ", LaneID: " + std::to_string(xodr_lane_id) +
        ", offset: " + std::to_string(offset));
  }
  const double mali_lane_s = target_lane->LaneSFromTrackS(xodr_s);
  const auto segment = dynamic_cast<const Segment*>(target_lane->segment());
  const auto road_curve = segment->road_curve();
  const double p = road_curve->PFromP(xodr_s);
  const double roll_at_p = road_curve->superelevation()->f(p);
  const double r = offset / std::cos(roll_at_p);
  const double r_max = target_lane->lane_bounds(mali_lane_s).max();
  const double r_min = target_lane->lane_bounds(mali_lane_s).min();
  if (r_min > r || r_max < r) {
    MALIDRIVE_THROW_MESSAGE(
        "The lane offset is out of bounds for the given OpenSCENARIO lane position: "
        "RoadID: " +
        std::to_string(xodr_road_id) + ", s: " + std::to_string(xodr_s) + ", LaneID: " + std::to_string(xodr_lane_id) +
        ", offset: " + std::to_string(offset) + " | Maliput LaneID: " + target_lane->id().string() +
        ", Maliput Lane's s-coordinate: " + std::to_string(mali_lane_s) + ", Maliput Lane's r-coordinate: " +
        std::to_string(r) + ", Maliput Lane's r-coordinate bounds: [" + std::to_string(r_min) + ", " +
        std::to_string(r_max) + "]" + ", Superelevation at s: " + std::to_string(roll_at_p));
  }
  return maliput::api::RoadPosition{target_lane, maliput::api::LanePosition{mali_lane_s, r, 0.}};
}

maliput::api::RoadPosition RoadGeometry::OpenScenarioRoadPositionToMaliputRoadPosition(int xodr_road_id, double xodr_s,
                                                                                       double xodr_t) const {
  MALIDRIVE_THROW_UNLESS(xodr_road_id >= 0);
  MALIDRIVE_THROW_UNLESS(xodr_s >= 0.);
  const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*> all_lanes = this->ById().GetLanes();
  const Segment* target_segment{nullptr};
  // Identify which is the segment that could contain position determined by xodr_road_id / xodr_s / xodr_t.
  // This only can match only one maliput::api::Segment.
  for (const auto& lane : all_lanes) {
    const Lane* mali_lane = dynamic_cast<const Lane*>(lane.second);
    MALIPUT_THROW_UNLESS(mali_lane != nullptr);

    if (mali_lane->get_track() == xodr_road_id && is_greater_than_or_close(xodr_s, mali_lane->get_track_s_start()) &&
        is_less_than_or_close(xodr_s, mali_lane->get_track_s_end())) {
      target_segment = dynamic_cast<const Segment*>(mali_lane->segment());
      MALIPUT_THROW_UNLESS(target_segment != nullptr);
      break;
    }
  }
  if (target_segment == nullptr) {
    MALIDRIVE_THROW_MESSAGE(
        "A maliput segment can't be found for the given OpenSCENARIO road position: "
        "RoadID: " +
        std::to_string(xodr_road_id) + ", s: " + std::to_string(xodr_s) + ", t: " + std::to_string(xodr_t));
  }
  const double p = target_segment->road_curve()->PFromP(xodr_s);
  const Lane* target_lane{nullptr};
  double r{};
  double mali_lane_s{};
  for (int i = 0; i < target_segment->num_lanes(); ++i) {
    const Lane* mali_lane = dynamic_cast<const Lane*>(target_segment->lane(i));
    // Obtains the r-coordinate in road curve's frame.
    const double r_road_curve = mali_lane->to_reference_r(p, 0.);
    const double width = mali_lane->lane_width_at(p);
    const double r_road_curve_min = r_road_curve - width / 2.;
    const double r_road_curve_max = r_road_curve + width / 2.;
    // r-coordinates are located in RoadCurve FRAME while t-coordinate are respect to Road Reference Line in the ground.
    const double roll_at_p = target_segment->road_curve()->superelevation()->f(p);
    const double xodr_t_projected_on_r = xodr_t / std::cos(roll_at_p);
    if (is_greater_than_or_close(xodr_t_projected_on_r, r_road_curve_min) &&
        is_less_than_or_close(xodr_t_projected_on_r, r_road_curve_max)) {
      // If the lane r value lies the limit of lanes then the right lane is the target lane as lanes are iterated from
      // right to left by definition.
      target_lane = mali_lane;
      r = mali_lane->to_lane_r(p, xodr_t_projected_on_r);
      mali_lane_s = target_lane->LaneSFromTrackS(xodr_s);
      break;
    }
  }
  if (target_lane == nullptr) {
    MALIDRIVE_THROW_MESSAGE(
        "A maliput lane can't be found for the given OpenSCENARIO road position: "
        "RoadID: " +
        std::to_string(xodr_road_id) + ", s: " + std::to_string(xodr_s) + ", t: " + std::to_string(xodr_t));
  }
  return maliput::api::RoadPosition{target_lane, maliput::api::LanePosition{mali_lane_s, r, 0.}};
}

std::string RoadGeometry::DoBackendCustomCommand(const std::string& command) const {
  CommandsHandler commands_handler(this);
  return commands_handler.Execute(commands_handler.ParseCommand(command));
}

}  // namespace malidrive
