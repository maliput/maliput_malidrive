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

#include <maliput/api/lane_data.h>
#include <maliput/geometry_base/brute_force_find_road_positions_strategy.h>
#include <maliput/geometry_base/filter_positions.h>
#include <string_view>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/constants.h"

using maliput::api::LaneEnd;

static constexpr double kEpsilon{1e-12};

namespace {

const malidrive::Lane* ToMalidrive(const maliput::api::Lane* lane) {
  const malidrive::Lane* mali_lane = dynamic_cast<const malidrive::Lane*>(lane);
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  return mali_lane;
}

const malidrive::Segment* ToMalidrive(const maliput::api::Segment* segment) {
  const malidrive::Segment* mali_segment = dynamic_cast<const malidrive::Segment*>(segment);
  MALIDRIVE_THROW_UNLESS(segment != nullptr);
  return mali_segment;
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
      const malidrive::RoadGeometry::OpenScenarioLanePosition xodr_lane_position{
          std::stoi(std::string(command.args[0])), std::stod(std::string(command.args[1])),
          std::stoi(std::string(command.args[2])), std::stod(std::string(command.args[3]))};
      const maliput::api::RoadPosition road_position =
          rg_->OpenScenarioLanePositionToMaliputRoadPosition(xodr_lane_position);
      return to_output_format(road_position);
    } else if (command.name == "OpenScenarioRoadPositionToMaliputRoadPosition") {
      if (command.args.size() != 3) {
        MALIDRIVE_THROW_MESSAGE(std::string("OpenScenarioRoadPositionToMaliputRoadPosition expects 3 arguments, got ") +
                                std::to_string(command.args.size()));
      }
      const malidrive::RoadGeometry::OpenScenarioRoadPosition xodr_road_position{
          std::stoi(std::string(command.args[0])), std::stod(std::string(command.args[1])),
          std::stod(std::string(command.args[2]))};
      const maliput::api::RoadPosition road_position =
          rg_->OpenScenarioRoadPositionToMaliputRoadPosition(xodr_road_position);
      return to_output_format(road_position);
    } else if (command.name == "MaliputRoadPositionToOpenScenarioLanePosition") {
      if (command.args.size() != 4) {
        MALIDRIVE_THROW_MESSAGE(std::string("MaliputRoadPositionToOpenScenarioLanePosition expects 4 arguments, got ") +
                                std::to_string(command.args.size()));
      }
      const maliput::api::LaneId lane_id(std::string(command.args[0]));
      const double s = std::stod(std::string(command.args[1]));
      const double r = std::stod(std::string(command.args[2]));
      const double h = std::stod(std::string(command.args[3]));
      const maliput::api::RoadPosition road_position =
          maliput::api::RoadPosition{rg_->ById().GetLane(lane_id), maliput::api::LanePosition{s, r, h}};
      const malidrive::RoadGeometry::OpenScenarioLanePosition xodr_lane_position =
          rg_->MaliputRoadPositionToOpenScenarioLanePosition(road_position);
      return to_output_format(xodr_lane_position);
    } else if (command.name == "MaliputRoadPositionToOpenScenarioRoadPosition") {
      if (command.args.size() != 4) {
        MALIDRIVE_THROW_MESSAGE(std::string("MaliputRoadPositionToOpenScenarioRoadPosition expects 4 arguments, got ") +
                                std::to_string(command.args.size()));
      }
      const maliput::api::LaneId lane_id(std::string(command.args[0]));
      const double s = std::stod(std::string(command.args[1]));
      const double r = std::stod(std::string(command.args[2]));
      const double h = std::stod(std::string(command.args[3]));
      const maliput::api::RoadPosition road_position =
          maliput::api::RoadPosition{rg_->ById().GetLane(lane_id), maliput::api::LanePosition{s, r, h}};
      const malidrive::RoadGeometry::OpenScenarioRoadPosition xodr_road_position =
          rg_->MaliputRoadPositionToOpenScenarioRoadPosition(road_position);
      return to_output_format(xodr_road_position);
    } else if (command.name == "OpenScenarioRelativeRoadPositionToMaliputRoadPosition") {
      if (command.args.size() != 5) {
        MALIDRIVE_THROW_MESSAGE(
            std::string("OpenScenarioRelativeRoadPositionToMaliputRoadPosition expects 5 arguments, got ") +
            std::to_string(command.args.size()));
      }
      const malidrive::RoadGeometry::OpenScenarioRoadPosition xodr_road_position{
          std::stoi(std::string(command.args[0])), std::stod(std::string(command.args[1])),
          std::stod(std::string(command.args[2]))};
      const double ds = std::stod(std::string(command.args[3]));
      const double dt = std::stod(std::string(command.args[4]));
      const maliput::api::RoadPosition road_position =
          rg_->OpenScenarioRelativeRoadPositionToMaliputRoadPosition(xodr_road_position, ds, dt);
      return to_output_format(road_position);
    } else if (command.name == "OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition") {
      if (command.args.size() != 6) {
        MALIDRIVE_THROW_MESSAGE(
            std::string("OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition expects 6 arguments, got ") +
            std::to_string(command.args.size()));
      }
      const malidrive::RoadGeometry::OpenScenarioLanePosition xodr_lane_position{
          std::stoi(std::string(command.args[0])),
          std::stod(std::string(command.args[2])),
          std::stoi(std::string(command.args[1])),
          0.,
      };
      const double d_lane = std::stod(std::string(command.args[3]));
      const double xodr_ds = std::stod(std::string(command.args[4]));
      const double offset = std::stod(std::string(command.args[5]));
      const maliput::api::RoadPosition road_position =
          rg_->OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(xodr_lane_position, d_lane, xodr_ds, offset);
      return to_output_format(road_position);
    } else if (command.name == "OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition") {
      if (command.args.size() != 6) {
        MALIDRIVE_THROW_MESSAGE(
            std::string("OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition expects 6 arguments, got ") +
            std::to_string(command.args.size()));
      }
      const malidrive::RoadGeometry::OpenScenarioLanePosition xodr_lane_position{
          std::stoi(std::string(command.args[0])),
          std::stod(std::string(command.args[2])),
          std::stoi(std::string(command.args[1])),
          0.,
      };
      const double d_lane = std::stod(std::string(command.args[3]));
      const double ds_lane = std::stod(std::string(command.args[4]));
      const double offset = std::stod(std::string(command.args[5]));
      const maliput::api::RoadPosition road_position =
          rg_->OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(xodr_lane_position, d_lane, ds_lane,
                                                                               offset);
      return to_output_format(road_position);
    } else if (command.name == "GetRoadOrientationAtOpenScenarioRoadPosition") {
      if (command.args.size() != 3) {
        MALIDRIVE_THROW_MESSAGE(std::string("GetRoadOrientationAtOpenScenarioRoadPosition expects 3 arguments, got ") +
                                std::to_string(command.args.size()));
      }
      const malidrive::RoadGeometry::OpenScenarioRoadPosition xodr_road_position{
          std::stoi(std::string(command.args[0])), std::stod(std::string(command.args[1])),
          std::stod(std::string(command.args[2]))};
      const maliput::math::RollPitchYaw rotation =
          rg_->GetRoadOrientationAtOpenScenarioRoadPosition(xodr_road_position);
      return to_output_format(rotation);
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

  // Converts an OpenScenarioLanePosition to a string using the format:
  // "<xodr_road_id>,<xodr_s>,<xodr_lane_id>,<offset>"
  std::string to_output_format(const malidrive::RoadGeometry::OpenScenarioLanePosition& lane_position) {
    return std::to_string(lane_position.road_id) + "," + std::to_string(lane_position.s) + "," +
           std::to_string(lane_position.lane_id) + "," + std::to_string(lane_position.offset);
  }

  // Converts a RollPitchYaw to a string using the format: "<roll>,<pitch>,<yaw>"
  std::string to_output_format(const maliput::math::RollPitchYaw& rotation) {
    return std::to_string(rotation.roll_angle()) + "," + std::to_string(rotation.pitch_angle()) + "," +
           std::to_string(rotation.yaw_angle());
  }

  // Converts an OpenScenarioRoadPosition to a string using the format: "<xodr_road_id>,<xodr_s>,<xodr_t>"
  std::string to_output_format(const malidrive::RoadGeometry::OpenScenarioRoadPosition& road_position) {
    return std::to_string(road_position.road_id) + "," + std::to_string(road_position.s) + "," +
           std::to_string(road_position.t);
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

const Segment* RoadGeometry::FindSegmentByOpenScenarioRoadPosition(
    const OpenScenarioRoadPosition& xodr_road_position) const {
  const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*> all_lanes = this->ById().GetLanes();
  const Segment* target_segment{nullptr};
  // Identify which is the segment that could contain position determined by xodr_road_id / xodr_s / xodr_t.
  // This only can match only one maliput::api::Segment.
  for (const auto& lane : all_lanes) {
    const Lane* mali_lane = dynamic_cast<const Lane*>(lane.second);
    MALIPUT_THROW_UNLESS(mali_lane != nullptr);

    if (mali_lane->get_track() == xodr_road_position.road_id &&
        is_greater_than_or_close(xodr_road_position.s, mali_lane->get_track_s_start()) &&
        is_less_than_or_close(xodr_road_position.s, mali_lane->get_track_s_end())) {
      target_segment = dynamic_cast<const Segment*>(mali_lane->segment());
      MALIPUT_THROW_UNLESS(target_segment != nullptr);
      break;
    }
  }
  if (target_segment == nullptr) {
    MALIDRIVE_THROW_MESSAGE(
        "A maliput segment can't be found for the given OpenSCENARIO road position: "
        "RoadID: " +
        std::to_string(xodr_road_position.road_id) + ", s: " + std::to_string(xodr_road_position.s) +
        ", t: " + std::to_string(xodr_road_position.t));
  }
  return target_segment;
}

const Lane* RoadGeometry::GetMaliputLaneFromOpenScenarioLanePosition(
    const OpenScenarioLanePosition& xodr_lane_position) const {
  MALIDRIVE_THROW_UNLESS(xodr_lane_position.road_id >= 0);
  MALIDRIVE_THROW_UNLESS(xodr_lane_position.s >= 0.);
  MALIDRIVE_THROW_UNLESS(xodr_lane_position.lane_id != 0);
  const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*> all_lanes = this->ById().GetLanes();
  const Lane* target_lane{nullptr};
  for (const auto& lane : all_lanes) {
    const Lane* mali_lane = dynamic_cast<const Lane*>(lane.second);
    if (mali_lane->get_track() == xodr_lane_position.road_id &&
        mali_lane->get_lane_id() == xodr_lane_position.lane_id) {
      const auto segment = dynamic_cast<const Segment*>(mali_lane->segment());
      MALIPUT_THROW_UNLESS(segment != nullptr);
      const double xodr_lane_position_s_constrained = segment->road_curve()->PFromP(xodr_lane_position.s);
      if (is_greater_than_or_close(xodr_lane_position_s_constrained, mali_lane->get_track_s_start()) &&
          is_less_than_or_close(xodr_lane_position_s_constrained, mali_lane->get_track_s_end())) {
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
        std::to_string(xodr_lane_position.road_id) + ", s: " + std::to_string(xodr_lane_position.s) + ", LaneID: " +
        std::to_string(xodr_lane_position.lane_id) + ", offset: " + std::to_string(xodr_lane_position.offset));
  }
  return target_lane;
}

maliput::api::RoadPosition RoadGeometry::OpenScenarioLanePositionToMaliputRoadPosition(
    const OpenScenarioLanePosition& xodr_lane_position) const {
  MALIDRIVE_THROW_UNLESS(xodr_lane_position.road_id >= 0);
  MALIDRIVE_THROW_UNLESS(xodr_lane_position.s >= 0.);
  MALIDRIVE_THROW_UNLESS(xodr_lane_position.lane_id != 0);
  const Lane* target_lane = GetMaliputLaneFromOpenScenarioLanePosition(xodr_lane_position);
  const double mali_lane_s = target_lane->LaneSFromTrackS(xodr_lane_position.s);
  const auto segment = dynamic_cast<const Segment*>(target_lane->segment());
  const auto road_curve = segment->road_curve();
  const double p = road_curve->PFromP(xodr_lane_position.s);
  const double roll_at_p = road_curve->superelevation()->f(p);
  const double r = xodr_lane_position.offset / std::cos(roll_at_p);
  const double r_max = target_lane->lane_bounds(mali_lane_s).max();
  const double r_min = target_lane->lane_bounds(mali_lane_s).min();
  if (r_min > r || r_max < r) {
    MALIDRIVE_THROW_MESSAGE(
        "The lane offset is out of bounds for the given OpenSCENARIO lane position: "
        "RoadID: " +
        std::to_string(xodr_lane_position.road_id) + ", s: " + std::to_string(xodr_lane_position.s) +
        ", LaneID: " + std::to_string(xodr_lane_position.lane_id) +
        ", offset: " + std::to_string(xodr_lane_position.offset) + " | Maliput LaneID: " + target_lane->id().string() +
        ", Maliput Lane's s-coordinate: " + std::to_string(mali_lane_s) + ", Maliput Lane's r-coordinate: " +
        std::to_string(r) + ", Maliput Lane's r-coordinate bounds: [" + std::to_string(r_min) + ", " +
        std::to_string(r_max) + "]" + ", Superelevation at s: " + std::to_string(roll_at_p));
  }
  return maliput::api::RoadPosition{target_lane, maliput::api::LanePosition{mali_lane_s, r, 0.}};
}

RoadGeometry::OpenScenarioLanePosition RoadGeometry::MaliputRoadPositionToOpenScenarioLanePosition(
    const maliput::api::RoadPosition& road_position) const {
  OpenScenarioLanePosition xodr_lane_position;
  const auto mali_lane = dynamic_cast<const Lane*>(road_position.lane);
  xodr_lane_position.road_id = mali_lane->get_track();
  xodr_lane_position.lane_id = mali_lane->get_lane_id();
  xodr_lane_position.s = mali_lane->TrackSFromLaneS(road_position.pos.s());
  const auto segment = dynamic_cast<const Segment*>(mali_lane->segment());
  const auto roll_at_p = segment->road_curve()->superelevation()->f(xodr_lane_position.s);
  xodr_lane_position.offset = road_position.pos.r() * std::cos(roll_at_p);
  return xodr_lane_position;
}

maliput::api::RoadPosition RoadGeometry::OpenScenarioRoadPositionToMaliputRoadPosition(
    const OpenScenarioRoadPosition& xodr_road_position) const {
  MALIDRIVE_THROW_UNLESS(xodr_road_position.road_id >= 0);
  MALIDRIVE_THROW_UNLESS(xodr_road_position.s >= 0.);
  const Segment* target_segment{this->FindSegmentByOpenScenarioRoadPosition(xodr_road_position)};
  MALIPUT_THROW_UNLESS(target_segment != nullptr);
  const double p = target_segment->road_curve()->PFromP(xodr_road_position.s);
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
    const double xodr_t_projected_on_r = xodr_road_position.t / std::cos(roll_at_p);
    if (is_greater_than_or_close(xodr_t_projected_on_r, r_road_curve_min) &&
        is_less_than_or_close(xodr_t_projected_on_r, r_road_curve_max)) {
      // If the lane r value lies the limit of lanes then the right lane is the target lane as lanes are iterated from
      // right to left by definition.
      target_lane = mali_lane;
      r = mali_lane->to_lane_r(p, xodr_t_projected_on_r);
      mali_lane_s = target_lane->LaneSFromTrackS(xodr_road_position.s);
      break;
    }
  }
  if (target_lane == nullptr) {
    MALIDRIVE_THROW_MESSAGE(
        "A maliput lane can't be found for the given OpenSCENARIO road position: "
        "RoadID: " +
        std::to_string(xodr_road_position.road_id) + ", s: " + std::to_string(xodr_road_position.s) +
        ", t: " + std::to_string(xodr_road_position.t));
  }
  return maliput::api::RoadPosition{target_lane, maliput::api::LanePosition{mali_lane_s, r, 0.}};
}

RoadGeometry::OpenScenarioRoadPosition RoadGeometry::MaliputRoadPositionToOpenScenarioRoadPosition(
    const maliput::api::RoadPosition& road_position) const {
  OpenScenarioRoadPosition xodr_road_position;
  const auto mali_lane = dynamic_cast<const Lane*>(road_position.lane);
  xodr_road_position.road_id = mali_lane->get_track();
  xodr_road_position.s = mali_lane->TrackSFromLaneS(road_position.pos.s());

  double t = mali_lane->to_reference_r(xodr_road_position.s, road_position.pos.r());
  const auto segment = dynamic_cast<const Segment*>(mali_lane->segment());
  const double roll_at_p = segment->road_curve()->superelevation()->f(xodr_road_position.s);
  xodr_road_position.t = t * std::cos(roll_at_p);
  return xodr_road_position;
}

maliput::api::RoadPosition RoadGeometry::OpenScenarioRelativeRoadPositionToMaliputRoadPosition(
    const OpenScenarioRoadPosition& xodr_reference_road_position, double xodr_ds, double xodr_dt) const {
  const Segment* reference_segment{this->FindSegmentByOpenScenarioRoadPosition(xodr_reference_road_position)};
  MALIPUT_THROW_UNLESS(reference_segment != nullptr);
  const road_curve::RoadCurve* reference_road_curve = reference_segment->road_curve();

  const double target_p = xodr_reference_road_position.s + xodr_ds;
  if (target_p >= reference_road_curve->p0() && target_p <= reference_road_curve->p1()) {
    const OpenScenarioRoadPosition new_os_road_pos{xodr_reference_road_position.road_id,
                                                   reference_segment->road_curve()->PFromP(target_p),
                                                   xodr_reference_road_position.t + xodr_dt};
    return OpenScenarioRoadPositionToMaliputRoadPosition(new_os_road_pos);
  } else {
    // The target_p falls outside this xodr_road.
    // We cover the case where:
    //  - we move backwards (negative ds) and next road is geometrically constructed in the opposite direction.
    //  - we move backwards (negative ds) and next road is geometrically constructed in the same direction.
    //  - we move forwards (positive ds) and next road is geometrically constructed in the opposite direction.
    //  - we move forwards (positive ds) and next road is geometrically constructed in the same direction.
    const bool forward_direction = xodr_ds >= 0.;
    const double xodr_s_to_road_end = forward_direction ? reference_road_curve->p1() - xodr_reference_road_position.s
                                                        : reference_road_curve->p0() - xodr_reference_road_position.s;
    const double new_raw_xodr_ds = xodr_ds - xodr_s_to_road_end;
    // We need to get the last road position before branching in order to identify which is the connected lane(ergo xodr
    // Road) to follow. So if we are moving forward we get the road position at p1, if we are moving backwards we get
    // the road position at p0.
    const maliput::api::RoadPosition last_road_position_before_branching =
        OpenScenarioRoadPositionToMaliputRoadPosition(
            OpenScenarioRoadPosition{xodr_reference_road_position.road_id,
                                     forward_direction ? reference_road_curve->p1() : reference_road_curve->p0(),
                                     xodr_reference_road_position.t});
    const std::optional<LaneEnd> lane_end = last_road_position_before_branching.lane->GetDefaultBranch(
        forward_direction ? LaneEnd::Which::kFinish : LaneEnd::Which::kStart);
    if (lane_end == std::nullopt) {
      // There is no default branch.
      MALIDRIVE_THROW_MESSAGE("There is no where connection road for the given OpenSCENARIO road position: RoadID: " +
                              std::to_string(xodr_reference_road_position.road_id) +
                              ", s: " + std::to_string(xodr_reference_road_position.s) +
                              ", t: " + std::to_string(xodr_reference_road_position.t));
    }
    const Segment* new_target_segment = ToMalidrive(lane_end->lane->segment());
    const road_curve::RoadCurve* new_reference_road_curve = new_target_segment->road_curve();
    const double new_xodr_reference_s =
        lane_end->end == LaneEnd::Which::kFinish ? new_reference_road_curve->p1() : new_reference_road_curve->p0();
    // Forward direction true + laneEnd::kStart -> no sign change for deltas
    // Forward direction true + laneEnd::kFinish -> sign change for deltas
    // Forward direction false + laneEnd::kStart -> sign change for deltas
    // Forward direction false + laneEnd::kFinish -> no sign change for deltas
    const double sign_for_new_deltas = forward_direction == (lane_end->end == LaneEnd::Which::kStart) ? 1. : -1.;
    // If we are moving backwards we need to invert the t-coordinate. We can do it because the specification indicates
    // that when continuing with the connected road
    // "...it is assumed that the reference line of the road of the reference entity continues seamlessly on the
    // connecting road (even if its shape changes)..."
    const double new_xodr_reference_t = sign_for_new_deltas * xodr_reference_road_position.t;
    const OpenScenarioRoadPosition new_xodr_reference_road_position{ToMalidrive(lane_end->lane)->get_track(),
                                                                    new_xodr_reference_s, new_xodr_reference_t};

    const double new_xodr_ds = sign_for_new_deltas * new_raw_xodr_ds;
    const double new_xodr_dt = sign_for_new_deltas * xodr_dt;
    return OpenScenarioRelativeRoadPositionToMaliputRoadPosition(new_xodr_reference_road_position, new_xodr_ds,
                                                                 new_xodr_dt);
  }
}

maliput::api::RoadPosition RoadGeometry::OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
    const OpenScenarioLanePosition& xodr_reference_lane_position, int d_lane, double xodr_ds, double offset) const {
  const Lane* reference_lane = GetMaliputLaneFromOpenScenarioLanePosition(xodr_reference_lane_position);
  const double target_s = xodr_reference_lane_position.s + xodr_ds;
  const Segment* reference_segment = dynamic_cast<const Segment*>(reference_lane->segment());
  MALIPUT_THROW_UNLESS(reference_segment != nullptr);
  const road_curve::RoadCurve* reference_road_curve = reference_segment->road_curve();

  maliput::api::RoadPosition target_position;
  // Target is in the same road.
  if (target_s >= reference_road_curve->p0() && target_s <= reference_road_curve->p1()) {
    const OpenScenarioLanePosition new_os_lane_pos{xodr_reference_lane_position.road_id, target_s,
                                                   xodr_reference_lane_position.lane_id, offset};
    target_position = OpenScenarioLanePositionToMaliputRoadPosition(new_os_lane_pos);
  } else {
    // The target_s falls outside this xodr_road.
    // We cover the case where:
    //  - we move backwards (negative ds) and next road is geometrically constructed in the opposite direction.
    //  - we move backwards (negative ds) and next road is geometrically constructed in the same direction.
    //  - we move forwards (positive ds) and next road is geometrically constructed in the opposite direction.
    //  - we move forwards (positive ds) and next road is geometrically constructed in the same direction.
    const bool forward_direction = xodr_ds >= 0.;
    const double xodr_s_to_road_end = forward_direction ? reference_road_curve->p1() - xodr_reference_lane_position.s
                                                        : reference_road_curve->p0() - xodr_reference_lane_position.s;
    const double new_raw_xodr_ds = xodr_ds - xodr_s_to_road_end;
    const maliput::api::RoadPosition last_road_position_before_branching =
        OpenScenarioLanePositionToMaliputRoadPosition(
            OpenScenarioLanePosition{xodr_reference_lane_position.road_id,
                                     forward_direction ? reference_road_curve->p1() : reference_road_curve->p0(),
                                     xodr_reference_lane_position.lane_id, xodr_reference_lane_position.offset});
    const std::optional<LaneEnd> lane_end = last_road_position_before_branching.lane->GetDefaultBranch(
        forward_direction ? LaneEnd::Which::kFinish : LaneEnd::Which::kStart);
    if (lane_end == std::nullopt) {
      // There is no default branch.
      MALIDRIVE_THROW_MESSAGE("There is no connection road for the given OpenSCENARIO lane position: RoadID: " +
                              std::to_string(xodr_reference_lane_position.road_id) +
                              ", s: " + std::to_string(xodr_reference_lane_position.s) +
                              ", LaneID: " + std::to_string(xodr_reference_lane_position.lane_id) +
                              ", offset: " + std::to_string(xodr_reference_lane_position.offset));
    }
    const Segment* new_target_segment = ToMalidrive(lane_end->lane->segment());
    const road_curve::RoadCurve* new_reference_road_curve = new_target_segment->road_curve();
    const double new_xodr_reference_s =
        lane_end->end == LaneEnd::Which::kFinish ? new_reference_road_curve->p1() : new_reference_road_curve->p0();
    // Forward direction true + laneEnd::kStart -> no sign change for deltas
    // Forward direction true + laneEnd::kFinish -> sign change for deltas
    // Forward direction false + laneEnd::kStart -> sign change for deltas
    // Forward direction false + laneEnd::kFinish -> no sign change for deltas
    const double sign_for_new_deltas = forward_direction == (lane_end->end == LaneEnd::Which::kStart) ? 1. : -1.;
    const double new_xodr_ds = sign_for_new_deltas * new_raw_xodr_ds;
    const int new_d_lane = sign_for_new_deltas * d_lane;
    const OpenScenarioLanePosition new_xodr_reference_lane_position{
        ToMalidrive(lane_end->lane)->get_track(), new_xodr_reference_s, ToMalidrive(lane_end->lane)->get_lane_id(),
        xodr_reference_lane_position.offset};
    return OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(new_xodr_reference_lane_position, new_d_lane,
                                                                           new_xodr_ds, offset);
  }

  const Lane* target_lane = ApplyOffsetToLane(reference_lane, d_lane);
  if (target_lane == nullptr) {
    MALIDRIVE_THROW_MESSAGE(
        "A maliput lane can't be found for the given OpenSCENARIO lane position: "
        "RoadID: " +
        std::to_string(xodr_reference_lane_position.road_id) + ", s: " + std::to_string(target_s) +
        ", LaneID: " + std::to_string(xodr_reference_lane_position.lane_id) + ", offset: " + std::to_string(offset));
  }
  target_position.lane = target_lane;
  return target_position;
}

maliput::api::RoadPosition RoadGeometry::OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(
    const OpenScenarioLanePosition& xodr_reference_lane_position, int d_lane, double ds_lane, double offset) const {
  const Lane* reference_lane = GetMaliputLaneFromOpenScenarioLanePosition(xodr_reference_lane_position);
  const double target_s = reference_lane->LaneSFromTrackS(xodr_reference_lane_position.s) + ds_lane;
  const Segment* reference_segment = dynamic_cast<const Segment*>(reference_lane->segment());
  MALIPUT_THROW_UNLESS(reference_segment != nullptr);
  const road_curve::RoadCurve* reference_road_curve = reference_segment->road_curve();

  maliput::api::RoadPosition target_position;
  // Target is in the same road.
  if (target_s >= reference_lane->LaneSFromTrackS(reference_road_curve->p0()) &&
      target_s <= reference_lane->LaneSFromTrackS(reference_road_curve->p1())) {
    const OpenScenarioLanePosition new_os_lane_pos{xodr_reference_lane_position.road_id,
                                                   reference_lane->TrackSFromLaneS(target_s),
                                                   xodr_reference_lane_position.lane_id, offset};
    target_position = OpenScenarioLanePositionToMaliputRoadPosition(new_os_lane_pos);
  } else {
    // The target_s falls outside this xodr_road.
    // We cover the case where:
    //  - we move backwards (negative ds) and next road is geometrically constructed in the opposite direction.
    //  - we move backwards (negative ds) and next road is geometrically constructed in the same direction.
    //  - we move forwards (positive ds) and next road is geometrically constructed in the opposite direction.
    //  - we move forwards (positive ds) and next road is geometrically constructed in the same direction.
    const bool forward_direction = ds_lane >= 0.;
    const double lane_s_to_road_end = forward_direction
                                          ? reference_lane->LaneSFromTrackS(reference_road_curve->p1()) -
                                                reference_lane->LaneSFromTrackS(xodr_reference_lane_position.s)
                                          : reference_lane->LaneSFromTrackS(reference_road_curve->p0()) -
                                                reference_lane->LaneSFromTrackS(xodr_reference_lane_position.s);
    const double new_raw_ds_lane = ds_lane - lane_s_to_road_end;
    const maliput::api::RoadPosition last_road_position_before_branching =
        OpenScenarioLanePositionToMaliputRoadPosition(
            OpenScenarioLanePosition{xodr_reference_lane_position.road_id,
                                     forward_direction ? reference_road_curve->p1() : reference_road_curve->p0(),
                                     xodr_reference_lane_position.lane_id, xodr_reference_lane_position.offset});
    const std::optional<LaneEnd> lane_end = last_road_position_before_branching.lane->GetDefaultBranch(
        forward_direction ? LaneEnd::Which::kFinish : LaneEnd::Which::kStart);
    if (lane_end == std::nullopt) {
      // There is no default branch.
      MALIDRIVE_THROW_MESSAGE("There is no connection road for the given OpenSCENARIO lane position: RoadID: " +
                              std::to_string(xodr_reference_lane_position.road_id) +
                              ", s: " + std::to_string(xodr_reference_lane_position.s) +
                              ", LaneID: " + std::to_string(xodr_reference_lane_position.lane_id) +
                              ", offset: " + std::to_string(xodr_reference_lane_position.offset));
    }
    const Segment* new_target_segment = ToMalidrive(lane_end->lane->segment());
    const road_curve::RoadCurve* new_reference_road_curve = new_target_segment->road_curve();
    const double new_xodr_reference_s =
        lane_end->end == LaneEnd::Which::kFinish ? new_reference_road_curve->p1() : new_reference_road_curve->p0();
    // Forward direction true + laneEnd::kStart -> no sign change for deltas
    // Forward direction true + laneEnd::kFinish -> sign change for deltas
    // Forward direction false + laneEnd::kStart -> sign change for deltas
    // Forward direction false + laneEnd::kFinish -> no sign change for deltas
    const double sign_for_new_deltas = forward_direction == (lane_end->end == LaneEnd::Which::kStart) ? 1. : -1.;
    const double new_ds_lane = sign_for_new_deltas * new_raw_ds_lane;
    const int new_d_lane = sign_for_new_deltas * d_lane;
    const OpenScenarioLanePosition new_xodr_reference_lane_position{
        ToMalidrive(lane_end->lane)->get_track(), new_xodr_reference_s, ToMalidrive(lane_end->lane)->get_lane_id(),
        xodr_reference_lane_position.offset};
    return OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(new_xodr_reference_lane_position, new_d_lane,
                                                                           new_ds_lane, offset);
  }

  const Lane* target_lane = ApplyOffsetToLane(reference_lane, d_lane);
  if (target_lane == nullptr) {
    MALIDRIVE_THROW_MESSAGE(
        "A maliput lane can't be found for the given OpenSCENARIO lane position: "
        "RoadID: " +
        std::to_string(xodr_reference_lane_position.road_id) + ", s: " + std::to_string(target_s) +
        ", LaneID: " + std::to_string(xodr_reference_lane_position.lane_id) + ", offset: " + std::to_string(offset));
  }
  target_position.lane = target_lane;
  return target_position;
}

const Lane* RoadGeometry::ApplyOffsetToLane(const Lane* initial_lane, int lane_offset) const {
  bool to_left = lane_offset >= 0;
  const Lane* target_lane = initial_lane;
  if (lane_offset != 0) {
    for (int asb_offset = lane_offset * lane_offset / std::abs(lane_offset); asb_offset != 0; --asb_offset) {
      target_lane = dynamic_cast<const Lane*>(to_left ? target_lane->to_left() : target_lane->to_right());
      MALIPUT_THROW_UNLESS(target_lane != nullptr);
    }
  }
  return target_lane;
}

maliput::math::RollPitchYaw RoadGeometry::GetRoadOrientationAtOpenScenarioRoadPosition(
    const OpenScenarioRoadPosition& xodr_road_position) const {
  MALIDRIVE_THROW_UNLESS(xodr_road_position.road_id >= 0);
  MALIDRIVE_THROW_UNLESS(xodr_road_position.s >= 0.);
  const Segment* target_segment{this->FindSegmentByOpenScenarioRoadPosition(xodr_road_position)};
  MALIPUT_THROW_UNLESS(target_segment != nullptr);
  const double p = target_segment->road_curve()->PFromP(xodr_road_position.s);
  return target_segment->road_curve()->Orientation(p);
}

std::string RoadGeometry::DoBackendCustomCommand(const std::string& command) const {
  CommandsHandler commands_handler(this);
  return commands_handler.Execute(commands_handler.ParseCommand(command));
}

}  // namespace malidrive
