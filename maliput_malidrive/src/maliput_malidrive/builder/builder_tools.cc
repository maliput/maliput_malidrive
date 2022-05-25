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
#include "maliput_malidrive/builder/builder_tools.h"

#include <algorithm>
#include <map>

#include <maliput/api/lane.h>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>
#include <tinyxml2.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/road_link.h"

using maliput::api::LaneId;

namespace malidrive {
namespace builder {
namespace {

// Aggregate information about an XODR Lane type and the type
// of vehicle (if any) that can drive on a lane.
struct XodrLaneProperties {
  bool is_driveable_lane{false};
  std::string vehicle_usage_value;
  std::optional<std::string> vehicle_exclusive_value;
};

// Holds aggregate information about XODR Lane types.
//
// This map performs as a link between OpenDRIVE's standard and maliput.
const std::map<xodr::Lane::Type, XodrLaneProperties> kXodrLaneTypesToMaliputProperties{
    {xodr::Lane::Type::kNone, {false, "Unrestricted", {}}},
    {xodr::Lane::Type::kDriving, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kStop, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kShoulder, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kBiking, {true, "NonPedestrians", {"NonMotorizedVehicleOnly"}}},
    {xodr::Lane::Type::kSidewalk, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kBorder, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kRestricted, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kParking, {false, "Unrestricted", {}}},
    {xodr::Lane::Type::kMwyEntry, {true, "NonPedestrians", {"MotorizedVehicleOnly"}}},
    {xodr::Lane::Type::kMwyExit, {true, "NonPedestrians", {"MotorizedVehicleOnly"}}},
    {xodr::Lane::Type::kSpecial1, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kSpecial2, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kSpecial3, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kRoadWorks, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kTram, {false, "NonPedestrians", {}}},
    {xodr::Lane::Type::kRail, {false, "NonPedestrians", {}}},
    {xodr::Lane::Type::kBidirectional, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kMedian, {false, "NonVehicles", {}}},
    {xodr::Lane::Type::kEntry, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kExit, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kOnRamp, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kOffRamp, {true, "NonPedestrians", {}}},
    {xodr::Lane::Type::kConnectingRamp, {true, "NonPedestrians", {}}},
};

}  // namespace

std::vector<maliput::api::LaneEnd> SolveLaneEndsForConnectingRoad(
    const maliput::api::RoadGeometry* rg, const MalidriveXodrLaneProperties& xodr_lane_properties,
    const std::map<xodr::RoadHeader::Id, xodr::RoadHeader>& road_headers, XodrConnectionType connection_type) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);

  std::vector<maliput::api::LaneEnd> connecting_lane_ends;

  // Successor / Predecessor is a road.
  const std::optional<xodr::RoadLink::LinkAttributes> road_link =
      connection_type == XodrConnectionType::kSuccessor ? xodr_lane_properties.road_header->road_link.successor
                                                        : xodr_lane_properties.road_header->road_link.predecessor;
  MALIDRIVE_VALIDATE(road_link != std::nullopt, maliput::common::assertion_error,
                     "SolveLaneEndsForConnectingRoad(). Trying to connect Xodr Road(" +
                         xodr_lane_properties.road_header->id.string() + ") but the road_link " +
                         (connection_type == XodrConnectionType::kSuccessor ? "successor" : "predecessor") +
                         " is empty.");
  MALIDRIVE_VALIDATE(road_link->element_type == xodr::RoadLink::ElementType::kRoad, maliput::common::assertion_error,
                     "SolveLaneEndsForConnectingRoad(). Expecting Xodr Road(" +
                         xodr_lane_properties.road_header->id.string() +
                         ") with road_link of type xodr::RoadLink::ElementType::kRoad but it is "
                         "xodr::RoadLink::ElementType::kJunction");

  // Finds the Road and connecting lanes section.
  const xodr::RoadHeader::Id road_header_id(road_link->element_id.string());
  MALIDRIVE_VALIDATE(
      road_headers.find(road_header_id) != road_headers.end(), maliput::common::assertion_error,
      "SolveLaneEndsForConnectingRoad(). RoadLink pointing to missing Xodr Road(" + road_header_id.string() + ").");

  const xodr::RoadHeader& road_header = road_headers.at(road_header_id);

  const std::optional<xodr::LaneLink::LinkAttributes> lane_link =
      connection_type == XodrConnectionType::kSuccessor ? xodr_lane_properties.lane->lane_link.successor
                                                        : xodr_lane_properties.lane->lane_link.predecessor;
  if (lane_link != std::nullopt) {
    // If the contact_point is a start point then we take the first LaneSection, otherwise we take the last LaneSection.
    const int xodr_connecting_lane_section_index = road_link->contact_point == xodr::RoadLink::ContactPoint::kStart
                                                       ? 0
                                                       : (road_header.lanes.lanes_section.size() - 1);
    const LaneId lane_id = GetLaneId(std::stoi(road_header.id.string()), xodr_connecting_lane_section_index,
                                     std::stoi(lane_link->id.string()));
    const maliput::api::Lane* lane = rg->ById().GetLane(lane_id);
    if (lane != nullptr) {
      connecting_lane_ends.push_back(
          maliput::api::LaneEnd(lane, road_link->contact_point == xodr::RoadLink::ContactPoint::kStart
                                          ? maliput::api::LaneEnd::Which::kStart
                                          : maliput::api::LaneEnd::Which::kFinish));
    } else {
      maliput::log()->error("Lane " + lane_id.string() + " could not be found or not drivable.");
    }
  }
  return connecting_lane_ends;
}

std::vector<maliput::api::LaneEnd> SolveLaneEndsForJunction(
    const maliput::api::RoadGeometry* rg, const MalidriveXodrLaneProperties& xodr_lane_properties,
    const std::map<xodr::RoadHeader::Id, xodr::RoadHeader>& road_headers,
    const std::unordered_map<xodr::Junction::Id, xodr::Junction>& junctions, XodrConnectionType connection_type) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);

  std::vector<maliput::api::LaneEnd> connecting_lane_ends;

  // Successor / Predecessor is a junction.
  const std::optional<xodr::RoadLink::LinkAttributes> road_link =
      connection_type == XodrConnectionType::kSuccessor ? xodr_lane_properties.road_header->road_link.successor
                                                        : xodr_lane_properties.road_header->road_link.predecessor;
  MALIDRIVE_VALIDATE(road_link != std::nullopt, maliput::common::assertion_error,
                     "SolveLaneEndsForJunction(). Trying to connect Xodr Road(" +
                         xodr_lane_properties.road_header->id.string() + ") but the road_link " +
                         (connection_type == XodrConnectionType::kSuccessor ? "successor" : "predecessor") +
                         " is empty.");
  MALIDRIVE_VALIDATE(
      road_link->element_type == xodr::RoadLink::ElementType::kJunction, maliput::common::assertion_error,
      "SolveLaneEndsForJunction(). Expecting Xodr Road(" + xodr_lane_properties.road_header->id.string() +
          ") with road_link of type xodr::RoadLink::ElementType::kJunction but it is "
          "xodr::RoadLink::ElementType::kRoad");

  // Gets the Junction that this road connects to.
  const auto junction = junctions.find(xodr::Junction::Id(road_link->element_id.string()));
  MALIDRIVE_VALIDATE(junction != junctions.end(), maliput::common::assertion_error,
                     "SolveLaneEndsForJunction(). RoadLink pointing to missing Xodr Junction(" +
                         road_link->element_id.string() + ").");

  // Look for the junction link that has as incomingRoad this road ID.
  for (const auto& connection : junction->second.connections) {
    if (xodr::RoadHeader::Id(connection.second.incoming_road) != xodr_lane_properties.road_header->id) continue;
    for (const auto& lane_link : connection.second.lane_links) {
      if (xodr::Lane::Id(lane_link.from.string()) != xodr_lane_properties.lane->id) continue;
      // So `lane_link.to` contains a lane that `xodr_lane_properties.lane` is connected to.
      // We have to find that lane in order to create the LaneEnd.

      // Get the RoadHeader of the connecting road.
      const auto road_header = road_headers.find(xodr::RoadHeader::Id(connection.second.connecting_road));
      MALIDRIVE_VALIDATE(road_header != road_headers.end(), maliput::common::assertion_error,
                         "SolveLaneEndsForJunction(). Xodr Junction(" + road_link->element_id.string() +
                             ") has Xodr Connection(" + connection.first.string() + ") with a connecting Xodr Road(" +
                             connection.second.connecting_road + ") that cannot be found.");
      // If the `contact_point` is a start point then we take the first LaneSection, otherwise we take the last
      // LaneSection.
      const int xodr_connecting_lane_section_index =
          connection.second.contact_point == xodr::Connection::ContactPoint::kStart
              ? 0
              : (road_header->second.lanes.lanes_section.size() - 1);
      // Create the LaneId that the lane we are looking for should have.
      const LaneId lane_id = GetLaneId(std::stoi(road_header->first.string()), xodr_connecting_lane_section_index,
                                       std::stoi(lane_link.to.string()));
      const maliput::api::Lane* lane = rg->ById().GetLane(lane_id);
      if (lane != nullptr) {
        connecting_lane_ends.push_back(
            maliput::api::LaneEnd(lane, connection.second.contact_point == xodr::Connection::ContactPoint::kStart
                                            ? maliput::api::LaneEnd::Which::kStart
                                            : maliput::api::LaneEnd::Which::kFinish));
      } else {
        maliput::log()->error("Lane " + lane_id.string() + " could not be found or not drivable.");
      }
    }
  }
  return connecting_lane_ends;
}

std::vector<maliput::api::LaneEnd> SolveLaneEndsWithinJunction(
    const maliput::api::RoadGeometry* rg, const MalidriveXodrLaneProperties& xodr_lane_properties,
    const std::map<xodr::RoadHeader::Id, xodr::RoadHeader>& road_headers, XodrConnectionType connection_type) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  // Successor / Predecessor is a road.
  const std::optional<xodr::RoadLink::LinkAttributes> road_link =
      connection_type == XodrConnectionType::kSuccessor ? xodr_lane_properties.road_header->road_link.successor
                                                        : xodr_lane_properties.road_header->road_link.predecessor;
  if (road_link == std::nullopt) {
    maliput::log()->debug("Trying to connect xodr Road: {}, lane: {} {} endpoint but it lacks of a {}.",
                          xodr_lane_properties.road_header->id.string(), xodr_lane_properties.lane->id.string(),
                          connection_type == XodrConnectionType::kSuccessor ? "end" : "start",
                          connection_type == XodrConnectionType::kSuccessor ? "successor" : "predecessor");
    return {};
  }
  if (road_link->element_type == xodr::RoadLink::ElementType::kJunction) {
    MALIDRIVE_THROW_MESSAGE("Junctions connected to junctions are not supported.");
  }
  return SolveLaneEndsForConnectingRoad(rg, xodr_lane_properties, road_headers, connection_type);
}

std::vector<maliput::api::LaneEnd> SolveLaneEndsForInnerLaneSection(
    const maliput::api::RoadGeometry* rg, const maliput::api::LaneEnd& lane_end,
    const MalidriveXodrLaneProperties& xodr_lane_properties) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);

  std::vector<maliput::api::LaneEnd> connecting_lane_ends;

  // Get the connecting LaneSection.
  // Evaluates predecessors for this lane.
  const auto lane_link{lane_end.end == maliput::api::LaneEnd::Which::kStart
                           ? xodr_lane_properties.lane->lane_link.predecessor
                           : xodr_lane_properties.lane->lane_link.successor};
  if (lane_link.has_value()) {
    const int connecting_lane_section_index = lane_end.end == maliput::api::LaneEnd::Which::kStart
                                                  ? xodr_lane_properties.lane_section_index - 1
                                                  : xodr_lane_properties.lane_section_index + 1;
    const LaneId lane_id = GetLaneId(std::stoi(xodr_lane_properties.road_header->id.string()),
                                     connecting_lane_section_index, std::stoi(lane_link->id.string()));
    const maliput::api::Lane* lane = rg->ById().GetLane(lane_id);
    if (lane != nullptr) {
      connecting_lane_ends.push_back(maliput::api::LaneEnd(lane, lane_end.end == maliput::api::LaneEnd::Which::kStart
                                                                     ? maliput::api::LaneEnd::Which::kFinish
                                                                     : maliput::api::LaneEnd::Which::kStart));
    } else {
      maliput::log()->error("Lane " + lane_id.string() + " could not be found or not drivable.");
    }
  }
  return connecting_lane_ends;
}

namespace {

const std::map<std::string, LaneTravelDirection::Direction> str_to_direction_map{
    {"undirected", LaneTravelDirection::Direction::kUndirected},
    {"forward", LaneTravelDirection::Direction::kForward},
    {"backward", LaneTravelDirection::Direction::kBackward},
    {"bidirectional", LaneTravelDirection::Direction::kBidirectional},
    {"undefined", LaneTravelDirection::Direction::kUndefined}};

const std::map<LaneTravelDirection::Direction, std::string> xodr_to_maliput_direction{
    {LaneTravelDirection::Direction::kUndirected, "Bidirectional"},
    {LaneTravelDirection::Direction::kForward, "WithS"},
    {LaneTravelDirection::Direction::kBackward, "AgainstS"},
    {LaneTravelDirection::Direction::kBidirectional, "Bidirectional"},
    {LaneTravelDirection::Direction::kUndefined, "Undefined"}};

}  // namespace

LaneTravelDirection::LaneTravelDirection(const std::optional<std::string>& user_data) {
  if (!user_data.has_value()) {
    travel_dir_ = LaneTravelDirection::Direction::kUndefined;
    return;
  }
  tinyxml2::XMLDocument doc;
  const auto e_result = doc.Parse(user_data.value().c_str());
  if (e_result != tinyxml2::XML_SUCCESS) {
    maliput::log()->error("UserData Node couldn't be parsed. XML formatting error.");
  }
  const tinyxml2::XMLElement* user_data_element = doc.FirstChildElement(kUserDataTag);
  if (user_data_element == nullptr) {
    travel_dir_ = LaneTravelDirection::Direction::kUndefined;
    return;
  }
  const tinyxml2::XMLElement* vector_lane_element = user_data_element->FirstChildElement(kVectorLaneTag);
  if (vector_lane_element == nullptr) {
    travel_dir_ = LaneTravelDirection::Direction::kUndefined;
    return;
  }
  const char* travel_dir_attribute = vector_lane_element->Attribute(kTravelDirTag);
  if (travel_dir_attribute == nullptr) {
    travel_dir_ = LaneTravelDirection::Direction::kUndefined;
    return;
  }
  travel_dir_ = str_to_direction(travel_dir_attribute);
}

std::string LaneTravelDirection::GetMaliputTravelDir() const { return xodr_to_maliput_direction.at(travel_dir_); }

LaneTravelDirection::Direction LaneTravelDirection::str_to_direction(const std::string& direction) const {
  if (str_to_direction_map.find(direction) == str_to_direction_map.end()) {
    MALIDRIVE_THROW_MESSAGE(direction + " is not a proper lane direction");
  }
  return str_to_direction_map.at(direction);
}

bool is_driveable_lane(const xodr::Lane& xodr_lane) {
  return kXodrLaneTypesToMaliputProperties.at(xodr_lane.type).is_driveable_lane && xodr_lane.id != xodr::Lane::Id("0");
}

bool AreOnlyNonDrivableLanes(const xodr::RoadHeader& xodr_road) {
  for (const auto& lane_section : xodr_road.lanes.lanes_section) {
    for (const auto& lane : lane_section.left_lanes) {
      if (is_driveable_lane(lane)) {
        return false;
      }
    }
    for (const auto& lane : lane_section.right_lanes) {
      if (is_driveable_lane(lane)) {
        return false;
      }
    }
  }
  return true;
}

std::string VehicleUsageValueForXodrLane(const xodr::Lane& xodr_lane) {
  return kXodrLaneTypesToMaliputProperties.at(xodr_lane.type).vehicle_usage_value;
}

std::optional<std::string> VehicleExclusiveValueForXodrLane(const xodr::Lane& xodr_lane) {
  return kXodrLaneTypesToMaliputProperties.at(xodr_lane.type).vehicle_exclusive_value;
}

std::vector<rules::XodrSpeedProperties> GetRoadTypeSpeedPropertiesInRange(const xodr::RoadHeader& xodr_road,
                                                                          double s_track_start, double s_track_end) {
  MALIDRIVE_THROW_UNLESS(s_track_start < s_track_end);
  MALIDRIVE_THROW_UNLESS(s_track_start >= 0.);
  // TODO(#567): Verify that `s_track_end` is not greater than road's length.
  //             Tolerance is needed.
  const std::vector<const xodr::RoadType*> road_types = xodr_road.GetRoadTypesInRange(s_track_start, s_track_end);
  const int road_types_size{static_cast<int>(road_types.size())};
  std::vector<rules::XodrSpeedProperties> speed_data;
  if (road_types_size) {
    for (int index = 0; index < road_types_size; index++) {
      const double s_start{index == 0 && s_track_start >= road_types[index]->s_0 ? s_track_start
                                                                                 : road_types[index]->s_0};
      const double s_end{index == road_types_size - 1 ? s_track_end : road_types[index + 1]->s_0};
      const double speed{road_types[index]->speed.max.has_value()
                             ? xodr::ConvertToMs(road_types[index]->speed.max.value(), road_types[index]->speed.unit)
                             : constants::kDefaultMaxSpeedLimit};
      speed_data.push_back({speed, s_start, s_end});
    }
  }
  return speed_data;
}

std::vector<rules::XodrSpeedProperties> GetLaneSpeedProperties(const xodr::Lane& xodr_lane, double s_track_start,
                                                               double s_track_end) {
  MALIDRIVE_THROW_UNLESS(s_track_start < s_track_end);
  MALIDRIVE_THROW_UNLESS(s_track_start >= 0.);
  // TODO(#567): Verify that `s_track_end` is not greater than roads's length.
  //             Tolerance is needed.
  std::vector<rules::XodrSpeedProperties> speed_data;
  const int lane_speed_size{static_cast<int>(xodr_lane.speed.size())};
  if (lane_speed_size) {
    for (int index = 0; index < lane_speed_size; index++) {
      const double s_end{index == lane_speed_size - 1 ? s_track_end
                                                      : xodr_lane.speed[index + 1].s_offset + s_track_start};
      speed_data.push_back({xodr::ConvertToMs(xodr_lane.speed[index].max, xodr_lane.speed[index].unit),
                            xodr_lane.speed[index].s_offset + s_track_start, s_end});
    }
  }
  return speed_data;
}

const xodr::RoadHeader& GetXodrRoadFromMalidriveLane(const Lane* lane) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  const xodr::DBManager* manager =
      static_cast<const malidrive::RoadGeometry*>(lane->segment()->junction()->road_geometry())->get_manager();
  MALIDRIVE_THROW_UNLESS(manager != nullptr);

  const auto& road_headers = manager->GetRoadHeaders();
  const xodr::RoadHeader::Id road_id{std::to_string(lane->get_track())};
  const xodr::Lane::Id lane_id{std::to_string(lane->get_lane_id())};
  const auto road_it = road_headers.find(road_id);
  if (road_it == road_headers.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("Road of the Lane Id: ") + lane->id().string() +
                            std::string(" couldn't be found."));
  }
  return road_headers.at(road_id);
}

const xodr::Lane& GetXodrLaneFromMalidriveLane(const Lane* lane) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  const xodr::RoadHeader& road_header = GetXodrRoadFromMalidriveLane(lane);
  const double s_half{(lane->get_track_s_end() - lane->get_track_s_start()) / 2 + lane->get_track_s_start()};
  const int lane_section_index = road_header.GetLaneSectionIndex(s_half);

  const auto& lanes = lane->get_lane_id() < 0 ? road_header.lanes.lanes_section[lane_section_index].right_lanes
                                              : road_header.lanes.lanes_section[lane_section_index].left_lanes;
  const auto lane_it = std::find_if(
      lanes.begin(), lanes.end(),
      [lane_id = lane->get_lane_id()](const xodr::Lane& lane) { return lane.id.string() == std::to_string(lane_id); });
  if (lane_it == lanes.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("Lane Id: ") + lane->id().string() + std::string(" couldn't be found."));
  }
  return *lane_it;
}

std::string GetDirectionUsageRuleStateType(const Lane* lane) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  const xodr::Lane& xodr_lane = GetXodrLaneFromMalidriveLane(lane);
  const LaneTravelDirection travel_dir(xodr_lane.user_data);
  return travel_dir.GetMaliputTravelDir();
}

std::vector<rules::XodrSpeedProperties> GetMaxSpeedLimitFor(const Lane* lane) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  const xodr::Lane& xodr_lane = GetXodrLaneFromMalidriveLane(lane);
  const double s_track_start{lane->get_track_s_start()};
  const double s_track_end{lane->get_track_s_end()};
  std::vector<rules::XodrSpeedProperties> speed_data = GetLaneSpeedProperties(xodr_lane, s_track_start, s_track_end);
  const xodr::RoadHeader& xodr_road = GetXodrRoadFromMalidriveLane(lane);
  const std::vector<rules::XodrSpeedProperties> speed_data_from_road =
      GetRoadTypeSpeedPropertiesInRange(xodr_road, s_track_start, s_track_end);
  if (!speed_data.empty()) {
    if (speed_data[0].s_start != s_track_start) {
      MALIDRIVE_THROW_UNLESS(speed_data[0].s_start > s_track_start);
      // If there is a gap at the beginning, try to complete it with the RoadType information.
      for (auto it = speed_data_from_road.rbegin(); it != speed_data_from_road.rend(); it++) {
        if (it->s_start < speed_data[0].s_start) {
          auto speed_to_be_added{*it};
          speed_to_be_added.s_end = speed_data[0].s_start;
          speed_data.insert(speed_data.begin(), speed_to_be_added);
        }
      }
    }
  } else if (!speed_data_from_road.empty()) {
    speed_data = std::move(speed_data_from_road);
  }
  // Check if there is a gap at the beginning and complete it with constants::kDefaultMaxSpeedLimit.
  if (speed_data.empty() || speed_data[0].s_start != s_track_start) {
    speed_data.insert(speed_data.begin(), {constants::kDefaultMaxSpeedLimit, s_track_start,
                                           speed_data.empty() ? s_track_end : speed_data[0].s_start});
  }
  return speed_data;
}

std::pair<std::string, std::optional<std::string>> VehicleUsageAndExclusiveRuleStateValues(const Lane* lane) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  const xodr::Lane& xodr_lane = GetXodrLaneFromMalidriveLane(lane);
  return std::make_pair(VehicleUsageValueForXodrLane(xodr_lane), VehicleExclusiveValueForXodrLane(xodr_lane));
}

std::optional<double> FindLocalMinFromCubicPol(double a, double b, double c, double d) {
  maliput::common::unused(d);
  std::optional<double> p_local_min{std::nullopt};
  if (std::abs(a) < constants::kStrictLinearTolerance) {
    // Quadratic polynomial
    if (std::abs(b) < constants::kStrictLinearTolerance) {
      // Linear polynomial doesn't have a local min.
      return p_local_min;
    }
    if (b > 0) {
      // Only when the parabola ascends could we have a "local minimum" value.
      p_local_min = -c / (2 * b);
    }
  } else {
    // Cubic polynomial
    const double det = b * b - 3 * a * c;
    if (det > constants::kStrictLinearTolerance) {
      // If b^2 – 3ac is nonpositive, the cubic function is strictly monotonic, so local min couldn't be found.
      const double x1 = (-b + std::sqrt(det)) / (3 * a);
      const double x2 = (-b - std::sqrt(det)) / (3 * a);
      for (const auto& p_local : {x1, x2}) {
        // f_2_p_local would be the second derivative of f(p) evaluated at p_local.
        // This would tell us if it is a local min or local max.
        const double f_2_p_local = 6 * a * p_local + 2 * b;
        if (f_2_p_local > 0) {
          p_local_min = p_local;
          break;
        }
      }
    }
  }
  return p_local_min;
}

}  // namespace builder
}  // namespace malidrive
