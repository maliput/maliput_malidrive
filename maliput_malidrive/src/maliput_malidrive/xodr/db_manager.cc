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
#include "maliput_malidrive/xodr/db_manager.h"

#include <variant>

#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/lateral_profile.h"
#include "maliput_malidrive/xodr/parser.h"
#include "maliput_malidrive/xodr/tools.h"

namespace malidrive {
namespace xodr {

class DBManager::Impl {
 public:
  // Tag in the XML file that indicates that the file is a XODR description.
  static constexpr const char* kXodrTag = "OpenDRIVE";

  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);

  // Creates a DBManager::Impl instance.
  // @param parser_configuration Holds the configuration for the parser.
  // @throw maliput::common::assertion_error When `parser_configuration.tolerance` is negative.
  explicit Impl(const ParserConfiguration& parser_configuration) : parser_configuration_(parser_configuration) {
    if (parser_configuration_.tolerance.has_value()) {
      MALIDRIVE_THROW_UNLESS(*parser_configuration_.tolerance >= 0);
    }
    maliput::log()->trace("XODR Parser configuration:");
    maliput::log()->trace("|__ tolerance: {}", (parser_configuration_.tolerance.has_value()
                                                    ? std::to_string(parser_configuration_.tolerance.value())
                                                    : "None"));
    maliput::log()->trace("|__ allow_schema_errors: {}",
                          parser_configuration_.allow_schema_errors ? "Enabled" : "Disabled");
    maliput::log()->trace("|__ allow_semantic_errors: {}",
                          parser_configuration_.allow_semantic_errors ? "Enabled" : "Disabled");
  };
  ~Impl() = default;

  // Parses a XMLDocument which contains a XODR description.
  // @param xodr_doc A XMLDocument.
  void ParseDoc(tinyxml2::XMLDocument* xodr_doc) {
    // Check if it is a XODR file.
    MALIDRIVE_TRACE("XODR parsing process has started.");
    MALIDRIVE_TRACE("Verifying XODR tag in the file.");
    tinyxml2::XMLElement* xodr_root_node = xodr_doc->FirstChildElement();
    MALIDRIVE_THROW_UNLESS(static_cast<std::string>(xodr_root_node->Value()) == static_cast<std::string>(kXodrTag));

    // Parse XODR header.
    MALIDRIVE_TRACE("Parsing header node.");
    tinyxml2::XMLElement* header_node = xodr_root_node->FirstChildElement(Header::kHeaderTag);
    MALIDRIVE_THROW_UNLESS(header_node != nullptr);
    header_ = NodeParser(header_node, parser_configuration_).As<Header>();

    // Parse XODR `road` headers.
    MALIDRIVE_TRACE("Parsing road headers.");
    tinyxml2::XMLElement* road_header_node = xodr_root_node->FirstChildElement(RoadHeader::kRoadHeaderTag);
    while (road_header_node) {
      const RoadHeader road_header = NodeParser(road_header_node, parser_configuration_).As<RoadHeader>();
      MALIDRIVE_TRACE("Parsing road id: " + road_header.id.string());
      road_headers_.emplace(road_header.id, road_header);
      road_header_node = road_header_node->NextSiblingElement(RoadHeader::kRoadHeaderTag);
    }

    // Parse XODR `junction` headers.
    MALIDRIVE_TRACE("Parsing junction headers.");
    tinyxml2::XMLElement* junction_node = xodr_root_node->FirstChildElement(Junction::kJunctionTag);
    while (junction_node) {
      const Junction junction = NodeParser(junction_node, parser_configuration_).As<Junction>();
      const auto id = junctions_.find(junction.id);
      if (id != junctions_.end()) {
        MALIDRIVE_THROW_MESSAGE(std::string("Junction Id: ") + junction.id.string() + " is duplicated.");
      }
      MALIDRIVE_TRACE("Junction id: " + junction.id.string() + " parsed.");
      junctions_.emplace(junction.id, junction);
      junction_node = junction_node->NextSiblingElement(Junction::kJunctionTag);
    }

    MALIDRIVE_TRACE("Completing missing LaneLinks connections for junctions");
    CompleteJunctionsLaneLinks();
    MALIDRIVE_TRACE("Verifying junctions connections.");
    VerifyJunctions();

    // Verify Links.
    MALIDRIVE_TRACE("Verifying RoadLinks.");
    for (const auto& road_header : road_headers_) {
      MALIDRIVE_TRACE("Verifying links for road id: " + road_header.first.string());
      // Links between roads.
      const auto predecessor_attributes = road_header.second.road_link.predecessor;
      if (predecessor_attributes.has_value()) {
        VerifyRoadLinks(road_header.second, predecessor_attributes.value(), true);
      }
      const auto successor_attributes = road_header.second.road_link.successor;
      if (successor_attributes.has_value()) {
        VerifyRoadLinks(road_header.second, successor_attributes.value(), false);
      }
      VerifyLinksBetweenLaneSectionsOfARoad(road_header.second);
    }

    MALIDRIVE_TRACE("Analyzing geometries' length.");
    AnalyzeGeometriesLength();
    MALIDRIVE_TRACE("Analyzing lanes-sections' length.");
    AnalyzeLaneSectionsLength();
  }

  // @returns A constant reference to xodr header.
  const Header& get_header() const { return header_; }

  // @returns A constant reference to road headers map.
  const std::map<RoadHeader::Id, RoadHeader>& get_road_headers() const { return road_headers_; }

  // @returns A constant reference to junction map.
  const std::unordered_map<Junction::Id, Junction>& get_junctions() const { return junctions_; }

  // @returns Data from the shortest Geometry in the entire XODR description.
  const XodrGeometryLengthData& get_shortest_geometry() const { return shortest_geometry_; }

  // @returns Data from the largest Geometry in the entire XODR description.
  const XodrGeometryLengthData& get_largest_geometry() const { return largest_geometry_; }

  // @returns Data from the shortest LaneSection in the entire XODR description.
  const XodrLaneSectionLengthData& get_shortest_lane_section() const { return shortest_lane_section_; }

  // @returns Data from the largest LaneSection in the entire XODR description.
  const XodrLaneSectionLengthData& get_largest_lane_section() const { return largest_lane_section_; }

  // @returns Data from the shortest gap between Geometries in the entire XODR description.
  const XodrGapBetweenGeometries& get_shortest_gap() const {
    if (!shortest_gap_.has_value()) {
      AnalyzeGapBetweenGeometries();
    }
    return shortest_gap_.value();
  }

  // @returns Data from the largest gap between Geometries in the entire XODR description.
  const XodrGapBetweenGeometries& get_largest_gap() const {
    if (!largest_gap_.has_value()) {
      AnalyzeGapBetweenGeometries();
    }
    return largest_gap_.value();
  }

  // @returns Data from the shortest gap between elevations in the entire XODR
  //          description.
  const XodrGapBetweenFunctions& get_shortest_elevation_gap() const {
    if (!shortest_elevation_gap_.has_value()) {
      AnalyzeGapBetweenElevationFunction();
    }
    return shortest_elevation_gap_.value();
  }

  // @returns Data from the largest gap between elevations in the entire XODR
  //          description.
  const XodrGapBetweenFunctions& get_largest_elevation_gap() const {
    if (!largest_elevation_gap_.has_value()) {
      AnalyzeGapBetweenElevationFunction();
    }
    return largest_elevation_gap_.value();
  }

  // @returns Data from the shortest gap between superelevations in the entire
  //          XODR description.
  const XodrGapBetweenFunctions& get_shortest_superelevation_gap() const {
    if (!shortest_superelevation_gap_.has_value()) {
      AnalyzeGapBetweenSuperelevationFunction();
    }
    return shortest_superelevation_gap_.value();
  }

  // @returns Data from the largest gap between superelevations in the entire
  //          XODR description.
  const XodrGapBetweenFunctions& get_largest_superelevation_gap() const {
    if (!largest_superelevation_gap_.has_value()) {
      AnalyzeGapBetweenSuperelevationFunction();
    }
    return largest_superelevation_gap_.value();
  }

  const std::vector<DBManager::XodrGeometriesToSimplify> GetGeometriesToSimplify(double tolerance) const {
    MALIDRIVE_THROW_UNLESS(tolerance >= 0.);

    std::vector<DBManager::XodrGeometriesToSimplify> geometries_to_simplify;
    double length = 0.;

    // Helper to reset local variables.
    auto reset_local_vars = [&length](const RoadHeader::Id& id, double geometry_length,
                                      DBManager::XodrGeometriesToSimplify* geometry_to_simplify) {
      length = geometry_length;
      *geometry_to_simplify = DBManager::XodrGeometriesToSimplify();
      geometry_to_simplify->road_header_id = id;
    };

    // Helper to get the action from the grouped geometry types.
    auto get_action_from_geometries = [](const DBManager::XodrGeometriesToSimplify& geometry_to_simplify,
                                         const std::vector<Geometry>& plain_view_geometries) {
      return plain_view_geometries[geometry_to_simplify.geometries[0]].type == Geometry::Type::kLine
                 ? DBManager::XodrGeometriesToSimplify::Action::kReplaceByLine
                 : DBManager::XodrGeometriesToSimplify::Action::kReplaceByArc;
    };

    // Helper to determine whether a XodrGeometriesToSimplify has more than one geometry or not.
    auto has_more_than_one = [](const DBManager::XodrGeometriesToSimplify& geometry_to_simplify) {
      return static_cast<int>(geometry_to_simplify.geometries.size()) > 1;
    };

    // Helper to determine whether two Geometries share the same type.
    auto evaluate_type = [](const Geometry& base_geometry, const Geometry& new_geometry) {
      return base_geometry.type == new_geometry.type;
    };

    // Helper to evaluate if base_geometry ends in the same place as new_geometry (within tolerance)
    // when the accumulated_length and new_geometry.length is considered.
    auto evaluate_tolerance = [tolerance](const Geometry& base_geometry, const Geometry& new_geometry,
                                          double accumulated_length) {
      const auto delta = ComputeEndpointWithNewLength(base_geometry, accumulated_length + new_geometry.length) -
                         ComputeEndpointWithNewLength(new_geometry, new_geometry.length);
      return delta.norm() <= tolerance;
    };

    for (const auto& road_header_id_road_header : road_headers_) {
      const std::vector<Geometry>& plain_view_geometries =
          road_header_id_road_header.second.reference_geometry.plan_view.geometries;

      DBManager::XodrGeometriesToSimplify geometry_to_simplify;
      reset_local_vars(road_header_id_road_header.first, plain_view_geometries[0].length, &geometry_to_simplify);
      // Sets the first geometry.
      geometry_to_simplify.geometries.push_back(0);

      for (int i = 1; i < static_cast<int>(plain_view_geometries.size()); ++i) {
        // Evaluates whether the type of the current geometry is the same to the previous and the
        // criteria to merge the geometry or not.
        switch (plain_view_geometries[i].type) {
          case Geometry::Type::kLine:
            // Lines must have the same orientation to be merged.
            if (evaluate_type(plain_view_geometries[geometry_to_simplify.geometries.front()],
                              plain_view_geometries[i]) &&
                plain_view_geometries[i - 1].orientation == plain_view_geometries[i].orientation &&
                evaluate_tolerance(plain_view_geometries[geometry_to_simplify.geometries.front()],
                                   plain_view_geometries[i], length)) {
              geometry_to_simplify.geometries.push_back(i);
              length += plain_view_geometries[i].length;
              geometry_to_simplify.action = get_action_from_geometries(geometry_to_simplify, plain_view_geometries);
            } else {
              if (has_more_than_one(geometry_to_simplify)) {
                geometries_to_simplify.push_back(geometry_to_simplify);
              }
              reset_local_vars(road_header_id_road_header.first, plain_view_geometries[i].length,
                               &geometry_to_simplify);
              // Sets the current geometry.
              geometry_to_simplify.geometries.push_back(i);
              geometry_to_simplify.action = get_action_from_geometries(geometry_to_simplify, plain_view_geometries);
            }
            break;

          case Geometry::Type::kArc:
            // Arcs must have the same curvature to be merged.
            if (evaluate_type(plain_view_geometries[geometry_to_simplify.geometries.front()],
                              plain_view_geometries[i]) &&
                std::get<Geometry::Arc>(plain_view_geometries[i - 1].description).curvature ==
                    std::get<Geometry::Arc>(plain_view_geometries[i].description).curvature &&
                evaluate_tolerance(plain_view_geometries[geometry_to_simplify.geometries.front()],
                                   plain_view_geometries[i], length)) {
              geometry_to_simplify.geometries.push_back(i);
              length += plain_view_geometries[i].length;
              geometry_to_simplify.action = get_action_from_geometries(geometry_to_simplify, plain_view_geometries);
            } else {
              if (has_more_than_one(geometry_to_simplify)) {
                geometries_to_simplify.push_back(geometry_to_simplify);
              }
              reset_local_vars(road_header_id_road_header.first, plain_view_geometries[i].length,
                               &geometry_to_simplify);
              // Sets the current geometry.
              geometry_to_simplify.geometries.push_back(i);
              geometry_to_simplify.action = get_action_from_geometries(geometry_to_simplify, plain_view_geometries);
            }
            break;

          default:
            MALIDRIVE_THROW_MESSAGE("Unrecognized geometry at Road: " + road_header_id_road_header.first.string());
            break;
        }
      }
      if (has_more_than_one(geometry_to_simplify)) {
        geometries_to_simplify.push_back(geometry_to_simplify);
      }
    }
    return geometries_to_simplify;
  }

 private:
  // Get all the lanes from the i-th lane section of `road_header_id`.
  // @param `road_header_id` Is a RoadHeader::Id.
  // @param `lane_section_index` Is a i-th lane section of the road.
  // @returns Returns all the lanes of the lane section.
  //
  // @throws maliput::common::assertion_error When `road_header_id` doesn't exist.
  // @throws maliput::common::assertion_error When `lane_section_index` is negative.
  std::map<Lane::Id, const Lane*> GetLanesFromLaneSection(const RoadHeader::Id& road_header_id,
                                                          int lane_section_index) {
    MALIDRIVE_THROW_UNLESS(road_headers_.find(road_header_id) != road_headers_.end());
    MALIDRIVE_THROW_UNLESS(lane_section_index >= 0);

    std::map<Lane::Id, const Lane*> lanes;
    const auto& lane_section = road_headers_.at(road_header_id).lanes.lanes_section[lane_section_index];
    // Left lanes.
    for (const auto& lane : lane_section.left_lanes) {
      lanes.emplace(lane.id, &lane);
    }
    // Center lane.
    lanes.emplace(lane_section.center_lane.id, &lane_section.center_lane);
    // Right lanes.
    for (const auto& lane : lane_section.right_lanes) {
      lanes.emplace(lane.id, &lane);
    }
    return lanes;
  }

  // Get the RoadHeader of the link road.
  // @param road_header_id Is the id of the road. Used only for logging purposes.
  // @param link Contains the information of the predecessor or successor road of `road_header_id`. Used only for
  // logging purposes.
  // @param is_predecessor True when `link` is a predecessor.
  // @returns The RoadHeader of the RoadLink of `road_header_id`.
  //
  // @throw maliput::common::assertion_error When predecessor/successor Road ids doesn't exists.
  const RoadHeader& GetRoadHeaderFromLink(const RoadHeader::Id& road_header_id, const RoadLink::LinkAttributes& link,
                                          bool is_predecessor) {
    maliput::common::unused(road_header_id);
    maliput::common::unused(is_predecessor);
    const auto road_header_link = road_headers_.find(RoadHeader::Id{link.element_id.string()});
    if (road_header_link == road_headers_.end()) {
      MALIDRIVE_THROW_MESSAGE(std::string("Unknown ") +
                              (is_predecessor ? std::string("predecessor ") : std::string("successor ")) +
                              link.element_id.string() + " in road " + road_header_id.string());
    }
    return road_header_link->second;
  }

  // @returns A vector of Connection pointers with `road_header_id` as incoming road or connecting road.
  // @param road_header_id Is the ID of the RoadHeader.
  // @param junction Is the junction that contains the connections.
  // @param is_incoming_road Is True when the `road_header_id` is an incoming road. And False when it is connecting
  // road.
  //
  // @throw maliput::common::assertion_error When `road_header_id` is not present in the junction.
  const std::vector<const Connection*> GetConnectionsByRoadId(const RoadHeader::Id& road_header_id,
                                                              const Junction& junction, bool is_incoming_road) {
    const auto& connections = junction.connections;
    std::vector<const Connection*> matched_connections;
    for (const auto& connection : connections) {
      if (RoadHeader::Id(is_incoming_road ? connection.second.incoming_road : connection.second.connecting_road) ==
          road_header_id) {
        matched_connections.push_back(&connection.second);
      }
    }
    return matched_connections;
  }

  // Verify that the lanes described in the connection actually exist in the road.
  // @param connection Connection in which `road_header` appears as an incoming road or as a connecting road.
  // @param incoming_road True when `road_header` is an incoming road in the connection. False when it is a connecting
  // road.
  // @param road_header Is the RoadHeader.
  // @param is_predecessor True when the connection under analysis link to the `road_header` predecessor.
  //
  // @throw maliput::common::assertion_error When lanes within a connection and the road's lanes don't match.
  // @throw maliput::common::assertion_error When `connection` is nullptr.
  void VerifyRoadLanesWithConnectionLanes(const Connection* connection, bool incoming_road,
                                          const RoadHeader& road_header, bool is_predecessor) {
    MALIDRIVE_THROW_UNLESS(connection != nullptr);
    const auto lane_ids_lanes = GetLanesFromLaneSection(
        road_header.id, is_predecessor ? 0 : (static_cast<int>(road_header.lanes.lanes_section.size()) - 1));
    for (const auto& lane_link : connection->lane_links) {
      const Lane::Id lane_id{incoming_road ? lane_link.from.string() : lane_link.to.string()};
      if (lane_ids_lanes.find(lane_id) == lane_ids_lanes.end()) {
        MALIDRIVE_THROW_MESSAGE(std::string("The Lane whose Id is: ") + lane_id.string() +
                                std::string(" belongs to Connection Id: ") + connection->id.string() +
                                std::string(" doesn't exist within the Road Id: ") + road_header.id.string());
      }
    }
  }

  // Verify that the Road's lanes match with the lanes in the predecessor/successor Road.
  // @param road_header Is the RoadHeader.
  // @param link Contains the information of the predecessor or successor road of `road_header`.
  // @param is_predecessor True when `link` is a predecessor.
  // @param road_header_link Is the Road that is connected to `road_header`.
  //
  // @throw maliput::common::assertion_error When `road_header` and `road_header_link`s Lane ids doesn't match.
  void VerifyLanesBetweenRoadAndRoadLink(const RoadHeader& road_header, const RoadLink::LinkAttributes& link,
                                         bool is_predecessor, const RoadHeader& road_header_link) {
    const auto lane_ids_lanes = GetLanesFromLaneSection(
        road_header.id, is_predecessor ? 0 : (static_cast<int>(road_header.lanes.lanes_section.size()) - 1));
    const auto lane_ids_lanes_link = GetLanesFromLaneSection(
        road_header_link.id, link.contact_point.value() == RoadLink::ContactPoint::kEnd
                                 ? (static_cast<int>(road_header_link.lanes.lanes_section.size()) - 1)
                                 : 0);
    for (const auto& lane_id_lane : lane_ids_lanes) {
      const std::optional<LaneLink::LinkAttributes> lane_link =
          is_predecessor ? lane_id_lane.second->lane_link.predecessor : lane_id_lane.second->lane_link.successor;
      if (lane_link.has_value() &&
          lane_ids_lanes_link.find(Lane::Id(lane_link->id.string())) == lane_ids_lanes_link.end()) {
        MALIDRIVE_THROW_MESSAGE(std::string("Unknown ") +
                                (is_predecessor ? std::string("predecessor lane ") : std::string("successor lane ")) +
                                lane_id_lane.first.string() + " in road " + road_header.id.string());
      }
    }
  }

  // Verify that the RoadLinking is reciprocal in the following cases:
  //  - `road_header` doesn't belong to a junction and is connected to `road_header_link`.
  //  - `road_header` belongs to a junction and is connected to `road_header_link`.
  //
  // @param road_header Is the RoadHeader.
  // @param link Contains the information of the predecessor or successor road of `road_header`.
  // @param road_header_link Is the Road that is connected to `road_header`.
  //
  // @throw maliput::common::assertion_error When `link.contact_point` doesn't have a value.
  // @throw maliput::common::assertion_error When the connection is not reciprocal between `road_header` and
  // `road_header_link` iff the semantic errors in the xodr aren't allowed.
  void VerifyLinkingIsReciprocal(const RoadHeader& road_header, const RoadLink::LinkAttributes& link,
                                 const RoadHeader& road_header_link) {
    MALIDRIVE_THROW_UNLESS(link.contact_point.has_value());
    const auto link_s_road_link = link.contact_point.value() == RoadLink::ContactPoint::kEnd
                                      ? road_header_link.road_link.successor
                                      : road_header_link.road_link.predecessor;
    MALIDRIVE_THROW_UNLESS(link_s_road_link.has_value());
    if (link_s_road_link->element_type == RoadLink::ElementType::kRoad) {
      if (RoadHeader::Id(link_s_road_link->element_id.string()) != road_header.id) {
        const std::string msg{"RoadHeader(" + road_header.id.string() + ") has a link pointing to RoadHeader(" +
                              road_header_link.id.string() +
                              (link.contact_point.value() == RoadLink::ContactPoint::kEnd
                                   ? ") but its successor is not reciprocal. It points to RoadHeader("
                                   : ") but its predecessor is not reciprocal. It points to RoadHeader(") +
                              link_s_road_link->element_id.string() + ")"};
        if (parser_configuration_.allow_semantic_errors) {
          maliput::log()->warn(msg);
        } else {
          MALIDRIVE_THROW_MESSAGE(msg);
        }
      }
    } else {
      if (Junction::Id(link_s_road_link->element_id.string()) != Junction::Id(road_header.junction)) {
        const std::string msg{"RoadHeader(" + road_header.id.string() + ") has a link pointing to RoadHeader(" +
                              road_header_link.id.string() +
                              (link.contact_point.value() == RoadLink::ContactPoint::kEnd
                                   ? ") but its successor is not reciprocal. It points to JunctionId("
                                   : ") but its predecessor is not reciprocal. It points to JunctionId(") +
                              link_s_road_link->element_id.string() + ") and RoadHeader(" + road_header.id.string() +
                              ") has JunctionId(" + road_header.junction + ")"};
        if (parser_configuration_.allow_semantic_errors) {
          maliput::log()->warn(msg);
        } else {
          MALIDRIVE_THROW_MESSAGE(msg);
        }
      }
    }
  }

  // Verifies that `road_header`'s `link` is consistent:
  //   - RoadLink points at a valid RoadHeader::Id or Junction::Id.
  //   - Linked Roads/junctions are also linked to the `road_header`.
  //   - Lanes' LaneLink are consistent between roads.
  // @param road_header Is the `road_header` to be analyzed.
  // @param link Contains the information of the predecessor or successor road of `road_header`.
  // @param is_predecessor True when `link` is a predecessor.
  //
  // @throw maliput::common::assertion_error When predecessor/successor Road ids doesn't exists.
  // @throw maliput::common::assertion_error When predecessor/successor Lane ids doesn't exists.
  void VerifyRoadLinks(const RoadHeader& road_header, const RoadLink::LinkAttributes& link, bool is_predecessor) {
    maliput::log()->debug("VerifyRoadLinks(Road({}), Link({}({})), {})", road_header.id.string(),
                          (link.element_type == RoadLink::ElementType::kRoad ? "Road" : "Junction"),
                          link.element_id.string(), (is_predecessor ? "predecessor" : "successor"));
    if (std::stoi(road_header.junction) < 0) {
      // Road doesn't belong to a junction.
      if (link.element_type == RoadLink::ElementType::kRoad) {
        // Linked to a Road.
        // Get RoadHeader from link.
        const auto road_header_link = GetRoadHeaderFromLink(road_header.id, link, is_predecessor);
        // Verify that road_header_link is also link to road_header.
        VerifyLinkingIsReciprocal(road_header, link, road_header_link);
        // Verify that the lane links match with lanes in the road link.
        VerifyLanesBetweenRoadAndRoadLink(road_header, link, is_predecessor, road_header_link);
      } else {
        // Linked to a Junction.
        // Verify that the connecting junction exists.
        const auto junction_link = junctions_.find(Junction::Id{link.element_id.string()});
        if (junction_link == junctions_.end()) {
          MALIDRIVE_THROW_MESSAGE(
              std::string("Unknown ") + (is_predecessor ? std::string("predecessor ") : std::string("successor ")) +
              std::string("junction id: ") + link.element_id.string() + " in road " + road_header.id.string());
        }
        // Verify road_header.id to be as an incoming road inside the connection map.
        const auto connections =
            GetConnectionsByRoadId(road_header.id, junction_link->second, true /* incoming road */);
        if (connections.size() == 0) {
          maliput::log()->debug(
              "Connection missing for Junction id {}. There is no incoming road id: {} that matches with a connection "
              "road id: {}",
              junction_link->first.string(), road_header.id.string(), link.element_id);
        } else {
          for (const auto& connection : connections) {
            // Verify that the lanes described in the connection actually exist in the road.
            VerifyRoadLanesWithConnectionLanes(connection, true, road_header, is_predecessor);
          }
        }
      }
    } else {
      // Road belongs to a junction.
      if (link.element_type == RoadLink::ElementType::kRoad) {
        // Linked to a Road.
        // Verifies that the road's junction exists.
        const auto junction = junctions_.find(Junction::Id{road_header.junction});
        if (junction == junctions_.end()) {
          MALIDRIVE_THROW_MESSAGE(std::string("Road Id: ") + road_header.id.string() +
                                  std::string(" belongs to a unknown junction."));
        }
        // Verify road_header.id to be a connection road inside the connection map.
        const auto connections = GetConnectionsByRoadId(road_header.id, junction->second, false /* connecting road */);
        // We only want the connection where the incoming road matches with`link.element_id`.
        const auto connection_it = std::find_if(connections.begin(), connections.end(), [&link](const Connection* c) {
          return c->incoming_road == link.element_id.string();
        });
        if (connection_it == connections.end()) {
          maliput::log()->debug(
              "Connection missing for Junction id: {}. There is no connection road id: {} that matches with an "
              "incoming road id: {}",
              junction->first.string(), road_header.id.string(), link.element_id);
        } else {
          // Verify that the lanes described in the connection actually exist in the road.
          VerifyRoadLanesWithConnectionLanes(*connection_it, false, road_header, is_predecessor);
        }
        // Get RoadHeader from link.
        const auto road_header_link = GetRoadHeaderFromLink(road_header.id, link, is_predecessor);
        // Verify road_header_link is connected to the junction.
        VerifyLinkingIsReciprocal(road_header, link, road_header_link);
        // Verify that the lane links match with lanes in the road link.
        VerifyLanesBetweenRoadAndRoadLink(road_header, link, is_predecessor, road_header_link);
      } else {
        // Linked to a Junction.
        MALIDRIVE_THROW_MESSAGE("Junctions linked to junctions are not supported.");
      }
    }
  }

  // Verifies that the junctions' connection map contains existent roads.
  // @note LaneLinks are verified in the VerifyRoadLinks method.
  //
  // @see VerifyRoadLinks.
  // @throws maliput::common::assertion_error When the connection's roads are not found.
  void VerifyJunctions() {
    for (const auto& junction : junctions_) {
      for (const auto& connection : junction.second.connections) {
        const auto incoming_road_header = road_headers_.find(RoadHeader::Id(connection.second.incoming_road));
        if (incoming_road_header == road_headers_.end()) {
          MALIDRIVE_THROW_MESSAGE(std::string("When verifying JunctionId: ") + junction.first.string() +
                                  std::string(" , connection: ") + connection.first.string() +
                                  std::string(" the incoming RoadHeaderId: ") + connection.second.incoming_road +
                                  std::string(" could not be found."));
        }
        const auto connecting_road_header = road_headers_.find(RoadHeader::Id(connection.second.connecting_road));
        if (connecting_road_header == road_headers_.end()) {
          MALIDRIVE_THROW_MESSAGE(std::string("When verifying JunctionId: ") + junction.first.string() +
                                  std::string(" , connection: ") + connection.first.string() +
                                  std::string(" the connecting road RoadHeaderId: ") +
                                  connection.second.connecting_road + std::string(" could not be found."));
        }
        if (Junction::Id(connecting_road_header->second.junction) != junction.first) {
          MALIDRIVE_THROW_MESSAGE(
              std::string("When verifying JunctionId: ") + junction.first.string() +
              std::string(" , the junction of the connecting road ") + connecting_road_header->second.id.string() +
              std::string(" is: ") + Junction::Id(connecting_road_header->second.junction).string() +
              std::string(" which does not match the expected junction id: ") + junction.first.string());
        }
      }
    }
  }

  // Complete missing LaneLinks connections.
  // When Junction's LaneLinks' are not shown in the xodr description mean that all incoming lanes
  // are linked to lanes with identical IDs on the connecting road.
  //
  // @throws maliput::common::assertion_error When the incoming road and connecting road within a Connection don't
  //                                          match their lanes id.
  void CompleteJunctionsLaneLinks() {
    for (auto& junction : junctions_) {
      for (auto& connection : junction.second.connections) {
        if (connection.second.lane_links.size() != 0) {
          continue;
        }
        // Get lanes from the incoming road.
        const auto incoming_road = road_headers_.at(RoadHeader::Id(connection.second.incoming_road));
        int lane_section_index_incoming_road{};
        if (incoming_road.road_link.predecessor.has_value() &&
            (incoming_road.road_link.predecessor->element_id.string() == junction.first.string())) {
          lane_section_index_incoming_road = 0;
        } else if (incoming_road.road_link.successor.has_value() &&
                   (incoming_road.road_link.successor->element_id.string() == junction.first.string())) {
          lane_section_index_incoming_road = static_cast<int>(incoming_road.lanes.lanes_section.size()) - 1;
        } else {
          MALIDRIVE_THROW_MESSAGE(std::string("Junction Id: ") + junction.first.string() +
                                  std::string(" & Connection Id: ") + connection.first.string() +
                                  std::string(" --> Incoming and Connecting Roads don't match."));
        }
        const auto incoming_road_lanes_id = GetLanesFromLaneSection(incoming_road.id, lane_section_index_incoming_road);

        // Get lanes from the connecting road.
        const auto connecting_road = road_headers_.at(RoadHeader::Id(connection.second.connecting_road));
        const auto connecting_road_lanes_id = GetLanesFromLaneSection(
            connecting_road.id, connection.second.contact_point == Connection::ContactPoint::kStart
                                    ? 0
                                    : (static_cast<int>(connecting_road.lanes.lanes_section.size()) - 1));
        MALIDRIVE_THROW_UNLESS(incoming_road_lanes_id.size() == connecting_road_lanes_id.size());
        // Compare lanes and create lane links.
        for (const auto& incoming_lane : incoming_road_lanes_id) {
          if (incoming_lane.first == Lane::Id("0")) {
            continue;
          }
          const auto connecting_lane_it = connecting_road_lanes_id.find(incoming_lane.first);
          if (connecting_lane_it == connecting_road_lanes_id.end()) {
            MALIDRIVE_THROW_MESSAGE(std::string("Junction Id: ") + junction.first.string() +
                                    std::string(" & Connection Id: ") + connection.first.string() +
                                    std::string(" --> Incoming and Connecting Roads' lanes don't match."));
          }
          const Connection::LaneLink lane_link{Connection::LaneLink::Id(incoming_lane.first.string()),
                                               Connection::LaneLink::Id(incoming_lane.first.string())};
          connection.second.lane_links.push_back(lane_link);
        }
      }
    }
  }

  // Verify lane links between two lane sections's lanes.
  // @param lanes_a Contains lanes of a LaneSection A.
  // @pre This LaneSection must be predecessor of B.
  // @param lanes_b Contains lanes of a LaneSection B.
  // @pre This LaneSection must be successor of A.
  // @throws maliput::common::assertion_error When lane links between both LaneSections are not coherent.
  void VerifyLaneLinksBetweenTwoLaneSections(const std::map<Lane::Id, const Lane*>& lanes_a,
                                             const std::map<Lane::Id, const Lane*>& lanes_b) {
    // Check successor lanes of `lanes_a`.
    for (const auto& lane_id_lane_a : lanes_a) {
      const std::optional<LaneLink::LinkAttributes> lane_link_a = lane_id_lane_a.second->lane_link.successor;
      if (lane_link_a.has_value()) {
        const auto lane_id_lane_b = lanes_b.find(Lane::Id(lane_link_a->id.string()));
        if (lane_id_lane_b == lanes_b.end()) {
          const std::string msg = std::string("Unknown successor lane ") + lane_link_a->id.string() +
                                  std::string(" for lane id ") + lane_id_lane_a.first.string();
          if (parser_configuration_.allow_semantic_errors) {
            maliput::log()->warn(msg);
            continue;
          } else {
            MALIDRIVE_THROW_MESSAGE(msg);
          }
        }
        const std::optional<LaneLink::LinkAttributes> lane_link_b = lane_id_lane_b->second->lane_link.predecessor;
        if (!lane_link_b.has_value() || (lane_link_b->id.string() != lane_id_lane_a.first.string())) {
          const std::string msg = "Lane id " + lane_id_lane_a.first.string() +
                                  " from one segment doesn't match successor/predecessor values with Lane id " +
                                  lane_id_lane_b->first.string() + " of the next segment.";
          if (parser_configuration_.allow_semantic_errors) {
            maliput::log()->warn(msg);
            continue;
          } else {
            MALIDRIVE_THROW_MESSAGE(msg);
          }
        }
      }
    }
    // Check predecessor lanes of `lanes_b`.
    for (const auto& lane_id_lane_b : lanes_b) {
      const std::optional<LaneLink::LinkAttributes> lane_link_b = lane_id_lane_b.second->lane_link.predecessor;
      if (lane_link_b.has_value()) {
        const auto lane_id_lane_a = lanes_a.find(Lane::Id(lane_link_b->id.string()));
        if (lane_id_lane_a == lanes_a.end()) {
          const std::string msg = std::string("Unknown predecessor lane ") + lane_link_b->id.string() +
                                  std::string(" for lane id ") + lane_id_lane_b.first.string();
          if (parser_configuration_.allow_semantic_errors) {
            maliput::log()->warn(msg);
            continue;
          } else {
            MALIDRIVE_THROW_MESSAGE(msg);
          }
        }
        const std::optional<LaneLink::LinkAttributes> lane_link_a = lane_id_lane_a->second->lane_link.successor;
        if (!lane_link_a.has_value() || (lane_link_a->id.string() != lane_id_lane_b.first.string())) {
          const std::string msg = "Lane id " + lane_id_lane_b.first.string() +
                                  " from one segment doesn't match successor/predecessor values with Lane id " +
                                  lane_id_lane_a->first.string() + " of the next segment.";
          if (parser_configuration_.allow_semantic_errors) {
            maliput::log()->warn(msg);
            continue;
          } else {
            MALIDRIVE_THROW_MESSAGE(msg);
          }
        }
      }
    }
  }

  // Verify that the internal links within a `road_header` are consistent.
  // @param road_header Contains all the LaneSections of a Road.
  //
  // @throws maliput::common::assertion_error When LaneLinks are not coherent between LaneSections.
  void VerifyLinksBetweenLaneSectionsOfARoad(const RoadHeader& road_header) {
    maliput::log()->trace("Verify lane links within Road Id: {}", road_header.id.string());
    for (int i = 0; i < static_cast<int>(road_header.lanes.lanes_section.size()) - 1; i++) {
      maliput::log()->trace("Verify lane links between lane section {} and {}", i, i + 1);
      VerifyLaneLinksBetweenTwoLaneSections(GetLanesFromLaneSection(road_header.id, i),
                                            GetLanesFromLaneSection(road_header.id, i + 1));
    }
  }

  // Analyze the geometries' length in order to look for the domain range value.
  // Once found, RoadHeader::Id and the index of the geometry within planView is saved too.
  // @throws maliput::common::assertion_error When there are no RoadHeaders.
  // @throws maliput::common::assertion_error When there is no geometries described within a Road.
  void AnalyzeGeometriesLength() {
    MALIDRIVE_THROW_UNLESS(!road_headers_.empty());
    for (const auto& road_header : road_headers_) {
      MALIDRIVE_THROW_UNLESS(!road_header.second.reference_geometry.plan_view.geometries.empty());
      int index{};
      for (const auto& geometry : road_header.second.reference_geometry.plan_view.geometries) {
        if (geometry.length < shortest_geometry_.length) {
          shortest_geometry_ = DBManager::XodrGeometryLengthData{road_header.first, index, geometry.length};
        }
        if (geometry.length > largest_geometry_.length) {
          largest_geometry_ = DBManager::XodrGeometryLengthData{road_header.first, index, geometry.length};
        }
        index++;
      }
    }
  }

  // Analyze the LaneSection' length in order to look for the domain range value.
  // Once found, RoadHeader::Id and the index of the LaneSection is saved too.
  // @throws maliput::common::assertion_error When there are no RoadHeaders.
  // @throws maliput::common::assertion_error When there is no LaneSection described within a Road.
  void AnalyzeLaneSectionsLength() {
    MALIDRIVE_THROW_UNLESS(!road_headers_.empty());
    for (const auto& road_header : road_headers_) {
      const int size{static_cast<int>(road_header.second.lanes.lanes_section.size())};
      MALIDRIVE_THROW_UNLESS(size > 0);
      for (int index = 0; index < size; ++index) {
        const double length{road_header.second.GetLaneSectionLength(index)};
        if (length < shortest_lane_section_.length) {
          shortest_lane_section_ = DBManager::XodrLaneSectionLengthData{road_header.first, index, length};
        }
        if (length > largest_lane_section_.length) {
          largest_lane_section_ = DBManager::XodrLaneSectionLengthData{road_header.first, index, length};
        }
      }
    }
  }

  // Analyze the Geometries' startpoint and endpoint within all Roads, in order to look for the minimum
  // and maximum distance between Geometries in the entire XODR description.
  // Once found, RoadHeader::Id and the indexes of the geometries involved in the gap are saved too.
  // @throws maliput::common::assertion_error When there are no RoadHeaders.
  // @throws maliput::common::assertion_error When there is no geometries described within a Road.
  void AnalyzeGapBetweenGeometries() const {
    MALIDRIVE_THROW_UNLESS(!road_headers_.empty());
    XodrGapBetweenGeometries shortest_gap{RoadHeader::Id("none"), {0, 0}, std::numeric_limits<double>::infinity()};
    XodrGapBetweenGeometries largest_gap{RoadHeader::Id("none"), {0, 0}, 0.};
    for (const auto& road_header : road_headers_) {
      MALIDRIVE_THROW_UNLESS(!road_header.second.reference_geometry.plan_view.geometries.empty());
      const auto geometries = road_header.second.reference_geometry.plan_view.geometries;
      for (int index = 0; index < static_cast<int>(geometries.size()) - 1; index++) {
        const double distance = GetDistanceBetweenGeometries(geometries[index], geometries[index + 1]);
        if (distance < shortest_gap.distance) {
          shortest_gap = XodrGapBetweenGeometries{road_header.first, {index, index + 1}, distance};
        }
        if (distance > largest_gap.distance) {
          largest_gap = XodrGapBetweenGeometries{road_header.first, {index, index + 1}, distance};
        }
      }
    }
    shortest_gap_ = std::move(shortest_gap);
    largest_gap_ = std::move(largest_gap);
  }

  // Analyze the elevations' startpoint and endpoint within all Roads, in order
  // to look for the minimum and maximum distance between elevations in the
  // entire XODR description.
  // Once found, RoadHeader::Id and the indexes of the involved elevation
  // functions are saved as well.
  void AnalyzeGapBetweenElevationFunction() const {
    MALIDRIVE_THROW_UNLESS(!road_headers_.empty());
    XodrGapBetweenFunctions largest_elevation_gap{RoadHeader::Id("none"), {0, 0}, 0.};
    XodrGapBetweenFunctions shortest_elevation_gap{
        RoadHeader::Id("none"), {0, 0}, std::numeric_limits<double>::infinity()};
    for (const auto& road_header : road_headers_) {
      const auto functions = road_header.second.reference_geometry.elevation_profile.elevations;
      for (int index = 0; index < static_cast<int>(functions.size()) - 1; index++) {
        const double distance =
            GetDistanceBetweenFunctions<ElevationProfile::Elevation>(functions[index], functions[index + 1]);
        if (distance < shortest_elevation_gap.distance) {
          shortest_elevation_gap = XodrGapBetweenFunctions{road_header.first, {index, index + 1}, distance};
        }
        if (distance > largest_elevation_gap.distance) {
          largest_elevation_gap = XodrGapBetweenFunctions{road_header.first, {index, index + 1}, distance};
        }
      }
    }
    shortest_elevation_gap_ = std::move(shortest_elevation_gap);
    largest_elevation_gap_ = std::move(largest_elevation_gap);
  }

  // Analyze the superelevations' startpoint and endpoint within all Roads, in
  // order to look for the minimum and maximum distance between superelevations
  // in the entire XODR description.
  // Once found, RoadHeader::Id and the indexes of the involved superelevation
  // functions are saved as well.
  void AnalyzeGapBetweenSuperelevationFunction() const {
    MALIDRIVE_THROW_UNLESS(!road_headers_.empty());
    XodrGapBetweenFunctions largest_superelevation_gap{RoadHeader::Id("none"), {0, 0}, 0.};
    XodrGapBetweenFunctions shortest_superelevation_gap{
        RoadHeader::Id("none"), {0, 0}, std::numeric_limits<double>::infinity()};
    for (const auto& road_header : road_headers_) {
      const auto functions = road_header.second.reference_geometry.lateral_profile.superelevations;
      for (int index = 0; index < static_cast<int>(functions.size()) - 1; index++) {
        const double distance =
            GetDistanceBetweenFunctions<LateralProfile::Superelevation>(functions[index], functions[index + 1]);
        if (distance < shortest_superelevation_gap.distance) {
          shortest_superelevation_gap = XodrGapBetweenFunctions{road_header.first, {index, index + 1}, distance};
        }
        if (distance > largest_superelevation_gap.distance) {
          largest_superelevation_gap = XodrGapBetweenFunctions{road_header.first, {index, index + 1}, distance};
        }
      }
    }
    shortest_superelevation_gap_ = std::move(shortest_superelevation_gap);
    largest_superelevation_gap_ = std::move(largest_superelevation_gap);
  }

  // Optional tolerance.
  ParserConfiguration parser_configuration_{};
  // Header of the XODR map.
  Header header_{};
  // Holds the RoadHeaders of the XODR map.
  std::map<RoadHeader::Id, RoadHeader> road_headers_{};
  // Holds the Junctions of the XODR map.
  std::unordered_map<Junction::Id, Junction> junctions_{};
  // {@ Holds data of the shortest and largest geometries.
  XodrGeometryLengthData shortest_geometry_{RoadHeader::Id("none"), 0, std::numeric_limits<double>::infinity()};
  XodrGeometryLengthData largest_geometry_{RoadHeader::Id("none"), 0, 0.};
  // @}

  // {@ Holds data of the shortest and largest LaneSections.
  XodrLaneSectionLengthData shortest_lane_section_{RoadHeader::Id("none"), 0, std::numeric_limits<double>::infinity()};
  XodrLaneSectionLengthData largest_lane_section_{RoadHeader::Id("none"), 0, 0.};
  // @}

  // @{ Holds data of the shortest and largest gaps between geometries.
  mutable std::optional<XodrGapBetweenGeometries> shortest_gap_{std::nullopt};
  mutable std::optional<XodrGapBetweenGeometries> largest_gap_{std::nullopt};
  // @}

  // @{ Holds data of the shortest and largest gaps between elevation functions.
  mutable std::optional<XodrGapBetweenFunctions> largest_elevation_gap_{std::nullopt};
  mutable std::optional<XodrGapBetweenFunctions> shortest_elevation_gap_{std::nullopt};
  // @}

  // @{ Holds data of the shortest and largest gaps between superelevation
  //    functions.
  mutable std::optional<XodrGapBetweenFunctions> largest_superelevation_gap_{std::nullopt};
  mutable std::optional<XodrGapBetweenFunctions> shortest_superelevation_gap_{std::nullopt};
  // @}
};

DBManager::~DBManager() = default;

DBManager::DBManager(tinyxml2::XMLDocument* xodr_doc, const ParserConfiguration& parser_configuration)
    : impl_(std::make_unique<Impl>(parser_configuration)) {
  MALIDRIVE_THROW_UNLESS(xodr_doc != nullptr);
  impl_->ParseDoc(xodr_doc);
}

const Header& DBManager::GetXodrHeader() const { return impl_->get_header(); }

const std::map<RoadHeader::Id, RoadHeader>& DBManager::GetRoadHeaders() const { return impl_->get_road_headers(); };

const std::unordered_map<Junction::Id, Junction>& DBManager::GetJunctions() const { return impl_->get_junctions(); };

const DBManager::XodrGeometryLengthData& DBManager::GetShortestGeometry() const {
  return impl_->get_shortest_geometry();
};

const DBManager::XodrGeometryLengthData& DBManager::GetLargestGeometry() const {
  return impl_->get_largest_geometry();
};

const DBManager::XodrLaneSectionLengthData& DBManager::GetShortestLaneSection() const {
  return impl_->get_shortest_lane_section();
};

const DBManager::XodrLaneSectionLengthData& DBManager::GetLargestLaneSection() const {
  return impl_->get_largest_lane_section();
};

const DBManager::XodrGapBetweenGeometries& DBManager::GetShortestGap() const { return impl_->get_shortest_gap(); };

const DBManager::XodrGapBetweenGeometries& DBManager::GetLargestGap() const { return impl_->get_largest_gap(); };

const DBManager::XodrGapBetweenFunctions& DBManager::GetLargestElevationGap() const {
  return impl_->get_largest_elevation_gap();
}

const DBManager::XodrGapBetweenFunctions& DBManager::GetShortestElevationGap() const {
  return impl_->get_shortest_elevation_gap();
}

const DBManager::XodrGapBetweenFunctions& DBManager::GetShortestSuperelevationGap() const {
  return impl_->get_shortest_superelevation_gap();
}

const DBManager::XodrGapBetweenFunctions& DBManager::GetLargestSuperelevationGap() const {
  return impl_->get_largest_superelevation_gap();
}

const std::vector<DBManager::XodrGeometriesToSimplify> DBManager::GetGeometriesToSimplify(double tolerance) const {
  return impl_->GetGeometriesToSimplify(tolerance);
}

std::unique_ptr<DBManager> LoadDataBaseFromFile(const std::string& filepath,
                                                const ParserConfiguration& parser_configuration) {
  tinyxml2::XMLDocument xodr_doc;
  MALIDRIVE_VALIDATE(xodr_doc.LoadFile(filepath.c_str()) == tinyxml2::XML_SUCCESS, maliput::common::assertion_error,
                     std::string("XODR file couldn't be loaded: ") + filepath.c_str());
  return std::make_unique<DBManager>(&xodr_doc, parser_configuration);
}

std::unique_ptr<DBManager> LoadDataBaseFromStr(const std::string& xodr_str,
                                               const ParserConfiguration& parser_configuration) {
  tinyxml2::XMLDocument xodr_doc;
  MALIDRIVE_THROW_UNLESS(xodr_doc.Parse(xodr_str.c_str()) == tinyxml2::XML_SUCCESS);
  return std::make_unique<DBManager>(&xodr_doc, parser_configuration);
}

}  // namespace xodr
}  // namespace malidrive
