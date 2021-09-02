// Copyright 2019 Toyota Research Institute
#pragma once

#include <map>
#include <optional>
#include <string>

#include "maliput_malidrive/builder/road_geometry_configuration.h"

namespace malidrive {
namespace builder {

/// Structure to hold file paths for multiple RoadNetwork structures.
struct RoadNetworkConfiguration {
  /// @name Road Network configuration's keys used in string map.
  ///
  /// @{
  static constexpr char const* kStrRoadRuleBook = "road_rule_book";
  static constexpr char const* kStrTrafficLightBook = "traffic_light_book";
  static constexpr char const* kStrPhaseRingBook = "phase_ring_book";
  static constexpr char const* kStrIntersectionBook = "intersection_book";
  /// @}

  /// Creates a RoadNetworkConfiguration out of a string dictionary.
  /// @param road_network_configuration A string-string map containing the configuration for the builder.
  static RoadNetworkConfiguration FromMap(const std::map<std::string, std::string>& road_network_configuration);

  /// @returns A string-string map containing the RoadGeometry configuration.
  std::map<std::string, std::string> ToStringMap() const;

  /// Road geometry configuration parameters.
  const RoadGeometryConfiguration road_geometry_configuration;
  /// Path to the configuration file to load a RoadRulebook
  std::optional<std::string> road_rule_book{std::nullopt};
  /// Path to the configuration file to load a TrafficLightBook.
  std::optional<std::string> traffic_light_book{std::nullopt};
  /// Path to the configuration file to load a PhaseRingBook.
  std::optional<std::string> phase_ring_book{std::nullopt};
  /// Path to the configuration file to load an IntersectionBook.
  std::optional<std::string> intersection_book{std::nullopt};
};

}  // namespace builder
}  // namespace malidrive
