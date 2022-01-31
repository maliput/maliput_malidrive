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
  /// Creates a RoadNetworkConfiguration out of a string dictionary.
  /// @details The keys of the map are listed at @ref road_network_configuration_builder_keys.
  /// @param road_network_configuration A string-string map containing the configuration for the builder.
  static RoadNetworkConfiguration FromMap(const std::map<std::string, std::string>& road_network_configuration);

  /// @details The keys of the map are listed at @ref road_network_configuration_builder_keys.
  /// @returns A string-string map containing the RoadGeometry configuration.
  std::map<std::string, std::string> ToStringMap() const;

  /// Road geometry configuration parameters.
  const RoadGeometryConfiguration road_geometry_configuration;
  /// Path to the configuration file to load a RuleRegistry
  std::optional<std::string> rule_registry{std::nullopt};
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
