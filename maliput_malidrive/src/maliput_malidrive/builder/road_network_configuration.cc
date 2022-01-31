// Copyright 2021 Toyota Research Institute

#include "maliput_malidrive/builder/road_network_configuration.h"

#include "maliput_malidrive/builder/params.h"

namespace malidrive {
namespace builder {

RoadNetworkConfiguration RoadNetworkConfiguration::FromMap(
    const std::map<std::string, std::string>& road_network_configuration) {
  RoadNetworkConfiguration rn_config{RoadGeometryConfiguration::FromMap(road_network_configuration)};

  auto it = road_network_configuration.find(params::kRuleRegistry);
  if (it != road_network_configuration.end()) {
    rn_config.rule_registry = std::make_optional(it->second);
  }
  it = road_network_configuration.find(params::kRoadRuleBook);
  if (it != road_network_configuration.end()) {
    rn_config.road_rule_book = std::make_optional(it->second);
  }
  it = road_network_configuration.find(params::kTrafficLightBook);
  if (it != road_network_configuration.end()) {
    rn_config.traffic_light_book = std::make_optional(it->second);
  }
  it = road_network_configuration.find(params::kPhaseRingBook);
  if (it != road_network_configuration.end()) {
    rn_config.phase_ring_book = std::make_optional(it->second);
  }
  it = road_network_configuration.find(params::kIntersectionBook);
  if (it != road_network_configuration.end()) {
    rn_config.intersection_book = std::make_optional(it->second);
  }
  return rn_config;
}

std::map<std::string, std::string> RoadNetworkConfiguration::ToStringMap() const {
  auto rg_config = road_geometry_configuration.ToStringMap();
  if (rule_registry.has_value()) {
    rg_config.emplace(params::kRuleRegistry, rule_registry.value());
  }
  if (road_rule_book.has_value()) {
    rg_config.emplace(params::kRoadRuleBook, road_rule_book.value());
  }
  if (traffic_light_book.has_value()) {
    rg_config.emplace(params::kTrafficLightBook, traffic_light_book.value());
  }
  if (phase_ring_book.has_value()) {
    rg_config.emplace(params::kPhaseRingBook, phase_ring_book.value());
  }
  if (intersection_book.has_value()) {
    rg_config.emplace(params::kIntersectionBook, intersection_book.value());
  }
  return rg_config;
}

}  // namespace builder
}  // namespace malidrive
