// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
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
