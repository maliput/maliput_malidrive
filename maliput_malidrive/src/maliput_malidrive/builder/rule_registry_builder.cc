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
#include "maliput_malidrive/builder/rule_registry_builder.h"

#include <maliput/base/rule_registry_loader.h>

#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/constants.h"

namespace malidrive {
namespace builder {

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

RuleRegistryBuilder::RuleRegistryBuilder(const maliput::api::RoadGeometry* rg,
                                         const std::optional<std::string>& rule_registry_file_path)
    : rg_{rg}, rule_registry_file_path_{rule_registry_file_path} {
  MALIDRIVE_THROW_UNLESS(rg_ != nullptr);
}

std::unique_ptr<maliput::api::rules::RuleRegistry> RuleRegistryBuilder::operator()() {
  maliput::log()->trace("{}", rule_registry_file_path_.has_value()
                                  ? "RuleRegistry file provided: " + rule_registry_file_path_.value()
                                  : "No RuleRegistry file provided");
  auto rule_registry = !rule_registry_file_path_.has_value()
                           ? std::make_unique<maliput::api::rules::RuleRegistry>()
                           : maliput::LoadRuleRegistryFromFile(rule_registry_file_path_.value());
  AddDiscreteValueRuleTypes(rule_registry.get());
  AddSpeedLimitRuleType(rule_registry.get());

  return rule_registry;
}

void RuleRegistryBuilder::AddDiscreteValueRuleTypes(maliput::api::rules::RuleRegistry* rule_registry) const {
  MALIDRIVE_THROW_UNLESS(rule_registry != nullptr);

  const Rule::RelatedRules empty_related_rules{};
  const Rule::RelatedUniqueIds empty_related_unique_ids{};
  const int severity{Rule::State::kStrict};

  for (const auto& rule_type_values : RuleTypesAndValues()) {
    std::vector<DiscreteValueRule::DiscreteValue> discrete_values;
    for (const std::string& discrete_value : rule_type_values.second) {
      discrete_values.push_back(
          DiscreteValueRule::DiscreteValue{severity, empty_related_rules, empty_related_unique_ids, discrete_value});
    }
    rule_registry->RegisterDiscreteValueRule(rule_type_values.first, discrete_values);
  }
  const auto& direction_usage_rule_type = maliput::BuildDirectionUsageRuleType();
  rule_registry->RegisterDiscreteValueRule(direction_usage_rule_type.first, direction_usage_rule_type.second);
}

std::unordered_map<DiscreteValueRule::TypeId, std::vector<std::string>> RuleRegistryBuilder::RuleTypesAndValues()
    const {
  return {
      // Defines whether or not vehicles and pedestrians can travel along a route.
      {rules::VehicleUsageRuleTypeId(),
       {
           "NonVehicles",     // Vehicles are not allowed to travel.
           "NonPedestrians",  // Pedestrian are not allowed to travel. When this value is set, it is
                              // usually paired with `VehicleExclusiveRuleType` to better specify which type
                              // of vehicle is allowed.
           "Unrestricted",    // No restriction for vehicles nor pedestrians to travel simultaneously.
                              // When this value is set, it is usually paired with `VehicleExclusiveRuleType`
                              // to better specify which type of vehicle is allowed.
       }},
      // Defines which type of vehicle can exclusively travel along a route.
      {rules::VehicleExclusiveRuleTypeId(),
       {
           "BusOnly",                   // Exclusive route for buses.
           "EmergencyVehiclesOnly",     // Exclusive route for emergency vehicles.
           "HighOccupancyVehicleOnly",  // Exclusive route for high occupancy vehicles, such as vehicles with at
                                        // least one passenger.
           "MotorizedVehicleOnly",      // Exclusive route for motorized vehicles, such as cars and motorbikes.
           "NonMotorizedVehicleOnly",   // Exclusive route for non motorized vehicles, such as bikes.
       }},
  };
}

void RuleRegistryBuilder::AddSpeedLimitRuleType(maliput::api::rules::RuleRegistry* rule_registry) const {
  MALIDRIVE_THROW_UNLESS(rule_registry != nullptr);

  const Rule::RelatedRules empty_related_rules{};
  const Rule::RelatedUniqueIds empty_related_unique_ids{};
  std::set<RangeValueRule::Range> speed_limit_range_set;
  // Finds the maximum speed limits in the RoadGeometry. Then registers with
  // the RuleRegistry a maliput::SpeedLimitRuleTypeId() RangeValueRule
  // containing the default minimum speed limit and the discovered maximum
  // speed limit.
  for (const auto lane_id_lane : rg_->ById().GetLanes()) {
    const auto max_speed_limits = GetMaxSpeedLimitFor(dynamic_cast<const Lane*>(lane_id_lane.second));
    for (const auto& speed_limit : max_speed_limits) {
      speed_limit_range_set.insert(RangeValueRule::Range{Rule::State::kStrict, empty_related_rules,
                                                         empty_related_unique_ids, "m/s",
                                                         constants::kDefaultMinSpeedLimit, speed_limit.max});
    }
  }
  rule_registry->RegisterRangeValueRule(
      maliput::SpeedLimitRuleTypeId(),
      std::vector<RangeValueRule::Range>(speed_limit_range_set.begin(), speed_limit_range_set.end()));
}

}  // namespace builder
}  // namespace malidrive
