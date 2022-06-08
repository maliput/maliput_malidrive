// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/builder/road_rulebook_builder.h"

#include <maliput/api/regions.h>
#include <maliput/base/road_rulebook_loader.h>

#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/id_providers.h"

namespace malidrive {
namespace builder {

using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::SRange;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

RoadRuleBookBuilder::RoadRuleBookBuilder(const maliput::api::RoadGeometry* rg,
                                         const maliput::api::rules::RuleRegistry* rule_registry,
                                         const std::optional<std::string>& road_rulebook_file_path)
    : rg_(rg), rule_registry_(rule_registry), road_rulebook_file_path_(road_rulebook_file_path) {
  MALIDRIVE_THROW_UNLESS(rg_ != nullptr);
  MALIDRIVE_THROW_UNLESS(rule_registry_ != nullptr);
}

std::unique_ptr<const maliput::api::rules::RoadRulebook> RoadRuleBookBuilder::operator()() {
  maliput::log()->trace("{}", road_rulebook_file_path_.has_value()
                                  ? "RoadRulebook file provided: " + road_rulebook_file_path_.value()
                                  : "No RoadRulebook file provided");
  ;

  auto rulebook = road_rulebook_file_path_.has_value()
                      ? maliput::LoadRoadRulebookFromFile(rg_, road_rulebook_file_path_.value(), *rule_registry_)
                      : std::make_unique<maliput::ManualRulebook>();

  maliput::ManualRulebook* rulebook_ptr = dynamic_cast<maliput::ManualRulebook*>(rulebook.get());
  MALIDRIVE_THROW_UNLESS(rulebook_ptr != nullptr);

  AddsXODRBasedRulesToRulebook(rg_, rule_registry_, rulebook_ptr);

  return rulebook;
}

LaneSRoute RoadRuleBookBuilder::CreateLaneSRouteFor(const Lane* lane) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  return LaneSRoute({LaneSRange(lane->id(), SRange(0., lane->length()))});
}

void RoadRuleBookBuilder::AddsXODRBasedRulesToRulebook(const maliput::api::RoadGeometry* rg,
                                                       const maliput::api::rules::RuleRegistry* rule_registry,
                                                       maliput::ManualRulebook* rulebook) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  MALIDRIVE_THROW_UNLESS(rule_registry != nullptr);
  MALIDRIVE_THROW_UNLESS(rulebook != nullptr);

  // Discrete Value Rules
  // @{
  // Creates vehicle usage and vehicle exclusive rules for the entire RoadGeometry.
  AddsVehicleExclusiveAndUsageRulesToRulebook(rg, rule_registry, rulebook);
  // Creates direction usage rules for the entire RoadGeometry.
  AddsDirectionUsageRulesToRulebook(rg, rule_registry, rulebook);
  // @}

  // Range Value Rules
  // @{
  // Creates speed limit rules for the entire RoadGeometry.
  AddsSpeedLimitRulesToRulebook(rg, rule_registry, rulebook);
  // @}
}

void RoadRuleBookBuilder::AddsVehicleExclusiveAndUsageRulesToRulebook(
    const maliput::api::RoadGeometry* rg, const maliput::api::rules::RuleRegistry* rule_registry,
    maliput::ManualRulebook* rulebook) {
  MALIDRIVE_DEMAND(rg != nullptr);
  MALIDRIVE_DEMAND(rule_registry != nullptr);
  MALIDRIVE_DEMAND(rulebook != nullptr);
  const int severity{Rule::State::kStrict};

  for (const auto& lane_id_lane : rg->ById().GetLanes()) {
    const Lane* lane = dynamic_cast<const Lane*>(lane_id_lane.second);
    MALIDRIVE_THROW_UNLESS(lane != nullptr);

    const std::pair<std::string, std::optional<std::string>> vehicle_rule_values =
        VehicleUsageAndExclusiveRuleStateValues(lane);

    const LaneSRoute lane_s_route = CreateLaneSRouteFor(lane);
    Rule::RelatedRules related_rules;
    if (vehicle_rule_values.second.has_value()) {
      const Rule::Id rule_id(GetRuleIdFrom(rules::VehicleExclusiveRuleTypeId(), lane->id()));
      rulebook->AddRule(rule_registry->BuildDiscreteValueRule(
          rule_id, rules::VehicleExclusiveRuleTypeId(), lane_s_route,
          {DiscreteValueRule::DiscreteValue{
              severity, {} /* related rules */, {} /* related_unique_ids */, vehicle_rule_values.second.value()}}));

      MALIDRIVE_VALIDATE(
          related_rules.emplace(rules::VehicleExclusiveRuleTypeId().string(), std::vector<Rule::Id>{rule_id}).second,
          maliput::common::assertion_error,
          "Failed to fill related rules for vehicle exclusive rule at lane : " + lane_id_lane.first.string());
    }

    rulebook->AddRule(rule_registry->BuildDiscreteValueRule(
        GetRuleIdFrom(rules::VehicleUsageRuleTypeId(), lane->id()), rules::VehicleUsageRuleTypeId(), lane_s_route,
        {DiscreteValueRule::DiscreteValue{
            severity, related_rules, {} /* related_unique_ids */, vehicle_rule_values.first}}));
  }
}

void RoadRuleBookBuilder::AddsSpeedLimitRulesToRulebook(const maliput::api::RoadGeometry* rg,
                                                        const maliput::api::rules::RuleRegistry* rule_registry,
                                                        maliput::ManualRulebook* rulebook) {
  MALIDRIVE_DEMAND(rg != nullptr);
  MALIDRIVE_DEMAND(rule_registry != nullptr);
  MALIDRIVE_DEMAND(rulebook != nullptr);
  const std::optional<maliput::api::rules::RuleRegistry::QueryResult> speed_limit_query_result =
      rule_registry->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
  MALIDRIVE_THROW_UNLESS(speed_limit_query_result.has_value());
  const auto ranges_ptr = std::get_if<std::vector<RangeValueRule::Range>>(&speed_limit_query_result->rule_values);
  MALIDRIVE_THROW_UNLESS(ranges_ptr != nullptr);

  // Returns an optional bearing a RangeValueRule::Range whose maximum is
  // `max_speed_limit` when found. Otherwise, std::nullopt is returned.
  auto range_from_max_speed_limit = [ranges = *(ranges_ptr)](double max_speed_limit) {
    std::optional<RangeValueRule::Range> result;
    const auto range_it =
        std::find_if(ranges.begin(), ranges.end(),
                     [max_speed_limit](const RangeValueRule::Range& range) { return range.max == max_speed_limit; });
    if (range_it != ranges.end()) {
      result = *range_it;
    }
    return result;
  };

  for (const auto& lane_id_lane : rg->ById().GetLanes()) {
    const Lane* lane = dynamic_cast<const Lane*>(lane_id_lane.second);
    MALIDRIVE_THROW_UNLESS(lane != nullptr);

    const auto max_speed_limits = GetMaxSpeedLimitFor(lane);
    UniqueIntegerProvider speed_limit_indexer(0);
    for (const auto& speed_limit : max_speed_limits) {
      const Rule::Id rule_id = GetRuleIdFrom(maliput::SpeedLimitRuleTypeId(), lane->id(), speed_limit_indexer.new_id());
      const std::optional<RangeValueRule::Range> range = range_from_max_speed_limit(speed_limit.max);
      const LaneSRoute lane_s_route({LaneSRange(
          lane->id(), SRange(lane->LaneSFromTrackS(speed_limit.s_start), lane->LaneSFromTrackS(speed_limit.s_end)))});
      MALIDRIVE_VALIDATE(range.has_value(), maliput::common::assertion_error,
                         "Failed to obtain speed limit rule range for lane : " + lane_id_lane.first.string());
      rulebook->AddRule(
          rule_registry->BuildRangeValueRule(rule_id, maliput::SpeedLimitRuleTypeId(), lane_s_route, {*range}));
    }
  }
}

void RoadRuleBookBuilder::AddsDirectionUsageRulesToRulebook(const maliput::api::RoadGeometry* rg,
                                                            const maliput::api::rules::RuleRegistry* rule_registry,
                                                            maliput::ManualRulebook* rulebook) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  MALIDRIVE_THROW_UNLESS(rule_registry != nullptr);
  MALIDRIVE_THROW_UNLESS(rulebook != nullptr);
  const int severity{Rule::State::kStrict};
  for (const auto& lane_id_lane : rg->ById().GetLanes()) {
    const Rule::RelatedRules empty_related_rules;
    const Rule::RelatedUniqueIds empty_related_unique_ids;
    const Lane* lane = dynamic_cast<const Lane*>(lane_id_lane.second);
    rulebook->AddRule(rule_registry->BuildDiscreteValueRule(
        GetRuleIdFrom(maliput::DirectionUsageRuleTypeId(), lane->id()), maliput::DirectionUsageRuleTypeId(),
        CreateLaneSRouteFor(lane),
        {DiscreteValueRule::DiscreteValue{severity, empty_related_rules, empty_related_unique_ids,
                                          GetDirectionUsageRuleStateType(lane)}}));
  }
}

}  // namespace builder
}  // namespace malidrive
