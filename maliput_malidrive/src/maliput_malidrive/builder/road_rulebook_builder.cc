// Copyright 2020 Toyota Research Institute
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

RoadRuleBookBuilder::RoadRuleBookBuilder(
    const maliput::api::RoadGeometry* rg, const maliput::api::rules::RuleRegistry* rule_registry,
    const std::optional<std::string>& road_rulebook_file_path,
    const std::vector<maliput::api::rules::DirectionUsageRule>& direction_usage_rules,
    const std::vector<maliput::api::rules::SpeedLimitRule>& speed_limit_rules)
    : rg_(rg),
      rule_registry_(rule_registry),
      road_rulebook_file_path_(road_rulebook_file_path),
      direction_usage_rules_(direction_usage_rules),
      speed_limit_rules_(speed_limit_rules) {
  MALIDRIVE_THROW_UNLESS(rg_ != nullptr);
  MALIDRIVE_THROW_UNLESS(rule_registry_ != nullptr);
}

std::unique_ptr<const maliput::api::rules::RoadRulebook> RoadRuleBookBuilder::operator()() {
  auto rulebook = !road_rulebook_file_path_.has_value()
                      ? std::make_unique<maliput::ManualRulebook>()
                      : maliput::LoadRoadRulebookFromFile(rg_, road_rulebook_file_path_.value());

  maliput::ManualRulebook* rulebook_ptr = dynamic_cast<maliput::ManualRulebook*>(rulebook.get());
  MALIDRIVE_THROW_UNLESS(rulebook_ptr != nullptr);

  // Creates vehicle usage and vehicle exclusive rules for the entire RoadGeometry.
  CreateVehicleRelatedRules(rulebook_ptr);

  // Creates speed limit rules for the entire RoadGeometry.
  CreateSpeedLimitRules(rulebook_ptr);

  // Creates direction usage rules for the entire RoadGeometry.
  CreateDirectionUsageRules(rulebook_ptr);

  // TODO(agalbachicar)   Remove when maliput::api::rules::SpeedLimitRules
  //                      are deprecated.
  // Add speed limit rules.
  for (const auto& speed_limit_rule : speed_limit_rules_) {
    rulebook_ptr->AddRule(speed_limit_rule);
  }

  // Add direction usage rules.
  for (const auto& direction_usage_rule : direction_usage_rules_) {
    rulebook_ptr->AddRule(direction_usage_rule);
  }

  return rulebook;
}

LaneSRoute RoadRuleBookBuilder::CreateLaneSRouteFor(const Lane* lane) const {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  return LaneSRoute({LaneSRange(lane->id(), SRange(0., lane->length()))});
}

void RoadRuleBookBuilder::CreateVehicleRelatedRules(maliput::ManualRulebook* rulebook) const {
  MALIDRIVE_DEMAND(rulebook != nullptr);
  const int severity{Rule::State::kStrict};

  for (const auto& lane_id_lane : rg_->ById().GetLanes()) {
    const Lane* lane = dynamic_cast<const Lane*>(lane_id_lane.second);
    MALIDRIVE_THROW_UNLESS(lane != nullptr);

    const std::pair<std::string, std::optional<std::string>> vehicle_rule_values =
        VehicleUsageAndExclusiveRuleStateValues(lane);

    const LaneSRoute lane_s_route = CreateLaneSRouteFor(lane);
    Rule::RelatedRules related_rules;
    if (vehicle_rule_values.second.has_value()) {
      const Rule::Id rule_id(GetRuleIdFrom(rules::VehicleExclusiveRuleTypeId(), lane->id()));
      rulebook->AddRule(rule_registry_->BuildDiscreteValueRule(
          rule_id, rules::VehicleExclusiveRuleTypeId(), lane_s_route,
          {DiscreteValueRule::DiscreteValue{
              severity, {} /* related rules */, {} /* related_unique_ids */, vehicle_rule_values.second.value()}}));

      MALIDRIVE_VALIDATE(
          related_rules.emplace(rules::VehicleExclusiveRuleTypeId().string(), std::vector<Rule::Id>{rule_id}).second,
          maliput::common::assertion_error,
          "Failed to fill related rules for vehicle exclusive rule at lane : " + lane_id_lane.first.string());
    }

    rulebook->AddRule(rule_registry_->BuildDiscreteValueRule(
        GetRuleIdFrom(rules::VehicleUsageRuleTypeId(), lane->id()), rules::VehicleUsageRuleTypeId(), lane_s_route,
        {DiscreteValueRule::DiscreteValue{
            severity, related_rules, {} /* related_unique_ids */, vehicle_rule_values.first}}));
  }
}

void RoadRuleBookBuilder::CreateSpeedLimitRules(maliput::ManualRulebook* rulebook) const {
  const std::optional<maliput::api::rules::RuleRegistry::QueryResult> speed_limit_query_result =
      rule_registry_->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
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

  for (const auto& lane_id_lane : rg_->ById().GetLanes()) {
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
          rule_registry_->BuildRangeValueRule(rule_id, maliput::SpeedLimitRuleTypeId(), lane_s_route, {*range}));
    }
  }
}

void RoadRuleBookBuilder::CreateDirectionUsageRules(maliput::ManualRulebook* rulebook) const {
  MALIDRIVE_THROW_UNLESS(rulebook != nullptr);
  const int severity{Rule::State::kStrict};
  for (const auto& lane_id_lane : rg_->ById().GetLanes()) {
    const Rule::RelatedRules empty_related_rules;
    const Rule::RelatedUniqueIds empty_related_unique_ids;
    const Lane* lane = dynamic_cast<const Lane*>(lane_id_lane.second);
    rulebook->AddRule(rule_registry_->BuildDiscreteValueRule(
        GetRuleIdFrom(maliput::DirectionUsageRuleTypeId(), lane->id()), maliput::DirectionUsageRuleTypeId(),
        CreateLaneSRouteFor(lane),
        {DiscreteValueRule::DiscreteValue{severity, empty_related_rules, empty_related_unique_ids,
                                          GetDirectionUsageRuleStateType(lane)}}));
  }
}

}  // namespace builder
}  // namespace malidrive
