// Copyright 2022 Toyota Research Institute
#include "maliput_malidrive/builder/road_rulebook_builder_old_rules.h"

#include <maliput/api/regions.h>
#include <maliput/base/road_rulebook_loader.h>

#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/builder/road_rulebook_builder.h"

namespace malidrive {
namespace builder {

using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::SRange;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

RoadRuleBookBuilderOldRules::RoadRuleBookBuilderOldRules(
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

std::unique_ptr<const maliput::api::rules::RoadRulebook> RoadRuleBookBuilderOldRules::operator()() {
  // TODO(francocipollone): Removes the load method that doesn't use the rule registry.
  maliput::log()->trace("{}", road_rulebook_file_path_.has_value()
                                  ? "RoadRulebook file provided: " + road_rulebook_file_path_.value()
                                  : "No RoadRulebook file provided");

  auto rulebook = road_rulebook_file_path_.has_value()
                      ? maliput::LoadRoadRulebookFromFile(rg_, road_rulebook_file_path_.value())
                      : std::make_unique<maliput::ManualRulebook>();

  maliput::ManualRulebook* rulebook_ptr = dynamic_cast<maliput::ManualRulebook*>(rulebook.get());
  MALIDRIVE_THROW_UNLESS(rulebook_ptr != nullptr);

  // Adds rules based on RuleRegistry.
  RoadRuleBookBuilder::AddsXODRBasedRulesToRulebook(rg_, rule_registry_, rulebook_ptr);

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

}  // namespace builder
}  // namespace malidrive
