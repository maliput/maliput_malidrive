// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/range_value_rule_state_provider_builder.h"

#include <maliput/base/manual_range_value_rule_state_provider.h>

namespace malidrive {
namespace builder {
namespace {

// Sets to `state_provider` the first range in each
// RangeValueRule::ranges() in `range_value_rules` as the default state.
// @throws maliput::common::assertion_error When `state_provider` is
//         nullptr.
void PopulateRangeValueRuleStates(const std::map<Rule::Id, RangeValueRule>& range_value_rules,
                                  maliput::ManualRangeValueRuleStateProvider* state_provider) {
  MALIDRIVE_THROW_UNLESS(state_provider != nullptr);
  for (const auto& rule_id_rule : range_value_rules) {
    state_provider->SetState(rule_id_rule.first, rule_id_rule.second.ranges().front(), std::nullopt, std::nullopt);
  }
}

}  // namespace

std::unique_ptr<maliput::api::rules::RangeValueRuleStateProvider> RangeValueRuleStateProviderBuilder::operator()()
    const {
  auto state_provider = std::make_unique<maliput::ManualRangeValueRuleStateProvider>(rulebook_);
  const maliput::api::rules::RoadRulebook::QueryResults all_rules = rulebook_->Rules();
  PopulateRangeValueRuleStates(all_rules.range_value_rules, state_provider.get());
  return state_provider;
}

}  // namespace builder
}  // namespace malidrive
