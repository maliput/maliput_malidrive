// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/discrete_value_rule_state_provider_builder.h"

#include "maliput/base/phase_based_right_of_way_discrete_value_rule_state_provider.h"

namespace malidrive {
namespace builder {
namespace {

// Sets to `state_provider` the first value in each
// DiscreteValueRule::values() in `discrete_value_rules` as the default
// state.
// @throws maliput::common::assertion_error When `state_provider` is
//         nullptr.
void PopulateDiscreteValueRuleStates(const std::map<Rule::Id, DiscreteValueRule>& discrete_value_rules,
                                     maliput::PhaseBasedRightOfWayDiscreteValueRuleStateProvider* state_provider) {
  MALIDRIVE_THROW_UNLESS(state_provider != nullptr);
  for (const auto& rule_id_rule : discrete_value_rules) {
    state_provider->SetState(rule_id_rule.first, rule_id_rule.second.values().front(), std::nullopt, std::nullopt);
  }
}

}  // namespace

std::unique_ptr<maliput::api::rules::DiscreteValueRuleStateProvider> DiscreteValueRuleStateProviderBuilder::operator()()
    const {
  auto state_provider = std::make_unique<maliput::PhaseBasedRightOfWayDiscreteValueRuleStateProvider>(
      rulebook_, phase_ring_book_, phase_provider_);
  const maliput::api::rules::RoadRulebook::QueryResults all_rules = rulebook_->Rules();
  PopulateDiscreteValueRuleStates(all_rules.discrete_value_rules, state_provider.get());
  return state_provider;
}

}  // namespace builder
}  // namespace malidrive
