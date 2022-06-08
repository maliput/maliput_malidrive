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
#include "maliput_malidrive/builder/discrete_value_rule_state_provider_builder.h"

#include <maliput/base/phased_discrete_rule_state_provider.h>

namespace malidrive {
namespace builder {
namespace {

// Sets to `state_provider` the first value in each
// DiscreteValueRule::states() in `discrete_value_rules` as the default
// state.
// @throws maliput::common::assertion_error When `state_provider` is
//         nullptr.
void PopulateDiscreteValueRuleStates(const std::map<Rule::Id, DiscreteValueRule>& discrete_value_rules,
                                     maliput::PhasedDiscreteRuleStateProvider* state_provider) {
  MALIDRIVE_THROW_UNLESS(state_provider != nullptr);
  for (const auto& rule_id_rule : discrete_value_rules) {
    state_provider->SetState(rule_id_rule.first, rule_id_rule.second.states().front(), std::nullopt, std::nullopt);
  }
}

}  // namespace

std::unique_ptr<maliput::api::rules::DiscreteValueRuleStateProvider> DiscreteValueRuleStateProviderBuilder::operator()()
    const {
  auto state_provider =
      std::make_unique<maliput::PhasedDiscreteRuleStateProvider>(rulebook_, phase_ring_book_, phase_provider_);
  const maliput::api::rules::RoadRulebook::QueryResults all_rules = rulebook_->Rules();
  PopulateDiscreteValueRuleStates(all_rules.discrete_value_rules, state_provider.get());
  return state_provider;
}

}  // namespace builder
}  // namespace malidrive
