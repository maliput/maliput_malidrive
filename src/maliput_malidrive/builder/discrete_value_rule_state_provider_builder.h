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
#include <memory>

#include <maliput/api/rules/discrete_value_rule_state_provider.h>
#include <maliput/api/rules/phase_provider.h>
#include <maliput/api/rules/phase_ring_book.h>
#include <maliput/api/rules/road_rulebook.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::Rule;

/// Functor to build a DiscreteValueRuleStateProvider.
class DiscreteValueRuleStateProviderBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValueRuleStateProviderBuilder)

  /// Constructs a DiscreteValueRuleStateProviderBuilder.
  ///
  /// @param rulebook A RoadRulebook to feed the DiscreteValueRuleStateProvider.
  ///        It must not be nullptr.
  /// @param phase_ring_book A PhaseRingBook to feed the
  ///        DiscreteValueRuleStateProvider. It must not be nullptr.
  /// @param phase_provider A PhaseProvider to feed the
  ///        DiscreteValueRuleStateProvider. It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When `rulebook`,
  ///         `phase_ring_book` or `phase_provider` are nullptr.
  explicit DiscreteValueRuleStateProviderBuilder(const maliput::api::rules::RoadRulebook* rulebook,
                                                 const maliput::api::rules::PhaseRingBook* phase_ring_book,
                                                 const maliput::api::rules::PhaseProvider* phase_provider)
      : rulebook_(rulebook), phase_ring_book_(phase_ring_book), phase_provider_(phase_provider) {
    MALIDRIVE_DEMAND(rulebook_ != nullptr);
    MALIDRIVE_DEMAND(phase_ring_book_ != nullptr);
    MALIDRIVE_DEMAND(phase_provider_ != nullptr);
  }

  /// Builds a maliput::PhasedDiscreteRuleStateProvider.
  std::unique_ptr<maliput::api::rules::DiscreteValueRuleStateProvider> operator()() const;

 private:
  const maliput::api::rules::RoadRulebook* rulebook_{};
  const maliput::api::rules::PhaseRingBook* phase_ring_book_{};
  const maliput::api::rules::PhaseProvider* phase_provider_{};
};

}  // namespace builder
}  // namespace malidrive
