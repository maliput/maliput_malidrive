// Copyright 2020 Toyota Research Institute
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

  /// Builds a maliput::PhaseBasedRightOfWayDiscreteValueRuleStateProvider.
  std::unique_ptr<maliput::api::rules::DiscreteValueRuleStateProvider> operator()() const;

 private:
  const maliput::api::rules::RoadRulebook* rulebook_{};
  const maliput::api::rules::PhaseRingBook* phase_ring_book_{};
  const maliput::api::rules::PhaseProvider* phase_provider_{};
};

}  // namespace builder
}  // namespace malidrive
