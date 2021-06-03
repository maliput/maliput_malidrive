// Copyright 2020 Toyota Research Institute

#include <memory>

#include <maliput/api/rules/range_value_rule_state_provider.h>
#include <maliput/api/rules/road_rulebook.h>
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

/// Functor to build a RangeValueRuleStateProvider.
class RangeValueRuleStateProviderBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RangeValueRuleStateProviderBuilder)

  /// Constructs a MalidriveRangeValueRuleStateProviderBuilder.
  ///
  /// @param rulebook A RoadRulebook to feed the RangeValueRuleStateProvider.
  ///        It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When `rulebook` is nullptr.
  explicit RangeValueRuleStateProviderBuilder(const maliput::api::rules::RoadRulebook* rulebook) : rulebook_(rulebook) {
    MALIDRIVE_DEMAND(rulebook_ != nullptr);
  }

  /// Builds a RangeValueRuleStateProvider.
  std::unique_ptr<maliput::api::rules::RangeValueRuleStateProvider> operator()() const;

 private:
  const maliput::api::rules::RoadRulebook* rulebook_{};
};

}  // namespace builder
}  // namespace malidrive
