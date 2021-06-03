// Copyright 2020 Toyota Research Institute
#include <memory>
#include <unordered_map>
#include <vector>

#include <maliput/api/road_geometry.h>
#include <maliput/base/rule_registry.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Functor to build a RuleRegistry.
class RuleRegistryBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleRegistryBuilder)

  /// Constructs a maliput::api::rules::RuleRegistry.
  ///
  /// @param rg A malidrive::RoadGeometry. It is used to extract speed limit
  /// rule ranges. It must not be nullptr.
  RuleRegistryBuilder(const maliput::api::RoadGeometry* rg) : rg_{rg} { MALIDRIVE_THROW_UNLESS(rg_ != nullptr); }

  /// Builds a RuleRegistry.
  ///
  /// Loads the following list of rule types:
  /// - maliput::api::rules::DiscreteValueRule types:
  ///   - @see rules::VehicleExclusiveRuleTypeId().
  ///   - @see rules::VehicleUsageRuleTypeId().
  /// - maliput::api::rules::RangeValueRule types:
  ///   - @see maliput::SpeedLimitRuleTypeId().
  /// TODO(agalbachicar)   Once the YAML loader for RuleRegistry is enabled in
  ///                      maliput, change this method to forward a call to
  ///                      the loader.
  std::unique_ptr<maliput::api::rules::RuleRegistry> operator()();

 private:
  // The following two methods are temporary and should be removed once
  // DiscreteValueRule types are loaded via a YAML loader.
  //@{

  // Adds maliput::api::rules::DiscreteValueRule types to `rule_registry`.
  //
  // @see VehicleRelatedRuleTypesAndValues() and @see maliput::BuildDirectionUsageRuleType()  for
  // maliput::api::rules::DiscreteValueRule types and discrete values.
  // All maliput::api::rules::DiscreteValueRule::DiscreteValues are marked with
  // maliput::api::rules::Rule::State::kStrict severity.
  // Direction usage rule type are included to the rule registry.
  //
  // @throws maliput::common::assertion_error When `rule_registry` is nullptr.
  void AddDiscreteValueRuleTypes(maliput::api::rules::RuleRegistry* rule_registry) const;

  // @returns An unordered map of DiscreteValueRule::TypeId and possible rule states.
  std::unordered_map<maliput::api::rules::DiscreteValueRule::TypeId, std::vector<std::string>> RuleTypesAndValues()
      const;

  //@}

  // Constructs a unique maliput::api::rules::RangeValueRule type to
  // describe all possible speed limit RangeValeRule::Ranges and adds them to
  // `rule_registry`.
  //
  // Traverses all lanes in the RoadGeometry and queries for the lane speed
  // limit at the beginning of the lane. Note that multiple speed limits
  // definitions could be found within an XODR lane, however,
  // this function is limited to sample only to the beginning of the Lane.(TODO(#553))
  //
  // All built ranges will have a minimum speed constants::kDefaultMaxSpeedLimit.
  // When no maximum speed is set, constants::kDefaultMaxSpeedLimit is used.
  //
  // @throws maliput::common::assertion_error When `rule_registry` is nullptr.
  void AddSpeedLimitRuleType(maliput::api::rules::RuleRegistry* rule_registry) const;

  const maliput::api::RoadGeometry* rg_{};
};

}  // namespace builder
}  // namespace malidrive
