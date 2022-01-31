// Copyright 2020 Toyota Research Institute
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/rule_registry.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Functor to build a RoadRulebook.
///
/// At the moment there are two api for the rules in maliput.
/// - The old rules structure composed by the rules:
///   - `SpeedLimitRule`
///   - `RightOfWayRule`
///   - `DirectionUsageRule`
/// - The new rules structure composed by:
///   - Rules depending on the type of value they hold:
///     - `DiscreteValueRule`
///     - `RangeValueRule`
///   - `RuleRegistry:` Used to register all the rule type that are allowed.
///   The latter allows you to customize and add as many rule types as necessary.
///
/// This builder could take different approaches when building the `RoadRulebook` in order to provide support for both
/// generation of rules.
///
/// The presence or not of a provided `RoadRulebook` file path will imply different actions when buildign the
/// `RoadRulebook`.
///  - When no `RoadRulebook` file is provided: Only rules obtained from the XODR are populated.
///  - When `RoadRulebook` is provided but `RuleRegistry` is not: In this case the builder considers that the old rule
///  format is being used. However, in order to populate new rules too, the RightOfWayRules are used to
///                                                      also create DiscreteValueRules of type Right-Of-Way-Rule and
///                                                      Vehicle-In-Stop-Behavior-Rule. See [RoadRulebook loader without
///                                                      rule
///                                                      registry](https://github.com/ToyotaResearchInstitute/maliput/blob/ab4bff490702c31abd2d0d9ff87383f6f00c45c8/maliput/src/base/road_rulebook_loader.cc#L351)
///                                                      provided by maliput.
///  - When both `RoadRulebook` and `RuleRegistry` files are provided: When `RuleRegistry` is provided, the builder
///  expects that the new rules structure is intended to be used.
///                                                                    At this point new rules are populated however the
///                                                                    old RightOfWayRule rules won't be filled as there
///                                                                    is no way to know what's the rule type that is
///                                                                    defined in the RuleRegistry for the right of way
///                                                                    type of rule. See [RoadRulebook loader with rule
///                                                                    registry](https://github.com/ToyotaResearchInstitute/maliput/blob/ab4bff490702c31abd2d0d9ff87383f6f00c45c8/maliput/src/base/road_rulebook_loader_using_rule_registry.cc#L206)
///                                                                    provided by maliput.
///
/// There are rules that are extracted from the XODR file, and they are used to populate the RoadRulbook no matter the
/// combination of `RoadRulebook` and `RuleRegistry` that is choosen. Those are:
///  - Speed limit: Used to fill both `SpeedLimitRule` and `RangeValueRule` of type Speed-Limit-Rule information.
///  - Direction usage: Used to fill both `DirectionUsageRule` and `DiscretValueRule` of type Speed-Limit-Rule
///  information.
///  - Vehicle exclusive: Used to fill `DiscretValueRule` of type Vehicle-Exclusive-Rule information.
///  - Vehicle usage: Used to fill `DiscretValueRule` of type Vehicle-Usage-Rule information.
///
/// TODO(francocipollone): Simplify builder once the old rules are deprecated from maliput.
class RoadRuleBookBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadRuleBookBuilder)
  RoadRuleBookBuilder() = delete;

  /// Constructs a RoadRuleBook.
  ///
  /// @param rg is the pointer to the RoadGeometry. It must not be nullptr.
  /// @param rule_registry is the pointer to the RuleRegistry. It must not be nullptr.
  /// @param road_rulebook_file_path to the yaml file to load the RoadRulebook.
  /// @param direction_usage_rules is a vector of DirectionUsageRules.
  /// @param speed_limit_rules is a vector of SpeedLimitRules.
  /// @param use_loader_with_rule_registry is a boolean to determine which maliput RoadRulebook loader to use.
  /// @throw maliput::assertion_error When `rg` or `rule_registry` are nullptr.
  RoadRuleBookBuilder(const maliput::api::RoadGeometry* rg, const maliput::api::rules::RuleRegistry* rule_registry,
                      const std::optional<std::string>& road_rulebook_file_path,
                      const std::vector<maliput::api::rules::DirectionUsageRule>& direction_usage_rules,
                      const std::vector<maliput::api::rules::SpeedLimitRule>& speed_limit_rules,
                      bool use_loader_with_rule_registry);

  /// Builds a ManualRulebook.
  ///
  /// Both Speed Limits Rules and Direction Usage Rules are added to the
  /// Rulebook after Rulebook yaml file is loaded.
  std::unique_ptr<const maliput::api::rules::RoadRulebook> operator()();

 private:
  // @returns A LaneSRoute that covers `lane.`
  // @throws maliput::common::assertion_error When `lane` is nullptr.
  maliput::api::LaneSRoute CreateLaneSRouteFor(const Lane* lane) const;

  // Creates vehicle usage and vehicle exclusive rules for all Lanes in the
  // RoadGeometry.
  // Rule values are created based on the XODR Lane Type.
  //
  // @param rulebook The pointer to the RoadRulebook based to add the rules.
  //        It must not be nullptr.
  void CreateVehicleRelatedRules(maliput::ManualRulebook* rulebook) const;

  // Creates speed limit rules for all Lanes in the RoadGeometry.
  // Rule values have been previously registered in the
  // maliput::api::rules::RuleRegistry, which was provided at time of
  // construction.
  //
  // Speed limit rules naming convention:
  // Rule Id: "<RuleType> + '/' + <LaneID> + '_' + <index>".
  // Where 'index' is a number that increments in order to differentiate between speed
  // rules limit within a Lane.
  //
  // @param rulebook The RoadRulebook to which to add the rules.
  //        It must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rulebook` is nullptr.
  // @throws maliput::common::assertion_error When
  // maliput::api::rules::RuleRegistry has no type equal to
  // maliput::SpeedLimitRuleTypeId().
  // @throws maliput::common::assertion_error When a Lane in the RoadGeometry
  // has a max speed limit that does not match any range in
  // maliput::SpeedLimitRuleTypeId() rule type.
  void CreateSpeedLimitRules(maliput::ManualRulebook* rulebook) const;

  // Creates direction usage rules for the entire RoadGeometry.
  // Direction Usage Rule Type must have been previously registered in the
  // maliput::api::rules::RuleRegistry, which was provided at time of
  // construction.
  //
  // @param rulebook The RoadRulebook to add the rules.
  //        It must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rulebook` is nullptr.
  void CreateDirectionUsageRules(maliput::ManualRulebook* rulebook) const;

  const maliput::api::RoadGeometry* rg_{};
  const maliput::api::rules::RuleRegistry* rule_registry_{};
  const std::optional<std::string> road_rulebook_file_path_{};
  std::vector<maliput::api::rules::DirectionUsageRule> direction_usage_rules_;
  std::vector<maliput::api::rules::SpeedLimitRule> speed_limit_rules_;
  std::function<std::pair<std::string, std::optional<std::string>>(const Lane*)>
      extract_vehicle_usage_and_vehicle_exclusive_from_lane_;
  bool use_loader_with_rule_registry_{true};
};

}  // namespace builder
}  // namespace malidrive
