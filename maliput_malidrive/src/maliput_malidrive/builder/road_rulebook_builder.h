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
  /// @throw maliput::assertion_error When `rg` or `rule_registry` are nullptr.
  RoadRuleBookBuilder(const maliput::api::RoadGeometry* rg, const maliput::api::rules::RuleRegistry* rule_registry,
                      const std::optional<std::string>& road_rulebook_file_path,
                      const std::vector<maliput::api::rules::DirectionUsageRule>& direction_usage_rules,
                      const std::vector<maliput::api::rules::SpeedLimitRule>& speed_limit_rules);

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
};

}  // namespace builder
}  // namespace malidrive
