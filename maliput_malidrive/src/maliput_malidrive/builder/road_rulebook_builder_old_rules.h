// Copyright 2022 Toyota Research Institute
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
/// The deprecated rule API is composed of.
///   - maliput::api::rules::SpeedLimitRule
///   - maliput::api::rules::RightOfWayRule
///   - maliput::api::rules::DirectionUsageRule
///
///  Rules with the old API are built out of the YAML file and created maliput::api::rules::RightOfWayRules are used to
///  create their analogous version
///    as maliput::api::rules::DiscreteValueRules of type "Right-Of-Way-Rule" and "Vehicle-In-Stop-Behavior-Rule".
///    @see [RoadRulebook loader without rule
///    registry](https://github.com/ToyotaResearchInstitute/maliput/blob/ab4bff490702c31abd2d0d9ff87383f6f00c45c8/maliput/src/base/road_rulebook_loader.cc)
///    provided by maliput.
///
/// The following rules are created regardless the values of `road_rulebook_file_path` and `rule_registry`:
///  - Speed limits: maliput::api::rules::SpeedLimitRule and maliput::api::rules::RangeValueRule whose type
///    is "Speed-Limit-Rule" are created.
///  - Direction usage: maliput::api::rules::DirectionUsageRule and maliput::api::rules::DiscretValueRule whose type
///    is "Speed-Limit-Rule" are created.
///  - Vehicle exclusive: maliput::api::rules::DiscretValueRule whose type is "Vehicle-Exclusive-Rule" are created.
///  - Vehicle usage: maliput::api::rules::DiscretValueRule whose type is "Vehicle-Usage-Rule" are created.
///
/// TODO(ToyotaResearchInstitute/maliput#108): remove RoadRulebook builder functor
class RoadRuleBookBuilderOldRules {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadRuleBookBuilderOldRules)
  RoadRuleBookBuilderOldRules() = delete;

  /// Constructs a RoadRuleBook.
  ///
  /// @param rg is the pointer to the RoadGeometry. It must not be nullptr.
  /// @param rule_registry is the pointer to the RuleRegistry. It must not be nullptr.
  /// @param road_rulebook_file_path to the yaml file to load the RoadRulebook.
  /// @param direction_usage_rules is a vector of DirectionUsageRules.
  /// @param speed_limit_rules is a vector of SpeedLimitRules.
  /// @throw maliput::assertion_error When `rg` or `rule_registry` are nullptr.
  RoadRuleBookBuilderOldRules(const maliput::api::RoadGeometry* rg,
                              const maliput::api::rules::RuleRegistry* rule_registry,
                              const std::optional<std::string>& road_rulebook_file_path,
                              const std::vector<maliput::api::rules::DirectionUsageRule>& direction_usage_rules,
                              const std::vector<maliput::api::rules::SpeedLimitRule>& speed_limit_rules);

  /// Builds a ManualRulebook.
  ///
  /// Both Speed Limits Rules and Direction Usage Rules are added to the
  /// Rulebook after Rulebook yaml file is loaded.
  std::unique_ptr<const maliput::api::rules::RoadRulebook> operator()();

 private:
  const maliput::api::RoadGeometry* rg_{};
  const maliput::api::rules::RuleRegistry* rule_registry_{};
  const std::optional<std::string> road_rulebook_file_path_{};
  std::vector<maliput::api::rules::DirectionUsageRule> direction_usage_rules_;
  std::vector<maliput::api::rules::SpeedLimitRule> speed_limit_rules_;
};

}  // namespace builder
}  // namespace malidrive
