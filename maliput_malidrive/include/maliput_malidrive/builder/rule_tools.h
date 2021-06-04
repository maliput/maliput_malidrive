// Copyright 2019 Toyota Research Institute
#pragma once

#include <functional>

#include <maliput/api/rules/discrete_value_rule.h>
#include <maliput/api/rules/rule.h>

namespace malidrive {

namespace builder {
namespace rules {

/// @returns a maliput::api::rules::Rule::TypeId initialized with
/// "Vehicle Exclusive Rule Type".
maliput::api::rules::Rule::TypeId VehicleExclusiveRuleTypeId();

/// @returns a maliput::api::rules::Rule::TypeId initialized with
/// "Vehicle Usage Rule Type".
maliput::api::rules::Rule::TypeId VehicleUsageRuleTypeId();

/// Defines keys used in api::rules::Rule::RelatedRules on Malidrive.
struct RelatedRulesKeys {
  static constexpr const char* kVehicleMotorization = "Vehicle Motorization";
};

/// Holds speed information obtained from the XODR.
struct XodrSpeedProperties {
  /// Max speed in meters per second.
  double max{};
  /// Start position (s-coordinate) relative to the XODR Track frame.
  double s_start{};
  /// End position (s-coordinate) relative to the XODR Track frame.
  double s_end{};
};

}  // namespace rules
}  // namespace builder
}  // namespace malidrive
