// Copyright 2019 Toyota Research Institute
#include "maliput_malidrive/builder/rule_tools.h"

#include <string>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {
namespace rules {

maliput::api::rules::Rule::TypeId VehicleExclusiveRuleTypeId() {
  return maliput::api::rules::Rule::TypeId{"Vehicle Exclusive Rule Type"};
}

maliput::api::rules::Rule::TypeId VehicleUsageRuleTypeId() {
  return maliput::api::rules::Rule::TypeId{"Vehicle Usage Rule Type"};
}

}  // namespace rules
}  // namespace builder
}  // namespace malidrive
