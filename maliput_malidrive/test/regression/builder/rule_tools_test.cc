// Copyright 2019 Toyota Research Institute
#include "maliput_malidrive/builder/rule_tools.h"

#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/base/manual_rulebook.h>

namespace malidrive {
namespace builder {
namespace rules {
namespace test {
namespace {

// Evaluates that all types are registered with the expected string.
GTEST_TEST(RuleTypeIds, Construction) {
  EXPECT_EQ(VehicleExclusiveRuleTypeId().string(), "Vehicle Exclusive Rule Type");
  EXPECT_EQ(VehicleUsageRuleTypeId().string(), "Vehicle Usage Rule Type");
}

// Evaluates that keys for related rules are registered with the expected string.
GTEST_TEST(RelatedRulesKeysStruct, VehicleMotorization) {
  EXPECT_EQ(RelatedRulesKeys::kVehicleMotorization, "Vehicle Motorization");
}

}  // namespace
}  // namespace test
}  // namespace rules
}  // namespace builder
}  // namespace malidrive
