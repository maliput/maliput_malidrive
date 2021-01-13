// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_configuration.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace builder {
namespace test {
namespace {

GTEST_TEST(BuildPolicyType, StringToTypeConversion) {
  EXPECT_EQ(BuildPolicy::Type::kSequential, BuildPolicy::FromStrToType("sequential"));
  EXPECT_EQ(BuildPolicy::Type::kParallel, BuildPolicy::FromStrToType("parallel"));
}

GTEST_TEST(SimplificationPolicy, StringToPolicyConversion) {
  EXPECT_EQ(RoadGeometryConfiguration::SimplificationPolicy::kNone,
            RoadGeometryConfiguration::FromStrToSimplificationPolicy("none"));
  EXPECT_EQ(RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel,
            RoadGeometryConfiguration::FromStrToSimplificationPolicy("simplify"));
}

GTEST_TEST(ToleranceSelectionPolicy, StringToPolicyConversion) {
  EXPECT_EQ(RoadGeometryConfiguration::ToleranceSelectionPolicy::kManualSelection,
            RoadGeometryConfiguration::FromStrToToleranceSelectionPolicy("manual"));
  EXPECT_EQ(RoadGeometryConfiguration::ToleranceSelectionPolicy::kAutomaticSelection,
            RoadGeometryConfiguration::FromStrToToleranceSelectionPolicy("automatic"));
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
