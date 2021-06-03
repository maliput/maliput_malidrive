// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/unit.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(UnitFeatures, UnitToStrMethod) {
  const std::string kExpectedStr{"km/h"};
  EXPECT_EQ(kExpectedStr, unit_to_str(Unit::kKph));
}

GTEST_TEST(UnitFeatures, StrToUnitMethod) {
  const Unit kExpectedUnit{Unit::kMph};
  const std::string kUnitStr{"mph"};
  const std::string kWrongUnitStr{"m/h"};
  EXPECT_EQ(kExpectedUnit, str_to_unit(kUnitStr));
  EXPECT_THROW(str_to_unit(kWrongUnitStr), maliput::common::assertion_error);
}

GTEST_TEST(UnitFeatures, ConvertToMs) {
  EXPECT_EQ(15., ConvertToMs(15., Unit::kMs));
  EXPECT_EQ(8.5, ConvertToMs(30.6, Unit::kKph));
  EXPECT_EQ(20.1168, ConvertToMs(45., Unit::kMph));
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
