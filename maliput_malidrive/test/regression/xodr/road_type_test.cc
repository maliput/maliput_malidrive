// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/road_type.h"

#include <gtest/gtest.h>

#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(RoadTypeSpeed, EqualityOperator) {
  const RoadType::Speed kSpeed{45. /* max */, Unit::kMph /* unit */};
  RoadType::Speed speed = kSpeed;

  EXPECT_EQ(kSpeed, speed);
  speed.max = 150.;
  EXPECT_NE(kSpeed, speed);
  speed.max = 45.;
  speed.unit = Unit::kMs;
  EXPECT_NE(kSpeed, speed);
  speed.unit = Unit::kMph;
  EXPECT_EQ(kSpeed, speed);
};

GTEST_TEST(RoadType, TypeToStrMethod) {
  const std::string kExpectedStr{"townLocal"};
  EXPECT_EQ(kExpectedStr, RoadType::type_to_str(RoadType::Type::kTownLocal));
}

GTEST_TEST(RoadType, StrToTypeMethod) {
  const RoadType::Type kExpectedType{RoadType::Type::kRural};
  const std::string kTypeStr{"rural"};
  const std::string kWrongTypeStr{"badType"};
  EXPECT_EQ(kExpectedType, RoadType::str_to_type(kTypeStr));
  EXPECT_THROW(RoadType::str_to_type(kWrongTypeStr), maliput::common::assertion_error);
}

GTEST_TEST(RoadType, EqualityOperator) {
  const RoadType::Speed kSpeed{45. /* max */, Unit::kMph /* unit */};
  const RoadType kRoadType{1.2 /* s0 */, RoadType::Type::kPedestrian /* type */, std::nullopt /* country */,
                           kSpeed /* speed */};
  RoadType road_type = kRoadType;
  EXPECT_EQ(kRoadType, road_type);
  road_type.s_0 = 0.5;
  EXPECT_NE(kRoadType, road_type);
  road_type.s_0 = 1.2;
  road_type.type = RoadType::Type::kTownExpressway;
  EXPECT_NE(kRoadType, road_type);
  road_type.type = RoadType::Type::kPedestrian;
  road_type.country = "64";
  EXPECT_NE(kRoadType, road_type);
  road_type.country = std::nullopt;
  road_type.speed.max = std::nullopt;
  EXPECT_NE(kRoadType, road_type);
  road_type.speed.max = 45.;
  EXPECT_EQ(kRoadType, road_type);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
