// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/junction.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(JunctionTest, StrToType) {
  const std::string kExpectedDefault{"default"};
  const std::string kExpectedVirtual{"virtual"};
  EXPECT_EQ(Junction::Type::kDefault, Junction::str_to_type(kExpectedDefault));
  EXPECT_EQ(Junction::Type::kVirtual, Junction::str_to_type(kExpectedVirtual));
  EXPECT_THROW(Junction::str_to_type("WrongValue"), maliput::common::assertion_error);
}

GTEST_TEST(JunctionTest, TypeToStr) {
  const std::string kExpectedDefault{"default"};
  const std::string kExpectedVirtual{"virtual"};
  EXPECT_EQ(kExpectedDefault, Junction::type_to_str(Junction::Type::kDefault));
  EXPECT_EQ(kExpectedVirtual, Junction::type_to_str(Junction::Type::kVirtual));
}

GTEST_TEST(JunctionTest, EqualityOperator) {
  const Connection kConnectionA{Connection::Id("10") /* id */,
                                "1" /* incoming_road */,
                                "2" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                Connection::Id("50") /* connection_master */,
                                Connection::Type::kDefault /* type */,
                                {{Connection::LaneLink::Id("3"), Connection::LaneLink::Id("4")},
                                 {Connection::LaneLink::Id("5"), Connection::LaneLink::Id("6")}} /* lane_links */};
  const Connection kConnectionB{Connection::Id("11") /* id */,
                                "2" /* incoming_road */,
                                "3" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                Connection::Id("51") /* connection_master */,
                                Connection::Type::kDefault /* type */,
                                {{Connection::LaneLink::Id("4"), Connection::LaneLink::Id("5")},
                                 {Connection::LaneLink::Id("6"), Connection::LaneLink::Id("7")}} /* lane_links */};
  const Junction kJunction{Junction::Id("358") /* id */,
                           "junctionTest" /* name */,
                           Junction::Type::kDefault /* type */,
                           {{kConnectionA.id, kConnectionA}, {kConnectionB.id, kConnectionB}} /* connections */};

  Junction junction = kJunction;

  EXPECT_EQ(kJunction, junction);
  junction.id = Junction::Id("400");
  EXPECT_NE(kJunction, junction);
  junction.id = Junction::Id("358");
  junction.name = std::string("junctionTestB");
  EXPECT_NE(kJunction, junction);
  junction.name = std::string("junctionTest");
  junction.type = Junction::Type::kVirtual;
  EXPECT_NE(kJunction, junction);
  junction.type = Junction::Type::kDefault;
  junction.connections.at(kConnectionA.id).lane_links[0].to = Connection::LaneLink::Id("35");
  EXPECT_NE(kJunction, junction);
  junction.connections.at(kConnectionA.id).lane_links[0].to = Connection::LaneLink::Id("4");
  EXPECT_EQ(kJunction, junction);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
