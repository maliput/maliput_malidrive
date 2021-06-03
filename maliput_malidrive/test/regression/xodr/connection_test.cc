// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/connection.h"

#include <gtest/gtest.h>

#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(ConnectionTest, StrToType) {
  const std::string kExpectedDefault{"default"};
  const std::string kExpectedVirtual{"virtual"};
  EXPECT_EQ(Connection::Type::kDefault, Connection::str_to_type(kExpectedDefault));
  EXPECT_EQ(Connection::Type::kVirtual, Connection::str_to_type(kExpectedVirtual));
  EXPECT_THROW(Connection::str_to_type("WrongValue"), maliput::common::assertion_error);
}

GTEST_TEST(ConnectionTest, TypeToStr) {
  const std::string kExpectedDefault{"default"};
  const std::string kExpectedVirtual{"virtual"};
  EXPECT_EQ(kExpectedDefault, Connection::type_to_str(Connection::Type::kDefault));
  EXPECT_EQ(kExpectedVirtual, Connection::type_to_str(Connection::Type::kVirtual));
}

GTEST_TEST(ConnectionTest, StrToContactPoint) {
  const std::string kExpectedStart{"start"};
  const std::string kExpectedEnd{"end"};
  EXPECT_EQ(Connection::ContactPoint::kStart, Connection::str_to_contact_point(kExpectedStart));
  EXPECT_EQ(Connection::ContactPoint::kEnd, Connection::str_to_contact_point(kExpectedEnd));
  EXPECT_THROW(Connection::str_to_contact_point("WrongValue"), maliput::common::assertion_error);
}

GTEST_TEST(ConnectionTest, ContactPointToStr) {
  const std::string kExpectedStart{"start"};
  const std::string kExpectedEnd{"end"};
  EXPECT_EQ(kExpectedStart, Connection::contact_point_to_str(Connection::ContactPoint::kStart));
  EXPECT_EQ(kExpectedEnd, Connection::contact_point_to_str(Connection::ContactPoint::kEnd));
}

GTEST_TEST(ConnectionTest, EqualityOperator) {
  const Connection::LaneLink lane_link_a{Connection::LaneLink::Id("3") /* from */,
                                         Connection::LaneLink::Id("4") /* to */};
  const Connection::LaneLink lane_link_b{Connection::LaneLink::Id("5") /* from */,
                                         Connection::LaneLink::Id("6") /* to */};

  const Connection kConnection{
      Connection::Id("10") /* id */,
      "1" /* incoming_road */,
      "2" /* connecting_road */,
      Connection::ContactPoint::kStart /* contact_point */,
      Connection::Id("50") /* connection_master */,
      Connection::Type::kDefault /* type */,
      {lane_link_a, lane_link_b} /* lane_links */
  };
  Connection connection = kConnection;

  EXPECT_EQ(kConnection, connection);
  connection.id = Connection::Id("15");
  EXPECT_NE(kConnection, connection);
  connection.id = Connection::Id("10");
  connection.incoming_road = "11";
  EXPECT_NE(kConnection, connection);
  connection.incoming_road = "1";
  connection.connecting_road = "22";
  EXPECT_NE(kConnection, connection);
  connection.connecting_road = "2";
  connection.contact_point = Connection::ContactPoint::kEnd;
  EXPECT_NE(kConnection, connection);
  connection.contact_point = Connection::ContactPoint::kStart;
  connection.connection_master = Connection::Id("500");
  EXPECT_NE(kConnection, connection);
  connection.connection_master = Connection::Id("50");
  connection.type = Connection::Type::kVirtual;
  EXPECT_NE(kConnection, connection);
  connection.type = Connection::Type::kDefault;
  connection.lane_links[0].from = Connection::LaneLink::Id("33");
  EXPECT_NE(kConnection, connection);
  connection.lane_links[0].from = Connection::LaneLink::Id("3");
  EXPECT_EQ(kConnection, connection);
  connection.lane_links[1].to = Connection::LaneLink::Id("66");
  EXPECT_NE(kConnection, connection);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
