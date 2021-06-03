// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/road_link.h"

#include <gtest/gtest.h>

#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(RoadLinkTest, StrToType) {
  const std::string kExpectedRoad{"road"};
  const std::string kExpectedJunction{"junction"};
  EXPECT_EQ(RoadLink::ElementType::kRoad, RoadLink::str_to_element_type(kExpectedRoad));
  EXPECT_EQ(RoadLink::ElementType::kJunction, RoadLink::str_to_element_type(kExpectedJunction));
  EXPECT_THROW(RoadLink::str_to_element_type("WrongValue"), maliput::common::assertion_error);
}

GTEST_TEST(RoadLinkTest, TypeToStr) {
  const std::string kExpectedRoad{"road"};
  const std::string kExpectedJunction{"junction"};
  EXPECT_EQ(kExpectedRoad, RoadLink::element_type_to_str(RoadLink::ElementType::kRoad));
  EXPECT_EQ(kExpectedJunction, RoadLink::element_type_to_str(RoadLink::ElementType::kJunction));
}

GTEST_TEST(RoadLinkTest, StrToContactPoint) {
  const std::string kExpectedStart{"start"};
  const std::string kExpectedEnd{"end"};
  EXPECT_EQ(RoadLink::ContactPoint::kStart, RoadLink::str_to_contact_point(kExpectedStart));
  EXPECT_EQ(RoadLink::ContactPoint::kEnd, RoadLink::str_to_contact_point(kExpectedEnd));
  EXPECT_THROW(RoadLink::str_to_contact_point("WrongValue"), maliput::common::assertion_error);
}

GTEST_TEST(RoadLinkTest, ContactPointToStr) {
  const std::string kExpectedStart{"start"};
  const std::string kExpectedEnd{"end"};
  EXPECT_EQ(kExpectedStart, RoadLink::contact_point_to_str(RoadLink::ContactPoint::kStart));
  EXPECT_EQ(kExpectedEnd, RoadLink::contact_point_to_str(RoadLink::ContactPoint::kEnd));
}

GTEST_TEST(RoadLinkTest, EqualityOperator) {
  const RoadLink::LinkAttributes kPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                              RoadLink::LinkAttributes::Id{"50"} /* elementId*/,
                                              RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                            RoadLink::LinkAttributes::Id{"80"} /* elementId*/,
                                            RoadLink::ContactPoint::kStart /* contactPoint*/};
  const RoadLink kRoadLink{kPredecessor, kSuccessor};
  RoadLink road_link = kRoadLink;

  EXPECT_EQ(kRoadLink, road_link);
  road_link.predecessor->element_id = RoadLink::LinkAttributes::Id("25");
  EXPECT_NE(kRoadLink, road_link);
  road_link.predecessor->element_id = RoadLink::LinkAttributes::Id("50");
  road_link.successor->element_id = RoadLink::LinkAttributes::Id("25");
  EXPECT_NE(kRoadLink, road_link);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
