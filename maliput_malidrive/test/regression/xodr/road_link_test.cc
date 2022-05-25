// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
