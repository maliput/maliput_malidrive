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
#include "maliput_malidrive/xodr/road_header.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(RoadHeader, HandTrafficRuleToStr) {
  const std::string kExpected1{"RHT"};
  const std::string kExpected2{"LHT"};
  EXPECT_EQ(kExpected1, RoadHeader::hand_traffic_rule_to_str(RoadHeader::HandTrafficRule::kRHT));
  EXPECT_EQ(kExpected2, RoadHeader::hand_traffic_rule_to_str(RoadHeader::HandTrafficRule::kLHT));
}

GTEST_TEST(RoadHeader, StrToHandTrafficRule) {
  const std::string kRHT{"RHT"};
  const std::string kLHT{"LHT"};
  EXPECT_EQ(RoadHeader::HandTrafficRule::kRHT, RoadHeader::str_to_hand_traffic_rule(kRHT));
  EXPECT_EQ(RoadHeader::HandTrafficRule::kLHT, RoadHeader::str_to_hand_traffic_rule(kLHT));
  const std::string kWrongValue{"WrongValue"};
  EXPECT_THROW(RoadHeader::str_to_hand_traffic_rule(kWrongValue), maliput::common::assertion_error);
}

GTEST_TEST(RoadHeader, EqualityOperator) {
  const ReferenceGeometry kReferenceGeometry{{{{1.23 /* s_0 */,
                                                {523.2 /* x */, 83.27 /* y */},
                                                0.77 /* orientation */,
                                                100. /* length */,
                                                Geometry::Type::kLine /* Type */,
                                                {Geometry::Line{}} /* description */}}}};
  const LaneLink kLaneLinkA{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* successor */};
  const LaneLink kLaneLinkB{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* successor */};
  const LaneOffset kLaneOffset{2.1 /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  const LaneSection kLaneSection{
      2.1 /* s_0 */,
      true /* single_side */,
      {{Lane::Id("1"), Lane::Type::kDriving, false, kLaneLinkA,
        std::vector<LaneWidth>{{1.1, 2.2, 3.3, 4.4, 5.5}}}} /* left_lanes */,
      {Lane::Id("0"), Lane::Type::kShoulder, false, {}, {}} /* center_lane */,
      {{Lane::Id("-1"), Lane::Type::kRestricted, false, kLaneLinkB,
        std::vector<LaneWidth>{{5.5, 6.6, 7.7, 8.8, 9.9}}}} /* right_lanes */
  };
  const RoadLink kRoadLink{RoadLink::LinkAttributes{RoadLink::ElementType::kRoad /* elementType */,
                                                    RoadLink::LinkAttributes::Id("50") /* elementId*/,
                                                    RoadLink::ContactPoint::kEnd /* contactPoint*/},
                           RoadLink::LinkAttributes{RoadLink::ElementType::kRoad /* elementType */,
                                                    RoadLink::LinkAttributes::Id("80") /* elementId*/,
                                                    RoadLink::ContactPoint::kStart /* contactPoint*/}};
  const RoadType::Speed kSpeed{45. /* max */, Unit::kMph /* unit */};
  const RoadType kRoadType{1.2 /* s0 */, RoadType::Type::kPedestrian /* type */, std::nullopt /* country */,
                           kSpeed /* speed */};
  const Lanes kLanes{{{kLaneOffset}}, {{kLaneSection}}};
  const RoadHeader kRoadHeader{"TestRoadHeader" /* name */,
                               10.65 /* length */,
                               RoadHeader::Id("Road 15") /* id */,
                               "-1" /* junction */,
                               RoadHeader::HandTrafficRule::kRHT /* rule */,
                               kRoadLink /* road_link */,
                               {kRoadType} /* road_type */,
                               kReferenceGeometry /* reference_geometry */,
                               kLanes /* lanes */};
  RoadHeader road_header = kRoadHeader;

  EXPECT_EQ(kRoadHeader, road_header);
  road_header.name = "DifferentName";
  EXPECT_NE(kRoadHeader, road_header);
  road_header.name = "TestRoadHeader";
  road_header.length = 3453.2;
  EXPECT_NE(kRoadHeader, road_header);
  road_header.length = 10.65;
  road_header.id = RoadHeader::Id("Road 30");
  EXPECT_NE(kRoadHeader, road_header);
  road_header.id = RoadHeader::Id("Road 15");
  road_header.junction = "4";
  EXPECT_NE(kRoadHeader, road_header);
  road_header.junction = "-1";
  road_header.rule = RoadHeader::HandTrafficRule::kLHT;
  EXPECT_NE(kRoadHeader, road_header);
  road_header.rule = RoadHeader::HandTrafficRule::kRHT;
  road_header.road_link.successor->element_id = RoadLink::LinkAttributes::Id("12");
  EXPECT_NE(kRoadHeader, road_header);
  road_header.road_link.successor->element_id = RoadLink::LinkAttributes::Id("80");
  road_header.road_types[0].s_0 = 45.;
  EXPECT_NE(kRoadHeader, road_header);
  road_header.road_types[0].s_0 = 1.2;
  road_header.reference_geometry.plan_view.geometries[0].s_0 = 56;
  EXPECT_NE(kRoadHeader, road_header);
  road_header.reference_geometry.plan_view.geometries[0].s_0 = 1.23;
  road_header.lanes.lanes_section[0].left_lanes[0].width_description[0].s_0 = 99.23;
  EXPECT_NE(kRoadHeader, road_header);
  road_header.lanes.lanes_section[0].left_lanes[0].width_description[0].s_0 = 1.1;
  EXPECT_EQ(kRoadHeader, road_header);
}

struct RoadHeaderTest : public ::testing::Test {
  const ReferenceGeometry kReferenceGeometry{
      {{{1.23, {523.2, 83.27}, 0.77, 50., Geometry::Type::kLine, {Geometry::Line{}} /* description */},
        {51.23, {523.2, 83.27}, 0.77, 50., Geometry::Type::kLine, {Geometry::Line{}} /* description */}}}};
  const LaneOffset kLaneOffset{2.1, 2.2, 3.3, 4.4, 5.5};
  const std::vector<LaneSection> kLaneSections{
      {10.0,
       true,
       {{Lane::Id("1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{1.1, 2.2, 3.3, 4.4, 5.5}}}},
       {Lane::Id("0"), Lane::Type::kShoulder, false, {}, {}},
       {{Lane::Id("-1"), Lane::Type::kRestricted, false, {}, std::vector<LaneWidth>{{5.5, 6.6, 7.7, 8.8, 9.9}}}}},
      {30.,
       true,
       {{Lane::Id("1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{1.1, 2.2, 3.3, 4.4, 5.5}}}},
       {Lane::Id("0"), Lane::Type::kShoulder, false, {}, {}},
       {{Lane::Id("-1"), Lane::Type::kRestricted, false, {}, std::vector<LaneWidth>{{5.5, 6.6, 7.7, 8.8, 9.9}}}}},
      {70.,
       true,
       {{Lane::Id("1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{1.1, 2.2, 3.3, 4.4, 5.5}}}},
       {Lane::Id("0"), Lane::Type::kShoulder, false, {}, {}},
       {{Lane::Id("-1"), Lane::Type::kRestricted, false, {}, std::vector<LaneWidth>{{5.5, 6.6, 7.7, 8.8, 9.9}}}}}};
  const Lanes kLanes{{{kLaneOffset}}, {{kLaneSections}}};
  const RoadType kRoadType0{10. /* s0 */,
                            RoadType::Type::kTown /* type */,
                            std::nullopt /* country */,
                            {45. /* max */, Unit::kMph /* unit */} /* speed */};
  const RoadType kRoadType1{53. /* s0 */,
                            RoadType::Type::kRural /* type */,
                            "maliput" /* country */,
                            {80. /* max */, Unit::kKph /* unit */} /* speed */};
  const RoadType kRoadType2{74. /* s0 */,
                            RoadType::Type::kRural /* type */,
                            "maliput" /* country */,
                            {120. /* max */, Unit::kKph /* unit */} /* speed */};
  const RoadHeader kRoadHeader{"TestRoadHeader",
                               100. /* length */,
                               RoadHeader::Id("Road 1") /* id */,
                               "-1" /* junction */,
                               RoadHeader::HandTrafficRule::kRHT /* rule */,
                               {} /* road_link */,
                               {kRoadType0, kRoadType1, kRoadType2} /* road_type */,
                               kReferenceGeometry /* reference_geometry */,
                               kLanes /* lanes */};
};

TEST_F(RoadHeaderTest, GetLaneSectionLength) {
  const double kExpectedLength0 = kLaneSections[1].s_0 - kLaneSections[0].s_0;
  EXPECT_EQ(kExpectedLength0, kRoadHeader.GetLaneSectionLength(0));
  const double kExpectedLength1 = kLaneSections[2].s_0 - kLaneSections[1].s_0;
  EXPECT_EQ(kExpectedLength1, kRoadHeader.GetLaneSectionLength(1));
  const double kExpectedLength2 = kRoadHeader.length - kExpectedLength0 - kExpectedLength1;
  EXPECT_EQ(kExpectedLength2, kRoadHeader.GetLaneSectionLength(2));
}

TEST_F(RoadHeaderTest, GetLaneSectionIndex) {
  EXPECT_THROW(kRoadHeader.GetLaneSectionIndex(9.99999), maliput::common::assertion_error);
  EXPECT_EQ(0, kRoadHeader.GetLaneSectionIndex(10.));
  EXPECT_EQ(0, kRoadHeader.GetLaneSectionIndex(29.99999));
  EXPECT_EQ(1, kRoadHeader.GetLaneSectionIndex(30.));
  EXPECT_EQ(1, kRoadHeader.GetLaneSectionIndex(69.99999));
  EXPECT_EQ(2, kRoadHeader.GetLaneSectionIndex(70.));
  EXPECT_EQ(2, kRoadHeader.GetLaneSectionIndex(109.99999));
  EXPECT_THROW(kRoadHeader.GetLaneSectionIndex(110.), maliput::common::assertion_error);
}

TEST_F(RoadHeaderTest, GetRoadTypeLength) {
  const double kExpectedLength0 = kRoadType1.s_0 - kRoadType0.s_0;
  EXPECT_EQ(kExpectedLength0, kRoadHeader.GetRoadTypeLength(0));
  const double kExpectedLength1 = kRoadType2.s_0 - kRoadType1.s_0;
  EXPECT_EQ(kExpectedLength1, kRoadHeader.GetRoadTypeLength(1));
  const double kExpectedLength2 = kRoadHeader.length - kExpectedLength0 - kExpectedLength1;
  EXPECT_EQ(kExpectedLength2, kRoadHeader.GetRoadTypeLength(2));
}

TEST_F(RoadHeaderTest, GetRoadType) {
  EXPECT_THROW(kRoadHeader.GetRoadType(9.99999), maliput::common::assertion_error);
  EXPECT_EQ(kRoadType0, *kRoadHeader.GetRoadType(10.));
  EXPECT_EQ(kRoadType0, *kRoadHeader.GetRoadType(52.99999));
  EXPECT_EQ(kRoadType1, *kRoadHeader.GetRoadType(53.));
  EXPECT_EQ(kRoadType1, *kRoadHeader.GetRoadType(73.99999));
  EXPECT_EQ(kRoadType2, *kRoadHeader.GetRoadType(74.));
  EXPECT_EQ(kRoadType2, *kRoadHeader.GetRoadType(109.99999));
  EXPECT_THROW(kRoadHeader.GetRoadType(110.), maliput::common::assertion_error);
}

TEST_F(RoadHeaderTest, GetRoadTypesInRangeThrows) {
  EXPECT_THROW(kRoadHeader.GetRoadTypesInRange(-1., 5.), maliput::common::assertion_error);
  EXPECT_THROW(kRoadHeader.GetRoadTypesInRange(5., 3.), maliput::common::assertion_error);
}

TEST_F(RoadHeaderTest, GetRoadTypesInRange) {
  {
    const auto road_types = kRoadHeader.GetRoadTypesInRange(10., 53.);
    EXPECT_EQ(1., road_types.size());
    EXPECT_EQ(kRoadType0, *road_types[0]);
  }
  {
    const auto road_types = kRoadHeader.GetRoadTypesInRange(53., 74.);
    EXPECT_EQ(1., road_types.size());
    EXPECT_EQ(kRoadType1, *road_types[0]);
  }
  {
    const auto road_types = kRoadHeader.GetRoadTypesInRange(74., 100.);
    EXPECT_EQ(1., road_types.size());
    EXPECT_EQ(kRoadType2, *road_types[0]);
  }
  {
    const auto road_types = kRoadHeader.GetRoadTypesInRange(15., 74.);
    EXPECT_EQ(2., road_types.size());
    EXPECT_EQ(kRoadType0, *road_types[0]);
    EXPECT_EQ(kRoadType1, *road_types[1]);
  }
  {
    const auto road_types = kRoadHeader.GetRoadTypesInRange(53., 99.);
    EXPECT_EQ(2., road_types.size());
    EXPECT_EQ(kRoadType1, *road_types[0]);
    EXPECT_EQ(kRoadType2, *road_types[1]);
  }
  {
    const auto road_types = kRoadHeader.GetRoadTypesInRange(20., 85.);
    EXPECT_EQ(3., road_types.size());
    EXPECT_EQ(kRoadType0, *road_types[0]);
    EXPECT_EQ(kRoadType1, *road_types[1]);
    EXPECT_EQ(kRoadType2, *road_types[2]);
  }
}

TEST_F(RoadHeaderTest, S0AndS1) {
  const double kExpectedS0{kReferenceGeometry.plan_view.geometries[0].s_0};
  const double kExpectedS1{kReferenceGeometry.plan_view.geometries[1].s_0 +
                           kReferenceGeometry.plan_view.geometries[1].length};
  EXPECT_EQ(kExpectedS0, kRoadHeader.s0());
  EXPECT_EQ(kExpectedS1, kRoadHeader.s1());
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
