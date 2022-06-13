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
#include "maliput_malidrive/xodr/db_manager.h"

#include <array>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/test_utilities/xodr_testing_map_descriptions.h"
#include "utility/resources.h"

namespace malidrive {
namespace xodr {
namespace test {
namespace {

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

// Template of a XODR description that contains only the xodr header.
constexpr const char* kXODRHeaderTemplate = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='{}' revMinor='{}' name='{}' version='{}' date='{}'
    north='{}' south='{}' east='{}' west='{}' vendor='{}' >
  </header>
  <road name="Road 0" length="1.4005927435591335e+2" id="0" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="1.3" length="2.">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s='0.'>
        <left>
          <lane id='1' type='driving' level= '0'>
            <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
          </lane>
        </left>
        <center>
          <lane id='0' type='driving' level= '0'>
          </lane>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
)R";

// Holds the tolerance used to check contiguity when parsing.
constexpr std::optional<double> kStrictParserSTolerance{malidrive::constants::kStrictLinearTolerance};
// Flag to not allow schema errors.
constexpr bool kDontAllowSchemaErrors{false};
// Flag to not allow semantic errors.
constexpr bool kDontAllowSemanticErrors{false};

// Tests the loading of a XODR description from a file.
GTEST_TEST(DBManager, LoadFromFile) {
  const std::string kXodrFile = "SingleLane.xodr";
  EXPECT_NO_THROW(LoadDataBaseFromFile(utility::FindResourceInPath(kXodrFile, kMalidriveResourceFolder),
                                       {kStrictParserSTolerance}));
}

// Tests the loading of a XODR description from a string.
GTEST_TEST(DBManagerTest, LoadFromString) {
  const std::string xodr_description =
      fmt::format(kXODRHeaderTemplate, 1. /* revMajor */, 1. /* revMinor */, "TestHeader" /* name */,
                  1.21 /* version */, "Wed Sep 19 12:00:00 2018" /* date */, 0. /* north */, 0. /* south */,
                  0. /* east */, 0. /* west */, "TestVendor" /* vendor */);
  EXPECT_NO_THROW(LoadDataBaseFromStr(xodr_description,
                                      {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}));
}

// Tests `DBManager::GetXodrHeader` method.
GTEST_TEST(DBManagerTest, GetXodrHeader) {
  const Header kExpectedHeader{1. /* revMajor */,
                               1. /* revMinor */,
                               "TestHeader" /* name */,
                               1.21 /* version */,
                               "Wed Sep 19 12:00:00 2018" /* date */,
                               1.1 /* north */,
                               2.2 /* south */,
                               3.3 /* east */,
                               4.4 /* west */,
                               "TestVendor" /* vendor */};
  const std::string xodr_description =
      fmt::format(kXODRHeaderTemplate, kExpectedHeader.rev_major, kExpectedHeader.rev_minor,
                  kExpectedHeader.name.value(), kExpectedHeader.version.value(), kExpectedHeader.date.value(),
                  kExpectedHeader.north.value(), kExpectedHeader.south.value(), kExpectedHeader.east.value(),
                  kExpectedHeader.west.value(), kExpectedHeader.vendor.value());
  const std::unique_ptr<DBManager> dut = LoadDataBaseFromStr(
      xodr_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  Header header = dut->GetXodrHeader();
  EXPECT_EQ(kExpectedHeader, header);
}

// Template of a XODR description in which road link values can be changed.
const std::string kXODRRoadHeaderLinksVariableTemplate = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='TestHeader' version='1.21' date='Wed Sep 19 12:00:00 2018'
    north='0.' south='0.' east='0.' west='0.' vendor='TestVendor' >
  </header>
  <road name='TestRoadHeader1' length='10.' id='15' junction='{}' rule='RHT'>"
    <link>
        {}
    </link>
    <planView>
      <geometry s='0.0' x='500.0' y='80.0' hdg='0.' length='10.'>
          <line/>
      </geometry>
    </planView>
    <lanes>
      <laneOffset s='0.5' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <laneSection s='0.0'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                      {}
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
              </lane>
          </left>
          <center>
              <lane id='0' type='driving' level= '0'>
              </lane>
          </center>
          <right>
              <lane id='-1' type='driving' level= '0'>
                  <link>
                      {}
                  </link>
                  <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
              </lane>
          </right>
      </laneSection>
    </lanes>
  </road>
  <road name='TestRoadHeader2' length='10.' id='16' junction='{}' rule='RHT'>"
    <link>
        {}
    </link>
    <planView>
      <geometry s='0.0' x='510.0' y='80.0' hdg='0.' length='10.'>
          <line/>
      </geometry>
    </planView>
    <lanes>
      <laneOffset s='0.5' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <laneSection s='0.0'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                      {}
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
              </lane>
          </left>
          <center>
              <lane id='0' type='driving' level= '0'>
              </lane>
          </center>
          <right>
              <lane id='-1' type='driving' level= '0'>
                  <link>
                      {}
                  </link>
                  <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
              </lane>
          </right>
      </laneSection>
    </lanes>
  </road>
  {}
</OpenDRIVE>
)R";

class DBManagerLinksTests : public ::testing::Test {
 protected:
  const std::string kRoadLinkRoadTemplate = "<{} elementType='{}' elementId='{}' contactPoint='{}'/>";
  const std::string kRoadLinkJunctionTemplate = "<{} elementType='{}' elementId='{}'/>";
  const std::string kRoadLaneLinkTemplate = "<{} id='{}'/>";
  const std::string kJunctionLaneLinkTemplate = "<laneLink from='{}' to='{}'/>";
  const std::string kConnectionTemplate = R"R(
    <connection id='{}' incomingRoad='{}' connectingRoad='{}' contactPoint='{}'>
      {}
      {}
    </connection>
)R";
  const std::string kJunctionTemplate = R"R(
  <junction id='{}' name=''>
    {}
  </junction>
)R";
  const std::string road_non_junction_id = "-1";
  const std::string road_junction_id = "160";
};

TEST_F(DBManagerLinksTests, ConsistentsLinks) {
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                                RoadLink::LinkAttributes::Id("16") /* elementId*/,
                                                RoadLink::ContactPoint::kStart /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("1")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};
  // Road 1.
  const std::string road_link_string_a = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kSuccessorTag, RoadLink::element_type_to_str(kRoadSuccessor.element_type),
      kRoadSuccessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadSuccessor.contact_point));
  const std::string lane_link_string_a_left =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kSuccessorTag, kLaneLinkLeft.id.string());
  const std::string lane_link_string_a_right =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kSuccessorTag, kLaneLinkRight.id.string());
  // Road 2.
  const std::string road_link_string_b = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kPredecessorTag, RoadLink::element_type_to_str(kRoadPredecessor.element_type),
      kRoadPredecessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadPredecessor.contact_point));
  const std::string lane_link_string_b_left =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kLaneLinkLeft.id.string());
  const std::string lane_link_string_b_right =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kLaneLinkRight.id.string());

  const std::string xodr_description =
      fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id, road_link_string_a,
                  lane_link_string_a_left, lane_link_string_a_right, road_non_junction_id, road_link_string_b,
                  lane_link_string_b_left, lane_link_string_b_right, "");
  EXPECT_NO_THROW(LoadDataBaseFromStr(xodr_description,
                                      {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}));
}

TEST_F(DBManagerLinksTests, EmptyLinks) {
  // No connections.
  const std::string xodr_description = fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id, "", "",
                                                   "", road_non_junction_id, "", "", "", "");
  EXPECT_NO_THROW(LoadDataBaseFromStr(xodr_description,
                                      {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}));
}

TEST_F(DBManagerLinksTests, UnknownPredecessorRoadLink) {
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("22") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const std::string road_link_string_b = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kPredecessorTag, RoadLink::element_type_to_str(kRoadPredecessor.element_type),
      kRoadPredecessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadPredecessor.contact_point));

  const std::string xodr_description = fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id, "", "",
                                                   "", road_non_junction_id, road_link_string_b, "", "", "");
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

TEST_F(DBManagerLinksTests, UnknownSuccessorRoadLink) {
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                                RoadLink::LinkAttributes::Id("22") /* elementId*/,
                                                RoadLink::ContactPoint::kStart /* contactPoint*/};
  const std::string road_link_string_a = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kSuccessorTag, RoadLink::element_type_to_str(kRoadSuccessor.element_type),
      kRoadSuccessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadSuccessor.contact_point));

  const std::string xodr_description = fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id,
                                                   road_link_string_a, "", "", road_non_junction_id, "", "", "", "");
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

TEST_F(DBManagerLinksTests, UnknownPredecessorLaneLink) {
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                                RoadLink::LinkAttributes::Id("16") /* elementId*/,
                                                RoadLink::ContactPoint::kStart /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("1")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};
  const LaneLink::LinkAttributes kWrongLaneLinkLeft{LaneLink::LinkAttributes::Id("45")};
  // Road 1.
  const std::string road_link_string_a = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kSuccessorTag, RoadLink::element_type_to_str(kRoadSuccessor.element_type),
      kRoadSuccessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadSuccessor.contact_point));
  const std::string lane_link_string_a_left =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kSuccessorTag, kLaneLinkLeft.id.string());
  const std::string lane_link_string_a_right =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kSuccessorTag, kLaneLinkRight.id.string());
  // Road 2.
  const std::string road_link_string_b = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kPredecessorTag, RoadLink::element_type_to_str(kRoadPredecessor.element_type),
      kRoadPredecessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadPredecessor.contact_point));
  const std::string lane_link_string_b_left =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kWrongLaneLinkLeft.id.string());
  const std::string lane_link_string_b_right =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kLaneLinkRight.id.string());

  const std::string xodr_description =
      fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id, road_link_string_a,
                  lane_link_string_a_left, lane_link_string_a_right, road_non_junction_id, road_link_string_b,
                  lane_link_string_b_left, lane_link_string_b_right, "");
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

TEST_F(DBManagerLinksTests, UnknownSuccessorLaneLink) {
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                                RoadLink::LinkAttributes::Id("16") /* elementId*/,
                                                RoadLink::ContactPoint::kStart /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("1")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};
  const LaneLink::LinkAttributes kWrongLaneLinkRight{LaneLink::LinkAttributes::Id("-45")};
  // Road 1.
  const std::string road_link_string_a = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kSuccessorTag, RoadLink::element_type_to_str(kRoadSuccessor.element_type),
      kRoadSuccessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadSuccessor.contact_point));
  const std::string lane_link_string_a_left =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kSuccessorTag, kLaneLinkLeft.id.string());
  const std::string lane_link_string_a_right =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kSuccessorTag, kLaneLinkRight.id.string());
  // Road 2.
  const std::string road_link_string_b = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kPredecessorTag, RoadLink::element_type_to_str(kRoadPredecessor.element_type),
      kRoadPredecessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadPredecessor.contact_point));
  const std::string lane_link_string_b_left =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kLaneLinkLeft.id.string());
  const std::string lane_link_string_b_right =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kWrongLaneLinkRight.id.string());

  const std::string xodr_description =
      fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id, road_link_string_a,
                  lane_link_string_a_left, lane_link_string_a_right, road_non_junction_id, road_link_string_b,
                  lane_link_string_b_left, lane_link_string_b_right, "");
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

// Tests Junctions by varying the connection map values.
class DBManagerJunctionLinksTests : public DBManagerLinksTests {
 protected:
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kJunction /* elementType */,
                                                RoadLink::LinkAttributes::Id("160") /* elementId*/,
                                                std::nullopt /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("1")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};
  // Road 1.
  const std::string road_link_string_a =
      fmt::format(kRoadLinkJunctionTemplate, RoadLink::kSuccessorTag,
                  RoadLink::element_type_to_str(kRoadSuccessor.element_type), kRoadSuccessor.element_id.string(), "");
  const std::string lane_link_string_a_left = "";
  const std::string lane_link_string_a_right = "";
  // Road 2.
  const std::string road_link_string_b = fmt::format(
      kRoadLinkRoadTemplate, RoadLink::kPredecessorTag, RoadLink::element_type_to_str(kRoadPredecessor.element_type),
      kRoadPredecessor.element_id.string(), RoadLink::contact_point_to_str(*kRoadPredecessor.contact_point));
  const std::string lane_link_string_b_left =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kLaneLinkLeft.id.string());
  const std::string lane_link_string_b_right =
      fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, kLaneLinkRight.id.string());

  std::string BuildXMLDescriptionFromJunction(const Junction& junction) {
    // JunctionLaneLink
    const std::string junction_lane_link_string_a =
        static_cast<int>(junction.connections.begin()->second.lane_links.size()) == 0
            ? std::string("")
            : fmt::format(kJunctionLaneLinkTemplate, junction.connections.begin()->second.lane_links[0].from.string(),
                          junction.connections.begin()->second.lane_links[0].to.string());
    const std::string junction_lane_link_string_b =
        static_cast<int>(junction.connections.begin()->second.lane_links.size()) == 0
            ? std::string("")
            : fmt::format(kJunctionLaneLinkTemplate, junction.connections.begin()->second.lane_links[1].from.string(),
                          junction.connections.begin()->second.lane_links[1].to.string());
    // Connection.
    const std::string connection_string = fmt::format(
        kConnectionTemplate, junction.connections.begin()->second.id.string(),
        junction.connections.begin()->second.incoming_road, junction.connections.begin()->second.connecting_road,
        Connection::contact_point_to_str(junction.connections.begin()->second.contact_point),
        junction_lane_link_string_a, junction_lane_link_string_b);
    // Junction.
    const std::string junction_string = fmt::format(kJunctionTemplate, junction.id.string(), connection_string);
    return fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id, road_link_string_a,
                       lane_link_string_a_left, lane_link_string_a_right, road_junction_id, road_link_string_b,
                       lane_link_string_b_left, lane_link_string_b_right, junction_string);
  }
};

TEST_F(DBManagerJunctionLinksTests, JunctionConsistentsLinks) {
  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "15" /* incoming_road */,
                                "16" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}} /* lane_links */};
  const Junction kJunction{Junction::Id(road_junction_id) /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};

  const std::string xml_description{BuildXMLDescriptionFromJunction(kJunction)};
  EXPECT_NO_THROW(LoadDataBaseFromStr(xml_description,
                                      {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}));
}

TEST_F(DBManagerJunctionLinksTests, JunctionUnknownIncomingRoad) {
  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "155" /* incoming_road */,
                                "16" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}} /* lane_links */};
  const Junction kJunction{Junction::Id(road_junction_id) /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};
  const std::string xml_description{BuildXMLDescriptionFromJunction(kJunction)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

TEST_F(DBManagerJunctionLinksTests, JunctionUnknownConnectingRoad) {
  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "15" /* incoming_road */,
                                "198" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}} /* lane_links */};
  const Junction kJunction{Junction::Id(road_junction_id) /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};
  const std::string xml_description{BuildXMLDescriptionFromJunction(kJunction)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

TEST_F(DBManagerJunctionLinksTests, JunctionWrongLaneLinkA) {
  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "15" /* incoming_road */,
                                "16" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-2"), Connection::LaneLink::Id("-1")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}} /* lane_links */};
  const Junction kJunction{Junction::Id(road_junction_id) /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};

  const std::string xml_description{BuildXMLDescriptionFromJunction(kJunction)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

TEST_F(DBManagerJunctionLinksTests, JunctionWrongLaneLinkB) {
  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "15" /* incoming_road */,
                                "16" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("15")}} /* lane_links */};
  const Junction kJunction{Junction::Id(road_junction_id) /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};

  const std::string xml_description{BuildXMLDescriptionFromJunction(kJunction)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

TEST_F(DBManagerJunctionLinksTests, WrongJunctionId) {
  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "15" /* incoming_road */,
                                "16" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}} /* lane_links */};
  const Junction kJunction{Junction::Id("33") /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};

  const std::string xml_description{BuildXMLDescriptionFromJunction(kJunction)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

// Tests auto-completing LaneLinks when incoming lanes and connecting lanes have identical ids.
TEST_F(DBManagerJunctionLinksTests, ConnectionWithoutLaneLinks) {
  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "15" /* incoming_road */,
                                "16" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {} /* lane_links */};
  const Junction kJunction{Junction::Id(road_junction_id) /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};
  const std::vector<Connection::LaneLink> kExpectedLaneLinks{
      {Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
      {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}};

  const std::string xml_description{BuildXMLDescriptionFromJunction(kJunction)};
  const auto dut =
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  const auto lane_links = dut->GetJunctions().at(kJunction.id).connections.at(kConnectionA.id).lane_links;
  EXPECT_EQ(kExpectedLaneLinks, lane_links);
}

// Tests Junctions by varying the road links.
class DBManagerJunctionLinksTestsB : public DBManagerLinksTests {
 protected:
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kJunction /* elementType */,
                                                RoadLink::LinkAttributes::Id("160") /* elementId*/,
                                                std::nullopt /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("1")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};

  const Connection kConnectionA{Connection::Id("1") /* id */,
                                "15" /* incoming_road */,
                                "16" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}} /* lane_links */};
  const Junction kJunction{Junction::Id("33") /* id */,
                           "junctionTest" /* name */,
                           std::nullopt /* type */,
                           {{kConnectionA.id, kConnectionA}} /* connections */};

  // JunctionLaneLink
  const std::string junction_lane_link_string_a = fmt::format(
      kJunctionLaneLinkTemplate, kConnectionA.lane_links[0].from.string(), kConnectionA.lane_links[0].to.string());
  const std::string junction_lane_link_string_b = fmt::format(
      kJunctionLaneLinkTemplate, kConnectionA.lane_links[1].from.string(), kConnectionA.lane_links[1].to.string());
  // Connection.
  const std::string connection_string =
      fmt::format(kConnectionTemplate, kConnectionA.id.string(), kConnectionA.incoming_road,
                  kConnectionA.connecting_road, Connection::contact_point_to_str(kConnectionA.contact_point),
                  junction_lane_link_string_a, junction_lane_link_string_b);
  // Junction.
  const std::string junction_string = fmt::format(kJunctionTemplate, kJunction.id.string(), connection_string);

  std::string BuildXMLDescriptionFromRoadLinks(const RoadLink::LinkAttributes& predecessor,
                                               const RoadLink::LinkAttributes& successor,
                                               const LaneLink::LinkAttributes& left,
                                               const LaneLink::LinkAttributes& right) {
    // Road 1.
    const std::string road_link_string_a =
        fmt::format(kRoadLinkJunctionTemplate, RoadLink::kSuccessorTag,
                    RoadLink::element_type_to_str(successor.element_type), successor.element_id.string(), "");
    const std::string lane_link_string_a_left = "";
    const std::string lane_link_string_a_right = "";
    // Road 2.
    const std::string road_link_string_b = fmt::format(
        kRoadLinkRoadTemplate, RoadLink::kPredecessorTag, RoadLink::element_type_to_str(predecessor.element_type),
        predecessor.element_id.string(), RoadLink::contact_point_to_str(*predecessor.contact_point));
    const std::string lane_link_string_b_left =
        fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, left.id.string());
    const std::string lane_link_string_b_right =
        fmt::format(kRoadLaneLinkTemplate, LaneLink::kPredecessorTag, right.id.string());
    return fmt::format(kXODRRoadHeaderLinksVariableTemplate, road_non_junction_id, road_link_string_a,
                       lane_link_string_a_left, lane_link_string_a_right, road_junction_id, road_link_string_b,
                       lane_link_string_b_left, lane_link_string_b_right, junction_string);
  }
};

TEST_F(DBManagerJunctionLinksTestsB, UnknownPredecessor) {
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("188") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kJunction /* elementType */,
                                                RoadLink::LinkAttributes::Id("160") /* elementId*/,
                                                std::nullopt /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("1")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};

  const std::string xml_description{
      BuildXMLDescriptionFromRoadLinks(kRoadPredecessor, kRoadSuccessor, kLaneLinkLeft, kLaneLinkRight)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

TEST_F(DBManagerJunctionLinksTestsB, UnknownSuccessor) {
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kJunction /* elementType */,
                                                RoadLink::LinkAttributes::Id("161") /* elementId*/,
                                                std::nullopt /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("1")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};

  const std::string xml_description{
      BuildXMLDescriptionFromRoadLinks(kRoadPredecessor, kRoadSuccessor, kLaneLinkLeft, kLaneLinkRight)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

TEST_F(DBManagerJunctionLinksTestsB, UnknownLaneLink) {
  const RoadLink::LinkAttributes kRoadPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                                  RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                                  RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kRoadSuccessor{RoadLink::ElementType::kJunction /* elementType */,
                                                RoadLink::LinkAttributes::Id("160") /* elementId*/,
                                                std::nullopt /* contactPoint*/};
  const LaneLink::LinkAttributes kLaneLinkLeft{LaneLink::LinkAttributes::Id("3")};
  const LaneLink::LinkAttributes kLaneLinkRight{LaneLink::LinkAttributes::Id("-1")};

  const std::string xml_description{
      BuildXMLDescriptionFromRoadLinks(kRoadPredecessor, kRoadSuccessor, kLaneLinkLeft, kLaneLinkRight)};
  EXPECT_THROW(
      LoadDataBaseFromStr(xml_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
      maliput::common::assertion_error);
}

// Template of a XODR description describing a multiple LaneSections Road in which lane links can be changed.
const std::string kXODRRoadHeaderMultipleLaneSectionLinksVariableTemplate = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='TestHeader' version='1.21' date='Wed Sep 19 12:00:00 2018'
    north='0.' south='0.' east='0.' west='0.' vendor='TestVendor' >
  </header>
  <road name='Road 0' length='10.' id='0' junction='-1' rule='RHT'>"
    <link>
    </link>
    <planView>
      <geometry s='0.0' x='500.0' y='80.0' hdg='0.' length='10.'>
          <line/>
      </geometry>
    </planView>
    <lanes>
      <laneOffset s='0.5' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <laneSection s='0.0'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                      {}
                      {}
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
              </lane>
          </left>
          <center>
              <lane id='0' type='driving' level= '0'>
              </lane>
          </center>
          <right>
              <lane id='-1' type='driving' level= '0'>
                  <link>
                      {}
                      {}
                  </link>
                  <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
              </lane>
          </right>
      </laneSection>
      <laneSection s='5.0'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                      {}
                      {}
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
              </lane>
          </left>
          <center>
              <lane id='0' type='driving' level= '0'>
              </lane>
          </center>
          <right>
              <lane id='-1' type='driving' level= '0'>
                  <link>
                      {}
                      {}
                  </link>
                  <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
              </lane>
          </right>
      </laneSection>
      <laneSection s='10.0'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                      {}
                      {}
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
              </lane>
          </left>
          <center>
              <lane id='0' type='driving' level= '0'>
              </lane>
          </center>
          <right>
              <lane id='-1' type='driving' level= '0'>
                  <link>
                      {}
                      {}
                  </link>
                  <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
              </lane>
          </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
)R";

// Tests LaneLinks connection within a Road with multiples LaneSections.
class DBManagerLaneLinksWithinARoad : public DBManagerLinksTests {
 protected:
  using OptLaneLink = std::optional<LaneLink::LinkAttributes>;
  std::string BuildXMLDescriptionFromLaneLinks(
      const std::array<std::pair<OptLaneLink, std::string>, 12>& lane_links_direction) {
    std::array<std::string, 12> lane_links_str;
    for (int i = 0; i < 12; i++) {
      lane_links_str[i] = lane_links_direction[i].first.has_value()
                              ? fmt::format(kRoadLaneLinkTemplate, lane_links_direction[i].second,
                                            lane_links_direction[i].first->id.string())
                              : "";
    }
    return fmt::format(kXODRRoadHeaderMultipleLaneSectionLinksVariableTemplate, lane_links_str[0], lane_links_str[1],
                       lane_links_str[2], lane_links_str[3], lane_links_str[4], lane_links_str[5], lane_links_str[6],
                       lane_links_str[7], lane_links_str[8], lane_links_str[9], lane_links_str[10], lane_links_str[11]);
  }
};

TEST_F(DBManagerLaneLinksWithinARoad, CorrectLaneLinks) {
  // Lane links of laneSection 0.
  const OptLaneLink kLane0LeftPredecessor(std::nullopt);
  const OptLaneLink kLane0LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane0RightPredecessor(std::nullopt);
  const OptLaneLink kLane0RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  const LaneLink kLaneLink0Left{kLane0LeftPredecessor, kLane0LeftSuccessor};
  const LaneLink kLaneLink0Right{kLane0RightPredecessor, kLane0RightSuccessor};
  // LaneSection 0's lanes.
  const Lane kLeftLane0{Lane::Id("1"), Lane::Type::kDriving, false, kLaneLink0Left,
                        std::vector<LaneWidth>{{0., 1., 2., 3., 4.}}};
  const Lane kCenterLane0{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane kRightLane0{Lane::Id("-1"), Lane::Type::kDriving, false, kLaneLink0Right,
                         std::vector<LaneWidth>{{5., 6., 7., 8., 9.}}};
  // LaneSection 0.
  const LaneSection kExpectedLaneSection0{0., std::nullopt, {kLeftLane0}, kCenterLane0, {kRightLane0}};
  // Lane links of laneSection 1.
  const OptLaneLink kLane1LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane1RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  const LaneLink kLaneLink1Left{kLane1LeftPredecessor, kLane1LeftSuccessor};
  const LaneLink kLaneLink1Right{kLane1RightPredecessor, kLane1RightSuccessor};
  // LaneSection 1's lanes.
  const Lane kLeftLane1{Lane::Id("1"), Lane::Type::kDriving, false, kLaneLink1Left,
                        std::vector<LaneWidth>{{0., 1., 2., 3., 4.}}};
  const Lane kCenterLane1{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane kRightLane1{Lane::Id("-1"), Lane::Type::kDriving, false, kLaneLink1Right,
                         std::vector<LaneWidth>{{5., 6., 7., 8., 9.}}};
  // LaneSection 1.
  const LaneSection kExpectedLaneSection1{5., std::nullopt, {kLeftLane1}, kCenterLane1, {kRightLane1}};
  // Lane links of laneSection 2.
  const OptLaneLink kLane2LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane2LeftSuccessor(std::nullopt);
  const OptLaneLink kLane2RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane2RightSuccessor(std::nullopt);
  const LaneLink kLaneLink2Left{kLane2LeftPredecessor, kLane2LeftSuccessor};
  const LaneLink kLaneLink2Right{kLane2RightPredecessor, kLane2RightSuccessor};
  // LaneSection 2's lanes.
  const Lane kLeftLane2{Lane::Id("1"), Lane::Type::kDriving, false, kLaneLink2Left,
                        std::vector<LaneWidth>{{0., 1., 2., 3., 4.}}};
  const Lane kCenterLane2{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane kRightLane2{Lane::Id("-1"), Lane::Type::kDriving, false, kLaneLink2Right,
                         std::vector<LaneWidth>{{5., 6., 7., 8., 9.}}};
  // LaneSection 2.
  const LaneSection kExpectedLaneSection2{10., std::nullopt, {kLeftLane2}, kCenterLane2, {kRightLane2}};
  const std::string xodr_description =
      BuildXMLDescriptionFromLaneLinks({{{kLane0LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane0RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2RightSuccessor, LaneLink::kSuccessorTag}}});

  const std::unique_ptr<DBManager> dut = LoadDataBaseFromStr(
      xodr_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  const auto lane_sections = dut->GetRoadHeaders().at(RoadHeader::Id("0")).lanes.lanes_section;
  EXPECT_EQ(kExpectedLaneSection0, lane_sections[0]);
  EXPECT_EQ(kExpectedLaneSection1, lane_sections[1]);
  EXPECT_EQ(kExpectedLaneSection2, lane_sections[2]);
}

TEST_F(DBManagerLaneLinksWithinARoad, ErrorInLaneLinkLaneSection0) {
  // Lane links of laneSection 0.
  const OptLaneLink kLane0LeftPredecessor(std::nullopt);
  const OptLaneLink kWrongLane0LeftSuccessor({LaneLink::LinkAttributes::Id("3")});
  const OptLaneLink kLane0RightPredecessor(std::nullopt);
  const OptLaneLink kLane0RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 1.
  const OptLaneLink kLane1LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane1RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 2.
  const OptLaneLink kLane2LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane2LeftSuccessor(std::nullopt);
  const OptLaneLink kLane2RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane2RightSuccessor(std::nullopt);
  const std::string xodr_description =
      BuildXMLDescriptionFromLaneLinks({{{kLane0LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kWrongLane0LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane0RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2RightSuccessor, LaneLink::kSuccessorTag}}});
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

TEST_F(DBManagerLaneLinksWithinARoad, ErrorInPredecessorLaneLinkLaneSection1) {
  // Lane links of laneSection 0.
  const OptLaneLink kLane0LeftPredecessor(std::nullopt);
  const OptLaneLink kLane0LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane0RightPredecessor(std::nullopt);
  const OptLaneLink kLane0RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 1.
  const OptLaneLink kLane1LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kWrongLane1RightPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 2.
  const OptLaneLink kLane2LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane2LeftSuccessor(std::nullopt);
  const OptLaneLink kLane2RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane2RightSuccessor(std::nullopt);
  const std::string xodr_description =
      BuildXMLDescriptionFromLaneLinks({{{kLane0LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane0RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kWrongLane1RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2RightSuccessor, LaneLink::kSuccessorTag}}});
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

TEST_F(DBManagerLaneLinksWithinARoad, ErrorInSuccessorLaneLinkLaneSection1) {
  // Lane links of laneSection 0.
  const OptLaneLink kLane0LeftPredecessor(std::nullopt);
  const OptLaneLink kLane0LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane0RightPredecessor(std::nullopt);
  const OptLaneLink kLane0RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 1.
  const OptLaneLink kLane1LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kWrongLane1LeftSuccessor({LaneLink::LinkAttributes::Id("-4")});
  const OptLaneLink kLane1RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane1RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 2.
  const OptLaneLink kLane2LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane2LeftSuccessor(std::nullopt);
  const OptLaneLink kLane2RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane2RightSuccessor(std::nullopt);
  const std::string xodr_description =
      BuildXMLDescriptionFromLaneLinks({{{kLane0LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane0RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kWrongLane1LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2RightSuccessor, LaneLink::kSuccessorTag}}});
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

TEST_F(DBManagerLaneLinksWithinARoad, ErrorInLaneLinkLaneSection2) {
  // Lane links of laneSection 0.
  const OptLaneLink kLane0LeftPredecessor(std::nullopt);
  const OptLaneLink kLane0LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane0RightPredecessor(std::nullopt);
  const OptLaneLink kLane0RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 1.
  const OptLaneLink kLane1LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane1RightPredecessor({LaneLink::LinkAttributes::Id("-1")});
  const OptLaneLink kLane1RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 2.
  const OptLaneLink kLane2LeftPredecessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane2LeftSuccessor(std::nullopt);
  const OptLaneLink kWrongLane2RightPredecessor({LaneLink::LinkAttributes::Id("-2")});
  const OptLaneLink kLane2RightSuccessor(std::nullopt);
  const std::string xodr_description =
      BuildXMLDescriptionFromLaneLinks({{{kLane0LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane0RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane2LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kWrongLane2RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane2RightSuccessor, LaneLink::kSuccessorTag}}});
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
}

// Allow semantic errors when lane links withing a Road aren't consistent.
TEST_F(DBManagerLaneLinksWithinARoad, RelaxErrorsInLaneLinkVerification) {
  constexpr bool kAllowSemanticErrors{true};
  // Lane links of laneSection 0.
  const OptLaneLink kLane0LeftPredecessor(std::nullopt);
  const OptLaneLink kLane0LeftSuccessor({LaneLink::LinkAttributes::Id("1")});
  const OptLaneLink kLane0RightPredecessor(std::nullopt);
  const OptLaneLink kLane0RightSuccessor({LaneLink::LinkAttributes::Id("-1")});
  // Lane links of laneSection 1.
  const OptLaneLink kLane1LeftPredecessor({LaneLink::LinkAttributes::Id("2")});
  const OptLaneLink kLane1LeftSuccessor({LaneLink::LinkAttributes::Id("2")});
  const OptLaneLink kLane1RightPredecessor({LaneLink::LinkAttributes::Id("-2")});
  const OptLaneLink kLane1RightSuccessor({LaneLink::LinkAttributes::Id("-2")});
  const std::string xodr_description =
      BuildXMLDescriptionFromLaneLinks({{{kLane0LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane0RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane0RightSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1LeftPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1LeftSuccessor, LaneLink::kSuccessorTag},
                                         {kLane1RightPredecessor, LaneLink::kPredecessorTag},
                                         {kLane1RightSuccessor, LaneLink::kSuccessorTag}}});
  // Semantic errors aren't allowed.
  EXPECT_THROW(LoadDataBaseFromStr(xodr_description,
                                   {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}),
               maliput::common::assertion_error);
  // Semantic errors are allowed.
  EXPECT_NO_THROW(
      LoadDataBaseFromStr(xodr_description, {kStrictParserSTolerance, kDontAllowSchemaErrors, kAllowSemanticErrors}));
}

// Template of a XODR description that contains several road headers.
const std::string kXODRRoadHeaderTemplate = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='TestHeader' version='1.21' date='Wed Sep 19 12:00:00 2018'
    north='0.' south='0.' east='0.' west='0.' vendor='TestVendor' >
  </header>
  <road name='TestRoadHeader1' length='10.65' id='15' junction='-1' rule='RHT'>"
    <link>
        <successor elementType="road" elementId="30" contactPoint="start"/>
    </link>
    <type s="0.0000000000000000e+0" type="unknown">
    </type>
    <type s="25.0000000000000000e+0" type="town">
        <speed max="45." unit="mph"/>
    </type>
    <type s="53.0000000000000000e+0" type="rural" country="maliput">
        <speed max="8."/>
    </type>
    <planView>
      <geometry s='0.5' x='523.2' y='83.27' hdg='0.77' length='10.65'>
          <line/>
      </geometry>
    </planView>
    <lanes>
      <laneOffset s='0.5' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <laneSection s='0.5'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                    <predecessor id='1'/>
                    <successor id='1'/>
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
                  <speed sOffset='0.' max='45.' unit='mph'/>
              </lane>
          </left>
          <center>
              <lane id='0' type='driving' level= '0'>
              </lane>
          </center>
          <right>
              <lane id='-1' type='driving' level= '0'>
                  <link>
                    <predecessor id='-1'/>
                    <successor id='-1'/>
                  </link>
                  <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
                  <speed sOffset='0.5' max='3.'/>
              </lane>
          </right>
      </laneSection>
    </lanes>
  </road>
  <road name='TestRoadHeader2' length='100.' id='30' junction='-1' rule='LHT'>"
    <link>
        <predecessor elementType="road" elementId="15" contactPoint="end"/>
    </link>
    <planView>
      <geometry s='0.5' x='100.' y='50.' hdg='0.20' length='100.'>
          <arc curvature='0.34'/>
      </geometry>
    </planView>
    <elevationProfile>
      <elevation s='1.1' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <elevation s='6.6' a='7.7' b='8.8' c='9.9' d='10.10'/>
    </elevationProfile>
    <lateralProfile>
      <superelevation s='15.51' a='25.52' b='35.53' c='45.54' d='55.55'/>
      <superelevation s='65.56' a='75.57' b='85.58' c='95.59' d='105.510'/>
    </lateralProfile>
    <lanes>
      <laneOffset s='0.5' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <laneSection s='0.5'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                    <predecessor id='1'/>
                    <successor id='1'/>
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
                  <speed sOffset='0.' max='45.' unit='mph'/>
              </lane>
          </left>
          <center>
              <lane id='0' type='driving' level= '0'>
              </lane>
          </center>
          <right>
              <lane id='-1' type='driving' level= '0'>
                  <link>
                    <predecessor id='-1'/>
                    <successor id='-1'/>
                  </link>
                  <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
                  <speed sOffset='0.5' max='3.'/>
              </lane>
          </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
)R";

// Tests DBManager::GetRoadHeaders method.
// TODO(francocipollone): Provide multiple xodr descriptions with good and bad xodr descriptions
// in order to improve the test's scope.
GTEST_TEST(DBManagerTest, GetRoadHeaders) {
  const RoadLink::LinkAttributes kPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                              RoadLink::LinkAttributes::Id("15") /* elementId*/,
                                              RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                            RoadLink::LinkAttributes::Id("30") /* elementId*/,
                                            RoadLink::ContactPoint::kStart /* contactPoint*/};
  const RoadLink kExpectedRoadLink1{std::nullopt, kSuccessor};
  const RoadLink kExpectedRoadLink2{kPredecessor, std::nullopt};
  const PlanView kArbitraryPlanView1{std::vector<Geometry>{Geometry{0.5 /* s_0 */,
                                                                    {523.2 /* x */, 83.27 /* y */},
                                                                    0.77 /* orientation */,
                                                                    10.65 /* length */,
                                                                    Geometry::Type::kLine /* Type */,
                                                                    Geometry::Line{} /* description */}}};
  const PlanView kArbitraryPlanView2{{Geometry{0.5 /* s_0 */,
                                               {100. /* x */, 50. /* y */},
                                               0.2 /* orientation */,
                                               100. /* length */,
                                               Geometry::Type::kArc /* Type */,
                                               Geometry::Arc{0.34} /* description */}}};
  const ElevationProfile kElevationProfile2{{{1.1, 2.2, 3.3, 4.4, 5.5}, {6.6, 7.7, 8.8, 9.9, 10.10}}};
  const LateralProfile kLateralProfile2{{{15.51, 25.52, 35.53, 45.54, 55.55}, {65.56, 75.57, 85.58, 95.59, 105.510}}};
  const LaneOffset kLaneOffset{0.5 /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  const LaneLink kLaneLinkA{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* successor */};
  const LaneLink kLaneLinkB{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* successor */};
  const RoadType kRoadType0{
      0. /* s0 */, RoadType::Type::kUnknown /* type */, std::nullopt /* country */, {} /* speed */};
  const RoadType kRoadType1{25. /* s0 */,
                            RoadType::Type::kTown /* type */,
                            std::nullopt /* country */,
                            {45. /* max */, Unit::kMph /* unit */} /* speed */};
  const RoadType kRoadType2{53. /* s0 */,
                            RoadType::Type::kRural /* type */,
                            "maliput" /* country */,
                            {8. /* max */, {} /* unit */} /* speed */};
  const Lane::Speed kLaneSpeedA{0.0 /* sOffset */, 45. /* max */, Unit::kMph /* unit */};
  const Lane::Speed kLaneSpeedB{0.5 /* sOffset */, 3. /* max */, Unit::kMs /* unit */};
  const Lane kLeftLane{
      Lane::Id("1"), Lane::Type::kDriving, false, kLaneLinkA, std::vector<LaneWidth>{{0., 1., 2., 3., 4.}},
      {kLaneSpeedA}};
  const Lane kCenterLane{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane kRightLane{
      Lane::Id("-1"), Lane::Type::kDriving, false, kLaneLinkB, std::vector<LaneWidth>{{5., 6., 7., 8., 9.}},
      {kLaneSpeedB}};
  const LaneSection kLaneSection{0.5 /* s_0 */,
                                 std::nullopt /* single_side */,
                                 {kLeftLane} /* left_lanes */,
                                 kCenterLane /* center_lane */,
                                 {kRightLane} /* right_lanes */};
  const Lanes kLanes{{{kLaneOffset}}, {{kLaneSection}}};
  const RoadHeader::Id kExpectedRoadHeaderIds[2]{RoadHeader::Id{"15"}, RoadHeader::Id{"30"}};
  const std::map<RoadHeader::Id, RoadHeader> kExpectedRoadHeaders{
      {kExpectedRoadHeaderIds[0],
       {"TestRoadHeader1" /* name */,
        10.65 /* length */,
        kExpectedRoadHeaderIds[0] /* id */,
        "-1" /* junction */,
        RoadHeader::HandTrafficRule::kRHT /* rule */,
        kExpectedRoadLink1 /* road_link */,
        {kRoadType0, kRoadType1, kRoadType2} /* road_types */,
        {kArbitraryPlanView1, {}} /* reference_geometry */,
        kLanes /* lanes */}},
      {kExpectedRoadHeaderIds[1],
       {"TestRoadHeader2" /* name */,
        100. /* length */,
        kExpectedRoadHeaderIds[1] /* id */,
        "-1" /* junction */,
        RoadHeader::HandTrafficRule::kLHT /* rule */,
        kExpectedRoadLink2 /* road_link */,
        {} /* road_types */,
        {kArbitraryPlanView2, kElevationProfile2, kLateralProfile2} /* reference_geometry */,
        kLanes /* lanes */}}};

  const std::unique_ptr<DBManager> dut = LoadDataBaseFromStr(
      kXODRRoadHeaderTemplate, {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  const std::map<RoadHeader::Id, RoadHeader> road_headers = dut->GetRoadHeaders();

  EXPECT_EQ(kExpectedRoadHeaders, road_headers);
}

// Tests DBManager::GetJunctions method.
// Loads TShapeRoad map:
//     - 3 roads that doesn't belong to a junction.
//     - 1 Junction
//       - 6 roads that belong to a junction.
GTEST_TEST(DBManagerTest, GetJunctions) {
  const Connection kConnection0{Connection::Id("0") /* id */,
                                "1" /* incoming_road */,
                                "4" /* connecting_road */,
                                Connection::ContactPoint::kEnd /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")}} /* lane_links */};
  const Connection kConnection1{Connection::Id("1") /* id */,
                                "0" /* incoming_road */,
                                "5" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")}} /* lane_links */};
  const Connection kConnection2{Connection::Id("2") /* id */,
                                "1" /* incoming_road */,
                                "6" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("1"), Connection::LaneLink::Id("-1")}} /* lane_links */};
  const Connection kConnection3{Connection::Id("3") /* id */,
                                "2" /* incoming_road */,
                                "7" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")}} /* lane_links */};
  const Connection kConnection4{Connection::Id("4") /* id */,
                                "2" /* incoming_road */,
                                "8" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")}} /* lane_links */};
  const Connection kConnection5{Connection::Id("5") /* id */,
                                "0" /* incoming_road */,
                                "9" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")}} /* lane_links */};
  const Junction kExpectedJunction{Junction::Id("3") /* id */,
                                   "" /* name */,
                                   std::nullopt /* type */,
                                   {{kConnection0.id, kConnection0},
                                    {kConnection1.id, kConnection1},
                                    {kConnection2.id, kConnection2},
                                    {kConnection3.id, kConnection3},
                                    {kConnection4.id, kConnection4},
                                    {kConnection5.id, kConnection5}} /* connections */};

  const std::unordered_map<Junction::Id, Junction> kExpectedJunctions{{kExpectedJunction.id, kExpectedJunction}};
  const std::string kXodrFile = "TShapeRoad.xodr";
  // If I decrease even more the tolerance, the TShapeRoad.xodr's geometries aren't contiguous in terms of the arc
  // length parameter.
  // TODO(#482): Decrease tolerance once the xodr file is fixed.
  const std::unique_ptr<DBManager> dut =
      LoadDataBaseFromFile(utility::FindResourceInPath(kXodrFile, kMalidriveResourceFolder), {1e-6});

  const std::unordered_map<Junction::Id, Junction> junctions = dut->GetJunctions();

  EXPECT_EQ(kExpectedJunctions, junctions);
}

// Returns a travelDir-userData node with the directive provided by `travel_dir`.
std::string LaneUserDataTravelDirTemplate(const std::string& travel_dir) {
  return fmt::format(
      R"R(<userData>
    <vectorLane travelDir="{}"/>
</userData>
)R",
      travel_dir);
}

// Loads Highway map.
//
// RoadLinks and Junctions were already tested in previous tests .
// However, this test's goal is showing the correct functioning by loading a long XODR description and selecting a
// random Road and Junction and checking that the values are well parsed.
GTEST_TEST(DBManagerTest, Highway) {
  const std::string kXodrFile = "Highway.xodr";
  const int NumOfRoads{37};
  const int NumOfJunctions{6};
  // Road 52 of Highway description:
  // @{
  const RoadLink::LinkAttributes kPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                              RoadLink::LinkAttributes::Id("1") /* elementId*/,
                                              RoadLink::ContactPoint::kStart /* contactPoint*/};
  const RoadLink::LinkAttributes kSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                            RoadLink::LinkAttributes::Id("13") /* elementId*/,
                                            RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink kExpectedRoadLink{kPredecessor, kSuccessor};
  const PlanView kPlanView{{Geometry{0. /* s_0 */,
                                     {-5.0699830717506373e+1 /* x */, 1.7300712434268502e+2 /* y */},
                                     4.9059097257144400e+0 /* orientation */,
                                     1.0336660934588906e+1 /* length */,
                                     Geometry::Type::kArc /* Type */,
                                     Geometry::Arc{-5.5921424523716991e-2} /* description */},
                            Geometry{1.0336660934588904e+1 /* s_0 */,
                                     {-5.1671813359707308e+1 /* x */, 1.6286021921862766e+2 /* y */},
                                     4.3278689214335735e+0 /* orientation */,
                                     3.0371685926263199e+0 /* length */,
                                     Geometry::Type::kArc /* Type */,
                                     Geometry::Arc{-5.5921424523716991e-2} /* description */},
                            Geometry{1.3373829527215225e+1 /* s_0 */,
                                     {-5.3044141851610902e+1 /* x */, 1.6015486258116991e+2 /* y */},
                                     4.1580261272152175e+0 /* orientation */,
                                     5.5402357751415368e+0 /* length */,
                                     Geometry::Type::kLine /* Type */,
                                     Geometry::Line{} /* description */},
                            Geometry{1.8914065302356761e+1 /* s_0 */,
                                     {-5.5960547992372049e+1 /* x */, 1.5544432764177000e+2 /* y */},
                                     4.1580261272152157e+0 /* orientation */,
                                     5.4279091869543947e+1 /* length */,
                                     Geometry::Type::kLine /* Type */,
                                     Geometry::Line{} /* description */},
                            Geometry{7.3193157171900708e+1 /* s_0 */,
                                     {-8.4533153245060589e+1 /* x */, 1.0929428925330896e+2 /* y */},
                                     4.1580261272152157e+0 /* orientation */,
                                     2.0500000000000682e+0 /* length */,
                                     Geometry::Type::kLine /* Type */,
                                     Geometry::Line{} /* description */}}};
  const ElevationProfile kElevationProfile{{
      ElevationProfile::Elevation{0.0000000000000000e+0, 1.0e+1, 0., 0., 0.},
      ElevationProfile::Elevation{1.0336660934588904e+1, 1.0e+1, 0., 0., 0.},
      ElevationProfile::Elevation{1.8914065302356761e+1, 1.0e+1, 0., 0., 0.},
      ElevationProfile::Elevation{7.3193157171900708e+1, 1.0e+1, 0., 0., 0.},
  }};
  const std::vector<LaneOffset> kLaneOffsets{
      {0.0000000000000000e+0 /* s */, 6.3500000000000001e-1 /* a */, 0. /* b */, 0. /* c */, 0. /* d */},
      {1.0336660934588904e+1 /* s */, 0.0000000000000000e+0 /* a */, 0. /* b */, 0. /* c */, 0. /* d */},
      {1.8914065302356761e+1 /* s */, 1.1500000000000011e+1 /* a */, 0. /* b */, 0. /* c */, 0. /* d */},
      {7.3193157171900708e+1 /* s */, 1.1500000000000011e+1 /* a */, 0. /* b */, 0. /* c */, 0. /* d */},
  };
  const LaneLink kLaneLink0_2{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"2"}} /* predecessor */,
                              LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* successor */};
  const LaneLink kLaneLink1_1{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-2"}} /* predecessor */,
                              LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-6"}} /* successor */};
  const LaneLink kLaneLink2_6{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* predecessor */,
                              LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-6"}} /* successor */};
  const LaneLink kLaneLink3_6{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-6"}} /* predecessor */,
                              LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"6"}} /* successor */};

  // lane section 0
  const Lane kCenterLane0{
      Lane::Id("0"), Lane::Type::kNone, false, {}, {}, {}, LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane0_1{Lane::Id("-1"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 6.3500000000000001e-1, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane0_2{Lane::Id("-2"),
                           Lane::Type::kDriving,
                           false,
                           kLaneLink0_2,
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const LaneSection kLaneSection0{0. /* s_0 */,
                                  std::nullopt /* single_side */,
                                  {} /* left_lanes */,
                                  kCenterLane0 /* center_lane */,
                                  {kRightLane0_2, kRightLane0_1} /* right_lanes */};
  // lane section 1
  const Lane kCenterLane1{Lane::Id("0"), Lane::Type::kNone, false, {}, {}, {}, {}};
  const Lane kRightLane1_1{Lane::Id("-1"),
                           Lane::Type::kDriving,
                           false,
                           kLaneLink1_1,
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const LaneSection kLaneSection1{1.0336660934588904e+1 /* s_0 */,
                                  std::nullopt /* single_side */,
                                  {} /* left_lanes */,
                                  kCenterLane1 /* center_lane */,
                                  {kRightLane1_1} /* right_lanes */};

  // lane section 2
  const Lane kCenterLane2{
      Lane::Id("0"), Lane::Type::kNone, false, {}, {}, {}, LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane2_1{Lane::Id("-1"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 5.0e-1, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane2_2{Lane::Id("-2"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane2_3{Lane::Id("-3"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 5.0e-1, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane2_4{Lane::Id("-4"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const Lane kRightLane2_5{Lane::Id("-5"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const Lane kRightLane2_6{Lane::Id("-6"),
                           Lane::Type::kDriving,
                           false,
                           kLaneLink2_6,
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const LaneSection kLaneSection2{
      1.8914065302356761e+1 /* s_0 */,
      std::nullopt /* single_side */,
      {} /* left_lanes */,
      kCenterLane2 /* center_lane */,
      {kRightLane2_6, kRightLane2_5, kRightLane2_4, kRightLane2_3, kRightLane2_2, kRightLane2_1} /* right_lanes */};

  // lane section 3
  const Lane kCenterLane3{
      Lane::Id("0"), Lane::Type::kNone, false, {}, {}, {}, LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane3_1{Lane::Id("-1"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 5.0e-1, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane3_2{Lane::Id("-2"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane3_3{Lane::Id("-3"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 5.0e-1, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("undirected")};
  const Lane kRightLane3_4{Lane::Id("-4"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const Lane kRightLane3_5{Lane::Id("-5"),
                           Lane::Type::kNone,
                           false,
                           {},
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const Lane kRightLane3_6{Lane::Id("-6"),
                           Lane::Type::kDriving,
                           false,
                           kLaneLink3_6,
                           std::vector<LaneWidth>{{0., 3.5, 0., 0., 0.}},
                           {},
                           LaneUserDataTravelDirTemplate("forward")};
  const LaneSection kLaneSection3{
      7.3193157171900708e+1 /* s_0 */,
      std::nullopt /* single_side */,
      {} /* left_lanes */,
      kCenterLane3 /* center_lane */,
      {kRightLane3_6, kRightLane3_5, kRightLane3_4, kRightLane3_3, kRightLane3_2, kRightLane3_1} /* right_lanes */};

  const Lanes kLanes{{kLaneOffsets}, {{kLaneSection0, kLaneSection1, kLaneSection2, kLaneSection3}}};
  const RoadHeader kExpectedRoadHeader{std::string("Road 52") /* name */,
                                       7.5243157171900776e+1 /* length */,
                                       RoadHeader::Id("52") /* id */,
                                       std::string("50") /* junction */,
                                       std::nullopt,
                                       kExpectedRoadLink /* road_link */,
                                       {} /* road_types */,
                                       {kPlanView, kElevationProfile, {}} /* reference_geometry */,
                                       kLanes /* lanes */};
  // @}

  // Junction 17 of Highway description:
  // @{
  const Connection kConnection17_0{Connection::Id("0") /* id */,
                                   "9" /* incoming_road */,
                                   "27" /* connecting_road */,
                                   Connection::ContactPoint::kStart /* contact_point */,
                                   std::nullopt /* connection_master */,
                                   std::nullopt /* type */,
                                   {{Connection::LaneLink::Id("7"), Connection::LaneLink::Id("7")},
                                    {Connection::LaneLink::Id("6"), Connection::LaneLink::Id("6")},
                                    {Connection::LaneLink::Id("5"), Connection::LaneLink::Id("5")},
                                    {Connection::LaneLink::Id("4"), Connection::LaneLink::Id("4")}} /* lane_links */};
  const Connection kConnection17_1{Connection::Id("1") /* id */,
                                   "10" /* incoming_road */,
                                   "27" /* connecting_road */,
                                   Connection::ContactPoint::kEnd /* contact_point */,
                                   std::nullopt /* connection_master */,
                                   std::nullopt /* type */,
                                   {{Connection::LaneLink::Id("7"), Connection::LaneLink::Id("7")},
                                    {Connection::LaneLink::Id("6"), Connection::LaneLink::Id("6")},
                                    {Connection::LaneLink::Id("5"), Connection::LaneLink::Id("5")},
                                    {Connection::LaneLink::Id("4"), Connection::LaneLink::Id("4")}} /* lane_links */};
  const Connection kConnection17_2{Connection::Id("2") /* id */,
                                   "9" /* incoming_road */,
                                   "28" /* connecting_road */,
                                   Connection::ContactPoint::kStart /* contact_point */,
                                   std::nullopt /* connection_master */,
                                   std::nullopt /* type */,
                                   {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                    {Connection::LaneLink::Id("-2"), Connection::LaneLink::Id("-2")},
                                    {Connection::LaneLink::Id("-3"), Connection::LaneLink::Id("-3")},
                                    {Connection::LaneLink::Id("-4"), Connection::LaneLink::Id("-4")}} /* lane_links */};
  const Connection kConnection17_3{Connection::Id("3") /* id */,
                                   "10" /* incoming_road */,
                                   "28" /* connecting_road */,
                                   Connection::ContactPoint::kEnd /* contact_point */,
                                   std::nullopt /* connection_master */,
                                   std::nullopt /* type */,
                                   {{Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")},
                                    {Connection::LaneLink::Id("-2"), Connection::LaneLink::Id("-2")},
                                    {Connection::LaneLink::Id("-3"), Connection::LaneLink::Id("-3")},
                                    {Connection::LaneLink::Id("-4"), Connection::LaneLink::Id("-4")}} /* lane_links */};
  const Connection kConnection17_4{Connection::Id("4") /* id */,
                                   "10" /* incoming_road */,
                                   "30" /* connecting_road */,
                                   Connection::ContactPoint::kStart /* contact_point */,
                                   std::nullopt /* connection_master */,
                                   std::nullopt /* type */,
                                   {{Connection::LaneLink::Id("7"), Connection::LaneLink::Id("-7")}} /* lane_links */};
  const Connection kConnection17_5{Connection::Id("5") /* id */,
                                   "4" /* incoming_road */,
                                   "30" /* connecting_road */,
                                   Connection::ContactPoint::kEnd /* contact_point */,
                                   std::nullopt /* connection_master */,
                                   std::nullopt /* type */,
                                   {{Connection::LaneLink::Id("1"), Connection::LaneLink::Id("-1")}} /* lane_links */};
  const Junction kExpectedJunction17{Junction::Id("17"),
                                     "junction17",
                                     std::nullopt,
                                     {{kConnection17_0.id, kConnection17_0},
                                      {kConnection17_1.id, kConnection17_1},
                                      {kConnection17_2.id, kConnection17_2},
                                      {kConnection17_3.id, kConnection17_3},
                                      {kConnection17_4.id, kConnection17_4},
                                      {kConnection17_5.id, kConnection17_5}}};
  // @}

  const std::unique_ptr<DBManager> dut =
      LoadDataBaseFromFile(utility::FindResourceInPath(kXodrFile, kMalidriveResourceFolder), {kStrictParserSTolerance});

  const std::map<RoadHeader::Id, RoadHeader> road_headers = dut->GetRoadHeaders();
  EXPECT_EQ(NumOfRoads, static_cast<int>(road_headers.size()));
  EXPECT_EQ(kExpectedRoadHeader, road_headers.at(kExpectedRoadHeader.id));

  const std::unordered_map<Junction::Id, Junction> junctions = dut->GetJunctions();
  EXPECT_EQ(NumOfJunctions, static_cast<int>(junctions.size()));
  EXPECT_EQ(kExpectedJunction17, junctions.at(kExpectedJunction17.id));
}

GTEST_TEST(DBManagerTest, LoadMapWith2Connections) {
  // Describes a junction with a connectivity map where the connecting road is repeated.
  // Used lane ids do not represent a real map, but they help to verify Lane Links work
  // fine when start and end road's lane ids are different.
  static const std::string kXODRMapWithMultipleConnections = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='TestHeader' version='1.21' date='Wed Sep 19 12:00:00 2018'
    north='0.' south='0.' east='0.' west='0.' vendor='TestVendor' >
  </header>
    <road name="Road 27" length="10" id="27" junction="-1">
        <link>
            <predecessor elementType="junction" elementId="831"/>
        </link>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10">
                <line/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.0000000000000000e+0">
                <center>
                    <lane id="0" type="none" level="false">
                    </lane>
                </center>
                <right>
                    <lane id="-2" type="driving" level="false">
                        <width sOffset="0.0000000000000000e+0" a="3.5000000000000000e+0" b="0.0000000000000000e+0" c="0.0000000000000000e+0" d="0.0000000000000000e+0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
    <road name="Road 40" length="2.4551210325000223e+0" id="40" junction="-1">
        <link>
            <successor elementType="junction" elementId="831"/>
        </link>
        <planView>
            <geometry s="0.0000000000000000e+0" x="20" y="0" hdg="0" length="10">
                <line/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.0000000000000000e+0">
                <center>
                    <lane id="0" type="none" level="false">
                    </lane>
                </center>
                <right>
                    <lane id="-4" type="driving" level="false">
                        <link>
                        </link>
                        <width sOffset="0.0000000000000000e+0" a="3.5000000000000000e+0" b="0.0000000000000000e+0" c="0.0000000000000000e+0" d="0.0000000000000000e+0"/>
                    </lane>
                    <lane id="-5" type="driving" level="false">
                        <link>
                        </link>
                        <width sOffset="0.0000000000000000e+0" a="3.5000000000000000e+0" b="0.0000000000000000e+0" c="0.0000000000000000e+0" d="0.0000000000000000e+0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
    <road name="Road 847" length="10" id="847" junction="831">
        <link>
            <predecessor elementType="road" elementId="40" contactPoint="end"/>
            <successor elementType="road" elementId="27" contactPoint="start"/>
        </link>
        <planView>
            <geometry s="0" x="10" y="0" hdg="0" length="10">
                <line/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.0000000000000000e+0">
                <center>
                    <lane id="0" type="none" level="false">
                    </lane>
                </center>
                <right>
                    <lane id="-5" type="driving" level="false">
                        <link>
                            <predecessor id="-5"/>
                            <successor id="-5"/>
                        </link>
                        <width sOffset="0.0000000000000000e+0" a="3.5000000000000000e+0" b="0.0000000000000000e+0" c="0.0000000000000000e+0" d="0.0000000000000000e+0"/>
                    </lane>
                </right>
            </laneSection>
            <laneSection s="1.1837467941303236e+1">
                <center>
                    <lane id="0" type="none" level="false">
                    </lane>
                </center>
                <right>
                    <lane id="-5" type="driving" level="false">
                        <link>
                            <predecessor id="-5"/>
                            <successor id="-2"/>
                        </link>
                        <width sOffset="0.0000000000000000e+0" a="3.5000000000000000e+0" b="0.0000000000000000e+0" c="0.0000000000000000e+0" d="0.0000000000000000e+0"/>
                    </lane>
                </right>
            </laneSection>
            <laneSection s="1.4335927289340910e+1">
                <center>
                    <lane id="0" type="none" level="false">
                    </lane>
                </center>
                <right>
                    <lane id="-2" type="driving" level="false">
                        <link>
                            <predecessor id="-5"/>
                            <successor id="-2"/>
                        </link>
                        <width sOffset="0.0000000000000000e+0" a="3.5000000000000000e+0" b="0.0000000000000000e+0" c="0.0000000000000000e+0" d="0.0000000000000000e+0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
    <junction id="831" name="junction831">
        <connection id="1" incomingRoad="40" connectingRoad="847" contactPoint="start">
            <laneLink from="-5" to="-5"/>
        </connection>
        <connection id="2" incomingRoad="27" connectingRoad="847" contactPoint="start">
            <laneLink from="-2" to="-2"/>
        </connection>
    </junction>
</OpenDRIVE>
)R";
  EXPECT_NO_THROW(LoadDataBaseFromStr(kXODRMapWithMultipleConnections,
                                      {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors}));
}

// Loads TShapeRoad.xodr and verifies the shortest and largest Geometry in the XODR.
GTEST_TEST(DBManager, GetGeometriesLengthInformation) {
  const std::string kXodrFile = "TShapeRoad.xodr";
  const auto db_manager = LoadDataBaseFromFile(utility::FindResourceInPath(kXodrFile, kMalidriveResourceFolder),
                                               {constants::kLinearTolerance});
  const DBManager::XodrGeometryLengthData kExpectedShortest{RoadHeader::Id("6"), 3, 6.8474312421988870e-2};
  const DBManager::XodrGeometryLengthData kExpectedLargest{RoadHeader::Id("0"), 0, 46.};
  const auto shortest_geometry{db_manager->GetShortestGeometry()};
  const auto largest_geometry{db_manager->GetLargestGeometry()};
  EXPECT_EQ(kExpectedShortest.road_header_id, shortest_geometry.road_header_id);
  EXPECT_EQ(kExpectedShortest.geometry_index, shortest_geometry.geometry_index);
  EXPECT_EQ(kExpectedShortest.length, shortest_geometry.length);
  EXPECT_EQ(kExpectedLargest.road_header_id, largest_geometry.road_header_id);
  EXPECT_EQ(kExpectedLargest.geometry_index, largest_geometry.geometry_index);
  EXPECT_EQ(kExpectedLargest.length, largest_geometry.length);
}

// Loads Highway.xodr and verifies the shortest and largest LaneSection in the XODR.
GTEST_TEST(DBManager, GetLaneSectionLengthInformation) {
  const double kTolerance{1e-14};
  const std::string kXodrFile = "Highway.xodr";
  const auto db_manager = LoadDataBaseFromFile(utility::FindResourceInPath(kXodrFile, kMalidriveResourceFolder),
                                               {constants::kLinearTolerance});
  const DBManager::XodrLaneSectionLengthData kExpectedShortest{RoadHeader::Id("4"), 0, 1.1649945427797022};
  const DBManager::XodrLaneSectionLengthData kExpectedLargest{RoadHeader::Id("11"), 0, 2.9161454554383585e+2};
  const auto shortest_lane_section{db_manager->GetShortestLaneSection()};
  const auto largest_lane_section{db_manager->GetLargestLaneSection()};
  EXPECT_EQ(kExpectedShortest.road_header_id, shortest_lane_section.road_header_id);
  EXPECT_EQ(kExpectedShortest.lane_section_index, shortest_lane_section.lane_section_index);
  EXPECT_NEAR(kExpectedShortest.length, shortest_lane_section.length, kTolerance);
  EXPECT_EQ(kExpectedLargest.road_header_id, largest_lane_section.road_header_id);
  EXPECT_EQ(kExpectedLargest.lane_section_index, largest_lane_section.lane_section_index);
  EXPECT_NEAR(kExpectedLargest.length, largest_lane_section.length, kTolerance);
}

// Loads TShapeRoad.xodr and verifies the shortest and largest GAP between Geometries in the XODR.
GTEST_TEST(DBManager, GetGeometriesGapInformation) {
  const double kTolerance{1e-14};
  const std::string kXodrFile = "TShapeRoad.xodr";
  const auto db_manager = LoadDataBaseFromFile(utility::FindResourceInPath(kXodrFile, kMalidriveResourceFolder),
                                               {constants::kLinearTolerance});
  const DBManager::XodrGapBetweenGeometries kExpectedShortest{RoadHeader::Id("6"), {2, 3}, 0.};
  const DBManager::XodrGapBetweenGeometries kExpectedLargest{RoadHeader::Id("6"), {1, 2}, 1.77636e-15};
  const auto shortest_geometry{db_manager->GetShortestGap()};
  const auto largest_geometry{db_manager->GetLargestGap()};
  EXPECT_EQ(kExpectedShortest.road_header_id, shortest_geometry.road_header_id);
  EXPECT_EQ(kExpectedShortest.geometry_index_pair, shortest_geometry.geometry_index_pair);
  EXPECT_NEAR(kExpectedShortest.distance, shortest_geometry.distance, kTolerance);
  EXPECT_EQ(kExpectedLargest.road_header_id, largest_geometry.road_header_id);
  EXPECT_EQ(kExpectedLargest.geometry_index_pair, largest_geometry.geometry_index_pair);
  EXPECT_NEAR(kExpectedLargest.distance, largest_geometry.distance, kTolerance);
}

// @{ Sample maps that have roads of different kinds to evaluate the road geometry simplification.
class DBManagerGetGeometrySimplificationTests : public ::testing::Test {
 protected:
  const std::optional<double> kLoaderNoToleranceCheck{};
  const double kTolerance{1.e-6};
};

TEST_F(DBManagerGetGeometrySimplificationTests, ThrowsWhenNegativeTolerance) {
  const auto dut = LoadDataBaseFromStr(malidrive::test::kXodrSingleGeometry,
                                       {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut->GetGeometriesToSimplify(-1.), maliput::common::assertion_error);
}

TEST_F(DBManagerGetGeometrySimplificationTests, NoSimplificationPossible) {
  {  // Single geometry in road.
    const auto dut = LoadDataBaseFromStr(malidrive::test::kXodrSingleGeometry,
                                         {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
    const std::vector<DBManager::XodrGeometriesToSimplify> result = dut->GetGeometriesToSimplify(kTolerance);
    ASSERT_TRUE(result.empty());
  }
  {  // Two geometries of different kinds.
    const auto dut = LoadDataBaseFromStr(malidrive::test::kXodrLineAndArcGeometry,
                                         {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
    const std::vector<DBManager::XodrGeometriesToSimplify> result = dut->GetGeometriesToSimplify(kTolerance);
    ASSERT_TRUE(result.empty());
  }
}

TEST_F(DBManagerGetGeometrySimplificationTests, SimplifiesLines) {
  const auto dut = LoadDataBaseFromStr(malidrive::test::kXodrWithLinesToBeSimplified,
                                       {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  const std::vector<DBManager::XodrGeometriesToSimplify> result = dut->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, result.size());
  ASSERT_EQ(RoadHeader::Id("1"), result[0].road_header_id);
  ASSERT_EQ(DBManager::XodrGeometriesToSimplify::Action::kReplaceByLine, result[0].action);
  ASSERT_EQ(3, result[0].geometries.size());
  EXPECT_EQ(0, result[0].geometries[0]);
  EXPECT_EQ(1, result[0].geometries[1]);
  EXPECT_EQ(2, result[0].geometries[2]);
}

TEST_F(DBManagerGetGeometrySimplificationTests, SimplifiesArcs) {
  const auto dut = LoadDataBaseFromStr(malidrive::test::kXodrWithArcsToBeSimplified,
                                       {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});

  const std::vector<DBManager::XodrGeometriesToSimplify> result = dut->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, result.size());
  ASSERT_EQ(RoadHeader::Id("1"), result[0].road_header_id);
  ASSERT_EQ(DBManager::XodrGeometriesToSimplify::Action::kReplaceByArc, result[0].action);
  ASSERT_EQ(3, result[0].geometries.size());
  EXPECT_EQ(0, result[0].geometries[0]);
  EXPECT_EQ(1, result[0].geometries[1]);
  EXPECT_EQ(2, result[0].geometries[2]);
}

TEST_F(DBManagerGetGeometrySimplificationTests, SimplifiesLinesBetweenArcs) {
  const auto dut = LoadDataBaseFromStr(malidrive::test::kXodrCombinedLinesWithArcs,
                                       {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});

  const std::vector<DBManager::XodrGeometriesToSimplify> result = dut->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, result.size());
  ASSERT_EQ(RoadHeader::Id("1"), result[0].road_header_id);
  ASSERT_EQ(DBManager::XodrGeometriesToSimplify::Action::kReplaceByLine, result[0].action);
  ASSERT_EQ(2, result[0].geometries.size());
  EXPECT_EQ(1, result[0].geometries[0]);
  EXPECT_EQ(2, result[0].geometries[1]);
}

TEST_F(DBManagerGetGeometrySimplificationTests, SimplifiesArcsBetweenLines) {
  const auto dut = LoadDataBaseFromStr(malidrive::test::kXodrCombinedArcsWithLines,
                                       {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});

  const std::vector<DBManager::XodrGeometriesToSimplify> result = dut->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, result.size());
  ASSERT_EQ(RoadHeader::Id("1"), result[0].road_header_id);
  ASSERT_EQ(DBManager::XodrGeometriesToSimplify::Action::kReplaceByArc, result[0].action);
  ASSERT_EQ(2, result[0].geometries.size());
  EXPECT_EQ(1, result[0].geometries[0]);
  EXPECT_EQ(2, result[0].geometries[1]);
}
// @}

// @{ Xodr maps and tests to evaluate gaps in elevation function.
constexpr const char* kFlatRoads = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='FlatRoads' version='1.0' date='Tue Oct 13 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="Road 1" length="100.0" id="1" junction="-1">
      <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0e+0" length="100.0">
                <line/>
            </geometry>
        </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
  <road name="Road 2" length="100.0" id="2" junction="-1">
      <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="10.0" hdg="0.0e+0" length="100.0">
                <line/>
            </geometry>
        </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

constexpr const char* kElevatedRoads = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='ElevatedRoads' version='1.0' date='Tue Oct 13 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="Road 1" length="100.0" id="1" junction="-1">
      <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0e+0" length="100.0">
                <line/>
            </geometry>
        </planView>
        <elevationProfile>
            <elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
            <elevation s="20.0" a="1.0" b="0.0" c="0.0" d="0.0"/>
            <elevation s="40.0" a="2.5" b="0.0" c="0.0" d="0.0"/>
        </elevationProfile>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
  <road name="Road 2" length="100.0" id="2" junction="-1">
      <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0e+0" length="100.0">
                <line/>
            </geometry>
        </planView>
        <elevationProfile>
            <elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
            <elevation s="50.0" a="0.123" b="0.0" c="0.0" d="0.0"/>
        </elevationProfile>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

class DBManagerGetElevationGaps : public ::testing::Test {
 protected:
  const std::optional<double> kLoaderNoToleranceCheck{};
  const double kTolerance{1.e-6};
};

TEST_F(DBManagerGetElevationGaps, EmptyResult) {
  const auto dut =
      LoadDataBaseFromStr(kFlatRoads, {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});

  const DBManager::XodrGapBetweenFunctions& shortest_elevation_gap = dut->GetShortestElevationGap();
  EXPECT_EQ(RoadHeader::Id("none"), shortest_elevation_gap.road_header_id);

  const DBManager::XodrGapBetweenFunctions& largest_elevation_gap = dut->GetLargestElevationGap();
  EXPECT_EQ(RoadHeader::Id("none"), largest_elevation_gap.road_header_id);
}

TEST_F(DBManagerGetElevationGaps, EvaluateLargestAndShortestGapInElevation) {
  const auto dut =
      LoadDataBaseFromStr(kElevatedRoads, {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});

  const DBManager::XodrGapBetweenFunctions& shortest_elevation_gap = dut->GetShortestElevationGap();
  EXPECT_EQ(RoadHeader::Id("2"), shortest_elevation_gap.road_header_id);
  EXPECT_EQ(0, shortest_elevation_gap.function_index_pair.first);
  EXPECT_EQ(1, shortest_elevation_gap.function_index_pair.second);
  EXPECT_NEAR(0.123, shortest_elevation_gap.distance, kTolerance);

  const DBManager::XodrGapBetweenFunctions& largest_elevation_gap = dut->GetLargestElevationGap();
  EXPECT_EQ(RoadHeader::Id("1"), largest_elevation_gap.road_header_id);
  EXPECT_EQ(1, largest_elevation_gap.function_index_pair.first);
  EXPECT_EQ(2, largest_elevation_gap.function_index_pair.second);
  EXPECT_NEAR(1.5, largest_elevation_gap.distance, kTolerance);
}
// @}

// @{ Xodr map and tests to evaluate gaps in superelevation function.
constexpr const char* kSuperelevatedRoads = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='ElevatedRoads' version='1.0' date='Tue Oct 13 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="Road 1" length="100.0" id="1" junction="-1">
      <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0e+0" length="100.0">
                <line/>
            </geometry>
        </planView>
        <lateralProfile>
            <superelevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
            <superelevation s="20.0" a="1.0" b="0.0" c="0.0" d="0.0"/>
            <superelevation s="40.0" a="2.5" b="0.0" c="0.0" d="0.0"/>
        </lateralProfile>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
  <road name="Road 2" length="100.0" id="2" junction="-1">
      <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0e+0" length="100.0">
                <line/>
            </geometry>
        </planView>
        <lateralProfile>
            <superelevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
            <superelevation s="50.0" a="0.123" b="0.0" c="0.0" d="0.0"/>
        </lateralProfile>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

class DBManagerGetSuperelevationGaps : public ::testing::Test {
 protected:
  const std::optional<double> kLoaderNoToleranceCheck{};
  const double kTolerance{1.e-6};
};

TEST_F(DBManagerGetSuperelevationGaps, EmptyResult) {
  const auto dut =
      LoadDataBaseFromStr(kFlatRoads, {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});

  const DBManager::XodrGapBetweenFunctions& shortest_superelevation_gap = dut->GetShortestSuperelevationGap();
  EXPECT_EQ(RoadHeader::Id("none"), shortest_superelevation_gap.road_header_id);

  const DBManager::XodrGapBetweenFunctions& largest_superelevation_gap = dut->GetLargestSuperelevationGap();
  EXPECT_EQ(RoadHeader::Id("none"), largest_superelevation_gap.road_header_id);
}

TEST_F(DBManagerGetSuperelevationGaps, EvaluateLargestAndShortestGapInSuperelevation) {
  const auto dut = LoadDataBaseFromStr(kSuperelevatedRoads,
                                       {kLoaderNoToleranceCheck, kDontAllowSchemaErrors, kDontAllowSemanticErrors});

  const DBManager::XodrGapBetweenFunctions& shortest_superelevation_gap = dut->GetShortestSuperelevationGap();
  EXPECT_EQ(RoadHeader::Id("2"), shortest_superelevation_gap.road_header_id);
  EXPECT_EQ(0, shortest_superelevation_gap.function_index_pair.first);
  EXPECT_EQ(1, shortest_superelevation_gap.function_index_pair.second);
  EXPECT_NEAR(0.123, shortest_superelevation_gap.distance, kTolerance);

  const DBManager::XodrGapBetweenFunctions& largest_superelevation_gap = dut->GetLargestSuperelevationGap();
  EXPECT_EQ(RoadHeader::Id("1"), largest_superelevation_gap.road_header_id);
  EXPECT_EQ(1, largest_superelevation_gap.function_index_pair.first);
  EXPECT_EQ(2, largest_superelevation_gap.function_index_pair.second);
  EXPECT_NEAR(1.5, largest_superelevation_gap.distance, kTolerance);
}
// @}

// @{ Xodr map and tests to evaluate gaps in superelevation function.
constexpr const char* kNoReciprocalLinkage = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='ElevatedRoads' version='1.0' date='Tue Oct 13 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="Road 1" length="100.0" id="1" junction="-1">
      <link>
          <successor elementType="road" elementId="1" contactPoint="start"/>
      </link>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0e+0" length="100.0">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <link>
                          <successor id="-1"/>
                      </link>
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
  <road name="Road 2" length="100.0" id="2" junction="-1">
      <link>
      </link>
      <planView>
          <geometry s="0.0" x="100.0" y="0.0" hdg="0.0e+0" length="100.0">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

class DBManagerNoReciprocalRoadLinkage : public ::testing::Test {
 protected:
  const std::optional<double> kLoaderNoToleranceCheck{};
};

TEST_F(DBManagerNoReciprocalRoadLinkage, NotAllowingSemanticErrors) {
  const bool allow_schema_errors{false};
  const bool allow_semantic_errors{false};
  EXPECT_THROW(
      LoadDataBaseFromStr(kNoReciprocalLinkage, {kLoaderNoToleranceCheck, allow_schema_errors, allow_semantic_errors}),
      maliput::common::assertion_error);
}

TEST_F(DBManagerNoReciprocalRoadLinkage, AllowingSemanticErrors) {
  const bool allow_schema_errors{false};
  const bool allow_semantic_errors{true};
  EXPECT_THROW(
      LoadDataBaseFromStr(kNoReciprocalLinkage, {kLoaderNoToleranceCheck, allow_schema_errors, allow_semantic_errors}),
      maliput::common::assertion_error);
}
// @}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
