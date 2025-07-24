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
#include "maliput_malidrive/xodr/parser.h"

#include <algorithm>
#include <sstream>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/connection.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/geo_reference.h"
#include "maliput_malidrive/xodr/geometry.h"
#include "maliput_malidrive/xodr/header.h"
#include "maliput_malidrive/xodr/junction.h"
#include "maliput_malidrive/xodr/lane.h"
#include "maliput_malidrive/xodr/lane_link.h"
#include "maliput_malidrive/xodr/lane_offset.h"
#include "maliput_malidrive/xodr/lane_section.h"
#include "maliput_malidrive/xodr/lane_width.h"
#include "maliput_malidrive/xodr/lanes.h"
#include "maliput_malidrive/xodr/lateral_profile.h"
#include "maliput_malidrive/xodr/offset.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/road_link.h"
#include "maliput_malidrive/xodr/road_type.h"
#include "utility/resources.h"

namespace malidrive {
namespace xodr {
namespace test {
namespace {

class ParsingTests : public ::testing::Test {
 protected:
  // Flag to not allow schema errors.
  static constexpr bool kDontAllowSchemaErrors{false};
  // Flag to not allow semantic errors.
  static constexpr bool kDontAllowSemanticErrors{false};
  // Flag to allow schema errors.
  static constexpr bool kAllowSchemaErrors{true};

  tinyxml2::XMLElement* LoadXMLAndGetNodeByName(const std::string& xml_str, const std::string& node_name) {
    MALIDRIVE_THROW_UNLESS(xml_doc_.Parse(xml_str.c_str()) == tinyxml2::XML_SUCCESS);
    tinyxml2::XMLElement* p_xml = xml_doc_.FirstChildElement();
    MALIDRIVE_THROW_UNLESS(p_xml != nullptr);
    tinyxml2::XMLElement* p_elem = p_xml->FirstChildElement(node_name.c_str());
    MALIDRIVE_THROW_UNLESS(p_elem != nullptr);
    return p_elem;
  }
  const std::optional<double> kNullParserSTolerance{std::nullopt};  // Disables the check because it is not needed.
  const std::optional<double> kStrictParserSTolerance{malidrive::constants::kStrictLinearTolerance};
  tinyxml2::XMLDocument xml_doc_;
};

// Basic XML node templatized.
// @param node_name: The name of the node.
// @param attribute_name_1: The name of the first attribute.
// @param attribute_value_1: The value of the first attribute.
// @param attribute_name_2: The name of the second attribute.
// @param attribute_value_2: The value of the second attribute.
// @returns A string that contains a XML node with two attributes.
std::string GetBasicXMLNode(const std::string& node_name, const std::string& attribute_name_1,
                            const std::string& attribute_value_1, const std::string& attribute_name_2,
                            const std::string& attribute_value_2) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<" << node_name << " " << attribute_name_1 << "='" << attribute_value_1 << "' " << attribute_name_2 << "='"
     << attribute_value_2 << "' >";
  ss << "</" << node_name << ">";
  ss << "</root>";
  return ss.str();
}

// Tests `NumberOfAttributes` method.
TEST_F(ParsingTests, NumberOfAttributes) {
  constexpr const char* kNode = "node";
  constexpr const char* kValue1 = "value1";
  constexpr const char* kValue2 = "value2";
  const int kNumberOfAttributes = 2;
  const double kExpectedValues[2]{1.57, 35.6};
  const std::string xml_description =
      GetBasicXMLNode(kNode, kValue1, std::to_string(kExpectedValues[0]), kValue2, std::to_string(kExpectedValues[1]));
  const ParserBase dut(LoadXMLAndGetNodeByName(xml_description, kNode),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(kNumberOfAttributes, dut.NumberOfAttributes());
}

// Tests `double` parsing.
TEST_F(ParsingTests, AttributeParserDouble) {
  constexpr const char* kNode = "node";
  constexpr const char* kValue1 = "value1";
  constexpr const char* kValue2 = "value2";
  const double kExpectedValues[2]{1.57, 35.6};
  std::string xml_description =
      GetBasicXMLNode(kNode, kValue1, std::to_string(kExpectedValues[0]), kValue2, std::to_string(kExpectedValues[1]));

  const AttributeParser dut(LoadXMLAndGetNodeByName(xml_description, kNode),
                            {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(kNode, dut.GetName());
  const std::optional<double> version_value_1 = dut.As<double>(kValue1);
  EXPECT_TRUE(version_value_1.has_value());
  EXPECT_EQ(kExpectedValues[0], version_value_1.value());

  const std::optional<double> version_value_2 = dut.As<double>(kValue2);
  EXPECT_TRUE(version_value_2.has_value());
  EXPECT_EQ(kExpectedValues[1], version_value_2.value());

  const char* kMissingAttribute = "MissingAttribute";
  const std::optional<double> missing_value = dut.As<double>(kMissingAttribute);
  EXPECT_FALSE(missing_value.has_value());

  const double kNanValue{NAN};

  xml_description = GetBasicXMLNode(kNode, kValue1, std::to_string(kNanValue), kValue2, std::to_string(kNanValue));
  const AttributeParser dut2(LoadXMLAndGetNodeByName(xml_description, kNode),
                             {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut2.As<double>(kValue1), maliput::common::assertion_error);
  EXPECT_THROW(dut2.As<double>(kValue2), maliput::common::assertion_error);
}

// Tests `string` parsing.
TEST_F(ParsingTests, AttributeParserString) {
  constexpr const char* kNode = "node";
  constexpr const char* kValue1 = "value1";
  constexpr const char* kValue2 = "value2";
  const std::string kExpectedValues[2]{"string_1", "string_2"};
  const std::string xml_description = GetBasicXMLNode(kNode, kValue1, kExpectedValues[0], kValue2, kExpectedValues[1]);

  const AttributeParser dut(LoadXMLAndGetNodeByName(xml_description, kNode),
                            {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(kNode, dut.GetName());
  const std::optional<std::string> version_value_1 = dut.As<std::string>(kValue1);
  EXPECT_TRUE(version_value_1.has_value());
  EXPECT_EQ(kExpectedValues[0], version_value_1.value());

  const std::optional<std::string> version_value_2 = dut.As<std::string>(kValue2);
  EXPECT_TRUE(version_value_2.has_value());
  EXPECT_EQ(kExpectedValues[1], version_value_2.value());

  const char* kMissingAttribute = "MissingAttribute";
  const std::optional<std::string> missing_value = dut.As<std::string>(kMissingAttribute);
  EXPECT_FALSE(missing_value.has_value());
}

// Basic XML node templatized for 5 attributes.
// @param node_name: The name of the node.
// @param attribute_name_1: The name of the first attribute.
// @param attribute_value_1: The value of the first attribute.
// @param attribute_name_2: The name of the second attribute.
// @param attribute_value_2: The value of the second attribute.
// @param attribute_name_3: The name of the third attribute.
// @param attribute_value_3: The value of the third attribute.
// @param attribute_name_4: The name of the fourth attribute.
// @param attribute_value_4: The value of the fourth attribute.
// @param attribute_name_5: The name of the fifth attribute.
// @param attribute_value_5: The value of the fifth attribute.
// @returns A string that contains a XML node with five attributes.
std::string GetBasicXMLNode5Attributes(const std::string& node_name, const std::string& attribute_name_1,
                                       const std::string& attribute_value_1, const std::string& attribute_name_2,
                                       const std::string& attribute_value_2, const std::string& attribute_name_3,
                                       const std::string& attribute_value_3, const std::string& attribute_name_4,
                                       const std::string& attribute_value_4, const std::string& attribute_name_5,
                                       const std::string& attribute_value_5) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<" << node_name << " " << attribute_name_1 << "='" << attribute_value_1 << "' " << attribute_name_2 << "='"
     << attribute_value_2 << "' " << attribute_name_3 << "='" << attribute_value_3 << "' " << attribute_name_4 << "='"
     << attribute_value_4 << "' " << attribute_name_5 << "='" << attribute_value_5 << "' >";
  ss << "</" << node_name << ">";
  ss << "</root>";
  return ss.str();
}

// Tests `bool` parsing.
TEST_F(ParsingTests, AttributeParserBool) {
  constexpr const char* kNode = "node";
  constexpr const char* kValue[5] = {"value1", "value2", "value3", "value4", "value5"};
  const std::string kStringValues[5]{"true", "false", "1", "0", "wrong_value"};
  const bool kExpectedValues[4]{true, false, true, false};
  const std::string xml_description =
      GetBasicXMLNode5Attributes(kNode, kValue[0], kStringValues[0], kValue[1], kStringValues[1], kValue[2],
                                 kStringValues[2], kValue[3], kStringValues[3], kValue[4], kStringValues[4]);

  const AttributeParser dut(LoadXMLAndGetNodeByName(xml_description, kNode),
                            {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(kNode, dut.GetName());
  for (int i = 0; i < 4; i++) {
    const std::optional<bool> parsed_value = dut.As<bool>(kValue[i]);
    EXPECT_TRUE(parsed_value.has_value());
    EXPECT_EQ(kExpectedValues[i], parsed_value.value());
  }
  EXPECT_THROW(dut.As<bool>(kValue[4]), maliput::common::assertion_error);

  const char* kMissingAttribute = "MissingAttribute";
  const std::optional<bool> missing_value = dut.As<bool>(kMissingAttribute);
  EXPECT_FALSE(missing_value.has_value());
}

// Tests `RoadHeader::HandTrafficRule` parsing.
TEST_F(ParsingTests, AttributeParserHandTrafficRule) {
  constexpr const char* kNode = "node";
  constexpr const char* kValue1 = "rule1";
  constexpr const char* kValue2 = "rule2";
  const std::string kExpectedValues[2]{"LHT", "NotValid"};
  const std::string xml_description = GetBasicXMLNode(kNode, kValue1, kExpectedValues[0], kValue2, kExpectedValues[1]);

  const AttributeParser dut(LoadXMLAndGetNodeByName(xml_description, kNode),
                            {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(kNode, dut.GetName());
  const std::optional<RoadHeader::HandTrafficRule> optional_hand_traffic_rule =
      dut.As<RoadHeader::HandTrafficRule>(kValue1);
  EXPECT_TRUE(optional_hand_traffic_rule.has_value());
  EXPECT_EQ(kExpectedValues[0], RoadHeader::hand_traffic_rule_to_str(optional_hand_traffic_rule.value()));

  EXPECT_THROW(dut.As<RoadHeader::HandTrafficRule>(kValue2), maliput::common::assertion_error);

  const char* kMissingAttribute = "MissingAttribute";
  const std::optional<RoadHeader::HandTrafficRule> missing_value =
      dut.As<RoadHeader::HandTrafficRule>(kMissingAttribute);
  EXPECT_FALSE(missing_value.has_value());
}

// Get a XML description of a XODR header.
// @param rev_major: The major revision number.
// @param rev_minor: The minor revision number.
// @param name: The name of the header.
// @param version: The version of the header.
// @param date: The date of the header.
// @param north: The north value of the header.
// @param south: The south value of the header.
// @param east: The east value of the header.
// @param west: The west value of the header.
// @param vendor: The vendor of the header.
// @returns A string that contains a XML description of a XODR header.
std::string GetXODRHeader(double rev_major, double rev_minor, const std::string& name, double version,
                          const std::string& date, double north, double south, double east, double west,
                          const std::string& vendor) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<header revMajor='" << rev_major << "' revMinor='" << rev_minor << "' name='" << name << "' version='"
     << version << "' date='" << date << "' north='" << north << "' south='" << south << "' east='" << east
     << "' west='" << west << "' vendor='" << vendor << "' >";
  ss << "</header>";
  ss << "</root>";
  return ss.str();
}

// Tests `Header` parsing.
TEST_F(ParsingTests, NodeParserHeader) {
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
  const std::string xml_description =
      GetXODRHeader(kExpectedHeader.rev_major, kExpectedHeader.rev_minor, kExpectedHeader.name.value(),
                    kExpectedHeader.version.value(), kExpectedHeader.date.value(), kExpectedHeader.north.value(),
                    kExpectedHeader.south.value(), kExpectedHeader.east.value(), kExpectedHeader.west.value(),
                    kExpectedHeader.vendor.value());

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Header::kHeaderTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Header::kHeaderTag, dut.GetName());
  EXPECT_EQ(kExpectedHeader, dut.As<Header>());
}

/// Get XODR GeoReference
std::string GetXODRGeoReference() {
  std::stringstream ss;
  ss << "<root>";
  ss << "<geoReference>";
  ss << "<![CDATA[+proj=utm +zone=32 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs]]>";
  ss << "</geoReference>";
  ss << "</root>";
  return ss.str();
}

// Tests `GeoReference` parsing.
TEST_F(ParsingTests, NodeParserGeoReference) {
  const GeoReference kExpectedGeoRef{
      "+proj=utm +zone=32 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs" /* projection_data */};
  const std::string xml_description = GetXODRGeoReference();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, GeoReference::kGeoReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(GeoReference::kGeoReferenceTag, dut.GetName());
  EXPECT_EQ(kExpectedGeoRef, dut.As<GeoReference>());
}

/// Get XODR Offset
std::string GetXODROffset(double x, double y, double z, double hdg) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<offset x='" << x << "' y='" << y << "' z='" << z << "' hdg='" << hdg << "'>";
  ss << "</offset>";
  ss << "</root>";
  return ss.str();
}

// Tests `GeoReference` parsing.
TEST_F(ParsingTests, NodeParserOffset) {
  const Offset kExpectedOffset{
      10 /* x */,
      20 /* y */,
      5 /* z */,
      3.14 /* hdg */,
  };
  const std::string xml_description =
      GetXODROffset(kExpectedOffset.x, kExpectedOffset.y, kExpectedOffset.z, kExpectedOffset.hdg);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Offset::kOffsetTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Offset::kOffsetTag, dut.GetName());
  EXPECT_EQ(kExpectedOffset, dut.As<Offset>());
}

// Get a XML description of a XODR RoadLink.
// @param pred_element_type: The predecessor element type.
// @param pred_element_id: The predecessor element id.
// @param pred_contact_point: The predecessor contact point.
// @param succ_element_type: The successor element type.
// @param succ_element_id: The successor element id.
// @param succ_contact_point: The successor contact point.
// @returns A string that contains a XML description of a XODR RoadLink.
std::string GetRoadLink(const std::string& pred_element_type, const std::string& pred_element_id,
                        const std::string& pred_contact_point, const std::string& succ_element_type,
                        const std::string& succ_element_id, const std::string& succ_contact_point) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<link>";
  ss << "<predecessor elementType='" << pred_element_type << "' elementId='" << pred_element_id << "' contactPoint='"
     << pred_contact_point << "' />";
  ss << "<successor elementType='" << succ_element_type << "' elementId='" << succ_element_id << "' contactPoint='"
     << succ_contact_point << "' />";
  ss << "</link>";
  ss << "</root>";
  return ss.str();
}
// Tests `RoadLink` parsing.
TEST_F(ParsingTests, NodeParserRoadLink) {
  const RoadLink::LinkAttributes kPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                              RoadLink::LinkAttributes::Id("50") /* elementId*/,
                                              RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                            RoadLink::LinkAttributes::Id("80") /* elementId*/,
                                            RoadLink::ContactPoint::kStart /* contactPoint*/};
  const RoadLink kExpectedRoadLink{kPredecessor, kSuccessor};
  const std::string xml_description =
      GetRoadLink(RoadLink::element_type_to_str(kExpectedRoadLink.predecessor->element_type),
                  kExpectedRoadLink.predecessor->element_id.string(),
                  RoadLink::contact_point_to_str(*kExpectedRoadLink.predecessor->contact_point),
                  RoadLink::element_type_to_str(kExpectedRoadLink.successor->element_type),
                  kExpectedRoadLink.successor->element_id.string(),
                  RoadLink::contact_point_to_str(*kExpectedRoadLink.successor->contact_point));

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, RoadLink::kRoadLinkTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(RoadLink::kRoadLinkTag, dut.GetName());
  const RoadLink road_link = dut.As<RoadLink>();
  EXPECT_EQ(kExpectedRoadLink, road_link);
}

// Get a XML description of a XODR RoadSpeed
// @param max The max speed.
// @param unit The unit of the speed.
// @returns A string that contains a XML description of a XODR RoadSpeed.
std::string GetRoadSpeed(double max, const std::string& unit) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<speed max='" << max << "' unit='" << unit << "' />";
  ss << "</root>";
  return ss.str();
}

// Tests `RoadType::Speed` parsing.
TEST_F(ParsingTests, NodeParserRoadSpeed) {
  const RoadType::Speed kExpectedSpeed{45. /* max */, Unit::kMph /* unit */};

  const std::string xml_description = GetRoadSpeed(kExpectedSpeed.max.value(), unit_to_str(kExpectedSpeed.unit));

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, RoadType::Speed::kSpeedTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(RoadType::Speed::kSpeedTag, dut.GetName());
  const RoadType::Speed speed = dut.As<RoadType::Speed>();
  EXPECT_EQ(kExpectedSpeed, speed);
}

// Get a XML description that contains a XODR RoadType.
// @param s The s value.
// @param type The type value.
// @param country The country value.
// @returns A string that contains a XML description of a XODR RoadType.
std::string GetRoadType(double s, const std::string& type, const std::string& country) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<type s='" << s << "' type='" << type << "' country='" << country << "'>";
  ss << "<speed max='45.' unit='mph'/>";
  ss << "</type>";
  ss << "</root>";
  return ss.str();
}
// Tests `RoadType` parsing.
TEST_F(ParsingTests, NodeParserRoadType) {
  const RoadType::Speed kSpeed{45. /* max */, Unit::kMph /* unit */};
  const RoadType kExpectedRoadType{1.2 /* s0 */, RoadType::Type::kPedestrian /* type */, "+54" /* country */,
                                   kSpeed /* speed */};
  const std::string xml_description = GetRoadType(kExpectedRoadType.s_0, RoadType::type_to_str(kExpectedRoadType.type),
                                                  kExpectedRoadType.country.value());

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, RoadType::kRoadTypeTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(RoadType::kRoadTypeTag, dut.GetName());
  const RoadType road_type = dut.As<RoadType>();
  EXPECT_EQ(kExpectedRoadType, road_type);
}

// Get a XML description that contains a XODR Geometry.
// @param s_0 The s_0 value.
// @param x The x value.
// @param y The y value.
// @param orientation The orientation value.
// @param length The length value.
// @param type The type value.
std::string GetGeometry(double s_0, double x, double y, double orientation, double length, const std::string& type) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<geometry s='" << s_0 << "' x='" << x << "' y='" << y << "' hdg='" << orientation << "' length='" << length
     << "'>";
  ss << "<" << type << "/>";
  ss << "</geometry>";
  ss << "</root>";
  return ss.str();
}

// Tests `Geometry` parsing.
TEST_F(ParsingTests, NodeParserLineGeometry) {
  const Geometry kExpectedGeometry{
      1.23 /* s_0 */,    {523.2 /* x */, 83.27 /* y */},   0.77 /* orientation */,
      100. /* length */, Geometry::Type::kLine /* Type */, {Geometry::Line{}} /* description */};
  const std::string xml_description = GetGeometry(
      kExpectedGeometry.s_0, kExpectedGeometry.start_point.x(), kExpectedGeometry.start_point.y(),
      kExpectedGeometry.orientation, kExpectedGeometry.length, Geometry::type_to_str(kExpectedGeometry.type));

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Geometry::kGeometryTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Geometry::kGeometryTag, dut.GetName());
  const Geometry geometry = dut.As<Geometry>();
  EXPECT_EQ(kExpectedGeometry, geometry);
}

// Tests `Geometry` parsing.
TEST_F(ParsingTests, NodeParserArcGeometry) {
  const Geometry kExpectedGeometry{
      1.23 /* s_0 */,    {523.2 /* x */, 83.27 /* y */},  0.77 /* orientation */,
      100. /* length */, Geometry::Type::kArc /* Type */, Geometry::Arc{0.5} /* description */};
  const std::string geometry_description =
      Geometry::type_to_str(kExpectedGeometry.type) + " " + Geometry::Arc::kCurvature + "='" +
      std::to_string(std::get<Geometry::Arc>(kExpectedGeometry.description).curvature) + "'";
  const std::string xml_description =
      GetGeometry(kExpectedGeometry.s_0, kExpectedGeometry.start_point.x(), kExpectedGeometry.start_point.y(),
                  kExpectedGeometry.orientation, kExpectedGeometry.length, geometry_description);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Geometry::kGeometryTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Geometry::kGeometryTag, dut.GetName());
  const Geometry geometry = dut.As<Geometry>();
  EXPECT_EQ(kExpectedGeometry, geometry);
}

// Tests `Geometry` parsing.
TEST_F(ParsingTests, NodeParserSpiralGeometry) {
  const Geometry kExpectedGeometry{
      1.23 /* s_0 */,    {523.2 /* x */, 83.27 /* y */},     0.77 /* orientation */,
      100. /* length */, Geometry::Type::kSpiral /* Type */, Geometry::Spiral{0.5, -0.25} /* description */};
  const std::string geometry_description =
      Geometry::type_to_str(kExpectedGeometry.type) + " " + Geometry::Spiral::kCurvStart + "='" +
      std::to_string(std::get<Geometry::Spiral>(kExpectedGeometry.description).curv_start) + "' " +
      Geometry::Spiral::kCurvEnd + "='" +
      std::to_string(std::get<Geometry::Spiral>(kExpectedGeometry.description).curv_end) + "'";
  const std::string xml_description =
      GetGeometry(kExpectedGeometry.s_0, kExpectedGeometry.start_point.x(), kExpectedGeometry.start_point.y(),
                  kExpectedGeometry.orientation, kExpectedGeometry.length, geometry_description);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Geometry::kGeometryTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Geometry::kGeometryTag, dut.GetName());
  const Geometry geometry = dut.As<Geometry>();
  EXPECT_EQ(kExpectedGeometry, geometry);
}

// Get a XML description that contains a XODR PlanView.
// @param s_0 The s_0 value of first geometry.
// @param x_0 The x_0 value of first geometry.
// @param y_0 The y_0 value of first geometry.
// @param orientation_0 The orientation value value of first geometry.
// @param length_0 The length value of first geometry.
// @param type_0 The type value of first geometry.
// @param s_1 The s_1 value of second geometry.
// @param x_1 The x_1 value of second geometry.
// @param y_1 The y_1 value of second geometry.
// @param orientation_1 The orientation value of second geometry.
// @param length_1 The length value of second geometry.
// @param type_1 The type value of second geometry.
// @param arc_description The arc description.
// @param curvature The curvature value.
// @returns A string that contains a XML description of a XODR PlanView.
std::string GetPlainView(double s_0, double x_0, double y_0, double orientation_0, double length_0,
                         const std::string& type_0, double s_1, double x_1, double y_1, double orientation_1,
                         double length_1, const std::string& type_1, const std::string& arc_description,
                         double curvature) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<planView>";
  ss << "<geometry s='" << s_0 << "' x='" << x_0 << "' y='" << y_0 << "' hdg='" << orientation_0 << "' length='"
     << length_0 << "'>";
  ss << "<" << type_0 << "/>";
  ss << "</geometry>";
  ss << "<geometry s='" << s_1 << "' x='" << x_1 << "' y='" << y_1 << "' hdg='" << orientation_1 << "' length='"
     << length_1 << "'>";
  ss << "<" << type_1 << " " << arc_description << "='" << curvature << "'/>";
  ss << "</geometry>";
  ss << "</planView>";
  ss << "</root>";
  return ss.str();
}

// Tests `PlanView` parsing.
TEST_F(ParsingTests, NodeParserPlanView) {
  const PlanView kExpectedPlanView{std::vector<Geometry>{Geometry{1.23 /* s_0 */,
                                                                  {523.2 /* x */, 83.27 /* y */},
                                                                  0.77 /* orientation */,
                                                                  100. /* length */,
                                                                  Geometry::Type::kLine /* Type */,
                                                                  Geometry::Line{} /* description */},
                                                         Geometry{101.23 /* s_0 */,
                                                                  {546.4 /* x */, 166.54 /* y */},
                                                                  1.54 /* orientation */,
                                                                  200. /* length */,
                                                                  Geometry::Type::kArc /* Type */,
                                                                  Geometry::Arc{0.5} /* description */}}};
  const std::string xml_description = GetPlainView(
      kExpectedPlanView.geometries[0].s_0, kExpectedPlanView.geometries[0].start_point.x(),
      kExpectedPlanView.geometries[0].start_point.y(), kExpectedPlanView.geometries[0].orientation,
      kExpectedPlanView.geometries[0].length, Geometry::type_to_str(kExpectedPlanView.geometries[0].type),
      kExpectedPlanView.geometries[1].s_0, kExpectedPlanView.geometries[1].start_point.x(),
      kExpectedPlanView.geometries[1].start_point.y(), kExpectedPlanView.geometries[1].orientation,
      kExpectedPlanView.geometries[1].length, Geometry::type_to_str(kExpectedPlanView.geometries[1].type),
      Geometry::Arc::kCurvature, std::get<Geometry::Arc>(kExpectedPlanView.geometries[1].description).curvature);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, PlanView::kPlanViewTag),
                       {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(PlanView::kPlanViewTag, dut.GetName());
  const PlanView plan_view = dut.As<PlanView>();
  EXPECT_EQ(kExpectedPlanView, plan_view);
}

// Tests `PlanView` parsing when geometries aren't contiguous.
TEST_F(ParsingTests, NodeParserPlanViewNonContiguous) {
  const PlanView kExpectedPlanView{std::vector<Geometry>{Geometry{1.23 /* s_0 */,
                                                                  {523.2 /* x */, 83.27 /* y */},
                                                                  0.77 /* orientation */,
                                                                  100. /* length */,
                                                                  Geometry::Type::kLine /* Type */,
                                                                  Geometry::Line{} /* description */},
                                                         Geometry{120. /* s_0 */,
                                                                  {546.4 /* x */, 166.54 /* y */},
                                                                  1.54 /* orientation */,
                                                                  200. /* length */,
                                                                  Geometry::Type::kArc /* Type */,
                                                                  Geometry::Arc{0.5} /* description */}}};
  const std::string xml_description = GetPlainView(
      kExpectedPlanView.geometries[0].s_0, kExpectedPlanView.geometries[0].start_point.x(),
      kExpectedPlanView.geometries[0].start_point.y(), kExpectedPlanView.geometries[0].orientation,
      kExpectedPlanView.geometries[0].length, Geometry::type_to_str(kExpectedPlanView.geometries[0].type),
      kExpectedPlanView.geometries[1].s_0, kExpectedPlanView.geometries[1].start_point.x(),
      kExpectedPlanView.geometries[1].start_point.y(), kExpectedPlanView.geometries[1].orientation,
      kExpectedPlanView.geometries[1].length, Geometry::type_to_str(kExpectedPlanView.geometries[1].type),
      Geometry::Arc::kCurvature, std::get<Geometry::Arc>(kExpectedPlanView.geometries[1].description).curvature);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, PlanView::kPlanViewTag),
                       {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut.As<PlanView>(), maliput::common::assertion_error);
}

// Get a XML description that contains a XODR elevation.
// @param s The s value.
// @param a The a value.
// @param b The b value.
// @param c The c value.
// @param d The d value.
// @returns A string that contains a XML description of a XODR elevation.
std::string GetElevation(double s, double a, double b, double c, double d) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<elevation s='" << s << "' a='" << a << "' b='" << b << "' c='" << c << "' d='" << d << "' />";
  ss << "</root>";
  return ss.str();
}

// Tests `Elevation` parsing.
TEST_F(ParsingTests, NodeParserElevation) {
  const ElevationProfile::Elevation kExpectedElevation{1.1 /* s0 */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */,
                                                       5.5 /* d */};

  const std::string xml_description = GetElevation(kExpectedElevation.s_0, kExpectedElevation.a, kExpectedElevation.b,
                                                   kExpectedElevation.c, kExpectedElevation.d);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, ElevationProfile::Elevation::kElevationTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(ElevationProfile::Elevation::kElevationTag, dut.GetName());
  const ElevationProfile::Elevation elevation = dut.As<ElevationProfile::Elevation>();
  EXPECT_EQ(kExpectedElevation, elevation);
}

// Template of a XML description that contains a XODR elevationProfile.
constexpr const char* kElevationProfileTemplate = R"R(
<root>
  <elevationProfile>
    <elevation s='1.1' a='2.2' b='3.3' c='4.4' d='5.5'/>
    <elevation s='6.6' a='7.7' b='8.8' c='9.9' d='10.10'/>
    <elevation s='11.11' a='12.12' b='13.13' c='14.14' d='15.15'/>
    <elevation s='16.16' a='17.17' b='18.18' c='19.19' d='20.20'/>
    <elevation s='21.21' a='22.22' b='23.23' c='24.24' d='25.25'/>
  </elevationProfile>
</root>
)R";

// Tests `ElevationProfile` parsing.
TEST_F(ParsingTests, NodeParserElevationProfile) {
  const ElevationProfile::Elevation kElevation1{1.1, 2.2, 3.3, 4.4, 5.5};
  const ElevationProfile::Elevation kElevation2{6.6, 7.7, 8.8, 9.9, 10.10};
  const ElevationProfile::Elevation kElevation3{11.11, 12.12, 13.13, 14.14, 15.15};
  const ElevationProfile::Elevation kElevation4{16.16, 17.17, 18.18, 19.19, 20.20};
  const ElevationProfile::Elevation kElevation5{21.21, 22.22, 23.23, 24.24, 25.25};
  const ElevationProfile kExpectedElevationProfile{{kElevation1, kElevation2, kElevation3, kElevation4, kElevation5}};

  const NodeParser dut(LoadXMLAndGetNodeByName(kElevationProfileTemplate, ElevationProfile::kElevationProfileTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(ElevationProfile::kElevationProfileTag, dut.GetName());
  const ElevationProfile elevation_profile = dut.As<ElevationProfile>();
  EXPECT_EQ(kExpectedElevationProfile, elevation_profile);
}

// Get a XML description that contains a XODR superelevation.
// @param s The s value.
// @param a The a value.
// @param b The b value.
// @param c The c value.
// @param d The d value.
// @returns A string that contains a XML description of a XODR superelevation.
std::string GetSuperelevation(double s, double a, double b, double c, double d) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<superelevation s='" << s << "' a='" << a << "' b='" << b << "' c='" << c << "' d='" << d << "' />";
  ss << "</root>";
  return ss.str();
}

// Tests `Superelevation` parsing.
TEST_F(ParsingTests, NodeParserSuperelevation) {
  const LateralProfile::Superelevation kExpectedSuperelevation{1.1 /* s0 */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */,
                                                               5.5 /* d */};

  const std::string xml_description =
      GetSuperelevation(kExpectedSuperelevation.s_0, kExpectedSuperelevation.a, kExpectedSuperelevation.b,
                        kExpectedSuperelevation.c, kExpectedSuperelevation.d);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, LateralProfile::Superelevation::kSuperelevationTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LateralProfile::Superelevation::kSuperelevationTag, dut.GetName());
  const LateralProfile::Superelevation superelevation = dut.As<LateralProfile::Superelevation>();
  EXPECT_EQ(kExpectedSuperelevation, superelevation);
}

// Template of a XML description that contains a XODR lateralProfile.
constexpr const char* kLateralProfileTemplate = R"R(
<root>
  <lateralProfile>
    <superelevation s='1.1' a='2.2' b='3.3' c='4.4' d='5.5'/>
    <superelevation s='6.6' a='7.7' b='8.8' c='9.9' d='10.10'/>
    <superelevation s='11.11' a='12.12' b='13.13' c='14.14' d='15.15'/>
    <superelevation s='16.16' a='17.17' b='18.18' c='19.19' d='20.20'/>
    <superelevation s='21.21' a='22.22' b='23.23' c='24.24' d='25.25'/>
  </lateralProfile>
</root>
)R";

// Tests `LateralProfile` parsing.
TEST_F(ParsingTests, NodeParserLateralProfile) {
  const LateralProfile::Superelevation kSuperelevation1{1.1, 2.2, 3.3, 4.4, 5.5};
  const LateralProfile::Superelevation kSuperelevation2{6.6, 7.7, 8.8, 9.9, 10.10};
  const LateralProfile::Superelevation kSuperelevation3{11.11, 12.12, 13.13, 14.14, 15.15};
  const LateralProfile::Superelevation kSuperelevation4{16.16, 17.17, 18.18, 19.19, 20.20};
  const LateralProfile::Superelevation kSuperelevation5{21.21, 22.22, 23.23, 24.24, 25.25};
  const LateralProfile kExpectedLateralProfile{
      {kSuperelevation1, kSuperelevation2, kSuperelevation3, kSuperelevation4, kSuperelevation5}};

  const NodeParser dut(LoadXMLAndGetNodeByName(kLateralProfileTemplate, LateralProfile::kLateralProfileTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LateralProfile::kLateralProfileTag, dut.GetName());
  const LateralProfile lateral_profile = dut.As<LateralProfile>();
  EXPECT_EQ(kExpectedLateralProfile, lateral_profile);
}

// Get a XML description that contains a XODR width node.
// @param s_offset The s_offset value.
// @param a The a value.
// @param b The b value.
// @param c The c value.
// @param d The d value.
// @returns A string that contains a XML description of a XODR width node.
std::string GetWidth(double s_offset, double a, double b, double c, double d) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<width sOffset='" << s_offset << "' a='" << a << "' b='" << b << "' c='" << c << "' d='" << d << "' />";
  ss << "</root>";
  return ss.str();
}

// Tests `LaneWidth` parsing.
TEST_F(ParsingTests, NodeParserLaneWidth) {
  const LaneWidth kExpectedLaneWidth{1.1 /* sOffset */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};

  const std::string xml_description = GetWidth(kExpectedLaneWidth.s_0, kExpectedLaneWidth.a, kExpectedLaneWidth.b,
                                               kExpectedLaneWidth.c, kExpectedLaneWidth.d);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, LaneWidth::kLaneWidthTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LaneWidth::kLaneWidthTag, dut.GetName());
  const LaneWidth lane_width = dut.As<LaneWidth>();
  EXPECT_EQ(kExpectedLaneWidth, lane_width);
}

// Get a XML description that contains a XODR Lane::Speed.
// @param s_offset The s_offset value.
// @param max The max value.
// @param unit The unit value.
// @returns A string that contains a XML description of a XODR Lane::Speed.
std::string GetLaneSpeed(double s_offset, double max, const std::string& unit) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<speed sOffset='" << s_offset << "' max='" << max << "' unit='" << unit << "' />";
  ss << "</root>";
  return ss.str();
}

// Tests `Lane::Speed` parsing.
TEST_F(ParsingTests, NodeParserLaneSpeed) {
  const Lane::Speed kExpectedSpeed{0.1 /* sOffset */, 45. /* max */, Unit::kMph /* unit */};

  const std::string xml_description =
      GetLaneSpeed(kExpectedSpeed.s_offset, kExpectedSpeed.max, unit_to_str(kExpectedSpeed.unit));

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Lane::Speed::kSpeedTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Lane::Speed::kSpeedTag, dut.GetName());
  const Lane::Speed speed = dut.As<Lane::Speed>();
  EXPECT_EQ(kExpectedSpeed, speed);
}

// Get a XML description that contains a XODR lane offset node.
// @param s The s value.
// @param a The a value.
// @param b The b value.
// @param c The c value.
// @param d The d value.
// @returns A string that contains a XML description of a XODR lane offset node.
std::string GetLaneOffset(double s, double a, double b, double c, double d) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<laneOffset s='" << s << "' a='" << a << "' b='" << b << "' c='" << c << "' d='" << d << "' />";
  ss << "</root>";
  return ss.str();
}

// Tests `LaneOffset` parsing.
TEST_F(ParsingTests, NodeParserLaneOffset) {
  const LaneOffset kExpectedLaneOffset{1.1 /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};

  const std::string xml_description =
      GetLaneOffset(kExpectedLaneOffset.s_0, kExpectedLaneOffset.a, kExpectedLaneOffset.b, kExpectedLaneOffset.c,
                    kExpectedLaneOffset.d);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, LaneOffset::kLaneOffsetTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LaneOffset::kLaneOffsetTag, dut.GetName());
  const LaneOffset lane_offset = dut.As<LaneOffset>();
  EXPECT_EQ(kExpectedLaneOffset, lane_offset);
}

// Get a XML description that contains a XODR LaneLink.
// @param pred_element_id The predecessor element id.
// @param succ_element_id The successor element id.
// @returns A string that contains a XML description of a XODR LaneLink.
std::string GetLaneLink(const std::string& pred_element_id, const std::string& succ_element_id) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<link>";
  ss << "<predecessor id='" << pred_element_id << "'/>";
  ss << "<successor id='" << succ_element_id << "'/>";
  ss << "</link>";
  ss << "</root>";
  return ss.str();
}

// Tests `LaneLink` parsing.
TEST_F(ParsingTests, NodeParserLaneLink) {
  const LaneLink::LinkAttributes kPredecessor{LaneLink::LinkAttributes::Id("50") /* elementId*/};
  const LaneLink::LinkAttributes kSuccessor{LaneLink::LinkAttributes::Id("80") /* elementId*/};
  const LaneLink kExpectedLaneLink{kPredecessor, kSuccessor};
  const std::string xml_description = GetLaneLink(kPredecessor.id.string(), kSuccessor.id.string());

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, LaneLink::kLaneLinkTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LaneLink::kLaneLinkTag, dut.GetName());
  const LaneLink road_link = dut.As<LaneLink>();
  EXPECT_EQ(kExpectedLaneLink, road_link);
}

// Get a XML description that contains a XODR Lane node.
// @param id The id value.
// @param type The type value.
// @param level The level value.
// @param user_data The userData XODR node value.
// @param advisory The advisory value.
// @param direction The direction value.
// @param dynamic_lane_direction The dynamicLaneDirection value.
// @param dynamic_lane_type The dynamicLaneType value.
// @param road_works The roadWorks value.
// @returns A string that contains a XML description of a XODR Lane node.
std::string GetLane(const std::string& id, const std::string& type, const std::string& level,
                    const std::string& user_data, const std::string& advisory, const std::string& direction,
                    const std::string& dynamic_lane_direction, const std::string& dynamic_lane_type,
                    const std::string& road_works) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<lane " << Lane::kId << "='" << id << "' " << Lane::kType << "='" << type << "' " << Lane::kLevel << "='"
     << level << "' " << Lane::kAdvisory << "='" << advisory << "' " << Lane::kDirection << "='" << direction << "' "
     << Lane::kDynamicLaneDirection << "='" << dynamic_lane_direction << "' " << Lane::kDynamicLaneType << "='"
     << dynamic_lane_type << "' " << Lane::kRoadWorks << "='" << road_works << "'>";
  ss << "<link>";
  ss << "<predecessor id='50'/>";
  ss << "<successor id='80'/>";
  ss << "</link>";
  ss << "<width sOffset='1.1' a='2.2' b='3.3' c='4.4' d='5.5'/>";
  ss << "<width sOffset='6.6' a='7.7' b='8.8' c='9.9' d='10.1'/>";
  ss << "<speed sOffset='0.1' max='45.' unit='mph'/>";
  ss << "<speed sOffset='0.5' max='3.'/>";
  ss << user_data;
  ss << "</lane>";
  ss << "</root>";
  return ss.str();
}

// Tests `Lane` parsing.
TEST_F(ParsingTests, NodeParserLane) {
  const LaneLink::LinkAttributes kPredecessor{LaneLink::LinkAttributes::Id("50") /* elementId*/};
  const LaneLink::LinkAttributes kSuccessor{LaneLink::LinkAttributes::Id("80") /* elementId*/};
  const LaneLink lane_link{kPredecessor, kSuccessor};
  const std::vector<LaneWidth> kWidthDescription{
      {1.1 /* sOffset */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */},
      {6.6 /* sOffset */, 7.7 /* a */, 8.8 /* b */, 9.9 /* c */, 10.1 /* d */}};
  const std::vector<Lane::Speed> kSpeed{{0.1 /* sOffset */, 45. /* max */, Unit::kMph /* unit */},
                                        {0.5 /* sOffset */, 3. /* max */, Unit::kMs /* unit */}};
  const std::optional<std::string> kUserData{"<userData/>\n"};
  const std::optional<Lane::Advisory> kAdvisory{Lane::Advisory::kInner};
  const std::optional<Lane::Direction> kDirection{Lane::Direction::kStandard};
  const std::optional<bool> kDynamicLaneDirection{true};
  const std::optional<bool> kDynamicLaneType{false};
  const std::optional<bool> kRoadWorks{false};
  const Lane kExpectedLane{
      Lane::Id("test_id") /* id */,
      Lane::Type::kDriving /* type */,
      false /* level */,
      lane_link /* lane_link */,
      kWidthDescription /* widths */,
      kSpeed /*speed*/,
      kUserData /* user_data */,
      kAdvisory /* advisory */,
      kDirection /* direction */,
      kDynamicLaneDirection /* dynamic_lane_direction */,
      kDynamicLaneType /* dynamic_lane_type */,
      kRoadWorks /* road_works */
  };
  const std::string xml_description = GetLane(
      kExpectedLane.id.string(), Lane::type_to_str(kExpectedLane.type), kExpectedLane.level.value() ? "true" : "false",
      kUserData.value(), Lane::advisory_to_str(kExpectedLane.advisory.value()),
      Lane::direction_to_str(kExpectedLane.direction.value()),
      kExpectedLane.dynamic_lane_direction.value() ? "true" : "false",
      kExpectedLane.dynamic_lane_type.value() ? "true" : "false", kExpectedLane.road_works.value() ? "true" : "false");
  std::cout << "XML description: " << xml_description << std::endl;

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Lane::kLaneTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Lane::kLaneTag, dut.GetName());
  const Lane lane = dut.As<Lane>();
  EXPECT_EQ(kExpectedLane.id, lane.id);
  EXPECT_EQ(kExpectedLane.type, lane.type);
  EXPECT_EQ(kExpectedLane.level, lane.level);
  EXPECT_EQ(kExpectedLane.speed, lane.speed);
  EXPECT_EQ(kExpectedLane.advisory, lane.advisory);
  EXPECT_EQ(kExpectedLane.direction, lane.direction);
  EXPECT_EQ(kExpectedLane.dynamic_lane_direction, lane.dynamic_lane_direction);
  EXPECT_EQ(kExpectedLane.dynamic_lane_type, lane.dynamic_lane_type);
  EXPECT_EQ(kExpectedLane.road_works, lane.road_works);
  EXPECT_EQ(kExpectedLane.user_data.value(), lane.user_data.value());
  EXPECT_EQ(kExpectedLane, lane);
}

// Get a XML description that contains a XODR LaneSection node.
// @param s The s value.
// @returns A string that contains a XML description of a XODR LaneSection node.
std::string GetLaneSection(double s) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<laneSection s='" << s << "'>";
  ss << R"R(
      <left>
          <lane id='1' type='driving' level= '0'>
              <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
          </lane>
      </left>
      <center>
          <lane id='0' type='driving' level= '0'>
          </lane>
      </center>
      <right>
          <lane id='-1' type='driving' level= '0'>
              <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
          </lane>
      </right>
  </laneSection>
</root>
)R";
  return ss.str();
}

// Tests `LaneSection` parsing.
TEST_F(ParsingTests, NodeParserLaneSection) {
  const Lane left_lane{Lane::Id("1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{0., 1., 2., 3., 4.}}};
  const Lane center_lane{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane right_lane{Lane::Id("-1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{5., 6., 7., 8., 9.}}};
  const LaneSection kExpectedLaneSection{1.58 /* s_0 */,
                                         std::nullopt /* single_side */,
                                         {left_lane} /* left_lanes */,
                                         center_lane /* center_lane */,
                                         {right_lane} /* right_lanes */};

  const std::string xml_description = GetLaneSection(kExpectedLaneSection.s_0);

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, LaneSection::kLaneSectionTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LaneSection::kLaneSectionTag, dut.GetName());
  const LaneSection lane_section = dut.As<LaneSection>();
  EXPECT_EQ(kExpectedLaneSection, lane_section);
}

// XML description that contains a XODR LaneSection node with a wrong center lane.
constexpr const char* kLaneSectionWrongCenterLaneWidth = R"R(
<root>
  <laneSection s='0'>
      <left>
          <lane id='1' type='driving' level= '0'>
              <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
          </lane>
      </left>
      <center>
          <lane id='0' type='driving' level= '0'>
              <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
          </lane>
      </center>
      <right>
          <lane id='-1' type='driving' level= '0'>
              <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
          </lane>
      </right>
  </laneSection>
</root>
)R";

// Tests `LaneSection` parsing.
TEST_F(ParsingTests, NodeParserLaneSectionWrongCenterLaneWidth) {
  const Lane left_lane{Lane::Id("1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{0., 1., 2., 3., 4.}}};
  const Lane center_lane{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane right_lane{Lane::Id("-1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{5., 6., 7., 8., 9.}}};
  const LaneSection kExpectedLaneSection{1.58 /* s_0 */,
                                         std::nullopt /* single_side */,
                                         {left_lane} /* left_lanes */,
                                         center_lane /* center_lane */,
                                         {right_lane} /* right_lanes */};

  const NodeParser dut(LoadXMLAndGetNodeByName(kLaneSectionWrongCenterLaneWidth, LaneSection::kLaneSectionTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LaneSection::kLaneSectionTag, dut.GetName());
  EXPECT_THROW(dut.As<LaneSection>(), maliput::common::assertion_error);
}

// XML description that contains a XODR LaneSection node with a wrong center lane.
constexpr const char* kLaneSectionWrongCenterLaneSpeed = R"R(
  <root>
    <laneSection s='0'>
        <left>
            <lane id='1' type='driving' level= '0'>
                <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
            </lane>
        </left>
        <center>
            <lane id='0' type='driving' level= '0'>
                <speed sOffset='0.' max='45.' unit='mph'/>
            </lane>
        </center>
        <right>
            <lane id='-1' type='driving' level= '0'>
                <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
            </lane>
        </right>
    </laneSection>
  </root>
  )R";

// Tests `LaneSection` parsing.
TEST_F(ParsingTests, NodeParserLaneSectionWrongCenterLaneSpeed) {
  const Lane left_lane{Lane::Id("1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{0., 1., 2., 3., 4.}}};
  const Lane center_lane{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane right_lane{Lane::Id("-1"), Lane::Type::kDriving, false, {}, std::vector<LaneWidth>{{5., 6., 7., 8., 9.}}};
  const LaneSection kExpectedLaneSection{1.58 /* s_0 */,
                                         std::nullopt /* single_side */,
                                         {left_lane} /* left_lanes */,
                                         center_lane /* center_lane */,
                                         {right_lane} /* right_lanes */};

  const NodeParser dut(LoadXMLAndGetNodeByName(kLaneSectionWrongCenterLaneSpeed, LaneSection::kLaneSectionTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(LaneSection::kLaneSectionTag, dut.GetName());
  EXPECT_THROW(dut.As<LaneSection>(), maliput::common::assertion_error);
}

// Template of a XML description that contains a XODR Lanes node.
constexpr const char* kLanesTemplate = R"R(
<root>
  <lanes>
    <laneOffset s='0.' a='2.2' b='3.3' c='4.4' d='5.5'/>
    <laneOffset s='1.5' a='2.2' b='3.3' c='4.4' d='5.5'/>
    <laneSection s='0.'>
        <left>
            <lane id='1' type='driving' level= '0'>
                <link>
                  <predecessor id='1'/>
                  <successor id='1'/>
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
                  <predecessor id='-1'/>
                  <successor id='-1'/>
                </link>
                <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
            </lane>
        </right>
    </laneSection>
    <laneSection s='1.5'>
        <left>
            <lane id='1' type='driving' level= '0'>
                <link>
                  <predecessor id='1'/>
                  <successor id='1'/>
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
                  <predecessor id='-1'/>
                  <successor id='-1'/>
                </link>
                <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
            </lane>
        </right>
    </laneSection>
  </lanes>
</root>
)R";

// Tests `Lanes` parsing.
TEST_F(ParsingTests, NodeParserLanes) {
  const LaneLink kLaneLinkA{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* successor */};
  const LaneLink kLaneLinkB{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* successor */};
  const LaneOffset kLaneOffset1{0. /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  const LaneOffset kLaneOffset2{1.5 /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  const Lane kLeftLane{Lane::Id("1"), Lane::Type::kDriving, false, kLaneLinkA,
                       std::vector<LaneWidth>{{0., 1., 2., 3., 4.}}};
  const Lane kCenterLane{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane kRightLane{Lane::Id("-1"), Lane::Type::kDriving, false, kLaneLinkB,
                        std::vector<LaneWidth>{{5., 6., 7., 8., 9.}}};
  const LaneSection kLaneSection1{0. /* s_0 */,
                                  std::nullopt /* single_side */,
                                  {kLeftLane} /* left_lanes */,
                                  kCenterLane /* center_lane */,
                                  {kRightLane} /* right_lanes */};
  const LaneSection kLaneSection2{1.5 /* s_0 */,
                                  std::nullopt /* single_side */,
                                  {kLeftLane} /* left_lanes */,
                                  kCenterLane /* center_lane */,
                                  {kRightLane} /* right_lanes */};
  const Lanes kExpectedLanes{{{kLaneOffset1}, {kLaneOffset2}}, {{kLaneSection1}, {kLaneSection2}}};

  const NodeParser dut(LoadXMLAndGetNodeByName(kLanesTemplate, Lanes::kLanesTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Lanes::kLanesTag, dut.GetName());
  const Lanes lanes = dut.As<Lanes>();
  EXPECT_EQ(kExpectedLanes, lanes);
}

// Get a XML description that contains a XODR RoadHeader.
// @param name The name value.
// @param length The length value.
// @param id The id value.
// @param junction The junction value.
// @param rule The rule value.
// @returns A string that contains a XML description of a XODR RoadHeader.
std::string GetRoadHeader(const std::string& name, double length, const std::string& id, const std::string& junction,
                          const std::string& rule) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<road name='" << name << "' length='" << length << "' id='" << id << "' junction='" << junction << "' rule='"
     << rule << "'>";
  ss << R"R(<link>
         <predecessor elementType="road" elementId="80" contactPoint="end"/>
         <successor elementType="road" elementId="10" contactPoint="start"/>
     </link>
     <type s="0.0000000000000000e+0" type="town">
         <speed max="45" unit="mph"/>
     </type>
     <type s="53.0000000000000000e+0" type="rural" country="maliput">
         <speed max="80" unit="km/h"/>
     </type>
    <planView>
      <geometry s='0.' x='523.2' y='83.27' hdg='0.77' length='100.'>
          <line/>
      </geometry>
      <geometry s='100.' x='546.4' y='166.54' hdg='1.54' length='200.'>
          <arc curvature='0.5'/>
      </geometry>
    </planView>
    <elevationProfile>
      <elevation s='1.1' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <elevation s='6.6' a='7.7' b='8.8' c='9.9' d='10.10'/>
      <elevation s='11.11' a='12.12' b='13.13' c='14.14' d='15.15'/>
      <elevation s='16.16' a='17.17' b='18.18' c='19.19' d='20.20'/>
      <elevation s='21.21' a='22.22' b='23.23' c='24.24' d='25.25'/>
    </elevationProfile>
    <lateralProfile>
      <superelevation s='15.51' a='25.52' b='35.53' c='45.54' d='55.55'/>
      <superelevation s='65.56' a='75.57' b='85.58' c='95.59' d='105.510'/>
      <superelevation s='115.511' a='125.512' b='135.513' c='145.514' d='155.515'/>
      <superelevation s='165.516' a='175.517' b='185.518' c='195.519' d='205.520'/>
      <superelevation s='215.521' a='225.522' b='235.523' c='245.524' d='255.525'/>
    </lateralProfile>
    <lanes>
      <laneOffset s='0.' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <laneOffset s='1.5' a='2.2' b='3.3' c='4.4' d='5.5'/>
      <laneSection s='0.'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                    <predecessor id='1'/>
                    <successor id='1'/>
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
                  <speed sOffset='0.' max='45.' unit='mph'/>
                  <speed sOffset='0.5' max='3.'/>
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
      <laneSection s='100.'>
          <left>
              <lane id='1' type='driving' level= '0'>
                  <link>
                    <predecessor id='1'/>
                    <successor id='1'/>
                  </link>
                  <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
                  <speed sOffset='0.' max='45.' unit='mph'/>
                  <speed sOffset='0.5' max='3.'/>
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
  </road>"
</root>
)R";
  return ss.str();
}

// Tests `RoadHeader` parsing.
TEST_F(ParsingTests, NodeParserRoadHeader) {
  const LaneLink kLaneLinkA{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"1"}} /* successor */};
  const LaneLink kLaneLinkB{LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* predecessor */,
                            LaneLink::LinkAttributes{LaneLink::LinkAttributes::Id{"-1"}} /* successor */};
  const RoadLink::LinkAttributes kPredecessor{RoadLink::ElementType::kRoad /* elementType */,
                                              RoadLink::LinkAttributes::Id("80") /* elementId*/,
                                              RoadLink::ContactPoint::kEnd /* contactPoint*/};
  const RoadLink::LinkAttributes kSuccessor{RoadLink::ElementType::kRoad /* elementType */,
                                            RoadLink::LinkAttributes::Id("10") /* elementId*/,
                                            RoadLink::ContactPoint::kStart /* contactPoint*/};
  const RoadLink kExpectedRoadLink{kPredecessor, kSuccessor};
  const RoadType kRoadType1{0. /* s0 */,
                            RoadType::Type::kTown /* type */,
                            std::nullopt /* country */,
                            {45. /* max */, Unit::kMph /* unit */} /* speed */};
  const RoadType kRoadType2{53. /* s0 */,
                            RoadType::Type::kRural /* type */,
                            "maliput" /* country */,
                            {80. /* max */, Unit::kKph /* unit */} /* speed */};
  const PlanView kArbitraryPlanView{std::vector<Geometry>{Geometry{0. /* s_0 */,
                                                                   {523.2 /* x */, 83.27 /* y */},
                                                                   0.77 /* orientation */,
                                                                   100. /* length */,
                                                                   Geometry::Type::kLine /* type */,
                                                                   Geometry::Line{} /* description */},
                                                          Geometry{100. /* s_0 */,
                                                                   {546.4 /* x */, 166.54 /* y */},
                                                                   1.54 /* orientation */,
                                                                   200. /* length */,
                                                                   Geometry::Type::kArc /* type */,
                                                                   Geometry::Arc{0.5} /* description */}}};
  const ElevationProfile kElevationProfile{{{1.1, 2.2, 3.3, 4.4, 5.5},
                                            {6.6, 7.7, 8.8, 9.9, 10.10},
                                            {11.11, 12.12, 13.13, 14.14, 15.15},
                                            {16.16, 17.17, 18.18, 19.19, 20.20},
                                            {21.21, 22.22, 23.23, 24.24, 25.25}}};
  const LateralProfile kLateralProfile{{{15.51, 25.52, 35.53, 45.54, 55.55},
                                        {65.56, 75.57, 85.58, 95.59, 105.510},
                                        {115.511, 125.512, 135.513, 145.514, 155.515},
                                        {165.516, 175.517, 185.518, 195.519, 205.520},
                                        {215.521, 225.522, 235.523, 245.524, 255.525}}};
  const LaneOffset kLaneOffset1{0. /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  const LaneOffset kLaneOffset2{1.5 /* s */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */, 5.5 /* d */};
  const Lane::Speed kLaneSpeedA{0.0 /* sOffset */, 45. /* max */, Unit::kMph /* unit */};
  const Lane::Speed kLaneSpeedB{0.5 /* sOffset */, 3. /* max */, Unit::kMs /* unit */};
  const Lane kLeftLane{Lane::Id("1"),
                       Lane::Type::kDriving,
                       false,
                       kLaneLinkA,
                       std::vector<LaneWidth>{{0., 1., 2., 3., 4.}},
                       {kLaneSpeedA, kLaneSpeedB}};
  const Lane kCenterLane{Lane::Id("0"), Lane::Type::kDriving, false, {}, {}};
  const Lane kRightLane{
      Lane::Id("-1"), Lane::Type::kDriving, false, kLaneLinkB, std::vector<LaneWidth>{{5., 6., 7., 8., 9.}},
      {kLaneSpeedB}};
  const LaneSection kLaneSection1{0. /* s_0 */,
                                  std::nullopt /* single_side */,
                                  {kLeftLane} /* left_lanes */,
                                  kCenterLane /* center_lane */,
                                  {kRightLane} /* right_lanes */};
  const LaneSection kLaneSection2{100. /* s_0 */,
                                  std::nullopt /* single_side */,
                                  {kLeftLane} /* left_lanes */,
                                  kCenterLane /* center_lane */,
                                  {kRightLane} /* right_lanes */};
  const Lanes kLanes{{{kLaneOffset1}, {kLaneOffset2}}, {{kLaneSection1}, {kLaneSection2}}};
  const RoadHeader kExpectedRoadHeader{
      "TestRoadHeader" /* name */,
      10.65 /* length */,
      RoadHeader::Id("Road 15") /* id */,
      "-1" /* junction */,
      RoadHeader::HandTrafficRule::kRHT /* rule */,
      kExpectedRoadLink /* road_link */,
      {kRoadType1, kRoadType2} /* road_types */,
      {kArbitraryPlanView, kElevationProfile, kLateralProfile} /* reference_geometry */,
      kLanes /* lanes */};
  const std::string xml_description = GetRoadHeader(
      kExpectedRoadHeader.name.value(), kExpectedRoadHeader.length, kExpectedRoadHeader.id.string(),
      kExpectedRoadHeader.junction, RoadHeader::hand_traffic_rule_to_str(kExpectedRoadHeader.rule.value()));

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, RoadHeader::kRoadHeaderTag),
                       {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(RoadHeader::kRoadHeaderTag, dut.GetName());
  EXPECT_EQ(kExpectedRoadHeader, dut.As<RoadHeader>());
}

// Get a XML description that contains a XODR Connection.
// @param id The id value.
// @param incoming_road The incoming_road value.
// @param connecting_road The connecting_road value.
// @param contact_point The contact_point value.
// @param connection_master The connection_master value.
// @param type The type value.
// @param from_1 From value of first lane link.
// @param to_1 To value of first lane link.
// @param from_2 From value of second lane link.
// @param to_2 To value of second lane link.
// @returns A string that contains a XML description of a XODR Connection.
std::string GetConnection(const std::string& id, const std::string& incoming_road, const std::string& connecting_road,
                          const std::string& contact_point, const std::string& connection_master,
                          const std::string& type, const std::string& from_1, const std::string& to_1,
                          const std::string& from_2, const std::string& to_2) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<connection id='" << id << "' incomingRoad='" << incoming_road << "' connectingRoad='" << connecting_road
     << "' contactPoint='" << contact_point << "' connectionMaster='" << connection_master << "' type='" << type
     << "'>";
  ss << "<laneLink from='" << from_1 << "' to='" << to_1 << "'/>";
  ss << "<laneLink from='" << from_2 << "' to='" << to_2 << "'/>";
  ss << "</connection>";
  ss << "</root>";
  return ss.str();
}

// Tests `Connection` parsing.
TEST_F(ParsingTests, NodeParserConnection) {
  const Connection kExpectedConnection{
      Connection::Id("10") /* id */,
      "1" /* incoming_road */,
      "2" /* connecting_road */,
      Connection::ContactPoint::kStart /* contact_point */,
      Connection::Id("50") /* connection_master */,
      Connection::Type::kDefault /* type */,
      {{Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")},
       {Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")}} /* lane_links */};

  const std::string xml_description = GetConnection(
      kExpectedConnection.id.string(), kExpectedConnection.incoming_road, kExpectedConnection.connecting_road,
      Connection::contact_point_to_str(kExpectedConnection.contact_point),
      kExpectedConnection.connection_master->string(), Connection::type_to_str(kExpectedConnection.type.value()),
      kExpectedConnection.lane_links[0].from.string(), kExpectedConnection.lane_links[0].to.string(),
      kExpectedConnection.lane_links[1].from.string(), kExpectedConnection.lane_links[1].to.string());

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Connection::kConnectionTag),
                       {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Connection::kConnectionTag, dut.GetName());
  const Connection connection = dut.As<Connection>();
  EXPECT_EQ(kExpectedConnection, connection);
}

// Get a XML description that contains a XODR Junction.
// @param junction_id The junction_id value.
// @param junction_name The junction_name value.
// @param junction_type The junction_type value.
// @returns A string that contains a XML description of a XODR Junction.
std::string GetJunction(const std::string& junction_id, const std::string& junction_name,
                        const std::string& junction_type) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<junction id='" << junction_id << "' name='" << junction_name << "' type='" << junction_type << "'>";
  ss << R"R(
    <connection id="10" incomingRoad="1" connectingRoad="2" contactPoint="start" connectionMaster="50" type="default">
        <laneLink from="1" to="1"/>
        <laneLink from="-1" to="-1"/>
    </connection>
    <connection id="11" incomingRoad="3" connectingRoad="4" contactPoint="end">
        <laneLink from="2" to="4"/>
        <laneLink from="1" to="3"/>
    </connection>
  </junction>
</root>
)R";
  return ss.str();
}

// Tests `Junction` parsing.
TEST_F(ParsingTests, NodeParserJunction) {
  const Connection kConnectionA{Connection::Id("10") /* id */,
                                "1" /* incoming_road */,
                                "2" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                Connection::Id("50") /* connection_master */,
                                Connection::Type::kDefault /* type */,
                                {{Connection::LaneLink::Id("1"), Connection::LaneLink::Id("1")},
                                 {Connection::LaneLink::Id("-1"), Connection::LaneLink::Id("-1")}} /* lane_links */};
  const Connection kConnectionB{Connection::Id("11") /* id */,
                                "3" /* incoming_road */,
                                "4" /* connecting_road */,
                                Connection::ContactPoint::kEnd /* contact_point */,
                                std::nullopt /* connection_master */,
                                std::nullopt /* type */,
                                {{Connection::LaneLink::Id("2"), Connection::LaneLink::Id("4")},
                                 {Connection::LaneLink::Id("1"), Connection::LaneLink::Id("3")}} /* lane_links */};

  const Junction kExpectedJunction{
      Junction::Id("358") /* id */,
      "junctionTest" /* name */,
      Junction::Type::kDefault /* type */,
      {{kConnectionA.id, kConnectionA}, {kConnectionB.id, kConnectionB}} /* connections */};

  const std::string xml_description = GetJunction(kExpectedJunction.id.string(), kExpectedJunction.name.value(),
                                                  Junction::type_to_str(kExpectedJunction.type.value()));

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Junction::kJunctionTag),
                       {kStrictParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Junction::kJunctionTag, dut.GetName());
  const Junction junction = dut.As<Junction>();
  EXPECT_EQ(kExpectedJunction, junction);
}

// Template of a XML description that contains a XODR Junction with no connections.
constexpr const char* kJunctionTemplateNoConnections = R"R(
<root>
  <junction id="358" name="JunctionTest" type="default">
  </junction>
</root>
)R";

// Tests `Junction` parsing with no connections.
TEST_F(ParsingTests, NodeParserJunctionNoConnections) {
  bool allow_schema_errors{true};
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(kJunctionTemplateNoConnections, Junction::kJunctionTag),
                                  {kStrictParserSTolerance, allow_schema_errors, kDontAllowSemanticErrors});
  EXPECT_NO_THROW(dut_permissive.As<Junction>());

  allow_schema_errors = false;
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(kJunctionTemplateNoConnections, Junction::kJunctionTag),
                              {kStrictParserSTolerance, allow_schema_errors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<Junction>(), maliput::common::assertion_error);
}

GTEST_TEST(XMLToText, ConvertXMLNodeToText) {
  constexpr const char* kArbitraryXMLDescription = R"R(<root>
    <type s="0." type="rural">
        <speed max="45." unit="mph"/>
    </type>
</root>
)R";

  tinyxml2::XMLDocument xml_doc;
  MALIDRIVE_THROW_UNLESS(xml_doc.Parse(kArbitraryXMLDescription) == tinyxml2::XML_SUCCESS);
  const std::string str_xml = ConvertXMLNodeToText(xml_doc.FirstChildElement());
  EXPECT_EQ(kArbitraryXMLDescription, str_xml);
}

// Get a XML description that contains function description that matches with ElevationProfile or LateralProfile
// descriptions.
// - 1st and 2nd descriptions are equal
// - 3rd and 4th descriptions are equal
// - 5th description start at the same S value as 4th description.
// @param profile The profile tag.
// @param description The description tag.
std::string GetElevationSuperelevation(const std::string& profile, const std::string& description) {
  std::stringstream ss;
  ss << "<root>";
  ss << "<" << profile << ">";
  ss << "<" << description << " s='0.' a='1.' b='2.' c='3.' d='4.'/>";
  ss << "<" << description << " s='0.' a='1.' b='2.' c='3.' d='4.'/>";
  ss << "<" << description << " s='5.' a='2.' b='0.' c='0.' d='0.'/>";
  ss << "<" << description << " s='5.' a='2.' b='0.' c='0.' d='0.'/>";
  ss << "<" << description << " s='5.' a='2.1' b='0.' c='0.' d='0.'/>";
  ss << "</" << profile << ">";
  ss << "</root>";
  return ss.str();
}

// Tests `ElevationProfile` and `LateralProfile` parsing when having repeated descriptions.
// @{
class ElevationRepeatedDescriptionsParsingTests : public ParsingTests {
 protected:
  const std::string xml_description =
      GetElevationSuperelevation(ElevationProfile::kElevationProfileTag, ElevationProfile::Elevation::kElevationTag);
};

TEST_F(ElevationRepeatedDescriptionsParsingTests, NotAllowingSchemaErrors) {
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(xml_description, ElevationProfile::kElevationProfileTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<ElevationProfile>(), maliput::common::assertion_error);
}

TEST_F(ElevationRepeatedDescriptionsParsingTests, AllowingSchemaErrors) {
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, ElevationProfile::kElevationProfileTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  ElevationProfile elevation_profile;
  ASSERT_NO_THROW(elevation_profile = dut_permissive.As<ElevationProfile>());
  const ElevationProfile::Elevation kExpectedElevation0{0.0, 1.0, 2.0, 3.0, 4.0};
  const ElevationProfile::Elevation kExpectedElevation1{5.0, 2.1, 0.0, 0.0, 0.0};
  const ElevationProfile kExpectedElevationProfile{{kExpectedElevation0, kExpectedElevation1}};
  EXPECT_EQ(kExpectedElevationProfile, elevation_profile);
}

class SuperelevationRepeatedDescriptionsParsingTests : public ParsingTests {
 protected:
  const std::string xml_description = GetElevationSuperelevation(LateralProfile::kLateralProfileTag,
                                                                 LateralProfile::Superelevation::kSuperelevationTag);
};

TEST_F(SuperelevationRepeatedDescriptionsParsingTests, NotAllowingSchemaErrors) {
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(xml_description, LateralProfile::kLateralProfileTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<LateralProfile>(), maliput::common::assertion_error);
}

TEST_F(SuperelevationRepeatedDescriptionsParsingTests, AllowingSchemaErrors) {
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, LateralProfile::kLateralProfileTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  LateralProfile lateral_profile;
  ASSERT_NO_THROW(lateral_profile = dut_permissive.As<LateralProfile>());
  const LateralProfile::Superelevation kExpectedSuperelevation0{0.0, 1.0, 2.0, 3.0, 4.0};
  const LateralProfile::Superelevation kExpectedSuperelevation1{5.0, 2.1, 0.0, 0.0, 0.0};
  const LateralProfile kExpectedLateralProfile{{kExpectedSuperelevation0, kExpectedSuperelevation1}};
  EXPECT_EQ(kExpectedLateralProfile, lateral_profile);
}
// @}

// Template of a XML description that contains a XODR Lane node with repeated width descriptions.
// - 1st and 2nd descriptions are equal
// - 3rd and 4th descriptions are equal
// - 5th description start at the same S value as 4th description.
constexpr const char* kLaneRepeatedWidthTemplate = R"R(
<root>
  <lane id='1' type='driving' >
     <width sOffset="0.0" a="1.0" b="2.0" c="3.0" d="4.0"/>
     <width sOffset="0.0" a="1.0" b="2.0" c="3.0" d="4.0"/>
     <width sOffset="5.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
     <width sOffset="5.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
     <width sOffset="5.0" a="2.1" b="0.0" c="0.0" d="0.0"/>
  </lane>
</root>
)R";

// Tests `Lane` parsing when having repeated width descriptions.
class LaneRepeatedWidthDescriptionsParsingTests : public ParsingTests {};

TEST_F(LaneRepeatedWidthDescriptionsParsingTests, NotAllowingSchemaErrors) {
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(kLaneRepeatedWidthTemplate, Lane::kLaneTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<Lane>(), maliput::common::assertion_error);
}

TEST_F(LaneRepeatedWidthDescriptionsParsingTests, AllowingSchemaErrors) {
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(kLaneRepeatedWidthTemplate, Lane::kLaneTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  Lane lane;
  ASSERT_NO_THROW(lane = dut_permissive.As<Lane>());
  const LaneWidth kExpectedLaneWidth0{0.0, 1.0, 2.0, 3.0, 4.0};
  const LaneWidth kExpectedLaneWidth1{5.0, 2.1, 0.0, 0.0, 0.0};
  const Lane kExpectedLane{
      Lane::Id("1") /* id */,
      Lane::Type::kDriving /* type */,
      std::nullopt /* level */,
      {std::nullopt, std::nullopt} /* lane_link */,
      {kExpectedLaneWidth0, kExpectedLaneWidth1} /* widths */,
      {} /* speed */,
      std::nullopt /* user_data */
  };
  EXPECT_EQ(kExpectedLane, lane);
}

// Template of a XML description that contains a XODR Lanes node with repeated laneOffset descriptions.
constexpr const char* kLanesNodeWithRepeatedLaneOffsetTemplate = R"R(
<root>
  <lanes>
    <laneOffset s="0.0" a="1.0" b="2.0" c="3.0" d="4.0"/>
    <laneOffset s="0.0" a="1.0" b="2.0" c="3.0" d="4.0"/>
    <laneOffset s="5.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
    <laneOffset s="5.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
    <laneOffset s="5.0" a="2.1" b="0.0" c="0.0" d="0.0"/>
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
</root>
)R";

// Tests `Lanes` parsing when having repeated `laneOffset` descriptions.
class LaneOffsetRepeatedWidthDescriptionsParsingTests : public ParsingTests {};

TEST_F(LaneOffsetRepeatedWidthDescriptionsParsingTests, NotAllowingSchemaErrors) {
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(kLanesNodeWithRepeatedLaneOffsetTemplate, Lanes::kLanesTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<Lanes>(), maliput::common::assertion_error);
}

TEST_F(LaneOffsetRepeatedWidthDescriptionsParsingTests, AllowingSchemaErrors) {
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(kLanesNodeWithRepeatedLaneOffsetTemplate, Lanes::kLanesTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  Lanes lanes;
  EXPECT_NO_THROW(lanes = dut_permissive.As<Lanes>());
  const LaneOffset kExpectedLaneOffset0{0.0, 1.0, 2.0, 3.0, 4.0};
  const LaneOffset kExpectedLaneOffset1{5.0, 2.1, 0.0, 0.0, 0.0};
  const std::vector<LaneOffset> kExpectedLaneOffsets{kExpectedLaneOffset0, kExpectedLaneOffset1};
  EXPECT_EQ(kExpectedLaneOffsets, lanes.lanes_offset);
}

class FunctionsWithNanValuesTests : public ParsingTests {
 protected:
  static constexpr double kSameStartS{3.0};
  static constexpr double kDifferentStartS{1.0};
  // Get a XML description that contains function description that matches with ElevationProfile or
  // Lateralprofile descriptions.
  // - 1st function has nan values
  //   - It is loadable only when `s=3.0`(same as the next function) and schema errors are allowed.
  // @param profile The profile tag.
  // @param description The description tag.
  // @param s The s value of the first function description.
  // @returns A string that contains a XML description of a XODR ElevationProfile or LateralProfile.
  static std::string GetNanElevationSuperelevation(const std::string& profile, const std::string& description,
                                                   double s) {
    std::stringstream ss;
    ss << "<root>";
    ss << "<" << profile << ">";
    ss << "<" << description << " s='" << s << "' a='nan' b='nan' c='nan' d='nan'/>";
    ss << "<" << description << " s='3.0' a='2.0' b='0.0' c='0.0' d='0.0'/>";
    ss << "<" << description << " s='5.0' a='2.1' b='0.0' c='0.0' d='0.0'/>";
    ss << "</" << profile << ">";
    ss << "</root>";
    return ss.str();
  }
  // Get a XML description that contains a XODR Lane node with width descriptions filled with nan values.
  // - 1st function has nan values
  //   - It is loadable only when `s=3.0`(same as the next function) and schema errors are allowed.
  // @param s The s value of the first function description.
  // @returns A string that contains a XML description of a XODR Lane with the above description.
  static std::string GetNanLaneWidth(double s) {
    std::stringstream ss;
    ss << "<root>";
    ss << "<lane id='1' type='driving' >";
    ss << "<width sOffset='" << s << "' a='nan' b='nan' c='nan' d='nan'/>";
    ss << "<width sOffset='3.0' a='2.0' b='0.0' c='0.0' d='0.0'/>";
    ss << "<width sOffset='5.0' a='2.1' b='0.0' c='0.0' d='0.0'/>";
    ss << "</lane>";
    ss << "</root>";
    return ss.str();
  }
  // Get a XML description that contains a XODR Lanes node with laneOffset descriptions filled with nan values.
  // @param s The s value of the first offset description.
  // @returns A string that contains a XML description of a XODR Lanes with the above description.
  static std::string GetLanesNodeWithNanLaneOffset(double s) {
    std::stringstream ss;
    ss << "<root>";
    ss << "<lanes>";
    ss << "<laneOffset s='" << s << "' a='nan' b='nan' c='nan' d='nan'/>";
    ss << R"R(
      <laneOffset s="3.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
      <laneOffset s="5.0" a="2.1" b="0.0" c="0.0" d="0.0"/>
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
  </root>
  )R";
    return ss.str();
  }
};

// Tests `ElevationProfile` and `LateralProfile` parsing when having nan values.
// @{
class ElevationNanDescriptionsParsingTests : public FunctionsWithNanValuesTests {};

TEST_F(ElevationNanDescriptionsParsingTests, NotAllowingSchemaErrors) {
  const std::string xml_description = GetNanElevationSuperelevation(
      ElevationProfile::kElevationProfileTag, ElevationProfile::Elevation::kElevationTag, kSameStartS);
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(xml_description, ElevationProfile::kElevationProfileTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<ElevationProfile>(), maliput::common::assertion_error);
}

// A function has NaN values and schema errors are allowed.
// However it throws because NaN values are allowed in the function iff
// the next function description starts at the same `s` value so as to discard the one with NaN values.
TEST_F(ElevationNanDescriptionsParsingTests, AllowingSchemaErrorsDifferentStartPoint) {
  const std::string xml_description = GetNanElevationSuperelevation(
      ElevationProfile::kElevationProfileTag, ElevationProfile::Elevation::kElevationTag, kDifferentStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, ElevationProfile::kElevationProfileTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_permissive.As<ElevationProfile>(), maliput::common::assertion_error);
}

TEST_F(ElevationNanDescriptionsParsingTests, AllowingSchemaErrorsSameStartPoint) {
  const std::string xml_description = GetNanElevationSuperelevation(
      ElevationProfile::kElevationProfileTag, ElevationProfile::Elevation::kElevationTag, kSameStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, ElevationProfile::kElevationProfileTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  ElevationProfile elevation_profile;
  ASSERT_NO_THROW(elevation_profile = dut_permissive.As<ElevationProfile>());
  const ElevationProfile::Elevation kExpectedElevation0{3.0, 2.0, 0.0, 0.0, 0.0};
  const ElevationProfile::Elevation kExpectedElevation1{5.0, 2.1, 0.0, 0.0, 0.0};
  const ElevationProfile kExpectedElevationProfile{{kExpectedElevation0, kExpectedElevation1}};
  EXPECT_EQ(kExpectedElevationProfile, elevation_profile);
}

class SuperelevationNanDescriptionsParsingTests : public FunctionsWithNanValuesTests {};

TEST_F(SuperelevationNanDescriptionsParsingTests, NotAllowingSchemaErrors) {
  const std::string xml_description = GetNanElevationSuperelevation(
      LateralProfile::kLateralProfileTag, LateralProfile::Superelevation::kSuperelevationTag, kSameStartS);
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(xml_description, LateralProfile::kLateralProfileTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<LateralProfile>(), maliput::common::assertion_error);
}

// A function has NaN values and schema errors are allowed.
// However it throws because NaN values are allowed in the function iff
// the next function description starts at the same `s` value so as to discard the one with NaN values.
TEST_F(SuperelevationNanDescriptionsParsingTests, AllowingSchemaErrorsDifferentStartPoint) {
  const std::string xml_description = GetNanElevationSuperelevation(
      LateralProfile::kLateralProfileTag, LateralProfile::Superelevation::kSuperelevationTag, kDifferentStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, LateralProfile::kLateralProfileTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_permissive.As<LateralProfile>(), maliput::common::assertion_error);
}

TEST_F(SuperelevationNanDescriptionsParsingTests, AllowingSchemaErrorsSameStartPoint) {
  const std::string xml_description = GetNanElevationSuperelevation(
      LateralProfile::kLateralProfileTag, LateralProfile::Superelevation::kSuperelevationTag, kSameStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, LateralProfile::kLateralProfileTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  LateralProfile lateral_profile;
  ASSERT_NO_THROW(lateral_profile = dut_permissive.As<LateralProfile>());
  const LateralProfile::Superelevation kExpectedSuperelevation0{3.0, 2.0, 0.0, 0.0, 0.0};
  const LateralProfile::Superelevation kExpectedSuperelevation1{5.0, 2.1, 0.0, 0.0, 0.0};
  const LateralProfile kExpectedLateralProfile{{kExpectedSuperelevation0, kExpectedSuperelevation1}};
  EXPECT_EQ(kExpectedLateralProfile, lateral_profile);
}
// @}

// Tests `Lane` parsing when having width descriptions with nan values.
class LaneWithNanWidthDescriptionsParsingTests : public FunctionsWithNanValuesTests {};

TEST_F(LaneWithNanWidthDescriptionsParsingTests, NotAllowingSchemaErrors) {
  const std::string xml_description = GetNanLaneWidth(kSameStartS);
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(xml_description, Lane::kLaneTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<Lane>(), maliput::common::assertion_error);
}

// A function has NaN values and schema errors are allowed.
// However it throws because NaN values are allowed in the function iff
// the next function description starts at the same `s` value so as to discard the one with NaN values.
TEST_F(LaneWithNanWidthDescriptionsParsingTests, AllowingSchemaErrorsDifferentStartPoint) {
  const std::string xml_description = GetNanLaneWidth(kDifferentStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, Lane::kLaneTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_permissive.As<Lane>(), maliput::common::assertion_error);
}

TEST_F(LaneWithNanWidthDescriptionsParsingTests, AllowingSchemaErrorsSameStartPoint) {
  const std::string xml_description = GetNanLaneWidth(kSameStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, Lane::kLaneTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  Lane lane;
  ASSERT_NO_THROW(lane = dut_permissive.As<Lane>());
  const LaneWidth kExpectedLaneWidth0{3.0, 2.0, 0.0, 0.0, 0.0};
  const LaneWidth kExpectedLaneWidth1{5.0, 2.1, 0.0, 0.0, 0.0};
  const Lane kExpectedLane{
      Lane::Id("1") /* id */,
      Lane::Type::kDriving /* type */,
      std::nullopt /* level */,
      {std::nullopt, std::nullopt} /* lane_link */,
      {kExpectedLaneWidth0, kExpectedLaneWidth1} /* widths */,
      {} /* speed */,
      std::nullopt /* user_data */
  };
  EXPECT_EQ(kExpectedLane, lane);
}

// Tests `Lanes` parsing when having offset descriptions with nan values.
class LanesWithNanLaneOffsetParsingTests : public FunctionsWithNanValuesTests {};

TEST_F(LanesWithNanLaneOffsetParsingTests, NotAllowingSchemaErrors) {
  const std::string xml_description = GetLanesNodeWithNanLaneOffset(kSameStartS);
  const NodeParser dut_strict(LoadXMLAndGetNodeByName(xml_description, Lanes::kLanesTag),
                              {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_strict.As<Lanes>(), maliput::common::assertion_error);
}

// A function has NaN values and schema errors are allowed.
// However it throws because NaN values are allowed in the function iff
// the next function description starts at the same `s` value so as to discard the one with NaN values.
TEST_F(LanesWithNanLaneOffsetParsingTests, AllowingSchemaErrorsDifferentStartPoint) {
  const std::string xml_description = GetLanesNodeWithNanLaneOffset(kDifferentStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, Lanes::kLanesTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_THROW(dut_permissive.As<Lanes>(), maliput::common::assertion_error);
}

TEST_F(LanesWithNanLaneOffsetParsingTests, AllowingSchemaErrorsSameStartPoint) {
  const std::string xml_description = GetLanesNodeWithNanLaneOffset(kSameStartS);
  const NodeParser dut_permissive(LoadXMLAndGetNodeByName(xml_description, Lanes::kLanesTag),
                                  {kNullParserSTolerance, kAllowSchemaErrors, kDontAllowSemanticErrors});
  Lanes lanes;
  EXPECT_NO_THROW(lanes = dut_permissive.As<Lanes>());
  const LaneOffset kExpectedLaneOffset0{3.0, 2.0, 0.0, 0.0, 0.0};
  const LaneOffset kExpectedLaneOffset1{5.0, 2.1, 0.0, 0.0, 0.0};
  const std::vector<LaneOffset> kExpectedLaneOffsets{kExpectedLaneOffset0, kExpectedLaneOffset1};
  EXPECT_EQ(kExpectedLaneOffsets, lanes.lanes_offset);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
