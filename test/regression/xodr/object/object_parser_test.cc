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
#include <algorithm>
#include <sstream>

#include <gtest/gtest.h>
#include <maliput/common/error.h>

#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/object/object.h"
#include "maliput_malidrive/xodr/parser.h"

namespace malidrive {
namespace xodr {
namespace test {
namespace {

class ObjectParsingTests : public ::testing::Test {
 protected:
  // Flag to not allow schema errors.
  static constexpr bool kDontAllowSchemaErrors{false};
  // Flag to not allow semantic errors.
  static constexpr bool kDontAllowSemanticErrors{false};
  // Flag to allow schema errors.
  static constexpr bool kAllowSchemaErrors{true};
  // Flag to support userData XODR parsing.
  static constexpr bool kUseUserDataTrafficDirection{true};
  // Flag to not support userData XODR parsing.
  static constexpr bool kDontSupportUserData{false};

  tinyxml2::XMLElement* LoadXMLAndGetNodeByName(const std::string& xml_str, const std::string& node_name) {
    MALIDRIVE_THROW_UNLESS(xml_doc_.Parse(xml_str.c_str()) == tinyxml2::XML_SUCCESS,
                           maliput::common::road_network_description_parser_error);
    tinyxml2::XMLElement* p_xml = xml_doc_.FirstChildElement();
    MALIDRIVE_THROW_UNLESS(p_xml != nullptr, maliput::common::road_network_description_parser_error);
    tinyxml2::XMLElement* p_elem = p_xml->FirstChildElement(node_name.c_str());
    MALIDRIVE_THROW_UNLESS(p_elem != nullptr, maliput::common::road_network_description_parser_error);
    return p_elem;
  }
  const std::optional<double> kNullParserSTolerance{std::nullopt};  // Disables the check because it is not needed.
  const std::optional<double> kStrictParserSTolerance{malidrive::constants::kStrictLinearTolerance};
  tinyxml2::XMLDocument xml_doc_;
};

// Get a XML description that contains a basic XODR object node.
std::string GetBasicObject() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<object type="building"
        subtype="building"
        name="house"
        id="0"
        s="1."
        t="2."
        zOffset="3."
        orientation="none"
        length="4."
        width="5."
        height="6."
        hdg="7."
        pitch="8."
        roll="9.">
</object>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Repeat node.
std::string GetRepeat() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<repeat
        s="1."
        length="2."
        distance="3."
        tStart="4."
        tEnd="5."
        heightStart="6."
        heightEnd="7."
        zOffsetStart="8."
        zOffsetEnd="9."
        widthStart="10."
        widthEnd="11."
        lengthStart="12."
        lengthEnd="13."
        radiusStart="14."
        radiusEnd="15."
        detachFromReferenceLine="true">
</repeat>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Outlines node.
std::string GetOutlines() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<outlines>
    <outline id="1" closed="true" fillType="grass" laneType="driving" outer="false">
        <cornerRoad s="1." t="2." dz="3." height="4." id="5"/>
        <cornerLocal u="6." v="7." z="8." height="9." id="10"/>
    </outline>
</outlines>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Skeleton node.
std::string GetSkeleton() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<skeleton>
    <polyline id="1">
        <vertexRoad s="2." t="3." dz="4." id="5" intersectionPoint="true" radius="6."/>
        <vertexLocal u="7." v="8." z="9." id="10" intersectionPoint="false" radius="11."/>
    </polyline>
</skeleton>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Material node.
std::string GetMaterial() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<material surface="asphalt" friction="0.8" roughness="0.02" roadMarkColor="white"/>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Validity node.
std::string GetValidity() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<validity fromLane="1" toLane="3"/>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR ParkingSpace node.
std::string GetParkingSpace() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<parkingSpace access="car" restrictions="none"/>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Markings node.
std::string GetMarkings() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<markings>
    <marking side="left" startOffset="0.1" stopOffset="0.2" lineLength="0.5" spaceLength="0.5" width="0.12" color="white" weight="standard" zOffset="0.05">
        <cornerReference id="1"/>
    </marking>
</markings>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Borders node.
std::string GetBorders() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<borders>
    <border outlineId="1" width="0.5" type="curb" useCompleteOutline="true"/>
</borders>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR ObjectReference node.
std::string GetObjectReference() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<objectReference id="1" s="10." t="2." orientation="+" validLength="5." zOffset="0.1">
    <validity fromLane="1" toLane="1"/>
</objectReference>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Tunnel node.
std::string GetTunnel() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<tunnel id="1" s="10." length="100." name="MyTunnel" type="standard" daylight="0.5" lighting="0.8">
    <validity fromLane="-1" toLane="1"/>
</tunnel>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Bridge node.
std::string GetBridge() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<bridge id="1" s="200." length="50." name="MyBridge" type="concrete">
    <validity fromLane="-1" toLane="1"/>
</bridge>)R";
  ss << "</root>";
  return ss.str();
}

// Get a XML description that contains a basic XODR Surface node.
std::string GetSurface() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<surface>
    <crg file="my_crg_file.crg" zScale="1.0" hideRoadSurfaceCRG="false"/>
</surface>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `object::Object` parsing.
TEST_F(ObjectParsingTests, NodeParserObject) {
  const object::Object kExpectedObject{
      std::nullopt,                           // dynamic
      7.,                                     // hdg
      6.,                                     // height
      object::Object::Id("0"),                // id
      4.,                                     // length
      "house",                                // name
      object::Orientation::kNone,             // orientation
      std::nullopt,                           // perp_to_road
      8.,                                     // pitch
      std::nullopt,                           // radius
      9.,                                     // roll
      1.,                                     // s
      "building",                             // subtype
      2.,                                     // t
      object::Object::ObjectType::kBuilding,  // type
      std::nullopt,                           // valid_length
      5.,                                     // width
      3.,                                     // z_offset
  };

  const std::string xml_description = GetBasicObject();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Object::kObjectTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Object::kObjectTag, dut.GetName());
  const object::Object obj = dut.As<object::Object>();
  EXPECT_EQ(kExpectedObject, obj);
}

// Tests `object::Repeat` parsing.
TEST_F(ObjectParsingTests, NodeParserRepeat) {
  const object::Repeat kExpectedRepeat{
      true,  // detach_from_reference_line
      3.,    // distance
      7.,    // height_end
      6.,    // height_start
      13.,   // length_end
      12.,   // length_start
      2.,    // length
      15.,   // radius_end
      14.,   // radius_start
      1.,    // s
      5.,    // t_end
      4.,    // t_start
      11.,   // width_end
      10.,   // width_start
      9.,    // z_offset_end
      8.,    // z_offset_start
  };

  const std::string xml_description = GetRepeat();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Repeat::kRepeatTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Repeat::kRepeatTag, dut.GetName());
  const object::Repeat repeat = dut.As<object::Repeat>();
  EXPECT_EQ(kExpectedRepeat, repeat);
}

// Tests `object::Outlines` parsing.
TEST_F(ObjectParsingTests, NodeParserOutlines) {
  const object::Outlines kExpectedOutlines{{
      {true,
       object::Outline::FillType::kGrass,
       object::Outline::Id("1"),
       Lane::Type::kDriving,
       false,
       {{3., 4., object::CornerRoad::Id("5"), 1., 2.}},
       {{9., object::CornerLocal::Id("10"), 6., 7., 8.}}},
  }};

  const std::string xml_description = GetOutlines();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Outlines::kOutlinesTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Outlines::kOutlinesTag, dut.GetName());
  const object::Outlines outlines = dut.As<object::Outlines>();
  EXPECT_EQ(kExpectedOutlines, outlines);
}

// Tests `object::Skeleton` parsing.
TEST_F(ObjectParsingTests, NodeParserSkeleton) {
  const object::Skeleton kExpectedSkeleton{{
      {object::Polyline::Id("1"),
       {{4., object::VertexRoad::Id("5"), true, 6., 2., 3.}},
       {{object::VertexLocal::Id("10"), false, 11., 7., 8., 9.}}},
  }};

  const std::string xml_description = GetSkeleton();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Skeleton::kSkeletonTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Skeleton::kSkeletonTag, dut.GetName());
  const object::Skeleton skeleton = dut.As<object::Skeleton>();
  EXPECT_EQ(kExpectedSkeleton, skeleton);
}

// Tests `object::Material` parsing.
TEST_F(ObjectParsingTests, NodeParserMaterial) {
  const object::Material kExpectedMaterial{0.8, Color::kWhite, 0.02, "asphalt"};

  const std::string xml_description = GetMaterial();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Material::kMaterialTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Material::kMaterialTag, dut.GetName());
  const object::Material material = dut.As<object::Material>();
  EXPECT_EQ(kExpectedMaterial, material);
}

// Tests `object::Validity` parsing.
TEST_F(ObjectParsingTests, NodeParserValidity) {
  const object::Validity kExpectedValidity{object::Validity::Id("1"), object::Validity::Id("3")};

  const std::string xml_description = GetValidity();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Validity::kValidityTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Validity::kValidityTag, dut.GetName());
  const object::Validity validity = dut.As<object::Validity>();
  EXPECT_EQ(kExpectedValidity, validity);
}

// Tests `object::ParkingSpace` parsing.
TEST_F(ObjectParsingTests, NodeParserParkingSpace) {
  const object::ParkingSpace kExpectedParkingSpace{object::ParkingSpace::Access::kCar, "none"};

  const std::string xml_description = GetParkingSpace();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::ParkingSpace::kParkingSpaceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::ParkingSpace::kParkingSpaceTag, dut.GetName());
  const object::ParkingSpace parking_space = dut.As<object::ParkingSpace>();
  EXPECT_EQ(kExpectedParkingSpace, parking_space);
}

// Tests `object::Markings` parsing.
TEST_F(ObjectParsingTests, NodeParserMarkings) {
  const object::Markings kExpectedMarkings{{
      {Color::kWhite,
       0.5,
       object::Marking::Side::kLeft,
       0.5,
       0.1,
       0.2,
       LaneRoadMark::Weight::kStandard,
       0.12,
       0.05,
       {{object::CornerReference::Id("1")}}},
  }};

  const std::string xml_description = GetMarkings();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Markings::kMarkingsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Markings::kMarkingsTag, dut.GetName());
  const object::Markings markings = dut.As<object::Markings>();
  EXPECT_EQ(kExpectedMarkings, markings);
}

// Tests `object::Borders` parsing.
TEST_F(ObjectParsingTests, NodeParserBorders) {
  const object::Borders kExpectedBorders{
      {{object::Border::Id("1"), object::Border::Type::kCurb, true, 0.5}},
  };

  const std::string xml_description = GetBorders();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Borders::kBordersTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Borders::kBordersTag, dut.GetName());
  const object::Borders borders = dut.As<object::Borders>();
  EXPECT_EQ(kExpectedBorders, borders);
}

// Tests `object::ObjectReference` parsing.
TEST_F(ObjectParsingTests, NodeParserObjectReference) {
  const object::ObjectReference kExpectedObjectReference{object::ObjectReference::Id("1"),
                                                         object::Orientation::kPositive,
                                                         10.,
                                                         2.,
                                                         5.,
                                                         0.1,
                                                         {{object::Validity::Id("1"), object::Validity::Id("1")}}};

  const std::string xml_description = GetObjectReference();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::ObjectReference::kObjectReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::ObjectReference::kObjectReferenceTag, dut.GetName());
  const object::ObjectReference object_reference = dut.As<object::ObjectReference>();
  EXPECT_EQ(kExpectedObjectReference, object_reference);
}

// Tests `object::Tunnel` parsing.
TEST_F(ObjectParsingTests, NodeParserTunnel) {
  const object::Tunnel kExpectedTunnel{0.5,
                                       object::Tunnel::Id("1"),
                                       100.,
                                       0.8,
                                       "MyTunnel",
                                       10.,
                                       object::Tunnel::Type::kStandard,
                                       {{object::Validity::Id("-1"), object::Validity::Id("1")}}};

  const std::string xml_description = GetTunnel();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Tunnel::kTunnelTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Tunnel::kTunnelTag, dut.GetName());
  const object::Tunnel tunnel = dut.As<object::Tunnel>();
  EXPECT_EQ(kExpectedTunnel, tunnel);
}

// Tests `object::Bridge` parsing.
TEST_F(ObjectParsingTests, NodeParserBridge) {
  const object::Bridge kExpectedBridge{object::Bridge::Id("1"),
                                       50.,
                                       "MyBridge",
                                       200.,
                                       object::Bridge::Type::kConcrete,
                                       {{object::Validity::Id("-1"), object::Validity::Id("1")}}};

  const std::string xml_description = GetBridge();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Bridge::kBridgeTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Bridge::kBridgeTag, dut.GetName());
  const object::Bridge bridge = dut.As<object::Bridge>();
  EXPECT_EQ(kExpectedBridge, bridge);
}

// Tests `object::Surface` parsing.
TEST_F(ObjectParsingTests, NodeParserSurface) {
  const object::Surface kExpectedSurface{
      {{
          "my_crg_file.crg",
          false,
          1.0,
      }},
  };

  const std::string xml_description = GetSurface();

  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, object::Surface::kSurfaceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(object::Surface::kSurfaceTag, dut.GetName());
  const object::Surface surface = dut.As<object::Surface>();
  EXPECT_EQ(kExpectedSurface, surface);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
