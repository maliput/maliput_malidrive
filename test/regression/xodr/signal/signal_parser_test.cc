// BSD 3-Clause License
//
// Copyright (c) 2026, Woven Planet. All rights reserved.
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
#include <sstream>

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include "maliput_malidrive/xodr/parser.h"
#include "maliput_malidrive/xodr/signal/board.h"
#include "maliput_malidrive/xodr/signal/controller.h"
#include "maliput_malidrive/xodr/signal/dependency.h"
#include "maliput_malidrive/xodr/signal/reference.h"
#include "maliput_malidrive/xodr/signal/semantics.h"
#include "maliput_malidrive/xodr/signal/sign.h"
#include "maliput_malidrive/xodr/signal/signal.h"
#include "maliput_malidrive/xodr/signal/signal_reference.h"

namespace malidrive {
namespace xodr {
namespace signal {
namespace test {
namespace {

class SignalParsingTests : public ::testing::Test {
 protected:
  tinyxml2::XMLElement* LoadXMLAndGetNodeByName(const std::string& xml_str, const std::string& node_name) {
    xml_doc_.Parse(xml_str.c_str());
    if (xml_doc_.Error()) {
      ADD_FAILURE() << "XML parsing error: " << xml_doc_.ErrorStr();
      return nullptr;
    }
    auto* element = xml_doc_.FirstChildElement("root");
    if (!element) {
      ADD_FAILURE() << "No root element found";
      return nullptr;
    }
    return element->FirstChildElement(node_name.c_str());
  }

  const std::optional<double> kNullParserSTolerance{std::nullopt};
  static constexpr bool kDontAllowSchemaErrors{false};
  static constexpr bool kDontAllowSemanticErrors{false};

  tinyxml2::XMLDocument xml_doc_;
};

// Get a XML description that contains a basic DisplayArea node.
std::string GetBasicDisplayArea() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<displayArea height="2.5" index="1" v="0.75" width="1.8" z="0.5"/>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::VmsBoard::DisplayArea` parsing.
TEST_F(SignalParsingTests, NodeParserDisplayArea) {
  const VmsBoard::DisplayArea kExpectedDisplayArea{2.5, 1, 0.75, 1.8, 0.5};

  const std::string xml_description = GetBasicDisplayArea();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, VmsBoard::DisplayArea::kDisplayAreaTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(VmsBoard::DisplayArea::kDisplayAreaTag, dut.GetName());
  const VmsBoard::DisplayArea display_area = dut.As<VmsBoard::DisplayArea>();
  EXPECT_EQ(kExpectedDisplayArea, display_area);
}

// Get a XML description that contains a basic VmsBoard node.
std::string GetBasicVmsBoard() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<vmsBoard displayHeight="3.0" displayType="led" displayWidth="5.0" v="0.5" z="1.2">
    <displayArea height="2.0" index="0" v="0.25" width="1.5" z="0.3"/>
</vmsBoard>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::VmsBoard` parsing.
TEST_F(SignalParsingTests, NodeParserVmsBoard) {
  const VmsBoard::DisplayArea kDisplayArea{2.0, 0, 0.25, 1.5, 0.3};
  const VmsBoard kExpectedVmsBoard{
      std::make_optional(3.0), VmsBoard::DisplayType::kLed, std::make_optional(5.0), 0.5, 1.2, {{kDisplayArea}}};

  const std::string xml_description = GetBasicVmsBoard();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, VmsBoard::kVmsBoardTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(VmsBoard::kVmsBoardTag, dut.GetName());
  const VmsBoard vms_board = dut.As<VmsBoard>();
  EXPECT_EQ(kExpectedVmsBoard, vms_board);
}

// Get a XML description that contains a basic StaticBoard node.
std::string GetBasicStaticBoard() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<staticBoard>
    <sign id="1" s="1.0" t="2.0" dynamic="no" orientation="+" zOffset="0.1" country="DE" type="274" subtype="60" height="0.77" width="0.77" v="0.5" z="1.2"/>
</staticBoard>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::StaticBoard` parsing.
TEST_F(SignalParsingTests, NodeParserStaticBoard) {
  const std::string xml_description = GetBasicStaticBoard();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, StaticBoard::kStaticBoardTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(StaticBoard::kStaticBoardTag, dut.GetName());
  const StaticBoard static_board = dut.As<StaticBoard>();
  EXPECT_EQ(1u, static_board.signs.size());
  EXPECT_DOUBLE_EQ(0.5, static_board.signs[0].v);
  EXPECT_DOUBLE_EQ(1.2, static_board.signs[0].z);
}

// Get a XML description that contains a basic Reference node.
std::string GetBasicReference() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<reference elementId="10" elementType="object" type="reference_type"/>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::Reference` parsing.
TEST_F(SignalParsingTests, NodeParserReference) {
  const Reference kExpectedReference{"10", Reference::ElementType::kObject, std::make_optional("reference_type")};

  const std::string xml_description = GetBasicReference();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Reference::kReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Reference::kReferenceTag, dut.GetName());
  const Reference reference = dut.As<Reference>();
  EXPECT_EQ(kExpectedReference, reference);
}

std::string GetReferenceMissingElementId() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<reference elementType="object" type="reference_type"/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserReferenceMissingElementId) {
  const std::string xml_description = GetReferenceMissingElementId();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Reference::kReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Reference::kReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<Reference>(), maliput::common::road_network_description_parser_error);
}

std::string GetReferenceMissingElementType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<reference elementId="10" type="reference_type"/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserReferenceMissingElementType) {
  const std::string xml_description = GetReferenceMissingElementType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Reference::kReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Reference::kReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<Reference>(), maliput::common::road_network_description_parser_error);
}

std::string GetReferenceInvalidElementType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<reference elementId="10" elementType="invalid" type="reference_type"/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserReferenceInvalidElementType) {
  const std::string xml_description = GetReferenceInvalidElementType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Reference::kReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Reference::kReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<Reference>(), maliput::common::road_network_description_parser_error);
}

// Get a XML description that contains a basic SignalReference node.
std::string GetBasicSignalReference() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<signalReference id="signal_1" orientation="+" s="10.0" t="5.0">
    <validity fromLane="1" toLane="3"/>
</signalReference>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::SignalReference` parsing.
TEST_F(SignalParsingTests, NodeParserSignalReference) {
  const Validity kValidity{Validity::Id("1"), Validity::Id("3")};
  const SignalReference kExpectedSignalReference{
      SignalReference::SignalId("signal_1"), SignalReference::Orientation::kWithS, 10.0, 5.0, {{kValidity}}};

  const std::string xml_description = GetBasicSignalReference();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, SignalReference::kSignalReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(SignalReference::kSignalReferenceTag, dut.GetName());
  const SignalReference signal_reference = dut.As<SignalReference>();
  EXPECT_EQ(kExpectedSignalReference, signal_reference);
}

std::string GetSignalReferenceMissingId() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<signalReference orientation="+" s="10.0" t="5.0">
    <validity fromLane="1" toLane="3"/>
</signalReference>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSignalReferenceMissingId) {
  const std::string xml_description = GetSignalReferenceMissingId();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, SignalReference::kSignalReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(SignalReference::kSignalReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<SignalReference>(), maliput::common::road_network_description_parser_error);
}

std::string GetSignalReferenceMissingOrientation() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<signalReference id="signal_1" s="10.0" t="5.0">
    <validity fromLane="1" toLane="3"/>
</signalReference>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSignalReferenceMissingOrientation) {
  const std::string xml_description = GetSignalReferenceMissingOrientation();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, SignalReference::kSignalReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(SignalReference::kSignalReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<SignalReference>(), maliput::common::road_network_description_parser_error);
}

std::string GetSignalReferenceInvalidOrientation() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<signalReference id="signal_1" orientation="invalid" s="10.0" t="5.0">
    <validity fromLane="1" toLane="3"/>
</signalReference>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSignalReferenceInvalidOrientation) {
  const std::string xml_description = GetSignalReferenceInvalidOrientation();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, SignalReference::kSignalReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(SignalReference::kSignalReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<SignalReference>(), maliput::common::road_network_description_parser_error);
}

std::string GetSignalReferenceMissingS() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<signalReference id="signal_1" orientation="+" t="5.0">
    <validity fromLane="1" toLane="3"/>
</signalReference>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSignalReferenceMissingS) {
  const std::string xml_description = GetSignalReferenceMissingS();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, SignalReference::kSignalReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(SignalReference::kSignalReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<SignalReference>(), maliput::common::road_network_description_parser_error);
}

std::string GetSignalReferenceMissingT() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<signalReference id="signal_1" orientation="+" s="10.0">
    <validity fromLane="1" toLane="3"/>
</signalReference>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSignalReferenceMissingT) {
  const std::string xml_description = GetSignalReferenceMissingT();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, SignalReference::kSignalReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(SignalReference::kSignalReferenceTag, dut.GetName());
  EXPECT_THROW(dut.As<SignalReference>(), maliput::common::road_network_description_parser_error);
}

std::string GetSignalReferenceMissingValidity() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<signalReference id="signal_1" orientation="+" s="10.0" t="5.0">
</signalReference>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSignalReferenceMissingValidity) {
  const SignalReference kExpectedSignalReference{
      SignalReference::SignalId("signal_1"), SignalReference::Orientation::kWithS, 10.0, 5.0, {}};

  const std::string xml_description = GetSignalReferenceMissingValidity();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, SignalReference::kSignalReferenceTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(SignalReference::kSignalReferenceTag, dut.GetName());
  const SignalReference signal_reference = dut.As<SignalReference>();
  EXPECT_EQ(kExpectedSignalReference, signal_reference);
}

// Get a XML description that contains a basic Dependency node.
std::string GetBasicDependency() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<dependency id="dep_signal_1" type="dependency_type"/>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::Dependency` parsing.
TEST_F(SignalParsingTests, NodeParserDependency) {
  const Dependency kExpectedDependency{Dependency::SignalId("dep_signal_1"), std::make_optional("dependency_type")};

  const std::string xml_description = GetBasicDependency();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Dependency::kDependencyTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Dependency::kDependencyTag, dut.GetName());
  const Dependency dependency = dut.As<Dependency>();
  EXPECT_EQ(kExpectedDependency, dependency);
}

std::string GetDependencyMissingId() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<dependency type="dependency_type"/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserDependencyMissingId) {
  const std::string xml_description = GetDependencyMissingId();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Dependency::kDependencyTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Dependency::kDependencyTag, dut.GetName());
  EXPECT_THROW(dut.As<Dependency>(), maliput::common::road_network_description_parser_error);
}

std::string GetDependencyMissingType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<dependency id="dep_signal_1"/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserDependencyMissingType) {
  const Dependency kExpectedDependency{Dependency::SignalId("dep_signal_1"), std::nullopt};

  const std::string xml_description = GetDependencyMissingType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Dependency::kDependencyTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Dependency::kDependencyTag, dut.GetName());
  const Dependency dependency = dut.As<Dependency>();
  EXPECT_EQ(kExpectedDependency, dependency);
}

// Get a XML description that contains a basic Control node.
std::string GetBasicControl() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<control signalId="control_signal" type="control_type"/>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::Control` parsing.
TEST_F(SignalParsingTests, NodeParserControl) {
  const Control kExpectedControl{Control::SignalId("control_signal"), std::make_optional("control_type")};

  const std::string xml_description = GetBasicControl();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Control::kControlTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Control::kControlTag, dut.GetName());
  const Control control = dut.As<Control>();
  EXPECT_EQ(kExpectedControl, control);
}

std::string GetControlMissingId() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<control type="control_type"/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserControlMissingId) {
  const std::string xml_description = GetControlMissingId();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Control::kControlTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Control::kControlTag, dut.GetName());
  EXPECT_THROW(dut.As<Control>(), maliput::common::road_network_description_parser_error);
}

std::string GetControlMissingType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<control signalId="control_signal"/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserControlMissingType) {
  const Control kExpectedControl{Control::SignalId("control_signal"), std::nullopt};

  const std::string xml_description = GetControlMissingType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Control::kControlTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Control::kControlTag, dut.GetName());
  const Control control = dut.As<Control>();
  EXPECT_EQ(kExpectedControl, control);
}

// Get a XML description that contains a basic Controller node.
std::string GetBasicController() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<controller id="controller_1" name="MyController" sequence="1">
    <control signalId="signal_1" type="control_type"/>
    <control signalId="signal_2"/>
</controller>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::Controller` parsing.
TEST_F(SignalParsingTests, NodeParserController) {
  const Control kControl1{Control::SignalId("signal_1"), std::make_optional("control_type")};
  const Control kControl2{Control::SignalId("signal_2"), std::nullopt};
  const Controller kExpectedController{Controller::Id("controller_1"),
                                       std::make_optional("MyController"),
                                       std::make_optional(1),
                                       {{kControl1, kControl2}}};

  const std::string xml_description = GetBasicController();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Controller::kControllerTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Controller::kControllerTag, dut.GetName());
  const Controller controller = dut.As<Controller>();
  EXPECT_EQ(kExpectedController, controller);
}

std::string GetControllerMissingId() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<controller name="MyController" sequence="1">
    <control signalId="signal_1" type="control_type"/>
    <control signalId="signal_2"/>
</controller>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserControllerMissingId) {
  const std::string xml_description = GetControllerMissingId();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Controller::kControllerTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Controller::kControllerTag, dut.GetName());
  EXPECT_THROW(dut.As<Controller>(), maliput::common::road_network_description_parser_error);
}

std::string GetControllerMissingNameAndSequence() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<controller id="controller_1">
    <control signalId="signal_1" type="control_type"/>
    <control signalId="signal_2"/>
</controller>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserControllerMissingNameAndSequence) {
  const Control kControl1{Control::SignalId("signal_1"), std::make_optional("control_type")};
  const Control kControl2{Control::SignalId("signal_2"), std::nullopt};
  const Controller kExpectedController{Controller::Id("controller_1"), std::nullopt, std::nullopt, {{kControl1, kControl2}}};

  const std::string xml_description = GetControllerMissingNameAndSequence();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Controller::kControllerTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Controller::kControllerTag, dut.GetName());
  const Controller controller = dut.As<Controller>();
  EXPECT_EQ(kExpectedController, controller);
}

std::string GetControllerMissingControls() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<controller id="controller_1" name="MyController" sequence="1">
</controller>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserControllerMissingControls) {
  const std::string xml_description = GetControllerMissingControls();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, Controller::kControllerTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(Controller::kControllerTag, dut.GetName());
  EXPECT_THROW(dut.As<Controller>(), maliput::common::road_network_description_parser_error);
}

// Get a XML description that contains a basic Semantics node.
std::string GetBasicSemantics() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <speed type="maximum" value="60" unit="km/h"/>
    <lane type="noOvertakeCars"/>
    <priority type="stop"/>
    <prohibited/>
    <warning/>
    <routing/>
    <streetname/>
    <parking/>
    <tourist/>
    <supplementaryAllows/>
    <supplementaryProhibits/>
    <supplementaryExplanatory/>
    <supplementaryTime type="time" value="12.0"/>
    <supplementaryDistance type="for" unit="m" value="100"/>
    <supplementaryEnvironment type="rain"/>
    </semantics>)R";
  ss << "</root>";
  return ss.str();
}

// Tests `signal::Semantics` parsing.
TEST_F(SignalParsingTests, NodeParserSemantics) {
  const Semantics kExpectedSemantics{
      {Semantics::Speed{Semantics::SemanticsSpeed::kMaximum, Semantics::UnitSpeed::kKmh, 60.0}},
      {Semantics::Lane{Semantics::SemanticsLane::kNoOvertakeCars}},
      {Semantics::Priority{Semantics::SemanticsPriority::kStop}},
      {Semantics::Prohibited{}},
      {Semantics::Warning{}},
      {Semantics::Routing{}},
      {Semantics::StreetName{}},
      {Semantics::Parking{}},
      {Semantics::Tourist{}},
      {Semantics::SupplementaryTime{Semantics::SemanticsSupplementaryTime::kTime, 12.0}},
      {Semantics::SupplementaryAllows{}},
      {Semantics::SupplementaryProhibits{}},
      {Semantics::SupplementaryDistance{Semantics::SemanticsSupplementaryDistance::kFor, Semantics::UnitDistance::kM,
                                        100.0}},
      {Semantics::SupplementaryEnvironment{Semantics::SemanticsSupplementaryEnvironment::kRain}},
      {Semantics::SupplementaryExplanatory{}}};

  const std::string xml_description = GetBasicSemantics();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  const Semantics semantics = dut.As<signal::Semantics>();
  EXPECT_EQ(kExpectedSemantics, semantics);
}

std::string GetEmptySemantics() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics/>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserEmptySemantics) {
  const std::string xml_description = GetEmptySemantics();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  const Semantics semantics = dut.As<signal::Semantics>();
  EXPECT_TRUE(semantics.speeds.empty());
  EXPECT_TRUE(semantics.lanes.empty());
  EXPECT_TRUE(semantics.priorities.empty());
  EXPECT_TRUE(semantics.prohibited.empty());
  EXPECT_TRUE(semantics.warnings.empty());
  EXPECT_TRUE(semantics.routings.empty());
  EXPECT_TRUE(semantics.street_names.empty());
  EXPECT_TRUE(semantics.parkings.empty());
  EXPECT_TRUE(semantics.tourists.empty());
  EXPECT_TRUE(semantics.supplementary_allows.empty());
  EXPECT_TRUE(semantics.supplementary_prohibits.empty());
  EXPECT_TRUE(semantics.supplementary_explanatories.empty());
  EXPECT_TRUE(semantics.supplementary_times.empty());
  EXPECT_TRUE(semantics.supplementary_distances.empty());
  EXPECT_TRUE(semantics.supplementary_environments.empty());
}

std::string GetSemanticsMissingSpeedValue() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <speed type="maximum" unit="km/h"/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSpeedValue) {
  const std::string xml_description = GetSemanticsMissingSpeedValue();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingSpeedType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <speed value="60" unit="km/h"/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSpeedType) {
  const std::string xml_description = GetSemanticsMissingSpeedType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingLaneType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <lane/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingLaneType) {
  const std::string xml_description = GetSemanticsMissingLaneType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingPriorityType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <priority/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsSpeedMissingPriorityType) {
  const std::string xml_description = GetSemanticsMissingPriorityType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingSupplementaryTimeType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <supplementaryTime value="12.0"/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSupplementaryTimeType) {
  const std::string xml_description = GetSemanticsMissingSupplementaryTimeType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingSupplementaryTimeValue() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <supplementaryTime type="time"/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSupplementaryTimeValue) {
  const std::string xml_description = GetSemanticsMissingSupplementaryTimeValue();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingSupplementaryDistanceType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <supplementaryDistance value="100" unit="m"/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSupplementaryDistanceType) {
  const std::string xml_description = GetSemanticsMissingSupplementaryDistanceType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingSupplementaryDistanceValue() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <supplementaryDistance type="for" unit="m"/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSupplementaryDistanceValue) {
  const std::string xml_description = GetSemanticsMissingSupplementaryDistanceValue();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingSupplementaryDistanceUnit() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <supplementaryDistance type="for" value="100"/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSupplementaryDistanceUnit) {
  const std::string xml_description = GetSemanticsMissingSupplementaryDistanceUnit();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

std::string GetSemanticsMissingSupplementaryEnvironmentType() {
  std::stringstream ss;
  ss << "<root>";
  ss << R"R(<semantics>
    <supplementaryEnvironment/>
</semantics>)R";
  ss << "</root>";
  return ss.str();
}

TEST_F(SignalParsingTests, NodeParserSemanticsMissingSupplementaryEnvironmentType) {
  const std::string xml_description = GetSemanticsMissingSupplementaryEnvironmentType();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, signal::Semantics::kSemanticsTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(signal::Semantics::kSemanticsTag, dut.GetName());
  EXPECT_THROW(dut.As<signal::Semantics>(), maliput::common::road_network_description_parser_error);
}

}  // namespace
}  // namespace test
}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
