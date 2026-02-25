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

// Tests `signal::DisplayArea` parsing.
TEST_F(SignalParsingTests, NodeParserDisplayArea) {
  const DisplayArea kExpectedDisplayArea{2.5, 1, 0.75, 1.8, 0.5};

  const std::string xml_description = GetBasicDisplayArea();
  const NodeParser dut(LoadXMLAndGetNodeByName(xml_description, DisplayArea::kDisplayAreaTag),
                       {kNullParserSTolerance, kDontAllowSchemaErrors, kDontAllowSemanticErrors});
  EXPECT_EQ(DisplayArea::kDisplayAreaTag, dut.GetName());
  const DisplayArea display_area = dut.As<DisplayArea>();
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
  const DisplayArea kDisplayArea{2.0, 0, 0.25, 1.5, 0.3};
  const VmsBoard kExpectedVmsBoard{std::make_optional(3.0), DisplayType::kLed, std::make_optional(5.0), 0.5, 1.2,
                                   {{kDisplayArea}}};

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

}  // namespace
}  // namespace test
}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
