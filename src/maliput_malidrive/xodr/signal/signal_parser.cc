// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include <map>
#include <vector>

#include <maliput/common/logger.h>

#include "maliput_malidrive/xodr/parser.h"
#include "maliput_malidrive/xodr/signal/board.h"
#include "maliput_malidrive/xodr/signal/controller.h"
#include "maliput_malidrive/xodr/signal/dependency.h"
#include "maliput_malidrive/xodr/signal/reference.h"
#include "maliput_malidrive/xodr/signal/sign.h"
#include "maliput_malidrive/xodr/signal/signal.h"
#include "maliput_malidrive/xodr/signal/signal_reference.h"

namespace malidrive {
namespace xodr {
namespace {

// {@ Constants used during parsing process.
static constexpr bool kDontAllowNan{false};
// }@

// Validates that `value` contains a valid double-type value.
//
// @param value double value
// @param allow_nan Determines whether to accept having NaN values or not.
// @returns The validated value.
//
// @throws maliput::common::road_network_description_parser_error When `value` is std::nullopt.
// @throws maliput::common::road_network_description_parser_error When `value.value()` is a NaN value iff `allow_nan` is
// false.
double ValidateDouble(const std::optional<double>& value, bool allow_nan) {
  MALIDRIVE_THROW_UNLESS(value != std::nullopt, maliput::common::road_network_description_parser_error);
  if (!allow_nan) {
    MALIDRIVE_THROW_UNLESS(!std::isnan(value.value()), maliput::common::road_network_description_parser_error);
  }
  return value.value();
}

}  // namespace

// Specialization to parse `signal::Control`'s node
template <>
signal::Control NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto signal_id = attribute_parser.As<std::string>(signal::Control::kSignalId);
  MALIDRIVE_THROW_UNLESS(signal_id != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto type = attribute_parser.As<std::string>(signal::Control::kType);
  // @}

  return {signal::Control::SignalId(signal_id.value_or("none")), type};
}

// Specialization to parse `signal::Controller`'s node
template <>
signal::Controller NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(signal::Controller::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto name = attribute_parser.As<std::string>(signal::Controller::kName);
  const auto sequence = attribute_parser.As<int>(signal::Controller::kSequence);
  if (sequence.has_value()) {
    MALIDRIVE_THROW_UNLESS(sequence.value() >= 0, maliput::common::road_network_description_parser_error);
  }
  // @}

  // Control elements
  tinyxml2::XMLElement* control_element_xml = element_->FirstChildElement(signal::Control::kControlTag);
  std::vector<signal::Control> controls;
  while (control_element_xml) {
    auto control = NodeParser(control_element_xml, parser_configuration_).As<signal::Control>();
    controls.push_back(control);
    control_element_xml = control_element_xml->NextSiblingElement(signal::Control::kControlTag);
  }
  MALIDRIVE_THROW_UNLESS(!controls.empty(), maliput::common::road_network_description_parser_error);

  return {signal::Controller::Id(id.value_or("none")), name, sequence, controls};
}

// Specialization to parse `signal::Dependency`'s node
template <>
signal::Dependency NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto signal_id = attribute_parser.As<std::string>(signal::Dependency::kSignalId);
  MALIDRIVE_THROW_UNLESS(signal_id != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto type = attribute_parser.As<std::string>(signal::Control::kType);
  // @}

  return {signal::Dependency::SignalId(signal_id.value_or("none")), type};
}

// Specialization to parse `signal::SignalReference`'s node
template <>
signal::SignalReference NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto signal_id = attribute_parser.As<std::string>(signal::SignalReference::kSignalId);
  MALIDRIVE_THROW_UNLESS(signal_id != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto orientation_str = attribute_parser.As<std::string>(signal::SignalReference::kOrientation);
  MALIDRIVE_THROW_UNLESS(orientation_str != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s = attribute_parser.As<double>(signal::SignalReference::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(s.value() >= 0, maliput::common::road_network_description_parser_error);
  const auto t = attribute_parser.As<double>(signal::SignalReference::kT);
  MALIDRIVE_THROW_UNLESS(t != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  signal::SignalReference::Orientation orientation;
  if (orientation_str.value() == "+") {
    orientation = signal::SignalReference::Orientation::kWithS;
  } else if (orientation_str.value() == "-") {
    orientation = signal::SignalReference::Orientation::kAgainstS;
  } else if (orientation_str.value() == "none") {
    orientation = signal::SignalReference::Orientation::kBidirectional;
  } else {
    MALIDRIVE_THROW_MESSAGE("Orientation must be \"+\", \"-\", or \"none\".",
                            maliput::common::road_network_description_parser_error);
  }

  // Validity elements
  tinyxml2::XMLElement* validity_element_xml = element_->FirstChildElement(Validity::kValidityTag);
  std::vector<Validity> validities;
  while (validity_element_xml) {
    auto validity = NodeParser(validity_element_xml, parser_configuration_).As<Validity>();
    validities.push_back(validity);
    validity_element_xml = validity_element_xml->NextSiblingElement(Validity::kValidityTag);
  }
  MALIDRIVE_THROW_UNLESS(!validities.empty(), maliput::common::road_network_description_parser_error);

  return {signal::SignalReference::SignalId(signal_id.value_or("none")), orientation, s.value(), t.value(), validities};
}

// Specialization to parse `signal::Reference`'s node
template <>
signal::Reference NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto element_id = attribute_parser.As<std::string>(signal::Reference::kElementId);
  MALIDRIVE_THROW_UNLESS(element_id != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto element_type_str = attribute_parser.As<std::string>(signal::Reference::kElementType);
  MALIDRIVE_THROW_UNLESS(element_type_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Reference::ElementType element_type;
  if (element_type_str.value() == "object") {
    element_type = signal::Reference::ElementType::kObject;
  } else if (element_type_str.value() == "signal") {
    element_type = signal::Reference::ElementType::kSignal;
  } else {
    MALIDRIVE_THROW_MESSAGE("ElementType must be \"object\" or \"signal\".",
                            maliput::common::road_network_description_parser_error);
  }
  // @}

  // Optional attributes.
  // @{
  const auto type = attribute_parser.As<std::string>(signal::Reference::kType);
  // @}

  return {element_id.value_or("none"), element_type, type};
}

// Specialization to parse `signal::Sign`'s node.
template <>
signal::Sign NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  signal::Signal signal = NodeParser(element_, parser_configuration_).As<signal::Signal>();
  // Non-optional attributes.
  // @{
  const double v = ValidateDouble(attribute_parser.As<double>(signal::Sign::kV), kDontAllowNan);
  const double z = ValidateDouble(attribute_parser.As<double>(signal::Sign::kZ), kDontAllowNan);
  // @}

  return {signal, v, z};
}

// Specialization to parse `signal::DisplayArea`'s node.
template <>
signal::DisplayArea NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const double height = ValidateDouble(attribute_parser.As<double>(signal::DisplayArea::kHeight), kDontAllowNan);
  const int index = attribute_parser.As<int>(signal::DisplayArea::kIndex).value_or(0);
  const double v = ValidateDouble(attribute_parser.As<double>(signal::DisplayArea::kV), kDontAllowNan);
  const double width = ValidateDouble(attribute_parser.As<double>(signal::DisplayArea::kWidth), kDontAllowNan);
  const double z = ValidateDouble(attribute_parser.As<double>(signal::DisplayArea::kZ), kDontAllowNan);
  // @}

  return {height, index, v, width, z};
}

// Specialization to parse `signal::VmsBoard`'s node.
template <>
signal::VmsBoard NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto display_type_str = attribute_parser.As<std::string>(signal::VmsBoard::kDisplayType);
  MALIDRIVE_THROW_UNLESS(display_type_str != std::nullopt, maliput::common::road_network_description_parser_error);

  signal::DisplayType display_type;
  if (display_type_str.value() == "led") {
    display_type = signal::DisplayType::kLed;
  } else if (display_type_str.value() == "monochromeGraphic") {
    display_type = signal::DisplayType::kMonochromeGraphic;
  } else if (display_type_str.value() == "rotatingPrismHorizontal") {
    display_type = signal::DisplayType::kRotatingPrismHorizontal;
  } else if (display_type_str.value() == "rotatingPrismVertical") {
    display_type = signal::DisplayType::kRotatingPrismVertical;
  } else if (display_type_str.value() == "simpleMatrix") {
    display_type = signal::DisplayType::kSimpleMatrix;
  } else {
    display_type = signal::DisplayType::kOther;
  }

  const double v = ValidateDouble(attribute_parser.As<double>(signal::VmsBoard::kV), kDontAllowNan);
  const double z = ValidateDouble(attribute_parser.As<double>(signal::VmsBoard::kZ), kDontAllowNan);
  // @}

  // Optional attributes.
  // @{
  const auto display_height = attribute_parser.As<double>(signal::VmsBoard::kDisplayHeight);
  const auto display_width = attribute_parser.As<double>(signal::VmsBoard::kDisplayWidth);
  // @}

  // DisplayArea elements
  tinyxml2::XMLElement* display_area_element_xml = element_->FirstChildElement(signal::DisplayArea::kDisplayAreaTag);
  std::vector<signal::DisplayArea> display_areas;
  while (display_area_element_xml) {
    auto display_area = NodeParser(display_area_element_xml, parser_configuration_).As<signal::DisplayArea>();
    display_areas.push_back(display_area);
    display_area_element_xml = display_area_element_xml->NextSiblingElement(signal::DisplayArea::kDisplayAreaTag);
  }

  return {display_height, display_type, display_width, v, z, display_areas};
}

// Specialization to parse `signal::StaticBoard`'s node.
template <>
signal::StaticBoard NodeParser::As() const {
  // Sign elements
  tinyxml2::XMLElement* sign_element_xml = element_->FirstChildElement(signal::Sign::kSignTag);
  std::vector<signal::Sign> signs;
  while (sign_element_xml) {
    auto sign = NodeParser(sign_element_xml, parser_configuration_).As<signal::Sign>();
    signs.push_back(sign);
    sign_element_xml = sign_element_xml->NextSiblingElement(signal::Sign::kSignTag);
  }

  return {signs};
}

}  // namespace xodr
}  // namespace malidrive