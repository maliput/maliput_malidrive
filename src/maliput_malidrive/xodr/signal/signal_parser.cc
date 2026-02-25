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
#include "maliput_malidrive/xodr/signal/controller.h"
#include "maliput_malidrive/xodr/signal/dependency.h"
#include "maliput_malidrive/xodr/signal/reference.h"
#include "maliput_malidrive/xodr/signal/signal.h"
#include "maliput_malidrive/xodr/signal/signal_reference.h"

namespace malidrive {
namespace xodr {

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

}  // namespace xodr
}  // namespace malidrive