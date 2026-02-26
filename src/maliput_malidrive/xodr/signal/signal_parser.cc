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
#include "maliput_malidrive/xodr/signal/semantics.h"
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

// Specialization to parse `signal::Semantics::Speed`'s node.
template <>
signal::Semantics::Speed NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Parse type
  const auto type_str = attribute_parser.As<std::string>(signal::Semantics::Speed::kType);
  MALIDRIVE_THROW_UNLESS(type_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::SemanticsSpeed type = signal::Semantics::SemanticsSpeed::kUnknown;
  if (type_str.value() == "maximum")
    type = signal::Semantics::SemanticsSpeed::kMaximum;
  else if (type_str.value() == "maximumEnd")
    type = signal::Semantics::SemanticsSpeed::kMaximumEnd;
  else if (type_str.value() == "minimum")
    type = signal::Semantics::SemanticsSpeed::kMinimum;
  else if (type_str.value() == "minimumEnd")
    type = signal::Semantics::SemanticsSpeed::kMinimumEnd;
  else if (type_str.value() == "recommended")
    type = signal::Semantics::SemanticsSpeed::kRecommended;
  else if (type_str.value() == "recommendedEnd")
    type = signal::Semantics::SemanticsSpeed::kRecommendedEnd;
  else if (type_str.value() == "zone")
    type = signal::Semantics::SemanticsSpeed::Zone;
  else if (type_str.value() == "zoneEnd")
    type = signal::Semantics::SemanticsSpeed::ZoneEnd;
  else
    type = signal::Semantics::SemanticsSpeed::kUnknown;

  // Parse unit
  const auto unit_str = attribute_parser.As<std::string>(signal::Semantics::Speed::kUnit);
  MALIDRIVE_THROW_UNLESS(unit_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::UnitSpeed unit = signal::Semantics::UnitSpeed::kUnknown;
  if (unit_str.value() == "m/s")
    unit = signal::Semantics::UnitSpeed::kMS;
  else if (unit_str.value() == "mph")
    unit = signal::Semantics::UnitSpeed::kMph;
  else if (unit_str.value() == "km/h")
    unit = signal::Semantics::UnitSpeed::kKmh;

  // Parse value
  const auto value_opt = attribute_parser.As<double>(signal::Semantics::Speed::kValue);
  MALIDRIVE_THROW_UNLESS(value_opt != std::nullopt, maliput::common::road_network_description_parser_error);
  const double value = ValidateDouble(value_opt, kDontAllowNan);

  return {type, unit, value};
}

// Specialization to parse `signal::Semantics::Lane`'s node.
template <>
signal::Semantics::Lane NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  const auto type_str = attribute_parser.As<std::string>(signal::Semantics::Lane::kType);
  MALIDRIVE_THROW_UNLESS(type_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::SemanticsLane type = signal::Semantics::SemanticsLane::kUnknown;
  if (type_str.value() == "noOvertakeCarsEnd")
    type = signal::Semantics::SemanticsLane::kNoOvertakeCarsEnd;
  else if (type_str.value() == "noOvertakeCars")
    type = signal::Semantics::SemanticsLane::kNoOvertakeCars;
  else if (type_str.value() == "noOvertakeTrucksEnd")
    type = signal::Semantics::SemanticsLane::kNoOvertakeTrucksEnd;
  else if (type_str.value() == "noOvertakeTrucks")
    type = signal::Semantics::SemanticsLane::kNoOvertakeTrucks;
  else if (type_str.value() == "priorityOverOncoming")
    type = signal::Semantics::SemanticsLane::kPriorityOverOncoming;
  else if (type_str.value() == "roundabout")
    type = signal::Semantics::SemanticsLane::kRoundabout;
  else if (type_str.value() == "yieldForOncoming")
    type = signal::Semantics::SemanticsLane::kYieldForOncoming;
  else
    type = signal::Semantics::SemanticsLane::kUnknown;

  return {type};
}

// Specialization to parse `signal::Semantics::Priority`'s node.
template <>
signal::Semantics::Priority NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  const auto type_str = attribute_parser.As<std::string>(signal::Semantics::Priority::kType);
  MALIDRIVE_THROW_UNLESS(type_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::SemanticsPriority type = signal::Semantics::SemanticsPriority::kUnknown;
  if (type_str.value() == "4Way")
    type = signal::Semantics::SemanticsPriority::k4Way;
  else if (type_str.value() == "keepClearLine")
    type = signal::Semantics::SemanticsPriority::kKeepClearLine;
  else if (type_str.value() == "noParkingLine")
    type = signal::Semantics::SemanticsPriority::kNoParkingLine;
  else if (type_str.value() == "noTurnOnRed")
    type = signal::Semantics::SemanticsPriority::kNoTurnOnRed;
  else if (type_str.value() == "priorityRoadEnd")
    type = signal::Semantics::SemanticsPriority::kPriorityRoadEnd;
  else if (type_str.value() == "priorityRoad")
    type = signal::Semantics::SemanticsPriority::kPriorityRoad;
  else if (type_str.value() == "priorityToTheRightRule")
    type = signal::Semantics::SemanticsPriority::kPriorityToTheRightRule;
  else if (type_str.value() == "stopLine")
    type = signal::Semantics::SemanticsPriority::kStopLine;
  else if (type_str.value() == "stop")
    type = signal::Semantics::SemanticsPriority::kStop;
  else if (type_str.value() == "trafficLight")
    type = signal::Semantics::SemanticsPriority::kTrafficLight;
  else if (type_str.value() == "turnOnRedAllowed")
    type = signal::Semantics::SemanticsPriority::kTurnOnRedAllowed;
  else if (type_str.value() == "waitingLine")
    type = signal::Semantics::SemanticsPriority::kWaitingLine;
  else if (type_str.value() == "yield")
    type = signal::Semantics::SemanticsPriority::kYield;
  else
    type = signal::Semantics::SemanticsPriority::kUnknown;

  return {type};
}

// Specializations for Semantics empty elements (tags that convey meaning just by being present)
template <>
signal::Semantics::Prohibited NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::Warning NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::Routing NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::StreetName NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::Parking NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::Tourist NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::SupplementaryAllows NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::SupplementaryProhibits NodeParser::As() const {
  return {};
}

template <>
signal::Semantics::SupplementaryExplanatory NodeParser::As() const {
  return {};
}

// Specialization to parse `signal::Semantics::SupplementaryTime`'s node.
template <>
signal::Semantics::SupplementaryTime NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  const auto type_str = attribute_parser.As<std::string>(signal::Semantics::SupplementaryTime::kType);
  MALIDRIVE_THROW_UNLESS(type_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::SemanticsSupplementaryTime type = signal::Semantics::SemanticsSupplementaryTime::kUnknown;
  if (type_str.value() == "time")
    type = signal::Semantics::SemanticsSupplementaryTime::kTime;
  else if (type_str.value() == "day")
    type = signal::Semantics::SemanticsSupplementaryTime::kDay;
  else
    type = signal::Semantics::SemanticsSupplementaryTime::kUnknown;

  const auto value_opt = attribute_parser.As<double>(signal::Semantics::SupplementaryTime::kValue);
  MALIDRIVE_THROW_UNLESS(value_opt != std::nullopt, maliput::common::road_network_description_parser_error);
  const double value = ValidateDouble(value_opt, kDontAllowNan);

  return {type, value};
}

// Specialization to parse `signal::Semantics::SupplementaryDistance`'s node.
template <>
signal::Semantics::SupplementaryDistance NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Parse type
  const auto type_str = attribute_parser.As<std::string>(signal::Semantics::SupplementaryDistance::kType);
  MALIDRIVE_THROW_UNLESS(type_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::SemanticsSupplementaryDistance type = signal::Semantics::SemanticsSupplementaryDistance::kUnknown;
  if (type_str.value() == "for")
    type = signal::Semantics::SemanticsSupplementaryDistance::kFor;
  else if (type_str.value() == "in")
    type = signal::Semantics::SemanticsSupplementaryDistance::kIn;
  else
    type = signal::Semantics::SemanticsSupplementaryDistance::kUnknown;

  // Parse unit
  const auto unit_str = attribute_parser.As<std::string>(signal::Semantics::SupplementaryDistance::kUnit);
  MALIDRIVE_THROW_UNLESS(unit_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::UnitDistance unit = signal::Semantics::UnitDistance::kUnknown;
  if (unit_str.value() == "m")
    unit = signal::Semantics::UnitDistance::kM;
  else if (unit_str.value() == "km")
    unit = signal::Semantics::UnitDistance::kKm;
  else if (unit_str.value() == "ft")
    unit = signal::Semantics::UnitDistance::kFt;
  else if (unit_str.value() == "mile")
    unit = signal::Semantics::UnitDistance::kMile;

  // Parse value
  const auto value_opt = attribute_parser.As<double>(signal::Semantics::SupplementaryDistance::kValue);
  MALIDRIVE_THROW_UNLESS(value_opt != std::nullopt, maliput::common::road_network_description_parser_error);
  const double value = ValidateDouble(value_opt, kDontAllowNan);

  return {type, unit, value};
}

// Specialization to parse `signal::Semantics::SupplementaryEnvironment`'s node.
template <>
signal::Semantics::SupplementaryEnvironment NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  const auto type_str = attribute_parser.As<std::string>(signal::Semantics::SupplementaryEnvironment::kType);
  MALIDRIVE_THROW_UNLESS(type_str != std::nullopt, maliput::common::road_network_description_parser_error);
  signal::Semantics::SemanticsSupplementaryEnvironment type =
      signal::Semantics::SemanticsSupplementaryEnvironment::kUnknown;
  if (type_str.value() == "rain")
    type = signal::Semantics::SemanticsSupplementaryEnvironment::kRain;
  else if (type_str.value() == "snow")
    type = signal::Semantics::SemanticsSupplementaryEnvironment::kSnow;
  else if (type_str.value() == "fog")
    type = signal::Semantics::SemanticsSupplementaryEnvironment::kFog;
  else
    type = signal::Semantics::SemanticsSupplementaryEnvironment::kUnknown;

  return {type};
}

// Specialization to parse `signal::Semantics`'s node.
template <>
signal::Semantics NodeParser::As() const {
  signal::Semantics semantics;

  // Helper lambda to parse 0..* element lists
  auto parse_elements = [&](const char* tag_name, auto& target_vector) {
    using ElementType = typename std::decay_t<decltype(target_vector)>::value_type;
    tinyxml2::XMLElement* child_xml = element_->FirstChildElement(tag_name);
    while (child_xml) {
      target_vector.push_back(NodeParser(child_xml, parser_configuration_).As<ElementType>());
      child_xml = child_xml->NextSiblingElement(tag_name);
    }
  };

  parse_elements(signal::Semantics::Speed::kSpeedTag, semantics.speeds);
  parse_elements(signal::Semantics::Lane::kLaneTag, semantics.lanes);
  parse_elements(signal::Semantics::Priority::kPriorityTag, semantics.priorities);
  parse_elements(signal::Semantics::Prohibited::kProhibitedTag, semantics.prohibited);
  parse_elements(signal::Semantics::Warning::kWarningTag, semantics.warnings);
  parse_elements(signal::Semantics::Routing::kRoutingTag, semantics.routings);
  parse_elements(signal::Semantics::StreetName::kStreetNameTag, semantics.street_names);
  parse_elements(signal::Semantics::Parking::kParkingTag, semantics.parkings);
  parse_elements(signal::Semantics::Tourist::kTouristTag, semantics.tourists);
  parse_elements(signal::Semantics::SupplementaryTime::kSupplementaryTimeTag, semantics.supplementary_times);
  parse_elements(signal::Semantics::SupplementaryAllows::kSupplementaryAllowsTag, semantics.supplementary_allows);
  parse_elements(signal::Semantics::SupplementaryProhibits::kSupplementaryProhibitsTag,
                 semantics.supplementary_prohibits);
  parse_elements(signal::Semantics::SupplementaryDistance::kSupplementaryDistanceTag,
                 semantics.supplementary_distances);
  parse_elements(signal::Semantics::SupplementaryEnvironment::kSupplementaryEnvironmentTag,
                 semantics.supplementary_environments);
  parse_elements(signal::Semantics::SupplementaryExplanatory::kSupplementaryExplanatoryTag,
                 semantics.supplementary_explanatories);

  return semantics;
}

}  // namespace xodr
}  // namespace malidrive
