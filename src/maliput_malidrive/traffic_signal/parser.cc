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
#include "maliput_malidrive/traffic_signal/parser.h"

#include <utility>

#include <maliput/common/maliput_throw.h>
#include <yaml-cpp/yaml.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/traffic_signal/yaml_helper.h"

namespace malidrive {
namespace traffic_signal {

namespace {

// Helper function to convert a color string to BulbColor enum.
// @param color_str The color string (e.g., "Red", "Yellow", "Green").
// @return The corresponding BulbColor enum value.
// @throws std::runtime_error if the color string is invalid.
maliput::api::rules::BulbColor StringToBulbColor(const std::string& color_str) {
  const auto mapper = maliput::api::rules::BulbColorMapper();
  for (const auto& pair : mapper) {
    if (pair.second == color_str) {
      return pair.first;
    }
  }
  MALIDRIVE_THROW_MESSAGE("Invalid bulb color: " + color_str, maliput::common::road_network_description_parser_error);
}

// Helper function to convert a type string to BulbType enum.
// @param type_str The type string (e.g., "Round", "Arrow").
// @return The corresponding BulbType enum value.
// @throws std::runtime_error if the type string is invalid.
maliput::api::rules::BulbType StringToBulbType(const std::string& type_str) {
  const auto mapper = maliput::api::rules::BulbTypeMapper();
  for (const auto& pair : mapper) {
    if (pair.second == type_str) {
      return pair.first;
    }
  }
  MALIDRIVE_THROW_MESSAGE("Invalid bulb type: " + type_str, maliput::common::road_network_description_parser_error);
}

// Helper function to convert a state string to BulbState enum.
// @param state_str The state string (e.g., "Off", "On", "Blinking").
// @return The corresponding BulbState enum value.
// @throws std::runtime_error if the state string is invalid.
maliput::api::rules::BulbState StringToBulbState(const std::string& state_str) {
  const auto mapper = maliput::api::rules::BulbStateMapper();
  for (const auto& pair : mapper) {
    if (pair.second == state_str) {
      return pair.first;
    }
  }
  MALIDRIVE_THROW_MESSAGE("Invalid bulb state: " + state_str, maliput::common::road_network_description_parser_error);
}

// Helper function to parse a bounding box from a YAML map.
// @param node The YAML node containing p_min and p_max keys.
// @return A Bulb::BoundingBox with the parsed values.
maliput::api::rules::Bulb::BoundingBox ParseBoundingBox(const YAML::Node& node) {
  auto bounding_box = maliput::api::rules::Bulb::BoundingBox();
  if (!node.IsDefined()) {
    // If not defined, return the default bounding box.
    return bounding_box;
  }
  MALIDRIVE_THROW_UNLESS(node.IsMap(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(node[BoundingBoxConstants::kPMin].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(node[BoundingBoxConstants::kPMax].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  const auto p_min = GetVector3(node[BoundingBoxConstants::kPMin], BoundingBoxConstants::kPMin);
  const auto p_max = GetVector3(node[BoundingBoxConstants::kPMax], BoundingBoxConstants::kPMax);

  bounding_box.p_BMin = p_min;
  bounding_box.p_BMax = p_max;

  return bounding_box;
}

// Helper function to parse a BulbDefinition from a YAML map.
// @param bulb_node The YAML node representing a bulb.
// @return A BulbDefinition with the parsed values.
BulbDefinition ParseBulb(const YAML::Node& bulb_node) {
  MALIDRIVE_THROW_UNLESS(bulb_node.IsMap(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(bulb_node[BulbConstants::kId].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(bulb_node[BulbConstants::kColor].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(bulb_node[BulbConstants::kType].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(bulb_node[BulbConstants::kPositionBulbGroup].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(bulb_node[BulbConstants::kOrientationBulbGroup].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  BulbDefinition bulb;
  bulb.id = GetRequiredStringField(bulb_node, BulbConstants::kId);
  bulb.color = StringToBulbColor(GetRequiredStringField(bulb_node, BulbConstants::kColor));
  bulb.type = StringToBulbType(GetRequiredStringField(bulb_node, BulbConstants::kType));
  bulb.position_bulb_group =
      GetVector3(bulb_node[BulbConstants::kPositionBulbGroup], BulbConstants::kPositionBulbGroup);
  bulb.orientation_bulb_group =
      GetQuaternion(bulb_node[BulbConstants::kOrientationBulbGroup], BulbConstants::kOrientationBulbGroup);

  // Parse bulb states (optional, defaults to ["Off", "On"]).
  if (bulb_node[BulbConstants::kStates].IsDefined()) {
    ValidateSequenceSize(bulb_node[BulbConstants::kStates], BulbConstants::kStates);
    for (const auto& state_node : bulb_node[BulbConstants::kStates]) {
      bulb.states.push_back(StringToBulbState(state_node.as<std::string>()));
    }
  } else {
    // Default states if not defined.
    bulb.states.push_back(maliput::api::rules::BulbState::kOff);
    bulb.states.push_back(maliput::api::rules::BulbState::kOn);
  }

  // Parse arrow orientation (only for Arrow type bulbs).
  if (const auto arrow_orientation = GetOptionalDoubleField(bulb_node, BulbConstants::kArrowOrientationRad)) {
    bulb.arrow_orientation_rad = arrow_orientation;
  }

  // Parse bounding box (optional).
  bulb.bounding_box = ParseBoundingBox(bulb_node[BulbConstants::kBoundingBox]);

  return bulb;
}

// Helper function to parse a BulbGroupDefinition from a YAML map.
// @param group_node The YAML node representing a bulb group.
// @return A BulbGroupDefinition with the parsed values.
BulbGroupDefinition ParseBulbGroup(const YAML::Node& group_node) {
  MALIDRIVE_THROW_UNLESS(group_node.IsMap(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(group_node[BulbGroupConstants::kPositionTrafficLight].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(group_node[BulbGroupConstants::kOrientationTrafficLight].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(group_node[BulbGroupConstants::kBulbs].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  ValidateSequenceSize(group_node[BulbGroupConstants::kBulbs], BulbGroupConstants::kBulbs);

  BulbGroupDefinition group;
  group.position_traffic_light =
      GetVector3(group_node[BulbGroupConstants::kPositionTrafficLight], BulbGroupConstants::kPositionTrafficLight);
  group.orientation_traffic_light = GetQuaternion(group_node[BulbGroupConstants::kOrientationTrafficLight],
                                                  BulbGroupConstants::kOrientationTrafficLight);

  // Parse bulbs.
  for (const auto& bulb_node : group_node[BulbGroupConstants::kBulbs]) {
    group.bulbs.push_back(ParseBulb(bulb_node));
  }

  return group;
}

// Helper function to parse a RuleState (rule-state) from a YAML map.
// @param rule_state_node The YAML node representing a rule state.
// @return A RuleState with the parsed values.
RuleState ParseRuleState(const YAML::Node& rule_state_node) {
  MALIDRIVE_THROW_UNLESS(rule_state_node.IsMap(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(rule_state_node[RuleStateConstants::kCondition].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  ValidateSequenceSize(rule_state_node[RuleStateConstants::kCondition], RuleStateConstants::kCondition);
  MALIDRIVE_THROW_UNLESS(rule_state_node[RuleStateConstants::kValue].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  RuleState rule_state;

  // Parse bulb conditions.
  for (const auto& condition_node : rule_state_node[RuleStateConstants::kCondition]) {
    MALIDRIVE_THROW_UNLESS(condition_node.IsMap(), maliput::common::road_network_description_parser_error);
    MALIDRIVE_THROW_UNLESS(condition_node[BulbStateConditionConstants::kBulb].IsDefined(),
                           maliput::common::road_network_description_parser_error);
    MALIDRIVE_THROW_UNLESS(condition_node[BulbStateConditionConstants::kState].IsDefined(),
                           maliput::common::road_network_description_parser_error);

    BulbStateCondition condition;
    condition.bulb_id = GetRequiredStringField(condition_node, BulbStateConditionConstants::kBulb);
    condition.state = StringToBulbState(GetRequiredStringField(condition_node, BulbStateConditionConstants::kState));

    rule_state.bulb_conditions.push_back(condition);
  }

  // Parse rule value.
  rule_state.rule_value = GetRequiredStringField(rule_state_node, RuleStateConstants::kValue);

  return rule_state;
}

// Helper function to parse a TrafficSignalDefinition from a YAML map.
// @param signal_node The YAML node representing a traffic signal definition.
// @return A TrafficSignalDefinition with the parsed values.
TrafficSignalDefinition ParseSignalDefinition(const YAML::Node& signal_node) {
  MALIDRIVE_THROW_UNLESS(signal_node.IsMap(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(signal_node[TrafficSignalConstants::kType].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(signal_node[TrafficSignalConstants::kDescription].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(signal_node[TrafficSignalConstants::kBulbGroup].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  TrafficSignalDefinition signal_definition;

  // Parse required fields.
  signal_definition.fingerprint.type = GetRequiredStringField(signal_node, TrafficSignalConstants::kType);
  signal_definition.description = GetRequiredStringField(signal_node, TrafficSignalConstants::kDescription);

  // Parse optional fields.
  if (const auto subtype = GetOptionalStringField(signal_node, TrafficSignalConstants::kSubtype)) {
    if (subtype.has_value() && (subtype.value() == "-1" || subtype.value() == "none")) {
      signal_definition.fingerprint.subtype = std::nullopt;
    } else {
      signal_definition.fingerprint.subtype = subtype;
    }
  }

  if (const auto country = GetOptionalStringField(signal_node, TrafficSignalConstants::kCountry)) {
    signal_definition.fingerprint.country = country;
  }

  if (const auto revision = GetOptionalStringField(signal_node, TrafficSignalConstants::kCountryRevision)) {
    signal_definition.fingerprint.country_revision = revision;
  }

  // Parse bulb group (exactly one expected).
  MALIDRIVE_THROW_UNLESS(signal_node[TrafficSignalConstants::kBulbGroup].IsSequence(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(signal_node[TrafficSignalConstants::kBulbGroup].size() == 1,
                         maliput::common::road_network_description_parser_error);
  signal_definition.bulb_group = ParseBulbGroup(signal_node[TrafficSignalConstants::kBulbGroup][0]);

  // Parse rule_states (optional).
  if (signal_node[TrafficSignalConstants::kRuleStates].IsDefined()) {
    ValidateSequenceSize(signal_node[TrafficSignalConstants::kRuleStates], TrafficSignalConstants::kRuleStates);
    for (const auto& rule_state_node : signal_node[TrafficSignalConstants::kRuleStates]) {
      signal_definition.rule_states.push_back(ParseRuleState(rule_state_node));
    }
  }

  return signal_definition;
}

std::unordered_map<TrafficSignalFingerprint, TrafficSignalDefinition> BuildFrom(const YAML::Node& root) {
  MALIDRIVE_THROW_UNLESS(root.IsMap(), maliput::common::road_network_description_parser_error);

  // Get traffic_signal_types array.
  const YAML::Node& signals_node = root["traffic_signal_types"];
  MALIDRIVE_THROW_UNLESS(signals_node.IsDefined(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(signals_node.IsSequence(), maliput::common::road_network_description_parser_error);

  std::unordered_map<TrafficSignalFingerprint, TrafficSignalDefinition> result;
  // Parse each signal definition.
  for (const auto& signal_node : signals_node) {
    const auto signal_definition = ParseSignalDefinition(signal_node);
    const auto& it = result.emplace(signal_definition.fingerprint, signal_definition);
    // TODO(@Santoi): Test this behavior.
    if (!it.second) {
      result[signal_definition.fingerprint] = signal_definition;
    }
  }

  return result;
}

}  // namespace

std::unordered_map<TrafficSignalFingerprint, TrafficSignalDefinition> TrafficSignalParser::LoadFromString(
    const std::string& yaml_content) {
  YAML::Node root = YAML::Load(yaml_content);
  return BuildFrom(root);
}

std::unordered_map<TrafficSignalFingerprint, TrafficSignalDefinition> TrafficSignalParser::LoadFromFile(
    const std::string& yaml_file_path) {
  YAML::Node root = YAML::LoadFile(yaml_file_path);
  return BuildFrom(root);
}

}  // namespace traffic_signal
}  // namespace malidrive
