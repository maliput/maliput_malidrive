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
#include "maliput_malidrive/traffic_control_device/parser.h"

#include <algorithm>
#include <cctype>
#include <unordered_map>
#include <utility>

#include <maliput/common/maliput_hash.h>
#include <maliput/common/maliput_throw.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/traffic_control_device/device_type.h"
#include "maliput_malidrive/traffic_control_device/yaml_helper.h"

namespace malidrive {
namespace traffic_control_device {

bool TrafficControlDeviceParser::IsWildcard(const std::string& value) { return value == kWildcard; }

bool TrafficControlDeviceParser::IsWildcard(const std::optional<std::string>& value) {
  return value.has_value() && *value == kWildcard;
}

int TrafficControlDeviceParser::Specificity(const TrafficControlDeviceFingerprint& fp) {
  // Each field contributes 1 to specificity when it is NOT a wildcard.
  // nullopt is specific; only "*" is non-specific.
  int score = 0;
  if (!IsWildcard(fp.type)) ++score;
  if (!IsWildcard(fp.subtype)) ++score;
  if (!IsWildcard(fp.country)) ++score;
  if (!IsWildcard(fp.country_revision)) ++score;
  if (!IsWildcard(fp.name)) ++score;
  return score;
}

int TrafficControlDeviceParser::SpecificityForObject(const TrafficControlDeviceFingerprint& fp) {
  // Object entries only carry type, subtype and name.
  int score = 0;
  if (!IsWildcard(fp.type)) ++score;
  if (!IsWildcard(fp.subtype)) ++score;
  if (!IsWildcard(fp.name)) ++score;
  return score;
}

bool TrafficControlDeviceParser::Matches(const TrafficControlDeviceFingerprint& db_entry,
                                         const TrafficControlDeviceFingerprint& query) {
  // For each field: db_entry matches if it is the wildcard OR equals the query field.
  auto field_matches = [](const std::string& db_val, const std::string& query_val) -> bool {
    return IsWildcard(db_val) || db_val == query_val;
  };
  auto optional_field_matches = [](const std::optional<std::string>& db_opt,
                                   const std::optional<std::string>& query_opt) -> bool {
    if (IsWildcard(db_opt)) return true;
    return db_opt == query_opt;
  };
  return field_matches(db_entry.type, query.type) && optional_field_matches(db_entry.subtype, query.subtype) &&
         optional_field_matches(db_entry.country, query.country) &&
         optional_field_matches(db_entry.country_revision, query.country_revision) &&
         optional_field_matches(db_entry.name, query.name);
}

bool TrafficControlDeviceParser::CanOverlap(const TrafficControlDeviceFingerprint& a,
                                            const TrafficControlDeviceFingerprint& b) {
  // Two entries can overlap when, for every field, at least one is wildcard OR both are equal.
  auto field_can_overlap = [](const std::string& va, const std::string& vb) -> bool {
    return IsWildcard(va) || IsWildcard(vb) || va == vb;
  };
  auto optional_field_can_overlap = [](const std::optional<std::string>& oa,
                                       const std::optional<std::string>& ob) -> bool {
    return IsWildcard(oa) || IsWildcard(ob) || oa == ob;
  };
  return field_can_overlap(a.type, b.type) && optional_field_can_overlap(a.subtype, b.subtype) &&
         optional_field_can_overlap(a.country, b.country) &&
         optional_field_can_overlap(a.country_revision, b.country_revision) &&
         optional_field_can_overlap(a.name, b.name);
}

namespace {

bool IsStrictPascalCaseToken(const std::string& token) {
  if (token.empty() || token.find('_') != std::string::npos) {
    return false;
  }
  if (!std::isupper(static_cast<unsigned char>(token.front()))) {
    return false;
  }
  return std::all_of(token.begin(), token.end(),
                     [](unsigned char c) { return std::isalnum(static_cast<unsigned char>(c)); });
}

// Helper function to convert a color string to BulbColor enum.
// @param color_str The color string (e.g., "Red", "Yellow", "Green").
// @return The corresponding BulbColor enum value.
// @throws std::runtime_error if the color string is invalid.
maliput::api::rules::BulbColor StringToBulbColor(const std::string& color_str) {
  using C = maliput::api::rules::BulbColor;
  static const auto kMapper = []() {
    std::unordered_map<std::string, C, maliput::common::DefaultHash> result;
    for (const auto& [bulb_color, bulb_color_str] : maliput::api::rules::BulbColorMapper()) {
      if (IsStrictPascalCaseToken(bulb_color_str)) {
        result.emplace(bulb_color_str, bulb_color);
      }
    }
    return result;
  }();
  const auto it = kMapper.find(color_str);
  if (it != kMapper.end()) {
    return it->second;
  }
  MALIDRIVE_THROW_MESSAGE("Invalid bulb color: " + color_str, maliput::common::road_network_description_parser_error);
}

// Helper function to convert a type string to BulbType enum.
// @param type_str The type string (e.g., "Round", "ArrowLeft", "arrow_left").
// @return The corresponding BulbType enum value.
// @throws std::runtime_error if the type string is invalid.
maliput::api::rules::BulbType StringToBulbType(const std::string& type_str) {
  using T = maliput::api::rules::BulbType;
  static const auto kMapper = []() {
    std::unordered_map<std::string, T, maliput::common::DefaultHash> result;
    for (const auto& [bulb_type_str, bulb_type] : maliput::api::rules::BulbTypeStringToEnumMapper()) {
      if (IsStrictPascalCaseToken(bulb_type_str)) {
        result.emplace(bulb_type_str, bulb_type);
      }
    }
    return result;
  }();
  const auto it = kMapper.find(type_str);
  if (it != kMapper.end()) {
    return it->second;
  }
  MALIDRIVE_THROW_MESSAGE("Invalid bulb type: " + type_str, maliput::common::road_network_description_parser_error);
}

// Helper function to convert a state string to BulbState enum.
// @param state_str The state string (e.g., "Off", "On", "Blinking").
// @return The corresponding BulbState enum value.
// @throws std::runtime_error if the state string is invalid.
maliput::api::rules::BulbState StringToBulbState(const std::string& state_str) {
  using S = maliput::api::rules::BulbState;
  static const auto kMapper = []() {
    std::unordered_map<std::string, S, maliput::common::DefaultHash> result;
    for (const auto& [bulb_state, bulb_state_str] : maliput::api::rules::BulbStateMapper()) {
      if (IsStrictPascalCaseToken(bulb_state_str)) {
        result.emplace(bulb_state_str, bulb_state);
      }
    }
    return result;
  }();
  const auto it = kMapper.find(state_str);
  if (it != kMapper.end()) {
    return it->second;
  }
  MALIDRIVE_THROW_MESSAGE("Invalid bulb state: " + state_str, maliput::common::road_network_description_parser_error);
}

// Helper function to parse a bulb's axis-aligned bounding box from a YAML map.
// @param node The YAML node containing p_min and p_max keys.
// @return A Bulb::BoundingBox with the parsed values, or the default Bulb::BoundingBox
//         when @p node is not defined.
maliput::api::rules::Bulb::BoundingBox ParseBulbBoundingBox(const YAML::Node& node) {
  maliput::api::rules::Bulb::BoundingBox bounding_box;
  if (!node.IsDefined()) {
    // If not defined, return the default bounding box.
    return bounding_box;
  }
  MALIDRIVE_THROW_UNLESS(node.IsMap(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(node[BulbBoundingBoxConstants::kPMin].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(node[BulbBoundingBoxConstants::kPMax].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  const auto p_min = GetVector3(node[BulbBoundingBoxConstants::kPMin], BulbBoundingBoxConstants::kPMin);
  const auto p_max = GetVector3(node[BulbBoundingBoxConstants::kPMax], BulbBoundingBoxConstants::kPMax);

  bounding_box.p_BMin = p_min;
  bounding_box.p_BMax = p_max;

  return bounding_box;
}

// Helper function to parse a device-level bounding box from a YAML map.
//
// Expected schema (all three fields required when the node is present):
//   default_bounding_box:
//     length: <number>  # >= 0, local u-axis
//     width:  <number>  # >= 0, local v-axis
//     height: <number>  # >= 0, local z-axis
//
// @return A BoundingBoxDimensions with the parsed values.
// @throws maliput::common::assertion_error When required fields are missing, when unknown keys
//         are present, or when any dimension is negative.
BoundingBoxDimensions ParseDeviceBoundingBox(const YAML::Node& node) {
  MALIDRIVE_THROW_UNLESS(node.IsDefined(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(node.IsMap(), maliput::common::road_network_description_parser_error);

  // Reject unknown keys to surface typos early.
  for (const auto& kv : node) {
    const auto key = kv.first.as<std::string>();
    if (key != DeviceBoundingBoxConstants::kLength && key != DeviceBoundingBoxConstants::kWidth &&
        key != DeviceBoundingBoxConstants::kHeight) {
      MALIDRIVE_THROW_MESSAGE("Unknown field in default_bounding_box: '" + key + "'",
                              maliput::common::road_network_description_parser_error);
    }
  }

  MALIDRIVE_THROW_UNLESS(node[DeviceBoundingBoxConstants::kLength].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(node[DeviceBoundingBoxConstants::kWidth].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(node[DeviceBoundingBoxConstants::kHeight].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  const double length = node[DeviceBoundingBoxConstants::kLength].as<double>();
  const double width = node[DeviceBoundingBoxConstants::kWidth].as<double>();
  const double height = node[DeviceBoundingBoxConstants::kHeight].as<double>();

  if (length < 0.0 || width < 0.0 || height < 0.0) {
    MALIDRIVE_THROW_MESSAGE("default_bounding_box dimensions must be non-negative",
                            maliput::common::road_network_description_parser_error);
  }

  return BoundingBoxDimensions{length, width, height};
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
  MALIDRIVE_THROW_UNLESS(bulb_node[BulbConstants::kPositionTrafficLight].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(bulb_node[BulbConstants::kOrientationTrafficLight].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  BulbDefinition bulb;
  bulb.id = GetRequiredStringField(bulb_node, BulbConstants::kId);
  bulb.color = StringToBulbColor(GetRequiredStringField(bulb_node, BulbConstants::kColor));
  bulb.type = StringToBulbType(GetRequiredStringField(bulb_node, BulbConstants::kType));
  bulb.position_traffic_light =
      GetVector3(bulb_node[BulbConstants::kPositionTrafficLight], BulbConstants::kPositionTrafficLight);
  bulb.orientation_traffic_light =
      GetQuaternion(bulb_node[BulbConstants::kOrientationTrafficLight], BulbConstants::kOrientationTrafficLight);

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

  // Parse initial_state (optional, defaults to kOff).
  if (const auto initial_state_str = GetOptionalStringField(bulb_node, BulbConstants::kInitialState)) {
    bulb.initial_state = StringToBulbState(initial_state_str.value());
  }

  // Parse bounding box (optional).
  bulb.bounding_box = ParseBulbBoundingBox(bulb_node[BulbConstants::kBoundingBox]);

  return bulb;
}

// Helper function to parse a RuleState (rule-state) from a YAML map.
// @param rule_state_node The YAML node representing a rule state.
// @return A RuleState with the parsed values.
RuleState ParseRuleState(const YAML::Node& rule_state_node) {
  MALIDRIVE_THROW_UNLESS(rule_state_node.IsMap(), maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(rule_state_node[RuleStateConstants::kConditions].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  ValidateSequenceSize(rule_state_node[RuleStateConstants::kConditions], RuleStateConstants::kConditions);
  MALIDRIVE_THROW_UNLESS(rule_state_node[RuleStateConstants::kValue].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  RuleState rule_state;

  // Parse bulb conditions.
  for (const auto& condition_node : rule_state_node[RuleStateConstants::kConditions]) {
    MALIDRIVE_THROW_UNLESS(condition_node.IsMap(), maliput::common::road_network_description_parser_error);
    MALIDRIVE_THROW_UNLESS(condition_node[BulbStateConditionConstants::kBulbId].IsDefined(),
                           maliput::common::road_network_description_parser_error);
    MALIDRIVE_THROW_UNLESS(condition_node[BulbStateConditionConstants::kState].IsDefined(),
                           maliput::common::road_network_description_parser_error);

    BulbStateCondition condition;
    condition.bulb_id = GetRequiredStringField(condition_node, BulbStateConditionConstants::kBulbId);
    condition.state = StringToBulbState(GetRequiredStringField(condition_node, BulbStateConditionConstants::kState));

    rule_state.bulb_conditions.push_back(condition);
  }

  // Parse rule value.
  rule_state.rule_value = GetRequiredStringField(rule_state_node, RuleStateConstants::kValue);

  return rule_state;
}

// Helper function to parse a TrafficControlDeviceDefinition from a YAML map.
// @param entry_node The YAML node representing one entry in odr_signal_types.
//        Expected structure:
//          odr_representation:
//            type: ...
//            subtype: ...
//            country: ...
//            country_revision: ...
//            name: ...
//          properties:
//            device_type: ...
//            device_semantics: ...
//            is_position_dynamic: ...
//            default_bounding_box: ...
//            description: ...
//            bulbs: ...
//            rule_states: ...
// @return A TrafficControlDeviceDefinition with the parsed values.
TrafficControlDeviceDefinition ParseSignalDefinition(const YAML::Node& entry_node) {
  MALIDRIVE_THROW_UNLESS(entry_node.IsMap(), maliput::common::road_network_description_parser_error);

  // Validate and extract the odr_representation sub-node.
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kOdrRepresentation].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kOdrRepresentation].IsMap(),
                         maliput::common::road_network_description_parser_error);
  const YAML::Node& repr_node = entry_node[TrafficControlDeviceConstants::kOdrRepresentation];

  // Validate and extract the properties sub-node.
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kProperties].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kProperties].IsMap(),
                         maliput::common::road_network_description_parser_error);
  const YAML::Node& props_node = entry_node[TrafficControlDeviceConstants::kProperties];

  // device_type is required.
  MALIDRIVE_THROW_UNLESS(props_node[TrafficControlDeviceConstants::kDeviceType].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  TrafficControlDeviceDefinition tcd_definition;

  // --- Parse fingerprint from odr_representation ---
  tcd_definition.fingerprint.type = GetRequiredStringField(repr_node, TrafficControlDeviceConstants::kType);

  if (const auto subtype = GetOptionalStringFieldWildcardDefault(repr_node, TrafficControlDeviceConstants::kSubtype)) {
    if (subtype.value() == "-1" || subtype.value() == "none") {
      tcd_definition.fingerprint.subtype = std::nullopt;
    } else {
      tcd_definition.fingerprint.subtype = subtype;
    }
  }

  if (const auto country = GetOptionalStringFieldWildcardDefault(repr_node, TrafficControlDeviceConstants::kCountry)) {
    tcd_definition.fingerprint.country = country;
  }

  if (const auto revision =
          GetOptionalStringFieldWildcardDefault(repr_node, TrafficControlDeviceConstants::kCountryRevision)) {
    tcd_definition.fingerprint.country_revision = revision;
  }

  if (const auto name = GetOptionalStringFieldWildcardDefault(repr_node, TrafficControlDeviceConstants::kName)) {
    tcd_definition.fingerprint.name = name;
  }

  // --- Parse properties ---
  const std::string device_type_str = GetRequiredStringField(props_node, TrafficControlDeviceConstants::kDeviceType);
  tcd_definition.device_type = StringToTrafficControlDeviceType(device_type_str);
  MALIDRIVE_VALIDATE(tcd_definition.device_type == TrafficControlDeviceType::kTrafficLight ||
                         tcd_definition.device_type == TrafficControlDeviceType::kTrafficSign,
                     maliput::common::road_network_description_parser_error,
                     "device_type '" + device_type_str +
                         "' is not allowed for odr_signal_types entries; "
                         "expected 'traffic_light' or 'traffic_sign'.");

  tcd_definition.device_semantics = GetOptionalStringField(props_node, TrafficControlDeviceConstants::kDeviceSemantics);

  // is_position_dynamic (optional bool, defaults to false).
  if (props_node[TrafficControlDeviceConstants::kIsPositionDynamic].IsDefined() &&
      !props_node[TrafficControlDeviceConstants::kIsPositionDynamic].IsNull()) {
    tcd_definition.is_position_dynamic = props_node[TrafficControlDeviceConstants::kIsPositionDynamic].as<bool>();
  }

  // description (optional).
  if (const auto desc = GetOptionalStringField(props_node, TrafficControlDeviceConstants::kDescription)) {
    tcd_definition.description = desc.value();
  }

  // default_bounding_box at device level (optional).
  if (props_node[TrafficControlDeviceConstants::kDefaultBoundingBox].IsDefined() &&
      !props_node[TrafficControlDeviceConstants::kDefaultBoundingBox].IsNull()) {
    tcd_definition.default_bounding_box =
        ParseDeviceBoundingBox(props_node[TrafficControlDeviceConstants::kDefaultBoundingBox]);
  }

  // Parse bulbs (optional sequence).
  if (props_node[TrafficControlDeviceConstants::kBulbs].IsDefined()) {
    ValidateSequenceSize(props_node[TrafficControlDeviceConstants::kBulbs], TrafficControlDeviceConstants::kBulbs);
    for (const auto& bulb_node : props_node[TrafficControlDeviceConstants::kBulbs]) {
      tcd_definition.bulbs.push_back(ParseBulb(bulb_node));
    }
  }

  // Parse rule_states (optional).
  if (props_node[TrafficControlDeviceConstants::kRuleStates].IsDefined()) {
    ValidateSequenceSize(props_node[TrafficControlDeviceConstants::kRuleStates],
                         TrafficControlDeviceConstants::kRuleStates);
    for (const auto& rule_state_node : props_node[TrafficControlDeviceConstants::kRuleStates]) {
      tcd_definition.rule_states.push_back(ParseRuleState(rule_state_node));
    }
  }

  return tcd_definition;
}

// Helper function to parse a TrafficControlDeviceDefinition from a YAML map representing
// an entry under `odr_object_types`. Object entries have a stricter schema than signal
// entries:
//   - `odr_representation` carries only `type`, `subtype`, and `name`. `country` and
//     `country_revision` are not allowed.
//   - `properties` does not allow `bulbs` or `rule_states`.
//   - `device_type` must be `road_marking` or `road_object`.
// @return A TrafficControlDeviceDefinition with the parsed values. country and
//         country_revision in the fingerprint are left as std::nullopt; bulbs and
//         rule_states are left empty.
TrafficControlDeviceDefinition ParseObjectDefinition(const YAML::Node& entry_node) {
  MALIDRIVE_THROW_UNLESS(entry_node.IsMap(), maliput::common::road_network_description_parser_error);

  // Validate and extract the odr_representation sub-node.
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kOdrRepresentation].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kOdrRepresentation].IsMap(),
                         maliput::common::road_network_description_parser_error);
  const YAML::Node& repr_node = entry_node[TrafficControlDeviceConstants::kOdrRepresentation];

  // Reject fields not allowed in odr_object_types entries.
  MALIDRIVE_VALIDATE(!repr_node[TrafficControlDeviceConstants::kCountry].IsDefined(),
                     maliput::common::road_network_description_parser_error,
                     "Field 'country' is not allowed in odr_object_types entries.");
  MALIDRIVE_VALIDATE(!repr_node[TrafficControlDeviceConstants::kCountryRevision].IsDefined(),
                     maliput::common::road_network_description_parser_error,
                     "Field 'country_revision' is not allowed in odr_object_types entries.");

  // Validate and extract the properties sub-node.
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kProperties].IsDefined(),
                         maliput::common::road_network_description_parser_error);
  MALIDRIVE_THROW_UNLESS(entry_node[TrafficControlDeviceConstants::kProperties].IsMap(),
                         maliput::common::road_network_description_parser_error);
  const YAML::Node& props_node = entry_node[TrafficControlDeviceConstants::kProperties];

  // Reject bulbs / rule_states in object entries.
  MALIDRIVE_VALIDATE(!props_node[TrafficControlDeviceConstants::kBulbs].IsDefined(),
                     maliput::common::road_network_description_parser_error,
                     "Field 'bulbs' is not allowed in odr_object_types entries.");
  MALIDRIVE_VALIDATE(!props_node[TrafficControlDeviceConstants::kRuleStates].IsDefined(),
                     maliput::common::road_network_description_parser_error,
                     "Field 'rule_states' is not allowed in odr_object_types entries.");

  // device_type is required.
  MALIDRIVE_THROW_UNLESS(props_node[TrafficControlDeviceConstants::kDeviceType].IsDefined(),
                         maliput::common::road_network_description_parser_error);

  TrafficControlDeviceDefinition tcd_definition;

  // --- Parse fingerprint from odr_representation ---
  tcd_definition.fingerprint.type = GetRequiredStringField(repr_node, TrafficControlDeviceConstants::kType);

  if (const auto subtype = GetOptionalStringField(repr_node, TrafficControlDeviceConstants::kSubtype)) {
    if (subtype.value() == "-1" || subtype.value() == "none") {
      tcd_definition.fingerprint.subtype = std::nullopt;
    } else {
      tcd_definition.fingerprint.subtype = subtype;
    }
  }

  if (const auto name = GetOptionalStringField(repr_node, TrafficControlDeviceConstants::kName)) {
    tcd_definition.fingerprint.name = name;
  }

  // country and country_revision left as std::nullopt by construction.

  // --- Parse properties ---
  const std::string device_type_str = GetRequiredStringField(props_node, TrafficControlDeviceConstants::kDeviceType);
  tcd_definition.device_type = StringToTrafficControlDeviceType(device_type_str);
  MALIDRIVE_VALIDATE(tcd_definition.device_type == TrafficControlDeviceType::kRoadMarking ||
                         tcd_definition.device_type == TrafficControlDeviceType::kRoadObject,
                     maliput::common::road_network_description_parser_error,
                     "device_type '" + device_type_str +
                         "' is not allowed for odr_object_types entries; "
                         "expected 'road_marking' or 'road_object'.");

  tcd_definition.device_semantics = GetOptionalStringField(props_node, TrafficControlDeviceConstants::kDeviceSemantics);

  // is_position_dynamic (optional bool, defaults to false).
  if (props_node[TrafficControlDeviceConstants::kIsPositionDynamic].IsDefined() &&
      !props_node[TrafficControlDeviceConstants::kIsPositionDynamic].IsNull()) {
    tcd_definition.is_position_dynamic = props_node[TrafficControlDeviceConstants::kIsPositionDynamic].as<bool>();
  }

  // description (optional).
  if (const auto desc = GetOptionalStringField(props_node, TrafficControlDeviceConstants::kDescription)) {
    tcd_definition.description = desc.value();
  }

  // default_bounding_box at device level (optional).
  if (props_node[TrafficControlDeviceConstants::kDefaultBoundingBox].IsDefined() &&
      !props_node[TrafficControlDeviceConstants::kDefaultBoundingBox].IsNull()) {
    tcd_definition.default_bounding_box =
        ParseDeviceBoundingBox(props_node[TrafficControlDeviceConstants::kDefaultBoundingBox]);
  }

  return tcd_definition;
}

}  // namespace

std::vector<TrafficControlDeviceDefinition> TrafficControlDeviceParser::BuildFrom(const YAML::Node& root) {
  MALIDRIVE_THROW_UNLESS(root.IsMap(), maliput::common::road_network_description_parser_error);

  const YAML::Node& signals_node = root[TrafficControlDeviceConstants::kOdrSignalTypes];
  const YAML::Node& objects_node = root[TrafficControlDeviceConstants::kOdrObjectTypes];

  // At least one of the two root keys must be present.
  MALIDRIVE_VALIDATE(signals_node.IsDefined() || objects_node.IsDefined(),
                     maliput::common::road_network_description_parser_error,
                     "Database must contain at least one of 'odr_signal_types' or 'odr_object_types'.");

  std::vector<TrafficControlDeviceDefinition> signals;
  if (signals_node.IsDefined()) {
    MALIDRIVE_THROW_UNLESS(signals_node.IsSequence(), maliput::common::road_network_description_parser_error);
    for (const auto& signal_node : signals_node) {
      signals.push_back(ParseSignalDefinition(signal_node));
    }
  }

  std::vector<TrafficControlDeviceDefinition> objects;
  if (objects_node.IsDefined()) {
    MALIDRIVE_THROW_UNLESS(objects_node.IsSequence(), maliput::common::road_network_description_parser_error);
    for (const auto& object_node : objects_node) {
      objects.push_back(ParseObjectDefinition(object_node));
    }
  }

  // Per-root pairwise conflict validation: two entries conflict when they can overlap AND
  // have equal specificity. Signals and objects live in disjoint namespaces in OpenDRIVE,
  // so we never compare across roots.
  auto validate_conflicts = [](const std::vector<TrafficControlDeviceDefinition>& entries,
                               int (*specificity)(const TrafficControlDeviceFingerprint&),
                               const std::string& root_key) {
    for (std::size_t i = 0; i < entries.size(); ++i) {
      for (std::size_t j = i + 1; j < entries.size(); ++j) {
        const auto& fp_i = entries[i].fingerprint;
        const auto& fp_j = entries[j].fingerprint;
        if (CanOverlap(fp_i, fp_j) && specificity(fp_i) == specificity(fp_j)) {
          MALIDRIVE_THROW_MESSAGE("Conflicting " + root_key +
                                      " entries with equal specificity: "
                                      "entry '" +
                                      entries[i].description + "' (type='" + fp_i.type +
                                      "') conflicts with "
                                      "entry '" +
                                      entries[j].description + "' (type='" + fp_j.type + "').",
                                  maliput::common::road_network_description_parser_error);
        }
      }
    }
  };

  validate_conflicts(signals, &TrafficControlDeviceParser::Specificity, "odr_signal_types");
  validate_conflicts(objects, &TrafficControlDeviceParser::SpecificityForObject, "odr_object_types");

  // Merge into a single flat vector. Downstream code discriminates entries by device_type.
  std::vector<TrafficControlDeviceDefinition> result;
  result.reserve(signals.size() + objects.size());
  result.insert(result.end(), std::make_move_iterator(signals.begin()), std::make_move_iterator(signals.end()));
  result.insert(result.end(), std::make_move_iterator(objects.begin()), std::make_move_iterator(objects.end()));
  return result;
}

std::vector<TrafficControlDeviceDefinition> TrafficControlDeviceParser::LoadFromString(
    const std::string& yaml_content) {
  YAML::Node root = YAML::Load(yaml_content);
  return BuildFrom(root);
}

std::vector<TrafficControlDeviceDefinition> TrafficControlDeviceParser::LoadFromFile(
    const std::string& yaml_file_path) {
  YAML::Node root = YAML::LoadFile(yaml_file_path);
  return BuildFrom(root);
}

}  // namespace traffic_control_device
}  // namespace malidrive
