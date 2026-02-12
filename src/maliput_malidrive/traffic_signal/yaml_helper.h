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
#pragma once

#include <optional>
#include <string>

#include <maliput/math/quaternion.h>
#include <maliput/math/vector.h>
#include <yaml-cpp/yaml.h>

namespace malidrive {
namespace traffic_signal {

/// Constants for traffic signal description YAML fields.
struct TrafficSignalConstants {
  static constexpr const char* kType = "type";
  static constexpr const char* kSubtype = "subtype";
  static constexpr const char* kCountry = "country";
  static constexpr const char* kCountryRevision = "country_revision";
  static constexpr const char* kDescription = "description";
  static constexpr const char* kBulbGroup = "bulb_group";
  static constexpr const char* kRuleStates = "rule_states";
};

/// Constants for bulb group YAML fields.
struct BulbGroupConstants {
  static constexpr const char* kPositionTrafficLight = "position_traffic_light";
  static constexpr const char* kOrientationTrafficLight = "orientation_traffic_light";
  static constexpr const char* kBulbs = "bulbs";
};

/// Constants for bulb YAML fields.
struct BulbConstants {
  static constexpr const char* kId = "id";
  static constexpr const char* kColor = "color";
  static constexpr const char* kType = "type";
  static constexpr const char* kPositionBulbGroup = "position_bulb_group";
  static constexpr const char* kOrientationBulbGroup = "orientation_bulb_group";
  static constexpr const char* kStates = "states";
  static constexpr const char* kArrowOrientationRad = "arrow_orientation_rad";
  static constexpr const char* kBoundingBox = "bounding_box";
};

/// Constants for bounding box YAML fields.
struct BoundingBoxConstants {
  static constexpr const char* kPMin = "p_min";
  static constexpr const char* kPMax = "p_max";
};

/// Constants for rule state YAML fields.
struct RuleStateConstants {
  static constexpr const char* kCondition = "condition";
  static constexpr const char* kValue = "value";
};

/// Constants for bulb state condition YAML fields.
struct BulbStateConditionConstants {
  static constexpr const char* kBulb = "bulb";
  static constexpr const char* kState = "state";
};

/// Validates that a YAML node is defined and of the expected type.
///
/// @param node The YAML node to validate.
/// @param field_name The name of the field being validated (for error messages).
/// @param expected_type A description of the expected YAML type
/// (e.g., "map", "sequence", "string value").
/// @throws maliput::common::road_network_description_parser_error if node is not defined or is of
///         an unexpected type.
void ValidateYamlNode(const YAML::Node& node, const std::string& field_name, const std::string& expected_type);

/// Retrieves a required string field from a YAML node.
///
/// @param node The YAML node containing the field.
/// @param field_name The name of the field to retrieve.
/// @returns The string value of the field.
/// @throws maliput::common::road_network_description_parser_error if the field is missing, null, or
///         cannot be converted to a string.
std::string GetRequiredStringField(const YAML::Node& node, const std::string& field_name);

/// Retrieves an optional string field from a YAML node.
///
/// @param node The YAML node containing the field.
/// @param field_name The name of the field to retrieve.
/// @returns An optional containing the string value if defined and not null,
///          std::nullopt otherwise.
std::optional<std::string> GetOptionalStringField(const YAML::Node& node, const std::string& field_name);

/// Retrieves an optional integer field from a YAML node.
///
/// @param node The YAML node containing the field.
/// @param field_name The name of the field to retrieve.
/// @returns An optional containing the integer value if defined and not null,
///          std::nullopt otherwise.
std::optional<int> GetOptionalIntField(const YAML::Node& node, const std::string& field_name);

/// Retrieves an optional double field from a YAML node.
///
/// @param node The YAML node containing the field.
/// @param field_name The name of the field to retrieve.
/// @returns An optional containing the double value if defined and not null,
///          std::nullopt otherwise.
std::optional<double> GetOptionalDoubleField(const YAML::Node& node, const std::string& field_name);

/// Extracts a Vector3 from a YAML sequence node.
///
/// @param node The YAML sequence node containing three values.
/// @param field_name The name of the field being extracted (for error messages).
/// @returns A Vector3 with (x, y, z) values.
/// @throws maliput::common::road_network_description_parser_error if the node is not a sequence with
///         exactly 3 elements.
maliput::math::Vector3 GetVector3(const YAML::Node& node, const std::string& field_name);

/// Extracts a Quaternion from a YAML sequence node.
///
/// @param node The YAML sequence node containing four values (w, x, y, z).
/// @param field_name The name of the field being extracted (for error messages).
/// @returns A Quaternion with (w, x, y, z) values.
/// @throws maliput::common::road_network_description_parser_error if the node is not a sequence with
///         exactly 4 elements or if any element cannot be converted to double.
maliput::math::Quaternion GetQuaternion(const YAML::Node& node, const std::string& field_name);

/// Validates that a YAML node is a sequence with the expected size.
///
/// @param node The YAML sequence node to validate.
/// @param field_name The name of the field being validated (for error messages).
/// @param expected_size The expected number of elements in the sequence. If 0
/// (default), size validation is skipped and only the sequence type is checked.
/// @throws maliput::common::road_network_description_parser_error if the node is not a sequence or
///         if expected_size > 0 and the actual size does not match.
void ValidateSequenceSize(const YAML::Node& node, const std::string& field_name, size_t expected_size = 0);

}  // namespace traffic_signal
}  // namespace malidrive
