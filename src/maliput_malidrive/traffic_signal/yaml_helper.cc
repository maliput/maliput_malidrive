// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota.
// All rights reserved.
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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "maliput_malidrive/traffic_signal/yaml_helper.h"

#include <maliput/common/maliput_throw.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace traffic_signal {

// ============================================================================
// TrafficSignalConstants Definitions
// ============================================================================

const char* TrafficSignalConstants::kType = "type";
const char* TrafficSignalConstants::kSubtype = "subtype";
const char* TrafficSignalConstants::kCountry = "country";
const char* TrafficSignalConstants::kCountryRevision = "country_revision";
const char* TrafficSignalConstants::kDescription = "description";
const char* TrafficSignalConstants::kBulbGroup = "bulb_group";
const char* TrafficSignalConstants::kRuleStates = "rule_states";

// ============================================================================
// BulbGroupConstants Definitions
// ============================================================================

const char* BulbGroupConstants::kPositionTrafficLight = "position_traffic_light";
const char* BulbGroupConstants::kOrientationTrafficLight = "orientation_traffic_light";
const char* BulbGroupConstants::kBulbs = "bulbs";

// ============================================================================
// BulbConstants Definitions
// ============================================================================

const char* BulbConstants::kId = "id";
const char* BulbConstants::kColor = "color";
const char* BulbConstants::kType = "type";
const char* BulbConstants::kPositionBulbGroup = "position_bulb_group";
const char* BulbConstants::kOrientationBulbGroup = "orientation_bulb_group";
const char* BulbConstants::kStates = "states";
const char* BulbConstants::kArrowOrientationRad = "arrow_orientation_rad";
const char* BulbConstants::kBoundingBox = "bounding_box";

// ============================================================================
// BoundingBoxConstants Definitions
// ============================================================================

const char* BoundingBoxConstants::kPMin = "p_min";
const char* BoundingBoxConstants::kPMax = "p_max";

// ============================================================================
// RuleStateConstants Definitions
// ============================================================================

const char* RuleStateConstants::kCondition = "condition";
const char* RuleStateConstants::kValue = "value";

// ============================================================================
// BulbStateConditionConstants Definitions
// ============================================================================

const char* BulbStateConditionConstants::kBulb = "bulb";
const char* BulbStateConditionConstants::kState = "state";

// ============================================================================
// Utility Function Implementations
// ============================================================================

void ValidateYamlNode(const YAML::Node& node, const std::string& field_name, const std::string& expected_type) {
  MALIDRIVE_VALIDATE(node.IsDefined(), maliput::common::assertion_error,
                     std::string("Field '" + field_name + "' is not defined in YAML. Expected: " + expected_type));
}

std::string GetRequiredStringField(const YAML::Node& node, const std::string& field_name) {
  ValidateYamlNode(node[field_name], field_name, "string value");
  MALIDRIVE_VALIDATE(!node[field_name].IsNull(), maliput::common::assertion_error,
                     "Field '" + field_name + "' is null in YAML");

  return node[field_name].as<std::string>();
}

std::optional<std::string> GetOptionalStringField(const YAML::Node& node, const std::string& field_name) {
  if (!node[field_name].IsDefined() || node[field_name].IsNull()) {
    return std::nullopt;
  }

  return node[field_name].as<std::string>();
}

std::optional<int> GetOptionalIntField(const YAML::Node& node, const std::string& field_name) {
  if (!node[field_name].IsDefined() || node[field_name].IsNull()) {
    return std::nullopt;
  }

  return node[field_name].as<int>();
}

std::optional<double> GetOptionalDoubleField(const YAML::Node& node, const std::string& field_name) {
  if (!node[field_name].IsDefined() || node[field_name].IsNull()) {
    return std::nullopt;
  }

  return node[field_name].as<double>();
}

maliput::math::Vector3 GetVector3(const YAML::Node& node, const std::string& field_name) {
  ValidateSequenceSize(node, field_name, 3);

  const double x = node[0].as<double>();
  const double y = node[1].as<double>();
  const double z = node[2].as<double>();
  return maliput::math::Vector3(x, y, z);
}

maliput::math::Quaternion GetQuaternion(const YAML::Node& node, const std::string& field_name) {
  ValidateSequenceSize(node, field_name, 4);

  const double w = node[0].as<double>();
  const double x = node[1].as<double>();
  const double y = node[2].as<double>();
  const double z = node[3].as<double>();
  return maliput::math::Quaternion(w, x, y, z);
}

void ValidateSequenceSize(const YAML::Node& node, const std::string& field_name, size_t expected_size) {
  MALIDRIVE_VALIDATE(node.IsSequence(), maliput::common::assertion_error,
                     "Field '" + field_name + "' is not a sequence in YAML. ");

  if (expected_size > 0) {
    MALIDRIVE_VALIDATE(node.size() == expected_size, maliput::common::assertion_error,
                       "Field '" + field_name + "' has incorrect sequence size. Expected: " +
                           std::to_string(expected_size) + ", Got: " + std::to_string(node.size()));
  }
}

}  // namespace traffic_signal
}  // namespace malidrive
