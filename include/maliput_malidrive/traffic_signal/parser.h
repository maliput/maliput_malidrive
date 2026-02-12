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

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <maliput/api/rules/traffic_lights.h>
#include <maliput/math/quaternion.h>
#include <maliput/math/vector.h>

namespace malidrive {
namespace traffic_signal {

/// Represents a bulb state condition in a rule definition.
/// Specifies that a particular bulb must be in a particular state.
struct BulbStateCondition {
  /// Bulb ID (unique within bulb group).
  std::string bulb_id;
  /// Required bulb state.
  maliput::api::rules::BulbState state;
};

/// Represents a rule state that maps bulb state combinations to a Right-Of-Way rule value.
/// When all bulb conditions in the list are met, the associated rule value applies.
struct RuleState {
  /// List of bulb state conditions that must all be met for this rule to apply.
  std::vector<BulbStateCondition> bulb_conditions;
  /// The Right-Of-Way rule value when this condition is met.
  std::string rule_value;
};

/// Represents a bulb definition within a bulb group in a signal.
/// Defines the physical and behavioral properties of a single bulb.
struct BulbDefinition {
  /// Bulb ID (unique within bulb group).
  std::string id;
  /// Bulb color.
  maliput::api::rules::BulbColor color;
  /// Bulb type (Round or Arrow).
  maliput::api::rules::BulbType type;
  /// Position of bulb frame origin relative to bulb group frame.
  maliput::math::Vector3 position_bulb_group;
  /// Orientation (quaternion [w, x, y, z]) of bulb frame relative to bulb group frame.
  maliput::math::Quaternion orientation_bulb_group;
  /// Possible states this bulb can be in. Defaults to [Off, On] if not specified.
  std::vector<maliput::api::rules::BulbState> states;
  /// Arrow orientation in radians (only for Arrow type bulbs).
  /// Required when type is Arrow.
  std::optional<double> arrow_orientation_rad;
  /// Custom bounding box for this bulb.
  /// If not specified, uses default maliput dimensions.
  maliput::api::rules::Bulb::BoundingBox bounding_box;
};

/// Represents a bulb group definition within a signal.
/// Bulbs in a group share the same orientation and are positioned relative to the traffic light.
struct BulbGroupDefinition {
  /// Position of group frame origin relative to traffic light frame.
  maliput::math::Vector3 position_traffic_light;
  /// Orientation (quaternion [w, x, y, z]) of group frame relative to traffic light frame.
  maliput::math::Quaternion orientation_traffic_light;
  /// Bulbs in this group.
  std::vector<BulbDefinition> bulbs;
};

/// Unique identifier for a traffic signal definition.
struct TrafficSignalFingerprint {
  /// Signal type identifier. Matches XODR signal type attribute.
  std::string type;
  /// Optional signal subtype for finer matching.
  std::optional<std::string> subtype;
  /// Optional country code or standard.
  std::optional<std::string> country;
  /// Optional country standard revision.
  std::optional<std::string> country_revision;

  bool operator==(const TrafficSignalFingerprint& other) const {
    return type == other.type && subtype == other.subtype && country == other.country &&
           country_revision == other.country_revision;
  }
  bool operator!=(const TrafficSignalFingerprint& other) const { return !(*this == other); }
};

/// Represents a traffic signal definition loaded from the database.
/// Describes the complete structure and rule logic for a particular signal.
struct TrafficSignalDefinition {
  /// Unique identifier for this signal definition.
  TrafficSignalFingerprint fingerprint;
  /// Human-readable description of this signal definition.
  std::string description;
  /// Bulb group for this signal definition.
  BulbGroupDefinition bulb_group;
  /// Rule conditions mapping bulb state combinations to Right-Of-Way rule values.
  std::vector<RuleState> rule_states;
};

/// Traffic signal parser that loads traffic signal definitions from a YAML database. These are then
/// used in tandem with XODR signal data to create maliput TrafficLights and associated Right-Of-Way rules.
///
/// Design:
/// - TrafficLights and DiscreteValueRules are independent systems that operate in parallel.
/// - Both are created from the same XODR signal, but are not formally linked.
/// - TrafficLights provide visual representation (bulbs with states).
/// - DiscreteValueRules provide Right-Of-Way behavior (discrete values mapped to bulb states).
/// - They are synchronized externally: when rule states change, bulb states change.
///
/// Workflow:
/// 1. Load signal database: TrafficSignalParser::LoadFromFile().
/// 2. For each signal in XODR, extract its type/subtype and look up signal definition.
/// 3. Create traffic light: TrafficSignalParser::CreateTrafficLight().
/// 4. Create rules (one per affected lane/road): TrafficSignalParser::CreateRules().
class TrafficSignalParser {
 public:
  /// Loads a traffic signal database from a YAML string.
  ///
  /// @param yaml_content The YAML content as a string.
  /// @return Map of signal identifiers to their definitions.
  /// @throws maliput::common::road_network_description_parser_error if YAML parsing fails or schema validation fails.
  static std::unordered_map<TrafficSignalFingerprint, TrafficSignalDefinition> LoadFromString(
      const std::string& yaml_content);

  /// Loads a traffic signal database from a YAML file.
  ///
  /// @param file_path Path to the YAML database file.
  /// @return Map of signal identifiers to their definitions.
  /// @throws maliput::common::road_network_description_parser_error YAML parsing or schema validation fails.
  static std::unordered_map<TrafficSignalFingerprint, TrafficSignalDefinition> LoadFromFile(
      const std::string& yaml_file_path);
};

}  // namespace traffic_signal
}  // namespace malidrive

namespace std {

/// Hash function to use TrafficSignalFingerprint as a key in unordered containers. Combines the hash of each member
/// variable.
template <>
struct hash<malidrive::traffic_signal::TrafficSignalFingerprint> {
  size_t operator()(const malidrive::traffic_signal::TrafficSignalFingerprint& f) const {
    size_t seed = 0;

    // https://www.boost.org/doc/libs/1_84_0/libs/container_hash/doc/html/hash.html#notes_hash_combine
    auto hash_combine = [&seed](const auto& v) {
      using T = std::decay_t<decltype(v)>;
      seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    };

    // 1. Hash the mandatory string.
    hash_combine(f.type);

    // 2. Hash optionals (only if they have values, otherwise use a constant).
    auto hash_optional = [&](const auto& opt) {
      if (opt) {
        hash_combine(*opt);
      } else {
        hash_combine(size_t(0));  // Or any sentinel value
      }
    };

    hash_optional(f.subtype);
    hash_optional(f.country);
    hash_optional(f.country_revision);

    return seed;
  }
};
}  // namespace std
