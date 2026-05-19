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
#include <yaml-cpp/yaml.h>

#include "maliput_malidrive/traffic_control_device/device_type.h"

namespace malidrive {
namespace traffic_control_device {

/// Represents a bulb state condition in a rule definition.
/// Specifies that a particular bulb must be in a particular state.
struct BulbStateCondition {
  /// Bulb ID (unique within traffic signal).
  std::string bulb_id;
  /// Required bulb state.
  maliput::api::rules::BulbState state;

  bool operator==(const BulbStateCondition& other) const { return bulb_id == other.bulb_id && state == other.state; }
  bool operator!=(const BulbStateCondition& other) const { return !(*this == other); }
};

/// Represents a rule state that maps bulb state combinations to a Right-Of-Way rule value.
/// When all bulb conditions in the list are met, the associated rule value applies.
struct RuleState {
  /// List of bulb state conditions that must all be met for this rule to apply.
  std::vector<BulbStateCondition> bulb_conditions;
  /// The Right-Of-Way rule value when this condition is met.
  std::string rule_value;

  bool operator==(const RuleState& other) const {
    return bulb_conditions == other.bulb_conditions && rule_value == other.rule_value;
  }
  bool operator!=(const RuleState& other) const { return !(*this == other); }
};

/// Represents a bulb definition within a signal.
/// Defines the physical and behavioral properties of a single bulb.
struct BulbDefinition {
  /// Bulb ID (unique within traffic signal).
  std::string id;
  /// Bulb color.
  maliput::api::rules::BulbColor color;
  /// Bulb type (Round or Arrow).
  maliput::api::rules::BulbType type;
  /// Position of bulb frame origin relative to traffic light frame.
  maliput::math::Vector3 position_traffic_light;
  /// Orientation (quaternion [w, x, y, z]) of bulb frame relative to traffic light frame.
  maliput::math::Quaternion orientation_traffic_light;
  /// Possible states this bulb can be in. Defaults to [Off, On] if not specified.
  std::vector<maliput::api::rules::BulbState> states;
  /// Arrow orientation in radians (only for Arrow type bulbs).
  /// Required when type is Arrow.
  std::optional<double> arrow_orientation_rad;
  /// Custom bounding box for this bulb.
  /// If not specified, uses default maliput dimensions.
  maliput::api::rules::Bulb::BoundingBox bounding_box;

  bool operator==(const BulbDefinition& other) const {
    return id == other.id && color == other.color && type == other.type &&
           position_traffic_light == other.position_traffic_light &&
           orientation_traffic_light == other.orientation_traffic_light && states == other.states &&
           arrow_orientation_rad == other.arrow_orientation_rad && bounding_box.p_BMin == other.bounding_box.p_BMin &&
           bounding_box.p_BMax == other.bounding_box.p_BMax;
  }
  bool operator!=(const BulbDefinition& other) const { return !(*this == other); }
};

/// Unique identifier for a traffic signal definition.
struct TrafficControlDeviceFingerprint {
  /// Signal type identifier. Matches XODR signal type attribute.
  std::string type;
  /// Optional signal subtype for finer matching.
  std::optional<std::string> subtype;
  /// Optional country code or standard.
  std::optional<std::string> country;
  /// Optional country standard revision.
  std::optional<std::string> country_revision;

  bool operator==(const TrafficControlDeviceFingerprint& other) const {
    return type == other.type && subtype == other.subtype && country == other.country &&
           country_revision == other.country_revision;
  }
  bool operator!=(const TrafficControlDeviceFingerprint& other) const { return !(*this == other); }
};

/// Represents a traffic signal definition loaded from the database.
/// Describes the complete structure and rule logic for a particular signal.
struct TrafficControlDeviceDefinition {
  /// Unique identifier for this signal definition.
  TrafficControlDeviceFingerprint fingerprint;
  /// Human-readable description of this signal definition.
  std::string description;
  /// Device category parsed from the `device_type` field in the YAML database `properties` section.
  /// Required field; validated at parse time — kUnknown is rejected with a parser error.
  TrafficControlDeviceType device_type{TrafficControlDeviceType::kUnknown};
  /// Optional device semantics string (e.g. "stop", "give_way", "no_overtaking").
  /// Read from properties.device_semantics.
  std::optional<std::string> device_semantics;
  /// Whether the device position is dynamic (moveable). Defaults to false.
  bool is_position_dynamic{false};
  /// Optional device-level bounding box. When set, represents the overall device dimensions.
  /// Individual bulbs may still carry their own per-bulb bounding boxes.
  std::optional<maliput::api::rules::Bulb::BoundingBox> default_bounding_box;
  /// Bulbs in this traffic signal (only relevant when device_type == "traffic_light").
  std::vector<BulbDefinition> bulbs;
  /// Rule conditions mapping bulb state combinations to Right-Of-Way rule values.
  std::vector<RuleState> rule_states;

  bool operator==(const TrafficControlDeviceDefinition& other) const {
    const bool bbox_equal =
        (default_bounding_box.has_value() == other.default_bounding_box.has_value()) &&
        (!default_bounding_box.has_value() || (default_bounding_box->p_BMin == other.default_bounding_box->p_BMin &&
                                               default_bounding_box->p_BMax == other.default_bounding_box->p_BMax));
    return fingerprint == other.fingerprint && description == other.description && device_type == other.device_type &&
           device_semantics == other.device_semantics && is_position_dynamic == other.is_position_dynamic &&
           bbox_equal && bulbs == other.bulbs && rule_states == other.rule_states;
  }
  bool operator!=(const TrafficControlDeviceDefinition& other) const { return !(*this == other); }
};

/// Parser that loads traffic control device definitions from a YAML database using the `odr_signal_types`
/// schema. The resulting definitions are used in tandem with XODR signal data to create maliput
/// `TrafficLight` and `TrafficSign` objects.
///
/// Design:
/// - Each definition carries a `device_type` field that determines whether a `TrafficLight` or
///   `TrafficSign` is created for a given XODR signal.
/// - For traffic lights: `TrafficLight` is created with its bulbs and associated rule states.
/// - For traffic signs: a `TrafficSign` is created with its type derived from the `device_semantics` field.
///
/// Workflow:
/// 1. Load the device database: `TrafficControlDeviceParser::LoadFromFile()` or `LoadFromString()`.
/// 2. For each signal in the XODR file, match its `type`/`subtype`/`country`/`country_revision`
///    against a loaded `TrafficControlDeviceFingerprint`.
/// 3. Inspect `device_type` to route the signal to the appropriate builder:
///    - `"traffic_light"` → build a `TrafficLight`.
///    - `"traffic_sign"`  → build a `TrafficSign` whose type is mapped from `device_semantics`.
/// 4. Link the resulting objects to their affected lanes using signal validity data from the XODR file.
class TrafficControlDeviceParser {
 public:
  /// Loads a traffic control device database from a YAML string.
  ///
  /// @param yaml_content The YAML content as a string.
  /// @return Vector of signal definitions. Entries are validated for conflicts at load time.
  /// @throws maliput::common::road_network_description_parser_error if YAML parsing fails, schema validation
  ///         fails, or two entries with equal specificity overlap.
  static std::vector<TrafficControlDeviceDefinition> LoadFromString(const std::string& yaml_content);

  /// Loads a traffic control device database from a YAML file.
  ///
  /// @param file_path Path to the YAML database file.
  /// @return Vector of signal definitions. Entries are validated for conflicts at load time.
  /// @throws maliput::common::road_network_description_parser_error if YAML parsing fails, schema validation
  ///         fails, or two entries with equal specificity overlap.
  static std::vector<TrafficControlDeviceDefinition> LoadFromFile(const std::string& yaml_file_path);

  /// The wildcard value for traffic control device fingerprint fields.
  /// A field set to this value matches any query value for that field.
  static constexpr char kWildcard[] = "*";

  /// Returns true if @p value is the wildcard string.
  static bool IsWildcard(const std::string& value);

  /// Returns true if @p value is present and equals the wildcard string.
  static bool IsWildcard(const std::optional<std::string>& value);

  /// Computes the specificity of @p fp: the number of fields that are NOT wildcards.
  /// @p nullopt counts as specific (constrains to "absent"); only `"*"` is non-specific.
  /// Returns a value in [0, 4].
  static int Specificity(const TrafficControlDeviceFingerprint& fp);

  /// Returns true if @p db_entry matches @p query.
  /// A field in @p db_entry matches the corresponding @p query field if the db_entry field is the
  /// wildcard string or equals the query field. The @p query is expected to never contain wildcards.
  static bool Matches(const TrafficControlDeviceFingerprint& db_entry, const TrafficControlDeviceFingerprint& query);

  /// Returns true if there exists at least one input that would match both @p a and @p b.
  /// Per field: overlap if at least one is wildcard OR both are equal.
  static bool CanOverlap(const TrafficControlDeviceFingerprint& a, const TrafficControlDeviceFingerprint& b);

 private:
  /// Builds a vector of definitions from a parsed YAML root node.
  static std::vector<TrafficControlDeviceDefinition> BuildFrom(const YAML::Node& root);
};

}  // namespace traffic_control_device
}  // namespace malidrive

namespace std {

/// Hash function to use TrafficControlDeviceFingerprint as a key in unordered containers. Combines the hash of each
/// member variable.
template <>
struct hash<malidrive::traffic_control_device::TrafficControlDeviceFingerprint> {
  size_t operator()(const malidrive::traffic_control_device::TrafficControlDeviceFingerprint& f) const {
    size_t seed = 0;

    // https://www.boost.org/doc/libs/1_84_0/libs/container_hash/doc/html/hash.html#notes_hash_combine
    // During the Boost formal review, Dave Harris pointed out that this suffers from the so-called
    // "zero trap"; if seed is initially 0, and all the inputs are 0 (or hash to 0), seed remains 0 no
    // matter how many input values are combined.
    // This is an undesirable property, because it causes containers of zeroes to have a zero hash value
    // regardless of their sizes.
    // To fix this, the arbitrary constant 0x9e3779b9 (the golden ratio in a 32 bit fixed point
    // representation) was added to the computation.
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
