// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_configuration.h"

#include <map>
#include <sstream>

namespace malidrive {
namespace builder {
namespace {

// Holds the conversion from string(keys) to BuildPolicy::Type(values);
const std::map<std::string, BuildPolicy::Type> str_to_build_policy_type{{"sequential", BuildPolicy::Type::kSequential},
                                                                        {"parallel", BuildPolicy::Type::kParallel}};

// Holds the conversion from string(keys) to SimplificationPolicy(values);
const std::map<std::string, RoadGeometryConfiguration::SimplificationPolicy> str_to_simplification_policy{
    {"none", RoadGeometryConfiguration::SimplificationPolicy::kNone},
    {"simplify", RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel}};

// Holds the conversion from string(keys) to ToleranceSelectionPolicy(values);
const std::map<std::string, RoadGeometryConfiguration::ToleranceSelectionPolicy> str_to_tolerance_selection_policy{
    {"manual", RoadGeometryConfiguration::ToleranceSelectionPolicy::kManualSelection},
    {"automatic", RoadGeometryConfiguration::ToleranceSelectionPolicy::kAutomaticSelection}};

// Holds the conversion from string(keys) to StandardStrictnessPolicy(values);
const std::map<std::string, RoadGeometryConfiguration::StandardStrictnessPolicy> str_to_standard_strictness_policy{
    {"strict", RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict},
    {"allow_schema_errors", RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors},
    {"allow_semantic_errors", RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors},
    {"permissive", RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive}};

// Holds the conversion from StandardStrictnessPolicy(keys) to string(values);
const std::map<RoadGeometryConfiguration::StandardStrictnessPolicy, std::string> standard_strictness_policy_to_str{
    {RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict, "strict"},
    {RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors, "allow_schema_errors"},
    {RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors, "allow_semantic_errors"},
    {RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive, "permissive"}};

// Returns a vector of strings as a result of splitting @p text by @p character.
// Example:
//    If `text` is "first|second|three" and the `token` is '|' then
//    the return value is the collection: {"first", "second", "three"}"
std::vector<std::string> SplitTextBy(const std::string& text, char token) {
  std::istringstream ss{text};
  std::vector<std::string> strings;
  std::string split_text;
  while (std::getline(ss, split_text, token)) {
    strings.push_back(split_text);
  }
  return strings;
}

}  // namespace

BuildPolicy::Type BuildPolicy::FromStrToType(const std::string& type) {
  const auto it = str_to_build_policy_type.find(type);
  if (it == str_to_build_policy_type.end()) {
    MALIDRIVE_THROW_MESSAGE("Unknown build policy type: " + type);
  }
  return str_to_build_policy_type.at(type);
}

RoadGeometryConfiguration::SimplificationPolicy RoadGeometryConfiguration::FromStrToSimplificationPolicy(
    const std::string& policy) {
  const auto it = str_to_simplification_policy.find(policy);
  if (it == str_to_simplification_policy.end()) {
    MALIDRIVE_THROW_MESSAGE("Unknown simplification policy: " + policy);
  }
  return str_to_simplification_policy.at(policy);
}

RoadGeometryConfiguration::ToleranceSelectionPolicy RoadGeometryConfiguration::FromStrToToleranceSelectionPolicy(
    const std::string& policy) {
  const auto it = str_to_tolerance_selection_policy.find(policy);
  if (it == str_to_tolerance_selection_policy.end()) {
    MALIDRIVE_THROW_MESSAGE("Unknown tolerance selection policy: " + policy);
  }
  return str_to_tolerance_selection_policy.at(policy);
}

RoadGeometryConfiguration::StandardStrictnessPolicy RoadGeometryConfiguration::FromStrToStandardStrictnessPolicy(
    const std::string& policy) {
  const char token = '|';
  const auto keys = SplitTextBy(policy, token);
  RoadGeometryConfiguration::StandardStrictnessPolicy strictness{
      RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict};
  for (const auto& key : keys) {
    const auto it = str_to_standard_strictness_policy.find(key);
    if (it == str_to_standard_strictness_policy.end()) {
      MALIDRIVE_THROW_MESSAGE("Unknown standard strictness policy: " + key);
    }
    strictness = strictness | str_to_standard_strictness_policy.at(key);
  }
  return strictness;
}

std::string RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(
    const RoadGeometryConfiguration::StandardStrictnessPolicy& policy) {
  switch (policy) {
    case RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict:
      return standard_strictness_policy_to_str.at(RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict);
      break;
    case RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive:
      return standard_strictness_policy_to_str.at(RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive);
      break;
    case RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors:
      return standard_strictness_policy_to_str.at(
          RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors);
      break;
    case RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors:
      return standard_strictness_policy_to_str.at(
          RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors);
      break;
    default:
      MALIPUT_THROW_MESSAGE("Unknown standard strictness policy.");
      break;
  }
}

RoadGeometryConfiguration::StandardStrictnessPolicy operator|(
    const RoadGeometryConfiguration::StandardStrictnessPolicy& first,
    const RoadGeometryConfiguration::StandardStrictnessPolicy& second) {
  return RoadGeometryConfiguration::StandardStrictnessPolicy(static_cast<unsigned int>(first) |
                                                             static_cast<unsigned int>(second));
}

RoadGeometryConfiguration::StandardStrictnessPolicy operator&(
    const RoadGeometryConfiguration::StandardStrictnessPolicy& first,
    const RoadGeometryConfiguration::StandardStrictnessPolicy& second) {
  return RoadGeometryConfiguration::StandardStrictnessPolicy(static_cast<unsigned int>(first) &
                                                             static_cast<unsigned int>(second));
}

}  // namespace builder
}  // namespace malidrive
