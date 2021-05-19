// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_configuration.h"

#include <map>

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
    {"permissive", RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive}};

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
  const auto it = str_to_standard_strictness_policy.find(policy);
  if (it == str_to_standard_strictness_policy.end()) {
    MALIDRIVE_THROW_MESSAGE("Unknown standard strictness policy: " + policy);
  }
  return str_to_standard_strictness_policy.at(policy);
}

}  // namespace builder
}  // namespace malidrive
