// Copyright 2019 Toyota Research Institute
#pragma once

#include <optional>
#include <string>

#include <maliput/api/road_network.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/constants.h"

namespace malidrive {
namespace builder {

/// Policy for use of concurrency while building a RoadGeometry.
struct BuildPolicy {
  enum Type {
    kSequential = 0,
    kParallel,
  };

  /// Converts string to a Type.
  ///
  /// @param type String to be translated to a Type. Valid strings match the enumerator's name except without the
  /// leading 'k' and is all lower case.
  /// @returns A BuildPolicy::Type value
  /// @throws maliput::common::assertion_error When `type` isn't a valid Type.
  static BuildPolicy::Type FromStrToType(const std::string& type);

  /// Converts a Type to a string.
  ///
  /// @param type Type to be translated to a string. Enumerator's name matches the string except
  /// without the leading 'k' and is all lower case.
  /// @returns A string.
  static std::string FromTypeToStr(const BuildPolicy::Type& type);

  Type type{Type::kSequential};
  std::optional<int> num_threads{};
};

/// RoadGeometry construction parameters.
struct RoadGeometryConfiguration {
  /// @name Road Geometry configuration's keys used in string map.
  ///
  /// @{
  static constexpr char const* kStrRoadGeometryId = "road_geometry_id";
  static constexpr char const* kStrOpendriveFile = "opendrive_file";
  static constexpr char const* kStrLinearTolerance = "linear_tolerance";
  static constexpr char const* kStrAngularTolerance = "angular_tolerance";
  static constexpr char const* kStrScaleLength = "scale_length";
  static constexpr char const* kStrInertialToBackendFrameTranslation = "inertial_to_backend_frame_translation";
  static constexpr char const* kStrBuildPolicy = "build_policy";
  static constexpr char const* kStrNumThreads = "num_threads";
  static constexpr char const* kStrSimplificationPolicy = "simplification_policy";
  static constexpr char const* kStrToleranceSelectionPolicy = "tolerance_selection_policy";
  static constexpr char const* kStrStandardStrictnessPolicy = "standard_strictness_policy";
  static constexpr char const* kStrOmitNonDrivableLanes = "omit_nondrivable_lanes";
  /// @}

  /// Level of flexibility in terms of adhering to the OpenDrive standard
  /// when constructing a RoadGeometry. This is useful when working with
  /// .xodr files that violate the OpenDrive standard. The policy is
  /// specified by taking the union of one or more of the following bit
  /// flags.
  enum class StandardStrictnessPolicy : unsigned int {
    kStrict = 0,                                             ///< Do not permit any errors.
    kAllowSchemaErrors = 1 << 0,                             ///< Allow schema syntax errors.
    kAllowSemanticErrors = 1 << 1,                           ///< Allow semantic errors.
    kPermissive = kAllowSemanticErrors | kAllowSchemaErrors  ///< Allow all previous violations.
  };

  /// Degree to which the RoadGeometry should be simplified.
  enum class SimplificationPolicy {
    kNone,                                         ///< No simplification must be done.
    kSimplifyWithinToleranceAndKeepGeometryModel,  ///< Merge geometry pieces and keep the original geometry model.
  };

  /// How the tolerance and scale length should be computed.
  enum class ToleranceSelectionPolicy {
    kManualSelection,     ///< Manual adjustment.
    kAutomaticSelection,  ///< Automatic selection.
  };

  /// Converts a string to a SimplificationPolicy.
  ///
  /// @param policy String to be translated to a SimplificationPolicy. Valid strings match the enumerator's name except
  /// without the leading 'k' and is all lower case.
  /// @returns A SimplificationPolicy value.
  /// @throws maliput::common::assertion_error When `policy` isn't a valid SimplificationPolicy.
  static SimplificationPolicy FromStrToSimplificationPolicy(const std::string& policy);

  /// Converts a SimplificationPolicy to a string.
  ///
  /// @param policy SimplificationPolicy to be translated to a string. Enumerator's name matches the string except
  /// without the leading 'k' and is all lower case.
  /// @returns A string.
  static std::string FromSimplificationPolicyToStr(const SimplificationPolicy& policy);

  /// Converts a string to a ToleranceSelectionPolicy.
  ///
  /// @param policy String to be translated to a ToleranceSelectionPolicy. Valid strings match the enumerator's name
  /// except without the leading 'k' and is all lower case.
  /// @returns A ToleranceSelectionPolicy value.
  /// @throws maliput::common::assertion_error When `policy` isn't a valid ToleranceSelectionPolicy.
  static ToleranceSelectionPolicy FromStrToToleranceSelectionPolicy(const std::string& policy);

  /// Converts a ToleranceSelectionPolicy to a string.
  ///
  /// @param policy ToleranceSelectionPolicy to be translated to a string. Enumerator's name matches the string except
  /// without the leading 'k' and is all lower case.
  /// @returns A string.
  static std::string FromToleranceSelectionPolicyToStr(const ToleranceSelectionPolicy& policy);

  /// Converts a string to a StandardStrictnessPolicy.
  ///
  /// @param policy String to be translated to a StandardStrictnessPolicy. Valid strings match the enumerator's name
  /// except without the leading 'k' and is all lower case.
  /// @returns A StandardStrictnessPolicy value.
  /// @throws maliput::common::assertion_error When `policy` isn't a valid StandardStrictnessPolicy.
  static StandardStrictnessPolicy FromStrToStandardStrictnessPolicy(const std::string& policy);

  /// Converts a StandardStrictnessPolicy to a string.
  ///
  /// @param policy StandardStrictnessPolicy to be translated to a String. Valid strings match the enumerator's name
  /// except without the leading 'k' and is all lower case.
  /// @returns A string value.
  static std::string FromStandardStrictnessPolicyToStr(const StandardStrictnessPolicy& policy);

  /// Creates a RoadGeometryConfiguration out of a string dictionary.
  /// @param road_geometry_configuration A string-string map containing the configuration for the builder.
  static RoadGeometryConfiguration FromMap(const std::map<std::string, std::string>& road_geometry_configuration);

  /// @returns A string-string map containing the RoadGeometry configuration.
  std::map<std::string, std::string> ToStringMap() const;

  /// @name Road Geometry Configuration's parameters.
  ///
  /// @{
  maliput::api::RoadGeometryId id{"maliput"};
  std::string opendrive_file{""};
  double linear_tolerance{constants::kLinearTolerance};
  double angular_tolerance{constants::kAngularTolerance};
  double scale_length{constants::kScaleLength};
  maliput::math::Vector3 inertial_to_backend_frame_translation{0., 0., 0.};
  BuildPolicy build_policy{BuildPolicy::Type::kSequential};
  SimplificationPolicy simplification_policy{SimplificationPolicy::kNone};
  ToleranceSelectionPolicy tolerance_selection_policy{ToleranceSelectionPolicy::kAutomaticSelection};
  StandardStrictnessPolicy standard_strictness_policy{StandardStrictnessPolicy::kPermissive};
  // Note: this flag will change its default to false. It might lead to badly
  // constructed RoadGeometries. Consider the following case:
  // @code{xml}
  // <left>
  //   <lane id="2" type="driving" level="false">
  //     <width sOffset="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
  //   </lane>
  //   <lane id="1" type="shoulder" level= "0">
  //     <width sOffset="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
  //   </lane>
  // </left>
  // <center>
  //     <lane id="0" type="none" level= "0"/>
  // </center>
  // @endcode
  // Lane 1 will not be considered but lane 2 yes. However, because of omitting
  // lane 1, the lane 2 will have an incorrect lane offset function.
  bool omit_nondrivable_lanes{true};
  /// @}
};

// Union operator.
RoadGeometryConfiguration::StandardStrictnessPolicy operator|(
    const RoadGeometryConfiguration::StandardStrictnessPolicy& first,
    const RoadGeometryConfiguration::StandardStrictnessPolicy& second);
// Intersection operator.
RoadGeometryConfiguration::StandardStrictnessPolicy operator&(
    const RoadGeometryConfiguration::StandardStrictnessPolicy& first,
    const RoadGeometryConfiguration::StandardStrictnessPolicy& second);

}  // namespace builder
}  // namespace malidrive
