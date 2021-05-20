// Copyright 2019 Toyota Research Institute
#pragma once

#include <optional>
#include <string>

#include "maliput/api/road_network.h"
#include "maliput/math/vector.h"

#include "maliput_malidrive/base/inertial_to_lane_mapping_config.h"
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

  Type type{Type::kSequential};
  std::optional<int> num_threads{};
};

/// RoadGeometry construction parameters.
struct RoadGeometryConfiguration {
  /// Level of flexibility in terms of adhering to the OpenDrive standard when constructing a RoadGeometry. This is
  /// useful when working with .xodr files that violate the OpenDrive standard.
  enum class StandardStrictnessPolicy {
    kPermissive,  ///< Allow the .xodr file to violate the standard. Continue on best-effort-basis.
    kStrict,      ///< Do not allow violation of the standard.
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

  /// Converts a string to a ToleranceSelectionPolicy.
  ///
  /// @param policy String to be translated to a ToleranceSelectionPolicy. Valid strings match the enumerator's name
  /// except without the leading 'k' and is all lower case.
  /// @returns A ToleranceSelectionPolicy value.
  /// @throws maliput::common::assertion_error When `policy` isn't a valid ToleranceSelectionPolicy.
  static ToleranceSelectionPolicy FromStrToToleranceSelectionPolicy(const std::string& policy);

  /// Converts a string to a StandardStrictnessPolicy.
  ///
  /// @param policy String to be translated to a StandardStrictnessPolicy. Valid strings match the enumerator's name
  /// except without the leading 'k' and is all lower case.
  /// @returns A StandardStrictnessPolicy value.
  /// @throws maliput::common::assertion_error When `policy` isn't a valid StandardStrictnessPolicy.
  static StandardStrictnessPolicy FromStrToStandardStrictnessPolicy(const std::string& policy);

  // Designed for use with uniform initialization.
  maliput::api::RoadGeometryId id;
  std::optional<std::string> opendrive_file;
  double linear_tolerance{constants::kLinearTolerance};
  double angular_tolerance{constants::kAngularTolerance};
  double scale_length{constants::kScaleLength};
  maliput::math::Vector3 inertial_to_backend_frame_translation{0., 0., 0.};
  InertialToLaneMappingConfig inertial_to_lane_mapping_config;
  BuildPolicy build_policy{BuildPolicy::Type::kSequential};
  SimplificationPolicy simplification_policy{SimplificationPolicy::kNone};
  ToleranceSelectionPolicy tolerance_selection_policy{ToleranceSelectionPolicy::kManualSelection};
  StandardStrictnessPolicy standard_strictness_policy{StandardStrictnessPolicy::kPermissive};
};

}  // namespace builder
}  // namespace malidrive
