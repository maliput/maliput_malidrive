// Copyright 2019 Toyota Research Institute
#pragma once

#include "maliput/api/road_network.h"
#include "maliput/math/vector.h"

#include "maliput_malidrive/base/inertial_to_lane_mapping_config.h"
#include "maliput_malidrive/constants.h"

namespace malidrive {
namespace builder {

/// Holds the policy for concurrency optimization
/// during lane creation.
struct BuildPolicy {
  enum Type {
    kSequential = 0,
    kParallel,
  };

  /// Convert string to a Type.
  /// @param type String to be translated to a Type.
  /// @returns A BuildPolicy::Type value
  /// @throws maliput::common::assertion_error When `type` isn't a valid Type.
  static BuildPolicy::Type FromStrToType(const std::string& type);

  Type type{Type::kSequential};
  std::optional<int> num_threads{};
};

/// Structure to wrap RoadGeometry parameters.
struct RoadGeometryConfiguration {
  /// Determines the flexibility to follow the OpenDrive standard when building the road geometry.
  enum class StandardStrictnessPolicy {
    kPermissive,  ///< Allows to the road geometry to not completely meet the standard.
    kStrict,      ///< No flexibility at all.
  };

  /// Determines whether a simplification to the geometry should be applied or
  /// not.
  enum class SimplificationPolicy {
    kNone,                                         ///< No simplification must be done.
    kSimplifyWithinToleranceAndKeepGeometryModel,  ///< Merge geometry pieces and keep the original geometry model.
  };

  /// Determines whether the tolerance and scale length should be automatically
  /// computed or not.
  enum class ToleranceSelectionPolicy {
    kManualSelection,     ///< Manual adjustment.
    kAutomaticSelection,  ///< Automatic selection.
  };

  /// Convert a string to a SimplificationPolicy.
  /// @param policy String to be translated to a SimplificationPolicy.
  /// @returns A SimplificationPolicy value
  /// @throws maliput::common::assertion_error When `policy` isn't a valid SimplificationPolicy.
  static SimplificationPolicy FromStrToSimplificationPolicy(const std::string& policy);

  /// Convert a string to a ToleranceSelectionPolicy.
  /// @param policy String to be translated to a ToleranceSelectionPolicy.
  /// @returns A ToleranceSelectionPolicy value
  /// @throws maliput::common::assertion_error When `policy` isn't a valid ToleranceSelectionPolicy.
  static ToleranceSelectionPolicy FromStrToToleranceSelectionPolicy(const std::string& policy);

  /// Convert a string to a StandardStrictnessPolicy.
  /// @param policy String to be translated to a StandardStrictnessPolicy.
  /// @returns A StandardStrictnessPolicy value
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
