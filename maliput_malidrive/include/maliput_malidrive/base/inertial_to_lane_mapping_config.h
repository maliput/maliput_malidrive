// Copyright 2019 Toyota Research Institute
// TODO(#701): Move file to malidrive package when this is no longer needed at RoadGeometryConfiguration.
#pragma once

#include <maliput/common/maliput_abort.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {

/// Holds the Inertial to Lane mapping information.
struct InertialToLaneMappingConfig {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InertialToLaneMappingConfig)

  InertialToLaneMappingConfig() = delete;

  /// Constructs a InertialToLaneMappingConfig.
  ///
  /// @param exploration_radius_step_in is the base and step when increasing the
  ///        exploration radius. It must be positive.
  /// @param max_iterations_in is the maximum number of iterations that
  ///        intersectCircle should try with increasing radii. It must be
  ///        positive.
  InertialToLaneMappingConfig(double exploration_radius_step_in, int max_iterations_in)
      : exploration_radius_step(exploration_radius_step_in), max_iterations(max_iterations_in) {
    MALIDRIVE_THROW_UNLESS(exploration_radius_step > 0);
    MALIDRIVE_THROW_UNLESS(max_iterations > 0);
  }
  double exploration_radius_step{};
  int max_iterations{};
};

}  // namespace malidrive
