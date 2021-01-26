// Copyright 2021 Toyota Research Institute
#pragma once

#include <optional>
#include <string>

#include "maliput_malidrive/builder/road_geometry_configuration.h"

namespace malidrive {
namespace test {

/// Obtains the builder::RoadGeometryConfiguration for a specific xodr file name.
///
/// This function holds a curated dictionary of xodr maps and their geometric
/// configurations.
///
/// @param xodr_file_name The XODR map file name.
/// @return A builder::RoadGeometryConfiguration for the specified
/// @p xodr_file_name or std::nullopt when it does not exist.
std::optional<builder::RoadGeometryConfiguration> GetRoadGeometryConfgurationFor(const std::string& xodr_file_name);

}  // namespace test
}  // namespace malidrive
