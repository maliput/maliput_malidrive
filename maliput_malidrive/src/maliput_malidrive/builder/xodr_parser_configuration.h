// Copyright 2021 Toyota Research Institute
#pragma once

#include "maliput_malidrive/builder/road_geometry_configuration.h"

#include "maliput_malidrive/xodr/parser_configuration.h"

namespace malidrive {
namespace builder {

/// Builds a xodr::ParserConfiguration from a RoadGeometryConfiguration struct.
///
/// @param rg_config The RoadGeometryConfiguration struct.
/// @return A xodr::ParserConfiguration struct.
xodr::ParserConfiguration XodrParserConfigurationFromRoadGeometryConfiguration(
    const RoadGeometryConfiguration& rg_config);

}  // namespace builder
}  // namespace malidrive
