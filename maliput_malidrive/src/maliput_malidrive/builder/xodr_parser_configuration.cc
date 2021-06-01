// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/builder/xodr_parser_configuration.h"

namespace malidrive {
namespace builder {

xodr::ParserConfiguration XodrParserConfigurationFromRoadGeometryConfiguration(
    const RoadGeometryConfiguration& rg_config) {
  return {rg_config.linear_tolerance,
          (rg_config.standard_strictness_policy &
           RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors) ==
              RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors,
          (rg_config.standard_strictness_policy &
           RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors) ==
              RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors};
}

}  // namespace builder
}  // namespace malidrive
