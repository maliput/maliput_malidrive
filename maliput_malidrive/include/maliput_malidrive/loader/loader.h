#pragma once

#include <map>
#include <memory>
#include <string>

#include <maliput/api/road_network.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace loader {

/// Builds a malidrive RoadNetwork.
///
/// Forwards a call to RoadNetworkBuilderT.
///
/// @param road_network_configuration A string-string map containing information about the RoadNetwork configuration
/// used during the loading process.
///
/// @details The parameters that can be set are the following:
/// - @b road_geometry_id : A string that works as ID of the RoadGeometry.
///   - Default: @e "maliput"
/// - @b opendrive_file : Path to the XODR file to be loaded.
///   - Default: ""
/// - @b linear_tolerance : RoadGeometry's linear tolerance.
///   - Default: @e "5e-2" (#malidrive::constants::kLinearTolerance)
/// - @b angular_tolerance : RoadGeometry's angular tolerance.
///   - Default: @e "1e-3" (#malidrive::constants::kAngularTolerance)
/// - @b scale_length : RoadGeometry's scale length.
///   - Default: @e "1.0" (#malidrive::constants::kScaleLength)
/// - @b inertial_to_backend_frame_translation : Translation from maliput to malidrive inertial frame.
///                                             The format of the 3-dimensional vector that is expected to be passed
///                                             should be {X , Y , Z}. Same format as maliput::math::Vector3 is
///                                             serialized.
///   - Default: @e "{0., 0., 0.}"
/// - @b build_policy : Determines the use of concurrency when building the RoadGeometry.
///   - Options:
///     - 1. @e "sequential"
///     - 2. @e "parallel"
///   - Default: @e "sequential"
/// - @b num_threads : Indicates the number of threads to be used to build the RoadGeometry when `build_policy` flag is
/// set to `parallel`.
///    - Default: std::thread::hardware_concurrency() minus one (running thread).
/// - @b simplification_policy : Determines geometries simplification for the XODR's roads.
///   - Options:
///     - 1. @e "none"
///     - 2. @e "simplify"
///   - Default: @e "none"
/// - @b tolerance_selection_policy : Tolerance selection method used by the builder.
///   - Options:
///    - 1. "manual" : @e linear_tolerance will be used.
///    - 2. "automatic" : @e linear_tolerance is used as base and bigger tolerances are used when the base tolerance
///    make the builder to fail.
///   - Default: @e "automatic"
/// - @b standard_strictness_policy : Indicates how permissive builder should be with the XODR description.
///   - Options:
///    - 1. @e "strict" : Do not permit any errors.
///    - 2. @e "allow_schema_errors" : Allow schema syntax errors.
///    - 3. @e "allow_semantic_errors" : Allow semantic errors.
///    - 4. @e "permissive": Allow all previous violations.
///   - Default: @e "permissive"
/// - @b omit_nondrivable_lanes : True for omitting building non-drivable lanes. False otherwise
///   - Options:
///     - 1. <em> "true", "True", "TRUE", "on", "On", "ON" </em>
///     - 2. <em> "false", "False",  "FALSE", "off", "Off", "OFF" </em>
///   - Default: @e "true"
/// - @b road_rule_book : Path to the configuration file to load a RoadRulebook
///   - Default: ""
/// - @b traffic_light_book : Path to the configuration file to load a TrafficLightBook
///   - Default: ""
/// - @b phase_ring_book : Path to the configuration file to load a PhaseRingBook
///   - Default: ""
/// - @b intersection_book : Path to the configuration file to load a IntersectionBook
///   - Default: ""
///
/// When parameters are omitted the default value will be used.
///
/// Example of use:
/// @code{cpp}
/// const std::map<std::string, std::string> road_network_configuration {
///   {"road_geometry_id", "appian_way_road_geometry"},
///   {"opendrive_file", "appian_way.xodr"},
///   {"linear_tolerance", "1e-3"},
///   {"angular_tolerance", "1e-3"},
///   {"inertial_to_backend_frame_translation", "{2.0, 2.0, 0.0}"},
/// };
/// auto road_network = malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(road_network_configuration);
/// @endcode
///
/// @return An unique_ptr to a RoadNetwork.
/// @tparam RoadNetworkBuilderT One of opendrive::builder::RoadNetworkBuilder or
///         builder::RoadNetworkBuilder.
template <class RoadNetworkBuilderT>
std::unique_ptr<maliput::api::RoadNetwork> Load(const std::map<std::string, std::string>& road_network_configuration) {
  return RoadNetworkBuilderT(road_network_configuration)();
}

}  // namespace loader
}  // namespace malidrive
