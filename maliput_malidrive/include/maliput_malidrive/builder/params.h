// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

namespace malidrive {
namespace builder {
namespace params {

/// @defgroup road_network_configuration_builder_keys RoadNetwork configuration builder keys
///
/// Parameters used during the RoadNetwork building process.
///
/// When parameters are omitted the default value will be used.
///
/// Example of use:
/// @code{cpp}
/// const std::map<std::string, std::string> road_network_configuration {
///   {"malidrive::builder::params::kRoadGeometryId", "appian_way_road_geometry"},
///   {"malidrive::builder::params::kOpendriveFile", "appian_way.xodr"},
///   {"malidrive::builder::params::kLinearTolerance", "1e-3"},
///   {"malidrive::builder::params::kAngularTolerance", "1e-3"},
///   {"malidrive::builder::params::kInertialToBackendFrameTranslation", "{2.0, 2.0, 0.0}"},
/// };
/// auto road_network = malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(road_network_configuration);
/// @endcode
///
/// Parameters related to the creation of the RoadGeometry can be seen at @ref road_geometry_configuration_builder_keys.
///
/// Parameters for passing filepath to road rulebook, traffic light book, phase ring book and intersection book are
/// listed below.
/// @{

/// Path to the configuration file to load a RoadRulebook
///   - Default: ""
static constexpr char const* kRoadRuleBook{"road_rule_book"};

/// Path to the configuration file to load a RoadRulebook
///   - Default: ""
static constexpr char const* kRuleRegistry{"rule_registry"};

/// Path to the configuration file to load a TrafficLightBook
///   - Default: ""
static constexpr char const* kTrafficLightBook{"traffic_light_book"};

/// Path to the configuration file to load a PhaseRingBook
///   - Default: ""
static constexpr char const* kPhaseRingBook{"phase_ring_book"};

/// Path to the configuration file to load a IntersectionBook
///   - Default: ""
static constexpr char const* kIntersectionBook{"intersection_book"};

/// @}

/// @defgroup road_geometry_configuration_builder_keys RoadGeometry configuration builder keys
/// Parameters used during the RoadGeometry building process.
/// See @ref road_network_configuration_builder_keys for further information.
///
/// @ingroup road_network_configuration_builder_keys
/// @{

/// A string that works as ID of the RoadGeometry.
///   - Default: @e "maliput"
static constexpr char const* kRoadGeometryId{"road_geometry_id"};

/// Path to the XODR file to be loaded.
///   - Default: ""
static constexpr char const* kOpendriveFile{"opendrive_file"};

/// RoadGeometry's linear tolerance.
///   - Default: By default it isn't set. When `max_linear_tolerance` isn't also set then @e "5e-2"
///   (#malidrive::constants::kLinearTolerance) is used.
static constexpr char const* kLinearTolerance{"linear_tolerance"};

/// A maximum allowed linear tolerance.
/// When this parameter is passed, the linear tolerance the builder will use
/// is defined within the range [`linear_tolerance`, `max_linear_tolerance`].
/// The builder is expected to iteratively try higher linear tolerances until it either finds
/// a value that works, or reaches this maximum value, at which point it will abort with a
/// failure.
/// When `linear_tolerance` isn't set, the minimum value of the range will be defined by
/// #malidrive::constants::kBaseLinearTolerance. It is recommended to define the minimum
/// value using the `linear_tolerance` parameter to hint the builder to a valid range,
/// otherwise this method could demand a considerable extra time while it tries out
/// relatively small linear tolerances.
///   - Default: By default it isn't set. The builder will try only `linear_tolerance`.
static constexpr char const* kMaxLinearTolerance{"max_linear_tolerance"};

/// RoadGeometry's angular tolerance.
///   - Default: @e "1e-3" (#malidrive::constants::kAngularTolerance)
static constexpr char const* kAngularTolerance{"angular_tolerance"};

/// RoadGeometry's scale length.
///   - Default: @e "1.0" (#malidrive::constants::kScaleLength)
static constexpr char const* kScaleLength{"scale_length"};

/// Translation from maliput to malidrive inertial frame.
/// The format of the 3-dimensional vector that is expected to be passed
/// should be {X , Y , Z}. Same format as maliput::math::Vector3 is
/// serialized.
///   - Default: @e "{0., 0., 0.}"
static constexpr char const* kInertialToBackendFrameTranslation{"inertial_to_backend_frame_translation"};

/// Determines the use of concurrency when building the RoadGeometry.
///   - Options:
///     - 1. @e "sequential"
///     - 2. @e "parallel"
///   - Default: @e "sequential"
static constexpr char const* kBuildPolicy{"build_policy"};

/// Indicates the number of threads to be used to build the RoadGeometry when `build_policy` flag is
/// set to `parallel`.
///    - Default: std::thread::hardware_concurrency() minus one (running thread).
static constexpr char const* kNumThreads{"num_threads"};

/// Determines geometries simplification for the XODR's roads.
///   - Options:
///     - 1. @e "none"
///     - 2. @e "simplify"
///   - Default: @e "none"
static constexpr char const* kSimplificationPolicy{"simplification_policy"};

/// Indicates how permissive builder should be with the XODR description.
///   - Options:
///    - 1. @e "strict" : Do not permit any errors.
///    - 2. @e "allow_schema_errors" : Allow schema syntax errors at a XODR level:
///      - 1. Allows having two functions starting at same 's' adding up to a picewise-defined polynomial for a given
///      XODR characteristic(laneWidth, laneOffset, etc): First description will be discarded.
///      - 2. Allows having functions starting at the end of the road: This function will be discarded as its length is
///      zero.
///      - 3. Allows having functions with NaN values. It is only accepted when there is a function that starts at
///      the same 's' so the function with NaN values can be discarded.
///      - 4. Allows having junction nodes without any connection.
///    - 3. @e "allow_semantic_errors" : Allow semantic errors.
///      - 1. At a XODR level it allows having non reciprocal Road-linkage and Lane-linkage within a Road.
///      - 2. Allows having lane width descriptions that are negative in certain range.
///      - 3. Disables G1 contiguity checks for non-drivable lanes. (G1 contiguity check on drivable lanes can't be
///      disabled.)
///    - 4. @e "permissive": Allow all previous violations.
///   - Default: @e "permissive"
static constexpr char const* kStandardStrictnessPolicy{"standard_strictness_policy"};

/// True for omitting building non-drivable lanes. False otherwise
///   - Options:
///     - 1. <em> "true", "True", "TRUE", "on", "On", "ON" </em>
///     - 2. <em> "false", "False",  "FALSE", "off", "Off", "OFF" </em>
///   - Default: @e "true"
static constexpr char const* kOmitNonDrivableLanes{"omit_nondrivable_lanes"};

/// @}

}  // namespace params
}  // namespace builder
}  // namespace malidrive
