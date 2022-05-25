// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
  /// @details The keys of the map are listed at @ref road_geometry_configuration_builder_keys.
  /// @param road_geometry_configuration A string-string map containing the configuration for the builder.
  static RoadGeometryConfiguration FromMap(const std::map<std::string, std::string>& road_geometry_configuration);

  /// Holds linear and angular tolerance to be used by the builder.
  /// A range could be selected for linear tolerance, allowing the builder to try
  /// different values of linear tolerances within that range searching for a value that works.
  struct BuildTolerance {
    /// Sets angular tolerance.
    /// @param angular_tolerance_in angular tolerance.
    explicit BuildTolerance(double angular_tolerance_in);

    /// Sets linear and angular tolerance.
    /// @param linear_tolerance_in linear tolerance.
    /// @param angular_tolerance_in angular tolerance.
    explicit BuildTolerance(double linear_tolerance_in, double angular_tolerance_in);

    /// Sets linear tolerance range and angular tolerance.
    /// @param min_linear_tolerance_in minimum linear tolerance to be used.
    /// @param max_linear_tolerance_in maximum linear tolerance to be used.
    /// @param angular_tolerance_in angular tolerance.
    explicit BuildTolerance(double min_linear_tolerance_in, double max_linear_tolerance_in,
                            double angular_tolerance_in);
    /// Nominal linear_tolerance to be used in the RoadGeometry. Corresponds to the minimumm range of linear tolerances
    /// when `max_linear_tolerance.has_value` is true.
    std::optional<double> linear_tolerance{std::nullopt};
    /// Maximum range of linear tolerances.
    std::optional<double> max_linear_tolerance{std::nullopt};
    /// Angular tolerance to be used in the RoadGeometry.
    double angular_tolerance{constants::kAngularTolerance};
  };

  /// @details The keys of the map are listed at @ref road_geometry_configuration_builder_keys.
  /// @returns A string-string map containing the RoadGeometry configuration.
  std::map<std::string, std::string> ToStringMap() const;

  /// @name Road Geometry Configuration's parameters.
  ///
  /// @{
  maliput::api::RoadGeometryId id{"maliput"};
  std::string opendrive_file{""};
  BuildTolerance tolerances{constants::kAngularTolerance};
  double scale_length{constants::kScaleLength};
  maliput::math::Vector3 inertial_to_backend_frame_translation{0., 0., 0.};
  BuildPolicy build_policy{BuildPolicy::Type::kSequential};
  SimplificationPolicy simplification_policy{SimplificationPolicy::kNone};
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
