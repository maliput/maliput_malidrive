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
#include "maliput_malidrive/builder/road_network_configuration.h"

#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "maliput_malidrive/builder/params.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

class RoadNetworkConfigurationTest : public ::testing::Test {
 protected:
  const maliput::math::Vector3 kRandomVector{1., 2., 3.};
  const BuildPolicy kBuildPolicy{BuildPolicy::Type::kParallel};
  const RoadGeometryConfiguration::SimplificationPolicy kSimplificationPolicy{
      RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel};
  const RoadGeometryConfiguration::StandardStrictnessPolicy kStandardStrictnessPolicy{
      RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive};
  const bool kOmitNondrivableLanes{false};
  const std::string kRgId{"test_id"};
  const std::string kOpendriveFile{"opendrive_file_test.xodr"};
  const std::optional<std::string> kRuleRegistry{"rule_registry_test.xodr"};
  const std::optional<std::string> kRoadRuleBook{"road_rule_book_test.xodr"};
  const std::optional<std::string> kTrafficLightBook{"traffic_light_book_test.xodr"};
  const std::optional<std::string> kPhaseRingBook{"phase_ring_book_test.xodr"};
  const std::optional<std::string> kIntersectionBook{"intersection_book_test.xodr"};
  const double kLinearTolerance{5e-5};
  const double kMaxLinearTolerance{5e-4};
  const double kAngularTolerance{5e-5};
  const double kScaleLength{2.};

  void ExpectEqual(const RoadNetworkConfiguration& lhs, const RoadNetworkConfiguration& rhs) {
    // RoadNetworkConfiguration parameters.
    EXPECT_EQ(lhs.rule_registry, rhs.rule_registry);
    EXPECT_EQ(lhs.road_rule_book, rhs.road_rule_book);
    EXPECT_EQ(lhs.traffic_light_book, rhs.traffic_light_book);
    EXPECT_EQ(lhs.phase_ring_book, rhs.phase_ring_book);
    EXPECT_EQ(lhs.intersection_book, rhs.intersection_book);
    // RoadGeometryConfiguration parameteres.
    EXPECT_EQ(lhs.road_geometry_configuration.id, rhs.road_geometry_configuration.id);
    EXPECT_EQ(lhs.road_geometry_configuration.opendrive_file, rhs.road_geometry_configuration.opendrive_file);
    EXPECT_EQ(lhs.road_geometry_configuration.tolerances.linear_tolerance,
              rhs.road_geometry_configuration.tolerances.linear_tolerance);
    EXPECT_EQ(lhs.road_geometry_configuration.tolerances.max_linear_tolerance,
              rhs.road_geometry_configuration.tolerances.max_linear_tolerance);
    EXPECT_EQ(lhs.road_geometry_configuration.tolerances.angular_tolerance,
              rhs.road_geometry_configuration.tolerances.angular_tolerance);
    EXPECT_EQ(lhs.road_geometry_configuration.scale_length, rhs.road_geometry_configuration.scale_length);
    EXPECT_EQ(lhs.road_geometry_configuration.inertial_to_backend_frame_translation,
              rhs.road_geometry_configuration.inertial_to_backend_frame_translation);
    EXPECT_EQ(lhs.road_geometry_configuration.build_policy.type, rhs.road_geometry_configuration.build_policy.type);
    EXPECT_EQ(lhs.road_geometry_configuration.build_policy.num_threads,
              rhs.road_geometry_configuration.build_policy.num_threads);
    EXPECT_EQ(lhs.road_geometry_configuration.simplification_policy,
              rhs.road_geometry_configuration.simplification_policy);
    EXPECT_EQ(lhs.road_geometry_configuration.standard_strictness_policy,
              rhs.road_geometry_configuration.standard_strictness_policy);
    EXPECT_EQ(lhs.road_geometry_configuration.omit_nondrivable_lanes,
              rhs.road_geometry_configuration.omit_nondrivable_lanes);
  }
};

TEST_F(RoadNetworkConfigurationTest, Constructor) {
  const RoadGeometryConfiguration rg_config{
      maliput::api::RoadGeometryId{kRgId},
      kOpendriveFile,
      builder::RoadGeometryConfiguration::BuildTolerance{kLinearTolerance, kMaxLinearTolerance, kAngularTolerance},
      kScaleLength,
      kRandomVector,
      kBuildPolicy,
      kSimplificationPolicy,
      kStandardStrictnessPolicy,
      kOmitNondrivableLanes};
  RoadNetworkConfiguration dut1{rg_config,         kRuleRegistry,  kRoadRuleBook,
                                kTrafficLightBook, kPhaseRingBook, kIntersectionBook};

  const std::map<std::string, std::string> rn_config_map{
      {params::kRoadGeometryId, kRgId},
      {params::kOpendriveFile, kOpendriveFile},
      {params::kLinearTolerance, std::to_string(kLinearTolerance)},
      {params::kMaxLinearTolerance, std::to_string(kMaxLinearTolerance)},
      {params::kAngularTolerance, std::to_string(kAngularTolerance)},
      {params::kScaleLength, std::to_string(kScaleLength)},
      {params::kInertialToBackendFrameTranslation, kRandomVector.to_str()},
      {params::kBuildPolicy, BuildPolicy::FromTypeToStr(kBuildPolicy.type)},
      {params::kSimplificationPolicy, RoadGeometryConfiguration::FromSimplificationPolicyToStr(kSimplificationPolicy)},
      {params::kStandardStrictnessPolicy,
       RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(kStandardStrictnessPolicy)},
      {params::kOmitNonDrivableLanes, (kOmitNondrivableLanes ? "true" : "false")},
      {params::kRuleRegistry, kRuleRegistry.value()},
      {params::kRoadRuleBook, kRoadRuleBook.value()},
      {params::kTrafficLightBook, kTrafficLightBook.value()},
      {params::kPhaseRingBook, kPhaseRingBook.value()},
      {params::kIntersectionBook, kIntersectionBook.value()},
  };

  const RoadNetworkConfiguration dut2{RoadNetworkConfiguration::FromMap(rn_config_map)};

  ExpectEqual(dut1, dut2);
}

TEST_F(RoadNetworkConfigurationTest, ToStringMap) {
  const RoadNetworkConfiguration dut1{
      {maliput::api::RoadGeometryId{kRgId}, kOpendriveFile,
       builder::RoadGeometryConfiguration::BuildTolerance{kLinearTolerance, kMaxLinearTolerance, kAngularTolerance},
       kScaleLength, kRandomVector, kBuildPolicy, kSimplificationPolicy, kStandardStrictnessPolicy,
       kOmitNondrivableLanes},
      kRuleRegistry,
      kRoadRuleBook,
      kTrafficLightBook,
      kPhaseRingBook,
      kIntersectionBook};

  const RoadNetworkConfiguration dut2{RoadNetworkConfiguration::FromMap(dut1.ToStringMap())};
  ExpectEqual(dut1, dut2);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
