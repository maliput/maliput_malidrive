// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/builder/road_network_configuration.h"

#include <optional>
#include <string>

#include <gtest/gtest.h>

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
  const RoadGeometryConfiguration::ToleranceSelectionPolicy kToleranceSelectionPolicy{
      RoadGeometryConfiguration::ToleranceSelectionPolicy::kAutomaticSelection};
  const RoadGeometryConfiguration::StandardStrictnessPolicy kStandardStrictnessPolicy{
      RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive};
  const bool kOmitNondrivableLanes{false};
  const std::string kRgId{"test_id"};
  const std::string kOpendriveFile{"opendrive_file_test.xodr"};
  const std::optional<std::string> kRoadRuleBook{"road_rule_book_test.xodr"};
  const std::optional<std::string> kTrafficLightBook{"traffic_light_book_test.xodr"};
  const std::optional<std::string> kPhaseRingBook{"phase_ring_book_test.xodr"};
  const std::optional<std::string> kIntersectionBook{"intersection_book_test.xodr"};
  const double kLinearTolerance{5e-5};
  const double kAngularTolerance{5e-5};
  const double kScaleLength{2.};

  void ExpectEqual(const RoadNetworkConfiguration& lhs, const RoadNetworkConfiguration& rhs) {
    // RoadNetworkConfiguration parameters.
    EXPECT_EQ(lhs.road_rule_book, rhs.road_rule_book);
    EXPECT_EQ(lhs.traffic_light_book, rhs.traffic_light_book);
    EXPECT_EQ(lhs.phase_ring_book, rhs.phase_ring_book);
    EXPECT_EQ(lhs.intersection_book, rhs.intersection_book);
    // RoadGeometryConfiguration parameteres.
    EXPECT_EQ(lhs.road_geometry_configuration.id, rhs.road_geometry_configuration.id);
    EXPECT_EQ(lhs.road_geometry_configuration.opendrive_file, rhs.road_geometry_configuration.opendrive_file);
    EXPECT_EQ(lhs.road_geometry_configuration.linear_tolerance, rhs.road_geometry_configuration.linear_tolerance);
    EXPECT_EQ(lhs.road_geometry_configuration.angular_tolerance, rhs.road_geometry_configuration.angular_tolerance);
    EXPECT_EQ(lhs.road_geometry_configuration.scale_length, rhs.road_geometry_configuration.scale_length);
    EXPECT_EQ(lhs.road_geometry_configuration.inertial_to_backend_frame_translation,
              rhs.road_geometry_configuration.inertial_to_backend_frame_translation);
    EXPECT_EQ(lhs.road_geometry_configuration.build_policy.type, rhs.road_geometry_configuration.build_policy.type);
    EXPECT_EQ(lhs.road_geometry_configuration.build_policy.num_threads,
              rhs.road_geometry_configuration.build_policy.num_threads);
    EXPECT_EQ(lhs.road_geometry_configuration.simplification_policy,
              rhs.road_geometry_configuration.simplification_policy);
    EXPECT_EQ(lhs.road_geometry_configuration.tolerance_selection_policy,
              rhs.road_geometry_configuration.tolerance_selection_policy);
    EXPECT_EQ(lhs.road_geometry_configuration.standard_strictness_policy,
              rhs.road_geometry_configuration.standard_strictness_policy);
    EXPECT_EQ(lhs.road_geometry_configuration.omit_nondrivable_lanes,
              rhs.road_geometry_configuration.omit_nondrivable_lanes);
  }
};

TEST_F(RoadNetworkConfigurationTest, Constructor) {
  const RoadGeometryConfiguration rg_config{maliput::api::RoadGeometryId{kRgId},
                                            kOpendriveFile,
                                            kLinearTolerance,
                                            kAngularTolerance,
                                            kScaleLength,
                                            kRandomVector,
                                            kBuildPolicy,
                                            kSimplificationPolicy,
                                            kToleranceSelectionPolicy,
                                            kStandardStrictnessPolicy,
                                            kOmitNondrivableLanes};
  RoadNetworkConfiguration dut1{rg_config, kRoadRuleBook, kTrafficLightBook, kPhaseRingBook, kIntersectionBook};

  const std::map<std::string, std::string> rn_config_map{
      {RoadGeometryConfiguration::kStrRoadGeometryId, kRgId},
      {RoadGeometryConfiguration::kStrOpendriveFile, kOpendriveFile},
      {RoadGeometryConfiguration::kStrLinearTolerance, std::to_string(kLinearTolerance)},
      {RoadGeometryConfiguration::kStrAngularTolerance, std::to_string(kAngularTolerance)},
      {RoadGeometryConfiguration::kStrScaleLength, std::to_string(kScaleLength)},
      {RoadGeometryConfiguration::kStrInertialToBackendFrameTranslation, kRandomVector.to_str()},
      {RoadGeometryConfiguration::kStrBuildPolicy, BuildPolicy::FromTypeToStr(kBuildPolicy.type)},
      {RoadGeometryConfiguration::kStrSimplificationPolicy,
       RoadGeometryConfiguration::FromSimplificationPolicyToStr(kSimplificationPolicy)},
      {RoadGeometryConfiguration::kStrToleranceSelectionPolicy,
       RoadGeometryConfiguration::FromToleranceSelectionPolicyToStr(kToleranceSelectionPolicy)},
      {RoadGeometryConfiguration::kStrStandardStrictnessPolicy,
       RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(kStandardStrictnessPolicy)},
      {RoadGeometryConfiguration::kStrOmitNonDrivableLanes, (kOmitNondrivableLanes ? "true" : "false")},
      {RoadNetworkConfiguration::kStrRoadRuleBook, kRoadRuleBook.value()},
      {RoadNetworkConfiguration::kStrTrafficLightBook, kTrafficLightBook.value()},
      {RoadNetworkConfiguration::kStrPhaseRingBook, kPhaseRingBook.value()},
      {RoadNetworkConfiguration::kStrIntersectionBook, kIntersectionBook.value()},
  };

  const RoadNetworkConfiguration dut2{RoadNetworkConfiguration::FromMap(rn_config_map)};

  ExpectEqual(dut1, dut2);
}

TEST_F(RoadNetworkConfigurationTest, ToStringMap) {
  const RoadNetworkConfiguration dut1{
      {maliput::api::RoadGeometryId{kRgId}, kOpendriveFile, kLinearTolerance, kAngularTolerance, kScaleLength,
       kRandomVector, kBuildPolicy, kSimplificationPolicy, kToleranceSelectionPolicy, kStandardStrictnessPolicy,
       kOmitNondrivableLanes},
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
