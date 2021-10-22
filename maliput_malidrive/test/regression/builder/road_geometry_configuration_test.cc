// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_configuration.h"

#include <gtest/gtest.h>

#include "maliput_malidrive/builder/params.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

class RoadGeometryConfigurationTest : public ::testing::Test {
 protected:
  const maliput::math::Vector3 kRandomVector{1., 2., 3.};
  const BuildPolicy kBuildPolicy{BuildPolicy::Type::kParallel, 4};
  const RoadGeometryConfiguration::SimplificationPolicy kSimplificationPolicy{
      RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel};
  const RoadGeometryConfiguration::StandardStrictnessPolicy kStandardStrictnessPolicy{
      RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive};
  const bool kOmitNondrivableLanes{false};
  const std::string kRgId{"test_id"};
  const std::string kOpendriveFile{"test.xodr"};
  const double kLinearTolerance{5e-5};
  const double kMaxLinearTolerance{1e-4};
  const double kAngularTolerance{5e-5};
  const double kScaleLength{2.};

  void ExpectEqual(const RoadGeometryConfiguration& lhs, const RoadGeometryConfiguration& rhs) {
    EXPECT_EQ(lhs.id, rhs.id);
    EXPECT_EQ(lhs.opendrive_file, rhs.opendrive_file);
    EXPECT_EQ(lhs.tolerances.linear_tolerance, rhs.tolerances.linear_tolerance);
    EXPECT_EQ(lhs.tolerances.max_linear_tolerance, rhs.tolerances.max_linear_tolerance);
    EXPECT_EQ(lhs.tolerances.angular_tolerance, rhs.tolerances.angular_tolerance);
    EXPECT_EQ(lhs.scale_length, rhs.scale_length);
    EXPECT_EQ(lhs.inertial_to_backend_frame_translation, rhs.inertial_to_backend_frame_translation);
    EXPECT_EQ(lhs.build_policy.type, rhs.build_policy.type);
    EXPECT_EQ(lhs.build_policy.num_threads, rhs.build_policy.num_threads);
    EXPECT_EQ(lhs.simplification_policy, rhs.simplification_policy);
    EXPECT_EQ(lhs.standard_strictness_policy, rhs.standard_strictness_policy);
    EXPECT_EQ(lhs.omit_nondrivable_lanes, rhs.omit_nondrivable_lanes);
  }
};

TEST_F(RoadGeometryConfigurationTest, Constructor) {
  const RoadGeometryConfiguration dut1{
      maliput::api::RoadGeometryId{kRgId},
      kOpendriveFile,
      builder::RoadGeometryConfiguration::BuildTolerance{kLinearTolerance, kMaxLinearTolerance, kAngularTolerance},
      kScaleLength,
      kRandomVector,
      kBuildPolicy,
      kSimplificationPolicy,
      kStandardStrictnessPolicy,
      kOmitNondrivableLanes};

  const std::map<std::string, std::string> rg_config_map{
      {params::kRoadGeometryId, kRgId},
      {params::kOpendriveFile, kOpendriveFile},
      {params::kLinearTolerance, std::to_string(kLinearTolerance)},
      {params::kMaxLinearTolerance, std::to_string(kMaxLinearTolerance)},
      {params::kAngularTolerance, std::to_string(kAngularTolerance)},
      {params::kScaleLength, std::to_string(kScaleLength)},
      {params::kInertialToBackendFrameTranslation, kRandomVector.to_str()},
      {params::kBuildPolicy, BuildPolicy::FromTypeToStr(kBuildPolicy.type)},
      {params::kNumThreads, std::to_string(kBuildPolicy.num_threads.value())},
      {params::kSimplificationPolicy, RoadGeometryConfiguration::FromSimplificationPolicyToStr(kSimplificationPolicy)},
      {params::kStandardStrictnessPolicy,
       RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(kStandardStrictnessPolicy)},
      {params::kOmitNonDrivableLanes, (kOmitNondrivableLanes ? "true" : "false")},
  };

  const RoadGeometryConfiguration dut2{RoadGeometryConfiguration::FromMap(rg_config_map)};

  ExpectEqual(dut1, dut2);
}

TEST_F(RoadGeometryConfigurationTest, ToStringMapUsingMaxLinearTolerance) {
  const RoadGeometryConfiguration dut1{
      maliput::api::RoadGeometryId{kRgId},
      kOpendriveFile,
      builder::RoadGeometryConfiguration::BuildTolerance{kLinearTolerance, kMaxLinearTolerance, kAngularTolerance},
      kScaleLength,
      kRandomVector,
      kBuildPolicy,
      kSimplificationPolicy,
      kStandardStrictnessPolicy,
      kOmitNondrivableLanes};

  const RoadGeometryConfiguration dut2{RoadGeometryConfiguration::FromMap(dut1.ToStringMap())};
  ExpectEqual(dut1, dut2);
}

TEST_F(RoadGeometryConfigurationTest, ToStringMapNotUsingMaxLinearTolerance) {
  const RoadGeometryConfiguration dut1{
      maliput::api::RoadGeometryId{kRgId},
      kOpendriveFile,
      builder::RoadGeometryConfiguration::BuildTolerance{kLinearTolerance, kAngularTolerance},
      kScaleLength,
      kRandomVector,
      kBuildPolicy,
      kSimplificationPolicy,
      kStandardStrictnessPolicy,
      kOmitNondrivableLanes};

  const RoadGeometryConfiguration dut2{RoadGeometryConfiguration::FromMap(dut1.ToStringMap())};
  ExpectEqual(dut1, dut2);
}

GTEST_TEST(BuildPolicyType, StringToTypeConversion) {
  EXPECT_EQ(BuildPolicy::Type::kSequential, BuildPolicy::FromStrToType("sequential"));
  EXPECT_EQ(BuildPolicy::Type::kParallel, BuildPolicy::FromStrToType("parallel"));
}

GTEST_TEST(SimplificationPolicy, StringToPolicyConversion) {
  EXPECT_EQ(RoadGeometryConfiguration::SimplificationPolicy::kNone,
            RoadGeometryConfiguration::FromStrToSimplificationPolicy("none"));
  EXPECT_EQ(RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel,
            RoadGeometryConfiguration::FromStrToSimplificationPolicy("simplify"));
}

GTEST_TEST(StandardStrictnessPolicy, StringToPolicyConversion) {
  EXPECT_EQ(RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict,
            RoadGeometryConfiguration::FromStrToStandardStrictnessPolicy("strict"));
  EXPECT_EQ(RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors,
            RoadGeometryConfiguration::FromStrToStandardStrictnessPolicy("allow_schema_errors"));
  EXPECT_EQ(RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors,
            RoadGeometryConfiguration::FromStrToStandardStrictnessPolicy("allow_semantic_errors"));
  EXPECT_EQ(RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive,
            RoadGeometryConfiguration::FromStrToStandardStrictnessPolicy("allow_schema_errors|allow_semantic_errors"));
  EXPECT_EQ(RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive,
            RoadGeometryConfiguration::FromStrToStandardStrictnessPolicy("permissive"));
}

GTEST_TEST(StandardStrictnessPolicy, PolicyToStringConversion) {
  EXPECT_EQ("strict", RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(
                          RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict));
  EXPECT_EQ("allow_schema_errors", RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(
                                       RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSchemaErrors));
  EXPECT_EQ("allow_semantic_errors", RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(
                                         RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors));
  EXPECT_EQ("permissive", RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(
                              RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive));
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
