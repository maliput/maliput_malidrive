// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/builder/rule_registry_builder.h"

#include <memory>
#include <string>
#include <variant>

#include <gtest/gtest.h>
#include <maliput/api/rules/rule_registry.h>
#include <maliput/base/rule_registry.h>

#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_geometry_builder.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

// Obtains all the keys from `map`.
template <typename T, typename Y>
std::vector<T> GetAllKeysFromMap(std::map<T, Y> map) {
  std::vector<T> keys{};
  for (const auto& key_value : map) {
    keys.push_back(key_value.first);
  }
  return keys;
}

// Tests RuleRegistryBuilder by verifying all the states of the rule types that are
// expected to be registered given the:
// 1 - Rule types that are added internally by the RuleRegistryBuilder.
// 2 - The rule types described at the YAML file.
//
// The Xodr and RuleRegistry used for evaluation is `figure8_trafficlights`.
class RuleRegistryBuilderTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto manager =
        xodr::LoadDataBaseFromFile(road_geometry_configuration_.opendrive_file, {constants::kLinearTolerance});
    road_geometry_ = builder::RoadGeometryBuilder(std::move(manager), road_geometry_configuration_)();
  }

  // Verifies if `discrete_value` is part of `discrete_values`.
  void TestRuleTypeDiscreteValue(const maliput::api::rules::RuleRegistry::QueryResult::DiscreteValues& discrete_values,
                                 const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
    const auto discrete_value_itr = std::find_if(
        discrete_values.begin(), discrete_values.end(),
        [&discrete_value](const maliput::api::rules::DiscreteValueRule::DiscreteValue& value) {
          return value.value == discrete_value.value && value.severity == discrete_value.severity &&
                 GetAllKeysFromMap(value.related_rules) == GetAllKeysFromMap(discrete_value.related_rules) &&
                 GetAllKeysFromMap(value.related_unique_ids) == GetAllKeysFromMap(discrete_value.related_unique_ids);
        });
    ASSERT_NE(discrete_values.end(), discrete_value_itr);
  }

  // Verifies if `value_value` is a value of a DiscreteValue of a vector of `discrete_values`.
  // Assumptions:
  // - Severity should be strict.
  // - RelatedRules should be empty.
  // - RelatedUniqueIds should be empty.
  void TestRuleTypeDiscreteValue(const maliput::api::rules::RuleRegistry::QueryResult::DiscreteValues& discrete_values,
                                 const std::string& value_value) {
    const auto discrete_value_itr = std::find_if(
        discrete_values.begin(), discrete_values.end(),
        [&value_value](const maliput::api::rules::DiscreteValueRule::DiscreteValue& value) {
          return value.value == value_value && value.severity == maliput::api::rules::Rule::State::kStrict &&
                 value.related_rules.empty() && value.related_unique_ids.empty();
        });
    ASSERT_NE(discrete_values.end(), discrete_value_itr);
  }

  // Verifies if `min` and `max` are limits of a Range of a vector of `Ranges`.
  // Assumptions:
  // - Severity should be strict.
  // - RelatedRules should be empty.
  // - RelatedUniqueIds should be empty.
  void TestRuleTypeRangeValue(const maliput::api::rules::RuleRegistry::QueryResult::Ranges& ranges, double min,
                              double max) {
    const auto discrete_value_itr = std::find_if(
        ranges.begin(), ranges.end(), [&min, &max](const maliput::api::rules::RangeValueRule::Range& value) {
          return value.min == min && value.max == max && value.severity == maliput::api::rules::Rule::State::kStrict &&
                 value.related_rules.empty() && value.related_unique_ids.empty();
        });
    ASSERT_NE(ranges.end(), discrete_value_itr);
  }

  // Verifies possible states of Vehicle-Exclusive Rule Type which is registered by the RuleRegistryBuilder.
  void TestProgramaticallyAddedVehicleExclusiveRuleType() {
    const std::optional<maliput::api::rules::RuleRegistry::QueryResult> vehicle_exclusive_states_opt =
        rule_registry_->GetPossibleStatesOfRuleType(malidrive::builder::rules::VehicleExclusiveRuleTypeId());
    ASSERT_NE(vehicle_exclusive_states_opt, std::nullopt);
    EXPECT_EQ(malidrive::builder::rules::VehicleExclusiveRuleTypeId(), vehicle_exclusive_states_opt.value().type_id);
    const auto vehicle_exclusive_states = std::get<maliput::api::rules::RuleRegistry::QueryResult::DiscreteValues>(
        vehicle_exclusive_states_opt.value().rule_values);
    EXPECT_EQ(5, vehicle_exclusive_states.size());
    TestRuleTypeDiscreteValue(vehicle_exclusive_states, "BusOnly");
    TestRuleTypeDiscreteValue(vehicle_exclusive_states, "EmergencyVehiclesOnly");
    TestRuleTypeDiscreteValue(vehicle_exclusive_states, "HighOccupancyVehicleOnly");
    TestRuleTypeDiscreteValue(vehicle_exclusive_states, "MotorizedVehicleOnly");
    TestRuleTypeDiscreteValue(vehicle_exclusive_states, "NonMotorizedVehicleOnly");
  }

  // Verifies possible states of Vehicle-Usage Rule Type which is registered by the RuleRegistryBuilder.
  void TestProgramaticallyAddedVehicleUsageRuleType() {
    const std::optional<maliput::api::rules::RuleRegistry::QueryResult> vehicle_usage_states_opt =
        rule_registry_->GetPossibleStatesOfRuleType(malidrive::builder::rules::VehicleUsageRuleTypeId());
    ASSERT_NE(vehicle_usage_states_opt, std::nullopt);
    EXPECT_EQ(malidrive::builder::rules::VehicleUsageRuleTypeId(), vehicle_usage_states_opt.value().type_id);
    const auto vehicle_usage_states = std::get<maliput::api::rules::RuleRegistry::QueryResult::DiscreteValues>(
        vehicle_usage_states_opt.value().rule_values);
    EXPECT_EQ(3, vehicle_usage_states.size());
    TestRuleTypeDiscreteValue(vehicle_usage_states, "NonVehicles");
    TestRuleTypeDiscreteValue(vehicle_usage_states, "NonPedestrians");
    TestRuleTypeDiscreteValue(vehicle_usage_states, "Unrestricted");
  }

  // Verifies possible states of Direction-Usage Rule Type which is registered by the RuleRegistryBuilder.
  void TestProgramaticallyAddedDirectionUsageRuleType() {
    const std::optional<maliput::api::rules::RuleRegistry::QueryResult> direction_usage_states_opt =
        rule_registry_->GetPossibleStatesOfRuleType(maliput::DirectionUsageRuleTypeId());
    ASSERT_NE(direction_usage_states_opt, std::nullopt);
    EXPECT_EQ(maliput::DirectionUsageRuleTypeId(), direction_usage_states_opt.value().type_id);
    const auto direction_usage_states = std::get<maliput::api::rules::RuleRegistry::QueryResult::DiscreteValues>(
        direction_usage_states_opt.value().rule_values);
    EXPECT_EQ(7, direction_usage_states.size());
    TestRuleTypeDiscreteValue(direction_usage_states, "WithS");
    TestRuleTypeDiscreteValue(direction_usage_states, "AgainstS");
    TestRuleTypeDiscreteValue(direction_usage_states, "Bidirectional");
    TestRuleTypeDiscreteValue(direction_usage_states, "BidirectionalTurnOnly");
    TestRuleTypeDiscreteValue(direction_usage_states, "NoUse");
    TestRuleTypeDiscreteValue(direction_usage_states, "Parking");
    TestRuleTypeDiscreteValue(direction_usage_states, "Undefined");
  }

  // Verifies possible states of Speed-Limit Rule Type which is registered by the RuleRegistryBuilder.
  void TestProgramaticallyAddedSpeedLimitRuleType() {
    const std::optional<maliput::api::rules::RuleRegistry::QueryResult> speed_limit_states_opt =
        rule_registry_->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
    ASSERT_NE(speed_limit_states_opt, std::nullopt);
    EXPECT_EQ(maliput::SpeedLimitRuleTypeId(), speed_limit_states_opt.value().type_id);
    const auto speed_limit_states =
        std::get<maliput::api::rules::RuleRegistry::QueryResult::Ranges>(speed_limit_states_opt.value().rule_values);
    ASSERT_EQ(2, speed_limit_states.size());
    TestRuleTypeRangeValue(speed_limit_states, constants::kDefaultMinSpeedLimit, constants::kDefaultMaxSpeedLimit);
    TestRuleTypeRangeValue(speed_limit_states, constants::kDefaultMinSpeedLimit, 17.8816 /* 40 mph */);
  }

  // Verifies possible states of Right-Of-Way Rule Type which is registered via YAML loader.
  void TestViaYamlAddedRightOfWayRuleType() {
    const std::optional<maliput::api::rules::RuleRegistry::QueryResult> right_of_way_states_opt =
        rule_registry_->GetPossibleStatesOfRuleType(maliput::RightOfWayRuleTypeId());
    ASSERT_NE(right_of_way_states_opt, std::nullopt);
    EXPECT_EQ(maliput::RightOfWayRuleTypeId(), right_of_way_states_opt.value().type_id);
    const auto right_of_way_states = std::get<maliput::api::rules::RuleRegistry::QueryResult::DiscreteValues>(
        right_of_way_states_opt.value().rule_values);
    EXPECT_EQ(3, right_of_way_states.size());

    TestRuleTypeDiscreteValue(right_of_way_states, maliput::api::rules::DiscreteValueRule::DiscreteValue{
                                                       maliput::api::rules::Rule::State::kStrict,
                                                       {{maliput::RelatedRulesKeys::kYieldGroup, {}},
                                                        {maliput::VehicleStopInZoneBehaviorRuleTypeId().string(), {}}},
                                                       {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {}}},
                                                       "Go"});
    TestRuleTypeDiscreteValue(right_of_way_states, maliput::api::rules::DiscreteValueRule::DiscreteValue{
                                                       maliput::api::rules::Rule::State::kStrict,
                                                       {{maliput::RelatedRulesKeys::kYieldGroup, {}},
                                                        {maliput::VehicleStopInZoneBehaviorRuleTypeId().string(), {}}},
                                                       {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {}}},
                                                       "Stop"});
    TestRuleTypeDiscreteValue(right_of_way_states, maliput::api::rules::DiscreteValueRule::DiscreteValue{
                                                       maliput::api::rules::Rule::State::kStrict,
                                                       {{maliput::RelatedRulesKeys::kYieldGroup, {}},
                                                        {maliput::VehicleStopInZoneBehaviorRuleTypeId().string(), {}}},
                                                       {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {}}},
                                                       "StopThenGo"});
  }

  // Verifies possible states of Vehicle-Stop-In-Zone-Behavior Rule Type which is registered via YAML loader.
  void TestViaYamlAddedVehicleStopInZoneBehaviorRuleType() {
    const maliput::api::rules::Rule::TypeId rule_type(maliput::VehicleStopInZoneBehaviorRuleTypeId());
    const std::optional<maliput::api::rules::RuleRegistry::QueryResult> vehicle_in_stop_behavior_states_opt =
        rule_registry_->GetPossibleStatesOfRuleType(rule_type);
    ASSERT_NE(vehicle_in_stop_behavior_states_opt, std::nullopt);
    EXPECT_EQ(rule_type, vehicle_in_stop_behavior_states_opt.value().type_id);
    const auto vehicle_in_stop_behavior_states =
        std::get<maliput::api::rules::RuleRegistry::QueryResult::DiscreteValues>(
            vehicle_in_stop_behavior_states_opt.value().rule_values);
    EXPECT_EQ(2, vehicle_in_stop_behavior_states.size());

    TestRuleTypeDiscreteValue(vehicle_in_stop_behavior_states, maliput::api::rules::DiscreteValueRule::DiscreteValue{
                                                                   maliput::api::rules::Rule::State::kStrict,
                                                                   {{maliput::RelatedRulesKeys::kYieldGroup, {}}},
                                                                   {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {}}},
                                                                   "DoNotStop"});
    TestRuleTypeDiscreteValue(vehicle_in_stop_behavior_states, maliput::api::rules::DiscreteValueRule::DiscreteValue{
                                                                   maliput::api::rules::Rule::State::kStrict,
                                                                   {{maliput::RelatedRulesKeys::kYieldGroup, {}}},
                                                                   {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {}}},
                                                                   "UnconstrainedParking"});
  }

  // Verifies possible states of Maximum-Weight Rule Type which is registered via YAML loader.
  void TestViaYamlAddedMaximumWeightRuleType() {
    const maliput::api::rules::Rule::TypeId rule_type("Maximum-Weight Rule Type");
    const std::optional<maliput::api::rules::RuleRegistry::QueryResult> maximum_height_states_opt =
        rule_registry_->GetPossibleStatesOfRuleType(rule_type);
    ASSERT_NE(maximum_height_states_opt, std::nullopt);
    EXPECT_EQ(rule_type, maximum_height_states_opt.value().type_id);
    const auto maximum_height_states =
        std::get<maliput::api::rules::RuleRegistry::QueryResult::Ranges>(maximum_height_states_opt.value().rule_values);
    ASSERT_EQ(1, maximum_height_states.size());
    TestRuleTypeRangeValue(maximum_height_states, 0., 15000.);
  }

 protected:
  const std::string map_id{"figure8_trafficlights/figure8_trafficlights"};
  const std::string xodr_file_path{utility::FindResourceInPath(map_id + ".xodr", kMalidriveResourceFolder)};
  const std::string rule_registry_path{
      utility::FindResourceInPath(map_id + "_new_rules.yaml", kMalidriveResourceFolder)};
  const RoadGeometryConfiguration road_geometry_configuration_{RoadGeometryConfiguration::FromMap({
      {"opendrive_file", xodr_file_path},
  })};

  std::unique_ptr<maliput::api::rules::RuleRegistry> rule_registry_;
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry_;
};

// No yaml file is provided for loading the RuleRegistry.
TEST_F(RuleRegistryBuilderTest, NoRuleRegistryFile) {
  rule_registry_ = RuleRegistryBuilder(road_geometry_.get(), std::nullopt)();
  TestProgramaticallyAddedVehicleExclusiveRuleType();
  TestProgramaticallyAddedVehicleUsageRuleType();
  TestProgramaticallyAddedDirectionUsageRuleType();
  TestProgramaticallyAddedSpeedLimitRuleType();
}

// yaml file is provided for loading the RuleRegistry.
TEST_F(RuleRegistryBuilderTest, WithRuleRegistryFile) {
  rule_registry_ = RuleRegistryBuilder(road_geometry_.get(), rule_registry_path)();
  // Test rule types added programatically by the RuleRegistryBuilder.
  TestProgramaticallyAddedVehicleExclusiveRuleType();
  TestProgramaticallyAddedVehicleUsageRuleType();
  TestProgramaticallyAddedDirectionUsageRuleType();
  TestProgramaticallyAddedSpeedLimitRuleType();

  // Test rule types loaded via YAML file.
  TestViaYamlAddedRightOfWayRuleType();
  TestViaYamlAddedVehicleStopInZoneBehaviorRuleType();
  TestViaYamlAddedMaximumWeightRuleType();
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
