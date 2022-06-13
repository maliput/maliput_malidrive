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
#include "maliput_malidrive/builder/road_rulebook_builder.h"

#include <memory>
#include <optional>
#include <string>
#include <type_traits>

#include <gtest/gtest.h>
#include <maliput/api/rules/discrete_value_rule.h>
#include <maliput/api/rules/range_value_rule.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/api/rules/rule_registry.h>
#include <maliput/api/unique_id.h>
#include <maliput/base/rule_registry.h>

#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_geometry_builder.h"
#include "maliput_malidrive/builder/rule_registry_builder.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

// Verifies if `rule_a` and `rule_b` are equal based on a `tolerance`. The `tolerance` value applyies when the region
// of the rule is being evaluated.
//  `T` param: DiscreteValueRules or RangeValueRules.
// TODO(https://github.com/ToyotaResearchInstitute/maliput/issues/463): Use "maliput/test_utilities/rules_compare.h"
// instead when zone(ranges) comparison includes tolerance.
template <typename T>
inline ::testing::AssertionResult IsEqual(const T& rule_a, const T& rule_b, double tolerance) {
  if (rule_a.id() != rule_b.id()) {
    return ::testing::AssertionFailure() << "Rules Id are different: " << rule_a.id().string() << " vs "
                                         << rule_b.id().string();
  }
  if (rule_a.type_id() != rule_b.type_id()) {
    return ::testing::AssertionFailure() << "Rule Type Id are different: " << rule_a.type_id().string() << " vs "
                                         << rule_b.type_id().string();
  }

  const auto lane_s_route_a = rule_a.zone();
  const auto lane_s_route_b = rule_b.zone();
  for (const auto& lane_s_range_a : lane_s_route_a.ranges()) {
    auto lane_s_range_b = std::find_if(lane_s_route_b.ranges().begin(), lane_s_route_b.ranges().end(),
                                       [&lane_s_range_a](const LaneSRange& lane_s_range_b) {
                                         return lane_s_range_b.lane_id() == lane_s_range_a.lane_id();
                                       });
    if (lane_s_range_b == lane_s_route_b.ranges().end()) {
      return ::testing::AssertionFailure()
             << "Zone ids are different: " << lane_s_range_a.lane_id().string() << " lane id from rule '"
             << rule_a.id().string() << "' doesn't belong to the zone of rule '" << rule_b.id().string() << "'";
    }
    if (std::abs(lane_s_range_a.s_range().s0() - lane_s_range_b->s_range().s0()) >= tolerance) {
      return ::testing::AssertionFailure()
             << "Zone ranges are different for rule " << rule_a.id().string() << "in lane id "
             << lane_s_range_a.lane_id().string() << ": s0: " << lane_s_range_a.s_range().s0() << " vs "
             << lane_s_range_b->s_range().s0();
    }
    if (std::abs(lane_s_range_a.s_range().s1() - lane_s_range_b->s_range().s1()) >= tolerance) {
      return ::testing::AssertionFailure()
             << "Zone ranges are different for rule " << rule_a.id().string() << "in lane id "
             << lane_s_range_a.lane_id().string() << ": s1: " << lane_s_range_a.s_range().s1() << " vs "
             << lane_s_range_b->s_range().s1();
    }
  }
  if constexpr (std::is_base_of<DiscreteValueRule, T>::value) {
    if (rule_a.states() != rule_b.states()) {
      return ::testing::AssertionFailure() << "DiscreteValues of " << rule_a.id().string() << " are different";
    }
  } else {
    if (rule_a.states() != rule_b.states()) {
      return ::testing::AssertionFailure() << "Ranges of " << rule_a.id().string() << " are different";
    }
  }
  return ::testing::AssertionSuccess();
}

// Tests RoadRulebookBuilder by verifying several rules that correspond to different rule types.
// The rules in the RoadRulebook are added by two different ways.
// 1 - Rules are added internally from rules created out of the XODR description:
//     - Speed Limit rules.
//     - Direction Usage rules.
//     - Vehicle Usage rules.
//     - Vehicle Exclusive rules.
// 2 - Rules are loaded via yaml file.
//
// Rules created via both methods should be evaluated.
//
// The Xodr, RuleRegistry, and RoadRulebook used for evaluation is `figure8_trafficlights`.
class RoadRulebookBuilderTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto manager =
        xodr::LoadDataBaseFromFile(road_geometry_configuration_.opendrive_file, {constants::kLinearTolerance});
    road_geometry_ = RoadGeometryBuilder(std::move(manager), road_geometry_configuration_)();
    rule_registry_ = RuleRegistryBuilder(road_geometry_.get(), rule_registry_path)();
  }

 protected:
  const std::string map_id{"figure8_trafficlights/figure8_trafficlights"};
  const std::string xodr_file_path{utility::FindResourceInPath(map_id + ".xodr", kMalidriveResourceFolder)};
  const std::string rule_registry_path{
      utility::FindResourceInPath(map_id + "_new_rules.yaml", kMalidriveResourceFolder)};
  const std::string road_rulebook_path{
      utility::FindResourceInPath(map_id + "_new_rules.yaml", kMalidriveResourceFolder)};
  const RoadGeometryConfiguration road_geometry_configuration_{RoadGeometryConfiguration::FromMap({
      {"opendrive_file", xodr_file_path},
      {"omit_nondrivable_lanes", "false"},
  })};

  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry_;
  std::unique_ptr<const maliput::api::rules::RoadRulebook> road_rulebook_;
  std::unique_ptr<maliput::api::rules::RuleRegistry> rule_registry_;
};

TEST_F(RoadRulebookBuilderTest, Constructor) {
  // Throws because RoadGeometry pointer is null.
  EXPECT_THROW(RoadRuleBookBuilder(nullptr, rule_registry_.get(), road_rulebook_path)(),
               maliput::common::assertion_error);
  // Throws because RuleRegistry pointer is null.
  EXPECT_THROW(RoadRuleBookBuilder(road_geometry_.get(), nullptr, road_rulebook_path)(),
               maliput::common::assertion_error);
  // Correct construction with RoadRulebook's yaml path is empty.
  EXPECT_NO_THROW(RoadRuleBookBuilder(road_geometry_.get(), rule_registry_.get(), std::nullopt)());
  // Correct contruction.
  EXPECT_NO_THROW(RoadRuleBookBuilder(road_geometry_.get(), rule_registry_.get(), road_rulebook_path)());
}

// Evaluate some rules that are loaded out of the information provided by the XODR file.
class RoadRulebookBuilderRulesFromXodrTest : public RoadRulebookBuilderTest {
 public:
  const std::vector<maliput::api::rules::DiscreteValueRule> expected_discrete_value_rules{
      {Rule::Id{maliput::DirectionUsageRuleTypeId().string() + "/32_1_1"},
       maliput::DirectionUsageRuleTypeId(),
       LaneSRoute{{{maliput::api::LaneId("32_1_1"), {0, 2.10}}}},
       {{Rule::State::kStrict, {}, {}, "AgainstS"}}},
      {Rule::Id{maliput::DirectionUsageRuleTypeId().string() + "/60_1_-1"},
       maliput::DirectionUsageRuleTypeId(),
       LaneSRoute{{{maliput::api::LaneId("60_1_-1"), {0, 2.09}}}},
       {{Rule::State::kStrict, {}, {}, "WithS"}}},
      {Rule::Id{maliput::DirectionUsageRuleTypeId().string() + "/57_0_2"},
       maliput::DirectionUsageRuleTypeId(),
       LaneSRoute{{{maliput::api::LaneId("57_0_2"), {0, 0.1799}}}},
       {{Rule::State::kStrict, {}, {}, "Bidirectional"}}},
      {Rule::Id{malidrive::builder::rules::VehicleUsageRuleTypeId().string() + "/32_1_1"},
       malidrive::builder::rules::VehicleUsageRuleTypeId(),
       LaneSRoute{{{maliput::api::LaneId("32_1_1"), {0, 2.10}}}},
       {{Rule::State::kStrict, {}, {}, "NonPedestrians"}}},
      {Rule::Id{malidrive::builder::rules::VehicleUsageRuleTypeId().string() + "/57_0_2"},
       malidrive::builder::rules::VehicleUsageRuleTypeId(),
       LaneSRoute{{{maliput::api::LaneId("57_0_2"), {0, 0.1799}}}},
       {{Rule::State::kStrict, {}, {}, "NonVehicles"}}},
  };
  const std::vector<maliput::api::rules::RangeValueRule> expected_range_value_rules{
      {Rule::Id{maliput::SpeedLimitRuleTypeId().string() + "/0_0_1_1"},
       maliput::SpeedLimitRuleTypeId(),
       LaneSRoute{{{maliput::api::LaneId("0_0_1"), {0, 169.272}}}},
       {{Rule::State::kStrict, {}, {}, {"m/s"}, {0.}, {17.8816}}}},
  };
};

TEST_F(RoadRulebookBuilderRulesFromXodrTest, VerifyRulesObtainedFromXODR) {
  road_rulebook_ = RoadRuleBookBuilder(road_geometry_.get(), rule_registry_.get(), std::nullopt)();
  for (const auto& expected_rule : expected_discrete_value_rules) {
    std::unique_ptr<DiscreteValueRule> rule;
    EXPECT_NO_THROW(rule =
                        std::make_unique<DiscreteValueRule>(road_rulebook_->GetDiscreteValueRule(expected_rule.id())));
    EXPECT_TRUE(IsEqual<DiscreteValueRule>(expected_rule, *rule, malidrive::constants::kLinearTolerance));
  }
  for (const auto& expected_rule : expected_range_value_rules) {
    std::unique_ptr<RangeValueRule> rule;
    road_rulebook_->GetRangeValueRule(expected_rule.id());
    EXPECT_NO_THROW(rule = std::make_unique<RangeValueRule>(road_rulebook_->GetRangeValueRule(expected_rule.id())));
    EXPECT_TRUE(IsEqual<RangeValueRule>(expected_rule, *rule, malidrive::constants::kLinearTolerance));
  }
}

// Evaluate some rules that are directly provided by the YAML file.
class RoadRulebookBuilderRulesFromYamlFileTest : public RoadRulebookBuilderTest {
 public:
  const std::vector<maliput::api::rules::DiscreteValueRule> expected_discrete_value_rules{
      {Rule::Id{maliput::VehicleStopInZoneBehaviorRuleTypeId().string() + "/SouthRightTurn"},
       maliput::VehicleStopInZoneBehaviorRuleTypeId(),
       LaneSRoute{{
           {maliput::api::LaneId("82_0_-1"), {0, 0.17999}},
           {maliput::api::LaneId("82_1_-1"), {0, 2.09}},
           {maliput::api::LaneId("82_2_-1"), {0, 7.9428}},
           {maliput::api::LaneId("82_3_-1"), {0, 2.09}},
           {maliput::api::LaneId("82_4_-1"), {0, 0.17999}},
       }},
       {
           {Rule::State::kStrict, {}, {}, "DoNotStop"},
       }},
      {Rule::Id{maliput::RightOfWayRuleTypeId().string() + "/SouthRightTurn"},
       maliput::RightOfWayRuleTypeId(),
       LaneSRoute{{
           {maliput::api::LaneId("82_0_-1"), {0, 0.17999}},
           {maliput::api::LaneId("82_1_-1"), {0, 2.09}},
           {maliput::api::LaneId("82_2_-1"), {0, 7.9428}},
           {maliput::api::LaneId("82_3_-1"), {0, 2.09}},
           {maliput::api::LaneId("82_4_-1"), {0, 0.17999}},
       }},
       {
           {Rule::State::kStrict,
            {{maliput::RelatedRulesKeys::kYieldGroup,
              {Rule::Id(maliput::RightOfWayRuleTypeId().string() + "/NorthLeftTurn")}},
             {maliput::VehicleStopInZoneBehaviorRuleTypeId().string(),
              {Rule::Id(maliput::VehicleStopInZoneBehaviorRuleTypeId().string() + "/SouthRightTurn")}}},
            {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {maliput::api::UniqueId("NorthFacing-NorthFacingBulbs")}}},
            "Go"},
           {Rule::State::kStrict,
            {{maliput::RelatedRulesKeys::kYieldGroup,
              {Rule::Id(maliput::RightOfWayRuleTypeId().string() + "/NorthLeftTurn"),
               Rule::Id(maliput::RightOfWayRuleTypeId().string() + "/WestStraight")}},
             {maliput::VehicleStopInZoneBehaviorRuleTypeId().string(),
              {Rule::Id(maliput::VehicleStopInZoneBehaviorRuleTypeId().string() + "/SouthRightTurn")}}},
            {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {maliput::api::UniqueId("NorthFacing-NorthFacingBulbs")}}},
            "StopThenGo"},
       }},
  };
  const std::vector<maliput::api::rules::RangeValueRule> expected_range_value_rules{
      {Rule::Id{std::string("Maximum-Weight Rule Type") + std::string("/EntireRoadNetwork")},
       Rule::TypeId("Maximum-Weight Rule Type"),
       LaneSRoute{{
           {maliput::api::LaneId("82_0_-1"), {0, 0.17999}},
           {maliput::api::LaneId("82_1_-1"), {0, 2.09}},
           {maliput::api::LaneId("82_2_-1"), {0, 7.9428}},
           {maliput::api::LaneId("82_3_-1"), {0, 2.09}},
           {maliput::api::LaneId("82_4_-1"), {0, 0.17999}},
       }},
       {
           {Rule::State::kStrict, {}, {}, "Urban Road", {0.}, {15000.}},
       }},
  };
};

TEST_F(RoadRulebookBuilderRulesFromYamlFileTest, RoadRulebookBuilderRulesFromYamlFileTest) {
  road_rulebook_ = RoadRuleBookBuilder(road_geometry_.get(), rule_registry_.get(), road_rulebook_path)();
  for (const auto& expected_rule : expected_discrete_value_rules) {
    std::unique_ptr<DiscreteValueRule> rule;
    EXPECT_NO_THROW(rule =
                        std::make_unique<DiscreteValueRule>(road_rulebook_->GetDiscreteValueRule(expected_rule.id())));
    EXPECT_TRUE(IsEqual<DiscreteValueRule>(expected_rule, *rule, malidrive::constants::kLinearTolerance));
  }
  for (const auto& expected_rule : expected_range_value_rules) {
    std::unique_ptr<RangeValueRule> rule;
    EXPECT_NO_THROW(rule = std::make_unique<RangeValueRule>(road_rulebook_->GetRangeValueRule(expected_rule.id())));
    EXPECT_TRUE(IsEqual<RangeValueRule>(expected_rule, *rule, malidrive::constants::kLinearTolerance));
  }
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
