// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/road_network_builder.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/regions.h>
#include <maliput/api/road_network_validator.h>
#include <maliput/api/rules/direction_usage_rule.h>
#include <maliput/api/rules/discrete_value_rule_state_provider.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/range_value_rule_state_provider.h>
#include <maliput/api/rules/right_of_way_rule.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/api/rules/speed_limit_rule.h>
#include <maliput/base/intersection_book.h>
#include <maliput/base/manual_phase_provider.h>
#include <maliput/base/manual_phase_ring_book.h>
#include <maliput/base/manual_range_value_rule_state_provider.h>
#include <maliput/base/manual_right_of_way_rule_state_provider.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/phase_based_right_of_way_rule_state_provider.h>
#include <maliput/base/rule_registry.h>
#include <maliput/base/traffic_light_book.h>

#include "maliput_malidrive/builder/road_geometry_builder.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/loader/loader.h"
#include "maliput_malidrive/test_utilities/road_geometry_configuration_for_xodrs.h"

#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::RoadGeometry;
using maliput::api::RoadGeometryId;
using maliput::api::SRange;
using maliput::api::rules::DirectionUsageRule;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::DiscreteValueRuleStateProvider;
using maliput::api::rules::Phase;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::RangeValueRuleStateProvider;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RoadRulebook;
using maliput::api::rules::Rule;
using maliput::api::rules::SpeedLimitRule;

using malidrive::test::GetRoadGeometryConfigurationFor;

// Functor to compare two maliput::api::rules::RangeValueRule::Ranges with
// a certain tolerance for `min` and `max`.
//
// It is thought to be used in functions like std::find_if.
struct RangeCompare {
  bool operator()(const RangeValueRule::Range& b) {
    if (a.severity != b.severity) {
      return false;
    }
    if (a.description != b.description) {
      return false;
    }
    if (std::abs(a.min - b.min) > tolerance) {
      return false;
    }
    if (std::abs(a.max - b.max) > tolerance) {
      return false;
    }
    return true;
  }
  RangeValueRule::Range a{};
  double tolerance{};
};

// Expects that two LaneSRanges are equal within `linear_tolerance`.
void CompareLaneSRange(const LaneSRange& dut, const LaneSRange& expectation, double linear_tolerance) {
  EXPECT_EQ(dut.lane_id(), expectation.lane_id());
  EXPECT_NEAR(dut.s_range().s0(), expectation.s_range().s0(), linear_tolerance);
  EXPECT_NEAR(dut.s_range().s1(), expectation.s_range().s1(), linear_tolerance);
}

// Holds the parameters to be checked.
struct RoadNetworkBuilderTestParameters {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadNetworkBuilderTestParameters);

  RoadNetworkBuilderTestParameters() = default;

  RoadNetworkBuilderTestParameters(const std::string& road_geometry_id_in, const std::string& path_to_xodr_file_in)
      : road_geometry_id(road_geometry_id_in), path_to_xodr_file(path_to_xodr_file_in) {}

  std::string road_geometry_id{};
  std::string path_to_xodr_file{};
};

// Returns a vector of test parameters for the Builder test.
std::vector<RoadNetworkBuilderTestParameters> InstantiateBuilderParameters() {
  return {
      /*
        SingleLane map has the following structure:
                          (0.,0.,0.)         (100.,0.,0.)
        Driving   | L: 1  |-------->---------| Width: 2.0m
        Track lane| L: 0  |========>=========| Width: 0m
        Driving   | L: -1 |-------->---------| Width: 2.0m
                          Road 1
                          Section 0
      */
      {"SingleLane", "SingleLane.xodr"},
      /*
        ArcLane map has the following structure:
                                (23.9388857642, 72.0457446219, 0.)
        Road 1                               \  \  \
        Section 0                             |  |  |
        - Curvature:0.025                     |  |  |
        - Length:100                          |  |  |
                            (0.,0.,0.)      _.'  /  /
        Driving   |Width: 2.0 m| L: 1 ....-'  _.'  /
        Track lane|Width: 0.0 m| L: 0 .....- ' _.'
        Driving   |Width: 2.0 m| L: -1......- '
      */
      {"ArcLane", "ArcLane.xodr"},
      /*
        SShapeRoad map has the following structure:
        - 1 Road
          - 1 Lane Section.
            - 3 Lanes: {-1 <Right, Driving>, 0 <Center, Track>, 1 <Left, Driving>}
              - Width: 2m each, except the center lane.
          - 3 geometries:
            - Arc: start: [x: 0., y: 0., h: 0.], curvature: 0.025m, length: 125.663706144m
            - Line: start: [x: 0., y: 80., h: π], length: 20.m
            - Arc: start: [x: -20., y: 80., h: π], curvature: -0.025m, length: 125.663706144m
      */
      {"SShapeRoad", "SShapeRoad.xodr"},
      /*
        LShapeRoad map has the following structure:
          - 3 Roads
            - 1 Lane Section.
              - 2 Lanes: {0 <Center, Track>, 1 <Left, Driving>}
                - Width: 2m each, except the center lane.
            - 3 geometries(1 per road):
              - Line: start: [x: 0., y: 0., h: 0.], length: 100.m
              - Arc: start: [x: 100., y: 0., h: 0.], curvature: 0.025, length: 62.831853072m
              - Line: start: [x: 140., y: 40., h: π/2], length: 100.m
      */
      {"LShapeRoad", "LShapeRoad.xodr"},
      /*
        LShapeRoadVariableLanes map has the following structure:
          - Road 1
            - 1 Lane Section.
              - 4 Lanes: {0 <Center, Track>, 1 <Left, Driving>, 2 <Left, Driving>, 3 <Left, Driving>}
                - Width: 2m each, except the center lane.
            - 1 geometry:
              - Line: start: [x: 0., y: 0., h: 0.], length: 100.m
          - Road 2
            - 1 Lane Section.
              - 3 Lanes: {0 <Center, Track>, 1 <Left, Driving>, 2 <Left, Driving>}
                - Width: 2m each, except the center lane.
            - 1 geometry:
              - Arc: start: [x: 100., y: 0., h: 0.], curvature: 0.025, length: 62.831853072m
          - Road 3
            - 1 Lane Section.
              - 2 Lanes: {0 <Center, Track>, 1 <Left, Driving>}
                - Width: 2m each, except the center lane.
            - 1 geometry:
              - Line: start: [x: 140., y: 40., h: π/2], length: 100.m

      */
      {"LShapeRoadVariableLanes", "LShapeRoadVariableLanes.xodr"},
      /*
        TShapeRoad describes a T shape intersection.
         - 3 Roads dont belong to a Junction.
         - 6 Roads belong to a Junction.
      */
      // TODO(#510): Test was commented out to avoid blocking development.
      //             See issue for more information.
      // {"TShapeRoad", "odr/TShapeRoad.xodr"},

      /*
        LineMultipleSections map has the following structure:
         - 1 Road:
                        (0.,0.,0.)   (33.3.,0.,0.)  (66.6,0.,0.)      (100.,0.,0.)
          Driving   | L: 1  |-------------->------------->------------| Width: 2.0m
          Track lane| L: 0  |==============>==========================| Width: 0m
          Driving   | L: -1 |-------------->------------->------------| Width: 2.0m
                                Section 0     Section 1     Section 0
      */
      // TODO(#510): Test was commented out to avoid blocking development.
      //             See issue for more information.
      // {"LineMultipleSections", "odr/LineMultipleSections.xodr"},

      /*
        ParkingGarageRamp describes a Road in the form of a parking garage ramp.
      */
      {"ParkingGarageRamp", "ParkingGarageRamp.xodr"},
      /*
        SShapeSuperelevatedRoad describes a road like SShapeRoad.xodr but it also includes superelevation.
      */
      {"SShapeSuperelevatedRoad", "SShapeSuperelevatedRoad.xodr"},
      /*
        LineVariableWidth describes a straight road with variable width.
      */
      {"LineVariableWidth", "LineVariableWidth.xodr"},
  };
}

// Set up basic configuration for the road geometry.
class RoadNetworkBuilderTest : public ::testing::TestWithParam<RoadNetworkBuilderTestParameters> {};

TEST_P(RoadNetworkBuilderTest, RoadGeometryBuilding) {
  RoadGeometryConfiguration road_geometry_configuration{
      GetRoadGeometryConfigurationFor(GetParam().path_to_xodr_file).value()};
  road_geometry_configuration.opendrive_file =
      utility::FindResource(road_geometry_configuration.opendrive_file.value());
  const std::unique_ptr<maliput::api::RoadNetwork> dut = builder::RoadNetworkBuilder(
      {road_geometry_configuration, std::nullopt, std::nullopt, std::nullopt, std::nullopt})();
  ASSERT_NE(dynamic_cast<const malidrive::RoadGeometry*>(dut->road_geometry()), nullptr);
  EXPECT_EQ(road_geometry_configuration.id, dut->road_geometry()->id());
  EXPECT_NO_THROW(maliput::api::ValidateRoadNetwork(
      *dut, {false, true /* Invariants */, false, false, false, false, false, false}));
}

INSTANTIATE_TEST_CASE_P(RoadNetworkBuilderTestGroup, RoadNetworkBuilderTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));

// TODO(#562): All the following tests are analogue to the tests performed in `road_network_builder_test`.
//             Code could be reused.

// Base testing class for the builder test.
class BuilderTest : public ::testing::Test {
 protected:
  static constexpr double kSpeedTolerance{constants::kSpeedTolerance};
};

// Evaluates registered rule types in RuleRegistry.
class RuleRegistryBuildTest : public BuilderTest {
 protected:
  void SetUp() override {
    const std::string kTShapeRoadYAMLPath{utility::FindResource("odr/TShapeRoad.yaml")};
    RoadGeometryConfiguration road_geometry_configuration{GetRoadGeometryConfigurationFor("TShapeRoad.xodr").value()};
    road_geometry_configuration.opendrive_file = utility::FindResource("odr/TShapeRoad.xodr");
    const RoadNetworkConfiguration road_network_configuration{road_geometry_configuration, kTShapeRoadYAMLPath,
                                                              kTShapeRoadYAMLPath, kTShapeRoadYAMLPath};
    rn_ = loader::Load<builder::RoadNetworkBuilder>(road_network_configuration);
    ASSERT_NE(rn_.get(), nullptr);
  }

  std::unique_ptr<maliput::api::RoadNetwork> rn_;
};

TEST_F(RuleRegistryBuildTest, DiscreteValueRuleTypesLoadTest) {
  const auto* rule_registry = rn_->rule_registry();
  ASSERT_NE(rule_registry, nullptr);

  // Two types, one for VehicleUsageRule type and other for VehicleUsageRule type.
  const std::map<Rule::TypeId, std::vector<std::string>> kExpectedDiscreteValues{
      {builder::rules::VehicleUsageRuleTypeId(), {"NonVehicles", "NonPedestrians", "Unrestricted"}},
      {builder::rules::VehicleExclusiveRuleTypeId(),
       {"BusOnly", "EmergencyVehiclesOnly", "HighOccupancyVehicleOnly", "MotorizedVehicleOnly",
        "NonMotorizedVehicleOnly"}},
      {maliput::DirectionUsageRuleTypeId(),
       {"WithS", "AgainstS", "Bidirectional", "BidirectionalTurnOnly", "NoUse", "Parking", "Undefined"}},
  };
  auto test_discrete_value_rule_type = [](const std::vector<DiscreteValueRule::DiscreteValue>& values,
                                          const std::vector<std::string>& expected_values) {
    EXPECT_EQ(values.size(), expected_values.size());
    const int severity{Rule::State::kStrict};
    for (const DiscreteValueRule::DiscreteValue& value : values) {
      EXPECT_EQ(value.severity, severity);
      EXPECT_NE(std::find(expected_values.begin(), expected_values.end(), value.value), expected_values.end());
    }
  };

  EXPECT_EQ(rule_registry->DiscreteValueRuleTypes().size(), kExpectedDiscreteValues.size());
  for (const auto& discrete_value_rule_type : rule_registry->DiscreteValueRuleTypes()) {
    // Tests Rule::TypeId.
    const auto expected_value_it = kExpectedDiscreteValues.find(discrete_value_rule_type.first);
    ASSERT_NE(expected_value_it, kExpectedDiscreteValues.end());
    // Tests the vector of DiscreteValueRule::DiscreteValues.
    test_discrete_value_rule_type(discrete_value_rule_type.second, expected_value_it->second);
  }
}

TEST_F(RuleRegistryBuildTest, RangeValueRuleTypesLoadTest) {
  const auto* rule_registry = rn_->rule_registry();
  ASSERT_NE(rule_registry, nullptr);

  const std::map<Rule::TypeId, std::vector<RangeValueRule::Range>> kExpectedRanges{
      // TShapeRoad has three XODR Roads (IDs in {0, 1, 2}) with a specific
      // speed node in which 40 MPH is set. Provided that the speed limit
      // value is not a default one, and the other XODR Roads do not have any
      // speed configuration, a second range is expected with constants::kDefaultMaxSpeedLimit.
      {maliput::SpeedLimitRuleTypeId(),
       {RangeValueRule::Range{Rule::State::kStrict,
                              {} /* related_rules */,
                              {} /* related_unique_ids */,
                              "m/s",
                              constants::kDefaultMinSpeedLimit,
                              xodr::ConvertToMs(40., xodr::Unit::kMph)},
        RangeValueRule::Range{Rule::State::kStrict,
                              {} /* related_rules */,
                              {} /* related_unique_ids */,
                              "m/s",
                              constants::kDefaultMinSpeedLimit,
                              constants::kDefaultMaxSpeedLimit}}},
  };

  auto test_range_rule_type = [tolerance = kSpeedTolerance](const std::vector<RangeValueRule::Range>& ranges,
                                                            const std::vector<RangeValueRule::Range>& expected_ranges) {
    EXPECT_EQ(ranges.size(), expected_ranges.size());
    for (const RangeValueRule::Range& range : ranges) {
      EXPECT_TRUE(std::find_if(expected_ranges.begin(), expected_ranges.end(), RangeCompare{range, tolerance}) !=
                  expected_ranges.end());
    }
  };

  EXPECT_EQ(rule_registry->RangeValueRuleTypes().size(), kExpectedRanges.size());
  for (const auto& range_value_rule_type : rule_registry->RangeValueRuleTypes()) {
    // Tests Rule::TypeId.
    const auto expected_range_it = kExpectedRanges.find(range_value_rule_type.first);
    ASSERT_NE(expected_range_it, kExpectedRanges.end());
    // Tests the vector of RangeValueRule::Ranges.
    test_range_rule_type(range_value_rule_type.second, expected_range_it->second);
  }
}

TEST_F(BuilderTest, CustomRoadNetworkEntitiesLoadersTest) {
  const std::string kTShapeRoadYAMLPath{utility::FindResource("odr/TShapeRoad.yaml")};
  RoadGeometryConfiguration road_geometry_configuration{GetRoadGeometryConfigurationFor("TShapeRoad.xodr").value()};
  road_geometry_configuration.opendrive_file = utility::FindResource("odr/TShapeRoad.xodr");
  const RoadNetworkConfiguration road_network_configuration{
      road_geometry_configuration, kTShapeRoadYAMLPath, kTShapeRoadYAMLPath, kTShapeRoadYAMLPath, kTShapeRoadYAMLPath};

  auto rn = loader::Load<builder::RoadNetworkBuilder>(road_network_configuration);
  ASSERT_NE(rn.get(), nullptr);

  const auto* rulebook = rn->rulebook();
  EXPECT_NE(rulebook, nullptr);
  EXPECT_NE(dynamic_cast<const maliput::ManualRulebook*>(rulebook), nullptr);
  EXPECT_NO_THROW(rulebook->GetRule(maliput::api::rules::RightOfWayRule::Id("EastApproach")));

  const auto* phase_ring_book = rn->phase_ring_book();
  EXPECT_NE(phase_ring_book, nullptr);
  EXPECT_NE(dynamic_cast<const maliput::ManualPhaseRingBook*>(phase_ring_book), nullptr);

  const auto* traffic_light_book = rn->traffic_light_book();
  EXPECT_NE(traffic_light_book, nullptr);
  EXPECT_NE(dynamic_cast<const maliput::TrafficLightBook*>(traffic_light_book), nullptr);
  const maliput::api::rules::TrafficLight* traffic_light =
      traffic_light_book->GetTrafficLight(maliput::api::rules::TrafficLight::Id("EastFacing"));
  EXPECT_NE(traffic_light, nullptr);
  auto intersection_book = rn->intersection_book();
  EXPECT_NE(intersection_book, nullptr);
  EXPECT_NE(dynamic_cast<const maliput::IntersectionBook*>(intersection_book), nullptr);
  auto intersection = intersection_book->GetIntersection(maliput::api::Intersection::Id("TIntersection"));
  EXPECT_NE(intersection, nullptr);

  // We should be able to phase and verify rule states as well.
  auto right_of_way_rule_state_provider = rn->right_of_way_rule_state_provider();
  auto phase_id = Phase::Id("AllStop");
  auto rule_id = RightOfWayRule::Id("EastApproach");
  auto go_rule_state_id = RightOfWayRule::State::Id("Go");
  auto stop_rule_state_id = RightOfWayRule::State::Id("Stop");
  auto result = right_of_way_rule_state_provider->GetState(rule_id);
  EXPECT_EQ(result.has_value(), true);
  EXPECT_EQ(go_rule_state_id, result.value().state);
  intersection->SetPhase(phase_id);
  result = right_of_way_rule_state_provider->GetState(rule_id);
  EXPECT_EQ(result.has_value(), true);
  EXPECT_EQ(stop_rule_state_id, result.value().state);
}

// Evaluates that the RoadNetwork is properly set based on current Builder
// development. This test should mutate and eventually vanish once
// properties are parsed and added to the RoadNetwork variables are correctly
// parsed.
TEST_F(BuilderTest, StubProperties) {
  const std::string kTShapeRoadYAMLPath{utility::FindResource("odr/TShapeRoad.yaml")};
  RoadGeometryConfiguration road_geometry_configuration{GetRoadGeometryConfigurationFor("TShapeRoad.xodr").value()};
  road_geometry_configuration.opendrive_file = utility::FindResource("odr/TShapeRoad.xodr");
  const RoadNetworkConfiguration road_network_configuration{road_geometry_configuration, kTShapeRoadYAMLPath,
                                                            kTShapeRoadYAMLPath, kTShapeRoadYAMLPath};

  auto rn = loader::Load<builder::RoadNetworkBuilder>(road_network_configuration);
  ASSERT_NE(rn.get(), nullptr);

  // TODO(agalbachicar)   Think of a way to test an empty Intersection vector.
  const auto* state_provider = rn->right_of_way_rule_state_provider();
  EXPECT_NE(state_provider, nullptr);
  EXPECT_NE(dynamic_cast<const maliput::PhaseBasedRightOfWayRuleStateProvider*>(state_provider), nullptr);

  const auto* phase_provider = rn->phase_provider();
  EXPECT_NE(phase_provider, nullptr);
  EXPECT_NE(dynamic_cast<const maliput::ManualPhaseProvider*>(phase_provider), nullptr);
}

// Holds reference values for a DirectionUsageRule check.
struct DirectionUsageReferenceValue {
  maliput::api::LaneId lane_id;
  SRange s_range;
  std::string state_type;
  int severity{};
};

// Holds all the reference values to DirectionUsageRules for a map.
struct DirectionUsageTruthTable {
  std::string file_path;
  std::vector<DirectionUsageReferenceValue> reference_values;
};

std::ostream& operator<<(std::ostream& os, const DirectionUsageTruthTable& tt) {
  os << tt.file_path << " ";
  return os;
}

// Class to hold common DirectionUsageRule test.
class DirectionUsageTest : public ::testing::TestWithParam<DirectionUsageTruthTable> {
 protected:
  static constexpr double kLinearTolerance{constants::kLinearTolerance};

  void SetUp() override {
    road_geometry_configuration_.opendrive_file =
        utility::FindResource(road_geometry_configuration_.opendrive_file.value());
    reference_values_ = GetParam().reference_values;
  }

  RoadGeometryConfiguration road_geometry_configuration_{GetRoadGeometryConfigurationFor(GetParam().file_path).value()};
  std::vector<DirectionUsageReferenceValue> reference_values_;
};

// Returns a vector containing expected values for different maps.
std::vector<DirectionUsageTruthTable> InstantiateDirectionUsageBuilderParameters() {
  const int strict_severity{Rule::State::kStrict};
  return {
      {"LineMultipleSections.xodr",
       {
           {LaneId("1_0_4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_1"), SRange(0., 33.3), "AgainstS", strict_severity},
           {LaneId("1_0_-1"), SRange(0., 33.3), "WithS", strict_severity},
           {LaneId("1_0_-2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_-3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_-4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_1"), SRange(0., 33.3), "AgainstS", strict_severity},
           {LaneId("1_1_-1"), SRange(0., 33.3), "WithS", strict_severity},
           {LaneId("1_1_-2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_-3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_-4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_2_4"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_3"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_2"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_1"), SRange(0., 33.4), "AgainstS", strict_severity},
           {LaneId("1_2_-1"), SRange(0., 33.4), "WithS", strict_severity},
           {LaneId("1_2_-2"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_-3"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_-4"), SRange(0., 33.4), "Bidirectional", strict_severity},
       }},
      {"LineMultipleSectionsMoreCases.xodr",
       {
           {LaneId("1_0_4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_1"), SRange(0., 33.3), "WithS", strict_severity},
           {LaneId("1_0_-1"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_-2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_-3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_0_-4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_1"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_-1"), SRange(0., 33.3), "WithS", strict_severity},
           {LaneId("1_1_-2"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_-3"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_1_-4"), SRange(0., 33.3), "Bidirectional", strict_severity},
           {LaneId("1_2_4"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_3"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_2"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_1"), SRange(0., 33.4), "AgainstS", strict_severity},
           {LaneId("1_2_-1"), SRange(0., 33.4), "Undefined", strict_severity},
           {LaneId("1_2_-2"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_-3"), SRange(0., 33.4), "Bidirectional", strict_severity},
           {LaneId("1_2_-4"), SRange(0., 33.4), "Bidirectional", strict_severity},
       }},
      {"TShapeRoad.xodr",
       {
           {LaneId("0_0_4"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("0_0_3"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("0_0_2"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("0_0_1"), SRange(0., 46.), "AgainstS", strict_severity},
           {LaneId("0_0_-1"), SRange(0., 46.), "WithS", strict_severity},
           {LaneId("0_0_-2"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("0_0_-3"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("0_0_-4"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("1_0_4"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("1_0_3"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("1_0_2"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("1_0_1"), SRange(0., 46.), "AgainstS", strict_severity},
           {LaneId("1_0_-1"), SRange(0., 46.), "WithS", strict_severity},
           {LaneId("1_0_-2"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("1_0_-3"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("1_0_-4"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("2_0_4"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("2_0_3"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("2_0_2"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("2_0_1"), SRange(0., 46.), "AgainstS", strict_severity},
           {LaneId("2_0_-1"), SRange(0., 46.), "WithS", strict_severity},
           {LaneId("2_0_-2"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("2_0_-3"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("2_0_-4"), SRange(0., 46.), "Bidirectional", strict_severity},
           {LaneId("4_0_1"), SRange(0., 8.), "AgainstS", strict_severity},
           {LaneId("5_0_-1"), SRange(0., 8.), "WithS", strict_severity},
           {LaneId("6_0_-1"), SRange(0., 9.0661910958999066), "WithS", strict_severity},
           {LaneId("7_0_-1"), SRange(0., 3.558741286878631), "WithS", strict_severity},
           {LaneId("8_0_-1"), SRange(0., 9.0661910958999101), "WithS", strict_severity},
           {LaneId("9_0_-1"), SRange(0., 3.5587412868786292), "WithS", strict_severity},
       }},
  };
}

// Evaluates DirectionUsageRules construction.
TEST_P(DirectionUsageTest, DirectionUsageRuleTest) {
  auto rn = loader::Load<builder::RoadNetworkBuilder>({road_geometry_configuration_});
  ASSERT_NE(rn.get(), nullptr);

  // { @ Code in this block must be removed once the old DirectionUsageRules are deprecated.
  const std::unordered_map<DirectionUsageRule::State::Type, std::string> state_to_string{
      {DirectionUsageRule::State::Type::kAgainstS, "AgainstS"},
      {DirectionUsageRule::State::Type::kBidirectional, "Bidirectional"},
      {DirectionUsageRule::State::Type::kWithS, "WithS"},
      {DirectionUsageRule::State::Type::kUndefined, "Undefined"},
  };
  // Verify coverage of the lanes in the RoadNetwork by DirectionUsage rules.
  for (const auto lane : rn->road_geometry()->ById().GetLanes()) {
    const auto results = rn->rulebook()->FindRules({{lane.first, {0.0, lane.second->length()}}}, 0);

    EXPECT_EQ(static_cast<int>(results.direction_usage.size()), 1);

    // Rules should cover the whole Lane.
    const DirectionUsageRule dut = results.direction_usage.begin()->second;

    // Only one State usage per rule.
    EXPECT_TRUE(dut.is_static());
    // TODO(agalbachicar)   When supporting multiple states, provide an
    //                      appropriate test.
    const DirectionUsageRule::State state = dut.static_state();
    EXPECT_EQ(state.id().string(), dut.id().string());

    /// Returns the zone to which this rule instance applies, then it is tested.
    const LaneSRange dut_zone = dut.zone();
    auto it_zone = std::find_if(reference_values_.begin(), reference_values_.end(),
                                [dut_zone](const DirectionUsageReferenceValue& expected_ref_value) {
                                  return expected_ref_value.lane_id.string() == dut_zone.lane_id().string();
                                });
    EXPECT_NE(it_zone, reference_values_.end());
    CompareLaneSRange(dut_zone, LaneSRange(it_zone->lane_id, it_zone->s_range), kLinearTolerance);
  }

  // Check directions and severity for lanes using FindRules
  for (const auto reference_value : reference_values_) {
    auto lane = rn->road_geometry()->ById().GetLane(reference_value.lane_id);
    ASSERT_NE(lane, nullptr);
    const SRange s_range(0., lane->length());

    const LaneSRange lane_s_range(lane->id(), s_range);
    const std::vector<LaneSRange> lane_s_ranges(1, lane_s_range);
    auto query_result = rn->rulebook()->FindRules(lane_s_ranges, 0.);
    const int n_direction_rules = static_cast<int>(query_result.direction_usage.size());
    EXPECT_EQ(n_direction_rules, 1);

    const auto& direction_rule = query_result.direction_usage.begin()->second;
    EXPECT_EQ(direction_rule.is_static(), true);

    const auto& states = direction_rule.states();
    EXPECT_EQ(static_cast<int>(states.size()), 1);

    const auto static_state = direction_rule.static_state();
    EXPECT_EQ(state_to_string.at(static_state.type()), reference_value.state_type);
    EXPECT_EQ(static_cast<int>(static_state.severity()), reference_value.severity);
  }
  // } @

  // Rule coverage by LaneSRange.
  for (const auto lane_id_lane : rn->road_geometry()->ById().GetLanes()) {
    const auto results = rn->rulebook()->FindRules({{lane_id_lane.first, {0.0, lane_id_lane.second->length()}}}, 0);
    EXPECT_TRUE(std::find_if(results.discrete_value_rules.begin(), results.discrete_value_rules.end(),
                             [](const auto& rule_id_rule) {
                               return rule_id_rule.second.type_id() == maliput::DirectionUsageRuleTypeId();
                             }) != results.discrete_value_rules.end());
  }
  // Rule construction.
  for (const DirectionUsageReferenceValue& reference_value : reference_values_) {
    const Rule::Id rule_id = GetRuleIdFrom(maliput::DirectionUsageRuleTypeId(), reference_value.lane_id);
    const DiscreteValueRule dut = rn->rulebook()->GetDiscreteValueRule(rule_id);
    EXPECT_EQ(dut.id(), rule_id);
    EXPECT_EQ(dut.type_id(), maliput::DirectionUsageRuleTypeId());
    EXPECT_EQ(static_cast<int>(dut.zone().ranges().size()), 1);
    CompareLaneSRange(dut.zone().ranges()[0], LaneSRange{reference_value.lane_id, {reference_value.s_range}},
                      kLinearTolerance);
    EXPECT_EQ(static_cast<int>(dut.values().size()), 1);
    EXPECT_EQ(dut.values()[0].severity, reference_value.severity);
    EXPECT_EQ(reference_value.state_type, dut.values()[0].value);
    EXPECT_TRUE(dut.values()[0].related_rules.empty());
  }
}

INSTANTIATE_TEST_CASE_P(DirectionUsageTestGroup, DirectionUsageTest,
                        ::testing::ValuesIn(InstantiateDirectionUsageBuilderParameters()));

// Holds reference values for VehicleUsageRule and VehicleExclusiveRule check.
struct VehicleRulesReferenceValue {
  maliput::api::LaneId lane_id;
  DiscreteValueRule::TypeId rule_type;
  int state_severity;
  std::string discrete_value;
  Rule::RelatedRules related_rules;
  SRange s_range;
};

// Holds reference values for VehicleUsageRule and VehicleExclusiveRule check.
struct VehicleRulesTruthTable {
  std::string file_path;
  std::vector<VehicleRulesReferenceValue> reference_values;
};

// Returns a vector containing expected values for different maps.
std::vector<VehicleRulesTruthTable> InstantiateVehicleRulesTestParameters() {
  const std::string non_motorized_vehicles_only{"NonMotorizedVehicleOnly"};
  const std::string non_pedestrians{"NonPedestrians"};
  const int strict_severity{Rule::State::kStrict};
  const Rule::RelatedRules empty_related_rules{};

  return {
      {"TShapeRoad.xodr",
       {
           {LaneId("0_0_1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 46.)},
           {LaneId("0_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 46.)},
           {LaneId("1_0_1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 46.)},
           {LaneId("1_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 46.)},
           {LaneId("2_0_1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 46.)},
           {LaneId("2_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 46.)},
           {LaneId("4_0_1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 8.)},
           {LaneId("5_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 8.)},
           {LaneId("6_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 9.0661910958999066)},
           {LaneId("7_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 3.558741286878631)},
           {LaneId("8_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 9.0661910958999101)},
           {LaneId("9_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 3.5587412868786292)},
       }},
      {"BikingLineLane.xodr",
       {
           {LaneId("1_0_2"), rules::VehicleExclusiveRuleTypeId(), strict_severity, non_motorized_vehicles_only,
            empty_related_rules, SRange(0., 100.)},
           {LaneId("1_0_2"),
            rules::VehicleUsageRuleTypeId(),
            strict_severity,
            non_pedestrians,
            {{rules::VehicleExclusiveRuleTypeId().string(),
              std::vector<Rule::Id>{GetRuleIdFrom(rules::VehicleExclusiveRuleTypeId(), LaneId("1_0_2"))}}},
            SRange(0., 100.)},
           {LaneId("1_0_1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 100.)},
           {LaneId("1_0_-1"), rules::VehicleUsageRuleTypeId(), strict_severity, non_pedestrians, empty_related_rules,
            SRange(0., 100.)},
       }},
  };
}

class VehicleRulesTest : public ::testing::TestWithParam<VehicleRulesTruthTable> {
 protected:
  static constexpr double kLinearTolerance{constants::kLinearTolerance};

  void SetUp() override {
    road_geometry_configuration_.opendrive_file =
        utility::FindResource(road_geometry_configuration_.opendrive_file.value());
    reference_values_ = GetParam().reference_values;
  }

  RoadGeometryConfiguration road_geometry_configuration_{GetRoadGeometryConfigurationFor(GetParam().file_path).value()};
  std::vector<VehicleRulesReferenceValue> reference_values_;
};

// Evaluates that all lanes have a VehicleUsageRule with the appropriate discrete value. Rule coverage zone must be
// always the full lane length.
TEST_P(VehicleRulesTest, VehicleRulesTest) {
  auto rn = loader::Load<builder::RoadNetworkBuilder>({road_geometry_configuration_});
  ASSERT_NE(rn.get(), nullptr);

  auto test_rule = [&](const DiscreteValueRule& rule, const VehicleRulesReferenceValue& reference_value) {
    EXPECT_EQ(rule.type_id(), reference_value.rule_type);
    EXPECT_EQ(static_cast<int>(rule.values().size()), 1);
    EXPECT_EQ(rule.values()[0].severity, reference_value.state_severity);
    EXPECT_EQ(rule.values()[0].related_rules.size(), reference_value.related_rules.size());
    for (const auto& rule_group_rule_ids : rule.values()[0].related_rules) {
      ASSERT_TRUE(reference_value.related_rules.find(rule_group_rule_ids.first) != reference_value.related_rules.end());
      for (const Rule::Id& rule_id : rule_group_rule_ids.second) {
        EXPECT_TRUE(std::find(reference_value.related_rules.at(rule_group_rule_ids.first).begin(),
                              reference_value.related_rules.at(rule_group_rule_ids.first).end(),
                              rule_id) != reference_value.related_rules.at(rule_group_rule_ids.first).end());
      }
    }
    EXPECT_EQ(rule.values()[0].value, reference_value.discrete_value);
    EXPECT_EQ(static_cast<int>(rule.zone().ranges().size()), 1);
    CompareLaneSRange(rule.zone().ranges()[0], LaneSRange(reference_value.lane_id, reference_value.s_range),
                      rn->road_geometry()->linear_tolerance());
  };

  // Evaluates rule attributes and retrieves them by Rule::Id and LaneSRange.
  for (const VehicleRulesReferenceValue& reference_value : reference_values_) {
    const Rule::Id rule_id(GetRuleIdFrom(reference_value.rule_type, reference_value.lane_id));
    test_rule(rn->rulebook()->GetDiscreteValueRule(rule_id), reference_value);

    const RoadRulebook::QueryResults query_result =
        rn->rulebook()->FindRules({LaneSRange(reference_value.lane_id, reference_value.s_range)}, kLinearTolerance);
    ASSERT_FALSE(query_result.discrete_value_rules.empty());
    EXPECT_TRUE(query_result.discrete_value_rules.find(rule_id) != query_result.discrete_value_rules.end());
  }
}

INSTANTIATE_TEST_CASE_P(VehicleRulesTestGroup, VehicleRulesTest,
                        ::testing::ValuesIn(InstantiateVehicleRulesTestParameters()));

// Holds reference values to create Rule::Id and evaluate that
// the state in the StateProvider is the same as the first value in
// the rule' set of values.
struct VehicleRulesStateProviderReferenceValue {
  maliput::api::LaneId lane_id;
  DiscreteValueRule::TypeId rule_type;
};

// Holds the file path and the references for the file.
struct VehicleRulesStateProviderTruthTable {
  std::string file_path;
  std::vector<VehicleRulesStateProviderReferenceValue> reference_values;
};

class VehicleRulesStateProviderTest : public ::testing::TestWithParam<VehicleRulesStateProviderTruthTable> {
 protected:
  void SetUp() override {
    road_geometry_configuration_.opendrive_file =
        utility::FindResource(road_geometry_configuration_.opendrive_file.value());
    reference_values_ = GetParam().reference_values;
  }

  RoadGeometryConfiguration road_geometry_configuration_{GetRoadGeometryConfigurationFor(GetParam().file_path).value()};
  std::vector<VehicleRulesStateProviderReferenceValue> reference_values_;
};

// Returns a vector containing expected values for different maps.
std::vector<VehicleRulesStateProviderTruthTable> InstantiateVehicleRulesStateProviderTestParameters() {
  return {
      {"TShapeRoad.xodr",
       {
           {LaneId("0_0_4"), rules::VehicleUsageRuleTypeId()},  {LaneId("0_0_3"), rules::VehicleUsageRuleTypeId()},
           {LaneId("0_0_2"), rules::VehicleUsageRuleTypeId()},  {LaneId("0_0_1"), rules::VehicleUsageRuleTypeId()},
           {LaneId("0_0_-1"), rules::VehicleUsageRuleTypeId()}, {LaneId("0_0_-2"), rules::VehicleUsageRuleTypeId()},
           {LaneId("0_0_-3"), rules::VehicleUsageRuleTypeId()}, {LaneId("0_0_-4"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_4"), rules::VehicleUsageRuleTypeId()},  {LaneId("1_0_3"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_2"), rules::VehicleUsageRuleTypeId()},  {LaneId("1_0_1"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_-1"), rules::VehicleUsageRuleTypeId()}, {LaneId("1_0_-2"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_-3"), rules::VehicleUsageRuleTypeId()}, {LaneId("1_0_-4"), rules::VehicleUsageRuleTypeId()},
           {LaneId("2_0_4"), rules::VehicleUsageRuleTypeId()},  {LaneId("2_0_3"), rules::VehicleUsageRuleTypeId()},
           {LaneId("2_0_2"), rules::VehicleUsageRuleTypeId()},  {LaneId("2_0_1"), rules::VehicleUsageRuleTypeId()},
           {LaneId("2_0_-1"), rules::VehicleUsageRuleTypeId()}, {LaneId("2_0_-2"), rules::VehicleUsageRuleTypeId()},
           {LaneId("2_0_-3"), rules::VehicleUsageRuleTypeId()}, {LaneId("2_0_-4"), rules::VehicleUsageRuleTypeId()},
           {LaneId("4_0_1"), rules::VehicleUsageRuleTypeId()},  {LaneId("5_0_-1"), rules::VehicleUsageRuleTypeId()},
           {LaneId("6_0_-1"), rules::VehicleUsageRuleTypeId()}, {LaneId("7_0_-1"), rules::VehicleUsageRuleTypeId()},
           {LaneId("8_0_-1"), rules::VehicleUsageRuleTypeId()}, {LaneId("9_0_-1"), rules::VehicleUsageRuleTypeId()},
       }},
      {"BikingLineLane.xodr",
       {
           {LaneId("1_0_4"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_3"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_2"), rules::VehicleExclusiveRuleTypeId()},
           {LaneId("1_0_2"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_1"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_-1"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_-2"), rules::VehicleUsageRuleTypeId()},
           {LaneId("1_0_-3"), rules::VehicleUsageRuleTypeId()},
       }},
  };
}

// Evaluates that the rule state provider has an entry for each rule, next
// is nullopt and the current state is the first state in rule's values.
TEST_P(VehicleRulesStateProviderTest, DiscreteValueRuleStateProviderTest) {
  auto rn = loader::Load<builder::RoadNetworkBuilder>({road_geometry_configuration_});
  ASSERT_NE(rn.get(), nullptr);

  for (const VehicleRulesStateProviderReferenceValue& reference_value : reference_values_) {
    const Rule::Id rule_id(GetRuleIdFrom(reference_value.rule_type, reference_value.lane_id));
    const DiscreteValueRule rule = rn->rulebook()->GetDiscreteValueRule(rule_id);

    const std::optional<DiscreteValueRuleStateProvider::StateResult> state_result =
        rn->discrete_value_rule_state_provider()->GetState(rule_id);
    EXPECT_TRUE(state_result.has_value());
    EXPECT_FALSE(state_result->next.has_value());
    EXPECT_EQ(state_result->state, rule.values().front());
  }
}

INSTANTIATE_TEST_CASE_P(VehicleRulesStateProviderTestGroup, VehicleRulesStateProviderTest,
                        ::testing::ValuesIn(InstantiateVehicleRulesStateProviderTestParameters()));

struct ValueAndRange {
  double max{};
  SRange s_range;
};

// Holds reference values for (new and old) SpeedLimitRule checks.
struct SpeedLimitRuleReferenceValue {
  maliput::api::LaneId lane_id;
  int state_severity;
  std::map<int, ValueAndRange> index_values_ranges;
};

// Holds reference values for (new and old) SpeedLimitRule checks.
struct SpeedLimitRuleTruthTable {
  std::string file_path;
  std::vector<SpeedLimitRuleReferenceValue> reference_values;
};

class SpeedLimitRuleBuilderTest : public ::testing::TestWithParam<SpeedLimitRuleTruthTable> {
 protected:
  static constexpr double kLinearTolerance{constants::kLinearTolerance};
  static constexpr double kSpeedTolerance{constants::kSpeedTolerance};

  void SetUp() override {
    road_geometry_configuration_.opendrive_file =
        utility::FindResource(road_geometry_configuration_.opendrive_file.value());
    reference_values_ = GetParam().reference_values;
  }

  RoadGeometryConfiguration road_geometry_configuration_{GetRoadGeometryConfigurationFor(GetParam().file_path).value()};
  std::vector<SpeedLimitRuleReferenceValue> reference_values_;
};

std::vector<SpeedLimitRuleTruthTable> SpeedLimitRuleTestParameters() {
  return {
      {"LineMultipleSections.xodr",
       {
           {LaneId{"1_0_4"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_0_3"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_0_2"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_0_1"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_0_-1"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_0_-2"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_0_-3"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_0_-4"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},

           {LaneId{"1_1_4"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_1_3"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_1_2"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_1_1"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_1_-1"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_1_-2"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_1_-3"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_1_-4"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},

           {LaneId{"1_2_4"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
           {LaneId{"1_2_3"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
           {LaneId{"1_2_2"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
           {LaneId{"1_1_1"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.3)}}}},
           {LaneId{"1_2_1"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
           {LaneId{"1_2_-1"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
           {LaneId{"1_2_-2"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
           {LaneId{"1_2_-3"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
           {LaneId{"1_2_-4"},
            static_cast<int>(SpeedLimitRule::Severity::kStrict),
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 33.4)}}}},
       }},
      {"TShapeRoad.xodr",
       {
           {LaneId("0_0_-4"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("0_0_-3"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("0_0_-2"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("0_0_-1"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("0_0_1"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("0_0_2"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("0_0_3"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("0_0_4"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_-4"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_-3"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_-2"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_-1"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_1"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_2"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_3"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("1_0_4"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_4"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_3"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_2"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_1"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_-1"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_-2"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_-3"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("2_0_-4"), Rule::State::kStrict, {{1, {xodr::ConvertToMs(40., xodr::Unit::kMph), SRange(0., 46.)}}}},
           {LaneId("4_0_1"), Rule::State::kStrict, {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 8.)}}}},
           {LaneId("5_0_-1"), Rule::State::kStrict, {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 8.)}}}},
           {LaneId("6_0_-1"),
            Rule::State::kStrict,
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 9.0661910958999066)}}}},
           {LaneId("7_0_-1"),
            Rule::State::kStrict,
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 3.558741286878631)}}}},
           {LaneId("8_0_-1"),
            Rule::State::kStrict,
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 9.0661910958999101)}}}},
           {LaneId("9_0_-1"),
            Rule::State::kStrict,
            {{1, {constants::kDefaultMaxSpeedLimit, SRange(0., 3.558741286878631)}}}},
       }},
      {"LineMultipleSpeeds.xodr",
       {
           {LaneId("1_0_1"),
            Rule::State::kStrict,
            {{1, {xodr::ConvertToMs(48., xodr::Unit::kMph), SRange(0., 10.)}},
             {2, {xodr::ConvertToMs(50., xodr::Unit::kMph), SRange(10., 20.)}},
             {3, {xodr::ConvertToMs(30., xodr::Unit::kMph), SRange(20., 33.3)}}}},
           {LaneId("1_1_1"),
            Rule::State::kStrict,
            {{1, {xodr::ConvertToMs(11., xodr::Unit::kMph), SRange(0., 6.7)}},
             {2, {xodr::ConvertToMs(30., xodr::Unit::kMph), SRange(6.7, 33.3)}}}},
           {LaneId("1_2_1"),
            Rule::State::kStrict,
            {{1, {xodr::ConvertToMs(22., xodr::Unit::kMph), SRange(0., 3.4)}},
             {2, {xodr::ConvertToMs(33., xodr::Unit::kMph), SRange(3.4, 19.4)}},
             {3, {xodr::ConvertToMs(44., xodr::Unit::kMph), SRange(19.4, 33.4)}}}},
       }},
  };
}

INSTANTIATE_TEST_CASE_P(SpeedLimitRuleBuilderTestGroup, SpeedLimitRuleBuilderTest,
                        ::testing::ValuesIn(SpeedLimitRuleTestParameters()));

TEST_P(SpeedLimitRuleBuilderTest, SpeedLimitRulesTest) {
  auto rn = loader::Load<builder::RoadNetworkBuilder>({road_geometry_configuration_});
  ASSERT_NE(rn.get(), nullptr);

  // { @ Code in this block must be removed once the old SpeedLimitRules are deprecated.
  for (const auto lane_id_lane : rn->road_geometry()->ById().GetLanes()) {
    // Gets the reference values for this LaneId.
    const SpeedLimitRuleReferenceValue reference_value =
        *(std::find_if(reference_values_.begin(), reference_values_.end(),
                       [lane_id = lane_id_lane.first](const SpeedLimitRuleReferenceValue& ref_value) {
                         return ref_value.lane_id == lane_id;
                       }));

    // Evaluates spatial coverage.
    const LaneSRange lane_s_range{lane_id_lane.first, {0.0, lane_id_lane.second->length()}};
    const auto results = rn->rulebook()->FindRules({lane_s_range}, 0. /* tolerance */);
    ASSERT_EQ(results.speed_limit.size(), reference_value.index_values_ranges.size());
    // Evaluates construction.
    int index{};
    for (const auto& speed_limit : results.speed_limit) {
      index++;
      const SpeedLimitRule dut = speed_limit.second;
      EXPECT_EQ(dut.severity(), SpeedLimitRule::Severity(reference_value.state_severity));
      // All SpeedLimitRules are set to constant::kDefaultMinSpeedLimit as it is not
      // defined in OpenDRIVE maps.
      EXPECT_DOUBLE_EQ(dut.min(), constants::kDefaultMinSpeedLimit);

      const auto index_value_range = std::find_if(
          reference_value.index_values_ranges.begin(), reference_value.index_values_ranges.end(),
          [index](const std::pair<int, ValueAndRange>& index_value_range) { return index_value_range.first == index; });
      ASSERT_TRUE(index_value_range != reference_value.index_values_ranges.end());

      EXPECT_NEAR(dut.max(), index_value_range->second.max, kSpeedTolerance);
      const LaneSRange lane_s_range_of_rule{
          lane_id_lane.first, {index_value_range->second.s_range.s0(), index_value_range->second.s_range.s1()}};
      CompareLaneSRange(dut.zone(), lane_s_range_of_rule, kLinearTolerance);
    }
  }
  // } @

  // Rule coverage by LaneSRange.
  for (const auto lane_id_lane : rn->road_geometry()->ById().GetLanes()) {
    const auto results = rn->rulebook()->FindRules({{lane_id_lane.first, {0.0, lane_id_lane.second->length()}}}, 0);
    EXPECT_TRUE(
        std::find_if(results.range_value_rules.begin(), results.range_value_rules.end(), [](const auto& rule_id_rule) {
          return rule_id_rule.second.type_id() == maliput::SpeedLimitRuleTypeId();
        }) != results.range_value_rules.end());
  }
  // Rule construction.
  for (const SpeedLimitRuleReferenceValue& reference_value : reference_values_) {
    for (const auto& index_value_range : reference_value.index_values_ranges) {
      const Rule::Id rule_id =
          GetRuleIdFrom(maliput::SpeedLimitRuleTypeId(), reference_value.lane_id, index_value_range.first);
      const RangeValueRule dut = rn->rulebook()->GetRangeValueRule(rule_id);
      EXPECT_EQ(dut.id(), rule_id);
      EXPECT_EQ(dut.type_id(), maliput::SpeedLimitRuleTypeId());
      EXPECT_EQ(static_cast<int>(dut.zone().ranges().size()), 1);
      CompareLaneSRange(dut.zone().ranges()[0], LaneSRange{reference_value.lane_id, index_value_range.second.s_range},
                        kLinearTolerance);
      EXPECT_EQ(static_cast<int>(dut.ranges().size()), 1);
      EXPECT_EQ(dut.ranges()[0].severity, reference_value.state_severity);
      EXPECT_TRUE(dut.ranges()[0].related_rules.empty());
      EXPECT_DOUBLE_EQ(dut.ranges()[0].min, constants::kDefaultMinSpeedLimit);
      EXPECT_NEAR(dut.ranges()[0].max, index_value_range.second.max, kSpeedTolerance);
    }
  }
}

// Holds reference values to create Rule::Id and evaluate that
// the state in the StateProvider is the same as the first value in
// the rule' set of values.
struct RangeValueRuleStateProviderReferenceValue {
  maliput::api::LaneId lane_id;
  RangeValueRule::TypeId rule_type;
};

// Holds the file path and the references for the file.
struct RangeValueRuleStateProviderTruthTable {
  std::string file_path;
  std::vector<RangeValueRuleStateProviderReferenceValue> reference_values;
};

class RangeValueRuleStateProviderTest : public ::testing::TestWithParam<RangeValueRuleStateProviderTruthTable> {
 protected:
  void SetUp() override {
    road_geometry_configuration_.opendrive_file =
        utility::FindResource(road_geometry_configuration_.opendrive_file.value());
    reference_values_ = GetParam().reference_values;
  }

  std::vector<RangeValueRuleStateProviderReferenceValue> reference_values_;
  RoadGeometryConfiguration road_geometry_configuration_{GetRoadGeometryConfigurationFor(GetParam().file_path).value()};
};

// Returns a vector containing expected values for different maps.
std::vector<RangeValueRuleStateProviderTruthTable> RangeValueRuleStateProviderTestParameters() {
  return {
      {"LineMultipleSections.xodr",
       {
           {LaneId{"1_0_1"}, maliput::SpeedLimitRuleTypeId()},
           {LaneId{"1_0_-1"}, maliput::SpeedLimitRuleTypeId()},
           {LaneId{"1_1_1"}, maliput::SpeedLimitRuleTypeId()},
           {LaneId{"1_1_-1"}, maliput::SpeedLimitRuleTypeId()},
           {LaneId{"1_2_1"}, maliput::SpeedLimitRuleTypeId()},
           {LaneId{"1_2_-1"}, maliput::SpeedLimitRuleTypeId()},
       }},
      {"TShapeRoad.xodr",
       {
           {LaneId("0_0_1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("0_0_-1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("1_0_1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("1_0_-1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("2_0_1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("2_0_-1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("4_0_1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("5_0_-1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("6_0_-1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("7_0_-1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("8_0_-1"), maliput::SpeedLimitRuleTypeId()},
           {LaneId("9_0_-1"), maliput::SpeedLimitRuleTypeId()},
       }},
  };
}

// Evaluates that the rule state provider has an entry for each rule, next
// is nullopt and the current state is the first state in rule's values.
TEST_P(RangeValueRuleStateProviderTest, RangeValueRuleStateProviderTest) {
  auto rn = loader::Load<builder::RoadNetworkBuilder>({road_geometry_configuration_});
  ASSERT_NE(rn.get(), nullptr);

  for (const RangeValueRuleStateProviderReferenceValue& reference_value : reference_values_) {
    const Rule::Id rule_id(GetRuleIdFrom(reference_value.rule_type, reference_value.lane_id, 1));
    const RangeValueRule rule = rn->rulebook()->GetRangeValueRule(rule_id);

    const std::optional<RangeValueRuleStateProvider::StateResult> state_result =
        rn->range_value_rule_state_provider()->GetState(rule_id);
    EXPECT_TRUE(state_result.has_value());
    EXPECT_FALSE(state_result->next.has_value());
    EXPECT_EQ(state_result->state, rule.ranges().front());
  }
}

INSTANTIATE_TEST_CASE_P(RangeValueRuleStateProviderTestGroup, RangeValueRuleStateProviderTest,
                        ::testing::ValuesIn(RangeValueRuleStateProviderTestParameters()));

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
