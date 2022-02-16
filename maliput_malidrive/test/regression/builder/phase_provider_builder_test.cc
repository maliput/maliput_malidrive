// Copyright 2022 Toyota Research Institute
#include "maliput_malidrive/builder/phase_provider_builder.h"

#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/rules/phase_ring.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/base/manual_phase_provider.h>
#include <maliput/base/manual_phase_ring_book.h>
#include <maliput/base/phase_ring_book_loader.h>
#include <maliput/base/traffic_light_book_loader.h>

#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_geometry_builder.h"
#include "maliput_malidrive/builder/road_rulebook_builder.h"
#include "maliput_malidrive/builder/rule_registry_builder.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

using maliput::api::rules::Phase;
using maliput::api::rules::PhaseProvider;
using maliput::api::rules::PhaseRing;

// Creates the stack of entities that are necessary to populate the ManualPhaseProvider through the PhaseProviderBuilder
// functor.
class PhaseProviderBuilderTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto manager =
        xodr::LoadDataBaseFromFile(road_geometry_configuration_.opendrive_file, {constants::kLinearTolerance});
    road_geometry_ = RoadGeometryBuilder(std::move(manager), road_geometry_configuration_)();
    rule_registry_ = RuleRegistryBuilder(road_geometry_.get(), rule_registry_path)();
    road_rulebook_ = RoadRuleBookBuilder(road_geometry_.get(), rule_registry_.get(), road_rulebook_path)();
    traffic_light_book_ = maliput::LoadTrafficLightBookFromFile(traffic_light_path);
    phase_ring_book_ =
        maliput::LoadPhaseRingBookFromFile(road_rulebook_.get(), traffic_light_book_.get(), phase_ring_book_path);
  }

 protected:
  const std::string map_id{"odr/figure8_trafficlights/figure8_trafficlights"};
  const std::string xodr_file_path{utility::FindResource(map_id + ".xodr")};
  const std::string rule_registry_path{utility::FindResource(map_id + "_new_rules.yaml")};
  const std::string road_rulebook_path{utility::FindResource(map_id + "_new_rules.yaml")};
  const std::string traffic_light_path{utility::FindResource(map_id + "_new_rules.yaml")};
  const std::string phase_ring_book_path{utility::FindResource(map_id + "_new_rules.yaml")};
  const RoadGeometryConfiguration road_geometry_configuration_{RoadGeometryConfiguration::FromMap({
      {"opendrive_file", xodr_file_path},
      {"omit_nondrivable_lanes", "false"},
  })};

  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry_;
  std::unique_ptr<const maliput::api::rules::RoadRulebook> road_rulebook_;
  std::unique_ptr<maliput::api::rules::RuleRegistry> rule_registry_;
  std::unique_ptr<maliput::api::rules::TrafficLightBook> traffic_light_book_;
  std::unique_ptr<maliput::api::rules::PhaseRingBook> phase_ring_book_;
};

TEST_F(PhaseProviderBuilderTest, Constructor) {
  // Throws because PhaseRingBook pointer is null.
  EXPECT_THROW(PhaseProviderBuilder(nullptr)(), maliput::common::assertion_error);
  // Correct contruction.
  EXPECT_NO_THROW(PhaseProviderBuilder(phase_ring_book_.get())());
}

TEST_F(PhaseProviderBuilderTest, PhaseProvider) {
  const PhaseRing::Id kPhaseRingId{"Figure8Intersection"};

  // Valid current phase and next phase pairs.
  const std::unordered_map<Phase::Id, Phase::Id> expected_phase_next_phase{
      {Phase::Id{"EastWestPhase"}, Phase::Id{"NorthSouthPhase"}},
      {Phase::Id{"NorthSouthPhase"}, Phase::Id{"EastWestPhase"}},
  };
  const std::unique_ptr<maliput::api::rules::PhaseProvider> dut = PhaseProviderBuilder(phase_ring_book_.get())();
  const std::optional<PhaseProvider::Result> provider_result = dut->GetPhase(kPhaseRingId);
  ASSERT_NE(std::nullopt, provider_result);
  // It is not possible to predict the current phase as the PhaseProvderBuilder arbitrarily selects the initial phase.
  // However it is known what are the possible results.
  const auto obtained_phase = expected_phase_next_phase.find(provider_result->state);
  EXPECT_NE(expected_phase_next_phase.end(), obtained_phase);

  ASSERT_NE(std::nullopt, provider_result->next);
  EXPECT_EQ(expected_phase_next_phase.at(provider_result->state), provider_result->next.value().state);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
