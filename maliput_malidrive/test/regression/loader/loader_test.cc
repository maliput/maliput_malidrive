// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/loader/loader.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/test_utilities/road_geometry_configuration_for_xodrs.h"
#include "maliput_malidrive/utility/resources.h"

namespace malidrive {
namespace loader {
namespace test {
namespace {

class LoaderTestSingleLane : public ::testing::Test {
 protected:
  const builder::RoadGeometryConfiguration road_geometry_configuration_{
      GetRoadGeometryConfigurationFor("SingleLane.xodr").value()};

  const int kNumLanes{2};
  const maliput::api::LaneId kLaneId1{"1_0_-1"};
  const maliput::api::LaneId kLaneId2{"1_0_1"};
};

TEST_F(LoaderTestSingleLane, RoadGeometryGetter) {
  const std::unique_ptr<maliput::api::RoadNetwork> dut = loader::Load<builder::RoadNetworkBuilder>(
      {road_geometry_configuration_, std::nullopt, std::nullopt, std::nullopt, std::nullopt});
  const auto rg = dut->road_geometry();
  EXPECT_EQ(road_geometry_configuration_.id, rg->id());
  const auto lane_id_lane = rg->ById().GetLanes();
  EXPECT_EQ(kNumLanes, lane_id_lane.size());
  EXPECT_NE(lane_id_lane.at(kLaneId1), nullptr);
  EXPECT_NE(lane_id_lane.at(kLaneId2), nullptr);
}

}  // namespace
}  // namespace test
}  // namespace loader
}  // namespace malidrive
