// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/loader/loader.h"

#include <memory>
#include <optional>

#include <gtest/gtest.h>

#include "maliput_malidrive/builder/params.h"
#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/test_utilities/road_geometry_configuration_for_xodrs.h"
#include "utility/resources.h"

namespace malidrive {
namespace loader {
namespace test {
namespace {

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

class LoaderTestSingleLane : public ::testing::Test {
 protected:
  const std::map<std::string, std::string> road_geometry_configuration_{
      {builder::params::kRoadGeometryId, "test_id"},
      {builder::params::kOpendriveFile, utility::FindResourceInPath("SingleLane.xodr", kMalidriveResourceFolder)},
      {builder::params::kLinearTolerance, "1e-3"},
      {builder::params::kAngularTolerance, "1e-3"},
      {builder::params::kScaleLength, "1"},
      {builder::params::kOmitNonDrivableLanes, "false"},
  };

  const int kNumLanes{2};
  const maliput::api::LaneId kLaneId1{"1_0_-1"};
  const maliput::api::LaneId kLaneId2{"1_0_1"};
};

TEST_F(LoaderTestSingleLane, LoadARoadNetwork) {
  const std::unique_ptr<maliput::api::RoadNetwork> dut =
      loader::Load<builder::RoadNetworkBuilder>(road_geometry_configuration_);
  const auto rg = dut->road_geometry();
  EXPECT_EQ(road_geometry_configuration_.at(builder::params::kRoadGeometryId), rg->id().string());
  const auto lane_id_lane = rg->ById().GetLanes();
  EXPECT_EQ(kNumLanes, lane_id_lane.size());
  EXPECT_NE(lane_id_lane.at(kLaneId1), nullptr);
  EXPECT_NE(lane_id_lane.at(kLaneId2), nullptr);
}

}  // namespace
}  // namespace test
}  // namespace loader
}  // namespace malidrive
