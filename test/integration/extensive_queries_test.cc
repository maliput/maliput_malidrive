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
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/common/filesystem.h>
#include <maliput/utility/generate_obj.h>

#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/loader/loader.h"
#include "maliput_malidrive/test_utilities/road_geometry_configuration_for_xodrs.h"
#include "utility/file_tools.h"

namespace malidrive {
namespace test {

// To improve test speed, each test should be written inside a function of this
// class, that way the same RoadNetwork is used every time and Load() is only
// called once for all tests. Note that the RoadGeometry can't be modified after
// the Build() stage.
class MalidriveExtensiveQueriesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    xodr_file_path_ = XODR_FILE;
    ASSERT_TRUE(!xodr_file_path_.empty());
  }

  void TearDown() override {
    // filesystem has no functionality for reading/walking a
    // directory, so we have to keep track of created files manually and
    // delete them by hand.
    for (const maliput::common::Path& path : paths_to_cleanup_) {
      EXPECT_TRUE(maliput::common::Filesystem::remove_file(path));
    }
    if (directory_.is_directory()) {
      ASSERT_TRUE(maliput::common::Filesystem::remove_directory(directory_));
    }
  }

  // Samples all roads, asking for different properties() at linear_tolerance
  // s-steps. The purpose of this test is to verify consistency with all the
  // available examples.
  void RunLaneBoundsTest() {
    const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*> lane_map =
        rn_->road_geometry()->ById().GetLanes();

    for (const auto lane_id_lane : lane_map) {
      const maliput::api::Lane* lane = lane_id_lane.second;
      ASSERT_NE(lane, nullptr);
      const double length = lane->length();
      const double kSStep{length * linear_tolerance_ / scale_length_};

      bool is_last_point{false};
      for (double s = 0.; s <= length && !is_last_point;) {
        ASSERT_NO_THROW(lane->lane_bounds(s)) << lane->id().string() << " fails at: " << s;
        s += kSStep;
        if (s > length) {
          s = length;
          is_last_point = true;
        }
      }
    }
  }

  // Evaluates all BranchPoints, making sure that a Lane does not appear on both
  // ends of the BranchPoint. There are two possible reasons for this behavior:
  // - The lane self connects in the map description, for example a roundabout.
  // - Lane's length is smaller than linear tolerance, which makes both extremes
  //   to be "self-connected" when they are not.
  void RunBranchPointTransitionTest() {
    const maliput::api::RoadGeometry* road_geometry = rn_->road_geometry();

    for (int i = 0; i < road_geometry->num_branch_points(); ++i) {
      const maliput::api::BranchPoint* bp = road_geometry->branch_point(i);
      ASSERT_NE(bp, nullptr);

      const maliput::api::LaneEndSet* a_lane_ends = bp->GetASide();
      const maliput::api::LaneEndSet* b_lane_ends = bp->GetBSide();

      // Evaluate transitions
      for (int i_a = 0; i_a < a_lane_ends->size(); i_a++) {
        const auto& a_lane_end = a_lane_ends->get(i_a);
        for (int i_b = 0; i_b < b_lane_ends->size(); i_b++) {
          const auto& b_lane_end = b_lane_ends->get(i_b);
          EXPECT_NE(a_lane_end.lane, b_lane_end.lane);
        }
      }
    }
  }

  // Lists different lane lines.
  enum LaneLine {
    kCenterLine,  //< It is the centerline of the lane.
    kLeftLine,    //< It is the left border of the lane.
    kRightLine,   //< It is the right border of the lane.
  };

  // Computes the r coordinate for a the `lane` `lane_line` at `s` position.
  //
  // `lane` must not be nullptr.
  // `s` must be non negative.
  double GetRCoordinateFrom(const maliput::api::Lane* lane, double s, LaneLine lane_line) {
    MALIDRIVE_DEMAND(lane != nullptr);
    MALIDRIVE_DEMAND(s >= 0.);

    double r{0.};

    switch (lane_line) {
      case LaneLine::kCenterLine: {
        r = 0.;
        break;
      }
      case LaneLine::kLeftLine: {
        const maliput::api::RBounds bounds = lane->lane_bounds(s);
        r = bounds.max();
        break;
      }
      case LaneLine::kRightLine: {
        const maliput::api::RBounds bounds = lane->lane_bounds(s);
        r = bounds.min();
        break;
      }
      // clang-format off
      default: { MALIDRIVE_ABORT_MSG("Invalid lane_line value."); }
        // clang-format on
    }
    return r;
  }

  // Evaluates the step distance on a custom `lane_line` within `lane`.
  //
  // `lane` must not be nullptr.
  void RunStepDistanceTestAt(const maliput::api::Lane* lane, LaneLine lane_line) {
    MALIDRIVE_DEMAND(lane != nullptr);

    const double length = lane->length();
    const double kSStep{linear_tolerance_};
    const double kZ{0.};
    double s0{0.};

    while ((s0 + kSStep) <= length) {
      const maliput::api::LanePosition lane_pos_0{s0, GetRCoordinateFrom(lane, s0, lane_line), kZ};

      const double s1 = std::min(s0 + kSStep, length);
      const maliput::api::LanePosition lane_pos_1{s1, GetRCoordinateFrom(lane, s1, lane_line), kZ};

      const double lane_frame_d = (lane_pos_0.srh() - lane_pos_1.srh()).norm();

      const maliput::api::InertialPosition inertial_pos_0 = lane->ToInertialPosition(lane_pos_0);
      const maliput::api::InertialPosition inertial_pos_1 = lane->ToInertialPosition(lane_pos_1);

      const double world_frame_d = (inertial_pos_0.xyz() - inertial_pos_1.xyz()).norm();

      // Lane Frame is not isotropic, but we can consider the worst case
      // scenario to compare RoadGeometry linear tolerance. Computed distance in
      // Lane Frame generally falls short as we are not correctly considering
      // path length along the curve that moves lane_pos_0 to lane_pos_1. It
      // is just a raw linear approximation which is good enough for the
      // magnitude orders.
      // Once LanePositions are converted to World Frame positions, each
      // conversion ideally has an uncertainty sphere (uniformly distributed
      // error) with radius linear_tolerance_. The maximum distance between those
      // two coordinates in World Frame happens when vectors are aligned and
      // their norms add.
      EXPECT_LE(world_frame_d, lane_frame_d + 2. * linear_tolerance_)
          << " on lane " << lane->id().string() << " at lane_pos: " << lane_pos_0 << std::endl;

      s0 += kSStep;
      if (s0 >= length) {
        break;
      }
    }
  }

  // Let A and B be two points a Lane Frame on the Lane's centerline and their
  // distance is linear tolerance. A and B are converted to the World Frame and
  // the distance between those two image points is computed and compared
  // against two linear tolerances.
  void RunStepDistanceTest() {
    const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*> lane_map =
        rn_->road_geometry()->ById().GetLanes();

    for (const auto lane_id_lane : lane_map) {
      const maliput::api::Lane* lane = lane_id_lane.second;
      ASSERT_NE(lane, nullptr);

      RunStepDistanceTestAt(lane, LaneLine::kCenterLine);
      RunStepDistanceTestAt(lane, LaneLine::kLeftLine);
      RunStepDistanceTestAt(lane, LaneLine::kRightLine);
    }
  }

  // Creates an OBJ from the XODR map.
  //
  // Evaluates that <xodr_file_name>.obj and <xodr_file_name>.mtl files are
  // created after a successful generation. Those two files are registered for
  // later clean up.
  void RunObjCreationTest() {
    // Constants for building.
    //@{
    constexpr const double kMaxGridUnit{1.};         // [m]
    constexpr const double kMinGridResolution{0.5};  // [m]
    constexpr const double kDrawElevationBounds{false};
    constexpr bool kDrawArrows{false};
    //@}
    maliput::utility::ObjFeatures features;
    features.max_grid_unit = kMaxGridUnit;
    features.min_grid_resolution = kMinGridResolution;
    features.draw_elevation_bounds = kDrawElevationBounds;
    features.draw_arrows = kDrawArrows;

    auto rg_config = GetRoadGeometryConfigurationFor(utility::GetFileNameFromPath(xodr_file_path_));
    std::cerr << utility::GetFileNameFromPath(xodr_file_path_) << std::endl;
    ASSERT_TRUE(rg_config.has_value());
    // Gets linear tolerance and scale length for the map.
    linear_tolerance_ = rg_config->tolerances.linear_tolerance.value();
    scale_length_ = rg_config->scale_length;
    // Sets the full xodr map file path.
    rg_config->opendrive_file = xodr_file_path_;
    // Lane building process is set to parallel.
    rg_config->build_policy.type = builder::BuildPolicy::Type::kParallel;
    rg_config->build_policy.num_threads = std::nullopt;
    rn_ = loader::Load<builder::RoadNetworkBuilder>(rg_config->ToStringMap());
    ASSERT_NE(rn_, nullptr);
    ASSERT_NE(rn_->road_geometry(), nullptr);
    ASSERT_NO_THROW({
      maliput::utility::GenerateObjFile(rn_->road_geometry(), directory_.get_path(),
                                        utility::GetFileNameFromPath(xodr_file_path_), features);
    });

    const std::string kXodrFileName{utility::GetFileNameFromPath(xodr_file_path_)};
    maliput::common::Path actual_obj_path(directory_);
    actual_obj_path.append(kXodrFileName + ".obj");
    EXPECT_TRUE(actual_obj_path.is_file());

    maliput::common::Path actual_mtl_path(directory_);
    actual_mtl_path.append(kXodrFileName + ".mtl");
    EXPECT_TRUE(actual_mtl_path.is_file());

    paths_to_cleanup_.push_back(actual_obj_path);
    paths_to_cleanup_.push_back(actual_mtl_path);
  }

  std::unique_ptr<const maliput::api::RoadNetwork> rn_{};
  std::string xodr_file_path_;
  maliput::common::Path directory_;
  std::vector<maliput::common::Path> paths_to_cleanup_;
  double linear_tolerance_{};
  double scale_length_{};
};

TEST_F(MalidriveExtensiveQueriesTest, QueriesTest) {
  auto rg_config = GetRoadGeometryConfigurationFor(utility::GetFileNameFromPath(xodr_file_path_));
  ASSERT_TRUE(rg_config.has_value());
  // Gets linear tolerance and scale length for the map.
  linear_tolerance_ = rg_config->tolerances.linear_tolerance.value();
  scale_length_ = rg_config->scale_length;
  // Sets the full xodr map file path.
  rg_config->opendrive_file = xodr_file_path_;
  rn_ = loader::Load<builder::RoadNetworkBuilder>(rg_config->ToStringMap());
  ASSERT_NE(rn_, nullptr);

  RunLaneBoundsTest();
  RunBranchPointTransitionTest();
  RunStepDistanceTest();
}

TEST_F(MalidriveExtensiveQueriesTest, MeshGenerationTest) {
  // Generates a directory to save generated OBJ files.
  directory_.set_as_temp();
  directory_.append("GenerateObjTest_" + utility::GetFileNameFromPath(xodr_file_path_));
  ASSERT_TRUE(maliput::common::Filesystem::create_directory(directory_));

  RunObjCreationTest();
}

}  // namespace test
}  // namespace malidrive

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
