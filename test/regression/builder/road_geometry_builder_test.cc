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
#include "maliput_malidrive/builder/road_geometry_builder.h"

#include <algorithm>
#include <map>

#include <gtest/gtest.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/test_utilities/road_geometry_configuration_for_xodrs.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

using maliput::api::JunctionId;
using maliput::api::LaneEnd;
using maliput::api::LaneId;
using maliput::api::RBounds;
using maliput::api::SegmentId;
using maliput::api::test::IsRBoundsClose;

using malidrive::test::GetRoadGeometryConfigurationFor;

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

/*
  Loaded map has the following structure:

                    (0.,0.,0.)         (100.,0.,0.)
  Driving   | L: 1  |-------->---------| Width: 2.0m
  Track lane| L: 0  |========>=========| Width: 0m
  Driving   | L: -1 |-------->---------| Width: 2.0m
                    Road 1
                    Section 0
*/
class BuilderTestSingleLane : public ::testing::Test {
 protected:
  static constexpr double kLinearTolerance{constants::kStrictLinearTolerance};
  static constexpr double kAngularTolerance{constants::kStrictAngularTolerance};
  static constexpr double kScaleLength{constants::kScaleLength};

  void SetUp() override {
    road_geometry_configuration_.tolerances.linear_tolerance = kLinearTolerance;
    road_geometry_configuration_.tolerances.max_linear_tolerance = std::nullopt;
    road_geometry_configuration_.tolerances.angular_tolerance = kAngularTolerance;
    manager_ = xodr::LoadDataBaseFromFile(
        utility::FindResourceInPath(road_geometry_configuration_.opendrive_file, kMalidriveResourceFolder),
        {kLinearTolerance});
  }

  RoadGeometryConfiguration road_geometry_configuration_{GetRoadGeometryConfigurationFor("SingleLane.xodr").value()};
  std::unique_ptr<xodr::DBManager> manager_;
};

TEST_F(BuilderTestSingleLane, RoadGeometryBuilderConstructor) {
  std::unique_ptr<const maliput::api::RoadGeometry> dut =
      builder::RoadGeometryBuilder(std::move(manager_), road_geometry_configuration_)();
  ASSERT_NE(dut.get(), nullptr);

  ASSERT_NE(dynamic_cast<const malidrive::RoadGeometry*>(dut.get()), nullptr);

  EXPECT_DOUBLE_EQ(dut->linear_tolerance(), kLinearTolerance);
  EXPECT_DOUBLE_EQ(dut->angular_tolerance(), kAngularTolerance);
  EXPECT_DOUBLE_EQ(dut->scale_length(), kScaleLength);
  EXPECT_EQ(dut->id(), road_geometry_configuration_.id);
}

TEST_F(BuilderTestSingleLane, RoadGeometryBuilderConstructorBadUsed) {
  {
    RoadGeometryConfiguration bad_config = road_geometry_configuration_;
    bad_config.tolerances.linear_tolerance = -5.;
    EXPECT_THROW(builder::RoadGeometryBuilder(std::move(manager_), bad_config), maliput::common::assertion_error);
  }
  {
    RoadGeometryConfiguration bad_config = road_geometry_configuration_;
    bad_config.tolerances.angular_tolerance = -5.;
    EXPECT_THROW(builder::RoadGeometryBuilder(std::move(manager_), bad_config), maliput::common::assertion_error);
  }
  {
    RoadGeometryConfiguration bad_config = road_geometry_configuration_;
    bad_config.scale_length = -5.;
    EXPECT_THROW(builder::RoadGeometryBuilder(std::move(manager_), bad_config), maliput::common::assertion_error);
  }
  {
    EXPECT_THROW(builder::RoadGeometryBuilder(nullptr, road_geometry_configuration_), maliput::common::assertion_error);
  }
}

// Holds attributes for the Road.
struct RoadAttributes {
  int road_id{};
  double heading{};
};

// Holds attributes for the Segment/LaneSection.
struct SegmentAttributes {
  SegmentId segment_id;
  double track_s_start{};
  double track_s_end{};
};

// Holds attributes for the Lane.
struct LaneAttributes {
  LaneAttributes() = delete;
  // Creates LaneAttributes.
  LaneAttributes(int xodr_road_id_in, int xodr_lane_section_id_in, int xodr_lane_id_in)
      : xodr_road_id(xodr_road_id_in), xodr_lane_section_id(xodr_lane_section_id_in), xodr_lane_id(xodr_lane_id_in) {
    lane_id = GetLaneId(xodr_road_id_in, xodr_lane_section_id, xodr_lane_id);
  }
  int xodr_road_id{};
  int xodr_lane_section_id{};
  int xodr_lane_id{};
  LaneId lane_id{"None"};
};

// Holds the parameters to be checked.
struct RoadGeometryBuilderTestParameters {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadGeometryBuilderTestParameters);

  RoadGeometryBuilderTestParameters() = default;

  RoadGeometryBuilderTestParameters(
      const std::string& road_geometry_id_in, const std::string& path_to_xodr_file_in,
      const std::vector<RoadAttributes>& roads_in,
      const std::map<JunctionId, std::vector<std::pair<SegmentAttributes, std::vector<LaneAttributes>>>>&
          junctions_segments_lanes_ids_in)
      : road_geometry_id(road_geometry_id_in),
        path_to_xodr_file(path_to_xodr_file_in),
        roads(roads_in),
        junctions_segments_lanes_ids(junctions_segments_lanes_ids_in) {}

  std::string road_geometry_id{};
  std::string path_to_xodr_file{};
  std::string build_policy{};
  std::vector<RoadAttributes> roads{};
  std::map<JunctionId, std::vector<std::pair<SegmentAttributes, std::vector<LaneAttributes>>>>
      junctions_segments_lanes_ids;
};

// Returns a vector of test parameters for the Builder test.
std::vector<RoadGeometryBuilderTestParameters> InstantiateBuilderParameters() {
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
      {"SingleLane",
       "SingleLane.xodr",
       {{1 /* road_id */, 0.0 /* heading */}},
       {{JunctionId("1_0"), {{{SegmentId("1_0"), 0, 100.}, {{1, 0, -1}, {1, 0, 1}}}}}}},
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
      {"ArcLane",
       "ArcLane.xodr",
       {{1 /* road_id */, 0.0 /* heading */}},
       {{JunctionId("1_0"), {{{SegmentId("1_0"), 0, 100.}, {{1, 0, -1}, {1, 0, 1}}}}}}},
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
      {"SShapeRoad",
       "SShapeRoad.xodr",
       {{1 /* road_id */, 0.0 /* heading */}},
       {{JunctionId("1_0"), {{{SegmentId("1_0"), 0, 271.327412287}, {{1, 0, -1}, {1, 0, 1}}}}}}},
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
      {"LShapeRoad",
       "LShapeRoad.xodr",
       {{1 /* road_id */, 0.0 /* heading */},
        {2 /* road_id */, 0.0 /* heading */},
        {3 /* road_id */, 1.570796327 /* heading */}},
       {{JunctionId("1_0"), {{{SegmentId("1_0"), 0, 100.}, {{1, 0, 1}}}}},
        {JunctionId("2_0"), {{{SegmentId("2_0"), 0, 62.831853072}, {{2, 0, 1}}}}},
        {JunctionId("3_0"), {{{SegmentId("3_0"), 0, 100.}, {{3, 0, 1}}}}}}},
      /*
        TShapeRoad map has the following structure:
          - 3 Roads that don't belong to a Junction.
          - 6 Roads that belong to a Junction.
          - 1 Lane Section per Road.
          - 3 Lanes(Only counting center and driving lanes.): {-1 <Right, Driving>, 0 <Center, Track>, 1 <Left,
        Driving>}
                - Width: 3.5m each, except the center lane.
      */
      {"TShapeRoad",
       "TShapeRoad.xodr",
       {{0 /* road_id */, 0.0 /* heading */},
        {1 /* road_id */, 0.0 /* heading */},
        {2 /* road_id */, 1.5707963267948966 /* heading */},
        {4 /* road_id */, 0.0 /* heading */},
        {5 /* road_id */, 0.0 /* heading */},
        {6 /* road_id */, 3.1415926535897931 /* heading */},
        {7 /* road_id */, 1.5707963267948966 /* heading */},
        {8 /* road_id */, 1.5707963267948966 /* heading */},
        {9 /* road_id */, 0.0 /* heading */}},
       {
           {JunctionId("0_0"),
            {{{SegmentId("0_0"), 0, 46.},
              {/* left { */ {0, 0, 4},
               {0, 0, 3},
               {0, 0, 2},
               {0, 0, 1} /* } left */,
               /* right { */ {0, 0, -1},
               {0, 0, -2},
               {0, 0, -3},
               {0, 0, -4} /* } right */}}}},
           {JunctionId("1_0"),
            {{{SegmentId("1_0"), 0, 46.},
              {/* left { */ {1, 0, 4},
               {1, 0, 3},
               {1, 0, 2},
               {1, 0, 1} /* } left */,
               /* right { */ {1, 0, -1},
               {1, 0, -2},
               {1, 0, -3},
               {1, 0, -4} /* } right */}}}},
           {JunctionId("2_0"),
            {{{SegmentId("2_0"), 0, 46.},
              {/* left { */ {2, 0, 4},
               {2, 0, 3},
               {2, 0, 2},
               {2, 0, 1} /* } left */,
               /* right { */ {2, 0, -1},
               {2, 0, -2},
               {2, 0, -3},
               {2, 0, -4} /* } right */}}}},
           {JunctionId("3"),
            {{{SegmentId("4_0"), 0, 8.}, {{4, 0, 1}}},
             {{SegmentId("5_0"), 0, 8.}, {{5, 0, -1}}},
             {{SegmentId("6_0"), 0, 6.31248635281257}, {{6, 0, -1}}},
             {{SegmentId("7_0"), 0, 6.312486352812575}, {{7, 0, -1}}},
             {{SegmentId("8_0"), 0, 6.3124863528125745}, {{8, 0, -1}}},
             {{SegmentId("9_0"), 0, 6.312486352812572}, {{9, 0, -1}}}}},
       }},
      /*
        LineMultpleSections map has the following structure:
          - 1 Road
            - 3 Lane Section.
              - 2 Driveable Lanes: {-1 <right, Driving>, 0 <Center, Track>, 1 <Left, Driving>}
                - Width: 2m each, except the center lane.
            - 3 geometries(1 per road):
              - Line: start: [x: 0., y: 0., h: 0.], length: 33.3m
              - Line: start: [x: 33.3, y: 0., h: 0.], length: 33.3m
              - Line: start: [x: 66.6, y: 0., h: 0], length: 33.4m
      */
      {"LineMultipleSections",
       "LineMultipleSections.xodr",
       {{1 /* road_id */, 0.0 /* heading */}},
       {{JunctionId("1_0"),
         {{{SegmentId("1_0"), 0, 33.3},
           {/* left { */ {1, 0, 4},
            {1, 0, 3},
            {1, 0, 2},
            {1, 0, 1} /* } left */,
            /* right { */ {1, 0, -1},
            {1, 0, -2},
            {1, 0, -3},
            {1, 0, -4} /* } right */}}}},
        {JunctionId("1_1"),
         {{{SegmentId("1_1"), 33.3, 66.6},
           {/* left { */ {1, 1, 4},
            {1, 1, 3},
            {1, 1, 2},
            {1, 1, 1} /* } left */,
            /* right { */ {1, 1, -1},
            {1, 1, -2},
            {1, 1, -3},
            {1, 1, -4} /* } right */}}}},
        {JunctionId("1_2"),
         {{{SegmentId("1_2"), 66.6, 100.},
           {/* left { */ {1, 2, 4},
            {1, 2, 3},
            {1, 2, 2},
            {1, 2, 1} /* } left */,
            /* right { */ {1, 2, -1},
            {1, 2, -2},
            {1, 2, -3},
            {1, 2, -4} /* } right */}}}}}},
      /*
        DisconnectedRoadInJunction map has the following structure:
          - Road 0 - no Junction, connects to Road 1
            - 1 Lane Section.
              - 1 Driveable Lanes: {-1 <left, Driving>, 0 <Center, Track>}
                - Width: 3.5m each, except the center lane.
            - 1 geometry:  Line: start: [x: 0., y: 0., h: 0.], length: 50.0m
          - Road 1 - Junction 2, connects to Road 1
            - 1 Lane Section.
              - 1 Driveable Lanes: {-1 <left, Driving>, 0 <Center, Track>}
                - Width: 3.5m each, except the center lane.
            - 1 geometry:  Line: start: [x: 0., y: 0., h: 0.], length: 50.0m
          - Junction 2
            - 1 connection: Road0:L1 -> Road1:L1.

        This road shows how a road inside a Junction might not be connected
        to anything in one of its endpoints.
      */
      {"DisconnectedRoadInJunction",
       "DisconnectedRoadInJunction.xodr",
       {{0 /* road_id */, 0.0 /* heading */}, {1 /* road_id */, 0.0 /* heading */}},
       {{JunctionId("0_0"), {{{SegmentId("0_0"), 0., 50.}, {{0, 0, 1}}}}},
        {JunctionId("2"), {{{SegmentId("1_0"), 0., 50.}, {{1, 0, 1}}}}}}},
  };
}

// Builder test that evaluates Junction, Segment and Lane construction.
class RoadGeometryBuilderBaseTest : public ::testing::TestWithParam<RoadGeometryBuilderTestParameters> {
 protected:
  void SetUp() override {
    road_geometry_configuration_.tolerances.linear_tolerance = kLinearTolerance;
    road_geometry_configuration_.tolerances.max_linear_tolerance = std::nullopt;
    road_geometry_configuration_.tolerances.angular_tolerance = kAngularTolerance;
    manager_ = xodr::LoadDataBaseFromFile(
        utility::FindResourceInPath(road_geometry_configuration_.opendrive_file, kMalidriveResourceFolder),
        {kLinearTolerance});
  }

  // Tests Junction, Segments and Lanes properties.
  void RunTest() {
    std::unique_ptr<const maliput::api::RoadGeometry> dut =
        builder::RoadGeometryBuilder(std::move(manager_), road_geometry_configuration_)();
    ASSERT_NE(dut.get(), nullptr);

    // Junctions.
    const int num_junctions = static_cast<int>(expected_junctions_segments_lanes_ids.size());
    EXPECT_EQ(num_junctions, dut->num_junctions());

    for (int i = 0; i < num_junctions; ++i) {
      const maliput::api::Junction* junction = dut->junction(i);
      ASSERT_NE(junction, nullptr);
      EXPECT_EQ(junction->road_geometry(), dut.get());
      EXPECT_NE(expected_junctions_segments_lanes_ids.find(junction->id()),
                expected_junctions_segments_lanes_ids.end());

      const int num_segments = static_cast<int>(expected_junctions_segments_lanes_ids.at(junction->id()).size());
      EXPECT_EQ(junction->num_segments(), num_segments);

      // Segments.
      for (int j = 0; j < num_segments; j++) {
        const maliput::api::Segment* segment = junction->segment(j);
        ASSERT_NE(segment, nullptr);
        EXPECT_EQ(junction, segment->junction());

        const auto& expected_segments = expected_junctions_segments_lanes_ids.at(junction->id());
        const auto expected_segment_it = std::find_if(
            expected_segments.begin(), expected_segments.end(),
            [segment](const auto expected_segment) { return expected_segment.first.segment_id == segment->id(); });
        EXPECT_NE(expected_segment_it, expected_segments.end());

        const int num_lanes = static_cast<int>(expected_segment_it->second.size());
        EXPECT_EQ(num_lanes, segment->num_lanes());

        // Lanes.
        for (int k = 0; k < num_lanes; k++) {
          const auto& expected_lanes = expected_segment_it->second;
          const maliput::api::Lane* lane = segment->lane(k);
          ASSERT_NE(lane, nullptr);
          EXPECT_EQ(segment, lane->segment());

          const auto expected_lane_it =
              std::find_if(expected_lanes.begin(), expected_lanes.end(),
                           [lane](const auto expected_lane) { return expected_lane.lane_id == lane->id(); });
          EXPECT_NE(expected_lane_it, expected_lanes.end());

          if (k == 0) {
            EXPECT_EQ(nullptr, lane->to_right());
          } else {
            EXPECT_EQ(segment->lane(k - 1), lane->to_right());
          }
          if (k == (num_lanes - 1)) {
            EXPECT_EQ(nullptr, lane->to_left());
          } else {
            EXPECT_EQ(segment->lane(k + 1), lane->to_left());
          }

          const auto expected_road_it = std::find_if(expected_roads.begin(), expected_roads.end(),
                                                     [expected_lane_it](const RoadAttributes& road_attributes) {
                                                       return road_attributes.road_id == expected_lane_it->xodr_road_id;
                                                     });
          EXPECT_NE(expected_road_it, expected_roads.end());
          EXPECT_NEAR(expected_road_it->heading, lane->GetOrientation({0., 0., 0.}).yaw(), kLinearTolerance);

          auto malidrive_lane = dynamic_cast<const Lane*>(lane);
          ASSERT_NE(malidrive_lane, nullptr);

          EXPECT_EQ(expected_lane_it->xodr_road_id, malidrive_lane->get_track());
          EXPECT_EQ(expected_lane_it->xodr_lane_id, malidrive_lane->get_lane_id());
          //@{  These two checks cannot be EXPECT_DOUBLE_EQ() because the builder
          //    manipulates the parameter values.
          EXPECT_NEAR(expected_segment_it->first.track_s_start, malidrive_lane->get_track_s_start(), kLinearTolerance);
          EXPECT_NEAR(expected_segment_it->first.track_s_end, malidrive_lane->get_track_s_end(), kLinearTolerance);
          //@}
        }
      }
    }
  }

 public:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{constants::kScaleLength};
  //@}
  const std::vector<RoadAttributes> expected_roads = GetParam().roads;
  const std::map<JunctionId, std::vector<std::pair<SegmentAttributes, std::vector<LaneAttributes>>>>
      expected_junctions_segments_lanes_ids = GetParam().junctions_segments_lanes_ids;
  RoadGeometryConfiguration road_geometry_configuration_{
      GetRoadGeometryConfigurationFor(GetParam().path_to_xodr_file).value()};
  std::unique_ptr<xodr::DBManager> manager_;
};

// @{ Runs the Builder with a sequential lane building process.
class RoadGeometryBuilderSequentialBuildPolicyTest : public RoadGeometryBuilderBaseTest {
 protected:
  void SetUp() override {
    RoadGeometryBuilderBaseTest::SetUp();
    road_geometry_configuration_.build_policy.type = BuildPolicy::Type::kSequential;
  }
};

TEST_P(RoadGeometryBuilderSequentialBuildPolicyTest, JunctionSegmentLaneTest) { RunTest(); }

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderSequentialBuildPolicyTestGroup, RoadGeometryBuilderSequentialBuildPolicyTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));
// @}

// @{ Runs the Builder with a parallel lane building process using an automatic selection of the number of threads.
class RoadGeometryBuilderParallelBuildPolicyAutomaticThreadsTest : public RoadGeometryBuilderBaseTest {
 protected:
  void SetUp() override {
    RoadGeometryBuilderBaseTest::SetUp();
    road_geometry_configuration_.build_policy.type = BuildPolicy::Type::kParallel;
    // Number of threads will be delimited by the hardware capacity minus one.
    road_geometry_configuration_.build_policy.num_threads = std::nullopt;
  }
};

TEST_P(RoadGeometryBuilderParallelBuildPolicyAutomaticThreadsTest, JunctionSegmentLaneTest) { RunTest(); }

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderParallelBuildPolicyAutomaticThreadsTestGroup,
                        RoadGeometryBuilderParallelBuildPolicyAutomaticThreadsTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));
// @}

// @{ Runs the Builder with a parallel lane building process using a manual selection of the number of threads.
class RoadGeometryBuilderParallelBuildPolicyManualThreadsTest : public RoadGeometryBuilderBaseTest {
 protected:
  void SetUp() override {
    RoadGeometryBuilderBaseTest::SetUp();
    road_geometry_configuration_.build_policy.type = BuildPolicy::Type::kParallel;
    road_geometry_configuration_.build_policy.num_threads = kNumThreads;
  }
  static constexpr int kNumThreads{10};
};

TEST_P(RoadGeometryBuilderParallelBuildPolicyManualThreadsTest, JunctionSegmentLaneTest) { RunTest(); }

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderParallelBuildPolicyManualThreadsTestGroup,
                        RoadGeometryBuilderParallelBuildPolicyManualThreadsTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));
// @}

// @{ Runs the Builder with a single linear tolerance option.
class RoadGeometryBuilderSingleLinearToleranceTest : public RoadGeometryBuilderBaseTest {
 protected:
  void SetUp() override {
    RoadGeometryBuilderBaseTest::SetUp();
    road_geometry_configuration_.tolerances.max_linear_tolerance = std::nullopt;
  }
};

TEST_P(RoadGeometryBuilderSingleLinearToleranceTest, JunctionSegmentLaneTest) { RunTest(); }

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderSingleLinearToleranceTestGroup, RoadGeometryBuilderSingleLinearToleranceTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));
// @}

// @{ Runs the Builder with an automatic tolerance selection.
class RoadGeometryBuilderLinearToleranceRangeTest : public RoadGeometryBuilderBaseTest {
 protected:
  void SetUp() override {
    RoadGeometryBuilderBaseTest::SetUp();
    // By default the RoadGeometryBuilderBaseTest::SetUp() is using GetRoadGeometryConfigurationFor() to get the
    // RoadGeometryConfigurations which already define a max_linear_tolerance.
    // However a new max linear tolerance is defined just for the purpose of the test.
    road_geometry_configuration_.tolerances.max_linear_tolerance =
        road_geometry_configuration_.tolerances.linear_tolerance.value() * 1.2;
  }
};

// Tests that the RoadGeometry is constructed.
TEST_P(RoadGeometryBuilderLinearToleranceRangeTest, BuildProcessTest) { RunTest(); }

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderLinearToleranceRangeTestGroup, RoadGeometryBuilderLinearToleranceRangeTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));
// @}

// @{ Runs the Builder with the `omit_nondrivable_lanes` flag enabled.
class RoadGeometryBuilderOmitNonDrivableLanesPolicyTest : public RoadGeometryBuilderBaseTest {
 protected:
  void SetUp() override {
    RoadGeometryBuilderBaseTest::SetUp();
    road_geometry_configuration_.omit_nondrivable_lanes = true;
  }
};

// Returns a vector of test parameters for the Builder test.
// Only driveable lanes are expected to be built.
std::vector<RoadGeometryBuilderTestParameters> InstantiateBuilderNonDrivableLanesParameters() {
  return {
      /*
        TShapeRoad map has the following structure:
          - 3 Roads that don't belong to a Junction.
          - 6 Roads that belong to a Junction.
          - 1 Lane Section per Road.
          - 3 Lanes(Only counting center and driving lanes.): {-1 <Right, Driving>, 0 <Center, Track>, 1 <Left,
        Driving>}
                - Width: 3.5m each, except the center lane.
      */
      {"TShapeRoad",
       "TShapeRoad.xodr",
       {{0 /* road_id */, 0.0 /* heading */},
        {1 /* road_id */, 0.0 /* heading */},
        {2 /* road_id */, 1.5707963267948966 /* heading */},
        {4 /* road_id */, 0.0 /* heading */},
        {5 /* road_id */, 0.0 /* heading */},
        {6 /* road_id */, 3.1415926535897931 /* heading */},
        {7 /* road_id */, 1.5707963267948966 /* heading */},
        {8 /* road_id */, 1.5707963267948966 /* heading */},
        {9 /* road_id */, 0.0 /* heading */}},
       {
           {JunctionId("0_0"),
            {{{SegmentId("0_0"), 0, 46.},
              {/* left { */ {0, 0, 1} /* } left */,
               /* right { */ {0, 0, -1} /* } right */}}}},
           {JunctionId("1_0"),
            {{{SegmentId("1_0"), 0, 46.},
              {/* left { */ {1, 0, 1} /* } left */,
               /* right { */ {1, 0, -1} /* } right */}}}},
           {JunctionId("2_0"),
            {{{SegmentId("2_0"), 0, 46.},
              {/* left { */ {2, 0, 1} /* } left */,
               /* right { */ {2, 0, -1} /* } right */}}}},
           {JunctionId("3"),
            {{{SegmentId("4_0"), 0, 8.}, {{4, 0, 1}}},
             {{SegmentId("5_0"), 0, 8.}, {{5, 0, -1}}},
             {{SegmentId("6_0"), 0, 6.31248635281257}, {{6, 0, -1}}},
             {{SegmentId("7_0"), 0, 6.312486352812575}, {{7, 0, -1}}},
             {{SegmentId("8_0"), 0, 6.3124863528125745}, {{8, 0, -1}}},
             {{SegmentId("9_0"), 0, 6.312486352812572}, {{9, 0, -1}}}}},
       }},
      /*
        LineMultpleSections map has the following structure:
          - 1 Road
            - 3 Lane Section.
              - 2 Driveable Lanes: {-1 <right, Driving>, 0 <Center, Track>, 1 <Left, Driving>}
                - Width: 2m each, except the center lane.
            - 3 geometries(1 per road):
              - Line: start: [x: 0., y: 0., h: 0.], length: 33.3m
              - Line: start: [x: 33.3, y: 0., h: 0.], length: 33.3m
              - Line: start: [x: 66.6, y: 0., h: 0], length: 33.4m
      */
      {"LineMultipleSections",
       "LineMultipleSections.xodr",
       {{1 /* road_id */, 0.0 /* heading */}},
       {{JunctionId("1_0"),
         {{{SegmentId("1_0"), 0, 33.3},
           {/* left { */ {1, 0, 1} /* } left */,
            /* right { */ {1, 0, -1} /* } right */}}}},
        {JunctionId("1_1"),
         {{{SegmentId("1_1"), 33.3, 66.6},
           {/* left { */ {1, 1, 1} /* } left */,
            /* right { */ {1, 1, -1} /* } right */}}}},
        {JunctionId("1_2"),
         {{{SegmentId("1_2"), 66.6, 100.},
           {/* left { */ {1, 2, 1} /* } left */,
            /* right { */ {1, 2, -1} /* } right */}}}}}},
  };
}

TEST_P(RoadGeometryBuilderOmitNonDrivableLanesPolicyTest, JunctionSegmentLaneTest) { RunTest(); }

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderOmitNonDrivableLanesPolicyTestGroup,
                        RoadGeometryBuilderOmitNonDrivableLanesPolicyTest,
                        ::testing::ValuesIn(InstantiateBuilderNonDrivableLanesParameters()));
// @}

// Alternative to BranchPoint that instead of using Lane pointers, uses LaneIds
// which are easy to define from the test description point of view.
struct ConnectionExpectation {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConnectionExpectation)

  ConnectionExpectation() = default;

  // Constructs the expectation based on two collections of LaneIds and
  // LaneEnd::Which pairs.
  //
  // @param x_side_in maps to the A/B side of the BranchPoint to evaluate.
  // @param y_side_in maps to the B/A side of the BranchPoint to evaluate.
  ConnectionExpectation(const std::vector<std::pair<LaneId, LaneEnd::Which>>& x_side_in,
                        const std::vector<std::pair<LaneId, LaneEnd::Which>>& y_side_in)
      : x_side(x_side_in), y_side(y_side_in) {}

  std::vector<std::pair<LaneId, LaneEnd::Which>> x_side{};
  std::vector<std::pair<LaneId, LaneEnd::Which>> y_side{};
};

// Compare functor between a pair <LaneId, LaneEnd::Which> and a LaneEnd.
// It is convenient for std::find_if().
struct CompareConnectionExpectation {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompareConnectionExpectation)

  CompareConnectionExpectation() = delete;

  explicit CompareConnectionExpectation(const LaneEnd _lane_end) : lane_end(_lane_end) {}

  bool operator()(const std::pair<LaneId, LaneEnd::Which>& lane_id_and_end) {
    return (lane_end.lane->id() == lane_id_and_end.first) && (lane_end.end == lane_id_and_end.second);
  }

  LaneEnd lane_end;
};

// Holds the parameters for a BranchPoint test.
struct BuilderBranchPointTestParameters {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BuilderBranchPointTestParameters)

  BuilderBranchPointTestParameters() = default;

  // Constructs the parameters with `_road_id`, `_path_to_xodr_file` and
  // `_expected_connections`.
  BuilderBranchPointTestParameters(
      const std::string& _road_id, const std::string& _path_to_xodr_file,
      const std::map<LaneId, std::pair<ConnectionExpectation, ConnectionExpectation>> _expected_connections)
      : road_id(_road_id), path_to_xodr_file(_path_to_xodr_file), expected_connections(_expected_connections) {}

  std::string road_id{};
  std::string path_to_xodr_file{};
  std::map<LaneId, std::pair<ConnectionExpectation, ConnectionExpectation>> expected_connections;
};

// Returns a vector of test parameters for the Builder BranchPoint test.
std::vector<BuilderBranchPointTestParameters> InstantiateBuilderBranchPointParameters() {
  return {
      {"LShapeRoad",
       "LShapeRoad.xodr",
       {
           {LaneId("1_0_1"),
            {ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("2_0_1"), LaneEnd::Which::kStart}})}},
           {LaneId("2_0_1"),
            {ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("2_0_1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("2_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("3_0_1"), LaneEnd::Which::kStart}})}},
           {LaneId("3_0_1"),
            {ConnectionExpectation({{LaneId("2_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("3_0_1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("3_0_1"), LaneEnd::Which::kFinish}}, {})}},
       }},
      {"LShapeRoadVariableLanes",
       "LShapeRoadVariableLanes.xodr",
       {
           {LaneId("1_0_1"),
            {ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("2_0_1"), LaneEnd::Which::kStart}})}},
           {LaneId("1_0_2"),
            {ConnectionExpectation({{LaneId("1_0_2"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation({{LaneId("1_0_2"), LaneEnd::Which::kFinish}},
                                   {{LaneId("2_0_2"), LaneEnd::Which::kStart}})}},
           {LaneId("1_0_3"),
            {ConnectionExpectation({{LaneId("1_0_3"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation({{LaneId("1_0_3"), LaneEnd::Which::kFinish}}, {})}},
           {LaneId("2_0_1"),
            {ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("2_0_1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("2_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("3_0_1"), LaneEnd::Which::kStart}})}},
           {LaneId("2_0_2"),
            {ConnectionExpectation({{LaneId("1_0_2"), LaneEnd::Which::kFinish}},
                                   {{LaneId("2_0_2"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("2_0_2"), LaneEnd::Which::kFinish}}, {})}},
           {LaneId("3_0_1"),
            {ConnectionExpectation({{LaneId("2_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("3_0_1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("3_0_1"), LaneEnd::Which::kFinish}}, {})}},
       }},
      {"TShapeRoad",
       "TShapeRoad.xodr",
       {
           {LaneId("0_0_1"),
            {ConnectionExpectation({{LaneId("0_0_1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation(
                 {{LaneId("0_0_1"), LaneEnd::Which::kFinish}},
                 {{LaneId("4_0_1"), LaneEnd::Which::kStart}, {LaneId("8_0_-1"), LaneEnd::Which::kFinish}})}},
           {LaneId("0_0_-1"),
            {ConnectionExpectation({{LaneId("0_0_-1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation(
                 {{LaneId("0_0_-1"), LaneEnd::Which::kFinish}},
                 {{LaneId("5_0_-1"), LaneEnd::Which::kStart}, {LaneId("9_0_-1"), LaneEnd::Which::kStart}})}},

           {LaneId("1_0_1"),
            {ConnectionExpectation(
                 {{LaneId("1_0_1"), LaneEnd::Which::kStart}},
                 {{LaneId("4_0_1"), LaneEnd::Which::kFinish}, {LaneId("6_0_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kFinish}}, {})}},
           {LaneId("1_0_-1"),
            {ConnectionExpectation(
                 {{LaneId("1_0_-1"), LaneEnd::Which::kStart}},
                 {{LaneId("5_0_-1"), LaneEnd::Which::kFinish}, {LaneId("7_0_-1"), LaneEnd::Which::kFinish}}),
             ConnectionExpectation({{LaneId("1_0_-1"), LaneEnd::Which::kFinish}}, {})}},

           {LaneId("2_0_1"),
            {ConnectionExpectation({{LaneId("2_0_1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation(
                 {{LaneId("2_0_1"), LaneEnd::Which::kFinish}},
                 {{LaneId("6_0_-1"), LaneEnd::Which::kFinish}, {LaneId("9_0_-1"), LaneEnd::Which::kFinish}})}},
           {LaneId("2_0_-1"),
            {ConnectionExpectation({{LaneId("2_0_-1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation(
                 {{LaneId("2_0_-1"), LaneEnd::Which::kFinish}},
                 {{LaneId("7_0_-1"), LaneEnd::Which::kStart}, {LaneId("8_0_-1"), LaneEnd::Which::kStart}})}},

           {LaneId("4_0_1"),
            {ConnectionExpectation(
                 {{LaneId("0_0_1"), LaneEnd::Which::kFinish}},
                 {{LaneId("4_0_1"), LaneEnd::Which::kStart}, {LaneId("8_0_-1"), LaneEnd::Which::kFinish}}),
             ConnectionExpectation(
                 {{LaneId("1_0_1"), LaneEnd::Which::kStart}},
                 {{LaneId("4_0_1"), LaneEnd::Which::kFinish}, {LaneId("6_0_-1"), LaneEnd::Which::kStart}})}},

           {LaneId("5_0_-1"),
            {ConnectionExpectation(
                 {{LaneId("0_0_-1"), LaneEnd::Which::kFinish}},
                 {{LaneId("5_0_-1"), LaneEnd::Which::kStart}, {LaneId("9_0_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation(
                 {{LaneId("1_0_-1"), LaneEnd::Which::kStart}},
                 {{LaneId("5_0_-1"), LaneEnd::Which::kFinish}, {LaneId("7_0_-1"), LaneEnd::Which::kFinish}})}},

           {LaneId("6_0_-1"),
            {ConnectionExpectation(
                 {{LaneId("1_0_1"), LaneEnd::Which::kStart}},
                 {{LaneId("4_0_1"), LaneEnd::Which::kFinish}, {LaneId("6_0_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation(
                 {{LaneId("2_0_1"), LaneEnd::Which::kFinish}},
                 {{LaneId("6_0_-1"), LaneEnd::Which::kFinish}, {LaneId("9_0_-1"), LaneEnd::Which::kFinish}})}},

           {LaneId("7_0_-1"),
            {ConnectionExpectation(
                 {{LaneId("2_0_-1"), LaneEnd::Which::kFinish}},
                 {{LaneId("7_0_-1"), LaneEnd::Which::kStart}, {LaneId("8_0_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation(
                 {{LaneId("1_0_-1"), LaneEnd::Which::kStart}},
                 {{LaneId("5_0_-1"), LaneEnd::Which::kFinish}, {LaneId("7_0_-1"), LaneEnd::Which::kFinish}})}},

           {LaneId("8_0_-1"),
            {ConnectionExpectation(
                 {{LaneId("2_0_-1"), LaneEnd::Which::kFinish}},
                 {{LaneId("7_0_-1"), LaneEnd::Which::kStart}, {LaneId("8_0_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation(
                 {{LaneId("0_0_1"), LaneEnd::Which::kFinish}},
                 {{LaneId("4_0_1"), LaneEnd::Which::kStart}, {LaneId("8_0_-1"), LaneEnd::Which::kFinish}})}},

           {LaneId("9_0_-1"),
            {ConnectionExpectation(
                 {{LaneId("0_0_-1"), LaneEnd::Which::kFinish}},
                 {{LaneId("5_0_-1"), LaneEnd::Which::kStart}, {LaneId("9_0_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation(
                 {{LaneId("2_0_1"), LaneEnd::Which::kFinish}},
                 {{LaneId("6_0_-1"), LaneEnd::Which::kFinish}, {LaneId("9_0_-1"), LaneEnd::Which::kFinish}})}},
       }},
      {"LineMultipleSections",
       "LineMultipleSections.xodr",
       {
           {LaneId("1_0_1"),
            {ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_1_1"), LaneEnd::Which::kStart}})}},

           {LaneId("1_0_-1"),
            {ConnectionExpectation({{LaneId("1_0_-1"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation({{LaneId("1_0_-1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_1_-1"), LaneEnd::Which::kStart}})}},

           {LaneId("1_1_1"),
            {ConnectionExpectation({{LaneId("1_0_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_1_1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("1_1_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_2_1"), LaneEnd::Which::kStart}})}},

           {LaneId("1_1_-1"),
            {ConnectionExpectation({{LaneId("1_0_-1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_1_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("1_1_-1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_2_-1"), LaneEnd::Which::kStart}})}},

           {LaneId("1_2_1"),
            {ConnectionExpectation({{LaneId("1_1_1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_2_1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("1_2_1"), LaneEnd::Which::kFinish}}, {})}},

           {LaneId("1_2_-1"),
            {ConnectionExpectation({{LaneId("1_1_-1"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1_2_-1"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("1_2_-1"), LaneEnd::Which::kFinish}}, {})}},
       }},
  };
}

// Class to setup the RoadGeometry and later test BranchPoint formation.
class BuilderBranchPointTest : public ::testing::TestWithParam<BuilderBranchPointTestParameters> {
 protected:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{constants::kScaleLength};
  //@}

  void SetUp() override {
    road_geometry_configuration_.tolerances.linear_tolerance = kLinearTolerance;
    road_geometry_configuration_.tolerances.max_linear_tolerance = std::nullopt;
    road_geometry_configuration_.tolerances.angular_tolerance = kAngularTolerance;
    auto manager = xodr::LoadDataBaseFromFile(
        utility::FindResourceInPath(road_geometry_configuration_.opendrive_file, kMalidriveResourceFolder),
        {kLinearTolerance});
    rg_ = builder::RoadGeometryBuilder(std::move(manager), road_geometry_configuration_)();
    expected_connections = GetParam().expected_connections;
  }

  RoadGeometryConfiguration road_geometry_configuration_{
      GetRoadGeometryConfigurationFor(GetParam().path_to_xodr_file).value()};
  std::unique_ptr<const maliput::api::RoadGeometry> rg_;
  std::map<LaneId, std::pair<ConnectionExpectation, ConnectionExpectation>> expected_connections;
};

// Tests Lane and BranchPoint formation.
TEST_P(BuilderBranchPointTest, LaneAndBranchPointTest) {
  // Evaluates that LaneEnds on both bp sides are the expected ones.
  auto test_branch_point = [](const maliput::api::BranchPoint* bp,
                              const ConnectionExpectation& connection_expectation) {
    ASSERT_NE(bp->GetASide(), nullptr);
    ASSERT_NE(bp->GetBSide(), nullptr);

    // Evaluates which side belongs to each connection expectation.
    const maliput::api::LaneEndSet* a_side = bp->GetASide();
    const maliput::api::LaneEndSet* b_side = bp->GetBSide();

    ASSERT_TRUE(a_side->size() != 0 || b_side->size() != 0);

    std::pair<const maliput::api::LaneEndSet*, std::vector<std::pair<LaneId, LaneEnd::Which>>> first_side;
    std::pair<const maliput::api::LaneEndSet*, std::vector<std::pair<LaneId, LaneEnd::Which>>> second_side;

    const LaneEnd dut = a_side->size() != 0 ? a_side->get(0) : b_side->get(0);
    const CompareConnectionExpectation cmp(dut);
    if (std::find_if(connection_expectation.x_side.begin(), connection_expectation.x_side.end(), cmp) !=
        connection_expectation.x_side.end()) {
      // a --> x
      first_side.first = a_side;
      first_side.second = connection_expectation.x_side;
      // b --> y
      second_side.first = b_side;
      second_side.second = connection_expectation.y_side;
    } else if (std::find_if(connection_expectation.y_side.begin(), connection_expectation.y_side.end(), cmp) !=
               connection_expectation.y_side.end()) {
      // b --> x
      first_side.first = b_side;
      first_side.second = connection_expectation.x_side;
      // x --> y
      second_side.first = a_side;
      second_side.second = connection_expectation.y_side;
    } else {
      GTEST_FAIL() << "BranchPoint: " << bp->id().string() << " does not match the expectation.";
    }

    for (int lane_end_index = 0; lane_end_index < first_side.first->size(); ++lane_end_index) {
      const LaneEnd dut = first_side.first->get(lane_end_index);
      const CompareConnectionExpectation cmp(dut);
      EXPECT_NE(std::find_if(first_side.second.begin(), first_side.second.end(), cmp), first_side.second.end());
    }

    for (int lane_end_index = 0; lane_end_index < second_side.first->size(); ++lane_end_index) {
      const LaneEnd dut = second_side.first->get(lane_end_index);
      const CompareConnectionExpectation cmp(dut);
      EXPECT_NE(std::find_if(second_side.second.begin(), second_side.second.end(), cmp), second_side.second.end());
    }
  };

  for (const auto& lane_id_connections : expected_connections) {
    const maliput::api::Lane* lane = rg_->ById().GetLane(lane_id_connections.first);
    EXPECT_NE(lane, nullptr);

    // Analyzes A and B sides at the start BranchPoint.
    const maliput::api::BranchPoint* start_bp = lane->GetBranchPoint(LaneEnd::Which::kStart);
    EXPECT_NE(start_bp, nullptr);
    EXPECT_EQ(start_bp->road_geometry(), rg_.get());
    test_branch_point(start_bp, lane_id_connections.second.first);

    // Analyzes A and B sides at the end BranchPoint.
    const maliput::api::BranchPoint* end_bp = lane->GetBranchPoint(LaneEnd::Which::kFinish);
    EXPECT_NE(end_bp, nullptr);
    EXPECT_EQ(end_bp->road_geometry(), rg_.get());
    test_branch_point(end_bp, lane_id_connections.second.second);
  }
}

INSTANTIATE_TEST_CASE_P(BuilderBranchPointTestGroup, BuilderBranchPointTest,
                        ::testing::ValuesIn(InstantiateBuilderBranchPointParameters()));

// Evaluates lane and segment bound computation.
// @{
struct SurfaceBoundariesTestParmeters {
  std::string road_geometry_id;
  std::string path_to_xodr_file;
  double width{};
};

std::vector<SurfaceBoundariesTestParmeters> InstantiateBuilderSurfaceBoundariesParameters() {
  return {
      {"SingleLane", "SingleLane.xodr", 2.}, {"ArcLane", "ArcLane.xodr", 2.}, {"SShapeRoad", "SShapeRoad.xodr", 2.}};
}

// Set up the RoadGeometry.
class RoadGeometryBuilderSurfaceBoundariesTest : public ::testing::TestWithParam<SurfaceBoundariesTestParmeters> {
 protected:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{constants::kScaleLength};
  //@}
  static constexpr double kSStart{0.};

  void SetUp() override {
    road_geometry_configuration_.tolerances.linear_tolerance = kLinearTolerance;
    road_geometry_configuration_.tolerances.max_linear_tolerance = std::nullopt;
    road_geometry_configuration_.tolerances.angular_tolerance = kAngularTolerance;
    dut_ = builder::RoadGeometryBuilder(
        xodr::LoadDataBaseFromFile(
            utility::FindResourceInPath(road_geometry_configuration_.opendrive_file, kMalidriveResourceFolder),
            {kLinearTolerance}),
        road_geometry_configuration_)();
  }

  RoadGeometryConfiguration road_geometry_configuration_{
      GetRoadGeometryConfigurationFor(GetParam().path_to_xodr_file).value()};
  std::unique_ptr<const maliput::api::RoadGeometry> dut_;
};

TEST_P(RoadGeometryBuilderSurfaceBoundariesTest, LaneBoundaries) {
  ASSERT_NE(dut_.get(), nullptr);

  const double width = GetParam().width;

  const auto* lane_right = dut_->ById().GetLane(LaneId("1_0_-1"));
  ASSERT_NE(lane_right, nullptr);
  EXPECT_TRUE(IsRBoundsClose(RBounds(-width / 2., width / 2.), lane_right->lane_bounds(kSStart), kLinearTolerance));
  EXPECT_TRUE(
      IsRBoundsClose(RBounds(-width / 2., width / 2. + width), lane_right->segment_bounds(kSStart), kLinearTolerance));

  const auto* lane_left = dut_->ById().GetLane(LaneId("1_0_1"));
  ASSERT_NE(lane_left, nullptr);
  EXPECT_TRUE(IsRBoundsClose(RBounds(-width / 2., width / 2.), lane_left->lane_bounds(kSStart), kLinearTolerance));
  EXPECT_TRUE(
      IsRBoundsClose(RBounds(-(width / 2. + width), width / 2.), lane_left->segment_bounds(kSStart), kLinearTolerance));
}

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderSurfaceBoundariesTestGroup, RoadGeometryBuilderSurfaceBoundariesTest,
                        ::testing::ValuesIn(InstantiateBuilderSurfaceBoundariesParameters()));
//@}

// Holds the parameters for checking the road geometry when non-drivable lanes are omitted.
struct RoadGeometryOmittingNonDrivableLanesParameters {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadGeometryOmittingNonDrivableLanesParameters)

  RoadGeometryOmittingNonDrivableLanesParameters() = delete;

  RoadGeometryOmittingNonDrivableLanesParameters(const std::string& xodr_file, bool omit_nondrivable_lanes,
                                                 const LaneId& lane_id, const maliput::api::LanePosition& lane_position,
                                                 const maliput::api::InertialPosition& inertial_position,
                                                 const std::vector<LaneId>& lanes_to_left,
                                                 const std::vector<LaneId>& lanes_to_right)
      : xodr_file(xodr_file),
        omit_nondrivable_lanes(omit_nondrivable_lanes),
        lane_id(lane_id),
        lane_position(lane_position),
        inertial_position(inertial_position),
        lanes_to_left(lanes_to_left),
        lanes_to_right(lanes_to_right) {}

  // Xodr file name.
  std::string xodr_file{};
  // Policy to whether omit the nondrivable lanes.
  bool omit_nondrivable_lanes{};
  // Lane id under analysis.
  LaneId lane_id;
  // A Lane position that belongs to #lane_id.
  maliput::api::LanePosition lane_position;
  // An inertial position that corresponds to #lane_position coordinate.
  maliput::api::InertialPosition inertial_position;
  // Lanes located to the left of #laneid.
  // The order of the LaneIds should replicate what is expected to get when calling `maliput::api::Lane::to_left()`
  // method.
  std::vector<LaneId> lanes_to_left;
  // Lanes located to the right of  #laneid.
  // The order of the LaneIds should replicate what is expected to get when calling `maliput::api::Lane::to_right()`
  // method.
  std::vector<LaneId> lanes_to_right;
};

std::vector<RoadGeometryOmittingNonDrivableLanesParameters> InstantiateRoadGeometryNonDrivableLanesParameters() {
  return {
      {
          {"StraightForward.xodr"},
          false /*omit_nondrivable_lanes*/,
          LaneId("0_0_-5"),
          // The lane and inertial position were selected to proof that when
          // the non-drivable lanes are omitted there is no shifting in the offset of the lane.
          {9.499999999999886, 0., 0.} /* lane_position */,
          {1.75, -490.5, 0.} /* inertial_position */,
          {LaneId("0_0_-4"), LaneId("0_0_-3"), LaneId("0_0_-2"), LaneId("0_0_-1")},
          {LaneId("0_0_-6"), LaneId("0_0_-7"), LaneId("0_0_-8")},
      },
      {
          {"StraightForward.xodr"},
          true /*omit_nondrivable_lanes*/,
          LaneId("0_0_-5"),
          {9.499999999999886, 0., 0.} /* lane_position */,
          {1.75, -490.5, 0.} /* inertial_position */,
          {LaneId("0_0_-4")},
          {/* No lanes to the right */},
      },
  };
}

class RoadGeometryOmittingNonDrivableLanesTest
    : public ::testing::TestWithParam<RoadGeometryOmittingNonDrivableLanesParameters> {
 protected:
  void SetUp() override {
    // Set up road geometry configuration with parameters.
    road_geometry_configuration_.omit_nondrivable_lanes = GetParam().omit_nondrivable_lanes;

    // Disable simplification, tolerance range and open drive flexibility.
    road_geometry_configuration_.simplification_policy =
        builder::RoadGeometryConfiguration::SimplificationPolicy::kNone;
    road_geometry_configuration_.tolerances.max_linear_tolerance = std::nullopt;
    road_geometry_configuration_.standard_strictness_policy =
        builder::RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict;

    // Load RoadGeometry.
    dut_ = builder::RoadGeometryBuilder(
        xodr::LoadDataBaseFromFile(
            utility::FindResourceInPath(road_geometry_configuration_.opendrive_file, kMalidriveResourceFolder),
            {road_geometry_configuration_.tolerances.linear_tolerance.value()}),
        road_geometry_configuration_)();

    // Fill useful pointers;
    lane_ = dut_->ById().GetLane(GetParam().lane_id);
    ASSERT_TRUE(lane_ != nullptr);
  }

  const maliput::api::Lane* lane_;
  builder::RoadGeometryConfiguration road_geometry_configuration_{
      malidrive::test::GetRoadGeometryConfigurationFor(GetParam().xodr_file).value()};
  std::unique_ptr<const maliput::api::RoadGeometry> dut_;
};

TEST_P(RoadGeometryOmittingNonDrivableLanesTest, LanesToLeft) {
  const std::vector<LaneId>& expected_lanes_to_left{GetParam().lanes_to_left};

  auto lefter_lane = lane_->to_left();
  int index{0};
  while (lefter_lane != nullptr) {
    EXPECT_EQ(expected_lanes_to_left[index], lefter_lane->id());
    lefter_lane = lefter_lane->to_left();
    index++;
  }
}

TEST_P(RoadGeometryOmittingNonDrivableLanesTest, LanesToRight) {
  const std::vector<LaneId>& expected_lanes_to_right{GetParam().lanes_to_right};

  auto righter_lane = lane_->to_right();
  int index{0};
  while (righter_lane != nullptr) {
    EXPECT_EQ(expected_lanes_to_right[index], righter_lane->id());
    righter_lane = righter_lane->to_right();
    index++;
  }
}

TEST_P(RoadGeometryOmittingNonDrivableLanesTest, InertialPositionAndRoundTripPosition) {
  // Inertial position
  const auto inertial_position = lane_->ToInertialPosition(GetParam().lane_position);
  EXPECT_TRUE(maliput::api::test::IsInertialPositionClose(GetParam().inertial_position, inertial_position,
                                                          constants::kLinearTolerance));

  // Round trip
  auto result = dut_->ToRoadPosition(inertial_position);
  EXPECT_EQ(lane_->id(), result.road_position.lane->id());
  EXPECT_TRUE(maliput::api::test::IsLanePositionClose(GetParam().lane_position, result.road_position.pos,
                                                      constants::kLinearTolerance));
}

INSTANTIATE_TEST_CASE_P(RoadGeometryOmittingNonDrivableLanesTestGroup, RoadGeometryOmittingNonDrivableLanesTest,
                        ::testing::ValuesIn(InstantiateRoadGeometryNonDrivableLanesParameters()));

class RoadGeometryNegativeLaneWidthTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  const std::string kXodrFile{"SingleRoadNegativeWidth.xodr"};
  builder::RoadGeometryConfiguration rg_config{malidrive::test::GetRoadGeometryConfigurationFor(kXodrFile).value()};
  std::unique_ptr<const maliput::api::RoadGeometry> dut_;
};

// Throws if negative width descriptions are found because of the strictness in the builder.
TEST_F(RoadGeometryNegativeLaneWidthTest, Strict) {
  rg_config.standard_strictness_policy = builder::RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict;
  rg_config.tolerances.linear_tolerance = constants::kStrictLinearTolerance;
  // Disables the tolerance range.
  rg_config.tolerances.max_linear_tolerance = std::nullopt;

  EXPECT_THROW(
      builder::RoadGeometryBuilder(
          xodr::LoadDataBaseFromFile(utility::FindResourceInPath(rg_config.opendrive_file, kMalidriveResourceFolder),
                                     {rg_config.tolerances.linear_tolerance.value()}),
          rg_config)(),
      maliput::common::assertion_error);
}

// Allow having negative width descriptions.
// Width is clamped to zero when a lane-bound query is called.
TEST_F(RoadGeometryNegativeLaneWidthTest, AllowNegativeWidthDescriptions) {
  rg_config.standard_strictness_policy =
      builder::RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors;

  ASSERT_NO_THROW(
      dut_ = builder::RoadGeometryBuilder(
          xodr::LoadDataBaseFromFile(utility::FindResourceInPath(rg_config.opendrive_file, kMalidriveResourceFolder),
                                     {rg_config.tolerances.linear_tolerance.value()}),
          rg_config)());
  ASSERT_NE(dut_, nullptr);

  // Let's make a `lane_bounds` query to a lane whose lane-width function has negative values.
  const maliput::api::LaneId lane_id("265_0_-5");
  const auto lane = dut_->ById().GetLane(lane_id);
  ASSERT_NE(lane, nullptr);
  // In a range (0, 2.56994) the width description has negative values.
  // `lane_bounds` query should return [0, 0] when queried in that range.
  auto bounds = lane->lane_bounds(1.5);
  EXPECT_EQ(0., bounds.min());
  EXPECT_EQ(0., bounds.max());
  // Verify that after the negative range we get the correct value.
  bounds = lane->lane_bounds(8.0);
  EXPECT_NEAR(-0.02839, bounds.min(), rg_config.tolerances.linear_tolerance.value());
  EXPECT_NEAR(0.02839, bounds.max(), rg_config.tolerances.linear_tolerance.value());
}

// Verifies G1 contiguity constraint being relaxed when semantic errors are allowed.
// - Roads with only non-drivable lanes that presents a jump in the elevation description function.
// - Roads with only non-drivable lanes that presents a jump in the superelevation description function.
// - Non-drivable lanes that presents a jump in the lane width description function.
class RoadGeometryNonContiguousInNonDrivableLanesTest : public testing::TestWithParam<const char*> {};

TEST_P(RoadGeometryNonContiguousInNonDrivableLanesTest, CheckG1ContiguityEnforcement) {
  builder::RoadGeometryConfiguration rg_config{malidrive::test::GetRoadGeometryConfigurationFor(GetParam()).value()};

  // Gap in non drivable road/lane + strict.
  rg_config.standard_strictness_policy = builder::RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict;

  EXPECT_THROW(
      builder::RoadGeometryBuilder(
          xodr::LoadDataBaseFromFile(utility::FindResourceInPath(rg_config.opendrive_file, kMalidriveResourceFolder),
                                     {rg_config.tolerances.linear_tolerance.value()}),
          rg_config)(),
      maliput::common::assertion_error);

  // Gap in non drivable road/lane + allow semantic errors.
  rg_config.standard_strictness_policy =
      builder::RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors;

  EXPECT_NO_THROW(builder::RoadGeometryBuilder(
      xodr::LoadDataBaseFromFile(utility::FindResourceInPath(rg_config.opendrive_file, kMalidriveResourceFolder),
                                 {rg_config.tolerances.linear_tolerance.value()}),
      rg_config)());
}

INSTANTIATE_TEST_CASE_P(CheckG1ContiguityEnforcementGroup, RoadGeometryNonContiguousInNonDrivableLanesTest,
                        ::testing::Values("GapInElevationNonDrivableRoad.xodr",
                                          "GapInSuperelevationNonDrivableRoad.xodr",
                                          "GapInLaneWidthNonDrivableLane.xodr"));

// Verifies RoadGeometryBuilder behavior for different combinations of
// linear_tolerance and max_linear_tolerance parameters.
// `GapInElevationNonDrivableRoad.xodr` XODR map is used as it has a 2.0m gap in an elevation description, condition
// that will impose on the builder to have at least a linear tolerance of 2.0m to correctly build a RoadGeometry.
class ToleranceSelectionPolicyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rg_config.opendrive_file = utility::FindResourceInPath(kXodrFile, kMalidriveResourceFolder);
    rg_config.omit_nondrivable_lanes = false;
    rg_config.standard_strictness_policy = RoadGeometryConfiguration::StandardStrictnessPolicy::kStrict;
    rg_config.tolerances.linear_tolerance = std::nullopt;
    rg_config.tolerances.max_linear_tolerance = std::nullopt;
  }

  const std::string kXodrFile{"GapInElevationNonDrivableRoad.xodr"};
  builder::RoadGeometryConfiguration rg_config{};
};

// No linear_tolerance/max_linear_tolerance are set. Default linear_tolerance will be used:
// malidrive::constants::kLinearTolerance. It throws because the gap in elevation is larger than linear tolerance.
TEST_F(ToleranceSelectionPolicyTest, DefaultLinearTolerance) {
  ASSERT_THROW(
      builder::RoadGeometryBuilder(xodr::LoadDataBaseFromFile(rg_config.opendrive_file, {std::nullopt}), rg_config)(),
      maliput::common::assertion_error);
}

// linear_tolerance is set to 2.0m.
// It doesn't throw.
TEST_F(ToleranceSelectionPolicyTest, OnlyValidLinearTolerance) {
  rg_config.tolerances.linear_tolerance = 2.0;
  ASSERT_NO_THROW(
      builder::RoadGeometryBuilder(xodr::LoadDataBaseFromFile(rg_config.opendrive_file, {std::nullopt}), rg_config)());
}

// max_linear_tolerance is set to 2.0m.
// A linear tolerance range defined by [constants::kBaseLinearTolerance, max_linear_tolerance] is set.
// The builder will search for a tolerance that works which will be the last one. (2.0m).
TEST_F(ToleranceSelectionPolicyTest, OnlyValidMaxLinearTolerance) {
  rg_config.tolerances.max_linear_tolerance = 2.0;
  ASSERT_NO_THROW(
      builder::RoadGeometryBuilder(xodr::LoadDataBaseFromFile(rg_config.opendrive_file, {std::nullopt}), rg_config)());
}

// linear_tolerance is set to 1.0m and max_linear_tolerance is set to 2.0m.
// A linear tolerance range defined by [linear_tolerance, max_linear_tolerance] is set.
// The builder will search for a tolerance that works which will be the last one. (2.0m).
TEST_F(ToleranceSelectionPolicyTest, ValidLinearToleranceRange) {
  rg_config.tolerances.linear_tolerance = 1.0;
  rg_config.tolerances.max_linear_tolerance = 2.0;
  ASSERT_NO_THROW(
      builder::RoadGeometryBuilder(xodr::LoadDataBaseFromFile(rg_config.opendrive_file, {std::nullopt}), rg_config)());
}

// linear_tolerance is set to 1.0m and max_linear_tolerance is set to 1.9m.
// A linear tolerance range defined by [linear_tolerance, max_linear_tolerance] is set.
// It throws as the builder can't find a tolerance that works defined in that range.
TEST_F(ToleranceSelectionPolicyTest, InvalidLinearToleranceRange) {
  rg_config.tolerances.linear_tolerance = 1.0;
  rg_config.tolerances.max_linear_tolerance = 1.9;
  ASSERT_THROW(
      builder::RoadGeometryBuilder(xodr::LoadDataBaseFromFile(rg_config.opendrive_file, {std::nullopt}), rg_config)(),
      maliput::common::assertion_error);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
