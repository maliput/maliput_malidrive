// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_builder_base.h"

#include <vector>

#include <gtest/gtest.h>

#include "maliput/geometry_base/branch_point.h"
#include "maliput/geometry_base/lane.h"

#include "maliput_malidrive/common/macros.h"

using maliput::api::BranchPointId;
using maliput::api::InertialPosition;
using maliput::api::LaneEnd;
using maliput::api::LaneId;
using maliput::api::Rotation;
using maliput::geometry_base::BranchPoint;

namespace malidrive {
namespace builder {
namespace test {
namespace {

class MockLane : public maliput::geometry_base::Lane {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLane);

  /// Constructs a partially-initialized MockLane.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::Lane for discussion on initialization.
  explicit MockLane(const LaneId& id) : maliput::geometry_base::Lane(id) {}

 private:
  double do_length() const override { MALIDRIVE_ABORT_MSG("Code should never reach this point"); }
  maliput::api::RBounds do_lane_bounds(double) const override {
    MALIDRIVE_ABORT_MSG("Code should never reach this point");
  }
  maliput::api::RBounds do_segment_bounds(double) const override {
    MALIDRIVE_ABORT_MSG("Code should never reach this point");
  }
  maliput::api::HBounds do_elevation_bounds(double, double) const override {
    MALIDRIVE_ABORT_MSG("Code should never reach this point");
  }
  maliput::api::InertialPosition DoToInertialPosition(const maliput::api::LanePosition&) const override {
    MALIDRIVE_ABORT_MSG("Code should never reach this point");
  }
  maliput::api::Rotation DoGetOrientation(const maliput::api::LanePosition&) const override {
    MALIDRIVE_ABORT_MSG("Code should never reach this point");
  }
  maliput::api::LanePosition DoEvalMotionDerivatives(const maliput::api::LanePosition&,
                                                     const maliput::api::IsoLaneVelocity&) const override {
    MALIDRIVE_ABORT_MSG("Code should never reach this point");
  }
  maliput::api::LanePositionResult DoToLanePosition(const maliput::api::InertialPosition&) const override {
    MALIDRIVE_ABORT_MSG("Code should never reach this point");
  }
};

class MockRoadGeometryBuilderBase : public RoadGeometryBuilderBase {
 public:
  MockRoadGeometryBuilderBase(const RoadGeometryConfiguration& road_geometry_configuration)
      : RoadGeometryBuilderBase(road_geometry_configuration) {}
  std::unique_ptr<const maliput::api::RoadGeometry> operator()() override { return {}; }

  void EXPECT_TRUE_IsLaneEndOnASide(const maliput::api::BranchPoint* bp, const maliput::api::LaneEnd& lane_end) {
    EXPECT_TRUE(IsLaneEndOnABSide(bp, lane_end, BranchPointSide::kASide));
  }
  void EXPECT_TRUE_IsLaneEndOnBSide(const maliput::api::BranchPoint* bp, const maliput::api::LaneEnd& lane_end) {
    EXPECT_TRUE(IsLaneEndOnABSide(bp, lane_end, BranchPointSide::kBSide));
  }
  void EXPECT_FALSE_IsLaneEndOnASide(const maliput::api::BranchPoint* bp, const maliput::api::LaneEnd& lane_end) {
    EXPECT_FALSE(IsLaneEndOnABSide(bp, lane_end, BranchPointSide::kASide));
  }
  void EXPECT_FALSE_IsLaneEndOnBSide(const maliput::api::BranchPoint* bp, const maliput::api::LaneEnd& lane_end) {
    EXPECT_FALSE(IsLaneEndOnABSide(bp, lane_end, BranchPointSide::kBSide));
  }

  void EXPECT_EQ_FindBranchpointByLaneEndASide(
      const maliput::api::LaneEnd& lane_end,
      const std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>>& bps,
      maliput::geometry_base::BranchPoint* bp_expected) {
    auto result = FindBranchpointByLaneEnd(lane_end, bps);
    EXPECT_EQ(bp_expected, result.first);
    if (bp_expected != nullptr) {
      EXPECT_EQ(BranchPointSide::kASide, result.second.value());
    }
  }
  void EXPECT_EQ_FindBranchpointByLaneEndBSide(
      const maliput::api::LaneEnd& lane_end,
      const std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>>& bps,
      maliput::geometry_base::BranchPoint* bp_expected) {
    auto result = FindBranchpointByLaneEnd(lane_end, bps);
    EXPECT_EQ(bp_expected, result.first);
    if (bp_expected != nullptr) {
      EXPECT_EQ(BranchPointSide::kBSide, result.second.value());
    }
  }
};

class RoadGeometryBuilderBaseTest : public ::testing::Test {
 protected:
  void SetUp() override { dut = std::make_unique<MockRoadGeometryBuilderBase>(kRoadGeometryConfiguration); }

  const RoadGeometryConfiguration kRoadGeometryConfiguration{maliput::api::RoadGeometryId("dumb_id"),
                                                             "path",
                                                             constants::kLinearTolerance,
                                                             constants::kAngularTolerance,
                                                             constants::kScaleLength,
                                                             maliput::math::Vector3{0., 0., 0.},
                                                             InertialToLaneMappingConfig(1e-3, 10)};
  std::unique_ptr<MockRoadGeometryBuilderBase> dut;
};

// Proposes the following lane layout
//
// l0 --> l1 --> l2
// l3 --^
//
TEST_F(RoadGeometryBuilderBaseTest, IsLaneEndOnABSideTest) {
  MockLane l0(LaneId("l0"));
  MockLane l1(LaneId("l1"));
  MockLane l2(LaneId("l2"));
  MockLane l3(LaneId("l3"));

  BranchPoint bp1(BranchPointId("bp1"));

  bp1.AddABranch(&l0, LaneEnd::Which::kFinish);
  bp1.AddABranch(&l3, LaneEnd::Which::kFinish);
  bp1.AddBBranch(&l1, LaneEnd::Which::kStart);

  dut->EXPECT_TRUE_IsLaneEndOnASide(&bp1, LaneEnd(&l0, LaneEnd::Which::kFinish));
  dut->EXPECT_TRUE_IsLaneEndOnASide(&bp1, LaneEnd(&l3, LaneEnd::Which::kFinish));
  dut->EXPECT_FALSE_IsLaneEndOnASide(&bp1, LaneEnd(&l2, LaneEnd::Which::kStart));
  dut->EXPECT_TRUE_IsLaneEndOnBSide(&bp1, LaneEnd(&l1, LaneEnd::Which::kStart));
  dut->EXPECT_FALSE_IsLaneEndOnBSide(&bp1, LaneEnd(&l0, LaneEnd::Which::kFinish));
}

TEST_F(RoadGeometryBuilderBaseTest, FindBranchpointByLaneEndTest) {
  MockLane l0(LaneId("l0"));
  MockLane l1(LaneId("l1"));
  MockLane l2(LaneId("l2"));
  MockLane l3(LaneId("l3"));
  MockLane l4(LaneId("l4"));  // Unconnected lane.

  std::vector<std::unique_ptr<BranchPoint>> bps;

  bps.push_back(std::make_unique<BranchPoint>(BranchPointId("bp0")));
  bps.back()->AddABranch(&l0, LaneEnd::Which::kStart);

  bps.push_back(std::make_unique<BranchPoint>(BranchPointId("bp1")));
  bps.back()->AddABranch(&l3, LaneEnd::Which::kStart);

  bps.push_back(std::make_unique<BranchPoint>(BranchPointId("bp2")));
  bps.back()->AddABranch(&l0, LaneEnd::Which::kFinish);
  bps.back()->AddABranch(&l3, LaneEnd::Which::kFinish);
  bps.back()->AddBBranch(&l1, LaneEnd::Which::kStart);

  bps.push_back(std::make_unique<BranchPoint>(BranchPointId("bp3")));
  bps.back()->AddABranch(&l1, LaneEnd::Which::kFinish);
  bps.back()->AddBBranch(&l2, LaneEnd::Which::kStart);

  bps.push_back(std::make_unique<BranchPoint>(BranchPointId("bp4")));
  bps.back()->AddABranch(&l2, LaneEnd::Which::kFinish);

  // l4, no BranchPoint for it.
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l4, LaneEnd::Which::kStart), bps, nullptr);
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l4, LaneEnd::Which::kFinish), bps, nullptr);

  // l0
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l0, LaneEnd::Which::kStart), bps, bps[0].get());
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l0, LaneEnd::Which::kFinish), bps, bps[2].get());

  // l3
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l3, LaneEnd::Which::kStart), bps, bps[1].get());
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l3, LaneEnd::Which::kFinish), bps, bps[2].get());

  // l1
  dut->EXPECT_EQ_FindBranchpointByLaneEndBSide(LaneEnd(&l1, LaneEnd::Which::kStart), bps, bps[2].get());
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l1, LaneEnd::Which::kFinish), bps, bps[3].get());

  // l2
  dut->EXPECT_EQ_FindBranchpointByLaneEndBSide(LaneEnd(&l2, LaneEnd::Which::kStart), bps, bps[3].get());
  dut->EXPECT_EQ_FindBranchpointByLaneEndASide(LaneEnd(&l2, LaneEnd::Which::kFinish), bps, bps[4].get());
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
