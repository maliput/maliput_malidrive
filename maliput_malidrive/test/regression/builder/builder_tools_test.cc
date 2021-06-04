// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/builder_tools.h"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/mock.h>

#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "utility/resources.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

// TODO(maliput#341): Remove this operator once it is implemented in maliput.
bool operator==(const maliput::api::LaneEnd& end_a, const maliput::api::LaneEnd& end_b) {
  return end_a.end == end_b.end && end_a.lane == end_b.lane;
}

// Creates a Road with:
//  - 4 LaneSections:
//     - 1 Lane per LaneSection.
//  Connections scheme:
//  - Lane A --> Lane B --> Lane C --> Lane D.
// The goal is to verify the SolveLaneEndsForInnerLaneSection() method.
class SolveLaneEndsInnerLaneSectionTest : public ::testing::Test {
 public:
  const int kNoneLaneEnds{0};
  const int kOneLaneEnds{1};
  const int kLaneSection0Index{0};
  const int kLaneSection1Index{1};
  const int kLaneSection2Index{2};
  const int kLaneSection3Index{3};

 protected:
  void SetUp() override {
    // Create RoadGeometry.
    lane_a = std::make_unique<maliput::api::test::MockLane>(maliput::api::LaneId("1_0_1"));
    lane_b = std::make_unique<maliput::api::test::MockLane>(maliput::api::LaneId("1_1_2"));
    lane_c = std::make_unique<maliput::api::test::MockLane>(maliput::api::LaneId("1_2_3"));
    lane_d = std::make_unique<maliput::api::test::MockLane>(maliput::api::LaneId("1_3_4"));
    auto mock_rg =
        std::make_unique<maliput::api::test::MockRoadGeometry>(maliput::api::RoadGeometryId("RoadGeometryTest"));
    mock_rg->GetIdIndex()->add_lane_to_map(lane_a->id(), lane_a.get());
    mock_rg->GetIdIndex()->add_lane_to_map(lane_b->id(), lane_b.get());
    mock_rg->GetIdIndex()->add_lane_to_map(lane_c->id(), lane_c.get());
    mock_rg->GetIdIndex()->add_lane_to_map(lane_d->id(), lane_d.get());
    rg = std::move(mock_rg);

    // Create LaneEnds.
    end_a_start = maliput::api::LaneEnd(lane_a.get(), maliput::api::LaneEnd::Which::kStart);
    end_a_finish = maliput::api::LaneEnd(lane_a.get(), maliput::api::LaneEnd::Which::kFinish);
    end_b_start = maliput::api::LaneEnd(lane_b.get(), maliput::api::LaneEnd::Which::kStart);
    end_b_finish = maliput::api::LaneEnd(lane_b.get(), maliput::api::LaneEnd::Which::kFinish);
    end_c_start = maliput::api::LaneEnd(lane_c.get(), maliput::api::LaneEnd::Which::kStart);
    end_c_finish = maliput::api::LaneEnd(lane_c.get(), maliput::api::LaneEnd::Which::kFinish);
    end_d_start = maliput::api::LaneEnd(lane_d.get(), maliput::api::LaneEnd::Which::kStart);
    end_d_finish = maliput::api::LaneEnd(lane_d.get(), maliput::api::LaneEnd::Which::kFinish);

    // Create XodrMalidriveLaneProperties.
    const xodr::LaneLink kLaneLinkA{
        std::nullopt /* predecessor */,
        xodr::LaneLink::LinkAttributes{xodr::LaneLink::LinkAttributes::Id{"2"}} /* successor */};
    const xodr::LaneLink kLaneLinkB{
        xodr::LaneLink::LinkAttributes{xodr::LaneLink::LinkAttributes::Id{"1"}} /* predecessor */,
        xodr::LaneLink::LinkAttributes{xodr::LaneLink::LinkAttributes::Id{"3"}} /* successor */};
    const xodr::LaneLink kLaneLinkC{
        xodr::LaneLink::LinkAttributes{xodr::LaneLink::LinkAttributes::Id{"2"}} /* predecessor */,
        xodr::LaneLink::LinkAttributes{xodr::LaneLink::LinkAttributes::Id{"4"}} /* successor */};
    const xodr::LaneLink kLaneLinkD{
        xodr::LaneLink::LinkAttributes{xodr::LaneLink::LinkAttributes::Id{"3"}} /* predecessor */,
        std::nullopt /* successor */};

    const xodr::Geometry kGeometry{0., {0., 0.}, 0.0, 100., xodr::Geometry::Type::kLine, xodr::Geometry::Line{}};
    const xodr::Lane kCenterLane{xodr::Lane::Id("0"), xodr::Lane::Type::kDriving, false, {}, {}};
    // LaneIds are created purely for testing purposes. It doesn't reflex real maps.
    const xodr::Lane kLeftLaneA{xodr::Lane::Id("1"), xodr::Lane::Type::kDriving, false, kLaneLinkA, {}};
    const xodr::Lane kLeftLaneB{xodr::Lane::Id("2"), xodr::Lane::Type::kDriving, false, kLaneLinkB, {}};
    const xodr::Lane kLeftLaneC{xodr::Lane::Id("3"), xodr::Lane::Type::kDriving, false, kLaneLinkC, {}};
    const xodr::Lane kLeftLaneD{xodr::Lane::Id("4"), xodr::Lane::Type::kDriving, false, kLaneLinkD, {}};

    const xodr::LaneSection kLaneSection0{0., std::nullopt, {kLeftLaneA}, kCenterLane, {}};
    const xodr::LaneSection kLaneSection1{25., std::nullopt, {kLeftLaneB}, kCenterLane, {}};
    const xodr::LaneSection kLaneSection2{50., std::nullopt, {kLeftLaneC}, kCenterLane, {}};
    const xodr::LaneSection kLaneSection3{75., std::nullopt, {kLeftLaneD}, kCenterLane, {}};
    const xodr::Lanes kLanes{{{}}, {{kLaneSection0}, {kLaneSection1}, {kLaneSection2}, {kLaneSection3}}};
    road_header = xodr::RoadHeader{"TestRoadHeader", 100.,  xodr::RoadHeader::Id("1"), "-1", std::nullopt, {}, {},
                                   {{{kGeometry}}},  kLanes};
  }
  std::unique_ptr<maliput::api::Lane> lane_a;
  std::unique_ptr<maliput::api::Lane> lane_b;
  std::unique_ptr<maliput::api::Lane> lane_c;
  std::unique_ptr<maliput::api::Lane> lane_d;
  std::unique_ptr<maliput::api::RoadGeometry> rg;
  maliput::api::LaneEnd end_a_start;
  maliput::api::LaneEnd end_a_finish;
  maliput::api::LaneEnd end_b_start;
  maliput::api::LaneEnd end_b_finish;
  maliput::api::LaneEnd end_c_start;
  maliput::api::LaneEnd end_c_finish;
  maliput::api::LaneEnd end_d_start;
  maliput::api::LaneEnd end_d_finish;
  xodr::RoadHeader road_header;
};

// Evaluates SolveLaneEndsInnerLaneSection() method.
TEST_F(SolveLaneEndsInnerLaneSectionTest, TestMethod) {
  const int LaneIndex{0};
  const int LaneEndIndex{0};
  {  // Lane A.
    const xodr::LaneSection* lane_section{&road_header.lanes.lanes_section[kLaneSection0Index]};
    const MalidriveXodrLaneProperties kMalidriveXodrLanePropertiesA(&road_header, lane_section, kLaneSection0Index,
                                                                    &lane_section->left_lanes[LaneIndex]);
    auto lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_a_start, kMalidriveXodrLanePropertiesA);
    EXPECT_EQ(kNoneLaneEnds, lane_ends.size());
    lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_a_finish, kMalidriveXodrLanePropertiesA);
    EXPECT_EQ(kOneLaneEnds, lane_ends.size());
    EXPECT_TRUE(end_b_start == lane_ends[LaneEndIndex]);
  }
  {  // Lane B.
    const xodr::LaneSection* lane_section{&road_header.lanes.lanes_section[kLaneSection1Index]};
    const MalidriveXodrLaneProperties kMalidriveXodrLanePropertiesB(&road_header, lane_section, kLaneSection1Index,
                                                                    &lane_section->left_lanes[LaneIndex]);
    auto lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_b_start, kMalidriveXodrLanePropertiesB);
    EXPECT_EQ(kOneLaneEnds, lane_ends.size());
    EXPECT_TRUE(end_a_finish == lane_ends[LaneEndIndex]);
    lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_b_finish, kMalidriveXodrLanePropertiesB);
    EXPECT_EQ(kOneLaneEnds, lane_ends.size());
    EXPECT_TRUE(end_c_start == lane_ends[LaneEndIndex]);
  }
  {  // Lane C.
    const xodr::LaneSection* lane_section{&road_header.lanes.lanes_section[kLaneSection2Index]};
    const MalidriveXodrLaneProperties kMalidriveXodrLanePropertiesC(&road_header, lane_section, kLaneSection2Index,
                                                                    &lane_section->left_lanes[LaneIndex]);
    auto lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_c_start, kMalidriveXodrLanePropertiesC);
    EXPECT_EQ(kOneLaneEnds, lane_ends.size());
    EXPECT_TRUE(end_b_finish == lane_ends[LaneEndIndex]);
    lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_c_finish, kMalidriveXodrLanePropertiesC);
    EXPECT_EQ(kOneLaneEnds, lane_ends.size());
    EXPECT_TRUE(end_d_start == lane_ends[LaneEndIndex]);
  }
  {  // Lane D.
    const xodr::LaneSection* lane_section{&road_header.lanes.lanes_section[kLaneSection3Index]};
    const MalidriveXodrLaneProperties kMalidriveXodrLanePropertiesD(&road_header, lane_section, kLaneSection3Index,
                                                                    &lane_section->left_lanes[LaneIndex]);
    auto lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_d_start, kMalidriveXodrLanePropertiesD);
    EXPECT_EQ(kOneLaneEnds, lane_ends.size());
    EXPECT_TRUE(end_c_finish == lane_ends[LaneEndIndex]);
    lane_ends = SolveLaneEndsForInnerLaneSection(rg.get(), end_d_finish, kMalidriveXodrLanePropertiesD);
    EXPECT_EQ(kNoneLaneEnds, lane_ends.size());
  }
}

GTEST_TEST(LaneTravelDirection, NonCompleteXmlNode) {
  // Empty string.
  EXPECT_EQ(LaneTravelDirection("").GetXodrTravelDir(), LaneTravelDirection::Direction::kUndefined);
  // XML formatting error.
  EXPECT_EQ(LaneTravelDirection("Lorem Ipsum").GetXodrTravelDir(), LaneTravelDirection::Direction::kUndefined);
  // Missing vectorLane node.
  EXPECT_EQ(LaneTravelDirection("<userData> <wrong node='true'/> </userData>").GetXodrTravelDir(),
            LaneTravelDirection::Direction::kUndefined);
  // Missing travelDir node.
  EXPECT_EQ(LaneTravelDirection("<userData> <vectorLane wrongAttribute='true'/> </userData>").GetXodrTravelDir(),
            LaneTravelDirection::Direction::kUndefined);
  // Direction value error.
  EXPECT_THROW(LaneTravelDirection("<userData> <vectorLane travelDir='wrongValue'/> </userData>"),
               maliput::common::assertion_error);
}

GTEST_TEST(LaneTravelDirection, TravelDir) {
  const char* kUserDataNode = R"R(
    <userData>
        <vectorLane travelDir="backward"/>
    </userData>
  )R";
  EXPECT_EQ(LaneTravelDirection::Direction::kBackward, LaneTravelDirection(kUserDataNode).GetXodrTravelDir());
  const std::string kExpectedMaliputValue{"AgainstS"};
  EXPECT_EQ(kExpectedMaliputValue, LaneTravelDirection(kUserDataNode).GetMaliputTravelDir());
}

/*
  Loaded map has the following structure:

                    (0.,0.,0.)         (100.,0.,0.)
  Sidewalk  | L: 4  |==================| Width: 2m
  Shoulder  | L: 3  |==================| Width: 0.2m
  Biking    | L: 2  |________>_________| Width: 1.2m
  Driving   | L: 1  |-------->---------| Width: 3.5m
  Track lane| L: 0  |========>=========| Width: 0m
  Driving   | L: -1 |-------->---------| Width: 3.5m
  Shoulder  | L: -2 |==================| Width: 0.2m
  Sidewalk  | L: -3 |==================| Width: 2m
                             ^
                    Road 1
                    Section 0

  <^> --> shows where TRACK Frame s-coordinate is placed for all lanes.
*/
class LanePropertiesTest : public ::testing::Test {
 public:
  const std::string kRoadId{"1"};
  const int kLeftSidewalkLaneIndex{3};
  const int kLeftShoulderLaneIndex{2};
  const int kLeftBikeLaneIndex{1};
  const int kLeftDrivingLaneIndex{0};
  const int kRightDrivingLaneIndex{2};
  const int kRightShoulderLaneIndex{1};
  const int kRightSidewalkLaneIndex{0};
  const std::string kNonPedestrians{"NonPedestrians"};
  const std::string kNonVehicles{"NonVehicles"};
  const std::string kUnrestricted{"Unrestricted"};
  const std::string kNonMotorizedVehicleOnly{"NonMotorizedVehicleOnly"};

 protected:
  void SetUp() override {
    manager_ = xodr::LoadDataBaseFromFile(utility::FindResource("odr/BikingLineLane.xodr"), {std::nullopt});
    road_header_ = &manager_->GetRoadHeaders().at(xodr::RoadHeader::Id(kRoadId));
  }
  std::unique_ptr<xodr::DBManager> manager_;
  const xodr::RoadHeader* road_header_;
};

// Evaluates whether or not a lane is driveable based on its XODR Lane type.
TEST_F(LanePropertiesTest, IsDriveableLane) {
  EXPECT_TRUE(is_driveable_lane(road_header_->lanes.lanes_section[0].left_lanes[kLeftBikeLaneIndex]));

  EXPECT_TRUE(is_driveable_lane(road_header_->lanes.lanes_section[0].left_lanes[kLeftDrivingLaneIndex]));
  EXPECT_TRUE(is_driveable_lane(road_header_->lanes.lanes_section[0].right_lanes[kRightDrivingLaneIndex]));

  EXPECT_FALSE(is_driveable_lane(road_header_->lanes.lanes_section[0].left_lanes[kLeftShoulderLaneIndex]));
  EXPECT_FALSE(is_driveable_lane(road_header_->lanes.lanes_section[0].right_lanes[kRightShoulderLaneIndex]));

  EXPECT_FALSE(is_driveable_lane(road_header_->lanes.lanes_section[0].left_lanes[kLeftSidewalkLaneIndex]));
  EXPECT_FALSE(is_driveable_lane(road_header_->lanes.lanes_section[0].right_lanes[kRightSidewalkLaneIndex]));

  EXPECT_FALSE(is_driveable_lane(road_header_->lanes.lanes_section[0].center_lane));
}

// Evaluates vehicle usage rule value based on XODR Lane type.
TEST_F(LanePropertiesTest, VehicleUsageValue) {
  EXPECT_EQ(kNonPedestrians,
            VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftBikeLaneIndex]));

  EXPECT_EQ(kNonPedestrians,
            VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftDrivingLaneIndex]));
  EXPECT_EQ(kNonPedestrians,
            VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].right_lanes[kRightDrivingLaneIndex]));

  EXPECT_EQ(kNonVehicles,
            VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftShoulderLaneIndex]));
  EXPECT_EQ(kNonVehicles,
            VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].right_lanes[kRightShoulderLaneIndex]));

  EXPECT_EQ(kNonVehicles,
            VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftSidewalkLaneIndex]));
  EXPECT_EQ(kNonVehicles,
            VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].right_lanes[kRightSidewalkLaneIndex]));

  EXPECT_EQ(kUnrestricted, VehicleUsageValueForXodrLane(road_header_->lanes.lanes_section[0].center_lane));
}

// Evaluates vehicle exclusive rule value based on XODR Lane type.
TEST_F(LanePropertiesTest, VehicleExclusiveValue) {
  const auto& vehicle_exclusive =
      VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftBikeLaneIndex]);
  EXPECT_TRUE(vehicle_exclusive.has_value());
  EXPECT_EQ(kNonMotorizedVehicleOnly, vehicle_exclusive.value());

  EXPECT_FALSE(VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftDrivingLaneIndex])
                   .has_value());
  EXPECT_FALSE(
      VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].right_lanes[kRightDrivingLaneIndex])
          .has_value());

  EXPECT_FALSE(VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftShoulderLaneIndex])
                   .has_value());
  EXPECT_FALSE(
      VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].right_lanes[kRightShoulderLaneIndex])
          .has_value());

  EXPECT_FALSE(VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].left_lanes[kLeftSidewalkLaneIndex])
                   .has_value());
  EXPECT_FALSE(
      VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].right_lanes[kRightSidewalkLaneIndex])
          .has_value());

  EXPECT_FALSE(VehicleExclusiveValueForXodrLane(road_header_->lanes.lanes_section[0].center_lane).has_value());
}

struct GetRoadTypeSpeedPropertiesInRangeTest : public ::testing::Test {
  const xodr::ReferenceGeometry kReferenceGeometry{{{{1.23, {523.2, 83.27}, 0.77, 100., xodr::Geometry::Type::kLine}}}};
  const std::vector<xodr::LaneSection> kLaneSections{{0.,
                                                      true,
                                                      {{xodr::Lane::Id("1"),
                                                        xodr::Lane::Type::kDriving,
                                                        false,
                                                        {},
                                                        std::vector<xodr::LaneWidth>{{1.1, 2.2, 3.3, 4.4, 5.5}}}},
                                                      {xodr::Lane::Id("0"), xodr::Lane::Type::kShoulder, false, {}, {}},
                                                      {{xodr::Lane::Id("-1"),
                                                        xodr::Lane::Type::kRestricted,
                                                        false,
                                                        {},
                                                        std::vector<xodr::LaneWidth>{{5.5, 6.6, 7.7, 8.8, 9.9}}}}},
                                                     {30.,
                                                      true,
                                                      {{xodr::Lane::Id("1"),
                                                        xodr::Lane::Type::kDriving,
                                                        false,
                                                        {},
                                                        std::vector<xodr::LaneWidth>{{1.1, 2.2, 3.3, 4.4, 5.5}}}},
                                                      {xodr::Lane::Id("0"), xodr::Lane::Type::kShoulder, false, {}, {}},
                                                      {{xodr::Lane::Id("-1"),
                                                        xodr::Lane::Type::kRestricted,
                                                        false,
                                                        {},
                                                        std::vector<xodr::LaneWidth>{{5.5, 6.6, 7.7, 8.8, 9.9}}}}},
                                                     {70.,
                                                      true,
                                                      {{xodr::Lane::Id("1"),
                                                        xodr::Lane::Type::kDriving,
                                                        false,
                                                        {},
                                                        std::vector<xodr::LaneWidth>{{1.1, 2.2, 3.3, 4.4, 5.5}}}},
                                                      {xodr::Lane::Id("0"), xodr::Lane::Type::kShoulder, false, {}, {}},
                                                      {{xodr::Lane::Id("-1"),
                                                        xodr::Lane::Type::kRestricted,
                                                        false,
                                                        {},
                                                        std::vector<xodr::LaneWidth>{{5.5, 6.6, 7.7, 8.8, 9.9}}}}}};
  const xodr::Lanes kLanes{{{}}, {{kLaneSections}}};
  const xodr::RoadType kRoadType0{0. /* s0 */,
                                  xodr::RoadType::Type::kTown /* type */,
                                  std::nullopt /* country */,
                                  {45. /* max */, xodr::Unit::kMph /* unit */} /* speed */};
  const xodr::RoadType kRoadType1{
      53. /* s0 */, xodr::RoadType::Type::kRural /* type */, "maliput" /* country */, {} /* speed */};
  const xodr::RoadType kRoadType2{74. /* s0 */,
                                  xodr::RoadType::Type::kRural /* type */,
                                  "maliput" /* country */,
                                  {120. /* max */, xodr::Unit::kKph /* unit */} /* speed */};
  const xodr::RoadHeader kRoadHeader{"TestRoadHeader",
                                     100. /* length */,
                                     xodr::RoadHeader::Id("Road 1") /* id */,
                                     "-1" /* junction */,
                                     xodr::RoadHeader::HandTrafficRule::kRHT /* rule */,
                                     {} /* road_link */,
                                     {kRoadType0, kRoadType1, kRoadType2} /* road_type */,
                                     kReferenceGeometry /* reference_geometry */,
                                     kLanes /* lanes */};
};

TEST_F(GetRoadTypeSpeedPropertiesInRangeTest, CompleteRange) {
  const auto speed_properties = GetRoadTypeSpeedPropertiesInRange(kRoadHeader, 0, 100.);
  EXPECT_EQ(3., speed_properties.size());
  EXPECT_EQ(0., speed_properties[0].s_start);
  EXPECT_EQ(53., speed_properties[0].s_end);
  EXPECT_EQ(xodr::ConvertToMs(45., xodr::Unit::kMph), speed_properties[0].max);
  EXPECT_EQ(53., speed_properties[1].s_start);
  EXPECT_EQ(74., speed_properties[1].s_end);
  EXPECT_EQ(constants::kDefaultMaxSpeedLimit, speed_properties[1].max);
  EXPECT_EQ(74., speed_properties[2].s_start);
  EXPECT_EQ(100., speed_properties[2].s_end);
  EXPECT_EQ(xodr::ConvertToMs(120., xodr::Unit::kKph), speed_properties[2].max);
}

TEST_F(GetRoadTypeSpeedPropertiesInRangeTest, FirstRoadType) {
  const auto speed_properties = GetRoadTypeSpeedPropertiesInRange(kRoadHeader, 0., 53.);
  EXPECT_EQ(1., speed_properties.size());
  EXPECT_EQ(0., speed_properties[0].s_start);
  EXPECT_EQ(53., speed_properties[0].s_end);
  EXPECT_EQ(xodr::ConvertToMs(45., xodr::Unit::kMph), speed_properties[0].max);
}

TEST_F(GetRoadTypeSpeedPropertiesInRangeTest, SecondRoadType) {
  const auto speed_properties = GetRoadTypeSpeedPropertiesInRange(kRoadHeader, 53, 74.);
  EXPECT_EQ(1., speed_properties.size());
  EXPECT_EQ(53., speed_properties[0].s_start);
  EXPECT_EQ(74., speed_properties[0].s_end);
  EXPECT_EQ(constants::kDefaultMaxSpeedLimit, speed_properties[0].max);
}

TEST_F(GetRoadTypeSpeedPropertiesInRangeTest, ThirdRoadType) {
  const auto speed_properties = GetRoadTypeSpeedPropertiesInRange(kRoadHeader, 74., 100.);
  EXPECT_EQ(1., speed_properties.size());
  EXPECT_EQ(74., speed_properties[0].s_start);
  EXPECT_EQ(100., speed_properties[0].s_end);
  EXPECT_EQ(xodr::ConvertToMs(120., xodr::Unit::kKph), speed_properties[0].max);
}

struct GetLaneSpeedPropertiesTest : public ::testing::Test {
  const xodr::Lane::Speed kLaneSpeed0{0.0 /* sOffset */, 45. /* max */, xodr::Unit::kMph /* unit */};
  const xodr::Lane::Speed kLaneSpeed1{53. /* sOffset */, 3. /* max */, xodr::Unit::kMs /* unit */};
  const xodr::Lane::Speed kLaneSpeed2{74. /* sOffset */, 60. /* max */, xodr::Unit::kKph /* unit */};
  const xodr::Lane kLane{xodr::Lane::Id("1"),
                         xodr::Lane::Type::kDriving,
                         false,
                         {},
                         std::vector<xodr::LaneWidth>{{0., 1., 2., 3., 4.}},
                         {kLaneSpeed0, kLaneSpeed1, kLaneSpeed2}};
};

TEST_F(GetLaneSpeedPropertiesTest, CompleteRange) {
  const auto speed_properties = GetLaneSpeedProperties(kLane, 0, 100.);
  EXPECT_EQ(3., speed_properties.size());
  EXPECT_EQ(0., speed_properties[0].s_start);
  EXPECT_EQ(53., speed_properties[0].s_end);
  EXPECT_EQ(xodr::ConvertToMs(45., xodr::Unit::kMph), speed_properties[0].max);
  EXPECT_EQ(53., speed_properties[1].s_start);
  EXPECT_EQ(74., speed_properties[1].s_end);
  EXPECT_EQ(xodr::ConvertToMs(3., xodr::Unit::kMs), speed_properties[1].max);
  EXPECT_EQ(74., speed_properties[2].s_start);
  EXPECT_EQ(100., speed_properties[2].s_end);
  EXPECT_EQ(xodr::ConvertToMs(60., xodr::Unit::kKph), speed_properties[2].max);
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
