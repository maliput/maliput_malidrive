// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/simplify_geometries.h"

#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput_malidrive/test_utilities/xodr_testing_map_descriptions.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/geometry.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

class SimplifyGeometriesTest : public ::testing::Test {
 protected:
  const std::optional<double> kLoaderNoToleranceCheck{};
  const double kTolerance{1e-6};
  const xodr::RoadHeader::Id kRoadId{"1"};
};

TEST_F(SimplifyGeometriesTest, NoSimplification) {
  const auto db_manager = xodr::LoadDataBaseFromStr(malidrive::test::kXodrSingleGeometry, {kLoaderNoToleranceCheck});
  const std::vector<xodr::Geometry>& parsed_geometries =
      db_manager->GetRoadHeaders().at(kRoadId).reference_geometry.plan_view.geometries;
  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      db_manager->GetGeometriesToSimplify(kTolerance);
  ASSERT_TRUE(geometries_to_simplify.empty());

  const std::vector<xodr::Geometry> simplified_geometries =
      SimplifyGeometries(parsed_geometries, geometries_to_simplify);
  EXPECT_EQ(1, simplified_geometries.size());
  EXPECT_EQ(parsed_geometries.front(), simplified_geometries.front());
}

TEST_F(SimplifyGeometriesTest, NoSimplificationLineAndArc) {
  const auto db_manager =
      xodr::LoadDataBaseFromStr(malidrive::test::kXodrLineAndArcGeometry, {kLoaderNoToleranceCheck});
  const std::vector<xodr::Geometry>& parsed_geometries =
      db_manager->GetRoadHeaders().at(kRoadId).reference_geometry.plan_view.geometries;
  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      db_manager->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(0, geometries_to_simplify.size());

  const std::vector<xodr::Geometry> simplified_geometries =
      SimplifyGeometries(parsed_geometries, geometries_to_simplify);
  EXPECT_EQ(2, simplified_geometries.size());
  EXPECT_EQ(parsed_geometries[0], simplified_geometries[0]);
  EXPECT_EQ(parsed_geometries[1], simplified_geometries[1]);
}

TEST_F(SimplifyGeometriesTest, SimplifiesLines) {
  const auto db_manager =
      xodr::LoadDataBaseFromStr(malidrive::test::kXodrWithLinesToBeSimplified, {kLoaderNoToleranceCheck});
  const std::vector<xodr::Geometry>& parsed_geometries =
      db_manager->GetRoadHeaders().at(kRoadId).reference_geometry.plan_view.geometries;
  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      db_manager->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, geometries_to_simplify.size());

  const std::vector<xodr::Geometry> simplified_geometries =
      SimplifyGeometries(parsed_geometries, geometries_to_simplify);
  EXPECT_EQ(1, simplified_geometries.size());
  EXPECT_EQ(parsed_geometries[0].s_0, simplified_geometries[0].s_0);
  EXPECT_EQ(parsed_geometries[0].start_point, simplified_geometries[0].start_point);
  EXPECT_EQ(parsed_geometries[0].orientation, simplified_geometries[0].orientation);
  EXPECT_NEAR(parsed_geometries[0].length + parsed_geometries[1].length + parsed_geometries[2].length,
              simplified_geometries[0].length, kTolerance);
  EXPECT_EQ(parsed_geometries[0].description, simplified_geometries[0].description);
}

TEST_F(SimplifyGeometriesTest, SimplifiesArcs) {
  const auto db_manager =
      xodr::LoadDataBaseFromStr(malidrive::test::kXodrWithArcsToBeSimplified, {kLoaderNoToleranceCheck});
  const std::vector<xodr::Geometry>& parsed_geometries =
      db_manager->GetRoadHeaders().at(kRoadId).reference_geometry.plan_view.geometries;
  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      db_manager->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, geometries_to_simplify.size());

  const std::vector<xodr::Geometry> simplified_geometries =
      SimplifyGeometries(parsed_geometries, geometries_to_simplify);
  EXPECT_EQ(1, simplified_geometries.size());
  EXPECT_EQ(parsed_geometries[0].s_0, simplified_geometries[0].s_0);
  EXPECT_EQ(parsed_geometries[0].start_point, simplified_geometries[0].start_point);
  EXPECT_EQ(parsed_geometries[0].orientation, simplified_geometries[0].orientation);
  EXPECT_NEAR(parsed_geometries[0].length + parsed_geometries[1].length + parsed_geometries[2].length,
              simplified_geometries[0].length, kTolerance);
  EXPECT_EQ(parsed_geometries[0].description, simplified_geometries[0].description);
}

TEST_F(SimplifyGeometriesTest, SimplifiesLinesBetweenArcs) {
  const auto db_manager =
      xodr::LoadDataBaseFromStr(malidrive::test::kXodrCombinedLinesWithArcs, {kLoaderNoToleranceCheck});
  const std::vector<xodr::Geometry>& parsed_geometries =
      db_manager->GetRoadHeaders().at(kRoadId).reference_geometry.plan_view.geometries;
  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      db_manager->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, geometries_to_simplify.size());

  const std::vector<xodr::Geometry> simplified_geometries =
      SimplifyGeometries(parsed_geometries, geometries_to_simplify);
  EXPECT_EQ(3, simplified_geometries.size());
  EXPECT_EQ(parsed_geometries[0], simplified_geometries[0]);

  EXPECT_EQ(parsed_geometries[1].s_0, simplified_geometries[1].s_0);
  EXPECT_EQ(parsed_geometries[1].start_point, simplified_geometries[1].start_point);
  EXPECT_EQ(parsed_geometries[1].orientation, simplified_geometries[1].orientation);
  EXPECT_NEAR(parsed_geometries[1].length + parsed_geometries[2].length, simplified_geometries[1].length, kTolerance);
  EXPECT_EQ(parsed_geometries[1].description, simplified_geometries[1].description);

  EXPECT_EQ(parsed_geometries[3].s_0, simplified_geometries[2].s_0);
}

TEST_F(SimplifyGeometriesTest, SimplifiesArcsBetweenLines) {
  const auto db_manager =
      xodr::LoadDataBaseFromStr(malidrive::test::kXodrCombinedArcsWithLines, {kLoaderNoToleranceCheck});
  const std::vector<xodr::Geometry>& parsed_geometries =
      db_manager->GetRoadHeaders().at(kRoadId).reference_geometry.plan_view.geometries;
  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      db_manager->GetGeometriesToSimplify(kTolerance);
  ASSERT_EQ(1, geometries_to_simplify.size());

  const std::vector<xodr::Geometry> simplified_geometries =
      SimplifyGeometries(parsed_geometries, geometries_to_simplify);
  EXPECT_EQ(3, simplified_geometries.size());
  EXPECT_EQ(parsed_geometries[0], simplified_geometries[0]);

  EXPECT_EQ(parsed_geometries[1].s_0, simplified_geometries[1].s_0);
  EXPECT_EQ(parsed_geometries[1].start_point, simplified_geometries[1].start_point);
  EXPECT_EQ(parsed_geometries[1].orientation, simplified_geometries[1].orientation);
  EXPECT_NEAR(parsed_geometries[1].length + parsed_geometries[2].length, simplified_geometries[1].length, kTolerance);
  EXPECT_EQ(parsed_geometries[1].description, simplified_geometries[1].description);

  EXPECT_EQ(parsed_geometries[3].s_0, simplified_geometries[2].s_0);
}
// @}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
