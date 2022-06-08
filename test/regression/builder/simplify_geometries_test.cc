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
