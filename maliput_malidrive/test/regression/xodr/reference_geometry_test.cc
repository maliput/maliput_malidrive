// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/reference_geometry.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(ReferenceGeometry, EqualityOperator) {
  const ElevationProfile kElevationProfile{{{1.23 /* s_0 */, 523.2 /* a */, 83.27 /* b */, 0.77 /* c */, 100. /* d */},
                                            {1.215 /* s_0 */, 1.2 /* a */, 35.27 /* b */, 0.2 /* c */, 564. /* d */}}};
  const LateralProfile kLateralProfile{{{5.23 /* s_0 */, 1523.2 /* a */, 883.27 /* b */, 10.77 /* c */, 156. /* d */},
                                        {6452. /* s_0 */, 15.5 /* a */, 1.27 /* b */, 5.2 /* c */, 56478. /* d */}}};
  const PlanView kPlanView{{{1.23 /* s_0 */,
                             {523.2 /* x */, 83.27 /* y */},
                             0.77 /* orientation */,
                             100. /* length */,
                             Geometry::Type::kLine /* Type */,
                             {Geometry::Line{}} /* description */},
                            {1.23 /* s_0 */,
                             {523.2 /* x */, 83.27 /* y */},
                             0.77 /* orientation */,
                             100. /* length */,
                             Geometry::Type::kLine /* Type */,
                             {Geometry::Line{}} /* description */}}};
  const ReferenceGeometry kReferenceGeometry{kPlanView, kElevationProfile, kLateralProfile};
  ReferenceGeometry reference_geometry = kReferenceGeometry;

  EXPECT_EQ(kReferenceGeometry, reference_geometry);
  reference_geometry.plan_view.geometries[0].length = 568.5;
  EXPECT_NE(kReferenceGeometry, reference_geometry);
  reference_geometry.plan_view.geometries[0].length = 100.;
  reference_geometry.elevation_profile.elevations[1].c = 89.;
  EXPECT_NE(kReferenceGeometry, reference_geometry);
  reference_geometry.elevation_profile.elevations[1].c = 0.2;
  reference_geometry.lateral_profile.superelevations[1].s_0 = 15.;
  EXPECT_NE(kReferenceGeometry, reference_geometry);
  reference_geometry.lateral_profile.superelevations[1].s_0 = 6452.;
  EXPECT_EQ(kReferenceGeometry, reference_geometry);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
