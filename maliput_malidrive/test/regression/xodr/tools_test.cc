// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/tools.h"

#include <cmath>

#include <gtest/gtest.h>

#include <maliput/common/assertion_error.h>
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/lateral_profile.h"

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(GetDistanceBetweenGeometries, LineTypeGeometries) {
  const Geometry kGeometry0{0. /* s_0 */,     {20. /* x */, 0. /* y */},        M_PI / 2 /* orientation */,
                            20. /* length */, Geometry::Type::kLine /* Type */, {Geometry::Line{}} /* description */};
  const Geometry kGeometry1{20. /* s_0 */,    {19. /* x */, 21. /* y */},       M_PI /* orientation */,
                            40. /* length */, Geometry::Type::kLine /* Type */, {Geometry::Line{}} /* description */};
  const Geometry kGeometry2{60. /* s_0 */,    {-19 /* x */, 21. /* y */},       -M_PI / 2 /* orientation */,
                            40. /* length */, Geometry::Type::kLine /* Type */, {Geometry::Line{}} /* description */};
  const Geometry kGeometry3{100. /* s_0 */,   {-15. /* x */, -25. /* y */},     0. /* orientation */,
                            40. /* length */, Geometry::Type::kLine /* Type */, {Geometry::Line{}} /* description */};
  const Geometry kGeometry4{140. /* s_0 */,   {14.9 /* x */, -25.1 /* y */},    M_PI / 2 /* orientation */,
                            20. /* length */, Geometry::Type::kLine /* Type */, {Geometry::Line{}} /* description */};

  const double kTolerance{1e-14};
  EXPECT_NEAR(sqrt(2), GetDistanceBetweenGeometries(kGeometry0, kGeometry1), kTolerance);
  EXPECT_NEAR(2., GetDistanceBetweenGeometries(kGeometry1, kGeometry2), kTolerance);
  EXPECT_NEAR(sqrt(52), GetDistanceBetweenGeometries(kGeometry2, kGeometry3), kTolerance);
  EXPECT_NEAR(sqrt(102.02), GetDistanceBetweenGeometries(kGeometry3, kGeometry4), kTolerance);
}

GTEST_TEST(GetDistanceBetweenGeometries, ArcTypeGeometries) {
  const Geometry kGeometry0{0. /* s_0 */,
                            {5. /* x */, -6. /* y */},
                            0. /* orientation */,
                            6.283185307179586 /* length */,
                            Geometry::Type::kArc /* Type */,
                            {Geometry::Arc{1. / 2.}} /* description */};
  const Geometry kGeometry1{7. /* s_0 */,
                            {4. /* x */, -2. /* y */},
                            M_PI /* orientation */,
                            14.137166941154069 /* length */,
                            Geometry::Type::kArc /* Type */,
                            {Geometry::Arc{1. / 9.}} /* description */};
  const Geometry kGeometry2{18. /* s_0 */,
                            {-4 /* x */, -11. /* y */},
                            -M_PI / 2 /* orientation */,
                            12.566370614359172 /* length */,
                            Geometry::Type::kArc /* Type */,
                            {Geometry::Arc{-1. / 4.}} /* description */};
  const Geometry kGeometry3{31. /* s_0 */,
                            {-12. /* x */, -9. /* y */},
                            M_PI / 2 /* orientation */,
                            26.703537555513243 /* length */,
                            Geometry::Type::kArc /* Type */,
                            {Geometry::Arc{-1. / 17.}} /* description */};
  const Geometry kGeometry4{58. /* s_0 */,
                            {5. /* x */, 6. /* y */},
                            0. /* orientation */,
                            6.283185307179586 /* length */,
                            Geometry::Type::kArc /* Type */,
                            {Geometry::Arc{1. / 2.}} /* description */};
  const double kTolerance{1e-14};
  EXPECT_NEAR(1., GetDistanceBetweenGeometries(kGeometry0, kGeometry1), kTolerance);
  EXPECT_NEAR(1., GetDistanceBetweenGeometries(kGeometry1, kGeometry2), kTolerance);
  EXPECT_NEAR(2., GetDistanceBetweenGeometries(kGeometry2, kGeometry3), kTolerance);
  EXPECT_NEAR(2., GetDistanceBetweenGeometries(kGeometry3, kGeometry4), kTolerance);
}

GTEST_TEST(GetDistanceBetweenFunctions, Elevation) {
  const double kTolerance{1e-14};
  const ElevationProfile::Elevation lhs{/* s_0 */ 0., /* a */ 1., /* b */ 2., /* c */ 3., /* d */ 4.};

  {
    const ElevationProfile::Elevation rhs{/* s_0 */ 1., /* a */ 10., /* b */ 0., /* c */ 0., /* d */ 0.};
    EXPECT_NEAR(0., GetDistanceBetweenFunctions<ElevationProfile::Elevation>(lhs, rhs), kTolerance);
  }
  {
    const ElevationProfile::Elevation rhs{/* s_0 */ 1., /* a */ 11., /* b */ 0., /* c */ 0., /* d */ 0.};
    EXPECT_NEAR(1., GetDistanceBetweenFunctions<ElevationProfile::Elevation>(lhs, rhs), kTolerance);
  }
}

GTEST_TEST(GetDistanceBetweenFunctions, Superelevation) {
  const double kTolerance{1e-14};
  const LateralProfile::Superelevation lhs{/* s_0 */ 0., /* a */ 1., /* b */ 2., /* c */ 3., /* d */ 4.};

  {
    const LateralProfile::Superelevation rhs{/* s_0 */ 1., /* a */ 10., /* b */ 0., /* c */ 0., /* d */ 0.};
    EXPECT_NEAR(0., GetDistanceBetweenFunctions<LateralProfile::Superelevation>(lhs, rhs), kTolerance);
  }
  {
    const LateralProfile::Superelevation rhs{/* s_0 */ 1., /* a */ 11., /* b */ 0., /* c */ 0., /* d */ 0.};
    EXPECT_NEAR(1., GetDistanceBetweenFunctions<LateralProfile::Superelevation>(lhs, rhs), kTolerance);
  }
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
