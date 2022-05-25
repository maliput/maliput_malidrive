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
