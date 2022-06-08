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
