// BSD 3-Clause License
//
// Copyright (c) 2025, Woven Planet. All rights reserved.
// Copyright (c) 2025, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/xodr/object/outlines.h"

#include <gtest/gtest.h>
#include <maliput/common/error.h>

namespace malidrive {
namespace xodr {
namespace object {
namespace test {
namespace {

GTEST_TEST(Outlines, EqualityOperator) {
  const Outline kOutline{false, Outline::FillType::kCobble, object::Outline::Id("test"), Lane::Type::kNone, false, {},
                         {}};
  const Outlines kOutlines{{kOutline}};

  Outlines outlines = kOutlines;

  EXPECT_EQ(kOutlines, outlines);
  // Test inequality
  outlines.outlines[0].closed = true;
  EXPECT_NE(kOutlines, outlines);
  outlines.outlines[0].closed = false;
  outlines.outlines[0].fill_type = Outline::FillType::kConcrete;
  EXPECT_NE(kOutlines, outlines);
  outlines.outlines[0].fill_type = Outline::FillType::kCobble;
  outlines.outlines[0].id = object::Outline::Id("test2");
  EXPECT_NE(kOutlines, outlines);
  outlines.outlines[0].id = object::Outline::Id("test");
  outlines.outlines[0].lane_type = Lane::Type::kDriving;
  EXPECT_NE(kOutlines, outlines);
  outlines.outlines[0].lane_type = Lane::Type::kNone;
  outlines.outlines[0].outer = true;
  EXPECT_NE(kOutlines, outlines);
}

GTEST_TEST(Outlines_CornerLocal, EqualityOperator) {
  const CornerLocal kCornerLocal{1., object::CornerLocal::Id("test"), 3., 4., 5.};
  CornerLocal corner_local = kCornerLocal;

  EXPECT_EQ(kCornerLocal, corner_local);
  // Test inequality
  corner_local.height = 2.;
  EXPECT_NE(kCornerLocal, corner_local);
  corner_local.height = 1.;
  corner_local.id = object::CornerLocal::Id("test2");
  EXPECT_NE(kCornerLocal, corner_local);
  corner_local.id = object::CornerLocal::Id("test");
  corner_local.u = 4.;
  EXPECT_NE(kCornerLocal, corner_local);
  corner_local.u = 3.;
  corner_local.v = 3.;
  EXPECT_NE(kCornerLocal, corner_local);
  corner_local.v = 2.;
  corner_local.z = 3.;
  EXPECT_NE(kCornerLocal, corner_local);
}

GTEST_TEST(Outlines_CornerRoad, EqualityOperator) {
  const CornerRoad kCornerRoad{1., 2., object::CornerRoad::Id("test"), 4., 5.};
  CornerRoad corner_road = kCornerRoad;

  EXPECT_EQ(kCornerRoad, corner_road);
  // Test inequality
  corner_road.dz = 2.;
  EXPECT_NE(kCornerRoad, corner_road);
  corner_road.dz = 1.;
  corner_road.height = 3.;
  EXPECT_NE(kCornerRoad, corner_road);
  corner_road.height = 2.;
  corner_road.id = object::CornerRoad::Id("test2");
  EXPECT_NE(kCornerRoad, corner_road);
  corner_road.id = object::CornerRoad::Id("test");
  corner_road.s = 5.;
  EXPECT_NE(kCornerRoad, corner_road);
  corner_road.s = 4.;
  corner_road.t = 6.;
  EXPECT_NE(kCornerRoad, corner_road);
}

}  // namespace
}  // namespace test
}  // namespace object
}  // namespace xodr
}  // namespace malidrive
