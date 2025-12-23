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
#include "maliput_malidrive/xodr/object/skeleton.h"

#include <gtest/gtest.h>
#include <maliput/common/error.h>

namespace malidrive {
namespace xodr {
namespace object {
namespace test {
namespace {

GTEST_TEST(VertexRoad, EqualityOperator) {
  const VertexRoad kVertexRoad{1., 2, false, 3., 4., 5.};
  VertexRoad vertex_road = kVertexRoad;

  EXPECT_EQ(kVertexRoad, vertex_road);
  // Test inequality
  vertex_road.dz = 2.;
  EXPECT_NE(kVertexRoad, vertex_road);
  vertex_road.dz = 1.;
  vertex_road.id = 3;
  EXPECT_NE(kVertexRoad, vertex_road);
  vertex_road.id = 2;
  vertex_road.s = 5.;
  EXPECT_NE(kVertexRoad, vertex_road);
  vertex_road.s = 4.;
  vertex_road.t = 6.;
  EXPECT_NE(kVertexRoad, vertex_road);
  vertex_road.t = 5.;
  vertex_road.radius = 6.;
  EXPECT_NE(kVertexRoad, vertex_road);
  vertex_road.radius = 3.;
  vertex_road.intersection_point = true;
  EXPECT_NE(kVertexRoad, vertex_road);
}

GTEST_TEST(VertexLocal, EqualityOperator) {
  const VertexLocal kVertexLocal{1, false, 2., 3., 4., 5.};

  VertexLocal vertex_local = kVertexLocal;

  EXPECT_EQ(kVertexLocal, vertex_local);
  // Test inequality
  vertex_local.intersection_point = true;
  EXPECT_NE(kVertexLocal, vertex_local);
  vertex_local.intersection_point = false;
  vertex_local.radius = 3.;
  EXPECT_NE(kVertexLocal, vertex_local);
  vertex_local.radius = 2.;
  vertex_local.u = 4.;
  EXPECT_NE(kVertexLocal, vertex_local);
  vertex_local.u = 3.;
  vertex_local.v = 5.;
  EXPECT_NE(kVertexLocal, vertex_local);
  vertex_local.v = 4.;
  vertex_local.z = 4.;
  EXPECT_NE(kVertexLocal, vertex_local);
  vertex_local.z = 5.;
  vertex_local.id = 6;
  EXPECT_NE(kVertexLocal, vertex_local);
}

GTEST_TEST(Polyline, EqualityOperator) {
  const Polyline kPolyline{1, {{1., 2, false, 3., 4., 5.}}, {{1, false, 2., 3., 4., 5.}}};
  Polyline polyline = kPolyline;

  EXPECT_EQ(kPolyline, polyline);
  // Test inequality
  polyline.id = 2;
  EXPECT_NE(kPolyline, polyline);
}

}  // namespace
}  // namespace test
}  // namespace object
}  // namespace xodr
}  // namespace malidrive