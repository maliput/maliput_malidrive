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
#include "maliput_malidrive/road_curve/ground_curve.h"

#include <gtest/gtest.h>
#include <maliput/test_utilities/maliput_math_compare.h>

#include "maliput_malidrive/test_utilities/ground_curve_stub.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

using maliput::math::test::CompareVectors;

// Exercises the interface.
GTEST_TEST(GrounCurveTest, InterfaceTest) {
  const maliput::math::Vector2 kG{1., 2.};
  const maliput::math::Vector2 kGDot{3., 4.};
  const double kHeading{5.};
  const double kHeadingDot{6.};
  const double kGInverse{7.};
  const double kArcLength{8.};
  const double kLinearTolerance{0.1};
  const double kP0{9.};
  const double kP1{10.};
  const bool kIsG1Contiguous{true};

  const GroundCurveStub dut(kG, kGDot, kHeading, kHeadingDot, kGInverse, kArcLength, kLinearTolerance, kP0, kP1,
                            kIsG1Contiguous);

  EXPECT_TRUE(CompareVectors(dut.G(kP0), kG));
  EXPECT_TRUE(CompareVectors(dut.GDot(kP0), kGDot));
  EXPECT_EQ(dut.Heading(kP0), kHeading);
  EXPECT_EQ(dut.HeadingDot(kP0), kHeadingDot);
  EXPECT_EQ(dut.GInverse(kG), kGInverse);
  EXPECT_EQ(dut.ArcLength(), kArcLength);
  EXPECT_EQ(dut.linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(dut.p0(), kP0);
  EXPECT_EQ(dut.p1(), kP1);
  EXPECT_EQ(dut.IsG1Contiguous(), kIsG1Contiguous);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
