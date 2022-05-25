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
#include "maliput_malidrive/road_curve/lane_offset.h"

#include <optional>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

class LaneOffsetTest : public ::testing::Test {
 public:
  void SetUp() override {}

 protected:
  const double kTolerance{1e-15};
  const double kP0{0.25};
  const double kP1{50.};
  const CubicPolynomial kPreviousLaneOffsetAtLeft{1., 2., 3., 4., kP0, kP1, kTolerance};
  const CubicPolynomial kPreviousLaneOffsetAtRight{-1., -2., -3., -4., kP0, kP1, kTolerance};
  const CubicPolynomial kPreviousLaneWidth{5., 6., 7., 8., kP0, kP1, kTolerance};
  const CubicPolynomial kLaneWidth{9., 10., 11., 12., kP0, kP1, kTolerance};
  const CubicPolynomial kReferenceLineOffset{4., 3., 2., 1., kP0, kP1, kTolerance};
  const std::optional<LaneOffset::AdjacentLaneFunctions> kNoAdjacentLane{std::nullopt};
};

TEST_F(LaneOffsetTest, Constructor) {
  // Correct construction.
  EXPECT_NO_THROW(LaneOffset({{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidth}}, &kLaneWidth, &kReferenceLineOffset,
                             LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance));
  // Correct construction.
  EXPECT_NO_THROW(LaneOffset(kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane,
                             kP0, kP1, kTolerance));

  // Same p0 and p1.
  EXPECT_THROW(LaneOffset(kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane, kP0,
                          kP0, kTolerance),
               maliput::common::assertion_error);
  // p1 < p0
  EXPECT_THROW(LaneOffset(kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane, kP1,
                          kP0, kTolerance),
               maliput::common::assertion_error);
  // p0 is less than 0.
  EXPECT_THROW(LaneOffset(kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane, -kP1,
                          kP0, kTolerance),
               maliput::common::assertion_error);
  // p0 of kLaneWidth doesn't match.
  EXPECT_THROW(LaneOffset(kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane,
                          kP0 - 10 * kTolerance, kP1, kTolerance),
               maliput::common::assertion_error);
  // p1 of kLaneWidth doesn't match.
  EXPECT_THROW(LaneOffset(kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane, kP0,
                          kP1 - 10 * kTolerance, kTolerance),
               maliput::common::assertion_error);

  const CubicPolynomial kPreviousLaneOffsetWrongP0{1., 2., 3., 4., kP0 - 10 * kTolerance, kP1, kTolerance};
  const CubicPolynomial kPreviousLaneOffsetWrongP1{1., 2., 3., 4., kP0, kP1 - 10 * kTolerance, kTolerance};
  // p0 of kPreviousLaneOffsetWrongP0  doesn't match.
  EXPECT_THROW(LaneOffset({{&kPreviousLaneOffsetWrongP0, &kPreviousLaneWidth}}, &kLaneWidth, &kReferenceLineOffset,
                          LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);
  // p1 of kPreviousLaneOffsetWrongP1 doesn't match.
  EXPECT_THROW(LaneOffset({{&kPreviousLaneOffsetWrongP1, &kPreviousLaneWidth}}, &kLaneWidth, &kReferenceLineOffset,
                          LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);

  const CubicPolynomial kPreviousLaneWidthWrongP0{5., 6., 7., 8., kP0 - 10 * kTolerance, kP1, kTolerance};
  const CubicPolynomial kPreviousLaneWidthWrongP1{5., 6., 7., 8., kP0, kP1 - 10 * kTolerance, kTolerance};
  // p0 of kPreviousLaneWidthWrongP0  doesn't match.
  EXPECT_THROW(LaneOffset({{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidthWrongP0}}, &kLaneWidth,
                          &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);
  // p1 of kPreviousLaneWidthWrongP1 doesn't match.
  EXPECT_THROW(LaneOffset({{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidthWrongP1}}, &kLaneWidth,
                          &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);

  // Current lane width is nullptr.
  EXPECT_THROW(LaneOffset({{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidth}}, nullptr, &kReferenceLineOffset,
                          LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);
  // Previous lane width is nullptr.
  EXPECT_THROW(LaneOffset({{&kPreviousLaneOffsetAtLeft, nullptr}}, &kLaneWidth, &kReferenceLineOffset,
                          LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);
  // Previous lane offest is nullptr.
  EXPECT_THROW(LaneOffset({{nullptr, &kPreviousLaneWidth}}, &kLaneWidth, &kReferenceLineOffset,
                          LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);
  // Reference line offset function is nullptr.
  EXPECT_THROW(LaneOffset({{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidth}}, &kLaneWidth, nullptr,
                          LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance),
               maliput::common::assertion_error);
}

// Evaluates a LaneOffset.
// Computations in this test can be reproduced in python using
// the following snippet:
// @code{
// def f(a, b, c, d, p):
//     return a * p**3 + b * p**2 + c * p + d
// def f_dot(a, b, c, p):
//     return 3 * a * p**2 + 2 * b * p + c
// def f_dot_dot(a, b, p):
//     return 6 * a * p + 2 * b
// def sum_coeff(lhs, rhs):
//     return [ lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2], lhs[3] + rhs[3]]
// def lane_sign(right):
//     return -1 if (right is True) else 1;

// def evaluate_composed_offset(no_adjacent, at_right):

//   kA = 0
//   kB = 1
//   kC = 2
//   kD = 3

//   # Reference line offset
//   ref_line_offset = [0.,0.,0.,0.]
//   # Current Lane Width
//   p_c = [9.,10.,11.,12.]

//   # Previous Lane Offset
//   p_a = [1.,2.,3.,4.]
//   # Previous Lane Width
//   p_b = [5.,6.,7.,8.]

//   if(at_right):
//   # Previous Lane Offset
//     p_a = [-1.,-2.,-3.,-4.]
//   if(no_adjacent):
//     # Reference line offset
//     ref_line_offset = [4.,3.,2.,1.]
//     # Previous Lane Offset
//     p_a = [0.,0.,0.,0.]
//     # Previous Lane Width
//     p_b = [0.,0.,0.,0.]

//   print("f, f_dot, and f_dot_dot at p=0.5")
//   print(f(ref_line_offset[kA],ref_line_offset[kB],ref_line_offset[kC],ref_line_offset[kD], 0.5) +
//         f(p_a[kA],p_a[kB],p_a[kC],p_a[kD], 0.5) +
//         f(p_b[kA],p_b[kB],p_b[kC],p_b[kD], 0.5) * lane_sign(at_right) / 2. +
//         f(p_c[kA],p_c[kB],p_c[kC],p_c[kD], 0.5) * lane_sign(at_right) / 2.)
//   print(f_dot(ref_line_offset[kA],ref_line_offset[kB],ref_line_offset[kC], 0.5) +
//         f_dot(p_a[kA],p_a[kB],p_a[kC], 0.5) +
//         f_dot(p_b[kA],p_b[kB],p_b[kC], 0.5) * lane_sign(at_right) / 2. +
//         f_dot(p_c[kA],p_c[kB],p_c[kC], 0.5) * lane_sign(at_right) / 2.)
//   print(f_dot_dot(ref_line_offset[kA],ref_line_offset[kB], 0.5) +
//         f_dot_dot(p_a[kA],p_a[kB], 0.5) +
//         f_dot_dot(p_b[kA],p_b[kB], 0.5) * lane_sign(at_right) / 2. +
//         f_dot_dot(p_c[kA],p_c[kB], 0.5) * lane_sign(at_right) / 2.)

//   print("\nf, f_dot, and f_dot_dot at p=20.5")
//   print(f(ref_line_offset[kA],ref_line_offset[kB],ref_line_offset[kC],ref_line_offset[kD], 20.5) +
//         f(p_a[kA],p_a[kB],p_a[kC],p_a[kD], 20.5) +
//         f(p_b[kA],p_b[kB],p_b[kC],p_b[kD], 20.5) * lane_sign(at_right) / 2. +
//         f(p_c[kA],p_c[kB],p_c[kC],p_c[kD], 20.5) * lane_sign(at_right) / 2.)
//   print(f_dot(ref_line_offset[kA],ref_line_offset[kB],ref_line_offset[kC], 20.5) +
//         f_dot(p_a[kA],p_a[kB],p_a[kC], 20.5) +
//         f_dot(p_b[kA],p_b[kB],p_b[kC], 20.5) * lane_sign(at_right) / 2. +
//         f_dot(p_c[kA],p_c[kB],p_c[kC], 20.5) * lane_sign(at_right) / 2.)
//   print(f_dot_dot(ref_line_offset[kA],ref_line_offset[kB], 20.5) +
//         f_dot_dot(p_a[kA],p_a[kB], 20.5) +
//         f_dot_dot(p_b[kA],p_b[kB], 20.5) * lane_sign(at_right) / 2. +
//         f_dot_dot(p_c[kA],p_c[kB], 20.5) * lane_sign(at_right) / 2.)

//   print("\nf, f_dot, and f_dot_dot at p=50.")
//   print(f(ref_line_offset[kA],ref_line_offset[kB],ref_line_offset[kC],ref_line_offset[kD], 50.) +
//         f(p_a[kA],p_a[kB],p_a[kC],p_a[kD], 50.) +
//         f(p_b[kA],p_b[kB],p_b[kC],p_b[kD], 50.) * lane_sign(at_right) / 2. +
//         f(p_c[kA],p_c[kB],p_c[kC],p_c[kD], 50.) * lane_sign(at_right) / 2.)
//   print(f_dot(ref_line_offset[kA],ref_line_offset[kB],ref_line_offset[kC], 50.) +
//         f_dot(p_a[kA],p_a[kB],p_a[kC], 50.) +
//         f_dot(p_b[kA],p_b[kB],p_b[kC], 50.) * lane_sign(at_right) / 2. +
//         f_dot(p_c[kA],p_c[kB],p_c[kC], 50.) * lane_sign(at_right) / 2.)
//   print(f_dot_dot(ref_line_offset[kA],ref_line_offset[kB], 50.) +
//         f_dot_dot(p_a[kA],p_a[kB], 50.) +
//         f_dot_dot(p_b[kA],p_b[kB], 50.) * lane_sign(at_right) / 2. +
//         f_dot_dot(p_c[kA],p_c[kB], 50.) * lane_sign(at_right) / 2.)

// evaluate_composed_offset(no_adjacent=False, at_right=False)
// } @endcode
TEST_F(LaneOffsetTest, FunctionApi) {
  // At left with inner lanes.
  {
    const LaneOffset dut{{{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidth}},
                         &kLaneWidth,
                         &kReferenceLineOffset,
                         LaneOffset::kAtLeftFromCenterLane,
                         kP0,
                         kP1,
                         kTolerance};
    EXPECT_NEAR(dut.f(0.5), 23.5, kTolerance);
    EXPECT_NEAR(dut.f_dot(0.5), 28.0, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(0.5), 44.0, kTolerance);

    EXPECT_NEAR(dut.f(20.5), 73383.5, kTolerance);
    EXPECT_NEAR(dut.f_dot(20.5), 10508.0, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(20.5), 1004.0, kTolerance);

    EXPECT_NEAR(dut.f(50.), 1025614.0, kTolerance);
    EXPECT_NEAR(dut.f_dot(50.), 61012.0, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(50.), 2420.0, kTolerance);
  }
  // At right with inner lanes.
  {
    const LaneOffset dut{{{&kPreviousLaneOffsetAtRight, &kPreviousLaneWidth}},
                         &kLaneWidth,
                         &kReferenceLineOffset,
                         LaneOffset::kAtRightFromCenterLane,
                         kP0,
                         kP1,
                         kTolerance};
    EXPECT_NEAR(dut.f(0.5), -23.5, kTolerance);
    EXPECT_NEAR(dut.f_dot(0.5), -28.0, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(0.5), -44.0, kTolerance);

    EXPECT_NEAR(dut.f(20.5), -73383.5, kTolerance);
    EXPECT_NEAR(dut.f_dot(20.5), -10508.0, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(20.5), -1004.0, kTolerance);

    EXPECT_NEAR(dut.f(50.), -1025614.0, kTolerance);
    EXPECT_NEAR(dut.f_dot(50.), -61012.0, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(50.), -2420.0, kTolerance);
  }
  // At left without inner lanes.
  {
    const LaneOffset dut{
        kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtLeftFromCenterLane, kP0, kP1, kTolerance};
    EXPECT_NEAR(dut.f(0.5), 13.8125, kTolerance);
    EXPECT_NEAR(dut.f_dot(0.5), 21.875, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(0.5), 41.5, kTolerance);

    EXPECT_NEAR(dut.f(20.5), 76751.3125, kTolerance);
    EXPECT_NEAR(dut.f_dot(20.5), 11051.875, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(20.5), 1061.5, kTolerance);

    EXPECT_NEAR(dut.f(50.), 1082882.0, kTolerance);
    EXPECT_NEAR(dut.f_dot(50.), 64557.5, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(50.), 2566.0, kTolerance);
  }
  // At right without inner lanes.
  {
    const LaneOffset dut{
        kNoAdjacentLane, &kLaneWidth, &kReferenceLineOffset, LaneOffset::kAtRightFromCenterLane, kP0, kP1, kTolerance};
    EXPECT_NEAR(dut.f(0.5), -7.3125, kTolerance);
    EXPECT_NEAR(dut.f_dot(0.5), -5.875, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(0.5), -5.5, kTolerance);

    EXPECT_NEAR(dut.f(20.5), -5224.8125, kTolerance);
    EXPECT_NEAR(dut.f_dot(20.5), -715.875, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(20.5), -65.5, kTolerance);

    EXPECT_NEAR(dut.f(50.), -67680.0, kTolerance);
    EXPECT_NEAR(dut.f_dot(50.), -3953.5, kTolerance);
    EXPECT_NEAR(dut.f_dot_dot(50.), -154.0, kTolerance);
  }
}

TEST_F(LaneOffsetTest, Range) {
  const LaneOffset dut{{{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidth}},
                       &kLaneWidth,
                       &kReferenceLineOffset,
                       LaneOffset::kAtLeftFromCenterLane,
                       kP0,
                       kP1,
                       kTolerance};

  EXPECT_THROW(dut.f(kP1 + 1.), maliput::common::assertion_error);
  EXPECT_THROW(dut.f_dot(kP1 + 1.), maliput::common::assertion_error);
  EXPECT_THROW(dut.f_dot_dot(kP1 + 1.), maliput::common::assertion_error);
}

TEST_F(LaneOffsetTest, IsG1Contiguous) {
  const LaneOffset dut{{{&kPreviousLaneOffsetAtLeft, &kPreviousLaneWidth}},
                       &kLaneWidth,
                       &kReferenceLineOffset,
                       LaneOffset::kAtLeftFromCenterLane,
                       kP0,
                       kP1,
                       kTolerance};
  EXPECT_TRUE(dut.IsG1Contiguous());
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
