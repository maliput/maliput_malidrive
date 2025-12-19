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
#include "maliput_malidrive/xodr/object/repeat.h"

#include <gtest/gtest.h>
#include <maliput/common/error.h>

namespace malidrive {
namespace xodr {
namespace object {
namespace test {
namespace {

GTEST_TEST(Repeat, EqualityOperator) {
  const Repeat kRepeat{true, 1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15.};
  Repeat repeat = kRepeat;

  EXPECT_EQ(kRepeat, repeat);
  // Test inequality
  repeat.detach_from_reference_line = false;
  EXPECT_NE(kRepeat, repeat);
  repeat.detach_from_reference_line = true;
  repeat.distance = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.distance = 1.;
  repeat.height_end = 3.;
  EXPECT_NE(kRepeat, repeat);
  repeat.height_end = 2.;
  repeat.height_start = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.height_start = 3.;
  repeat.length_end = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.length_end = 4.;
  repeat.length_start = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.length_start = 5.;
  repeat.length = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.length = 6.;
  repeat.radius_end = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.radius_end = 7.;
  repeat.radius_start = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.radius_start = 8.;
  repeat.s = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.s = 9.;
  repeat.t_end = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.t_end = 10.;
  repeat.t_start = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.t_start = 11.;
  repeat.width_end = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.width_end = 12.;
  repeat.width_start = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.width_start = 13.;
  repeat.z_offset_end = 2.;
  EXPECT_NE(kRepeat, repeat);
  repeat.z_offset_end = 14.;
  repeat.z_offset_start = 2.;
  EXPECT_NE(kRepeat, repeat);
}

}  // namespace
}  // namespace test
}  // namespace object
}  // namespace xodr
}  // namespace malidrive
