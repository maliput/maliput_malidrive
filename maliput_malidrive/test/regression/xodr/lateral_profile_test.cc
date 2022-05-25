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
#include "maliput_malidrive/xodr/lateral_profile.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Superelevation, EqualityOperator) {
  const LateralProfile::Superelevation kSuperelevation{1.1 /* s0 */, 2.2 /* a */, 3.3 /* b */, 4.4 /* c */,
                                                       5.5 /* d */};
  LateralProfile::Superelevation superelevation = kSuperelevation;

  EXPECT_EQ(kSuperelevation, superelevation);
  superelevation.s_0 = 5.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.s_0 = 1.1;
  superelevation.a = 5.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.a = 2.2;
  superelevation.b = 10.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.b = 3.3;
  superelevation.c = 10.;
  EXPECT_NE(kSuperelevation, superelevation);
  superelevation.c = 4.4;
  superelevation.d = 10.;
  EXPECT_NE(kSuperelevation, superelevation);
}

GTEST_TEST(LateralProfile, EqualityOperator) {
  const LateralProfile kLateralProfile{{{1.23 /* s_0 */, 523.2 /* a */, 83.27 /* b */, 0.77 /* c */, 100. /* d */},
                                        {1.215 /* s_0 */, 1.2 /* a */, 35.27 /* b */, 0.2 /* c */, 564. /* d */}}};
  LateralProfile lateral_profile = kLateralProfile;

  EXPECT_EQ(kLateralProfile, lateral_profile);
  lateral_profile.superelevations[0].c = 568.5;
  EXPECT_NE(kLateralProfile, lateral_profile);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
