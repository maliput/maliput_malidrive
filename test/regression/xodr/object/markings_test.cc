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
#include "maliput_malidrive/xodr/object/markings.h"

#include <gtest/gtest.h>
#include <maliput/common/error.h>

namespace malidrive {
namespace xodr {
namespace object {
namespace test {
namespace {

GTEST_TEST(CornerRoad, EqualityOperator) {
  const CornerReference kCornerReference{1};
  CornerReference corner_reference = kCornerReference;

  EXPECT_EQ(kCornerReference, corner_reference);
  // Test inequality
  corner_reference.id = 2;
  EXPECT_NE(kCornerReference, corner_reference);
}

GTEST_TEST(Marking, EqualityOperator) {
  const Marking kMarking{
      .color = Color::kBlack,
      .line_length = 1.,
      .side = Marking::Side::kLeft,
      .space_length = 2.,
      .start_offset = 3.,
      .stop_offset = 4.,
      .weight = LaneRoadMark::Weight::kBold,
      .width = 5.,
      .z_offset = 6.,
  };
  Marking marking = kMarking;

  EXPECT_EQ(kMarking, marking);
  // Test inequality
  marking.color = Color::kWhite;
  EXPECT_NE(kMarking, marking);
  marking.color = Color::kBlack;
  marking.line_length = 2.;
  EXPECT_NE(kMarking, marking);
  marking.line_length = 1.;
  marking.side = Marking::Side::kRight;
  EXPECT_NE(kMarking, marking);
  marking.side = Marking::Side::kLeft;
  marking.space_length = 3.;
  EXPECT_NE(kMarking, marking);
  marking.space_length = 2.;
  marking.start_offset = 4.;
  EXPECT_NE(kMarking, marking);
  marking.start_offset = 3.;
  marking.stop_offset = 5.;
  EXPECT_NE(kMarking, marking);
  marking.stop_offset = 4.;
  marking.weight = LaneRoadMark::Weight::kStandard;
  EXPECT_NE(kMarking, marking);
  marking.weight = LaneRoadMark::Weight::kBold;
  marking.width = 6.;
  EXPECT_NE(kMarking, marking);
  marking.width = 5.;
  marking.z_offset = 7.;
  EXPECT_NE(kMarking, marking);
}

GTEST_TEST(Markings, EqualityOperator) {
  const Markings kMarkings{
      .markings =
          {
              {
                  .color = Color::kBlack,
                  .line_length = 1.,
                  .side = Marking::Side::kLeft,
                  .space_length = 2.,
                  .start_offset = 3.,
                  .stop_offset = 4.,
                  .weight = LaneRoadMark::Weight::kBold,
                  .width = 5.,
                  .z_offset = 6.,
              },
          },
  };
  Markings markings = kMarkings;

  EXPECT_EQ(kMarkings, markings);
}

}  // namespace
}  // namespace test
}  // namespace object
}  // namespace xodr
}  // namespace malidrive