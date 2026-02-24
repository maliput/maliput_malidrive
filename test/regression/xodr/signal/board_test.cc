// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include "maliput_malidrive/xodr/signal/board.h"

#include <optional>

#include <gtest/gtest.h>

#include "maliput_malidrive/xodr/signal/sign.h"
#include "maliput_malidrive/xodr/signal/signal.h"

namespace malidrive {
namespace xodr {
namespace signal {
namespace test {
namespace {

GTEST_TEST(DisplayArea, EqualityOperator) {
  const DisplayArea kDisplayArea{2.0 /* height */, 1 /* index */, 0.5 /* v */, 1.5 /* width */, 0.8 /* z */};

  DisplayArea display_area = kDisplayArea;
  EXPECT_EQ(kDisplayArea, display_area);

  display_area.height = 3.0;
  EXPECT_NE(kDisplayArea, display_area);
  display_area = kDisplayArea;

  display_area.index = 2;
  EXPECT_NE(kDisplayArea, display_area);
  display_area = kDisplayArea;

  display_area.v = 1.0;
  EXPECT_NE(kDisplayArea, display_area);
  display_area = kDisplayArea;

  display_area.width = 2.5;
  EXPECT_NE(kDisplayArea, display_area);
  display_area = kDisplayArea;

  display_area.z = 1.5;
  EXPECT_NE(kDisplayArea, display_area);
}

GTEST_TEST(VmsBoard, EqualityOperator) {
  const DisplayArea kDisplayArea{2.0 /* height */, 1 /* index */, 0.5 /* v */, 1.5 /* width */, 0.8 /* z */};
  const VmsBoard kVmsBoard{
      std::make_optional(3.0) /* display_height */,
      DisplayType::kLed /* display_type */,
      std::make_optional(5.0) /* display_width */,
      0.5 /* v */,
      1.2 /* z */,
      {{kDisplayArea}} /* display_areas */
  };

  VmsBoard vms_board = kVmsBoard;
  EXPECT_EQ(kVmsBoard, vms_board);

  vms_board.display_height = std::make_optional(4.0);
  EXPECT_NE(kVmsBoard, vms_board);
  vms_board = kVmsBoard;

  vms_board.display_type = DisplayType::kMonochromeGraphic;
  EXPECT_NE(kVmsBoard, vms_board);
  vms_board = kVmsBoard;

  vms_board.display_width = std::make_optional(6.0);
  EXPECT_NE(kVmsBoard, vms_board);
  vms_board = kVmsBoard;

  vms_board.v = 1.0;
  EXPECT_NE(kVmsBoard, vms_board);
  vms_board = kVmsBoard;

  vms_board.z = 2.0;
  EXPECT_NE(kVmsBoard, vms_board);
  vms_board = kVmsBoard;

  vms_board.display_areas.clear();
  EXPECT_NE(kVmsBoard, vms_board);
}

GTEST_TEST(StaticBoard, EqualityOperator) {
  const Signal kSignal{
      1.0 /* s */,
      2.0 /* t */,
      signal::Signal::Id("signal_id") /* id */,
      std::make_optional("signal_name") /* name */,
      false /* dynamic */,
      "+" /* orientation */,
      0.1 /* z_offset */,
      std::make_optional("signal_country") /* country */,
      std::make_optional("signal_country_revision") /* country_revision */,
      "signal_type" /* type */,
      "signal_subtype" /* subtype */,
      std::nullopt /* value */,
      std::make_optional(1.0) /* height */,
      std::make_optional(1.0) /* width */,
      std::make_optional(1.0) /* h_offset */,
      std::make_optional(1.0) /* length */,
      std::make_optional(1.0) /* pitch */,
      std::make_optional(1.0) /* roll */,
      std::make_optional("signal_text") /* text */
  };
  const Sign kSign = Sign(kSignal, 0.0 /* v */, 0.0 /* z */);
  const StaticBoard kStaticBoard{{kSign}};

  StaticBoard static_board = kStaticBoard;
  EXPECT_EQ(kStaticBoard, static_board);

  static_board.signs.clear();
  EXPECT_NE(kStaticBoard, static_board);
}

}  // namespace
}  // namespace test
}  // namespace signal
}  // namespace xodr
}  // namespace malidrive