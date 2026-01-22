// BSD 3-Clause License
//
// Copyright (c) 2026, Woven Planet. All rights reserved.
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
#include "maliput_malidrive/xodr/signals.h"

#include <optional>

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Signals, EqualityOperator) {
  const Signal kSignal{
      1.0 /* s */,
      2.0 /* t */,
      "signal_id" /* id */,
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
  const Signals kSignals{{{kSignal}}};

  Signals signals = kSignals;
  EXPECT_EQ(kSignals, signals);
  signals.signals[0].s = 2.;
  EXPECT_NE(kSignals, signals);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
