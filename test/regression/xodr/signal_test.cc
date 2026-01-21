// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven Planet. All rights reserved.
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
#include "maliput_malidrive/xodr/signal.h"

#include <optional>

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Signal, EqualityOperator) {
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
      std::make_optional(Signal::Value{1.0, "signal_unit"}) /* value */,
      std::make_optional(1.0) /* height */,
      std::make_optional(1.0) /* width */,
      std::make_optional(1.0) /* h_offset */,
      std::make_optional(1.0) /* length */,
      std::make_optional(1.0) /* pitch */,
      std::make_optional(1.0) /* roll */,
      std::make_optional("signal_text") /* text */
  };

  Signal signal = kSignal;
  EXPECT_EQ(kSignal, signal);
  signal.s = 2.;
  EXPECT_NE(kSignal, signal);
}

GTEST_TEST(Value, EqualityOperator) {
  const Signal::Value kSignalValue{40.0, "km/h"};

  Signal::Value signal_value = kSignalValue;
  EXPECT_EQ(kSignalValue, signal_value);
  signal_value.value = 60.0;
  EXPECT_NE(kSignalValue, signal_value);
  signal_value.value = 40.0;
  signal_value.unit = "m/s";
  EXPECT_NE(kSignalValue, signal_value);
}

GTEST_TEST(Value, EqualityOperatorEmptyUnit) {
  const Signal::Value kSignalValue{40.0, {}};
  Signal::Value signal_value = kSignalValue;
  EXPECT_EQ(kSignalValue, signal_value);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
