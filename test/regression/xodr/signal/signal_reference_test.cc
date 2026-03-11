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
#include "maliput_malidrive/xodr/signal/signal_reference.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace signal {
namespace test {
namespace {

GTEST_TEST(SignalReference, EqualityOperator) {
  const Validity kValidity{Validity::Id("from"), Validity::Id("to")};
  const SignalReference kSignalReference{
      SignalReference::SignalId("signal_id") /* signal_id */,
      Orientation::kWithS /* orientation */,
      10.0 /* s */,
      5.0 /* t */,
      {{kValidity}} /* validities */
  };

  SignalReference signal_reference = kSignalReference;
  EXPECT_EQ(kSignalReference, signal_reference);

  signal_reference.signal_id = SignalReference::SignalId("new_signal_id");
  EXPECT_NE(kSignalReference, signal_reference);
  signal_reference = kSignalReference;

  signal_reference.orientation = Orientation::kAgainstS;
  EXPECT_NE(kSignalReference, signal_reference);
  signal_reference = kSignalReference;

  signal_reference.s = 20.0;
  EXPECT_NE(kSignalReference, signal_reference);
  signal_reference = kSignalReference;

  signal_reference.t = 10.0;
  EXPECT_NE(kSignalReference, signal_reference);
  signal_reference = kSignalReference;

  signal_reference.validities.clear();
  EXPECT_NE(kSignalReference, signal_reference);
}

}  // namespace
}  // namespace test
}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
