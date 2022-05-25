// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/common/macros.h"

#include <exception>
#include <string>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

// This test file only tests the behavior of non copy-pasted macros that come
// from Maliput / Delphyne.

namespace malidrive {
namespace test {

GTEST_TEST(MalidriveInternalMacros, ThrowIfNotInRangeTest) {
  EXPECT_THROW({ MALIDRIVE_IS_IN_RANGE(2., 3., 4.); }, maliput::common::assertion_error);
  EXPECT_NO_THROW({ MALIDRIVE_IS_IN_RANGE(3.5, 3., 4.); });
  EXPECT_NO_THROW({ MALIDRIVE_IS_IN_RANGE(3., 3., 4.); });
  EXPECT_NO_THROW({ MALIDRIVE_IS_IN_RANGE(4., 3., 4.); });
}

namespace {

// Dumb class for testing purposes.
class OxymoronException final : public std::runtime_error {
 public:
  explicit OxymoronException(const std::string& msg) : std::runtime_error(msg) {}
};

}  // namespace

GTEST_TEST(MalidriveInternalMacros, ValidateTest) {
  EXPECT_THROW({ MALIDRIVE_VALIDATE(true == false, OxymoronException, "Exception message goes here."); },
               OxymoronException);

  EXPECT_NO_THROW({ MALIDRIVE_VALIDATE(true, OxymoronException, "Exception message goes here."); });
}

// TODO(francocipollone): This test should be improved in order to correctly verify the generated log message.
//
GTEST_TEST(MalidriveInternalMacros, TraceTest) {
  const double dumb_d{27.12};
  const std::string dumb_str{"MALIDRIVE_TRACE test: "};
  EXPECT_NO_THROW(MALIDRIVE_TRACE(dumb_str + std::to_string(dumb_d)));
}

}  // namespace test
}  // namespace malidrive
