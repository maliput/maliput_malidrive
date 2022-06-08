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
#include "maliput_malidrive/xodr/unit.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(UnitFeatures, UnitToStrMethod) {
  const std::string kExpectedStr{"km/h"};
  EXPECT_EQ(kExpectedStr, unit_to_str(Unit::kKph));
}

GTEST_TEST(UnitFeatures, StrToUnitMethod) {
  const Unit kExpectedUnit{Unit::kMph};
  const std::string kUnitStr{"mph"};
  const std::string kWrongUnitStr{"m/h"};
  EXPECT_EQ(kExpectedUnit, str_to_unit(kUnitStr));
  EXPECT_THROW(str_to_unit(kWrongUnitStr), maliput::common::assertion_error);
}

GTEST_TEST(UnitFeatures, ConvertToMs) {
  EXPECT_EQ(15., ConvertToMs(15., Unit::kMs));
  EXPECT_EQ(8.5, ConvertToMs(30.6, Unit::kKph));
  EXPECT_EQ(20.1168, ConvertToMs(45., Unit::kMph));
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
