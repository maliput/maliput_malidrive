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
#include "maliput_malidrive/xodr/road_type.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(RoadTypeSpeed, EqualityOperator) {
  const RoadType::Speed kSpeed{45. /* max */, Unit::kMph /* unit */};
  RoadType::Speed speed = kSpeed;

  EXPECT_EQ(kSpeed, speed);
  speed.max = 150.;
  EXPECT_NE(kSpeed, speed);
  speed.max = 45.;
  speed.unit = Unit::kMs;
  EXPECT_NE(kSpeed, speed);
  speed.unit = Unit::kMph;
  EXPECT_EQ(kSpeed, speed);
};

GTEST_TEST(RoadType, TypeToStrMethod) {
  const std::string kExpectedStr{"townLocal"};
  EXPECT_EQ(kExpectedStr, RoadType::type_to_str(RoadType::Type::kTownLocal));
}

GTEST_TEST(RoadType, StrToTypeMethod) {
  const RoadType::Type kExpectedType{RoadType::Type::kRural};
  const std::string kTypeStr{"rural"};
  const std::string kWrongTypeStr{"badType"};
  EXPECT_EQ(kExpectedType, RoadType::str_to_type(kTypeStr));
  EXPECT_THROW(RoadType::str_to_type(kWrongTypeStr), maliput::common::assertion_error);
}

GTEST_TEST(RoadType, EqualityOperator) {
  const RoadType::Speed kSpeed{45. /* max */, Unit::kMph /* unit */};
  const RoadType kRoadType{1.2 /* s0 */, RoadType::Type::kPedestrian /* type */, std::nullopt /* country */,
                           kSpeed /* speed */};
  RoadType road_type = kRoadType;
  EXPECT_EQ(kRoadType, road_type);
  road_type.s_0 = 0.5;
  EXPECT_NE(kRoadType, road_type);
  road_type.s_0 = 1.2;
  road_type.type = RoadType::Type::kTownExpressway;
  EXPECT_NE(kRoadType, road_type);
  road_type.type = RoadType::Type::kPedestrian;
  road_type.country = "64";
  EXPECT_NE(kRoadType, road_type);
  road_type.country = std::nullopt;
  road_type.speed.max = std::nullopt;
  EXPECT_NE(kRoadType, road_type);
  road_type.speed.max = 45.;
  EXPECT_EQ(kRoadType, road_type);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
