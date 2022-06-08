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
#include "maliput_malidrive/xodr/junction.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(JunctionTest, StrToType) {
  const std::string kExpectedDefault{"default"};
  const std::string kExpectedVirtual{"virtual"};
  EXPECT_EQ(Junction::Type::kDefault, Junction::str_to_type(kExpectedDefault));
  EXPECT_EQ(Junction::Type::kVirtual, Junction::str_to_type(kExpectedVirtual));
  EXPECT_THROW(Junction::str_to_type("WrongValue"), maliput::common::assertion_error);
}

GTEST_TEST(JunctionTest, TypeToStr) {
  const std::string kExpectedDefault{"default"};
  const std::string kExpectedVirtual{"virtual"};
  EXPECT_EQ(kExpectedDefault, Junction::type_to_str(Junction::Type::kDefault));
  EXPECT_EQ(kExpectedVirtual, Junction::type_to_str(Junction::Type::kVirtual));
}

GTEST_TEST(JunctionTest, EqualityOperator) {
  const Connection kConnectionA{Connection::Id("10") /* id */,
                                "1" /* incoming_road */,
                                "2" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                Connection::Id("50") /* connection_master */,
                                Connection::Type::kDefault /* type */,
                                {{Connection::LaneLink::Id("3"), Connection::LaneLink::Id("4")},
                                 {Connection::LaneLink::Id("5"), Connection::LaneLink::Id("6")}} /* lane_links */};
  const Connection kConnectionB{Connection::Id("11") /* id */,
                                "2" /* incoming_road */,
                                "3" /* connecting_road */,
                                Connection::ContactPoint::kStart /* contact_point */,
                                Connection::Id("51") /* connection_master */,
                                Connection::Type::kDefault /* type */,
                                {{Connection::LaneLink::Id("4"), Connection::LaneLink::Id("5")},
                                 {Connection::LaneLink::Id("6"), Connection::LaneLink::Id("7")}} /* lane_links */};
  const Junction kJunction{Junction::Id("358") /* id */,
                           "junctionTest" /* name */,
                           Junction::Type::kDefault /* type */,
                           {{kConnectionA.id, kConnectionA}, {kConnectionB.id, kConnectionB}} /* connections */};

  Junction junction = kJunction;

  EXPECT_EQ(kJunction, junction);
  junction.id = Junction::Id("400");
  EXPECT_NE(kJunction, junction);
  junction.id = Junction::Id("358");
  junction.name = std::string("junctionTestB");
  EXPECT_NE(kJunction, junction);
  junction.name = std::string("junctionTest");
  junction.type = Junction::Type::kVirtual;
  EXPECT_NE(kJunction, junction);
  junction.type = Junction::Type::kDefault;
  junction.connections.at(kConnectionA.id).lane_links[0].to = Connection::LaneLink::Id("35");
  EXPECT_NE(kJunction, junction);
  junction.connections.at(kConnectionA.id).lane_links[0].to = Connection::LaneLink::Id("4");
  EXPECT_EQ(kJunction, junction);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
