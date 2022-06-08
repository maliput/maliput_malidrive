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
#include "maliput_malidrive/xodr/header.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Header, EqualityOperator) {
  const Header kHeader{1. /* revMajor */,
                       1. /* revMinor */,
                       "TestHeader" /* name */,
                       1.21 /* version */,
                       "Wed Sep 19 12:00:00 2018" /* date */,
                       1.1 /* north */,
                       2.2 /* south */,
                       3.3 /* east */,
                       4.4 /* west */,
                       "TestVendor" /* vendor */};
  Header header = kHeader;

  EXPECT_EQ(kHeader, header);
  header.rev_major = 32;
  EXPECT_NE(kHeader, header);
  header.rev_major = 1.;
  header.rev_minor = 10.;
  EXPECT_NE(kHeader, header);
  header.rev_minor = 1.;
  header.name = std::nullopt;
  EXPECT_NE(kHeader, header);
  header.name = "TestHeader";
  header.version = 2.5;
  EXPECT_NE(kHeader, header);
  header.version = 1.21;
  header.date = "Wed May 27 12:00:00 2018";
  EXPECT_NE(kHeader, header);
  header.date = "Wed Sep 19 12:00:00 2018";
  header.north = 8.;
  EXPECT_NE(kHeader, header);
  header.north = 1.1;
  header.south = 5.1;
  EXPECT_NE(kHeader, header);
  header.south = 2.2;
  header.east = 58.2;
  EXPECT_NE(kHeader, header);
  header.east = 3.3;
  header.west = 58.1;
  EXPECT_NE(kHeader, header);
  header.west = 4.4;
  header.vendor = "DifferentVendor";
  EXPECT_NE(kHeader, header);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
