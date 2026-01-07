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
#include "maliput_malidrive/xodr/object/object_reference.h"

#include <gtest/gtest.h>
#include <maliput/common/error.h>

namespace malidrive {
namespace xodr {
namespace object {
namespace test {
namespace {

GTEST_TEST(ObjectReference, EqualityOperator) {
  const ObjectReference kObjectReference{
      .id = object::ObjectReference::Id("test"),
      .orientation = Orientation::kNegative,
      .s = 1.0,
      .t = 2.0,
      .valid_length = 3.0,
      .z_offset = 4.0,
  };

  ObjectReference object_reference = kObjectReference;

  EXPECT_EQ(kObjectReference, object_reference);
  // Test inequality
  object_reference.id = object::ObjectReference::Id("test2");
  EXPECT_NE(kObjectReference, object_reference);
  object_reference.id = object::ObjectReference::Id("test");
  object_reference.orientation = Orientation::kPositive;
  EXPECT_NE(kObjectReference, object_reference);
  object_reference.orientation = Orientation::kNegative;
  object_reference.s = 2.0;
  EXPECT_NE(kObjectReference, object_reference);
  object_reference.s = 1.0;
  object_reference.t = 3.0;
  EXPECT_NE(kObjectReference, object_reference);
  object_reference.t = 2.0;
  object_reference.valid_length = 4.0;
  EXPECT_NE(kObjectReference, object_reference);
  object_reference.valid_length = 3.0;
  object_reference.z_offset = 5.0;
  EXPECT_NE(kObjectReference, object_reference);
}

}  // namespace
}  // namespace test
}  // namespace object
}  // namespace xodr
}  // namespace malidrive