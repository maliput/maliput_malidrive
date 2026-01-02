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
#include "maliput_malidrive/xodr/object/surface.h"

#include <gtest/gtest.h>
#include <maliput/common/error.h>

namespace malidrive {
namespace xodr {
namespace object {
namespace test {
namespace {

GTEST_TEST(CRG, EqualityOperator) {
  const CRG kCRG{
      .file = "test",
      .hide_road_surface_crg = true,
      .z_scale = 1.,
  };
  CRG crg = kCRG;

  EXPECT_EQ(kCRG, crg);
  // Test inequality
  crg.file = "test2";
  EXPECT_NE(kCRG, crg);
  crg.file = "test";
  crg.hide_road_surface_crg = false;
  EXPECT_NE(kCRG, crg);
  crg.hide_road_surface_crg = true;
  crg.z_scale = 2.;
  EXPECT_NE(kCRG, crg);
  crg.z_scale = 1.;
}

GTEST_TEST(Surface, EqualityOperator) {
  const Surface kSurface{
      .crg =
          CRG{
              .file = "test",
              .hide_road_surface_crg = true,
              .z_scale = 1.,
          },
  };
  Surface surface = kSurface;

  EXPECT_EQ(kSurface, surface);
}

}  // namespace
}  // namespace test
}  // namespace object
}  // namespace xodr
}  // namespace malidrive
