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
#include "maliput_malidrive/base/segment.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/road_curve.h"

namespace malidrive {
namespace test {
namespace {

using maliput::math::Vector2;

std::unique_ptr<road_curve::Function> MakeZeroCubicPolynomial(double p0, double p1, double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 0., p0, p1, linear_tolerance);
}

class SegmentTest : public ::testing::Test {
 protected:
  const maliput::api::SegmentId kId{"dut"};
  const double kP0{0.};
  const double kP1{100.};
  const double kLinearTolerance{1e-13};
  const double kScaleLength{1.};
  const Vector2 kXy0{10., 10.};
  const Vector2 kDXy{(kP1 - kP0) * std::sqrt(2.) / 2., (kP1 - kP0) * std::sqrt(2.) / 2.};
  const bool kAssertContiguity{true};
  void SetUp() override {
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance),
        kAssertContiguity);
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
  }
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
};

TEST_F(SegmentTest, Constructor) {
  EXPECT_NO_THROW(Segment(kId, road_curve_.get(), reference_line_offset_.get(), kP0, kP1));
}

TEST_F(SegmentTest, ConstructorAssertions) {
  EXPECT_THROW(Segment(kId, nullptr, reference_line_offset_.get(), kP0, kP1), maliput::common::assertion_error);
  EXPECT_THROW(Segment(kId, road_curve_.get(), nullptr, kP0, kP1), maliput::common::assertion_error);
  const double kWrongP0{5.};
  const double kWrongP1{3.};
  EXPECT_THROW(Segment(kId, road_curve_.get(), reference_line_offset_.get(), kWrongP0, kWrongP1),
               maliput::common::assertion_error);
}

TEST_F(SegmentTest, RoadCurve) {
  const Segment dut(kId, road_curve_.get(), reference_line_offset_.get(), kP0, kP1);
  EXPECT_EQ(road_curve_.get(), dut.road_curve());
}

TEST_F(SegmentTest, ReferenceLineOffset) {
  const Segment dut(kId, road_curve_.get(), reference_line_offset_.get(), kP0, kP1);
  EXPECT_EQ(reference_line_offset_.get(), dut.reference_line_offset());
}

}  // namespace
}  // namespace test
}  // namespace malidrive
