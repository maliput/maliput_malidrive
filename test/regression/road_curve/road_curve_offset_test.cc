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
#include "maliput_malidrive/road_curve/road_curve_offset.h"

#include <array>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/road_curve/arc_ground_curve.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

std::unique_ptr<road_curve::Function> MakeConstantCubicPolynomial(double d, double p0, double p1,
                                                                  double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., d, p0, p1, linear_tolerance);
}

using maliput::math::Vector2;

class RoadCurveOffsetTest : public ::testing::Test {
 public:
  const double kLinearTolerance{1e-12};
  const double kScaleLength{1.};
  const double kP0{10.};
  const double kP1{20.};
  const double kDeltaP{kP1 - kP0};
  const Vector2 kXy0{1., 2.};
  const double kZero{0.};
  const double kR0{0.};
  const double kRLeft{4.};
  const double kRRight{-2.5};
  const bool kAssertContiguity{true};

  const std::array<double, 5> kPs{kP0, kP0 + kDeltaP * 0.25, kP0 + kDeltaP * 0.5, kP0 + kDeltaP * 0.75, kP1};
  const std::unique_ptr<Function> lane_offset_0 = MakeConstantCubicPolynomial(kR0, kP0, kP1, kLinearTolerance);
  const std::unique_ptr<Function> lane_offset_left = MakeConstantCubicPolynomial(kRLeft, kP0, kP1, kLinearTolerance);
  const std::unique_ptr<Function> lane_offset_right = MakeConstantCubicPolynomial(kRRight, kP0, kP1, kLinearTolerance);

  std::unique_ptr<RoadCurve> road_curve_{};
};

TEST_F(RoadCurveOffsetTest, ConstructorAssertionRoadCurve) {
  EXPECT_THROW(RoadCurveOffset(nullptr, lane_offset_0.get(), kP0, kP1), maliput::common::assertion_error);
  EXPECT_THROW(RoadCurveOffset(road_curve_.get(), nullptr, kP0, kP1), maliput::common::assertion_error);
}

// Flat, non-elevated and non-superelevated line road curve. Offsets are not
// scaled at any offset.
class FlatLineRoadCurveTest : public RoadCurveOffsetTest {
 protected:
  void SetUp() override {
    auto ground_curve = std::make_unique<LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1);
    auto elevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kLinearTolerance);
    auto superelevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kLinearTolerance);
    road_curve_ = std::make_unique<RoadCurve>(kLinearTolerance, kScaleLength, std::move(ground_curve),
                                              std::move(elevation), std::move(superelevation), kAssertContiguity);
  }

  const Vector2 kDXy{3., 4.};
  const double kDeltaP{kP1 - kP0};
  const double kArcLength{kDXy.norm()};

  const std::array<double, 5> kSs{0., 0.25 * kArcLength, 0.5 * kArcLength, 0.75 * kArcLength, kArcLength};
};

TEST_F(FlatLineRoadCurveTest, ConstructorAssertionRange) {
  const double kWrongP0{5.};
  const double kWrongP1{3.};
  EXPECT_THROW(RoadCurveOffset(road_curve_.get(), lane_offset_0.get(), kWrongP0, kWrongP1),
               maliput::common::assertion_error);
}

TEST_F(FlatLineRoadCurveTest, RoadCurve) {
  const RoadCurveOffset dut(road_curve_.get(), lane_offset_0.get(), kP0, kP1);

  EXPECT_EQ(road_curve_.get(), dut.road_curve());
}

TEST_F(FlatLineRoadCurveTest, RelativeTolerance) {
  const double kTolerance{1e-3};
  auto ground_curve = std::make_unique<LineGroundCurve>(kTolerance, kXy0, kDXy, kP0, kP1);
  auto elevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kTolerance);
  auto superelevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kTolerance);
  road_curve_ = std::make_unique<RoadCurve>(kTolerance, kScaleLength, std::move(ground_curve), std::move(elevation),
                                            std::move(superelevation), kAssertContiguity);
  const RoadCurveOffset dut(road_curve_.get(), lane_offset_0.get(), kP0, kP1);

  EXPECT_DOUBLE_EQ(kTolerance / kDXy.norm(), dut.relative_tolerance());
}

TEST_F(FlatLineRoadCurveTest, RelativeToleranceClamped) {
  const RoadCurveOffset dut(road_curve_.get(), lane_offset_0.get(), kP0, kP1);

  EXPECT_DOUBLE_EQ(/* Minimum allowed value */ 1e-8, dut.relative_tolerance());
}

TEST_F(FlatLineRoadCurveTest, CalcSFromP) {
  const std::array<RoadCurveOffset, 3> duts{RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0, kP1}};

  for (const auto& dut : duts) {
    for (int i = 0; i < kPs.size(); ++i) {
      EXPECT_NEAR(kSs[i], dut.CalcSFromP(kPs[i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatLineRoadCurveTest, PFromS) {
  const std::array<RoadCurveOffset, 3> duts{RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0, kP1}};
  for (const auto& dut : duts) {
    auto p_from_s = dut.PFromS();
    for (int i = 0; i < kSs.size(); ++i) {
      EXPECT_NEAR(kPs[i], p_from_s(kSs[i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatLineRoadCurveTest, SFromP) {
  const std::array<RoadCurveOffset, 3> duts{RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0, kP1}};

  for (const auto& dut : duts) {
    auto s_from_p = dut.SFromP();
    for (int i = 0; i < kPs.size(); ++i) {
      EXPECT_NEAR(kSs[i], s_from_p(kPs[i]), kLinearTolerance);
    }
  }
}

// For a flat, non-elevated and non-superelevated line road curve.
// Test a RoadCurveOffset that uses a sub range of the entire RoadCurve domain.
class FlatLineRoadCurveSubRangeTest : public FlatLineRoadCurveTest {
 protected:
  const double kP0Sub{12.};
  const double kP1Sub{17.};
  const double kDeltaPSub{kP1Sub - kP0Sub};
  const std::array<double, 5> kSsSub{0 * kArcLength / 2, 0.25 * kArcLength / 2, 0.5 * kArcLength / 2,
                                     0.75 * kArcLength / 2, kArcLength / 2};
  const std::array<double, 5> kPsSub{kP0Sub, kP0Sub + kDeltaPSub * 0.25, kP0Sub + kDeltaPSub * 0.5,
                                     kP0Sub + kDeltaPSub * 0.75, kP1Sub};
};

TEST_F(FlatLineRoadCurveSubRangeTest, CalcSFromP) {
  const std::array<RoadCurveOffset, 3> duts{
      RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0Sub, kP1Sub}};
  for (const auto& dut : duts) {
    for (int i = 0; i < kPsSub.size(); ++i) {
      EXPECT_NEAR(kSsSub[i], dut.CalcSFromP(kPsSub[i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatLineRoadCurveSubRangeTest, SFromP) {
  const std::array<RoadCurveOffset, 3> duts{
      RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0Sub, kP1Sub}};
  for (const auto& dut : duts) {
    auto s_from_p = dut.SFromP();
    for (int i = 0; i < kPsSub.size(); ++i) {
      EXPECT_NEAR(kSsSub[i], s_from_p(kPsSub[i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatLineRoadCurveSubRangeTest, PFromS) {
  const std::array<RoadCurveOffset, 3> duts{
      RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0Sub, kP1Sub}};
  for (const auto& dut : duts) {
    auto p_from_s = dut.PFromS();
    for (int i = 0; i < kSsSub.size(); ++i) {
      EXPECT_NEAR(kPsSub[i], p_from_s(kSsSub[i]), kLinearTolerance);
    }
  }
}

// Flat, non-elevated and non-superelevated arc road curve.
class FlatArcRoadCurveTest : public RoadCurveOffsetTest {
 protected:
  void SetUp() override {
    auto ground_curve =
        std::make_unique<ArcGroundCurve>(kLinearTolerance, kXy0, kStartHeading, kCurvature, kArcLength, kP0, kP1);
    auto elevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kLinearTolerance);
    auto superelevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kLinearTolerance);
    road_curve_ = std::make_unique<RoadCurve>(kLinearTolerance, kScaleLength, std::move(ground_curve),
                                              std::move(elevation), std::move(superelevation), kAssertContiguity);
  }

  const double kStartHeading{M_PI / 3.};
  const double kCurvature{-0.025};  // Equivalent radius = 40m.
  const double kRadiusR0{std::abs(1. / kCurvature)};
  const double kRadiusRLeft{std::abs(1. / kCurvature) + kRLeft};
  const double kRadiusRRight{std::abs(1. / kCurvature) + kRRight};
  const double kArcLength{100.};
  const double kDTheta{kArcLength / kRadiusR0};
  const std::array<std::array<double, 5>, 3> kSs{/*{*/ 0.,
                                                 0.25 * kArcLength,
                                                 0.5 * kArcLength,
                                                 0.75 * kArcLength,
                                                 kArcLength /*}*/,
                                                 /*{*/ 0.,
                                                 0.25 * kDTheta* kRadiusRLeft,
                                                 0.5 * kDTheta* kRadiusRLeft,
                                                 0.75 * kDTheta* kRadiusRLeft,
                                                 kDTheta* kRadiusRLeft /*}*/,
                                                 /*{*/ 0.,
                                                 0.25 * kDTheta* kRadiusRRight,
                                                 0.5 * kDTheta* kRadiusRRight,
                                                 0.75 * kDTheta* kRadiusRRight,
                                                 kDTheta* kRadiusRRight /*}*/};
};

TEST_F(FlatArcRoadCurveTest, RoadCurve) {
  const RoadCurveOffset dut(road_curve_.get(), lane_offset_0.get(), kP0, kP1);

  EXPECT_EQ(road_curve_.get(), dut.road_curve());
}

TEST_F(FlatArcRoadCurveTest, RelativeTolerance) {
  const double kTolerance{1e-3};
  auto ground_curve =
      std::make_unique<ArcGroundCurve>(kTolerance, kXy0, kStartHeading, kCurvature, kArcLength, kP0, kP1);
  auto elevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kTolerance);
  auto superelevation = std::make_unique<CubicPolynomial>(kZero, kZero, kZero, kZero, kP0, kP1, kTolerance);
  road_curve_ = std::make_unique<RoadCurve>(kTolerance, kScaleLength, std::move(ground_curve), std::move(elevation),
                                            std::move(superelevation), kAssertContiguity);
  const RoadCurveOffset dut(road_curve_.get(), lane_offset_0.get(), kP0, kP1);

  EXPECT_DOUBLE_EQ(kTolerance / kArcLength, dut.relative_tolerance());
}

TEST_F(FlatArcRoadCurveTest, RelativeToleranceClamped) {
  const RoadCurveOffset dut(road_curve_.get(), lane_offset_0.get(), kP0, kP1);

  EXPECT_DOUBLE_EQ(/* Minimum allowed value */ 1e-8, dut.relative_tolerance());
}

TEST_F(FlatArcRoadCurveTest, CalcSFromP) {
  const std::array<RoadCurveOffset, 3> duts{RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0, kP1}};
  for (int j = 0; j < static_cast<int>(duts.size()); ++j) {
    for (int i = 0; i < kPs.size(); ++i) {
      EXPECT_NEAR(kSs[j][i], duts[j].CalcSFromP(kPs[i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatArcRoadCurveTest, PFromS) {
  const std::array<RoadCurveOffset, 3> duts{RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0, kP1}};

  for (int j = 0; j < static_cast<int>(duts.size()); ++j) {
    auto p_from_s = duts[j].PFromS();
    for (int i = 0; i < kSs.size(); ++i) {
      EXPECT_NEAR(kPs[i], p_from_s(kSs[j][i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatArcRoadCurveTest, SFromP) {
  const std::array<RoadCurveOffset, 3> duts{RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0, kP1},
                                            RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0, kP1}};

  for (int j = 0; j < static_cast<int>(duts.size()); ++j) {
    auto s_from_p = duts[j].SFromP();
    for (int i = 0; i < static_cast<int>(kPs.size()); ++i) {
      EXPECT_NEAR(kSs[j][i], s_from_p(kPs[i]), kLinearTolerance);
    }
  }
}

// For a flat, non-elevated and non-superelevated arc road curve.
// Test a RoadCurveOffset that uses a sub range of the entire RoadCurve domain.
class FlatArcRoadCurveSubRangeTest : public FlatArcRoadCurveTest {
 protected:
  const std::array<std::array<double, 5>, 3> kSsSub{/*{*/ 0.,
                                                    0.25 * kArcLength / 2,
                                                    0.5 * kArcLength / 2,
                                                    0.75 * kArcLength / 2,
                                                    kArcLength / 2 /*}*/,
                                                    /*{*/ 0.,
                                                    0.25 * kDTheta* kRadiusRLeft / 2,
                                                    0.5 * kDTheta* kRadiusRLeft / 2,
                                                    0.75 * kDTheta* kRadiusRLeft / 2,
                                                    kDTheta* kRadiusRLeft / 2 /*}*/,
                                                    /*{*/ 0.,
                                                    0.25 * kDTheta* kRadiusRRight / 2,
                                                    0.5 * kDTheta* kRadiusRRight / 2,
                                                    0.75 * kDTheta* kRadiusRRight / 2,
                                                    kDTheta* kRadiusRRight / 2 /*}*/};
  const double kP0Sub{12.};
  const double kP1Sub{17.};
  const double kDeltaPSub{kP1Sub - kP0Sub};

  const std::array<double, 5> kPsSub{kP0Sub, kP0Sub + kDeltaPSub * 0.25, kP0Sub + kDeltaPSub * 0.5,
                                     kP0Sub + kDeltaPSub * 0.75, kP1Sub};
};

TEST_F(FlatArcRoadCurveSubRangeTest, CalcSFromP) {
  const std::array<RoadCurveOffset, 3> duts{
      RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0Sub, kP1Sub}};

  for (int j = 0; j < static_cast<int>(duts.size()); ++j) {
    for (int i = 0; i < static_cast<int>(kPsSub.size()); ++i) {
      EXPECT_NEAR(kSsSub[j][i], duts[j].CalcSFromP(kPsSub[i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatArcRoadCurveSubRangeTest, PFromS) {
  const std::array<RoadCurveOffset, 3> duts{
      RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0Sub, kP1Sub}};

  for (int j = 0; j < static_cast<int>(duts.size()); ++j) {
    auto p_from_s = duts[j].PFromS();
    for (int i = 0; i < kSsSub.size(); ++i) {
      EXPECT_NEAR(kPsSub[i], p_from_s(kSsSub[j][i]), kLinearTolerance);
    }
  }
}

TEST_F(FlatArcRoadCurveSubRangeTest, SFromP) {
  const std::array<RoadCurveOffset, 3> duts{
      RoadCurveOffset{road_curve_.get(), lane_offset_0.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_left.get(), kP0Sub, kP1Sub},
      RoadCurveOffset{road_curve_.get(), lane_offset_right.get(), kP0Sub, kP1Sub}};

  for (int j = 0; j < static_cast<int>(duts.size()); ++j) {
    auto s_from_p = duts[j].SFromP();
    for (int i = 0; i < kPsSub.size(); ++i) {
      EXPECT_NEAR(kSsSub[j][i], s_from_p(kPsSub[i]), kLinearTolerance);
    }
  }
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
