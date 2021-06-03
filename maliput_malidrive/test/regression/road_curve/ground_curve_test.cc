// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/road_curve/ground_curve.h"

#include <gtest/gtest.h>

#include <maliput/test_utilities/maliput_math_compare.h>
#include "maliput_malidrive/test_utilities/ground_curve_stub.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

using maliput::math::test::CompareVectors;

// Exercises the interface.
GTEST_TEST(GrounCurveTest, InterfaceTest) {
  const maliput::math::Vector2 kG{1., 2.};
  const maliput::math::Vector2 kGDot{3., 4.};
  const double kHeading{5.};
  const double kHeadingDot{6.};
  const double kGInverse{7.};
  const double kArcLength{8.};
  const double kLinearTolerance{0.1};
  const double kP0{9.};
  const double kP1{10.};
  const bool kIsG1Contiguous{true};

  const GroundCurveStub dut(kG, kGDot, kHeading, kHeadingDot, kGInverse, kArcLength, kLinearTolerance, kP0, kP1,
                            kIsG1Contiguous);

  EXPECT_TRUE(CompareVectors(dut.G(kP0), kG));
  EXPECT_TRUE(CompareVectors(dut.GDot(kP0), kGDot));
  EXPECT_EQ(dut.Heading(kP0), kHeading);
  EXPECT_EQ(dut.HeadingDot(kP0), kHeadingDot);
  EXPECT_EQ(dut.GInverse(kG), kGInverse);
  EXPECT_EQ(dut.ArcLength(), kArcLength);
  EXPECT_EQ(dut.linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(dut.p0(), kP0);
  EXPECT_EQ(dut.p1(), kP1);
  EXPECT_EQ(dut.IsG1Contiguous(), kIsG1Contiguous);
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
