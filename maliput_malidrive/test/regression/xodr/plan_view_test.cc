// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/plan_view.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(PlanView, EqualityOperator) {
  const PlanView kPlanView{{{1.23 /* s_0 */,
                             {523.2 /* x */, 83.27 /* y */},
                             0.77 /* orientation */,
                             100. /* length */,
                             Geometry::Type::kLine /* Type */,
                             {Geometry::Line{}} /* description */},
                            {1.23 /* s_0 */,
                             {523.2 /* x */, 83.27 /* y */},
                             0.77 /* orientation */,
                             100. /* length */,
                             Geometry::Type::kLine /* Type */,
                             {Geometry::Line{}} /* description */}}};
  PlanView plan_view = kPlanView;

  EXPECT_EQ(kPlanView, plan_view);
  plan_view.geometries[0].length = 568.5;
  EXPECT_NE(kPlanView, plan_view);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
