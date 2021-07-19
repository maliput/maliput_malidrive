// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/geometry.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Geometry, TypeToStrMethod) {
  const std::string kExpectedStr{"line"};
  EXPECT_EQ(kExpectedStr, Geometry::type_to_str(Geometry::Type::kLine));
}

GTEST_TEST(Geometry, StrToTypeMethod) {
  const Geometry::Type kExpectedType{Geometry::Type::kLine};
  const std::string kTypeStr{"line"};
  const std::string kWrongTypeStr{"WrongType"};
  EXPECT_EQ(kExpectedType, Geometry::str_to_type(kTypeStr));
  EXPECT_THROW(Geometry::str_to_type(kWrongTypeStr), maliput::common::assertion_error);
}

GTEST_TEST(Geometry, EqualityOperator) {
  const Geometry kGeometry{1.23 /* s_0 */,    {523.2 /* x */, 83.27 /* y */},   0.77 /* orientation */,
                           100. /* length */, Geometry::Type::kLine /* Type */, {Geometry::Line{}} /* description */};
  Geometry geometry = kGeometry;

  EXPECT_EQ(kGeometry, geometry);
  geometry.s_0 = 568.5;
  EXPECT_NE(kGeometry, geometry);
  geometry.s_0 = 1.23;
  geometry.start_point.x() = 12.2;
  EXPECT_NE(kGeometry, geometry);
  geometry.start_point.x() = 523.2;
  geometry.start_point.y() = 23.2;
  EXPECT_NE(kGeometry, geometry);
  geometry.start_point.y() = 83.27;
  geometry.orientation = 1.23;
  EXPECT_NE(kGeometry, geometry);
  geometry.orientation = 0.77;
  geometry.length = 564.5;
  EXPECT_NE(kGeometry, geometry);
  geometry.length = 100.;

  Geometry geometry_arc = kGeometry;
  geometry_arc.type = Geometry::Type::kArc;
  geometry_arc.description = Geometry::Arc{0.5};
  EXPECT_NE(kGeometry, geometry_arc);
  geometry.type = Geometry::Type::kArc;
  geometry.description = Geometry::Arc{0.25};
  EXPECT_NE(geometry, geometry_arc);
  geometry.description = Geometry::Arc{0.5};
  EXPECT_EQ(geometry, geometry_arc);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
