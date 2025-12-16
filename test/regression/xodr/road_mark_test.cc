// BSD 3-Clause License
//
// Copyright (c) 2025, Woven Planet. All rights reserved.
// Copyright (c) 2020-2025, Toyota Research Institute. All rights reserved.
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
#include <gtest/gtest.h>
#include <maliput/common/error.h>

#include "maliput_malidrive/xodr/lane_road_mark.h"

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(SwayElement, EqualityOperator) {
  const SwayElement kSwayElement{1., 1., 1., 1., 1.};
  SwayElement sway_element = kSwayElement;

  EXPECT_EQ(kSwayElement, sway_element);
  // Test inequality
  sway_element.a = 2.;
  EXPECT_NE(kSwayElement, sway_element);
  sway_element.a = 1.;
  sway_element.b = 2.;
  EXPECT_NE(kSwayElement, sway_element);
  sway_element.b = 1.;
  sway_element.c = 2.;
  EXPECT_NE(kSwayElement, sway_element);
  sway_element.c = 1.;
  sway_element.d = 2.;
  EXPECT_NE(kSwayElement, sway_element);
  sway_element.d = 1.;
  sway_element.s_0 = 2.;
  EXPECT_NE(kSwayElement, sway_element);
  sway_element.s_0 = 1.;
  EXPECT_EQ(kSwayElement, sway_element);
}

GTEST_TEST(ExplicitElementLine, EqualityOperator) {
  const std::optional<double> width{1.};
  const std::optional<Rule> rule{Rule::kNone};
  const ExplicitElementLine kExplicitElementLine{1., rule, 1., 1., width};

  ExplicitElementLine explicit_element_line = kExplicitElementLine;
  EXPECT_EQ(kExplicitElementLine, explicit_element_line);
  // Test inequality
  explicit_element_line.length = 2.;
  EXPECT_NE(kExplicitElementLine, explicit_element_line);
  explicit_element_line.length = 1.;
  explicit_element_line.rule = Rule::kCaution;
  EXPECT_NE(kExplicitElementLine, explicit_element_line);
  explicit_element_line.rule = Rule::kNone;
  explicit_element_line.s_offset = 2.;
  EXPECT_NE(kExplicitElementLine, explicit_element_line);
  explicit_element_line.s_offset = 1.;
  explicit_element_line.t_offset = 2.;
  EXPECT_NE(kExplicitElementLine, explicit_element_line);
}

GTEST_TEST(ExplicitElement, EqualityOperator) {
  const ExplicitElement kExplicitElement{{ExplicitElementLine{1., Rule::kNone, 1., 1., std::nullopt}}};
  ExplicitElement explicit_element = kExplicitElement;

  EXPECT_EQ(kExplicitElement, explicit_element);
  // Test inequality
  explicit_element.lines = {ExplicitElementLine{2., Rule::kCaution, 1., 1., std::nullopt}};
  EXPECT_NE(kExplicitElement, explicit_element);
}

GTEST_TEST(TypeElementLine, EqualityOperator) {
  const std::optional<Color> color{Color::kRed};
  const std::optional<Rule> rule{Rule::kNone};
  const std::optional<double> width{1.};
  const TypeElementLine kTypeElementLine{color, 1., rule, 1., 1., 1., width};

  TypeElementLine type_element_line = kTypeElementLine;
  EXPECT_EQ(kTypeElementLine, type_element_line);
  // Test inequality
  type_element_line.length = 2.;
  EXPECT_NE(kTypeElementLine, type_element_line);
  type_element_line.length = 1.;
  type_element_line.rule = Rule::kCaution;
  EXPECT_NE(kTypeElementLine, type_element_line);
  type_element_line.rule = Rule::kNone;
  type_element_line.s_offset = 2.;
  EXPECT_NE(kTypeElementLine, type_element_line);
  type_element_line.s_offset = 1.;
  type_element_line.t_offset = 2.;
  EXPECT_NE(kTypeElementLine, type_element_line);
}

GTEST_TEST(TypeElement, EqualityOperator) {
  const std::optional<double> width{1.};
  const TypeElementLine kTypeElementLine{Color::kRed, 1., Rule::kNone, 1., 1., 1., width};
  const TypeElement kTypeElement{"name", 1., {kTypeElementLine}};
  TypeElement type_element = kTypeElement;

  EXPECT_EQ(kTypeElement, type_element);
  // Test inequality
  type_element.name = "name2";
  EXPECT_NE(kTypeElement, type_element);
  type_element.name = "name";
  type_element.width = 2.;
  EXPECT_NE(kTypeElement, type_element);
}

GTEST_TEST(LaneRoadMark, EqualityOperator) {
  const std::optional<LaneRoadMark::LaneChange> lane_change{LaneRoadMark::LaneChange::kBoth};
  const std::optional<LaneRoadMark::Weight> weight{LaneRoadMark::Weight::kBold};
  const std::optional<double> width{1.};
  const LaneRoadMark kLaneRoadMark{Color::kRed, 1.,   lane_change, "material", 1., LaneRoadMark::Type::kBroken,
                                   weight,      width};
  LaneRoadMark lane_road_mark = kLaneRoadMark;

  EXPECT_EQ(kLaneRoadMark, lane_road_mark);
  // Test inequality
  lane_road_mark.color = Color::kBlue;
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
  lane_road_mark.color = Color::kRed;
  lane_road_mark.height = 2.;
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
  lane_road_mark.height = 1.;
  lane_road_mark.lane_change = LaneRoadMark::LaneChange::kDecrease;
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
  lane_road_mark.lane_change = LaneRoadMark::LaneChange::kBoth;
  lane_road_mark.material = "material2";
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
  lane_road_mark.material = "material";
  lane_road_mark.s_offset = 2.;
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
  lane_road_mark.s_offset = 1.;
  lane_road_mark.type = LaneRoadMark::Type::kSolid;
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
  lane_road_mark.type = LaneRoadMark::Type::kBroken;
  lane_road_mark.weight = LaneRoadMark::Weight::kStandard;
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
  lane_road_mark.weight = LaneRoadMark::Weight::kBold;
  lane_road_mark.width = 2.;
  EXPECT_NE(kLaneRoadMark, lane_road_mark);
}

GTEST_TEST(Color, ColorToStr) {
  EXPECT_EQ(color_to_str(Color::kBlack), "black");
  EXPECT_EQ(color_to_str(Color::kBlue), "blue");
  EXPECT_EQ(color_to_str(Color::kGreen), "green");
  EXPECT_EQ(color_to_str(Color::kOrange), "orange");
  EXPECT_EQ(color_to_str(Color::kRed), "red");
  EXPECT_EQ(color_to_str(Color::kStandard), "standard");
  EXPECT_EQ(color_to_str(Color::kViolet), "violet");
  EXPECT_EQ(color_to_str(Color::kWhite), "white");
  EXPECT_EQ(color_to_str(Color::kYellow), "yellow");
}

GTEST_TEST(Color, StrToColor) {
  EXPECT_EQ(str_to_color("black"), Color::kBlack);
  EXPECT_EQ(str_to_color("blue"), Color::kBlue);
  EXPECT_EQ(str_to_color("green"), Color::kGreen);
  EXPECT_EQ(str_to_color("orange"), Color::kOrange);
  EXPECT_EQ(str_to_color("red"), Color::kRed);
  EXPECT_EQ(str_to_color("standard"), Color::kStandard);
  EXPECT_EQ(str_to_color("violet"), Color::kViolet);
  EXPECT_EQ(str_to_color("white"), Color::kWhite);
  EXPECT_EQ(str_to_color("yellow"), Color::kYellow);
}

GTEST_TEST(LaneRoadMark, LaneChangeToStr) {
  EXPECT_EQ(LaneRoadMark::lane_change_to_str(LaneRoadMark::LaneChange::kBoth), "both");
  EXPECT_EQ(LaneRoadMark::lane_change_to_str(LaneRoadMark::LaneChange::kDecrease), "decrease");
  EXPECT_EQ(LaneRoadMark::lane_change_to_str(LaneRoadMark::LaneChange::kIncrease), "increase");
  EXPECT_EQ(LaneRoadMark::lane_change_to_str(LaneRoadMark::LaneChange::kNone), "none");
}

GTEST_TEST(LaneRoadMark, StrToLaneChange) {
  EXPECT_EQ(LaneRoadMark::str_to_lane_change("both"), LaneRoadMark::LaneChange::kBoth);
  EXPECT_EQ(LaneRoadMark::str_to_lane_change("decrease"), LaneRoadMark::LaneChange::kDecrease);
  EXPECT_EQ(LaneRoadMark::str_to_lane_change("increase"), LaneRoadMark::LaneChange::kIncrease);
  EXPECT_EQ(LaneRoadMark::str_to_lane_change("none"), LaneRoadMark::LaneChange::kNone);
}

GTEST_TEST(LaneRoadMark, TypeToStr) {
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kBottsDots), "botts dots");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kBrokenBroken), "broken broken");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kBrokenSolid), "broken solid");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kBroken), "broken");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kCurb), "curb");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kCustom), "custom");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kEdge), "edge");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kGrass), "grass");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kNone), "none");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kSolidBroken), "solid broken");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kSolidSolid), "solid solid");
  EXPECT_EQ(LaneRoadMark::type_to_str(LaneRoadMark::Type::kSolid), "solid");
}

GTEST_TEST(LaneRoadMark, StrToType) {
  EXPECT_EQ(LaneRoadMark::str_to_type("botts dots"), LaneRoadMark::Type::kBottsDots);
  EXPECT_EQ(LaneRoadMark::str_to_type("broken broken"), LaneRoadMark::Type::kBrokenBroken);
  EXPECT_EQ(LaneRoadMark::str_to_type("broken solid"), LaneRoadMark::Type::kBrokenSolid);
  EXPECT_EQ(LaneRoadMark::str_to_type("broken"), LaneRoadMark::Type::kBroken);
  EXPECT_EQ(LaneRoadMark::str_to_type("curb"), LaneRoadMark::Type::kCurb);
  EXPECT_EQ(LaneRoadMark::str_to_type("custom"), LaneRoadMark::Type::kCustom);
  EXPECT_EQ(LaneRoadMark::str_to_type("edge"), LaneRoadMark::Type::kEdge);
  EXPECT_EQ(LaneRoadMark::str_to_type("grass"), LaneRoadMark::Type::kGrass);
  EXPECT_EQ(LaneRoadMark::str_to_type("none"), LaneRoadMark::Type::kNone);
  EXPECT_EQ(LaneRoadMark::str_to_type("solid broken"), LaneRoadMark::Type::kSolidBroken);
  EXPECT_EQ(LaneRoadMark::str_to_type("solid solid"), LaneRoadMark::Type::kSolidSolid);
  EXPECT_EQ(LaneRoadMark::str_to_type("solid"), LaneRoadMark::Type::kSolid);
}

GTEST_TEST(LaneRoadMark, WeightToStr) {
  EXPECT_EQ(LaneRoadMark::weight_to_str(LaneRoadMark::Weight::kBold), "bold");
  EXPECT_EQ(LaneRoadMark::weight_to_str(LaneRoadMark::Weight::kStandard), "standard");
}

GTEST_TEST(LaneRoadMark, StrToWeight) {
  EXPECT_EQ(LaneRoadMark::str_to_weight("bold"), LaneRoadMark::Weight::kBold);
  EXPECT_EQ(LaneRoadMark::str_to_weight("standard"), LaneRoadMark::Weight::kStandard);
}

GTEST_TEST(Rule, RuleToStr) {
  EXPECT_EQ(rule_to_str(Rule::kCaution), "caution");
  EXPECT_EQ(rule_to_str(Rule::kNoPassing), "no passing");
  EXPECT_EQ(rule_to_str(Rule::kNone), "none");
}

GTEST_TEST(Rule, StrToRule) {
  EXPECT_EQ(str_to_rule("caution"), Rule::kCaution);
  EXPECT_EQ(str_to_rule("no passing"), Rule::kNoPassing);
  EXPECT_EQ(str_to_rule("none"), Rule::kNone);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
