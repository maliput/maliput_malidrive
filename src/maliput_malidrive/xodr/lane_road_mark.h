// BSD 3-Clause License
//
// Copyright (c) 2026, Woven Planet. All rights reserved.
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
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Lane RoadMark.
/// For example, a XML node describing a XODR's lane roadMark:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///     <road>
///       <lanes>
///         <laneSection>
///           <right>
///             <lane>
///               <roadMark sOffset="0" type="solid" weight="standard" color="standard" width="1"/>
///             </lane>
///           </right>
///         </laneSection>
///       </lanes>
///     <road>
///       ...
///   </OpenDRIVE>
/// @endcode

struct LaneRoadMark {
  /// Convenient constants that hold the tag names in the XODR roadMark description.
  static constexpr const char* kLaneRoadMarkTag = "roadMark";
  static constexpr const char* kLaneRoadMarkColor = "color";
  static constexpr const char* kLaneRoadMarkHeight = "height";
  static constexpr const char* kLaneRoadMarkLaneChange = "laneChange";
  static constexpr const char* kLaneRoadMarkMaterial = "material";
  static constexpr const char* kOffset = "sOffset";
  static constexpr const char* kType = "type";
  static constexpr const char* kWeight = "weight";
  static constexpr const char* kWidth = "width";

  enum class Color {
    kBlack,
    kBlue,
    kGreen,
    kOrange,
    kRed,
    kStandard,
    kViolet,
    kWhite,
    kYellow,
  };

  enum class LaneChange {
    kBoth,
    kDecrease,
    kIncrease,
    kNone,
  };

  enum class Type {
    kBottsDots,
    kBrokenBroken,
    kBrokenSolid,
    kBroken,
    kCurb,
    kCustom,
    kEdge,
    kGrass,
    kNone,
    kSolidBroken,
    kSolidSolid,
    kSolid,
  };

  enum class Weight {
    kBold,
    kStandard,
  };

  /// Matches string with a Color.
  /// @param color Is a Color.
  /// @returns A string that matches with `color`.
  static std::string color_to_str(Color color);

  /// Matches Color with a string.
  /// @param color Is a string.
  /// @returns A Color that matches with `color`.
  /// @throw maliput::common::assertion_error When `color` doesn't match with a Color.
  static Color str_to_color(const std::string& color);

  /// Matches string with a LaneChange.
  /// @param lane_change Is a LaneChange.
  /// @returns A string that matches with `lane_change`.
  static std::string lane_change_to_str(LaneChange lane_change);

  /// Matches LaneChange with a string.
  /// @param lane_change Is a string.
  /// @returns A LaneChange that matches with `lane_change`.
  /// @throw maliput::common::assertion_error When `lane_change` doesn't match with a LaneChange.
  static LaneChange str_to_lane_change(const std::string& lane_change);

  /// Matches string with a Type.
  /// @param type Is a Type.
  /// @returns A string that matches with `type`.
  static std::string type_to_str(Type type);

  /// Matches Type with a string.
  /// @param type Is a string.
  /// @returns A Type that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a Type.
  static Type str_to_type(const std::string& type);

  /// Matches string with a Weight.
  /// @param weight Is a Weight.
  /// @returns A string that matches with `weight`.
  static std::string weight_to_str(Weight weight);

  /// Matches Weight with a string.
  /// @param weight Is a string.
  /// @returns A Weight that matches with `weight`.
  /// @throw maliput::common::assertion_error When `weight` doesn't match with a Weight.
  static Weight str_to_weight(const std::string& weight);

  /// Equality operator.
  bool operator==(const LaneRoadMark& other) const;

  /// Inequality operator.
  bool operator!=(const LaneRoadMark& other) const;

  /// Color of the road marking
  Color color{};

  /// Height of road mark above the road, i.e. thickness of the road mark [m]
  double height{};

  /// Allows a lane change in the indicated direction, taking into account that lanes are numbered in ascending
  /// order from right to left. If the attribute is missing, “both” is used as default.
  std::optional<LaneChange> lane_change{std::nullopt};

  /// Material of the road mark. Identifiers to be defined by the user, use "standard" as default value.
  std::string material{};

  /// Start position (s-coordinate) relative to the position of the preceding laneSection record.
  double s_offset{};

  /// Type of the road mark
  Type type{};

  /// Weight of the road mark. This attribute is optional if detailed definition is given below.
  std::optional<Weight> weight{std::nullopt};

  /// Width of the road mark. This attribute is optional if detailed definition is given by <line> element.
  std::optional<double> width{std::nullopt};

  /// Optional Type element for the roadMark
  std::optional<TypeElement> type_elem{std::nullopt};

  /// Optional Explicit element for the roadMark
  std::optional<ExplicitElement> explicit_elem{std::nullopt};

  /// Optional Sway element for the roadMark
  std::optional<SwayElement> sway_elem{std::nullopt};
};

enum class Rule {
  kCaution,
  kNoPassing,
  kNone,
};

/// Matches string with a RuleEnum.
/// @param rule Is a RuleEnum.
/// @returns A string that matches with `rule`.
static std::string rule_to_str(Rule rule);

/// Matches RuleEnum with a string.
/// @param rule Is a string.
/// @returns A Rule that matches with `rule`.
/// @throw maliput::common::assertion_error When `rule` doesn't match with a RuleEnum.
static Rule str_to_rule(const std::string& rule);

struct TypeElementLine {
  /// Convenient constants that hold the tag names in the XODR roadMark Line element description.
  static constexpr const char* kTypeLineTag = "line";
  static constexpr const char* kColor = "color";
  static constexpr const char* kLength = "length";
  static constexpr const char* kRule = "rule";
  static constexpr const char* kSOffset = "sOffset";
  static constexpr const char* kSpace = "space";
  static constexpr const char* kTOffset = "tOffset";
  static constexpr const char* kWidth = "width";

  /// Color of the road marking type line element
  std::optional<LaneRoadMark::Color> color{std::nullopt};

  /// Length of the visible part
  double length;

  /// Rule that must be observed when passing the line from inside, for example, from the lane with the lower absolute
  /// ID to the lane with the higher absolute ID
  std::optional<Rule> rule{std::nullopt};

  /// Initial longitudinal offset of the line definition from the start of the road mark definition
  double s_offset{};

  /// Length of the gap between the visible parts
  double space{};

  /// Lateral offset from the lane border.
  double t_offset{};

  // Line width
  std::optional<double> width{std::nullopt};

  /// Equality operator.
  bool operator==(const TypeElementLine& other) const;
};

struct TypeElement {
  /// Convenient constants that hold the tag names in the XODR roadMark Type element description.
  static constexpr const char* kTypeElementTag = "type";
  static constexpr const char* kName = "name";
  static constexpr const char* kWidth = "width";

  /// Name of the road mark type. May be chosen freely.
  std::string name{};

  /// Accumulated width of the road mark. In case of several <line> elements this @width is the sum of all @width
  /// of <line> elements and spaces in between, necessary to form the road mark.
  double width{};

  /// Line definition for a Type element, may contain one or more.
  std::vector<TypeElementLine> lines{};

  /// Equality operator.
  bool operator==(const TypeElement& other) const;
};

struct ExplicitElementLine {
  /// Convenient constants that hold the tag names in the XODR roadMark Line element description.
  static constexpr const char* kExplicitElementLineTag = "line";
  static constexpr const char* kLength = "length";
  static constexpr const char* kRule = "rule";
  static constexpr const char* kSOffset = "sOffset";
  static constexpr const char* kTOffset = "tOffset";
  static constexpr const char* kWidth = "width";

  /// Length of the visible part
  double length;

  /// Rule that must be observed when passing the line from inside, for example, from the lane with the lower absolute
  /// ID to the lane with the higher absolute ID
  std::optional<Rule> rule{std::nullopt};

  /// Initial longitudinal offset of the line definition from the start of the road mark definition
  double s_offset{};

  /// Lateral offset from the lane border.
  double t_offset{};

  // Line width
  std::optional<double> width{std::nullopt};

  /// Equality operator.
  bool operator==(const ExplicitElementLine& other) const;
};

struct ExplicitElement {
  /// Convenient constants that hold the tag names in the XODR roadMark Explicit description.
  static constexpr const char* kExplicitElementTag = "explicit";

  /// Line definition for an Explicit element, may contain one or more.
  std::vector<ExplicitElementLine> lines{};

  /// Equality operator.
  bool operator==(const ExplicitElement& other) const;
};

struct SwayElement {
  /// Convenient constants that hold the tag names in the XODR roadMark Sway element description.
  static constexpr const char* kSwayTag = "sway";
  static constexpr const char* kA = "a";
  static constexpr const char* kB = "b";
  static constexpr const char* kC = "c";
  static constexpr const char* kD = "d";
  static constexpr const char* kDS = "ds";

  /// Polynom parameter a, sway value at @s (ds=0)
  double a{};

  /// Polynom parameter b
  double b{};

  /// Polynom parameter c
  double c{};

  /// Polynom parameter d
  double d{};

  /// s-coordinate of start position of the <sway> element, relative to the @sOffset given in the <roadMark> element
  double ds{};

  /// Equality operator.
  bool operator==(const SwayElement& other) const;
};

}  // namespace xodr
}  // namespace malidrive
