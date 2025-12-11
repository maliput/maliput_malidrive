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

#include <optional>
#include <string>
#include <map>

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

// <roadMark sOffset="0.0" type="broken" color="white" width="0.12">

//     <!-- Standard dashed marking -->
//     <type sOffset="0.0" name="broken">
//         <line length="3.0" space="9.0"/>
//     </type>

//     <!-- Apply sway for 50 meters -->
//     <sway sOffset="0.0" length="50.0">
//         <vertex ds="0.0" dy="0.00"/>
//         <vertex ds="5.0" dy="0.20"/>
//         <vertex ds="10.0" dy="-0.20"/>
//         <vertex ds="15.0" dy="0.00"/>
//     </sway>

//     <!-- Change to custom explicit marking for a symbol -->
//     <explicit sOffset="50.0">
//         <outline>
//             <corner localX="0" localY="0"/>
//             <corner localX="4" localY="0"/>
//             <corner localX="4" localY="2"/>
//             <corner localX="0" localY="2"/>
//         </outline>
//     </explicit>

//     <!-- Switch to solid line after the marking -->
//     <type sOffset="60.0" name="solid">
//         <line length="0.0" space="0.0"/> <!-- continuous -->
//     </type>

// </roadMark>

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

	enum class Color{
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

	enum class LaneChange{
		kBoth,
		kDecrease,
		kIncrease,
		kNone,
	};

	enum class Type{
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

	enum class Weight{
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

  /// Allows a lane change in the indicated direction, taking into account that lanes are numbered in ascending order
  /// from right to left. If the attribute is missing, “both” is used as default.
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
};

}  // namespace xodr
}  // namespace malidrive
