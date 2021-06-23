// Copyright 2020 Toyota Research Institute
#pragma once

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Lane Width.
/// For example, a XML node describing a XODR's lane width:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///     <road>
///       <lanes>
///         <laneSection>
///           <right>
///             <lane>
///               <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
///             </lane>
///           </right>
///         </laneSection>
///       </lanes>
///     <road>
///       ...
///   </OpenDRIVE>
/// @endcode
struct LaneWidth {
  /// Convenient constants that hold the tag names in the XODR width description.
  static constexpr const char* kLaneWidthTag = "width";
  static constexpr const char* kOffset = "sOffset";
  static constexpr const char* kA = "a";
  static constexpr const char* kB = "b";
  static constexpr const char* kC = "c";
  static constexpr const char* kD = "d";

  /// Equality operator.
  bool operator==(const LaneWidth& other) const;

  /// Inequality operator.
  bool operator!=(const LaneWidth& other) const;

  /// Start position (s-coordinate) relative to the position of the preceding laneSection.
  double s_0{};
  /// Coefficients of a cubic polynomial: @f$a + b * p + c * p^2 + d * p^3@f$.
  /// @{
  double a{};
  double b{};
  double c{};
  double d{};
  /// @}
};

}  // namespace xodr
}  // namespace malidrive
