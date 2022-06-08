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
