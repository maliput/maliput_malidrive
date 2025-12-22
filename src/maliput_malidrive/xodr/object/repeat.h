// BSD 3-Clause License
//
// Copyright (c) 2025, Woven Planet. All rights reserved.
// Copyright (c) 2025, Toyota Research Institute. All rights reserved.
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

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object Repeat element.
struct Repeat {
  /// Convenient constants that hold the tag names in the XODR object repeat description.
  static constexpr const char* kRepeatTag = "repeat";
  static constexpr const char* kDetachFromReferenceLine = "detachFromReferenceLine";
  static constexpr const char* kDistance = "distance";
  static constexpr const char* kHeightEnd = "heightEnd";
  static constexpr const char* kHeightStart = "heightStart";
  static constexpr const char* kLengthEnd = "lengthEnd";
  static constexpr const char* kLengthStart = "lengthStart";
  static constexpr const char* kLength = "length";
  static constexpr const char* kRadiusEnd = "radiusEnd";
  static constexpr const char* kRadiusStart = "radiusStart";
  static constexpr const char* kS = "s";
  static constexpr const char* kTEnd = "tEnd";
  static constexpr const char* kTStart = "tStart";
  static constexpr const char* kWidthEnd = "widthEnd";
  static constexpr const char* kWidthStart = "widthStart";
  static constexpr const char* kZOffsetEnd = "zOffsetEnd";
  static constexpr const char* kZOffsetStart = "zOffsetStart";

  /// If true, the start and end positions are connected as a straight line which does not follow the road reference
  /// line.
  std::optional<bool> detach_from_reference_line{std::nullopt};
  /// Distance between two instances of the object
  double distance{};
  /// Height of the object at @s + @length
  double height_end{};
  /// Height of the object at @s
  double height_start{};
  /// Length of the object at @s + @length
  std::optional<double> length_end{std::nullopt};
  /// Length of the object at @s
  std::optional<double> length_start{std::nullopt};
  /// Length of the repeat area, along the road reference line in s-direction.
  double length{};
  /// Radius of the object at @s + @length
  std::optional<double> radius_end{std::nullopt};
  /// Radius of the object at @s
  std::optional<double> radius_start{std::nullopt};
  /// s-coordinate of start position, overrides the corresponding argument in the original <object> record
  double s{};
  /// Lateral offset of object’s reference point at @s + @length
  double t_end{};
  /// Lateral offset of objects reference point at @s
  double t_start{};
  /// Width of the object at @s + @length
  std::optional<double> width_end{std::nullopt};
  /// Width of the object at @s
  std::optional<double> width_start{std::nullopt};
  /// z-offset of the object at @s + @length, relative to the elevation of the road reference line
  double z_offset_end{};
  /// z-offset of the object at @s, relative to the elevation of the road reference line
  double z_offset_start{};

  bool operator==(const Repeat& other) const;
  bool operator!=(const Repeat& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive