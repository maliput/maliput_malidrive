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

#include <map>
#include <optional>
#include <vector>

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/colors.h"
#include "maliput_malidrive/xodr/lane_road_mark.h"

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object cornerReference element.
struct CornerReference {
  /// Id alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct CornerReference>;

  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kCornerReferenceTag = "cornerReference";
  static constexpr const char* kId = "id";

  /// Identifier of the referenced outline point.
  Id id{"none"};

  bool operator==(const CornerReference& other) const;
  bool operator!=(const CornerReference& other) const;
};

/// Holds the values of a XODR Object marking element.
struct Marking {
  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kMarkingTag = "marking";
  static constexpr const char* kColor = "color";
  static constexpr const char* kLineLength = "lineLength";
  static constexpr const char* kSide = "side";
  static constexpr const char* kSpaceLength = "spaceLength";
  static constexpr const char* kStartOffset = "startOffset";
  static constexpr const char* kStopOffset = "stopOffset";
  static constexpr const char* kWeight = "weight";
  static constexpr const char* kWidth = "width";
  static constexpr const char* kZOffset = "zOffset";

  enum class Side { kLeft, kRight, kFront, kRear };

  /// Color of the marking.
  Color color{};
  /// Length of the visible part.
  double line_length{};
  /// Side of the bounding box described in <object> element in the local coordinate system u/v.
  Side side{};
  /// Length of the gap between the visible parts.
  double space_length{};
  /// Lateral offset in u-direction from start of bounding box side where the first marking starts.
  double start_offset{};
  /// Lateral offset in u-direction from end of bounding box side where the marking ends.
  double stop_offset{};
  /// Optical "weight" of the marking.
  std::optional<LaneRoadMark::Weight> weight{std::nullopt};
  /// Width of the marking.
  std::optional<double> width{std::nullopt};
  /// Height of road mark above the road, i.e. thickness of the road mark.
  std::optional<double> z_offset{std::nullopt};

  /// Specifies a point by referencing an existing outline point.
  std::vector<CornerReference> corner_reference{};

  /// Matches string with a Side.
  /// @param side Is a Side.
  /// @returns A string that matches with `side`.
  static std::string side_to_str(Side side);

  /// Matches Side with a string.
  /// @param side Is a string.
  /// @returns A Side that matches with `Side`.
  /// @throw maliput::common::assertion_error When `side` doesn't match with a Side.
  static Side str_to_side(const std::string& side);

  bool operator==(const Marking& other) const;
  bool operator!=(const Marking& other) const;
};

/// Holds the values of a XODR Object markings element.
struct Markings {
  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kMarkingsTag = "markings";

  std::vector<Marking> markings{};

  bool operator==(const Markings& other) const;
  bool operator!=(const Markings& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
