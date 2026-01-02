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
#include <string>
#include <vector>

#include "maliput_malidrive/xodr/lane.h"

namespace malidrive {
namespace xodr {
namespace object {

struct CornerLocal {
  /// Convenient constants that hold the tag names in the XODR object's Outline cornerRoad description.
  static constexpr const char* kCornerLocalTag = "cornerLocal";
  static constexpr const char* kId = "id";
  static constexpr const char* kHeight = "height";
  static constexpr const char* kU = "u";
  static constexpr const char* kV = "v";
  static constexpr const char* kZ = "z";

  /// Height of the object at this corner, along the z-axis.
  double height{};
  /// ID of the outline point. Shall be unique within one outline.
  std::optional<int> id{std::nullopt};
  /// Local u-coordinate of the corner.
  double u{};
  /// Local v-coordinate of the corner.
  double v{};
  /// Local z-coordinate of the corner.
  double z{};

  bool operator==(const CornerLocal& other) const;
  bool operator!=(const CornerLocal& other) const;
};

struct CornerRoad {
  /// Convenient constants that hold the tag names in the XODR object's Outline cornerRoad description.
  static constexpr const char* kCornerRoadTag = "cornerRoad";
  static constexpr const char* kDz = "dz";
  static constexpr const char* kHeight = "height";
  static constexpr const char* kId = "id";
  static constexpr const char* kS = "s";
  static constexpr const char* kT = "t";

  /// dz of the corner relative to road reference line.
  double dz{};
  /// Height of the object at this corner, along the z-axis.
  double height{};
  /// ID of the outline point. Must be unique within one outline.
  std::optional<int> id{std::nullopt};
  /// s-coordinate of the corner.
  double s{};
  /// t-coordinate of the corner.
  double t{};

  bool operator==(const CornerRoad& other) const;
  bool operator!=(const CornerRoad& other) const;
};

struct Outline {
  /// Convenient constants that hold the tag names in the XODR object outline description.
  static constexpr const char* kOutlineTag = "outline";
  static constexpr const char* kClosed = "closed";
  static constexpr const char* kFillType = "fillType";
  static constexpr const char* kId = "id";
  static constexpr const char* kLaneType = "laneType";
  static constexpr const char* kOuter = "outer";

  enum class FillType {
    kAsphalt,
    kCobble,
    kConcrete,
    kGrass,
    kGravel,
    kPaint,
    kPavement,
    kSoil,
  };

  /// Matches string with a FillType.
  /// @param fill_type Is a FillType.
  /// @returns A string that matches with `fill_type`.
  static std::string fill_type_to_str(FillType fill_type);

  /// Matches a FillType with a string.
  /// @param fill_type Is a string.
  /// @returns A FillType that matches with `fill_type`.
  /// @throw maliput::common::assertion_error When `fill_type` doesn't match with a FillType.
  static FillType str_to_fill_type(const std::string& fill_type);

  bool operator==(const Outline& other) const;
  bool operator!=(const Outline& other) const;

  /// If true, the outline describes an area, not a linear feature.
  std::optional<bool> closed{std::nullopt};
  /// Type used to fill the area inside the outline.
  std::optional<FillType> fill_type{std::nullopt};
  /// ID of the outline. Must be unique within one object.
  std::optional<int> id{std::nullopt};
  /// Describes the lane type of the outline.
  std::optional<Lane::Type> lane_type{std::nullopt};
  /// Defines if outline is an outer outline of the object.
  std::optional<bool> outer{std::nullopt};

  /// Used to describe non-linear forms of objects. They are mutually exclusive with <cornerLocal> elements.
  /// <cornerRoad> elements describe the outline of objects relative to the road reference line with their s- and
  /// t-coordinates.
  std::vector<CornerRoad> corner_road{};
  /// Used to describe non-linear forms of objects. They are mutually exclusive with <cornerRoad> elements.
  /// <cornerLocal> elements describe the outline of objects within their local u- and v-coordinates.
  std::vector<CornerLocal> corner_local{};
};

/// Holds the values of a XODR Object outlines element.
struct Outlines {
  /// Convenient constants that hold the tag names in the XODR object outlines description.
  static constexpr const char* kOutlinesTag = "outlines";

  std::vector<Outline> outlines{};

  bool operator==(const Outlines& other) const;
  bool operator!=(const Outlines& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
