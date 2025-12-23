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

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/object/outlines.h"
#include "maliput_malidrive/xodr/object/repeat.h"

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object.
struct Object {
  /// Convenient constants that hold the tag names in the XODR object description.
  static constexpr const char* kObjectTag = "object";
  static constexpr const char* kDynamic = "dynamic";
  static constexpr const char* kHdg = "hdg";
  static constexpr const char* kHeight = "height";
  static constexpr const char* kId = "id";
  static constexpr const char* kLength = "length";
  static constexpr const char* kName = "name";
  static constexpr const char* kOrientation = "orientation";
  static constexpr const char* kPerpToRoad = "perpToRoad";
  static constexpr const char* kPitch = "pitch";
  static constexpr const char* kRadius = "radius";
  static constexpr const char* kRoll = "roll";
  static constexpr const char* kS = "s";
  static constexpr const char* kSubtype = "subtype";
  static constexpr const char* kT = "t";
  static constexpr const char* kType = "type";
  static constexpr const char* kValidLength = "validLength";
  static constexpr const char* kWidth = "width";
  static constexpr const char* kZOffset = "zOffset";

  enum class Orientation {
    kPositive,
    kNegative,
    kNone,
  };

  enum class ObjectType {
    kBarrier,
    kBike,
    kBuilding,
    kBus,
    kCar,
    kCrosswalk,
    kGantry,
    kMotorbike,
    kNone,
    kObstacle,
    kParkingSpace,
    kPatch,
    kPedestrian,
    kPole,
    kRailing,
    kRoadMark,
    kRoadSurface,
    kSoundBarrier,
    kStreetLamp,
    kTrafficIsland,
    kTrailer,
    kTrain,
    kTram,
    kTree,
    kVan,
    kVegetation,
    kWind,
  };

  /// Matches string with a Orientation.
  /// @param orientation Is a Orientation.
  /// @returns A string that matches with `orientation`.
  static std::string orientation_to_str(Orientation orientation);

  /// Matches Orientation with a string.
  /// @param orientation Is a string.
  /// @returns A Orientation that matches with `orientation`.
  /// @throw maliput::common::assertion_error When `orientation` doesn't match with a Orientation.
  static Orientation str_to_orientation(const std::string& orientation);

  /// Matches string with a ObjectType.
  /// @param object_type Is a ObjectType.
  /// @returns A string that matches with `object_type`.
  static std::string object_type_to_str(ObjectType object_type);

  /// Matches ObjectType with a string.
  /// @param object_type Is a string.
  /// @returns A ObjectType that matches with `object_type`.
  /// @throw maliput::common::assertion_error When `object_type` doesn't match with a ObjectType.
  static ObjectType str_to_object_type(const std::string& object_type);

  /// Equality operator.
  bool operator==(const Object& other) const;

  /// Inequality operator.
  bool operator!=(const Object& other) const;

  /// Indicates whether the object is dynamic or static, default value is “no” (static). Dynamic object cannot change
  /// its position.
  std::optional<bool> dynamic{std::nullopt};
  /// Heading angle of the object relative to road direction.
  std::optional<double> hdg{std::nullopt};
  /// Height of the object’s bounding box, defined in the local coordinate system u/v along the z-axis.
  std::optional<double> height{std::nullopt};
  /// Unique ID within database.
  std::string id{};
  /// Length of the object’s bounding box, alternative to @radius, defined in the local coordinate system u/v along the
  /// u-axis.
  std::optional<double> length{std::nullopt};
  /// Name of the object. May be chosen freely.
  std::optional<std::string> name{std::nullopt};
  /// "+" = valid in positive s-direction, "-" = valid in negative s-direction, "none" = valid in both directions (does
  /// not affect the heading).
  std::optional<Orientation> orientation{std::nullopt};
  /// Alternative to @pitch and @roll. If true, the object is vertically perpendicular to the road surface at all points
  /// and @pitch and @roll are ignored.
  std::optional<bool> perp_to_road{std::nullopt};
  /// Pitch angle relative to the x/y-plane.
  std::optional<double> pitch{std::nullopt};
  /// radius of the circular object’s bounding box, alternative to @length and @width. @radius is defined in the local
  /// coordinate system u/v.
  std::optional<double> radius{std::nullopt};
  /// Roll angle relative to the x/y-plane.
  std::optional<double> roll{std::nullopt};
  /// s-coordinate of object’s origin.
  double s{};
  /// Variant of a type.
  std::optional<std::string> subtype{std::nullopt};
  /// t-coordinate of object’s origin.
  double t{};
  /// Type of object.
  std::optional<ObjectType> type{std::nullopt};
  /// Validity of object along s-axis (0.0 for point object).
  std::optional<double> valid_length{std::nullopt};
  /// Width of the object’s bounding box, alternative to @radius,  defined in the local coordinate system u/v along the
  /// v-axis.
  std::optional<double> width{std::nullopt};
  /// z-offset of object’s origin relative to the elevation of the road reference line.
  double z_offset{};

  /// Repeated elements.
  std::optional<Repeat> repeats{std::nullopt};
  /// Outlines elements.
  std::optional<Outlines> outlines{std::nullopt};
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive