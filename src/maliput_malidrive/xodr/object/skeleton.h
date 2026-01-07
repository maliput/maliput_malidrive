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
#include <vector>

#include <maliput/api/type_specific_identifier.h>

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object skeleton polyline vertexRoad element.
struct VertexRoad {
  /// Id alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct VertexRoad>;

  /// Convenient constants that hold the tag names in the XODR object skeleton polyline vertexRoad description.
  static constexpr const char* kVertexRoadTag = "vertexRoad";
  static constexpr const char* kId = "id";
  static constexpr const char* kDZ = "dz";
  static constexpr const char* kIntersectionPoint = "intersectionPoint";
  static constexpr const char* kRadius = "radius";
  static constexpr const char* kS = "s";
  static constexpr const char* kT = "t";

  /// dz of the polyline point relative to road reference line parallel to z.
  double dz{};
  /// ID of the vertex point. Must be unique within one polyline.
  std::optional<Id> id{std::nullopt};
  /// Vertex point is intersecting the ground. "false" is used as default.
  std::optional<bool> intersection_point{std::nullopt};
  /// Local radius of the object at this vertex point, along the polyline.
  std::optional<double> radius{std::nullopt};
  /// s-coordinate of the corner.
  double s{};
  /// t-coordinate of the corner.
  double t{};

  /// Equality operator.
  bool operator==(const VertexRoad& other) const;

  /// Inequality operator.
  bool operator!=(const VertexRoad& other) const;
};

/// Holds the values of a XODR Object skeleton polyline vertexLocal element.
struct VertexLocal {
  /// Id alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct VertexLocal>;

  /// Convenient constants that hold the tag names in the XODR object skeleton polyline vertexLocal description.
  static constexpr const char* kVertexLocalTag = "vertexLocal";
  static constexpr const char* kId = "id";
  static constexpr const char* kIntersectionPoint = "intersectionPoint";
  static constexpr const char* kRadius = "radius";
  static constexpr const char* kU = "u";
  static constexpr const char* kV = "v";
  static constexpr const char* kZ = "z";

  /// ID of the vertex point. Must be unique within one polyline.
  std::optional<Id> id{std::nullopt};
  /// Vertex point is intersecting the ground. "false" is used as default.
  std::optional<bool> intersection_point{std::nullopt};
  /// Local radius of the object at this vertex point, along the polyline.
  std::optional<double> radius{std::nullopt};
  /// Local u-coordinate of the vertex point.
  double u{};
  /// Local v-coordinate of the vertex point.
  double v{};
  /// Local z-coordinate of the vertex point.
  double z{};

  /// Equality operator.
  bool operator==(const VertexLocal& other) const;

  /// Inequality operator.
  bool operator!=(const VertexLocal& other) const;
};

/// Holds the values of a XODR Object skeleton polyline element.
struct Polyline {
  /// Id alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct Polyline>;

  /// Convenient constants that hold the tag names in the XODR object skeleton polyline description.
  static constexpr const char* kPolylinesTag = "polyline";
  static constexpr const char* kId = "id";

  /// ID of the polyline. Must be unique within one object.
  std::optional<Id> id{std::nullopt};
  /// Used to describe a more detailed form of objects inside their bounding box. They are mutually exclusive with
  /// <vertexLocal> elements. <vertexRoad> elements describe a skeleton polyline of objects relative to the road
  /// reference line with their s- and t-coordinates.
  std::vector<VertexRoad> vertex_road{};
  /// Used to describe a more detailed form of objects inside their bounding box. They are mutually exclusive with
  /// <vertexRoad> elements. <vertexLocal> describe a skeleton polyline of objects within a local u/v coordinate system.
  std::vector<VertexLocal> vertex_local{};

  /// Equality operator.
  bool operator==(const Polyline& other) const;

  /// Inequality operator.
  bool operator!=(const Polyline& other) const;
};

/// Holds the values of a XODR Object skeleton element.
struct Skeleton {
  /// Convenient constants that hold the tag names in the XODR object skeleton description.
  static constexpr const char* kSkeletonTag = "skeleton";

  std::vector<Polyline> polylines{};

  /// Equality operator.
  bool operator==(const Skeleton& other) const;

  /// Inequality operator.
  bool operator!=(const Skeleton& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
