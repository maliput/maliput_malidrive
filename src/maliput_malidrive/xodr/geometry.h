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

#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <variant>

#include <maliput/math/vector.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR description's geometry header.
/// For example, a XML node describing a XODR's geometry:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///           <planView>
///               <geometry s="0.0000000000000000e+0" x="0.0000000000000000e+0" y="0.0000000000000000e+0"
///               hdg="1.3734007669450161e+0" length="1.4005927435591335e+2">
///                   <line/>
///               </geometry>
///               <geometry s="0.0000000000000000e+00" x="0.0" y="0.0" hdg="0.0" length="100.0">
///                   <arc curvature="0.025"/>
///               </geometry>
///           </planView>
///       ...
///   </OpenDRIVE>
/// @endcode
struct Geometry {
  /// Convenient constants that hold the tag names in the XODR Geometry header description.
  static constexpr const char* kGeometryTag = "geometry";
  static constexpr const char* kS0 = "s";
  static constexpr const char* kStartPointX = "x";
  static constexpr const char* kStartPointY = "y";
  static constexpr const char* kOrientation = "hdg";
  static constexpr const char* kLength = "length";

  /// Contains the types of geometric elements.
  enum class Type {
    kLine = 0,
    kArc,
  };

  /// Line geometry description.
  struct Line {
    bool operator==(const Line&) const { return true; }
  };

  /// Arc geometry description.
  struct Arc {
    /// Holds the tag name in the XODR Geometry description.
    static constexpr const char* kCurvature = "curvature";

    /// Arc's curvature.
    double curvature{};

    /// Equality operator.
    bool operator==(const Arc& other) const { return curvature == other.curvature; }
  };

  /// Matches string with a Type.
  /// @param type Is a Type.
  /// @returns A string that matches with `type`.
  static std::string type_to_str(Type type);

  /// Matches Type with a string.
  /// @param type Is a string.
  /// @returns A Type that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a Type.
  static Type str_to_type(const std::string& type);

  /// Equality operator.
  bool operator==(const Geometry& other) const;

  /// Inequality operator.
  bool operator!=(const Geometry& other) const;

  /// Start position (s-coordinate).
  double s_0{};
  /// Start position (X-Y inertial).
  maliput::math::Vector2 start_point{};
  /// Start orientation (inertial heading).
  double orientation{};
  /// Length of the element's reference line.
  double length{};
  /// Type of geometric element.
  Type type{Type::kLine};
  /// Description of the geometric type.
  std::variant<Line, Arc> description;
};

/// Streams a string representation of @p geometry into @p os.
/// Returns @p os.
/// @relates Geometry
std::ostream& operator<<(std::ostream& os, const Geometry& geometry);

}  // namespace xodr
}  // namespace malidrive
