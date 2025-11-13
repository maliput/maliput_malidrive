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
///               <geometry s="0.0000000000000000e+00" x="0.0" y="0.0" hdg="0.0" length="100.0">
///                   <spiral curvStart="0.025" curvEnd="0.05"/>
///               </geometry>
///               <geometry s="0.0000000000000000e+00" x="0.0" y="0.0" hdg="0.0" length="100.0">
///                   <paramPoly3 aU="0.0" bU="1.0" cU="0.0" dU="0.0" aV="0.0" bV="0.0" cV="0.1" dV="0.0" pRange="arcLength"/>
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
    kSpiral,
    kParamPoly3,
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

  /// Spiral geometry description.
  struct Spiral {
    /// Holds the tag name in the XODR Geometry description.
    static constexpr const char* kCurvStart = "curvStart";
    static constexpr const char* kCurvEnd = "curvEnd";

    /// Spiral's start curvature.
    double curv_start{};
    /// Spiral's end curvature.
    double curv_end{};

    /// Equality operator.
    bool operator==(const Spiral& other) const { return curv_start == other.curv_start && curv_end == other.curv_end; }
  };

  /// ParamPoly3 geometry description.
  /// Parametric cubic curve as defined in OpenDRIVE specification.
  /// See https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/09_geometries/09_06_param_poly3.html
  struct ParamPoly3 {
    /// Holds the tag names in the XODR Geometry description.
    static constexpr const char* kAU = "aU";
    static constexpr const char* kBU = "bU";
    static constexpr const char* kCU = "cU";
    static constexpr const char* kDU = "dU";
    static constexpr const char* kAV = "aV";
    static constexpr const char* kBV = "bV";
    static constexpr const char* kCV = "cV";
    static constexpr const char* kDV = "dV";
    static constexpr const char* kPRange = "pRange";

    /// Range of parameter p.
    enum class PRange {
      kArcLength,   ///< p in [0, length of geometry]
      kNormalized,  ///< p in [0, 1]
    };

    /// Polynomial coefficients for u(p) = aU + bU*p + cU*p² + dU*p³
    double aU{};
    double bU{};
    double cU{};
    double dU{};

    /// Polynomial coefficients for v(p) = aV + bV*p + cV*p² + dV*p³
    double aV{};
    double bV{};
    double cV{};
    double dV{};

    /// Range type for parameter p
    PRange p_range{PRange::kArcLength};

    /// Equality operator.
    bool operator==(const ParamPoly3& other) const {
      return aU == other.aU && bU == other.bU && cU == other.cU && dU == other.dU && aV == other.aV &&
             bV == other.bV && cV == other.cV && dV == other.dV && p_range == other.p_range;
    }
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
  std::variant<Line, Arc, Spiral, ParamPoly3> description;
};

/// Streams a string representation of @p geometry into @p os.
/// Returns @p os.
/// @relates Geometry
std::ostream& operator<<(std::ostream& os, const Geometry& geometry);

}  // namespace xodr
}  // namespace malidrive
