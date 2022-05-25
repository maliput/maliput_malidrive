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

#include <vector>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Contains a series of elevation records which define the characteristics of
/// the road's elevation along the reference line.
/// For example, a XML node describing a elevationProfile header:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///         <elevationProfile>
///             <elevation s="1.12459e+1" a="3.037354772e+0" b="-3.505664e-3" c="-3.38446126e-5" d="0.00000e+0"/>
///             <elevation s="2.12459e+1" a="4.037354772e+0" b="-5.505664e-5" c="-6.38446126e-5" d="1.00000e+0"/>
///             ...
///         </elevationProfile>
///       ...
///   </OpenDRIVE>
/// @endcode
struct ElevationProfile {
  /// The elevation record defines an elevation entry at a given reference line position.
  struct Elevation {
    /// Convenient constants that hold the tag names in the XODR elevation description.
    static constexpr const char* kElevationTag = "elevation";
    static constexpr const char* kS0 = "s";
    static constexpr const char* kA = "a";
    static constexpr const char* kB = "b";
    static constexpr const char* kC = "c";
    static constexpr const char* kD = "d";
    /// Equality operator.
    bool operator==(const Elevation& other) const;

    /// Inequality operator.
    bool operator!=(const Elevation& other) const;

    /// Start position (s-coordinate).
    double s_0{};
    /// Coefficients of a cubic polynomial: @f$ a + b * p + c * p^2 + d * p^3 @f$.
    /// @{
    double a{};
    double b{};
    double c{};
    double d{};
    /// @}
  };
  static constexpr const char* kElevationProfileTag = "elevationProfile";

  /// Equality operator.
  bool operator==(const ElevationProfile& other) const { return elevations == other.elevations; }

  /// Inequality operator.
  bool operator!=(const ElevationProfile& other) const { return elevations != other.elevations; }

  /// Contains the Elevations that describe the elevation of a Road.
  std::vector<Elevation> elevations{};
};

}  // namespace xodr
}  // namespace malidrive
