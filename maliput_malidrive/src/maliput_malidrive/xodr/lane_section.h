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

#include <optional>
#include <vector>

#include "maliput_malidrive/xodr/lane.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR LaneSection.
/// For example, a XML node describing a XODR's lane section:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///     <road>
///       <lanes>
///         <laneSection s="0.0000000000000000e+0" singleSide="true">
///           <left>
///             <lane>
///               ...
///             </lane>
///           </left>
///           <center>
///             <lane>
///               ...
///             </lane>
///           </center>
///           <right>
///             <lane>
///               ...
///             </lane>
///           </right>
///         </laneSection>
///       </lanes>
///     <road>
///       ...
///   </OpenDRIVE>
/// @endcode
struct LaneSection {
  /// Convenient constants that hold the tag names in the XODR lane section node.
  static constexpr const char* kLaneSectionTag = "laneSection";
  static constexpr const char* kS0 = "s";
  static constexpr const char* kSingleSide = "singleSide";
  static constexpr const char* kLeft = "left";
  static constexpr const char* kCenter = "center";
  static constexpr const char* kRight = "right";

  /// Equality operator.
  bool operator==(const LaneSection& other) const;

  /// Inequality operator.
  bool operator!=(const LaneSection& other) const;

  /// Start position (s-coordinate).
  double s_0{};
  /// Lane section entry is valid for one side only.
  std::optional<bool> single_side{std::nullopt};
  /// Lanes in the left side of the road.
  std::vector<Lane> left_lanes{};
  /// Center lane of the road.
  Lane center_lane{};
  /// Lanes in the right side of the road.
  std::vector<Lane> right_lanes{};
};

}  // namespace xodr
}  // namespace malidrive
