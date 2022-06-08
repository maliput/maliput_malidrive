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

#include "maliput_malidrive/xodr/lane_offset.h"
#include "maliput_malidrive/xodr/lane_section.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Lanes node.
/// For example, a XML node describing a XODR's lanes node:
/// @code{.xml}
///  <OpenDRIVE>
///       ...
///   <lanes>
///      <laneOffset s="0.0000000000000000e+0" a="-4.000000000e+0" b="0.000000000e+0" c="0.0000000e+0"
///      d="0.0000000e+0"/> <laneOffset s="3.8268524704053952e-2" a="-4.000000000e+0" b="0.000000000e+0"
///      c="0.0000000e+0" d="0.0000000e+0"/> <laneSection s="0.0000000000000000e+0">
///        ...
///      </laneSection>
///      <laneSection s="3.8268524704053952e-2">
///        ...
///      </laneSection>
///        ...
///   </lanes>
///     ...
///  </OpenDRIVE>
/// @endcode
struct Lanes {
  /// Hold the tag for the lanes in the XODR lanes description.
  static constexpr const char* kLanesTag = "lanes";

  /// Equality operator.
  bool operator==(const Lanes& other) const;

  /// Inequality operator.
  bool operator!=(const Lanes& other) const;

  /// Holds all the `LaneOffset`s in the road.
  std::vector<LaneOffset> lanes_offset{};
  /// Holds the `LaneSection`s that there are in the road.
  std::vector<LaneSection> lanes_section{};
};

}  // namespace xodr
}  // namespace malidrive
