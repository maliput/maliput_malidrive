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

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds the lane link description in a XODR.
/// For example:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///        <link>
///            <predecessor id="-1"/>
///            <successor id="-1"/>
///        </link>
///       ...
///   </OpenDRIVE>
/// @endcode
struct LaneLink {
  /// Convenient constants that hold the tag names in the XODR lane link header description.
  static constexpr const char* kLaneLinkTag = "link";
  static constexpr const char* kPredecessorTag = "predecessor";
  static constexpr const char* kSuccessorTag = "successor";

  /// Contains the information about the predecessor/successor road.
  struct LinkAttributes {
    using Id = maliput::api::TypeSpecificIdentifier<struct LinkAttributes>;
    /// Convenient constants that hold the tag names in the XODR lane link description.
    static constexpr const char* kId = "id";
    /// ID of the linked element.
    Id id{"none"};
    /// Equality operator.
    bool operator==(const LinkAttributes& other) const { return id == other.id; };
  };

  /// Equality operator.
  bool operator==(const LaneLink& other) const {
    return successor == other.successor && predecessor == other.predecessor;
  }
  /// Inequality operator.
  bool operator!=(const LaneLink& other) const { return !(*this == other); }

  /// Road's predecessor.
  std::optional<LinkAttributes> predecessor{std::nullopt};
  /// Road's successor.
  std::optional<LinkAttributes> successor{std::nullopt};
};

}  // namespace xodr
}  // namespace malidrive
