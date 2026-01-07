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

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object Border element.
struct Border {
  /// Id alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct Border>;

  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kBorderTag = "border";
  static constexpr const char* kOutlineId = "outlineId";
  static constexpr const char* kType = "type";
  static constexpr const char* kUseCompleteOutline = "useCompleteOutline";
  static constexpr const char* kWidth = "width";

  enum class Type { kConcrete, kCurb, kPaint };

  /// ID of the outline to use.
  Id outline_id{"none"};
  /// Appearance of border.
  Type type{};
  /// Use all outline points for border. “true” is used as default.
  std::optional<bool> use_complete_outline{std::nullopt};
  /// Border width.
  double width{};

  /// Matches string with a Type.
  /// @param border_type Is a Type.
  /// @returns A string that matches with `border_type`.
  static std::string border_type_to_str(Type border_type);

  /// Matches Type with a string.
  /// @param border_type Is a string.
  /// @returns A Type that matches with `Type`.
  /// @throw maliput::common::assertion_error When `border_type` doesn't match with a Type.
  static Type str_to_border_type(const std::string& border_type);

  bool operator==(const Border& other) const;
  bool operator!=(const Border& other) const;
};

/// Holds the values of a XODR Object Borders element.
struct Borders {
  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kBordersTag = "borders";
  static constexpr const char* kBorder = "border";

  /// Specifies a border along certain outline points.
  std::vector<Border> borders{};

  bool operator==(const Borders& other) const;
  bool operator!=(const Borders& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
