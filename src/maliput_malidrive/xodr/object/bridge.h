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
#include <string>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/object/validity.h"

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object bridge element.
struct Bridge {
  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kBridgeTag = "bridge";
  static constexpr const char* kId = "id";
  static constexpr const char* kLength = "length";
  static constexpr const char* kName = "name";
  static constexpr const char* kS = "s";
  static constexpr const char* kType = "type";

  enum class Type {
    kBrick,
    kConcrete,
    kSteel,
    kWood,
  };

  /// Unique ID within database.
  std::string id;
  /// Length of the tunnel (in s-direction).
  double length;
  /// Name of the tunnel. May be chosen freely.
  std::optional<std::string> name{std::nullopt};
  /// Starting coordinate.
  double s;
  /// Type of tunnel.
  Type type;

  /// Lane validities restrict signals and objects to specific lanes.
  std::optional<Validity> validity{std::nullopt};

  /// Matches string with a Type.
  /// @param type Is a Type.
  /// @returns A string that matches with `type`.
  static std::string type_to_str(Type type);

  /// Matches Type with a string.
  /// @param type Is a string.
  /// @returns A Type that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a Type.
  static Type str_to_type(const std::string& type);

  bool operator==(const Bridge& other) const;
  bool operator!=(const Bridge& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
