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
#include "maliput_malidrive/xodr/object/common.h"
#include "maliput_malidrive/xodr/object/validity.h"

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object objectReference element.
struct ObjectReference {
  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kObjectReferenceTag = "objectReference";
  static constexpr const char* kId = "id";
  static constexpr const char* kOrientation = "orientation";
  static constexpr const char* kS = "s";
  static constexpr const char* kT = "t";
  static constexpr const char* kValidLength = "validLength";
  static constexpr const char* kZOffset = "zOffset";

  /// Unique ID of the referred object within the database.
  std::string id{};
  /// Orientation.
  Orientation orientation{};
  /// m	s-coordinate.
  double s{};
  /// m	t-coordinate.
  double t{};
  /// Validity of the object along s-axis.
  std::optional<double> valid_length{std::nullopt};
  /// z offset relative to the elevation of the road reference line.
  std::optional<double> z_offset{std::nullopt};

  /// Lane validities restrict signals and objects to specific lanes.
  std::optional<Validity> validity{std::nullopt};

  bool operator==(const ObjectReference& other) const;
  bool operator!=(const ObjectReference& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
