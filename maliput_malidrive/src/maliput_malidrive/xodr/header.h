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
#include <ostream>
#include <string>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR description's header.
/// For example, a XML node describing a XODR's header:
/// @code{.xml}
///   <OpenDRIVE>
///       <header revMajor="1" revMinor="1" name="ExampleMap" version="1.00" date="Fri May 08 12:00:00 2020"
///       north="0.0000000000000000e+00" south="0.0000000000000000e+00" east="0.0000000000000000e+00"
///       west="0.0000000000000000e+00" vendor="VendorName">
///       </header>
///       ...
///       ...
///   </OpenDRIVE>
/// @endcode
struct Header {
  /// Convenient constants that hold the tag names in the XODR header description.
  static constexpr const char* kHeaderTag = "header";
  static constexpr const char* kXodrRevMajor = "revMajor";
  static constexpr const char* kXodrRevMinor = "revMinor";
  static constexpr const char* kXodrName = "name";
  static constexpr const char* kXodrVersion = "version";
  static constexpr const char* kXodrDate = "date";
  static constexpr const char* kXodrNorth = "north";
  static constexpr const char* kXodrSouth = "south";
  static constexpr const char* kXodrEast = "east";
  static constexpr const char* kXodrWest = "west";
  static constexpr const char* kXodrVendor = "vendor";

  /// Equality operator.
  bool operator==(const Header& other) const;

  /// Inequality operator.
  bool operator!=(const Header& other) const;

  /// Major revision number of Malidrive format.
  double rev_major{};
  /// Minor revision number of Malidrive format.
  double rev_minor{};
  /// Database name.
  std::optional<std::string> name{std::nullopt};
  /// Database's version number.
  std::optional<double> version{std::nullopt};
  /// Time/date of database creation according to ISO 8601.
  /// TODO(francocipollone): Use std::chrono::system_clock::time_point instead of std::string.
  std::optional<std::string> date{std::nullopt};
  /// Maximum inertial `y` value.
  std::optional<double> north{std::nullopt};
  /// Minimum inertial `y` value.
  std::optional<double> south{std::nullopt};
  /// Maximum inertial `x` value.
  std::optional<double> east{std::nullopt};
  /// Minimum inertial `x` value.
  std::optional<double> west{std::nullopt};
  /// Vendor name.
  std::optional<std::string> vendor{std::nullopt};
};

/// Streams a string representation of @p header into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const Header& header);

}  // namespace xodr
}  // namespace malidrive
