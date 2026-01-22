// BSD 3-Clause License
//
// Copyright (c) 2026, Woven Planet. All rights reserved.
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

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Signal Offset.
/// For example, a XML node describing a XODR's signal offset:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///       <signals>
///           <signal s="3981.4158159146"
///                   t="-14.0503"
///                   id="5000162"
///                   name="Vorschriftzeichen"
///                   dynamic="no"
///                   orientation="+"
///                   zOffset="3.8835"
///                   country="DE"
///                   countryRevision="2017"
///                   type="274"
///                   subtype="100"
///                   value="100"
///                   unit="km/h"
///                   height="0.77"
///                   width="0.77"
///                   hOffset="5.7595865"
///                   length="1.50"
///                   pitch="0.77"
///                   roll="0.77"
///                   text="custom signal text">
///           </signal>
///       </signals>
///       ...
///   </OpenDRIVE>
/// @endcode
struct Signal {
  /// Holds the value and unit of a XODR Signal.
  /// If @value is given, @unit is mandatory.
  struct Value {
    static constexpr const char* kValue = "value";
    static constexpr const char* kUnit = "unit";
    /// Equality operator.
    bool operator==(const Value& other) const { return value == other.value && unit == other.unit; }
    /// Inequality operator.
    bool operator!=(const Value& other) const { return !(*this == other); }

    /// Value of the signal.
    double value{};
    /// Unit of @value.
    std::string unit{};
  };

  /// Convenient constants that hold the tag names in the XODR signal description.
  static constexpr const char* kSignalTag = "signal";
  static constexpr const char* kS = "s";
  static constexpr const char* kT = "t";
  static constexpr const char* kId = "id";
  static constexpr const char* kName = "name";
  static constexpr const char* kDynamic = "dynamic";
  static constexpr const char* kOrientation = "orientation";
  static constexpr const char* kZOffset = "zOffset";
  static constexpr const char* kCountry = "country";
  static constexpr const char* kCountryRevision = "countryRevision";
  static constexpr const char* kType = "type";
  static constexpr const char* kSubtype = "subtype";
  static constexpr const char* kValue = "value";
  static constexpr const char* kHeight = "height";
  static constexpr const char* kWidth = "width";
  static constexpr const char* kHOffset = "hOffset";
  static constexpr const char* kLength = "length";
  static constexpr const char* kPitch = "pitch";
  static constexpr const char* kRoll = "roll";
  static constexpr const char* kText = "text";

  /// Equality operator.
  bool operator==(const Signal& other) const;

  /// Inequality operator.
  bool operator!=(const Signal& other) const;

  /// s-coordinate of signal's origin.
  double s{};

  /// t-coordinate of signal's origin.
  double t{};

  /// Unique ID of the signal within the OpenDRIVE file.
  std::string id{};

  /// Name of the signal. May be chosen freely.
  std::optional<std::string> name{std::nullopt};

  /// Indicates whether the signal is dynamic or static.
  bool dynamic{};

  /// "+" = valid in positive s-direction, "-" = valid in negative s-direction,
  /// "none" = valid in both directions.
  std::string orientation{};

  /// z-offset of signal's origin relative to the elevation of the road reference line.
  double z_offset{};

  /// Country code of the road, see ISO 3166-1, alpha-2 codes.
  std::optional<std::string> country{std::nullopt};

  /// Defines the year of the applied traffic rules.
  std::optional<std::string> country_revision{std::nullopt};

  /// Type identifier according to country code or "-1" / "none".
  std::string type{};

  /// Subtype identifier according to country code or "-1" / "none".
  std::string subtype{};

  /// Value of the signal, if value is given, unit is mandatory.
  std::optional<Value> value{std::nullopt};

  /// Height of the signal, measured from bottom edge of the signal.
  std::optional<double> height{std::nullopt};

  /// Width of the signal's bounding box, defined in the local coordinate system u/v along the v-axis.
  std::optional<double> width{std::nullopt};

  /// Heading offset of the signal relative to @orientation or road reference line.
  std::optional<double> h_offset{std::nullopt};

  /// Length of the signal's bounding box, defined in the local coordinate system u/v along the u-axis.
  std::optional<double> length{std::nullopt};

  /// Pitch angle of the signal, relative to the inertial system (xy-plane).
  std::optional<double> pitch{std::nullopt};

  /// Roll angle of the signal after applying pitch, relative to the inertial system (x''y''-plane).
  std::optional<double> roll{std::nullopt};

  /// Additional text associated with the signal.
  std::optional<std::string> text{std::nullopt};
};

}  // namespace xodr
}  // namespace malidrive
