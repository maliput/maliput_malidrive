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

#include <string>

namespace malidrive {
namespace xodr {

/// Enumerates the units allowed in the XODR description.
enum class Unit {
  // Meters per second.
  kMs = 0,
  // Miles per hour.
  kMph,
  // Kilometers per hour.
  kKph,
};

/// Convert to meters per second.
/// @param value Is the number to be converted.
/// @param unit Is the unit of `value`.
/// @returns The equivalent in meters per second.
/// @throw maliput::common::assertion_error When `unit` isn't Unit::kMs, Unit::kMph or Unit::kph.
double ConvertToMs(double value, Unit unit);

/// Matches string with a Unit.
/// @param unit Is a Unit.
/// @returns A string that matches with `unit`.
std::string unit_to_str(Unit unit);

/// Matches Unit with a string.
/// @param unit Is a string.
/// @returns A Unit that matches with `unit`.
/// @throw maliput::common::assertion_error When `unit` doesn't match with a Unit.
Unit str_to_unit(const std::string& unit);

}  // namespace xodr
}  // namespace malidrive
