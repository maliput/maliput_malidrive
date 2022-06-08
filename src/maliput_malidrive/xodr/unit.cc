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
#include "maliput_malidrive/xodr/unit.h"

#include <map>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {
namespace {

// Map for Unit to string conversion.
const std::map<Unit, std::string> unit_to_str_map{{Unit::kMs, "m/s"}, {Unit::kMph, "mph"}, {Unit::kKph, "km/h"}};

// Map for string to Unit conversion.
const std::map<std::string, Unit> str_to_unit_map{{"m/s", Unit::kMs}, {"mph", Unit::kMph}, {"km/h", Unit::kKph}};

}  // namespace

std::string unit_to_str(Unit unit) { return unit_to_str_map.at(unit); }

Unit str_to_unit(const std::string& unit) {
  if (str_to_unit_map.find(unit) == str_to_unit_map.end()) {
    MALIDRIVE_THROW_MESSAGE(unit + " unit type is not available.");
  }
  return str_to_unit_map.at(unit);
}

double ConvertToMs(double value, Unit unit) {
  const double kKphFactor{10. / 36.};
  const double kMphFactor{1609.344 / 3600.};
  switch (unit) {
    case Unit::kKph:
      return value * kKphFactor;
    case Unit::kMph:
      return value * kMphFactor;
    case Unit::kMs:
      return value;
    default:
      MALIDRIVE_THROW_MESSAGE("This unit conversion is not supported.");
  }
}

}  // namespace xodr
}  // namespace malidrive
