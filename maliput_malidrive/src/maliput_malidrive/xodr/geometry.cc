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
#include "maliput_malidrive/xodr/geometry.h"

namespace malidrive {
namespace xodr {
namespace {

// Map for Type to string conversion.
const std::map<Geometry::Type, std::string> type_to_str_map{{Geometry::Type::kLine, "line"},
                                                            {Geometry::Type::kArc, "arc"}};

// Map for string to Type conversion.
const std::map<std::string, Geometry::Type> str_to_type_map{{"line", Geometry::Type::kLine},
                                                            {"arc", Geometry::Type::kArc}};

}  // namespace

std::string Geometry::type_to_str(Geometry::Type type) { return type_to_str_map.at(type); }

Geometry::Type Geometry::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " geometry type is not available.");
  }
  return str_to_type_map.at(type);
}

bool Geometry::operator==(const Geometry& other) const {
  return s_0 == other.s_0 && start_point == other.start_point && orientation == other.orientation &&
         length == other.length && type == other.type && description == other.description;
}

bool Geometry::operator!=(const Geometry& other) const { return !(*this == other); }

std::ostream& operator<<(std::ostream& os, const Geometry& geometry) {
  os << "Geometry type: " << Geometry::type_to_str(geometry.type);
  switch (geometry.type) {
    case Geometry::Type::kArc:
      os << " - curvature: " << std::get<xodr::Geometry::Arc>(geometry.description).curvature;
      break;
    case Geometry::Type::kLine:
      break;
    default:
      MALIPUT_THROW_MESSAGE("Unknown Geometry::Type");
      break;
  }
  os << " | s: " << geometry.s_0 << " | {x, y} : " << geometry.start_point << " | hdg: " << geometry.orientation;
  os << "\n";
  return os;
}

}  // namespace xodr
}  // namespace malidrive
