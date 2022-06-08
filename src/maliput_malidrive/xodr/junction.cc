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
#include "maliput_malidrive/xodr/junction.h"

#include <map>

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, Junction::Type> str_to_type_map{{"default", Junction::Type::kDefault},
                                                            {"virtual", Junction::Type::kVirtual}};

const std::map<Junction::Type, std::string> type_to_str_map{{Junction::Type::kDefault, "default"},
                                                            {Junction::Type::kVirtual, "virtual"}};

}  // namespace

Junction::Type Junction::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " type is not available.");
  }
  return str_to_type_map.at(type);
}

std::string Junction::type_to_str(Type type) { return type_to_str_map.at(type); }

bool Junction::operator==(const Junction& other) const {
  return id == other.id && name == other.name && type == other.type && connections == other.connections;
}

std::ostream& operator<<(std::ostream& out, const Junction& junction) {
  out << "{\"id\": " << junction.id.string();
  out << ", \"name\": " << (junction.name.has_value() ? junction.name.value() : "");
  out << ", \"type\": {" << (junction.type.has_value() ? Junction::type_to_str(junction.type.value()) : "") << "}";
  out << "}";
  // TODO(malidrive#574): Once other entities acquire serialization overloads, we should include them.
  return out;
}

}  // namespace xodr
}  // namespace malidrive
