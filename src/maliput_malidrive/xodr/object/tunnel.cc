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
#include "maliput_malidrive/xodr/object/tunnel.h"

namespace malidrive {
namespace xodr {
namespace object {

namespace {
const std::map<Tunnel::Type, std::string> tunnel_type_to_str_map = {
    {Tunnel::Type::kStandard, "standard"},
    {Tunnel::Type::kUnderpass, "underpass"},
};

const std::map<std::string, Tunnel::Type> str_to_tunnel_type_map = {
    {"standard", Tunnel::Type::kStandard},
    {"underpass", Tunnel::Type::kUnderpass},
};
}  // namespace

std::string Tunnel::type_to_str(Tunnel::Type type) { return tunnel_type_to_str_map.at(type); }

Tunnel::Type Tunnel::str_to_type(const std::string& type) {
  if (str_to_tunnel_type_map.find(type) == str_to_tunnel_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " Tunnel type is not available.");
  }
  return str_to_tunnel_type_map.at(type);
}

bool Tunnel::operator==(const Tunnel& other) const {
  return daylight == other.daylight && id == other.id && length == other.length && lighting == other.lighting &&
         name == other.name && s == other.s && type == other.type && validity == other.validity;
}

bool Tunnel::operator!=(const Tunnel& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
