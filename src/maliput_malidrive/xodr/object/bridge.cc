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
#include "maliput_malidrive/xodr/object/bridge.h"

namespace malidrive {
namespace xodr {
namespace object {

namespace {
const std::map<Bridge::Type, std::string> bridge_type_to_str_map = {
    {Bridge::Type::kBrick, "brick"},
    {Bridge::Type::kConcrete, "concrete"},
    {Bridge::Type::kSteel, "steel"},
    {Bridge::Type::kWood, "wood"},
};

const std::map<std::string, Bridge::Type> str_to_bridge_type_map = {
    {"brick", Bridge::Type::kBrick},
    {"concrete", Bridge::Type::kConcrete},
    {"steel", Bridge::Type::kSteel},
    {"wood", Bridge::Type::kWood},
};
}  // namespace

std::string Bridge::type_to_str(Bridge::Type type) { return bridge_type_to_str_map.at(type); }

Bridge::Type Bridge::str_to_type(const std::string& type) {
  if (str_to_bridge_type_map.find(type) == str_to_bridge_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " Tunnel type is not available.");
  }
  return str_to_bridge_type_map.at(type);
}

bool Bridge::operator==(const Bridge& other) const {
  return id == other.id && length == other.length && name == other.name && s == other.s && type == other.type &&
         validity == other.validity;
}

bool Bridge::operator!=(const Bridge& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
