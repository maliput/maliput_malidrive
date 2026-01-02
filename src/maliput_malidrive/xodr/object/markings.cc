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
#include "maliput_malidrive/xodr/object/markings.h"

namespace malidrive {
namespace xodr {
namespace object {

namespace {

const std::map<Marking::Side, std::string> side_to_str_map{
    {Marking::Side::kLeft, "left"},
    {Marking::Side::kRight, "right"},
    {Marking::Side::kFront, "front"},
    {Marking::Side::kRear, "rear"},
};

const std::map<std::string, Marking::Side> str_to_side_map{
    {"left", Marking::Side::kLeft},
    {"right", Marking::Side::kRight},
    {"front", Marking::Side::kFront},
    {"rear", Marking::Side::kRear},
};

}  // namespace

std::string side_to_str(Marking::Side side) { return side_to_str_map.at(side); }

Marking::Side str_to_side(const std::string& side) {
  if (str_to_side_map.find(side) == str_to_side_map.end()) {
    MALIDRIVE_THROW_MESSAGE(side + " marking side is not available.");
  }
  return str_to_side_map.at(side);
}

bool Marking::operator==(const Marking& other) const {
  return color == other.color && line_length == other.line_length && side == other.side &&
         space_length == other.space_length && start_offset == other.start_offset && stop_offset == other.stop_offset &&
         weight == other.weight && width == other.width && z_offset == other.z_offset &&
         corner_reference == other.corner_reference;
}

bool Marking::operator!=(const Marking& other) const { return !(*this == other); }

bool Markings::operator==(const Markings& other) const { return markings == other.markings; }

bool Markings::operator!=(const Markings& other) const { return !(*this == other); }

bool CornerReference::operator==(const CornerReference& other) const { return id == other.id; }

bool CornerReference::operator!=(const CornerReference& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
