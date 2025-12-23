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
#include "maliput_malidrive/xodr/object/outlines.h"

namespace malidrive {
namespace xodr {
namespace object {

namespace {
const std::map<Outline::FillType, std::string> fill_type_to_str_map{
    {Outline::FillType::kAsphalt, "asphalt"},   {Outline::FillType::kCobble, "cobble"},
    {Outline::FillType::kConcrete, "concrete"}, {Outline::FillType::kGrass, "grass"},
    {Outline::FillType::kGravel, "gravel"},     {Outline::FillType::kPaint, "paint"},
    {Outline::FillType::kPavement, "pavement"}, {Outline::FillType::kSoil, "soil"},
};

const std::map<std::string, Outline::FillType> str_to_fill_type_map{
    {"asphalt", Outline::FillType::kAsphalt},   {"cobble", Outline::FillType::kCobble},
    {"concrete", Outline::FillType::kConcrete}, {"grass", Outline::FillType::kGrass},
    {"gravel", Outline::FillType::kGravel},     {"paint", Outline::FillType::kPaint},
    {"pavement", Outline::FillType::kPavement}, {"soil", Outline::FillType::kSoil},
};
}  // namespace

std::string Outline::fill_type_to_str(Outline::FillType fill_type) { return fill_type_to_str_map.at(fill_type); }

Outline::FillType Outline::str_to_fill_type(const std::string& fill_type) {
  if (str_to_fill_type_map.find(fill_type) == str_to_fill_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(fill_type + " Outline fill type is not available.");
  }
  return str_to_fill_type_map.at(fill_type);
}

bool CornerLocal::operator==(const CornerLocal& other) const {
  return height == other.height && id == other.id && u == other.u && v == other.v && z == other.z;
}

bool CornerLocal::operator!=(const CornerLocal& other) const { return !(*this == other); }

bool CornerRoad::operator==(const CornerRoad& other) const {
  return dz == other.dz && height == other.height && id == other.id && s == other.s && t == other.t;
}

bool CornerRoad::operator!=(const CornerRoad& other) const { return !(*this == other); }

bool Outline::operator==(const Outline& other) const {
  return closed == other.closed && fill_type == other.fill_type && id == other.id && lane_type == other.lane_type &&
         outer == other.outer && corner_local == other.corner_local && corner_road == other.corner_road;
}

bool Outline::operator!=(const Outline& other) const { return !(*this == other); }

bool Outlines::operator==(const Outlines& other) const { return outlines == other.outlines; }

bool Outlines::operator!=(const Outlines& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
