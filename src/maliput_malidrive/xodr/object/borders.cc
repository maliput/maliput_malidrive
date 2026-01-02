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
#include "maliput_malidrive/xodr/object/borders.h"

namespace malidrive {
namespace xodr {
namespace object {

namespace {

const std::map<Border::Type, std::string> border_type_to_str_map{
    {Border::Type::kConcrete, "concrete"},
    {Border::Type::kCurb, "curb"},
    {Border::Type::kPaint, "paint"},
};

const std::map<std::string, Border::Type> str_to_border_type_map{
    {"concrete", Border::Type::kConcrete},
    {"curb", Border::Type::kCurb},
    {"paint", Border::Type::kPaint},
};

}  // namespace

std::string Border::border_type_to_str(Border::Type border_type) { return border_type_to_str_map.at(border_type); }

Border::Type Border::str_to_border_type(const std::string& border_type) {
  if (str_to_border_type_map.find(border_type) == str_to_border_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(border_type + " border type is not available.");
  }
  return str_to_border_type_map.at(border_type);
}

bool Border::operator==(const Border& other) const {
  return outline_id == other.outline_id && type == other.type && use_complete_outline == other.use_complete_outline &&
         width == other.width;
}

bool Border::operator!=(const Border& other) const { return !(*this == other); }

bool Borders::operator==(const Borders& other) const { return borders == other.borders; }

bool Borders::operator!=(const Borders& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
