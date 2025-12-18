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
#include "maliput_malidrive/xodr/colors.h"

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, Color> str_to_color_map{
    {"black", Color::kBlack},   {"blue", Color::kBlue},   {"green", Color::kGreen},
    {"orange", Color::kOrange}, {"red", Color::kRed},     {"standard", Color::kStandard},
    {"violet", Color::kViolet}, {"white", Color::kWhite}, {"yellow", Color::kYellow},
};

const std::map<Color, std::string> color_to_str_map{
    {Color::kBlack, "black"},   {Color::kBlue, "blue"},   {Color::kGreen, "green"},
    {Color::kOrange, "orange"}, {Color::kRed, "red"},     {Color::kStandard, "standard"},
    {Color::kViolet, "violet"}, {Color::kWhite, "white"}, {Color::kYellow, "yellow"},
};

}  // namespace

std::string color_to_str(Color color) { return color_to_str_map.at(color); }

Color str_to_color(const std::string& color) {
  if (str_to_color_map.find(color) == str_to_color_map.end()) {
    MALIDRIVE_THROW_MESSAGE(color + " color is not available.");
  }
  return str_to_color_map.at(color);
}

}  // namespace xodr
}  // namespace malidrive
