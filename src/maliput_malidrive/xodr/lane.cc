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
#include "maliput_malidrive/xodr/lane.h"

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, Lane::Type> str_to_type_map{
    {"none", Lane::Type::kNone},
    {"driving", Lane::Type::kDriving},
    {"stop", Lane::Type::kStop},
    {"shoulder", Lane::Type::kShoulder},
    {"biking", Lane::Type::kBiking},
    {"sidewalk", Lane::Type::kSidewalk},
    {"border", Lane::Type::kBorder},
    {"restricted", Lane::Type::kRestricted},
    {"parking", Lane::Type::kParking},
    {"bidirectional", Lane::Type::kBidirectional},
    {"median", Lane::Type::kMedian},
    {"special1", Lane::Type::kSpecial1},
    {"special2", Lane::Type::kSpecial2},
    {"special3", Lane::Type::kSpecial3},
    {"roadworks", Lane::Type::kRoadWorks},
    {"tram", Lane::Type::kTram},
    {"rail", Lane::Type::kRail},
    {"entry", Lane::Type::kEntry},
    {"exit", Lane::Type::kExit},
    {"offRamp", Lane::Type::kOffRamp},
    {"onRamp", Lane::Type::kOnRamp},
    {"connectingRamp", Lane::Type::kConnectingRamp},
    {"bus", Lane::Type::kBus},
    {"taxi", Lane::Type::kTaxi},
    {"hov", Lane::Type::kHOV},
    {"mwyEntry", Lane::Type::kMwyEntry},
    {"mwyExit", Lane::Type::kMwyExit},
};

const std::map<Lane::Type, std::string> type_to_str_map{
    {Lane::Type::kNone, "none"},
    {Lane::Type::kDriving, "driving"},
    {Lane::Type::kStop, "stop"},
    {Lane::Type::kShoulder, "shoulder"},
    {Lane::Type::kBiking, "biking"},
    {Lane::Type::kSidewalk, "sidewalk"},
    {Lane::Type::kBorder, "border"},
    {Lane::Type::kRestricted, "restricted"},
    {Lane::Type::kParking, "parking"},
    {Lane::Type::kBidirectional, "bidirectional"},
    {Lane::Type::kMedian, "median"},
    {Lane::Type::kSpecial1, "special1"},
    {Lane::Type::kSpecial2, "special2"},
    {Lane::Type::kSpecial3, "special3"},
    {Lane::Type::kRoadWorks, "roadworks"},
    {Lane::Type::kTram, "tram"},
    {Lane::Type::kRail, "rail"},
    {Lane::Type::kEntry, "entry"},
    {Lane::Type::kExit, "exit"},
    {Lane::Type::kOffRamp, "offRamp"},
    {Lane::Type::kOnRamp, "onRamp"},
    {Lane::Type::kConnectingRamp, "connectingRamp"},
    {Lane::Type::kBus, "bus"},
    {Lane::Type::kTaxi, "taxi"},
    {Lane::Type::kHOV, "hov"},
    {Lane::Type::kMwyEntry, "mwyEntry"},
    {Lane::Type::kMwyExit, "mwyExit"},
};

}  // namespace

std::string Lane::type_to_str(Type type) { return type_to_str_map.at(type); }

Lane::Type Lane::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " lane type is not available.");
  }
  return str_to_type_map.at(type);
}

bool Lane::operator==(const Lane& other) const {
  return id == other.id && type == other.type && level == other.level && lane_link == other.lane_link &&
         width_description == other.width_description && speed == other.speed && user_data == other.user_data;
}

bool Lane::operator!=(const Lane& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
