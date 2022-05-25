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
#include "maliput_malidrive/xodr/road_type.h"

#include <map>
#include <string>

namespace malidrive {
namespace xodr {
namespace {

// Map for RoadType::Type to string conversion.
const std::map<RoadType::Type, std::string> type_to_str_map{{RoadType::Type::kUnknown, "unknown"},
                                                            {RoadType::Type::kRural, "rural"},
                                                            {RoadType::Type::kMotorway, "motorway"},
                                                            {RoadType::Type::kTown, "town"},
                                                            {RoadType::Type::kLowSpeed, "lowSpeed"},
                                                            {RoadType::Type::kPedestrian, "pedestrian"},
                                                            {RoadType::Type::kBicycle, "bicycle"},
                                                            {RoadType::Type::kTownExpressway, "townExpressway"},
                                                            {RoadType::Type::kTownCollector, "townCollector"},
                                                            {RoadType::Type::kTownArterial, "townArterial"},
                                                            {RoadType::Type::kTownPrivate, "townPrivate"},
                                                            {RoadType::Type::kTownLocal, "townLocal"},
                                                            {RoadType::Type::kTownPlayStreet, "townPlayStreet"}};

// Map for string to RoadType::Type conversion.
const std::map<std::string, RoadType::Type> str_to_type_map{{"unknown", RoadType::Type::kUnknown},
                                                            {"rural", RoadType::Type::kRural},
                                                            {"motorway", RoadType::Type::kMotorway},
                                                            {"town", RoadType::Type::kTown},
                                                            {"lowSpeed", RoadType::Type::kLowSpeed},
                                                            {"pedestrian", RoadType::Type::kPedestrian},
                                                            {"bicycle", RoadType::Type::kBicycle},
                                                            {"townExpressway", RoadType::Type::kTownExpressway},
                                                            {"townCollector", RoadType::Type::kTownCollector},
                                                            {"townArterial", RoadType::Type::kTownArterial},
                                                            {"townPrivate", RoadType::Type::kTownPrivate},
                                                            {"townLocal", RoadType::Type::kTownLocal},
                                                            {"townPlayStreet", RoadType::Type::kTownPlayStreet}};

}  // namespace

std::string RoadType::type_to_str(RoadType::Type type) { return type_to_str_map.at(type); }

RoadType::Type RoadType::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " type is not available.");
  }
  return str_to_type_map.at(type);
}

bool RoadType::operator==(const RoadType& other) const {
  return s_0 == other.s_0 && type == other.type && country == other.country && speed == other.speed;
}

bool RoadType::operator!=(const RoadType& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
