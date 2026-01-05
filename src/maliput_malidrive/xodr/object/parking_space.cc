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
#include "maliput_malidrive/xodr/object/parking_space.h"

namespace malidrive {
namespace xodr {
namespace object {

namespace {

const std::map<ParkingSpace::Access, std::string> access_to_str_map{
    {ParkingSpace::Access::kAll, "all"},
    {ParkingSpace::Access::kBus, "bus"},
    {ParkingSpace::Access::kCar, "car"},
    {ParkingSpace::Access::kElectric, "electric"},
    {ParkingSpace::Access::kHandicapped, "handicapped"},
    {ParkingSpace::Access::kResidents, "residents"},
    {ParkingSpace::Access::kTruck, "truck"},
    {ParkingSpace::Access::kWomen, "women"},
};

const std::map<std::string, ParkingSpace::Access> str_to_access_map{
    {"all", ParkingSpace::Access::kAll},
    {"bus", ParkingSpace::Access::kBus},
    {"car", ParkingSpace::Access::kCar},
    {"electric", ParkingSpace::Access::kElectric},
    {"handicapped", ParkingSpace::Access::kHandicapped},
    {"residents", ParkingSpace::Access::kResidents},
    {"truck", ParkingSpace::Access::kTruck},
    {"women", ParkingSpace::Access::kWomen},
};

}  // namespace

std::string ParkingSpace::access_to_str(ParkingSpace::Access access) { return access_to_str_map.at(access); }

ParkingSpace::Access ParkingSpace::str_to_access(const std::string& access) {
  if (str_to_access_map.find(access) == str_to_access_map.end()) {
    MALIDRIVE_THROW_MESSAGE(access + " parking space access is not available.");
  }
  return str_to_access_map.at(access);
}

bool ParkingSpace::operator==(const ParkingSpace& other) const {
  return access == other.access && restrictions == other.restrictions;
}

bool ParkingSpace::operator!=(const ParkingSpace& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
