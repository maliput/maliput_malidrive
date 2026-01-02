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
#include "maliput_malidrive/xodr/object/object.h"

namespace malidrive {
namespace xodr {
namespace object {

namespace {

const std::map<Object::ObjectType, std::string> object_type_to_str_map{
    {Object::ObjectType::kBarrier, "barrier"},
    {Object::ObjectType::kBike, "bike"},
    {Object::ObjectType::kBuilding, "building"},
    {Object::ObjectType::kBus, "bus"},
    {Object::ObjectType::kCar, "car"},
    {Object::ObjectType::kCrosswalk, "crosswalk"},
    {Object::ObjectType::kGantry, "gantry"},
    {Object::ObjectType::kMotorbike, "motorbike"},
    {Object::ObjectType::kNone, "none"},
    {Object::ObjectType::kObstacle, "obstacle"},
    {Object::ObjectType::kParkingSpace, "parkingSpace"},
    {Object::ObjectType::kPatch, "patch"},
    {Object::ObjectType::kPedestrian, "pedestrian"},
    {Object::ObjectType::kPole, "pole"},
    {Object::ObjectType::kRailing, "railing"},
    {Object::ObjectType::kRoadMark, "roadMark"},
    {Object::ObjectType::kRoadSurface, "roadSurface"},
    {Object::ObjectType::kSoundBarrier, "soundBarrier"},
    {Object::ObjectType::kStreetLamp, "streetLamp"},
    {Object::ObjectType::kTrafficIsland, "trafficIsland"},
    {Object::ObjectType::kTrailer, "trailer"},
    {Object::ObjectType::kTrain, "train"},
    {Object::ObjectType::kTram, "tram"},
    {Object::ObjectType::kTree, "tree"},
    {Object::ObjectType::kVan, "van"},
    {Object::ObjectType::kVegetation, "vegetation"},
    {Object::ObjectType::kWind, "wind"},
};

const std::map<std::string, Object::ObjectType> str_to_object_type_map{
    {"barrier", Object::ObjectType::kBarrier},
    {"bike", Object::ObjectType::kBike},
    {"building", Object::ObjectType::kBuilding},
    {"bus", Object::ObjectType::kBus},
    {"car", Object::ObjectType::kCar},
    {"crosswalk", Object::ObjectType::kCrosswalk},
    {"gantry", Object::ObjectType::kGantry},
    {"motorbike", Object::ObjectType::kMotorbike},
    {"none", Object::ObjectType::kNone},
    {"obstacle", Object::ObjectType::kObstacle},
    {"parkingSpace", Object::ObjectType::kParkingSpace},
    {"patch", Object::ObjectType::kPatch},
    {"pedestrian", Object::ObjectType::kPedestrian},
    {"pole", Object::ObjectType::kPole},
    {"railing", Object::ObjectType::kRailing},
    {"roadMark", Object::ObjectType::kRoadMark},
    {"roadSurface", Object::ObjectType::kRoadSurface},
    {"soundBarrier", Object::ObjectType::kSoundBarrier},
    {"streetLamp", Object::ObjectType::kStreetLamp},
    {"trafficIsland", Object::ObjectType::kTrafficIsland},
    {"trailer", Object::ObjectType::kTrailer},
    {"train", Object::ObjectType::kTrain},
    {"tram", Object::ObjectType::kTram},
    {"tree", Object::ObjectType::kTree},
    {"van", Object::ObjectType::kVan},
    {"vegetation", Object::ObjectType::kVegetation},
    {"wind", Object::ObjectType::kWind},
};

}  // namespace

std::string Object::object_type_to_str(Object::ObjectType object_type) {
  return object_type_to_str_map.at(object_type);
}

Object::ObjectType Object::str_to_object_type(const std::string& object_type) {
  if (str_to_object_type_map.find(object_type) == str_to_object_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(object_type + " Object type is not available.");
  }
  return str_to_object_type_map.at(object_type);
}

bool Object::operator==(const Object& other) const {
  // TODO
}

bool Object::operator!=(const Object& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive
