// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include "maliput_malidrive/builder/road_object_type_mapper.h"

namespace malidrive {
namespace builder {

maliput::api::objects::RoadObjectType MapXodrObjectType(
    const std::optional<xodr::object::Object::ObjectType>& xodr_type) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  if (!xodr_type.has_value()) {
    return MaliputType::kUnknown;
  }

  switch (xodr_type.value()) {
    case XodrType::kBarrier:
    case XodrType::kSoundBarrier:
    case XodrType::kRailing:
      return MaliputType::kBarrier;
    case XodrType::kBuilding:
      return MaliputType::kBuilding;
    case XodrType::kCrosswalk:
      return MaliputType::kCrosswalk;
    case XodrType::kGantry:
      return MaliputType::kGantry;
    case XodrType::kObstacle:
      return MaliputType::kObstacle;
    case XodrType::kParkingSpace:
      return MaliputType::kParkingSpace;
    case XodrType::kPole:
    case XodrType::kWind:
    case XodrType::kStreetLamp:
      return MaliputType::kPole;
    case XodrType::kRoadMark:
      return MaliputType::kRoadMark;
    case XodrType::kRoadSurface:
    case XodrType::kPatch:
      return MaliputType::kRoadSurface;
    case XodrType::kTrafficIsland:
      return MaliputType::kTrafficIsland;
    case XodrType::kTree:
      return MaliputType::kTree;
    case XodrType::kVegetation:
      return MaliputType::kVegetation;
    // Deprecated types map to kUnknown.
    case XodrType::kBike:
    case XodrType::kBus:
    case XodrType::kCar:
    case XodrType::kMotorbike:
    case XodrType::kPedestrian:
    case XodrType::kTrailer:
    case XodrType::kTrain:
    case XodrType::kTram:
    case XodrType::kVan:
    case XodrType::kNone:
      return MaliputType::kUnknown;
  }
  return MaliputType::kUnknown;  // Fallback for any future additions.
}

}  // namespace builder
}  // namespace malidrive
