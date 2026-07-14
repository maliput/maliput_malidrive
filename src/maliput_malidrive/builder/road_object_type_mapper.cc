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

// These subtype strings are based on OpenDRIVE 1.8.0 specification.
// See
// https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_14_object_examples.html
static const char* kGuardRailSubtype = "guardRail";
static const char* kRailingSubtype = "railing";
static const char* kPedestrianSubtype = "pedestrian";
static const char* kBicycleSubtype = "bicycle";
static const char* kWindSubtype = "wind";
static const char* kNoiseProtectionsSubtype = "noiseProtections";
static const char* kPatchSubtype = "patch";
static const char* kWallSubtype = "wall";
static const char* kJerseySubtype = "jerseyBarrier";
static const char* kRoadBlockageSubtype = "roadBlockage";
static const char* kStreetLampSubtype = "streetLamp";
static const char* kPermanentDelineatorSubtype = "permanentDelineator";

maliput::api::objects::RoadObjectType MapXodrObjectType(
    const std::optional<xodr::object::Object::ObjectType>& xodr_type, std::optional<std::string> xodr_subtype) {
  using XodrType = xodr::object::Object::ObjectType;
  using MaliputType = maliput::api::objects::RoadObjectType;

  if (!xodr_type.has_value()) {
    return MaliputType::kUnknown;
  }

  const std::string subtype = xodr_subtype.value_or("");

  switch (xodr_type.value()) {
    // @{
    case XodrType::kBarrier:
      if (subtype == kGuardRailSubtype) return MaliputType::kGuardRail;
      if (subtype == kNoiseProtectionsSubtype) return MaliputType::kSoundBarrier;
      if (subtype == kWallSubtype || subtype == kJerseySubtype) return MaliputType::kGuardWall;
      if (subtype == kRailingSubtype) return MaliputType::kRailing;
      // else, fall through to default barrier type.
      return MaliputType::kBarrier;
    case XodrType::kSoundBarrier:
      return MaliputType::kSoundBarrier;
    case XodrType::kRailing:
      return MaliputType::kRailing;
    // @}
    // @{
    case XodrType::kBuilding:
      return MaliputType::kBuilding;
    // @}
    case XodrType::kGantry:
      return MaliputType::kGantry;
    // @}
    // @{
    case XodrType::kObstacle:
      if (subtype == kRoadBlockageSubtype) return MaliputType::kPylon;
      return MaliputType::kObstacle;
      // @}
      // @{
    case XodrType::kPole:
      if (subtype == kStreetLampSubtype) return MaliputType::kStreetLamp;
      if (subtype == kWindSubtype) return MaliputType::kWind;
      if (subtype == kPermanentDelineatorSubtype) return MaliputType::kDelineator;
      return MaliputType::kPole;
    case XodrType::kWind:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kWind;
    case XodrType::kStreetLamp:
      return MaliputType::kStreetLamp;
    // @}
    // @{
    case XodrType::kRoadSurface:
      if (subtype == kPatchSubtype) return MaliputType::kPatch;
    case XodrType::kTrafficIsland:
      return MaliputType::kTrafficIsland;
    // @}
    // @{
    case XodrType::kTree:
      return MaliputType::kTree;
    // @}
    // @{
    case XodrType::kVegetation:
      return MaliputType::kVegetation;
    // @}
    // These types are now covered by RoadMarkingTypes, so we map them to kUnknown to avoid misclassification.
    // @{
    case XodrType::kCrosswalk:
      if (subtype == kPedestrianSubtype) return MaliputType::kPedestrianStatic;
      if (subtype == kBicycleSubtype) return MaliputType::kBikeStatic;
    case XodrType::kParkingSpace:
    case XodrType::kRoadMark:
      return MaliputType::kUnknown;
    // @}
    // These types are now covered by specific vehicle static types.
    // @{
    case XodrType::kBike:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kBikeStatic;
    case XodrType::kBus:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kBusStatic;
    case XodrType::kCar:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kCarStatic;
    case XodrType::kMotorbike:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kMotorbikeStatic;
    case XodrType::kPedestrian:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kPedestrianStatic;
    case XodrType::kTrailer:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kTrailerStatic;
    case XodrType::kTrain:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kTrainStatic;
    case XodrType::kTram:
      // Valid for OpenDRIVE version < 1.5.0
      return MaliputType::kTramStatic;
    case XodrType::kVan:
      return MaliputType::kVanStatic;
    // @}
    // Patch maps to kPatch.
    // @{
    case XodrType::kPatch:
      return MaliputType::kPatch;
    // @}
    // None maps to kUnknown.
    // @{
    case XodrType::kNone:
      return MaliputType::kUnknown;
      // @}
  }
  return MaliputType::kUnknown;  // Fallback for any future additions.
}

}  // namespace builder
}  // namespace malidrive
