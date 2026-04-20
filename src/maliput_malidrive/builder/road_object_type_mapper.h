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
#pragma once

#include <optional>
#include <string>

#include <maliput/api/objects/road_object.h>

#include "maliput_malidrive/xodr/object/object.h"

namespace malidrive {
namespace builder {

/// Maps an optional XODR @ref xodr::object::Object::ObjectType to a
/// @ref maliput::api::objects::RoadObjectType.
///
/// XODR types that have a direct maliput counterpart are mapped 1:1.
/// Vehicle-like types (kBike, kBus, kCar, kMotorbike, kPedestrian, kTrailer,
/// kTrain, kTram, kVan), and kNone are mapped to kUnknown.
/// kStreetLamp/kWind → kPole, kRailing/kSoundBarrier → kBarrier, kPatch → kRoadSurface.
/// A std::nullopt input maps to kUnknown.
///
/// When @p xodr_type is kRoadMark and @p name or @p subtype equals
/// "stopLine", the result is kStopLine instead of kRoadMark.
///
/// @param xodr_type The optional XODR object type to map.
/// @param name The optional object name.
/// @param subtype The optional object subtype.
/// @returns The corresponding maliput RoadObjectType.
maliput::api::objects::RoadObjectType MapXodrObjectType(
    const std::optional<xodr::object::Object::ObjectType>& xodr_type,
    const std::optional<std::string>& name = std::nullopt, const std::optional<std::string>& subtype = std::nullopt);

}  // namespace builder
}  // namespace malidrive
