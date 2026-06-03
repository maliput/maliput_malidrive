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

#include <string>

#include <maliput/api/objects/road_marking.h>

#include "maliput_malidrive/xodr/object/object.h"

namespace malidrive {
namespace builder {

/// Maps a device_semantics string from the YAML traffic control device database to a
/// @ref maliput::api::objects::RoadMarkingType enum.
///
/// The comparison is case-insensitive: "Stop_Line", "STOP_LINE", and "stop_line"
/// all map to RoadMarkingType::kStopLine.
///
/// Any unrecognized string (including "none" and "other") maps to kUnknown.
///
/// @param device_semantics The device_semantics value from the YAML database.
/// @returns The corresponding RoadMarkingType, or kUnknown if not recognized.
maliput::api::objects::RoadMarkingType MapRoadMarkingTypeString(const std::string& device_semantics);

/// Maps an optional XODR @ref xodr::object::Object::ObjectType to a
/// @ref maliput::api::objects::RoadMarkingType.
///
/// When @p xodr_type is kRoadMark and @p name or @p subtype equals
/// "stopLine", the result is kStopLine instead of kRoadMark.
///
/// @param xodr_type The optional XODR object type to map.
/// @param subtype The optional object subtype.
/// @returns The corresponding maliput RoadMarkingType.
maliput::api::objects::RoadMarkingType MapXodrObjectTypeToRoadMarkingType(
    const std::optional<xodr::object::Object::ObjectType>& xodr_type,
    const std::optional<std::string>& subtype = std::nullopt);
}  // namespace builder
}  // namespace malidrive
