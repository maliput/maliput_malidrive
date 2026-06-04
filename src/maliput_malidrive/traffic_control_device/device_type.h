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

namespace malidrive {
namespace traffic_control_device {

/// Identifies the category of a traffic control device.
///
/// Parsed from the `device_type` field of the YAML database `properties` section.
enum class TrafficControlDeviceType {
  kTrafficLight,  ///< Device is a traffic light ("TrafficLight").
  kTrafficSign,   ///< Device is a traffic sign ("TrafficSign").
  kRoadMarking,   ///< Device is a road marking ("RoadMarking").
  kRoadObject,    ///< Device is a road object ("RoadObject").
  kUnknown,       ///< Unrecognized or missing device_type value.
};

/// Converts a `device_type` string from the YAML database to a
/// @ref TrafficControlDeviceType enum value.
///
/// Recognized values (strict CamelCase):
///   - `"TrafficLight"` → `kTrafficLight`
///   - `"TrafficSign"`  → `kTrafficSign`
///   - `"RoadMarking"`  → `kRoadMarking`
///   - `"RoadObject"`   → `kRoadObject`
///
/// Any other string maps to `kUnknown`.
///
/// @param device_type_str  The `device_type` string from the YAML `properties` section.
/// @returns The corresponding @ref TrafficControlDeviceType.
TrafficControlDeviceType StringToTrafficControlDeviceType(const std::string& device_type_str);

/// Converts a @ref TrafficControlDeviceType enum value back to its canonical
/// YAML string representation.
///
///   - `kTrafficLight` → `"TrafficLight"`
///   - `kTrafficSign`  → `"TrafficSign"`
///   - `kRoadMarking`  → `"RoadMarking"`
///   - `kRoadObject`   → `"RoadObject"`
///   - `kUnknown`      → `"Unknown"`
///
/// @param device_type  The @ref TrafficControlDeviceType to convert.
/// @returns The corresponding string.
const char* TrafficControlDeviceTypeToString(TrafficControlDeviceType device_type);

}  // namespace traffic_control_device
}  // namespace malidrive
