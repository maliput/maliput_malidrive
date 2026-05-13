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
#include "maliput_malidrive/traffic_control_device/device_type.h"

#include <algorithm>
#include <cctype>
#include <unordered_map>

#include <maliput/common/maliput_hash.h>

namespace malidrive {
namespace traffic_control_device {

TrafficControlDeviceType StringToTrafficControlDeviceType(const std::string& device_type_str) {
  using T = TrafficControlDeviceType;
  static const std::unordered_map<std::string, T, maliput::common::DefaultHash> kMapper{
      {"traffic_light", T::kTrafficLight},
      {"traffic_sign", T::kTrafficSign},
  };
  std::string lower = device_type_str;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  const auto it = kMapper.find(lower);
  return it != kMapper.end() ? it->second : T::kUnknown;
}

const char* TrafficControlDeviceTypeToString(TrafficControlDeviceType device_type) {
  switch (device_type) {
    case TrafficControlDeviceType::kTrafficLight:
      return "traffic_light";
    case TrafficControlDeviceType::kTrafficSign:
      return "traffic_sign";
    default:
      return "unknown";
  }
}

}  // namespace traffic_control_device
}  // namespace malidrive
