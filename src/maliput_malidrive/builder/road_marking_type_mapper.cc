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
#include "maliput_malidrive/builder/road_marking_type_mapper.h"

#include <algorithm>
#include <cctype>
#include <unordered_map>

#include <maliput/common/maliput_hash.h>

namespace malidrive {
namespace builder {

maliput::api::objects::RoadMarkingType MapRoadMarkingTypeString(const std::string& device_semantics) {
  using T = maliput::api::objects::RoadMarkingType;
  static const std::unordered_map<std::string, T, maliput::common::DefaultHash> kMapper{
      {"stop", T::kStop},
      {"stop_line", T::kStopLine},
      {"crosswalk", T::kCrosswalk},
      {"zebra_crossing", T::kCrosswalk},
      {"parking_space", T::kParkingSpace},
      {"emergency_lane", T::kEmergencyLane},
      {"speed_limit", T::kSpeedLimit},
      {"no_stopping", T::kDoNotStop},
      {"rail_road_crossing", T::kRailRoad},
      {"give_way", T::kGiveWay},
      {"arrow_turn_right", T::kArrowTurnRight},
      {"arrow_turn_left", T::kArrowTurnLeft},
      {"arrow_forward_turn_right", T::kArrowForwardTurnRight},
      {"arrow_forward_turn_left", T::kArrowForwardTurnLeft},
      {"arrow_forward", T::kArrowForward},
      {"arrow_forward_turn_right_turn_left", T::kArrowForwardTurnRightTurnLeft},
      {"arrow_turn_right_turn_left", T::kArrowTurnRightTurnLeft},
      {"arrow_u_turn_right", T::kArrowUTurnRight},
      {"arrow_u_turn_left", T::kArrowUTurnLeft},
  };
  std::string lower = device_semantics;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  const auto it = kMapper.find(lower);
  return it != kMapper.end() ? it->second : T::kUnknown;
}

}  // namespace builder
}  // namespace malidrive
