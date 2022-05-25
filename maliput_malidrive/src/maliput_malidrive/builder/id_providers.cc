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
#include "maliput_malidrive/builder/id_providers.h"

#include <string>

namespace malidrive {
namespace builder {

maliput::api::BranchPointId GetBranchPointId(int branch_point_index) {
  return maliput::api::BranchPointId(std::to_string(branch_point_index));
}

maliput::api::JunctionId GetJunctionId(int xodr_track_id, int xodr_lane_section_index) {
  return maliput::api::JunctionId(std::to_string(xodr_track_id) + "_" + std::to_string(xodr_lane_section_index));
}

maliput::api::JunctionId GetJunctionId(int xodr_junction_id) {
  return maliput::api::JunctionId(std::to_string(xodr_junction_id));
}

maliput::api::LaneId GetLaneId(int xodr_track_id, int xodr_lane_section_index, int xodr_lane_id) {
  return maliput::api::LaneId(std::to_string(xodr_track_id) + "_" + std::to_string(xodr_lane_section_index) + "_" +
                              std::to_string(xodr_lane_id));
}

maliput::api::SegmentId GetSegmentId(int xodr_track_id, int xodr_lane_section_index) {
  return maliput::api::SegmentId(std::to_string(xodr_track_id) + "_" + std::to_string(xodr_lane_section_index));
}

maliput::api::rules::SpeedLimitRule::Id GetSpeedLimitId(const maliput::api::LaneId& lane_id, int speed_limit_index) {
  return maliput::api::rules::SpeedLimitRule::Id(lane_id.string() + "_" + std::to_string(speed_limit_index));
}

maliput::api::rules::DirectionUsageRule::Id GetDirectionUsageRuleId(const maliput::api::LaneId& lane_id,
                                                                    int direction_usage_index) {
  return maliput::api::rules::DirectionUsageRule::Id(lane_id.string() + "_" + std::to_string(direction_usage_index));
}

maliput::api::rules::DirectionUsageRule::State::Id GetDirectionUsageRuleStateId(
    const maliput::api::rules::DirectionUsageRule::Id& rule_id) {
  return maliput::api::rules::DirectionUsageRule::State::Id(rule_id.string());
}

maliput::api::rules::Rule::Id GetRuleIdFrom(const maliput::api::rules::Rule::TypeId& rule_type_id,
                                            const maliput::api::LaneId& lane_id) {
  return maliput::api::rules::Rule::Id(rule_type_id.string() + "/" + lane_id.string());
}

maliput::api::rules::Rule::Id GetRuleIdFrom(const maliput::api::rules::Rule::TypeId& rule_type_id,
                                            const maliput::api::LaneId& lane_id, int index) {
  return maliput::api::rules::Rule::Id(rule_type_id.string() + "/" + lane_id.string() + "_" + std::to_string(index));
}

}  // namespace builder
}  // namespace malidrive
