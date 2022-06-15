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
#include "maliput_malidrive/builder/speed_limit_builder.h"

#include <maliput/api/junction.h>
#include <maliput/common/logger.h>

#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/constants.h"

namespace malidrive {
namespace builder {

using maliput::api::LaneSRange;
using maliput::api::SRange;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
using maliput::api::rules::SpeedLimitRule;

std::vector<SpeedLimitRule> SpeedLimitBuilder::operator()() {
  maliput::log()->trace("Building SpeedLimitRules...");
  std::vector<SpeedLimitRule> speed_limits;
  for (int i = 0; i < rg_->num_junctions(); ++i) {
    const maliput::api::Junction* junction = rg_->junction(i);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const std::vector<SpeedLimitRule> segment_speed_limits = BuildSpeedLimitFor(junction->segment(j));
      speed_limits.insert(speed_limits.end(), segment_speed_limits.begin(), segment_speed_limits.end());
    }
  }
  maliput::log()->trace("All SpeedLimitRules are built.");
  return speed_limits;
}

std::vector<maliput::api::rules::SpeedLimitRule> SpeedLimitBuilder::BuildSpeedLimitFor(
    const maliput::api::Segment* segment) {
  MALIDRIVE_THROW_UNLESS(segment != nullptr);

  const double kDefaultMinSpeedLimit{constants::kDefaultMinSpeedLimit};
  const SpeedLimitRule::Severity kDefaultSeverity{SpeedLimitRule::Severity::kStrict};

  std::vector<SpeedLimitRule> speed_limits;
  for (int i = 0; i < segment->num_lanes(); ++i) {
    const Lane* lane = dynamic_cast<const Lane*>(segment->lane(i));
    const auto max_speed_limits = GetMaxSpeedLimitFor(lane);
    UniqueIntegerProvider speed_limit_indexer(0);
    for (const auto& speed_limit : max_speed_limits) {
      const double s0{lane->LaneSFromTrackS(speed_limit.s_start)};
      const double s1{lane->LaneSFromTrackS(speed_limit.s_end)};
      const LaneSRange lane_s_range(lane->id(), SRange(s0, s1));
      const SpeedLimitRule speed_limit_rule(GetSpeedLimitId(lane->id(), speed_limit_indexer.new_id()), lane_s_range,
                                            kDefaultSeverity, kDefaultMinSpeedLimit, speed_limit.max);
      maliput::log()->trace("Built SpeedLimitRule {} for Lane {} in SRange: [{},{}]", speed_limit_rule.id().string(),
                            lane->id().string(), s0, s1);
      speed_limits.push_back(speed_limit_rule);
    }
  }
  return speed_limits;
}
#pragma GCC diagnostic pop

}  // namespace builder
}  // namespace malidrive
