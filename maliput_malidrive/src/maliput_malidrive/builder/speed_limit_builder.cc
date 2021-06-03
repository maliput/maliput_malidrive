// Copyright 2020 Toyota Research Institute
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

}  // namespace builder
}  // namespace malidrive
