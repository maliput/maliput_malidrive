// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/direction_usage_builder.h"

#include <map>
#include <unordered_map>

#include "maliput_malidrive/builder/builder_tools.h"

namespace malidrive {
namespace builder {

using maliput::api::rules::DirectionUsageRule;

std::vector<DirectionUsageRule> DirectionUsageBuilder::operator()() {
  maliput::log()->trace("Building DirectionUsageRules...");
  const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*> lanes = rg_->ById().GetLanes();
  // Since Malidrive's RoadNetwork builder computes DirectionUsageRule IDs based on the order in which Lanes are
  // visited, sort the Lanes first. This increases determinism w.r.t. ensuring that the same DirectionUsageRule ID is
  // applied to a particular Lane, as long as the set of Lanes within the RoadGeometry does not change. For more
  // information, see issue #211.
  const std::map<maliput::api::LaneId, const maliput::api::Lane*> sorted_lanes(lanes.begin(), lanes.end());
  std::vector<DirectionUsageRule> direction_usage_rules;
  for (const auto lane_id_lane : sorted_lanes) {
    direction_usage_rules.push_back(BuildDirectionUsageRuleFor(lane_id_lane.second));
    maliput::log()->trace("Built DirectionUsageRule {} for Lane {}.", direction_usage_rules.back().id().string(),
                          lane_id_lane.first.string());
  }

  maliput::log()->trace("All DirectionUsageRules are built.");
  return direction_usage_rules;
}

DirectionUsageRule::State::Type DirectionUsageBuilder::ParseStateType(const std::string& state) const {
  const std::unordered_map<std::string, DirectionUsageRule::State::Type> string_to_state{
      {"AgainstS", DirectionUsageRule::State::Type::kAgainstS},
      {"WithS", DirectionUsageRule::State::Type::kWithS},
      {"Bidirectional", DirectionUsageRule::State::Type::kBidirectional},
      {"Undefined", DirectionUsageRule::State::Type::kUndefined},
  };

  MALIDRIVE_THROW_UNLESS(string_to_state.find(state) != string_to_state.end());
  return string_to_state.at(state);
}

DirectionUsageRule::State DirectionUsageBuilder::BuildDirectionUsageRuleStateFor(const DirectionUsageRule::Id& rule_id,
                                                                                 const Lane* lane) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  const DirectionUsageRule::State::Id state_id = GetDirectionUsageRuleStateId(rule_id);
  const DirectionUsageRule::State::Type state_type = ParseStateType(GetDirectionUsageRuleStateType(lane));
  return DirectionUsageRule::State(state_id, state_type, DirectionUsageRule::State::Severity::kStrict);
}

DirectionUsageRule DirectionUsageBuilder::BuildDirectionUsageRuleFor(const maliput::api::Lane* lane) {
  const Lane* mali_lane = dynamic_cast<const Lane*>(lane);
  MALIDRIVE_THROW_UNLESS(mali_lane != nullptr);

  const DirectionUsageRule::Id rule_id = GetDirectionUsageRuleId(mali_lane->id(), direction_usage_indexer_.new_id());
  const maliput::api::LaneSRange lane_s_range(mali_lane->id(), maliput::api::SRange(0., mali_lane->length()));

  return DirectionUsageRule(rule_id, lane_s_range, {BuildDirectionUsageRuleStateFor(rule_id, mali_lane)});
}

}  // namespace builder
}  // namespace malidrive
