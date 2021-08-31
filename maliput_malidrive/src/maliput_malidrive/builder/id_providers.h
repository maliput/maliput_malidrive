// Copyright 2019 Toyota Research Institute
#pragma once

#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/rules/direction_usage_rule.h>
#include <maliput/api/rules/rule.h>
#include <maliput/api/rules/speed_limit_rule.h>
#include <maliput/api/segment.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Returns a BranchPointId whose base string is:
/// {`branch_point_index`}.
maliput::api::BranchPointId GetBranchPointId(int branch_point_index);

/// Returns a JunctionId whose base string is:
/// {`xodr_track_id`\_`xodr_lane_section_index`}.
maliput::api::JunctionId GetJunctionId(int xodr_track_id, int xodr_lane_section_index);

/// Returns a JunctionId whose base string is: {`xodr_junction_id`}.
maliput::api::JunctionId GetJunctionId(int xodr_junction_id);

/// Returns a LaneId whose base string is:
/// {`xodr_track_id`\_`xodr_lane_section_index`\_`xodr_lane_id`}.
maliput::api::LaneId GetLaneId(int xodr_track_id, int xodr_lane_section_index, int xodr_lane_id);

/// Returns a SegmentId whose base string is:
/// {`xodr_track_id`\_`xodr_lane_section_index`}.
maliput::api::SegmentId GetSegmentId(int xodr_track_id, int xodr_lane_section_index);

/// Returns a SpeedLimitRule::Id whose base string is:
/// {`lane_id`\_`speed_limit_index`}.
maliput::api::rules::SpeedLimitRule::Id GetSpeedLimitId(const maliput::api::LaneId& lane_id, int speed_limit_index);

/// Returns a DirectionUsageRule::Id whose base string is:
/// {`lane_id`\_`direction_usage_index`}.
maliput::api::rules::DirectionUsageRule::Id GetDirectionUsageRuleId(const maliput::api::LaneId& lane_id,
                                                                    int direction_usage_index);

/// Returns a DirectionUsageRule::State::Id whose base string is: {`rule_id`}.
maliput::api::rules::DirectionUsageRule::State::Id GetDirectionUsageRuleStateId(
    const maliput::api::rules::DirectionUsageRule::Id& rule_id);

/// Returns a Rule::Id whose base string is:
/// {`rule_type_id.string()`'/'`lane_id.string()`}.
maliput::api::rules::Rule::Id GetRuleIdFrom(const maliput::api::rules::Rule::TypeId& rule_type_id,
                                            const maliput::api::LaneId& lane_id);

/// Returns a Rule::Id whose base string is:
/// {`rule_type_id.string()`'/'`lane_id.string()`'_'`index`}.
maliput::api::rules::Rule::Id GetRuleIdFrom(const maliput::api::rules::Rule::TypeId& rule_type_id,
                                            const maliput::api::LaneId& lane_id, int index);

/// Class to handle the index arithmetic and get always an increasing integer
/// as ID.
class UniqueIntegerProvider {
 public:
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniqueIntegerProvider)
  UniqueIntegerProvider() = delete;

  /// Constructs an UniqueIntegerProvider.
  /// @param base_id is the base integer to be used as ID and start
  /// incrementing.
  explicit UniqueIntegerProvider(int base_id) : id_(base_id) {}

  /// @return A new ID, which differs from the latest provided in +1.
  int new_id() { return ++id_; }

  /// @return The latest provided ID.
  int get_last_id() const { return id_; }

 private:
  int id_{};
};

}  // namespace builder
}  // namespace malidrive
