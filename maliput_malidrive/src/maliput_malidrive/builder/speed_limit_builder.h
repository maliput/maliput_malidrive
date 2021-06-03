// Copyright 2020 Toyota Research Institute
#pragma once

#include <vector>

#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/speed_limit_rule.h>
#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Functor to build a vector of SpeedLimitRules.
/// TODO(agalbachicar)   Remove when maliput::api::rules::SpeedLimitRules
///                      are deprecated.
class SpeedLimitBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(SpeedLimitBuilder)
  SpeedLimitBuilder() = delete;

  /// Constructs a SpeedLimitBuilder.
  ///
  /// @param rg is the RoadGeometry pointer. It must not be nullptr.
  SpeedLimitBuilder(const maliput::api::RoadGeometry* rg) : rg_(rg) { MALIDRIVE_THROW_UNLESS(rg_ != nullptr); }

  /// Builds a vector of SpeedLimitRules for each Lane in rg.
  std::vector<maliput::api::rules::SpeedLimitRule> operator()();

 private:
  // Builds a maliput::api::rules::SpeedLimitRule for each Lane within `segment`.
  //
  // When there is no maximum speed limit for the lane, the default defined in
  // Constants is used.
  //
  // All the SpeedLimitRules are built with Severity::kStrict as there is no
  // information about it in the map.
  //
  // @pre `segment` must not be nullptr.
  std::vector<maliput::api::rules::SpeedLimitRule> BuildSpeedLimitFor(const maliput::api::Segment* segment);

  const maliput::api::RoadGeometry* rg_{};
};

}  // namespace builder
}  // namespace malidrive
