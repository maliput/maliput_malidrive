// Copyright 2020 Toyota Research Institute
#pragma once

#include <string>
#include <vector>

#include <maliput/api/regions.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/direction_usage_rule.h>
#include <maliput/common/logger.h>
#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/id_providers.h"

namespace malidrive {
namespace builder {

/// Functor to build a a vector of maliput::api::rules::DirectionUsageRules.
class DirectionUsageBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectionUsageBuilder)
  DirectionUsageBuilder() = delete;

  /// Constructs a DirectionUsageBuilder.
  ///
  /// @param rg is the pointer to the maliput::api::RoadGeometry. It must not be nullptr.
  DirectionUsageBuilder(const maliput::api::RoadGeometry* rg) : rg_(rg), direction_usage_indexer_(/* base ID*/ 0) {
    MALIDRIVE_THROW_UNLESS(rg_ != nullptr);
  }

  /// Builds a vector of maliput::api::rules::DirectionUsageRule.
  ///
  /// Traverses all the lanes within rg and builds a rule for each Lane.
  std::vector<maliput::api::rules::DirectionUsageRule> operator()();

 private:
  /// Parses `state` and returns the equivalent DirectionUsageRule::State::Type.
  /// Possible return types are:
  /// - kAgainstS --> "backward"
  /// - kBidirectional --> "bidirectional"
  /// - kBidirectional --> "undirected"
  /// - kWithS --> "forward"
  /// - kUndefined --> "undefined"
  ///
  /// @param state is the maliput::api::rules::DirectionUsageRule::State::Type to be parsed.
  /// @return An optional of maliput::api::rules::DirectionUsageRule::State::Type with the parsed type.
  /// @throws maliput::common::assertion_error When `state` is unrecognized.
  maliput::api::rules::DirectionUsageRule::State::Type ParseStateType(const std::string& state) const;

  /// Builds a maliput::api::rules::DirectionUsageRule::State for a maliput::api::rules::DirectionUsageRule.
  ///
  /// DirectionUsageRule::State is filled with XODR Lane "userData" node. It
  /// should contain a child node with a "travelDir" attribute. Its value
  /// will be parsed and converted to a DirectionUsage::State::Type.
  /// DirectionUsage::State::Severity will be set to kStrict unless the parsing
  /// fails leading to a default constructed DirectionUsageRule::State with
  /// kBidirectional type and kPreferred severity.
  ///
  /// @param rule_id is the ID of the DirectionUsageRule.
  /// @param lane is Lane pointer.  It must not be nullptr and its type
  ///        must be `malidrive::Lane`.
  maliput::api::rules::DirectionUsageRule::State BuildDirectionUsageRuleStateFor(
      const maliput::api::rules::DirectionUsageRule::Id& rule_id, const Lane* lane);

  /// Builds a maliput::api::rules::DirectionUsageRule for `lane`.
  ///
  /// @param lane is the Lane to inspect. It must not be nullptr and its type
  ///        must be `malidrive::Lane`.
  maliput::api::rules::DirectionUsageRule BuildDirectionUsageRuleFor(const maliput::api::Lane* lane);

  const maliput::api::RoadGeometry* rg_{};
  UniqueIntegerProvider direction_usage_indexer_;
};

}  // namespace builder
}  // namespace malidrive
