// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_builder_base.h"

#include "maliput_malidrive/common/macros.h"

#include "maliput/common/logger.h"
#include "maliput/geometry_base/branch_point.h"

namespace malidrive {
namespace builder {

using maliput::geometry_base::BranchPoint;

RoadGeometryBuilderBase::RoadGeometryBuilderBase(const RoadGeometryConfiguration& road_geometry_configuration)
    : inertial_to_lane_mapping_config_(road_geometry_configuration.inertial_to_lane_mapping_config),
      id_(road_geometry_configuration.id),
      build_policy_(road_geometry_configuration.build_policy),
      linear_tolerance_(road_geometry_configuration.linear_tolerance),
      angular_tolerance_(road_geometry_configuration.angular_tolerance),
      scale_length_(road_geometry_configuration.scale_length),
      inertial_to_backend_frame_translation_(road_geometry_configuration.inertial_to_backend_frame_translation),
      branch_point_indexer_(0 /* base ID */) {
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ >= 0.);
  MALIDRIVE_THROW_UNLESS(angular_tolerance_ >= 0.);
  MALIDRIVE_THROW_UNLESS(scale_length_ >= 0.);
}

void RoadGeometryBuilderBase::AttachLaneEndToBranchPoint(const maliput::api::LaneEnd& lane_end,
                                                         const std::vector<maliput::api::LaneEnd>& lane_ends) {
  // First, we look for any BranchPoint which already has `lane_end`.
  std::pair<BranchPoint*, std::optional<BranchPointSide>> bp_side = FindBranchpointByLaneEnd(lane_end, bps_);
  if (bp_side.first == nullptr) {  // We should look for other lane_ends
    for (const maliput::api::LaneEnd& le : lane_ends) {
      bp_side = FindBranchpointByLaneEnd(le, bps_);
      if (bp_side.first != nullptr) {
        bp_side.second =
            bp_side.second.value() == BranchPointSide::kASide ? BranchPointSide::kBSide : BranchPointSide::kASide;
        break;
      }
    }
  }
  if (bp_side.first == nullptr) {
    bps_.push_back(std::make_unique<BranchPoint>(GetNewBranchPointId()));
    maliput::log()->trace("Created BranchPoint ID: {}.", bps_.back()->id().string());
    bp_side = std::make_pair<BranchPoint*, std::optional<BranchPointSide>>(bps_.back().get(), BranchPointSide::kASide);
  }

  auto mutable_geometry_base_lane_cast = [](const maliput::api::Lane* l) -> maliput::geometry_base::Lane* {
    return const_cast<maliput::geometry_base::Lane*>(dynamic_cast<const maliput::geometry_base::Lane*>(l));
  };
  if (bp_side.second.value() == BranchPointSide::kASide) {
    if (!IsLaneEndOnABSide(bp_side.first, lane_end, BranchPointSide::kASide)) {
      bp_side.first->AddABranch(mutable_geometry_base_lane_cast(lane_end.lane), lane_end.end);
    }
    for (const maliput::api::LaneEnd& le : lane_ends) {
      if (!IsLaneEndOnABSide(bp_side.first, le, BranchPointSide::kBSide)) {
        bp_side.first->AddBBranch(mutable_geometry_base_lane_cast(le.lane), le.end);
      }
    }
  } else {
    for (const maliput::api::LaneEnd& le : lane_ends) {
      if (!IsLaneEndOnABSide(bp_side.first, le, BranchPointSide::kASide)) {
        bp_side.first->AddABranch(mutable_geometry_base_lane_cast(le.lane), le.end);
      }
    }
    if (!IsLaneEndOnABSide(bp_side.first, lane_end, BranchPointSide::kBSide)) {
      bp_side.first->AddBBranch(mutable_geometry_base_lane_cast(lane_end.lane), lane_end.end);
    }
  }
  maliput::log()->trace("LaneEnd ({}, {}) is attached to BranchPoint {}.", lane_end.lane->id().string(),
                        lane_end.end == maliput::api::LaneEnd::Which::kStart ? "start" : "end",
                        bp_side.first->id().string());
}

void RoadGeometryBuilderBase::SetDefaultsToBranchPoints() {
  maliput::log()->trace("Setting defaults to BranchPoints.");
  for (int i = 0; i < static_cast<int>(bps_.size()); ++i) {
    const maliput::api::LaneEndSet* a_side_set = bps_[i]->GetASide();
    const maliput::api::LaneEndSet* b_side_set = bps_[i]->GetBSide();
    // We expect to have at least one in A side.
    MALIDRIVE_THROW_UNLESS(a_side_set->size() > 0);
    for (int j = 0; j < b_side_set->size(); ++j) {
      bps_[i]->SetDefault(b_side_set->get(j), a_side_set->get(0));
    }
    if (b_side_set->size() > 0) {
      for (int j = 0; j < a_side_set->size(); ++j) {
        bps_[i]->SetDefault(a_side_set->get(j), b_side_set->get(0));
      }
    }
  }
}

maliput::api::BranchPointId RoadGeometryBuilderBase::GetNewBranchPointId() {
  return GetBranchPointId(branch_point_indexer_.new_id());
}

bool RoadGeometryBuilderBase::IsLaneEndOnABSide(const maliput::api::BranchPoint* bp,
                                                const maliput::api::LaneEnd& lane_end, BranchPointSide bp_side) {
  MALIDRIVE_THROW_UNLESS(bp != nullptr);

  auto equiv = [](const maliput::api::LaneEnd& lane_end_a, const maliput::api::LaneEnd& lane_end_b) {
    return lane_end_a.lane == lane_end_b.lane && lane_end_a.end == lane_end_b.end;
  };

  const maliput::api::LaneEndSet* lane_end_set = bp_side == BranchPointSide::kASide ? bp->GetASide() : bp->GetBSide();

  for (int j = 0; j < lane_end_set->size(); ++j) {
    if (equiv(lane_end_set->get(j), lane_end)) {
      return true;
    }
  }
  return false;
}

// TODO(agalbachicar)   Move this BranchPoint type to be
//                      maliput::api::BranchPoint
std::pair<maliput::geometry_base::BranchPoint*, std::optional<RoadGeometryBuilderBase::BranchPointSide>>
RoadGeometryBuilderBase::FindBranchpointByLaneEnd(
    const maliput::api::LaneEnd& lane_end,
    const std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>>& bps) {
  for (size_t i = 0; i < bps.size(); ++i) {
    if (IsLaneEndOnABSide(bps[i].get(), lane_end, BranchPointSide::kASide)) {
      return {bps[i].get(), {BranchPointSide::kASide}};
    }
    if (IsLaneEndOnABSide(bps[i].get(), lane_end, BranchPointSide::kBSide)) {
      return {bps[i].get(), {BranchPointSide::kBSide}};
    }
  }
  return {nullptr, {}};
}

}  // namespace builder
}  // namespace malidrive
