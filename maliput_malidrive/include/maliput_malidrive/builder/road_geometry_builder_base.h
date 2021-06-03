// Copyright 2020 Toyota Research Institute
#pragma once

#include <map>
#include <memory>
#include <vector>

#include <maliput/geometry_base/junction.h>
#include <maliput/math/vector.h>
#include "maliput_malidrive/base/inertial_to_lane_mapping_config.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/builder/road_geometry_configuration.h"

namespace malidrive {
namespace builder {

/// OpenDRIVE's format hierarchy is simple and built on top of a XML format.
/// Details can be found in
/// https://www.asam.net/standards/detail/opendrive/
/// As a shortcut, the map follows this structure:
///
/// OpenDrive
///   - road
///     - lane_section
///       - lane
///       - ...
///       - lane
///     - ...
///     - lane_section
///       - lane
///       - ...
///       - lane
///   - ...
///   - road
///     - lane_section
///       - lane
///       - ...
///       - lane
///     - ...
///     - lane_section
///       - lane
///       - ...
///       - lane
///   - junction
///     - connection
///       - lane link
///       - ...
///       - lane link
///     - ...
///     - connection
///       - lane link
///       - ...
///       - lane link
//   - ...
///   - junction
///     - connection
///       - lane link
///       - ...
///       - lane link
///     - ...
///     - connection
///       - lane link
///       - ...
///       - lane link
///
/// Thus, for Junction -> Segment -> Lane creation, the build process consists
/// of visiting all the nodes and creating the equivalent entities.
///
/// To properly connect Lanes through BranchPoints, it is important to
/// understand the logic behind road linkage first in OpenDRIVE. `Road`s can be
/// joint using `Junction`s (when there is more than one option `Lane` to
/// follow) or `Successor` and `Predecessor` tags (when linkage is trivially
/// solved). A `Road` could be marked as part of a `Junction` or not, depending
/// if the connecting _end_ ends in a `Junction`. At the same time,
/// outer `LaneSection`s hold `Lane`s that could connect to either a `Junction`
/// or another `Road`. However, inner `LaneSection`s hold `Lane`s that can only
/// connect to other `Lane`s that belong to a consecutive `LaneSection`, or end
/// in the middle of the road. Given that, `BranchPoint`s need to be solved in
/// different stages:
///
///   - Inner `LaneSection`s
///   - Outer `LaneSection`s with `Junction`s / `Road`s
class RoadGeometryBuilderBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometryBuilderBase)

  RoadGeometryBuilderBase() = delete;
  virtual ~RoadGeometryBuilderBase() = default;

  /// Builder interface to construct a RoadGeometry whose ID is
  /// `road_geometry_configuration.id`.
  ///
  /// Resulting maliput::api::RoadGeometry will have `linear_tolerance`,
  /// `angular_tolerance` and `scale_length` properties set by the
  /// `road_geometry_configuration`.
  ///
  /// Note: the `opendrive_file` parameter of `road_geometry_configuration` is
  /// ignored because a manager is expected to emerge .
  ///
  /// @throws maliput::common::assertion_error When
  /// `road_geometry_configuration.linear_tolerance`,
  /// `road_geometry_configuration.angular_tolerance` or
  /// `road_geometry_configuration.scale_length` are negative.
  RoadGeometryBuilderBase(const RoadGeometryConfiguration& road_geometry_configuration);

  /// Creates a maliput equivalent backend (malidrive::RoadGeometry).
  virtual std::unique_ptr<const maliput::api::RoadGeometry> operator()() = 0;

 protected:
  /// Convenient enumeration to identify on which side of a BranchPoint a LaneEnd
  /// is.
  enum class BranchPointSide { kASide = 0, kBSide };

  // Given `lane_end`, it looks for a BranchPoint that has it on either A or B
  // side. If there is none, it will look for a BranchPoint that has any of the
  // connecting LaneEnds from `lane_ends`. If there is not any BranchPoint, a
  // new one is created.
  //
  // `lane_end` will be attached to the BranchPoint following these rules:
  //
  // - If the BranchPoint is new, it will go to the A side.
  // - If the BranchPoint already has it, it will not be added.
  // - If the BranchPoint does not have it but it has another LaneEnd, it will
  //   go to the opposite side (A --> B, B --> A).
  //
  // After that assignment, `lane_ends` are attached to the BranchPoint
  // following previous rules but considering that they should be on the
  // opposite side of `lane_end`.
  void AttachLaneEndToBranchPoint(const maliput::api::LaneEnd& lane_end,
                                  const std::vector<maliput::api::LaneEnd>& lane_ends);

  // Sets the first LaneEnd on the opposite side of a BranchPoint as
  // default.
  //
  // @throws maliput::common::assertion_error When a BranchPoint does not have
  //         any LaneEnd on ASide. Note this function will be only called after
  //         all BranchPoints have been created.
  void SetDefaultsToBranchPoints();

  // Returns a BranchPointId whose base string is an increasing and unique
  // integer.
  maliput::api::BranchPointId GetNewBranchPointId();

  // @return true When `lane_end` belongs to the `bp_side` of `bp`.
  //
  // `bp` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `bp` is nullptr..
  static bool IsLaneEndOnABSide(const maliput::api::BranchPoint* bp, const maliput::api::LaneEnd& lane_end,
                                BranchPointSide bp_side);

  // Finds a BranchPoint and its side in `bps` where `lane_end` lives.
  //
  // @return A pair with the pointer to the BranchPoint and an optional with the
  // side of the BranchPoint where `lane_end` lives. If no BranchPoint can be
  // found, then return value will be <nullptr, nullopt>.
  static std::pair<maliput::geometry_base::BranchPoint*, std::optional<BranchPointSide>> FindBranchpointByLaneEnd(
      const maliput::api::LaneEnd& lane_end,
      const std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>>& bps);

  const InertialToLaneMappingConfig inertial_to_lane_mapping_config_;
  const maliput::api::RoadGeometryId id_;
  const BuildPolicy build_policy_{};

  // @{ Tolerance, scale length and Inertial to Backend Frame translation values
  //    used to construct the RoadGeometry.
  double linear_tolerance_{};
  double angular_tolerance_{};
  double scale_length_{};
  maliput::math::Vector3 inertial_to_backend_frame_translation_{};
  // @}

  UniqueIntegerProvider branch_point_indexer_;
  std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>> bps_{};
  std::map<maliput::api::JunctionId, maliput::geometry_base::Junction*> junctions_{};
};

}  // namespace builder
}  // namespace malidrive
