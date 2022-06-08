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
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <maliput/geometry_base/junction.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/builder/road_curve_factory.h"
#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace builder {

/// Builder class on top of the `xodr::DBManager` which should
/// already have loaded the map. It will construct a RoadGeometry that maps to
/// the XODR description.
///
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
///   - ...
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
class RoadGeometryBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometryBuilder)

  RoadGeometryBuilder() = delete;

  /// Builds a RoadGeometry using malidrive backend whose ID is
  /// `road_geometry_configuration.id`.
  ///
  /// `manager` must not be nullptr. Ownership will be transferred to the
  /// resulting RoadGeometry.
  ///
  /// Resulting maliput::api::RoadGeometry will have properties set by the
  /// `road_geometry_configuration`.
  ///
  /// Note: the `opendrive_file` parameter of `road_geometry_configuration` is
  /// ignored because a manager is expected to emerge.
  ///
  /// @throws maliput::common::assertion_error When
  /// `road_geometry_configuration.tolerances.linear_tolerance`,
  /// `road_geometry_configuration.tolerances.angular_tolerance` or
  /// `road_geometry_configuration.scale_length` are negative.
  /// @throws maliput::common::assertion_error When `road_geometry_configuration.tolerances.max_linear_tolerance` is
  /// less than `road_geometry_configuration.tolerances.linear_tolerance`.
  /// @throws maliput::common::assertion_error When `manager` is nullptr.
  RoadGeometryBuilder(std::unique_ptr<xodr::DBManager> manager,
                      const RoadGeometryConfiguration& road_geometry_configuration);

  /// Creates a maliput equivalent backend (malidrive::RoadGeometry).
  ///
  /// When RoadGeometryConfiguration::ToleranceSelectionPolicy::kManualSelection
  /// is used, the initial linear_tolerance, angular_tolerance and scale_length
  /// are used. Otherwise, up to constants::kAutomaticSelectionTrials + 1 trials
  /// are done with increasing linear_tolerance, angular_tolerance and
  /// scale_length equal to constants::kScaleLength. Each iteration will add a
  /// 10% to linear_tolerance and angular_tolerance.
  ///
  /// Consider using small or default values of linear_tolerance and
  /// angular_tolerance to granularly try with different configurations.
  std::unique_ptr<const maliput::api::RoadGeometry> operator()();

 private:
  // Associates a malidrive::Lane to a XODR Lane.
  struct MatchingLanes {
    Lane* malidrive_lane{};
    MalidriveXodrLaneProperties xodr_lane;
  };

  // Holds the attributes needed to build all the Lanes of a Segment.
  struct SegmentConstructionAttributes {
    const xodr::RoadHeader* road_header{};
    const xodr::LaneSection* lane_section{};
    int lane_section_index{};
  };

  // Holds the lane construction task result.
  struct LaneConstructionResult {
    Segment* segment{};
    std::unique_ptr<Lane> lane{};
    MalidriveXodrLaneProperties xodr_lane_properties{nullptr /*road_header*/, nullptr /*lane_section*/,
                                                     0 /*lane_section_index*/, nullptr /*lane*/};
  };

  // Functor for creating lanes of an entire junction.
  struct LanesBuilder {
    // Constructs a LanesBuilder.
    // `junction_segments_attributes_in` Contains all the attributes needed to build all the Lanes of a given Junction.
    // `factory_in` Is a pointer to the RoadCurveFactoryBase.
    // `rg_config_in` road geometry configuration.
    // `rg_in` Is a pointer to the RoadGeometry.
    //
    // Note: All input parameters are aliased and thus must remain valid for the duration of this class instance.
    //
    // @throws maliput::common::assertion_error When `rg` is nullptr.
    // @throws maliput::common::assertion_error When `factory` is nullptr.
    LanesBuilder(const std::pair<maliput::geometry_base::Junction*,
                                 std::map<Segment*, RoadGeometryBuilder::SegmentConstructionAttributes>>&
                     junction_segments_attributes_in,
                 const RoadCurveFactoryBase* factory_in, const RoadGeometryConfiguration& rg_config_in,
                 RoadGeometry* rg_in)
        : junction_segments_attributes(junction_segments_attributes_in),
          factory(factory_in),
          rg_config(rg_config_in),
          rg(rg_in) {
      MALIDRIVE_THROW_UNLESS(rg != nullptr);
      MALIDRIVE_THROW_UNLESS(factory != nullptr);
    }

    // Returns A vector containing all the created Lanes and its properties.
    std::vector<RoadGeometryBuilder::LaneConstructionResult> operator()();

    // Contains all the attributes needed to build all the lanes of a given junction.
    const std::pair<maliput::geometry_base::Junction*,
                    std::map<Segment*, RoadGeometryBuilder::SegmentConstructionAttributes>>
        junction_segments_attributes;

    const RoadCurveFactoryBase* factory{};
    const RoadGeometryConfiguration& rg_config;
    RoadGeometry* rg{};
  };

  // Convenient enumeration to identify on which side of a BranchPoint a LaneEnd
  // is.
  enum class BranchPointSide { kASide = 0, kBSide };

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

  // Builds a Lane and returns within a LaneConstructionResult that holds extra attributes related to the lane.
  // `lane` must not be nullptr.
  // `road_header` must not be nullptr.
  // `lane_section` must not be nullptr.
  // `xodr_lane_section_index` must be non-negative.
  // `factory` must not be nullptr.
  // `rg_config` road geometry configuration.
  // `segment` must not be nullptr.
  // `adjacent_lane_functions` holds the offset and width functions of the immediate inner lane, must not be nullptr.
  //
  // @throws maliput::common::assertion_error When aforementioned conditions aren't met.
  static LaneConstructionResult BuildLane(const xodr::Lane* lane, const xodr::RoadHeader* road_header,
                                          const xodr::LaneSection* lane_section, int xodr_lane_section_index,
                                          const RoadCurveFactoryBase* factory,
                                          const RoadGeometryConfiguration& rg_config, Segment* segment,
                                          road_curve::LaneOffset::AdjacentLaneFunctions* adjacent_lane_functions);
  // Builds malidrive::Lanes from the XODR `lane_section` and returns a vector of
  // LaneConstructionResult objects containing the built Lane and properties needed to later on
  // add the Lane to its correspondant Segment.
  // While the Lanes are built from the center to the external lanes to correctly compute their
  // lane offset, the returned vector is filled with the Lanes in right-to-left order of the Segment.
  //
  // `road_header` must not be nullptr.
  // `lane_section` must not be nullptr.
  // `xodr_lane_section_index` is the index of the LaneSection within the road and mustn't be negative.
  // `rg` must not be nullptr.
  // `factory` must not be nullptr.
  // `rg_config` road geometry configuration.
  // `segment` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When either `segment`,
  //         `lane_section`, `road_header` or `rg` are nullptr.
  static std::vector<LaneConstructionResult> BuildLanesForSegment(const xodr::RoadHeader* road_header,
                                                                  const xodr::LaneSection* lane_section,
                                                                  int xodr_lane_section_index,
                                                                  const RoadCurveFactoryBase* factory,
                                                                  const RoadGeometryConfiguration& rg_config,
                                                                  RoadGeometry* rg, Segment* segment);

  // Analyzes the width description of the Lane and looks for negative width values.
  // In order to guarantee non-negative values, each piece of the piecewise-defined lane width function must comply
  // with:
  //   1. Non-negative sign of the width at the beginning of the lane width function.
  //   2. Non-negative sign of the width at the end of the lane width function.
  //   3. If a local minimum is found in the range of the lane width function, then the function
  //      evaluated at that local minimum should be non-negative.
  //
  // `lane_widths` Width descriptions of a lane.
  // `lane_id` ID of the lane.
  // `xodr_lane_length` Length of the Lane, matches LaneSection length.
  // `linear_tolerance` Linear tolerance.
  // `allow_negative_width` If false it throws when negative width is found, otherwise it logs a detailed warning.
  //
  // @throws maliput::common::assertion_error When negatives width values are found and `allow_negative_width` is set
  // false.
  static void VerifyNonNegativeLaneWidth(const std::vector<xodr::LaneWidth>& lane_widths,
                                         const maliput::api::LaneId& lane_id, double xodr_lane_length,
                                         double linear_tolerance, bool allow_negative_width);

  // Builds a RoadCurve.
  //
  // `road_header` contains the geometry description of the curve model.
  // `geometries_to_simplify` contains which geometries can be simplified from `road_header`.
  //
  // @throws maliput::common::assertion_error When the start point of the geometry doesn't match with the start point of
  // the lane section.
  std::unique_ptr<road_curve::RoadCurve> BuildRoadCurve(
      const xodr::RoadHeader& road_header,
      const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify);

  // Returns a maliput::geometry_base::Junction whose ID will be "`xodr_track_id`_`lane_section_index`.
  //
  // `xodr_track_id` must be non-negative number.
  // `lane_section_index` must be non negative.
  //
  // @throws maliput::common::assertion_error When `xodr_track_id` holds a negative number.
  // @throws maliput::common::assertion_error When `lane_section_index` is negative.
  std::unique_ptr<maliput::geometry_base::Junction> BuildJunction(const std::string& xodr_track_id,
                                                                  int lane_section_index);

  // Returns a maliput::geometry_base::Junction whose ID will be `xodr_junction_id`.
  //
  // `xodr_junction_id` must be non-negative number.
  //
  // @throws maliput::common::assertion_error When `xodr_junction_id` is
  //         negative.
  std::unique_ptr<maliput::geometry_base::Junction> BuildJunction(const std::string& xodr_junction_id);

  // Returns a ground curve built from `geometry`.
  //
  // `geometries` Is a vector of xodr::Geometry. It must not be empty.
  // `geometries_to_simplify` contains which geometries can be simplified from `geometries`.
  //
  // @throws maliput::common::assertion_error When `geometries` is empty.
  // @throws maliput::common::assertion_error When any item in `geometries` has an unsupported type.
  std::unique_ptr<malidrive::road_curve::GroundCurve> MakeGroundCurve(
      const std::vector<xodr::Geometry>& geometries,
      const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify);

  // Finds or creates for each lane a BranchPoint for each end.
  // When #rg_config_.omit_nondrivable_lane is true, no BranchPoints are built for the non-drivable lanes.
  //
  // `rg` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr.
  void BuildBranchPointsForLanes(RoadGeometry* rg);

  // Finds or creates for `lane` a BranchPoint for each end.
  //
  // `lane` must not be nullptr.
  // `rg` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When either `lane` or `rg` are
  //         nullptr.
  void FindOrCreateBranchPointFor(const MalidriveXodrLaneProperties& xodr_lane_properties, Lane* lane,
                                  RoadGeometry* rg);

  // Identifies which kind of XODR Lane maps to `lane_end.lane` and then the
  // connection it refers. We can identify the following types:
  //
  // If the road does not belong to a Junction:
  // - Inner LaneSection: XODR Lane would only connect to other lane within the
  //   same road.
  // - Extreme LaneSection: XODR Lane would only connect to other XODR Road, or
  //   Junction.
  // If the road belongs to a Junction:
  // - If the Lane Section is internal, it is treated as before, otherwise, it
  //   checks which is the other Road it connects to.
  //
  // Returns a vector of LaneEnds that connect to `lane_end`.
  //
  // `rg` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr.
  std::vector<maliput::api::LaneEnd> FindConnectingLaneEndsForLaneEnd(
      const maliput::api::LaneEnd& lane_end, const MalidriveXodrLaneProperties& xodr_lane_properties, RoadGeometry* rg);

  // Returns the Lanes of the RoadGeometry created from multiple threads.
  // Each thread will take care of build all the Lanes of a particualar junction.
  //
  // `num_of_threads` Is the number of threads.
  // `rg` Is a pointer to the RoadGeometry.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr or num_of_threads is less than 1.
  std::vector<LaneConstructionResult> LanesBuilderParallelPolicy(std::size_t num_of_threads, RoadGeometry* rg);

  // Returns the Lanes of the RoadGeometry sequentally created.
  //
  // `rg` Is a pointer to the RoadGeometry.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr.
  std::vector<LaneConstructionResult> LanesBuilderSequentialPolicy(RoadGeometry* rg);

  // Builds all the Lanes of the RoadGeometry and adds them to their correspondent segments.
  // When #rg_config_.omit_nondrivable_lane is true, non-drivable lanes
  // are added to the segment but they will be hidden.
  //
  // `rg` Is a pointer to the RoadGeometry.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr.
  void FillSegmentsWithLanes(RoadGeometry* rg);

  // Executes the build process itself.
  //
  // Visits nodes in the xodr map via DBManager to build Junctions, Segments and
  // Lanes. Then it constructs the BranchPoints based on linkage information in
  // the database.
  std::unique_ptr<const maliput::api::RoadGeometry> DoBuild();

  // Resets this builder state and loads new values of
  // maliput::api::RoadGeometry geometric invariants.
  //
  // Also, clears the collections branch_point_indexer_, bps_ and junctions_.
  //
  // Resulting maliput::api::RoadGeometry will have `linear_tolerance`,
  // `angular_tolerance` and `scale_length` properties.
  //
  // @throws maliput::common::assertion_error When any of `linear_tolerance`,
  //         `angular_tolerance` or `scale_length` are negative.
  void Reset(double linear_tolerance, double angular_tolerance, double scale_length);

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

  // Provides unique IDs for the BranchPoints.
  UniqueIntegerProvider branch_point_indexer_{0 /* base ID */};

  // Holds the built BranchPoints.
  std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>> bps_{};

  // Holds the built Junctions.
  std::map<maliput::api::JunctionId, maliput::geometry_base::Junction*> junctions_{};

  // Holds the configuration of this builder.
  RoadGeometryConfiguration rg_config_;

  // Holds the xodr database.
  std::unique_ptr<xodr::DBManager> manager_;

  // Holds the factory to build road curves.
  std::unique_ptr<RoadCurveFactoryBase> factory_;

  // Key on LaneId to ensure iteration over this map is deterministic. This
  // ensures default branch point selection is deterministic.
  std::map<maliput::api::LaneId, MatchingLanes> lane_xodr_lane_properties_;

  // Map holding all the construction attributes used to build the segments and lanes of each junction.
  std::map<maliput::geometry_base::Junction*, std::map<Segment*, SegmentConstructionAttributes>>
      junctions_segments_attributes_;
};

}  // namespace builder
}  // namespace malidrive
