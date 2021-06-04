// Copyright 2020 Toyota Research Institute
#pragma once

#include <map>
#include <memory>
#include <vector>

#include <maliput/geometry_base/junction.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/builder/road_curve_factory.h"
#include "maliput_malidrive/builder/road_geometry_builder_base.h"
#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace builder {

/// Builder class on top of the `xodr::DBManager` which should
/// already have loaded the map. It will construct a RoadGeometry that maps to
/// the XODR description.
class RoadGeometryBuilder : public RoadGeometryBuilderBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometryBuilder)

  RoadGeometryBuilder() = delete;

  /// Builds a RoadGeometry using malidrive2 backend.
  ///
  /// `manager` must not be nullptr. Ownership will be transferred to the
  /// resulting RoadGeometry.
  ///
  /// `factory` must not be nullptr. Used to create the road curve for the lanes.
  ///
  /// @throws maliput::common::assertion_error When `manager` or `factory` are
  /// nullptr.
  /// @see RoadGeometryBuilderBase::RoadGeometryBuilderBase() for further details.
  RoadGeometryBuilder(std::unique_ptr<xodr::DBManager> manager,
                      const RoadGeometryConfiguration& road_geometry_configuration,
                      std::unique_ptr<RoadCurveFactoryBase> factory);

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
  std::unique_ptr<const maliput::api::RoadGeometry> operator()() override;

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
    // `rg_in` Is a pointer to the RoadGeometry.
    // `factory_in` Is a pointer to the RoadCurveFactoryBase.
    //
    // @throws maliput::common::assertion_error When `rg` is nullptr.
    // @throws maliput::common::assertion_error When `factory` is nullptr.
    LanesBuilder(const std::pair<maliput::geometry_base::Junction*,
                                 std::map<Segment*, RoadGeometryBuilder::SegmentConstructionAttributes>>&
                     junction_segments_attributes_in,
                 RoadGeometry* rg_in, const RoadCurveFactoryBase* factory_in, bool omit_nondrivable_lanes_in)
        : junction_segments_attributes(junction_segments_attributes_in),
          factory(factory_in),
          omit_nondrivable_lanes(omit_nondrivable_lanes_in),
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
    const bool omit_nondrivable_lanes{true};
    RoadGeometry* rg{};
  };

  // Builds a Lane and returns within a LaneConstructionResult that holds extra attributes related to the lane.
  // `lane` must not be nullptr.
  // `road_header` must not be nullptr.
  // `lane_section` must not be nullptr.
  // `xodr_lane_section_index` must be non-negative.
  // `factory` must not be nullptr.
  // `segment` must not be nullptr.
  // `adjacent_lane_functions` holds the offset and width functions of the immediate inner lane, must not be nullptr.
  //
  // @throws maliput::common::assertion_error When aforementioned conditions aren't met.
  static LaneConstructionResult BuildLane(const xodr::Lane* lane, const xodr::RoadHeader* road_header,
                                          const xodr::LaneSection* lane_section, int xodr_lane_section_index,
                                          const RoadCurveFactoryBase* factory, Segment* segment,
                                          road_curve::LaneOffset::AdjacentLaneFunctions* adjacent_lane_functions);
  // Builds malidrive::Lanes from the XODR `lane_section` and returns a vector of
  // LaneConstructionResult objects containing the built Lane and properties needed to later on
  // add the Lane to its correspondant Segment.
  // While the Lanes are built from the center to the external lanes to correctly compute their
  // lane offset, the returned vector is filled with the Lanes in right-to-left order of the Segment.
  //
  // When `omit_nondrivable_lanes` is true, nondriveable lanes will be omitted.
  //
  // `road_header` must not be nullptr.
  // `lane_section` must not be nullptr.
  // `xodr_lane_section_index` is the index of the LaneSection within the road and mustn't be negative.
  // `rg` must not be nullptr.
  // `factory` must not be nullptr.
  // `segment` must not be nullptr.
  //
  // @throws maliput::common::assertion_error When either `segment`,
  //         `lane_section`, `road_header` or `rg` are nullptr.
  static std::vector<LaneConstructionResult> BuildLanesForSegment(
      const xodr::RoadHeader* road_header, const xodr::LaneSection* lane_section, int xodr_lane_section_index,
      const RoadCurveFactoryBase* factory, bool omit_nondrivable_lanes, RoadGeometry* rg, Segment* segment);

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
