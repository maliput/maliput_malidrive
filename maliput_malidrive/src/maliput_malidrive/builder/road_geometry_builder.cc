// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_builder.h"

#include "maliput/common/logger.h"
#include "maliput/common/maliput_unused.h"
#include "maliput/geometry_base/junction.h"
#include "maliput_malidrive/builder/determine_tolerance.h"
#include "maliput_malidrive/builder/road_curve_factory.h"
#include "maliput_malidrive/builder/simplify_geometries.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/lane_offset.h"
#include "maliput_malidrive/road_curve/piecewise_function.h"
#include "maliput_malidrive/road_curve/scaled_domain_function.h"

namespace malidrive {
namespace builder {
namespace {

// Returns a built lane.
// `adjacent_lane_functions` holds the offset and width functions of the immediate inner lane.
// `lane` must not be nullptr.
// `xodr_lane_section_index` must be non-negative.
// `road_header` must not be nullptr.
// `lane_section` must not be nullptr.
// `segment` must not be nullptr.
// `factory` must not be nullptr.
//
// @throws maliput::common::assertion_error When aforementioned conditions aren't met.
std::unique_ptr<Lane> BuildLane(const xodr::Lane* lane,
                                road_curve::LaneOffset::AdjacentLaneFunctions& adjacent_lane_functions,
                                int xodr_lane_section_index, const xodr::RoadHeader* road_header,
                                const xodr::LaneSection* lane_section, const Segment* segment,
                                const RoadCurveFactoryBase* factory) {
  MALIDRIVE_THROW_UNLESS(xodr_lane_section_index >= 0);
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  MALIDRIVE_THROW_UNLESS(road_header != nullptr);
  MALIDRIVE_THROW_UNLESS(lane_section != nullptr);
  MALIDRIVE_THROW_UNLESS(segment != nullptr);
  MALIDRIVE_THROW_UNLESS(factory != nullptr);
  const int xodr_track_id = std::stoi(road_header->id.string());
  MALIDRIVE_THROW_UNLESS(xodr_track_id >= 0);
  const int xodr_lane_id = std::stoi(lane->id.string());
  // Build a maliput::api::LaneId.
  const maliput::api::LaneId lane_id = GetLaneId(xodr_track_id, xodr_lane_section_index, xodr_lane_id);

  // Build a maliput::api::HBounds.
  // TODO(francocipollone): Obtain the elevation bounds out of the XODR's elevation profile record once it is parsed
  // into xodr::RoadHeader.
  const maliput::api::HBounds elevation_bounds{0., 0.};

  //@{
  // The ground curve might have just one geometry or multiple geometry definitions. When it is
  // defined as a piecewise curve, the XODR Track s parameter, known as p parameter might not be
  // exactly the same at the intersection of two adjacent pieces. That would lead to a mismatch
  // between the parameter the ground curve exposes and those constructed by the width and offset.
  // To keep them coupled, ScaledDomainFunction wraps them to use the same domain as the ground
  // curve.

  const double p_0{segment->road_curve()->PFromP(lane_section->s_0)};
  const double p_1{
      segment->road_curve()->PFromP(lane_section->s_0 + road_header->GetLaneSectionLength(xodr_lane_section_index))};
  // Build a road_curve::CubicPolynomial for the lane width.
  std::unique_ptr<road_curve::Function> lane_width = std::make_unique<road_curve::ScaledDomainFunction>(
      factory->MakeLaneWidth(lane->width_description, p_0, p_1), p_0, p_1, factory->linear_tolerance());

  // Build a road_curve::CubicPolynomial for the lane offset.
  const bool no_adjacent_lane{adjacent_lane_functions.width == nullptr && adjacent_lane_functions.offset == nullptr};
  std::unique_ptr<road_curve::Function> lane_offset = std::make_unique<road_curve::ScaledDomainFunction>(
      std::make_unique<road_curve::LaneOffset>(
          (no_adjacent_lane ? std::nullopt : std::make_optional<>(adjacent_lane_functions)), lane_width.get(),
          segment->reference_line_offset(), xodr_lane_id < 0 ? true : false, p_0, p_1, factory->linear_tolerance()),
      p_0, p_1, factory->linear_tolerance());

  //@}
  adjacent_lane_functions.width = lane_width.get();
  adjacent_lane_functions.offset = lane_offset.get();
  return std::make_unique<Lane>(lane_id, xodr_track_id, xodr_lane_id, elevation_bounds, segment->road_curve(),
                                std::move(lane_width), std::move(lane_offset), p_0, p_1);
}

// Filters all the xodr::DBManager::XodrGeometriesToSimplify that refer to the
// xodr::RoadHeader::ID `id`.
//
// @param geometries_to_simplify The result of xodr::DBManager::GetGeometriesToSimplify()
// @param id The xodr::RoadHeader::Id to filter `geometries_to_simplify`.
// @return A vector of xodr::DBManager::XodrGeometriesToSimplify that refer to `id`.
std::vector<xodr::DBManager::XodrGeometriesToSimplify> FilterGeometriesToSimplifyByRoadHeaderId(
    const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify,
    const xodr::RoadHeader::Id& id) {
  std::vector<xodr::DBManager::XodrGeometriesToSimplify> result;
  std::copy_if(geometries_to_simplify.begin(), geometries_to_simplify.end(), std::back_inserter(result),
               [id](const xodr::DBManager::XodrGeometriesToSimplify& geometry_to_simplify) {
                 return id == geometry_to_simplify.road_header_id;
               });
  return result;
}

}  // namespace

RoadGeometryBuilder::RoadGeometryBuilder(std::unique_ptr<xodr::DBManager> manager,
                                         const RoadGeometryConfiguration& road_geometry_configuration,
                                         std::unique_ptr<RoadCurveFactoryBase> factory)
    : RoadGeometryBuilderBase(road_geometry_configuration),
      simplification_policy_(road_geometry_configuration.simplification_policy),
      manager_(std::move(manager)),
      factory_(std::move(factory)) {
  MALIDRIVE_THROW_UNLESS(manager_.get());
  MALIDRIVE_THROW_UNLESS(factory_.get());
  if (simplification_policy_ ==
      RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel) {
    maliput::log()->trace("Enabled the simplification. Mode: SimplifyWithinToleranceAndKeepGeometryModel");
  }
  if (tolerance_selection_policy_ == RoadGeometryConfiguration::ToleranceSelectionPolicy::kAutomaticSelection) {
    maliput::log()->trace("Enabled automatic tolerance selection.");
  }
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryBuilder::operator()() {
  maliput::log()->trace("Starting to build malidrive::RoadGeometry.");

  if (tolerance_selection_policy_ == RoadGeometryConfiguration::ToleranceSelectionPolicy::kAutomaticSelection) {
    linear_tolerance_ = DetermineRoadGeometryLinearTolerance(manager_.get());
    angular_tolerance_ = DetermineRoadGeometryAngularTolerance(manager_.get());
    scale_length_ = DetermineRoadGeometryScaleLength(manager_.get(), linear_tolerance_, angular_tolerance_);
  }
  maliput::log()->trace("Using: linear_tolerance: {}", linear_tolerance_);
  maliput::log()->trace("Using: angular_tolerance: {}", angular_tolerance_);
  maliput::log()->trace("Using: scale_length: {}", scale_length_);

  const std::unordered_map<xodr::RoadHeader::Id, xodr::RoadHeader> road_headers = manager_->GetRoadHeaders();

  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      simplification_policy_ ==
              RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel
          ? manager_->GetGeometriesToSimplify(linear_tolerance_)
          : std::vector<xodr::DBManager::XodrGeometriesToSimplify>();

  auto rg =
      std::make_unique<RoadGeometry>(id_, std::move(manager_), linear_tolerance_, angular_tolerance_, scale_length_);

  maliput::log()->trace("Visiting XODR Roads...");
  for (const auto& road_header : road_headers) {
    maliput::log()->trace("Visiting XODR Road ID: {}.", road_header.first);
    auto road_curve = BuildRoadCurve(
        road_header.second, FilterGeometriesToSimplifyByRoadHeaderId(geometries_to_simplify, road_header.first));
    auto reference_line_offset =
        factory_->MakeReferenceLineOffset(road_header.second.lanes.lanes_offset, road_curve->p0(), road_curve->p1());
    // Add RoadCurve and the reference-line-offset function to the RoadGeometry.
    rg->AddRoadCharacteristics(road_header.first, std::move(road_curve), std::move(reference_line_offset));
    int lane_section_index = 0;
    for (const auto& lane_section : road_header.second.lanes.lanes_section) {
      maliput::log()->trace("Visiting XODR LaneSection: {} of Road: {}...", lane_section_index, road_header.first);
      maliput::geometry_base::Junction* junction{nullptr};
      // If junction id is specified in the xodr description.
      if (road_header.second.junction != "-1") {
        const maliput::api::JunctionId junction_id{road_header.second.junction};
        if (junctions_.find(junction_id) == junctions_.end()) {
          junction = rg->AddJunction(BuildJunction(road_header.second.junction));
          maliput::log()->trace("Built Junction ID: {}.", junction->id().string());
          junctions_[junction->id()] = junction;
        } else {
          junction = junctions_.at(junction_id);
          maliput::log()->trace("Using Junction ID: {}.", junction->id().string());
        }
      } else {
        junction = rg->AddJunction(BuildJunction(road_header.first.string(), lane_section_index));
        maliput::log()->trace("Built Junction ID: {}.", junction->id().string());
        junctions_[junction->id()] = junction;
      }
      // Add a segment for each lane section.
      const auto road_curve = rg->GetRoadCurve(road_header.first);
      const auto reference_line_offset = rg->GetReferenceLineOffset(road_header.first);
      Segment* segment = junction->AddSegment(std::make_unique<Segment>(
          GetSegmentId(std::stoi(road_header.first.string()), lane_section_index), road_curve, reference_line_offset,
          road_curve->PFromP(lane_section.s_0),
          road_curve->PFromP(lane_section.s_0 + road_header.second.GetLaneSectionLength(lane_section_index))));
      maliput::log()->trace("Built Segment ID: {}.", segment->id().string());

      maliput::log()->trace("Visiting XODR Lanes from XODR LaneSection: {} in XODR Road: {}...", lane_section_index,
                            road_header.first);
      // Process right lanes of the lane section.
      BuildLanesForSegment(lane_section.right_lanes, lane_section_index, &lane_section, &road_header.second,
                           road_curve::LaneOffset::kAtRightFromCenterLane, segment, rg.get());
      // Process left lanes of the lane section.
      BuildLanesForSegment(lane_section.left_lanes, lane_section_index, &lane_section, &road_header.second,
                           road_curve::LaneOffset::kAtLeftFromCenterLane, segment, rg.get());
      lane_section_index++;
    }
  }
  BuildBranchPointsForLanes(rg.get());
  SetDefaultsToBranchPoints();
  for (size_t i = 0; i < bps_.size(); ++i) {
    rg->AddBranchPoint(std::move(bps_[i]));
  }

  maliput::log()->trace("RoadGeometry is built.");
  return std::move(rg);
}

std::unique_ptr<road_curve::RoadCurve> RoadGeometryBuilder::BuildRoadCurve(
    const xodr::RoadHeader& road_header,
    const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify) {
  const auto& start_geometry = road_header.reference_geometry.plan_view.geometries.begin();
  const auto start_lane_section = road_header.lanes.lanes_section.begin();
  if (std::abs(start_geometry->s_0 - start_lane_section->s_0) >= linear_tolerance_) {
    MALIDRIVE_THROW_MESSAGE(
        std::string("Start geometry differs more than linear_tolerance from the start lane section s coordinate.") +
        std::string("RoadId: ") + road_header.id.string() + std::string(", geometry.s0: ") +
        std::to_string(start_geometry->s_0) + std::string(", start lane section s0: ") +
        std::to_string(start_lane_section->s_0));
  }
  const auto& geometries{road_header.reference_geometry.plan_view.geometries};
  auto ground_curve = MakeGroundCurve(geometries, geometries_to_simplify);
  auto elevation = std::make_unique<road_curve::ScaledDomainFunction>(
      factory_->MakeElevation(road_header.reference_geometry.elevation_profile, geometries.begin()->s_0,
                              (geometries.end() - 1)->s_0 + (geometries.end() - 1)->length),
      ground_curve->p0(), ground_curve->p1(), linear_tolerance_);
  auto superelevation = std::make_unique<road_curve::ScaledDomainFunction>(
      factory_->MakeSuperelevation(road_header.reference_geometry.lateral_profile, geometries.begin()->s_0,
                                   (geometries.end() - 1)->s_0 + (geometries.end() - 1)->length),
      ground_curve->p0(), ground_curve->p1(), linear_tolerance_);
  return factory_->MakeMalidriveRoadCurve(std::move(ground_curve), std::move(elevation), std::move(superelevation));
}

void RoadGeometryBuilder::BuildLanesForSegment(const std::vector<xodr::Lane>& lanes, int xodr_lane_section_index,
                                               const xodr::LaneSection* lane_section,
                                               const xodr::RoadHeader* road_header, bool reverse_build,
                                               Segment* segment, RoadGeometry* rg) {
  MALIDRIVE_THROW_UNLESS(lane_section != nullptr);
  MALIDRIVE_THROW_UNLESS(road_header != nullptr);
  MALIDRIVE_THROW_UNLESS(segment != nullptr);
  MALIDRIVE_THROW_UNLESS(rg != nullptr);

  std::vector<std::unique_ptr<Lane>> built_lanes;
  road_curve::LaneOffset::AdjacentLaneFunctions adjacent_lane_functions{nullptr, nullptr};
  // Build a lane out of a xodr::Lane.
  auto build_lane = [&adjacent_lane_functions, &built_lanes, xodr_lane_section_index, &lane_section, &road_header,
                     &segment, &lane_properties = this->lane_xodr_lane_properties_,
                     factory = this->factory_.get()](const xodr::Lane& xodr_lane) {
    // TODO(#596): Consider offset of non-driveable lanes.
    if (!is_driveable_lane(xodr_lane)) {
      return;
    }
    std::unique_ptr<Lane> lane = BuildLane(&xodr_lane, adjacent_lane_functions, xodr_lane_section_index, road_header,
                                           lane_section, segment, factory);
    maliput::log()->trace("Built Lane ID: {}.", lane->id().string());
    const auto result = lane_properties.insert(
        {lane->id(), MatchingLanes{lane.get(), MalidriveXodrLaneProperties(road_header, lane_section,
                                                                           xodr_lane_section_index, &xodr_lane)}});
    MALIDRIVE_THROW_UNLESS(result.second == true);
    built_lanes.push_back(std::move(lane));
  };

  // Lanes must be built from the center to the external lanes to correctly compute their
  // lane offset. The center lane is the reference and lanes to the right and to the left
  // use their immediate adjacent towards the center lane to compute the their own
  // lane offset polynomial. Segments expect lanes in order from the top most right to
  // the top most left. Code below guarantees the right construction and registration
  // order.
  if (reverse_build) {
    for (auto it = lanes.crbegin(); it != lanes.crend(); ++it) {
      build_lane(*it);
    }
    for (auto it = built_lanes.rbegin(); it != built_lanes.rend(); ++it) {
      segment->AddLane(std::move(*it));
    }
  } else {
    for (const auto& lane : lanes) {
      build_lane(lane);
    }
    for (auto& built_lane : built_lanes) {
      segment->AddLane(std::move(built_lane));
    }
  }
}

std::unique_ptr<maliput::geometry_base::Junction> RoadGeometryBuilder::BuildJunction(const std::string& xodr_track_id,
                                                                                     int lane_section_index) {
  const int xodr_track = std::stoi(xodr_track_id);
  MALIDRIVE_THROW_UNLESS(xodr_track >= 0);
  MALIDRIVE_THROW_UNLESS(lane_section_index >= 0);
  return std::make_unique<maliput::geometry_base::Junction>(GetJunctionId(xodr_track, lane_section_index));
}

std::unique_ptr<maliput::geometry_base::Junction> RoadGeometryBuilder::BuildJunction(
    const std::string& xodr_junction_id) {
  const int xodr_junction = std::stoi(xodr_junction_id);
  MALIDRIVE_THROW_UNLESS(xodr_junction >= 0);
  return std::make_unique<maliput::geometry_base::Junction>(GetJunctionId(xodr_junction));
}

std::unique_ptr<malidrive::road_curve::GroundCurve> RoadGeometryBuilder::MakeGroundCurve(
    const std::vector<xodr::Geometry>& geometries,
    const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify) {
  MALIDRIVE_THROW_UNLESS(!geometries.empty());

  if (!geometries_to_simplify.empty()) {
    maliput::log()->trace("Simplifying XODR Road {}.", geometries_to_simplify.front().road_header_id.string());
  }

  if (geometries.size() == 1) {
    const auto start_geometry = geometries.begin();
    switch (start_geometry->type) {
      case xodr::Geometry::Type::kLine:
        return factory_->MakeLineGroundCurve(*start_geometry);
      case xodr::Geometry::Type::kArc:
        return factory_->MakeArcGroundCurve(*start_geometry);
      default:
        MALIDRIVE_THROW_MESSAGE("Geometry " + xodr::Geometry::type_to_str(start_geometry->type) + " cannot be built");
    }
  }
  return factory_->MakePiecewiseGroundCurve(SimplifyGeometries(geometries, geometries_to_simplify));
}

void RoadGeometryBuilder::BuildBranchPointsForLanes(RoadGeometry* rg) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  maliput::log()->trace("Building BranchPoints for Lanes...");

  for (auto& lane_xodr_lane_properties : lane_xodr_lane_properties_) {
    FindOrCreateBranchPointFor(lane_xodr_lane_properties.second.xodr_lane,
                               lane_xodr_lane_properties.second.malidrive_lane, rg);
  }
}

void RoadGeometryBuilder::FindOrCreateBranchPointFor(const MalidriveXodrLaneProperties& xodr_lane_properties,
                                                     Lane* lane, RoadGeometry* rg) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  MALIDRIVE_THROW_UNLESS(lane != nullptr);

  std::vector<maliput::api::LaneEnd> connecting_lane_ends;
  // Start BP
  const maliput::api::LaneEnd start_lane_end(lane, maliput::api::LaneEnd::Which::kStart);
  maliput::log()->trace("Looking for start connections of Lane ID: {}.", lane->id().string());
  // Gets all the connecting LaneEnds to which this lane end connects to.
  connecting_lane_ends = FindConnectingLaneEndsForLaneEnd(start_lane_end, xodr_lane_properties, rg);
  // FindsOrCreates and Attaches to the BranchPoint.
  AttachLaneEndToBranchPoint(start_lane_end, connecting_lane_ends);

  // End BP
  const maliput::api::LaneEnd end_lane_end(lane, maliput::api::LaneEnd::Which::kFinish);
  maliput::log()->trace("Looking for end connections of Lane ID: {}.", lane->id().string());
  // Gets all the connecting LaneEnds to which this lane end connects to.
  connecting_lane_ends = FindConnectingLaneEndsForLaneEnd(end_lane_end, xodr_lane_properties, rg);
  // FindsOrCreates and Attaches to the BranchPoint.
  AttachLaneEndToBranchPoint(end_lane_end, connecting_lane_ends);
}

std::vector<maliput::api::LaneEnd> RoadGeometryBuilder::FindConnectingLaneEndsForLaneEnd(
    const maliput::api::LaneEnd& lane_end, const MalidriveXodrLaneProperties& xodr_lane_properties, RoadGeometry* rg) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);

  // Checks if the LaneSection is an inner LaneSection or not. When it is
  // inner, the Lane end look up is simplified as it connects to lanes which
  // live in the previous / next LaneSection.
  const int num_lane_sections = static_cast<int>(xodr_lane_properties.road_header->lanes.lanes_section.size());
  if (num_lane_sections > 1 &&
      ((xodr_lane_properties.lane_section_index > 0 &&
        xodr_lane_properties.lane_section_index < num_lane_sections - 1) ||
       (xodr_lane_properties.lane_section_index == 0 && lane_end.end == maliput::api::LaneEnd::Which::kFinish) ||
       (xodr_lane_properties.lane_section_index == num_lane_sections - 1 &&
        lane_end.end == maliput::api::LaneEnd::Which::kStart))) {
    return SolveLaneEndsForInnerLaneSection(rg, lane_end, xodr_lane_properties);
  }

  // Checks Lanes at the extremes of a Road and that road does not belong to a
  // Junction.
  if (std::stoi(xodr_lane_properties.road_header->junction) < 0) {
    // It doesn't belong to a Junction, but it could be connected to one.
    if (lane_end.end == maliput::api::LaneEnd::Which::kStart) {
      if (xodr_lane_properties.road_header->road_link.predecessor.has_value()) {
        if (xodr_lane_properties.road_header->road_link.predecessor->element_type ==
            xodr::RoadLink::ElementType::kRoad) {
          return SolveLaneEndsForConnectingRoad(rg, xodr_lane_properties, rg->get_manager()->GetRoadHeaders(),
                                                XodrConnectionType::kPredecessor);
        } else {
          // Predecessor is a junction.
          return SolveLaneEndsForJunction(rg, xodr_lane_properties, rg->get_manager()->GetRoadHeaders(),
                                          rg->get_manager()->GetJunctions(), XodrConnectionType::kPredecessor);
        }
      }
    } else {
      if (xodr_lane_properties.road_header->road_link.successor.has_value()) {
        if (xodr_lane_properties.road_header->road_link.successor->element_type == xodr::RoadLink::ElementType::kRoad) {
          return SolveLaneEndsForConnectingRoad(rg, xodr_lane_properties, rg->get_manager()->GetRoadHeaders(),
                                                XodrConnectionType::kSuccessor);
        } else {
          // Successor is a junction.
          return SolveLaneEndsForJunction(rg, xodr_lane_properties, rg->get_manager()->GetRoadHeaders(),
                                          rg->get_manager()->GetJunctions(), XodrConnectionType::kSuccessor);
        }
      }
    }
    return {};
  } else {
    // Should look into the junction to look for the LaneEnds.
    return SolveLaneEndsWithinJunction(rg, xodr_lane_properties, rg->get_manager()->GetRoadHeaders(),
                                       lane_end.end != maliput::api::LaneEnd::Which::kStart
                                           ? XodrConnectionType::kSuccessor
                                           : XodrConnectionType::kPredecessor);
  }
}

}  // namespace builder
}  // namespace malidrive