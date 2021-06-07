// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/road_geometry_builder.h"

#include <array>
#include <future>
#include <iterator>
#include <thread>

#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/geometry_base/junction.h>
#include <maliput/utilities/thread_pool.h>

#include "maliput_malidrive/builder/determine_tolerance.h"
#include "maliput_malidrive/builder/road_curve_factory.h"
#include "maliput_malidrive/builder/simplify_geometries.h"
#include "maliput_malidrive/builder/xodr_parser_configuration.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/lane_offset.h"
#include "maliput_malidrive/road_curve/piecewise_function.h"
#include "maliput_malidrive/road_curve/scaled_domain_function.h"

namespace malidrive {
namespace builder {
namespace {

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

// When `build_policy.num_threads.has_value()` it returns its value,
// otherwise what the hardware supports minus one (running thread).
//
// `build_policy` Is the build policy for the RoadGeometryBuilder.
std::size_t GetEffectiveNumberOfThreads(const malidrive::builder::BuildPolicy& build_policy) {
  return build_policy.num_threads.has_value() ? static_cast<std::size_t>(build_policy.num_threads.value())
                                              : std::thread::hardware_concurrency() - 1;
  ;
}

}  // namespace

RoadGeometryBuilder::RoadGeometryBuilder(std::unique_ptr<xodr::DBManager> manager,
                                         const RoadGeometryConfiguration& road_geometry_configuration,
                                         std::unique_ptr<RoadCurveFactoryBase> factory)
    : RoadGeometryBuilderBase(road_geometry_configuration),
      rg_config_(road_geometry_configuration),
      manager_(std::move(manager)),
      factory_(std::move(factory)) {
  MALIDRIVE_THROW_UNLESS(manager_.get());
  MALIDRIVE_THROW_UNLESS(factory_.get());
  maliput::log()->trace(
      "Build policy for the RoadGeometry building process: {}",
      build_policy_.type == BuildPolicy::Type::kSequential
          ? "sequential"
          : "parallel -- " +
                (build_policy_.num_threads.has_value()
                     ? std::to_string(GetEffectiveNumberOfThreads(build_policy_)) + " threads(manual)"
                     : std::to_string(GetEffectiveNumberOfThreads(build_policy_)) + " threads(automatic)"));

  maliput::log()->trace(
      "Strictness for meeting the OpenDrive standard: {}",
      RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(rg_config_.standard_strictness_policy));

  if (rg_config_.simplification_policy ==
      RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel) {
    maliput::log()->trace("Enabled the simplification. Mode: SimplifyWithinToleranceAndKeepGeometryModel");
  }
  if (rg_config_.tolerance_selection_policy ==
      RoadGeometryConfiguration::ToleranceSelectionPolicy::kAutomaticSelection) {
    maliput::log()->trace("Enabled automatic tolerance selection.");
  }
}

RoadGeometryBuilder::LaneConstructionResult RoadGeometryBuilder::BuildLane(
    const xodr::Lane* lane, const xodr::RoadHeader* road_header, const xodr::LaneSection* lane_section,
    int xodr_lane_section_index, const RoadCurveFactoryBase* factory, Segment* segment,
    road_curve::LaneOffset::AdjacentLaneFunctions* adjacent_lane_functions) {
  MALIDRIVE_THROW_UNLESS(lane != nullptr);
  MALIDRIVE_THROW_UNLESS(road_header != nullptr);
  MALIDRIVE_THROW_UNLESS(lane_section != nullptr);
  MALIDRIVE_THROW_UNLESS(xodr_lane_section_index >= 0);
  MALIDRIVE_THROW_UNLESS(factory != nullptr);
  MALIDRIVE_THROW_UNLESS(segment != nullptr);
  MALIDRIVE_THROW_UNLESS(xodr_lane_section_index >= 0);
  const int xodr_track_id = std::stoi(road_header->id.string());
  MALIDRIVE_THROW_UNLESS(xodr_track_id >= 0);
  const int xodr_lane_id = std::stoi(lane->id.string());
  // Build a maliput::api::LaneId.
  const maliput::api::LaneId lane_id = GetLaneId(xodr_track_id, xodr_lane_section_index, xodr_lane_id);

  // Build a maliput::api::HBounds.
  // TODO(#69): Un-hardcode the elevation bound.
  const maliput::api::HBounds elevation_bounds{0., 5.};

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
  const bool no_adjacent_lane{adjacent_lane_functions->width == nullptr && adjacent_lane_functions->offset == nullptr};
  std::unique_ptr<road_curve::Function> lane_offset = std::make_unique<road_curve::ScaledDomainFunction>(
      std::make_unique<road_curve::LaneOffset>(
          (no_adjacent_lane ? std::nullopt : std::make_optional(*adjacent_lane_functions)), lane_width.get(),
          segment->reference_line_offset(), xodr_lane_id < 0 ? true : false, p_0, p_1, factory->linear_tolerance()),
      p_0, p_1, factory->linear_tolerance());

  //@}
  adjacent_lane_functions->width = lane_width.get();
  adjacent_lane_functions->offset = lane_offset.get();
  auto built_lane =
      std::make_unique<Lane>(lane_id, xodr_track_id, xodr_lane_id, elevation_bounds, segment->road_curve(),
                             std::move(lane_width), std::move(lane_offset), p_0, p_1);
  return {segment, std::move(built_lane), {road_header, lane_section, xodr_lane_section_index, lane}};
}

std::vector<RoadGeometryBuilder::LaneConstructionResult> RoadGeometryBuilder::LanesBuilderParallelPolicy(
    std::size_t num_of_threads, RoadGeometry* rg) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  MALIDRIVE_THROW_UNLESS(num_of_threads > 0);
  maliput::utility::ThreadPool task_executor(num_of_threads);

  // Queue all the tasks in the thread pool. Each task will build all the lanes of a junction.
  std::vector<std::future<std::vector<RoadGeometryBuilder::LaneConstructionResult>>> lanes_construction_results;
  for (const auto& junction_segments_attributes : junctions_segments_attributes_) {
    lanes_construction_results.push_back(task_executor.Queue(
        LanesBuilder(junction_segments_attributes, rg, factory_.get(), rg_config_.omit_nondrivable_lanes)));
  }
  // The threads are on hold until start method is called.
  task_executor.Start();
  // Await the result of the tasks and then destroy the threads.
  task_executor.Finish();
  // Collect the tasks results.
  std::vector<RoadGeometryBuilder::LaneConstructionResult> lanes_results;
  for (auto& future_lane_construction_result : lanes_construction_results) {
    for (auto& lane_result : future_lane_construction_result.get()) {
      lanes_results.push_back(std::move(lane_result));
    }
  }
  return lanes_results;
}

std::vector<RoadGeometryBuilder::LaneConstructionResult> RoadGeometryBuilder::LanesBuilderSequentialPolicy(
    RoadGeometry* rg) {
  std::vector<LaneConstructionResult> built_lanes_result;
  for (const auto& junction_segments_attributes : junctions_segments_attributes_) {
    for (const auto& segment_attributes : junction_segments_attributes.second) {
      // Process lanes of the lane section.
      auto lanes_result =
          BuildLanesForSegment(segment_attributes.second.road_header, segment_attributes.second.lane_section,
                               segment_attributes.second.lane_section_index, factory_.get(),
                               rg_config_.omit_nondrivable_lanes, rg, segment_attributes.first);
      built_lanes_result.insert(built_lanes_result.end(), std::make_move_iterator(lanes_result.begin()),
                                std::make_move_iterator(lanes_result.end()));
    }
  }
  return built_lanes_result;
}

std::vector<RoadGeometryBuilder::LaneConstructionResult> RoadGeometryBuilder::LanesBuilder::operator()() {
  std::vector<LaneConstructionResult> built_lanes_result;
  for (const auto& segment_attributes : junction_segments_attributes.second) {
    // Process lanes of the lane section.
    auto lanes_result = BuildLanesForSegment(
        segment_attributes.second.road_header, segment_attributes.second.lane_section,
        segment_attributes.second.lane_section_index, factory, omit_nondrivable_lanes, rg, segment_attributes.first);
    built_lanes_result.insert(built_lanes_result.end(), std::make_move_iterator(lanes_result.begin()),
                              std::make_move_iterator(lanes_result.end()));
  }
  return built_lanes_result;
}

void RoadGeometryBuilder::FillSegmentsWithLanes(RoadGeometry* rg) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  std::vector<LaneConstructionResult> built_lanes_result =
      build_policy_.type == malidrive::builder::BuildPolicy::Type::kParallel
          ? LanesBuilderParallelPolicy(GetEffectiveNumberOfThreads(build_policy_), rg)
          : LanesBuilderSequentialPolicy(rg);
  for (auto& built_lane : built_lanes_result) {
    const auto result = lane_xodr_lane_properties_.insert(
        {built_lane.lane->id(), {built_lane.lane.get(), built_lane.xodr_lane_properties}});
    MALIDRIVE_THROW_UNLESS(result.second == true);

    maliput::log()->trace("Lane ID: {} added to segment {}", built_lane.lane->id().string(),
                          built_lane.segment->id().string());
    built_lane.segment->AddLane(std::move(built_lane.lane));
  }
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryBuilder::operator()() {
  maliput::log()->trace("Starting to build malidrive::RoadGeometry.");

  if (rg_config_.tolerance_selection_policy == RoadGeometryConfiguration::ToleranceSelectionPolicy::kManualSelection) {
    maliput::log()->trace("Manual tolerance selection builder.");
    return DoBuild();
  }

  std::array<double, constants::kMaxToleranceSelectionRounds + 1> linear_tolerances{};
  std::array<double, constants::kMaxToleranceSelectionRounds + 1> angular_tolerances{};
  std::array<double, constants::kMaxToleranceSelectionRounds + 1> scale_lengths{};

  // Tries with default values first.
  linear_tolerances[0] = linear_tolerance_;
  angular_tolerances[0] = angular_tolerance_;
  scale_lengths[0] = scale_length_;

  // Populates the vector with higher tolerance values but always use the same scale length.
  for (size_t i = 1; i < linear_tolerances.size(); ++i) {
    linear_tolerances[i] = linear_tolerances[i - 1] * 1.1;
    angular_tolerances[i] = angular_tolerances[i - 1] * 1.1;
    scale_lengths[i] = constants::kScaleLength;
  }

  // @{ Code in doc-bloc goes against https://drake.mit.edu/styleguide/cppguide.html#Exceptions
  //    See https://github.com/ToyotaResearchInstitute/maliput_malidrive/pull/77#discussion_r643434626
  //    for the discussion about it.
  //    There is a try-catch block that captures maliput::common::assertion_error
  //    exception types which are only thrown by maliput and maliput_malidrive.
  //    Because of extensive testing, maliput::common::assertion_error types are
  //    expected only when a linear or angular tolerance constraint is violated.
  //    In order to comply with the style-guide, a major refactor to the code
  //    is required. In case none of the tolerances are suitable to construct
  //    the RoadGeometry, a maliput::common::assertion_error exception will be
  //    thrown.
  // Iterates over the tolerances.
  maliput::log()->debug("Starting linear and angular tolerance trials to build the RoadGeometry.");
  for (size_t i = 0; i < linear_tolerances.size(); ++i) {
    maliput::log()->debug("Iteration [{}] with (linear_tolerance: {}, angular_tolerance: {}, scale_length: {}).", i,
                          linear_tolerances[i], angular_tolerances[i], scale_lengths[i]);
    try {
      Reset(linear_tolerances[i], angular_tolerances[i], scale_lengths[i]);
      return DoBuild();
    } catch (maliput::common::assertion_error& e) {
      maliput::log()->warn(
          "Iteration [{}] failed with : (linear_tolerance: {}, angular_tolerance: {}, scale_length: {}). "
          "Error: {}",
          linear_tolerance_, angular_tolerance_, scale_length_, e.what());
    }
    // @{ TODO(#12): It goes against dependency injection. Should use a provider instead.
    maliput::log()->trace("Rebuilding the DBManager");
    manager_ = xodr::LoadDataBaseFromFile(rg_config_.opendrive_file.value(),
                                          XodrParserConfigurationFromRoadGeometryConfiguration(rg_config_));
    // @}
  }
  const std::string file_description =
      rg_config_.opendrive_file ? ("from " + rg_config_.opendrive_file.value()) : "(No OpenDRIVE file specified)";
  MALIDRIVE_THROW_MESSAGE("None of the tolerances worked to build a RoadGeometry " + file_description + ".");
  // @}
}

void RoadGeometryBuilder::Reset(double linear_tolerance, double angular_tolerance, double scale_length) {
  // @{ Reset this' members
  rg_config_.linear_tolerance = linear_tolerance;
  rg_config_.angular_tolerance = angular_tolerance;
  rg_config_.scale_length = scale_length;
  lane_xodr_lane_properties_.clear();
  junctions_segments_attributes_.clear();
  // TODO(#12): It goes against dependency injection. Should use a provider instead.
  factory_ = std::make_unique<builder::RoadCurveFactory>(linear_tolerance, scale_length, angular_tolerance);
  // @}

  // @{ Reset parent members
  linear_tolerance_ = linear_tolerance;
  angular_tolerance_ = angular_tolerance;
  scale_length_ = scale_length;
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ >= 0.);
  MALIDRIVE_THROW_UNLESS(angular_tolerance_ >= 0.);
  MALIDRIVE_THROW_UNLESS(scale_length_ >= 0.);

  branch_point_indexer_ = UniqueIntegerProvider(0 /* base ID */);
  bps_.clear();
  junctions_.clear();
  // @}
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryBuilder::DoBuild() {
  maliput::log()->trace("Using: linear_tolerance: {}", linear_tolerance_);
  maliput::log()->trace("Using: angular_tolerance: {}", angular_tolerance_);
  maliput::log()->trace("Using: scale_length: {}", scale_length_);

  const std::unordered_map<xodr::RoadHeader::Id, xodr::RoadHeader> road_headers = manager_->GetRoadHeaders();

  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      rg_config_.simplification_policy ==
              RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel
          ? manager_->GetGeometriesToSimplify(linear_tolerance_)
          : std::vector<xodr::DBManager::XodrGeometriesToSimplify>();

  auto rg = std::make_unique<RoadGeometry>(id_, std::move(manager_), linear_tolerance_, angular_tolerance_,
                                           scale_length_, inertial_to_backend_frame_translation_);

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

      // Save all the attributes for building the lanes later on.
      junctions_segments_attributes_[junction][segment] = {&road_header.second, &lane_section, lane_section_index};

      lane_section_index++;
    }
  }
  FillSegmentsWithLanes(rg.get());

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

std::vector<RoadGeometryBuilder::LaneConstructionResult> RoadGeometryBuilder::BuildLanesForSegment(
    const xodr::RoadHeader* road_header, const xodr::LaneSection* lane_section, int xodr_lane_section_index,
    const RoadCurveFactoryBase* factory, bool omit_nondrivable_lanes, RoadGeometry* rg, Segment* segment) {
  MALIDRIVE_THROW_UNLESS(lane_section != nullptr);
  MALIDRIVE_THROW_UNLESS(road_header != nullptr);
  MALIDRIVE_THROW_UNLESS(segment != nullptr);
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  MALIDRIVE_THROW_UNLESS(factory != nullptr);

  std::vector<RoadGeometryBuilder::LaneConstructionResult> built_lanes_result;
  road_curve::LaneOffset::AdjacentLaneFunctions adjacent_lane_functions{nullptr, nullptr};

  // Lanes must be built from the center to the external lanes to correctly compute their
  // lane offset. The center lane is the reference and lanes to the right and to the left
  // use their immediate adjacent towards the center lane to compute the their own
  // lane offset polynomial. Segments expect lanes in order from the top most right to
  // the top most left. Code below guarantees the right construction and registration
  // order.
  for (auto lane_it = lane_section->right_lanes.crbegin(); lane_it != lane_section->right_lanes.crend(); ++lane_it) {
    // Skip nondriveable lanes when the builder flag is turned on.
    if (omit_nondrivable_lanes && !is_driveable_lane(*lane_it)) {
      continue;
    }
    LaneConstructionResult lane_construction_result = BuildLane(
        &(*lane_it), road_header, lane_section, xodr_lane_section_index, factory, segment, &adjacent_lane_functions);
    maliput::log()->trace("Built Lane ID: {}.", lane_construction_result.lane->id().string());
    built_lanes_result.insert(built_lanes_result.begin(), std::move(lane_construction_result));
  }
  adjacent_lane_functions = road_curve::LaneOffset::AdjacentLaneFunctions{nullptr, nullptr};
  for (auto lane_it = lane_section->left_lanes.cbegin(); lane_it != lane_section->left_lanes.cend(); ++lane_it) {
    // Skip nondriveable lanes when the builder flag is turned on.
    if (omit_nondrivable_lanes && !is_driveable_lane(*lane_it)) {
      continue;
    }
    LaneConstructionResult lane_construction_result = BuildLane(
        &(*lane_it), road_header, lane_section, xodr_lane_section_index, factory, segment, &adjacent_lane_functions);
    maliput::log()->trace("Built Lane ID: {}.", lane_construction_result.lane->id().string());
    built_lanes_result.push_back(std::move(lane_construction_result));
  }
  return std::move(built_lanes_result);
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
