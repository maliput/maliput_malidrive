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
#include "maliput_malidrive/builder/road_geometry_builder.h"

#include <array>
#include <future>
#include <iterator>
#include <thread>

#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/geometry_base/branch_point.h>
#include <maliput/geometry_base/junction.h>
#include <maliput/utility/thread_pool.h>

#include "maliput_malidrive/builder/determine_tolerance.h"
#include "maliput_malidrive/builder/road_curve_factory.h"
#include "maliput_malidrive/builder/simplify_geometries.h"
#include "maliput_malidrive/builder/xodr_parser_configuration.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/lane_offset.h"
#include "maliput_malidrive/road_curve/piecewise_function.h"
#include "maliput_malidrive/road_curve/scaled_domain_function.h"

namespace malidrive {
namespace builder {

using maliput::geometry_base::BranchPoint;

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
                                         const RoadGeometryConfiguration& road_geometry_configuration)
    : rg_config_(road_geometry_configuration), manager_(std::move(manager)) {
  MALIDRIVE_THROW_UNLESS(manager_.get());
  MALIDRIVE_THROW_UNLESS(rg_config_.scale_length >= 0.);
  MALIDRIVE_VALIDATE(rg_config_.tolerances.angular_tolerance >= 0, maliput::common::assertion_error,
                     std::string("angular tolerance should be non-negative: ") +
                         std::to_string(rg_config_.tolerances.angular_tolerance));

  if (rg_config_.tolerances.linear_tolerance.has_value()) {
    MALIDRIVE_VALIDATE(rg_config_.tolerances.linear_tolerance.value() >= 0, maliput::common::assertion_error,
                       std::string("linear tolerance should be non-negative: ") +
                           std::to_string(rg_config_.tolerances.linear_tolerance.value()));
  } else {
    maliput::log()->trace("No `linear_tolerance` parameter is provided:");
    // When no linear_tolerance is specified there are two possible scenarios.
    // - If max_linear_tolerance is set, the linear_tolerance is expected to be the minimum value of
    // an allowed linear_tolerance range. See RoadGeometryBuilder::operator().
    // - If max_linear_tolerance isn't set, then no tolerance range will be defined, therefore linear_tolerance should
    // take a default value.
    if (rg_config_.tolerances.max_linear_tolerance.has_value()) {
      rg_config_.tolerances.linear_tolerance = constants::kBaseLinearTolerance;
    } else {
      rg_config_.tolerances.linear_tolerance = constants::kLinearTolerance;
    }
    maliput::log()->trace("|__ Using `linear_tolerance`: {}", rg_config_.tolerances.linear_tolerance.value());
  }

  // Verifies max_linear_tolerance value.
  if (rg_config_.tolerances.max_linear_tolerance.has_value()) {
    MALIDRIVE_VALIDATE(
        rg_config_.tolerances.max_linear_tolerance.value() >= rg_config_.tolerances.linear_tolerance,
        maliput::common::assertion_error,
        std::string("max_linear_tolerance should be greater than or equal to linear_tolerance.\n linear_tolerance: ") +
            std::to_string(rg_config_.tolerances.linear_tolerance.value()) + std::string("\n max_linear_tolerance: ") +
            std::to_string(rg_config_.tolerances.max_linear_tolerance.value()));
  }

  maliput::log()->trace("Tolerances:");
  rg_config_.tolerances.max_linear_tolerance.has_value()
      ? maliput::log()->trace("|__ linear tolerance: [{}, {}]", rg_config_.tolerances.linear_tolerance.value(),
                              rg_config_.tolerances.max_linear_tolerance.value())
      : maliput::log()->trace("|__ linear tolerance: {}", rg_config_.tolerances.linear_tolerance.value());
  maliput::log()->trace("|__ angular tolerance: {}", rg_config_.tolerances.angular_tolerance);

  maliput::log()->trace(
      "Build policy for the RoadGeometry building process: {}",
      rg_config_.build_policy.type == BuildPolicy::Type::kSequential
          ? "sequential"
          : "parallel -- " +
                (rg_config_.build_policy.num_threads.has_value()
                     ? std::to_string(GetEffectiveNumberOfThreads(rg_config_.build_policy)) + " threads(manual)"
                     : std::to_string(GetEffectiveNumberOfThreads(rg_config_.build_policy)) + " threads(automatic)"));

  maliput::log()->trace(
      "Strictness for meeting the OpenDrive standard: {}",
      RoadGeometryConfiguration::FromStandardStrictnessPolicyToStr(rg_config_.standard_strictness_policy));

  maliput::log()->trace("Omit non-drivable lanes policy: {}",
                        rg_config_.omit_nondrivable_lanes ? "enabled" : "disabled");

  maliput::log()->trace(
      "Geometry simplification policy: {}",
      rg_config_.simplification_policy ==
              RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel
          ? "SimplifyWithinToleranceAndKeepGeometryModel"
          : "None");

  factory_ = std::make_unique<builder::RoadCurveFactory>(
      rg_config_.tolerances.linear_tolerance.value(), rg_config_.scale_length, rg_config_.tolerances.angular_tolerance);
}

void RoadGeometryBuilder::VerifyNonNegativeLaneWidth(const std::vector<xodr::LaneWidth>& lane_widths,
                                                     const maliput::api::LaneId& lane_id, double xodr_lane_length,
                                                     double linear_tolerance, bool allow_negative_width) {
  const int num_polynomials = static_cast<int>(lane_widths.size());
  for (int i = 0; i < num_polynomials; i++) {
    if (lane_widths[i].a >= 0 && lane_widths[i].b >= 0 && lane_widths[i].c >= 0 && lane_widths[i].d >= 0) {
      // If all coefficients are non-negative, given that the function will be evaluated for non-negative p then
      // we can assume that it is a valid function before moving forward with the analysis.
      continue;
    }
    const bool end{i == num_polynomials - 1};
    // Start p value for the i-th lane width function in its range.
    const double p0{0.};
    // End p value for the i-th lane width function.
    const double p1{end ? (xodr_lane_length - lane_widths[i].s_0) : (lane_widths[i + 1].s_0 - lane_widths[i].s_0)};
    if (p1 - p0 < 0) {
      // If there is no range or it isn't valid then this verification is pointless.
      // This case will be taken into account down the river by the builder:
      // See `malidrive::builder::RoadCurveFactory::MakeLaneWidth`.
      continue;
    }
    // clang-format off
    auto eval_cubic_polynomial = [& lane_width = lane_widths[i]](double p) {
      return lane_width.d * p * p * p + lane_width.c * p * p + lane_width.b * p + lane_width.a;
    };
    // clang-format on

    // Verifies the image of the functions at p0 and p1.
    const double f_p0 = eval_cubic_polynomial(p0);
    const double f_p1 = eval_cubic_polynomial(p1);
    bool non_negative_limits{true};
    if (f_p0 < -linear_tolerance / 2.) {
      non_negative_limits = false;
      const std::string msg{"Lane " + lane_id.string() + " 's LaneWidth[" + std::to_string(i) +
                            "] function which range is [" + std::to_string(p0) + " , " + std::to_string(p1) +
                            "] is negative at the beginning of the range, at [lane_width_s = " + std::to_string(p0) +
                            " | lane_s = " + std::to_string(lane_widths[i].s_0 + p0) +
                            "] with a width value of: " + std::to_string(f_p0)};
      maliput::log()->warn(msg);
      if (!allow_negative_width) {
        MALIPUT_THROW_MESSAGE(msg);
      }
    }
    if (f_p1 < -linear_tolerance / 2.) {
      non_negative_limits = false;
      const std::string msg{"Lane " + lane_id.string() + " 's LaneWidth[" + std::to_string(i) +
                            "] function which range is [" + std::to_string(p0) + " , " + std::to_string(p1) +
                            "] is negative at the end of the range, at [lane_width_s = " + std::to_string(p1) +
                            " | lane_s = " + std::to_string(lane_widths[i].s_0 + p1) +
                            "] with a width value of: " + std::to_string(f_p1)};
      maliput::log()->warn(msg);
      if (!allow_negative_width) {
        MALIPUT_THROW_MESSAGE(msg);
      }
    }
    if (non_negative_limits) {
      // Once it is confirmed that we have non-negative width at the ends of the range we can analyze the
      // cubic locals to verify that there is no negative values in between.
      // Only when having a local min between p0 and p1 we can say that there may be a negative range, given that a
      // local min will make the polynomial to decrease in image. If there is a local min located in the range, we
      // have to verify then if that local min is located above or below of abscissa axis.
      const std::optional<double> p_local_min{
          FindLocalMinFromCubicPol(lane_widths[i].d, lane_widths[i].c, lane_widths[i].b, lane_widths[i].a)};
      if (p_local_min.has_value() && p_local_min.value() >= p0 && p_local_min.value() <= p1) {
        const double min_width = eval_cubic_polynomial(p_local_min.value());
        if (min_width < -linear_tolerance / 2.) {
          const std::string msg{"Lane " + lane_id.string() + " 's LaneWidth[" + std::to_string(i) +
                                "] function which range is [" + std::to_string(p0) + " , " + std::to_string(p1) +
                                "] presents negative values with a minimum value of [ " + std::to_string(min_width) +
                                "] located at [lane_width_s = " + std::to_string(p_local_min.value()) +
                                " | lane_s = " + std::to_string(p_local_min.value() + lane_widths[i].s_0) + "]"};
          maliput::log()->warn(msg);
          if (!allow_negative_width) {
            MALIPUT_THROW_MESSAGE(msg);
          }
        }
      }
    }
  }
}

RoadGeometryBuilder::LaneConstructionResult RoadGeometryBuilder::BuildLane(
    const xodr::Lane* lane, const xodr::RoadHeader* road_header, const xodr::LaneSection* lane_section,
    int xodr_lane_section_index, const RoadCurveFactoryBase* factory, const RoadGeometryConfiguration& rg_config,
    Segment* segment, road_curve::LaneOffset::AdjacentLaneFunctions* adjacent_lane_functions) {
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

  maliput::log()->trace("Creating LaneWidth for lane id {}_{}_{}", lane_id.string());

  const double xodr_p_0_lane{lane_section->s_0};
  const double xodr_p_1_lane{lane_section->s_0 + road_header->GetLaneSectionLength(xodr_lane_section_index)};
  // The ground curve might have just one geometry or multiple geometry definitions. When it is
  // defined as a piecewise curve, the XODR Track s parameter, known as p parameter might not be
  // exactly the same at the intersection of two adjacent pieces. That would lead to a mismatch
  // between the parameter the ground curve exposes and those constructed by the width and offset.
  // To keep them coupled, ScaledDomainFunction wraps them to use the same domain as the ground
  // curve.
  const double road_curve_p_0_lane{segment->road_curve()->PFromP(xodr_p_0_lane)};
  const double road_curve_p_1_lane{segment->road_curve()->PFromP(xodr_p_1_lane)};
  // Build a road_curve::CubicPolynomial for the lane width.
  const bool allow_semantic_errors{(rg_config.standard_strictness_policy &
                                    RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors) ==
                                   RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors};
  VerifyNonNegativeLaneWidth(lane->width_description, lane_id,
                             road_header->GetLaneSectionLength(xodr_lane_section_index), factory->linear_tolerance(),
                             allow_semantic_errors /* allow_negative_width */);
  // When semantic errors aren't allowed G1 contiguity must be enforced for all lanes.
  // Otherwise, only drivable lanes are enforced.
  const bool enforce_contiguity = !allow_semantic_errors || is_driveable_lane(*lane);
  std::unique_ptr<road_curve::Function> lane_width = std::make_unique<road_curve::ScaledDomainFunction>(
      factory->MakeLaneWidth(lane->width_description, xodr_p_0_lane, xodr_p_1_lane, enforce_contiguity),
      road_curve_p_0_lane, road_curve_p_1_lane, factory->linear_tolerance());

  maliput::log()->trace("Creating LaneOffset for lane id {}", lane_id.string());
  // Build a road_curve::CubicPolynomial for the lane offset.
  const bool no_adjacent_lane{adjacent_lane_functions->width == nullptr && adjacent_lane_functions->offset == nullptr};
  std::unique_ptr<road_curve::Function> lane_offset = std::make_unique<road_curve::ScaledDomainFunction>(
      std::make_unique<road_curve::LaneOffset>(
          (no_adjacent_lane ? std::nullopt : std::make_optional(*adjacent_lane_functions)), lane_width.get(),
          segment->reference_line_offset(), xodr_lane_id < 0 ? true : false, road_curve_p_0_lane, road_curve_p_1_lane,
          factory->linear_tolerance()),
      road_curve_p_0_lane, road_curve_p_1_lane, factory->linear_tolerance());

  //@}
  adjacent_lane_functions->width = lane_width.get();
  adjacent_lane_functions->offset = lane_offset.get();
  maliput::log()->trace("Building lane id {}", lane_id.string());
  auto built_lane =
      std::make_unique<Lane>(lane_id, xodr_track_id, xodr_lane_id, elevation_bounds, segment->road_curve(),
                             std::move(lane_width), std::move(lane_offset), road_curve_p_0_lane, road_curve_p_1_lane);
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
    lanes_construction_results.push_back(
        task_executor.Queue(LanesBuilder(junction_segments_attributes, factory_.get(), rg_config_, rg)));
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
      auto lanes_result = BuildLanesForSegment(
          segment_attributes.second.road_header, segment_attributes.second.lane_section,
          segment_attributes.second.lane_section_index, factory_.get(), rg_config_, rg, segment_attributes.first);
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
        segment_attributes.second.lane_section_index, factory, rg_config, rg, segment_attributes.first);
    built_lanes_result.insert(built_lanes_result.end(), std::make_move_iterator(lanes_result.begin()),
                              std::make_move_iterator(lanes_result.end()));
  }
  return built_lanes_result;
}

void RoadGeometryBuilder::FillSegmentsWithLanes(RoadGeometry* rg) {
  MALIDRIVE_THROW_UNLESS(rg != nullptr);
  std::vector<LaneConstructionResult> built_lanes_result =
      rg_config_.build_policy.type == malidrive::builder::BuildPolicy::Type::kParallel
          ? LanesBuilderParallelPolicy(GetEffectiveNumberOfThreads(rg_config_.build_policy), rg)
          : LanesBuilderSequentialPolicy(rg);
  for (auto& built_lane : built_lanes_result) {
    const auto result = lane_xodr_lane_properties_.insert(
        {built_lane.lane->id(), {built_lane.lane.get(), built_lane.xodr_lane_properties}});
    MALIDRIVE_THROW_UNLESS(result.second == true);

    const bool hide_lane =
        rg_config_.omit_nondrivable_lanes && !is_driveable_lane(*built_lane.xodr_lane_properties.lane);
    maliput::log()->trace("Lane ID: {}{} added to segment {}", built_lane.lane->id().string(),
                          hide_lane ? "(hidden)" : "", built_lane.segment->id().string());
    built_lane.segment->AddLane(std::move(built_lane.lane), hide_lane);
  }
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryBuilder::operator()() {
  maliput::log()->trace("Starting to build malidrive::RoadGeometry.");

  if (!rg_config_.tolerances.max_linear_tolerance.has_value()) {
    return DoBuild();
  }

  // Because a max linear tolerance is provided, the tolerance range logic is enabled.
  // A linear tolerance range is defined by `rg_config_tolerances.linear_tolerance` and
  // `rg_config_tolerances.max_linear_tolerance`.
  // The build process will iteratively increase the linear tolerance being used every time it fails building the
  // RoadGeometry. In each iteration, the linear tolerance is increased a percentage value defined by
  // constants::kToleranceStepMultiplier.
  std::vector<double> linear_tolerances{};
  // Tries with default values first.
  linear_tolerances.push_back(rg_config_.tolerances.linear_tolerance.value());

  // Populates the vector with higher linear tolerance values.
  double new_linear_tolerance = linear_tolerances.back() * constants::kToleranceStepMultiplier;
  while (new_linear_tolerance < rg_config_.tolerances.max_linear_tolerance.value()) {
    linear_tolerances.push_back(new_linear_tolerance);
    new_linear_tolerance = linear_tolerances.back() * constants::kToleranceStepMultiplier;
  }
  // Adds maximum linear tolerance at the end.
  linear_tolerances.push_back(rg_config_.tolerances.max_linear_tolerance.value());

  // Use always same angular tolerance and scale length.
  const std::vector<double> angular_tolerances(linear_tolerances.size(), rg_config_.tolerances.angular_tolerance);
  const std::vector<double> scale_lengths(linear_tolerances.size(), rg_config_.scale_length);

  Reset(linear_tolerances[0], angular_tolerances[0], scale_lengths[0]);

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
      std::unique_ptr<const maliput::api::RoadGeometry> rg = DoBuild();
      maliput::log()->info(
          "RoadGeometry loaded successfully after iteration [{}] using:\n\t|__ linear_tolerance = {}\n\t|__ "
          "angular_tolerance = {}\n\t|__ scale_length = {}",
          i, rg_config_.tolerances.linear_tolerance.value(), rg_config_.tolerances.angular_tolerance,
          rg_config_.scale_length);
      return rg;
    } catch (maliput::common::assertion_error& e) {
      maliput::log()->warn(
          "Iteration [{}] failed with : (linear_tolerance: {}, angular_tolerance: {}, scale_length: {}). "
          "Error: {}",
          i, rg_config_.tolerances.linear_tolerance.value(), rg_config_.tolerances.angular_tolerance,
          rg_config_.scale_length, e.what());
    }
    if (i < linear_tolerances.size() - 1) {
      Reset(linear_tolerances[i + 1], angular_tolerances[i + 1], scale_lengths[i + 1]);
      // @{ TODO(#12): It goes against dependency injection. Should use a provider instead.
      maliput::log()->trace("Rebuilding the DBManager");
      manager_ = xodr::LoadDataBaseFromFile(rg_config_.opendrive_file,
                                            XodrParserConfigurationFromRoadGeometryConfiguration(rg_config_));
      // @}
    }
  }
  const std::string file_description = "from " + rg_config_.opendrive_file;
  const std::string linear_tolerance_description =
      "Used linear tolerances are in the range [" + std::to_string(linear_tolerances[0]) + ", " +
      std::to_string(linear_tolerances.back()) + "] with an increasing step of " +
      std::to_string(static_cast<int>((constants::kToleranceStepMultiplier - 1) * 100)) + "% per iteration.";
  MALIDRIVE_THROW_MESSAGE("None of the tolerances(" + std::to_string(linear_tolerances.size()) +
                          ") worked to build a RoadGeometry " + file_description + ".\n\t" +
                          linear_tolerance_description);
  // @}
}

void RoadGeometryBuilder::Reset(double linear_tolerance, double angular_tolerance, double scale_length) {
  rg_config_.tolerances.linear_tolerance = linear_tolerance;
  rg_config_.tolerances.angular_tolerance = angular_tolerance;
  rg_config_.scale_length = scale_length;
  MALIDRIVE_THROW_UNLESS(rg_config_.tolerances.linear_tolerance.value() >= 0.);
  MALIDRIVE_THROW_UNLESS(rg_config_.tolerances.angular_tolerance >= 0.);
  MALIDRIVE_THROW_UNLESS(rg_config_.scale_length >= 0.);
  // TODO(#12): It goes against dependency injection. Should use a provider instead.
  factory_ = std::make_unique<builder::RoadCurveFactory>(linear_tolerance, scale_length, angular_tolerance);

  lane_xodr_lane_properties_.clear();
  junctions_segments_attributes_.clear();
  branch_point_indexer_ = UniqueIntegerProvider(0 /* base ID */);
  bps_.clear();
  junctions_.clear();
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryBuilder::DoBuild() {
  maliput::log()->trace("Using: linear_tolerance: {}", rg_config_.tolerances.linear_tolerance.value());
  maliput::log()->trace("Using: angular_tolerance: {}", rg_config_.tolerances.angular_tolerance);
  maliput::log()->trace("Using: scale_length: {}", rg_config_.scale_length);

  const std::map<xodr::RoadHeader::Id, xodr::RoadHeader> road_headers = manager_->GetRoadHeaders();

  const std::vector<xodr::DBManager::XodrGeometriesToSimplify> geometries_to_simplify =
      rg_config_.simplification_policy ==
              RoadGeometryConfiguration::SimplificationPolicy::kSimplifyWithinToleranceAndKeepGeometryModel
          ? manager_->GetGeometriesToSimplify(rg_config_.tolerances.linear_tolerance.value())
          : std::vector<xodr::DBManager::XodrGeometriesToSimplify>();

  auto rg =
      std::make_unique<RoadGeometry>(rg_config_.id, std::move(manager_), rg_config_.tolerances.linear_tolerance.value(),
                                     rg_config_.tolerances.angular_tolerance, rg_config_.scale_length,
                                     rg_config_.inertial_to_backend_frame_translation);

  maliput::log()->trace("Visiting XODR Roads...");
  for (const auto& road_header : road_headers) {
    maliput::log()->trace("Visiting XODR Road ID: {}.", road_header.first);
    auto road_curve = BuildRoadCurve(
        road_header.second, FilterGeometriesToSimplifyByRoadHeaderId(geometries_to_simplify, road_header.first));
    maliput::log()->trace("Creating ReferenceLineOffset for road id {}", road_header.first.string());
    auto reference_line_offset = std::make_unique<road_curve::ScaledDomainFunction>(
        factory_->MakeReferenceLineOffset(road_header.second.lanes.lanes_offset, road_header.second.s0(),
                                          road_header.second.s1()),
        road_curve->p0(), road_curve->p1(), rg_config_.tolerances.linear_tolerance.value());
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
  return rg;
}

std::unique_ptr<road_curve::RoadCurve> RoadGeometryBuilder::BuildRoadCurve(
    const xodr::RoadHeader& road_header,
    const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify) {
  const auto& start_geometry = road_header.reference_geometry.plan_view.geometries.begin();
  const auto start_lane_section = road_header.lanes.lanes_section.begin();
  if (std::abs(start_geometry->s_0 - start_lane_section->s_0) >= rg_config_.tolerances.linear_tolerance) {
    MALIDRIVE_THROW_MESSAGE(
        std::string("Start geometry differs more than linear_tolerance from the start lane section s coordinate.") +
        std::string("RoadId: ") + road_header.id.string() + std::string(", geometry.s0: ") +
        std::to_string(start_geometry->s_0) + std::string(", start lane section s0: ") +
        std::to_string(start_lane_section->s_0));
  }
  const auto& geometries{road_header.reference_geometry.plan_view.geometries};
  maliput::log()->trace("Creating GroundCurve for road id {}", road_header.id.string());
  auto ground_curve = MakeGroundCurve(geometries, geometries_to_simplify);
  maliput::log()->trace("Creating elevation function for road id {}", road_header.id.string());
  const bool allow_semantic_errors{(rg_config_.standard_strictness_policy &
                                    RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors) ==
                                   RoadGeometryConfiguration::StandardStrictnessPolicy::kAllowSemanticErrors};
  // When semantic errors aren't allowed G1 contiguity must be enforced for all lanes.
  // Otherwise, only drivable lanes are enforced.
  const bool enforce_contiguity = !allow_semantic_errors || !AreOnlyNonDrivableLanes(road_header);
  auto elevation = std::make_unique<road_curve::ScaledDomainFunction>(
      factory_->MakeElevation(road_header.reference_geometry.elevation_profile, road_header.s0(), road_header.s1(),
                              enforce_contiguity),
      ground_curve->p0(), ground_curve->p1(), rg_config_.tolerances.linear_tolerance.value());
  maliput::log()->trace("Creating superelevation function for road id {}", road_header.id.string());
  auto superelevation = std::make_unique<road_curve::ScaledDomainFunction>(
      factory_->MakeSuperelevation(road_header.reference_geometry.lateral_profile, road_header.s0(), road_header.s1(),
                                   enforce_contiguity),
      ground_curve->p0(), ground_curve->p1(), rg_config_.tolerances.linear_tolerance.value());
  maliput::log()->trace("Creating RoadCurve for road id {}", road_header.id.string());
  auto road_curve = factory_->MakeMalidriveRoadCurve(std::move(ground_curve), std::move(elevation),
                                                     std::move(superelevation), enforce_contiguity);
  maliput::log()->trace("RoadCurve for road id {} created.", road_header.id.string());
  return road_curve;
}

std::vector<RoadGeometryBuilder::LaneConstructionResult> RoadGeometryBuilder::BuildLanesForSegment(
    const xodr::RoadHeader* road_header, const xodr::LaneSection* lane_section, int xodr_lane_section_index,
    const RoadCurveFactoryBase* factory, const RoadGeometryConfiguration& rg_config, RoadGeometry* rg,
    Segment* segment) {
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
    maliput::log()->trace("Building Lane ID: {}_{}_{}.", road_header->id.string(), xodr_lane_section_index,
                          lane_it->id.string());
    LaneConstructionResult lane_construction_result =
        BuildLane(&(*lane_it), road_header, lane_section, xodr_lane_section_index, factory, rg_config, segment,
                  &adjacent_lane_functions);
    maliput::log()->trace("Built Lane ID: {}.", lane_construction_result.lane->id().string());
    built_lanes_result.insert(built_lanes_result.begin(), std::move(lane_construction_result));
  }
  adjacent_lane_functions = road_curve::LaneOffset::AdjacentLaneFunctions{nullptr, nullptr};
  for (auto lane_it = lane_section->left_lanes.cbegin(); lane_it != lane_section->left_lanes.cend(); ++lane_it) {
    LaneConstructionResult lane_construction_result =
        BuildLane(&(*lane_it), road_header, lane_section, xodr_lane_section_index, factory, rg_config, segment,
                  &adjacent_lane_functions);
    maliput::log()->trace("Built Lane ID: {}.", lane_construction_result.lane->id().string());
    built_lanes_result.push_back(std::move(lane_construction_result));
  }
  return built_lanes_result;
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
    if (rg_config_.omit_nondrivable_lanes && !is_driveable_lane(*lane_xodr_lane_properties.second.xodr_lane.lane)) {
      continue;
    }
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

void RoadGeometryBuilder::AttachLaneEndToBranchPoint(const maliput::api::LaneEnd& lane_end,
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

void RoadGeometryBuilder::SetDefaultsToBranchPoints() {
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

maliput::api::BranchPointId RoadGeometryBuilder::GetNewBranchPointId() {
  return GetBranchPointId(branch_point_indexer_.new_id());
}

bool RoadGeometryBuilder::IsLaneEndOnABSide(const maliput::api::BranchPoint* bp, const maliput::api::LaneEnd& lane_end,
                                            BranchPointSide bp_side) {
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
std::pair<maliput::geometry_base::BranchPoint*, std::optional<RoadGeometryBuilder::BranchPointSide>>
RoadGeometryBuilder::FindBranchpointByLaneEnd(
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
