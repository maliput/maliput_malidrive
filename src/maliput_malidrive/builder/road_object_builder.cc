// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include "maliput_malidrive/builder/road_object_builder.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <maliput/api/lane_data.h>
#include <maliput/api/objects/road_object.h>
#include <maliput/common/logger.h>
#include <maliput/math/bounding_box.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/road_object_type_mapper.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

namespace {

/// Concrete subclass of maliput::api::objects::RoadObject.
/// The base class has a protected constructor, so this subclass is needed
/// to construct instances.
class MalidriveRoadObject final : public maliput::api::objects::RoadObject {
 public:
  MalidriveRoadObject(const Id& id, maliput::api::objects::RoadObjectType type,
                      const maliput::api::objects::RoadObjectPosition& position,
                      const maliput::api::Rotation& orientation, const maliput::math::BoundingBox& bounding_box,
                      bool is_dynamic, std::vector<maliput::api::LaneId> related_lanes, std::optional<std::string> name,
                      std::optional<std::string> subtype,
                      std::vector<std::unique_ptr<maliput::api::objects::Outline>> outlines,
                      std::unordered_map<std::string, std::string> properties,
                      std::vector<maliput::api::objects::ContinuousObject> continuous_properties, bool is_movable)
      : RoadObject(id, type, position, orientation, bounding_box, is_dynamic, std::move(related_lanes), std::move(name),
                   std::move(subtype), std::move(outlines), std::move(properties), std::move(continuous_properties),
                   is_movable) {}
};

std::optional<std::string> NormalizeSubtype(const std::string& subtype) {
  if (subtype.empty() || subtype == "-1" || subtype == "none") {
    return std::nullopt;
  }
  return subtype;
}

std::optional<maliput::api::objects::RoadObjectType> StringToRoadObjectType(const std::string& device_semantics) {
  const auto mapper = maliput::api::objects::RoadObjectTypeMapper();
  for (const auto& [type, name] : mapper) {
    if (device_semantics == name) {
      return type;
    }
  }
  return std::nullopt;
}

/// Returns the step made for a linear interpolation between @p start and @p end, given the @p ratio.
double Lerp(double start, double end, double ratio) { return start + (end - start) * ratio; }

/// Clamps a scalar into [0, 1].
double Clamp01(double value) { return std::max(0., std::min(1., value)); }

/// Resolves a repeat width boundary, falling back to object-level dimensions.
double ResolveRepeatWidthBoundary(const xodr::object::Object& object,
                                  const std::optional<double>& repeat_width_boundary) {
  // Repeat width may be omitted in XODR. In that case, use object-level
  // dimensions so continuous samples remain available.
  if (repeat_width_boundary.has_value()) {
    return repeat_width_boundary.value();
  }
  if (object.width.has_value()) {
    return object.width.value();
  }
  if (object.radius.has_value()) {
    return object.radius.value() * 2.;
  }
  return 0.;
}

/// Builds the s-sample set for a qualifying repeat.
///
/// Sampling uses the repeat length and the configured samples-per-road value.
/// Repeat start and end are always present in the returned vector.
std::vector<double> BuildSampleSCoordinates(const xodr::object::Repeat& repeat, int samples_per_road) {
  MALIDRIVE_VALIDATE(samples_per_road > 0, maliput::common::assertion_error, "samples_per_road must be positive.");

  const double start_s = repeat.s;
  const double end_s = repeat.s + repeat.length;
  const double min_s = std::min(start_s, end_s);
  const double max_s = std::max(start_s, end_s);

  if (std::abs(max_s - min_s) < 1e-12) {
    return {start_s};
  }

  const double nominal_step = (max_s - min_s) / static_cast<double>(samples_per_road);
  const double step = nominal_step > 0. ? nominal_step : (max_s - min_s);

  std::vector<double> samples{start_s};
  double sample_s = min_s + step;
  const double kEpsilon = 1e-10;
  while (sample_s < max_s - kEpsilon) {
    samples.push_back(start_s <= end_s ? sample_s : (start_s - (sample_s - min_s)));
    sample_s += step;
  }
  if (std::abs(samples.back() - end_s) > kEpsilon) {
    samples.push_back(end_s);
  }
  return samples;
}

/// Converts qualifying XODR repeat entries into ContinuousObject samples.
///
/// Only repeats with distance == 0 are considered. By default, each sample is
/// projected from OpenDRIVE road coordinates. The returned inertial position's
/// lateral coordinate is centered on the object (so interpolated width extends
/// width/2 to each side along the object r-axis).
///
/// The returned inertial position's z-coordinate is anchored at the interpolated
/// object bottom for that s-coordinate, and the interpolated height extends upward from that point.
/// When detachFromReferenceLine is true, the sampled points are interpolated between
/// the repeat endpoints as a straight line.
maliput::api::InertialPosition BuildRepeatSamplePoint(const xodr::object::Object& object,
                                                      const xodr::object::Repeat& repeat,
                                                      const maliput::api::RoadGeometry* road_geometry,
                                                      const xodr::RoadHeader::Id& road_id, double sample_road_s,
                                                      double ratio, const malidrive::RoadGeometry* mali_rg) {
  const double t = Lerp(repeat.t_start, repeat.t_end, ratio);
  const double z_offset = Lerp(repeat.z_offset_start, repeat.z_offset_end, ratio);
  const double width_start = ResolveRepeatWidthBoundary(object, repeat.width_start);
  const double width_end = ResolveRepeatWidthBoundary(object, repeat.width_end);

  const double adjusted_s = AdjustSCoordinateToLaneSection(road_geometry, road_id, sample_road_s, object.id.string());
  const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_position{std::stoi(road_id.string()), adjusted_s, t};
  const maliput::api::RoadPosition sample_road_position =
      mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_position, true);

  maliput::api::InertialPosition sample_point = sample_road_position.ToInertialPosition();
  sample_point.set_z(sample_point.z() + z_offset);
  return sample_point;
}

std::vector<maliput::api::objects::ContinuousObject> BuildContinuousProperties(
    const xodr::object::Object& object, const xodr::RoadHeader::Id& road_id,
    const maliput::api::RoadGeometry* road_geometry, int samples_per_road) {
  std::vector<maliput::api::objects::ContinuousObject> continuous_properties;
  if (object.repeats.empty()) {
    return continuous_properties;
  }

  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  for (const auto& repeat : object.repeats) {
    // Current scope: only repeats that model one continuous object instance.
    // Discrete object repeats are treated as separate RoadObjects.
    if (repeat.distance != 0.) {
      continue;
    }
    const auto sample_s_coordinates = BuildSampleSCoordinates(repeat, samples_per_road);
    const bool detach_from_reference_line = repeat.detach_from_reference_line.value_or(false);
    std::optional<maliput::api::InertialPosition> detached_start_point;
    std::optional<maliput::api::InertialPosition> detached_end_point;
    if (detach_from_reference_line) {
      detached_start_point = BuildRepeatSamplePoint(object, repeat, road_geometry, road_id, repeat.s, 0., mali_rg);
      detached_end_point =
          BuildRepeatSamplePoint(object, repeat, road_geometry, road_id, repeat.s + repeat.length, 1., mali_rg);
    }
    for (const double sample_road_s : sample_s_coordinates) {
      const double span = repeat.length;
      const double ratio = std::abs(span) < 1e-12 ? 0. : Clamp01((sample_road_s - repeat.s) / span);

      const double width_start = ResolveRepeatWidthBoundary(object, repeat.width_start);
      const double width_end = ResolveRepeatWidthBoundary(object, repeat.width_end);
      const double width = Lerp(width_start, width_end, ratio);
      const double height = Lerp(repeat.height_start, repeat.height_end, ratio);

      std::optional<maliput::api::InertialPosition> sample_point;
      // When detachFromReferenceLine is true, the sample point is interpolated between the repeat endpoints as a
      // straight line.
      if (detach_from_reference_line) {
        MALIDRIVE_VALIDATE(detached_start_point.has_value() && detached_end_point.has_value(), std::logic_error,
                           "Detached repeat endpoints are not initialized.");
        sample_point = maliput::api::InertialPosition{
            Lerp(detached_start_point->x(), detached_end_point->x(), ratio),
            Lerp(detached_start_point->y(), detached_end_point->y(), ratio),
            Lerp(detached_start_point->z(), detached_end_point->z(), ratio),
        };
      } else {
        sample_point = BuildRepeatSamplePoint(object, repeat, road_geometry, road_id, sample_road_s, ratio, mali_rg);
      }
      continuous_properties.emplace_back(width, height, *sample_point);
    }
  }
  return continuous_properties;
}

}  // namespace

RoadObjectBuilder::RoadObjectBuilder(SourceType source_type, const xodr::object::Object& object,
                                     const xodr::RoadHeader::Id& road_id,
                                     const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                                     const maliput::api::RoadGeometry* road_geometry,
                                     std::vector<xodr::DBManager::ObjectReferenceOnRoad> object_references,
                                     int continuous_object_samples_per_road)
    : source_type_(source_type),
      object_(&object),
      road_id_(road_id),
      loader_(loader),
      road_geometry_(road_geometry),
      object_references_(std::move(object_references)),
      continuous_object_samples_per_road_(continuous_object_samples_per_road) {
  MALIDRIVE_VALIDATE(source_type_ == SourceType::kObject, std::invalid_argument,
                     "RoadObjectBuilder object constructor requires SourceType::kObject.");
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
  MALIDRIVE_VALIDATE(continuous_object_samples_per_road_ > 0, std::invalid_argument,
                     "continuous_object_samples_per_road must be positive.");
}

RoadObjectBuilder::RoadObjectBuilder(SourceType source_type, const xodr::signal::Signal& signal,
                                     const xodr::RoadHeader::Id& road_id,
                                     const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                                     const maliput::api::RoadGeometry* road_geometry,
                                     std::vector<xodr::DBManager::SignalReferenceOnRoad> signal_references,
                                     int continuous_object_samples_per_road)
    : source_type_(source_type),
      signal_(&signal),
      road_id_(road_id),
      loader_(loader),
      road_geometry_(road_geometry),
      signal_references_(std::move(signal_references)),
      continuous_object_samples_per_road_(continuous_object_samples_per_road) {
  MALIDRIVE_VALIDATE(source_type_ == SourceType::kSignal, std::invalid_argument,
                     "RoadObjectBuilder signal constructor requires SourceType::kSignal.");
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
  MALIDRIVE_VALIDATE(continuous_object_samples_per_road_ > 0, std::invalid_argument,
                     "continuous_object_samples_per_road must be positive.");
}

std::unique_ptr<maliput::api::objects::RoadObject> RoadObjectBuilder::operator()() const {
  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  switch (source_type_) {
    case SourceType::kObject: {
      MALIDRIVE_VALIDATE(object_ != nullptr, maliput::common::assertion_error,
                         "RoadObjectBuilder object source is not set.");
      const auto& object = *object_;
      const std::string type_str =
          object.type.has_value() ? xodr::object::Object::object_type_to_str(object.type.value()) : "";
      const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
          type_str, object.subtype, std::nullopt, std::nullopt, object.name,
      };
      bool is_movable = false;
      const auto definition_opt = loader_.Lookup(fingerprint, traffic_control_device::OpenDriveElementType::kObject);
      if (definition_opt.has_value() &&
          definition_opt.value().device_type == traffic_control_device::TrafficControlDeviceType::kRoadObject) {
        is_movable = definition_opt.value().is_position_dynamic;
      }

      // --- Position ---
      double adjusted_s = AdjustSCoordinateToLaneSection(road_geometry_, road_id_, object.s, object.id.string());
      const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()),
                                                                                adjusted_s, object.t};
      const maliput::api::RoadPosition rp =
          mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
      maliput::api::InertialPosition inertial_pos = rp.ToInertialPosition();
      inertial_pos.set_z(inertial_pos.z() + object.z_offset);
      const maliput::api::objects::RoadObjectPosition position(inertial_pos, rp.lane->id(), rp.pos);

      // --- Orientation ---
      const bool perp_to_road = object.perp_to_road.value_or(false);
      const maliput::math::RollPitchYaw road_orientation =
          mali_rg->GetRoadOrientationAtOpenScenarioRoadPosition(osc_road_position);
      const double hdg = object.hdg.value_or(0.);
      const double pitch = perp_to_road ? 0. : object.pitch.value_or(0.);
      const double roll = perp_to_road ? 0. : object.roll.value_or(0.);
      const maliput::api::Rotation orientation =
          maliput::api::Rotation::FromRpy(road_orientation.roll_angle() + roll, road_orientation.pitch_angle() + pitch,
                                          road_orientation.yaw_angle() + hdg);

      // --- Bounding box ---
      double bb_length = object.length.value_or(0.);
      double bb_width = object.width.value_or(0.);
      const double bb_height = object.height.value_or(0.);
      if (object.radius.has_value()) {
        bb_length = 2.0 * object.radius.value();
        bb_width = 2.0 * object.radius.value();
      }
      const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., bb_height / 2.),
                                                    maliput::math::Vector3(bb_length, bb_width, bb_height),
                                                    maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

      // --- Type ---
      const maliput::api::objects::RoadObjectType type = MapXodrObjectType(object.type, object.subtype);
      // --- Related lanes ---
      auto related_lanes = ResolveLaneIds(object, adjusted_s, road_id_, object_references_, road_geometry_);
      // --- Outlines ---
      auto outlines = BuildOutlines(object, road_id_, road_geometry_, inertial_pos, orientation);
      // Repeats with distance == 0 are represented as sampled continuous properties.
      auto continuous_properties =
          BuildContinuousProperties(object, road_id_, road_geometry_, continuous_object_samples_per_road_);

      std::unordered_map<std::string, std::string> properties;
      if (!object.materials.empty()) {
        properties["material"] = object.materials[0].surface.value_or("");
        properties["subtype"] = object.subtype.value_or("");
      }

      maliput::log()->debug("RoadObjectBuilder: creating RoadObject id='", object.id.string(),
                            "' type=", static_cast<int>(type), " position=(", inertial_pos.x(), ", ", inertial_pos.y(),
                            ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

      return std::make_unique<MalidriveRoadObject>(
          maliput::api::objects::RoadObject::Id(object.id.string()), type, position, orientation, bounding_box,
          object.dynamic.value_or(false), std::move(related_lanes), object.name, object.subtype, std::move(outlines),
          std::move(properties), std::move(continuous_properties), is_movable);
    }
    case SourceType::kSignal: {
      MALIDRIVE_VALIDATE(signal_ != nullptr, maliput::common::assertion_error,
                         "RoadObjectBuilder signal source is not set.");
      const auto& signal = *signal_;
      const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
          signal.type, NormalizeSubtype(signal.subtype), signal.country, signal.country_revision, signal.name,
      };
      bool is_movable = false;
      std::optional<maliput::api::objects::RoadObjectType> type_from_db;
      const auto definition_opt = loader_.Lookup(fingerprint, traffic_control_device::OpenDriveElementType::kSignal);
      if (definition_opt.has_value() &&
          definition_opt.value().device_type == traffic_control_device::TrafficControlDeviceType::kRoadObject) {
        is_movable = definition_opt.value().is_position_dynamic;
        if (definition_opt.value().device_semantics.has_value()) {
          type_from_db = StringToRoadObjectType(definition_opt.value().device_semantics.value());
        }
      }

      // --- Position ---
      double adjusted_s = AdjustSCoordinateToLaneSection(road_geometry_, road_id_, signal.s, signal.id.string());
      const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()),
                                                                                adjusted_s, signal.t};
      const maliput::api::RoadPosition rp =
          mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
      maliput::api::InertialPosition inertial_pos = rp.ToInertialPosition();
      inertial_pos.set_z(inertial_pos.z() + signal.z_offset);
      const maliput::api::objects::RoadObjectPosition position(inertial_pos, rp.lane->id(), rp.pos);

      // --- Orientation ---
      const maliput::math::RollPitchYaw road_orientation =
          mali_rg->GetRoadOrientationAtOpenScenarioRoadPosition(osc_road_position);
      const double orientation_offset = signal.orientation == xodr::Orientation::kAgainstS ? 0. : M_PI;
      const maliput::api::Rotation orientation = maliput::api::Rotation::FromRpy(
          road_orientation.roll_angle() + signal.roll.value_or(0.),
          road_orientation.pitch_angle() + signal.pitch.value_or(0.),
          road_orientation.yaw_angle() + signal.h_offset.value_or(0.) + orientation_offset);

      // --- Bounding box ---
      const auto default_bounding_box =
          definition_opt.has_value()
              ? definition_opt->default_bounding_box.value_or(traffic_control_device::BoundingBoxDimensions{})
              : traffic_control_device::BoundingBoxDimensions{};
      const double bb_length = signal.length.value_or(default_bounding_box.length);
      const double bb_width = signal.width.value_or(default_bounding_box.width);
      const double bb_height = signal.height.value_or(default_bounding_box.height);
      const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., 0.),
                                                    maliput::math::Vector3(bb_length, bb_width, bb_height),
                                                    maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

      // --- Related lanes ---
      auto related_lanes = ResolveLaneIds(signal, adjusted_s, road_id_, signal_references_, road_geometry_);
      // --- Type ---
      const auto type = type_from_db.value_or(maliput::api::objects::RoadObjectType::kUnknown);

      maliput::log()->debug("RoadObjectBuilder: creating RoadObject id='", signal.id.string(),
                            "' type=", static_cast<int>(type), " position=(", inertial_pos.x(), ", ", inertial_pos.y(),
                            ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

      return std::make_unique<MalidriveRoadObject>(
          maliput::api::objects::RoadObject::Id(signal.id.string()), type, position, orientation, bounding_box,
          signal.dynamic, std::move(related_lanes), signal.name, NormalizeSubtype(signal.subtype),
          std::vector<std::unique_ptr<maliput::api::objects::Outline>>{},
          std::unordered_map<std::string, std::string>{}, std::vector<maliput::api::objects::ContinuousObject>{},
          is_movable);
    }
  }

  MALIDRIVE_THROW_MESSAGE("RoadObjectBuilder received an unsupported source type.", maliput::common::assertion_error);
}

}  // namespace builder
}  // namespace malidrive
