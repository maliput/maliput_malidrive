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
#include "maliput_malidrive/builder/road_marking_builder.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <maliput/api/lane_data.h>
#include <maliput/api/objects/road_marking.h>
#include <maliput/api/objects/road_object.h>
#include <maliput/common/logger.h>
#include <maliput/math/bounding_box.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/road_marking_type_mapper.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/traffic_control_device/parser.h"

namespace {

std::optional<std::string> NormalizeSubtype(const std::string& subtype) {
  if (subtype.empty() || subtype == "-1" || subtype == "none") {
    return std::nullopt;
  }
  return subtype;
}

}  // namespace

namespace malidrive {
namespace builder {

RoadMarkingBuilder::RoadMarkingBuilder(SourceType source_type, const xodr::signal::Signal& signal,
                                       const xodr::RoadHeader::Id& road_id,
                                       const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                                       const maliput::api::RoadGeometry* road_geometry,
                                       std::vector<xodr::DBManager::SignalReferenceOnRoad> signal_references)
    : source_type_(source_type),
      signal_(&signal),
      road_id_(road_id),
      loader_(loader),
      road_geometry_(road_geometry),
      signal_references_(std::move(signal_references)) {
  MALIDRIVE_VALIDATE(source_type_ == SourceType::kSignal, std::invalid_argument,
                     "RoadMarkingBuilder signal constructor requires SourceType::kSignal.");
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

RoadMarkingBuilder::RoadMarkingBuilder(SourceType source_type, const xodr::object::Object& object,
                                       const xodr::RoadHeader::Id& road_id,
                                       const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                                       const maliput::api::RoadGeometry* road_geometry,
                                       std::vector<xodr::DBManager::ObjectReferenceOnRoad> object_references)
    : source_type_(source_type),
      object_(&object),
      road_id_(road_id),
      loader_(loader),
      road_geometry_(road_geometry),
      object_references_(std::move(object_references)) {
  MALIDRIVE_VALIDATE(source_type_ == SourceType::kObject, std::invalid_argument,
                     "RoadMarkingBuilder object constructor requires SourceType::kObject.");
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

std::unique_ptr<maliput::api::objects::RoadMarking> RoadMarkingBuilder::operator()() const {
  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  switch (source_type_) {
    case SourceType::kObject: {
      MALIDRIVE_VALIDATE(object_ != nullptr, maliput::common::assertion_error,
                         "RoadMarkingBuilder object source is not set.");
      const auto& object = *object_;
      const std::string type_str =
          object.type.has_value() ? xodr::object::Object::object_type_to_str(object.type.value()) : "";
      const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
          type_str, object.subtype, std::nullopt, std::nullopt, object.name,
      };

      const auto definition_opt = loader_.Lookup(fingerprint, traffic_control_device::OpenDriveElementType::kObject);
      if (!definition_opt.has_value()) {
        maliput::log()->debug("RoadMarkingBuilder: no definition found for object id='", object.id.string(), "' type='",
                              type_str, "' subtype='", object.subtype.value_or(""), "' name='",
                              object.name.value_or(""), "'. Skipping RoadMarking creation.");
        return nullptr;
      }
      const auto& definition = definition_opt.value();
      if (definition.device_type != traffic_control_device::TrafficControlDeviceType::kRoadMarking) {
        maliput::log()->debug("RoadMarkingBuilder: object id='", object.id.string(), "' has device_type='",
                              traffic_control_device::TrafficControlDeviceTypeToString(definition.device_type),
                              "', expected 'RoadMarking'. Skipping RoadMarking creation.");
        return nullptr;
      }

      const auto marking_type = MapRoadMarkingTypeString(definition.device_semantics.value_or("Other"));
      if (marking_type == maliput::api::objects::RoadMarkingType::kUnknown) {
        maliput::log()->debug("RoadMarkingBuilder: unrecognized device_semantics='",
                              definition.device_semantics.value_or("Other"), "' for object id='", object.id.string(),
                              "'. Defaulting to RoadMarkingType::kUnknown.");
      }

      double adjusted_s = AdjustSCoordinateToLaneSection(road_geometry_, road_id_, object.s, object.id.string());
      const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), adjusted_s,
                                                                                object.t};
      const maliput::api::RoadPosition rp =
          mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
      maliput::api::InertialPosition inertial_pos = rp.ToInertialPosition();
      inertial_pos.set_z(inertial_pos.z() + object.z_offset);

      const maliput::api::objects::RoadObjectPosition position(inertial_pos, rp.lane->id(), rp.pos);
      const bool perp_to_road = object.perp_to_road.value_or(false);
      const maliput::math::RollPitchYaw road_orientation =
          mali_rg->GetRoadOrientationAtOpenScenarioRoadPosition(osc_road_position);
      const double hdg = object.hdg.value_or(0.);
      const double pitch = perp_to_road ? 0. : object.pitch.value_or(0.);
      const double roll = perp_to_road ? 0. : object.roll.value_or(0.);
      const double orientation_offset =
          (object.orientation.has_value() && object.orientation.value() == xodr::Orientation::kAgainstS) ? M_PI : 0.;
      const maliput::api::Rotation orientation =
          maliput::api::Rotation::FromRpy(road_orientation.roll_angle() + roll, road_orientation.pitch_angle() + pitch,
                                          road_orientation.yaw_angle() + hdg + orientation_offset);

      const auto default_bounding_box =
          definition.default_bounding_box.value_or(traffic_control_device::BoundingBoxDimensions{});
      double bb_length = object.length.value_or(default_bounding_box.length);
      double bb_width = object.width.value_or(default_bounding_box.width);
      const double bb_height = object.height.value_or(default_bounding_box.height);
      if (object.radius.has_value()) {
        bb_length = 2.0 * object.radius.value();
        bb_width = 2.0 * object.radius.value();
      }
      const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., bb_height / 2.),
                                                    maliput::math::Vector3(bb_length, bb_width, bb_height),
                                                    maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

      auto related_lanes = ResolveLaneIds(object, adjusted_s, road_id_, object_references_, road_geometry_);
      auto outlines = BuildOutlines(object, road_id_, road_geometry_, inertial_pos, orientation);

      maliput::log()->debug("RoadMarkingBuilder: creating RoadMarking id='", object.id.string(),
                            "' type=", static_cast<int>(marking_type), " position=(", inertial_pos.x(), ", ",
                            inertial_pos.y(), ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

      return std::make_unique<maliput::api::objects::RoadMarking>(
          maliput::api::objects::RoadMarking::Id(object.id.string()), marking_type, position, orientation, bounding_box,
          std::move(related_lanes), object.name, std::move(outlines));
    }
    case SourceType::kSignal: {
      MALIDRIVE_VALIDATE(signal_ != nullptr, maliput::common::assertion_error,
                         "RoadMarkingBuilder signal source is not set.");
      const auto& signal = *signal_;
      const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
          signal.type, NormalizeSubtype(signal.subtype), signal.country, signal.country_revision, signal.name,
      };

      const auto definition_opt = loader_.Lookup(fingerprint, traffic_control_device::OpenDriveElementType::kSignal);
      if (!definition_opt.has_value()) {
        maliput::log()->debug("RoadMarkingBuilder: no definition found for signal id='", signal.id.string(), "' type='",
                              signal.type, "' subtype='", signal.subtype, "' name='", signal.name.value_or(""),
                              "'. Skipping RoadMarking creation.");
        return nullptr;
      }
      const auto& definition = definition_opt.value();
      if (definition.device_type != traffic_control_device::TrafficControlDeviceType::kRoadMarking) {
        maliput::log()->debug("RoadMarkingBuilder: signal id='", signal.id.string(), "' has device_type='",
                              traffic_control_device::TrafficControlDeviceTypeToString(definition.device_type),
                              "', expected 'RoadMarking'. Skipping RoadMarking creation.");
        return nullptr;
      }

      const auto marking_type = MapRoadMarkingTypeString(definition.device_semantics.value_or("Other"));
      if (marking_type == maliput::api::objects::RoadMarkingType::kUnknown) {
        maliput::log()->debug("RoadMarkingBuilder: unrecognized device_semantics='",
                              definition.device_semantics.value_or("Other"), "' for signal id='", signal.id.string(),
                              "'. Defaulting to RoadMarkingType::kUnknown.");
      }

      double adjusted_s = AdjustSCoordinateToLaneSection(road_geometry_, road_id_, signal.s, signal.id.string());
      const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), adjusted_s,
                                                                                signal.t};
      const maliput::api::RoadPosition rp =
          mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
      maliput::api::InertialPosition inertial_pos = rp.ToInertialPosition();
      inertial_pos.set_z(inertial_pos.z() + signal.z_offset);

      const maliput::api::objects::RoadObjectPosition position(inertial_pos, rp.lane->id(), rp.pos);
      const maliput::math::RollPitchYaw road_orientation =
          mali_rg->GetRoadOrientationAtOpenScenarioRoadPosition(osc_road_position);
      const double orientation_offset = signal.orientation == xodr::Orientation::kAgainstS ? 0. : M_PI;
      const maliput::api::Rotation orientation = maliput::api::Rotation::FromRpy(
          road_orientation.roll_angle() + signal.roll.value_or(0.),
          road_orientation.pitch_angle() + signal.pitch.value_or(0.),
          road_orientation.yaw_angle() + signal.h_offset.value_or(0.) + orientation_offset);

      const auto default_bounding_box =
          definition.default_bounding_box.value_or(traffic_control_device::BoundingBoxDimensions{});
      const double bb_length = signal.length.value_or(default_bounding_box.length);
      const double bb_width = signal.width.value_or(default_bounding_box.width);
      const double bb_height = signal.height.value_or(default_bounding_box.height);
      const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., 0.),
                                                    maliput::math::Vector3(bb_length, bb_width, bb_height),
                                                    maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

      auto related_lanes = ResolveLaneIds(signal, adjusted_s, road_id_, signal_references_, road_geometry_);

      maliput::log()->debug("RoadMarkingBuilder: creating RoadMarking id='", signal.id.string(),
                            "' type=", static_cast<int>(marking_type), " position=(", inertial_pos.x(), ", ",
                            inertial_pos.y(), ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

      return std::make_unique<maliput::api::objects::RoadMarking>(
          maliput::api::objects::RoadMarking::Id(signal.id.string()), marking_type, position, orientation, bounding_box,
          std::move(related_lanes), signal.name, std::vector<std::unique_ptr<maliput::api::objects::Outline>>{});
    }
  }

  MALIDRIVE_THROW_MESSAGE("RoadMarkingBuilder received an unsupported source type.", maliput::common::assertion_error);
}

}  // namespace builder
}  // namespace malidrive
