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

#include <cmath>
#include <iomanip>
#include <sstream>
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

static constexpr double kEpsilon{1e-10};

namespace {

bool is_almost_equal(double a, double b) { return std::abs(a - b) < kEpsilon; }

}  // namespace
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
                      std::unordered_map<std::string, std::string> properties, bool is_movable)
      : RoadObject(id, type, position, orientation, bounding_box, is_dynamic, std::move(related_lanes), std::move(name),
                   std::move(subtype), std::move(outlines), std::move(properties), is_movable) {}
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

}  // namespace

RoadObjectBuilder::RoadObjectBuilder(SourceType source_type, const xodr::object::Object& object,
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
                     "RoadObjectBuilder object constructor requires SourceType::kObject.");
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

RoadObjectBuilder::RoadObjectBuilder(SourceType source_type, const xodr::signal::Signal& signal,
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
                     "RoadObjectBuilder signal constructor requires SourceType::kSignal.");
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
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
      double object_s = object.s;
      if (is_almost_equal(object.s, mali_rg->GetRoadCurve(road_id_)->p1())) {
        std::ostringstream s_str, adjusted_s_str;
        s_str << std::fixed << std::setprecision(10) << object.s;
        adjusted_s_str << std::fixed << std::setprecision(10) << (object.s - kEpsilon);
        maliput::log()->warn("RoadObjectBuilder: Object ", object.id.string(), " has s coordinate ", s_str.str(),
                             " equal to the road length. Adjusting s to ", adjusted_s_str.str(),
                             " to avoid potential issues with lane association and orientation.");
        object_s -= kEpsilon;
      }

      const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), object_s,
                                                                                object.t};
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
      auto related_lanes = ResolveLaneIds(object, road_id_, object_references_, road_geometry_);
      // --- Outlines ---
      auto outlines = BuildOutlines(object, road_id_, road_geometry_, inertial_pos, orientation);

      std::unordered_map<std::string, std::string> properties;
      if (!object.materials.empty()) {
        properties["material"] = object.materials[0].surface.value_or("");
        properties["subtype"] = object.subtype.value_or("");
      }

      maliput::log()->debug("RoadObjectBuilder: creating RoadObject id='", object.id.string(),
                            "' type=", static_cast<int>(type), " position=(", inertial_pos.x(), ", ", inertial_pos.y(),
                            ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

      return std::make_unique<MalidriveRoadObject>(maliput::api::objects::RoadObject::Id(object.id.string()), type,
                                                   position, orientation, bounding_box, object.dynamic.value_or(false),
                                                   std::move(related_lanes), object.name, object.subtype,
                                                   std::move(outlines), std::move(properties), is_movable);
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
      double signal_s = signal.s;
      if (is_almost_equal(signal.s, mali_rg->GetRoadCurve(road_id_)->p1())) {
        std::ostringstream s_str, adjusted_s_str;
        s_str << std::fixed << std::setprecision(10) << signal.s;
        adjusted_s_str << std::fixed << std::setprecision(10) << (signal.s - kEpsilon);
        maliput::log()->warn("RoadObjectBuilder: Signal ", signal.id.string(), " has s coordinate ", s_str.str(),
                             " equal to the road length. Adjusting s to ", adjusted_s_str.str(),
                             " to avoid potential issues with lane association and orientation.");
        signal_s -= kEpsilon;
      }

      const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), signal_s,
                                                                                signal.t};
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
      auto related_lanes = ResolveLaneIds(signal, road_id_, signal_references_, road_geometry_);
      // --- Type ---
      const auto type = type_from_db.value_or(maliput::api::objects::RoadObjectType::kUnknown);

      maliput::log()->debug("RoadObjectBuilder: creating RoadObject id='", signal.id.string(),
                            "' type=", static_cast<int>(type), " position=(", inertial_pos.x(), ", ", inertial_pos.y(),
                            ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

      return std::make_unique<MalidriveRoadObject>(
          maliput::api::objects::RoadObject::Id(signal.id.string()), type, position, orientation, bounding_box,
          signal.dynamic, std::move(related_lanes), signal.name, NormalizeSubtype(signal.subtype),
          std::vector<std::unique_ptr<maliput::api::objects::Outline>>{},
          std::unordered_map<std::string, std::string>{}, is_movable);
    }
  }

  MALIDRIVE_THROW_MESSAGE("RoadObjectBuilder received an unsupported source type.", maliput::common::assertion_error);
}

}  // namespace builder
}  // namespace malidrive
