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

static constexpr double kEpsilon{1e-10};

namespace malidrive {
namespace builder {

RoadMarkingBuilder::RoadMarkingBuilder(const xodr::object::Object& object, const xodr::RoadHeader::Id& road_id,
                                       const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                                       const maliput::api::RoadGeometry* road_geometry)
    : object_(object), road_id_(road_id), loader_(loader), road_geometry_(road_geometry) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

std::unique_ptr<maliput::api::objects::RoadMarking> RoadMarkingBuilder::operator()() const {
  // Build the fingerprint from the XODR object attributes.
  const std::string type_str =
      object_.type.has_value() ? xodr::object::Object::object_type_to_str(object_.type.value()) : "";
  const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
      type_str,     object_.subtype,
      std::nullopt,  // country (not used for objects)
      std::nullopt,  // country_revision (not used for objects)
      object_.name,
  };

  const auto definition_opt = loader_.Lookup(fingerprint);
  if (!definition_opt.has_value()) {
    maliput::log()->debug("RoadMarkingBuilder: no definition found for object id='", object_.id.string(), "' type='",
                          type_str, "' subtype='", object_.subtype.value_or(""), "' name='", object_.name.value_or(""),
                          "'. Skipping RoadMarking creation.");
    return nullptr;
  }
  const auto& definition = definition_opt.value();

  // Only build RoadMarking objects for objects whose database device_type is kRoadMarking.
  if (definition.device_type != traffic_control_device::TrafficControlDeviceType::kRoadMarking) {
    maliput::log()->debug("RoadMarkingBuilder: object id='", object_.id.string(), "' has device_type='",
                          traffic_control_device::TrafficControlDeviceTypeToString(definition.device_type),
                          "', expected 'road_marking'. Skipping RoadMarking creation.");
    return nullptr;
  }

  const auto marking_type = MapRoadMarkingTypeString(definition.device_semantics.value_or("other"));
  if (marking_type == maliput::api::objects::RoadMarkingType::kUnknown) {
    maliput::log()->debug("RoadMarkingBuilder: unrecognized device_semantics='",
                          definition.device_semantics.value_or("other"), "' for object id='", object_.id.string(),
                          "'. Defaulting to RoadMarkingType::kUnknown.");
  }

  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  // --- Position ---
  double object_s = object_.s;
  if (std::abs(object_.s - mali_rg->GetRoadCurve(road_id_)->p1()) < kEpsilon) {
    std::ostringstream s_str, adjusted_s_str;
    s_str << std::fixed << std::setprecision(10) << object_.s;
    adjusted_s_str << std::fixed << std::setprecision(10) << (object_.s - kEpsilon);
    maliput::log()->warn("RoadMarkingBuilder: Object ", object_.id.string(), " has s coordinate ", s_str.str(),
                         " equal to the road length. Adjusting s to ", adjusted_s_str.str(),
                         " to avoid potential issues with lane association and orientation.");
    object_s -= kEpsilon;
  }
  const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), object_s,
                                                                            object_.t};
  const maliput::api::RoadPosition rp = mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
  maliput::api::InertialPosition inertial_pos = rp.ToInertialPosition();
  inertial_pos.set_z(object_.z_offset);

  const maliput::api::objects::RoadObjectPosition position(inertial_pos, rp.lane->id(), rp.pos);

  // --- Orientation ---
  const bool perp_to_road = object_.perp_to_road.value_or(false);
  const maliput::math::RollPitchYaw road_orientation =
      mali_rg->GetRoadOrientationAtOpenScenarioRoadPosition(osc_road_position);
  const double hdg = object_.hdg.value_or(0.);
  const double pitch = perp_to_road ? 0. : object_.pitch.value_or(0.);
  const double roll = perp_to_road ? 0. : object_.roll.value_or(0.);
  const double orientation_offset =
      (object_.orientation.has_value() && object_.orientation.value() == xodr::object::Orientation::kNegative) ? M_PI
                                                                                                               : 0.;
  const maliput::api::Rotation orientation =
      maliput::api::Rotation::FromRpy(road_orientation.roll_angle() + roll, road_orientation.pitch_angle() + pitch,
                                      road_orientation.yaw_angle() + hdg + orientation_offset);

  malidrive::traffic_control_device::BoundingBoxDimensions bounding_box_dimensions{};
  // --- Bounding box ---
  double bb_length = object_.length.value_or(definition.default_bounding_box.value_or(bounding_box_dimensions).length);
  double bb_width = object_.width.value_or(definition.default_bounding_box.value_or(bounding_box_dimensions).width);
  const double bb_height =
      object_.height.value_or(definition.default_bounding_box.value_or(bounding_box_dimensions).height);
  if (object_.radius.has_value()) {
    bb_length = 2.0 * object_.radius.value();
    bb_width = 2.0 * object_.radius.value();
  }
  const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., 0.),
                                                maliput::math::Vector3(bb_length, bb_width, bb_height),
                                                maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

  // --- Related lanes ---
  auto related_lanes = ResolveLaneIds(road_id_, object_s, object_.validities, road_geometry_);

  // --- Outlines ---
  auto outlines = BuildOutlines(object_, road_id_, road_geometry_, inertial_pos, orientation);

  maliput::log()->debug("RoadMarkingBuilder: creating RoadMarking id='", object_.id.string(),
                        "' type=", static_cast<int>(marking_type), " position=(", inertial_pos.x(), ", ",
                        inertial_pos.y(), ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

  return std::make_unique<maliput::api::objects::RoadMarking>(
      maliput::api::objects::RoadMarking::Id(object_.id.string()), marking_type, position, orientation, bounding_box,
      std::move(related_lanes), object_.name, std::move(outlines));
}

}  // namespace builder
}  // namespace malidrive
