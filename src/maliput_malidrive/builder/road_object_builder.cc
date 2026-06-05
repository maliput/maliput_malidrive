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

}  // namespace

RoadObjectBuilder::RoadObjectBuilder(const xodr::object::Object& object, const xodr::RoadHeader::Id& road_id,
                                     const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                                     const maliput::api::RoadGeometry* road_geometry)
    : object_(object), road_id_(road_id), loader_(loader), road_geometry_(road_geometry) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

std::unique_ptr<maliput::api::objects::RoadObject> RoadObjectBuilder::operator()() const {
  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  // Look up the object definition to retrieve is_movable from the database.
  const std::string type_str =
      object_.type.has_value() ? xodr::object::Object::object_type_to_str(object_.type.value()) : "";
  const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
      type_str,     object_.subtype,
      std::nullopt,  // country (not used for objects)
      std::nullopt,  // country_revision (not used for objects)
      object_.name,
  };

  bool is_movable = false;  // Default to false if definition not found or device_type is not kRoadObject.
  const auto definition_opt = loader_.Lookup(fingerprint);
  if (definition_opt.has_value() &&
      definition_opt.value().device_type == traffic_control_device::TrafficControlDeviceType::kRoadObject) {
    is_movable = definition_opt.value().is_position_dynamic;
  }

  // --- Position ---
  double object_s = object_.s;
  if (is_almost_equal(object_.s, mali_rg->GetRoadCurve(road_id_)->p1())) {
    std::ostringstream s_str, adjusted_s_str;
    s_str << std::fixed << std::setprecision(10) << object_.s;
    adjusted_s_str << std::fixed << std::setprecision(10) << (object_.s - kEpsilon);
    maliput::log()->warn("RoadObjectBuilder: Object ", object_.id.string(), " has s coordinate ", s_str.str(),
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

  // --- Bounding box ---
  double bb_length = object_.length.value_or(0.);
  double bb_width = object_.width.value_or(0.);
  const double bb_height = object_.height.value_or(0.);
  if (object_.radius.has_value()) {
    bb_length = 2.0 * object_.radius.value();
    bb_width = 2.0 * object_.radius.value();
  }
  const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., 0.),
                                                maliput::math::Vector3(bb_length, bb_width, bb_height),
                                                maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

  // --- Type ---
  const maliput::api::objects::RoadObjectType type = MapXodrObjectType(object_.type);

  // --- Related lanes ---
  auto related_lanes = ResolveLaneIds(road_id_, object_s, object_.validities, road_geometry_);

  // --- Outlines ---
  auto outlines = BuildOutlines(object_, road_id_, road_geometry_, inertial_pos, orientation);

  // --- Properties ---
  std::unordered_map<std::string, std::string> properties;
  if (!object_.materials.empty()) {
    properties["material"] = object_.materials[0].surface.value_or("");
    properties["subtype"] = object_.subtype.value_or("");
  }

  // --- is_dynamic ---
  const bool is_dynamic = object_.dynamic.value_or(false);

  maliput::log()->debug("RoadObjectBuilder: creating RoadObject id='", object_.id.string(),
                        "' type=", static_cast<int>(type), " position=(", inertial_pos.x(), ", ", inertial_pos.y(),
                        ", ", inertial_pos.z(), ") related_lanes=", related_lanes.size(), ".");

  return std::make_unique<MalidriveRoadObject>(
      maliput::api::objects::RoadObject::Id(object_.id.string()), type, position, orientation, bounding_box, is_dynamic,
      std::move(related_lanes), object_.name, object_.subtype, std::move(outlines), std::move(properties), is_movable);
}

}  // namespace builder
}  // namespace malidrive
