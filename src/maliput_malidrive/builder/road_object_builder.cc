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
                      bool is_dynamic, std::vector<maliput::api::LaneId> related_lanes,
                      std::optional<std::string> name, std::optional<std::string> subtype,
                      std::vector<std::unique_ptr<maliput::api::objects::Outline>> outlines,
                      std::unordered_map<std::string, std::string> properties)
      : RoadObject(id, type, position, orientation, bounding_box, is_dynamic, std::move(related_lanes),
                   std::move(name), std::move(subtype), std::move(outlines), std::move(properties)) {}
};

/// Converts XODR outline corners to maliput OutlineCorners and Outlines.
std::vector<std::unique_ptr<maliput::api::objects::Outline>> BuildOutlines(
    const xodr::object::Object& object, const xodr::RoadHeader::Id& road_id,
    const malidrive::RoadGeometry* mali_rg, const maliput::api::InertialPosition& object_inertial_pos,
    const maliput::api::Rotation& object_orientation) {
  std::vector<std::unique_ptr<maliput::api::objects::Outline>> result;
  if (!object.outlines.has_value()) {
    return result;
  }

  int outline_index = 0;
  for (const auto& xodr_outline : object.outlines->outlines) {
    std::vector<maliput::api::objects::OutlineCorner> corners;

    if (!xodr_outline.corner_road.empty()) {
      for (const auto& cr : xodr_outline.corner_road) {
        const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_pos{std::stoi(road_id.string()), cr.s, cr.t};
        const maliput::api::RoadPosition rp =
            mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_pos, true);
        maliput::api::InertialPosition ip = rp.ToInertialPosition();
        ip.set_z(ip.z() + cr.dz);
        corners.emplace_back(maliput::math::Vector3(ip.x(), ip.y(), ip.z()), cr.height);
      }
    } else if (!xodr_outline.corner_local.empty()) {
      // Transform local (u, v, z) to inertial using the object's position and orientation.
      const double cos_yaw = std::cos(object_orientation.yaw());
      const double sin_yaw = std::sin(object_orientation.yaw());
      for (const auto& cl : xodr_outline.corner_local) {
        const double x = object_inertial_pos.x() + cos_yaw * cl.u - sin_yaw * cl.v;
        const double y = object_inertial_pos.y() + sin_yaw * cl.u + cos_yaw * cl.v;
        const double z = object_inertial_pos.z() + cl.z;
        corners.emplace_back(maliput::math::Vector3(x, y, z), cl.height);
      }
    }

    if (corners.size() >= 3u) {
      const std::string outline_id_str =
          xodr_outline.id.has_value() ? xodr_outline.id->string()
                                      : (object.id.string() + "_outline_" + std::to_string(outline_index));
      const bool closed = xodr_outline.closed.value_or(true);
      result.push_back(std::make_unique<maliput::api::objects::Outline>(
          maliput::api::objects::Outline::Id(outline_id_str), std::move(corners), closed));
    }
    ++outline_index;
  }
  return result;
}

}  // namespace

RoadObjectBuilder::RoadObjectBuilder(const xodr::object::Object& object, const xodr::RoadHeader::Id& road_id,
                                     const maliput::api::RoadGeometry* road_geometry)
    : object_(object), road_id_(road_id), road_geometry_(road_geometry) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

std::unique_ptr<maliput::api::objects::RoadObject> RoadObjectBuilder::operator()() const {
  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  // --- Position ---
  const malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), object_.s,
                                                                            object_.t};
  const maliput::api::RoadPosition rp =
      mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
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
      maliput::api::Rotation::FromRpy(road_orientation.roll_angle() + roll,
                                      road_orientation.pitch_angle() + pitch,
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
  auto related_lanes = ResolveLaneIds(road_id_, object_.s, object_.validities, road_geometry_);

  // --- Outlines ---
  auto outlines = BuildOutlines(object_, road_id_, mali_rg, inertial_pos, orientation);

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
      std::move(related_lanes), object_.name, object_.subtype, std::move(outlines), std::move(properties));
}

}  // namespace builder
}  // namespace malidrive
