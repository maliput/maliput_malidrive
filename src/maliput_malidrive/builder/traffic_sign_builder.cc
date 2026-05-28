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
#include "maliput_malidrive/builder/traffic_sign_builder.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <maliput/api/lane_data.h>
#include <maliput/api/rules/traffic_sign.h>
#include <maliput/common/logger.h>
#include <maliput/math/bounding_box.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/builder_tools.h"
#include "maliput_malidrive/builder/traffic_sign_type_mapper.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/traffic_control_device/parser.h"
#include "maliput_malidrive/xodr/signal/orientation.h"

namespace malidrive {
namespace builder {

namespace {

/// Normalizes a XODR signal subtype string to match the fingerprint convention
/// used by @ref traffic_control_device::TrafficControlDeviceParser.
///
/// The parser treats the YAML values "-1" and "none" as std::nullopt (no
/// subtype constraint). We apply the same normalization here so that the
/// fingerprint built from the XODR signal matches the stored fingerprint.
std::optional<std::string> NormalizeSubtype(const std::string& subtype) {
  if (subtype.empty() || subtype == "-1" || subtype == "none") {
    return std::nullopt;
  }
  return subtype;
}

}  // namespace

TrafficSignBuilder::TrafficSignBuilder(const xodr::signal::Signal& signal, const xodr::RoadHeader::Id& road_id,
                                       const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                                       const maliput::api::RoadGeometry* road_geometry,
                                       std::vector<xodr::DBManager::SignalReferenceOnRoad> signal_references)
    : signal_(signal),
      road_id_(road_id),
      loader_(loader),
      road_geometry_(road_geometry),
      signal_references_(std::move(signal_references)) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

std::unique_ptr<const maliput::api::rules::TrafficSign> TrafficSignBuilder::operator()() const {
  const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
      signal_.type,
      NormalizeSubtype(signal_.subtype),
      signal_.country,
      signal_.country_revision,
  };

  const auto definition_opt = loader_.Lookup(fingerprint);
  if (!definition_opt.has_value()) {
    maliput::log()->debug("TrafficSignBuilder: no definition found for signal id='", signal_.id.string(), "' type='",
                          signal_.type, "' subtype='", signal_.subtype, "'. Skipping TrafficSign creation.");
    return nullptr;
  }
  const auto& definition = definition_opt.value();

  // Only build TrafficSign objects for signals whose database device_type is kTrafficSign.
  if (definition.device_type != traffic_control_device::TrafficControlDeviceType::kTrafficSign) {
    maliput::log()->debug("TrafficSignBuilder: signal id='", signal_.id.string(), "' has device_type='",
                          traffic_control_device::TrafficControlDeviceTypeToString(definition.device_type),
                          "', expected 'traffic_sign'. Skipping TrafficSign creation.");
    return nullptr;
  }

  const auto sign_meaning = MapSignTypeString(definition.device_semantics.value_or("other"));
  if (sign_meaning == maliput::api::rules::TrafficSignType::kUnknown) {
    maliput::log()->debug("TrafficSignBuilder: unrecognized device_semantics='",
                          definition.device_semantics.value_or("other"), "' for signal id='", signal_.id.string(),
                          "'. Defaulting to TrafficSignType::kUnknown.");
  }

  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), signal_.s,
                                                                      signal_.t};
  // We allow off-road positions conversions here since traffic signs in XODR files tend to be placed slightly off
  // the road.
  maliput::api::RoadPosition rp = mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
  maliput::api::InertialPosition pos = rp.ToInertialPosition();
  pos.set_z(signal_.z_offset);
  // The traffic sign's orientation is set based on the lane's orientation at the traffic sign's position.
  const double orientation =
      rp.lane->GetOrientation(rp.pos).yaw() + (signal_.h_offset.has_value() ? signal_.h_offset.value() : 0.);
  const maliput::api::Rotation orientation_road_network = maliput::api::Rotation::FromRpy(
      0., 0., orientation + (signal_.orientation != xodr::signal::Orientation::kAgainstS ? M_PI : 0.));

  auto related_lanes =
      ResolveAndDeduplicateLaneIds(road_id_, signal_.s, signal_.validities, signal_references_, road_geometry_);

  // Build a bounding box from the signal's physical dimensions when available.
  // The bounding box position and orientation are expressed in the sign's local frame.
  const double depth = signal_.length.value_or(0.);
  const double width = signal_.width.value_or(0.);
  const double height = signal_.height.value_or(0.);
  const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., 0.),
                                                maliput::math::Vector3(depth, width, height),
                                                maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

  maliput::log()->debug("TrafficSignBuilder: creating TrafficSign for signal id='", signal_.id.string(), "' type='",
                        signal_.type, "' subtype='", signal_.subtype, "' device_semantics='",
                        definition.device_semantics.value_or("other"), "'. TrafficSign position: (x=", pos.x(),
                        ", y=", pos.y(), ", z=", pos.z(),
                        ") with orientation (roll=0, pitch=0, yaw=", orientation_road_network.yaw(),
                        "), related_lanes=", related_lanes.size(), ".");

  return std::make_unique<maliput::api::rules::TrafficSign>(maliput::api::rules::TrafficSign::Id(signal_.id.string()),
                                                            sign_meaning, pos, orientation_road_network, signal_.text,
                                                            std::move(related_lanes), bounding_box);
}

}  // namespace builder
}  // namespace malidrive
