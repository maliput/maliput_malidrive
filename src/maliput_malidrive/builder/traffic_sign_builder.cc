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
#include <unordered_map>
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

/// Maps a string to a TrafficSignValueUnit enum value.
/// @param unit_string The string representation of the unit.
/// @return The corresponding TrafficSignValueUnit enum value or std::nullopt if the string does not map to any Unit.
std::optional<maliput::api::rules::TrafficSignValueUnit> StringToValueUnit(const std::string& unit_string) {
  const auto mapper = maliput::api::rules::TrafficSignValueUnitMapper();
  for (const auto& pair : mapper) {
    if (pair.second == unit_string) {
      return std::make_optional(pair.first);
    }
  }
  return std::nullopt;
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

  const auto definition_opt = loader_.Lookup(fingerprint, traffic_control_device::OpenDriveElementType::kSignal);
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
                          "', expected 'TrafficSign'. Skipping TrafficSign creation.");
    return nullptr;
  }

  const auto sign_meaning = MapSignTypeString(definition.device_semantics.value_or("Unknown"));
  if (sign_meaning == maliput::api::rules::TrafficSignType::kUnknown) {
    maliput::log()->debug("TrafficSignBuilder: unrecognized device_semantics='",
                          definition.device_semantics.value_or("Unknown"), "' for signal id='", signal_.id.string(),
                          "'. Defaulting to TrafficSignType::kUnknown.");
  }

  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), signal_.s,
                                                                      signal_.t};
  // We allow off-road positions conversions here since traffic signs in XODR files tend to be placed slightly off
  // the road.
  maliput::api::RoadPosition rp = mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
  maliput::api::InertialPosition pos = rp.ToInertialPosition();
  pos.set_z(pos.z() + signal_.z_offset);
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
  const maliput::math::BoundingBox bounding_box{maliput::math::Vector3(0., 0., height / 2.),
                                                maliput::math::Vector3(depth, width, height),
                                                maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};

  // Map the XODR signal value/unit pair to a TrafficSignValue, if present.
  // A value of -1.0 is the XODR sentinel for "no value" and is skipped.
  std::optional<maliput::api::rules::TrafficSignValue> traffic_sign_value;
  if (signal_.value.has_value() && signal_.value->value != -1.0) {
    const auto mapped_unit = StringToValueUnit(signal_.value->unit);
    if (!mapped_unit.has_value()) {
      MALIDRIVE_THROW_MESSAGE("TrafficSignBuilder: signal id='" + signal_.id.string() +
                                  "' has value attribute with unrecognized unit '" + signal_.value->unit + "'.",
                              maliput::common::road_network_description_parser_error);
    }
    traffic_sign_value = maliput::api::rules::TrafficSignValue{signal_.value->value, mapped_unit.value()};
  }

  // Expose signal fingerprint metadata as backend-specific properties.
  std::unordered_map<std::string, std::string> properties{{"type", signal_.type}};
  const auto normalized_subtype = NormalizeSubtype(signal_.subtype);
  if (normalized_subtype.has_value()) {
    properties.emplace("subtype", normalized_subtype.value());
  }
  if (signal_.country.has_value()) {
    properties.emplace("country", signal_.country.value());
  }
  if (signal_.country_revision.has_value()) {
    properties.emplace("country_revision", signal_.country_revision.value());
  }
  if (signal_.name.has_value()) {
    properties.emplace("name", signal_.name.value());
  }

  // TrafficSign dynamic semantics are sourced from the XODR signal's `dynamic`.
  const bool is_dynamic = signal_.dynamic;
  // TrafficSign positional movability is sourced from DB `is_position_dynamic`.
  const bool is_movable = definition.is_position_dynamic;

  // Converts dependent XODR signal IDs to TrafficSign::Id for the TrafficSign constructor.
  std::vector<maliput::api::rules::TrafficSign::Id> dependent_signs;
  for (const auto& dependent_sign : signal_.dependencies) {
    dependent_signs.emplace_back(dependent_sign.signal_id.string());
  }

  maliput::log()->debug("TrafficSignBuilder: creating TrafficSign for signal id='", signal_.id.string(), "' type='",
                        signal_.type, "' subtype='", signal_.subtype, "' device_semantics='",
                        definition.device_semantics.value_or("Unknown"), "'. TrafficSign position: (x=", pos.x(),
                        ", y=", pos.y(), ", z=", pos.z(),
                        ") with orientation (roll=0, pitch=0, yaw=", orientation_road_network.yaw(),
                        "), related_lanes=", related_lanes.size(), ".");

  return std::make_unique<maliput::api::rules::TrafficSign>(
      maliput::api::rules::TrafficSign::Id(signal_.id.string()), sign_meaning, pos, orientation_road_network,
      signal_.text, std::move(related_lanes), std::move(dependent_signs), bounding_box, traffic_sign_value,
      std::move(properties), is_dynamic, is_movable);
}

}  // namespace builder
}  // namespace malidrive
