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
#include "maliput_malidrive/builder/traffic_light_builder.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <maliput/api/lane_data.h>
#include <maliput/common/logger.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/signal/orientation.h"
#include "maliput_malidrive/traffic_signal/parser.h"

namespace malidrive {
namespace builder {

namespace {

/// Normalizes a XODR signal subtype string to match the fingerprint convention
/// used by @ref traffic_signal::TrafficSignalParser.
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

TrafficLightBuilder::TrafficLightBuilder(const xodr::signal::Signal& signal, const xodr::RoadHeader::Id& road_id,
                                         const traffic_signal::TrafficSignalDatabaseLoader& loader,
                                         const maliput::api::RoadGeometry* road_geometry)
    : signal_(signal), road_id_(road_id), loader_(loader), road_geometry_(road_geometry) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, std::invalid_argument, "road_geometry must not be nullptr.");
}

std::unique_ptr<const maliput::api::rules::TrafficLight> TrafficLightBuilder::operator()() const {
  const traffic_signal::TrafficSignalFingerprint fingerprint{
      signal_.type,
      NormalizeSubtype(signal_.subtype),
      signal_.country,
      signal_.country_revision,
  };

  const auto definition_opt = loader_.Lookup(fingerprint);
  if (!definition_opt.has_value()) {
    maliput::log()->debug("TrafficLightBuilder: no definition found for signal id='", signal_.id.string(), "' type='",
                          signal_.type, "' subtype='", signal_.subtype, "'. Skipping TrafficLight creation.");
    return nullptr;
  }
  const auto& definition = definition_opt.value();

  // Build Bulbs from the definition's bulb list.
  // Bulb positions and orientations are expressed relative to the traffic light
  // frame. Because we create a single BulbGroup at the traffic light origin, the
  // bulb-relative-to-bulb-group offsets equal the bulb-relative-to-traffic-light
  // offsets from the YAML.
  std::vector<std::unique_ptr<maliput::api::rules::Bulb>> bulbs;
  bulbs.reserve(definition.bulbs.size());
  for (const auto& bulb_def : definition.bulbs) {
    const std::optional<std::vector<maliput::api::rules::BulbState>> states_opt =
        bulb_def.states.empty() ? std::nullopt : std::make_optional(bulb_def.states);

    bulbs.push_back(std::make_unique<maliput::api::rules::Bulb>(
        maliput::api::rules::Bulb::Id(bulb_def.id),
        maliput::api::InertialPosition::FromXyz(bulb_def.position_traffic_light),
        maliput::api::Rotation::FromQuat(bulb_def.orientation_traffic_light), bulb_def.color, bulb_def.type,
        bulb_def.arrow_orientation_rad, states_opt, bulb_def.bounding_box));
  }

  maliput::api::rules::TrafficLight::Id traffic_light_id =
      maliput::api::rules::TrafficLight::Id("TrafficLight_" + signal_.id.string());
  // Wrap all bulbs in a single BulbGroup placed at the traffic light origin.
  std::vector<std::unique_ptr<maliput::api::rules::BulbGroup>> bulb_groups;
  bulb_groups.push_back(std::make_unique<maliput::api::rules::BulbGroup>(
      maliput::api::rules::BulbGroup::Id(traffic_light_id.string() + "_Bulbs"),
      maliput::api::InertialPosition(0., 0., 0.), maliput::api::Rotation(), std::move(bulbs)));

  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  malidrive::RoadGeometry::OpenScenarioRoadPosition osc_road_position{std::stoi(road_id_.string()), signal_.s,
                                                                      signal_.t};
  // We allow off-road positions conversions here since traffic lights in XODR files tend to be placed slightly off the
  // road.
  maliput::api::RoadPosition rp = mali_rg->OpenScenarioRoadPositionToMaliputRoadPosition(osc_road_position, true);
  maliput::api::InertialPosition pos = rp.ToInertialPosition();
  pos.set_z(signal_.z_offset);
  // The traffic light's orientation is set based on the lane's orientation at the traffic light's position.
  double orientation = rp.lane->GetOrientation(rp.pos).yaw() + (signal_.h_offset.has_value() ? signal_.h_offset.value() : 0.);
  maliput::api::Rotation orientation_road_network = maliput::api::Rotation::FromRpy(0., 0., orientation + (signal_.orientation == xodr::signal::Orientation::kAgainstS ? M_PI : 0.));

  maliput::log()->debug("TrafficLightBuilder: creating TrafficLight for signal id='", signal_.id.string(), "' type='",
                        signal_.type, "' subtype='", signal_.subtype,
                        "'. TrafficLight position: (x=", pos.x(), ", y=", pos.y(), ", z=", pos.z(), ") with orientation (roll=0, pitch=0, yaw=", orientation_road_network.yaw(), ").");

  return std::make_unique<maliput::api::rules::TrafficLight>(maliput::api::rules::TrafficLight::Id(signal_.id.string()),
                                                             pos, orientation_road_network,
                                                             std::move(bulb_groups));
}

}  // namespace builder
}  // namespace malidrive
