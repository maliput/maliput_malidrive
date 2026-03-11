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
#pragma once

#include <memory>

#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/traffic_lights.h>

#include "maliput_malidrive/traffic_signal/traffic_signal_database_loader.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/signal/signal.h"

namespace malidrive {
namespace builder {

/// Builds a single @ref maliput::api::rules::TrafficLight from a XODR
/// @ref xodr::signal::Signal and its matching definition in the traffic
/// signal YAML database.
///
/// The builder follows the functor pattern: construct it with all required
/// inputs and call @ref operator()() to produce the result.
///
/// When no matching @ref traffic_signal::TrafficSignalDefinition is found in
/// the loader for the given signal's type/subtype/country fingerprint, the
/// builder returns nullptr. Callers are responsible for checking the return
/// value.
///
/// @note TrafficLight position is currently set to the origin as a placeholder.
///       Computation of the true inertial position from the signal's (s, t,
///       z_offset) road-frame coordinates is left for a future step.
class TrafficLightBuilder {
 public:
  /// Constructs a TrafficLightBuilder.
  ///
  /// @param signal The XODR signal to build a @ref maliput::api::rules::TrafficLight from.
  /// @param road_id The XODR road's ID the @p signal belongs to.
  /// @param loader The @ref traffic_signal::TrafficSignalDatabaseLoader providing access
  ///        to the traffic signal YAML database.
  /// @param road_geometry Pointer to the road geometry. Must not be nullptr.
  /// @throws std::invalid_argument if @p road_geometry is nullptr.
  TrafficLightBuilder(const xodr::signal::Signal& signal, const xodr::RoadHeader::Id& road_id,
                      const traffic_signal::TrafficSignalDatabaseLoader& loader,
                      const maliput::api::RoadGeometry* road_geometry);

  /// Builds and returns the @ref maliput::api::rules::TrafficLight.
  ///
  /// @returns A unique_ptr to the constructed TrafficLight, or nullptr if no
  ///          matching definition was found in the YAML database for the signal.
  std::unique_ptr<const maliput::api::rules::TrafficLight> operator()() const;

 private:
  const xodr::signal::Signal& signal_;
  const xodr::RoadHeader::Id& road_id_;
  const traffic_signal::TrafficSignalDatabaseLoader& loader_;
  const maliput::api::RoadGeometry* road_geometry_;
};

}  // namespace builder
}  // namespace malidrive
