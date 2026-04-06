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
#include <maliput/api/rules/traffic_sign.h>

#include "maliput_malidrive/traffic_signal/traffic_signal_database_loader.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/signal/signal.h"

namespace malidrive {
namespace builder {

/// Builds a single @ref maliput::api::rules::TrafficSign from a XODR
/// @ref xodr::signal::Signal and its matching definition in the traffic
/// signal YAML database.
///
/// The builder follows the functor pattern: construct it with all required
/// inputs and call @ref operator()() to produce the result.
///
/// When no matching @ref traffic_signal::TrafficSignalDefinition is found in
/// the loader for the given signal's type/subtype/country fingerprint, or when
/// the definition's sign_type cannot be mapped to a known
/// @ref maliput::api::rules::TrafficSignType, the builder returns nullptr.
class TrafficSignBuilder {
 public:
  /// Constructs a TrafficSignBuilder.
  ///
  /// @param signal The XODR signal to build a @ref maliput::api::rules::TrafficSign from.
  /// @param road_id The XODR road's ID the @p signal belongs to.
  /// @param loader The @ref traffic_signal::TrafficSignalDatabaseLoader providing access
  ///        to the traffic signal YAML database.
  /// @param road_geometry Pointer to the road geometry. Must not be nullptr.
  /// @param signal_references Signal references from other roads that point to @p signal.
  ///        Each entry carries the referencing road's ID and the reference itself, whose
  ///        validity/s-coordinate is used to resolve related lanes on the referencing road.
  /// @throws std::invalid_argument if @p road_geometry is nullptr.
  TrafficSignBuilder(const xodr::signal::Signal& signal, const xodr::RoadHeader::Id& road_id,
                     const traffic_signal::TrafficSignalDatabaseLoader& loader,
                     const maliput::api::RoadGeometry* road_geometry,
                     std::vector<xodr::DBManager::SignalReferenceOnRoad> signal_references = {});

  /// Builds and returns the @ref maliput::api::rules::TrafficSign.
  ///
  /// @returns A unique_ptr to the constructed TrafficSign.
  std::unique_ptr<const maliput::api::rules::TrafficSign> operator()() const;

 private:
  const xodr::signal::Signal& signal_;
  const xodr::RoadHeader::Id& road_id_;
  const traffic_signal::TrafficSignalDatabaseLoader& loader_;
  const maliput::api::RoadGeometry* road_geometry_;
  const std::vector<xodr::DBManager::SignalReferenceOnRoad> signal_references_;
};

}  // namespace builder
}  // namespace malidrive
