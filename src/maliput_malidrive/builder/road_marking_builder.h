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
#include <vector>

#include <maliput/api/objects/road_marking.h>
#include <maliput/api/road_geometry.h>

#include "maliput_malidrive/traffic_control_device/traffic_control_device_database_loader.h"
#include "maliput_malidrive/xodr/db_manager.h"
#include "maliput_malidrive/xodr/object/object.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/signal/signal.h"

namespace malidrive {
namespace builder {

/// Builds a single @ref maliput::api::objects::RoadMarking from either an XODR
/// @ref xodr::object::Object or an XODR @ref xodr::signal::Signal together
/// with its matching definition in the traffic control device YAML database.
///
/// The builder follows the functor pattern: construct it with all required
/// inputs and call @ref operator()() to produce the result.
///
/// The source type must be selected explicitly at construction time, and only
/// the matching source payload is accepted.
///
/// When no matching @ref traffic_control_device::TrafficControlDeviceDefinition is found in
/// the loader for the selected source fingerprint, or the device_type is not
/// @ref traffic_control_device::TrafficControlDeviceType::kRoadMarking, the builder returns nullptr.
class RoadMarkingBuilder {
 public:
  enum class SourceType {
    kObject,
    kSignal,
  };

  /// Constructs a RoadMarkingBuilder.
  ///
  /// @param source_type The XODR element type this builder will read from.
  /// @param object The XODR object to build a @ref maliput::api::objects::RoadMarking from.
  /// @param road_id The XODR road's ID the @p object belongs to.
  /// @param loader The @ref traffic_control_device::TrafficControlDeviceDatabaseLoader providing access
  ///        to the traffic control device YAML database.
  /// @param road_geometry Pointer to the road geometry. Must not be nullptr.
  /// @param object_references Object references from other roads that point to @p object.
  /// @throws std::invalid_argument if @p source_type is not kObject or @p road_geometry is nullptr.
  RoadMarkingBuilder(SourceType source_type, const xodr::object::Object& object, const xodr::RoadHeader::Id& road_id,
                     const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                     const maliput::api::RoadGeometry* road_geometry,
                     std::vector<xodr::DBManager::ObjectReferenceOnRoad> object_references = {});

  /// Constructs a RoadMarkingBuilder.
  ///
  /// @param source_type The XODR element type this builder will read from.
  /// @param signal The XODR signal to build a @ref maliput::api::objects::RoadMarking from.
  /// @param road_id The XODR road's ID the @p object belongs to.
  /// @param loader The @ref traffic_control_device::TrafficControlDeviceDatabaseLoader providing access
  ///        to the traffic control device YAML database.
  /// @param road_geometry Pointer to the road geometry. Must not be nullptr.
  /// @param signal_references Signal references from other roads that point to @p signal.
  /// @throws std::invalid_argument if @p source_type is not kSignal or @p road_geometry is nullptr.
  RoadMarkingBuilder(SourceType source_type, const xodr::signal::Signal& signal, const xodr::RoadHeader::Id& road_id,
                     const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader,
                     const maliput::api::RoadGeometry* road_geometry,
                     std::vector<xodr::DBManager::SignalReferenceOnRoad> signal_references = {});

  /// Builds and returns the @ref maliput::api::objects::RoadMarking.
  ///
  /// @returns A unique_ptr to the constructed RoadMarking, or nullptr if no
  ///          matching definition is found or the device_type is not kRoadMarking.
  std::unique_ptr<maliput::api::objects::RoadMarking> operator()() const;

 private:
  SourceType source_type_;
  const xodr::object::Object* object_{nullptr};
  const xodr::signal::Signal* signal_{nullptr};
  const xodr::RoadHeader::Id& road_id_;
  const traffic_control_device::TrafficControlDeviceDatabaseLoader& loader_;
  const maliput::api::RoadGeometry* road_geometry_;
  const std::vector<xodr::DBManager::ObjectReferenceOnRoad> object_references_;
  const std::vector<xodr::DBManager::SignalReferenceOnRoad> signal_references_;
};

}  // namespace builder
}  // namespace malidrive
