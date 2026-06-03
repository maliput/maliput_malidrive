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
#include <optional>
#include <string>

#include <maliput/api/objects/road_marking_book.h>
#include <maliput/api/objects/road_object_book.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/api/rules/traffic_sign_book.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Holds the four books produced by @ref TrafficControlDeviceBooksBuilder.
struct TrafficControlDeviceBooks {
  std::unique_ptr<maliput::api::rules::TrafficLightBook> traffic_light_book;
  std::unique_ptr<maliput::api::rules::TrafficSignBook> traffic_sign_book;
  std::unique_ptr<maliput::api::objects::RoadObjectBook> road_object_book;
  std::unique_ptr<maliput::api::objects::RoadMarkingBook> road_marking_book;
};

/// Builds all four traffic control device books in a single pass over the XODR
/// signals and objects, using a shared traffic control device YAML database.
///
/// This unified builder ensures the database is loaded exactly once and each
/// XODR element is routed to the appropriate book based on its `device_type`:
///
///   - XODR **signals**:
///     - `"traffic_light"` → TrafficLightBook
///     - `"traffic_sign"`  → TrafficSignBook
///     - Other/no match    → skipped
///
///   - XODR **objects**:
///     - `"road_marking"`  → RoadMarkingBook
///     - `"road_object"`   → RoadObjectBook
///     - No match          → skipped
class TrafficControlDeviceBooksBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficControlDeviceBooksBuilder)

  /// Constructs a TrafficControlDeviceBooksBuilder.
  ///
  /// @param road_geometry Pointer to the built RoadGeometry. Must not be nullptr.
  /// @param traffic_light_book_path Optional path to a YAML file used to seed
  ///        the TrafficLightBook before processing XODR signals.
  /// @param traffic_control_device_db_path Optional path to the YAML database that maps
  ///        XODR signals/objects to traffic control device definitions.
  ///        When `std::nullopt`, all books will be empty (or seeded only from
  ///        @p traffic_light_book_path for the TrafficLightBook).
  /// @param allow_non_driveable_lanes If false, any XODR road without driveable lanes will
  ///        be skipped when building books.
  /// @throws maliput::common::assertion_error When @p road_geometry is nullptr.
  TrafficControlDeviceBooksBuilder(const maliput::api::RoadGeometry* road_geometry,
                                   std::optional<std::string> traffic_light_book_path,
                                   std::optional<std::string> traffic_control_device_db_path,
                                   bool allow_non_driveable_lanes);

  TrafficControlDeviceBooksBuilder() = delete;

  /// Builds and returns all four books.
  ///
  /// @returns A @ref TrafficControlDeviceBooks holding the populated books.
  TrafficControlDeviceBooks operator()() const;

 private:
  const maliput::api::RoadGeometry* road_geometry_;
  const std::optional<std::string> traffic_light_book_path_;
  const std::optional<std::string> traffic_control_device_db_path_;
  const bool allow_non_driveable_lanes_;
};

}  // namespace builder
}  // namespace malidrive
