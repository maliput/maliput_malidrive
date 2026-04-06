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

#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/api/rules/traffic_sign_book.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Holds the pair of books produced by @ref TrafficSignalBooksBuilder.
struct TrafficSignalBooks {
  std::unique_ptr<maliput::api::rules::TrafficLightBook> traffic_light_book;
  std::unique_ptr<maliput::api::rules::TrafficSignBook> traffic_sign_book;
};

/// Builds a @ref maliput::api::rules::TrafficLightBook and a
/// @ref maliput::api::rules::TrafficSignBook in a single pass over the XODR
/// signals and the traffic signal YAML database.
///
/// Compared to the alternative design (separate `TrafficLightBookBuilder` /
/// `TrafficSignBookBuilder`), this builder:
///  - Parses the YAML database exactly once.
///  - Routes each XODR signal to the right book using the `sign_type` field of
///    the matching @ref traffic_signal::TrafficSignalDefinition:
///      - `"traffic_light"` → @ref maliput::api::rules::TrafficLight
///        added to the TrafficLightBook.
///      - any other value → @ref maliput::api::rules::TrafficSign added to the
///        TrafficSignBook.
///  - Fetches cross-road signal references for all matched signals (both
///    traffic lights and traffic signs may be referenced across roads).
class TrafficSignalBooksBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficSignalBooksBuilder)

  /// Constructs a TrafficSignalBooksBuilder.
  ///
  /// @param road_geometry Pointer to the built RoadGeometry. Must not be nullptr.
  /// @param traffic_light_book_path Optional path to a YAML file used to seed
  ///        the TrafficLightBook before processing XODR signals.
  ///        When `std::nullopt`, an empty book is used as the base.
  /// @param traffic_signal_db_path Optional path to the YAML database that maps
  ///        XODR signals to traffic signal definitions.
  ///        When `std::nullopt`, both books will be empty (or seeded only from
  ///        @p traffic_light_book_path for the TrafficLightBook).
  /// @throws maliput::common::assertion_error When @p road_geometry is nullptr.
  TrafficSignalBooksBuilder(const maliput::api::RoadGeometry* road_geometry,
                            std::optional<std::string> traffic_light_book_path,
                            std::optional<std::string> traffic_signal_db_path);

  TrafficSignalBooksBuilder() = delete;

  /// Builds and returns both books.
  ///
  /// @returns A @ref TrafficSignalBooks holding the populated TrafficLightBook
  ///          and TrafficSignBook.
  TrafficSignalBooks operator()() const;

 private:
  const maliput::api::RoadGeometry* road_geometry_;
  const std::optional<std::string> traffic_light_book_path_;
  const std::optional<std::string> traffic_signal_db_path_;
};

}  // namespace builder
}  // namespace malidrive
