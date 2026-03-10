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

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Builds a maliput::api::rules::TrafficLightBook.
///
/// An initial @ref maliput::api::rules::TrafficLightBook is constructed from
/// the file specified in @p traffic_light_book_path. When @p traffic_signal_db_path
/// is set, the book is also populated by iterating all XODR signals in @p road_geometry
/// and constructing one @ref maliput::api::rules::TrafficLight per signal that has a
/// matching entry in the YAML database. Otherwise, the existing file-based
/// @p traffic_light_book_path is used (or an empty book when neither parameter is set).
class TrafficLightBookBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLightBookBuilder)

  /// Constructs a TrafficLightBookBuilder.
  ///
  /// @param road_geometry Pointer to the built RoadGeometry. Must not be nullptr.
  /// @param traffic_light_book_path Path to a YAML file to initialise the TrafficLightBook.
  ///        When `std::nullopt`, an empty book is used as the base.
  /// @param traffic_signal_db_path Path to the YAML database that maps XODR signals to
  ///        traffic light definitions. When `std::nullopt`, no XODR-based traffic lights
  ///        are added.
  /// @throws maliput::common::traffic_light_book_error When @p road_geometry is nullptr.
  TrafficLightBookBuilder(const maliput::api::RoadGeometry* road_geometry,
                          std::optional<std::string> traffic_light_book_path,
                          std::optional<std::string> traffic_signal_db_path);

  TrafficLightBookBuilder() = delete;

  /// Builds and returns a maliput::api::rules::TrafficLightBook.
  std::unique_ptr<maliput::api::rules::TrafficLightBook> operator()() const;

 private:
  const maliput::api::RoadGeometry* road_geometry_;
  const std::optional<std::string> traffic_light_book_path_;
  const std::optional<std::string> traffic_signal_db_path_;
};

}  // namespace builder
}  // namespace malidrive