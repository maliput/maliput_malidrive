// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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

#include <map>
#include <memory>
#include <string>

#include <maliput/api/road_network.h>

#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

class RoadNetworkBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetworkBuilder);

  /// Constructs a RoadNetworkBuilder.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  explicit RoadNetworkBuilder(const std::map<std::string, std::string>& road_network_configuration)
      : road_network_configuration_(road_network_configuration) {}

  /// @return A maliput_malidrive RoadNetwork.
  std::unique_ptr<maliput::api::RoadNetwork> operator()() const;

 private:
  /// Builds the @ref maliput::api::rules::TrafficLightBook.
  ///
  /// An initial @ref maliput::api::rules::TrafficLightBook is constructed from the file specified in
  /// @p rn_config.traffic_light_book. When @p rn_config.traffic_signal_db is set, the book is also populated by
  /// iterating all XODR signals in @p rg and constructing one
  /// @ref maliput::api::rules::TrafficLight per signal that has a matching
  /// entry in the YAML database. Otherwise, the existing file-based
  /// @p rn_config.traffic_light_book path is used (or an empty book when
  /// neither parameter is set).
  ///
  /// @param rn_config Parsed RoadNetwork configuration.
  /// @param rg Pointer to the built RoadGeometry. Must not be nullptr.
  std::unique_ptr<maliput::api::rules::TrafficLightBook> BuildTrafficLightBook(
      const RoadNetworkConfiguration& rn_config, const maliput::api::RoadGeometry* rg) const;

  const std::map<std::string, std::string> road_network_configuration_;
};

}  // namespace builder
}  // namespace malidrive
