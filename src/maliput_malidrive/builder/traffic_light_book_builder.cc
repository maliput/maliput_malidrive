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
#include "maliput_malidrive/builder/traffic_light_book_builder.h"

#include <string>
#include <vector>

#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/base/traffic_light_book.h>
#include <maliput/base/traffic_light_book_loader.h>
#include <maliput/common/error.h>
#include <maliput/common/logger.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/traffic_light_builder.h"
#include "maliput_malidrive/traffic_signal/traffic_signal_database_loader.h"

namespace malidrive {
namespace builder {

TrafficLightBookBuilder::TrafficLightBookBuilder(const maliput::api::RoadGeometry* road_geometry,
                                                 std::optional<std::string> traffic_light_book_path,
                                                 std::optional<std::string> traffic_signal_db_path)
    : road_geometry_(road_geometry),
      traffic_light_book_path_(std::move(traffic_light_book_path)),
      traffic_signal_db_path_(std::move(traffic_signal_db_path)) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, maliput::common::traffic_light_book_error,
                     "RoadGeometry is null when building TrafficLightBook.");
}

std::unique_ptr<maliput::api::rules::TrafficLightBook> TrafficLightBookBuilder::operator()() const {
  auto traffic_light_book = !traffic_light_book_path_.has_value()
                                ? std::make_unique<maliput::TrafficLightBook>()
                                : maliput::LoadTrafficLightBookFromFile(traffic_light_book_path_.value());
  if (!traffic_signal_db_path_.has_value()) {
    // Fall back to the file-based book or an empty book.
    return traffic_light_book;
  }

  // Obtain the DBManager via the concrete malidrive::RoadGeometry.
  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, std::runtime_error, "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  const traffic_signal::TrafficSignalDatabaseLoader loader(traffic_signal_db_path_);

  const auto signal_refs_by_id = mali_rg->get_manager()->GetSignalReferencesBySignalId();

  maliput::log()->trace("Building TrafficLights from XODR signals and YAML database...");
  for (const auto& [road_id, road_header] : mali_rg->get_manager()->GetRoadHeaders()) {
    if (!road_header.signals.has_value()) {
      continue;
    }
    for (const auto& signal : road_header.signals->signals) {
      // Look up any signal references that point to this signal.
      std::vector<xodr::DBManager::SignalReferenceOnRoad> refs;
      const auto refs_it = signal_refs_by_id.find(signal.id.string());
      if (refs_it != signal_refs_by_id.end()) {
        refs = refs_it->second;
      }
      auto tl = builder::TrafficLightBuilder(signal, road_id, loader, road_geometry_, std::move(refs))();
      if (tl) {
        auto* tlb = dynamic_cast<maliput::TrafficLightBook*>(traffic_light_book.get());
        tlb->AddTrafficLight(std::move(tl));
      }
    }
  }
  maliput::log()->trace("Built TrafficLights from XODR signals.");

  return traffic_light_book;
}

}  // namespace builder
}  // namespace malidrive
