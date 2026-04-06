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
#include "maliput_malidrive/builder/traffic_signal_books_builder.h"

#include <utility>

#include <maliput/base/traffic_light_book.h>
#include <maliput/base/traffic_light_book_loader.h>
#include <maliput/base/traffic_sign_book.h>
#include <maliput/common/logger.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/traffic_light_builder.h"
#include "maliput_malidrive/builder/traffic_sign_builder.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/traffic_signal/parser.h"
#include "maliput_malidrive/traffic_signal/traffic_signal_database_loader.h"

namespace malidrive {
namespace builder {

TrafficSignalBooksBuilder::TrafficSignalBooksBuilder(const maliput::api::RoadGeometry* road_geometry,
                                                     std::optional<std::string> traffic_light_book_path,
                                                     std::optional<std::string> traffic_signal_db_path)
    : road_geometry_(road_geometry),
      traffic_light_book_path_(std::move(traffic_light_book_path)),
      traffic_signal_db_path_(std::move(traffic_signal_db_path)) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, maliput::common::assertion_error, "road_geometry must not be nullptr.");
}

TrafficSignalBooks TrafficSignalBooksBuilder::operator()() const {
  maliput::log()->trace("Building TrafficLights and TrafficSigns from XODR signals and YAML database...");

  // Parse the YAML database exactly once.
  const traffic_signal::TrafficSignalDatabaseLoader loader(traffic_signal_db_path_);

  // Seed TrafficLightBook from file or start with an empty one.
  auto tlb = traffic_light_book_path_.has_value()
                 ? maliput::LoadTrafficLightBookFromFile(traffic_light_book_path_.value())
                 : std::make_unique<maliput::TrafficLightBook>();

  auto tsb = std::make_unique<maliput::TrafficSignBook>();

  // Obtain the DBManager via the concrete malidrive::RoadGeometry.
  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  // Pre-build the signal-reference index once.
  const auto signal_refs_by_id = mali_rg->get_manager()->GetSignalReferencesBySignalId();

  // Single pass over all XODR signals.
  for (const auto& [road_id, road_header] : mali_rg->get_manager()->GetRoadHeaders()) {
    if (!road_header.signals.has_value()) {
      continue;
    }
    for (const auto& signal : road_header.signals->signals) {
      const traffic_signal::TrafficSignalFingerprint fingerprint{
          signal.type,
          // Normalize "-1" / "none" / "" → std::nullopt to match the DB key.
          [&]() -> std::optional<std::string> {
            if (signal.subtype.empty() || signal.subtype == "-1" || signal.subtype == "none") {
              return std::nullopt;
            }
            return signal.subtype;
          }(),
          signal.country,
          signal.country_revision,
      };

      const auto definition_opt = loader.Lookup(fingerprint);
      if (!definition_opt.has_value()) {
        maliput::log()->debug("TrafficSignalBooksBuilder: no definition found for signal id='", signal.id.string(),
                              "' type='", signal.type, "' subtype='", signal.subtype, "'. Skipping.");
        continue;
      }

      const auto& definition = definition_opt.value();

      // Fetch cross-road signal references for all matched signals (both traffic
      // lights and traffic signs may be referenced from multiple roads).
      std::vector<xodr::DBManager::SignalReferenceOnRoad> refs;
      const auto refs_it = signal_refs_by_id.find(signal.id.string());
      if (refs_it != signal_refs_by_id.end()) {
        refs = refs_it->second;
      }

      if (definition.sign_type == TrafficLightBuilder::kTrafficLightSignType) {
        auto tl = TrafficLightBuilder(signal, road_id, loader, road_geometry_, refs)();
        if (tl) {
          auto* tlb_impl = dynamic_cast<maliput::TrafficLightBook*>(tlb.get());
          tlb_impl->AddTrafficLight(std::move(tl));
        }
      } else {
        // Static traffic sign.
        auto ts = TrafficSignBuilder(signal, road_id, loader, road_geometry_, std::move(refs))();
        if (ts) {
          tsb->AddTrafficSign(std::move(ts));
        }
      }
    }
  }

  maliput::log()->trace("Built TrafficLights and TrafficSigns from XODR signals.");

  return TrafficSignalBooks{std::move(tlb), std::move(tsb)};
}

}  // namespace builder
}  // namespace malidrive
