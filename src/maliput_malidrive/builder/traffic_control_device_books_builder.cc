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
#include "maliput_malidrive/builder/traffic_control_device_books_builder.h"

#include <utility>

#include <maliput/base/road_marking_book.h>
#include <maliput/base/road_object_book.h>
#include <maliput/base/traffic_light_book.h>
#include <maliput/base/traffic_light_book_loader.h>
#include <maliput/base/traffic_sign_book.h>
#include <maliput/common/logger.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/road_marking_builder.h"
#include "maliput_malidrive/builder/road_object_builder.h"
#include "maliput_malidrive/builder/traffic_light_builder.h"
#include "maliput_malidrive/builder/traffic_sign_builder.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/traffic_control_device/parser.h"
#include "maliput_malidrive/traffic_control_device/traffic_control_device_database_loader.h"
#include "maliput_malidrive/xodr/object/object.h"

namespace malidrive {
namespace builder {

TrafficControlDeviceBooksBuilder::TrafficControlDeviceBooksBuilder(
    const maliput::api::RoadGeometry* road_geometry, std::optional<std::string> traffic_light_book_path,
    std::optional<std::string> traffic_control_device_db_path)
    : road_geometry_(road_geometry),
      traffic_light_book_path_(std::move(traffic_light_book_path)),
      traffic_control_device_db_path_(std::move(traffic_control_device_db_path)) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, maliput::common::assertion_error, "road_geometry must not be nullptr.");
}

TrafficControlDeviceBooks TrafficControlDeviceBooksBuilder::operator()() const {
  maliput::log()->trace("Building all TrafficControlDevice books from XODR signals/objects and YAML database...");

  // Parse the YAML database exactly once.
  const traffic_control_device::TrafficControlDeviceDatabaseLoader loader(traffic_control_device_db_path_);

  // Seed TrafficLightBook from file or start with an empty one.
  auto tlb = traffic_light_book_path_.has_value()
                 ? maliput::LoadTrafficLightBookFromFile(traffic_light_book_path_.value())
                 : std::make_unique<maliput::TrafficLightBook>();

  auto tsb = std::make_unique<maliput::TrafficSignBook>();
  auto rob = std::make_unique<maliput::RoadObjectBook>();
  auto rmb = std::make_unique<maliput::RoadMarkingBook>();

  // Obtain the DBManager via the concrete malidrive::RoadGeometry.
  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  // Pre-build the signal-reference index once.
  const auto signal_refs_by_id = mali_rg->get_manager()->GetSignalReferencesBySignalId();

  // --- Signals pass: route to TrafficLightBook or TrafficSignBook ---
  for (const auto& [road_id, road_header] : mali_rg->get_manager()->GetRoadHeaders()) {
    if (!road_header.signals.has_value()) {
      continue;
    }
    for (const auto& signal : road_header.signals->signals) {
      const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
          signal.type,
          [&]() -> std::optional<std::string> {
            if (signal.subtype.empty() || signal.subtype == "-1" || signal.subtype == "none") {
              return std::nullopt;
            }
            return signal.subtype;
          }(),
          signal.country,
          signal.country_revision,
          signal.name,
      };

      const auto definition_opt = loader.Lookup(fingerprint);
      if (!definition_opt.has_value()) {
        maliput::log()->debug("TrafficControlDeviceBooksBuilder: no definition found for signal id='",
                              signal.id.string(), "' type='", signal.type, "' subtype='", signal.subtype,
                              "'. Skipping.");
        continue;
      }

      const auto& definition = definition_opt.value();

      // Fetch cross-road signal references.
      std::vector<xodr::DBManager::SignalReferenceOnRoad> refs;
      const auto refs_it = signal_refs_by_id.find(signal.id.string());
      if (refs_it != signal_refs_by_id.end()) {
        refs = refs_it->second;
      }

      if (definition.device_type == traffic_control_device::TrafficControlDeviceType::kTrafficLight) {
        auto tl = TrafficLightBuilder(signal, road_id, loader, road_geometry_, std::move(refs))();
        if (tl) {
          auto* tlb_impl = dynamic_cast<maliput::TrafficLightBook*>(tlb.get());
          tlb_impl->AddTrafficLight(std::move(tl));
        }
      } else if (definition.device_type == traffic_control_device::TrafficControlDeviceType::kTrafficSign) {
        auto ts = TrafficSignBuilder(signal, road_id, loader, road_geometry_, std::move(refs))();
        if (ts) {
          tsb->AddTrafficSign(std::move(ts));
        }
      } else {
        maliput::log()->debug("TrafficControlDeviceBooksBuilder: signal id='", signal.id.string(),
                              "' has device_type='",
                              traffic_control_device::TrafficControlDeviceTypeToString(definition.device_type),
                              "' which is not valid for signals. Skipping.");
      }
    }
  }

  // --- Objects pass: route to RoadMarkingBook or RoadObjectBook ---
  for (const auto& [road_id, road_header] : mali_rg->get_manager()->GetRoadHeaders()) {
    if (!road_header.objects.has_value()) {
      continue;
    }
    for (const auto& object : road_header.objects->objects) {
      const std::string type_str =
          object.type.has_value() ? xodr::object::Object::object_type_to_str(object.type.value()) : "";

      const traffic_control_device::TrafficControlDeviceFingerprint fingerprint{
          type_str,     object.subtype,
          std::nullopt,  // country (not used for objects)
          std::nullopt,  // country_revision (not used for objects)
          object.name,
      };

      const auto definition_opt = loader.Lookup(fingerprint);
      if (!definition_opt.has_value()) {
        maliput::log()->debug("TrafficControlDeviceBooksBuilder: no definition found for object id='",
                              object.id.string(), "' type='", type_str, "' subtype='", object.subtype.value_or(""),
                              "' name='", object.name.value_or(""), "'. Defaulting in RoadObject creation.");
        try {
          auto ro = RoadObjectBuilder(object, road_id, road_geometry_)();
          if (ro) {
            rob->AddRoadObject(std::move(ro));
          }
        } catch (const std::exception& e) {
          maliput::log()->warn("TrafficControlDeviceBooksBuilder: failed to build default RoadObject id='",
                               object.id.string(), "' on road '", road_id.string(), "': ", e.what(), ". Skipping.");
        }
        continue;
      }

      const auto& definition = definition_opt.value();

      if (definition.device_type == traffic_control_device::TrafficControlDeviceType::kRoadMarking) {
        try {
          auto rm = RoadMarkingBuilder(object, road_id, loader, road_geometry_)();
          if (rm) {
            rmb->AddRoadMarking(std::move(rm));
          }
        } catch (const std::exception& e) {
          maliput::log()->warn("TrafficControlDeviceBooksBuilder: failed to build RoadMarking from object id='",
                               object.id.string(), "' on road '", road_id.string(), "': ", e.what(), ". Skipping.");
        }
      } else if (definition.device_type == traffic_control_device::TrafficControlDeviceType::kRoadObject) {
        try {
          auto ro = RoadObjectBuilder(object, road_id, road_geometry_)();
          if (ro) {
            rob->AddRoadObject(std::move(ro));
          }
        } catch (const std::exception& e) {
          maliput::log()->warn("TrafficControlDeviceBooksBuilder: failed to build RoadObject id='", object.id.string(),
                               "' on road '", road_id.string(), "': ", e.what(), ". Skipping.");
        }
      } else {
        maliput::log()->debug("TrafficControlDeviceBooksBuilder: object id='", object.id.string(),
                              "' has device_type='",
                              traffic_control_device::TrafficControlDeviceTypeToString(definition.device_type),
                              "' which is not valid for objects. Skipping.");
      }
    }
  }

  maliput::log()->trace("Built all TrafficControlDevice books.");

  return TrafficControlDeviceBooks{std::move(tlb), std::move(tsb), std::move(rob), std::move(rmb)};
}

}  // namespace builder
}  // namespace malidrive
