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
#include "maliput_malidrive/traffic_control_device/traffic_control_device_database_loader.h"

#include <filesystem>
#include <string_view>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace traffic_control_device {

namespace {

bool ends_with(std::string_view str, std::string_view suffix) {
  const auto str_size = str.size();
  const auto suffix_size = suffix.size();
  if (suffix_size > str_size) {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
}

}  // namespace

TrafficControlDeviceDatabaseLoader::TrafficControlDeviceDatabaseLoader(const std::optional<std::string>& database) {
  if (database.has_value()) {
    // Checks if value is a path to a file or the content of the YAML database.
    // If the value ends with ".yaml" or ".yml", it is considered a path. Otherwise, it is considered content.
    if (ends_with(database.value(), ".yaml") || ends_with(database.value(), ".yml")) {
      definitions_ = TrafficControlDeviceParser::LoadFromFile(database.value());
    } else {
      definitions_ = TrafficControlDeviceParser::LoadFromString(database.value());
    }
  }
}

std::optional<TrafficControlDeviceDefinition> TrafficControlDeviceDatabaseLoader::Lookup(
    const TrafficControlDeviceFingerprint& fingerprint) const {
  const TrafficControlDeviceDefinition* best = nullptr;
  int best_specificity = -1;

  for (const auto& def : definitions_) {
    if (TrafficControlDeviceParser::Matches(def.fingerprint, fingerprint)) {
      const int specificity = TrafficControlDeviceParser::Specificity(def.fingerprint);
      if (specificity > best_specificity) {
        best_specificity = specificity;
        best = &def;
      } else if (specificity == best_specificity) {
        // Two matches with equal specificity should have been caught at load time.
        MALIDRIVE_THROW_MESSAGE(
            "Multiple traffic control device database entries with equal specificity matched the "
            "same query; this indicates a database conflict that was not caught at load time.",
            maliput::common::road_network_description_parser_error);
      }
    }
  }

  if (best == nullptr) {
    return std::nullopt;
  }
  return *best;
}

}  // namespace traffic_control_device
}  // namespace malidrive
