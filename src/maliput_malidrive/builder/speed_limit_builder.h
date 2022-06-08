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

#include <vector>

#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/speed_limit_rule.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/id_providers.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Functor to build a vector of SpeedLimitRules.
/// TODO(agalbachicar)   Remove when maliput::api::rules::SpeedLimitRules
///                      are deprecated.
class SpeedLimitBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(SpeedLimitBuilder)
  SpeedLimitBuilder() = delete;

  /// Constructs a SpeedLimitBuilder.
  ///
  /// @param rg is the RoadGeometry pointer. It must not be nullptr.
  SpeedLimitBuilder(const maliput::api::RoadGeometry* rg) : rg_(rg) { MALIDRIVE_THROW_UNLESS(rg_ != nullptr); }

  /// Builds a vector of SpeedLimitRules for each Lane in rg.
  std::vector<maliput::api::rules::SpeedLimitRule> operator()();

 private:
  // Builds a maliput::api::rules::SpeedLimitRule for each Lane within `segment`.
  //
  // When there is no maximum speed limit for the lane, the default defined in
  // Constants is used.
  //
  // All the SpeedLimitRules are built with Severity::kStrict as there is no
  // information about it in the map.
  //
  // @pre `segment` must not be nullptr.
  std::vector<maliput::api::rules::SpeedLimitRule> BuildSpeedLimitFor(const maliput::api::Segment* segment);

  const maliput::api::RoadGeometry* rg_{};
};

}  // namespace builder
}  // namespace malidrive
