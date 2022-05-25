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
#include <memory>
#include <unordered_map>
#include <vector>

#include <maliput/api/road_geometry.h>
#include <maliput/base/rule_registry.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Functor to build a RuleRegistry.
class RuleRegistryBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleRegistryBuilder)

  /// Constructs a maliput::api::rules::RuleRegistry.
  ///
  /// @param rg A malidrive::RoadGeometry. It is used to extract speed limit
  /// rule ranges. It must not be nullptr.
  /// @param rule_registry_file_path YAML file path for loading the maliput::api::rules::RuleRegistry.
  RuleRegistryBuilder(const maliput::api::RoadGeometry* rg, const std::optional<std::string>& rule_registry_file_path);

  /// Builds a maliput::api::rules::RuleRegistry.
  ///
  /// The maliput::api::rules::RuleRegistry is built in two steps:
  /// 1- Loads maliput::api::rules::RuleRegistry with rule types obtained from the YAML file if it is provided.
  ///
  /// 2- Programatically loads the following list of rule types:
  ///    - maliput::api::rules::DiscreteValueRule types:
  ///      - @see malidrive::builder::rules::VehicleExclusiveRuleTypeId().
  ///      - @see malidrive::builder::rules::VehicleUsageRuleTypeId().
  ///      - @see maliput::DirectionUsageRuleTypeId().
  ///    - maliput::api::rules::RangeValueRule types:
  ///      - @see maliput::SpeedLimitRuleTypeId().
  std::unique_ptr<maliput::api::rules::RuleRegistry> operator()();

 private:
  // The following two methods are temporary and should be removed once
  // DiscreteValueRule types are loaded via a YAML loader.
  //@{

  // Adds maliput::api::rules::DiscreteValueRule types to `rule_registry`.
  //
  // @see VehicleRelatedRuleTypesAndValues() and @see maliput::BuildDirectionUsageRuleType()  for
  // maliput::api::rules::DiscreteValueRule types and discrete values.
  // All maliput::api::rules::DiscreteValueRule::DiscreteValues are marked with
  // maliput::api::rules::Rule::State::kStrict severity.
  // Direction usage rule type are included to the rule registry.
  //
  // @throws maliput::common::assertion_error When `rule_registry` is nullptr.
  void AddDiscreteValueRuleTypes(maliput::api::rules::RuleRegistry* rule_registry) const;

  // @returns An unordered map of DiscreteValueRule::TypeId and possible rule states.
  std::unordered_map<maliput::api::rules::DiscreteValueRule::TypeId, std::vector<std::string>> RuleTypesAndValues()
      const;

  //@}

  // Constructs a unique maliput::api::rules::RangeValueRule type to
  // describe all possible speed limit RangeValeRule::Ranges and adds them to
  // `rule_registry`.
  //
  // Traverses all lanes in the RoadGeometry and queries for the lane speed
  // limit at the beginning of the lane. Note that multiple speed limits
  // definitions could be found within an XODR lane, however,
  // this function is limited to sample only to the beginning of the Lane.(TODO(#553))
  //
  // All built ranges will have a minimum speed constants::kDefaultMaxSpeedLimit.
  // When no maximum speed is set, constants::kDefaultMaxSpeedLimit is used.
  //
  // @throws maliput::common::assertion_error When `rule_registry` is nullptr.
  void AddSpeedLimitRuleType(maliput::api::rules::RuleRegistry* rule_registry) const;

  const maliput::api::RoadGeometry* rg_{};
  const std::optional<std::string> rule_registry_file_path_{};
};

}  // namespace builder
}  // namespace malidrive
