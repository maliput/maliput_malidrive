// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include <optional>
#include <string>
#include <vector>

#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/rule_registry.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Functor to build a RoadRulebook.
///
/// The rules API is composed of:
///   - maliput::api::rules::Rules depending on the type of value they hold:
///     - maliput::api::rules::DiscreteValueRule
///     - maliput::api::rules::RangeValueRule
///   - maliput::api::rules::RuleRegistry: it is used to register all the rule type that are allowed.
///
/// The building procedure is based on top a loader method provided by maliput: [RoadRulebook loader with rule
/// registry](https://github.com/ToyotaResearchInstitute/maliput/blob/ab4bff490702c31abd2d0d9ff87383f6f00c45c8/maliput/src/base/road_rulebook_loader_using_rule_registry.cc#L206)
///
/// The following rules are created and added programatically on top what the RoadRulebook YAML file describes.
///  - Speed limits: maliput::api::rules::RangeValueRule whose type
///    is "Speed-Limit Rule Type" are created.
///  - Direction usage: maliput::api::rules::DiscretValueRule whose type
///    is "Direction-Usage Rule Type" are created.
///  - Vehicle exclusive: maliput::api::rules::DiscretValueRule whose type is "Vehicle-Exclusive Rule Type" are created.
///  - Vehicle usage: maliput::api::rules::DiscretValueRule whose type is "Vehicle-Usage Rule Type" are created.
///
class RoadRuleBookBuilder {
 public:
  /// Populates RoadRulebook using rule types previously defined in the RuleRegistry.
  /// These rules' values are obtained from the XODR file description. They are of type:
  /// - Vehicle-Usage Rule Type
  /// - Vehicle-Exclusive Rule Type
  /// - Speed-Limit Rule Type
  /// - Direction-Usage Rule Type
  ///
  /// See malidrive::builder::RuleRegistryBuilder for further information.
  static void AddsXODRBasedRulesToRulebook(const maliput::api::RoadGeometry* rg,
                                           const maliput::api::rules::RuleRegistry* rule_registry,
                                           maliput::ManualRulebook* rulebook);

  /// Constructs a RoadRuleBook.
  ///
  /// @param rg is the pointer to the RoadGeometry. It must not be nullptr.
  /// @param rule_registry is the pointer to the RuleRegistry. It must not be nullptr.
  /// @param road_rulebook_file_path to the yaml file to load the RoadRulebook.
  /// @throw maliput::assertion_error When `rg` or `rule_registry` are nullptr.
  RoadRuleBookBuilder(const maliput::api::RoadGeometry* rg, const maliput::api::rules::RuleRegistry* rule_registry,
                      const std::optional<std::string>& road_rulebook_file_path);

  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadRuleBookBuilder)
  RoadRuleBookBuilder() = delete;

  /// Builds a ManualRulebook.
  std::unique_ptr<const maliput::api::rules::RoadRulebook> operator()();

 private:
  // Creates vehicle usage and vehicle exclusive rules for all Lanes in the
  // RoadGeometry.
  // Rule values are created based on the XODR Lane Type.
  //
  // @param rg is the pointer to the RoadGeometry. It must not be nullptr.
  // @param rule_registry is the pointer to the RuleRegistry. It must not be nullptr.
  // @param rulebook The pointer to the RoadRulebook based to add the rules.
  //        It must not be nullptr.
  static void AddsVehicleExclusiveAndUsageRulesToRulebook(const maliput::api::RoadGeometry* rg,
                                                          const maliput::api::rules::RuleRegistry* rule_registry,
                                                          maliput::ManualRulebook* rulebook);

  // Creates speed limit rules for all Lanes in the RoadGeometry.
  // Rule values have been previously registered in the
  // maliput::api::rules::RuleRegistry, which was provided at time of
  // construction.
  //
  // Speed limit rules naming convention:
  // Rule Id: "<RuleType> + '/' + <LaneID> + '_' + <index>".
  // Where 'index' is a number that increments in order to differentiate between speed
  // rules limit within a Lane.
  //
  // @param rg is the pointer to the RoadGeometry. It must not be nullptr.
  // @param rule_registry is the pointer to the RuleRegistry. It must not be nullptr.
  // @param rulebook The RoadRulebook to which to add the rules.
  //        It must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rg` is nullptr.
  // @throws maliput::common::assertion_error When `rule_registry` is nullptr.
  // @throws maliput::common::assertion_error When `rulebook` is nullptr.
  // @throws maliput::common::assertion_error When
  // maliput::api::rules::RuleRegistry has no type equal to
  // maliput::SpeedLimitRuleTypeId().
  // @throws maliput::common::assertion_error When a Lane in the RoadGeometry
  // has a max speed limit that does not match any range in
  // maliput::SpeedLimitRuleTypeId() rule type.
  static void AddsSpeedLimitRulesToRulebook(const maliput::api::RoadGeometry* rg,
                                            const maliput::api::rules::RuleRegistry* rule_registry,
                                            maliput::ManualRulebook* rulebook);

  // Creates direction usage rules for the entire RoadGeometry.
  // Direction Usage Rule Type must have been previously registered in the
  // maliput::api::rules::RuleRegistry, which was provided at time of
  // construction.
  //
  // @param rg is the pointer to the RoadGeometry. It must not be nullptr.
  // @param rule_registry is the pointer to the RuleRegistry. It must not be nullptr.
  // @param rulebook The RoadRulebook to add the rules.
  //        It must not be nullptr.
  //
  // @throws maliput::common::assertion_error When `rulebook` is nullptr.
  static void AddsDirectionUsageRulesToRulebook(const maliput::api::RoadGeometry* rg,
                                                const maliput::api::rules::RuleRegistry* rule_registry,
                                                maliput::ManualRulebook* rulebook);

  // @returns A LaneSRoute that covers `lane.`
  // @throws maliput::common::assertion_error When `lane` is nullptr.
  static maliput::api::LaneSRoute CreateLaneSRouteFor(const Lane* lane);

  const maliput::api::RoadGeometry* rg_{};
  const maliput::api::rules::RuleRegistry* rule_registry_{};
  const std::optional<std::string> road_rulebook_file_path_{};
};

}  // namespace builder
}  // namespace malidrive
