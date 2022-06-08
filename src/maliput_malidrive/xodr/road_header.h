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

#include <optional>
#include <ostream>
#include <string>

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/lanes.h"
#include "maliput_malidrive/xodr/reference_geometry.h"
#include "maliput_malidrive/xodr/road_link.h"
#include "maliput_malidrive/xodr/road_type.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR description's Road header.
/// For example, a XODR description with a Road header:
/// @code{.xml}
///   <OpenDRIVE>
///       <road name="Road 0" length="1.4005927435591335e+2" id="0" junction="-1" rule="RHT">
///           <link>
///               <predecessor elementType="road" elementId="80" contactPoint="end"/>
///               <successor elementType="road" elementId="10" contactPoint="start"/>
///           </link>
///           <type s="0.0000000000000000e+0" type="town">
///               <speed max="40" unit="mph"/>
///           </type>
///           <planView>
///               <geometry s="0.0000000000000000e+0" x="0.0000000000000000e+0" y="0.0000000000000000e+0"
///               hdg="1.3734007669450161e+0" length="1.4005927435591335e+2">
///                   <line/>
///               </geometry>
///           </planView>
///           <lanes>
///               <laneOffset s='0.' a='2.2' b='3.3' c='4.4' d='5.5'/>
///               <laneSection s='0.'>
///                   <left>
///                       <lane id='1' type='driving' level= '0'>
///                           <width sOffset='0.' a='1.' b='2.' c='3.' d='4.'/>
///                       </lane>
///                   </left>
///                   <center>
///                       <lane id='0' type='driving' level= '0'>
///                       </lane>
///                   </center>
///                   <right>
///                       <lane id='-1' type='driving' level= '0'>
///                           <width sOffset='5.' a='6.' b='7.' c='8.' d='9.'/>
///                       </lane>
///                   </right>
///               </laneSection>
///           </lanes>
///       </road>
///   </OpenDRIVE>
/// @endcode
struct RoadHeader {
  /// Id alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct RoadHeader>;

  /// Convenient constants that hold the tag names in the XODR road header description.
  static constexpr const char* kRoadHeaderTag = "road";
  static constexpr const char* kName = "name";
  static constexpr const char* kLength = "length";
  static constexpr const char* kId = "id";
  static constexpr const char* kJunction = "junction";
  static constexpr const char* kRule = "rule";

  /// Holds the hand traffic rules.
  enum class HandTrafficRule {
    /// Right handed traffic.
    kRHT = 0,
    /// Left handed traffic.
    kLHT,
  };

  /// Matches string with a HandTrafficRule.
  /// @param rule Is a HandTrafficRule.
  /// @returns A string that matches with `rule`.
  static std::string hand_traffic_rule_to_str(HandTrafficRule rule);

  /// Matches HandTrafficRule with a string.
  /// @param rule Is a string.
  /// @returns A HandtrafficRule that matches with `rule`.
  /// @throw maliput::common::assertion_error When `rule` doesn't match with a HandTrafficRule.
  static HandTrafficRule str_to_hand_traffic_rule(const std::string& rule);

  /// Get length of a lane section.
  /// @param index The `index`-th LaneSection to retrieve its length.
  /// @returns The length of the `index`-th LaneSection.
  /// @throw maliput::common::assertion_error When `index` is negative.
  /// @throw maliput::common::assertion_error When `index` is greater than the numbers of LaneSections in the road.
  double GetLaneSectionLength(int index) const;

  /// Get the lane section index for a particular s-cooridnate.
  /// @param s A s-coordinate of the Road.
  /// @returns The index of the lane section.
  /// @throw maliput::common::assertion_error When neither LaneSection's range contain the `s` coordinate.
  int GetLaneSectionIndex(double s) const;

  /// Get length of a RoadType.
  /// @param index The `index`-th RoadType to retrieve its length.
  /// @returns The length of the `index`-th RoadType.
  /// @throw maliput::common::assertion_error When `index` is negative.
  /// @throw maliput::common::assertion_error When `index` is greater than the numbers of RoadTypes in the road.
  double GetRoadTypeLength(int index) const;

  /// Get the RoadType for a particular s-cooridnate.
  /// @param s A s-coordinate of the Road.
  /// @returns A pointer to the correspondant RoadType. Nullptr if there is no RoadTypes described.
  /// @throw maliput::common::assertion_error When neither RoadType's range contain the `s` coordinate.
  const RoadType* GetRoadType(double s) const;

  /// Get the RoadTypes that are present within the range: [`s_start`, `s_end`].
  /// @param s_start Track s-coordinate that specifies the start of the range.
  /// @param s_end Track s-coordinate that specifies the end of the range.
  /// @returns A vector of RoadTypes.
  /// @throw maliput::common::assertion_error When `s_start` is greater than `s_end`.
  /// @throw maliput::common::assertion_error When `s_start` is negative.
  std::vector<const RoadType*> GetRoadTypesInRange(double s_start, double s_end) const;

  /// Get start s value of the Road.
  /// It is delimited by the start point of the first geometry in the planView.
  double s0() const;

  /// Get start s value of the Road.
  /// It is computed as the sum of the last geometry's start point and its length.
  double s1() const;

  /// Equality operator.
  bool operator==(const RoadHeader& other) const;

  /// Inequality operator.
  bool operator!=(const RoadHeader& other) const;

  /// Name of the road.
  std::optional<std::string> name{std::nullopt};
  /// Total length of the reference line in the xy-plane.
  double length{};
  /// Unique ID within database.
  Id id{"none"};
  /// ID of the junction to which the road belongs as a connecting road.(-1 for none.)
  std::string junction{};
  /// Hand traffic rule of the road.
  std::optional<HandTrafficRule> rule{std::nullopt};
  /// Contains the road link.
  RoadLink road_link{std::nullopt, std::nullopt};
  /// Contains the road types of the road.
  std::vector<RoadType> road_types{};
  /// Contains the geometry description of the road.
  ReferenceGeometry reference_geometry{};
  /// Holds the road's lanes.
  Lanes lanes{};
};

/// Streams a string representation of @p road_header into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const RoadHeader& road_header);

}  // namespace xodr
}  // namespace malidrive
