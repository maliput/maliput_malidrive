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
#include <optional>

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/xodr/lane_link.h"
#include "maliput_malidrive/xodr/lane_width.h"
#include "maliput_malidrive/xodr/unit.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Lane.
/// For example, a XML node describing a XODR's lane:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///     <road>
///       <lanes>
///         <laneSection>
///           <left>
///             <lane id="-1" type="driving" level="false">
///               <link>
///                   <predecessor id="-1"/>
///                   <successor id="-1"/>
///               </link>
///               <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
///               <speed sOffset="0.0" max="40" unit="mph"/>
///               <speed sOffset="1.5" max="45" unit="mph"/>
///               <userData>
///                  ...
///               </userData>
///               ...
///             </lane>
///           </left>
///           <center>
///             <lane id="0" type="driving" level="false">
///           </center>
///           <right>
///             <lane id="1" type="driving" level="false">
///               <link>
///                   <predecessor id="1"/>
///                   <successor id="1"/>
///               </link>
///               <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
///               ...
///             </lane>
///           </right>
///         </laneSection>
///       </lanes>
///     <road>
///       ...
///   </OpenDRIVE>
/// @endcode
struct Lane {
  /// Id alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct Lane>;

  /// Convenient constants that hold the tag names in XODR lane description.
  static constexpr const char* kLaneTag = "lane";
  static constexpr const char* kId = "id";
  static constexpr const char* kType = "type";
  static constexpr const char* kLevel = "level";
  static constexpr const char* kUserData = "userData";

  /// Holds types of lanes.
  enum class Type {
    kNone,
    kDriving,
    kStop,
    kShoulder,
    kBiking,
    kSidewalk,
    kBorder,
    kRestricted,
    kParking,
    /// `bidirectional` could be used on a narrow road which may be used in both directios or in a continuous two-way
    /// left turn lane on multi-lane roads.
    kBidirectional,
    kMedian,
    kSpecial1,
    kSpecial2,
    kSpecial3,
    kRoadWorks,
    kTram,
    kRail,
    kEntry,
    kExit,
    kOffRamp,
    kOnRamp,
    kConnectingRamp,
    kBus,
    kTaxi,
    /// High-occupancy vehicle / carpool vehicle.
    kHOV,
    kMwyEntry,
    kMwyExit,
  };

  /// Speed description.
  struct Speed {
    /// Holds the tag name in the XODR Lane Speed description.
    static constexpr const char* kSpeedTag = "speed";
    static constexpr const char* kSOffset = "sOffset";
    static constexpr const char* kMax = "max";
    static constexpr const char* kUnit = "unit";

    /// Start position (s-coordinate) relative to the position of the preceding laneSection record.
    double s_offset{};

    /// Maximum allowed speed.
    double max{};

    /// Unit of the `max` attribute.
    Unit unit{Unit::kMs};

    /// Equality operator.
    bool operator==(const Speed& other) const {
      return s_offset == other.s_offset && max == other.max && unit == other.unit;
    }

    /// Inequality operator.
    bool operator!=(const Speed& other) const { return !(*this == other); }
  };

  /// Matches string with a Type.
  /// @param type Is a Type.
  /// @returns A string that matches with `type`.
  static std::string type_to_str(Type type);

  /// Matches Type with a string.
  /// @param type Is a string.
  /// @returns A Type that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a Type.
  static Type str_to_type(const std::string& type);

  /// Equality operator.
  bool operator==(const Lane& other) const;

  /// Inequality operator.
  bool operator!=(const Lane& other) const;

  /// ID of the lane.
  Id id{"none"};
  /// Type of the lane.
  Type type{};
  /// Indicates whether apply or not superelevation and crossfall to this lane:
  /// - true: keep lane on level, .i.e. do not apply superelevation or crossfall.
  /// - false: apply superelevation and crossfall to this lane.
  std::optional<bool> level{std::nullopt};
  /// Contains the lane link.
  LaneLink lane_link{std::nullopt, std::nullopt};
  /// Widths of the lane.
  std::vector<LaneWidth> width_description{};
  /// Speed records of the lane.
  std::vector<Speed> speed{};
  /// Contains ancillary data in XML format. It holds the entire userData node.
  std::optional<std::string> user_data{std::nullopt};
};

}  // namespace xodr
}  // namespace malidrive
