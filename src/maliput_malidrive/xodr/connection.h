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

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds a Connection description of a XODR junction.
/// Provides information about a single connection within a junction.
/// For example:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///        <connection id="0" incomingRoad="166" connectingRoad="409" contactPoint="start">
///            <laneLink from="-2" to="-2"/>
///            <laneLink from="-3" to="-3"/>
///            <laneLink from="-4" to="-4"/>
///        </connection>
///       ...
///   </OpenDRIVE>
/// @endcode
struct Connection {
  using Id = maliput::api::TypeSpecificIdentifier<struct Connection>;

  /// Convenient constants that hold the tag names in the XODR connection description.
  static constexpr const char* kConnectionTag = "connection";
  static constexpr const char* kId = "id";
  static constexpr const char* kIncomingRoad = "incomingRoad";
  static constexpr const char* kConnectingRoad = "connectingRoad";
  static constexpr const char* kContactPoint = "contactPoint";
  static constexpr const char* kConnectionMaster = "connectionMaster";
  static constexpr const char* kType = "type";

  /// Enum the contact points.
  enum ContactPoint { kStart = 0, kEnd };

  /// Enum junction types.
  enum Type { kDefault = 0, kVirtual };

  /// Holds a LaneLink description of a XODR junction.
  /// The junction lane link record provides information
  /// about the lanes which are linked between incoming
  /// road and connecting road.
  struct LaneLink {
    using Id = maliput::api::TypeSpecificIdentifier<struct LaneLink>;

    /// Convenient constants that hold the tag names in the XODR laneLink description.
    static constexpr const char* kLaneLinkTag = "laneLink";
    static constexpr const char* kFrom = "from";
    static constexpr const char* kTo = "to";

    /// ID of the incoming lane.
    Id from{"none"};
    /// ID of the connecting lane.
    Id to{"none"};

    /// Equality operator.
    bool operator==(const LaneLink& other) const { return from == other.from && to == other.to; }
  };

  /// Matches Type with a string.
  /// @param type Is a string.
  /// @returns A Type that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a Type.
  static Type str_to_type(const std::string& type);

  /// Matches string with a Type.
  /// @param type Is a Type.
  /// @returns A string that matches with `type`.
  static std::string type_to_str(Type type);

  /// Matches ContactPoint with a string.
  /// @param contact_point Is a string.
  /// @returns A ContactPoint that matches with `contact_point`.
  /// @throw maliput::common::assertion_error When `contact_point` doesn't match with a ContactPoint.
  static ContactPoint str_to_contact_point(const std::string& contact_point);

  /// Matches string with a ContactPoint.
  /// @param contact_point Is a ContactPoint.
  /// @returns A string that matches with `contact_point`.
  static std::string contact_point_to_str(ContactPoint contact_point);

  /// Equality operator.
  bool operator==(const Connection& other) const;
  /// Inequality operator.
  bool operator!=(const Connection& other) const { return !(*this == other); }

  /// Connetion's id.
  Id id{"None"};
  /// ID of the incoming road.
  std::string incoming_road{};
  /// ID of the connecting path.
  std::string connecting_road{};
  /// Contact point on the connecting road.
  ContactPoint contact_point{ContactPoint::kStart};
  /// ID of the connection which will act as master.
  std::optional<Id> connection_master{std::nullopt};
  /// Type of the connection, required for virtual connections only.
  std::optional<Type> type{Type::kDefault};
  /// Links between lanes.
  std::vector<LaneLink> lane_links{};
};

}  // namespace xodr
}  // namespace malidrive
