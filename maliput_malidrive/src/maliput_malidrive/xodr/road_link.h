// Copyright 2020 Toyota Research Institute
#pragma once

#include <optional>

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds the road link description of a XODR road.
/// For example:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///        <link>
///            <predecessor elementType="road" elementId="80" contactPoint="end"/>
///            <successor elementType="road" elementId="10" contactPoint="start"/>
///        </link>
///       ...
///   </OpenDRIVE>
/// @endcode
struct RoadLink {
  /// Convenient constants that hold the tag names in the XODR road header description.
  static constexpr const char* kRoadLinkTag = "link";
  static constexpr const char* kPredecessorTag = "predecessor";
  static constexpr const char* kSuccessorTag = "successor";
  /// Enum the contact points.
  enum ContactPoint { kStart = 0, kEnd };
  /// Enum the types of the link's element.
  enum ElementType { kRoad = 0, kJunction };

  /// Contains the information about the predecessor/successor road.
  struct LinkAttributes {
    using Id = maliput::api::TypeSpecificIdentifier<struct LinkAttributes>;
    /// Convenient constants that hold the tag names in the XODR road header description.
    static constexpr const char* kElementType = "elementType";
    static constexpr const char* kElementId = "elementId";
    static constexpr const char* kContactPoint = "contactPoint";
    /// Type of the linked element.
    ElementType element_type{};
    /// ID of the linked element.
    Id element_id{"none"};
    /// Contact point of link on the linked element.
    std::optional<ContactPoint> contact_point{std::nullopt};

    /// Equality operator.
    bool operator==(const LinkAttributes& other) const;
  };

  /// Matches ElementType with a string.
  /// @param type Is a string.
  /// @returns A ElementType that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a ElementType.
  static ElementType str_to_element_type(const std::string& type);

  /// Matches string with a ElementType.
  /// @param type Is a ElementType.
  /// @returns A string that matches with `type`.
  static std::string element_type_to_str(ElementType type);

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
  bool operator==(const RoadLink& other) const {
    return successor == other.successor && predecessor == other.predecessor;
  }
  /// Inequality operator.
  bool operator!=(const RoadLink& other) const { return !(*this == other); }

  /// Road's predecessor.
  std::optional<LinkAttributes> predecessor{std::nullopt};
  /// Road's successor.
  std::optional<LinkAttributes> successor{std::nullopt};
};

}  // namespace xodr
}  // namespace malidrive
