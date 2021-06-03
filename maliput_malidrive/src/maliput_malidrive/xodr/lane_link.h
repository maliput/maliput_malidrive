// Copyright 2020 Toyota Research Institute
#pragma once

#include <optional>

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

/// Holds the lane link description in a XODR.
/// For example:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///        <link>
///            <predecessor id="-1"/>
///            <successor id="-1"/>
///        </link>
///       ...
///   </OpenDRIVE>
/// @endcode
struct LaneLink {
  /// Convenient constants that hold the tag names in the XODR lane link header description.
  static constexpr const char* kLaneLinkTag = "link";
  static constexpr const char* kPredecessorTag = "predecessor";
  static constexpr const char* kSuccessorTag = "successor";

  /// Contains the information about the predecessor/successor road.
  struct LinkAttributes {
    using Id = maliput::api::TypeSpecificIdentifier<struct LinkAttributes>;
    /// Convenient constants that hold the tag names in the XODR lane link description.
    static constexpr const char* kId = "id";
    /// ID of the linked element.
    Id id{"none"};
    /// Equality operator.
    bool operator==(const LinkAttributes& other) const { return id == other.id; };
  };

  /// Equality operator.
  bool operator==(const LaneLink& other) const {
    return successor == other.successor && predecessor == other.predecessor;
  }
  /// Inequality operator.
  bool operator!=(const LaneLink& other) const { return !(*this == other); }

  /// Road's predecessor.
  std::optional<LinkAttributes> predecessor{std::nullopt};
  /// Road's successor.
  std::optional<LinkAttributes> successor{std::nullopt};
};

}  // namespace xodr
}  // namespace malidrive
