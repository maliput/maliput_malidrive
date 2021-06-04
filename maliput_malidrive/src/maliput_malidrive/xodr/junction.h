// Copyright 2020 Toyota Research Institute
#pragma once

#include <optional>
#include <ostream>
#include <unordered_map>

#include <maliput/api/type_specific_identifier.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/connection.h"

namespace malidrive {
namespace xodr {

/// Holds a junction description of a XODR road.
/// For example:
/// @code{.xml}
///   <OpenDRIVE>
///       ...
///    <junction id="406" name="junction406">
///        <connection id="0" incomingRoad="166" connectingRoad="409" contactPoint="start">
///            <laneLink from="-2" to="-2"/>
///            <laneLink from="-3" to="-3"/>
///            <laneLink from="-4" to="-4"/>
///        </connection>
///    </junction>
///       ...
///   </OpenDRIVE>
/// @endcode
struct Junction {
  using Id = maliput::api::TypeSpecificIdentifier<struct Junction>;

  /// Convenient constants that hold the tag names in the XODR junction description.
  static constexpr const char* kJunctionTag = "junction";
  static constexpr const char* kId = "id";
  static constexpr const char* kName = "name";
  static constexpr const char* kType = "type";

  /// Enum junction types.
  enum Type { kDefault = 0, kVirtual };

  /// Matches Type with a string.
  /// @param type Is a string.
  /// @returns A Type that matches with `type`.
  /// @throw maliput::common::assertion_error When `type` doesn't match with a Type.
  static Type str_to_type(const std::string& type);

  /// Matches string with a Type.
  /// @param type Is a Type.
  /// @returns A string that matches with `type`.
  static std::string type_to_str(Type type);

  /// Equality operator.
  bool operator==(const Junction& other) const;
  /// Inequality operator.
  bool operator!=(const Junction& other) const { return !(*this == other); }

  /// Junction's id.
  Id id{"None"};
  /// Junction's name.
  std::optional<std::string> name{std::nullopt};
  /// Type of the junction, required for "virtual" junctions only.
  std::optional<Type> type{Type::kDefault};
  /// Connections within the junction.
  std::unordered_map<Connection::Id, Connection> connections{};
};

/// Streams a string representation of @p junction into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const Junction& junction);

}  // namespace xodr
}  // namespace malidrive
