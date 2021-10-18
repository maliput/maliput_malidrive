// Copyright 2021 Toyota Research Institute
#pragma once

#include <optional>

namespace malidrive {
namespace xodr {

/// Holds the configuration for the parser. @see xodr::ParserBase
struct ParserConfiguration {
  /// When active, the parser will relaxe constraints related to xodr syntax.
  /// It will not throw upon the following conditions:
  /// - Junctions without connections.
  /// - Functions starting at the same `s`.
  /// - Functions with NaN values as coefficients starting at same `s` as the following function.
  /// - Functions starting at the end of the road.
  bool allow_schema_errors{true};
  /// When active, the parser will relaxe constraints related to xodr semantic.
  /// It will not throw upon the following conditions:
  /// - Non reciprocal Road linkage.
  /// - Non reciprocal Lane linkage within a Road.
  bool allow_semantic_errors{true};
};

}  // namespace xodr
}  // namespace malidrive
