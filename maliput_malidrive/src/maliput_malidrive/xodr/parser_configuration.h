// Copyright 2021 Toyota Research Institute
#pragma once

#include <optional>

namespace malidrive {
namespace xodr {

/// Holds the configuration for the parser. @see xodr::ParserBase
struct ParserConfiguration {
  /// Tolerance used to verify values in the XML node. When
  /// it is std::nullopt, no contiguity check is performed.
  std::optional<double> tolerance{std::nullopt};

  /// When active, the parser will relaxe constraints related to xodr syntax.
  /// It will not throw upon the following conditions:
  /// - Junctions without connections.
  bool allow_schema_errors{true};
  /// When active, the parser will relaxe constraints related to xodr semantic.
  /// It will not throw upon the following conditions:
  /// - Non reciprocal Road linkage.
  bool allow_semantic_errors{true};
};

}  // namespace xodr
}  // namespace malidrive
