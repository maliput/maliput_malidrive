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

  /// When active, the parser will not throw upon the following conditions:
  /// - Junctions without connections.
  bool permissive_mode{true};
};

}  // namespace xodr
}  // namespace malidrive
