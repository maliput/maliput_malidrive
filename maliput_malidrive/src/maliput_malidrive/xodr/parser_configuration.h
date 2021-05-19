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

  /// Determines whether the xodr should be parsed in a permissive mode.
  /// If activated, the OpenDrive standard is relaxed when:
  ///  1. Junction node doesn't contain any connections.
  bool permissive_mode{true};
};

}  // namespace xodr
}  // namespace malidrive
