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
};

}  // namespace xodr
}  // namespace malidrive
