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
