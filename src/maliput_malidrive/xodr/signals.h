// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven Planet. All rights reserved.
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

#include <vector>

#include "maliput_malidrive/xodr/signal.h"

namespace malidrive {
namespace xodr {

/// Holds the values of a XODR Signals node.
/// For example, a XML node describing a XODR's signals node:
/// @code{.xml}
///  <OpenDRIVE>
///       ...
///   <signals>
///      <signal s="0.0000000000000000e+0" id="1" name="signal1" ... />
///      <signal s="3.8268524704053952e-2" id="2" name="signal2" ... />
///   </signals>
///     ...
///  </OpenDRIVE>
/// @endcode
struct Signals {
  /// Hold the tag for the signals in the XODR signals description.
  static constexpr const char* kSignalsTag = "signals";

  /// Equality operator.
  bool operator==(const Signals& other) const;

  /// Inequality operator.
  bool operator!=(const Signals& other) const;

  /// Holds all the `Signal`s in the road.
  std::vector<Signal> signals{};
};

}  // namespace xodr
}  // namespace malidrive
