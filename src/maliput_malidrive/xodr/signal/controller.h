// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include <string>
#include <vector>

#include <maliput/api/type_specific_identifier.h>

namespace malidrive {
namespace xodr {
namespace signal {

/// Holds the values of a XODR control element.
struct Control {
  /// Signal ID alias.
  using SignalId = maliput::api::TypeSpecificIdentifier<struct Control>;

  /// Convenient constants that hold the tag names in the XODR control description.
  static constexpr const char* kControlTag = "control";
  static constexpr const char* kSignalId = "signalId";
  static constexpr const char* kType = "type";

  /// ID of the controlled signal.
  SignalId signal_id;
  /// Type of control.
  std::optional<std::string> type{};

  /// Equality operator.
  bool operator==(const Control& other) const;

  /// Inequality operator.
  bool operator!=(const Control& other) const;
};

/// Holds the values of a XODR controller element.
struct Controller {
  /// Controller ID alias.
  using Id = maliput::api::TypeSpecificIdentifier<struct Controller>;

  /// Convenient constants that hold the tag names in the XODR controller description.
  static constexpr const char* kControllerTag = "controller";
  static constexpr const char* kId = "id";
  static constexpr const char* kName = "name";
  static constexpr const char* kSequence = "sequence";

  /// ID of the controller.
  Id id;
  /// Maximum ID of the lanes for which the parent element is valid.
  std::optional<std::string> name{};
  /// Sequence number of the controller with respect to other controllers of the same logical level.
  /// It acts like some sort of priority, where the controller with the lowest sequence number has the highest priority.
  std::optional<int> sequence{};
  /// Controlled signals.
  std::vector<Control> controls{};

  /// Equality operator.
  bool operator==(const Controller& other) const;

  /// Inequality operator.
  bool operator!=(const Controller& other) const;
};

}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
