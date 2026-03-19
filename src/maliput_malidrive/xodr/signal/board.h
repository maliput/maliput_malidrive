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

#include "maliput_malidrive/xodr/signal/dependency.h"
#include "maliput_malidrive/xodr/signal/reference.h"
#include "maliput_malidrive/xodr/validity.h"

namespace malidrive {
namespace xodr {
namespace signal {

struct Sign;

/// Holds the values of an XODR StaticBoard.
struct StaticBoard {
  static constexpr const char* kStaticBoardTag = "staticBoard";

  /// Signs that are displayed on the static board.
  std::vector<Sign> signs;
  std::vector<Dependency> dependencies;
  std::vector<Reference> references;
  std::vector<malidrive::xodr::Validity> validities;

  // These are needed so the compiler can write std::vector<Sign> initializers and destroyers without knowing the
  // complete definition of Sign.
  StaticBoard();
  ~StaticBoard();
  StaticBoard(const StaticBoard&);
  StaticBoard& operator=(const StaticBoard&);
  StaticBoard(StaticBoard&&) noexcept;
  StaticBoard& operator=(StaticBoard&&) noexcept;

  StaticBoard(const std::vector<Sign>& signs_init, const std::vector<Dependency>& dependencies_init,
              const std::vector<Reference>& references_init,
              const std::vector<malidrive::xodr::Validity>& validities_init);

  bool operator==(const StaticBoard& other) const;
  bool operator!=(const StaticBoard& other) const;
};

/// Holds the values of an XODR Variable message board.
struct VmsBoard {
  static constexpr const char* kVmsBoardTag = "vmsBoard";
  static constexpr const char* kDisplayHeight = "displayHeight";
  static constexpr const char* kDisplayType = "displayType";
  static constexpr const char* kDisplayWidth = "displayWidth";
  static constexpr const char* kV = "v";
  static constexpr const char* kZ = "z";

  /// Represents the e_road_signals_displayType of a variable message board.
  enum class DisplayType {
    kOther,
    /// Full LED boards.
    kLed,
    kMonochromeGraphic,
    kRotatingPrismHorizontal,
    kRotatingPrismVertical,
    kSimpleMatrix,
  };

  /// Holds the values of an XODR DisplayArea.
  struct DisplayArea {
    static constexpr const char* kDisplayAreaTag = "displayArea";
    static constexpr const char* kHeight = "height";
    static constexpr const char* kIndex = "index";
    static constexpr const char* kV = "v";
    static constexpr const char* kWidth = "width";
    static constexpr const char* kZ = "z";

    /// Height of the display area.
    double height{};
    /// Index of the display area.
    int index{};
    /// Local v-coordinate of the display area on the board.
    double v{};
    /// Width of the display area.
    double width{};
    /// Local z-coordinate of the display area on the board.
    double z{};

    bool operator==(const DisplayArea& other) const;
    bool operator!=(const DisplayArea& other) const;
  };

  /// Height of the display.
  std::optional<double> display_height;
  /// Functional type of the display.
  DisplayType display_type{};
  /// Width of the display.
  std::optional<double> display_width;
  /// Local v-coordinate of the board.
  double v{};
  /// Local z-coordinate of the board.
  double z{};

  /// Collection of defined display areas on this variable message board.
  std::vector<DisplayArea> display_areas;
  std::vector<Dependency> dependencies;
  std::vector<Reference> references;
  std::vector<malidrive::xodr::Validity> validities;

  bool operator==(const VmsBoard& other) const;
  bool operator!=(const VmsBoard& other) const;
};

}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
