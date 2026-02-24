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

#include "maliput_malidrive/xodr/signal/board.h"

#include "maliput_malidrive/xodr/signal/sign.h"

namespace malidrive {
namespace xodr {
namespace signal {

StaticBoard::StaticBoard() = default;
StaticBoard::~StaticBoard() = default;
StaticBoard::StaticBoard(const StaticBoard&) = default;
StaticBoard& StaticBoard::operator=(const StaticBoard&) = default;
StaticBoard::StaticBoard(StaticBoard&&) noexcept = default;
StaticBoard& StaticBoard::operator=(StaticBoard&&) noexcept = default;

StaticBoard::StaticBoard(const std::vector<Sign>& signs_init) : signs(signs_init) {}

bool StaticBoard::operator==(const StaticBoard& other) const { return signs == other.signs; }

bool StaticBoard::operator!=(const StaticBoard& other) const { return !(*this == other); }

bool DisplayArea::operator==(const DisplayArea& other) const {
  return height == other.height && index == other.index && v == other.v && width == other.width && z == other.z;
}

bool DisplayArea::operator!=(const DisplayArea& other) const { return !(*this == other); }

bool VmsBoard::operator==(const VmsBoard& other) const {
  return display_height == other.display_height && display_type == other.display_type &&
         display_width == other.display_width && v == other.v && z == other.z && display_areas == other.display_areas;
}

bool VmsBoard::operator!=(const VmsBoard& other) const { return !(*this == other); }

bool VmsBoardReference::operator==(const VmsBoardReference& other) const {
  return group_index == other.group_index && signal_id == other.signal_id && vms_index == other.vms_index;
}

bool VmsBoardReference::operator!=(const VmsBoardReference& other) const { return !(*this == other); }

bool VmsGroup::operator==(const VmsGroup& other) const {
  return id == other.id && vms_board_references == other.vms_board_references;
}

bool VmsGroup::operator!=(const VmsGroup& other) const { return !(*this == other); }

}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
