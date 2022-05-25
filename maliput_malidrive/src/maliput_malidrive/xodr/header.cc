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
#include "maliput_malidrive/xodr/header.h"

namespace malidrive {
namespace xodr {

bool Header::operator==(const Header& other) const {
  return rev_major == other.rev_major && rev_minor == other.rev_minor && name == other.name &&
         version == other.version && date == other.date && north == other.north && south == other.south &&
         east == other.east && west == other.west && vendor == other.vendor;
}

bool Header::operator!=(const Header& other) const { return !(*this == other); }

std::ostream& operator<<(std::ostream& out, const Header& header) {
  out << "{ \"rev_major\": " << header.rev_major << ", \"rev_minor\": " << header.rev_minor;
  out << ", \"name\": {" << (header.name.has_value() ? header.name.value() : "") << "}";
  out << ", \"version\": {" << (header.version.has_value() ? std::to_string(header.version.value()) : "") << "}";
  out << ", \"date\": {" << (header.date.has_value() ? header.date.value() : "") << "}";
  out << ", \"north\": {" << (header.north.has_value() ? std::to_string(header.north.value()) : "") << "}";
  out << ", \"south\": {" << (header.south.has_value() ? std::to_string(header.south.value()) : "") << "}";
  out << ", \"east\": {" << (header.east.has_value() ? std::to_string(header.east.value()) : "") << "}";
  out << ", \"west\": {" << (header.west.has_value() ? std::to_string(header.west.value()) : "") << "}";
  out << ", \"vendor\": {" << (header.vendor.has_value() ? header.vendor.value() : "") << "}";
  out << "}";

  return out;
}

}  // namespace xodr
}  // namespace malidrive
