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
#include "maliput_malidrive/xodr/signal/semantics.h"

namespace malidrive {
namespace xodr {
namespace signal {

bool Semantics::Speed::operator==(const Speed& other) const {
  return type == other.type && unit == other.unit && value == other.value;
}

bool Semantics::Speed::operator!=(const Speed& other) const { return !(*this == other); }

bool Semantics::Lane::operator==(const Lane& other) const { return type == other.type; }

bool Semantics::Lane::operator!=(const Lane& other) const { return !(*this == other); }

bool Semantics::Priority::operator==(const Priority& other) const { return type == other.type; }

bool Semantics::Priority::operator!=(const Priority& other) const { return !(*this == other); }

bool Semantics::Prohibited::operator==(const Prohibited& /*other*/) const { return true; }

bool Semantics::Prohibited::operator!=(const Prohibited& other) const { return !(*this == other); }

bool Semantics::Warning::operator==(const Warning& /*other*/) const { return true; }

bool Semantics::Warning::operator!=(const Warning& other) const { return !(*this == other); }

bool Semantics::Routing::operator==(const Routing& /*other*/) const { return true; }

bool Semantics::Routing::operator!=(const Routing& other) const { return !(*this == other); }

bool Semantics::StreetName::operator==(const StreetName& /*other*/) const { return true; }

bool Semantics::StreetName::operator!=(const StreetName& other) const { return !(*this == other); }

bool Semantics::Parking::operator==(const Parking& /*other*/) const { return true; }

bool Semantics::Parking::operator!=(const Parking& other) const { return !(*this == other); }

bool Semantics::Tourist::operator==(const Tourist& /*other*/) const { return true; }

bool Semantics::Tourist::operator!=(const Tourist& other) const { return !(*this == other); }

bool Semantics::SupplementaryTime::operator==(const SupplementaryTime& other) const {
  return type == other.type && value == other.value;
}

bool Semantics::SupplementaryTime::operator!=(const SupplementaryTime& other) const { return !(*this == other); }

bool Semantics::SupplementaryAllows::operator==(const SupplementaryAllows& /*other*/) const { return true; }

bool Semantics::SupplementaryAllows::operator!=(const SupplementaryAllows& other) const { return !(*this == other); }

bool Semantics::SupplementaryProhibits::operator==(const SupplementaryProhibits& /*other*/) const { return true; }

bool Semantics::SupplementaryProhibits::operator!=(const SupplementaryProhibits& other) const {
  return !(*this == other);
}

bool Semantics::SupplementaryDistance::operator==(const SupplementaryDistance& other) const {
  return type == other.type && unit == other.unit && value == other.value;
}

bool Semantics::SupplementaryDistance::operator!=(const SupplementaryDistance& other) const {
  return !(*this == other);
}

bool Semantics::SupplementaryEnvironment::operator==(const SupplementaryEnvironment& other) const {
  return type == other.type;
}

bool Semantics::SupplementaryEnvironment::operator!=(const SupplementaryEnvironment& other) const {
  return !(*this == other);
}

bool Semantics::SupplementaryExplanatory::operator==(const SupplementaryExplanatory& /*other*/) const { return true; }

bool Semantics::SupplementaryExplanatory::operator!=(const SupplementaryExplanatory& other) const {
  return !(*this == other);
}

bool Semantics::operator==(const Semantics& other) const {
  return speeds == other.speeds && lanes == other.lanes && priorities == other.priorities &&
         prohibited == other.prohibited && warnings == other.warnings && routings == other.routings &&
         street_names == other.street_names && parkings == other.parkings && tourists == other.tourists &&
         supplementary_times == other.supplementary_times && supplementary_allows == other.supplementary_allows &&
         supplementary_prohibits == other.supplementary_prohibits &&
         supplementary_distances == other.supplementary_distances &&
         supplementary_environments == other.supplementary_environments &&
         supplementary_explanatories == other.supplementary_explanatories;
}

bool Semantics::operator!=(const Semantics& other) const { return !(*this == other); }

}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
