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

#include "maliput_malidrive/xodr/signal/signal.h"

namespace malidrive {
namespace xodr {
namespace signal {

/// Holds the values of an XODR Sign.
/// For example, a XML node describing a XODR's sign:
/// <signal s="4.0"
///         t="1.0"
///         id="534"
///         name="board"
///         dynamic="no"
///         orientation="+"
///         zOffset="5.00"
///         country="OpenDRIVE"
///         type="staticBoard"
///         subtype="-1"
///         hOffset="0"
///         pitch="0"
///         roll="0"
///         height="2.0"
///         width="1.5">
///     <validity from="-2" to="-2"/>
///     <staticBoard>
///         <sign id="535" Country="DE" type="274" subtype="60" countryRevision="2017" v="-0.5" z="1.5" width="0.5"
///         height="0.5" value="60" unit="km/h">
///             <validity from="-2" to="-2"/>
///             <signalDependency id ="536"/>
///             <signalDependency id ="537"/>
///         </sign>
///         <sign id="536" Country="DE" type="1010" subtype="51" countryRevision="2017" v="-0.75" z="0.9" width="0.420"
///         height="0.231"/> <sign id="537" Country="DE" type="1040" subtype="30" countryRevision="2017" v="-0.75"
///         z="0.6" width="0.420" height="0.231" value="22000600"/> <sign id="538" Country="DE" type="1012" subtype="36"
///         countryRevision="2017" v="-0.75" z="0.3" width="0.420" height="0.231"/> <sign id="539" Country="DE"
///         type="274" subtype="80" countryRevision="2017" v="0.75" z="1.5" width="0.420" height="0.231" value="100"
///         unit="km/h">
///             <signalDependency id ="540" />
///         </sign>
///         <sign id="540" Country="DE" type="1040" subtype="30" countryRevision="2017" v="-0.75" z="0.6" width="0.420"
///         height="0.231" value="22000600"/> <sign id="541" Country="DE" type="1012" subtype="36"
///         countryRevision="2017" v="-0.75" z="0.3" width="0.420" height="0.231"/>
///     </staticBoard>
/// </signal>
struct Sign : public Signal {
  static constexpr const char* kSignTag = "sign";
  static constexpr const char* kV = "v";
  static constexpr const char* kZ = "z";

  /// Local v-coordinate of the sign on a board.
  double v{};
  /// Local z-coordinate of the sign on a board.
  double z{};

  bool operator==(const Sign& other) const;
  bool operator!=(const Sign& other) const;
};

}  // namespace signal
}  // namespace xodr
}  // namespace malidrive