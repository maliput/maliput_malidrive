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
#include "maliput_malidrive/builder/phase_provider_builder.h"

#include <memory>
#include <optional>
#include <vector>

#include "maliput/api/rules/phase.h"

namespace malidrive {
namespace builder {

using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;

std::unique_ptr<maliput::ManualPhaseProvider> PhaseProviderBuilder::operator()() const {
  auto manual_phase_provider = std::make_unique<maliput::ManualPhaseProvider>();
  for (const auto& phase_ring_id : phase_ring_book_->GetPhaseRings()) {
    const std::optional<PhaseRing> phase_ring = phase_ring_book_->GetPhaseRing(phase_ring_id);
    MALIPUT_THROW_UNLESS(phase_ring != std::nullopt);
    std::optional<Phase::Id> next_phase_id = std::nullopt;
    std::optional<double> duration_until = std::nullopt;

    const std::unordered_map<Phase::Id, Phase>& phases = phase_ring->phases();
    if (phases.empty()) continue;
    // As `phases` is an unordered map, the initial phase is randomly selected even though always the "begin" value of
    // the collection is selected.
    const Phase::Id initial_phase = phases.begin()->first;
    const std::vector<PhaseRing::NextPhase> next_phases = phase_ring->next_phases().at(initial_phase);
    if (!next_phases.empty()) {
      // Arbitrarily selects the first next phase.
      const PhaseRing::NextPhase& n = next_phases.front();
      next_phase_id = n.id;
      duration_until = n.duration_until;
    }
    manual_phase_provider->AddPhaseRing(phase_ring_id, initial_phase, next_phase_id, duration_until);
  }

  return manual_phase_provider;
}

}  // namespace builder
}  // namespace malidrive
