// Copyright 2022 Toyota Research Institute
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
