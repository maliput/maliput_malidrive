// Copyright 2022 Toyota Research Institute
#include <memory>

#include <maliput/api/rules/phase_ring_book.h>
#include <maliput/base/manual_phase_provider.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Functor to build a ManualPhaseProvider.
class PhaseProviderBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseProviderBuilder)

  /// Constructs a PhaseProviderBuilder.
  ///
  /// @param phase_ring_book A PhaseRingBook to feed the
  ///        PhaseProviderBuilder. It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When `phase_ring_book` is nullptr.
  explicit PhaseProviderBuilder(const maliput::api::rules::PhaseRingBook* phase_ring_book)
      : phase_ring_book_(phase_ring_book) {
    MALIDRIVE_THROW_UNLESS(phase_ring_book_ != nullptr);
  }

  /// Builds a maliput::ManualPhaseProvider.
  std::unique_ptr<maliput::ManualPhaseProvider> operator()() const;

 private:
  const maliput::api::rules::PhaseRingBook* phase_ring_book_{};
};

}  // namespace builder
}  // namespace malidrive
