// Copyright 2019 Toyota Research Institute
#pragma once

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace road_curve {

/// Functor to validate if a number is within [min, max] and considering
/// assuming a tolerance. This tolerance extends the range to be
/// [min - tolerance, max + tolerance].
/// The functor not only validates that number is within the range, but also
/// adapts it to be in the interval (min, max) where the difference between the
/// close and open range is epsilon.
/// This functor is motivated to wrap MALIDRIVE_DEMAND() and
/// MALIDRIVE_IS_IN_RANGE() calls plus a std::clamp() to interface with ODRM.
class OpenRangeValidator {
 public:
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OpenRangeValidator)

  /// Creates a OpenRangeValidator instance that uses a relative epsilon to validate the values.
  /// This relative epsilon is computed by multiplying `epsilon` by the range.
  /// @param min lower extreme of the range.
  /// @param max upper extreme of the range.
  /// @param tolerance is the range extension to be accepted.
  /// @param epsilon is the relative minimum difference that separates a number within
  ///                range to be distinct from extremes. It is relative to the range.
  /// @returns A OpenRangeValidator instance.
  /// @throws maliput::common::assertion_error When `tolerance` is non positive.
  /// @throws maliput::common::assertion_error When `epsilon` is not in
  ///         [0, tolerance].
  /// @throws maliput::common::assertion_error When `min` + `epsilon` > `max` or
  ///         `max` - `epsilon` < `min`
  static OpenRangeValidator GetRelativeEpsilonValidator(double min, double max, double tolerance, double epsilon);

  /// Creates a OpenRangeValidator instance that uses a `epsilon` as the absolute epsilon to validate the values.
  /// @param min lower extreme of the range.
  /// @param max upper extreme of the range.
  /// @param tolerance is the range extension to be accepted.
  /// @param epsilon is minimum difference that separates a number within the
  ///        range to be distinct from range extremes (`min` and `max`).
  /// @returns A OpenRangeValidator instance.
  /// @throws maliput::common::assertion_error When `tolerance` is non positive.
  /// @throws maliput::common::assertion_error When `epsilon` is not in
  ///         [0, tolerance].
  /// @throws maliput::common::assertion_error When `min` + `epsilon` > `max` or
  ///         `max` - `epsilon` < `min`
  static OpenRangeValidator GetAbsoluteEpsilonValidator(double min, double max, double tolerance, double epsilon);

  OpenRangeValidator() = delete;

  /// Evaluates whether `s` is in range or not.
  /// @returns `s` when it is within the open range. If `s` is equal to either
  /// range extremes or the difference to them is less or equal to tolerance, it
  /// returns the closest open range value. Otherwise, it
  /// @throws maliput::common::assertion_error.
  double operator()(double s) const;

 private:
  // The provided epsilon can be used relatively or absolutely.
  enum class EpsilonUse { kAbsolute = 0, kRelative };

  // Constructs the functor.
  //
  // @param min lower extreme of the range.
  // @param max upper extreme of the range.
  // @param tolerance is the range extension to be accepted.
  // @param epsilon is minimum difference that separates a number within the
  //        range to be distinct from range extremes (`min` and `max`).
  // @param epsilon_mode selects how `epsilon` will be used: absolute or relative.
  // @throws maliput::common::assertion_error When `tolerance` is non positive.
  // @throws maliput::common::assertion_error When `epsilon` is not in
  //         [0, tolerance].
  // @throws maliput::common::assertion_error When `min` + `epsilon` > `max` or
  //         `max` - `epsilon` < `min`
  OpenRangeValidator(double min, double max, double tolerance, double epsilon, const EpsilonUse& epsilon_mode);

  double min_{};
  double max_{};
  double tolerance_{};
  double epsilon_{};
};

}  // namespace road_curve
}  // namespace malidrive
