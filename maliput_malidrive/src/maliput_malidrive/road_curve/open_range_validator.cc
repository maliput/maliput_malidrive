// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/road_curve/open_range_validator.h"

#include <algorithm>

namespace malidrive {
namespace road_curve {

OpenRangeValidator OpenRangeValidator::GetRelativeEpsilonValidator(double min, double max, double tolerance,
                                                                   double epsilon) {
  return OpenRangeValidator{min, max, tolerance, epsilon, EpsilonUse::kRelative};
}

OpenRangeValidator OpenRangeValidator::GetAbsoluteEpsilonValidator(double min, double max, double tolerance,
                                                                   double epsilon) {
  return OpenRangeValidator{min, max, tolerance, epsilon, EpsilonUse::kAbsolute};
}

OpenRangeValidator::OpenRangeValidator(double min, double max, double tolerance, double epsilon,
                                       const EpsilonUse& epsilon_mode)
    : min_(min), max_(max), tolerance_(tolerance), epsilon_(epsilon) {
  MALIDRIVE_THROW_UNLESS(tolerance_ > 0.);

  if (epsilon_mode == EpsilonUse::kRelative) {
    // Multiplying the range by kEpsilon creates an epsilon value that is relative to the
    // length of the range. This avoids numerical errors when having a long range and a very small epsilon value
    // (~1e-14) leads to calculation that goes beyond the minimal useful digit of the double type.
    epsilon_ = (max_ - min_) * epsilon_;
  }
  MALIDRIVE_IS_IN_RANGE(epsilon_, 0., tolerance_);
  MALIDRIVE_VALIDATE((min_ + epsilon_) <= max_, maliput::common::assertion_error,
                     std::string("Open range lower bound <") + std::to_string((min_ + epsilon_)) +
                         "> is greater than <" + std::to_string(max_) + ">");
  MALIDRIVE_VALIDATE(min <= (max_ - epsilon_), maliput::common::assertion_error,
                     std::string("Open range upper bound <") + std::to_string((max_ - epsilon_)) + "> is less than <" +
                         std::to_string(min) + ">");
}

double OpenRangeValidator::operator()(double s) const {
  MALIDRIVE_IS_IN_RANGE(s, min_ - tolerance_, max_ + tolerance_);
  return std::clamp(s, min_ + epsilon_, max_ - epsilon_);
}

}  // namespace road_curve
}  // namespace malidrive
