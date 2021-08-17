// Copyright 2020 Toyota Research Institute
#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/function.h"

namespace malidrive {
namespace road_curve {

/// Describes a Function defined in pieces.
///
/// Queries accept p ∈ [p0, p1] with a linear tolerance.
class PiecewiseFunction : public Function {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(PiecewiseFunction)

  /// Continuity check behavior
  enum class ContinuityCheck {
    kLog = 0,  ///> Log when continuity is violated.
    kThrow,    ///> Throw when continuity is violated.
  };

  /// Constructs PiecewiseFunction from a collection of Functions.
  ///
  /// Functions are expected to be C1 continuous and C1 continuity is evaluated at
  /// the extents. Constraints can be disabled with `continuity_check`.
  ///
  /// @param functions Hold the Functions.
  /// @param tolerance Tolerance used to verify continuity.
  /// @param continuity_check Select continuity check behavior. ContinuityCheck::kThrow by default.
  ///
  /// @throws maliput::common::assertion_error When @p functions is empty.
  /// @throws maliput::common::assertion_error When @p functions has a nullptr
  ///         item.
  /// @throws maliput::common::assertion_error When two consecutive items in
  ///         @p functions are not C¹ contiguous up to @p tolerance and @p continuity_check is kThrow.
  PiecewiseFunction(std::vector<std::unique_ptr<Function>> functions, double tolerance,
                    const ContinuityCheck& continuity_check);

  /// Constructs PiecewiseFunction from a collection of Functions.
  ///
  /// Uses PiecewiseFunction() constructor using ContinuityCheck::kThrow.
  ///
  /// @param functions Hold the Functions.
  /// @param tolerance Tolerance used to verify continuity.
  PiecewiseFunction(std::vector<std::unique_ptr<Function>> functions, double tolerance);

 private:
  // Holds the interval of p values in `this` Function domain that map to one of
  // the pieces in `functions_`.
  struct FunctionInterval {
    FunctionInterval() = delete;

    // Constructs a range @f$ [`min_in`, `max_in`) @f$.
    // @throws maliput::common::assertion_error When `max_in` is not greater
    //         than `min_in`.
    FunctionInterval(double min_in, double max_in) : min(min_in), max(max_in) { MALIDRIVE_THROW_UNLESS(max >= min); }

    // Constructs a range @f$ [`x`, `x`) @f$.
    FunctionInterval(double x) : min(x), max(x) {}

    // Operator less than implementation to provide strict weak ordering to
    // std::map.
    bool operator<(const FunctionInterval& rhs) const;

    double min{};
    double max{};
  };

  // Finds the Function and the p parameter in the Function's domain to which
  // @p p maps to.
  // @throws maliput::common::assertion_error When p is not in [`p0()`, `p1()`].
  std::pair<const Function*, double> GetFunctionAndPAt(double p) const;

  double do_f(double p) const override;
  double do_f_dot(double p) const override;
  double do_f_dot_dot(double p) const override;
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return is_g1_contiguous; }

  std::vector<std::unique_ptr<Function>> functions_;
  double p0_{};
  double p1_{};
  std::map<FunctionInterval, Function*> interval_function_;
  double linear_tolerance_{};
  bool is_g1_contiguous{true};
};

}  // namespace road_curve
}  // namespace malidrive
