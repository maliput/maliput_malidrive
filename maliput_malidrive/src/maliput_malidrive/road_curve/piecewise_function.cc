// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/road_curve/piecewise_function.h"

#include <cmath>

#include "maliput_malidrive/road_curve/open_range_validator.h"

namespace malidrive {
namespace road_curve {
namespace {

struct ContiguityChecker {
  ContiguityChecker() = delete;

  explicit ContiguityChecker(double tolerance_in) : tolerance(tolerance_in) { MALIDRIVE_THROW_UNLESS(tolerance > 0.); }

  // Evaluates whether `lhs` is G1 contiguous with `rhs`.
  // @throws maliput::common::assertion_error When `lhs` is not G1 contiguous with `rhs`.
  void operator()(const Function* lhs, const Function* rhs) const {
    const double f_distance = std::abs(lhs->f(lhs->p1()) - rhs->f(rhs->p0()));
    MALIDRIVE_VALIDATE(f_distance <= tolerance, maliput::common::assertion_error,
                       std::string("Error when constructing piecewise function. Endpoint distance is <") +
                           std::to_string(f_distance) +
                           "> which is greater than tolerance: " + std::to_string(tolerance) + ">.");
    const double f_dot_distance = std::abs(lhs->f_dot(lhs->p1()) - rhs->f_dot(rhs->p0()));
    MALIDRIVE_VALIDATE(f_dot_distance <= tolerance, maliput::common::assertion_error,
                       std::string("Error when constructing piecewise function. Endpoint derivative distance is <") +
                           std::to_string(f_dot_distance) +
                           "> which is greater than tolerance: " + std::to_string(tolerance) + ">.");
  }

  double tolerance{};
};

}  // namespace

PiecewiseFunction::PiecewiseFunction(std::vector<std::unique_ptr<Function>> functions, double tolerance,
                                     bool no_contiguity_check)
    : functions_(std::move(functions)), linear_tolerance_(tolerance) {
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0.);
  MALIDRIVE_THROW_UNLESS(!functions_.empty());

  MALIDRIVE_THROW_UNLESS(functions_[0].get() != nullptr);
  p0_ = functions_[0]->p0();
  double p = p0_;
  Function* previous_function{nullptr};
  const ContiguityChecker checker(linear_tolerance_);
  for (const auto& function : functions_) {
    MALIDRIVE_THROW_UNLESS(function.get() != nullptr);
    MALIDRIVE_THROW_UNLESS(function->IsG1Contiguous());
    if (previous_function != nullptr && !no_contiguity_check) {
      checker(previous_function, function.get());
    }
    const double delta_p = function->p1() - function->p0();
    interval_function_[FunctionInterval(p, p + delta_p)] = function.get();
    p += delta_p;
    previous_function = function.get();
  }
  p1_ = p;
}

std::pair<const Function*, double> PiecewiseFunction::GetFunctionAndPAt(double p) const {
  p = OpenRangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_, Function::kEpsilon)(p);

  auto search_it = interval_function_.find(FunctionInterval(p));
  if (search_it == interval_function_.end()) {
    if (p != p1_) {
      MALIDRIVE_THROW_MESSAGE(std::string("p = ") + std::to_string(p) +
                              std::string(" doesn't match with any Function interval."));
    } else {
      search_it = --interval_function_.end();
    }
  }
  const Function* function = search_it->second;
  const FunctionInterval interval = search_it->first;

  const double p_result = function->p0() + p - interval.min;

  return {function, p_result};
}

double PiecewiseFunction::do_f(double p) const {
  p = OpenRangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_, Function::kEpsilon)(p);
  const std::pair<const Function*, double> function_p = GetFunctionAndPAt(p);
  return function_p.first->f(function_p.second);
}

double PiecewiseFunction::do_f_dot(double p) const {
  p = OpenRangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_, Function::kEpsilon)(p);
  const std::pair<const Function*, double> function_p = GetFunctionAndPAt(p);
  return function_p.first->f_dot(function_p.second);
}

double PiecewiseFunction::do_f_dot_dot(double p) const {
  p = OpenRangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_, Function::kEpsilon)(p);
  const std::pair<const Function*, double> function_p = GetFunctionAndPAt(p);
  return function_p.first->f_dot_dot(function_p.second);
}

bool PiecewiseFunction::FunctionInterval::operator<(const FunctionInterval& rhs) const {
  if (min < rhs.min) {
    return max <= rhs.max ? true : false;
  } else {
    return false;
  }
}

}  // namespace road_curve
}  // namespace malidrive
