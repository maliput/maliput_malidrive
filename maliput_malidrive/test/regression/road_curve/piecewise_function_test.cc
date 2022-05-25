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
#include "maliput_malidrive/road_curve/piecewise_function.h"

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/test_utilities/function_stub.h"

namespace malidrive {
namespace road_curve {
namespace test {
namespace {

class PiecewiseFunctionTest : public ::testing::Test {
 public:
  const double kTolerance{1e-3};
  const bool kIsG1Contiguous{true};

  const double kFA{123.};
  const double kFDotA{1.};
  const double kFDotDotA{1.};
  const double kP0A{10.};
  const double kP1A{20.};

  const double kFB{kFA + kTolerance / 3.};
  const double kFDotB{kFDotA - kTolerance / 2.};
  const double kFDotDotB{2.};
  const double kP0B{11.};
  const double kP1B{21.};

  const double kFC{kFB - kTolerance / 2.};
  const double kFDotC{kFDotB + kTolerance / 3.};
  const double kFDotDotC{0.};
  const double kP0C{2.};
  const double kP1C{32.};

  const double kFD{kFA + 3. * kTolerance};
  const double kFDotD{kFDotA - kTolerance / 2.};
  const double kFDotDotD{2.};
  const double kP0D{13.};
  const double kP1D{23.};

  const double kFE{kFA + kTolerance / 3.};
  const double kFDotE{kFDotA - 3. * kTolerance};
  const double kFDotDotE{2.};
  const double kP0E{14.};
  const double kP1E{24.};
};

TEST_F(PiecewiseFunctionTest, ConstructorNoThrow) {
  std::vector<std::unique_ptr<Function>> functions;
  functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
  functions.push_back(std::make_unique<FunctionStub>(kFB, kFDotB, kFDotDotB, kP0B, kP1B, kIsG1Contiguous));
  functions.push_back(std::make_unique<FunctionStub>(kFC, kFDotC, kFDotDotC, kP0C, kP1C, kIsG1Contiguous));

  EXPECT_NO_THROW(PiecewiseFunction(std::move(functions), kTolerance));
}

TEST_F(PiecewiseFunctionTest, ConstructorUnmetContraints) {
  {  // Empty function vector.
    std::vector<std::unique_ptr<Function>> empty_vector;
    EXPECT_THROW(PiecewiseFunction(std::move(empty_vector), kTolerance), maliput::common::assertion_error);
  }
  {  // Negative tolerance.
    std::vector<std::unique_ptr<Function>> functions;
    functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
    EXPECT_THROW(PiecewiseFunction(std::move(functions), -kTolerance), maliput::common::assertion_error);
  }
  {  // Zero tolerance.
    std::vector<std::unique_ptr<Function>> functions;
    functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
    EXPECT_THROW(PiecewiseFunction(std::move(functions), 0.), maliput::common::assertion_error);
  }
  {  // Vector holds a nullptr.
    std::vector<std::unique_ptr<Function>> functions;
    functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
    functions.push_back(nullptr);
    functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
    EXPECT_THROW(PiecewiseFunction(std::move(functions), kTolerance), maliput::common::assertion_error);
  }
  {  // Functions are not C0.
    std::vector<std::unique_ptr<Function>> functions;
    functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
    functions.push_back(std::make_unique<FunctionStub>(kFD, kFDotD, kFDotDotD, kP0D, kP1D, kIsG1Contiguous));
    EXPECT_THROW(PiecewiseFunction(std::move(functions), kTolerance), maliput::common::assertion_error);
  }
  {  // Functions are not C1.
    std::vector<std::unique_ptr<Function>> functions;
    functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
    functions.push_back(std::make_unique<FunctionStub>(kFE, kFDotE, kFDotDotE, kP0E, kP1E, kIsG1Contiguous));
    EXPECT_THROW(PiecewiseFunction(std::move(functions), kTolerance), maliput::common::assertion_error);
  }
  {  // Functions are not C1 but continuity check only logs.
    const PiecewiseFunction::ContinuityCheck kContinuityCheckOnlyLog{PiecewiseFunction::ContinuityCheck::kLog};
    std::vector<std::unique_ptr<Function>> functions;
    functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
    functions.push_back(std::make_unique<FunctionStub>(kFE, kFDotE, kFDotDotE, kP0E, kP1E, kIsG1Contiguous));
    std::unique_ptr<Function> function{nullptr};
    ASSERT_NO_THROW(function =
                        std::make_unique<PiecewiseFunction>(std::move(functions), kTolerance, kContinuityCheckOnlyLog));
    EXPECT_FALSE(function->IsG1Contiguous());
  }
}

TEST_F(PiecewiseFunctionTest, FunctionApi) {
  const double kEpsilon = 1e-10;

  std::vector<std::unique_ptr<Function>> functions;
  functions.push_back(std::make_unique<FunctionStub>(kFA, kFDotA, kFDotDotA, kP0A, kP1A, kIsG1Contiguous));
  functions.push_back(std::make_unique<FunctionStub>(kFB, kFDotB, kFDotDotB, kP0B, kP1B, kIsG1Contiguous));
  functions.push_back(std::make_unique<FunctionStub>(kFC, kFDotC, kFDotDotC, kP0C, kP1C, kIsG1Contiguous));

  const PiecewiseFunction dut(std::move(functions), kTolerance);

  EXPECT_DOUBLE_EQ(/* kP0A */ 10., dut.p0());
  EXPECT_DOUBLE_EQ(/* kP0A + (kP1A - kP0A) + (kP1B - kP0B) + (kP1C - kP0C)*/ 60., dut.p1());

  EXPECT_TRUE(dut.IsG1Contiguous());

  EXPECT_DOUBLE_EQ(kFA, dut.f((kP1A + kP0A) / 2.));
  EXPECT_DOUBLE_EQ(kFA, dut.f(kP1A - kEpsilon));
  EXPECT_DOUBLE_EQ(kFB, dut.f(kP1A));
  EXPECT_DOUBLE_EQ(kFB, dut.f(kP1A + (kP1B - kP0B) / 2.));
  EXPECT_DOUBLE_EQ(kFB, dut.f(kP1A + kP1B - kP0B - kEpsilon));
  EXPECT_DOUBLE_EQ(kFC, dut.f(kP1A + kP1B - kP0B));
  EXPECT_DOUBLE_EQ(kFC, dut.f(kP0A + kP1A - kP0A + kP1B - kP0B + kP1C - kP0C));

  EXPECT_DOUBLE_EQ(kFDotA, dut.f_dot((kP1A + kP0A) / 2.));
  EXPECT_DOUBLE_EQ(kFDotA, dut.f_dot(kP1A - kEpsilon));
  EXPECT_DOUBLE_EQ(kFDotB, dut.f_dot(kP1A));
  EXPECT_DOUBLE_EQ(kFDotB, dut.f_dot(kP1A + (kP1B - kP0B) / 2.));
  EXPECT_DOUBLE_EQ(kFDotB, dut.f_dot(kP1A + kP1B - kP0B - kEpsilon));
  EXPECT_DOUBLE_EQ(kFDotC, dut.f_dot(kP1A + kP1B - kP0B));
  EXPECT_DOUBLE_EQ(kFDotC, dut.f_dot(kP1A + kP1B - kP0B + kP1C - kP0C));

  EXPECT_DOUBLE_EQ(kFDotDotA, dut.f_dot_dot((kP1A + kP0A) / 2.));
  EXPECT_DOUBLE_EQ(kFDotDotA, dut.f_dot_dot(kP1A - kEpsilon));
  EXPECT_DOUBLE_EQ(kFDotDotB, dut.f_dot_dot(kP1A));
  EXPECT_DOUBLE_EQ(kFDotDotB, dut.f_dot_dot(kP1A + (kP1B - kP0B) / 2.));
  EXPECT_DOUBLE_EQ(kFDotDotB, dut.f_dot_dot(kP1A + kP1B - kP0B - kEpsilon));
  EXPECT_DOUBLE_EQ(kFDotDotC, dut.f_dot_dot(kP1A + kP1B - kP0B));
  EXPECT_DOUBLE_EQ(kFDotDotC, dut.f_dot_dot(kP1A + kP1B - kP0B + kP1C - kP0C));
}

}  // namespace
}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
