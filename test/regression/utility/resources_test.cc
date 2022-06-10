// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet
// All rights reserved.
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
#include "utility/resources.h"

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

namespace utility {
namespace test {
namespace {

static constexpr char kMalidriveProjectSourcePath[] = DEF_PROJECT_SOURCE_DIR;

class FindResourceTest : public ::testing::Test {
 public:
  static constexpr char kMalidriveResourceEnv[] = "MALIPUT_MALIDRIVE_RESOURCE_ROOT";

  void SetUp() override { setenv(kMalidriveResourceEnv, kMalidriveProjectSourcePath, 1); }
};

TEST_F(FindResourceTest, NotFound) {
  const std::string kWrongFile{"WrongFile"};
  EXPECT_THROW(FindResource(kWrongFile), std::runtime_error);
}

TEST_F(FindResourceTest, Found) {
  const std::string kTShapeRoadXodrFile{"TShapeRoad.xodr"};
  EXPECT_NO_THROW(FindResource(kTShapeRoadXodrFile));
}

class FindResourceInEnvPathTest : public FindResourceTest {
 public:
  void SetUp() override {
    setenv(kMalidriveResourceEnv, (std::string(kMalidriveProjectSourcePath) + "/resources").c_str(), 1);
  }
};

TEST_F(FindResourceInEnvPathTest, NotFound) {
  const std::string kWrongFile{"WrongFile"};
  EXPECT_THROW(FindResourceInEnvPath(kWrongFile, FindResourceTest::kMalidriveResourceEnv), std::runtime_error);
}

TEST_F(FindResourceInEnvPathTest, Found) {
  const std::string kTShapeRoadXodrFile{"TShapeRoad.xodr"};
  EXPECT_NO_THROW(FindResourceInEnvPath(kTShapeRoadXodrFile, FindResourceTest::kMalidriveResourceEnv));
}

class FindResourceInPathTest : public ::testing::Test {
 public:
  const std::string kMalidriveResourceFolder = std::string(kMalidriveProjectSourcePath) + "/resources";
};

TEST_F(FindResourceInPathTest, NotFound) {
  const std::string kWrongFile{"WrongFile"};
  EXPECT_THROW(FindResourceInPath(kWrongFile, kMalidriveResourceFolder), std::runtime_error);
}

TEST_F(FindResourceInPathTest, Found) {
  const std::string kTShapeRoadXodrFile{"TShapeRoad.xodr"};
  EXPECT_NO_THROW(FindResourceInPath(kTShapeRoadXodrFile, kMalidriveResourceFolder));
}

}  // namespace
}  // namespace test
}  // namespace utility
