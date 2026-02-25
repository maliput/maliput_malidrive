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
#include "maliput_malidrive/xodr/signal/semantics.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace signal {
namespace test {
namespace {

// Tests for Semantics::Speed
GTEST_TEST(Speed, EqualityOperator) {
  const Semantics::Speed kSpeed{Semantics::SemanticsSpeed::kMaximum, Semantics::UnitSpeed::kKmh, 50.0};

  Semantics::Speed speed = kSpeed;
  EXPECT_EQ(kSpeed, speed);

  speed.type = Semantics::SemanticsSpeed::kMinimum;
  EXPECT_NE(kSpeed, speed);
  speed = kSpeed;

  speed.unit = Semantics::UnitSpeed::kMS;
  EXPECT_NE(kSpeed, speed);
  speed = kSpeed;

  speed.value = 25.0;
  EXPECT_NE(kSpeed, speed);
}

// Tests for Semantics::Lane
GTEST_TEST(Lane, EqualityOperator) {
  const Semantics::Lane kLane{Semantics::SemanticsLane::kNoOvertakeCars};

  Semantics::Lane lane = kLane;
  EXPECT_EQ(kLane, lane);

  lane.type = Semantics::SemanticsLane::kRoundabout;
  EXPECT_NE(kLane, lane);
}

// Tests for Priority
GTEST_TEST(Priority, EqualityOperator) {
  const Semantics::Priority kPriority{Semantics::SemanticsPriority::kStop};

  Semantics::Priority priority = kPriority;
  EXPECT_EQ(kPriority, priority);

  priority.type = Semantics::SemanticsPriority::kYield;
  EXPECT_NE(kPriority, priority);
}

// Tests for Semantics::Prohibited (no fields, always equal)
GTEST_TEST(Prohibited, EqualityOperator) {
  const Semantics::Prohibited kProhibited{};

  Semantics::Prohibited prohibited = kProhibited;
  EXPECT_EQ(kProhibited, prohibited);
  EXPECT_FALSE(kProhibited != prohibited);
}

// Tests for Semantics::Warning (no fields, always equal)
GTEST_TEST(Warning, EqualityOperator) {
  const Semantics::Warning kWarning{};

  Semantics::Warning warning = kWarning;
  EXPECT_EQ(kWarning, warning);
  EXPECT_FALSE(kWarning != warning);
}

// Tests for Semantics::Routing (no fields, always equal)
GTEST_TEST(Routing, EqualityOperator) {
  const Semantics::Routing kRouting{};

  Semantics::Routing routing = kRouting;
  EXPECT_EQ(kRouting, routing);
  EXPECT_FALSE(kRouting != routing);
}

// Tests for Semantics::StreetName (no fields, always equal)
GTEST_TEST(StreetName, EqualityOperator) {
  const Semantics::StreetName kStreetName{};

  Semantics::StreetName street_name = kStreetName;
  EXPECT_EQ(kStreetName, street_name);
  EXPECT_FALSE(kStreetName != street_name);
}

// Tests for Semantics::Parking (no fields, always equal)
GTEST_TEST(Parking, EqualityOperator) {
  const Semantics::Parking kParking{};

  Semantics::Parking parking = kParking;
  EXPECT_EQ(kParking, parking);
  EXPECT_FALSE(kParking != parking);
}

// Tests for Semantics::Tourist (no fields, always equal)
GTEST_TEST(Tourist, EqualityOperator) {
  const Semantics::Tourist kTourist{};

  Semantics::Tourist tourist = kTourist;
  EXPECT_EQ(kTourist, tourist);
  EXPECT_FALSE(kTourist != tourist);
}

// Tests for Semantics::SupplementaryTime
GTEST_TEST(SupplementaryTime, EqualityOperator) {
  const Semantics::SupplementaryTime kSupplementaryTime{Semantics::SemanticsSupplementaryTime::kDay, 5.0};

  Semantics::SupplementaryTime supplementary_time = kSupplementaryTime;
  EXPECT_EQ(kSupplementaryTime, supplementary_time);

  supplementary_time.type = Semantics::SemanticsSupplementaryTime::kTime;
  EXPECT_NE(kSupplementaryTime, supplementary_time);
  supplementary_time = kSupplementaryTime;

  supplementary_time.value = 10.0;
  EXPECT_NE(kSupplementaryTime, supplementary_time);
}

// Tests for Semantics::SupplementaryAllows (no fields, always equal)
GTEST_TEST(SupplementaryAllows, EqualityOperator) {
  const Semantics::SupplementaryAllows kSupplementaryAllows{};

  Semantics::SupplementaryAllows supplementary_allows = kSupplementaryAllows;
  EXPECT_EQ(kSupplementaryAllows, supplementary_allows);
  EXPECT_FALSE(kSupplementaryAllows != supplementary_allows);
}

// Tests for Semantics::SupplementaryProhibits (no fields, always equal)
GTEST_TEST(SupplementaryProhibits, EqualityOperator) {
  const Semantics::SupplementaryProhibits kSupplementaryProhibits{};

  Semantics::SupplementaryProhibits supplementary_prohibits = kSupplementaryProhibits;
  EXPECT_EQ(kSupplementaryProhibits, supplementary_prohibits);
  EXPECT_FALSE(kSupplementaryProhibits != supplementary_prohibits);
}

// Tests for Semantics::SupplementaryDistance
GTEST_TEST(SupplementaryDistance, EqualityOperator) {
  const Semantics::SupplementaryDistance kSupplementaryDistance{Semantics::SemanticsSupplementaryDistance::kFor,
                                                                Semantics::UnitDistance::kM, 100.0};

  Semantics::SupplementaryDistance supplementary_distance = kSupplementaryDistance;
  EXPECT_EQ(kSupplementaryDistance, supplementary_distance);

  supplementary_distance.type = Semantics::SemanticsSupplementaryDistance::kIn;
  EXPECT_NE(kSupplementaryDistance, supplementary_distance);
  supplementary_distance = kSupplementaryDistance;

  supplementary_distance.unit = Semantics::UnitDistance::kKm;
  EXPECT_NE(kSupplementaryDistance, supplementary_distance);
  supplementary_distance = kSupplementaryDistance;

  supplementary_distance.value = 50.0;
  EXPECT_NE(kSupplementaryDistance, supplementary_distance);
}

// Tests for Semantics::SupplementaryEnvironment
GTEST_TEST(SupplementaryEnvironment, EqualityOperator) {
  const Semantics::SupplementaryEnvironment kSupplementaryEnvironment{
      Semantics::SemanticsSupplementaryEnvironment::kSnow};

  Semantics::SupplementaryEnvironment supplementary_environment = kSupplementaryEnvironment;
  EXPECT_EQ(kSupplementaryEnvironment, supplementary_environment);

  supplementary_environment.type = Semantics::SemanticsSupplementaryEnvironment::kRain;
  EXPECT_NE(kSupplementaryEnvironment, supplementary_environment);
}

// Tests for Semantics::SupplementaryExplanatory (no fields, always equal)
GTEST_TEST(SupplementaryExplanatory, EqualityOperator) {
  const Semantics::SupplementaryExplanatory kSupplementaryExplanatory{};

  Semantics::SupplementaryExplanatory supplementary_explanatory = kSupplementaryExplanatory;
  EXPECT_EQ(kSupplementaryExplanatory, supplementary_explanatory);
  EXPECT_FALSE(kSupplementaryExplanatory != supplementary_explanatory);
}

// Tests for Semantics
GTEST_TEST(Semantics, EqualityOperator) {
  const Semantics kSemantics{
      {Semantics::Speed{Semantics::SemanticsSpeed::kMaximum, Semantics::UnitSpeed::kKmh, 50.0}},  // speeds
      {Semantics::Lane{Semantics::SemanticsLane::kRoundabout}},                                   // lanes
      {Semantics::Priority{Semantics::SemanticsPriority::kStop}},                                 // priorities
      {Semantics::Prohibited{}},                                                                  // prohibited
      {Semantics::Warning{}},                                                                     // warnings
      {Semantics::Routing{}},                                                                     // routings
      {Semantics::StreetName{}},                                                                  // street_names
      {Semantics::Parking{}},                                                                     // parkings
      {Semantics::Tourist{}},                                                                     // tourists
      {Semantics::SupplementaryTime{Semantics::SemanticsSupplementaryTime::kDay, 5.0}},           // supplementary_times
      {Semantics::SupplementaryAllows{}},     // supplementary_allows
      {Semantics::SupplementaryProhibits{}},  // supplementary_prohibits
      {Semantics::SupplementaryDistance{Semantics::SemanticsSupplementaryDistance::kFor, Semantics::UnitDistance::kM,
                                        100.0}},  // supplementary_distances
      {Semantics::SupplementaryEnvironment{
          Semantics::SemanticsSupplementaryEnvironment::kSnow}},  // supplementary_environments
      {Semantics::SupplementaryExplanatory{}}                     // supplementary_explanatories
  };

  Semantics semantics = kSemantics;
  EXPECT_EQ(kSemantics, semantics);

  semantics.speeds[0].value = 25.0;
  EXPECT_NE(kSemantics, semantics);
  semantics = kSemantics;

  semantics.lanes[0].type = Semantics::SemanticsLane::kNoOvertakeCars;
  EXPECT_NE(kSemantics, semantics);
  semantics = kSemantics;

  semantics.priorities[0].type = Semantics::SemanticsPriority::kYield;
  EXPECT_NE(kSemantics, semantics);
  semantics = kSemantics;

  semantics.supplementary_times[0].value = 10.0;
  EXPECT_NE(kSemantics, semantics);
  semantics = kSemantics;

  semantics.supplementary_distances[0].value = 50.0;
  EXPECT_NE(kSemantics, semantics);
  semantics = kSemantics;

  semantics.supplementary_environments[0].type = Semantics::SemanticsSupplementaryEnvironment::kRain;
  EXPECT_NE(kSemantics, semantics);
}

}  // namespace
}  // namespace test
}  // namespace signal
}  // namespace xodr
}  // namespace malidrive
