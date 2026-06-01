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
#include "maliput_malidrive/traffic_control_device/parser.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <gtest/gtest.h>
#include <maliput/common/maliput_throw.h>

#include "maliput_malidrive/traffic_control_device/traffic_control_device_database_loader.h"
#include "utility/resources.h"

namespace malidrive {
namespace traffic_control_device {
namespace test {
namespace {

static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

constexpr double kTolerance = 1e-6;

const char kTrafficControlDeviceDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
      subtype: "-1"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_light
      description: "Standard three-bulb vertical traffic light"
      bulbs:
        - id: "RedBulb"
          position_traffic_light: [0.0, 0.0, 0.4]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Red"
          type: "Round"
          states: ["Off", "On", "Blinking"]
          bounding_box:
            p_min: [-0.0889, -0.1778, -0.1778]
            p_max: [0.0889, 0.1778, 0.1778]
        - id: "YellowBulb"
          position_traffic_light: [0.0, 0.0, 0.0]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Yellow"
          type: "Round"
          states: ["Off", "On", "Blinking"]
        - id: "GreenBulb"
          position_traffic_light: [0.0, 0.0, -0.4]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Green"
          type: "Round"
          states: ["Off", "On"]
      rule_states:
        - conditions:
            - bulb_id: "RedBulb"
              state: "On"
            - bulb_id: "YellowBulb"
              state: "Off"
            - bulb_id: "GreenBulb"
              state: "Off"
          value: "Stop"
        - conditions:
            - bulb_id: "RedBulb"
              state: "Off"
            - bulb_id: "YellowBulb"
              state: "Off"
            - bulb_id: "GreenBulb"
              state: "On"
          value: "Go"
  - odr_representation:
      type: "1000011"
      subtype: "10"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_light
      description: "Traffic light with arrow"
      bulbs:
        - id: "GreenArrow"
          position_traffic_light: [0.0, 0.0, 0.0]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Green"
          type: "Arrow"
          states: ["Off", "On"]
          arrow_orientation_rad: 1.5707963267948966
          initial_state: "On"
      rule_states:
        - conditions:
            - bulb_id: "GreenArrow"
              state: "On"
          value: "Go"
  - odr_representation:
      type: "default_bulb_state"
      subtype: null
      country: null
      country_revision: null
    properties:
      device_type: traffic_light
      description: "Signal with default bulb states and bounding box"
      bulbs:
        - id: "DefaultBulb"
          position_traffic_light: [0.0, 0.0, 0.0]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Red"
          type: "Round"
      rule_states: []
  - odr_representation:
      type: "206"
      subtype: "30"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_sign
      device_semantics: no_overtaking
      description: "No overtaking sign"
      bulbs: []
      rule_states:
        - conditions: []
          value: "Stop"
)";

const char kRepeatedTrafficControlDeviceDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1234567"
      subtype: "11"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_light
      description: "Single red bulb traffic light"
      bulbs:
        - id: "RedBulb"
          position_traffic_light: [0.0, 0.0, 0.4]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Red"
          type: "Round"
          states: ["Off", "On", "Blinking"]
          bounding_box:
            p_min: [-0.0889, -0.1778, -0.1778]
            p_max: [0.0889, 0.1778, 0.1778]
      rule_states:
        - conditions:
            - bulb_id: "RedBulb"
              state: "On"
          value: "Stop"
        - conditions:
            - bulb_id: "RedBulb"
              state: "Off"
          value: "Go"
  - odr_representation:
      type: "1234567"
      subtype: "11"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_light
      description: "Single green bulb traffic light"
      bulbs:
        - id: "GreenBulb"
          position_traffic_light: [0.0, 0.0, 0.4]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Green"
          type: "Round"
          states: ["Off", "On", "Blinking"]
          bounding_box:
            p_min: [-0.0889, -0.1778, -0.1778]
            p_max: [0.0889, 0.1778, 0.1778]
      rule_states:
        - conditions:
            - bulb_id: "GreenBulb"
              state: "On"
          value: "Go"
        - conditions:
            - bulb_id: "GreenBulb"
              state: "Off"
          value: "Stop"
)";

const char kCaseInsensitiveBulbEnumsDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "case_insensitive_bulb_enums"
      subtype: null
      country: null
      country_revision: null
    properties:
      device_type: traffic_light
      description: "Traffic light with case-insensitive bulb enums"
      bulbs:
        - id: "ArrowLeftBulb"
          position_traffic_light: [0.0, 0.0, 0.2]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "red"
          type: "arrow_left"
          states: ["off", "On", "BLINKING"]
          initial_state: "bLiNkInG"
        - id: "UTurnBulb"
          position_traffic_light: [0.0, 0.0, -0.2]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "GrEeN"
          type: "UtUrNlEfT"
          states: ["OFF"]
      rule_states:
        - conditions:
            - bulb_id: "ArrowLeftBulb"
              state: "blinking"
            - bulb_id: "UTurnBulb"
              state: "off"
          value: "ProceedWithCaution"
)";

const char kBulbEnumVariantsDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "bulb_enum_variants"
      subtype: null
      country: null
      country_revision: null
    properties:
      device_type: traffic_light
      description: "Traffic light with bulb enum variants"
      bulbs:
        - id: "SnakeCaseBulb"
          position_traffic_light: [0.0, 0.0, 0.5]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "red"
          type: "arrow_left"
          states: ["off", "on", "blinking"]
        - id: "PascalCaseArrowBulb"
          position_traffic_light: [0.0, 0.0, 0.3]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Yellow"
          type: "ArrowLeft"
          states: ["Off"]
          initial_state: "Off"
        - id: "PascalCaseUTurnBulb"
          position_traffic_light: [0.0, 0.0, 0.1]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "Green"
          type: "UTurnLeft"
          states: ["On"]
          initial_state: "On"
        - id: "CaseInsensitiveBulb"
          position_traffic_light: [0.0, 0.0, -0.1]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "yellow"
          type: "Arrow_Left"
          states: ["BLINKING"]
          initial_state: "OFF"
        - id: "SnakeCaseUTurnBulb"
          position_traffic_light: [0.0, 0.0, -0.3]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "green"
          type: "u_turn_left"
          states: ["off"]
          initial_state: "on"
        - id: "DontWalkBulb"
          position_traffic_light: [0.0, 0.0, -0.5]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: "RED"
          type: "dont_walk"
          states: ["on"]
          initial_state: "on"
      rule_states:
        - conditions:
            - bulb_id: "SnakeCaseBulb"
              state: "blinking"
            - bulb_id: "PascalCaseArrowBulb"
              state: "Off"
            - bulb_id: "PascalCaseUTurnBulb"
              state: "On"
            - bulb_id: "CaseInsensitiveBulb"
              state: "off"
            - bulb_id: "SnakeCaseUTurnBulb"
              state: "on"
            - bulb_id: "DontWalkBulb"
              state: "ON"
          value: "VariantState"
)";

// Returns a pointer to the first definition whose fingerprint equals @p fp, or nullptr if not found.
const TrafficControlDeviceDefinition* FindDefinition(const std::vector<TrafficControlDeviceDefinition>& defs,
                                                     const TrafficControlDeviceFingerprint& fp) {
  const auto it = std::find_if(defs.begin(), defs.end(),
                               [&fp](const TrafficControlDeviceDefinition& d) { return d.fingerprint == fp; });
  return it != defs.end() ? &(*it) : nullptr;
}

class TrafficControlDeviceParserTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    signal_definitions_ = TrafficControlDeviceParser::LoadFromString(kTrafficControlDeviceDb);
  }
  static std::vector<TrafficControlDeviceDefinition> signal_definitions_;
};

// Needed to allocate memory for static member variable.
std::vector<TrafficControlDeviceDefinition> TrafficControlDeviceParserTest::signal_definitions_;

TEST_F(TrafficControlDeviceParserTest, LoadFromString) {
  EXPECT_EQ(signal_definitions_.size(), 4);
  const auto& standard_fingerprint = TrafficControlDeviceFingerprint{
      .type = "1000001",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const auto& arrow_fingerprint = TrafficControlDeviceFingerprint{
      .type = "1000011",
      .subtype = "10",
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  EXPECT_NE(FindDefinition(signal_definitions_, standard_fingerprint), nullptr);
  EXPECT_NE(FindDefinition(signal_definitions_, arrow_fingerprint), nullptr);
}

TEST_F(TrafficControlDeviceParserTest, ValidateTrafficSign) {
  const TrafficControlDeviceFingerprint sign_fingerprint{
      .type = "206",
      .subtype = "30",
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const auto* dut_ptr = FindDefinition(signal_definitions_, sign_fingerprint);
  ASSERT_NE(dut_ptr, nullptr);
  const auto& dut = *dut_ptr;

  EXPECT_EQ(dut.fingerprint.type, "206");
  EXPECT_EQ(dut.fingerprint.subtype, "30");
  EXPECT_EQ(dut.fingerprint.country, "OpenDRIVE");
  EXPECT_EQ(dut.description, "No overtaking sign");
  EXPECT_EQ(dut.device_type, traffic_control_device::TrafficControlDeviceType::kTrafficSign);
  EXPECT_EQ(dut.device_semantics, "no_overtaking");

  EXPECT_EQ(dut.bulbs.size(), 0);
  EXPECT_EQ(dut.rule_states.size(), 1);
  EXPECT_EQ(dut.rule_states[0].rule_value, "Stop");
  EXPECT_EQ(dut.rule_states[0].bulb_conditions.size(), 0);
}

TEST_F(TrafficControlDeviceParserTest, ValidateSignalType1000001) {
  const TrafficControlDeviceFingerprint fingerprint{
      .type = "1000001",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const auto* dut_ptr = FindDefinition(signal_definitions_, fingerprint);
  ASSERT_NE(dut_ptr, nullptr);
  const auto& dut = *dut_ptr;

  EXPECT_EQ(dut.fingerprint.type, "1000001");
  EXPECT_EQ(dut.fingerprint.subtype, std::nullopt);
  EXPECT_EQ(dut.fingerprint.country, "OpenDRIVE");
  EXPECT_EQ(dut.description, "Standard three-bulb vertical traffic light");
  EXPECT_EQ(dut.device_type, traffic_control_device::TrafficControlDeviceType::kTrafficLight);
  EXPECT_FALSE(dut.device_semantics.has_value());

  EXPECT_EQ(dut.bulbs.size(), 3);

  // Check red bulb
  const auto& red_bulb = dut.bulbs[0];
  EXPECT_EQ(red_bulb.id, "RedBulb");
  EXPECT_EQ(red_bulb.color, maliput::api::rules::BulbColor::kRed);
  EXPECT_EQ(red_bulb.type, maliput::api::rules::BulbType::kRound);
  EXPECT_NEAR(red_bulb.position_traffic_light.x(), 0.0, kTolerance);
  EXPECT_NEAR(red_bulb.position_traffic_light.z(), 0.4, kTolerance);
  EXPECT_EQ(red_bulb.states.size(), 3);

  // Check rule conditions
  EXPECT_EQ(dut.rule_states.size(), 2);
  EXPECT_EQ(dut.rule_states[0].rule_value, "Stop");
  EXPECT_EQ(dut.rule_states[0].bulb_conditions.size(), 3);
  EXPECT_EQ(dut.rule_states[1].rule_value, "Go");
  EXPECT_EQ(dut.rule_states[1].bulb_conditions.size(), 3);
}

TEST_F(TrafficControlDeviceParserTest, ValidateArrowSignal) {
  const TrafficControlDeviceFingerprint arrow_fingerprint{
      .type = "1000011",
      .subtype = "10",
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const auto* dut_ptr = FindDefinition(signal_definitions_, arrow_fingerprint);
  ASSERT_NE(dut_ptr, nullptr);
  const auto& dut = *dut_ptr;

  EXPECT_EQ(dut.fingerprint.type, "1000011");
  EXPECT_EQ(dut.fingerprint.subtype, "10");
  EXPECT_EQ(dut.description, "Traffic light with arrow");
  EXPECT_EQ(dut.device_type, traffic_control_device::TrafficControlDeviceType::kTrafficLight);
  EXPECT_FALSE(dut.device_semantics.has_value());

  EXPECT_EQ(dut.bulbs.size(), 1);

  const auto& arrow_bulb = dut.bulbs[0];
  EXPECT_EQ(arrow_bulb.type, maliput::api::rules::BulbType::kArrow);
  EXPECT_TRUE(arrow_bulb.arrow_orientation_rad.has_value());
  EXPECT_NEAR(arrow_bulb.arrow_orientation_rad.value(), 1.570796, kTolerance);
  ASSERT_TRUE(arrow_bulb.initial_state.has_value());
  EXPECT_EQ(arrow_bulb.initial_state.value(), maliput::api::rules::BulbState::kOn);
}

GTEST_TEST(TrafficControlDeviceParserBulbEnumsTest, LoadCaseInsensitiveBulbEnums) {
  const auto definitions = TrafficControlDeviceParser::LoadFromString(kCaseInsensitiveBulbEnumsDb);
  ASSERT_EQ(definitions.size(), 1);

  const auto& dut = definitions.front();
  ASSERT_EQ(dut.bulbs.size(), 2);
  ASSERT_EQ(dut.rule_states.size(), 1);

  const auto& arrow_left_bulb = dut.bulbs[0];
  EXPECT_EQ(arrow_left_bulb.color, maliput::api::rules::BulbColor::kRed);
  EXPECT_EQ(arrow_left_bulb.type, maliput::api::rules::BulbType::kArrowLeft);
  EXPECT_EQ(arrow_left_bulb.states.size(), 3);
  EXPECT_EQ(arrow_left_bulb.states[0], maliput::api::rules::BulbState::kOff);
  EXPECT_EQ(arrow_left_bulb.states[1], maliput::api::rules::BulbState::kOn);
  EXPECT_EQ(arrow_left_bulb.states[2], maliput::api::rules::BulbState::kBlinking);
  ASSERT_TRUE(arrow_left_bulb.initial_state.has_value());
  EXPECT_EQ(arrow_left_bulb.initial_state.value(), maliput::api::rules::BulbState::kBlinking);

  const auto& u_turn_bulb = dut.bulbs[1];
  EXPECT_EQ(u_turn_bulb.color, maliput::api::rules::BulbColor::kGreen);
  EXPECT_EQ(u_turn_bulb.type, maliput::api::rules::BulbType::kUTurnLeft);
  ASSERT_EQ(u_turn_bulb.states.size(), 1);
  EXPECT_EQ(u_turn_bulb.states[0], maliput::api::rules::BulbState::kOff);

  const auto& rule_state = dut.rule_states[0];
  ASSERT_EQ(rule_state.bulb_conditions.size(), 2);
  EXPECT_EQ(rule_state.bulb_conditions[0].state, maliput::api::rules::BulbState::kBlinking);
  EXPECT_EQ(rule_state.bulb_conditions[1].state, maliput::api::rules::BulbState::kOff);
}

GTEST_TEST(TrafficControlDeviceParserBulbEnumsTest, LoadSnakeCasePascalCaseAndCaseInsensitiveBulbEnums) {
  const auto definitions = TrafficControlDeviceParser::LoadFromString(kBulbEnumVariantsDb);
  ASSERT_EQ(definitions.size(), 1);

  const auto& dut = definitions.front();
  ASSERT_EQ(dut.bulbs.size(), 6);
  ASSERT_EQ(dut.rule_states.size(), 1);

  const auto& snake_case_bulb = dut.bulbs[0];
  EXPECT_EQ(snake_case_bulb.color, maliput::api::rules::BulbColor::kRed);
  EXPECT_EQ(snake_case_bulb.type, maliput::api::rules::BulbType::kArrowLeft);
  ASSERT_EQ(snake_case_bulb.states.size(), 3);
  EXPECT_EQ(snake_case_bulb.states[0], maliput::api::rules::BulbState::kOff);
  EXPECT_EQ(snake_case_bulb.states[1], maliput::api::rules::BulbState::kOn);
  EXPECT_EQ(snake_case_bulb.states[2], maliput::api::rules::BulbState::kBlinking);
  EXPECT_FALSE(snake_case_bulb.initial_state.has_value());

  const auto& pascal_case_arrow_bulb = dut.bulbs[1];
  EXPECT_EQ(pascal_case_arrow_bulb.color, maliput::api::rules::BulbColor::kYellow);
  EXPECT_EQ(pascal_case_arrow_bulb.type, maliput::api::rules::BulbType::kArrowLeft);
  ASSERT_TRUE(pascal_case_arrow_bulb.initial_state.has_value());
  EXPECT_EQ(pascal_case_arrow_bulb.initial_state.value(), maliput::api::rules::BulbState::kOff);

  const auto& pascal_case_u_turn_bulb = dut.bulbs[2];
  EXPECT_EQ(pascal_case_u_turn_bulb.color, maliput::api::rules::BulbColor::kGreen);
  EXPECT_EQ(pascal_case_u_turn_bulb.type, maliput::api::rules::BulbType::kUTurnLeft);
  ASSERT_TRUE(pascal_case_u_turn_bulb.initial_state.has_value());
  EXPECT_EQ(pascal_case_u_turn_bulb.initial_state.value(), maliput::api::rules::BulbState::kOn);

  const auto& case_insensitive_bulb = dut.bulbs[3];
  EXPECT_EQ(case_insensitive_bulb.color, maliput::api::rules::BulbColor::kYellow);
  EXPECT_EQ(case_insensitive_bulb.type, maliput::api::rules::BulbType::kArrowLeft);
  ASSERT_EQ(case_insensitive_bulb.states.size(), 1);
  EXPECT_EQ(case_insensitive_bulb.states[0], maliput::api::rules::BulbState::kBlinking);
  ASSERT_TRUE(case_insensitive_bulb.initial_state.has_value());
  EXPECT_EQ(case_insensitive_bulb.initial_state.value(), maliput::api::rules::BulbState::kOff);

  const auto& snake_case_u_turn_bulb = dut.bulbs[4];
  EXPECT_EQ(snake_case_u_turn_bulb.color, maliput::api::rules::BulbColor::kGreen);
  EXPECT_EQ(snake_case_u_turn_bulb.type, maliput::api::rules::BulbType::kUTurnLeft);
  ASSERT_TRUE(snake_case_u_turn_bulb.initial_state.has_value());
  EXPECT_EQ(snake_case_u_turn_bulb.initial_state.value(), maliput::api::rules::BulbState::kOn);

  const auto& dont_walk_bulb = dut.bulbs[5];
  EXPECT_EQ(dont_walk_bulb.color, maliput::api::rules::BulbColor::kRed);
  EXPECT_EQ(dont_walk_bulb.type, maliput::api::rules::BulbType::kDontWalk);
  ASSERT_TRUE(dont_walk_bulb.initial_state.has_value());
  EXPECT_EQ(dont_walk_bulb.initial_state.value(), maliput::api::rules::BulbState::kOn);

  const auto& rule_state = dut.rule_states[0];
  ASSERT_EQ(rule_state.bulb_conditions.size(), 6);
  EXPECT_EQ(rule_state.bulb_conditions[0].state, maliput::api::rules::BulbState::kBlinking);
  EXPECT_EQ(rule_state.bulb_conditions[1].state, maliput::api::rules::BulbState::kOff);
  EXPECT_EQ(rule_state.bulb_conditions[2].state, maliput::api::rules::BulbState::kOn);
  EXPECT_EQ(rule_state.bulb_conditions[3].state, maliput::api::rules::BulbState::kOff);
  EXPECT_EQ(rule_state.bulb_conditions[4].state, maliput::api::rules::BulbState::kOn);
  EXPECT_EQ(rule_state.bulb_conditions[5].state, maliput::api::rules::BulbState::kOn);
}

TEST_F(TrafficControlDeviceParserTest, DefaultBulbStatesAndBoundingBox) {
  const TrafficControlDeviceFingerprint default_fingerprint{
      .type = "default_bulb_state",
      .subtype = std::nullopt,
      .country = std::nullopt,
      .country_revision = std::nullopt,
  };
  const auto* dut_ptr = FindDefinition(signal_definitions_, default_fingerprint);
  ASSERT_NE(dut_ptr, nullptr);
  const auto& dut = *dut_ptr;

  EXPECT_EQ(dut.fingerprint.type, "default_bulb_state");
  EXPECT_EQ(dut.description, "Signal with default bulb states and bounding box");
  EXPECT_EQ(dut.device_type, traffic_control_device::TrafficControlDeviceType::kTrafficLight);

  EXPECT_EQ(dut.bulbs.size(), 1);

  const auto& default_bulb = dut.bulbs[0];
  EXPECT_EQ(default_bulb.id, "DefaultBulb");
  EXPECT_EQ(default_bulb.color, maliput::api::rules::BulbColor::kRed);
  EXPECT_EQ(default_bulb.type, maliput::api::rules::BulbType::kRound);

  // Verify default states.
  EXPECT_EQ(default_bulb.states.size(), 2);
  EXPECT_EQ(default_bulb.states[0], maliput::api::rules::BulbState::kOff);
  EXPECT_EQ(default_bulb.states[1], maliput::api::rules::BulbState::kOn);

  // Verify default bounding box.
  EXPECT_NEAR(default_bulb.bounding_box.p_BMin.x(), -0.0889, kTolerance);
  EXPECT_NEAR(default_bulb.bounding_box.p_BMin.y(), -0.1778, kTolerance);
  EXPECT_NEAR(default_bulb.bounding_box.p_BMin.z(), -0.1778, kTolerance);
  EXPECT_NEAR(default_bulb.bounding_box.p_BMax.x(), 0.0889, kTolerance);
  EXPECT_NEAR(default_bulb.bounding_box.p_BMax.y(), 0.1778, kTolerance);
  EXPECT_NEAR(default_bulb.bounding_box.p_BMax.z(), 0.1778, kTolerance);
}

GTEST_TEST(RepeatedTrafficControlDeviceParserTest, RepeatedFingerprintThrows) {
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kRepeatedTrafficControlDeviceDb),
               maliput::common::road_network_description_parser_error);
}

GTEST_TEST(TrafficControlDeviceYamlFileParserTest, LoadFromFile) {
  const std::string yaml_file_path = utility::FindResourceInPath(
      "traffic_control_device_db/traffic_control_device_db_example.yaml", kMalidriveResourceFolder);

  std::ifstream yaml_file(yaml_file_path);
  std::stringstream buffer;
  buffer << yaml_file.rdbuf();
  const std::string yaml_content = buffer.str();
  yaml_file.close();

  // Load from file and load from the file content string.
  const auto traffic_control_definitions_from_file = TrafficControlDeviceParser::LoadFromFile(yaml_file_path);
  const auto traffic_control_definitions_from_string = TrafficControlDeviceParser::LoadFromString(yaml_content);

  EXPECT_EQ(traffic_control_definitions_from_file, traffic_control_definitions_from_string);
}

GTEST_TEST(BulbStateConditionEqualityOperatorTest, EqualConditions) {
  const BulbStateCondition condition1{
      .bulb_id = "RedBulb",
      .state = maliput::api::rules::BulbState::kOn,
  };
  const BulbStateCondition condition2{
      .bulb_id = "RedBulb",
      .state = maliput::api::rules::BulbState::kOn,
  };
  EXPECT_EQ(condition1, condition2);
  EXPECT_TRUE(condition1 == condition2);
}

GTEST_TEST(BulbStateConditionEqualityOperatorTest, DifferentBulbId) {
  const BulbStateCondition condition1{
      .bulb_id = "RedBulb",
      .state = maliput::api::rules::BulbState::kOn,
  };
  const BulbStateCondition condition2{
      .bulb_id = "GreenBulb",
      .state = maliput::api::rules::BulbState::kOn,
  };
  EXPECT_NE(condition1, condition2);
  EXPECT_TRUE(condition1 != condition2);
}

GTEST_TEST(BulbStateConditionEqualityOperatorTest, DifferentState) {
  const BulbStateCondition condition1{
      .bulb_id = "RedBulb",
      .state = maliput::api::rules::BulbState::kOn,
  };
  const BulbStateCondition condition2{
      .bulb_id = "RedBulb",
      .state = maliput::api::rules::BulbState::kOff,
  };
  EXPECT_NE(condition1, condition2);
  EXPECT_TRUE(condition1 != condition2);
}

GTEST_TEST(RuleStateEqualityOperatorTest, EqualRuleStates) {
  const RuleState rule_state1{
      .bulb_conditions =
          {
              BulbStateCondition{
                  .bulb_id = "RedBulb",
                  .state = maliput::api::rules::BulbState::kOn,
              },
              BulbStateCondition{
                  .bulb_id = "YellowBulb",
                  .state = maliput::api::rules::BulbState::kOff,
              },
          },
      .rule_value = "Stop",
  };
  const RuleState rule_state2{
      .bulb_conditions =
          {
              BulbStateCondition{
                  .bulb_id = "RedBulb",
                  .state = maliput::api::rules::BulbState::kOn,
              },
              BulbStateCondition{
                  .bulb_id = "YellowBulb",
                  .state = maliput::api::rules::BulbState::kOff,
              },
          },
      .rule_value = "Stop",
  };
  EXPECT_EQ(rule_state1, rule_state2);
  EXPECT_TRUE(rule_state1 == rule_state2);
}

GTEST_TEST(RuleStateEqualityOperatorTest, DifferentRuleValue) {
  const RuleState rule_state1{
      .bulb_conditions =
          {
              BulbStateCondition{
                  .bulb_id = "RedBulb",
                  .state = maliput::api::rules::BulbState::kOn,
              },
          },
      .rule_value = "Stop",
  };
  const RuleState rule_state2{
      .bulb_conditions =
          {
              BulbStateCondition{
                  .bulb_id = "RedBulb",
                  .state = maliput::api::rules::BulbState::kOn,
              },
          },
      .rule_value = "Go",
  };
  EXPECT_NE(rule_state1, rule_state2);
  EXPECT_TRUE(rule_state1 != rule_state2);
}

GTEST_TEST(RuleStateEqualityOperatorTest, DifferentBulbConditions) {
  const RuleState rule_state1{
      .bulb_conditions =
          {
              BulbStateCondition{
                  .bulb_id = "RedBulb",
                  .state = maliput::api::rules::BulbState::kOn,
              },
          },
      .rule_value = "Stop",
  };
  const RuleState rule_state2{
      .bulb_conditions =
          {
              BulbStateCondition{
                  .bulb_id = "GreenBulb",
                  .state = maliput::api::rules::BulbState::kOn,
              },
          },
      .rule_value = "Stop",
  };
  EXPECT_NE(rule_state1, rule_state2);
  EXPECT_TRUE(rule_state1 != rule_state2);
}

GTEST_TEST(BulbDefinitionEqualityOperatorTest, EqualBulbDefinitions) {
  const BulbDefinition bulb1{
      .id = "RedBulb",
      .color = maliput::api::rules::BulbColor::kRed,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
              maliput::api::rules::BulbState::kBlinking,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = std::nullopt,
      .bounding_box = {},
  };
  const BulbDefinition bulb2{
      .id = "RedBulb",
      .color = maliput::api::rules::BulbColor::kRed,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
              maliput::api::rules::BulbState::kBlinking,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = std::nullopt,
      .bounding_box = {},
  };
  EXPECT_EQ(bulb1, bulb2);
  EXPECT_TRUE(bulb1 == bulb2);
}

GTEST_TEST(BulbDefinitionEqualityOperatorTest, DifferentId) {
  const BulbDefinition bulb1{
      .id = "RedBulb",
      .color = maliput::api::rules::BulbColor::kRed,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = std::nullopt,
      .bounding_box = {},
  };
  const BulbDefinition bulb2{
      .id = "GreenBulb",
      .color = maliput::api::rules::BulbColor::kRed,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = std::nullopt,
      .bounding_box = {},
  };
  EXPECT_NE(bulb1, bulb2);
  EXPECT_TRUE(bulb1 != bulb2);
}

GTEST_TEST(BulbDefinitionEqualityOperatorTest, DifferentColor) {
  const BulbDefinition bulb1{
      .id = "RedBulb",
      .color = maliput::api::rules::BulbColor::kRed,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = std::nullopt,
      .bounding_box = {},
  };
  const BulbDefinition bulb2{
      .id = "RedBulb",
      .color = maliput::api::rules::BulbColor::kGreen,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = std::nullopt,
      .bounding_box = {},
  };
  EXPECT_NE(bulb1, bulb2);
  EXPECT_TRUE(bulb1 != bulb2);
}

GTEST_TEST(BulbDefinitionEqualityOperatorTest, DifferentInitialState) {
  const BulbDefinition bulb1{
      .id = "RedBulb",
      .color = maliput::api::rules::BulbColor::kRed,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = maliput::api::rules::BulbState::kOff,
      .bounding_box = {},
  };
  const BulbDefinition bulb2{
      .id = "RedBulb",
      .color = maliput::api::rules::BulbColor::kRed,
      .type = maliput::api::rules::BulbType::kRound,
      .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
      .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
      .states =
          {
              maliput::api::rules::BulbState::kOff,
              maliput::api::rules::BulbState::kOn,
          },
      .arrow_orientation_rad = std::nullopt,
      .initial_state = maliput::api::rules::BulbState::kOn,
      .bounding_box = {},
  };
  EXPECT_NE(bulb1, bulb2);
  EXPECT_TRUE(bulb1 != bulb2);
}

GTEST_TEST(TrafficControlDeviceFingerprintEqualityOperatorTest, EqualFingerprints) {
  const TrafficControlDeviceFingerprint fingerprint1{
      .type = "1000001",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const TrafficControlDeviceFingerprint fingerprint2{
      .type = "1000001",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  EXPECT_EQ(fingerprint1, fingerprint2);
  EXPECT_TRUE(fingerprint1 == fingerprint2);
}

GTEST_TEST(TrafficControlDeviceFingerprintEqualityOperatorTest, DifferentType) {
  const TrafficControlDeviceFingerprint fingerprint1{
      .type = "1000001",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const TrafficControlDeviceFingerprint fingerprint2{
      .type = "1000011",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  EXPECT_NE(fingerprint1, fingerprint2);
  EXPECT_TRUE(fingerprint1 != fingerprint2);
}

GTEST_TEST(TrafficControlDeviceFingerprintEqualityOperatorTest, DifferentSubtype) {
  const TrafficControlDeviceFingerprint fingerprint1{
      .type = "1000011",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const TrafficControlDeviceFingerprint fingerprint2{
      .type = "1000011",
      .subtype = "10",
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  EXPECT_NE(fingerprint1, fingerprint2);
  EXPECT_TRUE(fingerprint1 != fingerprint2);
}

GTEST_TEST(TrafficControlDeviceFingerprintEqualityOperatorTest, DifferentCountry) {
  const TrafficControlDeviceFingerprint fingerprint1{
      .type = "1000001",
      .subtype = std::nullopt,
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const TrafficControlDeviceFingerprint fingerprint2{
      .type = "1000001",
      .subtype = std::nullopt,
      .country = "ARG",
      .country_revision = std::nullopt,
  };
  EXPECT_NE(fingerprint1, fingerprint2);
  EXPECT_TRUE(fingerprint1 != fingerprint2);
}

GTEST_TEST(TrafficControlDeviceDefinitionEqualityOperatorTest, EqualTrafficControlDeviceDefinitions) {
  const TrafficControlDeviceDefinition definition1{
      .fingerprint =
          {
              .type = "1000001",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Standard three-bulb vertical traffic light",
      .bulbs =
          {
              BulbDefinition{
                  .id = "RedBulb",
                  .color = maliput::api::rules::BulbColor::kRed,
                  .type = maliput::api::rules::BulbType::kRound,
                  .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
                  .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
                  .states =
                      {
                          maliput::api::rules::BulbState::kOff,
                          maliput::api::rules::BulbState::kOn,
                      },
                  .arrow_orientation_rad = std::nullopt,
                  .initial_state = std::nullopt,
                  .bounding_box = {},
              },
          },
      .rule_states =
          {
              RuleState{
                  .bulb_conditions =
                      {
                          BulbStateCondition{
                              .bulb_id = "RedBulb",
                              .state = maliput::api::rules::BulbState::kOn,
                          },
                      },
                  .rule_value = "Stop",
              },
          },
  };
  const TrafficControlDeviceDefinition definition2{
      .fingerprint =
          {
              .type = "1000001",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Standard three-bulb vertical traffic light",
      .bulbs =
          {
              BulbDefinition{
                  .id = "RedBulb",
                  .color = maliput::api::rules::BulbColor::kRed,
                  .type = maliput::api::rules::BulbType::kRound,
                  .position_traffic_light = maliput::math::Vector3(0.0, 0.0, 0.4),
                  .orientation_traffic_light = maliput::math::Quaternion(1.0, 0.0, 0.0, 0.0),
                  .states =
                      {
                          maliput::api::rules::BulbState::kOff,
                          maliput::api::rules::BulbState::kOn,
                      },
                  .arrow_orientation_rad = std::nullopt,
                  .initial_state = std::nullopt,
                  .bounding_box = {},
              },
          },
      .rule_states =
          {
              RuleState{
                  .bulb_conditions =
                      {
                          BulbStateCondition{
                              .bulb_id = "RedBulb",
                              .state = maliput::api::rules::BulbState::kOn,
                          },
                      },
                  .rule_value = "Stop",
              },
          },
  };
  EXPECT_EQ(definition1, definition2);
  EXPECT_TRUE(definition1 == definition2);
}

GTEST_TEST(TrafficControlDeviceDefinitionEqualityOperatorTest, DifferentFingerprint) {
  const TrafficControlDeviceDefinition definition1{
      .fingerprint =
          {
              .type = "1000001",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Standard three-bulb vertical traffic light",
      .bulbs = {},
      .rule_states = {},
  };
  const TrafficControlDeviceDefinition definition2{
      .fingerprint =
          {
              .type = "1000011",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Standard three-bulb vertical traffic light",
      .bulbs = {},
      .rule_states = {},
  };
  EXPECT_NE(definition1, definition2);
  EXPECT_TRUE(definition1 != definition2);
}

GTEST_TEST(TrafficControlDeviceDefinitionEqualityOperatorTest, DifferentDescription) {
  const TrafficControlDeviceDefinition definition1{
      .fingerprint =
          {
              .type = "1000001",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Standard three-bulb vertical traffic light",
      .bulbs = {},
      .rule_states = {},
  };
  const TrafficControlDeviceDefinition definition2{
      .fingerprint =
          {
              .type = "1000001",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Traffic light with arrow",
      .bulbs = {},
      .rule_states = {},
  };
  EXPECT_NE(definition1, definition2);
  EXPECT_TRUE(definition1 != definition2);
}

GTEST_TEST(TrafficControlDeviceDefinitionEqualityOperatorTest, DifferentRuleStates) {
  const TrafficControlDeviceDefinition definition1{
      .fingerprint =
          {
              .type = "1000001",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Standard three-bulb vertical traffic light",
      .bulbs = {},
      .rule_states =
          {
              RuleState{
                  .bulb_conditions =
                      {
                          BulbStateCondition{
                              .bulb_id = "RedBulb",
                              .state = maliput::api::rules::BulbState::kOn,
                          },
                      },
                  .rule_value = "Stop",
              },
          },
  };
  const TrafficControlDeviceDefinition definition2{
      .fingerprint =
          {
              .type = "1000001",
              .subtype = std::nullopt,
              .country = "OpenDRIVE",
              .country_revision = std::nullopt,
          },
      .description = "Standard three-bulb vertical traffic light",
      .bulbs = {},
      .rule_states =
          {
              RuleState{
                  .bulb_conditions =
                      {
                          BulbStateCondition{
                              .bulb_id = "GreenBulb",
                              .state = maliput::api::rules::BulbState::kOn,
                          },
                      },
                  .rule_value = "Go",
              },
          },
  };
  EXPECT_NE(definition1, definition2);
  EXPECT_TRUE(definition1 != definition2);
}

// ---------------------------------------------------------------------------
// IsWildcard tests
// ---------------------------------------------------------------------------

GTEST_TEST(IsWildcardStringTest, WildcardStringReturnsTrue) {
  EXPECT_TRUE(TrafficControlDeviceParser::IsWildcard(std::string("*")));
}

GTEST_TEST(IsWildcardStringTest, NonWildcardStringReturnsFalse) {
  EXPECT_FALSE(TrafficControlDeviceParser::IsWildcard(std::string("1000001")));
}

GTEST_TEST(IsWildcardStringTest, EmptyStringReturnsFalse) {
  EXPECT_FALSE(TrafficControlDeviceParser::IsWildcard(std::string("")));
}

GTEST_TEST(IsWildcardOptionalTest, PresentWildcardReturnsTrue) {
  EXPECT_TRUE(TrafficControlDeviceParser::IsWildcard(std::optional<std::string>{"*"}));
}

GTEST_TEST(IsWildcardOptionalTest, NulloptReturnsFalse) {
  EXPECT_FALSE(TrafficControlDeviceParser::IsWildcard(std::optional<std::string>{std::nullopt}));
}

GTEST_TEST(IsWildcardOptionalTest, PresentNonWildcardReturnsFalse) {
  EXPECT_FALSE(TrafficControlDeviceParser::IsWildcard(std::optional<std::string>{"OpenDRIVE"}));
}

// ---------------------------------------------------------------------------
// CanOverlap tests
// ---------------------------------------------------------------------------

GTEST_TEST(CanOverlapTest, BothAllWildcardsOverlap) {
  const TrafficControlDeviceFingerprint a{
      .type = "*", .subtype = "*", .country = "*", .country_revision = "*", .name = "*"};
  const TrafficControlDeviceFingerprint b{
      .type = "*", .subtype = "*", .country = "*", .country_revision = "*", .name = "*"};
  EXPECT_TRUE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, SameConcreteValuesOverlap) {
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = "10",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  const TrafficControlDeviceFingerprint b{.type = "1000001",
                                          .subtype = "10",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, WildcardOnOneSideOverlapsConcreteOnOtherSide) {
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = "*",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  const TrafficControlDeviceFingerprint b{.type = "1000001",
                                          .subtype = "10",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, WildcardOnOneSideOverlapsNulloptOnOtherSide) {
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = "*",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  const TrafficControlDeviceFingerprint b{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, DifferentConcreteValuesOnOneFieldNoOverlap) {
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = "10",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  const TrafficControlDeviceFingerprint b{.type = "9999999",
                                          .subtype = "10",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  EXPECT_FALSE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, ConcreteVsNulloptOnSameFieldNoOverlap) {
  // subtype = "10" vs subtype = nullopt: neither is a wildcard and they differ.
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = "10",
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  const TrafficControlDeviceFingerprint b{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = "OpenDRIVE",
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  EXPECT_FALSE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, BothNulloptOnFieldOverlap) {
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = std::nullopt,
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  const TrafficControlDeviceFingerprint b{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = std::nullopt,
                                          .country_revision = std::nullopt,
                                          .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, NameFieldDifferencePreventsOverlap) {
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = std::nullopt,
                                          .country_revision = std::nullopt,
                                          .name = "stop_sign"};
  const TrafficControlDeviceFingerprint b{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = std::nullopt,
                                          .country_revision = std::nullopt,
                                          .name = "yield_sign"};
  EXPECT_FALSE(TrafficControlDeviceParser::CanOverlap(a, b));
}

GTEST_TEST(CanOverlapTest, WildcardNameOverlapsAnyName) {
  const TrafficControlDeviceFingerprint a{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = std::nullopt,
                                          .country_revision = std::nullopt,
                                          .name = "*"};
  const TrafficControlDeviceFingerprint b{.type = "1000001",
                                          .subtype = std::nullopt,
                                          .country = std::nullopt,
                                          .country_revision = std::nullopt,
                                          .name = "stop_sign"};
  EXPECT_TRUE(TrafficControlDeviceParser::CanOverlap(a, b));
}

// ---------------------------------------------------------------------------
// YAML fixtures for wildcard / specificity / lookup tests
// ---------------------------------------------------------------------------

const char kWildcardParsingDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "*"
      subtype: "*"
      country: "*"
      country_revision: "*"
      name: "*"
    properties:
      device_type: traffic_sign
      device_semantics: generic
      description: "All-wildcard entry"
      bulbs: []
      rule_states: []
)";

const char kConflictingDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
      subtype: "*"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_sign
      device_semantics: generic
      description: "Entry A"
      bulbs: []
      rule_states: []
  - odr_representation:
      type: "1000001"
      subtype: "*"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_sign
      device_semantics: generic
      description: "Entry B"
      bulbs: []
      rule_states: []
)";

const char kDifferentSpecificityDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
      subtype: "*"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_sign
      device_semantics: more_specific
      description: "More specific entry"
      bulbs: []
      rule_states: []
  - odr_representation:
      type: "1000001"
      subtype: "*"
      country: "*"
      country_revision: null
    properties:
      device_type: traffic_sign
      device_semantics: less_specific
      description: "Less specific entry"
      bulbs: []
      rule_states: []
)";

const char kWildcardLookupDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
      subtype: "10"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_sign
      device_semantics: exact_match
      description: "Exact match entry"
      bulbs: []
      rule_states: []
  - odr_representation:
      type: "1000001"
      subtype: "*"
      country: "OpenDRIVE"
      country_revision: null
    properties:
      device_type: traffic_sign
      device_semantics: partial_wildcard
      description: "Partial wildcard entry"
      bulbs: []
      rule_states: []
  - odr_representation:
      type: "*"
      subtype: "*"
      country: "*"
      country_revision: "*"
    properties:
      device_type: traffic_sign
      device_semantics: catch_all
      description: "Catch-all entry"
      bulbs: []
      rule_states: []
)";

// ---- Step 5: New wildcard / specificity / conflict / lookup tests ----

GTEST_TEST(WildcardParsingTest, AllFieldsStoredAsLiteralAsterisk) {
  const auto defs = TrafficControlDeviceParser::LoadFromString(kWildcardParsingDb);
  ASSERT_EQ(defs.size(), 1u);
  EXPECT_EQ(defs[0].fingerprint.type, "*");
  EXPECT_EQ(defs[0].fingerprint.subtype, "*");
  EXPECT_EQ(defs[0].fingerprint.country, "*");
  EXPECT_EQ(defs[0].fingerprint.country_revision, "*");
  EXPECT_EQ(defs[0].fingerprint.name, "*");
}

GTEST_TEST(SpecificityCalculationTest, AllConcrete) {
  const TrafficControlDeviceFingerprint fp{
      .type = "1000001", .subtype = "10", .country = "OpenDRIVE", .country_revision = "2020", .name = "TrafficLight"};
  EXPECT_EQ(TrafficControlDeviceParser::Specificity(fp), 5);
}

GTEST_TEST(SpecificityCalculationTest, AllWildcards) {
  const TrafficControlDeviceFingerprint fp{
      .type = "*", .subtype = "*", .country = "*", .country_revision = "*", .name = "*"};
  EXPECT_EQ(TrafficControlDeviceParser::Specificity(fp), 0);
}

GTEST_TEST(SpecificityCalculationTest, NulloptIsSpecific) {
  const TrafficControlDeviceFingerprint fp{.type = "1000001",
                                           .subtype = std::nullopt,
                                           .country = std::nullopt,
                                           .country_revision = std::nullopt,
                                           .name = std::nullopt};
  EXPECT_EQ(TrafficControlDeviceParser::Specificity(fp), 5);
}

GTEST_TEST(SpecificityCalculationTest, Mixed) {
  // type concrete, subtype wildcard, country concrete, country_revision nullopt, name nullopt -> 4
  const TrafficControlDeviceFingerprint fp{.type = "1000001",
                                           .subtype = "*",
                                           .country = "OpenDRIVE",
                                           .country_revision = std::nullopt,
                                           .name = std::nullopt};
  EXPECT_EQ(TrafficControlDeviceParser::Specificity(fp), 4);
}

GTEST_TEST(MatchingLogicTest, ExactMatchReturnsTrue) {
  const TrafficControlDeviceFingerprint db_entry{.type = "1000001",
                                                 .subtype = "10",
                                                 .country = "OpenDRIVE",
                                                 .country_revision = std::nullopt,
                                                 .name = std::nullopt};
  const TrafficControlDeviceFingerprint query{.type = "1000001",
                                              .subtype = "10",
                                              .country = "OpenDRIVE",
                                              .country_revision = std::nullopt,
                                              .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query));
}

GTEST_TEST(MatchingLogicTest, WildcardTypeMatchesAny) {
  const TrafficControlDeviceFingerprint db_entry{.type = "*",
                                                 .subtype = std::nullopt,
                                                 .country = std::nullopt,
                                                 .country_revision = std::nullopt,
                                                 .name = std::nullopt};
  const TrafficControlDeviceFingerprint query{.type = "anything",
                                              .subtype = std::nullopt,
                                              .country = std::nullopt,
                                              .country_revision = std::nullopt,
                                              .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query));
}

GTEST_TEST(MatchingLogicTest, WildcardSubtypeMatchesPresentAndNullopt) {
  const TrafficControlDeviceFingerprint db_entry{.type = "1000001",
                                                 .subtype = "*",
                                                 .country = std::nullopt,
                                                 .country_revision = std::nullopt,
                                                 .name = std::nullopt};
  const TrafficControlDeviceFingerprint query_with_subtype{.type = "1000001",
                                                           .subtype = "10",
                                                           .country = std::nullopt,
                                                           .country_revision = std::nullopt,
                                                           .name = std::nullopt};
  const TrafficControlDeviceFingerprint query_nullopt_subtype{.type = "1000001",
                                                              .subtype = "10",
                                                              .country = std::nullopt,
                                                              .country_revision = std::nullopt,
                                                              .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query_with_subtype));
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query_nullopt_subtype));
}

GTEST_TEST(MatchingLogicTest, NulloptSubtypeOnlyMatchesNullopt) {
  const TrafficControlDeviceFingerprint db_entry{.type = "1000001",
                                                 .subtype = std::nullopt,
                                                 .country = std::nullopt,
                                                 .country_revision = std::nullopt,
                                                 .name = std::nullopt};
  const TrafficControlDeviceFingerprint query_nullopt{.type = "1000001",
                                                      .subtype = std::nullopt,
                                                      .country = std::nullopt,
                                                      .country_revision = std::nullopt,
                                                      .name = std::nullopt};
  const TrafficControlDeviceFingerprint query_concrete{.type = "1000001",
                                                       .subtype = "10",
                                                       .country = std::nullopt,
                                                       .country_revision = std::nullopt,
                                                       .name = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query_nullopt));
  EXPECT_FALSE(TrafficControlDeviceParser::Matches(db_entry, query_concrete));
}

GTEST_TEST(MatchingLogicTest, ConcreteMismatchReturnsFalse) {
  const TrafficControlDeviceFingerprint db_entry{.type = "1000001",
                                                 .subtype = "10",
                                                 .country = "OpenDRIVE",
                                                 .country_revision = std::nullopt,
                                                 .name = std::nullopt};
  const TrafficControlDeviceFingerprint query{.type = "1000002",
                                              .subtype = "10",
                                              .country = "OpenDRIVE",
                                              .country_revision = std::nullopt,
                                              .name = std::nullopt};
  EXPECT_FALSE(TrafficControlDeviceParser::Matches(db_entry, query));
}

GTEST_TEST(MatchingLogicTest, WildcardNameMatchesPresentAndNullopt) {
  const TrafficControlDeviceFingerprint db_entry{.type = "1000001",
                                                 .subtype = std::nullopt,
                                                 .country = std::nullopt,
                                                 .country_revision = std::nullopt,
                                                 .name = "*"};
  const TrafficControlDeviceFingerprint query_with_name{.type = "1000001",
                                                        .subtype = std::nullopt,
                                                        .country = std::nullopt,
                                                        .country_revision = std::nullopt,
                                                        .name = "stop_sign"};
  const TrafficControlDeviceFingerprint query_nullopt_name{
      .type = "1000001", .subtype = std::nullopt, .country = std::nullopt, .country_revision = std::nullopt};
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query_with_name));
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query_nullopt_name));
}

GTEST_TEST(MatchingLogicTest, NulloptNameOnlyMatchesNullopt) {
  const TrafficControlDeviceFingerprint db_entry{.type = "1000001",
                                                 .subtype = std::nullopt,
                                                 .country = std::nullopt,
                                                 .country_revision = std::nullopt,
                                                 .name = std::nullopt};
  const TrafficControlDeviceFingerprint query_nullopt{.type = "1000001",
                                                      .subtype = std::nullopt,
                                                      .country = std::nullopt,
                                                      .country_revision = std::nullopt,
                                                      .name = std::nullopt};
  const TrafficControlDeviceFingerprint query_with_name{.type = "1000001",
                                                        .subtype = std::nullopt,
                                                        .country = std::nullopt,
                                                        .country_revision = std::nullopt,
                                                        .name = "stop_sign"};
  EXPECT_TRUE(TrafficControlDeviceParser::Matches(db_entry, query_nullopt));
  EXPECT_FALSE(TrafficControlDeviceParser::Matches(db_entry, query_with_name));
}

GTEST_TEST(ConflictDetectionTest, EqualSpecificityOverlapThrows) {
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kConflictingDb),
               maliput::common::road_network_description_parser_error);
}

GTEST_TEST(ConflictDetectionTest, DifferentSpecificityOverlapDoesNotThrow) {
  EXPECT_NO_THROW(TrafficControlDeviceParser::LoadFromString(kDifferentSpecificityDb));
}

GTEST_TEST(WildcardLookupTest, ExactMatchPreferredOverWildcard) {
  const auto loader = TrafficControlDeviceDatabaseLoader::FromString(kWildcardLookupDb);
  const TrafficControlDeviceFingerprint query{.type = "1000001",
                                              .subtype = "10",
                                              .country = "OpenDRIVE",
                                              .country_revision = std::nullopt,
                                              .name = std::nullopt};
  const auto result = loader.Lookup(query);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->device_semantics, "exact_match");
}

GTEST_TEST(WildcardLookupTest, PartialWildcardPreferredOverCatchAll) {
  const auto loader = TrafficControlDeviceDatabaseLoader::FromString(kWildcardLookupDb);
  const TrafficControlDeviceFingerprint query{.type = "1000001",
                                              .subtype = "99",
                                              .country = "OpenDRIVE",
                                              .country_revision = std::nullopt,
                                              .name = std::nullopt};
  const auto result = loader.Lookup(query);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->device_semantics, "partial_wildcard");
}

GTEST_TEST(WildcardLookupTest, FallbackToCatchAll) {
  const auto loader = TrafficControlDeviceDatabaseLoader::FromString(kWildcardLookupDb);
  const TrafficControlDeviceFingerprint query{.type = "9999999",
                                              .subtype = std::nullopt,
                                              .country = std::nullopt,
                                              .country_revision = std::nullopt,
                                              .name = std::nullopt};
  const auto result = loader.Lookup(query);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->device_semantics, "catch_all");
}

GTEST_TEST(WildcardLookupTest, NoMatchReturnsNullopt) {
  const auto loader = TrafficControlDeviceDatabaseLoader::FromString(kTrafficControlDeviceDb);
  const TrafficControlDeviceFingerprint query{.type = "9999999",
                                              .subtype = std::nullopt,
                                              .country = std::nullopt,
                                              .country_revision = std::nullopt,
                                              .name = std::nullopt};
  const auto result = loader.Lookup(query);
  EXPECT_FALSE(result.has_value());
}

GTEST_TEST(WildcardLookupTest, CatchAllMatchesAnyQuery) {
  const auto loader = TrafficControlDeviceDatabaseLoader::FromString(kWildcardParsingDb);
  const TrafficControlDeviceFingerprint query{.type = "any_type",
                                              .subtype = "any_subtype",
                                              .country = "any_country",
                                              .country_revision = "rev1",
                                              .name = "any_name"};
  const auto result = loader.Lookup(query);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->device_semantics, "generic");
}

GTEST_TEST(WildcardLookupTest, ConcreteNameWinsOverWildcardName) {
  const char kNameLookupDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
      name: "stop_sign"
    properties:
      device_type: traffic_sign
      device_semantics: concrete_name
      description: "Concrete name entry"
      bulbs: []
      rule_states: []
  - odr_representation:
      type: "1000001"
      name: "*"
    properties:
      device_type: traffic_sign
      device_semantics: wildcard_name
      description: "Wildcard name entry"
      bulbs: []
      rule_states: []
)";
  const auto loader = TrafficControlDeviceDatabaseLoader::FromString(kNameLookupDb);
  const TrafficControlDeviceFingerprint query{.type = "1000001", .name = "stop_sign"};
  const auto result = loader.Lookup(query);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->device_semantics, "concrete_name");
}

// ============================================================================
// odr_object_types parsing
// ============================================================================

// Helper that filters `defs` by device_type for assertions on object/signal subsets.
std::vector<TrafficControlDeviceDefinition> FilterByDeviceType(const std::vector<TrafficControlDeviceDefinition>& defs,
                                                               TrafficControlDeviceType type) {
  std::vector<TrafficControlDeviceDefinition> out;
  std::copy_if(defs.begin(), defs.end(), std::back_inserter(out),
               [type](const TrafficControlDeviceDefinition& d) { return d.device_type == type; });
  return out;
}

GTEST_TEST(OdrObjectTypesParserTest, LoadObjectsOnlyDatabase) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
      name: "StopLine"
    properties:
      device_type: road_marking
      device_semantics: stop_line
      description: "Stop line road marking"
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      description: "Generic pole"
      is_position_dynamic: false
      default_bounding_box:
        length: 0.2
        width: 0.2
        height: 2.0
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  EXPECT_EQ(defs.size(), 2);

  const auto road_markings = FilterByDeviceType(defs, TrafficControlDeviceType::kRoadMarking);
  ASSERT_EQ(road_markings.size(), 1);
  EXPECT_EQ(road_markings[0].fingerprint.type, "roadMark");
  ASSERT_TRUE(road_markings[0].fingerprint.name.has_value());
  EXPECT_EQ(road_markings[0].fingerprint.name.value(), "StopLine");
  EXPECT_FALSE(road_markings[0].fingerprint.country.has_value());
  EXPECT_FALSE(road_markings[0].fingerprint.country_revision.has_value());
  ASSERT_TRUE(road_markings[0].device_semantics.has_value());
  EXPECT_EQ(road_markings[0].device_semantics.value(), "stop_line");
  EXPECT_TRUE(road_markings[0].bulbs.empty());
  EXPECT_TRUE(road_markings[0].rule_states.empty());

  const auto road_objects = FilterByDeviceType(defs, TrafficControlDeviceType::kRoadObject);
  ASSERT_EQ(road_objects.size(), 1);
  EXPECT_EQ(road_objects[0].fingerprint.type, "pole");
  EXPECT_FALSE(road_objects[0].fingerprint.subtype.has_value());
  EXPECT_FALSE(road_objects[0].fingerprint.name.has_value());
  ASSERT_TRUE(road_objects[0].default_bounding_box.has_value());
  EXPECT_NEAR(road_objects[0].default_bounding_box->length, 0.2, kTolerance);
  EXPECT_NEAR(road_objects[0].default_bounding_box->width, 0.2, kTolerance);
  EXPECT_NEAR(road_objects[0].default_bounding_box->height, 2.0, kTolerance);
}

GTEST_TEST(OdrObjectTypesParserTest, LoadBothRootsDatabase) {
  const char kDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "206"
      subtype: "-1"
    properties:
      device_type: traffic_sign
      device_semantics: give_way
      description: "Give way sign"
      rule_states:
        - conditions: []
          value: "StopIfSafe"
odr_object_types:
  - odr_representation:
      type: crosswalk
    properties:
      device_type: road_marking
      device_semantics: crosswalk
      description: "Generic crosswalk"
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 2);
  EXPECT_EQ(FilterByDeviceType(defs, TrafficControlDeviceType::kTrafficSign).size(), 1);
  EXPECT_EQ(FilterByDeviceType(defs, TrafficControlDeviceType::kRoadMarking).size(), 1);
}

GTEST_TEST(OdrObjectTypesParserTest, EmptyObjectArrayIsValid) {
  const char kDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "206"
    properties:
      device_type: traffic_sign
      description: "Sign"
odr_object_types: []
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 1);
  EXPECT_EQ(defs[0].device_type, TrafficControlDeviceType::kTrafficSign);
}

GTEST_TEST(OdrObjectTypesParserTest, MissingBothRootsIsRejected) {
  const char kDb[] = R"(
some_unrelated_key: 42
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, RejectsCountryInObject) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
      country: "DE"
    properties:
      device_type: road_marking
      description: "Should fail"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, RejectsCountryRevisionInObject) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
      country_revision: "2017"
    properties:
      device_type: road_marking
      description: "Should fail"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, RejectsBulbsInObject) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
    properties:
      device_type: road_marking
      description: "Should fail"
      bulbs:
        - id: "Bulb"
          position_traffic_light: [0.0, 0.0, 0.0]
          orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
          color: red
          type: round
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, RejectsRuleStatesInObject) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
    properties:
      device_type: road_marking
      description: "Should fail"
      rule_states:
        - conditions: []
          value: "Stop"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, RejectsSignalDeviceTypeInObject) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
    properties:
      device_type: traffic_light
      description: "Should fail"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, RejectsTrafficSignDeviceTypeInObject) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
    properties:
      device_type: traffic_sign
      description: "Should fail"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, RejectsObjectDeviceTypeInSignal) {
  const char kRoadMarkingInSignal[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
    properties:
      device_type: road_marking
      description: "Should fail"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kRoadMarkingInSignal),
               maliput::common::road_network_description_parser_error);
  const char kRoadObjectInSignal[] = R"(
odr_signal_types:
  - odr_representation:
      type: "1000001"
    properties:
      device_type: road_object
      description: "Should fail"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kRoadObjectInSignal),
               maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, EqualSpecificityObjectConflictIsRejected) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
      name: "*"
    properties:
      device_type: road_marking
      description: "First wildcard"
  - odr_representation:
      type: roadMark
      name: "*"
    properties:
      device_type: road_marking
      description: "Second wildcard"
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(OdrObjectTypesParserTest, SignalAndObjectWithSameFingerprintDoNotConflict) {
  // The two entries share `type = "roadMark"` and have equal specificity. Per the per-root
  // validation rule, they MUST NOT be flagged as conflicting because signals and objects
  // live in disjoint namespaces.
  const char kDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "roadMark"
    properties:
      device_type: traffic_sign
      description: "Signal entry"
      rule_states: []
odr_object_types:
  - odr_representation:
      type: "roadMark"
    properties:
      device_type: road_marking
      description: "Object entry"
)";
  EXPECT_NO_THROW({
    const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
    EXPECT_EQ(defs.size(), 2);
  });
}

GTEST_TEST(OdrObjectTypesParserTest, WildcardWorksForObjectFields) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: roadMark
      name: "*"
    properties:
      device_type: road_marking
      description: "Wildcard name catch-all"
  - odr_representation:
      type: roadMark
      name: "StopLine"
    properties:
      device_type: road_marking
      device_semantics: stop_line
      description: "Specific stop line"
)";
  // Two roadMark entries, one with wildcard name and one with concrete name —
  // different specificities, so no conflict.
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 2);
}

GTEST_TEST(OdrObjectTypesParserTest, TypeOnlyObjectEntryParses) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      description: "Minimal object entry"
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 1);
  EXPECT_EQ(defs[0].fingerprint.type, "pole");
  EXPECT_FALSE(defs[0].fingerprint.subtype.has_value());
  EXPECT_FALSE(defs[0].fingerprint.name.has_value());
  EXPECT_FALSE(defs[0].fingerprint.country.has_value());
  EXPECT_FALSE(defs[0].fingerprint.country_revision.has_value());
}

// ============================================================================
// SpecificityForObject — direct unit tests
// ============================================================================

GTEST_TEST(SpecificityForObjectTest, CountsOnlyTypeSubtypeName) {
  const TrafficControlDeviceFingerprint all_wildcards{
      .type = "*", .subtype = "*", .country = std::nullopt, .country_revision = std::nullopt, .name = "*"};
  EXPECT_EQ(TrafficControlDeviceParser::SpecificityForObject(all_wildcards), 0);

  const TrafficControlDeviceFingerprint type_only{
      .type = "roadMark", .subtype = "*", .country = std::nullopt, .country_revision = std::nullopt, .name = "*"};
  EXPECT_EQ(TrafficControlDeviceParser::SpecificityForObject(type_only), 1);

  const TrafficControlDeviceFingerprint all_three{.type = "roadMark",
                                                  .subtype = "10",
                                                  .country = std::nullopt,
                                                  .country_revision = std::nullopt,
                                                  .name = "StopLine"};
  EXPECT_EQ(TrafficControlDeviceParser::SpecificityForObject(all_three), 3);
}

// ---------------------------------------------------------------------------
// Device-level default_bounding_box (BoundingBoxDimensions) parsing.
// ---------------------------------------------------------------------------

GTEST_TEST(DefaultBoundingBoxParserTest, FullySpecifiedFields) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box:
        length: 0.5
        width: 0.6
        height: 2.4
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 1);
  ASSERT_TRUE(defs[0].default_bounding_box.has_value());
  const auto& bbox = *defs[0].default_bounding_box;
  EXPECT_NEAR(bbox.length, 0.5, kTolerance);
  EXPECT_NEAR(bbox.width, 0.6, kTolerance);
  EXPECT_NEAR(bbox.height, 2.4, kTolerance);
}

GTEST_TEST(DefaultBoundingBoxParserTest, WorksOnSignalEntries) {
  const char kDb[] = R"(
odr_signal_types:
  - odr_representation:
      type: "206"
    properties:
      device_type: traffic_sign
      device_semantics: stop
      default_bounding_box:
        length: 0.05
        width: 0.6
        height: 0.6
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 1);
  ASSERT_TRUE(defs[0].default_bounding_box.has_value());
  EXPECT_NEAR(defs[0].default_bounding_box->length, 0.05, kTolerance);
  EXPECT_NEAR(defs[0].default_bounding_box->width, 0.6, kTolerance);
  EXPECT_NEAR(defs[0].default_bounding_box->height, 0.6, kTolerance);
}

GTEST_TEST(DefaultBoundingBoxParserTest, MissingLengthRejected) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box:
        width: 1.0
        height: 1.0
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(DefaultBoundingBoxParserTest, MissingWidthRejected) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box:
        length: 1.0
        height: 1.0
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(DefaultBoundingBoxParserTest, MissingHeightRejected) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box:
        length: 1.0
        width: 1.0
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(DefaultBoundingBoxParserTest, NegativeDimensionRejected) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box:
        length: 1.0
        width: -1.0
        height: 1.0
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(DefaultBoundingBoxParserTest, UnknownFieldRejected) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box:
        length: 1.0
        width: 1.0
        height: 1.0
        p_min: [0.0, 0.0, 0.0]
)";
  EXPECT_THROW(TrafficControlDeviceParser::LoadFromString(kDb), maliput::common::road_network_description_parser_error);
}

GTEST_TEST(DefaultBoundingBoxParserTest, NullDefaultBoundingBoxLeavesItUnset) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box: null
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 1);
  EXPECT_FALSE(defs[0].default_bounding_box.has_value());
}

GTEST_TEST(DefaultBoundingBoxParserTest, ZeroDimensionsAllowed) {
  const char kDb[] = R"(
odr_object_types:
  - odr_representation:
      type: pole
    properties:
      device_type: road_object
      default_bounding_box:
        length: 0.0
        width: 0.0
        height: 0.0
)";
  const auto defs = TrafficControlDeviceParser::LoadFromString(kDb);
  ASSERT_EQ(defs.size(), 1);
  ASSERT_TRUE(defs[0].default_bounding_box.has_value());
  EXPECT_NEAR(defs[0].default_bounding_box->length, 0.0, kTolerance);
  EXPECT_NEAR(defs[0].default_bounding_box->width, 0.0, kTolerance);
  EXPECT_NEAR(defs[0].default_bounding_box->height, 0.0, kTolerance);
}

}  // namespace
}  // namespace test
}  // namespace traffic_control_device
}  // namespace malidrive
