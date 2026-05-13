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

#include <fstream>
#include <sstream>

#include <gtest/gtest.h>

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

class TrafficControlDeviceParserTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    signal_definitions_ = TrafficControlDeviceParser::LoadFromString(kTrafficControlDeviceDb);
  }
  static std::unordered_map<TrafficControlDeviceFingerprint, TrafficControlDeviceDefinition> signal_definitions_;
};

// Needed to allocate memory for static member variable.
std::unordered_map<TrafficControlDeviceFingerprint, TrafficControlDeviceDefinition>
    TrafficControlDeviceParserTest::signal_definitions_;

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
  EXPECT_TRUE(signal_definitions_.find(standard_fingerprint) != signal_definitions_.end());
  EXPECT_TRUE(signal_definitions_.find(arrow_fingerprint) != signal_definitions_.end());
}

TEST_F(TrafficControlDeviceParserTest, ValidateTrafficSign) {
  const TrafficControlDeviceFingerprint sign_fingerprint{
      .type = "206",
      .subtype = "30",
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  const auto& dut = signal_definitions_.at(sign_fingerprint);

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
  const auto& dut = signal_definitions_.at(fingerprint);

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
  const auto& dut = signal_definitions_.at(arrow_fingerprint);

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
}

TEST_F(TrafficControlDeviceParserTest, DefaultBulbStatesAndBoundingBox) {
  const TrafficControlDeviceFingerprint default_fingerprint{
      .type = "default_bulb_state",
      .subtype = std::nullopt,
      .country = std::nullopt,
      .country_revision = std::nullopt,
  };
  const auto& dut = signal_definitions_.at(default_fingerprint);

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

GTEST_TEST(RepeatedTrafficControlDeviceParserTest, RepeatedFingerprintOverwrites) {
  const auto signal_definitions = TrafficControlDeviceParser::LoadFromString(kRepeatedTrafficControlDeviceDb);
  EXPECT_EQ(signal_definitions.size(), 1);
  const TrafficControlDeviceFingerprint fingerprint{
      .type = "1234567",
      .subtype = "11",
      .country = "OpenDRIVE",
      .country_revision = std::nullopt,
  };
  EXPECT_TRUE(signal_definitions.find(fingerprint) != signal_definitions.end());
  const auto& definition = signal_definitions.at(fingerprint);
  EXPECT_EQ(definition.description, "Single green bulb traffic light");
  EXPECT_EQ(definition.device_type, traffic_control_device::TrafficControlDeviceType::kTrafficLight);
  EXPECT_EQ(definition.bulbs.size(), 1);
  EXPECT_EQ(definition.bulbs[0].id, "GreenBulb");
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

}  // namespace
}  // namespace test
}  // namespace traffic_control_device
}  // namespace malidrive
