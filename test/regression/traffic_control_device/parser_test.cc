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

}  // namespace
}  // namespace test
}  // namespace traffic_control_device
}  // namespace malidrive
