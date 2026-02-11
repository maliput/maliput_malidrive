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
#include "maliput_malidrive/traffic_signal/parser.h"

#include <gtest/gtest.h>
#include <maliput/api/rules/traffic_lights.h>

namespace malidrive {
namespace traffic_signal {
namespace test {
namespace {

constexpr double kTolerance = 1e-6;

const char kTrafficSignalDb[] = R"(
traffic_signal_types:
  - type: "1000001"
    subtype: -1
    country: "OpenDRIVE"
    country_revision: null
    description: "Standard three-bulb vertical traffic light"
    bulb_group:
      - position_traffic_light: [0.0, 0.0, 0.0]
        orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
        bulbs:
          - id: "RedBulb"
            position_bulb_group: [0.0, 0.0, 0.4]
            orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
            color: "Red"
            type: "Round"
            states: ["Off", "On", "Blinking"]
            bounding_box:
              p_min: [-0.0889, -0.1778, -0.1778]
              p_max: [0.0889, 0.1778, 0.1778]
          - id: "YellowBulb"
            position_bulb_group: [0.0, 0.0, 0.0]
            orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
            color: "Yellow"
            type: "Round"
            states: ["Off", "On", "Blinking"]
          - id: "GreenBulb"
            position_bulb_group: [0.0, 0.0, -0.4]
            orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
            color: "Green"
            type: "Round"
            states: ["Off", "On"]
    rule_states:
      - condition:
          - bulb: "RedBulb"
            state: "On"
          - bulb: "YellowBulb"
            state: "Off"
          - bulb: "GreenBulb"
            state: "Off"
        value: "Stop"
      - condition:
          - bulb: "RedBulb"
            state: "Off"
          - bulb: "YellowBulb"
            state: "Off"
          - bulb: "GreenBulb"
            state: "On"
        value: "Go"
  - type: "1000011"
    subtype: 10
    country: "OpenDRIVE"
    country_revision: null
    description: "Traffic light with arrow"
    bulb_group:
      - position_traffic_light: [0.0, 0.0, 0.0]
        orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
        bulbs:
          - id: "GreenArrow"
            position_bulb_group: [0.0, 0.0, 0.0]
            orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
            color: "Green"
            type: "Arrow"
            states: ["Off", "On"]
            arrow_orientation_rad: 1.5707963267948966
    rule_states:
      - condition:
          - bulb: "GreenArrow"
            state: "On"
        value: "Go"
)";

GTEST_TEST(TrafficSignalParserTest, LoadFromString) {
  const auto signal_definitions = TrafficSignalParser::LoadFromString(kTrafficSignalDb);

  EXPECT_EQ(signal_definitions.size(), 2);
  EXPECT_TRUE(signal_definitions.find("1000001") != signal_definitions.end());
  EXPECT_TRUE(signal_definitions.find("1000011") != signal_definitions.end());
}

GTEST_TEST(TrafficSignalParserTest, ValidateSignalType1000001) {
  const auto signal_definitions = TrafficSignalParser::LoadFromString(kTrafficSignalDb);
  const auto& dut = signal_definitions.at("1000001");

  EXPECT_EQ(dut.type, "1000001");
  EXPECT_EQ(dut.subtype, -1);
  EXPECT_EQ(dut.country, "OpenDRIVE");
  EXPECT_EQ(dut.description, "Standard three-bulb vertical traffic light");

  const auto& group = dut.bulb_group;
  EXPECT_EQ(group.bulbs.size(), 3);

  // Check red bulb
  const auto& red_bulb = group.bulbs[0];
  EXPECT_EQ(red_bulb.id, "RedBulb");
  EXPECT_EQ(red_bulb.color, maliput::api::rules::BulbColor::kRed);
  EXPECT_EQ(red_bulb.type, maliput::api::rules::BulbType::kRound);
  EXPECT_NEAR(red_bulb.position_bulb_group.x(), 0.0, kTolerance);
  EXPECT_NEAR(red_bulb.position_bulb_group.z(), 0.4, kTolerance);
  EXPECT_EQ(red_bulb.states.size(), 3);

  // Check rule conditions
  EXPECT_EQ(dut.rule_states.size(), 2);
  EXPECT_EQ(dut.rule_states[0].rule_value, "Stop");
  EXPECT_EQ(dut.rule_states[0].bulb_conditions.size(), 3);
  EXPECT_EQ(dut.rule_states[1].rule_value, "Go");
  EXPECT_EQ(dut.rule_states[1].bulb_conditions.size(), 3);
}

GTEST_TEST(TrafficSignalParserTest, ValidateArrowSignal) {
  const auto signal_definitions = TrafficSignalParser::LoadFromString(kTrafficSignalDb);
  const auto& dut = signal_definitions.at("1000011");

  EXPECT_EQ(dut.type, "1000011");
  EXPECT_EQ(dut.subtype, 10);
  EXPECT_EQ(dut.description, "Traffic light with arrow");

  const auto& group = dut.bulb_group;
  EXPECT_EQ(group.bulbs.size(), 1);

  const auto& arrow_bulb = group.bulbs[0];
  EXPECT_EQ(arrow_bulb.type, maliput::api::rules::BulbType::kArrow);
  EXPECT_TRUE(arrow_bulb.arrow_orientation_rad.has_value());
  EXPECT_NEAR(arrow_bulb.arrow_orientation_rad.value(), 1.570796, kTolerance);
}

}  // namespace
}  // namespace test
}  // namespace traffic_signal
}  // namespace malidrive
