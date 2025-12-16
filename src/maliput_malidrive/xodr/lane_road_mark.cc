// BSD 3-Clause License
//
// Copyright (c) 2025, Woven Planet. All rights reserved.
// Copyright (c) 2020-2025, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/xodr/lane_road_mark.h"

namespace malidrive {
namespace xodr {
namespace {

const std::map<LaneRoadMark::LaneChange, std::string> lane_change_to_str_map{
    {LaneRoadMark::LaneChange::kBoth, "both"},
    {LaneRoadMark::LaneChange::kDecrease, "decrease"},
    {LaneRoadMark::LaneChange::kIncrease, "increase"},
    {LaneRoadMark::LaneChange::kNone, "none"},
};

const std::map<std::string, LaneRoadMark::LaneChange> str_to_lane_change_map{
    {"both", LaneRoadMark::LaneChange::kBoth},
    {"decrease", LaneRoadMark::LaneChange::kDecrease},
    {"increase", LaneRoadMark::LaneChange::kIncrease},
    {"none", LaneRoadMark::LaneChange::kNone},
};

const std::map<std::string, LaneRoadMark::Type> str_to_type_map{
    {"botts dots", LaneRoadMark::Type::kBottsDots},
    {"broken broken", LaneRoadMark::Type::kBrokenBroken},
    {"broken solid", LaneRoadMark::Type::kBrokenSolid},
    {"broken", LaneRoadMark::Type::kBroken},
    {"curb", LaneRoadMark::Type::kCurb},
    {"custom", LaneRoadMark::Type::kCustom},
    {"edge", LaneRoadMark::Type::kEdge},
    {"grass", LaneRoadMark::Type::kGrass},
    {"none", LaneRoadMark::Type::kNone},
    {"solid broken", LaneRoadMark::Type::kSolidBroken},
    {"solid solid", LaneRoadMark::Type::kSolidSolid},
    {"solid", LaneRoadMark::Type::kSolid},
};

const std::map<LaneRoadMark::Type, std::string> type_to_str_map{
    {LaneRoadMark::Type::kBottsDots, "botts dots"},
    {LaneRoadMark::Type::kBrokenBroken, "broken broken"},
    {LaneRoadMark::Type::kBrokenSolid, "broken solid"},
    {LaneRoadMark::Type::kBroken, "broken"},
    {LaneRoadMark::Type::kCurb, "curb"},
    {LaneRoadMark::Type::kCustom, "custom"},
    {LaneRoadMark::Type::kEdge, "edge"},
    {LaneRoadMark::Type::kGrass, "grass"},
    {LaneRoadMark::Type::kNone, "none"},
    {LaneRoadMark::Type::kSolidBroken, "solid broken"},
    {LaneRoadMark::Type::kSolidSolid, "solid solid"},
    {LaneRoadMark::Type::kSolid, "solid"},
};

const std::map<LaneRoadMark::Weight, std::string> weight_to_str_map{
    {LaneRoadMark::Weight::kBold, "bold"},
    {LaneRoadMark::Weight::kStandard, "standard"},
};

const std::map<std::string, LaneRoadMark::Weight> str_to_weight_map{
    {"bold", LaneRoadMark::Weight::kBold},
    {"standard", LaneRoadMark::Weight::kStandard},
};

const std::map<Rule, std::string> rule_to_str_map{
    {Rule::kCaution, "caution"},
    {Rule::kNoPassing, "no passing"},
    {Rule::kNone, "none"},
};

const std::map<std::string, Rule> str_to_rule_map{
    {"caution", Rule::kCaution},
    {"no passing", Rule::kNoPassing},
    {"none", Rule::kNone},
};

}  // namespace

/// Roadmark

std::string LaneRoadMark::lane_change_to_str(LaneChange lane_change) { return lane_change_to_str_map.at(lane_change); }

LaneRoadMark::LaneChange LaneRoadMark::str_to_lane_change(const std::string& lane_change) {
  if (str_to_lane_change_map.find(lane_change) == str_to_lane_change_map.end()) {
    MALIDRIVE_THROW_MESSAGE(lane_change + " lane road mark lane change is not available.");
  }
  return str_to_lane_change_map.at(lane_change);
}

std::string LaneRoadMark::type_to_str(Type type) { return type_to_str_map.at(type); }

LaneRoadMark::Type LaneRoadMark::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " lane road mark type is not available.");
  }
  return str_to_type_map.at(type);
}

std::string LaneRoadMark::weight_to_str(Weight weight) { return weight_to_str_map.at(weight); }

LaneRoadMark::Weight LaneRoadMark::str_to_weight(const std::string& weight) {
  if (str_to_weight_map.find(weight) == str_to_weight_map.end()) {
    MALIDRIVE_THROW_MESSAGE(weight + " lane road mark weight is not available.");
  }
  return str_to_weight_map.at(weight);
}

bool LaneRoadMark::operator==(const LaneRoadMark& other) const {
  return color == other.color && height == other.height && lane_change == other.lane_change &&
         material == other.material && s_offset == other.s_offset && type == other.type && weight == other.weight &&
         width == other.width && this->type_elems == other.type_elems && explicit_elems == other.explicit_elems &&
         sway_elems == other.sway_elems;
}

bool LaneRoadMark::operator!=(const LaneRoadMark& other) const { return !(*this == other); }

/// Rule

std::string rule_to_str(Rule rule) { return rule_to_str_map.at(rule); }

Rule str_to_rule(const std::string& rule) {
  if (str_to_rule_map.find(rule) == str_to_rule_map.end()) {
    MALIDRIVE_THROW_MESSAGE(rule + " lane road mark line rule is not available.");
  }
  return str_to_rule_map.at(rule);
}

/// TypeElementLine

bool TypeElementLine::operator==(const TypeElementLine& other) const {
  return color == other.color && length == other.length && rule == other.rule && s_offset == other.s_offset &&
         space == other.space && t_offset == other.t_offset && width == other.width;
}

/// TypeElement

bool TypeElement::operator==(const TypeElement& other) const {
  return name == other.name && width == other.width && lines == other.lines;
}

/// ExplicitElementLine

bool ExplicitElementLine::operator==(const ExplicitElementLine& other) const {
  return length == other.length && rule == other.rule && s_offset == other.s_offset && t_offset == other.t_offset &&
         width == other.width;
}

/// ExplicitElement

bool ExplicitElement::operator==(const ExplicitElement& other) const { return lines == other.lines; }

/// SwayElement

bool SwayElement::operator==(const SwayElement& other) const {
  return a == other.a && b == other.b && c == other.c && d == other.d && s_0 == other.s_0;
}

}  // namespace xodr
}  // namespace malidrive
