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
namespace{

const std::map<std::string, LaneRoadMark::Color> str_to_color_map{
    {"black", LaneRoadMark::Color::kBlack},
    {"blue", LaneRoadMark::Color::kBlue},
    {"green", LaneRoadMark::Color::kGreen},
    {"orange", LaneRoadMark::Color::kOrange},
    {"red", LaneRoadMark::Color::kRed},
    {"standard", LaneRoadMark::Color::kStandard},
    {"violet", LaneRoadMark::Color::kViolet},
    {"white", LaneRoadMark::Color::kWhite},
    {"yellow", LaneRoadMark::Color::kYellow},
};

const std::map<LaneRoadMark::Color, std::string> color_to_str_map{
    {LaneRoadMark::Color::kBlack, "black"},
    {LaneRoadMark::Color::kBlue, "blue"},
    {LaneRoadMark::Color::kGreen, "green"},
    {LaneRoadMark::Color::kOrange, "orange"},
    {LaneRoadMark::Color::kRed, "red"},
    {LaneRoadMark::Color::kStandard, "standard"},
    {LaneRoadMark::Color::kViolet, "violet"},
    {LaneRoadMark::Color::kWhite, "white"},
    {LaneRoadMark::Color::kYellow, "yellow"},
};

const std::map<LaneRoadMark::LaneChange, std::string> lane_change_to_str_map{
    {LaneRoadMark::LaneChange::kBoth, "both"},
    {LaneRoadMark::LaneChange::kDecrease, "decrease"},
    {LaneRoadMark::LaneChange::kIncrease, "increase"},
    {LaneRoadMark::LaneChange::kNone, "none"},
};

const std::map<std::string, LaneRoadMark::LaneChange> str_to_lane_change_map{
    {"both",LaneRoadMark::LaneChange::kBoth},
    {"decrease" ,LaneRoadMark::LaneChange::kDecrease},
    {"increase" ,LaneRoadMark::LaneChange::kIncrease},
    {"none" ,LaneRoadMark::LaneChange::kNone},
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

const std::map<std::string, LaneRoadMark::LaneChange> str_to_weight_map{
    {"bold",LaneRoadMark::Weight::kBold},
    {"standard" ,LaneRoadMark::Weight::kStandard},
};

} // namespace

std::string LaneRoadMark::color_to_str(Color color){return color_to_str_map.at(color);}

LaneRoadMark::Color LaneRoadMark::str_to_color(const std::string& color){
    if (str_to_color_map.find(color) == str_to_color_map.end()) {
        MALIDRIVE_THROW_MESSAGE(color + " lane road mark color is not available.");
    }
    return str_to_color_map.at(color);
}

std::string LaneRoadMark::lane_change_to_str(LaneChange lane_change){return lane_change_to_str_map.at(lane_change);}

LaneRoadMark::LaneChange LaneRoadMark::str_to_lane_change(const std::string& lane_change){
    if (str_to_lane_change_map.find(lane_change) == str_to_lane_change_map.end()) {
        MALIDRIVE_THROW_MESSAGE(lane_change + " lane road mark lane change is not available.");
    }
    return str_to_lane_change_map.at(lane_change);
}

std::string LaneRoadMark::type_to_str(Type type){return type_to_str_map.at(type);}

LaneRoadMark::Type LaneRoadMark::str_to_type(const std::string& type){
    if (str_to_type_map.find(type) == str_to_type_map.end()) {
        MALIDRIVE_THROW_MESSAGE(type + " lane road mark type is not available.");
    }
    return str_to_type_map.at(type);
}

std::string LaneRoadMark::weight_to_str(Weight weight){return weight_to_str_map.at(weight);}

LaneRoadMark::Weight LaneRoadMark::str_to_weight(const std::string& weight){
    if (str_to_weight_map.find(weight) == str_to_weight_map.end()) {
        MALIDRIVE_THROW_MESSAGE(weight + " lane road mark weight is not available.");
    }
    return str_to_weight_map.at(weight);
}

bool LaneRoadMark::operator==(const LaneRoadMark& other) const {
    // TODO
}

bool LaneRoadMark::operator!=(const LaneRoadMark& other) const { return !(*this == other); }

}  // namespace xodr
}  // namespace malidrive
