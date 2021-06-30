// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/road_header.h"

namespace malidrive {
namespace xodr {
namespace {

// Map for HandTrafficRule to string conversion.
const std::map<RoadHeader::HandTrafficRule, std::string> hand_traffic_rule_to_str_map{
    {RoadHeader::HandTrafficRule::kRHT, "RHT"}, {RoadHeader::HandTrafficRule::kLHT, "LHT"}};

// Map for string to HandTrafficRule conversion.
const std::map<std::string, RoadHeader::HandTrafficRule> str_to_hand_traffic_rule_map{
    {"RHT", RoadHeader::HandTrafficRule::kRHT}, {"LHT", RoadHeader::HandTrafficRule::kLHT}};

}  // namespace

std::string RoadHeader::hand_traffic_rule_to_str(RoadHeader::HandTrafficRule rule) {
  return hand_traffic_rule_to_str_map.at(rule);
}

RoadHeader::HandTrafficRule RoadHeader::str_to_hand_traffic_rule(const std::string& rule) {
  MALIDRIVE_THROW_UNLESS(str_to_hand_traffic_rule_map.find(rule) != str_to_hand_traffic_rule_map.end());
  return str_to_hand_traffic_rule_map.at(rule);
}

double RoadHeader::GetLaneSectionLength(int index) const {
  MALIDRIVE_THROW_UNLESS(index >= 0);
  MALIDRIVE_THROW_UNLESS(index < static_cast<int>(lanes.lanes_section.size()));
  if (index == static_cast<int>(lanes.lanes_section.size()) - 1) {
    return length - (lanes.lanes_section[index].s_0 - lanes.lanes_section[0].s_0);
  } else {
    return lanes.lanes_section[index + 1].s_0 - lanes.lanes_section[index].s_0;
  }
}

int RoadHeader::GetLaneSectionIndex(double s) const {
  const int size{static_cast<int>(lanes.lanes_section.size())};
  for (int index = 0; index < size; index++) {
    const double s_0{lanes.lanes_section[index].s_0};
    if (s >= s_0 && s < s_0 + GetLaneSectionLength(index)) {
      return index;
    }
  }
  MALIDRIVE_THROW_MESSAGE(std::string("Coordinate s: ") + std::to_string(s) +
                          std::string(" is not part of any LaneSection of Road Id: ") + id.string());
}

double RoadHeader::GetRoadTypeLength(int index) const {
  MALIDRIVE_THROW_UNLESS(index >= 0);
  MALIDRIVE_THROW_UNLESS(index < static_cast<int>(road_types.size()));
  if (index == static_cast<int>(road_types.size()) - 1) {
    return length - (road_types[index].s_0 - road_types[0].s_0);
  } else {
    return road_types[index + 1].s_0 - road_types[index].s_0;
  }
}

const RoadType* RoadHeader::GetRoadType(double s) const {
  const int size{static_cast<int>(road_types.size())};
  if (size == 0) {
    return nullptr;
  }
  for (int index = 0; index < size; index++) {
    const double s_0{road_types[index].s_0};
    if (s >= s_0 && s < s_0 + GetRoadTypeLength(index)) {
      return &road_types[index];
    }
  }
  MALIDRIVE_THROW_MESSAGE(std::string("Coordinate s: ") + std::to_string(s) +
                          std::string(" is not part of any RoadType of Road Id: ") + id.string());
}

std::vector<const RoadType*> RoadHeader::GetRoadTypesInRange(double s_start, double s_end) const {
  MALIDRIVE_THROW_UNLESS(s_start < s_end);
  MALIDRIVE_THROW_UNLESS(s_start >= 0.);
  // TODO(#567): Verify that `s_end` is not greater than road's length.
  //             Tolerance is needed.
  std::vector<const RoadType*> road_types_in_range;
  for (int index = 0; index < static_cast<int>(road_types.size()); index++) {
    const RoadType& type = road_types[index];
    const double type_s_1 = GetRoadTypeLength(index) + type.s_0;
    if (type.s_0 <= s_start && s_start < type_s_1) {
      road_types_in_range.push_back(&type);
    } else if (type.s_0 < s_end && s_end <= type_s_1) {
      road_types_in_range.push_back(&type);
    } else if (s_start < type.s_0 && s_end >= type_s_1) {
      road_types_in_range.push_back(&type);
    }
  }
  return road_types_in_range;
}

double RoadHeader::s0() const { return reference_geometry.plan_view.geometries.begin()->s_0; }

double RoadHeader::s1() const {
  return (reference_geometry.plan_view.geometries.end() - 1)->s_0 +
         (reference_geometry.plan_view.geometries.end() - 1)->length;
}

bool RoadHeader::operator==(const RoadHeader& other) const {
  return name == other.name && length == other.length && id == other.id && junction == other.junction &&
         rule == other.rule && road_link == other.road_link && road_types == other.road_types &&
         reference_geometry == other.reference_geometry && lanes == other.lanes;
}

bool RoadHeader::operator!=(const RoadHeader& other) const { return !(*this == other); }

std::ostream& operator<<(std::ostream& out, const RoadHeader& road_header) {
  out << "{ \"name\": {" << (road_header.name.has_value() ? road_header.name.value() : "") << "}";
  out << ", \"length\": " << road_header.length;
  out << ", \"id\": " << road_header.id.string();
  out << ", \"junction\": " << road_header.junction;
  out << "}";
  // TODO(malidrive#574): Once other entities acquire serialization overloads, we should include them.
  return out;
}

}  // namespace xodr
}  // namespace malidrive
