// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/parser.h"

#include <map>
#include <vector>

#include <maliput/common/logger.h>

#include "maliput_malidrive/xodr/connection.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/geometry.h"
#include "maliput_malidrive/xodr/header.h"
#include "maliput_malidrive/xodr/junction.h"
#include "maliput_malidrive/xodr/lane.h"
#include "maliput_malidrive/xodr/lane_link.h"
#include "maliput_malidrive/xodr/lane_offset.h"
#include "maliput_malidrive/xodr/lane_section.h"
#include "maliput_malidrive/xodr/lane_width.h"
#include "maliput_malidrive/xodr/lanes.h"
#include "maliput_malidrive/xodr/lateral_profile.h"
#include "maliput_malidrive/xodr/plan_view.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/road_link.h"
#include "maliput_malidrive/xodr/road_type.h"
#include "maliput_malidrive/xodr/unit.h"

namespace malidrive {
namespace xodr {
namespace {

// Determines whether `geometry_a` and `geometry_b` geometries are continguos in terms of the arc length parameter.
bool IsContiguous(const Geometry& geometry_a, const Geometry& geometry_b, double tolerance) {
  MALIDRIVE_THROW_UNLESS(tolerance >= 0);
  return std::abs(geometry_a.s_0 + geometry_a.length - geometry_b.s_0) <= tolerance;
}

// Adds `new_function` description into `functions` collection.
// The functions descriptions defines several aspect of a Road in the xodr like elevation, superelevation, lane
// offset and lane width of the lanes.
// The type `T` should define the following members:
//     - s_0: start position.
//     - a,b,c and d : Coefficients of a cubic polynomial: @f$ a + b * (s - s_0) + c * (s - s_0)^2 + d * (s - s_0)^3
//     @f$.
// In addition, equal operator must be overloaded.
//
//  - When `new_function` description is identical to `functions->back()` description then the latter is discarded and
//  replaced by `new_function`.
//  - When `allow_semantic_errors` is true and only `new_function.s_0` value is equal to `functions->back().s_0` value
//  then latter description is discarded and replaced by `new_function`.
//  - When `allow_semantic_errors` is false and only `new_function.s_0` value is equal to `functions->back().s_0` value
//  it throws.
//
// @param new_function New function to be added.
// @param node_id Name of the xml node under analysis, used for proper logging messages.
// @param allow_semantic_errors Indicates the permittivity.
// @param xml_node XML node that is used to improve logging messages.
// @param functions Collection of functions.
//
// @throws maliput::common::assertion_error When `allow_semantic_errors` is false and only `new_function.s_0` value is
// equal to `functions->back().s_0` value.
template <typename T>
void AddPolynomialDescriptionToCollection(const T& new_function, const std::string& node_id, bool allow_semantic_errors,
                                          tinyxml2::XMLElement* xml_node, std::vector<T>* functions) {
  if (!functions->empty()) {
    if (new_function == functions->back()) {
      std::string msg{node_id + " node describes two identical functions:\n" + ConvertXMLNodeToText(xml_node) +
                      "Discarding the first repeated description."};
      DuplicateCurlyBracesForFmtLogging(&msg);
      maliput::log()->trace(msg);
      functions->pop_back();
    } else if (new_function.s_0 == functions->back().s_0) {
      // Comparing double values is controversial. However, the values here share the same origin:
      // They come from the tinyxml2's parser so it is expected that if they are the same in the xodr file description
      // then they are the same after being parsed.
      // (even though not necessarily the values in the XML and the values after parsing are the same).
      std::string msg{node_id + " node describes two functions starting at the same s:\n" +
                      ConvertXMLNodeToText(xml_node)};
      DuplicateCurlyBracesForFmtLogging(&msg);
      if (!allow_semantic_errors) {
        MALIDRIVE_THROW_MESSAGE(msg);
      }
      maliput::log()->warn(msg + "Discarding the first description starting at s = {}", new_function.s_0);
      functions->pop_back();
    }
  }
  functions->push_back(std::move(new_function));
}

}  // namespace

int ParserBase::NumberOfAttributes() const {
  std::vector<const tinyxml2::XMLAttribute*> attributes;
  const tinyxml2::XMLAttribute* first_attribute = element_->FirstAttribute();
  attributes.push_back(first_attribute);
  while (attributes[attributes.size() - 1] != nullptr) {
    const tinyxml2::XMLAttribute* attribute = attributes[attributes.size() - 1]->Next();
    attributes.push_back(attribute);
  }
  return attributes.size() - 1;
}

// Specialization to parse as `double` the attribute's value.
template <>
std::optional<double> AttributeParser::As(const std::string& attribute_name) const {
  double value{};
  const auto result = element_->QueryDoubleAttribute(attribute_name.c_str(), &value);
  if (result != tinyxml2::XML_SUCCESS) {
    return std::nullopt;
  }
  if (!std::isnan(value)) {
    return std::make_optional(value);
  }
  const std::string serialized_node{ConvertXMLNodeToText(element_)};
  maliput::log()->error("Attributes with NaN values are not supported. \n {}", serialized_node);
  MALIDRIVE_THROW_MESSAGE("Attributes with NaN values are not supported. " + serialized_node);
}

// Specialization to parse as `std::string` the attribute's value.
template <>
std::optional<std::string> AttributeParser::As(const std::string& attribute_name) const {
  const char* attribute_as_str_ptr = element_->Attribute(attribute_name.c_str());
  return attribute_as_str_ptr != nullptr ? std::make_optional(static_cast<std::string>(attribute_as_str_ptr))
                                         : std::nullopt;
}

// Specialization to parse as `bool` the attribute's value.
template <>
std::optional<bool> AttributeParser::As(const std::string& attribute_name) const {
  const std::string kTrueStr = "true";
  const std::string kTrueNum = "1";
  const std::string kFalseStr = "false";
  const std::string kFalseNum = "0";
  const char* attribute_as_str_ptr = element_->Attribute(attribute_name.c_str());
  if (attribute_as_str_ptr == nullptr) return std::nullopt;
  const std::string attribute_value = static_cast<std::string>(attribute_as_str_ptr);
  MALIDRIVE_THROW_UNLESS(attribute_value == kTrueStr || attribute_value == kTrueNum || attribute_value == kFalseStr ||
                         attribute_value == kFalseNum);
  return attribute_value == kTrueStr || attribute_value == kTrueNum;
}

// Specialization to parse as `Lane::Type` the attribute's value.
template <>
std::optional<Lane::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  if (type.has_value()) {
    return Lane::str_to_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `RoadLink::ElementType` the attribute's value.
template <>
std::optional<RoadLink::ElementType> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> element_type = As<std::string>(attribute_name);
  if (element_type.has_value()) {
    return RoadLink::str_to_element_type(element_type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `RoadLink::ContactPoint` the attribute's value.
template <>
std::optional<RoadLink::ContactPoint> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> contact_point = As<std::string>(attribute_name);
  if (contact_point.has_value()) {
    return RoadLink::str_to_contact_point(contact_point.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `Connection::ContactPoint` the attribute's value.
template <>
std::optional<Connection::ContactPoint> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> contact_point = As<std::string>(attribute_name);
  if (contact_point.has_value()) {
    return Connection::str_to_contact_point(contact_point.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `Junction::Type` the attribute's value.
template <>
std::optional<Junction::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  if (type.has_value()) {
    return Junction::str_to_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `Connection::Type` the attribute's value.
template <>
std::optional<Connection::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  if (type.has_value()) {
    return Connection::str_to_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `RoadHeader::HandTrafficRule` the attribute's value.
template <>
std::optional<RoadHeader::HandTrafficRule> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> rule = As<std::string>(attribute_name);
  return rule.has_value()
             ? std::make_optional<RoadHeader::HandTrafficRule>(RoadHeader::str_to_hand_traffic_rule(rule.value()))
             : std::nullopt;
}

// Specialization to parse as `RoadType::Type` the attribute's value.
template <>
std::optional<RoadType::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  return type.has_value() ? std::make_optional<RoadType::Type>(RoadType::str_to_type(type.value())) : std::nullopt;
}

// Specialization to parse as `Unit` the attribute's value.
template <>
std::optional<Unit> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> unit = As<std::string>(attribute_name);
  return unit.has_value() ? std::make_optional<Unit>(str_to_unit(unit.value())) : std::nullopt;
}

// Specialization to parse `Header`'s node.
template <>
Header NodeParser::As() const {
  Header header{};
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto rev_major = attribute_parser.As<double>(Header::kXodrRevMajor);
  MALIDRIVE_THROW_UNLESS(rev_major != std::nullopt);
  header.rev_major = rev_major.value();

  const auto rev_minor = attribute_parser.As<double>(Header::kXodrRevMinor);
  MALIDRIVE_THROW_UNLESS(rev_minor != std::nullopt);
  header.rev_minor = rev_minor.value();
  // @}

  // Optional attributes.
  // @{
  header.name = attribute_parser.As<std::string>(Header::kXodrName);
  header.date = attribute_parser.As<std::string>(Header::kXodrDate);
  header.version = attribute_parser.As<double>(Header::kXodrVersion);
  header.north = attribute_parser.As<double>(Header::kXodrNorth);
  header.south = attribute_parser.As<double>(Header::kXodrSouth);
  header.east = attribute_parser.As<double>(Header::kXodrEast);
  header.west = attribute_parser.As<double>(Header::kXodrWest);
  header.vendor = attribute_parser.As<std::string>(Header::kXodrVendor);
  // @}

  return header;
}

// Specialization to parse `RoadLink::LinkAttributes`'s node.
template <>
RoadLink::LinkAttributes NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const auto element_type = attribute_parser.As<RoadLink::ElementType>(RoadLink::LinkAttributes::kElementType);
  MALIDRIVE_THROW_UNLESS(element_type != std::nullopt);
  const auto element_id = attribute_parser.As<std::string>(RoadLink::LinkAttributes::kElementId);
  MALIDRIVE_THROW_UNLESS(element_id != std::nullopt);

  const auto contact_point = attribute_parser.As<RoadLink::ContactPoint>(RoadLink::LinkAttributes::kContactPoint);
  switch (*element_type) {
    case RoadLink::ElementType::kRoad:
      MALIDRIVE_THROW_UNLESS(contact_point != std::nullopt);
      break;
    case RoadLink::ElementType::kJunction:
      MALIDRIVE_THROW_UNLESS(contact_point == std::nullopt);
      break;
    default:
      MALIDRIVE_THROW_MESSAGE("Invalid elementType value for RoadLink's description.");
      break;
  }
  return {*element_type, RoadLink::LinkAttributes::Id(*element_id), contact_point};
}

// Specialization to parse `RoadLink`'s node.
template <>
RoadLink NodeParser::As() const {
  RoadLink road_link{};
  tinyxml2::XMLElement* predecessor_element(element_->FirstChildElement(RoadLink::kPredecessorTag));
  if (predecessor_element != nullptr) {
    road_link.predecessor = NodeParser(predecessor_element, parser_configuration_).As<RoadLink::LinkAttributes>();
  }
  tinyxml2::XMLElement* successor_element(element_->FirstChildElement(RoadLink::kSuccessorTag));
  if (successor_element != nullptr) {
    road_link.successor = NodeParser(successor_element, parser_configuration_).As<RoadLink::LinkAttributes>();
  }
  return road_link;
}

// Specialization to parse `Line`'s node.
template <>
Geometry::Line NodeParser::As() const {
  if (NumberOfAttributes()) {
    MALIDRIVE_THROW_MESSAGE(std::string("Bad Line description. Line node doesn't allow attributes: ") +
                            ConvertXMLNodeToText(element_));
  }
  return Geometry::Line{};
}

// Specialization to parse `Arc`'s node.
template <>
Geometry::Arc NodeParser::As() const {
  if (NumberOfAttributes() != 1) {
    MALIDRIVE_THROW_MESSAGE(std::string("Bad Arc description. Arc demands only one argument: 'curvature'. ") +
                            ConvertXMLNodeToText(element_));
  }
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const auto curvature = attribute_parser.As<double>(Geometry::Arc::kCurvature);
  MALIDRIVE_THROW_UNLESS(curvature != std::nullopt);
  return Geometry::Arc{curvature.value()};
}

// Specialization to parse `LaneWidth`'s node.
template <>
LaneWidth NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto offset = attribute_parser.As<double>(LaneWidth::kOffset);
  MALIDRIVE_THROW_UNLESS(offset != std::nullopt);

  const auto a_param = attribute_parser.As<double>(LaneWidth::kA);
  MALIDRIVE_THROW_UNLESS(a_param != std::nullopt);

  const auto b_param = attribute_parser.As<double>(LaneWidth::kB);
  MALIDRIVE_THROW_UNLESS(b_param != std::nullopt);

  const auto c_param = attribute_parser.As<double>(LaneWidth::kC);
  MALIDRIVE_THROW_UNLESS(c_param != std::nullopt);

  const auto d_param = attribute_parser.As<double>(LaneWidth::kD);
  MALIDRIVE_THROW_UNLESS(d_param != std::nullopt);
  // @}
  return {offset.value(), a_param.value(), b_param.value(), c_param.value(), d_param.value()};
}

// Specialization to parse `LaneOffset`'s node.
template <>
LaneOffset NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto s_0 = attribute_parser.As<double>(LaneOffset::kS0);
  MALIDRIVE_THROW_UNLESS(s_0 != std::nullopt);

  const auto a_param = attribute_parser.As<double>(LaneOffset::kA);
  MALIDRIVE_THROW_UNLESS(a_param != std::nullopt);

  const auto b_param = attribute_parser.As<double>(LaneWidth::kB);
  MALIDRIVE_THROW_UNLESS(b_param != std::nullopt);

  const auto c_param = attribute_parser.As<double>(LaneWidth::kC);
  MALIDRIVE_THROW_UNLESS(c_param != std::nullopt);

  const auto d_param = attribute_parser.As<double>(LaneWidth::kD);
  MALIDRIVE_THROW_UNLESS(d_param != std::nullopt);
  // @}
  return {s_0.value(), a_param.value(), b_param.value(), c_param.value(), d_param.value()};
}

// Specialization to parse `LaneLink::LinkAttributes`'s node.
template <>
LaneLink::LinkAttributes NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const auto id = attribute_parser.As<std::string>(LaneLink::LinkAttributes::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt);
  return {LaneLink::LinkAttributes::Id(*id)};
}

// Specialization to parse `LaneLink`'s node.
template <>
LaneLink NodeParser::As() const {
  LaneLink road_link{};
  tinyxml2::XMLElement* predecessor_element(element_->FirstChildElement(LaneLink::kPredecessorTag));
  if (predecessor_element != nullptr) {
    road_link.predecessor = NodeParser(predecessor_element, parser_configuration_).As<LaneLink::LinkAttributes>();
  }
  tinyxml2::XMLElement* successor_element(element_->FirstChildElement(LaneLink::kSuccessorTag));
  if (successor_element != nullptr) {
    road_link.successor = NodeParser(successor_element, parser_configuration_).As<LaneLink::LinkAttributes>();
  }
  return road_link;
}

// Specialization to parse `RoadType::Speed`'s node.
template <>
RoadType::Speed NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  RoadType::Speed speed{};

  const auto max = attribute_parser.As<std::string>(RoadType::Speed::kMax);
  MALIDRIVE_THROW_UNLESS(max != std::nullopt);
  speed.max = *max != RoadType::Speed::kUnlimitedSpeedStrings[0] && *max != RoadType::Speed::kUnlimitedSpeedStrings[1]
                  ? std::make_optional<double>(std::stod(max.value()))
                  : std::nullopt;

  const auto unit = attribute_parser.As<Unit>(RoadType::Speed::kUnit);
  speed.unit = unit.has_value() ? unit.value() : Unit::kMs;
  return speed;
}

// Specialization to parse `RoadType`'s node.
template <>
RoadType NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  RoadType road_type{};

  const auto s_0 = attribute_parser.As<double>(RoadType::kS0);
  MALIDRIVE_THROW_UNLESS(s_0 != std::nullopt);
  road_type.s_0 = *s_0;

  const auto type = attribute_parser.As<RoadType::Type>(RoadType::kRoadTypeTag);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt);
  road_type.type = *type;

  road_type.country = attribute_parser.As<std::string>(RoadType::kCountry);

  tinyxml2::XMLElement* speed_element = element_->FirstChildElement(RoadType::Speed::kSpeedTag);
  if (speed_element != nullptr) {
    road_type.speed = NodeParser(speed_element, parser_configuration_).As<RoadType::Speed>();
  }
  return road_type;
}

// Specialization to parse `Lane::Speed`'s node.
template <>
Lane::Speed NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  Lane::Speed speed{};

  const auto s_offset = attribute_parser.As<double>(Lane::Speed::kSOffset);
  MALIDRIVE_THROW_UNLESS(s_offset != std::nullopt);
  speed.s_offset = s_offset.value();

  const auto max = attribute_parser.As<double>(Lane::Speed::kMax);
  MALIDRIVE_THROW_UNLESS(max != std::nullopt);
  speed.max = max.value();

  const auto unit = attribute_parser.As<Unit>(Lane::Speed::kUnit);
  speed.unit = unit.has_value() ? unit.value() : Unit::kMs;
  return speed;
}

// Specialization to parse `Lane`'s node.
template <>
Lane NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(Lane::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt);

  const auto type = attribute_parser.As<Lane::Type>(Lane::kType);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt);
  // @}

  // Optional attributes.
  // @{
  const auto level = attribute_parser.As<bool>(Lane::kLevel);
  // @}

  // Elements.
  LaneLink lane_link{};
  tinyxml2::XMLElement* lane_link_element = element_->FirstChildElement(LaneLink::kLaneLinkTag);
  if (lane_link_element != nullptr) {
    lane_link = NodeParser(lane_link_element, parser_configuration_).As<LaneLink>();
  }

  tinyxml2::XMLElement* width_element = element_->FirstChildElement(LaneWidth::kLaneWidthTag);
  std::vector<LaneWidth> width_description;
  while (width_element) {
    auto lane_width = NodeParser(width_element, parser_configuration_).As<LaneWidth>();

    AddPolynomialDescriptionToCollection(std::move(lane_width), Lane::kLaneTag,
                                         parser_configuration_.allow_semantic_errors, element_, &width_description);

    width_element = width_element->NextSiblingElement(LaneWidth::kLaneWidthTag);
  }

  tinyxml2::XMLElement* speed_element = element_->FirstChildElement(Lane::Speed::kSpeedTag);
  std::vector<Lane::Speed> speeds;
  while (speed_element) {
    speeds.push_back(NodeParser(speed_element, parser_configuration_).As<Lane::Speed>());
    speed_element = speed_element->NextSiblingElement(Lane::Speed::kSpeedTag);
  }

  std::optional<std::string> user_data{std::nullopt};
  tinyxml2::XMLElement* user_data_element = element_->FirstChildElement(Lane::kUserData);
  if (user_data_element != nullptr) {
    user_data = ConvertXMLNodeToText(user_data_element);
  }
  return {Lane::Id(id.value()), type.value(), level, lane_link, width_description, speeds, user_data};
}

namespace {

// Returns all the lanes located in `element`.
// `is_center_node` specifies whether the lanes that are being parsed are center lanes or not.
std::vector<Lane> GetAllLanesFromNode(tinyxml2::XMLElement* element, bool is_center_node,
                                      const ParserConfiguration& parse_configuration) {
  std::vector<Lane> lanes;
  tinyxml2::XMLElement* lane_element_ptr = element->FirstChildElement(Lane::kLaneTag);
  while (lane_element_ptr != nullptr) {
    const NodeParser node_parser(lane_element_ptr, parse_configuration);
    const Lane lane = node_parser.As<Lane>();
    // Center lanes must not have `widths` description.
    // While right and left lanes need at least one width entry.
    MALIDRIVE_THROW_UNLESS(is_center_node ? lane.width_description.size() == 0 : lane.width_description.size() > 0);
    // Center lanes must not have `speed` records.
    MALIDRIVE_THROW_UNLESS(is_center_node ? lane.speed.size() == 0 : true);
    MALIDRIVE_TRACE("Lane id: '" + lane.id.string() + "' parsed.");
    lanes.insert(lanes.begin(), lane);
    lane_element_ptr = lane_element_ptr->NextSiblingElement(Lane::kLaneTag);
  }
  return lanes;
}

}  // namespace

// Specialization to parse `LaneSection`'s node.
template <>
LaneSection NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  // Non-optional attributes.
  // @{
  const std::optional<double> s_0 = attribute_parser.As<double>(LaneSection::kS0);
  MALIDRIVE_THROW_UNLESS(s_0 != std::nullopt);
  // @}

  // Optional attributes.
  // @{
  const std::optional<bool> single_side = attribute_parser.As<bool>(LaneSection::kSingleSide);
  // @}

  // Elements.
  // Left lanes.
  MALIDRIVE_TRACE("Parsing left lanes.");
  std::vector<Lane> left_lanes;
  tinyxml2::XMLElement* left_element_ptr = element_->FirstChildElement(LaneSection::kLeft);
  if (left_element_ptr != nullptr) {
    left_lanes = GetAllLanesFromNode(left_element_ptr, false, parser_configuration_);
  }
  // Center lane.
  MALIDRIVE_TRACE("Parsing center lane.");
  tinyxml2::XMLElement* center_element_ptr = element_->FirstChildElement(LaneSection::kCenter);
  MALIDRIVE_THROW_UNLESS(center_element_ptr != nullptr);
  std::vector<Lane> center_lanes = GetAllLanesFromNode(center_element_ptr, true, parser_configuration_);
  MALIDRIVE_THROW_UNLESS(center_lanes.size() == 1);
  // Right lanes.
  MALIDRIVE_TRACE("Parsing right lanes.");
  std::vector<Lane> right_lanes;
  tinyxml2::XMLElement* right_element_ptr = element_->FirstChildElement(LaneSection::kRight);
  if (right_element_ptr != nullptr) {
    right_lanes = GetAllLanesFromNode(right_element_ptr, false, parser_configuration_);
  }

  return {s_0.value(), single_side, left_lanes, center_lanes[0], right_lanes};
}

// Specialization to parse `Lanes`'s node.
template <>
Lanes NodeParser::As() const {
  // Optional element.
  MALIDRIVE_TRACE("Parsing laneOffset.");
  std::vector<LaneOffset> lanes_offsets;
  tinyxml2::XMLElement* lane_offset_element_ptr = element_->FirstChildElement(LaneOffset::kLaneOffsetTag);
  while (lane_offset_element_ptr != nullptr) {
    auto lane_offset = NodeParser(lane_offset_element_ptr, parser_configuration_).As<LaneOffset>();
    AddPolynomialDescriptionToCollection(std::move(lane_offset), LaneOffset::kLaneOffsetTag,
                                         parser_configuration_.allow_semantic_errors, element_, &lanes_offsets);
    lane_offset_element_ptr = lane_offset_element_ptr->NextSiblingElement(LaneOffset::kLaneOffsetTag);
  }
  // Non optional element.
  MALIDRIVE_TRACE("Parsing all laneSections.");
  std::vector<LaneSection> lanes_section;
  tinyxml2::XMLElement* lane_section_element_ptr = element_->FirstChildElement(LaneSection::kLaneSectionTag);
  int index{};
  while (lane_section_element_ptr != nullptr) {
    MALIDRIVE_TRACE("Parsing laneSection #" + std::to_string(index));
    const NodeParser node_parser(lane_section_element_ptr, parser_configuration_);
    lanes_section.push_back(node_parser.As<LaneSection>());
    lane_section_element_ptr = lane_section_element_ptr->NextSiblingElement(LaneSection::kLaneSectionTag);
    index++;
  }
  // At least one lane section must be defined for each road.
  MALIDRIVE_THROW_UNLESS(lanes_section.size() > 0);

  return {lanes_offsets, lanes_section};
}

// Specialization to parse `Geometry`'s node.
template <>
Geometry NodeParser::As() const {
  Geometry geometry{};
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto s_0 = attribute_parser.As<double>(Geometry::kS0);
  MALIDRIVE_THROW_UNLESS(s_0 != std::nullopt);
  geometry.s_0 = s_0.value();

  const auto x = attribute_parser.As<double>(Geometry::kStartPointX);
  MALIDRIVE_THROW_UNLESS(x != std::nullopt);
  geometry.start_point.x() = x.value();

  const auto y = attribute_parser.As<double>(Geometry::kStartPointY);
  MALIDRIVE_THROW_UNLESS(y != std::nullopt);
  geometry.start_point.y() = y.value();

  const auto orientation = attribute_parser.As<double>(Geometry::kOrientation);
  MALIDRIVE_THROW_UNLESS(orientation != std::nullopt);
  geometry.orientation = orientation.value();

  const auto length = attribute_parser.As<double>(Geometry::kLength);
  MALIDRIVE_THROW_UNLESS(length != std::nullopt);
  geometry.length = length.value();

  const NodeParser geometry_type(element_->FirstChildElement(), parser_configuration_);
  geometry.type = Geometry::str_to_type(geometry_type.GetName());
  switch (geometry.type) {
    case Geometry::Type::kLine:
      geometry.description = geometry_type.As<Geometry::Line>();
      break;
    case Geometry::Type::kArc:
      geometry.description = geometry_type.As<Geometry::Arc>();
      break;
    default:
      MALIDRIVE_THROW_MESSAGE(std::string("The Geometry type '") + Geometry::type_to_str(geometry.type) +
                              std::string("' is not supported."));
  }
  // @}
  return geometry;
}

// Specialization to parse `elevation`'s node.
template <>
ElevationProfile::Elevation NodeParser::As() const {
  ElevationProfile::Elevation elevation{};
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto s_0 = attribute_parser.As<double>(ElevationProfile::Elevation::kS0);
  MALIDRIVE_THROW_UNLESS(s_0 != std::nullopt);
  elevation.s_0 = s_0.value();

  const auto a = attribute_parser.As<double>(ElevationProfile::Elevation::kA);
  MALIDRIVE_THROW_UNLESS(a != std::nullopt);
  elevation.a = a.value();

  const auto b = attribute_parser.As<double>(ElevationProfile::Elevation::kB);
  MALIDRIVE_THROW_UNLESS(b != std::nullopt);
  elevation.b = b.value();

  const auto c = attribute_parser.As<double>(ElevationProfile::Elevation::kC);
  MALIDRIVE_THROW_UNLESS(c != std::nullopt);
  elevation.c = c.value();

  const auto d = attribute_parser.As<double>(ElevationProfile::Elevation::kD);
  MALIDRIVE_THROW_UNLESS(d != std::nullopt);
  elevation.d = d.value();
  // @}
  return elevation;
}

// Specialization to parse `elevationProfile`'s node.
template <>
ElevationProfile NodeParser::As() const {
  std::vector<ElevationProfile::Elevation> elevations;
  tinyxml2::XMLElement* elevation_element(element_->FirstChildElement(ElevationProfile::Elevation::kElevationTag));
  while (elevation_element) {
    auto elevation = NodeParser(elevation_element, parser_configuration_).As<ElevationProfile::Elevation>();
    AddPolynomialDescriptionToCollection(std::move(elevation), ElevationProfile::kElevationProfileTag,
                                         parser_configuration_.allow_semantic_errors, element_, &elevations);
    elevation_element = elevation_element->NextSiblingElement(ElevationProfile::Elevation::kElevationTag);
  }
  return {elevations};
}

// Specialization to parse `superelevation`'s node.
template <>
LateralProfile::Superelevation NodeParser::As() const {
  LateralProfile::Superelevation superelevation{};
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto s_0 = attribute_parser.As<double>(LateralProfile::Superelevation::kS0);
  MALIDRIVE_THROW_UNLESS(s_0 != std::nullopt);
  superelevation.s_0 = s_0.value();

  const auto a = attribute_parser.As<double>(LateralProfile::Superelevation::kA);
  MALIDRIVE_THROW_UNLESS(a != std::nullopt);
  superelevation.a = a.value();

  const auto b = attribute_parser.As<double>(LateralProfile::Superelevation::kB);
  MALIDRIVE_THROW_UNLESS(b != std::nullopt);
  superelevation.b = b.value();

  const auto c = attribute_parser.As<double>(LateralProfile::Superelevation::kC);
  MALIDRIVE_THROW_UNLESS(c != std::nullopt);
  superelevation.c = c.value();

  const auto d = attribute_parser.As<double>(LateralProfile::Superelevation::kD);
  MALIDRIVE_THROW_UNLESS(d != std::nullopt);
  superelevation.d = d.value();
  // @}
  return superelevation;
}

// Specialization to parse `lateralProfile`'s node.
template <>
LateralProfile NodeParser::As() const {
  std::vector<LateralProfile::Superelevation> superelevations;
  tinyxml2::XMLElement* superelevation_element(
      element_->FirstChildElement(LateralProfile::Superelevation::kSuperelevationTag));
  while (superelevation_element) {
    auto superelevation =
        NodeParser(superelevation_element, parser_configuration_).As<LateralProfile::Superelevation>();
    AddPolynomialDescriptionToCollection(std::move(superelevation), LateralProfile::kLateralProfileTag,
                                         parser_configuration_.allow_semantic_errors, element_, &superelevations);
    superelevation_element =
        superelevation_element->NextSiblingElement(LateralProfile::Superelevation::kSuperelevationTag);
  }
  return {superelevations};
}

// Specialization to parse `PlanView`'s node.
template <>
PlanView NodeParser::As() const {
  std::vector<Geometry> geometries;
  tinyxml2::XMLElement* geometry_element(element_->FirstChildElement(Geometry::kGeometryTag));
  MALIDRIVE_THROW_UNLESS(geometry_element != nullptr);
  while (geometry_element) {
    const NodeParser geometry_node(geometry_element, parser_configuration_);
    auto geometry = geometry_node.As<Geometry>();
    if (parser_configuration_.tolerance.has_value() && geometries.size() > 0) {
      if (!IsContiguous(geometries.back(), geometry, *parser_configuration_.tolerance)) {
        MALIDRIVE_THROW_MESSAGE("Geometries doesn't meet contiguity constraint.");
      };
    }
    geometries.push_back(std::move(geometry));
    geometry_element = geometry_element->NextSiblingElement(Geometry::kGeometryTag);
  }
  return {geometries};
}

// Specialization to parse `Road`'s node.
template <>
RoadHeader NodeParser::As() const {
  RoadHeader road_header{};
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(RoadHeader::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt);
  road_header.id = RoadHeader::Id(id.value());

  MALIDRIVE_TRACE("Parsing road id: " + road_header.id.string());

  const auto length = attribute_parser.As<double>(RoadHeader::kLength);
  MALIDRIVE_THROW_UNLESS(length != std::nullopt);
  road_header.length = length.value();

  const auto junction = attribute_parser.As<std::string>(RoadHeader::kJunction);
  MALIDRIVE_THROW_UNLESS(junction != std::nullopt);
  road_header.junction = junction.value();
  // @}

  // Optional attributes.
  // @{
  road_header.name = attribute_parser.As<std::string>(RoadHeader::kName);
  road_header.rule = attribute_parser.As<RoadHeader::HandTrafficRule>(RoadHeader::kRule);
  // @}

  // Get RoadLink.
  // @{
  MALIDRIVE_TRACE("Parsing road link.");
  const auto link_element = element_->FirstChildElement(RoadLink::kRoadLinkTag);
  if (link_element) {
    const NodeParser road_link(link_element, parser_configuration_);
    road_header.road_link = road_link.As<RoadLink>();
  }
  // Get Types.
  MALIDRIVE_TRACE("Parsing road type.");
  auto type_element = element_->FirstChildElement(RoadType::kRoadTypeTag);
  while (type_element) {
    road_header.road_types.push_back(NodeParser(type_element, parser_configuration_).As<RoadType>());
    type_element = type_element->NextSiblingElement(RoadType::kRoadTypeTag);
  }

  // @}
  // Fill reference_geometry value.
  // @{
  // Get PlanView.
  MALIDRIVE_TRACE("Parsing planView.");
  tinyxml2::XMLElement* plan_view_element(element_->FirstChildElement(PlanView::kPlanViewTag));
  MALIDRIVE_THROW_UNLESS(plan_view_element != nullptr);
  road_header.reference_geometry.plan_view = NodeParser(plan_view_element, parser_configuration_).As<PlanView>();
  MALIDRIVE_TRACE("Parsing elevationProfile.");
  // Get ElevationProfile.
  tinyxml2::XMLElement* elevation_profile_element(element_->FirstChildElement(ElevationProfile::kElevationProfileTag));
  if (elevation_profile_element) {
    road_header.reference_geometry.elevation_profile =
        NodeParser(elevation_profile_element, parser_configuration_).As<ElevationProfile>();
  }
  // Get LateralProfile.
  MALIDRIVE_TRACE("Parsing lateralProfile.");
  tinyxml2::XMLElement* lateral_profile_element(element_->FirstChildElement(LateralProfile::kLateralProfileTag));
  if (lateral_profile_element) {
    road_header.reference_geometry.lateral_profile =
        NodeParser(lateral_profile_element, parser_configuration_).As<LateralProfile>();
  }
  // @}

  // Get Lanes.
  // @{
  MALIDRIVE_TRACE("Parsing lanes.");
  const NodeParser lanes(element_->FirstChildElement(Lanes::kLanesTag), parser_configuration_);
  road_header.lanes = Lanes{lanes.As<Lanes>()};
  // @}

  return road_header;
}

// Specialization to parse `Connection::LaneLink`'s node.
template <>
Connection::LaneLink NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto from = attribute_parser.As<std::string>(Connection::LaneLink::kFrom);
  MALIDRIVE_THROW_UNLESS(from != std::nullopt);

  const auto to = attribute_parser.As<std::string>(Connection::LaneLink::kTo);
  MALIDRIVE_THROW_UNLESS(to != std::nullopt);
  // @}
  return {Connection::LaneLink::Id(*from), Connection::LaneLink::Id(*to)};
}

// Specialization to parse `Connection`'s node.
template <>
Connection NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(Connection::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt);

  const auto incoming_road = attribute_parser.As<std::string>(Connection::kIncomingRoad);
  MALIDRIVE_THROW_UNLESS(incoming_road != std::nullopt);

  const auto connecting_road = attribute_parser.As<std::string>(Connection::kConnectingRoad);
  MALIDRIVE_THROW_UNLESS(connecting_road != std::nullopt);

  const auto contact_point = attribute_parser.As<Connection::ContactPoint>(Connection::kContactPoint);
  MALIDRIVE_THROW_UNLESS(contact_point != std::nullopt);
  // @}

  // Optional attributes.
  // @{
  const auto master_id = attribute_parser.As<std::string>(Connection::kConnectionMaster);
  const std::optional<Connection::Id> connection_master =
      master_id.has_value() ? std::make_optional<>(Connection::Id(*master_id)) : std::nullopt;

  const auto type = attribute_parser.As<Connection::Type>(Connection::kType);
  // TODO(#477): Support virtual connections.
  if (type.has_value() && type.value() != Connection::Type::kDefault) {
    MALIDRIVE_THROW_MESSAGE(std::string("Only default connection type is supported: Error at Connection Id: ") + *id);
  }
  // @}

  tinyxml2::XMLElement* lane_link_element(element_->FirstChildElement(Connection::LaneLink::kLaneLinkTag));
  std::vector<Connection::LaneLink> lane_links;
  if (lane_link_element != nullptr) {
    while (lane_link_element) {
      const auto lane_link = NodeParser(lane_link_element, parser_configuration_).As<Connection::LaneLink>();
      lane_links.push_back(std::move(lane_link));
      lane_link_element = lane_link_element->NextSiblingElement(Connection::LaneLink::kLaneLinkTag);
    }
  }
  return {Connection::Id(*id), *incoming_road, *connecting_road, *contact_point, connection_master, type, lane_links};
}

// Specialization to parse `Junction`'s node.
template <>
Junction NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(Junction::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt);
  // @}

  // Optional attributes.
  // @{
  const auto name = attribute_parser.As<std::string>(Junction::kName);

  const auto type = attribute_parser.As<Junction::Type>(Junction::kType);
  // TODO(#477): Support virtual junctions.
  if (type.has_value() && (type.value() != Junction::Type::kDefault)) {
    MALIDRIVE_THROW_MESSAGE(std::string("Only default junction type is supported: Error at Junction Id: ") + *id);
  }
  // @}

  tinyxml2::XMLElement* connection_element(element_->FirstChildElement(Connection::kConnectionTag));
  if (!connection_element) {
    std::string msg{"Junction (" + id.value() + ") has no connections:\n" + ConvertXMLNodeToText(element_)};
    if (!parser_configuration_.allow_schema_errors) {
      MALIDRIVE_THROW_MESSAGE(msg);
    }
    DuplicateCurlyBracesForFmtLogging(&msg);
    maliput::log()->warn(msg);
  }
  std::unordered_map<Connection::Id, Connection> connections;
  while (connection_element) {
    const auto connection = NodeParser(connection_element, parser_configuration_).As<Connection>();
    if (connections.find(connection.id) != connections.end()) {
      MALIDRIVE_THROW_MESSAGE(std::string("Connection Id: ") + connection.id.string() + std::string(" is duplicated."));
    }
    connections.insert({connection.id, std::move(connection)});
    connection_element = connection_element->NextSiblingElement(Connection::kConnectionTag);
  }
  return {Junction::Id(*id), name, type, connections};
}

std::string ConvertXMLNodeToText(tinyxml2::XMLElement* element) {
  MALIDRIVE_THROW_UNLESS(element != nullptr);
  tinyxml2::XMLPrinter printer;
  element->Accept(&printer);
  return printer.CStr();
}

void DuplicateCurlyBracesForFmtLogging(std::string* text) {
  MALIDRIVE_THROW_UNLESS(text != nullptr);
  // Duplicates `key` if found in `text`.
  auto duplicates_key = [](const std::string& key, std::string* text) {
    std::size_t last_pos = text->find(key);
    while (last_pos != text->npos) {
      text->insert(last_pos, key);
      last_pos = text->find(key, last_pos + 2);
    }
  };
  duplicates_key("{", text);
  duplicates_key("}", text);
}

}  // namespace xodr
}  // namespace malidrive
