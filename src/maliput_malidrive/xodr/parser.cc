// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/xodr/parser.h"

#include <map>
#include <vector>

#include <maliput/common/logger.h>

#include "maliput_malidrive/xodr/colors.h"
#include "maliput_malidrive/xodr/connection.h"
#include "maliput_malidrive/xodr/elevation_profile.h"
#include "maliput_malidrive/xodr/geo_reference.h"
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
#include "maliput_malidrive/xodr/object/object.h"
#include "maliput_malidrive/xodr/offset.h"
#include "maliput_malidrive/xodr/plan_view.h"
#include "maliput_malidrive/xodr/road_header.h"
#include "maliput_malidrive/xodr/road_link.h"
#include "maliput_malidrive/xodr/road_type.h"
#include "maliput_malidrive/xodr/unit.h"

namespace malidrive {
namespace xodr {
namespace {

// {@ Constants used during parsing process.
static constexpr bool kDontAllowNan{false};
// }@

// Validates that `value` contains a valid double-type value.
//
// @param value double value
// @param allow_nan Determines whether to accept having NaN values or not.
// @returns The validated value.
//
// @throws maliput::common::road_network_description_parser_error When `value` is std::nullopt.
// @throws maliput::common::road_network_description_parser_error When `value.value()` is a NaN value iff `allow_nan` is
// false.
double ValidateDouble(const std::optional<double>& value, bool allow_nan) {
  MALIDRIVE_THROW_UNLESS(value != std::nullopt, maliput::common::road_network_description_parser_error);
  if (!allow_nan) {
    MALIDRIVE_THROW_UNLESS(!std::isnan(value.value()), maliput::common::road_network_description_parser_error);
  }
  return value.value();
}

// Determines whether `geometry_a` and `geometry_b` geometries are continguos in terms of the arc length parameter.
bool IsContiguous(const Geometry& geometry_a, const Geometry& geometry_b, double tolerance) {
  MALIDRIVE_THROW_UNLESS(tolerance >= 0, maliput::common::road_network_description_parser_error);
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
//  - When `allow_schema_errors` is true and only `new_function.s_0` value is equal to `functions->back().s_0` value
//  then latter description is discarded and replaced by `new_function`.
//  - When `allow_schema_errors` is false and only `new_function.s_0` value is equal to `functions->back().s_0` value
//  it throws.
//
// @param new_function New function to be added.
// @param node_id Name of the xml node under analysis, used for proper logging messages.
// @param allow_schema_errors Indicates the permittivity.
// @param xml_node XML node that is used to improve logging messages.
// @param functions Collection of functions.
//
// @throws maliput::common::road_network_description_parser_error When `allow_schema_errors` is false and only
// `new_function.s_0` value is equal to `functions->back().s_0` value.
template <typename T>
void AddPolynomialDescriptionToCollection(const T& new_function, const std::string& node_id, bool allow_schema_errors,
                                          tinyxml2::XMLElement* xml_node, std::vector<T>* functions) {
  if (!functions->empty()) {
    if (new_function == functions->back()) {
      std::string msg{node_id + " node describes two identical functions:\n" + ConvertXMLNodeToText(xml_node) +
                      "Discarding the first repeated description."};
      maliput::log()->trace(msg);
      functions->pop_back();
    } else if (new_function.s_0 == functions->back().s_0) {
      // Comparing double values is controversial. However, the values here share the same origin:
      // They come from the tinyxml2's parser so it is expected that if they are the same in the xodr file description
      // then they are the same after being parsed.
      // (even though not necessarily the values in the XML and the values after parsing are the same).
      std::string msg{node_id + " node describes two functions starting at the same s:\n" +
                      ConvertXMLNodeToText(xml_node)};
      if (!allow_schema_errors) {
        MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
      }
      maliput::log()->debug(msg + "Discarding the first description starting at s = ", new_function.s_0);
      functions->pop_back();
    }
  }
  functions->push_back(std::move(new_function));
}

// Determines whether a collection of `T` functions has valid coefficients. "NaN" values are invalid.
//
// The type `T` should define the following members:
//     - {a, b, c, d} : coefficients of a cubic polynomial
//                      such @f$ a + b * (s - s_0) + c * (s - s_0)^2 + d * (s - s_0)^3 @f$.
// Tipically used with ElevationProfile::Elevation, LateralProfile::Superelevation, LaneOffset and LaneWidth.
//
// @param functions Collection of functions.
// @returns True when the functions' coeffcients don't contain "NaN" values.
template <typename T>
bool AreFunctionsCoeffValid(const std::vector<T>& functions) {
  for (const auto& function : functions) {
    if (std::isnan(function.a) || std::isnan(function.b) || std::isnan(function.c) || std::isnan(function.d)) {
      return false;
    }
  }
  return true;
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
  if (std::isnan(value)) {
    std::string msg{"Attributes with NaN values has been found. " + ConvertXMLNodeToText(element_)};
    maliput::log()->debug(msg);
    if (!parser_configuration_.allow_schema_errors) {
      MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
    }
  }
  return std::make_optional(value);
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
  const std::string kTrueYes = "yes";
  const std::string kTrueNum = "1";
  const std::string kFalseStr = "false";
  const std::string kFalseNo = "no";
  const std::string kFalseNum = "0";
  const char* attribute_as_str_ptr = element_->Attribute(attribute_name.c_str());
  if (attribute_as_str_ptr == nullptr) return std::nullopt;
  const std::string attribute_value = static_cast<std::string>(attribute_as_str_ptr);
  MALIDRIVE_THROW_UNLESS(attribute_value == kTrueStr || attribute_value == kTrueNum || attribute_value == kFalseStr ||
                             attribute_value == kFalseNum || attribute_value == kFalseNo || attribute_value == kTrueYes,
                         maliput::common::road_network_description_parser_error);
  return attribute_value == kTrueStr || attribute_value == kTrueNum || attribute_value == kTrueYes;
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

// Specialization to parse as `Lane::Advisory` the attribute's value.
template <>
std::optional<Lane::Advisory> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> advisory = As<std::string>(attribute_name);
  if (advisory.has_value()) {
    return Lane::str_to_advisory(advisory.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `Lane::Direction` the attribute's value.
template <>
std::optional<Lane::Direction> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> direction = As<std::string>(attribute_name);
  if (direction.has_value()) {
    return Lane::str_to_direction(direction.value());
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

// Specialization to parse as `Color` the attribute's value.
template <>
std::optional<Color> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> color = As<std::string>(attribute_name);
  return color.has_value() ? std::make_optional<Color>(str_to_color(color.value())) : std::nullopt;
}

// Specialization to parse as `LaneRoadMark::Type` the attribute's value.
template <>
std::optional<LaneRoadMark::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> lane_road_mark_type = As<std::string>(attribute_name);
  return lane_road_mark_type.has_value()
             ? std::make_optional<LaneRoadMark::Type>(LaneRoadMark::str_to_type(lane_road_mark_type.value()))
             : std::nullopt;
}

// Specialization to parse as `LaneRoadMark::LaneChange` the attribute's value.
template <>
std::optional<LaneRoadMark::LaneChange> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> lane_road_mark_lane_change = As<std::string>(attribute_name);
  return lane_road_mark_lane_change.has_value()
             ? std::make_optional<LaneRoadMark::LaneChange>(
                   LaneRoadMark::str_to_lane_change(lane_road_mark_lane_change.value()))
             : std::nullopt;
}

// Specialization to parse as `LaneRoadMark::Weight` the attribute's value.
template <>
std::optional<LaneRoadMark::Weight> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> lane_road_mark_weight = As<std::string>(attribute_name);
  return lane_road_mark_weight.has_value()
             ? std::make_optional<LaneRoadMark::Weight>(LaneRoadMark::str_to_weight(lane_road_mark_weight.value()))
             : std::nullopt;
}

// Specialization to parse as `Rule` the attribute's value.
template <>
std::optional<Rule> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> rule = As<std::string>(attribute_name);
  return rule.has_value() ? std::make_optional<Rule>(str_to_rule(rule.value())) : std::nullopt;
}

// Specialization to parse `GeoReference`'s node.
template <>
GeoReference NodeParser::As() const {
  GeoReference georef{};
  const AttributeParser attribute_parser(element_, parser_configuration_);
  MALIDRIVE_TRACE("Parsing geoReference.");
  const char* projection_data = element_->GetText();
  MALIDRIVE_THROW_UNLESS(projection_data != nullptr, maliput::common::road_network_description_parser_error);
  georef.projection_data = projection_data;
  return georef;
}

// Specialization to parse `Offset`'s node.
template <>
Offset NodeParser::As() const {
  Offset offset{};
  const AttributeParser attribute_parser(element_, parser_configuration_);
  MALIDRIVE_TRACE("Parsing offset.");

  // Non-optional attributes.
  // @{
  const auto x = attribute_parser.As<double>(Offset::kX);
  offset.x = ValidateDouble(x, kDontAllowNan);
  const auto y = attribute_parser.As<double>(Offset::kY);
  offset.y = ValidateDouble(y, kDontAllowNan);
  const auto z = attribute_parser.As<double>(Offset::kZ);
  offset.z = ValidateDouble(z, kDontAllowNan);
  const auto hdg = attribute_parser.As<double>(Offset::kHeading);
  offset.hdg = ValidateDouble(hdg, kDontAllowNan);
  // @}

  return offset;
}

// Specialization to parse `Header`'s node.
template <>
Header NodeParser::As() const {
  Header header{};
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto rev_major = attribute_parser.As<double>(Header::kXodrRevMajor);
  header.rev_major = ValidateDouble(rev_major, kDontAllowNan);

  const auto rev_minor = attribute_parser.As<double>(Header::kXodrRevMinor);
  header.rev_minor = ValidateDouble(rev_minor, kDontAllowNan);
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

  tinyxml2::XMLElement* geo_reference_element = element_->FirstChildElement(GeoReference::kGeoReferenceTag);
  if (geo_reference_element != nullptr) {
    header.geo_reference = NodeParser(geo_reference_element, parser_configuration_).As<GeoReference>();
  }
  tinyxml2::XMLElement* offset_element = element_->FirstChildElement(Offset::kOffsetTag);
  if (offset_element != nullptr) {
    header.offset = NodeParser(offset_element, parser_configuration_).As<Offset>();
  }
  return header;
}

// Specialization to parse `RoadLink::LinkAttributes`'s node.
template <>
RoadLink::LinkAttributes NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const auto element_type = attribute_parser.As<RoadLink::ElementType>(RoadLink::LinkAttributes::kElementType);
  MALIDRIVE_THROW_UNLESS(element_type != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto element_id = attribute_parser.As<std::string>(RoadLink::LinkAttributes::kElementId);
  MALIDRIVE_THROW_UNLESS(element_id != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto contact_point = attribute_parser.As<RoadLink::ContactPoint>(RoadLink::LinkAttributes::kContactPoint);
  switch (*element_type) {
    case RoadLink::ElementType::kRoad:
      if (contact_point == std::nullopt) {
        const std::string msg = "RoadLink to Road demands contactPoint attribute.";
        maliput::log()->debug(msg);
        MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
      }
      break;
    case RoadLink::ElementType::kJunction:
      if (contact_point != std::nullopt) {
        const std::string msg = "RoadLink to Junction does not support contactPoint attribute.";
        maliput::log()->debug(msg);
        if (!parser_configuration_.allow_schema_errors) {
          MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
        }
      }
      break;
    default:
      MALIDRIVE_THROW_MESSAGE("Invalid elementType value for RoadLink's description.",
                              maliput::common::road_network_description_parser_error);
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
    MALIDRIVE_THROW_MESSAGE(
        std::string("Bad Line description. Line node doesn't allow attributes: ") + ConvertXMLNodeToText(element_),
        maliput::common::road_network_description_parser_error);
  }
  return Geometry::Line{};
}

// Specialization to parse `Arc`'s node.
template <>
Geometry::Arc NodeParser::As() const {
  if (NumberOfAttributes() != 1) {
    MALIDRIVE_THROW_MESSAGE(std::string("Bad Arc description. Arc demands only one argument: 'curvature'. ") +
                                ConvertXMLNodeToText(element_),
                            maliput::common::road_network_description_parser_error);
  }
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const auto curvature = attribute_parser.As<double>(Geometry::Arc::kCurvature);
  return Geometry::Arc{ValidateDouble(curvature, kDontAllowNan)};
}

// Specialization to parse `Spiral`'s node.
template <>
Geometry::Spiral NodeParser::As() const {
  if (NumberOfAttributes() != 2) {
    MALIDRIVE_THROW_MESSAGE(std::string("Bad Spiral description. Spiral demands only two arguments: 'curvStart' and "
                                        "'curvEnd'. ") +
                                ConvertXMLNodeToText(element_),
                            maliput::common::road_network_description_parser_error);
  }
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const auto curv_start = attribute_parser.As<double>(Geometry::Spiral::kCurvStart);
  const auto curv_end = attribute_parser.As<double>(Geometry::Spiral::kCurvEnd);
  return Geometry::Spiral{ValidateDouble(curv_start, kDontAllowNan), ValidateDouble(curv_end, kDontAllowNan)};
}

// Specialization to parse `ParamPoly3`'s node.
template <>
Geometry::ParamPoly3 NodeParser::As() const {
  if (NumberOfAttributes() != 9) {
    MALIDRIVE_THROW_MESSAGE(
        std::string("Bad ParamPoly3 description. ParamPoly3 demands nine arguments: 'aU', 'bU', 'cU', 'dU', 'aV', "
                    "'bV', 'cV', 'dV', 'pRange'. ") +
            ConvertXMLNodeToText(element_),
        maliput::common::road_network_description_parser_error);
  }
  const AttributeParser attribute_parser(element_, parser_configuration_);

  Geometry::ParamPoly3 param_poly3;
  param_poly3.aU = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kAU), kDontAllowNan);
  param_poly3.bU = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kBU), kDontAllowNan);
  param_poly3.cU = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kCU), kDontAllowNan);
  param_poly3.dU = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kDU), kDontAllowNan);
  param_poly3.aV = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kAV), kDontAllowNan);
  param_poly3.bV = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kBV), kDontAllowNan);
  param_poly3.cV = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kCV), kDontAllowNan);
  param_poly3.dV = ValidateDouble(attribute_parser.As<double>(Geometry::ParamPoly3::kDV), kDontAllowNan);

  const auto p_range_opt = attribute_parser.As<std::string>(Geometry::ParamPoly3::kPRange);
  if (!p_range_opt.has_value()) {
    MALIDRIVE_THROW_MESSAGE(std::string("Bad ParamPoly3 description. Missing required attribute 'pRange'. ") +
                                ConvertXMLNodeToText(element_),
                            maliput::common::road_network_description_parser_error);
  }
  const std::string p_range_str = p_range_opt.value();
  if (p_range_str == "arcLength") {
    param_poly3.p_range = Geometry::ParamPoly3::PRange::kArcLength;
  } else if (p_range_str == "normalized") {
    param_poly3.p_range = Geometry::ParamPoly3::PRange::kNormalized;
  } else {
    MALIDRIVE_THROW_MESSAGE(std::string("Bad ParamPoly3 description. Invalid pRange value: '") + p_range_str +
                                "'. Expected 'arcLength' or 'normalized'. " + ConvertXMLNodeToText(element_),
                            maliput::common::road_network_description_parser_error);
  }

  return param_poly3;
}

// Specialization to parse `LaneWidth`'s node.
template <>
LaneWidth NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const bool allow_nan = parser_configuration_.allow_schema_errors;

  // Non-optional attributes.
  // @{
  const double offset = ValidateDouble(attribute_parser.As<double>(LaneWidth::kOffset), kDontAllowNan);
  const double a_param = ValidateDouble(attribute_parser.As<double>(LaneWidth::kA), allow_nan);
  const double b_param = ValidateDouble(attribute_parser.As<double>(LaneWidth::kB), allow_nan);
  const double c_param = ValidateDouble(attribute_parser.As<double>(LaneWidth::kC), allow_nan);
  const double d_param = ValidateDouble(attribute_parser.As<double>(LaneWidth::kD), allow_nan);
  // @}
  return {offset, a_param, b_param, c_param, d_param};
}

// Specialization to parse `LaneOffset`'s node.
template <>
LaneOffset NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const bool allow_nan = parser_configuration_.allow_schema_errors;

  // Non-optional attributes.
  // @{
  const double s_0 = ValidateDouble(attribute_parser.As<double>(LaneOffset::kS0), kDontAllowNan);
  const double a_param = ValidateDouble(attribute_parser.As<double>(LaneOffset::kA), allow_nan);
  const double b_param = ValidateDouble(attribute_parser.As<double>(LaneWidth::kB), allow_nan);
  const double c_param = ValidateDouble(attribute_parser.As<double>(LaneWidth::kC), allow_nan);
  const double d_param = ValidateDouble(attribute_parser.As<double>(LaneWidth::kD), allow_nan);
  // @}
  return {s_0, a_param, b_param, c_param, d_param};
}

// Specialization to parse `LaneLink::LinkAttributes`'s node.
template <>
LaneLink::LinkAttributes NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const auto id = attribute_parser.As<std::string>(LaneLink::LinkAttributes::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
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
  MALIDRIVE_THROW_UNLESS(max != std::nullopt, maliput::common::road_network_description_parser_error);
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
  road_type.s_0 = ValidateDouble(s_0, kDontAllowNan);

  const auto type = attribute_parser.As<RoadType::Type>(RoadType::kRoadTypeTag);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt, maliput::common::road_network_description_parser_error);
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
  speed.s_offset = ValidateDouble(s_offset, kDontAllowNan);

  const auto max = attribute_parser.As<double>(Lane::Speed::kMax);
  speed.max = ValidateDouble(max, kDontAllowNan);

  const auto unit = attribute_parser.As<Unit>(Lane::Speed::kUnit);
  speed.unit = unit.has_value() ? unit.value() : Unit::kMs;
  return speed;
}

// Specialization to parse `ExplicitElementLine`'s node.
template <>
SwayElement NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto a = attribute_parser.As<double>(SwayElement::kA);
  MALIDRIVE_THROW_UNLESS(a != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto b = attribute_parser.As<double>(SwayElement::kB);
  MALIDRIVE_THROW_UNLESS(b != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto c = attribute_parser.As<double>(SwayElement::kC);
  MALIDRIVE_THROW_UNLESS(c != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto d = attribute_parser.As<double>(SwayElement::kD);
  MALIDRIVE_THROW_UNLESS(d != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto ds = attribute_parser.As<double>(SwayElement::kDS);
  MALIDRIVE_THROW_UNLESS(ds != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  return {a.value(), b.value(), c.value(), d.value(), ds.value()};
}

// Specialization to parse `ExplicitElementLine`'s node.
template <>
ExplicitElementLine NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto length = attribute_parser.As<double>(ExplicitElementLine::kLength);
  MALIDRIVE_THROW_UNLESS(length != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s_offset = attribute_parser.As<double>(ExplicitElementLine::kSOffset);
  MALIDRIVE_THROW_UNLESS(s_offset != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto t_offset = attribute_parser.As<double>(ExplicitElementLine::kTOffset);
  MALIDRIVE_THROW_UNLESS(t_offset != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto rule = attribute_parser.As<Rule>(ExplicitElementLine::kRule);
  const auto width = attribute_parser.As<double>(ExplicitElementLine::kWidth);
  // @}

  return {length.value(), rule, s_offset.value(), t_offset.value(), width};
}

// Specialization to parse `ExplicitElement`'s node.
template <>
ExplicitElement NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Line elements
  tinyxml2::XMLElement* explicit_element_line_xml =
      element_->FirstChildElement(ExplicitElementLine::kExplicitElementLineTag);
  std::vector<ExplicitElementLine> explicit_element_lines;
  while (explicit_element_line_xml) {
    auto explicit_element_line = NodeParser(explicit_element_line_xml, parser_configuration_).As<ExplicitElementLine>();
    explicit_element_lines.push_back(explicit_element_line);
    explicit_element_line_xml =
        explicit_element_line_xml->NextSiblingElement(ExplicitElementLine::kExplicitElementLineTag);
  }

  return {explicit_element_lines};
}

// Specialization to parse `TypeElementLine`'s node.
template <>
TypeElementLine NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto length = attribute_parser.As<double>(TypeElementLine::kLength);
  MALIDRIVE_THROW_UNLESS(length != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s_offset = attribute_parser.As<double>(TypeElementLine::kSOffset);
  MALIDRIVE_THROW_UNLESS(s_offset != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto space = attribute_parser.As<double>(TypeElementLine::kSpace);
  MALIDRIVE_THROW_UNLESS(space != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto t_offset = attribute_parser.As<double>(TypeElementLine::kTOffset);
  MALIDRIVE_THROW_UNLESS(t_offset != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto color = attribute_parser.As<Color>(TypeElementLine::kColor);
  const auto rule = attribute_parser.As<Rule>(TypeElementLine::kRule);
  const auto width = attribute_parser.As<double>(TypeElementLine::kWidth);
  // @}

  return {color, length.value(), rule, s_offset.value(), space.value(), t_offset.value(), width};
}

// Specialization to parse `TypeElement`'s node.
template <>
TypeElement NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto name = attribute_parser.As<std::string>(TypeElement::kName);
  MALIDRIVE_THROW_UNLESS(name != std::nullopt, maliput::common::road_network_description_parser_error);
  auto width = attribute_parser.As<double>(TypeElement::kWidth);
  if (parser_configuration_.allow_schema_errors && width == std::nullopt) {
    width = 0.1;
  } else if (width == std::nullopt) {
    MALIDRIVE_THROW_UNLESS(width != std::nullopt, maliput::common::road_network_description_parser_error);
  }
  // @}

  // Line elements
  tinyxml2::XMLElement* type_element_line_xml = element_->FirstChildElement(TypeElementLine::kTypeLineTag);
  std::vector<TypeElementLine> type_element_lines;
  while (type_element_line_xml) {
    auto type_element_line = NodeParser(type_element_line_xml, parser_configuration_).As<TypeElementLine>();
    type_element_lines.push_back(type_element_line);
    type_element_line_xml = type_element_line_xml->NextSiblingElement(TypeElementLine::kTypeLineTag);
  }

  return {name.value(), width.value(), type_element_lines};
}

// Specialization to parse `LaneRoadkMark`'s node.
template <>
LaneRoadMark NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  auto color = attribute_parser.As<Color>(LaneRoadMark::kLaneRoadMarkColor);
  if (parser_configuration_.allow_schema_errors && color == std::nullopt) {
    color = Color::kWhite;
  } else if (color == std::nullopt) {
    MALIDRIVE_THROW_UNLESS(color != std::nullopt, maliput::common::road_network_description_parser_error);
  }
  const auto offset = attribute_parser.As<double>(LaneRoadMark::kOffset);
  MALIDRIVE_THROW_UNLESS(offset != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto type = attribute_parser.As<LaneRoadMark::Type>(LaneRoadMark::kType);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto height = attribute_parser.As<double>(LaneRoadMark::kLaneRoadMarkHeight);
  const auto lane_change = attribute_parser.As<LaneRoadMark::LaneChange>(LaneRoadMark::kLaneRoadMarkLaneChange);
  const auto material = attribute_parser.As<std::string>(LaneRoadMark::kLaneRoadMarkMaterial);
  const auto weight = attribute_parser.As<LaneRoadMark::Weight>(LaneRoadMark::kWeight);
  const auto width = attribute_parser.As<double>(LaneRoadMark::kWidth);
  // @}

  // Elements
  // Type
  tinyxml2::XMLElement* road_mark_type_element_xml = element_->FirstChildElement(TypeElement::kTypeElementTag);
  std::vector<TypeElement> road_mark_type_elements;
  while (road_mark_type_element_xml) {
    auto road_mark_type_element = NodeParser(road_mark_type_element_xml, parser_configuration_).As<TypeElement>();
    road_mark_type_elements.push_back(road_mark_type_element);
    road_mark_type_element_xml = road_mark_type_element_xml->NextSiblingElement(TypeElement::kTypeElementTag);
  }

  // Explicit
  tinyxml2::XMLElement* road_mark_explicit_element_xml =
      element_->FirstChildElement(ExplicitElement::kExplicitElementTag);
  std::vector<ExplicitElement> road_mark_explicit_elements;
  while (road_mark_explicit_element_xml) {
    auto road_mark_explicit_element =
        NodeParser(road_mark_explicit_element_xml, parser_configuration_).As<ExplicitElement>();
    road_mark_explicit_elements.push_back(road_mark_explicit_element);
    road_mark_explicit_element_xml =
        road_mark_explicit_element_xml->NextSiblingElement(ExplicitElement::kExplicitElementTag);
  }

  // Sway
  tinyxml2::XMLElement* sway_element_xml = element_->FirstChildElement(SwayElement::kSwayTag);
  std::vector<SwayElement> sway_elements;
  while (sway_element_xml) {
    auto sway_element = NodeParser(sway_element_xml, parser_configuration_).As<SwayElement>();

    AddPolynomialDescriptionToCollection(std::move(sway_element), Lane::kLaneTag,
                                         parser_configuration_.allow_schema_errors, element_, &sway_elements);
    sway_element_xml = sway_element_xml->NextSiblingElement(SwayElement::kSwayTag);
  }

  return {color.value(),
          height,
          lane_change,
          material,
          offset.value(),
          type.value(),
          weight,
          width,
          road_mark_type_elements,
          road_mark_explicit_elements,
          sway_elements};
}

// Specialization to parse `Lane`'s node.
template <>
Lane NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(Lane::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto type = attribute_parser.As<Lane::Type>(Lane::kType);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto level = attribute_parser.As<bool>(Lane::kLevel);
  const auto advisory = attribute_parser.As<Lane::Advisory>(Lane::kAdvisory);
  const auto direction = attribute_parser.As<Lane::Direction>(Lane::kDirection);
  const auto dynamic_lane_direction = attribute_parser.As<bool>(Lane::kDynamicLaneDirection);
  const auto dynamic_lane_type = attribute_parser.As<bool>(Lane::kDynamicLaneType);
  const auto road_works = attribute_parser.As<bool>(Lane::kRoadWorks);
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
                                         parser_configuration_.allow_schema_errors, element_, &width_description);
    width_element = width_element->NextSiblingElement(LaneWidth::kLaneWidthTag);
  }

  // roadMark
  tinyxml2::XMLElement* road_marks_element_xml = element_->FirstChildElement(LaneRoadMark::kLaneRoadMarkTag);
  std::vector<LaneRoadMark> road_marks;
  while (road_marks_element_xml) {
    auto road_mark = NodeParser(road_marks_element_xml, parser_configuration_).As<LaneRoadMark>();
    road_marks.push_back(road_mark);
    road_marks_element_xml = road_marks_element_xml->NextSiblingElement(LaneRoadMark::kLaneRoadMarkTag);
  }

  // Only when schema errors are allowed is possible to find NaN values in the functions, otherwise
  // the NaN values would have caused an error when parsing the double value in the XODR file.
  // (See `AttributeParser` specialization for double types).
  // To avid the calling `AreFunctionsCoeffValid` method when the conditions were already verified is
  // that the `parser_configuration_.allow_schema_errors` flag is first checked.
  if (parser_configuration_.allow_schema_errors && !AreFunctionsCoeffValid(width_description)) {
    std::string msg{std::string(Lane::kLaneTag) + " node has a width description with NaN values:\n" +
                    ConvertXMLNodeToText(element_)};
    MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
  }

  tinyxml2::XMLElement* speed_element = element_->FirstChildElement(Lane::Speed::kSpeedTag);
  std::vector<Lane::Speed> speeds;
  while (speed_element) {
    speeds.push_back(NodeParser(speed_element, parser_configuration_).As<Lane::Speed>());
    speed_element = speed_element->NextSiblingElement(Lane::Speed::kSpeedTag);
  }

  std::optional<std::string> user_data{std::nullopt};
  tinyxml2::XMLElement* user_data_element = nullptr;
  if (parser_configuration_.use_userdata_traffic_direction) {
    user_data_element = element_->FirstChildElement(Lane::kUserData);
  }
  if (user_data_element != nullptr) {
    user_data = ConvertXMLNodeToText(user_data_element);
  }
  return {Lane::Id(id.value()),
          type.value(),
          level,
          lane_link,
          width_description,
          road_marks,
          speeds,
          user_data,
          advisory,
          direction,
          dynamic_lane_direction,
          dynamic_lane_type,
          road_works};
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
    if (is_center_node) {
      // Center lanes must not have `widths` description.
      // While right and left lanes need at least one width entry.
      // Center lanes must not have `speed` records.
      if (lane.width_description.size() > 0 || !lane.speed.empty()) {
        // Invalid center lane.
        const std::string msg{std::string(Lane::kLaneTag) +
                              " node describes a center lane with widths or speeds description:\n" +
                              ConvertXMLNodeToText(lane_element_ptr)};
        if (!parse_configuration.allow_schema_errors) {
          MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
        }
        maliput::log()->warn(msg + "\nDiscarding the center lane's width and speeds descriptions.");
      }
    } else {
      // Right and left lanes must have at least one width entry.
      MALIDRIVE_THROW_UNLESS(lane.width_description.size() > 0, maliput::common::road_network_description_parser_error);
    }

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
  const double s_0 = ValidateDouble(attribute_parser.As<double>(LaneSection::kS0), kDontAllowNan);
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
  MALIDRIVE_THROW_UNLESS(center_element_ptr != nullptr, maliput::common::road_network_description_parser_error);
  std::vector<Lane> center_lanes = GetAllLanesFromNode(center_element_ptr, true, parser_configuration_);
  MALIDRIVE_THROW_UNLESS(center_lanes.size() == 1, maliput::common::road_network_description_parser_error);
  // Right lanes.
  MALIDRIVE_TRACE("Parsing right lanes.");
  std::vector<Lane> right_lanes;
  tinyxml2::XMLElement* right_element_ptr = element_->FirstChildElement(LaneSection::kRight);
  if (right_element_ptr != nullptr) {
    right_lanes = GetAllLanesFromNode(right_element_ptr, false, parser_configuration_);
  }

  return {s_0, single_side, left_lanes, center_lanes[0], right_lanes};
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
                                         parser_configuration_.allow_schema_errors, element_, &lanes_offsets);
    lane_offset_element_ptr = lane_offset_element_ptr->NextSiblingElement(LaneOffset::kLaneOffsetTag);
  }
  // Only when schema errors are allowed is possible to find NaN values in the functions, otherwise
  // the NaN values would have caused an error when parsing the double value in the XODR file.
  // (See `AttributeParser` specialization for double types).
  // To avid the calling `AreFunctionsCoeffValid` method when the conditions were already verified is
  // that the `parser_configuration_.allow_schema_errors` flag is first checked.
  if (parser_configuration_.allow_schema_errors && !AreFunctionsCoeffValid(lanes_offsets)) {
    std::string msg{std::string(LaneOffset::kLaneOffsetTag) +
                    " node describes a lane offset description with NaN values:\n" + ConvertXMLNodeToText(element_)};
    MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
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
  MALIDRIVE_THROW_UNLESS(lanes_section.size() > 0, maliput::common::road_network_description_parser_error);

  return {lanes_offsets, lanes_section};
}

// Specialization to parse `Geometry`'s node.
template <>
Geometry NodeParser::As() const {
  Geometry geometry{};
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  geometry.s_0 = ValidateDouble(attribute_parser.As<double>(Geometry::kS0), kDontAllowNan);
  geometry.start_point.x() = ValidateDouble(attribute_parser.As<double>(Geometry::kStartPointX), kDontAllowNan);
  geometry.start_point.y() = ValidateDouble(attribute_parser.As<double>(Geometry::kStartPointY), kDontAllowNan);
  geometry.orientation = ValidateDouble(attribute_parser.As<double>(Geometry::kOrientation), kDontAllowNan);
  geometry.length = ValidateDouble(attribute_parser.As<double>(Geometry::kLength), kDontAllowNan);

  const NodeParser geometry_type(element_->FirstChildElement(), parser_configuration_);
  geometry.type = Geometry::str_to_type(geometry_type.GetName());
  switch (geometry.type) {
    case Geometry::Type::kLine:
      geometry.description = geometry_type.As<Geometry::Line>();
      break;
    case Geometry::Type::kArc:
      geometry.description = geometry_type.As<Geometry::Arc>();
      break;
    case Geometry::Type::kSpiral:
      geometry.description = geometry_type.As<Geometry::Spiral>();
      break;
    case Geometry::Type::kParamPoly3:
      geometry.description = geometry_type.As<Geometry::ParamPoly3>();
      break;
    default:
      MALIDRIVE_THROW_MESSAGE(std::string("The Geometry type '") + Geometry::type_to_str(geometry.type) +
                                  std::string("' is not supported."),
                              maliput::common::road_network_description_parser_error);
  }
  // @}
  return geometry;
}

// Specialization to parse `elevation`'s node.
template <>
ElevationProfile::Elevation NodeParser::As() const {
  ElevationProfile::Elevation elevation{};
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const bool allow_nan = parser_configuration_.allow_schema_errors;
  // Non-optional attributes.
  // @{
  elevation.s_0 = ValidateDouble(attribute_parser.As<double>(ElevationProfile::Elevation::kS0), kDontAllowNan);
  elevation.a = ValidateDouble(attribute_parser.As<double>(ElevationProfile::Elevation::kA), allow_nan);
  elevation.b = ValidateDouble(attribute_parser.As<double>(ElevationProfile::Elevation::kB), allow_nan);
  elevation.c = ValidateDouble(attribute_parser.As<double>(ElevationProfile::Elevation::kC), allow_nan);
  elevation.d = ValidateDouble(attribute_parser.As<double>(ElevationProfile::Elevation::kD), allow_nan);
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
                                         parser_configuration_.allow_schema_errors, element_, &elevations);
    elevation_element = elevation_element->NextSiblingElement(ElevationProfile::Elevation::kElevationTag);
  }
  // Only when schema errors are allowed is possible to find NaN values in the functions, otherwise
  // the NaN values would have caused an error when parsing the double value in the XODR file.
  // (See `AttributeParser` specialization for double types).
  // To avid the calling `AreFunctionsCoeffValid` method when the conditions were already verified is
  // that the `parser_configuration_.allow_schema_errors` flag is first checked.
  if (parser_configuration_.allow_schema_errors && !AreFunctionsCoeffValid(elevations)) {
    std::string msg{std::string(ElevationProfile::kElevationProfileTag) +
                    " node describes a elevation description with NaN values:\n" + ConvertXMLNodeToText(element_)};
    MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
  }
  return {elevations};
}

// Specialization to parse `superelevation`'s node.
template <>
LateralProfile::Superelevation NodeParser::As() const {
  LateralProfile::Superelevation superelevation{};
  const AttributeParser attribute_parser(element_, parser_configuration_);
  const bool allow_nan = parser_configuration_.allow_schema_errors;

  // Non-optional attributes.
  // @{
  superelevation.s_0 = ValidateDouble(attribute_parser.As<double>(LateralProfile::Superelevation::kS0), kDontAllowNan);
  superelevation.a = ValidateDouble(attribute_parser.As<double>(LateralProfile::Superelevation::kA), allow_nan);
  superelevation.b = ValidateDouble(attribute_parser.As<double>(LateralProfile::Superelevation::kB), allow_nan);
  superelevation.c = ValidateDouble(attribute_parser.As<double>(LateralProfile::Superelevation::kC), allow_nan);
  superelevation.d = ValidateDouble(attribute_parser.As<double>(LateralProfile::Superelevation::kD), allow_nan);
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
                                         parser_configuration_.allow_schema_errors, element_, &superelevations);
    superelevation_element =
        superelevation_element->NextSiblingElement(LateralProfile::Superelevation::kSuperelevationTag);
  }
  // Only when schema errors are allowed is possible to find NaN values in the functions, otherwise
  // the NaN values would have caused an error when parsing the double value in the XODR file.
  // (See `AttributeParser` specialization for double types).
  // To avid the calling `AreFunctionsCoeffValid` method when the conditions were already verified is
  // that the `parser_configuration_.allow_schema_errors` flag is first checked.
  if (parser_configuration_.allow_schema_errors && !AreFunctionsCoeffValid(superelevations)) {
    std::string msg{std::string(LateralProfile::kLateralProfileTag) +
                    " node describes a superelevation description with NaN values:\n" + ConvertXMLNodeToText(element_)};
    MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
  }
  return {superelevations};
}

// Specialization to parse `PlanView`'s node.
template <>
PlanView NodeParser::As() const {
  std::vector<Geometry> geometries;
  tinyxml2::XMLElement* geometry_element(element_->FirstChildElement(Geometry::kGeometryTag));
  MALIDRIVE_THROW_UNLESS(geometry_element != nullptr, maliput::common::road_network_description_parser_error);
  while (geometry_element) {
    const NodeParser geometry_node(geometry_element, parser_configuration_);
    auto geometry = geometry_node.As<Geometry>();
    if (parser_configuration_.tolerance.has_value() && geometries.size() > 0) {
      if (!IsContiguous(geometries.back(), geometry, *parser_configuration_.tolerance)) {
        MALIDRIVE_THROW_MESSAGE("Geometries doesn't meet contiguity constraint.",
                                maliput::common::road_network_description_parser_error);
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
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
  road_header.id = RoadHeader::Id(id.value());

  MALIDRIVE_TRACE("Parsing road id: " + road_header.id.string());

  const auto length = attribute_parser.As<double>(RoadHeader::kLength);
  road_header.length = ValidateDouble(length, kDontAllowNan);

  const auto junction = attribute_parser.As<std::string>(RoadHeader::kJunction);
  MALIDRIVE_THROW_UNLESS(junction != std::nullopt, maliput::common::road_network_description_parser_error);
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
  MALIDRIVE_THROW_UNLESS(plan_view_element != nullptr, maliput::common::road_network_description_parser_error);
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

  // Get Objects.
  // @{
  MALIDRIVE_TRACE("Parsing Objects.");
  tinyxml2::XMLElement* objects_element(element_->FirstChildElement(object::Objects::kObjectsTag));
  if (objects_element) {
    road_header.objects = NodeParser(objects_element, parser_configuration_).As<object::Objects>();
  }
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
  MALIDRIVE_THROW_UNLESS(from != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto to = attribute_parser.As<std::string>(Connection::LaneLink::kTo);
  MALIDRIVE_THROW_UNLESS(to != std::nullopt, maliput::common::road_network_description_parser_error);
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
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto incoming_road = attribute_parser.As<std::string>(Connection::kIncomingRoad);
  MALIDRIVE_THROW_UNLESS(incoming_road != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto connecting_road = attribute_parser.As<std::string>(Connection::kConnectingRoad);
  MALIDRIVE_THROW_UNLESS(connecting_road != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto contact_point = attribute_parser.As<Connection::ContactPoint>(Connection::kContactPoint);
  MALIDRIVE_THROW_UNLESS(contact_point != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto master_id = attribute_parser.As<std::string>(Connection::kConnectionMaster);
  const std::optional<Connection::Id> connection_master =
      master_id.has_value() ? std::make_optional<>(Connection::Id(*master_id)) : std::nullopt;

  const auto type = attribute_parser.As<Connection::Type>(Connection::kType);
  // TODO(#477): Support virtual connections.
  if (type.has_value() && type.value() != Connection::Type::kDefault) {
    MALIDRIVE_THROW_MESSAGE(std::string("Only default connection type is supported: Error at Connection Id: ") + *id,
                            maliput::common::road_network_description_parser_error);
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
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto name = attribute_parser.As<std::string>(Junction::kName);

  const auto type = attribute_parser.As<Junction::Type>(Junction::kType);
  // TODO(#477): Support virtual junctions.
  if (type.has_value() && (type.value() != Junction::Type::kDefault)) {
    MALIDRIVE_THROW_MESSAGE(std::string("Only default junction type is supported: Error at Junction Id: ") + *id,
                            maliput::common::road_network_description_parser_error);
  }
  // @}

  tinyxml2::XMLElement* connection_element(element_->FirstChildElement(Connection::kConnectionTag));
  if (!connection_element) {
    std::string msg{"Junction (" + id.value() + ") has no connections:\n" + ConvertXMLNodeToText(element_)};
    if (!parser_configuration_.allow_schema_errors) {
      MALIDRIVE_THROW_MESSAGE(msg, maliput::common::road_network_description_parser_error);
    }
    maliput::log()->debug(msg);
  }
  std::map<Connection::Id, Connection> connections;
  while (connection_element) {
    const auto connection = NodeParser(connection_element, parser_configuration_).As<Connection>();
    if (connections.find(connection.id) != connections.end()) {
      MALIDRIVE_THROW_MESSAGE(std::string("Connection Id: ") + connection.id.string() + std::string(" is duplicated."),
                              maliput::common::road_network_description_parser_error);
    }
    connections.insert({connection.id, std::move(connection)});
    connection_element = connection_element->NextSiblingElement(Connection::kConnectionTag);
  }
  return {Junction::Id(*id), name, type, connections};
}

std::string ConvertXMLNodeToText(tinyxml2::XMLElement* element) {
  MALIDRIVE_THROW_UNLESS(element != nullptr, maliput::common::road_network_description_parser_error);
  tinyxml2::XMLPrinter printer;
  element->Accept(&printer);
  return printer.CStr();
}

}  // namespace xodr
}  // namespace malidrive
