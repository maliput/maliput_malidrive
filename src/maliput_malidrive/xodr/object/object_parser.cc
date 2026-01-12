// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
// Copyright (c) 2026, Toyota Research Institute. All rights reserved.
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
#include <map>
#include <vector>

#include <maliput/common/logger.h>

#include "maliput_malidrive/xodr/object/object.h"
#include "maliput_malidrive/xodr/parser.h"

namespace malidrive {
namespace xodr {

// Specialization to parse as `object::Orientation` the attribute's value.
template <>
std::optional<object::Orientation> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> orientation = As<std::string>(attribute_name);
  if (orientation.has_value()) {
    return object::str_to_orientation(orientation.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `object::Object::ObjectType` the attribute's value.
template <>
std::optional<object::Object::ObjectType> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  // Special check for some parsers
  if (type.has_value()) {
    if (parser_configuration_.allow_schema_errors) {
      if (type.value() == "-1") {
        return std::optional<object::Object::ObjectType>(object::Object::ObjectType::kNone);
      }
      if (type.value() == "") {
        return std::nullopt;
      }
    }
    return object::Object::str_to_object_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `object::Outline::FillType` the attribute's value.
template <>
std::optional<object::Outline::FillType> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  if (type.has_value()) {
    return object::Outline::str_to_fill_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `object::ParkingSpace::Access` the attribute's value.
template <>
std::optional<object::ParkingSpace::Access> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> access = As<std::string>(attribute_name);
  if (access.has_value()) {
    return object::ParkingSpace::str_to_access(access.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `object::Marking::Side` the attribute's value.
template <>
std::optional<object::Marking::Side> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> side = As<std::string>(attribute_name);
  if (side.has_value()) {
    return object::Marking::str_to_side(side.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `object::Border::Type` the attribute's value.
template <>
std::optional<object::Border::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  if (type.has_value()) {
    return object::Border::str_to_border_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `object::Tunnel::Type` the attribute's value.
template <>
std::optional<object::Tunnel::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  if (type.has_value()) {
    return object::Tunnel::str_to_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse as `object::Bridge::Type` the attribute's value.
template <>
std::optional<object::Bridge::Type> AttributeParser::As(const std::string& attribute_name) const {
  const std::optional<std::string> type = As<std::string>(attribute_name);
  if (type.has_value()) {
    return object::Bridge::str_to_type(type.value());
  } else {
    return std::nullopt;
  }
}

// Specialization to parse `object::CornerLocal`'s node.
template <>
object::CornerLocal NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto height = attribute_parser.As<double>(object::CornerLocal::kHeight);
  MALIDRIVE_THROW_UNLESS(height != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto u = attribute_parser.As<double>(object::CornerLocal::kU);
  MALIDRIVE_THROW_UNLESS(u != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto v = attribute_parser.As<double>(object::CornerLocal::kV);
  MALIDRIVE_THROW_UNLESS(v != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto z = attribute_parser.As<double>(object::CornerLocal::kZ);
  MALIDRIVE_THROW_UNLESS(z != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::CornerLocal::kId);
  // @}

  return {height.value(), object::CornerLocal::Id(id.value_or("none")), u.value(), v.value(), z.value()};
}

// Specialization to parse `object::CornerRoad`'s node.
template <>
object::CornerRoad NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto dz = attribute_parser.As<double>(object::CornerRoad::kDz);
  MALIDRIVE_THROW_UNLESS(dz != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto height = attribute_parser.As<double>(object::CornerRoad::kHeight);
  MALIDRIVE_THROW_UNLESS(height != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s = attribute_parser.As<double>(object::CornerRoad::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto t = attribute_parser.As<double>(object::CornerRoad::kT);
  MALIDRIVE_THROW_UNLESS(t != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::CornerRoad::kId);
  // @}

  return {dz.value(), height.value(), object::CornerRoad::Id(id.value_or("none")), s.value(), t.value()};
}

// Specialization to parse `object::Repeat`'s node.
template <>
object::Repeat NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto distance = attribute_parser.As<double>(object::Repeat::kDistance);
  MALIDRIVE_THROW_UNLESS(distance != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto height_end = attribute_parser.As<double>(object::Repeat::kHeightEnd);
  MALIDRIVE_THROW_UNLESS(height_end != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto height_start = attribute_parser.As<double>(object::Repeat::kHeightStart);
  MALIDRIVE_THROW_UNLESS(height_start != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto length = attribute_parser.As<double>(object::Repeat::kLength);
  MALIDRIVE_THROW_UNLESS(length != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s = attribute_parser.As<double>(object::Repeat::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto t_end = attribute_parser.As<double>(object::Repeat::kTEnd);
  MALIDRIVE_THROW_UNLESS(t_end != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto t_start = attribute_parser.As<double>(object::Repeat::kTStart);
  MALIDRIVE_THROW_UNLESS(t_start != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto z_offset_end = attribute_parser.As<double>(object::Repeat::kZOffsetEnd);
  MALIDRIVE_THROW_UNLESS(z_offset_end != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto z_offset_start = attribute_parser.As<double>(object::Repeat::kZOffsetStart);
  MALIDRIVE_THROW_UNLESS(z_offset_start != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto detach_from_reference_line = attribute_parser.As<bool>(object::Repeat::kDetachFromReferenceLine);
  const auto length_end = attribute_parser.As<double>(object::Repeat::kLengthEnd);
  const auto length_start = attribute_parser.As<double>(object::Repeat::kLengthStart);
  const auto radius_end = attribute_parser.As<double>(object::Repeat::kRadiusEnd);
  const auto radius_start = attribute_parser.As<double>(object::Repeat::kRadiusStart);
  const auto width_end = attribute_parser.As<double>(object::Repeat::kWidthEnd);
  const auto width_start = attribute_parser.As<double>(object::Repeat::kWidthStart);
  // @}

  return {detach_from_reference_line,
          distance.value(),
          height_end.value(),
          height_start.value(),
          length_end,
          length_start,
          length.value(),
          radius_end,
          radius_start,
          s.value(),
          t_end.value(),
          t_start.value(),
          width_end,
          width_start,
          z_offset_end.value(),
          z_offset_start.value()};
}

// Specialization to parse `object::Outline`'s node.
template <>
object::Outline NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Optional attributes.
  // @{
  const auto dynamic = attribute_parser.As<bool>(object::Outline::kClosed);
  const auto fill_type = attribute_parser.As<object::Outline::FillType>(object::Outline::kFillType);
  const auto id = attribute_parser.As<std::string>(object::Outline::kId);
  const auto lane_type = attribute_parser.As<Lane::Type>(object::Outline::kLaneType);
  const auto outer = attribute_parser.As<bool>(object::Outline::kOuter);
  // @}

  // CornerRoad elements
  tinyxml2::XMLElement* corner_road_element_xml = element_->FirstChildElement(object::CornerRoad::kCornerRoadTag);
  std::vector<object::CornerRoad> corner_roads;
  while (corner_road_element_xml) {
    auto corner_road = NodeParser(corner_road_element_xml, parser_configuration_).As<object::CornerRoad>();
    corner_roads.push_back(corner_road);
    corner_road_element_xml = corner_road_element_xml->NextSiblingElement(object::CornerRoad::kCornerRoadTag);
  }

  // CornerLocal elements
  tinyxml2::XMLElement* corner_local_element_xml = element_->FirstChildElement(object::CornerLocal::kCornerLocalTag);
  std::vector<object::CornerLocal> corner_locals;
  while (corner_local_element_xml) {
    auto corner_local = NodeParser(corner_local_element_xml, parser_configuration_).As<object::CornerLocal>();
    corner_locals.push_back(corner_local);
    corner_local_element_xml = corner_local_element_xml->NextSiblingElement(object::CornerLocal::kCornerLocalTag);
  }

  return {dynamic, fill_type, object::Outline::Id(id.value_or("none")), lane_type, outer, corner_roads, corner_locals};
}

// Specialization to parse `object::Outlines`'s node.
template <>
object::Outlines NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Outline elements
  tinyxml2::XMLElement* outline_element_xml = element_->FirstChildElement(object::Outline::kOutlineTag);
  std::vector<object::Outline> outlines;
  while (outline_element_xml) {
    auto outline = NodeParser(outline_element_xml, parser_configuration_).As<object::Outline>();
    outlines.push_back(outline);
    outline_element_xml = outline_element_xml->NextSiblingElement(object::Outline::kOutlineTag);
  }
  return {outlines};
}

// Specialization to parse `object::VertexRoad`'s node.
template <>
object::VertexRoad NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto dz = attribute_parser.As<double>(object::VertexRoad::kDZ);
  MALIDRIVE_THROW_UNLESS(dz != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s = attribute_parser.As<double>(object::VertexRoad::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto t = attribute_parser.As<double>(object::VertexRoad::kT);
  MALIDRIVE_THROW_UNLESS(t != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::VertexRoad::kId);
  const auto intersection_point = attribute_parser.As<bool>(object::VertexRoad::kIntersectionPoint);
  const auto radius = attribute_parser.As<double>(object::VertexRoad::kRadius);
  // @}

  return {dz.value(), object::VertexRoad::Id(id.value_or("none")), intersection_point, radius, s.value(), t.value()};
}

// Specialization to parse `object::VertexLocal`'s node.
template <>
object::VertexLocal NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto u = attribute_parser.As<double>(object::VertexLocal::kU);
  MALIDRIVE_THROW_UNLESS(u != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto v = attribute_parser.As<double>(object::VertexLocal::kV);
  MALIDRIVE_THROW_UNLESS(v != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto z = attribute_parser.As<double>(object::VertexLocal::kZ);
  MALIDRIVE_THROW_UNLESS(z != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::VertexLocal::kId);
  const auto intersection_point = attribute_parser.As<bool>(object::VertexLocal::kIntersectionPoint);
  const auto radius = attribute_parser.As<double>(object::VertexLocal::kRadius);
  // @}

  return {object::VertexLocal::Id(id.value_or("none")), intersection_point, radius, u.value(), v.value(), z.value()};
}

// Specialization to parse `object::Polyline`'s node.
template <>
object::Polyline NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::Polyline::kId);
  // @}

  // VertexRoad elements
  tinyxml2::XMLElement* vertex_road_element_xml = element_->FirstChildElement(object::VertexRoad::kVertexRoadTag);
  std::vector<object::VertexRoad> vertex_roads;
  while (vertex_road_element_xml) {
    auto vertex_road = NodeParser(vertex_road_element_xml, parser_configuration_).As<object::VertexRoad>();
    vertex_roads.push_back(vertex_road);
    vertex_road_element_xml = vertex_road_element_xml->NextSiblingElement(object::VertexRoad::kVertexRoadTag);
  }

  // VertexLocal elements
  tinyxml2::XMLElement* vertex_local_element_xml = element_->FirstChildElement(object::VertexLocal::kVertexLocalTag);
  std::vector<object::VertexLocal> vertex_locals;
  while (vertex_local_element_xml) {
    auto vertex_local = NodeParser(vertex_local_element_xml, parser_configuration_).As<object::VertexLocal>();
    vertex_locals.push_back(vertex_local);
    vertex_local_element_xml = vertex_local_element_xml->NextSiblingElement(object::VertexLocal::kVertexLocalTag);
  }

  return {object::Polyline::Id(id.value_or("none")), vertex_roads, vertex_locals};
}

// Specialization to parse `object::Skeleton`'s node.
template <>
object::Skeleton NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Outline elements
  tinyxml2::XMLElement* polyline_element_xml = element_->FirstChildElement(object::Polyline::kPolylinesTag);
  std::vector<object::Polyline> polylines;
  while (polyline_element_xml) {
    auto polyline = NodeParser(polyline_element_xml, parser_configuration_).As<object::Polyline>();
    polylines.push_back(polyline);
    polyline_element_xml = polyline_element_xml->NextSiblingElement(object::Polyline::kPolylinesTag);
  }
  return {polylines};
}

// Specialization to parse `object::Material`'s node.
template <>
object::Material NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Optional attributes.
  // @{
  const auto friction = attribute_parser.As<double>(object::Material::kFriction);
  const auto color = attribute_parser.As<Color>(object::Material::kRoadMarkColor);
  const auto roughness = attribute_parser.As<double>(object::Material::kRoughness);
  const auto surface = attribute_parser.As<std::string>(object::Material::kSurface);
  // @}

  return {friction, color, roughness, surface};
}

// Specialization to parse `object::Validity`'s node.
template <>
object::Validity NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto from_lane = attribute_parser.As<std::string>(object::Validity::kFromLane);
  MALIDRIVE_THROW_UNLESS(from_lane != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto to_lane = attribute_parser.As<std::string>(object::Validity::kToLane);
  MALIDRIVE_THROW_UNLESS(to_lane != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  return {object::Validity::Id(from_lane.value_or("none")), object::Validity::Id(to_lane.value_or("none"))};
}

// Specialization to parse `object::ParkingSpace`'s node.
template <>
object::ParkingSpace NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto access = attribute_parser.As<object::ParkingSpace::Access>(object::ParkingSpace::kAccess);
  MALIDRIVE_THROW_UNLESS(access != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto restrictions = attribute_parser.As<std::string>(object::ParkingSpace::kRestrictions);
  // @}

  return {access.value(), restrictions};
}

// Specialization to parse `object::CornerReference`'s node.
template <>
object::CornerReference NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::CornerReference::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  return {object::CornerReference::Id(id.value_or("none"))};
}

// Specialization to parse `object::Marking`'s node.
template <>
object::Marking NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto color = attribute_parser.As<Color>(object::Marking::kColor);
  MALIDRIVE_THROW_UNLESS(color != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto line_length = attribute_parser.As<double>(object::Marking::kLineLength);
  MALIDRIVE_THROW_UNLESS(line_length != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto side = attribute_parser.As<object::Marking::Side>(object::Marking::kSide);
  MALIDRIVE_THROW_UNLESS(side != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto space_length = attribute_parser.As<double>(object::Marking::kSpaceLength);
  MALIDRIVE_THROW_UNLESS(space_length != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto start_offset = attribute_parser.As<double>(object::Marking::kStartOffset);
  MALIDRIVE_THROW_UNLESS(start_offset != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto stop_offset = attribute_parser.As<double>(object::Marking::kStopOffset);
  MALIDRIVE_THROW_UNLESS(stop_offset != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto weight = attribute_parser.As<LaneRoadMark::Weight>(object::Marking::kWeight);
  const auto width = attribute_parser.As<double>(object::Marking::kWidth);
  const auto z_offset = attribute_parser.As<double>(object::Marking::kZOffset);
  // @}

  // CornerReference elements
  tinyxml2::XMLElement* corner_reference_element_xml =
      element_->FirstChildElement(object::CornerReference::kCornerReferenceTag);
  std::vector<object::CornerReference> corner_references;
  while (corner_reference_element_xml) {
    auto corner_reference =
        NodeParser(corner_reference_element_xml, parser_configuration_).As<object::CornerReference>();
    corner_references.push_back(corner_reference);
    corner_reference_element_xml =
        corner_reference_element_xml->NextSiblingElement(object::CornerReference::kCornerReferenceTag);
  }

  return {color.value(),        line_length.value(), side.value(), space_length.value(),
          start_offset.value(), stop_offset.value(), weight,       width,
          z_offset.value(),     corner_references};
}

// Specialization to parse `object::Markings`'s node.
template <>
object::Markings NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Outline elements
  tinyxml2::XMLElement* marking_element_xml = element_->FirstChildElement(object::Marking::kMarkingTag);
  std::vector<object::Marking> markings;
  while (marking_element_xml) {
    auto marking = NodeParser(marking_element_xml, parser_configuration_).As<object::Marking>();
    markings.push_back(marking);
    marking_element_xml = marking_element_xml->NextSiblingElement(object::Marking::kMarkingTag);
  }
  return {markings};
}

// Specialization to parse `object::Border`'s node.
template <>
object::Border NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto outline_id = attribute_parser.As<std::string>(object::Border::kOutlineId);
  MALIDRIVE_THROW_UNLESS(outline_id != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto width = attribute_parser.As<double>(object::Border::kWidth);
  MALIDRIVE_THROW_UNLESS(width != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto type = attribute_parser.As<object::Border::Type>(object::Border::kType);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto use_complete_outline = attribute_parser.As<bool>(object::Border::kUseCompleteOutline);
  // @}

  return {object::Border::Id(outline_id.value_or("none")), type.value(), use_complete_outline, width.value()};
}

// Specialization to parse `object::Borders`'s node.
template <>
object::Borders NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Border elements
  tinyxml2::XMLElement* border_element_xml = element_->FirstChildElement(object::Border::kBorderTag);
  std::vector<object::Border> borders;
  while (border_element_xml) {
    auto border = NodeParser(border_element_xml, parser_configuration_).As<object::Border>();
    borders.push_back(border);
    border_element_xml = border_element_xml->NextSiblingElement(object::Border::kBorderTag);
  }
  return {borders};
}

// Specialization to parse `object::ObjectReference`'s node.
template <>
object::ObjectReference NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::ObjectReference::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto orientation = attribute_parser.As<object::Orientation>(object::ObjectReference::kOrientation);
  MALIDRIVE_THROW_UNLESS(orientation != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s = attribute_parser.As<double>(object::ObjectReference::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto t = attribute_parser.As<double>(object::ObjectReference::kT);
  MALIDRIVE_THROW_UNLESS(t != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto valid_length = attribute_parser.As<double>(object::ObjectReference::kValidLength);
  const auto z_offset = attribute_parser.As<double>(object::ObjectReference::kZOffset);
  // @}

  // Validity elements
  tinyxml2::XMLElement* validity_element_xml = element_->FirstChildElement(object::Validity::kValidityTag);
  std::vector<object::Validity> validities;
  while (validity_element_xml) {
    auto validity = NodeParser(validity_element_xml, parser_configuration_).As<object::Validity>();
    validities.push_back(validity);
    validity_element_xml = validity_element_xml->NextSiblingElement(object::Validity::kValidityTag);
  }

  return {object::ObjectReference::Id(id.value_or("none")),
          orientation.value(),
          s.value(),
          t.value(),
          valid_length,
          z_offset,
          validities};
}

// Specialization to parse `object::Tunnel`'s node.
template <>
object::Tunnel NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::Tunnel::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto length = attribute_parser.As<double>(object::Tunnel::kLength);
  MALIDRIVE_THROW_UNLESS(length != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s = attribute_parser.As<double>(object::Tunnel::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto type = attribute_parser.As<object::Tunnel::Type>(object::Tunnel::kType);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto daylight = attribute_parser.As<double>(object::Tunnel::kDayLight);
  const auto lighting = attribute_parser.As<double>(object::Tunnel::kLighting);
  const auto name = attribute_parser.As<std::string>(object::Tunnel::kName);
  // @}

  // Validity elements
  tinyxml2::XMLElement* validity_element_xml = element_->FirstChildElement(object::Validity::kValidityTag);
  std::vector<object::Validity> validities;
  while (validity_element_xml) {
    auto validity = NodeParser(validity_element_xml, parser_configuration_).As<object::Validity>();
    validities.push_back(validity);
    validity_element_xml = validity_element_xml->NextSiblingElement(object::Validity::kValidityTag);
  }

  return {daylight,  object::Tunnel::Id(id.value_or("none")), length.value(), lighting, name, s.value(), type.value(),
          validities};
}

// Specialization to parse `object::Bridge`'s node.
template <>
object::Bridge NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::Bridge::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto length = attribute_parser.As<double>(object::Bridge::kLength);
  MALIDRIVE_THROW_UNLESS(length != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto s = attribute_parser.As<double>(object::Bridge::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto type = attribute_parser.As<object::Bridge::Type>(object::Bridge::kType);
  MALIDRIVE_THROW_UNLESS(type != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto name = attribute_parser.As<std::string>(object::Bridge::kName);
  // @}

  // Validity elements
  tinyxml2::XMLElement* validity_element_xml = element_->FirstChildElement(object::Validity::kValidityTag);
  std::vector<object::Validity> validities;
  while (validity_element_xml) {
    auto validity = NodeParser(validity_element_xml, parser_configuration_).As<object::Validity>();
    validities.push_back(validity);
    validity_element_xml = validity_element_xml->NextSiblingElement(object::Validity::kValidityTag);
  }

  return {object::Bridge::Id(id.value_or("none")), length.value(), name, s.value(), type.value(), validities};
}

// Specialization to parse `object::CRG`'s node.
template <>
object::CRG NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto file = attribute_parser.As<std::string>(object::CRG::kFile);
  MALIDRIVE_THROW_UNLESS(file != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto hide_road_surface_crg = attribute_parser.As<bool>(object::CRG::kHideRoadSurfaceCRG);
  MALIDRIVE_THROW_UNLESS(hide_road_surface_crg != std::nullopt, maliput::common::road_network_description_parser_error);
  const auto z_scale = attribute_parser.As<double>(object::CRG::kZScale);
  MALIDRIVE_THROW_UNLESS(z_scale != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  return {file.value(), hide_road_surface_crg.value(), z_scale.value()};
}

// Specialization to parse `object::Surface`'s node.
template <>
object::Surface NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // CRG elements
  tinyxml2::XMLElement* crg_element_xml = element_->FirstChildElement(object::CRG::kCRGTag);
  std::optional<object::CRG> crg;
  if (crg_element_xml) {
    crg = NodeParser(crg_element_xml, parser_configuration_).As<object::CRG>();
  }

  return {crg};
}

// Specialization to parse `Object`'s node.
template <>
object::Object NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Non-optional attributes.
  // @{
  const auto id = attribute_parser.As<std::string>(object::Object::kId);
  MALIDRIVE_THROW_UNLESS(id != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto s = attribute_parser.As<double>(object::Object::kS);
  MALIDRIVE_THROW_UNLESS(s != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto t = attribute_parser.As<double>(object::Object::kT);
  MALIDRIVE_THROW_UNLESS(t != std::nullopt, maliput::common::road_network_description_parser_error);

  const auto z_offset = attribute_parser.As<double>(object::Object::kZOffset);
  MALIDRIVE_THROW_UNLESS(z_offset != std::nullopt, maliput::common::road_network_description_parser_error);
  // @}

  // Optional attributes.
  // @{
  const auto dynamic = attribute_parser.As<bool>(object::Object::kDynamic);
  const auto hdg = attribute_parser.As<double>(object::Object::kHdg);
  const auto height = attribute_parser.As<double>(object::Object::kHeight);
  const auto length = attribute_parser.As<double>(object::Object::kLength);
  const auto name = attribute_parser.As<std::string>(object::Object::kName);
  const auto orientation = attribute_parser.As<object::Orientation>(object::Object::kOrientation);
  const auto perp_to_road = attribute_parser.As<bool>(object::Object::kPerpToRoad);
  const auto pitch = attribute_parser.As<double>(object::Object::kPitch);
  const auto radius = attribute_parser.As<double>(object::Object::kRadius);
  const auto roll = attribute_parser.As<double>(object::Object::kRoll);
  const auto subtype = attribute_parser.As<std::string>(object::Object::kSubtype);
  const auto type = attribute_parser.As<object::Object::ObjectType>(object::Object::kType);
  const auto valid_length = attribute_parser.As<double>(object::Object::kValidLength);
  const auto width = attribute_parser.As<double>(object::Object::kWidth);
  // @}

  // Elements.
  // Repeat elements
  tinyxml2::XMLElement* repeat_element_xml = element_->FirstChildElement(object::Repeat::kRepeatTag);
  std::vector<object::Repeat> repeats;
  while (repeat_element_xml) {
    auto repeat = NodeParser(repeat_element_xml, parser_configuration_).As<object::Repeat>();
    repeats.push_back(repeat);
    repeat_element_xml = repeat_element_xml->NextSiblingElement(object::Repeat::kRepeatTag);
  }

  // Outlines element
  tinyxml2::XMLElement* outlines_element_xml = element_->FirstChildElement(object::Outlines::kOutlinesTag);
  std::optional<object::Outlines> outlines;
  if (outlines_element_xml) {
    outlines = NodeParser(outlines_element_xml, parser_configuration_).As<object::Outlines>();
  }

  // Skeleton element
  tinyxml2::XMLElement* skeleton_element_xml = element_->FirstChildElement(object::Skeleton::kSkeletonTag);
  std::optional<object::Skeleton> skeleton;
  if (skeleton_element_xml) {
    skeleton = NodeParser(skeleton_element_xml, parser_configuration_).As<object::Skeleton>();
  }

  // Material elements
  tinyxml2::XMLElement* material_element_xml = element_->FirstChildElement(object::Material::kMaterialTag);
  std::vector<object::Material> materials;
  while (material_element_xml) {
    auto material = NodeParser(material_element_xml, parser_configuration_).As<object::Material>();
    materials.push_back(material);
    material_element_xml = material_element_xml->NextSiblingElement(object::Material::kMaterialTag);
  }

  // Validity elements
  tinyxml2::XMLElement* validity_element_xml = element_->FirstChildElement(object::Validity::kValidityTag);
  std::vector<object::Validity> validities;
  while (validity_element_xml) {
    auto validity = NodeParser(validity_element_xml, parser_configuration_).As<object::Validity>();
    validities.push_back(validity);
    validity_element_xml = validity_element_xml->NextSiblingElement(object::Validity::kValidityTag);
  }

  // ParkingSpace element
  tinyxml2::XMLElement* parking_space_element_xml = element_->FirstChildElement(object::ParkingSpace::kParkingSpaceTag);
  std::optional<object::ParkingSpace> parking_space;
  if (parking_space_element_xml) {
    parking_space = NodeParser(parking_space_element_xml, parser_configuration_).As<object::ParkingSpace>();
  }

  // Markings element
  tinyxml2::XMLElement* markings_element_xml = element_->FirstChildElement(object::Markings::kMarkingsTag);
  std::optional<object::Markings> markings;
  if (markings_element_xml) {
    markings = NodeParser(markings_element_xml, parser_configuration_).As<object::Markings>();
  }

  // Borders element
  tinyxml2::XMLElement* borders_element_xml = element_->FirstChildElement(object::Borders::kBordersTag);
  std::optional<object::Borders> borders;
  if (borders_element_xml) {
    borders = NodeParser(borders_element_xml, parser_configuration_).As<object::Borders>();
  }

  // ObjectReferences elements
  tinyxml2::XMLElement* object_reference_element_xml =
      element_->FirstChildElement(object::ObjectReference::kObjectReferenceTag);
  std::vector<object::ObjectReference> object_references;
  while (object_reference_element_xml) {
    auto object_reference =
        NodeParser(object_reference_element_xml, parser_configuration_).As<object::ObjectReference>();
    object_references.push_back(object_reference);
    object_reference_element_xml =
        object_reference_element_xml->NextSiblingElement(object::ObjectReference::kObjectReferenceTag);
  }

  // Tunnel elements
  tinyxml2::XMLElement* tunnel_element_xml = element_->FirstChildElement(object::Tunnel::kTunnelTag);
  std::vector<object::Tunnel> tunnels;
  while (tunnel_element_xml) {
    auto tunnel = NodeParser(tunnel_element_xml, parser_configuration_).As<object::Tunnel>();
    tunnels.push_back(tunnel);
    tunnel_element_xml = tunnel_element_xml->NextSiblingElement(object::Tunnel::kTunnelTag);
  }

  // Bridge elements
  tinyxml2::XMLElement* bridge_element_xml = element_->FirstChildElement(object::Bridge::kBridgeTag);
  std::vector<object::Bridge> bridges;
  while (bridge_element_xml) {
    auto bridge = NodeParser(bridge_element_xml, parser_configuration_).As<object::Bridge>();
    bridges.push_back(bridge);
    bridge_element_xml = bridge_element_xml->NextSiblingElement(object::Bridge::kBridgeTag);
  }

  // Surface element
  tinyxml2::XMLElement* surface_element_xml = element_->FirstChildElement(object::Surface::kSurfaceTag);
  std::optional<object::Surface> surface;
  if (surface_element_xml) {
    surface = NodeParser(surface_element_xml, parser_configuration_).As<object::Surface>();
  }

  return {dynamic,
          hdg,
          height,
          object::Object::Id(id.value_or("none")),
          length,
          name,
          orientation,
          perp_to_road,
          pitch,
          radius,
          roll,
          s.value(),
          subtype,
          t.value(),
          type,
          valid_length,
          width,
          z_offset.value(),
          repeats,
          outlines,
          skeleton,
          materials,
          validities,
          parking_space,
          markings,
          borders,
          object_references,
          tunnels,
          bridges,
          surface};
}

// Specialization to parse `object::Objects`'s node.
template <>
object::Objects NodeParser::As() const {
  const AttributeParser attribute_parser(element_, parser_configuration_);

  // Object elements
  tinyxml2::XMLElement* objects_element_xml = element_->FirstChildElement(object::Object::kObjectTag);
  std::vector<object::Object> objects;
  while (objects_element_xml) {
    auto object = NodeParser(objects_element_xml, parser_configuration_).As<object::Object>();
    objects.push_back(object);
    objects_element_xml = objects_element_xml->NextSiblingElement(object::Object::kObjectTag);
  }

  return {objects};
}

}  // namespace xodr
}  // namespace malidrive
