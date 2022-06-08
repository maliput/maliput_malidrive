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
#include "maliput_malidrive/xodr/connection.h"

#include <map>

namespace malidrive {
namespace xodr {
namespace {

const std::map<std::string, Connection::Type> str_to_type_map{{"default", Connection::Type::kDefault},
                                                              {"virtual", Connection::Type::kVirtual}};

const std::map<Connection::Type, std::string> type_to_str_map{{Connection::Type::kDefault, "default"},
                                                              {Connection::Type::kVirtual, "virtual"}};

const std::map<std::string, Connection::ContactPoint> str_to_contact_point_map{
    {"start", Connection::ContactPoint::kStart}, {"end", Connection::ContactPoint::kEnd}};

const std::map<Connection::ContactPoint, std::string> contact_point_to_str_map{
    {Connection::ContactPoint::kStart, "start"}, {Connection::ContactPoint::kEnd, "end"}};

}  // namespace

Connection::Type Connection::str_to_type(const std::string& type) {
  if (str_to_type_map.find(type) == str_to_type_map.end()) {
    MALIDRIVE_THROW_MESSAGE(type + " type is not available.");
  }
  return str_to_type_map.at(type);
}

std::string Connection::type_to_str(Type type) { return type_to_str_map.at(type); }

Connection::ContactPoint Connection::str_to_contact_point(const std::string& contact_point) {
  if (str_to_contact_point_map.find(contact_point) == str_to_contact_point_map.end()) {
    MALIDRIVE_THROW_MESSAGE(contact_point + " contact point is not available.");
  }
  return str_to_contact_point_map.at(contact_point);
}

std::string Connection::contact_point_to_str(ContactPoint contact_point) {
  return contact_point_to_str_map.at(contact_point);
}

bool Connection::operator==(const Connection& other) const {
  return id == other.id && incoming_road == other.incoming_road && connecting_road == other.connecting_road &&
         contact_point == other.contact_point && connection_master == other.connection_master && type == other.type &&
         lane_links == other.lane_links;
}

}  // namespace xodr
}  // namespace malidrive
