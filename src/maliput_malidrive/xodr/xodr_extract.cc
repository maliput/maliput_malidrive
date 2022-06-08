// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/xodr/xodr_extract.h"

#include <algorithm>

#include <maliput/common/logger.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {

std::string XodrExtract(tinyxml2::XMLDocument* xodr_doc, const std::vector<std::string>& road_ids,
                        bool update_linkage) {
  MALIDRIVE_THROW_UNLESS(xodr_doc != nullptr);
  MALIDRIVE_THROW_UNLESS(!road_ids.empty());

  const std::string kOpenDrive{"OpenDRIVE"};
  const std::string kRoad{"road"};
  const std::string kRoadId{"id"};
  std::vector<std::string> road_ids_left = road_ids;
  // Check if it is a XODR file.
  tinyxml2::XMLElement* xodr_root_node = xodr_doc->FirstChildElement();
  MALIDRIVE_VALIDATE(static_cast<std::string>(xodr_root_node->Value()) == static_cast<std::string>(kOpenDrive),
                     maliput::common::assertion_error, "XML file doesn't correspond to a XODR file.");

  // Holds pointers to the road nodes that will be used to generate a new XODR file.
  std::vector<tinyxml2::XMLElement*> output_road_header_nodes{};
  // Look for the `road_ids` among the <road> nodes.
  tinyxml2::XMLElement* road_header_node = xodr_root_node->FirstChildElement(kRoad.c_str());
  while (road_header_node) {
    const char* road_id_attribute = road_header_node->Attribute(kRoadId.c_str());
    MALIDRIVE_THROW_UNLESS(road_id_attribute != nullptr);
    auto road_id_it = std::find(road_ids_left.begin(), road_ids_left.end(), road_id_attribute);
    if (road_id_it == road_ids_left.end()) {
      road_header_node = road_header_node->NextSiblingElement(kRoad.c_str());
      continue;
    }
    maliput::log()->debug("RoadId {} has been found.", *road_id_it);
    if (update_linkage) {
      // Update junction id to not match any junction.
      road_header_node->SetAttribute("junction", "-1");
      // Remove road link node.
      tinyxml2::XMLElement* road_link_node = road_header_node->FirstChildElement("link");
      road_header_node->DeleteChild(road_link_node);
    }
    road_ids_left.erase(road_id_it);
    output_road_header_nodes.push_back(road_header_node);
    // Move to the next road header.
    road_header_node = road_header_node->NextSiblingElement(kRoad.c_str());
  }

  for (const auto& road_id : road_ids_left) {
    maliput::log()->error("RoadId {} wasn't found. Skipping road.", road_id);
  }

  // Generate new xodr description.
  std::string output_str{R"R(<OpenDRIVE><header revMajor="1" revMinor="4" name="" vendor="xodr_extract App"/>)R"};
  for (const auto& road_node : output_road_header_nodes) {
    tinyxml2::XMLPrinter printer;
    road_node->Accept(&printer);
    output_str += printer.CStr();
  }
  output_str += std::string{R"R(</OpenDRIVE>)R"};
  tinyxml2::XMLDocument output_doc;
  // Parses XML description to validate the format.
  output_doc.Parse(output_str.c_str());
  // Serializes new XODR description.
  tinyxml2::XMLPrinter printer;
  output_doc.Print(&printer);
  return printer.CStr();
}

}  // namespace xodr
}  // namespace malidrive
