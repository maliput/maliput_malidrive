// Copyright 2021 Toyota Research Institute
/// @file xodr_extract.cc
///
/// Generate a new XODR file which is filled with Roads that are extracted from an another XODR file.
/// @code{sh}
/// xodr_extract <xodr_file> <road_1> <road_2> ... <road_n> --output_file_path=<output_file_path>
/// @endcode
///
/// Use `--help` argument to see available arguments.
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <maliput/common/logger.h>
#include <tinyxml2.h>

#include "log_level_flag.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace applications {
namespace xodr {
namespace {

// @{ CLI Arguments
DEFINE_bool(update_linkage, true, "Update road linkage information and junction id information. Default true");
DEFINE_string(output_file_path, "./xodr_extract_output.xodr", "Output XODR file path");
MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG();
// @}

// @return A string with the usage message.
std::string GetUsageMessage() {
  std::stringstream ss;
  ss << "CLI for XODR creation out of selected Roads of another XODR file" << std::endl << std::endl;
  ss << "  xodr_extract <xodr_file> <road_1> <road_2> ... <road_n>" << std::endl;
  return ss.str();
}

// @returns road ids in a vector form located at `argv`. `argc` and `argv` are the arguments
// received by the CLI application.
std::vector<std::string> GetRoadIdsFromCLI(int argc, char** argv) {
  std::vector<std::string> road_ids{};
  for (int i = 2; i < argc; ++i) {
    road_ids.push_back(argv[i]);
  }
  return road_ids;
}

std::string FromVectorStrToString(const std::vector<std::string>& vector_str) {
  std::stringstream ss;
  for (const auto& v : vector_str) {
    ss << v << ", ";
  }
  return ss.str();
}

// Extracts Roads out of a XODR file and a new XODR file.
// The typical use would be:
// @code{cpp}
//   XodrExtract xodr_extract(xodr_file_path);
//   xodr_extract.Extract(road_ids, true);
//   xodr_extract.GenerateXodrFile(output_file_path)
// @endcode
class XodrExtract {
 public:
  // XodrExtract Constructor
  // @param xodr_file_path XODR file path.
  //
  // @throws maliput::common::assertion_error When @p xodr_file_path can't be loaded.
  explicit XodrExtract(const std::string& xodr_file_path) {
    MALIDRIVE_VALIDATE(xodr_doc_.LoadFile(xodr_file_path.c_str()) == tinyxml2::XML_SUCCESS,
                       maliput::common::assertion_error,
                       std::string("XODR file named: ") + xodr_file_path.c_str() + std::string(" couldn't be loaded."));
    IdentifyRoadNodes();
  }

  // Extract Roads from the XODR file and add them to an internal collection.
  // See #GenerateXodrFile() to actual create a XODR file out of the extracted roads.
  //
  // @param road_ids Collection of road ids that are wanted to be extracted.
  // @param update_linkage True if predecessor,successor nodes and the junction id of the of the Roads should be updated
  //                       by removing them. Otherwise those nodes won't be modified.
  void Extract(const std::vector<std::string>& road_ids, bool update_linkage) {
    for (const auto& road_id : road_ids) {
      const auto road_id_it = road_header_nodes_.find(road_id);
      if (road_id_it == road_header_nodes_.end()) {
        maliput::log()->error("RoadId {} wasn't found. Skipping road. ", road_id);
        continue;
      }
      if (update_linkage) {
        // Update junction id to not match any junction.
        road_id_it->second->SetAttribute("junction", "-1");
        // Remove road link node.
        tinyxml2::XMLElement* road_header_node = road_id_it->second->FirstChildElement("link");
        road_id_it->second->DeleteChild(road_header_node);
      }
      output_road_header_nodes_.insert(*road_id_it);
      maliput::log()->debug("RoadId {} has been found.", road_id);
    }
  }

  // Create a XODR file using:
  //  - Default header file with minimum information.
  //  - Roads selected by calling #Extract() method.
  // @param output_file Is the path of the output XODR file.
  void GenerateXodrFile(const std::string& output_file) {
    std::string output_str{R"R(<OpenDRIVE><header revMajor="1" revMinor="4" name="" vendor="xodr_extract App"/>)R"};
    for (const auto& road_node : output_road_header_nodes_) {
      tinyxml2::XMLPrinter printer;
      road_node.second->Accept(&printer);
      output_str += printer.CStr();
    }
    output_str += std::string{R"R(</OpenDRIVE>)R"};
    tinyxml2::XMLDocument output_doc;
    output_doc.Parse(output_str.c_str());
    output_doc.SaveFile(output_file.c_str());
  }

 private:
  // Identify Road nodes in the XODR files.
  // @throws maliput::common::assertion_error When the XML file isn't a XODR file.
  // @throws maliput::common::assertion_error When XODR file's roads doesn't have 'id' attributes.
  void IdentifyRoadNodes() {
    const std::string kOpenDrive{"OpenDRIVE"};
    const std::string kRoad{"road"};
    const std::string kRoadId{"id"};
    // Check if it is a XODR file.
    tinyxml2::XMLElement* xodr_root_node = xodr_doc_.FirstChildElement();
    MALIDRIVE_VALIDATE(static_cast<std::string>(xodr_root_node->Value()) == static_cast<std::string>(kOpenDrive),
                       maliput::common::assertion_error, "XML file doesn't correspond to a XODR file.");

    // Parse XODR `road` headers.
    tinyxml2::XMLElement* road_header_node = xodr_root_node->FirstChildElement(kRoad.c_str());
    while (road_header_node) {
      const char* road_id_attribute = road_header_node->Attribute(kRoadId.c_str());
      MALIDRIVE_THROW_UNLESS(road_id_attribute != nullptr);
      road_header_nodes_.emplace(road_id_attribute, road_header_node);
      road_header_node = road_header_node->NextSiblingElement(kRoad.c_str());
    }
  }

  // XMLDocument instance holding XODR file passed to constructor.
  tinyxml2::XMLDocument xodr_doc_;
  // Holds pointers to the road nodes in the XODR files.
  // Key of the map is the id of the road while the value corresponds to the XMLElement pointer to the node.
  std::map<std::string, tinyxml2::XMLElement*> road_header_nodes_{};
  // Holds pointers to the road nodes that will be used to generate a new XODR file. This collection is increased by
  // calling #Extract() method. Key of the map is the id of the road while the value corresponds to the XMLElement
  // pointer to the node.
  std::map<std::string, tinyxml2::XMLElement*> output_road_header_nodes_{};
};

int Main(int argc, char** argv) {
  // Handles CLI arguments.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(GetUsageMessage());
  if (argc < 3) {
    std::cout << "\nWrong number of arguments. See application documentation: \n" << std::endl;
    gflags::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  maliput::common::set_log_level(FLAGS_log_level);
  const std::vector<std::string> road_ids = GetRoadIdsFromCLI(argc, argv);
  maliput::log()->info(
      "xodr_extract application\n\t|__ xodr_file_path: {}\n\t|__ output_file_path: {}\n\t|__ update_linkage: {}\n\t|__ "
      "road_ids({}): {} ",
      argv[1], FLAGS_output_file_path, FLAGS_update_linkage ? "True" : "False", road_ids.size(),
      FromVectorStrToString(road_ids));

  XodrExtract xodr_extract{argv[1]};
  xodr_extract.Extract(road_ids, FLAGS_update_linkage);
  xodr_extract.GenerateXodrFile(FLAGS_output_file_path);
  maliput::log()->info("XODR file created: {}", FLAGS_output_file_path);

  return 1;
}

}  // namespace
}  // namespace xodr
}  // namespace applications
}  // namespace malidrive

int main(int argc, char** argv) { return malidrive::applications::xodr::Main(argc, argv); }
