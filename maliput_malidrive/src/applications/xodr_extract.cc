// Copyright 2021 Toyota Research Institute
///
/// Generate a new XODR file which is filled with Roads that are extracted from an another XODR file.
/// @code{sh}
/// xodr_extract <xodr_file> <road_1> <road_2> ... <road_n> --output_file_path=<output_file_path>
/// @endcode
///
/// Use `--help` argument to see available arguments.
#include <iostream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <maliput/common/logger.h>
#include <tinyxml2.h>

#include "log_level_flag.h"
#include "maliput_malidrive/common/macros.h"
// clang-format off
#include "maliput_malidrive/xodr/xodr_extract.h"
// clang-format on

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

  // Loads XODR file.
  tinyxml2::XMLDocument xodr_doc;
  MALIDRIVE_VALIDATE(xodr_doc.LoadFile(argv[1]) == tinyxml2::XML_SUCCESS, maliput::common::assertion_error,
                     std::string("XODR file named: ") + argv[1] + std::string(" couldn't be loaded."));

  // Generates a new XODR description.
  const std::string new_xodr_description = malidrive::xodr::XodrExtract(&xodr_doc, road_ids, FLAGS_update_linkage);
  // Dumps new XODR to disk.
  tinyxml2::XMLDocument output_xodr_doc;
  MALIDRIVE_VALIDATE(output_xodr_doc.Parse(new_xodr_description.c_str()) == tinyxml2::XML_SUCCESS,
                     maliput::common::assertion_error, std::string("New created XODR file named can't be parsed."));
  output_xodr_doc.SaveFile(FLAGS_output_file_path.c_str());
  maliput::log()->info("XODR file created: {}", FLAGS_output_file_path);

  return 1;
}

}  // namespace
}  // namespace xodr
}  // namespace applications
}  // namespace malidrive

int main(int argc, char** argv) { return malidrive::applications::xodr::Main(argc, argv); }
