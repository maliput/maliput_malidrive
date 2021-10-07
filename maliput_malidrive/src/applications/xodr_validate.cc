// Copyright 2020 Toyota Research Institute
// Validates a XODR map.
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <gflags/gflags.h>
#include <maliput/common/logger.h>

#include "applications/log_level_flag.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace applications {
namespace xodr {
namespace {

// @return A string with the usage message.
std::string GetUsageMessage() {
  std::stringstream ss;
  ss << "CLI for XODR validation:" << std::endl << std::endl;
  ss << "  xodr_validate --xodr_file=<xodr_file_path>" << std::endl;
  return ss.str();
}

// @{ CLI Arguments
DEFINE_string(xodr_file, "", "XODR input file defining a Malidrive road geometry");
DEFINE_double(tolerance, 1e-3, "Tolerance to validate continuity in piecewise defined geometries.");
DEFINE_bool(allow_schema_errors, false, "If true, the XODR parser will attempt to work around XODR schema violations.");
DEFINE_bool(allow_semantic_errors, false,
            "If true, the XODR parser will attempt to work around XODR semantic violations.");
MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG();
// @}

int Main(int argc, char** argv) {
  // Handles CLI arguments.
  gflags::SetUsageMessage(GetUsageMessage());
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  maliput::common::set_log_level(FLAGS_log_level);
  if (FLAGS_xodr_file.empty()) {
    maliput::log()->error("No input file specified.");
    return 1;
  }
  maliput::log()->info("Parser: Allow schema errors: {}", (FLAGS_allow_schema_errors ? "enabled" : "disabled"));
  maliput::log()->info("Parser: Allow semantic errors: {}", (FLAGS_allow_semantic_errors ? "enabled" : "disabled"));

  // Tries to load the XODR map and logs the result.
  try {
    auto db_manager = malidrive::xodr::LoadDataBaseFromFile(
        FLAGS_xodr_file, {FLAGS_tolerance, FLAGS_allow_schema_errors, FLAGS_allow_semantic_errors});
    std::cout << "Successfully loaded the map." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << "The map could not be loaded." << std::endl;
  }
  return 1;
}

}  // namespace
}  // namespace xodr
}  // namespace applications
}  // namespace malidrive

int main(int argc, char** argv) { return malidrive::applications::xodr::Main(argc, argv); }
