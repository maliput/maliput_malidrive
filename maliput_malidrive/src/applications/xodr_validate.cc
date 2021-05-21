// Copyright 2020 Toyota Research Institute
// Validates a XODR map.
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <gflags/gflags.h>

#include "log_level_flag.h"
#include "maliput/common/logger.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace applications {
namespace xodr {
namespace {

// @{ CLI Arguments
DEFINE_string(xodr_file, "", "XODR input file defining a Malidrive road geometry");
DEFINE_double(tolerance, 1e-3, "Tolerance to validate continuity in piecewise defined geometries.");
DEFINE_bool(permissive_mode, false, "If true, the xodr parser is more flexible according to the OpenDrive standard.");
MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG();
// @}

int Main(int argc, char** argv) {
  // Handles CLI arguments.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  maliput::common::set_log_level(FLAGS_log_level);
  if (FLAGS_xodr_file.empty()) {
    maliput::log()->error("No input file specified.");
    return 1;
  }
  maliput::log()->info("Parser permissive mode: {}", FLAGS_permissive_mode ? "enabled" : "disabled");

  // Tries to load the XODR map and logs the result.
  try {
    auto db_manager = malidrive::xodr::LoadDataBaseFromFile(FLAGS_xodr_file, {FLAGS_tolerance, FLAGS_permissive_mode});
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
