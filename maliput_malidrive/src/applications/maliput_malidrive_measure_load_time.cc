// Copyright 2021 Toyota Research Institute
#include <chrono>
#include <numeric>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "log_level_flag.h"
#include "maliput/common/logger.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/loader/loader.h"
#include "utility/file_tools.h"

namespace malidrive {
namespace applications {
namespace {

DEFINE_string(xodr_file_path, "", "XODR file path.");
DEFINE_int32(iterations, 1, "Number of iterations for loading the Road Geometry. Default: 1");
DEFINE_string(build_policy, "sequential", "Build policy, it could be `sequential` or `parallel`. Default: sequential");
DEFINE_int32(threads, 0, "Number of threads to create the Road Geometry. Default: Automatic selection.");

MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG();

// Returns a string summarizing how to use the application.
std::string GetUsageString() {
  return "[MEASURE_LOAD_TIME_APPLICATION]\n\n//----- How to use -----//\n\n "
         "malidrive_measure_load_time --xodr_file_path=<path_to_xodr_file> "
         "--iterations=<number_of_iterations> "
         "--build_policy=<sequential_or_parallel> "
         "--threads=<number_of_threads>\n\n"
         "Note:\n--threads is the number of threads a thread pool will spawn."
         "It is optional and only takes effect when "
         "--build_policy=parallel is passed. \nIf not provided, the number "
         "of threads minus one the architecture supports will be used.\n\n";
}

// Convenient converter from string to a BuildPolicy::Type object.
// @param policy_str Is the string containing the name of a policy.
// @returns A BuildPolicy::Type that matches `policy_str`.
//
// @throw maliput::common::assertion_error When `policy_str` doesn't match any BuildPolicy::Type.
builder::BuildPolicy::Type StringToBuildTypePolicy(const std::string& policy_str) {
  if (policy_str == "sequential") {
    return builder::BuildPolicy::Type::kSequential;
  } else if (policy_str == "parallel") {
    return builder::BuildPolicy::Type::kParallel;
  } else {
    MALIDRIVE_ABORT_MSG("Unknown policy: " + policy_str);
  }
}

// Measure the time that it takes to create the RoadNetwork.
//
// @param xodr_filename Is the name of the XODR map to load.
// @param build_policy Indicates whether use multiple threads for building lanes or not.
// @return The time in seconds.
double MeasureLoadTime(const std::string& xodr_filename, const builder::BuildPolicy& build_policy) {
  auto start = std::chrono::high_resolution_clock::now();
  auto rn = malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(
      {{maliput::api::RoadGeometryId(utility::GetFileNameFromPath(xodr_filename)), xodr_filename,
        constants::kLinearTolerance, constants::kAngularTolerance, constants::kScaleLength,
        InertialToLaneMappingConfig(constants::kExplorationRadius, constants::kNumIterations), build_policy}});
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = (end - start);
  return duration.count();
}

int Main(int argc, char* argv[]) {
  maliput::log()->info(GetUsageString());
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  maliput::common::set_log_level(FLAGS_log_level);

  MALIDRIVE_DEMAND(FLAGS_iterations > 0);

  std::vector<double> times;
  times.reserve(FLAGS_iterations);

  const std::optional<int> num_threads{FLAGS_threads > 0 ? std::make_optional<int>(FLAGS_threads) : std::nullopt};
  for (int i = 0; i < FLAGS_iterations; i++) {
    times.push_back(MeasureLoadTime(FLAGS_xodr_file_path,
                                    builder::BuildPolicy{StringToBuildTypePolicy(FLAGS_build_policy), num_threads}));
  }
  const double mean_time = (std::accumulate(times.begin(), times.end(), 0.)) / static_cast<double>(times.size());
  maliput::log()->info("\tMean time after {} iterations is ----> {} seconds.\n", FLAGS_iterations, mean_time);

  return 0;
}

}  // namespace
}  // namespace applications
}  // namespace malidrive

int main(int argc, char* argv[]) { return malidrive::applications::Main(argc, argv); }
