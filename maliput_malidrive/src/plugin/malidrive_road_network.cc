#include <memory>

#include "maliput/plugin/road_network_plugin.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/constants.h"

namespace malidrive {
namespace plugin {
namespace {

malidrive::builder::RoadNetworkConfiguration GetPropertiesFromStringMap(
    const std::map<std::string, std::string> parameters) {
  std::string id{"Maliput_malidrive RoadGeometry"};
  auto it = parameters.find("road_geometry_id");
  if (it != parameters.end()) {
    id = it->second;
  }
  const maliput::api::RoadGeometryId rg_id{id};

  std::string opendrive_file;
  it = parameters.find("opendrive_file");
  if (it != parameters.end()) {
    opendrive_file = it->second;
  }

  double linear_tolerance{malidrive::constants::kLinearTolerance};
  it = parameters.find("linear_tolerance");
  if (it != parameters.end()) {
    linear_tolerance = std::stod(it->second);
  }
  double angular_tolerance{malidrive::constants::kAngularTolerance};
  it = parameters.find("angular_tolerance");
  if (it != parameters.end()) {
    angular_tolerance = std::stod(it->second);
  }
  double scale_length{malidrive::constants::kScaleLength};
  it = parameters.find("scale_length");
  if (it != parameters.end()) {
    scale_length = std::stod(it->second);
  }
  // TODO: Parse more parameters.
  malidrive::InertialToLaneMappingConfig inertial_to_lane(malidrive::constants::kExplorationRadius,
                                                          malidrive::constants::kNumIterations);
  return {rg_id, opendrive_file, linear_tolerance, angular_tolerance, scale_length, inertial_to_lane};
}

}  // namespace

class MalidriveRoadNetwork : public maliput::plugin::RoadNetworkPlugin {
 public:
  std::unique_ptr<const maliput::api::RoadNetwork> LoadRoadNetwork(
      const std::map<std::string, std::string> parameters) const override {
    return malidrive::builder::RoadNetworkBuilder(GetPropertiesFromStringMap(parameters))();
  }
};

// the class factories
extern "C" std::unique_ptr<maliput::plugin::RoadNetworkPlugin> LoadMaliputRoadNetwork() {
  return std::make_unique<MalidriveRoadNetwork>();
}

}  // namespace plugin
}  // namespace malidrive
