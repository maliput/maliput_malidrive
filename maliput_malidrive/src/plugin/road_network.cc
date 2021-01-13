// Copyright 2021 Toyota Research Institute
#include <memory>
#include <optional>

#include "maliput/plugin/road_network_loader.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/constants.h"

namespace malidrive {
namespace plugin {
namespace {

// Return a builder::RoadNetworkConfiguration object out of a map of strings.
// @param parameters  A dictionary of properties to fill in a builder::RoadNetworkConfiguration struct.
//                    Keys are the names of attributes in builder::RoadNetworkConfiguration.
// @returns A builder::RoadNetworkConfiguration.
builder::RoadNetworkConfiguration GetPropertiesFromStringMap(const std::map<std::string, std::string>& parameters) {
  auto it = parameters.find("road_geometry_id");
  const maliput::api::RoadGeometryId rg_id{it != parameters.end() ? it->second : "Maliput_malidrive RoadGeometry"};

  it = parameters.find("opendrive_file");
  const std::string opendrive_file{it != parameters.end() ? it->second : ""};

  it = parameters.find("linear_tolerance");
  const double linear_tolerance{it != parameters.end() ? std::stod(it->second) : constants::kLinearTolerance};

  it = parameters.find("angular_tolerance");
  const double angular_tolerance{it != parameters.end() ? std::stod(it->second) : constants::kAngularTolerance};

  it = parameters.find("scale_length");
  const double scale_length{it != parameters.end() ? std::stod(it->second) : constants::kScaleLength};

  // TODO(#4): Not being parsed because it is not used in the maliput_malidrive backend.
  const InertialToLaneMappingConfig inertial_to_lane(constants::kExplorationRadius, constants::kNumIterations);

  it = parameters.find("build_policy");
  const builder::BuildPolicy build_policy{it != parameters.end() ? builder::BuildPolicy::FromStrToType(it->second)
                                                                 : builder::BuildPolicy::Type::kSequential};

  it = parameters.find("simplification_policy");
  const builder::RoadGeometryConfiguration::SimplificationPolicy simplification_policy{
      it != parameters.end() ? builder::RoadGeometryConfiguration::FromStrToSimplificationPolicy(it->second)
                             : builder::RoadGeometryConfiguration::SimplificationPolicy::kNone};

  it = parameters.find("tolerance_selection_policy");
  const builder::RoadGeometryConfiguration::ToleranceSelectionPolicy tolerance_selection_policy{
      it != parameters.end() ? builder::RoadGeometryConfiguration::FromStrToToleranceSelectionPolicy(it->second)
                             : builder::RoadGeometryConfiguration::ToleranceSelectionPolicy::kManualSelection};

  it = parameters.find("road_rule_book");
  const std::optional<std::string> road_rule_book{it != parameters.end() ? std::make_optional(it->second)
                                                                         : std::nullopt};

  it = parameters.find("traffic_light_book");
  const std::optional<std::string> traffic_light_book{it != parameters.end() ? std::make_optional(it->second)
                                                                             : std::nullopt};

  it = parameters.find("phase_ring_book");
  const std::optional<std::string> phase_ring_book{it != parameters.end() ? std::make_optional(it->second)
                                                                          : std::nullopt};

  it = parameters.find("intersection_book");
  const std::optional<std::string> intersection_book{it != parameters.end() ? std::make_optional(it->second)
                                                                            : std::nullopt};

  return {{rg_id, opendrive_file, linear_tolerance, angular_tolerance, scale_length, inertial_to_lane, build_policy,
           simplification_policy, tolerance_selection_policy},
          road_rule_book,
          traffic_light_book,
          phase_ring_book,
          intersection_book};
}

// Implementation of a maliput::plugin::RoadNetworkLoader using maliput_malidrive backend.
class RoadNetworkLoader : public maliput::plugin::RoadNetworkLoader {
 public:
  std::unique_ptr<const maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const override {
    return malidrive::builder::RoadNetworkBuilder(GetPropertiesFromStringMap(properties))();
  }
};

}  // namespace

REGISTER_ROAD_NETWORK_LOADER_PLUGIN("maliput_malidrive", RoadNetworkLoader);

}  // namespace plugin
}  // namespace malidrive
