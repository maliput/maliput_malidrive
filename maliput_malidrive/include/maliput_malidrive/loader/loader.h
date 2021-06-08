#pragma once

#include <memory>

#include <maliput/api/road_network.h>

#include "maliput_malidrive/base/inertial_to_lane_mapping_config.h"
#include "maliput_malidrive/builder/road_geometry_configuration.h"
#include "maliput_malidrive/builder/road_network_configuration.h"

namespace malidrive {
namespace loader {

/// Builds a malidrive RoadNetwork.
///
/// Forwards a call to RoadNetworkBuilderT.
///
/// @param road_network_configuration contains a RoadGeometryConfiguration and
///        the configuration files for RoadNetwork entity loaders. The
///        RoadGeometryConfiguration in the network configuration must be valid.
///
/// @throws std::runtime_err When the OpenDrive file cannot be loaded or
/// the RoadGeometry can not be constructed.
/// @return A unique_ptr to a RoadNetwork.
/// @tparam RoadNetworkBuilderT One of opendrive::builder::RoadNetworkBuilder or
///         builder::RoadNetworkBuilder.
template <class RoadNetworkBuilderT>
std::unique_ptr<maliput::api::RoadNetwork> Load(const builder::RoadNetworkConfiguration& road_network_configuration) {
  MALIDRIVE_VALIDATE(!road_network_configuration.road_geometry_configuration.opendrive_file.empty(),
                     std::runtime_error, "opendrive_file cannot be empty");
  return RoadNetworkBuilderT(road_network_configuration)();
}

}  // namespace loader
}  // namespace malidrive
