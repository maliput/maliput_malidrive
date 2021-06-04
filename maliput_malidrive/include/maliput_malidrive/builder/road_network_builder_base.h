// Copyright 2020 Toyota Research Institute
#pragma once

#include <memory>

#include "maliput_malidrive/builder/road_network_configuration.h"
#include "maliput_malidrive/common/macros.h"

#include <maliput/api/road_network.h>

namespace malidrive {
namespace builder {

/// Interface to build a RoadNetwork with a malidrive backend.
class RoadNetworkBuilderBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetworkBuilderBase);
  virtual ~RoadNetworkBuilderBase() = default;

  /// Constructs a RoadNetworkBuilderBase.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  RoadNetworkBuilderBase(const RoadNetworkConfiguration& road_network_configuration)
      : road_network_configuration_(road_network_configuration) {}

  /// @return A maliput::api::RoadNetwork.
  virtual std::unique_ptr<maliput::api::RoadNetwork> operator()() const = 0;

 protected:
  const RoadNetworkConfiguration road_network_configuration_;
};

}  // namespace builder
}  // namespace malidrive
