##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput_malidrive LANGUAGES C CXX VERSION 0.4.1)

##############################################################################
# CMake Support
##############################################################################

# Set CMAKE_INSTALL_<INCLUDEDIR|BINDIR|LIBDIR>
include(GNUInstallDirs)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)

find_package(maliput REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

include(${PROJECT_SOURCE_DIR}/cmake/DefaultCFlags.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/SanitizersConfig.cmake)

set(BUILD_SHARED_LIBS true)

##############################################################################
# Sources
##############################################################################

add_subdirectory(src)

##############################################################################
# Tests
##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  enable_testing()
  ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  # Integration tests will be armed by default, but it could be configured by
  # the user
  if(NOT MALIPUT_MALIDRIVE_INTEGRATION_TESTS)
    set(MALIPUT_MALIDRIVE_INTEGRATION_TESTS ON CACHE BOOL "Enable disable integration tests")
  endif()
  message(STATUS "INTEGRATION Tests are: " ${MALIPUT_MALIDRIVE_INTEGRATION_TESTS})

  add_subdirectory(test)
endif()

##############################################################################
# Docs
##############################################################################

if(BUILD_DOCS)
  message(STATUS "Doxygen generation - Enabled")
  find_package(ament_cmake_doxygen REQUIRED)
    ament_doxygen_generate(doxygen_maliput_malidrive
      CONFIG_OVERLAY doc/Doxyfile.overlay.in
      DEPENDENCIES maliput
      TEST_ON_WARNS
    )
  add_definitions(-DBUILD_DOCS)
else()
  message(STATUS "Doxygen generation - Disabled")
endif()


##############################################################################
# Wheel
##############################################################################

if (WHEEL_GENERATION)
  include(FetchContent)
  message(STATUS "Wheel generation - Enabled")
  FetchContent_Declare(python-cmake-wheel
  GIT_REPOSITORY "https://github.com/maliput/python-cmake-wheel"
  GIT_TAG        "main"
  GIT_SHALLOW    ON)

  FetchContent_GetProperties(python-cmake-wheel)
  FetchContent_MakeAvailable(python-cmake-wheel)
  list(APPEND CMAKE_MODULE_PATH "${python-cmake-wheel_SOURCE_DIR}")
  include(python-wheel)
  set(WHEEL_DEPLOY_DIRECTORY "${CMAKE_BINARY_DIR}/wheel")

  set (DEPLOY_FILES "")
  list(APPEND DEPLOY_FILES "${PROJECT_SOURCE_DIR}/maliput_malidrive/__init__.py")
  # TODO(francocipollone): Deploy the resources folder as well

  add_wheel(road_network
    NAME maliput_malidrive
    AUTHOR "Franco Cipollone"
    URL "https://github.com/maliput/maliput_malidrive"
    VERSION "${PROJECT_VERSION}"
    DESCRIPTION "OpenDRIVE-based maliput backend."
    LICENSE_PATH "${PROJECT_SOURCE_DIR}/LICENSE"
    DEPLOY_FILES ${DEPLOY_FILES}
    MALIPUT_PLUGIN_ENTRY_POINT "maliput_malidrive = maliput_malidrive:get_plugin_path"
  )
else()
  message(STATUS "Wheel generation - Disabled")
endif()

##############################################################################
# Export
##############################################################################

set(MALIPUT_MALIDRIVE_RESOURCES_PATH share/maliput_malidrive/resources)
set(OPENDRIVE_SAMPLES_PATH ${MALIPUT_MALIDRIVE_RESOURCES_PATH}/odr)
install (FILES resources/ArcLane.xodr
               resources/ArcLaneRolled.xodr
               resources/ArcLaneRolledAndOffset.xodr
               resources/ArcElevatedRoad.xodr
               resources/BikingLineLane.xodr
               resources/ComplexLaneSection.xodr
               resources/Crossing8Course.xodr
               resources/CrossingComplex8Course.xodr
               resources/DriveableAndPedestrian.xodr
               resources/DisconnectedRoadInJunction.xodr
               resources/Figure8.xodr
               resources/FlatTown01.xodr
               resources/GapInElevationNonDrivableRoad.xodr
               resources/GapInLaneWidthNonDrivableLane.xodr
               resources/GapInSuperelevationNonDrivableRoad.xodr
               resources/Highway.xodr
               resources/kpit_map.xodr
               resources/LeftRightSidedRoads.xodr
               resources/LineMultipleSections.xodr
               resources/LineMultipleSectionsMoreCases.xodr
               resources/LineMultipleSpeeds.xodr
               resources/LineVariableOffset.xodr
               resources/LineVariableWidth.xodr
               resources/LoopRoadPedestrianCrosswalk.xodr
               resources/LoopRoadPedestrianCrosswalk.yaml
               resources/LShapeRoad.xodr
               resources/LShapeRoadVariableLanes.xodr
               resources/LShapeSection.xodr
               resources/onramp.xodr
               resources/ParkingGarageRamp.xodr
               resources/Roundabout.xodr
               resources/RRFigure8.xodr
               resources/RRLongRoad.xodr
               resources/SingleLane.xodr
               resources/SingleRoadComplexDescription.xodr
               resources/SingleRoadComplexDescription2.xodr
               resources/SingleRoadHighCoefficients.xodr
               resources/SingleRoadNanValues.xodr
               resources/SingleRoadNegativeWidth.xodr
               resources/SingleRoadPedestrianCrosswalk.xodr
               resources/SingleRoadPedestrianCrosswalk.yaml
               resources/SingleRoadTinyGeometry.xodr
               resources/SingleRoadTwoGeometries.xodr
               resources/SmallTownRoads.xodr
               resources/SpiralRoad.xodr
               resources/SShapeRoad.xodr
               resources/SShapeSuperelevatedRoad.xodr
               resources/StraightForward.xodr
               resources/Town01.xodr
               resources/Town02.xodr
               resources/Town03.xodr
               resources/Town04.xodr
               resources/Town05.xodr
               resources/Town06.xodr
               resources/Town07.xodr
               resources/TShapeRoad.xodr
               resources/TShapeRoad.yaml
         DESTINATION ${OPENDRIVE_SAMPLES_PATH})

install (FILES resources/cloverleaf/cloverleaf.xodr
               resources/cloverleaf/cloverleaf.xodr.geojson
               resources/cloverleaf/cloverleaf.xodr.yaml
         DESTINATION ${OPENDRIVE_SAMPLES_PATH}/cloverleaf)

install (FILES resources/figure8_trafficlights/figure8_trafficlights.xodr
               resources/figure8_trafficlights/figure8_trafficlights.yaml
               resources/figure8_trafficlights/figure8_trafficlights_new_rules.yaml
               resources/figure8_trafficlights/lane-s-route_east-left-turn.yaml
               resources/figure8_trafficlights/lane-s-route_east-right-turn.yaml
               resources/figure8_trafficlights/lane-s-route_east-straight.yaml
               resources/figure8_trafficlights/lane-s-route_north-left-turn.yaml
               resources/figure8_trafficlights/lane-s-route_north-right-turn.yaml
               resources/figure8_trafficlights/lane-s-route_north-straight.yaml
               resources/figure8_trafficlights/lane-s-route_south-left-turn.yaml
               resources/figure8_trafficlights/lane-s-route_south-right-turn.yaml
               resources/figure8_trafficlights/lane-s-route_south-straight.yaml
               resources/figure8_trafficlights/lane-s-route_west-left-turn.yaml
               resources/figure8_trafficlights/lane-s-route_west-right-turn.yaml
               resources/figure8_trafficlights/lane-s-route_west-straight.yaml
         DESTINATION ${OPENDRIVE_SAMPLES_PATH}/figure8_trafficlights)

# Unsets the cache, expecting a fresh configuration after each cmake run.
unset(MALIPUT_MALIDRIVE_INTEGRATION_TESTS CACHE)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(maliput)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

ament_package()
