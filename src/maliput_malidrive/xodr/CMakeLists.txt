##############################################################################
# xodr
##############################################################################
add_library(xodr
  colors.cc
  connection.cc
  db_manager.cc
  elevation_profile.cc
  geo_reference.cc
  geometry.cc
  header.cc
  junction.cc
  lane_offset.cc
  lane_road_mark.cc
  lane_section.cc
  lane_width.cc
  lane.cc
  lanes.cc
  lateral_profile.cc
  object/borders.cc
  object/bridge.cc
  object/common.cc
  object/markings.cc
  object/material.cc
  object/object_parser.cc
  object/object_reference.cc
  object/object.cc
  object/outlines.cc
  object/parking_space.cc
  object/repeat.cc
  object/skeleton.cc
  object/surface.cc
  object/tunnel.cc
  object/validity.cc
  offset.cc
  parser.cc
  road_header.cc
  road_link.cc
  road_type.cc
  tools.cc
  unit.cc
)
add_library(maliput_malidrive::xodr ALIAS xodr)
set_target_properties(xodr
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_xodr
)

target_include_directories(
  xodr
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  xodr
    maliput::api
    maliput::math
    maliput_malidrive::common
    tinyxml2
)

install(TARGETS xodr
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

##############################################################################
# xodr_extract
##############################################################################
add_library(extract
  xodr_extract.cc
)
add_library(maliput_malidrive::xodr_extract ALIAS extract)
set_target_properties(extract
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_xodr_extract
)

target_include_directories(
  extract
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  extract
    maliput_malidrive::common
    tinyxml2
)

install(TARGETS extract
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
