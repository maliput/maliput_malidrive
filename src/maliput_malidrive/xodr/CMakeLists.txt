##############################################################################
# xodr
##############################################################################
add_library(xodr
  connection.cc
  db_manager.cc
  elevation_profile.cc
  geometry.cc
  header.cc
  junction.cc
  lane.cc
  lanes.cc
  lane_offset.cc
  lane_section.cc
  lane_width.cc
  lateral_profile.cc
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
