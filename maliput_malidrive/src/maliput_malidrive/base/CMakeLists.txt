##############################################################################
# MaliputMalidrive base
##############################################################################
add_library(base
  lane.cc
  road_geometry.cc
)
add_library(maliput_malidrive::base ALIAS base)
set_target_properties(base
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_base
)

target_include_directories(
  base
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  base
    maliput::api
    maliput::base
    maliput::geometry_base
    maliput::math
    maliput_drake::analysis
    maliput_drake::common
    maliput_drake::framework
    maliput_drake::math
    maliput_drake::trajectories
    maliput_malidrive::common
    maliput_malidrive::road_curve
    maliput_malidrive::xodr
)

install(TARGETS base
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
