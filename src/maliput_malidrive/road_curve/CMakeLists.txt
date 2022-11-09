##############################################################################
# Malidrive road_curve
##############################################################################

add_library(road_curve
  arc_ground_curve.cc
  lane_offset.cc
  line_ground_curve.cc
  piecewise_function.cc
  piecewise_ground_curve.cc
  road_curve.cc
  road_curve_offset.cc
)
add_library(maliput_malidrive::road_curve ALIAS road_curve)
set_target_properties(road_curve
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_road_curve
)

target_include_directories(
  road_curve
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  road_curve
    maliput::common
    maliput::math
    maliput_malidrive::common
    maliput_drake::analysis
    maliput_drake::common
    maliput_drake::framework
    maliput_drake::math
    maliput_drake::trajectories
)

install(TARGETS road_curve
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
