include(${PROJECT_SOURCE_DIR}/cmake/MaliputMalidriveFlags.cmake)

add_library(log_level_flag INTERFACE)
target_sources(log_level_flag
  INTERFACE
    log_level_flag.h
)

install(
  FILES
    log_level_flag.h
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/applications
)

add_executable(xodr_query xodr_query.cc)
target_link_libraries(
  xodr_query
  PUBLIC
    gflags
    maliput::common
    utility
    maliput_malidrive::xodr
  INTERFACE
    log_level_flag
)
install(
  TARGETS
     xodr_query
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/applications
)

add_executable(xodr_validate xodr_validate.cc)
target_link_libraries(
  xodr_validate
  PUBLIC
    gflags
    maliput::common
    utility
    maliput_malidrive::xodr
  INTERFACE
    log_level_flag
)
install(
  TARGETS
     xodr_validate
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/applications
)

add_executable(maliput_malidrive_measure_load_time maliput_malidrive_measure_load_time.cc)
target_link_libraries(
  maliput_malidrive_measure_load_time
  PUBLIC
    gflags
    maliput::common
    maliput_malidrive::loader
    maliput_malidrive::utility
  INTERFACE
    maliput_malidrive::log_level_flag
)
install(
  TARGETS
     maliput_malidrive_measure_load_time
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/applications
)
