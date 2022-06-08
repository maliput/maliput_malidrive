add_library(log_level_flag INTERFACE)
target_sources(
  log_level_flag
  INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/log_level_flag.h
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
  PRIVATE
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
  PRIVATE
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

add_executable(xodr_extract xodr_extract.cc)
target_link_libraries(
  xodr_extract
  PRIVATE
    gflags
    maliput::common
    maliput_malidrive::common
    maliput_malidrive::xodr_extract
    tinyxml2
  INTERFACE
    log_level_flag
)
install(
  TARGETS
     xodr_extract
  DESTINATION
    ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/applications
)
